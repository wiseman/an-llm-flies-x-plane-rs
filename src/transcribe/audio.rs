//! Microphone capture + resample to 24 kHz mono PCM16 for the OpenAI
//! Realtime transcription API. Built on `cpal` (cross-platform) and a
//! simple linear-interpolating resampler (voice-grade is plenty for STT).
//!
//! Mirrors the capture portion of Python's `sim_pilot/speech.py`
//! (not ported in the original Rust port).

use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::Arc;

use anyhow::{anyhow, Context, Result};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{FromSample, Sample, SampleFormat, SizedSample, Stream, StreamConfig};
use crossbeam_channel::Sender;

/// Target sample rate for the OpenAI Realtime transcription API.
pub const TARGET_SR: u32 = 24_000;

/// Flush PCM frames to the WebSocket in ~20 ms chunks.
const FLUSH_SAMPLES: usize = 480;

/// Owner of an active cpal input stream. Dropping stops the capture.
pub struct AudioCapture {
    _stream: Stream,
    pub amp: Arc<AtomicU8>,
    pub recording: Arc<AtomicBool>,
}

/// Open the default input device and begin streaming. While `recording`
/// is true, ~20 ms frames of 24 kHz mono PCM16 are pushed to `pcm_tx`.
/// The `amp` field tracks an 0..=8 RMS bucket for the TUI indicator.
pub fn start_capture(pcm_tx: Sender<Vec<i16>>) -> Result<AudioCapture> {
    let host = cpal::default_host();
    let device = host
        .default_input_device()
        .ok_or_else(|| anyhow!("no default input device"))?;
    let supported = device
        .default_input_config()
        .context("failed to query default input config")?;

    let sample_fmt = supported.sample_format();
    let channels = supported.channels() as usize;
    let source_sr = supported.sample_rate().0;
    let stream_cfg: StreamConfig = supported.into();

    let amp = Arc::new(AtomicU8::new(0));
    let recording = Arc::new(AtomicBool::new(false));

    let stream = match sample_fmt {
        SampleFormat::F32 => build_stream::<f32>(
            &device,
            &stream_cfg,
            channels,
            source_sr,
            pcm_tx,
            amp.clone(),
            recording.clone(),
        )?,
        SampleFormat::I16 => build_stream::<i16>(
            &device,
            &stream_cfg,
            channels,
            source_sr,
            pcm_tx,
            amp.clone(),
            recording.clone(),
        )?,
        SampleFormat::U16 => build_stream::<u16>(
            &device,
            &stream_cfg,
            channels,
            source_sr,
            pcm_tx,
            amp.clone(),
            recording.clone(),
        )?,
        other => return Err(anyhow!("unsupported sample format: {other:?}")),
    };
    stream.play().context("failed to start input stream")?;

    Ok(AudioCapture {
        _stream: stream,
        amp,
        recording,
    })
}

fn build_stream<T>(
    device: &cpal::Device,
    config: &StreamConfig,
    channels: usize,
    source_sr: u32,
    pcm_tx: Sender<Vec<i16>>,
    amp: Arc<AtomicU8>,
    recording: Arc<AtomicBool>,
) -> Result<Stream>
where
    T: SizedSample + 'static,
    f32: FromSample<T>,
{
    let mut resampler = LinearResampler::new(source_sr, TARGET_SR);
    let mut accum: Vec<i16> = Vec::with_capacity(FLUSH_SAMPLES * 2);
    let err_fn = |e| eprintln!("audio stream error: {e}");
    let stream = device
        .build_input_stream(
            config,
            move |data: &[T], _: &cpal::InputCallbackInfo| {
                on_samples(
                    data,
                    channels,
                    &mut resampler,
                    &mut accum,
                    &pcm_tx,
                    &amp,
                    &recording,
                );
            },
            err_fn,
            None,
        )
        .context("failed to build input stream")?;
    Ok(stream)
}

fn on_samples<T>(
    data: &[T],
    channels: usize,
    resampler: &mut LinearResampler,
    accum: &mut Vec<i16>,
    pcm_tx: &Sender<Vec<i16>>,
    amp: &AtomicU8,
    recording: &AtomicBool,
) where
    T: Sample + Copy,
    f32: FromSample<T>,
{
    let mono: Vec<f32> = if channels <= 1 {
        data.iter().map(|&s| s.to_sample::<f32>()).collect()
    } else {
        data.chunks(channels)
            .map(|frame| {
                frame.iter().map(|&s| s.to_sample::<f32>()).sum::<f32>() / channels as f32
            })
            .collect()
    };

    let sum_sq: f32 = mono.iter().map(|x| x * x).sum();
    let rms = if mono.is_empty() {
        0.0
    } else {
        (sum_sq / mono.len() as f32).sqrt()
    };
    // Scale so typical speech hits ~4-6, quiet room ~0, shouting ~8.
    let level = ((rms * 12.0).clamp(0.0, 8.0)) as u8;
    amp.store(level, Ordering::Release);

    resampler.process(&mono, accum);
    if recording.load(Ordering::Acquire) {
        while accum.len() >= FLUSH_SAMPLES {
            let frame: Vec<i16> = accum.drain(..FLUSH_SAMPLES).collect();
            let _ = pcm_tx.try_send(frame);
        }
    } else {
        accum.clear();
    }
}

/// Simple linear-interpolating resampler. Not audiophile, but fine for STT.
struct LinearResampler {
    ratio: f64,
    phase: f64,
    prev: f32,
}

impl LinearResampler {
    fn new(src: u32, dst: u32) -> Self {
        Self {
            ratio: src as f64 / dst as f64,
            phase: 0.0,
            prev: 0.0,
        }
    }

    fn process(&mut self, mono_in: &[f32], out: &mut Vec<i16>) {
        for &sample in mono_in {
            while self.phase < 1.0 {
                let t = self.phase as f32;
                let v = self.prev + (sample - self.prev) * t;
                out.push((v.clamp(-1.0, 1.0) * i16::MAX as f32) as i16);
                self.phase += self.ratio;
            }
            self.phase -= 1.0;
            self.prev = sample;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_resampler_48k_to_24k_halves_length() {
        let mut r = LinearResampler::new(48_000, 24_000);
        let input: Vec<f32> = (0..480).map(|i| (i as f32).sin() * 0.1).collect();
        let mut out = Vec::new();
        r.process(&input, &mut out);
        // 2:1 ratio → roughly half the samples out, within ±2.
        assert!(out.len() >= 238 && out.len() <= 242, "got {} samples", out.len());
    }

    #[test]
    fn linear_resampler_identity_passthrough() {
        let mut r = LinearResampler::new(24_000, 24_000);
        let input = vec![0.5_f32; 100];
        let mut out = Vec::new();
        r.process(&input, &mut out);
        // Identity ratio: one-to-one output. The first sample is
        // interpolated from the 0.0 initial `prev`, so skip it.
        assert_eq!(out.len(), 100);
        for v in &out[1..] {
            assert!((*v - 16383).abs() <= 1, "got {}", v);
        }
    }
}
