//! Push-to-talk state machine + worker thread.
//!
//! Spawns one thread that:
//!   1. Opens the default input device (cpal) and starts a stream.
//!   2. Opens a fresh Deepgram WebSocket session on every PTT hold
//!      (keyterms can't be updated mid-session, so a new session is the
//!      only way to swap vocabulary per-hold).
//!   3. Routes PCM frames → WS; routes WS delta/completed events →
//!      shared state / finalized channel.
//!
//! The TUI communicates with the worker through:
//!   - `event_tx` (Start { prefix, atc_mode }, Stop, Shutdown)
//!   - `snapshot()` — read-only view of mode/partial/prefix/amp/last_error
//!   - `try_recv_final()` — drain completed transcripts

use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context, Result};
use crossbeam_channel::{bounded, unbounded, Receiver, Sender};
use parking_lot::Mutex;

use crate::bus::SimBus;
use crate::core::mission_manager::PilotCore;
use crate::llm::tools::{ToolBridge, ToolContext};
use crate::transcribe::audio::{start_capture, AudioCapture};
use crate::transcribe::deepgram_ws::{DeepgramClient, DeepgramEvent};
use crate::transcribe::keyterms::build_keyterms;

#[derive(Debug, Clone)]
pub enum PttEvent {
    Start { prefix: String, atc_mode: bool },
    Stop,
    Shutdown,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PttMode {
    Idle,
    Recording,
    Finalizing,
}

#[derive(Debug, Clone)]
pub struct PttSnapshot {
    pub mode: PttMode,
    pub partial: String,
    pub prefix: String,
    pub amp: u8,
    pub last_error: Option<String>,
}

struct Inner {
    mode: PttMode,
    partial: String,
    prefix: String,
    last_error: Option<String>,
}

pub struct PttController {
    state: Arc<Mutex<Inner>>,
    amp: Arc<AtomicU8>,
    event_tx: Sender<PttEvent>,
    finalized_rx: Receiver<String>,
    worker: Option<thread::JoinHandle<()>>,
}

impl PttController {
    /// Spawn the worker, initializing mic capture synchronously so that a
    /// missing / blocked microphone is reported up front as `Err`.
    /// The Deepgram WS connection is deferred until the first `start()`.
    pub fn spawn(
        api_key: String,
        bus: Option<SimBus>,
        pilot: Arc<Mutex<PilotCore>>,
        tool_ctx: Arc<ToolContext>,
        bridge: Option<Arc<dyn ToolBridge>>,
    ) -> Result<Self> {
        let (event_tx, event_rx) = unbounded::<PttEvent>();
        let (finalized_tx, finalized_rx) = unbounded::<String>();
        let (ready_tx, ready_rx) = bounded::<Result<Arc<AtomicU8>, String>>(1);

        let state = Arc::new(Mutex::new(Inner {
            mode: PttMode::Idle,
            partial: String::new(),
            prefix: String::new(),
            last_error: None,
        }));
        let state_for_worker = state.clone();

        let worker = thread::Builder::new()
            .name("ptt-worker".into())
            .spawn(move || {
                worker_loop(
                    event_rx,
                    finalized_tx,
                    api_key,
                    state_for_worker,
                    ready_tx,
                    bus,
                    pilot,
                    tool_ctx,
                    bridge,
                );
            })
            .context("spawn ptt-worker")?;

        let amp = ready_rx
            .recv_timeout(Duration::from_secs(5))
            .context("ptt-worker did not signal ready")?
            .map_err(|e| anyhow!("audio init failed: {e}"))?;

        Ok(Self {
            state,
            amp,
            event_tx,
            finalized_rx,
            worker: Some(worker),
        })
    }

    pub fn start(&self, prefix: String, atc_mode: bool) {
        let _ = self.event_tx.send(PttEvent::Start { prefix, atc_mode });
    }

    pub fn stop(&self) {
        let _ = self.event_tx.send(PttEvent::Stop);
    }

    pub fn snapshot(&self) -> PttSnapshot {
        let s = self.state.lock();
        PttSnapshot {
            mode: s.mode.clone(),
            partial: s.partial.clone(),
            prefix: s.prefix.clone(),
            amp: self.amp.load(Ordering::Acquire),
            last_error: s.last_error.clone(),
        }
    }

    pub fn try_recv_final(&self) -> Option<String> {
        self.finalized_rx.try_recv().ok()
    }
}

impl Drop for PttController {
    fn drop(&mut self) {
        let _ = self.event_tx.send(PttEvent::Shutdown);
        if let Some(h) = self.worker.take() {
            let _ = h.join();
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn worker_loop(
    event_rx: Receiver<PttEvent>,
    finalized_tx: Sender<String>,
    api_key: String,
    state: Arc<Mutex<Inner>>,
    ready_tx: Sender<Result<Arc<AtomicU8>, String>>,
    bus: Option<SimBus>,
    pilot: Arc<Mutex<PilotCore>>,
    tool_ctx: Arc<ToolContext>,
    bridge: Option<Arc<dyn ToolBridge>>,
) {
    let log = |msg: String| {
        if let Some(b) = &bus {
            b.push_log(msg);
        }
    };
    // 20-second headroom: the WS connect() call blocks this thread, so
    // cpal frames pile up in this queue during the handshake and are
    // flushed en masse once the socket is ready. 1024 * 20 ms = 20 s,
    // comfortably larger than any realistic handshake stall.
    let (pcm_tx, pcm_rx) = bounded::<Vec<i16>>(1024);
    // After Stop we wait at most this long for Deepgram to emit the
    // final Results + UtteranceEnd. If they don't arrive we drain
    // whatever's buffered, emit it as a final, and close the WS so the
    // TUI can't be wedged in Finalizing by a missed server event.
    const FINALIZE_TIMEOUT: Duration = Duration::from_millis(2_000);
    let audio: AudioCapture = match start_capture(pcm_tx) {
        Ok(a) => a,
        Err(e) => {
            let _ = ready_tx.send(Err(format!("{e}")));
            return;
        }
    };
    let _ = ready_tx.send(Ok(audio.amp.clone()));

    let mut ws: Option<DeepgramClient> = None;
    let mut finalize_deadline: Option<Instant> = None;
    // Did the mic see speech-level audio (amp ≥ 3 ≈ normal speech) during
    // the current hold? If yes but the server returns an empty final, we
    // log a "silent drop" diagnostic — a known class of bug on streaming
    // STT backends where the socket accepts audio but emits no transcript.
    let mut had_audio_signal = false;
    const SPEECH_AMP_THRESHOLD: u8 = 3;

    loop {
        // 1. Drain control events.
        match event_rx.try_recv() {
            Ok(PttEvent::Start { prefix, atc_mode }) => {
                // Drop any stale WS from a prior hold — Deepgram keyterms
                // are bound at connect time, so a fresh session is the
                // only way to get per-hold vocab.
                if let Some(c) = ws.take() {
                    let _ = c.close();
                }
                // Clear any pcm frames queued from before the hold.
                while pcm_rx.try_recv().is_ok() {}
                finalize_deadline = None;
                had_audio_signal = false;
                {
                    let mut s = state.lock();
                    s.mode = PttMode::Recording;
                    s.prefix = prefix;
                    s.partial.clear();
                    s.last_error = None;
                }
                audio.recording.store(true, Ordering::Release);

                // Build dynamic keyterms from current flight state.
                let keyterms = build_keyterms(&pilot, bridge.as_deref(), &tool_ctx, atc_mode);
                log(format!("voice: deepgram connect ({} keyterms)", keyterms.len()));

                match DeepgramClient::connect(&api_key, &keyterms) {
                    Ok(mut c) => {
                        // Flush any frames captured during the handshake
                        // immediately, ahead of the next poll iteration,
                        // so the start of the hold isn't delayed by the
                        // 5 ms loop sleep.
                        let mut flushed = 0usize;
                        while let Ok(frame) = pcm_rx.try_recv() {
                            if c.send_audio(&frame).is_err() {
                                break;
                            }
                            flushed += 1;
                        }
                        if flushed > 0 {
                            log(format!(
                                "voice: flushed {flushed} pre-handshake frame(s)"
                            ));
                        }
                        ws = Some(c);
                    }
                    Err(e) => {
                        let msg = format!("voice: deepgram connect: {e:#}");
                        log(msg.clone());
                        audio.recording.store(false, Ordering::Release);
                        let mut s = state.lock();
                        s.mode = PttMode::Idle;
                        s.last_error = Some(msg);
                    }
                }
            }
            Ok(PttEvent::Stop) => {
                audio.recording.store(false, Ordering::Release);
                // Flush any straggler frames, then tell Deepgram to close
                // the stream so it emits any pending finals.
                if let Some(c) = ws.as_mut() {
                    while let Ok(frame) = pcm_rx.try_recv() {
                        if c.send_audio(&frame).is_err() {
                            break;
                        }
                    }
                    if let Err(e) = c.finalize() {
                        log(format!("voice: finalize: {e}"));
                    }
                    finalize_deadline = Some(Instant::now() + FINALIZE_TIMEOUT);
                } else {
                    // Nothing to wait on — no session was ever opened.
                    state.lock().mode = PttMode::Idle;
                }
                if ws.is_some() {
                    state.lock().mode = PttMode::Finalizing;
                }
            }
            Ok(PttEvent::Shutdown) => break,
            Err(crossbeam_channel::TryRecvError::Empty) => {}
            Err(crossbeam_channel::TryRecvError::Disconnected) => break,
        }

        // 2. Drain PCM frames → WS (only when recording is on).
        if let Some(c) = ws.as_mut() {
            let mut send_err = None;
            while let Ok(frame) = pcm_rx.try_recv() {
                if let Err(e) = c.send_audio(&frame) {
                    send_err = Some(format!("send: {e}"));
                    break;
                }
            }
            if let Some(e) = send_err {
                state.lock().last_error = Some(e);
                ws = None;
            }
        } else {
            // Drop any accumulated frames so cpal doesn't back up.
            while pcm_rx.try_recv().is_ok() {}
        }

        // Track whether we've seen speech-level audio this hold.
        if audio.recording.load(Ordering::Acquire)
            && audio.amp.load(Ordering::Acquire) >= SPEECH_AMP_THRESHOLD
        {
            had_audio_signal = true;
        }

        // 3. Poll WS for one event.
        if let Some(c) = ws.as_mut() {
            match c.poll() {
                Ok(Some(DeepgramEvent::Delta(partial))) => {
                    state.lock().partial = partial;
                }
                Ok(Some(DeepgramEvent::Completed(text))) => {
                    // Multiple Completed events can fire in a single hold
                    // (UtteranceEnd per speech pause). Only go Idle when
                    // the user has released — otherwise keep listening on
                    // the same WS for the next utterance.
                    let still_holding = audio.recording.load(Ordering::Acquire);
                    {
                        let mut s = state.lock();
                        s.partial.clear();
                        if !still_holding {
                            s.mode = PttMode::Idle;
                        }
                    }
                    let _ = finalized_tx.send(text);
                    if !still_holding {
                        if let Some(c) = ws.take() {
                            let _ = c.close();
                        }
                        finalize_deadline = None;
                    }
                }
                Ok(Some(DeepgramEvent::Closed(maybe_text))) => {
                    // Server said goodbye. Flush any tail text that
                    // wasn't already promoted, then unconditionally drop
                    // the WS and clear the finalize deadline so we don't
                    // false-trip the silent-drop path on a clean exit.
                    if let Some(text) = maybe_text {
                        state.lock().partial.clear();
                        let _ = finalized_tx.send(text);
                    }
                    if let Some(c) = ws.take() {
                        let _ = c.close();
                    }
                    finalize_deadline = None;
                    let still_holding = audio.recording.load(Ordering::Acquire);
                    if !still_holding {
                        state.lock().mode = PttMode::Idle;
                    }
                }
                Ok(Some(DeepgramEvent::Error(msg))) => {
                    log(format!("voice: error: {msg}"));
                    state.lock().last_error = Some(msg);
                }
                Ok(None) => {}
                Err(e) => {
                    log(format!("voice: ws recv: {e}"));
                    state.lock().last_error = Some(format!("recv: {e}"));
                    ws = None;
                    let still_holding = audio.recording.load(Ordering::Acquire);
                    if !still_holding {
                        state.lock().mode = PttMode::Idle;
                        finalize_deadline = None;
                    }
                }
            }
        }

        // 4. Finalize timeout: if Stop fired but the server never sent a
        //    Completed/close by the deadline, promote whatever's buffered
        //    so the TUI doesn't wedge in Finalizing.
        if let Some(deadline) = finalize_deadline {
            if Instant::now() >= deadline {
                let drained = ws.as_mut().and_then(|c| c.drain_final());
                let had_text = drained.is_some();
                if let Some(text) = drained {
                    let _ = finalized_tx.send(text);
                }
                if let Some(c) = ws.take() {
                    let _ = c.close();
                }
                {
                    let mut s = state.lock();
                    s.partial.clear();
                    s.mode = PttMode::Idle;
                }
                finalize_deadline = None;
                if !had_text && had_audio_signal {
                    // Silent-drop: we streamed speech-level audio for the
                    // whole hold but Deepgram returned no transcript.
                    // Surface it so the user knows their message didn't
                    // land (and can retry) rather than silently swallowing
                    // the hold.
                    log(
                        "voice: silent drop detected — audio was sent but \
                         transcript was empty. Try holding again."
                            .to_string(),
                    );
                    state.lock().last_error =
                        Some("silent drop — retry by holding again".to_string());
                } else if !had_text {
                    log("voice: finalize timeout, no buffered text".to_string());
                } else {
                    log("voice: finalize timeout, flushed buffered text".to_string());
                }
                had_audio_signal = false;
            }
        }

        thread::sleep(Duration::from_millis(5));
    }

    if let Some(c) = ws.take() {
        let _ = c.close();
    }
}
