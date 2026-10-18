//! The demo's radio: ATC and pilot transmissions spoken by TTS through a
//! radio filter, one at a time like a real (half-duplex) frequency.
//!
//! One thread renders and plays everything in order, so the pilot never
//! talks over the controller. Clips are cached by content
//! (`~/.cache/sim_pilot/tts/`), so a second take of the same script
//! costs nothing and ATC can be rendered before the take starts.
//!
//! Backends: ElevenLabs (`ELEVENLABS_API_KEY`, Flash v2.5), Kokoro (a
//! persistent local worker, `scripts/demo/kokoro_server.py`), or macOS
//! `say`, which is also the fallback when the others fail.

use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Instant;

use anyhow::{anyhow, bail, Context, Result};
use crossbeam_channel::{unbounded, Receiver, Sender};
use parking_lot::Mutex;

use crate::bus::{LogKind, SimBus};
use crate::demo::script::{AudioSetup, TtsBackend, Voice};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speaker {
    Atc,
    Pilot,
}

/// One transmission to play. `id` comes back in `RadioEvent`s.
#[derive(Debug, Clone)]
pub struct TxJob {
    pub id: u64,
    pub speaker: Speaker,
    pub spoken: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RadioEvent {
    /// The clip started playing (or failed to render and was skipped).
    Started { id: u64, speaker: Speaker, spoken: String },
    Done(u64),
}

enum Work {
    Prewarm(Speaker, String),
    Play(TxJob),
}

struct RadioState {
    busy: AtomicBool,
    queued: AtomicUsize,
    last_end: Mutex<Instant>,
}

/// A handle on the radio thread. Clones share it (the pilot's tool
/// handler transmits; the demo director transmits ATC and reads events).
#[derive(Clone)]
pub struct RadioHandle {
    tx: Sender<Work>,
    events_rx: Receiver<RadioEvent>,
    state: Arc<RadioState>,
}

impl RadioHandle {
    pub fn spawn(audio: AudioSetup, bus: SimBus) -> Self {
        let (tx, rx) = unbounded::<Work>();
        let (events_tx, events_rx) = unbounded::<RadioEvent>();
        let state = Arc::new(RadioState {
            busy: AtomicBool::new(false),
            queued: AtomicUsize::new(0),
            last_end: Mutex::new(Instant::now()),
        });
        let st = state.clone();
        thread::Builder::new()
            .name("demo-radio".into())
            .spawn(move || {
                let mut voices = Voices::default();
                for w in rx {
                    match w {
                        Work::Prewarm(sp, spoken) => {
                            let v = voice_for(&audio, sp);
                            if let Err(e) = voices.render(v, &spoken) {
                                bus.push_log_kind(LogKind::Error, format!("[radio] rendering {spoken:?}: {e:#}"));
                            }
                        }
                        Work::Play(job) => {
                            st.busy.store(true, Ordering::Release);
                            st.queued.fetch_sub(1, Ordering::AcqRel);
                            let v = voice_for(&audio, job.speaker);
                            let clip = voices.render(v, &job.spoken);
                            let _ = events_tx.send(RadioEvent::Started {
                                id: job.id,
                                speaker: job.speaker,
                                spoken: job.spoken.clone(),
                            });
                            match clip {
                                Ok(path) => {
                                    if let Err(e) = play(&path) {
                                        bus.push_log_kind(LogKind::Error, format!("[radio] playing: {e:#}"));
                                    }
                                }
                                Err(e) => bus.push_log_kind(LogKind::Error, format!("[radio] rendering {:?}: {e:#}", job.spoken)),
                            }
                            *st.last_end.lock() = Instant::now();
                            st.busy.store(false, Ordering::Release);
                            let _ = events_tx.send(RadioEvent::Done(job.id));
                        }
                    }
                }
            })
            .expect("spawn demo-radio");
        Self { tx, events_rx, state }
    }

    /// Render (and cache) a clip ahead of time.
    pub fn prewarm(&self, speaker: Speaker, spoken: &str) {
        let _ = self.tx.send(Work::Prewarm(speaker, spoken.to_string()));
    }

    pub fn transmit(&self, job: TxJob) {
        self.state.queued.fetch_add(1, Ordering::AcqRel);
        let _ = self.tx.send(Work::Play(job));
    }

    /// Nothing playing or queued, and `min_gap_s` of silence since the
    /// last transmission ended.
    pub fn is_clear(&self, min_gap_s: f64) -> bool {
        !self.state.busy.load(Ordering::Acquire)
            && self.state.queued.load(Ordering::Acquire) == 0
            && self.state.last_end.lock().elapsed().as_secs_f64() >= min_gap_s
    }

    pub fn events(&self) -> Vec<RadioEvent> {
        self.events_rx.try_iter().collect()
    }
}

fn voice_for(audio: &AudioSetup, sp: Speaker) -> &Voice {
    match sp {
        Speaker::Atc => &audio.atc,
        Speaker::Pilot => &audio.pilot,
    }
}

/// Renders clips, keeping the Kokoro worker alive between them.
#[derive(Default)]
pub struct Voices {
    kokoro: Option<KokoroWorker>,
}

impl Voices {
    /// The radio clip for `spoken` in `voice`, from the cache or freshly
    /// rendered. Falls back to `say` when the voice's backend fails.
    pub fn render(&mut self, voice: &Voice, spoken: &str) -> Result<PathBuf> {
        let dir = cache_dir()?;
        let key = format!(
            "v1|{:?}|{}|{:?}|{}|{}|{}",
            voice.backend,
            voice.voice,
            voice.speed,
            voice.settings.as_ref().map(|s| s.to_string()).unwrap_or_default(),
            voice.preset.as_deref().unwrap_or("atc"),
            spoken
        );
        let out = dir.join(format!("{:016x}.radio.wav", fnv64(&key)));
        if out.exists() {
            return Ok(out);
        }
        let raw = match self.synth(voice, spoken, &dir) {
            Ok(r) => r,
            Err(e) if voice.backend != TtsBackend::Say => {
                let fallback = Voice { backend: TtsBackend::Say, ..voice.clone() };
                self.synth(&fallback, spoken, &dir).with_context(|| format!("{:?} failed ({e:#}); say too", voice.backend))?
            }
            Err(e) => return Err(e),
        };
        radio_filter(&raw, &out, voice.preset.as_deref().unwrap_or("atc"))?;
        let _ = std::fs::remove_file(&raw.path);
        Ok(out)
    }

    fn synth(&mut self, voice: &Voice, spoken: &str, dir: &Path) -> Result<RawClip> {
        let tmp = dir.join(format!("{:016x}.raw", fnv64(&format!("{spoken}|{}|{:?}", voice.voice, voice.backend))));
        match voice.backend {
            TtsBackend::Elevenlabs => {
                let key = std::env::var("ELEVENLABS_API_KEY").map_err(|_| anyhow!("ELEVENLABS_API_KEY is not set"))?;
                let mut body = serde_json::json!({"text": spoken, "model_id": "eleven_flash_v2_5"});
                if let Some(s) = &voice.settings {
                    body["voice_settings"] = s.clone();
                }
                if let Some(speed) = voice.speed {
                    body["voice_settings"]["speed"] = serde_json::json!(speed.clamp(0.7, 1.2));
                }
                let url = format!("https://api.elevenlabs.io/v1/text-to-speech/{}?output_format=pcm_24000", voice.voice);
                let resp = ureq::post(&url)
                    .set("xi-api-key", &key)
                    .set("Content-Type", "application/json")
                    .send_string(&body.to_string())
                    .map_err(|e| anyhow!("ElevenLabs: {e}"))?;
                let mut bytes = Vec::new();
                std::io::Read::read_to_end(&mut resp.into_reader(), &mut bytes)?;
                std::fs::write(&tmp, &bytes)?;
                Ok(RawClip { path: tmp, pcm_s16le_hz: Some(24_000) })
            }
            TtsBackend::Kokoro => {
                if self.kokoro.is_none() {
                    self.kokoro = Some(KokoroWorker::start()?);
                }
                let wav = tmp.with_extension("wav");
                let speed = voice.speed.unwrap_or(1.0);
                if let Err(e) = self.kokoro.as_mut().unwrap().synth(spoken, &voice.voice, speed, &wav) {
                    self.kokoro = None; // restart next time
                    return Err(e);
                }
                Ok(RawClip { path: wav, pcm_s16le_hz: None })
            }
            TtsBackend::Say => {
                let aiff = tmp.with_extension("aiff");
                let rate = (180.0 * voice.speed.unwrap_or(1.0)).round().to_string();
                let with_voice = Command::new("say").args(["-v", &voice.voice, "-r", &rate, "-o"]).arg(&aiff).arg(spoken).status();
                if !with_voice.is_ok_and(|s| s.success()) {
                    // Not a `say` voice (a Kokoro or ElevenLabs name): the default one.
                    let st = Command::new("say").args(["-r", &rate, "-o"]).arg(&aiff).arg(spoken).status()?;
                    if !st.success() {
                        bail!("say failed");
                    }
                }
                Ok(RawClip { path: aiff, pcm_s16le_hz: None })
            }
        }
    }
}

struct RawClip {
    path: PathBuf,
    /// Headerless 16-bit PCM at this rate (ElevenLabs); else a file
    /// ffmpeg can read.
    pcm_s16le_hz: Option<u32>,
}

/// Band-limited, compressed, a little crushed, with a key-up click of
/// noise before and a squelch tail after: a VHF radio.
fn radio_filter(raw: &RawClip, out: &Path, preset: &str) -> Result<()> {
    let noise = if preset == "pilot" { 0.012 } else { 0.035 };
    let mut cmd = Command::new("ffmpeg");
    cmd.args(["-hide_banner", "-loglevel", "error", "-y"]);
    if let Some(hz) = raw.pcm_s16le_hz {
        cmd.args(["-f", "s16le", "-ar", &hz.to_string(), "-ac", "1"]);
    }
    cmd.arg("-i").arg(&raw.path);
    cmd.args(["-f", "lavfi", "-i", &format!("anoisesrc=color=pink:amplitude={noise}:sample_rate=16000")]);
    cmd.args([
        "-filter_complex",
        "[0:a]aresample=16000,highpass=f=350,lowpass=f=3000,\
         acompressor=threshold=-20dB:ratio=6:attack=5:release=60:makeup=2,\
         acrusher=bits=10:mode=log:mix=0.25,adelay=150:all=1,apad=pad_dur=0.3[v];\
         [1:a]highpass=f=300,lowpass=f=3200[n];\
         [v][n]amix=inputs=2:duration=first:dropout_transition=0:normalize=0,alimiter=limit=0.9",
        "-ac",
        "1",
        "-ar",
        "16000",
    ]);
    let tmp = out.with_extension("tmp.wav");
    cmd.arg(&tmp);
    let st = cmd.status().context("running ffmpeg")?;
    if !st.success() {
        bail!("ffmpeg radio filter failed");
    }
    std::fs::rename(&tmp, out)?;
    Ok(())
}

/// Play a clip and wait for it to finish.
pub fn play(path: &Path) -> Result<()> {
    let st = Command::new("afplay").arg(path).status().context("running afplay")?;
    if !st.success() {
        bail!("afplay failed");
    }
    Ok(())
}

fn cache_dir() -> Result<PathBuf> {
    let home = std::env::var("HOME").map_err(|_| anyhow!("HOME is not set"))?;
    let dir = PathBuf::from(home).join(".cache/sim_pilot/tts");
    std::fs::create_dir_all(&dir)?;
    Ok(dir)
}

/// FNV-1a, 64-bit: a cache key stable across Rust versions.
fn fnv64(s: &str) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in s.bytes() {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// The Kokoro model, loaded once in a Python process that renders one
/// line per request.
pub struct KokoroWorker {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
}

const KOKORO_SERVER: &str = include_str!("../../scripts/demo/kokoro_server.py");

impl KokoroWorker {
    pub fn start() -> Result<Self> {
        let dir = cache_dir()?;
        let script = dir.join("kokoro_server.py");
        std::fs::write(&script, KOKORO_SERVER)?;
        let venv_python = PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(".venvs/kokoro/bin/python");
        let mut cmd = match std::env::var("KOKORO_PYTHON") {
            Ok(p) => Command::new(p),
            Err(_) if venv_python.exists() => Command::new(venv_python),
            Err(_) => {
                let mut c = Command::new("uv");
                c.args(["run", "--script"]);
                c
            }
        };
        let mut child = cmd
            .arg(&script)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .context("starting the Kokoro worker")?;
        let stdin = child.stdin.take().unwrap();
        let mut stdout = BufReader::new(child.stdout.take().unwrap());
        let mut line = String::new();
        stdout.read_line(&mut line)?;
        if !line.contains("ready") {
            bail!("Kokoro worker didn't start: {line:?}");
        }
        Ok(Self { child, stdin, stdout })
    }

    pub fn synth(&mut self, text: &str, voice: &str, speed: f64, out: &Path) -> Result<()> {
        let req = serde_json::json!({"text": text, "voice": voice, "speed": speed, "out": out});
        writeln!(self.stdin, "{req}")?;
        self.stdin.flush()?;
        let mut line = String::new();
        self.stdout.read_line(&mut line)?;
        let resp: serde_json::Value = serde_json::from_str(&line).with_context(|| format!("Kokoro said {line:?}"))?;
        if resp["ok"].as_bool() != Some(true) {
            bail!("Kokoro: {}", resp["error"]);
        }
        Ok(())
    }
}

impl Drop for KokoroWorker {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cache_keys_are_stable() {
        assert_eq!(fnv64(""), 0xcbf2_9ce4_8422_2325);
        assert_eq!(fnv64("a"), 0xaf63_dc4c_8601_ec8c);
    }
}
