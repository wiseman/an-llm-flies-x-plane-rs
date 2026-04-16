//! Push-to-talk state machine + worker thread.
//!
//! Spawns one thread that:
//!   1. Opens the default input device (cpal) and starts a stream.
//!   2. Lazily connects the OpenAI Realtime WebSocket on first `Start`.
//!   3. Routes PCM frames → WS; routes WS delta/completed events →
//!      shared state / finalized channel.
//!
//! The TUI communicates with the worker through:
//!   - `event_tx` (Start { prefix }, Stop, Shutdown)
//!   - `snapshot()` — read-only view of mode/partial/prefix/amp/last_error
//!   - `try_recv_final()` — drain completed transcripts (one per PTT session)

use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use crossbeam_channel::{bounded, unbounded, Receiver, Sender};
use parking_lot::Mutex;

use crate::bus::SimBus;
use crate::transcribe::audio::{start_capture, AudioCapture};
use crate::transcribe::realtime_ws::{RealtimeClient, RealtimeEvent};

#[derive(Debug, Clone)]
pub enum PttEvent {
    Start { prefix: String },
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
    /// The Realtime WS connection is deferred until the first `start()`.
    pub fn spawn(api_key: String, bus: Option<SimBus>) -> Result<Self> {
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

    pub fn start(&self, prefix: String) {
        let _ = self.event_tx.send(PttEvent::Start { prefix });
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

fn worker_loop(
    event_rx: Receiver<PttEvent>,
    finalized_tx: Sender<String>,
    api_key: String,
    state: Arc<Mutex<Inner>>,
    ready_tx: Sender<Result<Arc<AtomicU8>, String>>,
    bus: Option<SimBus>,
) {
    let log = |msg: String| {
        if let Some(b) = &bus {
            b.push_log(msg);
        }
    };
    // Always drain the pcm channel so the stream doesn't back up when idle.
    let (pcm_tx, pcm_rx) = bounded::<Vec<i16>>(256);
    let audio: AudioCapture = match start_capture(pcm_tx) {
        Ok(a) => a,
        Err(e) => {
            let _ = ready_tx.send(Err(format!("{e}")));
            return;
        }
    };
    // Signal readiness with the amp handle so the controller's snapshot()
    // can publish the live level without taking the mutex.
    let _ = ready_tx.send(Ok(audio.amp.clone()));

    let mut ws: Option<RealtimeClient> = None;

    loop {
        // 1. Drain control events.
        match event_rx.try_recv() {
            Ok(PttEvent::Start { prefix }) => {
                if ws.is_none() {
                    match RealtimeClient::connect(&api_key) {
                        Ok(c) => {
                            log("voice: ws connected".to_string());
                            ws = Some(c);
                        }
                        Err(e) => {
                            let msg = format!("voice: ws connect failed: {e}");
                            log(msg.clone());
                            state.lock().last_error = Some(msg);
                            continue;
                        }
                    }
                }
                {
                    let mut s = state.lock();
                    s.mode = PttMode::Recording;
                    s.prefix = prefix;
                    s.partial.clear();
                    s.last_error = None;
                }
                audio.recording.store(true, Ordering::Release);
            }
            Ok(PttEvent::Stop) => {
                // Just stop sending audio. The server's VAD will detect
                // silence within ~500 ms and auto-commit the buffer,
                // which triggers the final transcription delta/completed
                // events. An explicit commit here races with the server
                // and errors out when the server already committed.
                audio.recording.store(false, Ordering::Release);
                state.lock().mode = PttMode::Finalizing;
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

        // 3. Poll WS for one event.
        if let Some(c) = ws.as_mut() {
            match c.poll() {
                Ok(Some(RealtimeEvent::Delta(partial))) => {
                    state.lock().partial = partial;
                }
                Ok(Some(RealtimeEvent::Completed(text))) => {
                    // The server's VAD auto-commits on every speech-end
                    // pause, so multiple `completed` events can fire in a
                    // single PTT hold. Only transition to Idle when the
                    // user has actually released (audio.recording is off).
                    // Otherwise stay in Recording — the partial gets
                    // flushed to the input buffer and we keep listening.
                    let still_holding = audio.recording.load(Ordering::Acquire);
                    {
                        let mut s = state.lock();
                        s.partial.clear();
                        if !still_holding {
                            s.mode = PttMode::Idle;
                        }
                    }
                    let _ = finalized_tx.send(text);
                    let _ = c.clear_buffer();
                }
                Ok(Some(RealtimeEvent::Error(msg))) => {
                    log(format!("voice: error: {msg}"));
                    state.lock().last_error = Some(msg);
                }
                Ok(None) => {}
                Err(e) => {
                    log(format!("voice: ws recv: {e}"));
                    state.lock().last_error = Some(format!("recv: {e}"));
                    ws = None;
                }
            }
        }

        thread::sleep(Duration::from_millis(5));
    }

    if let Some(c) = ws.take() {
        let _ = c.close();
    }
}
