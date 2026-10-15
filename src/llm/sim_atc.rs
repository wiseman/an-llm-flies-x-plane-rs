//! Simulated ATC for evals: a Claude Code session (Haiku by default)
//! playing every controller at the airport.
//!
//! The pilot's `broadcast_on_radio` calls are handed to `SimAtc::hear`
//! with the frequency and the aircraft's status. The controller session
//! answers in FAA phraseology, or with `NONE` when a call needs no reply,
//! and each answer goes to the radio pane and to the pilot as an `[atc]`
//! message. While a call is waiting for its answer, `composing()` is true
//! and the eval harness runs the sim at 1x (real time) instead of as fast
//! as it can, so the controller's thinking time becomes a realistic radio
//! delay rather than minutes of sim time.

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::Result;
use crossbeam_channel::{unbounded, Receiver, RecvTimeoutError, Sender};

use crate::bus::{LogKind, SimBus};
use crate::llm::claude_code::{ClaudeCodeConfig, ClaudeSession};
use crate::llm::conversation::IncomingMessage;

/// The model the eval harness's controller runs by default.
pub const DEFAULT_SIM_ATC_MODEL: &str = "claude-haiku-5-5";

/// A controller gets this long to answer one call.
const REPLY_BUDGET: Duration = Duration::from_secs(60);

/// One transmission from the pilot.
pub struct RadioCall {
    /// Frequency the pilot transmitted on (MHz), if known.
    pub freq_mhz: Option<f64>,
    pub message: String,
    /// The aircraft's status JSON at the time of the call.
    pub status_json: String,
}

pub struct SimAtc {
    calls: Sender<RadioCall>,
    pending: Arc<AtomicUsize>,
    stop: Arc<AtomicBool>,
    handle: parking_lot::Mutex<Option<JoinHandle<()>>>,
}

impl SimAtc {
    /// Start the controller. `airport_brief` describes the airport
    /// (runways, frequencies) for the system prompt; answers are sent to
    /// the pilot on `pilot_input`.
    pub fn start(
        model: &str,
        airport_brief: &str,
        pilot_input: Sender<IncomingMessage>,
        bus: Option<SimBus>,
    ) -> Result<Arc<Self>> {
        let cfg = ClaudeCodeConfig::new(model);
        let session = ClaudeSession::plain(&cfg, "[sim-atc]", system_prompt(airport_brief), bus.clone())?;
        let (calls, rx) = unbounded();
        let pending = Arc::new(AtomicUsize::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let handle = {
            let pending = pending.clone();
            let stop = stop.clone();
            std::thread::Builder::new()
                .name("sim-atc".into())
                .spawn(move || run(session, rx, pending, stop, pilot_input, bus))?
        };
        Ok(Arc::new(Self { calls, pending, stop, handle: parking_lot::Mutex::new(Some(handle)) }))
    }

    /// Hand a pilot transmission to the controller. Counts as composing
    /// from now until the answer (if any) is on its way to the pilot.
    pub fn hear(&self, call: RadioCall) {
        self.pending.fetch_add(1, Ordering::AcqRel);
        if self.calls.send(call).is_err() {
            self.pending.fetch_sub(1, Ordering::AcqRel);
        }
    }

    /// A call is waiting for its answer.
    pub fn composing(&self) -> bool {
        self.pending.load(Ordering::Acquire) > 0
    }

    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Release);
        if let Some(h) = self.handle.lock().take() {
            let _ = h.join();
        }
    }
}

fn run(
    mut session: ClaudeSession,
    calls: Receiver<RadioCall>,
    pending: Arc<AtomicUsize>,
    stop: Arc<AtomicBool>,
    pilot_input: Sender<IncomingMessage>,
    bus: Option<SimBus>,
) {
    while !stop.load(Ordering::Acquire) {
        let call = match calls.recv_timeout(Duration::from_millis(200)) {
            Ok(c) => c,
            Err(RecvTimeoutError::Timeout) => continue,
            Err(RecvTimeoutError::Disconnected) => break,
        };
        let freq = call.freq_mhz.map(|f| format!("{f:.3}")).unwrap_or_else(|| "unknown".into());
        let content = format!(
            "Pilot transmits on {freq} MHz: \"{}\"\n\nAircraft status: {}",
            call.message, call.status_json
        );
        match session.run_turn(&content, Instant::now() + REPLY_BUDGET, &stop) {
            Ok(reply) => {
                if let Some(reply) = transmission(&reply) {
                    if let Some(b) = &bus {
                        b.push_radio(format!("[ATC {freq}] {reply}"));
                    }
                    let _ = pilot_input.send(IncomingMessage::atc(reply));
                }
            }
            Err(e) => {
                if let Some(b) = &bus {
                    b.push_log_kind(LogKind::Error, format!("[sim-atc] {e:#}"));
                }
            }
        }
        pending.fetch_sub(1, Ordering::AcqRel);
    }
    session.kill();
}

/// The controller's reply, or `None` when it chose not to transmit.
fn transmission(reply: &str) -> Option<String> {
    let reply = reply.trim().trim_matches('"').trim();
    (!reply.is_empty() && !reply.eq_ignore_ascii_case("NONE")).then(|| reply.to_string())
}

fn system_prompt(airport_brief: &str) -> String {
    format!(
        "You are air traffic control in a flight simulator, playing every controller at \
         this airport: clearance delivery, ground and tower, plus approach/departure \
         if listed.\n\n{airport_brief}\n\n\
         Wind is calm and there is no other traffic. One aircraft is flying; each \
         message gives the frequency the pilot transmitted on, what they said, and the \
         aircraft's current status (position, phase, active automation).\n\n\
         Reply with exactly what the controller on that frequency transmits back, and \
         nothing else: no quotes, no commentary. Use standard FAA phraseology, the \
         pilot's callsign, and keep it short. Give the clearances a real controller \
         would when they're requested (taxi, takeoff, closed traffic or pattern entry, \
         landing, taxi to parking); with no traffic, approve requests promptly. If a \
         call needs no answer (a correct readback, or a call on a frequency nobody \
         here monitors), reply with exactly NONE."
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn none_and_blank_replies_are_silence() {
        assert_eq!(transmission("NONE"), None);
        assert_eq!(transmission("  none \n"), None);
        assert_eq!(transmission(""), None);
        assert_eq!(
            transmission("\"Cessna 2SP, runway 12, cleared for takeoff.\""),
            Some("Cessna 2SP, runway 12, cleared for takeoff.".to_string())
        );
    }
}
