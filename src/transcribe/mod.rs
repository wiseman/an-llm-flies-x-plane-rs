//! Push-to-talk voice transcription. Captures audio from the default
//! input device, streams it to the OpenAI Realtime transcription API,
//! and surfaces partial + finalized text back to the TUI.
//!
//! Not directly mirrored from the Python port — `sim_pilot/speech.py`
//! used a batch Whisper call + sounddevice capture. We do streaming
//! partials via the Realtime WebSocket instead.

pub mod audio;
pub mod ptt;
pub mod realtime_ws;

pub use ptt::{PttController, PttMode, PttSnapshot};
