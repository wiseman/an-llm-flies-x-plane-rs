//! Push-to-talk voice transcription. Captures audio from the default
//! input device, streams it to Deepgram's Nova 3 streaming API with
//! per-hold keyterm biasing (airports, ATC facilities, runways, and
//! taxiways drawn from the current flight state), and surfaces partial
//! + finalized text back to the TUI.

pub mod audio;
pub mod deepgram_ws;
pub mod keyterms;
pub mod ptt;

pub use ptt::{PttController, PttMode, PttSnapshot};
