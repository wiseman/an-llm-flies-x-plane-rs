//! OpenAI Realtime transcription WebSocket client.
//!
//! One session lives for the lifetime of the `PttController`. On connect
//! we send a single `transcription_session.update` to configure PCM16 24
//! kHz input + the `gpt-4o-mini-transcribe` model with server-side VAD
//! disabled (we commit explicitly on key release).
//!
//! Wire protocol reference:
//! https://platform.openai.com/docs/api-reference/realtime-client-events

use std::io;
use std::net::TcpStream;

use anyhow::{anyhow, Context, Result};
use base64::engine::general_purpose::STANDARD as B64;
use base64::Engine;
use serde_json::{json, Value};
use tungstenite::client::IntoClientRequest;
use tungstenite::http::HeaderValue;
use tungstenite::stream::MaybeTlsStream;
use tungstenite::{connect, Message, WebSocket};

const URL: &str = "wss://api.openai.com/v1/realtime?intent=transcription";

/// Events surfaced to the PTT worker. The WS server also emits many other
/// event types (session.created, speech_started/stopped, etc) that we
/// silently consume.
#[derive(Debug, Clone)]
pub enum RealtimeEvent {
    /// Running partial. The string is the accumulated transcript since
    /// the last `commit` — safe to display as the in-progress text.
    Delta(String),
    /// Final transcript for the committed audio buffer.
    Completed(String),
    /// Server-reported error.
    Error(String),
}

pub struct RealtimeClient {
    ws: WebSocket<MaybeTlsStream<TcpStream>>,
    partial: String,
}

impl RealtimeClient {
    pub fn connect(api_key: &str) -> Result<Self> {
        let mut req = URL.into_client_request().context("build ws request")?;
        req.headers_mut().insert(
            "Authorization",
            HeaderValue::from_str(&format!("Bearer {api_key}")).context("auth header")?,
        );
        req.headers_mut()
            .insert("OpenAI-Beta", HeaderValue::from_static("realtime=v1"));

        let (ws, _resp) = connect(req).context("ws handshake failed")?;
        let mut client = Self {
            ws,
            partial: String::new(),
        };
        client.set_nonblocking()?;
        client.send_session_update()?;
        Ok(client)
    }

    fn set_nonblocking(&mut self) -> Result<()> {
        let tcp: &TcpStream = match self.ws.get_ref() {
            MaybeTlsStream::Plain(s) => s,
            MaybeTlsStream::Rustls(s) => s.get_ref(),
            _ => return Err(anyhow!("unsupported tls backend")),
        };
        tcp.set_nonblocking(true)
            .context("set_nonblocking on tcp")?;
        Ok(())
    }

    fn send_session_update(&mut self) -> Result<()> {
        // `transcription_session.update` requires config under `session` —
        // the live server rejects the flat form with "Missing required
        // parameter: 'session'." (The developers.openai.com docs show the
        // flat form; it does not match the wire reality.)
        //
        // server_vad with default 500 ms silence window. The server
        // auto-commits the buffer once silence is detected, which is what
        // finalizes a transcription turn. We deliberately do NOT send our
        // own `input_audio_buffer.commit` on key release — doing so races
        // with the server's auto-commit and produces "buffer too small"
        // errors when release arrives after the server already committed.
        let msg = json!({
            "type": "transcription_session.update",
            "session": {
                "input_audio_format": "pcm16",
                "input_audio_transcription": {
                    "model": "gpt-4o-transcribe",
                    "language": "en"
                },
                "turn_detection": {
                    "type": "server_vad",
                    "threshold": 0.5,
                    "prefix_padding_ms": 300,
                    "silence_duration_ms": 500
                }
            }
        });
        self.write(msg.to_string())
    }

    pub fn send_audio(&mut self, pcm: &[i16]) -> Result<()> {
        let mut bytes = Vec::with_capacity(pcm.len() * 2);
        for s in pcm {
            bytes.extend_from_slice(&s.to_le_bytes());
        }
        let b64 = B64.encode(&bytes);
        let msg = json!({"type": "input_audio_buffer.append", "audio": b64});
        self.write(msg.to_string())
    }

    pub fn commit(&mut self) -> Result<()> {
        self.write(json!({"type": "input_audio_buffer.commit"}).to_string())
    }

    pub fn clear_buffer(&mut self) -> Result<()> {
        self.write(json!({"type": "input_audio_buffer.clear"}).to_string())
    }

    fn write(&mut self, text: String) -> Result<()> {
        self.ws
            .send(Message::Text(text))
            .context("ws send failed")?;
        Ok(())
    }

    /// Non-blocking poll for the next event. Returns `Ok(None)` if no data
    /// is available. Silently drops events we don't care about (session
    /// created, VAD markers, etc).
    pub fn poll(&mut self) -> Result<Option<RealtimeEvent>> {
        loop {
            match self.ws.read() {
                Ok(Message::Text(t)) => {
                    if let Some(ev) = self.parse(&t) {
                        return Ok(Some(ev));
                    }
                }
                Ok(Message::Close(_)) => return Err(anyhow!("ws closed by peer")),
                Ok(_) => continue,
                Err(tungstenite::Error::Io(e)) if e.kind() == io::ErrorKind::WouldBlock => {
                    return Ok(None);
                }
                Err(e) => return Err(anyhow!("ws read failed: {e}")),
            }
        }
    }

    fn parse(&mut self, text: &str) -> Option<RealtimeEvent> {
        let v: Value = match serde_json::from_str(text) {
            Ok(v) => v,
            Err(_) => return None,
        };
        let ty = v.get("type").and_then(|t| t.as_str()).unwrap_or("");
        match ty {
            "conversation.item.input_audio_transcription.delta" => {
                let delta = v
                    .get("delta")
                    .and_then(|s| s.as_str())
                    .unwrap_or("")
                    .to_string();
                self.partial.push_str(&delta);
                Some(RealtimeEvent::Delta(self.partial.clone()))
            }
            "conversation.item.input_audio_transcription.completed" => {
                let transcript = v
                    .get("transcript")
                    .and_then(|s| s.as_str())
                    .map(str::to_string)
                    .unwrap_or_else(|| std::mem::take(&mut self.partial));
                self.partial.clear();
                Some(RealtimeEvent::Completed(transcript))
            }
            "error" => {
                let detail = v
                    .get("error")
                    .and_then(|e| e.get("message"))
                    .and_then(|m| m.as_str())
                    .unwrap_or("unknown error")
                    .to_string();
                Some(RealtimeEvent::Error(detail))
            }
            "conversation.item.input_audio_transcription.failed" => {
                let detail = v
                    .get("error")
                    .and_then(|e| e.get("message"))
                    .and_then(|m| m.as_str())
                    .unwrap_or("transcription failed")
                    .to_string();
                Some(RealtimeEvent::Error(detail))
            }
            _ => None,
        }
    }

    pub fn close(mut self) -> Result<()> {
        let _ = self.ws.close(None);
        Ok(())
    }
}

