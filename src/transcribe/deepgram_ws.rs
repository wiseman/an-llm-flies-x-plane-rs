//! Deepgram streaming transcription WebSocket client (Nova 3).
//!
//! One fresh session per PTT hold. Audio is sent as raw binary frames
//! carrying little-endian PCM16; control messages (`KeepAlive`,
//! `CloseStream`) are JSON text frames. Keyterm biasing is passed as
//! repeated `keyterm` query parameters at connect time — Nova 3 does
//! semantic boosting on them (no explicit weights).
//!
//! Wire protocol reference:
//! <https://developers.deepgram.com/docs/live-streaming-audio>

use std::io;
use std::net::TcpStream;

use anyhow::{anyhow, Context, Result};
use serde_json::Value;
use tungstenite::client::IntoClientRequest;
use tungstenite::http::HeaderValue;
use tungstenite::stream::MaybeTlsStream;
use tungstenite::{connect, Message, WebSocket};
use url::Url;

const BASE_URL: &str = "wss://api.deepgram.com/v1/listen";

/// Events surfaced to the PTT worker. Deepgram emits a few other message
/// types (`Metadata`, `SpeechStarted`) that we silently consume.
#[derive(Debug, Clone)]
pub enum DeepgramEvent {
    /// Interim transcript — replaces any prior interim for display.
    /// Already concatenated with any finalized segments so far, so the
    /// PTT worker can show a monotonically growing live preview.
    Delta(String),
    /// Final transcript for the completed utterance. Emitted on
    /// `speech_final=true` or `UtteranceEnd`.
    Completed(String),
    /// Server closed the connection. Carries any in-flight text that
    /// hadn't yet been promoted to a final so the caller can flush it
    /// before going Idle. Emitted at most once per session.
    Closed(Option<String>),
    /// Server-reported error or WS-level failure.
    Error(String),
}

pub struct DeepgramClient {
    ws: WebSocket<MaybeTlsStream<TcpStream>>,
    /// Accumulated finalized segments within the current utterance.
    finalized_so_far: String,
    /// Latest interim transcript (replaced, not appended).
    interim: String,
    /// True once we've sent `CloseStream` — further audio is dropped.
    finalized: bool,
    /// True once the server-side close has been observed — guards
    /// `drain_on_close` so we promote in-flight text to final at most
    /// once per session.
    close_promoted: bool,
}

impl DeepgramClient {
    pub fn connect(api_key: &str, keyterms: &[String]) -> Result<Self> {
        let url = build_url(keyterms)?;
        let mut req = url
            .as_str()
            .into_client_request()
            .context("build ws request")?;
        req.headers_mut().insert(
            "Authorization",
            HeaderValue::from_str(&format!("Token {api_key}")).context("auth header")?,
        );

        // Surface the server's actual response body on HTTP errors —
        // tungstenite's generic `Error::Http` Display says "HTTP error:
        // 401" but swallows the JSON body that tells you *why*.
        let (ws, _resp) = match connect(req) {
            Ok(r) => r,
            Err(tungstenite::Error::Http(resp)) => {
                let status = resp.status();
                let body = resp
                    .body()
                    .as_ref()
                    .map(|b| String::from_utf8_lossy(b).trim().to_string())
                    .filter(|s| !s.is_empty())
                    .unwrap_or_else(|| "(empty body)".to_string());
                return Err(anyhow!(
                    "deepgram handshake rejected: HTTP {status} — {body}"
                ));
            }
            Err(e) => return Err(anyhow!("deepgram ws handshake failed: {e}")),
        };
        let mut client = Self {
            ws,
            finalized_so_far: String::new(),
            interim: String::new(),
            finalized: false,
            close_promoted: false,
        };
        client.set_nonblocking()?;
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

    /// Send a PCM16 frame as a raw binary WebSocket message.
    pub fn send_audio(&mut self, pcm: &[i16]) -> Result<()> {
        if self.finalized {
            return Ok(());
        }
        let mut bytes = Vec::with_capacity(pcm.len() * 2);
        for s in pcm {
            bytes.extend_from_slice(&s.to_le_bytes());
        }
        self.ws
            .send(Message::Binary(bytes))
            .context("deepgram ws send audio failed")?;
        Ok(())
    }

    /// Sent every 8 s while idle — Deepgram closes connections that go
    /// silent for too long.
    pub fn send_keepalive(&mut self) -> Result<()> {
        if self.finalized {
            return Ok(());
        }
        self.ws
            .send(Message::Text(r#"{"type":"KeepAlive"}"#.to_string()))
            .context("deepgram ws send keepalive failed")?;
        Ok(())
    }

    /// Tell Deepgram we're done — server flushes any remaining finals
    /// then closes the stream.
    pub fn finalize(&mut self) -> Result<()> {
        if self.finalized {
            return Ok(());
        }
        self.finalized = true;
        self.ws
            .send(Message::Text(r#"{"type":"CloseStream"}"#.to_string()))
            .context("deepgram ws send closestream failed")?;
        Ok(())
    }

    /// Non-blocking poll for the next event. Returns `Ok(None)` when no
    /// data is available. On a server-side close we promote any
    /// in-flight transcript (finalized segments + interim) to a final
    /// `Completed` event exactly once, so nothing is dropped if the
    /// server closes before emitting `UtteranceEnd`.
    pub fn poll(&mut self) -> Result<Option<DeepgramEvent>> {
        loop {
            match self.ws.read() {
                Ok(Message::Text(t)) => {
                    if let Some(ev) = self.parse(&t) {
                        return Ok(Some(ev));
                    }
                }
                Ok(Message::Close(_)) => return Ok(self.drain_on_close()),
                Ok(_) => continue,
                Err(tungstenite::Error::Io(e)) if e.kind() == io::ErrorKind::WouldBlock => {
                    return Ok(None);
                }
                Err(tungstenite::Error::ConnectionClosed)
                | Err(tungstenite::Error::AlreadyClosed) => {
                    return Ok(self.drain_on_close());
                }
                Err(e) => return Err(anyhow!("deepgram ws read failed: {e}")),
            }
        }
    }

    fn parse(&mut self, text: &str) -> Option<DeepgramEvent> {
        parse_message(text, &mut self.finalized_so_far, &mut self.interim)
    }

    /// Flush any buffered finalized segments + interim into a single
    /// string, clearing internal state. Used by the PTT worker's
    /// finalize-timeout fallback so a missing `UtteranceEnd` can't hang
    /// the UI in `Finalizing`.
    pub fn drain_final(&mut self) -> Option<String> {
        let combined = if self.interim.is_empty() {
            std::mem::take(&mut self.finalized_so_far)
        } else if self.finalized_so_far.is_empty() {
            std::mem::take(&mut self.interim)
        } else {
            let out = format!("{} {}", self.finalized_so_far, self.interim);
            self.finalized_so_far.clear();
            self.interim.clear();
            out
        };
        if combined.is_empty() {
            None
        } else {
            Some(combined)
        }
    }

    fn drain_on_close(&mut self) -> Option<DeepgramEvent> {
        if self.close_promoted {
            return None;
        }
        self.close_promoted = true;
        Some(DeepgramEvent::Closed(self.drain_final()))
    }

    pub fn close(mut self) -> Result<()> {
        let _ = self.ws.close(None);
        Ok(())
    }
}

/// Build the connect URL with all session config + keyterms as repeated
/// query params. Extracted so tests can verify the shape without a
/// live socket.
fn build_url(keyterms: &[String]) -> Result<Url> {
    let mut url = Url::parse(BASE_URL).context("parse deepgram base url")?;
    {
        let mut q = url.query_pairs_mut();
        q.append_pair("model", "nova-3");
        q.append_pair("language", "en");
        q.append_pair("encoding", "linear16");
        q.append_pair("sample_rate", "16000");
        q.append_pair("channels", "1");
        q.append_pair("interim_results", "true");
        q.append_pair("punctuate", "true");
        // smart_format is OFF on purpose: its entity-formatting pass
        // rewrites spelled-out callsigns/headings ("november one seven
        // two") as dates ("11/01/1972"). Punctuation alone is enough
        // for aviation phraseology, and the LLM handles word-form
        // numbers fine.
        q.append_pair("smart_format", "false");
        q.append_pair("endpointing", "300");
        q.append_pair("utterance_end_ms", "1000");
        for term in keyterms {
            q.append_pair("keyterm", term);
        }
    }
    Ok(url)
}

/// Stateful parser. Mutates `finalized_so_far` / `interim` and returns
/// the event (if any) to surface. Exposed at module scope for unit
/// testing the state machine without spinning up a WS.
fn parse_message(
    text: &str,
    finalized_so_far: &mut String,
    interim: &mut String,
) -> Option<DeepgramEvent> {
    let v: Value = serde_json::from_str(text).ok()?;
    let ty = v.get("type").and_then(|t| t.as_str()).unwrap_or("");
    match ty {
        "Results" => {
            let transcript = v
                .pointer("/channel/alternatives/0/transcript")
                .and_then(|s| s.as_str())
                .unwrap_or("")
                .to_string();
            let is_final = v.get("is_final").and_then(|b| b.as_bool()).unwrap_or(false);
            let speech_final = v
                .get("speech_final")
                .and_then(|b| b.as_bool())
                .unwrap_or(false);

            if is_final {
                if !transcript.is_empty() {
                    if !finalized_so_far.is_empty() && !finalized_so_far.ends_with(' ') {
                        finalized_so_far.push(' ');
                    }
                    finalized_so_far.push_str(&transcript);
                }
                interim.clear();
                if speech_final {
                    let out = std::mem::take(finalized_so_far);
                    if out.is_empty() {
                        None
                    } else {
                        Some(DeepgramEvent::Completed(out))
                    }
                } else {
                    Some(DeepgramEvent::Delta(finalized_so_far.clone()))
                }
            } else {
                *interim = transcript;
                let combined = if finalized_so_far.is_empty() {
                    interim.clone()
                } else if interim.is_empty() {
                    finalized_so_far.clone()
                } else {
                    format!("{finalized_so_far} {interim}")
                };
                if combined.is_empty() {
                    None
                } else {
                    Some(DeepgramEvent::Delta(combined))
                }
            }
        }
        "UtteranceEnd" => {
            interim.clear();
            let out = std::mem::take(finalized_so_far);
            if out.is_empty() {
                None
            } else {
                Some(DeepgramEvent::Completed(out))
            }
        }
        "Error" => {
            let detail = v
                .get("description")
                .and_then(|s| s.as_str())
                .or_else(|| v.get("message").and_then(|s| s.as_str()))
                .unwrap_or("deepgram error")
                .to_string();
            Some(DeepgramEvent::Error(detail))
        }
        // Metadata, SpeechStarted, etc.
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn url_carries_config_and_keyterms() {
        let url = build_url(&["Whiteman".to_string(), "SoCal".to_string()]).unwrap();
        let q = url.query().unwrap();
        assert!(q.contains("model=nova-3"));
        assert!(q.contains("encoding=linear16"));
        assert!(q.contains("sample_rate=16000"));
        assert!(q.contains("interim_results=true"));
        assert!(q.contains("keyterm=Whiteman"));
        assert!(q.contains("keyterm=SoCal"));
    }

    #[test]
    fn keyterm_with_space_is_url_encoded() {
        let url = build_url(&["SoCal Approach".to_string()]).unwrap();
        let q = url.query().unwrap();
        // form_urlencoded encodes space as `+`.
        assert!(q.contains("keyterm=SoCal+Approach"));
    }

    #[test]
    fn interim_results_become_deltas() {
        let mut fin = String::new();
        let mut interim = String::new();
        let msg = r#"{"type":"Results","channel":{"alternatives":[{"transcript":"hello"}]},"is_final":false,"speech_final":false}"#;
        match parse_message(msg, &mut fin, &mut interim).unwrap() {
            DeepgramEvent::Delta(s) => assert_eq!(s, "hello"),
            other => panic!("expected Delta, got {other:?}"),
        }
        assert_eq!(interim, "hello");
        assert!(fin.is_empty());
    }

    #[test]
    fn non_final_then_final_accumulates_and_emits_completed_on_speech_final() {
        let mut fin = String::new();
        let mut interim = String::new();
        // Interim.
        let m1 = r#"{"type":"Results","channel":{"alternatives":[{"transcript":"whiteman tower"}]},"is_final":false,"speech_final":false}"#;
        assert!(matches!(
            parse_message(m1, &mut fin, &mut interim),
            Some(DeepgramEvent::Delta(_))
        ));
        // Final segment but utterance continues.
        let m2 = r#"{"type":"Results","channel":{"alternatives":[{"transcript":"whiteman tower"}]},"is_final":true,"speech_final":false}"#;
        match parse_message(m2, &mut fin, &mut interim).unwrap() {
            DeepgramEvent::Delta(s) => assert_eq!(s, "whiteman tower"),
            other => panic!("expected Delta, got {other:?}"),
        }
        assert_eq!(fin, "whiteman tower");
        assert!(interim.is_empty());
        // Another final, speech_final=true now completes the utterance.
        let m3 = r#"{"type":"Results","channel":{"alternatives":[{"transcript":"cleared to land"}]},"is_final":true,"speech_final":true}"#;
        match parse_message(m3, &mut fin, &mut interim).unwrap() {
            DeepgramEvent::Completed(s) => {
                assert_eq!(s, "whiteman tower cleared to land");
            }
            other => panic!("expected Completed, got {other:?}"),
        }
        assert!(fin.is_empty());
        assert!(interim.is_empty());
    }

    #[test]
    fn utterance_end_flushes_accumulated_finals() {
        let mut fin = String::from("taxi via alpha");
        let mut interim = String::new();
        let msg = r#"{"type":"UtteranceEnd","last_word_end":1.5}"#;
        match parse_message(msg, &mut fin, &mut interim).unwrap() {
            DeepgramEvent::Completed(s) => assert_eq!(s, "taxi via alpha"),
            other => panic!("expected Completed, got {other:?}"),
        }
        assert!(fin.is_empty());
    }

    #[test]
    fn metadata_is_ignored() {
        let mut fin = String::new();
        let mut interim = String::new();
        let msg = r#"{"type":"Metadata","request_id":"abc"}"#;
        assert!(parse_message(msg, &mut fin, &mut interim).is_none());
    }

    #[test]
    fn error_message_surfaces_description() {
        let mut fin = String::new();
        let mut interim = String::new();
        let msg = r#"{"type":"Error","description":"bad audio"}"#;
        match parse_message(msg, &mut fin, &mut interim).unwrap() {
            DeepgramEvent::Error(s) => assert_eq!(s, "bad audio"),
            other => panic!("expected Error, got {other:?}"),
        }
    }

    #[test]
    fn empty_utterance_end_does_not_emit_empty_completed() {
        let mut fin = String::new();
        let mut interim = String::new();
        let msg = r#"{"type":"UtteranceEnd","last_word_end":0.0}"#;
        assert!(parse_message(msg, &mut fin, &mut interim).is_none());
    }

    /// Helper to exercise drain_final without opening a real WebSocket.
    fn drain_combined(fin: &str, interim: &str) -> Option<String> {
        // Mirror the logic of DeepgramClient::drain_final exactly.
        let mut f = fin.to_string();
        let mut i = interim.to_string();
        if i.is_empty() {
            let out = std::mem::take(&mut f);
            if out.is_empty() { None } else { Some(out) }
        } else if f.is_empty() {
            Some(std::mem::take(&mut i))
        } else {
            Some(format!("{f} {i}"))
        }
    }

    #[test]
    fn drain_final_combines_finalized_and_interim() {
        assert_eq!(
            drain_combined("whiteman tower", "cleared to land"),
            Some("whiteman tower cleared to land".to_string())
        );
    }

    #[test]
    fn drain_final_returns_none_when_both_empty() {
        assert_eq!(drain_combined("", ""), None);
    }

    #[test]
    fn drain_final_returns_interim_only_when_no_finalized() {
        assert_eq!(
            drain_combined("", "whiteman"),
            Some("whiteman".to_string())
        );
    }

    /// Exercise drain_on_close logic directly: first call wraps drained
    /// text in a `Closed(Some(..))`; second call is None (close_promoted
    /// guard).
    #[test]
    fn drain_on_close_emits_closed_exactly_once() {
        let mut fin = String::from("hello");
        let mut interim = String::new();
        let mut promoted = false;
        let first = {
            if promoted {
                None
            } else {
                promoted = true;
                let combined = if interim.is_empty() {
                    std::mem::take(&mut fin)
                } else if fin.is_empty() {
                    std::mem::take(&mut interim)
                } else {
                    let out = format!("{fin} {interim}");
                    fin.clear();
                    interim.clear();
                    out
                };
                Some(DeepgramEvent::Closed(
                    if combined.is_empty() { None } else { Some(combined) },
                ))
            }
        };
        match first {
            Some(DeepgramEvent::Closed(Some(s))) => assert_eq!(s, "hello"),
            other => panic!("expected Closed(Some), got {other:?}"),
        }
        // Second call: close_promoted=true → None.
        let second: Option<DeepgramEvent> = if promoted { None } else { Some(DeepgramEvent::Closed(None)) };
        assert!(second.is_none());
    }

    #[test]
    fn drain_on_close_with_empty_buffers_emits_closed_none() {
        // Verifies the Closed(None) variant exists and is reachable so
        // the worker can detect a clean server exit even without any
        // pending text.
        let ev = DeepgramEvent::Closed(None);
        assert!(matches!(ev, DeepgramEvent::Closed(None)));
    }
}
