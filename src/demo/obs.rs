//! OBS control over obs-websocket v5 (built into OBS 28+): switch
//! scenes, start and stop recording, set the radio caption.
//!
//! Requests go to a worker thread that connects lazily and reconnects
//! after failures, so a missing OBS costs one warning and nothing else.
//! Config: `OBS_WS_URL` (default `ws://127.0.0.1:4455`), and
//! `OBS_WS_PASSWORD` when OBS has authentication on.

use std::net::TcpStream;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Result};
use base64::Engine;
use crossbeam_channel::{unbounded, Sender};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use tungstenite::stream::MaybeTlsStream;
use tungstenite::{Message, WebSocket};

use crate::bus::{LogKind, SimBus};

type Ws = WebSocket<MaybeTlsStream<TcpStream>>;

/// The obs-websocket authentication string for `password`, given the
/// server's salt and challenge.
pub fn auth_string(password: &str, salt: &str, challenge: &str) -> String {
    let b64 = base64::engine::general_purpose::STANDARD;
    let secret = b64.encode(Sha256::digest(format!("{password}{salt}").as_bytes()));
    b64.encode(Sha256::digest(format!("{secret}{challenge}").as_bytes()))
}

/// A connected client.
pub struct ObsClient {
    ws: Ws,
    next_id: u64,
}

impl ObsClient {
    pub fn connect(url: &str, password: Option<&str>) -> Result<Self> {
        let (mut ws, _) = tungstenite::connect(url).map_err(|e| anyhow!("connecting to OBS at {url}: {e}"))?;
        if let MaybeTlsStream::Plain(s) = ws.get_mut() {
            let _ = s.set_read_timeout(Some(Duration::from_secs(5)));
        }
        let hello = read_op(&mut ws, 0)?;
        let mut identify = json!({"rpcVersion": 1, "eventSubscriptions": 0});
        if let Some(auth) = hello.get("authentication") {
            let pw = password.ok_or_else(|| anyhow!("OBS wants a password: set OBS_WS_PASSWORD"))?;
            let (salt, challenge) = (auth["salt"].as_str().unwrap_or(""), auth["challenge"].as_str().unwrap_or(""));
            identify["authentication"] = json!(auth_string(pw, salt, challenge));
        }
        ws.send(Message::Text(json!({"op": 1, "d": identify}).to_string()))?;
        read_op(&mut ws, 2)?;
        Ok(Self { ws, next_id: 1 })
    }

    /// Send a request and wait for its response data.
    pub fn request(&mut self, request_type: &str, data: Value) -> Result<Value> {
        let id = self.next_id.to_string();
        self.next_id += 1;
        let msg = json!({"op": 6, "d": {"requestType": request_type, "requestId": id, "requestData": data}});
        self.ws.send(Message::Text(msg.to_string()))?;
        loop {
            let d = read_op(&mut self.ws, 7)?;
            if d["requestId"].as_str() != Some(id.as_str()) {
                continue;
            }
            if d["requestStatus"]["result"].as_bool() != Some(true) {
                bail!("OBS {request_type}: {}", d["requestStatus"]["comment"].as_str().unwrap_or("failed"));
            }
            return Ok(d["responseData"].clone());
        }
    }

    pub fn scene_names(&mut self) -> Result<Vec<String>> {
        let r = self.request("GetSceneList", json!({}))?;
        Ok(r["scenes"].as_array().into_iter().flatten().filter_map(|s| s["sceneName"].as_str().map(String::from)).collect())
    }
}

/// The next message with opcode `op` (skipping events), as its `d`.
fn read_op(ws: &mut Ws, op: u64) -> Result<Value> {
    loop {
        match ws.read()? {
            Message::Text(t) => {
                let v: Value = serde_json::from_str(&t)?;
                if v["op"].as_u64() == Some(op) {
                    return Ok(v["d"].clone());
                }
            }
            Message::Close(_) => bail!("OBS closed the connection"),
            _ => {}
        }
    }
}

enum ObsWork {
    Request(String, Value),
}

/// Fire-and-forget OBS requests, on a worker thread.
#[derive(Clone)]
pub struct ObsHandle {
    tx: Sender<ObsWork>,
}

impl ObsHandle {
    pub fn spawn(bus: SimBus) -> Self {
        let url = std::env::var("OBS_WS_URL").unwrap_or_else(|_| "ws://127.0.0.1:4455".into());
        let password = std::env::var("OBS_WS_PASSWORD").ok().filter(|p| !p.is_empty());
        let (tx, rx) = unbounded::<ObsWork>();
        thread::Builder::new()
            .name("demo-obs".into())
            .spawn(move || {
                let mut client: Option<ObsClient> = None;
                let mut warned = false;
                let mut retry_at = Instant::now();
                for ObsWork::Request(kind, data) in rx {
                    if client.is_none() && Instant::now() >= retry_at {
                        match ObsClient::connect(&url, password.as_deref()) {
                            Ok(c) => {
                                bus.push_log_kind(LogKind::Demo, format!("OBS connected at {url}"));
                                client = Some(c);
                            }
                            Err(e) => {
                                if !warned {
                                    bus.push_log_kind(LogKind::Error, format!("[demo] OBS: {e:#} (switch scenes by hand)"));
                                    warned = true;
                                }
                                retry_at = Instant::now() + Duration::from_secs(10);
                            }
                        }
                    }
                    let Some(c) = client.as_mut() else { continue };
                    if let Err(e) = c.request(&kind, data) {
                        bus.push_log_kind(LogKind::Error, format!("[demo] OBS {kind}: {e:#}"));
                        if e.to_string().contains("closed") || e.downcast_ref::<tungstenite::Error>().is_some() {
                            client = None;
                        }
                    } else if kind == "StartRecord" {
                        bus.push_log_kind(LogKind::Demo, "OBS recording".to_string());
                    }
                }
            })
            .expect("spawn demo-obs");
        Self { tx }
    }

    fn send(&self, kind: &str, data: Value) {
        let _ = self.tx.send(ObsWork::Request(kind.to_string(), data));
    }

    pub fn scene(&self, name: &str) {
        self.send("SetCurrentProgramScene", json!({"sceneName": name}));
    }

    pub fn start_record(&self) {
        self.send("StartRecord", json!({}));
    }

    pub fn stop_record(&self) {
        self.send("StopRecord", json!({}));
    }

    /// A chapter marker in the recording (OBS 30.2+, hybrid MP4).
    pub fn chapter(&self, name: &str) {
        self.send("CreateRecordChapter", json!({"chapterName": name}));
    }

    /// Set a text source's text (the radio caption).
    pub fn caption(&self, input: &str, text: &str) {
        self.send("SetInputSettings", json!({"inputName": input, "inputSettings": {"text": text}}));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auth_string_matches_the_protocol_example() {
        // obs-websocket v5 spec: password "supersecretpassword", salt
        // "lM1GncleQOaCu9lT1yeUZhFYnqhsLLP1G5lAGo3ixaI=", challenge
        // "+IxH4CnCiqpX1rM9scsNynZzbOe4KhDeYcTNS3PDaeY=".
        assert_eq!(
            auth_string(
                "supersecretpassword",
                "lM1GncleQOaCu9lT1yeUZhFYnqhsLLP1G5lAGo3ixaI=",
                "+IxH4CnCiqpX1rM9scsNynZzbOe4KhDeYcTNS3PDaeY="
            ),
            "1Ct943GAT+6YQUUX47Ia/ncufilbe6+oD6lY+5kaCu4="
        );
    }
}
