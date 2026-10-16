//! Minimal MCP server exposing the pilot's tools to a Claude Code child
//! process (see `claude_code.rs`).
//!
//! Streamable-HTTP transport, JSON responses only: Claude Code POSTs one
//! JSON-RPC message per request and we answer with a single JSON body.
//! We never push server-to-client messages, so GET (the optional SSE
//! stream) gets 405 and notifications get 202. Methods: `initialize`,
//! `tools/list`, `tools/call`, `ping`; anything else is "method not
//! found" (Claude Code probes `server/discover` first and copes).
//!
//! Tool calls go through `conversation::execute_tool_call`, so they're
//! dispatched, counted, and logged exactly like the API tool loop's.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use anyhow::{anyhow, Result};
use serde_json::{json, Value};

use crate::bus::SimBus;
use crate::llm::conversation::{ends_turn, execute_tool_call};
use crate::llm::tools::ToolContext;

/// MCP server name; Claude Code exposes our tools as
/// `mcp__{SERVER_NAME}__{tool}`.
pub const SERVER_NAME: &str = "sim_pilot";

/// Appended to `sleep` / `mission_complete` results. The Claude Code
/// worker also interrupts the turn (see `end_turn_requested`), since
/// models don't reliably stop on their own.
pub const END_TURN_NOTE: &str = "\n\nEnd your turn now: send no further tool calls or text.";

pub struct McpServer {
    pub url: String,
    /// Set when a turn-ending tool (`sleep`, `mission_complete`) runs, so
    /// the worker can end Claude Code's turn the way the API loop does.
    pub end_turn_requested: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl McpServer {
    /// Serve on an ephemeral localhost port until dropped.
    pub fn start(ctx: Arc<ToolContext>, bus: Option<SimBus>) -> Result<Self> {
        let server = tiny_http::Server::http("127.0.0.1:0")
            .map_err(|e| anyhow!("starting MCP server: {e}"))?;
        let port = server
            .server_addr()
            .to_ip()
            .ok_or_else(|| anyhow!("MCP server has no IP address"))?
            .port();
        let stop = Arc::new(AtomicBool::new(false));
        let thread_stop = stop.clone();
        let end_turn_requested = Arc::new(AtomicBool::new(false));
        let end_turn = end_turn_requested.clone();
        let handle = std::thread::Builder::new()
            .name("mcp-server".into())
            .spawn(move || {
                while !thread_stop.load(Ordering::Acquire) {
                    match server.recv_timeout(Duration::from_millis(200)) {
                        Ok(Some(request)) => serve(request, &ctx, bus.as_ref(), &end_turn),
                        Ok(None) => continue,
                        Err(_) => return,
                    }
                }
            })?;
        Ok(Self {
            url: format!("http://127.0.0.1:{port}/mcp"),
            end_turn_requested,
            stop,
            handle: Some(handle),
        })
    }

    /// `--mcp-config` JSON pointing Claude Code at this server.
    pub fn claude_config_json(&self) -> String {
        json!({ "mcpServers": { SERVER_NAME: { "type": "http", "url": self.url } } }).to_string()
    }
}

impl Drop for McpServer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn serve(
    mut request: tiny_http::Request,
    ctx: &ToolContext,
    bus: Option<&SimBus>,
    end_turn_requested: &AtomicBool,
) {
    if *request.method() != tiny_http::Method::Post {
        let _ = request.respond(tiny_http::Response::empty(405));
        return;
    }
    let mut body = String::new();
    if request.as_reader().read_to_string(&mut body).is_err() {
        let _ = request.respond(tiny_http::Response::empty(400));
        return;
    }
    let (response, ends_turn_call) = match serde_json::from_str::<Value>(&body) {
        Ok(message) => {
            let ends = message.get("method").and_then(Value::as_str) == Some("tools/call")
                && message
                    .pointer("/params/name")
                    .and_then(Value::as_str)
                    .is_some_and(ends_turn);
            (handle_jsonrpc(&message, ctx, bus), ends)
        }
        Err(e) => (Some(error_response(Value::Null, -32700, &format!("parse error: {e}"))), false),
    };
    let _ = match response {
        // Notifications (no id) get no body.
        None => request.respond(tiny_http::Response::empty(202)),
        Some(v) => {
            let header = tiny_http::Header::from_bytes(&b"Content-Type"[..], &b"application/json"[..])
                .expect("static header");
            request.respond(tiny_http::Response::from_string(v.to_string()).with_header(header))
        }
    };
    // Only after the result is on its way, so the interrupt lands after
    // Claude Code has it.
    if ends_turn_call {
        end_turn_requested.store(true, Ordering::Release);
    }
}

/// Answer one JSON-RPC message; `None` for notifications.
pub fn handle_jsonrpc(message: &Value, ctx: &ToolContext, bus: Option<&SimBus>) -> Option<Value> {
    let id = message.get("id")?.clone();
    let method = message.get("method").and_then(Value::as_str).unwrap_or("");
    let params = message.get("params").cloned().unwrap_or(Value::Null);
    let result = match method {
        "initialize" => json!({
            "protocolVersion": params
                .get("protocolVersion")
                .cloned()
                .unwrap_or(json!("2025-06-18")),
            "capabilities": { "tools": {} },
            "serverInfo": { "name": SERVER_NAME, "version": env!("CARGO_PKG_VERSION") },
        }),
        "ping" => json!({}),
        "tools/list" => {
            let tools: Vec<Value> = crate::llm::tools::tool_schemas(ctx.mission_complete_tx.is_some())
                .into_iter()
                .map(|t| json!({ "name": t.name, "description": t.description, "inputSchema": t.parameters }))
                .collect();
            json!({ "tools": tools })
        }
        "tools/call" => {
            let Some(name) = params.get("name").and_then(Value::as_str) else {
                return Some(error_response(id, -32602, "tools/call needs a tool name"));
            };
            let arguments = params.get("arguments").cloned().unwrap_or_else(|| json!({}));
            let result = execute_tool_call(name, &arguments.to_string(), ctx, bus);
            let mut text = result.output;
            if ends_turn(name) {
                text.push_str(END_TURN_NOTE);
            }
            json!({ "content": [{ "type": "text", "text": text }], "isError": !result.ok })
        }
        other => return Some(error_response(id, -32601, &format!("method not found: {other}"))),
    };
    Some(json!({ "jsonrpc": "2.0", "id": id, "result": result }))
}

fn error_response(id: Value, code: i64, message: &str) -> Value {
    json!({ "jsonrpc": "2.0", "id": id, "error": { "code": code, "message": message } })
}
