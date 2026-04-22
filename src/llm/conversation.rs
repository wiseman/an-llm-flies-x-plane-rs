//! Multi-turn Responses-API conversation loop. Mirrors llm/conversation.py.
//!
//! `Conversation` holds the pinned system prompt + rotating message history.
//! History grows unbounded — there is no compaction.
//! `run_conversation_loop` pulls one `IncomingMessage` off the queue, appends
//! it, then drives the Responses API in a loop: each tool call is dispatched
//! and its output posted back as a `function_call_output` item. The loop
//! terminates on (a) no function calls in the response, (b) a `sleep` tool
//! call, or (c) the wall-clock budget running out.

use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::{Duration, Instant};

use crossbeam_channel::Receiver;
use serde_json::{json, Value};

use crate::bus::{LogKind, SimBus};
use crate::llm::responses_client::ResponsesBackend;
use crate::llm::tools::{dispatch_tool, tool_schemas, ToolContext};

pub const SYSTEM_PROMPT: &str = include_str!("system_prompt.md");
pub const SYSTEM_PROMPT_REALISTIC_OVERLAY: &str = include_str!("system_prompt_realistic_overlay.md");
pub const SYSTEM_PROMPT_REALISTIC: &str = concat!(
    include_str!("system_prompt.md"),
    "\n",
    include_str!("system_prompt_realistic_overlay.md"),
);
const RUNTIME_NORMAL_NOTICE: &str =
    "Return to normal pilot mode. Disregard any realistic-pilot overlay delivered earlier; \
resume the base persona defined at the top of this conversation.";
pub const DEFAULT_PER_REQUEST_TIMEOUT_S: u64 = 60;
pub const DEFAULT_TOTAL_WALL_BUDGET_S: u64 = 120;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PilotMode {
    Normal,
    Realistic,
}

impl PilotMode {
    pub fn system_prompt(self) -> &'static str {
        match self {
            PilotMode::Normal => SYSTEM_PROMPT,
            PilotMode::Realistic => SYSTEM_PROMPT_REALISTIC,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            PilotMode::Normal => "normal",
            PilotMode::Realistic => "realistic",
        }
    }

    pub fn parse(raw: &str) -> Option<Self> {
        match raw.trim().to_ascii_lowercase().as_str() {
            "normal" => Some(PilotMode::Normal),
            "realistic" => Some(PilotMode::Realistic),
            _ => None,
        }
    }

    /// Text to inject into rotating history when the mode is switched at
    /// runtime via `/mode`. For realistic, this is the overlay verbatim so
    /// the model sees the same instructions that `--pilot-mode realistic`
    /// would have baked into the system prompt. For normal, it's a short
    /// directive cancelling any earlier overlay.
    pub fn runtime_overlay(self) -> &'static str {
        match self {
            PilotMode::Normal => RUNTIME_NORMAL_NOTICE,
            PilotMode::Realistic => SYSTEM_PROMPT_REALISTIC_OVERLAY,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IncomingSource {
    Operator,
    Atc,
    Heartbeat,
    ModeSwitch(PilotMode),
}

impl IncomingSource {
    pub fn tag(&self) -> &'static str {
        match self {
            IncomingSource::Operator => "OPERATOR",
            IncomingSource::Atc => "ATC",
            IncomingSource::Heartbeat => "HEARTBEAT",
            IncomingSource::ModeSwitch(_) => "MODE_SWITCH",
        }
    }
}

#[derive(Debug, Clone)]
pub struct IncomingMessage {
    pub source: IncomingSource,
    pub text: String,
}

impl IncomingMessage {
    pub fn operator(text: impl Into<String>) -> Self {
        Self { source: IncomingSource::Operator, text: text.into() }
    }
    pub fn atc(text: impl Into<String>) -> Self {
        Self { source: IncomingSource::Atc, text: text.into() }
    }
    pub fn heartbeat(text: impl Into<String>) -> Self {
        Self { source: IncomingSource::Heartbeat, text: text.into() }
    }
    pub fn mode_switch(mode: PilotMode) -> Self {
        Self { source: IncomingSource::ModeSwitch(mode), text: mode.label().to_string() }
    }
}

fn user_item(text: &str) -> Value {
    json!({
        "role": "user",
        "content": [{"type": "input_text", "text": text}]
    })
}

fn system_item(text: &str) -> Value {
    json!({
        "role": "system",
        "content": [{"type": "input_text", "text": text}]
    })
}

pub struct Conversation {
    pub system_prompt: String,
    pub pinned_items: Vec<Value>,
    pub rotating_items: Vec<Value>,
}

impl Conversation {
    pub fn new(system_prompt: impl Into<String>) -> Self {
        let prompt = system_prompt.into();
        let pinned = vec![system_item(&prompt)];
        Self {
            system_prompt: prompt,
            pinned_items: pinned,
            rotating_items: Vec::new(),
        }
    }

    /// Replace the pinned system prompt in place. History (`rotating_items`)
    /// is preserved so the model stays in the same turn; only the leading
    /// system item is rewritten.
    pub fn set_system_prompt(&mut self, prompt: impl Into<String>) {
        let prompt = prompt.into();
        if self.pinned_items.is_empty() {
            self.pinned_items.push(system_item(&prompt));
        } else {
            self.pinned_items[0] = system_item(&prompt);
        }
        self.system_prompt = prompt;
    }

    pub fn append_operator_message(&mut self, text: &str) {
        self.rotating_items.push(user_item(&format!("[OPERATOR] {}", text)));
    }
    pub fn append_atc_message(&mut self, text: &str) {
        self.rotating_items.push(user_item(&format!("[ATC] {}", text)));
    }
    pub fn append_heartbeat_message(&mut self, text: &str) {
        self.rotating_items.push(user_item(&format!("[HEARTBEAT] {}", text)));
    }
    pub fn append_mode_switch_message(&mut self, text: &str) {
        self.rotating_items.push(user_item(&format!("[MODE_SWITCH] {}", text)));
    }
    pub fn append_response_items(&mut self, items: &[Value]) {
        for item in items {
            self.rotating_items.push(item.clone());
        }
    }
    pub fn append_function_call_output(&mut self, call_id: &str, output: &str) -> anyhow::Result<()> {
        if call_id.is_empty() {
            return Err(anyhow::anyhow!("function_call_output requires a call_id"));
        }
        self.rotating_items.push(json!({
            "type": "function_call_output",
            "call_id": call_id,
            "output": output,
        }));
        Ok(())
    }

    pub fn build_input(&self, active_profiles_summary: &str) -> Vec<Value> {
        let summary = system_item(&format!(
            "Active profiles: {}",
            if active_profiles_summary.is_empty() { "(none)" } else { active_profiles_summary }
        ));
        let mut out = Vec::with_capacity(self.pinned_items.len() + self.rotating_items.len() + 1);
        out.extend(self.pinned_items.iter().cloned());
        out.extend(self.rotating_items.iter().cloned());
        out.push(summary);
        out
    }
}

pub fn run_conversation_loop(
    client: &dyn ResponsesBackend,
    tool_context: &ToolContext,
    input_queue: &Receiver<IncomingMessage>,
    stop: Arc<AtomicBool>,
    per_request_timeout_s: u64,
    total_wall_budget_s: u64,
    initial_mode: PilotMode,
    bus: Option<&SimBus>,
) {
    let mut conv = Conversation::new(initial_mode.system_prompt());
    emit_log_kind(
        bus,
        LogKind::Mode,
        &format!("pilot mode: {}", initial_mode.label()),
    );
    while !stop.load(Ordering::Acquire) {
        let message = match input_queue.recv_timeout(Duration::from_millis(500)) {
            Ok(m) => m,
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => continue,
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => return,
        };
        if let Err(e) = handle_message(
            &mut conv,
            client,
            tool_context,
            &message,
            per_request_timeout_s,
            total_wall_budget_s,
            bus,
        ) {
            emit_log_kind(
                bus,
                LogKind::Error,
                &format!("[llm-worker] error handling message: {:?}", e),
            );
        }
    }
}

fn handle_message(
    conv: &mut Conversation,
    client: &dyn ResponsesBackend,
    tool_context: &ToolContext,
    message: &IncomingMessage,
    per_request_timeout_s: u64,
    total_wall_budget_s: u64,
    bus: Option<&SimBus>,
) -> anyhow::Result<()> {
    match &message.source {
        IncomingSource::Operator => conv.append_operator_message(&message.text),
        IncomingSource::Heartbeat => {
            conv.append_heartbeat_message(&message.text);
            emit_log_kind(bus, LogKind::Heartbeat, &message.text);
        }
        IncomingSource::Atc => {
            conv.append_atc_message(&message.text);
            emit_radio(bus, &format!("[atc] {}", message.text));
        }
        IncomingSource::ModeSwitch(mode) => {
            conv.append_mode_switch_message(mode.runtime_overlay());
            emit_log_kind(
                bus,
                LogKind::Mode,
                &format!(
                    "pilot mode switched to {} (overlay injected as user message)",
                    mode.label()
                ),
            );
            return Ok(());
        }
    }

    let deadline = Instant::now() + Duration::from_secs(total_wall_budget_s);
    loop {
        if Instant::now() >= deadline {
            emit_log_kind(bus, LogKind::Error, "wall-clock budget exceeded; ending turn");
            return Ok(());
        }
        let profiles = tool_context.pilot.lock().list_profile_names().join(", ");
        let input = conv.build_input(&profiles);
        let remaining = deadline.saturating_duration_since(Instant::now()).as_secs();
        let timeout = per_request_timeout_s.min(remaining.max(1));
        let response = client.create_response(&input, &tool_schemas(), timeout)?;
        log_usage(&response, client, bus);
        let Some(output_items) = response.get("output").and_then(|v| v.as_array()).cloned() else {
            emit_log_kind(
                bus,
                LogKind::Error,
                &format!("unexpected output shape: {}", response),
            );
            return Ok(());
        };
        conv.append_response_items(&output_items);

        let function_calls: Vec<Value> = output_items
            .iter()
            .filter(|item| item.get("type").and_then(|v| v.as_str()) == Some("function_call"))
            .cloned()
            .collect();

        if function_calls.is_empty() {
            emit_assistant_text(&output_items, bus);
            return Ok(());
        }

        let mut slept = false;
        for call in function_calls {
            let call_id = call.get("call_id").or_else(|| call.get("id"));
            let call_id = call_id.and_then(|v| v.as_str()).unwrap_or("").to_string();
            if call_id.is_empty() {
                emit_log_kind(
                    bus,
                    LogKind::Error,
                    &format!("function_call missing call_id: {}", call),
                );
                continue;
            }
            let name = call.get("name").and_then(|v| v.as_str()).unwrap_or("?").to_string();
            let result = dispatch_tool(&call, tool_context);
            if name == "sleep" {
                slept = true;
            }
            emit_log_kind(
                bus,
                LogKind::ToolCall,
                &format!("tool {} -> {}", name, result),
            );
            conv.append_function_call_output(&call_id, &result)?;
        }
        if slept {
            return Ok(());
        }
    }
}

fn emit_log_kind(bus: Option<&SimBus>, kind: LogKind, text: &str) {
    match bus {
        Some(b) => b.push_log_kind(kind, text),
        None => {
            // Stdout fallback for the no-bus path keeps the legacy
            // bracket-tag so operators watching stdout see the same
            // subsystem labels they would in the .log file.
            match kind.legacy_prefix() {
                Some(p) => println!("{}{}", p, text),
                None => println!("{}", text),
            }
        }
    }
}

fn emit_radio(bus: Option<&SimBus>, text: &str) {
    match bus {
        Some(b) => b.push_radio(text),
        None => println!("{}", text),
    }
}

/// Pull `usage` from the Responses-API payload and push one log line with
/// the per-call input/cached/output counts plus the session-cumulative totals
/// from the backend's `cache_snapshot()`. Silent if the response has no
/// `usage` (older model versions, stubbed test clients).
fn log_usage(response: &Value, client: &dyn ResponsesBackend, bus: Option<&SimBus>) {
    let Some(usage) = response.get("usage") else { return };
    let input = usage.get("input_tokens").and_then(|v| v.as_u64()).unwrap_or(0);
    let cached = usage
        .get("input_tokens_details")
        .and_then(|d| d.get("cached_tokens"))
        .and_then(|v| v.as_u64())
        .unwrap_or(0);
    let output = usage.get("output_tokens").and_then(|v| v.as_u64()).unwrap_or(0);
    let s = client.cache_snapshot();
    emit_log_kind(
        bus,
        LogKind::Tokens,
        &format!(
            "tokens in={} (cached={}) out={} | session in={} out={} calls={}",
            input, cached, output, s.total_input_tokens, s.total_output_tokens, s.total_requests,
        ),
    );
}

fn emit_assistant_text(output_items: &[Value], bus: Option<&SimBus>) {
    for item in output_items {
        if item.get("type").and_then(|v| v.as_str()) == Some("message") {
            if let Some(content) = item.get("content").and_then(|v| v.as_array()) {
                for part in content {
                    if part.get("type").and_then(|v| v.as_str()) == Some("output_text") {
                        if let Some(text) = part.get("text").and_then(|v| v.as_str()) {
                            if !text.trim().is_empty() {
                                emit_log_kind(bus, LogKind::Llm, text);
                            }
                        }
                    }
                }
            }
        }
    }
}
