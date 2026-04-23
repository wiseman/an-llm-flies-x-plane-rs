//! Multi-turn LLM conversation loop.
//!
//! `Conversation` holds the pinned system prompt + rotating message history
//! in provider-neutral `Message` form. History grows unbounded — there is no
//! compaction.  `run_conversation_loop` pulls one `IncomingMessage` off the
//! queue, appends it, then drives the backend in a loop: each tool call is
//! dispatched and its output appended as a tool_result message. The loop
//! terminates on (a) no tool calls in the response, (b) a `sleep` tool call,
//! or (c) the wall-clock budget running out.

use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::{Duration, Instant};

use crossbeam_channel::Receiver;

use crate::bus::{LogKind, SimBus};
use crate::llm::backend::{
    LlmBackend, LlmRequest, LlmResponse, Message, Part, ReasoningEffort, Role,
};
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

pub struct Conversation {
    pub pinned: Vec<Message>,
    pub rotating: Vec<Message>,
}

impl Conversation {
    pub fn new(system_prompt: impl Into<String>) -> Self {
        Self {
            pinned: vec![Message::system(system_prompt.into())],
            rotating: Vec::new(),
        }
    }

    /// The current system prompt as a string slice. Derived from
    /// `pinned[0]`; returns `""` if the head slot has been cleared.
    pub fn system_prompt(&self) -> &str {
        self.pinned_text(0).unwrap_or("")
    }

    /// Replace the pinned system prompt in place. History (`rotating`) is
    /// preserved so the model stays in the same turn; only the leading
    /// system message is rewritten.
    pub fn set_system_prompt(&mut self, prompt: impl Into<String>) {
        let msg = Message::system(prompt.into());
        if self.pinned.is_empty() {
            self.pinned.push(msg);
        } else {
            self.pinned[0] = msg;
        }
    }

    pub fn append_operator_message(&mut self, text: &str) {
        self.rotating
            .push(Message::user_text(format!("[OPERATOR] {}", text)));
    }
    pub fn append_atc_message(&mut self, text: &str) {
        self.rotating
            .push(Message::user_text(format!("[ATC] {}", text)));
    }
    pub fn append_heartbeat_message(&mut self, text: &str) {
        self.rotating
            .push(Message::user_text(format!("[HEARTBEAT] {}", text)));
    }
    pub fn append_mode_switch_message(&mut self, text: &str) {
        self.rotating
            .push(Message::user_text(format!("[MODE_SWITCH] {}", text)));
    }

    /// Record the assistant output from one response. Text parts become
    /// one assistant text message per contiguous group; tool_call parts
    /// each become a standalone assistant message carrying just that call
    /// so subsequent tool_result messages pair one-to-one.
    pub fn append_response(&mut self, output: &[Part]) {
        let mut text_buf: Vec<String> = Vec::new();
        let flush_text = |buf: &mut Vec<String>, rotating: &mut Vec<Message>| {
            if !buf.is_empty() {
                let joined = buf.join("\n");
                buf.clear();
                rotating.push(Message::assistant_text(joined));
            }
        };
        for part in output {
            match part {
                Part::Text(t) => {
                    if !t.is_empty() {
                        text_buf.push(t.clone());
                    }
                }
                Part::ToolCall { id, name, arguments } => {
                    flush_text(&mut text_buf, &mut self.rotating);
                    self.rotating.push(Message::assistant_tool_call(
                        id.clone(),
                        name.clone(),
                        arguments.clone(),
                    ));
                }
                Part::ToolResult { .. } => {
                    // Assistants don't emit tool_results; ignore if the
                    // backend ever leaks one into the output stream.
                }
            }
        }
        flush_text(&mut text_buf, &mut self.rotating);
    }

    pub fn append_tool_result(&mut self, call_id: &str, output: &str) -> anyhow::Result<()> {
        if call_id.is_empty() {
            return Err(anyhow::anyhow!("tool_result requires a call_id"));
        }
        self.rotating.push(Message::tool_result(call_id, output));
        Ok(())
    }

    /// Build the full input for one turn: pinned + rotating + a trailing
    /// system message summarizing the currently-active guidance
    /// profiles. The trailing summary changes every turn by design — it
    /// is explicitly *not* part of the cacheable prefix.
    pub fn build_input(&self, active_profiles_summary: &str) -> Vec<Message> {
        let summary_text = format!(
            "Active profiles: {}",
            if active_profiles_summary.is_empty() { "(none)" } else { active_profiles_summary }
        );
        let mut out = Vec::with_capacity(self.pinned.len() + self.rotating.len() + 1);
        out.extend(self.pinned.iter().cloned());
        out.extend(self.rotating.iter().cloned());
        out.push(Message::system(summary_text));
        out
    }

    /// Helper used by tests to read the tag on the N-th rotating
    /// message (e.g. `[OPERATOR]`). Returns `None` if the slot isn't a
    /// user text message.
    pub fn rotating_text(&self, idx: usize) -> Option<&str> {
        let msg = self.rotating.get(idx)?;
        if msg.role != Role::User {
            return None;
        }
        match msg.content.first()? {
            Part::Text(t) => Some(t.as_str()),
            _ => None,
        }
    }

    pub fn pinned_text(&self, idx: usize) -> Option<&str> {
        let msg = self.pinned.get(idx)?;
        match msg.content.first()? {
            Part::Text(t) => Some(t.as_str()),
            _ => None,
        }
    }
}

pub fn run_conversation_loop(
    client: &dyn LlmBackend,
    tool_context: &ToolContext,
    input_queue: &Receiver<IncomingMessage>,
    stop: Arc<AtomicBool>,
    per_request_timeout_s: u64,
    total_wall_budget_s: u64,
    initial_mode: PilotMode,
    reasoning_effort: Option<ReasoningEffort>,
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
            reasoning_effort,
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
    client: &dyn LlmBackend,
    tool_context: &ToolContext,
    message: &IncomingMessage,
    per_request_timeout_s: u64,
    total_wall_budget_s: u64,
    reasoning_effort: Option<ReasoningEffort>,
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
    let tools = tool_schemas();
    loop {
        if Instant::now() >= deadline {
            emit_log_kind(bus, LogKind::Error, "wall-clock budget exceeded; ending turn");
            return Ok(());
        }
        let profiles = tool_context.pilot.lock().list_profile_names().join(", ");
        let messages = conv.build_input(&profiles);
        let remaining = deadline.saturating_duration_since(Instant::now()).as_secs();
        let timeout = per_request_timeout_s.min(remaining.max(1));
        let response = {
            let _busy = LlmBusyGuard::new(bus);
            client.create_response(&LlmRequest {
                messages: &messages,
                tools: &tools,
                reasoning_effort,
                timeout_secs: timeout,
            })?
        };
        log_usage(&response, client, bus);
        emit_assistant_text(&response.output, bus);
        conv.append_response(&response.output);

        let tool_calls: Vec<(String, String, String)> = response
            .output
            .iter()
            .filter_map(|p| match p {
                Part::ToolCall { id, name, arguments } => {
                    Some((id.clone(), name.clone(), arguments.clone()))
                }
                _ => None,
            })
            .collect();

        if tool_calls.is_empty() {
            return Ok(());
        }

        let mut slept = false;
        for (call_id, name, arguments) in tool_calls {
            if call_id.is_empty() {
                emit_log_kind(
                    bus,
                    LogKind::Error,
                    &format!("tool_call missing call_id: name={}", name),
                );
                continue;
            }
            let result = dispatch_tool(&name, &arguments, tool_context);
            if name == "sleep" {
                slept = true;
            }
            emit_log_kind(
                bus,
                LogKind::ToolCall,
                &format!("tool {} -> {}", name, result),
            );
            conv.append_tool_result(&call_id, &result)?;
        }
        if slept {
            return Ok(());
        }
    }
}

/// Flips the bus `llm_busy` flag on for as long as this guard lives and
/// clears it on drop — including when `?` unwinds through the HTTP
/// call. The TUI polls the flag every frame to animate the waiting
/// spinner.
struct LlmBusyGuard<'a> {
    bus: Option<&'a SimBus>,
}

impl<'a> LlmBusyGuard<'a> {
    fn new(bus: Option<&'a SimBus>) -> Self {
        if let Some(b) = bus {
            b.set_llm_busy(true);
        }
        Self { bus }
    }
}

impl Drop for LlmBusyGuard<'_> {
    fn drop(&mut self) {
        if let Some(b) = self.bus {
            b.set_llm_busy(false);
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

/// Push one log line with the per-call input/cached/output counts plus
/// session-cumulative totals from the backend's `cache_snapshot()`.
/// Silent when the response carries no usage (older model versions,
/// stubbed test clients).
fn log_usage(response: &LlmResponse, client: &dyn LlmBackend, bus: Option<&SimBus>) {
    let u = &response.usage;
    // Suppress the line when every counter is zero — test stubs that
    // return empty `LlmUsage::default()` shouldn't clutter the bus.
    if u.input_tokens == 0 && u.output_tokens == 0 && u.cached_tokens == 0 {
        return;
    }
    let s = client.cache_snapshot();
    emit_log_kind(
        bus,
        LogKind::Tokens,
        &format!(
            "tokens in={} (cached={}) out={} | session in={} out={} calls={}",
            u.input_tokens,
            u.cached_tokens,
            u.output_tokens,
            s.total_input_tokens,
            s.total_output_tokens,
            s.total_requests,
        ),
    );
}

fn emit_assistant_text(output: &[Part], bus: Option<&SimBus>) {
    for part in output {
        if let Part::Text(t) = part {
            if !t.trim().is_empty() {
                emit_log_kind(bus, LogKind::Llm, t);
            }
        }
    }
}
