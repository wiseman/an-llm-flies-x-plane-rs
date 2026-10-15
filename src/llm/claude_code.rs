//! Pilot the aircraft through a headless Claude Code process.
//!
//! Instead of calling an LLM API ourselves, we run one long-lived
//! `claude -p` child (so it uses the operator's Claude subscription, no
//! API key) and let it run its own agent loop. Our tools reach it through
//! an in-process MCP server (`mcp_server.rs`), so tool calls are
//! dispatched, counted, and logged exactly as in the API loop.
//!
//! Protocol (verified against Claude Code 2.1.300):
//! - stdin: one stream-json line per incoming message,
//!   `{"type":"user","message":{"role":"user","content":"…"}}`. The
//!   process keeps reading stdin, so one child serves the whole session.
//! - stdout: stream-json events. `system`/`init` (carries `session_id`,
//!   re-sent each turn), `assistant` (content blocks: `text`, `thinking`,
//!   `tool_use`), `user` (tool results), `rate_limit_event`, and one
//!   `result` per turn with `usage` (`input_tokens`,
//!   `cache_read_input_tokens`, `cache_creation_input_tokens`,
//!   `output_tokens`), `num_turns`, `total_cost_usd`, `is_error`.
//! - Isolation: `--setting-sources ""` plus our own `--system-prompt`
//!   and an empty working directory keep the user's CLAUDE.md, hooks,
//!   plugins and settings out of the pilot's context; `--tools ""`
//!   removes the built-in tools; `--strict-mcp-config` loads only our
//!   server; `--allowedTools mcp__sim_pilot__* --permission-mode dontAsk`
//!   runs our tools without prompting and denies anything else.
//! - Interrupting a turn (stop flag, wall budget) kills the child; the
//!   next message respawns it with `--resume <session_id>`.

use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context, Result};
use crossbeam_channel::{unbounded, Receiver, RecvTimeoutError};
use serde_json::{json, Value};

use crate::bus::{LogKind, SimBus};
use crate::llm::backend::{CacheStats, LlmUsage, ReasoningEffort};
use crate::llm::anthropic::parse_usage;
use crate::llm::conversation::{
    active_profiles_line, announce_incoming, emit_log_kind, emit_text_lines, for_each_incoming,
    usage_log_line, IncomingMessage, IncomingSource, PilotMode,
};
use crate::llm::mcp_server::{McpServer, SERVER_NAME};
use crate::llm::tools::ToolContext;

/// Added to the pilot system prompt when it runs under Claude Code.
const CLAUDE_CODE_ADDENDUM: &str = "\n\n## Running under Claude Code\n\n\
Your tools come from the MCP server `sim_pilot`, so their names appear as \
`mcp__sim_pilot__<tool>` (for example `mcp__sim_pilot__engage_park`). Every \
tool named in this prompt refers to that tool. After calling `sleep` or \
`mission_complete`, end your turn: no further tool calls or text.\n";

/// Env vars that make Claude Code authenticate with something other than
/// the claude.ai subscription login.
const SUBSCRIPTION_OVERRIDING_ENV: [&str; 3] =
    ["ANTHROPIC_API_KEY", "ANTHROPIC_AUTH_TOKEN", "CLAUDE_CODE_USE_BEDROCK"];

#[derive(Clone)]
pub struct ClaudeCodeConfig {
    /// `--model` value: an alias (`opus`) or a full id (`claude-opus-5-5`).
    pub model: String,
    /// `--effort` value; `PilotBrain::spawn_worker` sets it from the
    /// worker's reasoning effort.
    pub effort: Option<ReasoningEffort>,
    /// The `claude` executable. `CLAUDE_CODE_BIN` overrides (tests use a
    /// fake script).
    pub binary: PathBuf,
    /// Token totals, shared with whoever reports them (TUI, eval summary).
    pub stats: Arc<CacheStats>,
}

impl ClaudeCodeConfig {
    pub fn new(model: impl Into<String>) -> Self {
        Self {
            model: model.into(),
            effort: None,
            binary: std::env::var_os("CLAUDE_CODE_BIN")
                .map(PathBuf::from)
                .unwrap_or_else(|| PathBuf::from("claude")),
            stats: Arc::new(CacheStats::default()),
        }
    }
}

/// Worker-thread entry point; the Claude Code counterpart of
/// `conversation::run_conversation_loop`.
pub fn run_claude_code_loop(
    cfg: &ClaudeCodeConfig,
    tool_context: Arc<ToolContext>,
    input_queue: &Receiver<IncomingMessage>,
    stop: Arc<AtomicBool>,
    total_wall_budget_s: u64,
    initial_mode: PilotMode,
    bus: Option<SimBus>,
) {
    let bus_ref = bus.as_ref();
    emit_log_kind(bus_ref, LogKind::Mode, &format!("pilot mode: {}", initial_mode.label()));
    let mut session = match ClaudeSession::pilot(cfg, tool_context.clone(), initial_mode, bus.clone()) {
        Ok(s) => s,
        Err(e) => {
            emit_log_kind(bus_ref, LogKind::Error, &format!("[claude-code] can't start: {e:#}"));
            return;
        }
    };
    // Mode overlays have no reply of their own; they ride along with the
    // next real message.
    let mut pending_overlay: Option<String> = None;
    for_each_incoming(input_queue, &tool_context, &stop, bus_ref, |message| {
        handle_one(message, &mut session, &mut pending_overlay, &tool_context, total_wall_budget_s, &stop, bus_ref)
    });
    session.kill();
}

/// One incoming message → one Claude Code turn (mode switches just
/// queue their overlay for the next turn).
fn handle_one(
    message: &IncomingMessage,
    session: &mut ClaudeSession,
    pending_overlay: &mut Option<String>,
    tool_context: &ToolContext,
    total_wall_budget_s: u64,
    stop: &AtomicBool,
    bus_ref: Option<&SimBus>,
) {
    announce_incoming(message, bus_ref);
    if let IncomingSource::ModeSwitch(mode) = &message.source {
        *pending_overlay = Some(format!("[MODE_SWITCH] {}", mode.runtime_overlay()));
        return;
    }
    let profiles = tool_context.pilot.lock().list_profile_names().join(", ");
    let mut content = pending_overlay.take().map(|o| o + "\n\n").unwrap_or_default();
    content.push_str(&format!("[{}] {}", message.source.tag(), message.text));
    content.push_str(&format!("\n\n{}", active_profiles_line(&profiles)));
    let deadline = Instant::now() + Duration::from_secs(total_wall_budget_s);
    if let Err(e) = session.run_turn(&content, deadline, stop) {
        emit_log_kind(bus_ref, LogKind::Error, &format!("[claude-code] {e:#}"));
    }
}

/// What one stdout line means to us.
#[derive(Debug, Clone)]
pub enum ClaudeEvent {
    /// `api_key_source` is `"none"` when running on the subscription login.
    Init { session_id: String, api_key_source: Option<String> },
    Text(String),
    Thinking(String),
    Result {
        usage: LlmUsage,
        num_turns: u64,
        cost_usd: Option<f64>,
        is_error: bool,
        error_text: Option<String>,
    },
    Other,
}

/// Parse one stream-json line into the events it carries (an assistant
/// message can hold several content blocks).
pub fn parse_stream_line(line: &str) -> Vec<ClaudeEvent> {
    let Ok(v) = serde_json::from_str::<Value>(line) else {
        return vec![ClaudeEvent::Other];
    };
    match v.get("type").and_then(Value::as_str) {
        Some("system") if v.get("subtype").and_then(Value::as_str) == Some("init") => {
            match v.get("session_id").and_then(Value::as_str) {
                Some(id) => vec![ClaudeEvent::Init {
                    session_id: id.to_string(),
                    api_key_source: v.get("apiKeySource").and_then(Value::as_str).map(str::to_string),
                }],
                None => vec![ClaudeEvent::Other],
            }
        }
        Some("assistant") => v
            .pointer("/message/content")
            .and_then(Value::as_array)
            .map(|blocks| {
                blocks
                    .iter()
                    .filter_map(|b| match b.get("type").and_then(Value::as_str) {
                        Some("text") => b.get("text").and_then(Value::as_str).map(|t| ClaudeEvent::Text(t.to_string())),
                        Some("thinking") => b
                            .get("thinking")
                            .and_then(Value::as_str)
                            .filter(|t| !t.trim().is_empty())
                            .map(|t| ClaudeEvent::Thinking(t.to_string())),
                        _ => None,
                    })
                    .collect()
            })
            .unwrap_or_default(),
        Some("result") => {
            let is_error = v.get("is_error").and_then(Value::as_bool).unwrap_or(false)
                || v.get("subtype").and_then(Value::as_str).is_some_and(|s| s != "success");
            vec![ClaudeEvent::Result {
                // Same `usage` object as the Messages API.
                usage: parse_usage(&v),
                num_turns: v.get("num_turns").and_then(Value::as_u64).unwrap_or(1),
                cost_usd: v.get("total_cost_usd").and_then(Value::as_f64),
                is_error,
                error_text: is_error.then(|| {
                    v.get("result")
                        .and_then(Value::as_str)
                        .map(str::to_string)
                        .unwrap_or_else(|| v.get("subtype").map(|s| s.to_string()).unwrap_or_default())
                }),
            }]
        }
        _ => vec![ClaudeEvent::Other],
    }
}

/// One long-lived `claude -p` stream-json process and its session id.
/// The pilot's carries our tools over MCP and counts its turns and
/// tokens; a plain one (the simulated ATC) has no tools and just talks.
pub(crate) struct ClaudeSession {
    cfg: ClaudeCodeConfig,
    /// Log prefix, e.g. `[claude-code]`.
    label: &'static str,
    /// The pilot's tool context: turn counting and token log lines.
    tool_context: Option<Arc<ToolContext>>,
    bus: Option<SimBus>,
    mcp: Option<McpServer>,
    system_prompt: String,
    work_dir: PathBuf,
    child: Option<ClaudeChild>,
    session_id: Option<String>,
}

struct ClaudeChild {
    process: Child,
    stdin: ChildStdin,
    /// Parsed stdout lines; `None` marks EOF (the process exited).
    events: Receiver<Option<String>>,
}

impl ClaudeSession {
    fn pilot(
        cfg: &ClaudeCodeConfig,
        tool_context: Arc<ToolContext>,
        mode: PilotMode,
        bus: Option<SimBus>,
    ) -> Result<Self> {
        let mcp = McpServer::start(tool_context.clone(), bus.clone())?;
        emit_log_kind(
            bus.as_ref(),
            LogKind::System,
            &format!("[claude-code] tools served at {}", mcp.url),
        );
        let system_prompt = format!("{}{}", mode.system_prompt(), CLAUDE_CODE_ADDENDUM);
        let mut session = Self::plain(cfg, "[claude-code]", system_prompt, bus)?;
        session.tool_context = Some(tool_context);
        session.mcp = Some(mcp);
        Ok(session)
    }

    /// A tool-less session with its own system prompt.
    pub(crate) fn plain(
        cfg: &ClaudeCodeConfig,
        label: &'static str,
        system_prompt: String,
        bus: Option<SimBus>,
    ) -> Result<Self> {
        // An empty working directory: no project CLAUDE.md to discover.
        let work_dir = std::env::temp_dir().join(format!(
            "sim-pilot-claude-{}-{}",
            std::process::id(),
            label.trim_matches(|c| c == '[' || c == ']')
        ));
        std::fs::create_dir_all(&work_dir)
            .with_context(|| format!("creating {}", work_dir.display()))?;
        Ok(Self {
            cfg: cfg.clone(),
            label,
            tool_context: None,
            bus,
            mcp: None,
            system_prompt,
            work_dir,
            child: None,
            session_id: None,
        })
    }

    fn spawn(&mut self) -> Result<()> {
        let mut cmd = Command::new(&self.cfg.binary);
        cmd.args([
            "-p",
            "--input-format",
            "stream-json",
            "--output-format",
            "stream-json",
            "--verbose",
            "--model",
            &self.cfg.model,
            "--system-prompt",
            &self.system_prompt,
            "--tools",
            "",
            "--setting-sources",
            "",
            "--strict-mcp-config",
            "--permission-mode",
            "dontAsk",
        ]);
        if let Some(mcp) = &self.mcp {
            cmd.args([
                "--mcp-config",
                &mcp.claude_config_json(),
                "--allowedTools",
                &format!("mcp__{SERVER_NAME}__*"),
            ]);
        }
        if let Some(effort) = self.cfg.effort {
            cmd.args(["--effort", effort.as_str()]);
        }
        if let Some(id) = &self.session_id {
            cmd.args(["--resume", id]);
        }
        // Use the operator's claude.ai subscription login. An inherited
        // API key (sim-pilot loads `.env`, which often holds one for the
        // `anthropic` provider) would take precedence and bill the API.
        for var in SUBSCRIPTION_OVERRIDING_ENV {
            cmd.env_remove(var);
        }
        cmd.current_dir(&self.work_dir)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        let mut process = cmd
            .spawn()
            .with_context(|| format!("launching {}", self.cfg.binary.display()))?;
        let stdin = process.stdin.take().ok_or_else(|| anyhow!("no stdin"))?;
        let stdout = process.stdout.take().ok_or_else(|| anyhow!("no stdout"))?;
        let stderr = process.stderr.take().ok_or_else(|| anyhow!("no stderr"))?;

        let (tx, events) = unbounded();
        std::thread::Builder::new().name("claude-stdout".into()).spawn(move || {
            for line in BufReader::new(stdout).lines().map_while(Result::ok) {
                if tx.send(Some(line)).is_err() {
                    return;
                }
            }
            let _ = tx.send(None);
        })?;
        let bus = self.bus.clone();
        let label = self.label;
        std::thread::Builder::new().name("claude-stderr".into()).spawn(move || {
            for line in BufReader::new(stderr).lines().map_while(Result::ok) {
                if !line.trim().is_empty() {
                    emit_log_kind(bus.as_ref(), LogKind::Error, &format!("{label} {line}"));
                }
            }
        })?;
        emit_log_kind(
            self.bus.as_ref(),
            LogKind::System,
            &format!(
                "{} started {} --model {}{}",
                self.label,
                self.cfg.binary.display(),
                self.cfg.model,
                self.session_id.as_deref().map(|id| format!(" --resume {id}")).unwrap_or_default()
            ),
        );
        self.child = Some(ClaudeChild { process, stdin, events });
        Ok(())
    }

    pub(crate) fn kill(&mut self) {
        if let Some(mut child) = self.child.take() {
            let _ = child.process.kill();
            let _ = child.process.wait();
        }
    }

    /// Send one user message and stream events until Claude Code finishes
    /// the turn, the wall budget runs out, or `stop` is raised. Returns the
    /// turn's assistant text. The pilot's text and thinking also go to the
    /// log as they arrive.
    pub(crate) fn run_turn(&mut self, content: &str, deadline: Instant, stop: &AtomicBool) -> Result<String> {
        if self.child.is_none() {
            self.spawn()?;
        }
        let line = json!({ "type": "user", "message": { "role": "user", "content": content } });
        {
            let child = self.child.as_mut().expect("spawned");
            if writeln!(child.stdin, "{line}").and_then(|_| child.stdin.flush()).is_err() {
                // The process died between turns; restart and retry once.
                self.kill();
                self.spawn()?;
                let child = self.child.as_mut().expect("spawned");
                writeln!(child.stdin, "{line}")?;
                child.stdin.flush()?;
            }
        }
        let bus = self.bus.clone();
        let is_pilot = self.tool_context.is_some();
        let end_turn_requested = self.mcp.as_ref().map(|m| m.end_turn_requested.clone());
        if let Some(flag) = &end_turn_requested {
            flag.store(false, Ordering::Release);
        }
        let mut interrupted = false;
        let mut text = String::new();
        loop {
            if stop.load(Ordering::Acquire) {
                self.kill();
                return Ok(text);
            }
            // A turn-ending tool ran: end the turn here, as the API loop
            // does. Models keep polling (sleep, get_status, sleep, ...)
            // otherwise, and queued operator/ATC messages wait behind them.
            if !interrupted && end_turn_requested.as_ref().is_some_and(|f| f.load(Ordering::Acquire)) {
                interrupted = true;
                let request = json!({
                    "type": "control_request",
                    "request_id": "end-turn",
                    "request": { "subtype": "interrupt" },
                });
                let child = self.child.as_mut().expect("spawned");
                if writeln!(child.stdin, "{request}").and_then(|_| child.stdin.flush()).is_err() {
                    self.kill();
                    return Ok(text);
                }
            }
            if Instant::now() >= deadline {
                self.kill();
                return Err(anyhow!("wall-clock budget exceeded; interrupted the turn"));
            }
            let events = &self.child.as_ref().expect("spawned").events;
            let line = match events.recv_timeout(Duration::from_millis(200)) {
                Ok(Some(line)) => line,
                Ok(None) | Err(RecvTimeoutError::Disconnected) => {
                    self.kill();
                    return Err(anyhow!("claude exited mid-turn; it will be resumed on the next message"));
                }
                Err(RecvTimeoutError::Timeout) => continue,
            };
            for event in parse_stream_line(&line) {
                match event {
                    ClaudeEvent::Init { session_id, api_key_source } => {
                        if self.session_id.is_none() {
                            emit_log_kind(
                                bus.as_ref(),
                                LogKind::System,
                                &format!(
                                    "{} session {session_id} auth={}",
                                    self.label,
                                    api_key_source.as_deref().unwrap_or("?")
                                ),
                            );
                        }
                        self.session_id = Some(session_id);
                    }
                    ClaudeEvent::Text(t) => {
                        if is_pilot {
                            emit_text_lines(bus.as_ref(), LogKind::Llm, t.lines());
                        }
                        if !text.is_empty() {
                            text.push('\n');
                        }
                        text.push_str(&t);
                    }
                    ClaudeEvent::Thinking(t) => {
                        if is_pilot {
                            emit_text_lines(bus.as_ref(), LogKind::Thinking, t.lines());
                        }
                    }
                    ClaudeEvent::Other => {}
                    ClaudeEvent::Result { usage, num_turns, cost_usd, is_error, error_text } => {
                        self.record_usage(&usage, num_turns, cost_usd);
                        // Our own interrupt ends the turn as an error.
                        if is_error && !interrupted {
                            return Err(anyhow!("turn ended with an error: {}", error_text.unwrap_or_default()));
                        }
                        return Ok(text);
                    }
                }
            }
        }
    }

    fn record_usage(&self, usage: &LlmUsage, num_turns: u64, cost_usd: Option<f64>) {
        self.cfg.stats.record(usage);
        // Only the pilot's turns and tokens count toward eval budgets and
        // the `tokens` log line the transcript tool parses.
        let Some(tool_context) = &self.tool_context else { return };
        if let Some(counter) = tool_context.turn_counter.as_ref() {
            counter.fetch_add(num_turns.max(1) as usize, Ordering::Relaxed);
        }
        let extra = format!(
            " turns={num_turns}{}",
            cost_usd.map(|c| format!(" equiv_cost=${c:.4}")).unwrap_or_default()
        );
        emit_log_kind(
            self.bus.as_ref(),
            LogKind::Tokens,
            &usage_log_line(usage, &extra, &self.cfg.stats.snapshot()),
        );
    }
}

impl Drop for ClaudeSession {
    fn drop(&mut self) {
        self.kill();
        let _ = std::fs::remove_dir(&self.work_dir);
    }
}
