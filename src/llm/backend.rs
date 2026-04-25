//! Provider-neutral types and trait for talking to an LLM.
//!
//! Everything upstream of the backend (`conversation.rs`, `tools.rs`,
//! `live_runner.rs`, `main.rs`) speaks these types. Each provider module
//! (`openai.rs`, `anthropic.rs`, `gemini.rs`) translates them to its own
//! wire format at the trait boundary.

use std::fs;
use std::path::Path;
use std::str::FromStr;
use std::sync::{Arc, Mutex};

use anyhow::{anyhow, Result};
use clap::ValueEnum;
use serde_json::Value;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    System,
    User,
    Assistant,
}

/// One content block inside a `Message`. `arguments` in `ToolCall` and
/// `content` in `ToolResult` are JSON-encoded strings — the same shape
/// the LLM itself emits, so we don't eagerly parse-and-re-emit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Part {
    Text(String),
    ToolCall {
        id: String,
        name: String,
        arguments: String,
    },
    ToolResult {
        id: String,
        content: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Message {
    pub role: Role,
    pub content: Vec<Part>,
}

impl Message {
    pub fn system(text: impl Into<String>) -> Self {
        Self {
            role: Role::System,
            content: vec![Part::Text(text.into())],
        }
    }

    pub fn user_text(text: impl Into<String>) -> Self {
        Self {
            role: Role::User,
            content: vec![Part::Text(text.into())],
        }
    }

    pub fn tool_result(id: impl Into<String>, content: impl Into<String>) -> Self {
        Self {
            role: Role::User,
            content: vec![Part::ToolResult {
                id: id.into(),
                content: content.into(),
            }],
        }
    }

    pub fn assistant_text(text: impl Into<String>) -> Self {
        Self {
            role: Role::Assistant,
            content: vec![Part::Text(text.into())],
        }
    }

    pub fn assistant_tool_call(
        id: impl Into<String>,
        name: impl Into<String>,
        arguments: impl Into<String>,
    ) -> Self {
        Self {
            role: Role::Assistant,
            content: vec![Part::ToolCall {
                id: id.into(),
                name: name.into(),
                arguments: arguments.into(),
            }],
        }
    }
}

/// Tool declaration in a form every provider can translate. `parameters`
/// is a JSON Schema `object` (the same blob OpenAI's `parameters`,
/// Anthropic's `input_schema`, and Gemini's `parameters` all expect).
#[derive(Debug, Clone)]
pub struct ToolDef {
    pub name: String,
    pub description: String,
    pub parameters: Value,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReasoningEffort {
    Low,
    Medium,
    High,
}

impl ReasoningEffort {
    pub fn as_str(self) -> &'static str {
        match self {
            ReasoningEffort::Low => "low",
            ReasoningEffort::Medium => "medium",
            ReasoningEffort::High => "high",
        }
    }

    /// Rough token budget for providers whose thinking knob is a
    /// budget-in-tokens rather than a low/medium/high string (Anthropic
    /// `thinking.budget_tokens`, Gemini `thinkingConfig.thinkingBudget`).
    pub fn token_budget(self) -> u32 {
        match self {
            ReasoningEffort::Low => 2048,
            ReasoningEffort::Medium => 8192,
            ReasoningEffort::High => 16384,
        }
    }
}

impl FromStr for ReasoningEffort {
    type Err = String;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s.trim().to_ascii_lowercase().as_str() {
            "low" => Ok(ReasoningEffort::Low),
            "medium" | "med" => Ok(ReasoningEffort::Medium),
            "high" => Ok(ReasoningEffort::High),
            other => Err(format!("unknown reasoning effort {:?}", other)),
        }
    }
}

pub struct LlmRequest<'a> {
    pub messages: &'a [Message],
    pub tools: &'a [ToolDef],
    pub reasoning_effort: Option<ReasoningEffort>,
    pub timeout_secs: u64,
    /// Optional index into `messages` of the last message that is
    /// byte-stable across turns. Providers that support explicit prefix
    /// caching (Anthropic) use this to place a cache breakpoint; providers
    /// with implicit/automatic caching (OpenAI, Gemini 2.5) ignore it.
    /// `None` = no messages-side cache hint.
    pub cache_anchor_idx: Option<usize>,
}

#[derive(Debug, Clone, Default)]
pub struct LlmUsage {
    pub input_tokens: u64,
    pub cached_tokens: u64,
    /// Tokens written *into* the prefix cache this turn. Anthropic only —
    /// they bill cache writes at 1.25× base, so it has to be tracked
    /// separately from `input_tokens` (which already includes it) for
    /// honest cost math. OpenAI/Gemini have no analogous tier and
    /// always report 0.
    pub cache_creation_tokens: u64,
    pub output_tokens: u64,
}

#[derive(Debug, Clone)]
pub struct LlmResponse {
    pub output: Vec<Part>,
    pub usage: LlmUsage,
    /// Provider-native blob, preserved for the transcript dumper. Never
    /// parsed by upstream code.
    pub raw: Value,
}

pub trait LlmBackend: Send + Sync {
    fn create_response(&self, req: &LlmRequest<'_>) -> Result<LlmResponse>;

    /// Cumulative per-session token totals. Default zero for test stubs.
    fn cache_snapshot(&self) -> CacheSnapshot {
        CacheSnapshot::default()
    }

    /// Shared stats handle for the TUI's live cache-hit display. Default
    /// `None` for test stubs that don't track stats.
    fn cache_stats_arc(&self) -> Option<Arc<CacheStats>> {
        None
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum LlmProvider {
    #[value(name = "openai", aliases = ["oai"])]
    OpenAi,
    #[value(name = "anthropic", aliases = ["claude"])]
    Anthropic,
    #[value(name = "gemini", aliases = ["google"])]
    Gemini,
}

impl LlmProvider {
    pub fn as_str(self) -> &'static str {
        match self {
            LlmProvider::OpenAi => "openai",
            LlmProvider::Anthropic => "anthropic",
            LlmProvider::Gemini => "gemini",
        }
    }

    /// Default model when `--pilot-llm-model` is omitted. Caller may
    /// still override with an explicit flag.
    pub fn default_model(self) -> &'static str {
        match self {
            LlmProvider::OpenAi => "gpt-5.4-2026-03-05",
            LlmProvider::Anthropic => "claude-sonnet-4-6",
            LlmProvider::Gemini => "gemini-2.5-flash",
        }
    }
}

impl FromStr for LlmProvider {
    type Err = String;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s.trim().to_ascii_lowercase().as_str() {
            "openai" | "oai" => Ok(LlmProvider::OpenAi),
            "anthropic" | "claude" => Ok(LlmProvider::Anthropic),
            "gemini" | "google" => Ok(LlmProvider::Gemini),
            other => Err(format!("unknown llm provider {:?}", other)),
        }
    }
}

// ---------- shared cache-stats plumbing ----------

#[derive(Debug, Clone, Copy, Default)]
pub struct CacheSnapshot {
    pub total_input_tokens: u64,
    pub total_cached_tokens: u64,
    /// Cumulative cache-write tokens. Anthropic-only; OpenAI/Gemini stay 0.
    pub total_cache_creation_tokens: u64,
    pub total_output_tokens: u64,
    pub total_requests: u64,
}

/// Thread-safe cumulative token counter shared across every call on one
/// backend. Each provider holds an `Arc<CacheStats>` and surfaces it via
/// `cache_snapshot()` for the conversation loop's token log line and via
/// `cache_stats_arc()` for the TUI's live cache-hit panel.
#[derive(Default)]
pub struct CacheStats {
    inner: Mutex<CacheSnapshot>,
}

impl CacheStats {
    pub fn record(&self, usage: &LlmUsage) {
        let mut g = self.inner.lock().unwrap();
        g.total_input_tokens += usage.input_tokens;
        g.total_cached_tokens += usage.cached_tokens;
        g.total_cache_creation_tokens += usage.cache_creation_tokens;
        g.total_output_tokens += usage.output_tokens;
        g.total_requests += 1;
    }

    pub fn snapshot(&self) -> CacheSnapshot {
        *self.inner.lock().unwrap()
    }

    pub fn hit_rate(&self) -> Option<f64> {
        let s = self.snapshot();
        if s.total_input_tokens == 0 {
            None
        } else {
            Some(s.total_cached_tokens as f64 / s.total_input_tokens as f64)
        }
    }
}

/// Run `req` and return the successful `Response`. On a non-2xx status,
/// read the response body and bake it into the error so schema / auth /
/// validation messages from the API surface in the log instead of being
/// reduced to a bare "status code 4xx".
pub fn send_with_body_on_error(
    req: ureq::Request,
    payload: &str,
    url: &str,
) -> Result<ureq::Response> {
    match req.send_string(payload) {
        Ok(r) => Ok(r),
        Err(ureq::Error::Status(code, response)) => {
            let body = response.into_string().unwrap_or_default();
            Err(anyhow!("POST {} -> HTTP {}: {}", url, code, body))
        }
        Err(e) => Err(anyhow::Error::new(e).context(format!("posting to {}", url))),
    }
}

/// Best-effort transcript dump of one API round. Writes the request
/// pretty-printed; the response slot is filled with a `(pending)`
/// placeholder when called before the HTTP call completes, and with the
/// full payload once it returns. Errors are swallowed — a broken
/// transcript dump shouldn't take down the pilot.
pub fn write_transcript(path: &Path, request: &Value, response: Option<&Value>) {
    if let Some(parent) = path.parent() {
        let _ = fs::create_dir_all(parent);
    }
    let request_str = serde_json::to_string_pretty(request)
        .unwrap_or_else(|e| format!("(request serialize failed: {e})"));
    let response_str = match response {
        Some(r) => serde_json::to_string_pretty(r)
            .unwrap_or_else(|e| format!("(response serialize failed: {e})")),
        None => "(pending — HTTP request in flight)".to_string(),
    };
    let _ = fs::write(
        path,
        format!(
            "=== REQUEST ===\n{request_str}\n\n=== RESPONSE ===\n{response_str}\n"
        ),
    );
}
