//! Which LLM flies, and the worker thread that drives it.
//!
//! API providers (OpenAI, Anthropic, Gemini) are stateless
//! request/response backends driven by `conversation::run_conversation_loop`.
//! Claude Code runs its own agent loop, so it gets its own worker
//! (`claude_code::run_claude_code_loop`). Runners build a `PilotBrain` and
//! call `spawn_worker` without caring which it is.

use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::thread::JoinHandle;

use crossbeam_channel::Receiver;

use crate::bus::SimBus;
use crate::llm::anthropic::AnthropicBackend;
use crate::llm::backend::{CacheStats, LlmBackend, LlmProvider, ReasoningEffort};
use crate::llm::claude_code::{run_claude_code_loop, ClaudeCodeConfig};
use crate::llm::conversation::{run_conversation_loop, IncomingMessage, PilotMode};
use crate::llm::gemini::GeminiBackend;
use crate::llm::openai::OpenAiBackend;
use crate::llm::tools::ToolContext;

pub enum PilotBrain {
    Api(Arc<dyn LlmBackend>),
    ClaudeCode(ClaudeCodeConfig),
}

/// Everything the LLM worker thread needs besides the brain.
pub struct WorkerArgs {
    pub tool_context: Arc<ToolContext>,
    pub input_queue: Receiver<IncomingMessage>,
    pub stop: Arc<AtomicBool>,
    /// Per-HTTP-request timeout (API providers only).
    pub per_request_timeout_s: u64,
    /// Wall-clock cap on one incoming message's whole turn.
    pub total_wall_budget_s: u64,
    pub pilot_mode: PilotMode,
    pub reasoning_effort: Option<ReasoningEffort>,
    pub bus: SimBus,
}

impl PilotBrain {
    /// `transcript_path` is where API backends write the raw request /
    /// response transcript; Claude Code keeps its own session log.
    pub fn build(provider: LlmProvider, model: &str, transcript_path: Option<&PathBuf>) -> Self {
        match provider {
            LlmProvider::OpenAi => {
                let mut c = OpenAiBackend::new(model.to_string());
                c.transcript_path = transcript_path.cloned();
                PilotBrain::Api(Arc::new(c))
            }
            LlmProvider::Anthropic => {
                let mut c = AnthropicBackend::new(model.to_string());
                c.transcript_path = transcript_path.cloned();
                PilotBrain::Api(Arc::new(c))
            }
            LlmProvider::Gemini => {
                let mut c = GeminiBackend::new(model.to_string());
                c.transcript_path = transcript_path.cloned();
                PilotBrain::Api(Arc::new(c))
            }
            LlmProvider::ClaudeCode => PilotBrain::ClaudeCode(ClaudeCodeConfig::new(model)),
        }
    }

    /// Token totals for the TUI and eval summaries.
    pub fn cache_stats(&self) -> Option<Arc<CacheStats>> {
        match self {
            PilotBrain::Api(backend) => backend.cache_stats_arc(),
            PilotBrain::ClaudeCode(cfg) => Some(cfg.stats.clone()),
        }
    }

    pub fn spawn_worker(self, args: WorkerArgs) -> std::io::Result<JoinHandle<()>> {
        std::thread::Builder::new().name("llm-worker".into()).spawn(move || match self {
            PilotBrain::Api(backend) => run_conversation_loop(
                &*backend,
                &args.tool_context,
                &args.input_queue,
                args.stop,
                args.per_request_timeout_s,
                args.total_wall_budget_s,
                args.pilot_mode,
                args.reasoning_effort,
                Some(&args.bus),
            ),
            PilotBrain::ClaudeCode(mut cfg) => {
                cfg.effort = args.reasoning_effort;
                run_claude_code_loop(
                    &cfg,
                    args.tool_context,
                    &args.input_queue,
                    args.stop,
                    args.total_wall_budget_s,
                    args.pilot_mode,
                    Some(args.bus),
                )
            }
        })
    }
}
