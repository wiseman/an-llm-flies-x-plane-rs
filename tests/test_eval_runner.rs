//! Eval-harness plumbing tests. Uses a scripted `LlmBackend` stub
//! (no real API calls) and an inline `ResolvedStart` (no apt.dat cache
//! needed) to exercise the control-loop, outcome channel, and
//! mission_complete tool wiring end-to-end without burning wall time
//! or tokens.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::Result;
use serde_json::{json, Value};
use tempfile::TempDir;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::eval_runner::{
    run_eval_core, EvalConfig, OutcomeSource, ResolvedStart,
};
use xplane_pilot::llm::backend::{
    CacheSnapshot, CacheStats, LlmBackend, LlmRequest, LlmResponse, LlmUsage, Part,
};
use xplane_pilot::llm::conversation::{IncomingMessage, PilotMode};
use xplane_pilot::llm::backend::LlmProvider;

/// Scripted backend — returns pre-built responses in order. Each entry
/// becomes one `create_response` turn. Usage counts on the scripted
/// response flow into the shared `CacheStats`, so tests can drive the
/// token-budget path.
struct StubClient {
    scripted: Mutex<Vec<LlmResponse>>,
    call_count: Mutex<usize>,
    default: LlmResponse,
    stats: Arc<CacheStats>,
}

impl StubClient {
    fn new(scripted: Vec<LlmResponse>) -> Self {
        Self {
            scripted: Mutex::new(scripted),
            call_count: Mutex::new(0),
            default: empty_response(),
            stats: Arc::new(CacheStats::default()),
        }
    }

    fn calls(&self) -> usize {
        *self.call_count.lock().unwrap()
    }
}

impl LlmBackend for StubClient {
    fn create_response(&self, _req: &LlmRequest<'_>) -> Result<LlmResponse> {
        // Tiny pause to mimic network latency — without it, fast
        // scripted bursts run on a single thread slice and the control
        // loop never gets a chance to enforce its budgets between
        // turns. Production LLM calls take seconds, so a 1 ms sleep
        // here is still ~1000x faster than reality.
        std::thread::sleep(Duration::from_millis(1));
        *self.call_count.lock().unwrap() += 1;
        let mut s = self.scripted.lock().unwrap();
        let resp = if s.is_empty() {
            self.default.clone()
        } else {
            s.remove(0)
        };
        self.stats.record(&resp.usage);
        Ok(resp)
    }

    fn cache_snapshot(&self) -> CacheSnapshot {
        self.stats.snapshot()
    }

    fn cache_stats_arc(&self) -> Option<Arc<CacheStats>> {
        Some(self.stats.clone())
    }
}

fn empty_response() -> LlmResponse {
    LlmResponse {
        output: Vec::new(),
        usage: LlmUsage::default(),
        raw: Value::Null,
    }
}

fn tool_call(name: &str, args: Value, id: &str) -> LlmResponse {
    LlmResponse {
        output: vec![Part::ToolCall {
            id: id.to_string(),
            name: name.to_string(),
            arguments: args.to_string(),
        }],
        usage: LlmUsage::default(),
        raw: Value::Null,
    }
}

fn tool_call_with_usage(name: &str, args: Value, id: &str, out_tokens: u64) -> LlmResponse {
    LlmResponse {
        output: vec![Part::ToolCall {
            id: id.to_string(),
            name: name.to_string(),
            arguments: args.to_string(),
        }],
        usage: LlmUsage {
            input_tokens: 100,
            cached_tokens: 0,
            cache_creation_tokens: 0,
            output_tokens: out_tokens,
        },
        raw: Value::Null,
    }
}

fn kwhp_resolved() -> ResolvedStart {
    // Real KWHP values are close enough for the test — the exact numbers
    // don't matter, we just need a self-consistent geometry. Runway 12
    // threshold / heading pulled from apt.dat.
    ResolvedStart {
        runway_ident: "12".to_string(),
        runway_threshold_lat_deg: 34.2583,
        runway_threshold_lon_deg: -118.4129,
        runway_course_deg: 121.0,
        runway_length_ft: 4120.0,
        runway_displaced_threshold_ft: 0.0,
        field_elevation_ft: 1003.0,
        // Parking spot ~400 ft north of the threshold, facing runway-ish.
        parking_lat_deg: 34.2594,
        parking_lon_deg: -118.4135,
        parking_heading_deg: 180.0,
    }
}

fn make_eval_config(tmp: &TempDir, stem: &str) -> EvalConfig {
    EvalConfig {
        airport: "KWHP".to_string(),
        parking_spot: "Test Ramp".to_string(),
        apt_dat_cache_dir: tmp.path().to_path_buf(),
        llm_provider: LlmProvider::OpenAi,
        llm_model: "stub-model".to_string(),
        llm_reasoning_effort: None,
        startup_messages: vec![IncomingMessage::operator("Eval test startup.")],
        pilot_mode: PilotMode::Normal,
        max_turns: 40,
        max_output_tokens: 0,
        max_sim_seconds: 120.0,
        dt: 0.2,
        heartbeat_interval_s: 30.0,
        output_dir: tmp.path().to_path_buf(),
        session_stem: stem.to_string(),
        verbose: false,
    }
}

#[test]
fn mission_complete_tool_drives_outcome_through_channel() {
    let tmp = TempDir::new().unwrap();
    let backend = Arc::new(StubClient::new(vec![tool_call(
        "mission_complete",
        json!({"success": true, "summary": "test ok"}),
        "call_mc_1",
    )]));
    let cfg = make_eval_config(&tmp, "eval-stub-success");

    let start = std::time::Instant::now();
    let result = run_eval_core(
        load_default_config_bundle(),
        cfg,
        kwhp_resolved(),
        backend.clone() as Arc<dyn LlmBackend>,
    )
    .expect("run_eval_core should succeed");
    let wall = start.elapsed();

    assert_eq!(
        result.outcome.source,
        OutcomeSource::LlmReport,
        "outcome should come from mission_complete tool"
    );
    assert!(result.outcome.success, "stub reported success");
    assert_eq!(result.outcome.summary, "test ok");
    assert_eq!(
        result.turn_count, 1,
        "stub scripted one create_response call"
    );
    assert_eq!(
        backend.calls(),
        1,
        "backend.create_response called exactly once"
    );
    assert!(
        wall < Duration::from_secs(5),
        "test should complete in well under 5s; took {:?}",
        wall
    );
    assert!(
        result.summary_json.as_ref().is_some_and(|p| p.exists()),
        "summary JSON should be on disk"
    );
    assert!(!result.crashed, "no crash in happy path");
}

#[test]
fn turn_budget_triggers_timeout_outcome() {
    let tmp = TempDir::new().unwrap();
    // Stub always returns empty — but the conversation loop exits when
    // there are no tool calls, so we need the stub to keep emitting
    // tool calls that don't end the eval. A sequence of `get_status`
    // calls does the trick: each call round-trips a tool result and
    // prompts another model turn.
    let mut script = Vec::new();
    for i in 0..20 {
        script.push(tool_call(
            "get_status",
            json!({}),
            &format!("call_{}", i),
        ));
    }
    let backend = Arc::new(StubClient::new(script));
    let mut cfg = make_eval_config(&tmp, "eval-stub-turn-budget");
    cfg.max_turns = 3;
    cfg.max_sim_seconds = 10_000.0; // ensure sim timeout doesn't win

    let result = run_eval_core(
        load_default_config_bundle(),
        cfg,
        kwhp_resolved(),
        backend as Arc<dyn LlmBackend>,
    )
    .expect("run_eval_core should succeed");

    assert_eq!(
        result.outcome.source,
        OutcomeSource::TimeoutTurns,
        "expected turn budget to fire first, got outcome={:?} summary={}",
        result.outcome.source, result.outcome.summary
    );
    assert!(!result.outcome.success);
    assert!(
        result.turn_count >= 3,
        "turn_count should be at least the budget; got {}",
        result.turn_count
    );
}

#[test]
fn output_token_budget_triggers_timeout() {
    let tmp = TempDir::new().unwrap();
    // 10 get_status calls each charging 2_000 output tokens → 20k
    // cumulative. With max_output_tokens=5_000 the budget fires
    // after roughly the third turn.
    let mut script = Vec::new();
    for i in 0..10 {
        script.push(tool_call_with_usage(
            "get_status",
            json!({}),
            &format!("call_{}", i),
            2_000,
        ));
    }
    let backend = Arc::new(StubClient::new(script));
    let mut cfg = make_eval_config(&tmp, "eval-stub-token-budget");
    cfg.max_turns = 100;
    cfg.max_sim_seconds = 10_000.0;
    cfg.max_output_tokens = 5_000;

    let result = run_eval_core(
        load_default_config_bundle(),
        cfg,
        kwhp_resolved(),
        backend.clone() as Arc<dyn LlmBackend>,
    )
    .expect("run_eval_core should succeed");

    assert_eq!(
        result.outcome.source,
        OutcomeSource::TimeoutTokens,
        "expected token budget timeout, got {:?}: {}",
        result.outcome.source, result.outcome.summary
    );
    assert!(
        result.total_output_tokens >= 5_000,
        "output tokens should exceed the budget; got {}",
        result.total_output_tokens
    );
    assert!(
        result.turn_count < 10,
        "should fire before all 10 scripted turns run; got {}",
        result.turn_count
    );
}

#[test]
fn sim_budget_triggers_timeout_when_llm_idle() {
    let tmp = TempDir::new().unwrap();
    // Stub returns a single sleep(600) tool call that suppresses
    // heartbeats for 10 minutes of sim time. With max_sim_seconds=5
    // the sim-time budget fires long before heartbeats resume.
    let backend = Arc::new(StubClient::new(vec![tool_call(
        "sleep",
        json!({"suppress_idle_heartbeat_s": 600}),
        "call_sleep_1",
    )]));
    let mut cfg = make_eval_config(&tmp, "eval-stub-sim-budget");
    cfg.max_sim_seconds = 5.0;
    cfg.max_turns = 100;

    let result = run_eval_core(
        load_default_config_bundle(),
        cfg,
        kwhp_resolved(),
        backend as Arc<dyn LlmBackend>,
    )
    .expect("run_eval_core should succeed");

    assert_eq!(
        result.outcome.source,
        OutcomeSource::TimeoutSim,
        "expected sim-time timeout, got {:?}: {}",
        result.outcome.source, result.outcome.summary
    );
    assert!(result.sim_duration_s >= 5.0);
}

#[test]
fn tool_failures_are_counted_in_eval_result() {
    // Script a mix: one bad call (unknown tool) + one good call
    // (get_status) + mission_complete. Counters should record 3 total
    // calls, 1 failure, with a breakdown per tool name.
    let tmp = TempDir::new().unwrap();
    let script = vec![
        tool_call("not_a_real_tool", json!({}), "call_bad_1"),
        tool_call("get_status", json!({}), "call_good_1"),
        tool_call(
            "mission_complete",
            json!({"success": true, "summary": "done"}),
            "call_mc_1",
        ),
    ];
    let backend = Arc::new(StubClient::new(script));
    let cfg = make_eval_config(&tmp, "eval-stub-tool-failures");

    let result = run_eval_core(
        load_default_config_bundle(),
        cfg,
        kwhp_resolved(),
        backend as Arc<dyn LlmBackend>,
    )
    .expect("run_eval_core should succeed");

    assert_eq!(result.tool_stats.total, 3, "three tools called");
    assert_eq!(result.tool_stats.failed, 1, "one tool failed");
    let bad = result
        .tool_stats
        .per_tool
        .iter()
        .find(|(n, _)| n == "not_a_real_tool")
        .expect("bad tool recorded");
    assert_eq!(bad.1.calls, 1);
    assert_eq!(bad.1.failures, 1);
    let good = result
        .tool_stats
        .per_tool
        .iter()
        .find(|(n, _)| n == "get_status")
        .expect("good tool recorded");
    assert_eq!(good.1.calls, 1);
    assert_eq!(good.1.failures, 0);
}
