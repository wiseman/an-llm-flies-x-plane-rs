//! Plans end to end through the eval harness, with a scripted
//! pilot instead of an LLM: scenario YAML -> airborne start -> scripted
//! ATC -> `fly(code)` tool -> Python -> `PlanProfile` -> checks.
//!
//! Needs the apt.dat parquet cache (it reads KVNY's runways); skips with
//! a note when X-Plane's apt.dat isn't installed.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::Result;
use serde_json::{json, Value};
use tempfile::TempDir;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::data::{apt_dat, parquet};
use xplane_pilot::eval_runner::{run_eval_with, EvalConfig};
use xplane_pilot::eval_scenario::Scenario;
use xplane_pilot::llm::backend::{
    CacheSnapshot, CacheStats, LlmBackend, LlmProvider, LlmRequest, LlmResponse, LlmUsage, Part, Role,
};
use xplane_pilot::llm::brain::PilotBrain;
use xplane_pilot::llm::conversation::{IncomingMessage, PilotMode};

/// Replies to each incoming message from a script keyed on what the
/// message says; everything else gets `sleep`.
struct ScriptedPilot {
    /// (substring of the latest user message, tool call to make)
    rules: Vec<(&'static str, &'static str, Value)>,
    used: Mutex<Vec<bool>>,
    stats: Arc<CacheStats>,
}

impl LlmBackend for ScriptedPilot {
    fn create_response(&self, req: &LlmRequest<'_>) -> Result<LlmResponse> {
        std::thread::sleep(Duration::from_millis(1));
        // Everything the user side said since the pilot last spoke.
        let last_user: String = req
            .messages
            .iter()
            .rev()
            .take_while(|m| m.role != Role::Assistant)
            .filter(|m| m.role == Role::User)
            .flat_map(|m| m.content.iter())
            .filter_map(|p| match p {
                Part::Text(t) => Some(t.to_ascii_lowercase()),
                _ => None,
            })
            .collect::<Vec<_>>()
            .join("\n");
        // After a tool result, end the turn.
        let after_tool = req
            .messages
            .last()
            .is_some_and(|m| m.content.iter().any(|p| matches!(p, Part::ToolResult { .. })));
        if std::env::var("STUB_TRACE").is_ok() {
            eprintln!("STUB after_tool={after_tool} last_user={:?}", &last_user[..last_user.len().min(200)]);
        }
        let mut used = self.used.lock().unwrap();
        let call = if after_tool {
            None
        } else {
            self.rules
                .iter()
                .enumerate()
                .find(|(i, (needle, _, _))| !used[*i] && last_user.contains(needle))
                .map(|(i, (_, name, args))| {
                    used[i] = true;
                    (*name, args.clone())
                })
        };
        let (name, args) = call.unwrap_or(("sleep", json!({"suppress_idle_heartbeat_s": null})));
        Ok(LlmResponse {
            output: vec![Part::ToolCall { id: format!("c{}", used.len()), name: name.into(), arguments: args.to_string() }],
            usage: LlmUsage::default(),
            raw: Value::Null,
            thoughts: Vec::new(),
        })
    }

    fn cache_snapshot(&self) -> CacheSnapshot {
        self.stats.snapshot()
    }

    fn cache_stats_arc(&self) -> Option<Arc<CacheStats>> {
        Some(self.stats.clone())
    }
}

fn cache_dir() -> Option<std::path::PathBuf> {
    let apt = apt_dat::default_apt_dat_path()?;
    if !apt.exists() {
        return None;
    }
    parquet::resolve(&apt, None).ok().map(|c| c.dir)
}

#[test]
fn a1_scripted_pilot_extends_and_turns_base_on_the_call() {
    let Some(cache) = cache_dir() else {
        eprintln!("skipping: no X-Plane apt.dat");
        return;
    };
    let scenario: Scenario =
        serde_yaml_ng::from_str(include_str!("../evals/plan/A1-extend-downwind-call-base.yaml")).unwrap();
    let pilot = ScriptedPilot {
        rules: vec![
            ("extend downwind", "fly", json!({
                "code": "p = pattern(apt.runway('16R'), RIGHT)\np.extend('downwind', OPEN)\np.turn('base', when=atc('base'))\njoin(p)",
                "airport": "KVNY",
            })),
            ("turn base now", "fly", json!({"code": "release('base')", "airport": null})),
        ],
        used: Mutex::new(vec![false; 2]),
        stats: Arc::new(CacheStats::default()),
    };
    let tmp = TempDir::new().unwrap();
    let cfg = EvalConfig {
        airport: scenario.airport.clone(),
        parking_spot: String::new(),
        apt_dat_cache_dir: cache,
        llm_provider: LlmProvider::OpenAi,
        llm_model: "scripted".into(),
        llm_reasoning_effort: None,
        startup_messages: vec![IncomingMessage::operator(scenario.briefing.clone().unwrap_or_default())],
        pilot_mode: PilotMode::Normal,
        max_turns: 200,
        max_output_tokens: 0,
        max_sim_seconds: 1200.0,
        dt: 0.2,
        heartbeat_interval_s: 300.0,
        output_dir: tmp.path().to_path_buf(),
        session_stem: "a1".into(),
        verbose: false,
        sim_atc_model: None,
        scenario: Some(scenario),
    };
    let brain = PilotBrain::Api(Arc::new(pilot));
    let result = run_eval_with(load_default_config_bundle(), cfg, brain).unwrap();
    for c in &result.checks {
        eprintln!("{} {} — {}", if c.pass { "PASS" } else { "FAIL" }, c.check, c.detail);
    }
    if result.checks.iter().any(|c| !c.pass) {
        if let Some(log) = &result.log_file {
            let text = std::fs::read_to_string(log).unwrap_or_default();
            for line in text.lines().filter(|l| !l.contains("status={")).take(80) {
                eprintln!("{}", &line[..line.len().min(300)]);
            }
        }
    }
    assert!(!result.checks.is_empty());
    assert!(result.checks.iter().all(|c| c.pass), "checks failed; log in {}", tmp.path().display());
}

/// Every scenario parses, and its message and check references resolve
/// against the real runways and the place lists.
#[test]
fn all_scenarios_parse_and_resolve() {
    use xplane_pilot::eval_scenario::{load, MessageSchedule};
    use xplane_pilot::plan::cond::RCond;
    use xplane_pilot::plan::gazetteer::GAZETTEER;
    use xplane_pilot::plan::resolve::Resolver;
    let Some(cache) = cache_dir() else {
        eprintln!("skipping: no X-Plane apt.dat");
        return;
    };
    let cfg = load_default_config_bundle();
    let pilot = Arc::new(parking_lot::Mutex::new(xplane_pilot::core::mission_manager::PilotCore::new(cfg.clone())));
    let mut ctx = xplane_pilot::llm::tools::ToolContext::new(pilot, cfg);
    ctx.apt_dat_cache_dir = Some(cache);
    let airports = xplane_pilot::llm::tools::DuckAirports { ctx: &ctx };
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("evals/plan");
    let mut n = 0;
    for entry in std::fs::read_dir(dir).unwrap() {
        let path = entry.unwrap().path();
        if path.extension().and_then(|e| e.to_str()) != Some("yaml") {
            continue;
        }
        let s = load(&path).unwrap_or_else(|e| panic!("{}: {e:#}", path.display()));
        let georef = xplane_pilot::sim::xplane_bridge::GeoReference { threshold_lat_deg: 34.2, threshold_lon_deg: -118.45 };
        let mut r = Resolver::new(georef, &GAZETTEER, &airports);
        r.airport(&s.airport).unwrap_or_else(|e| panic!("{}: {e:#}", path.display()));
        if let Some(rw) = s.start.runway() {
            assert!(r.airport(&s.airport).unwrap().runway(rw).is_some(), "{}: no runway {rw}", path.display());
        }
        MessageSchedule::with_order(&s.messages, &mut r, s.sequential).unwrap_or_else(|e| panic!("{}: {e:#}", path.display()));
        let elev = r.airport(&s.airport).unwrap().elevation_ft;
        let traffic = xplane_pilot::eval_scenario::ScriptedTraffic::new(&s.traffic, &s.airport, elev, &mut r)
            .unwrap_or_else(|e| panic!("{} traffic: {e:#}", path.display()));
        assert_eq!(traffic.at(0.0).len(), s.traffic.iter().filter(|t| t.start_s <= 0.0).count(), "{}", path.display());
        // Resolve every condition and reference inside the checks.
        let v: serde_yaml_ng::Value = serde_yaml_ng::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        for c in v["checks"].as_sequence().unwrap() {
            for key in ["holds", "until", "cond"] {
                if let Some(cv) = c.get(key) {
                    let cond: xplane_pilot::plan::ir::Cond = serde_yaml_ng::from_value(cv.clone()).unwrap();
                    RCond::resolve(&cond, &mut r).unwrap_or_else(|e| panic!("{} {key}: {e:#}", path.display()));
                }
            }
            if let Some(near) = c.get("near") {
                let rf: xplane_pilot::plan::ir::Ref = serde_yaml_ng::from_value(near.clone()).unwrap();
                r.resolve(&rf).unwrap_or_else(|e| panic!("{} near: {e:#}", path.display()));
            }
        }
        n += 1;
    }
    assert!(n >= 20, "only {n} scenarios");
}
