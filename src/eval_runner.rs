//! End-to-end eval harness for the LLM pilot.
//!
//! Wires the real `run_conversation_loop`, `HeartbeatPump`, `SimBus`, and
//! tool dispatch on top of the deterministic `SimpleAircraftModel` so we
//! can measure LLM performance (turn count, tool calls, success/failure,
//! crash/timeout) across prompt or tool changes without burning X-Plane
//! real-time. Sim time drives the heartbeat clock, and sim advancement
//! pauses while the LLM worker is busy — so `sleep(N)` means exactly N
//! sim seconds regardless of model latency.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context, Result};
use crossbeam_channel::{unbounded, Receiver, Sender};
use parking_lot::Mutex as PLMutex;

use crate::bus::{FileLog, LogKind, SimBus};
use crate::config::{AirportConfig, ConfigBundle};
use crate::core::mission_manager::PilotCore;
use crate::data::parquet::{AIRPORTS_FILE, COMMS_FILE, PARKING_SPOTS_FILE, RUNWAYS_FILE};
use crate::live_runner::{Clock, HeartbeatPump, SimClock};
use crate::llm::anthropic::AnthropicBackend;
use crate::llm::backend::{LlmBackend, LlmProvider, ReasoningEffort};
use crate::llm::conversation::{run_conversation_loop, IncomingMessage, PilotMode};
use crate::llm::gemini::GeminiBackend;
use crate::llm::openai::OpenAiBackend;
use crate::llm::tools::{ToolBridge, ToolContext, ToolCounters, ToolCountersSnapshot};
use crate::sim::datarefs::{HAS_CRASHED, PARKING_BRAKE_RATIO};
use crate::sim::simple_bridge::SimpleToolBridge;
use crate::sim::simple_dynamics::{DynamicsState, SimpleAircraftModel};
use crate::sim::xplane_bridge::{geodetic_offset_ft, GeoReference};
use crate::types::{FlightPhase, Runway, Vec2};

/// Why the eval loop ended. Fed through a channel by either the
/// `mission_complete` tool (LLM-reported) or the control loop
/// (auto-detected state) — whichever comes first wins.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OutcomeSource {
    /// LLM called the `mission_complete` tool.
    LlmReport,
    /// Control loop saw `phase==TaxiClear` + parking brake set +
    /// groundspeed < threshold, or the `park` profile in
    /// `completed_profiles`.
    AutoSuccess,
    /// `has_crashed` dataref set, or altitude well below field
    /// elevation.
    AutoCrash,
    /// Sim-time budget exceeded.
    TimeoutSim,
    /// Turn-count budget exceeded.
    TimeoutTurns,
    /// Cumulative output-token budget exceeded. Output tokens are the
    /// expensive direction with every provider, so this is the most
    /// cost-relevant cap.
    TimeoutTokens,
    /// External shutdown (Ctrl-C or harness shutdown signal).
    Shutdown,
}

#[derive(Debug, Clone)]
pub struct MissionOutcome {
    pub success: bool,
    pub summary: String,
    pub source: OutcomeSource,
}

impl MissionOutcome {
    pub fn from_llm(success: bool, summary: impl Into<String>) -> Self {
        Self {
            success,
            summary: summary.into(),
            source: OutcomeSource::LlmReport,
        }
    }
    pub fn auto_success(summary: impl Into<String>) -> Self {
        Self {
            success: true,
            summary: summary.into(),
            source: OutcomeSource::AutoSuccess,
        }
    }
    pub fn auto_crash(summary: impl Into<String>) -> Self {
        Self {
            success: false,
            summary: summary.into(),
            source: OutcomeSource::AutoCrash,
        }
    }
    pub fn timeout_sim(summary: impl Into<String>) -> Self {
        Self {
            success: false,
            summary: summary.into(),
            source: OutcomeSource::TimeoutSim,
        }
    }
    pub fn timeout_turns(summary: impl Into<String>) -> Self {
        Self {
            success: false,
            summary: summary.into(),
            source: OutcomeSource::TimeoutTurns,
        }
    }
    pub fn timeout_tokens(summary: impl Into<String>) -> Self {
        Self {
            success: false,
            summary: summary.into(),
            source: OutcomeSource::TimeoutTokens,
        }
    }
}

/// Non-blocking outcome sender. `try_send` so a full buffer (shouldn't
/// happen with an unbounded channel) or a closed receiver (harness
/// already shutting down) never panics from inside the tool handler.
pub fn try_send_outcome(tx: &Sender<MissionOutcome>, outcome: MissionOutcome) {
    let _ = tx.try_send(outcome);
}

#[derive(Debug, Clone)]
pub struct EvalConfig {
    /// ICAO airport ident where the aircraft starts + ultimately lands
    /// (e.g. "KWHP"). Looked up in `airports.parquet` for ARP +
    /// elevation, and in `runways.parquet` for the reference runway.
    pub airport: String,
    /// apt.dat parking spot name (e.g. "Ramp 7", "Gate A4") where the
    /// aircraft spawns.
    pub parking_spot: String,
    /// Directory holding `airports.parquet`, `runways.parquet`,
    /// `parking_spots.parquet` etc. Required — the eval needs the
    /// parquet tables for its own startup query and the LLM's
    /// sql_query/taxi tools.
    pub apt_dat_cache_dir: PathBuf,
    pub llm_provider: LlmProvider,
    pub llm_model: String,
    pub llm_reasoning_effort: Option<String>,
    /// Startup messages delivered to the LLM before the sim starts
    /// ticking. Each is passed through as-is (operator vs ATC source
    /// is already baked into the `IncomingMessage`). The LLM otherwise
    /// knows nothing beyond its system prompt. Empty is allowed; the
    /// LLM will sit until the first heartbeat fires.
    pub startup_messages: Vec<IncomingMessage>,
    pub pilot_mode: PilotMode,
    /// End the eval after this many LLM turns (`create_response`
    /// invocations). Prevents a stuck model from running forever.
    /// Calibrated from historical runs: the longest observed real
    /// flight used 123 turns, so the default leaves ~60% headroom.
    pub max_turns: usize,
    /// End the eval after this many cumulative *output* tokens across
    /// all turns. Output tokens are the cost-dominant direction on
    /// every provider, so this is the most meaningful spend cap.
    /// Historical max observed was ~10,400; default gives ~4x
    /// headroom. Set to 0 to disable.
    pub max_output_tokens: u64,
    /// End the eval after this many simulated seconds. Independent of
    /// `max_turns` — whichever fires first wins.
    pub max_sim_seconds: f64,
    /// Sim timestep in seconds. 0.2 matches `ScenarioRunner`; smaller
    /// values give finer-grained control but cost CPU.
    pub dt: f64,
    /// Heartbeat idle-cadence, in sim seconds. Default 30 s mirrors the
    /// live runtime.
    pub heartbeat_interval_s: f64,
    /// Directory to write the log file + transcript + summary into.
    /// Created if missing.
    pub output_dir: PathBuf,
    /// Session-stem used to name artefacts inside `output_dir`
    /// (e.g. "eval-20260424-101200"). `.log`, `.txt`, `.json` are
    /// appended.
    pub session_stem: String,
    /// Echo every log entry (LLM text, tool calls, heartbeats, tokens,
    /// startup info) to stdout as it happens. Matches the x-plane
    /// harness's headless behaviour; handy for watching a run in real
    /// time. The log file is always written regardless.
    pub verbose: bool,
}

#[derive(Debug, Clone)]
pub struct EvalResult {
    pub outcome: MissionOutcome,
    pub turn_count: usize,
    pub total_input_tokens: u64,
    pub total_cached_tokens: u64,
    pub total_output_tokens: u64,
    pub sim_duration_s: f64,
    pub wall_duration_s: f64,
    pub final_phase: Option<FlightPhase>,
    pub crashed: bool,
    pub max_alt_agl_ft: f64,
    pub final_fuel_kg: f64,
    pub tool_stats: ToolCountersSnapshot,
    pub log_file: Option<PathBuf>,
    pub transcript_file: Option<PathBuf>,
    pub summary_json: Option<PathBuf>,
}

/// Resolved lat/lon/heading/length for the start state + runway frame.
/// `pub` so tests can construct one directly without standing up a
/// parquet cache.
#[derive(Debug, Clone)]
pub struct ResolvedStart {
    /// Which runway end was actually chosen (either the one the caller
    /// Which runway ended up as the initial `RunwayFrame` anchor —
    /// the first one returned for the airport by parquet row order.
    /// Logged at startup and echoed back into the `Runway::id` field
    /// of the composed config so the initial frame is self-describing.
    pub runway_ident: String,
    pub runway_threshold_lat_deg: f64,
    pub runway_threshold_lon_deg: f64,
    pub runway_course_deg: f64,
    pub runway_length_ft: f64,
    pub runway_displaced_threshold_ft: f64,
    pub field_elevation_ft: f64,
    pub parking_lat_deg: f64,
    pub parking_lon_deg: f64,
    pub parking_heading_deg: f64,
}

fn resolve_start(cfg: &EvalConfig) -> Result<ResolvedStart> {
    use crate::llm::tools::open_apt_dat_parquet;
    let cache = &cfg.apt_dat_cache_dir;
    let airports = cache.join(AIRPORTS_FILE);
    let runways = cache.join(RUNWAYS_FILE);
    let comms = cache.join(COMMS_FILE);
    let parking = cache.join(PARKING_SPOTS_FILE);
    for p in [&airports, &runways, &comms, &parking] {
        if !p.exists() {
            return Err(anyhow!(
                "apt.dat parquet file missing: {} (run with --apt-dat-path once to build the cache)",
                p.display()
            ));
        }
    }
    let conn = open_apt_dat_parquet(&airports, &runways, &comms)?;

    // Field elevation from airports table.
    let field_elevation_ft: f64 = conn
        .query_row(
            "SELECT elevation_ft FROM airports WHERE ident = ? LIMIT 1",
            [&cfg.airport],
            |row| row.get(0),
        )
        .with_context(|| format!("looking up airport {}", cfg.airport))?;

    // Pick the first runway at the airport as the initial RunwayFrame
    // anchor. The LLM discovers runways via sql_query and picks its
    // own when it calls engage_takeoff / engage_pattern_fly — those
    // tools build their own frame from the LLM-chosen runway, so the
    // initial anchor is just scaffolding for the idle-state pilot.
    let (runway_ident, rw_lat, rw_lon, rw_hdg, rw_len, rw_disp): (
        String,
        f64,
        f64,
        f64,
        f64,
        f64,
    ) = conn
        .query_row(
            "SELECT le_ident, le_latitude_deg, le_longitude_deg, le_heading_degT,
                    length_ft, le_displaced_threshold_ft
             FROM runways
             WHERE airport_ident = ?1
             ORDER BY id
             LIMIT 1",
            [&cfg.airport],
            |row| {
                Ok((
                    row.get::<_, String>(0)?,
                    row.get::<_, f64>(1)?,
                    row.get::<_, f64>(2)?,
                    row.get::<_, f64>(3)?,
                    row.get::<_, f64>(4)?,
                    row.get::<_, f64>(5)?,
                ))
            },
        )
        .with_context(|| format!("looking up runways at {}", cfg.airport))?;

    // Parking spot.
    let (spot_lat, spot_lon, spot_hdg): (f64, f64, f64) = conn
        .query_row(
            "SELECT latitude_deg, longitude_deg, heading_true_deg
             FROM parking_spots
             WHERE airport_ident = ?1 AND name = ?2
             LIMIT 1",
            [&cfg.airport, &cfg.parking_spot],
            |row| {
                Ok((
                    row.get::<_, f64>(0)?,
                    row.get::<_, f64>(1)?,
                    row.get::<_, f64>(2)?,
                ))
            },
        )
        .with_context(|| {
            format!(
                "looking up parking spot {:?} at {}",
                cfg.parking_spot, cfg.airport
            )
        })?;

    Ok(ResolvedStart {
        runway_ident,
        runway_threshold_lat_deg: rw_lat,
        runway_threshold_lon_deg: rw_lon,
        runway_course_deg: rw_hdg,
        runway_length_ft: rw_len,
        runway_displaced_threshold_ft: rw_disp,
        field_elevation_ft,
        parking_lat_deg: spot_lat,
        parking_lon_deg: spot_lon,
        parking_heading_deg: spot_hdg,
    })
}

fn build_eval_config_bundle(
    base: ConfigBundle,
    resolved: &ResolvedStart,
    airport: &str,
    runway_ident: &str,
    runway_threshold_ft: Vec2,
) -> ConfigBundle {
    let runway = Runway {
        id: Some(runway_ident.to_string()),
        // Runway-frame origin, in world-frame feet, relative to the
        // GeoReference anchor (which sits at the aircraft's starting
        // lat/lon). `RunwayFrame::to_runway_frame` subtracts this then
        // rotates by course.
        threshold_ft: runway_threshold_ft,
        course_deg: resolved.runway_course_deg,
        length_ft: resolved.runway_length_ft,
        displaced_threshold_ft: resolved.runway_displaced_threshold_ft,
        ..base.airport.runway.clone()
    };
    let airport_cfg = AirportConfig {
        airport: Some(airport.to_string()),
        field_elevation_ft: resolved.field_elevation_ft,
        runway,
        mission: base.airport.mission.clone(),
    };
    ConfigBundle {
        airport: airport_cfg,
        ..base
    }
}

fn build_llm_client(
    provider: LlmProvider,
    model: &str,
    transcript_path: Option<&PathBuf>,
) -> Arc<dyn LlmBackend> {
    match provider {
        LlmProvider::OpenAi => {
            let mut c = OpenAiBackend::new(model.to_string());
            c.transcript_path = transcript_path.cloned();
            Arc::new(c)
        }
        LlmProvider::Anthropic => {
            let mut c = AnthropicBackend::new(model.to_string());
            c.transcript_path = transcript_path.cloned();
            Arc::new(c)
        }
        LlmProvider::Gemini => {
            let mut c = GeminiBackend::new(model.to_string());
            c.transcript_path = transcript_path.cloned();
            Arc::new(c)
        }
    }
}

/// Parameters the control-loop thread needs. Packed up so `run_eval`
/// doesn't have to pass a dozen args to a helper. Ownership/shares:
/// everything here is either `Arc`-cloned or `Copy`-cheap.
struct ControlLoopCtx {
    dynamics: Arc<PLMutex<DynamicsState>>,
    model: SimpleAircraftModel,
    pilot: Arc<PLMutex<PilotCore>>,
    bridge: Arc<SimpleToolBridge>,
    bus: SimBus,
    pump: Arc<HeartbeatPump>,
    sim_clock: SimClock,
    outcome_tx: Sender<MissionOutcome>,
    outcome_rx: Receiver<MissionOutcome>,
    turn_counter: Arc<AtomicUsize>,
    cache_stats: Option<Arc<crate::llm::backend::CacheStats>>,
    stop: Arc<AtomicBool>,
    /// LLM worker's stop flag. Flipped on outcome so the worker bails
    /// between turns instead of finishing its scripted burst.
    llm_stop: Arc<AtomicBool>,
    dt: f64,
    max_sim_seconds: f64,
    max_turns: usize,
    /// Output-token cap. `0` disables the check.
    max_output_tokens: u64,
    field_elevation_ft: f64,
}

/// Drive the sim forward until the outcome channel yields a result (or
/// budget expires). Returns the final outcome + peak AGL + final fuel.
fn run_control_loop(ctx: ControlLoopCtx) -> (MissionOutcome, f64, f64) {
    let mut max_alt_agl_ft: f64 = 0.0;
    let mut been_airborne = false;
    let wall_spin_sleep = Duration::from_millis(1);
    // Without flipping llm_stop, the worker can race past the budget
    // while we're still constructing the outcome.
    fn finish(
        ctx: &ControlLoopCtx,
        outcome: MissionOutcome,
        max_alt_agl_ft: f64,
    ) -> (MissionOutcome, f64, f64) {
        ctx.llm_stop.store(true, Ordering::Release);
        let fuel = ctx
            .bridge
            .get_dataref_value(crate::sim::datarefs::FUEL_KG.name)
            .unwrap_or(0.0);
        (outcome, max_alt_agl_ft, fuel)
    }
    loop {
        if ctx.stop.load(Ordering::Acquire) {
            return finish(
                &ctx,
                MissionOutcome {
                    success: false,
                    summary: "external shutdown".to_string(),
                    source: OutcomeSource::Shutdown,
                },
                max_alt_agl_ft,
            );
        }

        if let Ok(o) = ctx.outcome_rx.try_recv() {
            return finish(&ctx, o, max_alt_agl_ft);
        }

        // Budget checks run every iteration — a fast-stub run can blow
        // past max_turns / max_output_tokens between two ticks.
        let turns_now = ctx.turn_counter.load(Ordering::Relaxed);
        if turns_now >= ctx.max_turns {
            let outcome = MissionOutcome::timeout_turns(format!(
                "turn budget exhausted at {} turns (>= {})",
                turns_now, ctx.max_turns
            ));
            let _ = ctx.outcome_tx.try_send(outcome.clone());
            return finish(&ctx, outcome, max_alt_agl_ft);
        }
        let output_tokens = ctx
            .cache_stats
            .as_ref()
            .map(|cs| cs.snapshot().total_output_tokens)
            .unwrap_or(0);
        if ctx.max_output_tokens > 0 && output_tokens >= ctx.max_output_tokens {
            let outcome = MissionOutcome::timeout_tokens(format!(
                "output-token budget exhausted at {} tokens (>= {})",
                output_tokens, ctx.max_output_tokens
            ));
            let _ = ctx.outcome_tx.try_send(outcome.clone());
            return finish(&ctx, outcome, max_alt_agl_ft);
        }

        // Sim is paused while the LLM deliberates so sleep(N) stays
        // anchored to sim time.
        if ctx.bus.is_llm_busy() {
            thread::sleep(wall_spin_sleep);
            continue;
        }

        {
            let mut pilot = ctx.pilot.lock();
            let mut dyn_state = ctx.dynamics.lock();
            let (_, commands) = pilot.update(&dyn_state, ctx.dt);
            ctx.model.step(&mut dyn_state, &commands, ctx.dt);
        }
        ctx.bridge.refresh_derived_datarefs(ctx.dt);
        ctx.sim_clock.advance_secs(ctx.dt);
        ctx.pump.check_and_emit();

        // Update airborne tracking + auto-detection state.
        let (on_ground, alt_ft, gs_kt, phase_now, sim_time_s) = {
            let d = ctx.dynamics.lock();
            let snap = ctx.pilot.lock().latest_snapshot.clone();
            (
                d.on_ground,
                d.altitude_ft,
                d.ground_velocity_ft_s.length() / crate::types::KT_TO_FPS,
                snap.as_ref().and_then(|s| s.phase),
                d.time_s,
            )
        };
        let agl = (alt_ft - ctx.field_elevation_ft).max(0.0);
        if agl > max_alt_agl_ft {
            max_alt_agl_ft = agl;
        }
        if !on_ground {
            been_airborne = true;
        }

        // Auto-detect crash.
        let crashed = ctx
            .bridge
            .get_dataref_value(HAS_CRASHED.name)
            .map(|v| v >= 0.5)
            .unwrap_or(false);
        if crashed {
            let outcome = MissionOutcome::auto_crash(format!(
                "has_crashed set at t_sim={:.1}s phase={:?}",
                sim_time_s, phase_now
            ));
            let _ = ctx.outcome_tx.try_send(outcome.clone());
            return finish(&ctx, outcome, max_alt_agl_ft);
        }

        // Auto-success: been airborne, now stopped with parking brake.
        // been_airborne=false at startup excludes the initial state.
        if been_airborne {
            let brake_set = ctx
                .bridge
                .get_dataref_value(PARKING_BRAKE_RATIO.name)
                .map(|v| v >= 0.5)
                .unwrap_or(false);
            if on_ground && gs_kt < 1.0 && brake_set {
                let outcome = MissionOutcome::auto_success(format!(
                    "post-flight stopped with parking brake set at t_sim={:.1}s phase={:?}",
                    sim_time_s, phase_now
                ));
                let _ = ctx.outcome_tx.try_send(outcome.clone());
                return finish(&ctx, outcome, max_alt_agl_ft);
            }
        }

        // Sim-time budget goes last so it sees the state we just
        // stepped into.
        if sim_time_s >= ctx.max_sim_seconds {
            let outcome = MissionOutcome::timeout_sim(format!(
                "sim-time budget exhausted at t_sim={:.1}s (>= {:.0}s); phase={:?}",
                sim_time_s, ctx.max_sim_seconds, phase_now
            ));
            let _ = ctx.outcome_tx.try_send(outcome.clone());
            return finish(&ctx, outcome, max_alt_agl_ft);
        }
    }
}

pub fn run_eval(base_config: ConfigBundle, cfg: EvalConfig) -> Result<EvalResult> {
    let resolved = resolve_start(&cfg).context("resolving eval start state")?;
    let transcript_path = cfg.output_dir.join(format!("{}.txt", cfg.session_stem));
    let backend = build_llm_client(cfg.llm_provider, &cfg.llm_model, Some(&transcript_path));
    run_eval_core(base_config, cfg, resolved, backend)
}

/// Core eval loop with the parquet resolution and backend construction
/// factored out. Tests use this directly with a stub backend + inline
/// `ResolvedStart`, avoiding the need to build an apt.dat cache and
/// provide an API key.
pub fn run_eval_core(
    base_config: ConfigBundle,
    cfg: EvalConfig,
    resolved: ResolvedStart,
    backend: Arc<dyn LlmBackend>,
) -> Result<EvalResult> {
    // --- startup artefacts ---
    std::fs::create_dir_all(&cfg.output_dir)
        .with_context(|| format!("creating {}", cfg.output_dir.display()))?;
    let log_path = cfg.output_dir.join(format!("{}.log", cfg.session_stem));
    let transcript_path = cfg.output_dir.join(format!("{}.txt", cfg.session_stem));
    let summary_path = cfg.output_dir.join(format!("{}.json", cfg.session_stem));

    let file_log = Arc::new(FileLog::new(&log_path).context("opening file log")?);
    let bus = SimBus::with_file_log(cfg.verbose, file_log.clone());
    bus.push_log_kind(
        LogKind::System,
        format!("eval session: {}", cfg.session_stem),
    );
    bus.push_log_kind(
        LogKind::System,
        format!(
            "eval target: airport={} parking={:?}",
            cfg.airport, cfg.parking_spot
        ),
    );
    bus.push_log_kind(
        LogKind::System,
        format!(
            "initial runway frame anchored at {} (auto-picked): threshold=({:.6},{:.6}) course={:.0} length_ft={:.0}",
            resolved.runway_ident,
            resolved.runway_threshold_lat_deg,
            resolved.runway_threshold_lon_deg,
            resolved.runway_course_deg,
            resolved.runway_length_ft,
        ),
    );
    bus.push_log_kind(
        LogKind::System,
        format!(
            "resolved parking {:?}: ({:.6},{:.6}) heading={:.0} field_elev={:.0}ft",
            cfg.parking_spot,
            resolved.parking_lat_deg,
            resolved.parking_lon_deg,
            resolved.parking_heading_deg,
            resolved.field_elevation_ft,
        ),
    );

    // GeoReference is anchored at the aircraft's starting lat/lon —
    // exactly the live X-Plane convention (`run_live_xplane` seeds the
    // anchor from `probe_bootstrap_sample.posi`). That means
    // `position_ft = Vec2::ZERO` at startup, and the runway
    // threshold's world-frame offset falls out of `geodetic_offset_ft`.
    let georef = GeoReference {
        threshold_lat_deg: resolved.parking_lat_deg,
        threshold_lon_deg: resolved.parking_lon_deg,
    };
    let runway_threshold_ft = geodetic_offset_ft(
        resolved.runway_threshold_lat_deg,
        resolved.runway_threshold_lon_deg,
        georef,
    );
    let eval_config = build_eval_config_bundle(
        base_config,
        &resolved,
        &cfg.airport,
        &resolved.runway_ident,
        runway_threshold_ft,
    );

    // --- dynamics + bridge ---
    let model = SimpleAircraftModel::new(eval_config.clone(), Vec2::ZERO);
    let dyn_initial = model.initial_state_at(
        Vec2::ZERO,
        resolved.parking_heading_deg,
        resolved.field_elevation_ft,
    );
    let dynamics = Arc::new(PLMutex::new(dyn_initial));
    let bridge = Arc::new(SimpleToolBridge::new(
        dynamics.clone(),
        georef,
        resolved.field_elevation_ft,
    ));
    let tool_bridge: Arc<dyn ToolBridge> = bridge.clone();

    // --- pilot + bus wiring ---
    let pilot_arc = Arc::new(PLMutex::new(PilotCore::new(eval_config.clone())));

    // --- channels ---
    let (input_tx, input_rx) = unbounded::<IncomingMessage>();
    let (outcome_tx, outcome_rx) = unbounded::<MissionOutcome>();

    // --- sim clock + heartbeat pump ---
    let sim_clock = SimClock::new();
    let clock_trait: Arc<dyn Clock> = Arc::new(sim_clock.clone());
    let pump = Arc::new(
        HeartbeatPump::new(
            pilot_arc.clone(),
            Some(bus.clone()),
            input_tx.clone(),
            cfg.heartbeat_interval_s,
            clock_trait,
        )
        .with_bridge(tool_bridge.clone()),
    );

    // --- tool context ---
    let reasoning_effort: Option<ReasoningEffort> = cfg
        .llm_reasoning_effort
        .as_deref()
        .and_then(|s| s.parse::<ReasoningEffort>().ok());
    bus.push_log_kind(
        LogKind::System,
        format!(
            "llm provider={} model={}{}",
            cfg.llm_provider.as_str(),
            cfg.llm_model,
            match reasoning_effort {
                Some(e) => format!(" reasoning={}", e.as_str()),
                None => String::new(),
            }
        ),
    );

    let turn_counter = Arc::new(AtomicUsize::new(0));
    let tool_counters = Arc::new(ToolCounters::default());

    let tool_ctx = Arc::new(ToolContext {
        pilot: pilot_arc.clone(),
        bridge: Some(tool_bridge),
        config: Arc::new(PLMutex::new(eval_config.clone())),
        recent_broadcasts: Arc::new(PLMutex::new(Vec::new())),
        apt_dat_cache_dir: Some(cfg.apt_dat_cache_dir.clone()),
        bus: Some(bus.clone()),
        runway_conn: Arc::new(Mutex::new(None)),
        heartbeat_pump: Some(pump.clone()),
        mission_complete_tx: Some(outcome_tx.clone()),
        turn_counter: Some(turn_counter.clone()),
        tool_counters: Some(tool_counters.clone()),
    });

    // --- deliver startup messages ---
    for msg in &cfg.startup_messages {
        bus.push_log_kind(
            LogKind::System,
            format!("[startup {}] {}", msg.source.tag(), msg.text),
        );
        input_tx.send(msg.clone()).ok();
    }

    // --- spawn LLM worker ---
    let llm_stop = Arc::new(AtomicBool::new(false));
    let llm_thread = {
        let client = backend.clone();
        let ctx = tool_ctx.clone();
        let rx = input_rx.clone();
        let stop = llm_stop.clone();
        let bus_clone = bus.clone();
        let pilot_mode = cfg.pilot_mode;
        thread::Builder::new()
            .name("llm-worker".into())
            .spawn(move || {
                run_conversation_loop(
                    &*client,
                    &ctx,
                    &rx,
                    stop,
                    60,
                    // Budget is measured in wall seconds inside the
                    // conversation loop. We'd rather bound the eval by
                    // sim seconds + turn count — so lift this cap to
                    // effectively-never so wall latency doesn't kill
                    // the turn early. The control loop still gets the
                    // last word via the outcome channel + stop flag.
                    3_600,
                    pilot_mode,
                    reasoning_effort,
                    Some(&bus_clone),
                );
            })?
    };

    // --- run the control loop (this thread) ---
    let wall_start = Instant::now();
    let control_stop = Arc::new(AtomicBool::new(false));
    let cache_stats = backend.cache_stats_arc();
    let ctrl_ctx = ControlLoopCtx {
        dynamics: dynamics.clone(),
        model,
        pilot: pilot_arc.clone(),
        bridge: bridge.clone(),
        bus: bus.clone(),
        pump: pump.clone(),
        sim_clock: sim_clock.clone(),
        outcome_tx: outcome_tx.clone(),
        outcome_rx: outcome_rx.clone(),
        turn_counter: turn_counter.clone(),
        cache_stats: cache_stats.clone(),
        stop: control_stop.clone(),
        llm_stop: llm_stop.clone(),
        dt: cfg.dt,
        max_sim_seconds: cfg.max_sim_seconds,
        max_turns: cfg.max_turns,
        max_output_tokens: cfg.max_output_tokens,
        field_elevation_ft: resolved.field_elevation_ft,
    };
    let (outcome, max_alt_agl_ft, final_fuel_kg) = run_control_loop(ctrl_ctx);
    let wall_duration_s = wall_start.elapsed().as_secs_f64();

    // --- shutdown LLM worker ---
    llm_stop.store(true, Ordering::Release);
    // Close the input channel so the worker's recv_timeout unblocks
    // quickly even if we happen to be between ticks.
    drop(input_tx);
    let _ = llm_thread.join();

    // --- collect final metrics ---
    let final_snapshot = pilot_arc.lock().latest_snapshot.clone();
    let final_phase = final_snapshot.as_ref().and_then(|s| s.phase);
    let crashed = bridge
        .get_dataref_value(HAS_CRASHED.name)
        .map(|v| v >= 0.5)
        .unwrap_or(false);
    let sim_duration_s = dynamics.lock().time_s;
    let turn_count = turn_counter.load(Ordering::Relaxed);
    let token_totals = cache_stats.as_ref().map(|s| s.snapshot()).unwrap_or_default();
    let tool_stats = tool_counters.snapshot();

    bus.push_log_kind(
        LogKind::System,
        format!(
            "eval finished: {:?} success={} turns={} tools={}/{} (failed/total) tokens(in={} cached={} out={}) sim={:.1}s wall={:.1}s",
            outcome.source,
            outcome.success,
            turn_count,
            tool_stats.failed,
            tool_stats.total,
            token_totals.total_input_tokens,
            token_totals.total_cached_tokens,
            token_totals.total_output_tokens,
            sim_duration_s,
            wall_duration_s,
        ),
    );
    if !tool_stats.per_tool.is_empty() {
        let breakdown = tool_stats
            .per_tool
            .iter()
            .map(|(name, s)| format!("{}={}/{}", name, s.failures, s.calls))
            .collect::<Vec<_>>()
            .join(" ");
        bus.push_log_kind(LogKind::System, format!("tool breakdown: {}", breakdown));
    }

    // --- write summary JSON ---
    let tool_breakdown_json: serde_json::Map<String, serde_json::Value> = tool_stats
        .per_tool
        .iter()
        .map(|(name, s)| {
            (
                name.clone(),
                serde_json::json!({ "calls": s.calls, "failures": s.failures }),
            )
        })
        .collect();
    let summary = serde_json::json!({
        "session_stem": cfg.session_stem,
        "airport": cfg.airport,
        "parking_spot": cfg.parking_spot,
        "initial_runway_frame_ident": resolved.runway_ident,
        "llm_provider": cfg.llm_provider.as_str(),
        "llm_model": cfg.llm_model,
        "outcome": {
            "success": outcome.success,
            "source": format!("{:?}", outcome.source),
            "summary": outcome.summary,
        },
        "turns": turn_count,
        "total_input_tokens": token_totals.total_input_tokens,
        "total_cached_tokens": token_totals.total_cached_tokens,
        "total_output_tokens": token_totals.total_output_tokens,
        "sim_duration_s": sim_duration_s,
        "wall_duration_s": wall_duration_s,
        "final_phase": final_phase.map(|p| p.value().to_string()),
        "crashed": crashed,
        "max_alt_agl_ft": max_alt_agl_ft,
        "final_fuel_kg": final_fuel_kg,
        "tool_calls": tool_stats.total,
        "tool_failures": tool_stats.failed,
        "tool_breakdown": tool_breakdown_json,
    });
    std::fs::write(&summary_path, serde_json::to_string_pretty(&summary)?)
        .with_context(|| format!("writing {}", summary_path.display()))?;

    bus.close();

    Ok(EvalResult {
        outcome,
        turn_count,
        total_input_tokens: token_totals.total_input_tokens,
        total_cached_tokens: token_totals.total_cached_tokens,
        total_output_tokens: token_totals.total_output_tokens,
        sim_duration_s,
        wall_duration_s,
        final_phase,
        crashed,
        max_alt_agl_ft,
        final_fuel_kg,
        tool_stats,
        log_file: Some(log_path),
        transcript_file: Some(transcript_path),
        summary_json: Some(summary_path),
    })
}
