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
use crate::llm::sim_atc::SimAtc;
use crate::live_runner::{Clock, HeartbeatPump, SimClock};
use crate::llm::backend::{LlmProvider, ReasoningEffort};
use crate::llm::brain::{PilotBrain, WorkerArgs};
use crate::llm::conversation::{IncomingMessage, PilotMode};
use crate::llm::tools::{ToolBridge, ToolContext, ToolCounters, ToolCountersSnapshot};
use crate::sim::datarefs::{HAS_CRASHED, PARKING_BRAKE_RATIO};
use crate::sim::simple_bridge::SimpleToolBridge;
use crate::sim::simple_dynamics::{DynamicsState, SimpleAircraftModel};
use crate::sim::xplane_bridge::{geodetic_offset_ft, GeoReference};
use crate::track::{TrackPoint, TrackRecorder};
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
    /// Model for a simulated controller that answers the pilot's radio
    /// calls (see `llm::sim_atc`). `None` = nobody answers.
    pub sim_atc_model: Option<String>,
    /// A scripted scenario (`evals/plan/*.yaml`): start state, timed or
    /// triggered ATC calls, end condition and checks. `None` = the
    /// classic parked-start mission eval.
    pub scenario: Option<crate::eval_scenario::Scenario>,
}

#[derive(Debug, Clone)]
pub struct EvalResult {
    pub outcome: MissionOutcome,
    pub turn_count: usize,
    pub total_input_tokens: u64,
    pub total_cached_tokens: u64,
    /// Cumulative cache-write tokens. Anthropic-only (billed at 1.25×
    /// base); always 0 for OpenAI/Gemini.
    pub total_cache_creation_tokens: u64,
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
    pub track_csv: Option<PathBuf>,
    pub track_kml: Option<PathBuf>,
    /// Scenario checks, scored over the flown track.
    pub checks: Vec<crate::eval_scenario::CheckResult>,
    pub ground_svg: Option<PathBuf>,
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
    pub runway_width_ft: f64,
    pub runway_displaced_threshold_ft: f64,
    pub field_elevation_ft: f64,
    pub parking_lat_deg: f64,
    pub parking_lon_deg: f64,
    pub parking_heading_deg: f64,
}

/// Runways and frequencies at the eval airport, for the simulated ATC.
fn airport_brief(cfg: &EvalConfig) -> Result<String> {
    use crate::llm::tools::open_apt_dat_parquet;
    let cache = &cfg.apt_dat_cache_dir;
    let conn = open_apt_dat_parquet(
        &cache.join(AIRPORTS_FILE),
        &cache.join(RUNWAYS_FILE),
        &cache.join(COMMS_FILE),
    )?;
    let name: String = conn.query_row(
        "SELECT name FROM airports WHERE ident = ? LIMIT 1",
        [&cfg.airport],
        |r| r.get(0),
    )?;
    let mut out = format!("Airport: {} ({name}).\nRunways:\n", cfg.airport);
    let mut stmt = conn.prepare(
        "SELECT le_ident, he_ident, length_ft FROM runways WHERE airport_ident = ? AND closed = 0",
    )?;
    let rows = stmt.query_map([&cfg.airport], |r| {
        Ok((r.get::<_, String>(0)?, r.get::<_, String>(1)?, r.get::<_, Option<f64>>(2)?))
    })?;
    for row in rows {
        let (le, he, len) = row?;
        out.push_str(&format!("- {le}/{he}, {:.0} ft\n", len.unwrap_or(0.0)));
    }
    out.push_str("Frequencies:\n");
    let mut stmt = conn.prepare(
        "SELECT kind, freq_mhz, label FROM comms WHERE airport_ident = ? ORDER BY kind",
    )?;
    let rows = stmt.query_map([&cfg.airport], |r| {
        Ok((r.get::<_, String>(0)?, r.get::<_, f64>(1)?, r.get::<_, Option<String>>(2)?))
    })?;
    for row in rows {
        let (kind, freq, label) = row?;
        out.push_str(&format!("- {kind} {freq:.3} {}\n", label.unwrap_or_default()));
    }
    Ok(out)
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
    // own when it flies a plan — that builds its own frame from the LLM-chosen runway, so the
    // initial anchor is just scaffolding for the idle-state pilot.
    #[allow(clippy::type_complexity)]
    let (runway_ident, rw_lat, rw_lon, rw_hdg, rw_len, rw_disp, rw_width): (
        String,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
    ) = conn
        .query_row(
            "SELECT le_ident, le_latitude_deg, le_longitude_deg, le_heading_degT,
                    length_ft, le_displaced_threshold_ft, width_ft
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
                    row.get::<_, f64>(6)?,
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
        runway_width_ft: rw_width,
        runway_displaced_threshold_ft: rw_disp,
        field_elevation_ft,
        parking_lat_deg: spot_lat,
        parking_lon_deg: spot_lon,
        parking_heading_deg: spot_hdg,
    })
}

/// Start for an airborne or on-runway scenario: the named runway anchors
/// the frame and the georeference sits at the airport reference point.
fn resolve_start_runway(cfg: &EvalConfig, runway: &str, heading_deg: Option<f64>) -> Result<ResolvedStart> {
    use crate::llm::tools::{open_apt_dat_parquet, query_runway_end};
    let cache = &cfg.apt_dat_cache_dir;
    let conn = open_apt_dat_parquet(&cache.join(AIRPORTS_FILE), &cache.join(RUNWAYS_FILE), &cache.join(COMMS_FILE))?;
    let (elev, lat, lon): (f64, f64, f64) = conn
        .query_row(
            "SELECT elevation_ft, latitude_deg, longitude_deg FROM airports WHERE ident = ? LIMIT 1",
            [&cfg.airport],
            |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)),
        )
        .with_context(|| format!("looking up airport {}", cfg.airport))?;
    let end = query_runway_end(&conn, &cfg.airport, runway)?;
    Ok(ResolvedStart {
        runway_ident: runway.to_string(),
        runway_threshold_lat_deg: end.threshold_lat_deg,
        runway_threshold_lon_deg: end.threshold_lon_deg,
        runway_course_deg: end.course_deg,
        runway_length_ft: end.length_ft,
        runway_width_ft: end.width_ft,
        runway_displaced_threshold_ft: end.displaced_threshold_ft,
        field_elevation_ft: elev,
        parking_lat_deg: lat,
        parking_lon_deg: lon,
        parking_heading_deg: heading_deg.unwrap_or(end.course_deg),
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
        width_ft: resolved.runway_width_ft,
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

/// Parameters the control-loop thread needs. Packed up so `run_eval`
/// doesn't have to pass a dozen args to a helper. Ownership/shares:
/// everything here is either `Arc`-cloned or `Copy`-cheap.
struct ControlLoopCtx {
    /// While it's composing an answer, the sim runs at 1x.
    sim_atc: Option<Arc<SimAtc>>,
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
    /// Flight-track recorder. Sampled once per sim-second so the KML
    /// replay cadence matches the simulated flight, not wall-clock.
    track_recorder: Option<Arc<PLMutex<TrackRecorder>>>,
    /// Wall-clock anchor for KML `<when>` timestamps. Combined with
    /// `t_sim` so Google Earth replays at simulated pace even when the
    /// eval ran much faster (or slower) than realtime.
    track_wall_start: chrono::DateTime<chrono::Utc>,
    /// Scenario: scripted messages, the per-tick track, and whether the
    /// run ends once the aircraft has landed and stopped (or at a set
    /// time).
    schedule: Option<crate::eval_scenario::MessageSchedule>,
    /// When each scripted message went out, for scoring.
    sent_at: Arc<PLMutex<Vec<Option<f64>>>>,
    input_tx: Sender<IncomingMessage>,
    /// The pilot's inbox. The sim holds still while messages wait in it,
    /// as it does while the LLM is thinking: a slow-starting worker (a
    /// Claude Code child) mustn't let the sim run past its first message.
    input_rx: Receiver<IncomingMessage>,
    samples: Arc<PLMutex<Vec<crate::eval_scenario::Sample>>>,
    scenario_end: Option<ScenarioEnd>,
    /// Scripted traffic, fed to the pilot's traffic picture every tick.
    traffic: Option<Arc<crate::eval_scenario::ScriptedTraffic>>,
}

#[derive(Debug, Clone, Copy)]
struct ScenarioEnd {
    /// End after landing and slowing to a stop.
    on_landed: bool,
    /// End (as a completed run) at this sim time.
    after_s: Option<f64>,
}

/// Build a `TrackPoint` and hand it to the recorder. Called only when
/// the 1 Hz sim-time gate fires, so the extra dynamics/snapshot reads
/// stay off the 5 Hz hot path.
#[allow(clippy::too_many_arguments)]
fn sample_eval_track(
    recorder: &Arc<PLMutex<TrackRecorder>>,
    bridge: &Arc<SimpleToolBridge>,
    dynamics: &Arc<PLMutex<DynamicsState>>,
    pilot: &Arc<PLMutex<PilotCore>>,
    bus: &SimBus,
    wall_start: chrono::DateTime<chrono::Utc>,
    sim_time_s: f64,
    alt_msl_ft: f64,
    alt_agl_ft: f64,
    gs_kt: f64,
    phase: Option<FlightPhase>,
    on_ground: bool,
) {
    use crate::llm::tools::ToolBridge;
    let tool_bridge: &dyn ToolBridge = bridge.as_ref();
    let lat = tool_bridge.get_dataref_value(crate::sim::datarefs::LATITUDE_DEG.name);
    let lon = tool_bridge.get_dataref_value(crate::sim::datarefs::LONGITUDE_DEG.name);
    let (Some(lat_deg), Some(lon_deg)) = (lat, lon) else {
        return;
    };
    let (heading_deg, ias_kt, vs_fpm) = {
        let d = dynamics.lock();
        (d.heading_deg, d.ias_kt, d.vertical_speed_ft_s * 60.0)
    };
    // Ground-track angle is ill-defined at zero groundspeed; fall back
    // to heading when the pilot snapshot isn't available yet either.
    let track_deg = pilot
        .lock()
        .latest_snapshot
        .as_ref()
        .map(|s| s.state.track_deg)
        .unwrap_or(heading_deg);
    let wall_time =
        wall_start + chrono::Duration::milliseconds((sim_time_s * 1000.0) as i64);
    let pt = TrackPoint {
        wall_time,
        t_sim: sim_time_s,
        lat_deg,
        lon_deg,
        alt_msl_ft,
        alt_agl_ft,
        heading_deg,
        track_deg,
        ias_kt,
        gs_kt,
        vs_fpm,
        phase: phase
            .map(|p| p.value().to_string())
            .unwrap_or_else(|| "none".to_string()),
        on_ground,
    };
    if let Err(e) = recorder.lock().record(pt) {
        bus.push_log_kind(LogKind::Error, format!("[track] write error: {e}"));
    }
}

/// Drive the sim forward until the outcome channel yields a result (or
/// budget expires). Returns the final outcome + peak AGL + final fuel.
fn run_control_loop(mut ctx: ControlLoopCtx) -> (MissionOutcome, f64, f64) {
    let mut max_alt_agl_ft: f64 = 0.0;
    let mut been_airborne = false;
    let wall_spin_sleep = Duration::from_millis(1);
    // Sample the track once per sim-second. `0.0` first-sample fires on
    // tick 1 since sim_time_s starts at 0 and monotonically increases.
    let mut next_track_sim_s: f64 = 0.0;
    let track_period_sim_s: f64 = 1.0;
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
        if ctx.bus.is_llm_busy() || !ctx.input_rx.is_empty() {
            thread::sleep(wall_spin_sleep);
            continue;
        }

        {
            let mut pilot = ctx.pilot.lock();
            let mut dyn_state = ctx.dynamics.lock();
            if let Some(t) = &ctx.traffic {
                pilot.traffic.set(t.at(dyn_state.time_s));
            }
            let (_, commands) = pilot.update(&dyn_state, ctx.dt);
            ctx.model.step(&mut dyn_state, &commands, ctx.dt);
        }
        ctx.bridge.refresh_derived_datarefs(ctx.dt);
        ctx.sim_clock.advance_secs(ctx.dt);
        ctx.pump.check_and_emit();
        // A radio call is waiting for the controller: real time, so its
        // thinking reads as a radio delay of seconds, not minutes.
        if ctx.sim_atc.as_ref().is_some_and(|a| a.composing()) {
            thread::sleep(Duration::from_secs_f64(ctx.dt));
        }

        // Update airborne tracking + auto-detection state.
        let (on_ground, alt_ft, gs_kt, phase_now, sim_time_s) = {
            let d = ctx.dynamics.lock();
            let phase = ctx.pilot.lock().latest_snapshot.as_ref().and_then(|s| s.phase);
            (
                d.on_ground,
                d.altitude_ft,
                d.ground_velocity_ft_s.length() / crate::types::KT_TO_FPS,
                phase,
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

        if ctx.scenario_end.is_some() {
            let (pos, track, bank_deg, vs_fpm) = {
                let d = ctx.dynamics.lock();
                let trk = if d.ground_velocity_ft_s.length() > 1.0 {
                    crate::types::vector_to_heading(d.ground_velocity_ft_s)
                } else {
                    d.heading_deg
                };
                (d.position_ft, trk, d.roll_deg, d.vertical_speed_ft_s * 60.0)
            };
            let sample = crate::eval_scenario::Sample {
                t: sim_time_s,
                pos,
                alt_msl_ft: alt_ft,
                alt_agl_ft: agl,
                track_deg: track,
                phase: phase_now,
                on_ground,
                crashed: ctx.bridge.get_dataref_value(HAS_CRASHED.name).is_some_and(|v| v >= 0.5),
                bank_deg,
                vs_fpm,
            };
            ctx.samples.lock().push(sample);
            if let Some(schedule) = ctx.schedule.as_mut() {
                let due = schedule.due(&sample, ctx.dt);
                if !due.is_empty() {
                    *ctx.sent_at.lock() = schedule.sent_at.clone();
                }
                for (i, m) in due {
                    if let Some(text) = &m.atc {
                        ctx.bus.push_log_kind(LogKind::System, format!("[scenario msg {i} t={sim_time_s:.0}s] ATC: {text}"));
                        ctx.bus.push_radio(format!("[atc] {text}"));
                        let _ = ctx.input_tx.send(IncomingMessage::atc(text.clone()));
                    }
                    if let Some(text) = &m.operator {
                        ctx.bus.push_log_kind(LogKind::System, format!("[scenario msg {i} t={sim_time_s:.0}s] operator: {text}"));
                        let _ = ctx.input_tx.send(IncomingMessage::operator(text.clone()));
                    }
                }
            }
            let end = ctx.scenario_end.unwrap();
            if end.on_landed && been_airborne && on_ground && gs_kt < 3.0 {
                let outcome = MissionOutcome::auto_success(format!("landed and stopped at t_sim={sim_time_s:.1}s"));
                let _ = ctx.outcome_tx.try_send(outcome.clone());
                return finish(&ctx, outcome, max_alt_agl_ft);
            }
            if end.after_s.is_some_and(|t| sim_time_s >= t) {
                let outcome = MissionOutcome::auto_success(format!("scenario time reached at t_sim={sim_time_s:.1}s"));
                let _ = ctx.outcome_tx.try_send(outcome.clone());
                return finish(&ctx, outcome, max_alt_agl_ft);
            }
        }

        // Sample the flight track at 1 Hz in sim-time. All extra field
        // reads happen inside the gate so the main 5 Hz path stays lean.
        if let Some(recorder) = ctx.track_recorder.as_ref() {
            if sim_time_s >= next_track_sim_s {
                sample_eval_track(
                    recorder,
                    &ctx.bridge,
                    &ctx.dynamics,
                    &ctx.pilot,
                    &ctx.bus,
                    ctx.track_wall_start,
                    sim_time_s,
                    alt_ft,
                    agl,
                    gs_kt,
                    phase_now,
                    on_ground,
                );
                next_track_sim_s += track_period_sim_s;
            }
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
    let transcript_path = cfg.output_dir.join(format!("{}.txt", cfg.session_stem));
    let brain = PilotBrain::build(cfg.llm_provider, &cfg.llm_model, Some(&transcript_path));
    run_eval_with(base_config, cfg, brain)
}

/// `run_eval` with the brain supplied (tests script the pilot), still
/// resolving the start from the apt.dat cache.
pub fn run_eval_with(base_config: ConfigBundle, cfg: EvalConfig, brain: PilotBrain) -> Result<EvalResult> {
    use crate::eval_scenario::Start;
    let resolved = match cfg.scenario.as_ref().map(|s| &s.start) {
        Some(Start::Airborne { airborne }) => resolve_start_runway(&cfg, &airborne.runway, Some(airborne.heading_deg)),
        Some(Start::Runway { runway }) => resolve_start_runway(&cfg, &runway.runway, None),
        _ => resolve_start(&cfg),
    }
    .context("resolving eval start state")?;
    run_eval_with_brain(base_config, cfg, resolved, brain)
}

/// Core eval loop with the parquet resolution and brain construction
/// factored out. Tests use this directly with a stub backend (or a fake
/// Claude Code) and an inline `ResolvedStart`, avoiding the need to build
/// an apt.dat cache and provide an API key.
pub fn run_eval_with_brain(
    base_config: ConfigBundle,
    cfg: EvalConfig,
    resolved: ResolvedStart,
    brain: PilotBrain,
) -> Result<EvalResult> {
    // --- startup artefacts ---
    std::fs::create_dir_all(&cfg.output_dir)
        .with_context(|| format!("creating {}", cfg.output_dir.display()))?;
    let log_path = cfg.output_dir.join(format!("{}.log", cfg.session_stem));
    let transcript_path = cfg.output_dir.join(format!("{}.txt", cfg.session_stem));
    let summary_path = cfg.output_dir.join(format!("{}.json", cfg.session_stem));
    let track_csv_path = cfg.output_dir.join(format!("{}.csv", cfg.session_stem));
    let track_kml_path = cfg.output_dir.join(format!("{}.kml", cfg.session_stem));

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
    let wind_kt = cfg.scenario.as_ref().and_then(|s| s.wind).map(|w| w.vector_kt()).unwrap_or(Vec2::ZERO);
    let model = SimpleAircraftModel::new(eval_config.clone(), wind_kt);
    let mut dyn_initial = model.initial_state_at(
        Vec2::ZERO,
        resolved.parking_heading_deg,
        resolved.field_elevation_ft,
    );
    // Parked with the parking brake set, like a ramp start in X-Plane.
    dyn_initial.parking_brake = 1.0;
    let airborne_start = cfg.scenario.as_ref().and_then(|s| match &s.start {
        crate::eval_scenario::Start::Airborne { airborne } => Some(airborne.clone()),
        _ => None,
    });
    if let Some(crate::eval_scenario::Start::Runway { runway }) = cfg.scenario.as_ref().map(|s| &s.start) {
        let frame = crate::guidance::runway_geometry::RunwayFrame::new(eval_config.airport.runway.clone());
        dyn_initial.position_ft = frame.to_world_frame(Vec2::new(runway.along_ft, 0.0));
        dyn_initial.heading_deg = eval_config.airport.runway.course_deg;
        dyn_initial.parking_brake = 0.0;
    }
    if let Some(a) = &airborne_start {
        let frame = crate::guidance::runway_geometry::RunwayFrame::new(eval_config.airport.runway.clone());
        dyn_initial.position_ft = frame.to_world_frame(Vec2::new(a.along_ft, a.right_ft));
        dyn_initial.altitude_ft = a
            .alt_msl_ft
            .or(a.alt_agl_ft.map(|h| resolved.field_elevation_ft + h))
            .unwrap_or(resolved.field_elevation_ft + eval_config.pattern.altitude_agl_ft);
        dyn_initial.heading_deg = a.heading_deg;
        dyn_initial.ias_kt = a.ias_kt;
        dyn_initial.on_ground = false;
        dyn_initial.parking_brake = 0.0;
        dyn_initial.throttle_pos = 0.5;
        dyn_initial.pitch_deg = 2.0;
        dyn_initial.ground_velocity_ft_s =
            crate::types::heading_to_vector(a.heading_deg, a.ias_kt * crate::types::KT_TO_FPS);
    }
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
            "pilot llm: provider={} model={}{}",
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

    let sim_atc = match &cfg.sim_atc_model {
        Some(model) => {
            let brief = airport_brief(&cfg).context("describing the airport for the simulated ATC")?;
            bus.push_log_kind(LogKind::System, format!("simulated ATC: claude-code / {model}"));
            Some(SimAtc::start(model, &brief, input_tx.clone(), Some(bus.clone()))?)
        }
        None => None,
    };

    let tool_ctx = Arc::new(ToolContext {
        pilot: pilot_arc.clone(),
        bridge: Some(tool_bridge),
        config: Arc::new(PLMutex::new(eval_config.clone())),
        recent_broadcasts: Arc::new(PLMutex::new(Vec::new())),
        radio_log: Arc::new(PLMutex::new(Vec::new())),
        apt_dat_cache_dir: Some(cfg.apt_dat_cache_dir.clone()),
        bus: Some(bus.clone()),
        runway_conn: Arc::new(Mutex::new(None)),
        heartbeat_pump: Some(pump.clone()),
        // Scenarios end on their own conditions; mission_complete would
        // only let the pilot stop early.
        mission_complete_tx: cfg.scenario.is_none().then(|| outcome_tx.clone()),
        turn_counter: Some(turn_counter.clone()),
        tool_counters: Some(tool_counters.clone()),
        taxi_graphs: Default::default(),
        sim_atc: sim_atc.clone(),
    });

    // Prime `latest_snapshot` so the LLM's first `get_status` returns
    // real position/profile data instead of `{"status":"uninitialized"}`.
    // Without this, the LLM thread races ahead of the control loop:
    // startup messages are queued below before any sim tick can run,
    // and the `llm_busy` gate then keeps the sim paused until the LLM
    // calls `sleep` — burning turns on uninitialized state.
    {
        let mut pilot = pilot_arc.lock();
        let dyn_state = dynamics.lock();
        let _ = pilot.update(&dyn_state, cfg.dt);
        // An airborne start hands over with the autopilot holding
        // heading, altitude and speed, so nothing coasts while the pilot
        // reads the briefing.
        if airborne_start.is_some() {
            use crate::core::profiles::{AltitudeHoldProfile, HeadingHoldProfile, SpeedHoldProfile};
            let _ = pilot.engage_profiles(vec![
                Box::new(HeadingHoldProfile::new(dyn_state.heading_deg, eval_config.limits.max_bank_pattern_deg)),
                Box::new(AltitudeHoldProfile::new(dyn_state.altitude_ft)),
                Box::new(SpeedHoldProfile::new(dyn_state.ias_kt)),
            ]);
            let _ = pilot.update(&dyn_state, cfg.dt);
        }
    }

    // Scenario plumbing: scripted messages (resolved against the same
    // place list and runways the plan uses) and a per-tick track.
    let scenario_resolver_parts = (georef, crate::llm::tools::DuckAirports { ctx: &tool_ctx });
    let schedule = match &cfg.scenario {
        Some(s) => {
            let mut r = crate::plan::resolve::Resolver::new(
                scenario_resolver_parts.0,
                &crate::plan::gazetteer::GAZETTEER,
                &scenario_resolver_parts.1,
            );
            Some(crate::eval_scenario::MessageSchedule::new(&s.messages, &mut r).context("scenario messages")?)
        }
        None => None,
    };
    let scripted_traffic = match &cfg.scenario {
        Some(s) if !s.traffic.is_empty() => {
            let mut r = crate::plan::resolve::Resolver::new(
                scenario_resolver_parts.0,
                &crate::plan::gazetteer::GAZETTEER,
                &scenario_resolver_parts.1,
            );
            let t = crate::eval_scenario::ScriptedTraffic::new(&s.traffic, &cfg.airport, resolved.field_elevation_ft, &mut r)
                .context("scenario traffic")?;
            {
                let mut pilot = pilot_arc.lock();
                pilot.traffic.set(t.at(0.0));
                pilot.refresh_traffic_status();
            }
            Some(Arc::new(t))
        }
        _ => None,
    };
    let samples: Arc<PLMutex<Vec<crate::eval_scenario::Sample>>> = Arc::new(PLMutex::new(Vec::new()));
    let sent_at: Arc<PLMutex<Vec<Option<f64>>>> = Arc::new(PLMutex::new(Vec::new()));

    // --- deliver startup messages ---
    for msg in &cfg.startup_messages {
        bus.push_log_kind(
            LogKind::System,
            format!("[startup {}] {}", msg.source.tag(), msg.text),
        );
        if msg.source == crate::llm::conversation::IncomingSource::Atc {
            bus.push_radio(format!("[atc] {}", msg.text));
        }
        input_tx.send(msg.clone()).ok();
    }

    // --- spawn LLM worker ---
    let llm_stop = Arc::new(AtomicBool::new(false));
    // Budget is measured in wall seconds inside the worker. We'd rather
    // bound the eval by sim seconds + turn count — so lift this cap to
    // effectively-never so wall latency doesn't kill the turn early. The
    // control loop still gets the last word via the outcome channel +
    // stop flag.
    let llm_cache_stats = brain.cache_stats();
    let llm_thread = brain.spawn_worker(WorkerArgs {
        tool_context: tool_ctx.clone(),
        input_queue: input_rx.clone(),
        stop: llm_stop.clone(),
        per_request_timeout_s: 60,
        total_wall_budget_s: 3_600,
        pilot_mode: cfg.pilot_mode,
        reasoning_effort,
        bus: bus.clone(),
    })?;

    // --- track recorder ---
    let track_recorder = match TrackRecorder::new(&track_csv_path) {
        Ok(r) => {
            bus.push_log_kind(
                LogKind::System,
                format!("track_csv={} (1 Hz sim-time)", track_csv_path.display()),
            );
            Some(Arc::new(PLMutex::new(r)))
        }
        Err(e) => {
            bus.push_log_kind(
                LogKind::Error,
                format!("track recorder disabled: {e}"),
            );
            None
        }
    };

    // --- run the control loop (this thread) ---
    let wall_start = Instant::now();
    let track_wall_start = chrono::Utc::now();
    let control_stop = Arc::new(AtomicBool::new(false));
    let cache_stats = llm_cache_stats;
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
        track_recorder: track_recorder.clone(),
        track_wall_start,
        sim_atc: sim_atc.clone(),
        schedule,
        sent_at: sent_at.clone(),
        input_tx: input_tx.clone(),
        input_rx: input_rx.clone(),
        samples: samples.clone(),
        traffic: scripted_traffic.clone(),
        scenario_end: cfg.scenario.as_ref().map(|s| ScenarioEnd {
            on_landed: s.end_after_s.is_none(),
            after_s: s.end_after_s,
        }),
    };
    let (outcome, max_alt_agl_ft, final_fuel_kg) = run_control_loop(ctrl_ctx);
    if let Some(atc) = &sim_atc {
        atc.shutdown();
    }
    let wall_duration_s = wall_start.elapsed().as_secs_f64();

    // --- shutdown LLM worker ---
    llm_stop.store(true, Ordering::Release);
    // Close the input channel so the worker's recv_timeout unblocks
    // quickly even if we happen to be between ticks.
    drop(input_tx);
    let _ = llm_thread.join();

    // --- flush KML ---
    let track_kml_written = if let Some(rec) = &track_recorder {
        let guard = rec.lock();
        match guard.write_kml(&track_kml_path, &cfg.session_stem) {
            Ok(_) => {
                bus.push_log_kind(
                    LogKind::System,
                    format!(
                        "track_kml={} ({} points)",
                        track_kml_path.display(),
                        guard.len()
                    ),
                );
                true
            }
            Err(e) => {
                bus.push_log_kind(
                    LogKind::Error,
                    format!("track_kml write failed: {e}"),
                );
                false
            }
        }
    } else {
        false
    };

    // --- score scenario checks + ground track ---
    let ground_svg_path = cfg.output_dir.join(format!("{}-ground.svg", cfg.session_stem));
    let (checks, ground_svg) = match &cfg.scenario {
        Some(s) => {
            use crate::plan::resolve::AirportSource;
            let samples = samples.lock().clone();
            let sent = sent_at.lock().clone();
            let airports = crate::llm::tools::DuckAirports { ctx: &tool_ctx };
            let mut resolver =
                crate::plan::resolve::Resolver::new(georef, &crate::plan::gazetteer::GAZETTEER, &airports);
            let apt = airports.airport(&cfg.airport).ok();
            let mut frame_for = |rwy: &str| -> Result<crate::guidance::runway_geometry::RunwayFrame> {
                let apt = apt.as_ref().ok_or_else(|| anyhow!("no airport data for {}", cfg.airport))?;
                let info = apt.runway(rwy).ok_or_else(|| anyhow!("no runway {rwy} at {}", cfg.airport))?;
                Ok(crate::guidance::runway_geometry::RunwayFrame::new(crate::plan::compile::runway_from_info(
                    info,
                    georef,
                    crate::types::TrafficSide::Left,
                )))
            };
            let radio = tool_ctx.radio_log.lock().clone();
            let extras = crate::eval_scenario::ScoreExtras { traffic: scripted_traffic.as_deref(), radio: &radio };
            let checks =
                crate::eval_scenario::score(&s.checks, &samples, &sent, &mut resolver, &mut frame_for, cfg.dt, &extras);
            for c in &checks {
                bus.push_log_kind(
                    LogKind::System,
                    format!("check {}: {} — {}", if c.pass { "PASS" } else { "FAIL" }, c.check, c.detail),
                );
            }
            let runways: Vec<(String, Vec2, Vec2)> = apt
                .as_ref()
                .map(|a| {
                    a.runways
                        .iter()
                        .map(|r| (r.ident.clone(), geodetic_offset_ft(r.lat, r.lon, georef), geodetic_offset_ft(r.far_lat, r.far_lon, georef)))
                        .collect()
                })
                .unwrap_or_default();
            let title = format!("{} — {}", s.id, s.description.clone().unwrap_or_default());
            let svg = match crate::eval_scenario::write_ground_svg(
                &ground_svg_path,
                &title,
                &samples,
                &sent,
                &crate::plan::gazetteer::GAZETTEER,
                &mut resolver,
                &runways,
                scripted_traffic.as_deref(),
            ) {
                Ok(()) => Some(ground_svg_path.clone()),
                Err(e) => {
                    bus.push_log_kind(LogKind::Error, format!("ground svg: {e:#}"));
                    None
                }
            };
            (checks, svg)
        }
        None => (Vec::new(), None),
    };

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
            "eval finished: {:?} success={} turns={} tools={}/{} (failed/total) tokens(in={} cached={} written={} out={}) sim={:.1}s wall={:.1}s",
            outcome.source,
            outcome.success,
            turn_count,
            tool_stats.failed,
            tool_stats.total,
            token_totals.total_input_tokens,
            token_totals.total_cached_tokens,
            token_totals.total_cache_creation_tokens,
            token_totals.total_output_tokens,
            sim_duration_s,
            wall_duration_s,
        ),
    );
    if !tool_stats.per_tool.is_empty() {
        bus.push_log_kind(
            LogKind::System,
            format!("tool breakdown: {}", tool_stats.format_breakdown()),
        );
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
        "total_cache_creation_tokens": token_totals.total_cache_creation_tokens,
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
        "scenario": cfg.scenario.as_ref().map(|s| serde_json::json!({"id": s.id, "items": s.items})),
        "checks": checks,
        "checks_passed": checks.iter().filter(|c| c.pass).count(),
        "checks_total": checks.len(),
    });
    std::fs::write(&summary_path, serde_json::to_string_pretty(&summary)?)
        .with_context(|| format!("writing {}", summary_path.display()))?;

    bus.close();

    Ok(EvalResult {
        outcome,
        turn_count,
        total_input_tokens: token_totals.total_input_tokens,
        total_cached_tokens: token_totals.total_cached_tokens,
        total_cache_creation_tokens: token_totals.total_cache_creation_tokens,
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
        track_csv: track_recorder.as_ref().map(|_| track_csv_path),
        track_kml: track_kml_written.then_some(track_kml_path),
        checks,
        ground_svg,
    })
}
