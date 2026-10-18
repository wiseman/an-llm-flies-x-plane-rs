//! Tool handlers + JSON schemas for the LLM.
//!
//! Each tool is dispatched from a parsed `function_call` item returned by the
//! Responses API. Handlers mutate `PilotCore` state (engaging/disengaging
//! profiles, arming pattern-fly triggers) or talk to the X-Plane bridge
//! (radio, parking brake, flaps). The SQL query tool uses DuckDB with the
//! spatial extension loaded so the agent can do real geospatial lookups.

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use anyhow::{anyhow, Result};
use crossbeam_channel::Sender;
use parking_lot::Mutex as PLMutex;
use serde_json::{json, Map, Value};

use crate::bus::SimBus;
use crate::config::ConfigBundle;
use crate::eval_runner::{try_send_outcome, MissionOutcome};
use crate::llm::backend::ToolDef;
use crate::core::mission_manager::{
    ActiveClearance, ClearanceSource, MissionGoal, PilotCore, StatusSnapshot,
};
use crate::core::landing_rollout::ArrivalPlan;
use crate::core::profiles::{GuidanceProfile, RunwayExitPlan, TakeoffRun, TaxiProfile};
use crate::data::airspace_query::{self, AircraftSnapshot, AirspaceSource};
use crate::data::parquet::{
    AIRPORTS_FILE, AIRSPACES_FILE, COMMS_FILE, PARKING_SPOTS_FILE, RUNWAYS_FILE, TAXI_EDGES_FILE,
    TAXI_NODES_FILE,
};
use crate::guidance::runway_geometry::RunwayFrame;
use crate::guidance::taxi_route::{self, PlannedTaxi, TaxiDestination, TaxiGoal, TaxiStart};
use crate::sim::datarefs::{
    COM1_FREQUENCY_HZ_833, COM2_FREQUENCY_HZ_833, FLAP_HANDLE_REQUEST_RATIO, LATITUDE_DEG,
    LONGITUDE_DEG, PARKING_BRAKE_RATIO,
};
use crate::sim::xplane_bridge::{geodetic_offset_ft, GeoReference};
use crate::types::{
    heading_to_vector, FlightPhase, Runway, StraightLeg, TrafficSide, Vec2, M_TO_FT, NM_TO_FT,
};

pub const SQL_QUERY_MAX_ROWS: usize = 50;

/// Prefix every tool handler uses for error returns. Checked at the
/// dispatch boundary (one place) to derive `ToolResult.ok`.
pub const TOOL_ERROR_PREFIX: &str = "error:";

/// Error from the tools that act on the pattern being flown.
const NO_PATTERN: &str = "error: no pattern is being flown; fly a plan with join(p) first";

/// Structured return of `dispatch_tool`. `output` is the exact string
/// handed back to the LLM; `ok` is the structural success flag the
/// harness and counters use without substring-matching.
#[derive(Debug, Clone)]
pub struct ToolResult {
    pub output: String,
    pub ok: bool,
}

impl ToolResult {
    pub fn from_output(output: String) -> Self {
        let ok = !output.starts_with(TOOL_ERROR_PREFIX);
        Self { output, ok }
    }
}

/// Per-tool + aggregate counters. Handed to `ToolContext` via an
/// `Arc` so the conversation loop and the eval harness share it.
#[derive(Debug, Default)]
pub struct ToolCounters {
    total: AtomicUsize,
    failed: AtomicUsize,
    per_tool: PLMutex<HashMap<String, ToolStat>>,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ToolStat {
    pub calls: usize,
    pub failures: usize,
}

#[derive(Debug, Default, Clone)]
pub struct ToolCountersSnapshot {
    pub total: usize,
    pub failed: usize,
    pub per_tool: Vec<(String, ToolStat)>,
}

impl ToolCountersSnapshot {
    /// Render `per_tool` as a single-line `name=failures/calls ...`
    /// string for the CLI + bus log. Empty string when no tools were
    /// called, so callers can skip the prefix.
    pub fn format_breakdown(&self) -> String {
        self.per_tool
            .iter()
            .map(|(name, s)| format!("{}={}/{}", name, s.failures, s.calls))
            .collect::<Vec<_>>()
            .join(" ")
    }
}

impl ToolCounters {
    pub fn record(&self, name: &str, ok: bool) {
        self.total.fetch_add(1, Ordering::Relaxed);
        if !ok {
            self.failed.fetch_add(1, Ordering::Relaxed);
        }
        let mut map = self.per_tool.lock();
        let entry = map.entry(name.to_string()).or_default();
        entry.calls += 1;
        if !ok {
            entry.failures += 1;
        }
    }

    pub fn snapshot(&self) -> ToolCountersSnapshot {
        let per_tool_map = self.per_tool.lock().clone();
        let mut per_tool: Vec<(String, ToolStat)> = per_tool_map.into_iter().collect();
        per_tool.sort_by(|a, b| b.1.calls.cmp(&a.1.calls).then_with(|| a.0.cmp(&b.0)));
        ToolCountersSnapshot {
            total: self.total.load(Ordering::Relaxed),
            failed: self.failed.load(Ordering::Relaxed),
            per_tool,
        }
    }
}

/// Write-capable view over the bridge. The real `XPlaneWebBridge` is one
/// implementation; tests use a fake that records writes. The abstraction
/// keeps `tools.rs` testable without needing a live X-Plane.
pub trait ToolBridge: Send + Sync {
    fn georef(&self) -> GeoReference;
    fn get_dataref_value(&self, name: &str) -> Option<f64>;
    fn write_dataref_values(&self, updates: &[(String, f64)]) -> Result<()>;
    /// Aircraft tail number / registration (e.g. "N172SP"). Default
    /// `None` covers the simple/eval bridge and test fakes; the live
    /// X-Plane bridge fetches it once at startup from
    /// `sim/aircraft/view/acf_tailnum`.
    fn aircraft_tail_number(&self) -> Option<&str> { None }
}

pub struct ToolContext {
    pub pilot: Arc<PLMutex<PilotCore>>,
    pub bridge: Option<Arc<dyn ToolBridge>>,
    pub config: Arc<PLMutex<ConfigBundle>>,
    pub recent_broadcasts: Arc<PLMutex<Vec<String>>>,
    /// Every transmission with its sim time, for eval checks.
    pub radio_log: Arc<PLMutex<Vec<(f64, String)>>>,
    /// Directory holding the three apt.dat-derived parquet files
    /// (airports / runways / comms). Populated on startup by
    /// `data::parquet::resolve`; `None` only when no apt.dat was locatable.
    pub apt_dat_cache_dir: Option<PathBuf>,
    pub bus: Option<SimBus>,
    pub runway_conn: Arc<Mutex<Option<duckdb::Connection>>>,
    /// Handle into the live heartbeat pump. Only `sleep(suppress_idle_heartbeat_s=...)`
    /// needs it today. `None` when heartbeats are disabled or in tests.
    pub heartbeat_pump: Option<Arc<crate::live_runner::HeartbeatPump>>,
    /// Eval harness hook: when set, the `mission_complete` tool is
    /// registered in the schema list + dispatch table and, when called,
    /// ships a `MissionOutcome` down this channel. `None` in the live
    /// X-Plane runtime — the tool is then invisible to the LLM.
    pub mission_complete_tx: Option<Sender<MissionOutcome>>,
    /// Eval harness hook: incremented once per `create_response` call in
    /// `run_conversation_loop`. `None` outside the eval harness — the
    /// conversation loop is expected to no-op when absent.
    pub turn_counter: Option<Arc<AtomicUsize>>,
    /// Per-tool call / failure counters, incremented by the conversation
    /// loop when a tool returns. `None` outside the eval harness.
    pub tool_counters: Option<Arc<ToolCounters>>,
    /// Taxi graphs by airport, loaded once: apt.dat doesn't change while
    /// running, and one arrival plan can need the same graph several times.
    pub taxi_graphs: Arc<PLMutex<HashMap<String, Arc<taxi_route::TaxiGraph>>>>,
    /// Eval harness: a simulated controller that hears `broadcast_on_radio`
    /// calls and answers them. `None` everywhere else.
    pub sim_atc: Option<Arc<crate::llm::sim_atc::SimAtc>>,
}

impl ToolContext {
    pub fn new(
        pilot: Arc<PLMutex<PilotCore>>,
        config: ConfigBundle,
    ) -> Self {
        Self {
            pilot,
            bridge: None,
            config: Arc::new(PLMutex::new(config)),
            recent_broadcasts: Arc::new(PLMutex::new(Vec::new())),
            radio_log: Arc::new(PLMutex::new(Vec::new())),
            apt_dat_cache_dir: None,
            bus: None,
            runway_conn: Arc::new(Mutex::new(None)),
            heartbeat_pump: None,
            mission_complete_tx: None,
            turn_counter: None,
            tool_counters: None,
            taxi_graphs: Default::default(),
            sim_atc: None,
        }
    }
}

/// `airport`'s taxi graph, from `ToolContext::taxi_graphs` after the
/// first load.
fn taxi_graph(
    ctx: &ToolContext,
    conn: &duckdb::Connection,
    airport: &str,
) -> anyhow::Result<Arc<taxi_route::TaxiGraph>> {
    let key = airport.to_ascii_uppercase();
    if let Some(graph) = ctx.taxi_graphs.lock().get(&key) {
        return Ok(graph.clone());
    }
    let graph = Arc::new(taxi_route::load_graph(conn, airport)?);
    ctx.taxi_graphs.lock().insert(key, graph.clone());
    Ok(graph)
}

// ---------- helpers ----------

fn format_displaced(displaced: &[String]) -> String {
    if displaced.is_empty() {
        String::new()
    } else {
        format!(" (displaced: {})", displaced.join(", "))
    }
}

/// Engage `profile`. `Ok` carries the " (displaced: …)" suffix for the
/// tool result, `Err` the refusal as a tool error.
fn engage(ctx: &ToolContext, profile: Box<dyn GuidanceProfile>) -> Result<String, String> {
    let displaced = ctx
        .pilot
        .lock()
        .engage_profile(profile)
        .map_err(|e| format!("error: {e}"))?;
    Ok(format_displaced(&displaced))
}

/// `engage` for a profile flown against a newly looked-up runway
/// (`runway_config`): the runway becomes the pilot's reference only if
/// the engage goes through.
fn engage_on_runway(
    ctx: &ToolContext,
    config: ConfigBundle,
    profile: Box<dyn GuidanceProfile>,
) -> Result<String, String> {
    let airport = config.airport.clone();
    let displaced = ctx
        .pilot
        .lock()
        .engage_profiles_with_config(config, vec![profile])
        .map_err(|e| format!("error: {e}"))?;
    ctx.config.lock().airport = airport;
    Ok(format_displaced(&displaced))
}

/// Read the three fuel datarefs (`m_fuel_total`, `acf_m_fuel_tot`, engine-0
/// fuel flow) and shape them into the `"fuel"` object embedded in the
/// status payload. Returns `None` when no bridge is attached or when
/// `m_fuel_total` hasn't been reported yet — both of which happen in
/// offline / test contexts. Flow is converted from kg/s to kg/hr since
/// that reads more naturally to the LLM and matches how pilots think
/// about burn rates.
fn build_fuel_payload(bridge: Option<&dyn ToolBridge>) -> Option<Value> {
    let b = bridge?;
    let kg = b.get_dataref_value(crate::sim::datarefs::FUEL_KG.name)?;
    let mut obj = json!({ "kg": round_f64(kg, 2) });
    if let Some(capacity) = b.get_dataref_value(crate::sim::datarefs::FUEL_CAPACITY_KG.name) {
        if capacity > 0.0 {
            obj.as_object_mut()
                .unwrap()
                .insert("capacity_kg".into(), json!(round_f64(capacity, 2)));
            obj.as_object_mut().unwrap().insert(
                "fraction".into(),
                json!(round_f64((kg / capacity).clamp(0.0, 1.0), 3)),
            );
        }
    }
    if let Some(flow_sec) =
        b.get_dataref_value(crate::sim::datarefs::FUEL_FLOW_KG_SEC.name)
    {
        obj.as_object_mut().unwrap().insert(
            "flow_kg_hr".into(),
            json!(round_f64(flow_sec * 3600.0, 2)),
        );
    }
    Some(obj)
}

pub fn build_status_payload(
    snapshot: Option<&StatusSnapshot>,
    bridge: Option<&dyn ToolBridge>,
    airspace: Option<&AirspaceSource>,
) -> Value {
    let Some(snapshot) = snapshot else {
        return json!({ "status": "uninitialized" });
    };
    let state = &snapshot.state;
    let (lat_deg, lon_deg, has_crashed) = match bridge {
        Some(b) => (
            b.get_dataref_value(LATITUDE_DEG.name),
            b.get_dataref_value(LONGITUDE_DEG.name),
            b.get_dataref_value(crate::sim::datarefs::HAS_CRASHED.name)
                .map(|v| v >= 0.5),
        ),
        None => (None, None, None),
    };
    let mut obj = json!({
        "t_sim": round_f64(state.t_sim, 2),
        "active_profiles": snapshot.active_profiles.clone(),
        "phase": snapshot.phase.map(|p| p.value().to_string()),
        "lat_deg": lat_deg.map(|v| round_f64(v, 6)),
        "lon_deg": lon_deg.map(|v| round_f64(v, 6)),
        "alt_msl_ft": round_f64(state.alt_msl_ft, 1),
        // Height above field elevation, the same number the TUI and tool
        // errors use; plus the radio altimeter when the sim has one, since
        // runway ends can sit 10-20 ft off the field's published elevation.
        "alt_agl_ft": round_f64(state.alt_agl_ft, 1),
        "radio_alt_ft": state
            .alt_radio_agl_ft
            .filter(|_| !state.on_ground)
            .map(|v| round_f64(v, 1)),
        "indicated_airspeed_knots": round_f64(state.ias_kt, 1),
        "groundspeed_kts": round_f64(state.gs_kt, 1),
        "vs_fpm": round_f64(state.vs_fpm, 1),
        "heading_deg": round_f64(state.heading_deg, 1),
        "track_deg": round_f64(state.track_deg, 1),
        "pitch_deg": round_f64(state.pitch_deg, 1),
        "roll_deg": round_f64(state.roll_deg, 1),
        "on_ground": state.on_ground,
        "throttle_pos": round_f64(state.throttle_pos, 2),
        "flap_index": state.flap_index,
        "gear_down": state.gear_down,
    });
    if let Some(crashed) = has_crashed {
        obj.as_object_mut()
            .unwrap()
            .insert("has_crashed".into(), Value::Bool(crashed));
    }
    // What each active profile is doing ("taxi": "→ FBO Parking via D→A
    // · leg 2/4 …") and what the phase machine is waiting for, so the LLM
    // can tell e.g. an arrival taxi from one it still needs to start.
    let details: Map<String, Value> = snapshot
        .active_profiles
        .iter()
        .zip(&snapshot.profile_mode_line_suffixes)
        .filter_map(|(name, suffix)| suffix.as_ref().map(|s| (name.clone(), json!(s))))
        .collect();
    if !details.is_empty() {
        obj.as_object_mut()
            .unwrap()
            .insert("profile_details".into(), Value::Object(details));
    }
    if let Some(hint) = &snapshot.transition_hint {
        obj.as_object_mut().unwrap().insert(
            "next".into(),
            json!({ "phase": hint.next_phase.value(), "when": hint.condition_text }),
        );
    }
    if let Some(b) = bridge {
        if let Some(rpm) = b.get_dataref_value(crate::sim::datarefs::ENGINE_RPM.name) {
            obj.as_object_mut()
                .unwrap()
                .insert("engine_rpm".into(), json!(rpm.round() as i64));
        }
        if let Some(pb) = b.get_dataref_value(PARKING_BRAKE_RATIO.name) {
            let map = obj.as_object_mut().unwrap();
            map.insert("parking_brake_set".into(), Value::Bool(pb >= 0.5));
            map.insert("parking_brake_ratio".into(), json!(round_f64(pb, 2)));
        }
    }
    if let Some(fuel) = build_fuel_payload(bridge) {
        obj.as_object_mut().unwrap().insert("fuel".into(), fuel);
    }
    if let Some(plan) = &snapshot.plan_status {
        obj.as_object_mut().unwrap().insert("plan".into(), plan.clone());
    }
    if !snapshot.traffic.is_empty() {
        obj.as_object_mut().unwrap().insert("traffic".into(), json!(snapshot.traffic));
    }

    // Airspace awareness is gated on (a) a live lat/lon (otherwise there's
    // no global position to query against) and (b) being airborne — on the
    // ground or during takeoff-roll / rollout / taxi the airspace lines
    // would be pure noise.
    if let (Some(src), Some(lat), Some(lon)) = (airspace, lat_deg, lon_deg) {
        if airspace_emission_enabled(state.on_ground, snapshot.phase) {
            if let Some(v) = query_airspace_value(
                src,
                AircraftSnapshot {
                    lat_deg: lat,
                    lon_deg: lon,
                    alt_msl_ft: state.alt_msl_ft,
                    track_deg: state.track_deg,
                    gs_kt: state.gs_kt,
                    vs_fpm: state.vs_fpm,
                },
            ) {
                if !v.is_null() {
                    obj.as_object_mut()
                        .unwrap()
                        .insert("airspace".into(), v);
                }
            }
        }
    }
    obj
}

fn airspace_emission_enabled(on_ground: bool, phase: Option<FlightPhase>) -> bool {
    if on_ground {
        return false;
    }
    !matches!(
        phase,
        Some(FlightPhase::Preflight)
            | Some(FlightPhase::TakeoffRoll)
            | Some(FlightPhase::Rollout)
            | Some(FlightPhase::TaxiClear)
    )
}

/// Open (or reuse) the airspace DuckDB connection, run the query, and
/// format the result. Returns `Value::Null` when the query has nothing to
/// report or the connection can't be opened (missing parquet, etc.) so
/// the caller can omit the `"airspace"` key.
fn query_airspace_value(src: &AirspaceSource, snap: AircraftSnapshot) -> Option<Value> {
    let mut guard = src.conn.lock().ok()?;
    if guard.is_none() {
        let airports = src.cache_dir.join(AIRPORTS_FILE);
        let runways = src.cache_dir.join(RUNWAYS_FILE);
        let comms = src.cache_dir.join(COMMS_FILE);
        if !airports.exists() || !runways.exists() || !comms.exists() {
            return None;
        }
        match open_apt_dat_parquet(&airports, &runways, &comms) {
            Ok(c) => *guard = Some(c),
            Err(_) => return None,
        }
    }
    let conn = guard.as_ref().unwrap();
    let states = airspace_query::query_airspace_states(conn, snap).ok()?;
    Some(airspace_query::format_states_json(&states))
}

fn round_f64(v: f64, places: u32) -> f64 {
    let m = 10f64.powi(places as i32);
    (v * m).round() / m
}

fn parking_brake_ratio(ctx: &ToolContext) -> Option<f64> {
    ctx.bridge.as_ref().and_then(|b| b.get_dataref_value(PARKING_BRAKE_RATIO.name))
}

/// Deduplicate the `plan.runway_crossings` list (preserving first-seen
/// order) and join with `"; "`. Returns `None` when nothing to report so
/// the caller can skip the surrounding formatting.
fn format_unique_crossings(crossings: &[String]) -> Option<String> {
    if crossings.is_empty() {
        return None;
    }
    let mut seen = std::collections::HashSet::new();
    let unique: Vec<&str> = crossings
        .iter()
        .filter(|z| seen.insert(z.as_str()))
        .map(String::as_str)
        .collect();
    Some(unique.join("; "))
}

/// Set the parking brake now (the sim may be paused while the LLM
/// thinks) and tell the pilot, which owns the setting.
fn write_parking_brake(ctx: &ToolContext, bridge: &Arc<dyn ToolBridge>, ratio: f64) -> Result<(), String> {
    bridge
        .write_dataref_values(&[(PARKING_BRAKE_RATIO.name.to_string(), ratio)])
        .map_err(|e| e.to_string())?;
    ctx.pilot.lock().parking_brake.note_written(ratio);
    Ok(())
}

/// Release the parking brake if set. Returns `Some(prior_ratio)` when a
/// release actually happened, so callers can note it in their summary;
/// returns `None` when nothing needed doing.
fn release_parking_brake_if_set(
    ctx: &ToolContext,
    bridge: &Arc<dyn ToolBridge>,
) -> Result<Option<f64>, String> {
    match parking_brake_ratio(ctx) {
        Some(v) if v >= 0.5 => write_parking_brake(ctx, bridge, 0.0)
            .map(|_| Some(v))
            .map_err(|e| format!("releasing parking brake: {}", e)),
        _ => Ok(None),
    }
}

// ---------- individual tool handlers ----------

pub fn tool_get_status(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    let mut pilot = ctx.pilot.lock();
    let mut snap = pilot.latest_snapshot.clone();
    // The snapshot is from the last control tick, and the sim may be
    // paused while the LLM works: show the profile stack and plan as they
    // are now, so a profile engaged this turn doesn't look missing.
    let live_plan = pilot
        .profile_mut::<crate::core::plan_profile::PlanProfile>()
        .map(|p| p.status_json());
    if let (Some(s), Some(plan)) = (snap.as_mut(), live_plan) {
        s.active_profiles = pilot.list_profile_names();
        s.plan_status = Some(plan);
    }
    drop(pilot);
    let bridge_ref: Option<&dyn ToolBridge> = ctx.bridge.as_deref();
    let airspace = airspace_source(ctx);
    serde_json::to_string(&build_status_payload(
        snap.as_ref(),
        bridge_ref,
        airspace.as_ref(),
    ))
    .unwrap()
}

/// Derive an `AirspaceSource` from a `ToolContext`, sharing its
/// `runway_conn` so the heartbeat and the LLM's tool calls reuse a single
/// DuckDB connection (DuckDB connections are not safe for concurrent use
/// across threads anyway, so sharing via mutex is correct).
pub fn airspace_source(ctx: &ToolContext) -> Option<AirspaceSource> {
    let dir = ctx.apt_dat_cache_dir.as_ref()?;
    if !dir.join(AIRSPACES_FILE).exists() {
        return None;
    }
    Some(AirspaceSource {
        conn: ctx.runway_conn.clone(),
        cache_dir: dir.clone(),
    })
}

/// Cap on `suppress_idle_heartbeat_s` to guard against the LLM handing us
/// a pathological value (e.g. an hour+ of silence during which a real issue
/// could go unreported). State-change heartbeats still fire regardless.
const SLEEP_MAX_SUPPRESS_IDLE_S: f64 = 600.0;

/// Epsilon below which we treat the aircraft as stationary. gs_kt comes
/// from the dynamics estimator and can carry a sliver of sensor noise
/// even when the aircraft is parked, so the cutoff is a small positive
/// number rather than zero.
const SLEEP_STATIONARY_GS_KT: f64 = 0.5;

pub fn tool_sleep(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    // Refuse to sleep while the aircraft is moving AND no non-idle
    // profile is flying it. `sleep` yields the turn and relies on the
    // active profiles to keep flying — with only `idle_*` profiles
    // engaged, dozing off just lets the aircraft coast uncontrolled.
    // Once a real profile (a plan, taxi, park, takeoff)
    // owns the aircraft, sleep is the right thing to do even at speed.
    let snapshot = ctx.pilot.lock().latest_snapshot.clone();
    if let Some(snap) = snapshot {
        let only_idle = snap
            .active_profiles
            .iter()
            .all(|n| n.starts_with("idle_"));
        if only_idle && snap.state.gs_kt > SLEEP_STATIONARY_GS_KT {
            return format!(
                "error: aircraft is moving at {:.1} kt groundspeed with only idle profiles active — sleep is only allowed when a control profile is flying the aircraft, or when stationary. Fly a plan (fly) or engage a ground profile (engage_taxi / engage_park), or bring the aircraft to a stop before sleeping.",
                snap.state.gs_kt,
            );
        }
    }

    let requested = args.get("suppress_idle_heartbeat_s").and_then(|v| v.as_f64());
    if let Some(secs) = requested {
        let clamped = secs.clamp(0.0, SLEEP_MAX_SUPPRESS_IDLE_S);
        if let Some(pump) = &ctx.heartbeat_pump {
            pump.suppress_idle_heartbeat_for_seconds(clamped);
        }
        if clamped > 0.0 {
            return format!(
                "sleeping; idle heartbeat suppressed for {}s (state-change heartbeats still fire)",
                clamped.round() as i64,
            );
        }
    }
    "sleeping; waiting for next external message".to_string()
}

fn arg_f64(args: &Map<String, Value>, key: &str) -> Result<f64> {
    args.get(key)
        .and_then(|v| v.as_f64())
        .ok_or_else(|| anyhow!("missing or non-numeric argument {}", key))
}
fn arg_str<'a>(args: &'a Map<String, Value>, key: &str) -> Result<&'a str> {
    args.get(key)
        .and_then(|v| v.as_str())
        .ok_or_else(|| anyhow!("missing or non-string argument {}", key))
}
fn arg_bool(args: &Map<String, Value>, key: &str) -> Result<bool> {
    args.get(key)
        .and_then(|v| v.as_bool())
        .ok_or_else(|| anyhow!("missing or non-bool argument {}", key))
}
fn arg_i64(args: &Map<String, Value>, key: &str) -> Result<i64> {
    args.get(key)
        .and_then(|v| v.as_i64())
        .ok_or_else(|| anyhow!("missing or non-integer argument {}", key))
}
fn arg_string_array(args: &Map<String, Value>, key: &str) -> Vec<String> {
    args.get(key)
        .and_then(|v| v.as_array())
        .map(|a| a.iter().filter_map(|v| v.as_str().map(str::to_string)).collect())
        .unwrap_or_default()
}

fn radio_dataref(radio: &str) -> Result<&'static str, String> {
    match radio {
        "com1" => Ok(COM1_FREQUENCY_HZ_833.name),
        "com2" => Ok(COM2_FREQUENCY_HZ_833.name),
        other => Err(format!("unknown radio {:?} (expected com1, com2)", other)),
    }
}

pub fn ensure_runway_conn(ctx: &ToolContext) -> Result<()> {
    let Some(dir) = &ctx.apt_dat_cache_dir else {
        return Err(anyhow!(
            "apt.dat parquet cache is not configured (set --apt-dat-path or place apt.dat at the standard X-Plane 12 location)"
        ));
    };
    let airports = dir.join(AIRPORTS_FILE);
    let runways = dir.join(RUNWAYS_FILE);
    let comms = dir.join(COMMS_FILE);
    let taxi_nodes = dir.join(TAXI_NODES_FILE);
    let taxi_edges = dir.join(TAXI_EDGES_FILE);
    let parking_spots = dir.join(PARKING_SPOTS_FILE);
    for p in [&airports, &runways, &comms, &taxi_nodes, &taxi_edges, &parking_spots] {
        if !p.exists() {
            return Err(anyhow!("parquet file missing at {}", p.display()));
        }
    }
    let mut guard = ctx.runway_conn.lock().unwrap();
    if guard.is_none() {
        *guard = Some(open_apt_dat_parquet(&airports, &runways, &comms)?);
    }
    Ok(())
}

pub fn open_apt_dat_parquet(
    airports: &std::path::Path,
    runways: &std::path::Path,
    comms: &std::path::Path,
) -> Result<duckdb::Connection> {
    let conn = duckdb::Connection::open_in_memory()?;
    // Spatial extension is required — the parquet files carry GEOMETRY
    // columns (WKB) that only decode when spatial is loaded.
    conn.execute_batch("INSTALL spatial; LOAD spatial;")?;
    // Computed GEOMETRY columns live in the view, not in the parquet — the
    // current duckdb-rs + DuckDB spatial pairing panics on `SELECT *` when a
    // GEOMETRY column comes off disk. Running ST_Point at view time avoids
    // that, costs nothing perf-wise (38k rows), and keeps the schema the
    // LLM queries (`arp`, `centerline`, `le_threshold`, `he_threshold`).
    conn.execute_batch(&format!(
        "CREATE VIEW airports AS
           SELECT ident, name, elevation_ft, icao_code, iata_code, faa_code,
                  latitude_deg, longitude_deg,
                  CASE WHEN latitude_deg IS NULL OR longitude_deg IS NULL
                       THEN NULL
                       ELSE ST_Point(longitude_deg, latitude_deg)
                  END AS arp
           FROM read_parquet('{airports}');
         CREATE VIEW runways AS
           SELECT id, airport_ident, length_ft, width_ft, surface, lighted, closed,
                  le_ident, le_latitude_deg, le_longitude_deg, le_elevation_ft,
                  le_heading_degT, le_displaced_threshold_ft,
                  he_ident, he_latitude_deg, he_longitude_deg, he_elevation_ft,
                  he_heading_degT, he_displaced_threshold_ft,
                  ST_MakeLine(
                      ST_Point(le_longitude_deg, le_latitude_deg),
                      ST_Point(he_longitude_deg, he_latitude_deg)
                  ) AS centerline,
                  ST_Point(le_longitude_deg, le_latitude_deg) AS le_threshold,
                  ST_Point(he_longitude_deg, he_latitude_deg) AS he_threshold
           FROM read_parquet('{runways}');
         CREATE VIEW comms AS
           SELECT * FROM read_parquet('{comms}');
         CREATE VIEW taxi_nodes AS
           SELECT airport_ident, node_id, latitude_deg, longitude_deg, usage,
                  ST_Point(longitude_deg, latitude_deg) AS point
           FROM read_parquet('{taxi_nodes}');
         CREATE VIEW taxi_edges AS
           SELECT * FROM read_parquet('{taxi_edges}');
         CREATE VIEW parking_spots AS
           SELECT airport_ident, name, kind, categories,
                  heading_true_deg, latitude_deg, longitude_deg,
                  icao_category, operation_type, airlines,
                  ST_Point(longitude_deg, latitude_deg) AS location
           FROM read_parquet('{parking_spots}');",
        airports = airports.display(),
        runways = runways.display(),
        comms = comms.display(),
        taxi_nodes = dir_join(airports, TAXI_NODES_FILE).display(),
        taxi_edges = dir_join(airports, TAXI_EDGES_FILE).display(),
        parking_spots = dir_join(airports, PARKING_SPOTS_FILE).display(),
    ))?;
    // The airspaces parquet is opt-in (built only when airspace.txt was
    // located). The view is registered only when the file is present so
    // that cache directories built before airspace support still work.
    let airspaces = dir_join(airports, AIRSPACES_FILE);
    if airspaces.exists() {
        conn.execute_batch(&format!(
            "CREATE VIEW airspaces AS
               SELECT id, class, name, bottom_ft_msl, top_ft_msl,
                      bottom_is_gnd, top_is_gnd,
                      min_lat, max_lat, min_lon, max_lon, polygon_wkt,
                      ST_GeomFromText(polygon_wkt) AS footprint
               FROM read_parquet('{airspaces}');",
            airspaces = airspaces.display(),
        ))?;
    }
    Ok(conn)
}

fn dir_join(sibling: &std::path::Path, name: &str) -> PathBuf {
    sibling.parent().unwrap_or_else(|| std::path::Path::new("")).join(name)
}

/// The pilot's config with `airport_ident`'s runway `runway_ident` (from
/// apt.dat) as its reference runway, flown with `side` traffic.
fn runway_config(ctx: &ToolContext, airport_ident: &str, runway_ident: &str, side: &str) -> Result<ConfigBundle> {
    let bridge = ctx
        .bridge
        .as_ref()
        .ok_or_else(|| anyhow!("aircraft systems bridge unavailable; cannot compute world-frame threshold"))?;
    ensure_runway_conn(ctx)?;
    let end = {
        let guard = ctx.runway_conn.lock().unwrap();
        query_runway_end(guard.as_ref().unwrap(), airport_ident, runway_ident)?
    };
    let traffic_side = TrafficSide::from_str(side)
        .ok_or_else(|| anyhow!("invalid side {:?}; expected 'left' or 'right'", side))?;
    // Runway frame anchors at the pavement end (apt.dat row-100 lat/lon)
    // so x=0 means "start of usable takeoff pavement" and x=length is
    // "far end of pavement". The displaced landing threshold sits
    // `displaced_ft` inside from x=0 — `touchdown_runway_x_ft()` adds
    // that offset when computing the landing aim point. Do not bake the
    // displacement into `threshold_ft`: takeoff position checks then
    // think the aircraft is off the runway at the pavement end.
    let pavement_end_ft =
        geodetic_offset_ft(end.threshold_lat_deg, end.threshold_lon_deg, bridge.georef());
    let mut cfg = ctx.config.lock().clone();
    cfg.airport.airport = Some(airport_ident.to_string());
    cfg.airport.field_elevation_ft = end.elevation_ft;
    cfg.airport.runway = Runway {
        id: Some(runway_ident.to_string()),
        threshold_ft: pavement_end_ft,
        course_deg: end.course_deg,
        length_ft: end.length_ft,
        // Aim 600 ft past the landing threshold, as a plan's
        // pattern does.
        touchdown_zone_ft: crate::plan::compile::TOUCHDOWN_ZONE_FT,
        displaced_threshold_ft: end.displaced_threshold_ft,
        width_ft: end.width_ft,
        traffic_side,
    };
    Ok(cfg)
}

/// One end of a runway, from the `runways` table.
#[derive(Debug, Clone)]
pub struct RunwayEnd {
    /// The end's pavement-end coordinates (apt.dat row 100).
    pub threshold_lat_deg: f64,
    pub threshold_lon_deg: f64,
    pub course_deg: f64,
    pub length_ft: f64,
    pub width_ft: f64,
    pub elevation_ft: f64,
    pub displaced_threshold_ft: f64,
}

/// Look up `runway_ident` at `airport_ident` (either end) in `conn`'s
/// `runways` table.
pub fn query_runway_end(
    conn: &duckdb::Connection,
    airport_ident: &str,
    runway_ident: &str,
) -> Result<RunwayEnd> {
    let mut stmt = conn.prepare(
        "SELECT le_ident, he_ident, \
                le_latitude_deg, le_longitude_deg, le_heading_degT, le_elevation_ft, le_displaced_threshold_ft, \
                he_latitude_deg, he_longitude_deg, he_heading_degT, he_elevation_ft, he_displaced_threshold_ft, \
                length_ft, width_ft \
         FROM runways \
         WHERE airport_ident = ? AND (le_ident = ? OR he_ident = ?) AND closed = 0 \
         LIMIT 1",
    )?;
    let mut rows = stmt.query([airport_ident, runway_ident, runway_ident])?;
    let row = rows.next()?.ok_or_else(|| {
        anyhow!("runway {:?} at {:?} not found in database", runway_ident, airport_ident)
    })?;
    let le_ident: Option<String> = row.get(0)?;
    let he_ident: Option<String> = row.get(1)?;
    let first = if le_ident.as_deref() == Some(runway_ident) {
        2
    } else if he_ident.as_deref() == Some(runway_ident) {
        7
    } else {
        return Err(anyhow!(
            "runway lookup returned {:?}/{:?} for query {:?}; internal mismatch",
            le_ident,
            he_ident,
            runway_ident
        ));
    };
    let lat: Option<f64> = row.get(first)?;
    let lon: Option<f64> = row.get(first + 1)?;
    let course: Option<f64> = row.get(first + 2)?;
    let elevation: Option<f64> = row.get(first + 3)?;
    let displaced: Option<f64> = row.get(first + 4)?;
    let length_ft: Option<f64> = row.get(12)?;
    let width_ft: Option<f64> = row.get(13)?;
    let (Some(lat), Some(lon), Some(course)) = (lat, lon, course) else {
        return Err(anyhow!(
            "runway {}/{} has no threshold coordinates or course in the database",
            airport_ident,
            runway_ident
        ));
    };
    Ok(RunwayEnd {
        threshold_lat_deg: lat,
        threshold_lon_deg: lon,
        course_deg: course,
        length_ft: length_ft.unwrap_or(5000.0),
        width_ft: width_ft.unwrap_or(crate::config::DEFAULT_RUNWAY_WIDTH_FT),
        elevation_ft: elevation.unwrap_or(0.0),
        displaced_threshold_ft: displaced.unwrap_or(0.0),
    })
}

/// The tool config with `runway` at `airport_ident` as the reference
/// runway, to build profiles against before `engage_on_runway` installs
/// it in the pilot.
/// Airport and runway data for plan resolution, from the apt.dat views.
pub struct DuckAirports<'a> {
    pub ctx: &'a ToolContext,
}

impl crate::plan::resolve::AirportSource for DuckAirports<'_> {
    fn airport(&self, ident: &str) -> Result<crate::plan::resolve::AirportData> {
        use crate::plan::resolve::{AirportData, RunwayInfo};
        ensure_runway_conn(self.ctx)?;
        let guard = self.ctx.runway_conn.lock().unwrap();
        let conn = guard.as_ref().unwrap();
        let (elevation_ft, lat, lon): (Option<f64>, Option<f64>, Option<f64>) = conn
            .query_row(
                "SELECT elevation_ft, latitude_deg, longitude_deg FROM airports WHERE ident = ? LIMIT 1",
                [ident],
                |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)),
            )
            .map_err(|_| anyhow!("airport {ident:?} not found in the database"))?;
        let mut stmt = conn.prepare(
            "SELECT le_ident, he_ident, le_latitude_deg, le_longitude_deg, he_latitude_deg, he_longitude_deg, \
                    le_heading_degT, he_heading_degT, le_displaced_threshold_ft, he_displaced_threshold_ft, \
                    length_ft, width_ft \
             FROM runways WHERE airport_ident = ? AND closed = 0",
        )?;
        let mut rows = stmt.query([ident])?;
        let mut runways = Vec::new();
        while let Some(row) = rows.next()? {
            let le: Option<String> = row.get(0)?;
            let he: Option<String> = row.get(1)?;
            let (Some(le_lat), Some(le_lon), Some(he_lat), Some(he_lon)) =
                (row.get::<_, Option<f64>>(2)?, row.get::<_, Option<f64>>(3)?, row.get::<_, Option<f64>>(4)?, row.get::<_, Option<f64>>(5)?)
            else {
                continue;
            };
            let le_hdg: Option<f64> = row.get(6)?;
            let he_hdg: Option<f64> = row.get(7)?;
            let le_disp: Option<f64> = row.get(8)?;
            let he_disp: Option<f64> = row.get(9)?;
            let length: f64 = row.get::<_, Option<f64>>(10)?.unwrap_or(5000.0);
            let width: f64 = row.get::<_, Option<f64>>(11)?.unwrap_or(crate::config::DEFAULT_RUNWAY_WIDTH_FT);
            for (id, lat, lon, flat, flon, hdg, disp) in [
                (le, le_lat, le_lon, he_lat, he_lon, le_hdg, le_disp),
                (he, he_lat, he_lon, le_lat, le_lon, he_hdg, he_disp),
            ] {
                let Some(id) = id else { continue };
                runways.push(RunwayInfo {
                    ident: id,
                    lat,
                    lon,
                    far_lat: flat,
                    far_lon: flon,
                    course_deg: hdg.unwrap_or_else(|| crate::types::initial_bearing_deg(lat, lon, flat, flon)),
                    length_ft: length,
                    width_ft: width,
                    displaced_threshold_ft: disp.unwrap_or(0.0),
                });
            }
        }
        Ok(AirportData {
            ident: ident.to_string(),
            elevation_ft: elevation_ft.unwrap_or(0.0),
            lat: lat.unwrap_or(0.0),
            lon: lon.unwrap_or(0.0),
            runways,
        })
    }
}

/// The nearest airport with runways, for a plan with no airport named.
fn nearest_airport(ctx: &ToolContext, lat: f64, lon: f64) -> Result<String> {
    ensure_runway_conn(ctx)?;
    let guard = ctx.runway_conn.lock().unwrap();
    let conn = guard.as_ref().unwrap();
    let cos = lat.to_radians().cos();
    let ident: String = conn.query_row(
        "SELECT a.ident FROM airports a \
         WHERE a.latitude_deg BETWEEN ?1 - 0.5 AND ?1 + 0.5 \
           AND EXISTS (SELECT 1 FROM runways r WHERE r.airport_ident = a.ident AND r.closed = 0) \
         ORDER BY power(a.latitude_deg - ?1, 2) + power((a.longitude_deg - ?2) * ?3, 2) LIMIT 1",
        duckdb::params![lat, lon, cos],
        |r| r.get(0),
    )?;
    Ok(ident)
}

/// `fly(code)`: build or amend the plan from pilot-vocabulary Python.
pub fn tool_fly(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    use crate::core::plan_profile::PlanProfile;
    use crate::plan::compile::RStepKind;
    let code = match arg_str(args, "code") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {e}"),
    };
    let Some(bridge) = ctx.bridge.clone() else {
        return "error: aircraft systems bridge unavailable".to_string();
    };
    let (snapshot, plan_status, plan_airport) = {
        let mut pilot = ctx.pilot.lock();
        let snap = pilot.latest_snapshot.clone();
        let (status, airport) = match pilot.profile_mut::<PlanProfile>() {
            Some(p) => (Some(p.status_json()), p.pattern_runway().map(|(a, _)| a)),
            None => (None, None),
        };
        (snap, status, airport)
    };
    let Some(snapshot) = snapshot else {
        return "error: no aircraft state yet; call get_status first".to_string();
    };
    let state = snapshot.state.clone();
    let airport = match args.get("airport").and_then(|v| v.as_str()).filter(|s| !s.trim().is_empty()) {
        Some(a) => a.trim().to_ascii_uppercase(),
        None => match plan_airport.or_else(|| ctx.config.lock().airport.airport.clone()) {
            Some(a) => a,
            None => {
                let (Some(lat), Some(lon)) = (bridge.get_dataref_value(LATITUDE_DEG.name), bridge.get_dataref_value(LONGITUDE_DEG.name)) else {
                    return "error: no airport given and no position to find the nearest one".to_string();
                };
                match nearest_airport(ctx, lat, lon) {
                    Ok(a) => a,
                    Err(e) => return format!("error: finding the nearest airport: {e}"),
                }
            }
        },
    };
    if let Some(bus) = &ctx.bus {
        bus.push_log_kind(crate::bus::LogKind::Plan, format!("fly ({airport}):\n{}", code.trim_end()));
    }
    let base = ctx.config.lock().clone();
    let airports = DuckAirports { ctx };
    let prepared = match crate::plan::session::prepare_with_traffic(
        &code,
        &airport,
        &base,
        &crate::plan::gazetteer::GAZETTEER,
        &airports,
        bridge.georef(),
        plan_status.as_ref(),
        &snapshot.traffic,
    ) {
        Ok(p) => p,
        Err(e) => return format!("error: {e:#}"),
    };

    // go_around() needs a pattern in the air to go around from.
    if prepared.delta.go_around && ctx.pilot.lock().pattern_fly_mut().is_none() {
        return if landed(ctx) {
            "error: already on the ground after landing; a go-around is no longer possible (for a touch-and-go, fly the pattern with land=TOUCH_AND_GO before touchdown)".to_string()
        } else {
            NO_PATTERN.to_string()
        };
    }

    // On the ground a plan can only start with the pattern (a takeoff
    // into closed traffic) from a lined-up position.
    let mut takeoff_run = None;
    if state.on_ground {
        if let Some(steps) = &prepared.delta.steps {
            match steps.first().map(|s| &s.kind) {
                Some(RStepKind::Pattern { pattern, .. }) | Some(RStepKind::Takeoff { pattern }) => {
                    if let Some(msg) = parking_brake_refusal(ctx, "fly") {
                        return msg;
                    }
                    match validate_takeoff_position(ctx, &pattern.frame, &pattern.spec.runway) {
                        Ok(run) => takeoff_run = Some(run),
                        Err(msg) => return msg,
                    }
                }
                Some(_) => {
                    return "error: on the ground, a plan can only start with takeoff(rwy) or join(p) (a takeoff into closed traffic) from the runway; taxi and line up with the ground tools first".to_string();
                }
                None => {}
            }
        }
    }

    let reference = prepared.delta.reference_pattern().map(|p| p.config.clone());
    // The reply: a header, the plan as the autopilot reads it, notes, and
    // anything the code printed.
    let reply = |header: String, notes: Vec<String>| {
        let mut out = vec![header];
        out.extend(prepared.summary.iter().map(|l| format!("  {l}")));
        out.extend(notes.into_iter().map(|n| format!("  note: {n}")));
        out
    };
    let printed = (!prepared.printed.is_empty()).then(|| format!("printed:\n{}", prepared.printed.trim_end()));

    // preview=true: fly the change ahead in the simple model, change nothing.
    if prepared.preview || args.get("preview").and_then(|v| v.as_bool()).unwrap_or(false) {
        let (profile, cfg, traffic, notes) = {
            let mut pilot = ctx.pilot.lock();
            let cfg = reference.clone().unwrap_or_else(|| pilot.config.clone());
            let traffic = pilot.traffic.snapshot();
            let picture = crate::core::traffic::TrafficPicture::default();
            let mut profile = match pilot.profile_mut::<PlanProfile>() {
                Some(p) => p.preview_copy(picture),
                None => PlanProfile::new(base.clone(), &state).with_traffic(picture),
            };
            let notes = profile.apply(prepared.delta, &state);
            (profile, cfg, traffic, notes)
        };
        let pv = crate::plan::preview::preview(&cfg, profile, &state, traffic, PREVIEW_HORIZON_S);
        let mut out = reply("preview only; nothing changed. The plan would be:".to_string(), notes);
        out.extend(pv.lines());
        out.extend(printed);
        return out.join("\n");
    }

    // Amend the plan being flown, checked and applied under one lock so a
    // hand-off in between (touchdown) can't drop the change.
    let mut delta = Some(prepared.delta);
    let amended = {
        let mut pilot = ctx.pilot.lock();
        let notes = pilot.profile_mut::<PlanProfile>().map(|p| p.apply(delta.take().unwrap(), &state));
        if notes.is_some() {
            if let Some(cfg) = &reference {
                if pilot.config.airport.runway.id != cfg.airport.runway.id
                    || pilot.config.airport.airport != cfg.airport.airport
                {
                    pilot.set_reference_config(cfg.clone());
                    ctx.config.lock().airport = cfg.airport.clone();
                }
            }
        }
        notes
    };
    let mut out = match (amended, delta) {
        (Some(notes), _) => reply("plan amended:".to_string(), notes),
        (None, Some(delta)) => {
            let traffic = ctx.pilot.lock().traffic.clone();
            let mut profile = PlanProfile::new(base.clone(), &state).with_traffic(traffic);
            let notes = profile.apply(delta, &state);
            let engaged = match reference {
                Some(cfg) => engage_on_runway(ctx, cfg, Box::new(profile)),
                None => engage(ctx, Box::new(profile)),
            };
            match engaged {
                Ok(displaced) => reply(format!("plan engaged{displaced}:"), notes),
                Err(e) => return e,
            }
        }
        (None, None) => unreachable!("the delta is used only when a plan was amended"),
    };
    // The takeoff roll's status line tracks the runway ahead.
    if let Some(run) = takeoff_run {
        if let Some(pf) = ctx.pilot.lock().pattern_fly_mut() {
            pf.takeoff_run = Some(run);
        }
    }
    out.extend(printed);
    out.join("\n")
}

/// Fast path brake check — returns an error string if the parking brake
/// is set, so takeoff / pattern handlers can fail before the expensive
/// runway-lookup round-trip. None means the brake is released or the
/// dataref isn't available.
fn parking_brake_refusal(ctx: &ToolContext, tool_name: &str) -> Option<String> {
    let v = parking_brake_ratio(ctx)?;
    if v >= 0.5 {
        Some(format!(
            "error: parking brake is SET (ratio={:.2}) — call set_parking_brake(engaged=False) first, then retry {}",
            v, tool_name
        ))
    } else {
        None
    }
}

/// Takeoff-position gate for a `fly` plan that starts on the ground (a
/// takeoff, or closed traffic), against the runway it takes off from:
/// on the centerline, on the pavement, lined up, stopped, with enough
/// runway ahead. Returns the run ahead on success.
fn validate_takeoff_position(
    ctx: &ToolContext,
    runway_frame: &RunwayFrame,
    runway_ident: &str,
) -> Result<TakeoffRun, String> {
    let pilot = ctx.pilot.lock();
    let Some(snap) = pilot.latest_snapshot.as_ref() else {
        return Err(
            "error: no aircraft state yet — call get_status first so the control loop publishes a snapshot".to_string(),
        );
    };
    let pos_rwy = runway_frame.to_runway_frame(snap.state.position_ft);
    let along = pos_rwy.x;
    let cross = pos_rwy.y;
    let length = runway_frame.runway.length_ft;
    let course = runway_frame.runway.course_deg;
    let heading_err = crate::types::wrap_degrees_180(course - snap.state.heading_deg).abs();

    if cross.abs() > 100.0 {
        return Err(format!(
            "error: aircraft is {:.0} ft off runway {} centerline (max 100 ft). Taxi onto the runway first — call engage_line_up runway={} to move into position.",
            cross.abs(), runway_ident, runway_ident
        ));
    }
    if along < -100.0 || along > length {
        return Err(format!(
            "error: aircraft is at along-track {:.0} ft on runway {} (runway extends 0..{:.0} ft). Position is not on the runway — use engage_taxi or engage_line_up to get here first.",
            along, runway_ident, length
        ));
    }
    if heading_err > 15.0 {
        return Err(format!(
            "error: aircraft heading {:.0}° is not aligned with runway {} course {:.0}° (Δ={:.0}° > 15°). Call engage_line_up runway={} and wait for it to finish before taking off.",
            snap.state.heading_deg, runway_ident, course, heading_err, runway_ident
        ));
    }
    if snap.state.gs_kt > 3.0 {
        return Err(format!(
            "error: aircraft moving at {:.1} kt — stop before taking off (usually means engage_line_up is still in progress)",
            snap.state.gs_kt
        ));
    }
    let start_along = along.max(0.0);
    let usable = length - start_along;
    // Below this there's no realistic chance of reaching Vr with margin;
    // refuse rather than let the profile roll off the end. 1000 ft is a
    // generous C172-at-sea-level floor; density-altitude-aware limit is
    // a follow-up.
    const MIN_USABLE_LENGTH_FT: f64 = 1000.0;
    if usable < MIN_USABLE_LENGTH_FT {
        return Err(format!(
            "error: only {:.0} ft of runway {} available from current position (min {:.0} ft). \
             Not enough room to accelerate to Vr. Taxi back to a position with more runway ahead.",
            usable, runway_ident, MIN_USABLE_LENGTH_FT
        ));
    }
    Ok(TakeoffRun { start_along_ft: start_along, usable_length_ft: usable })
}

// ---------- dead-stick (engine-out) landing tools ----------

/// Default traffic pattern side when the LLM omits the `side` argument.
const DEFAULT_TRAFFIC_SIDE: &str = "left";

/// Cap on rows returned by `find_dead_stick_candidates`. Bound by the
/// SQL LIMIT so the database doesn't waste work surfacing rows the
/// caller will discard.
const MAX_DEAD_STICK_CANDIDATES: i64 = 25;

/// Default candidate count when the LLM omits `max_candidates`.
const DEFAULT_DEAD_STICK_CANDIDATES: i64 = 5;

/// Minimum runway length we'll suggest for a dead-stick landing.
const MIN_DEAD_STICK_LENGTH_FT: f64 = 1500.0;

/// Score weights applied to each candidate after the SQL pass.
/// `margin_nm` is the dominant term (1.0); a 20-knot headwind is worth
/// roughly 1 NM of margin (`SCORE_HEADWIND_PER_KT * 20 = 1.0`); each
/// 1000 ft of runway length adds 0.1 NM-equivalent.
const SCORE_HEADWIND_PER_KT: f64 = 0.05;
const SCORE_LENGTH_PER_FT: f64 = 0.0001;

/// Reach buffer on the still-air glide — refuse engagement if the
/// runway is more than `1.05 × glide_ratio × alt_agl` away. Catches
/// gross mistakes; wind isn't subtracted yet so the LLM should bias
/// toward `margin_nm > 1` from the candidates query.
const REACH_BUFFER_FACTOR: f64 = 1.05;

/// Approximate feet per degree of latitude (great-circle, near-equator
/// equivalent). Good to ~0.5% across the latitudes we care about — fine
/// for the side-of-centerline heuristic in `find_dead_stick_candidates`.
const FT_PER_DEG_LAT: f64 = 364567.0;

/// One scored runway end candidate for a dead-stick landing. Built from
/// the SQL result + post-scored on the Rust side for headwind alignment
/// and length preference.
#[derive(Debug, Clone)]
struct DeadStickCandidate {
    airport_ident: String,
    rwy_ident: String,
    course_deg: f64,
    length_ft: f64,
    surface: String,
    dist_nm: f64,
    margin_nm: f64,
    headwind_kt: f64,
    /// Recommended traffic-pattern side ("left" or "right") for the
    /// dead-stick join. Picked so the aircraft's current position is
    /// already on the pattern side — joining the pattern from the
    /// inside saves the centerline-crossing maneuver and the altitude
    /// burned in the recapture turn. Ignores published traffic-side
    /// convention since with the engine out, glide path beats
    /// procedure.
    side: &'static str,
    score: f64,
}

pub fn tool_find_dead_stick_candidates(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let lat = match arg_f64(args, "lat") {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    let lon = match arg_f64(args, "lon") {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    let alt_agl_ft = match arg_f64(args, "alt_agl_ft") {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    // heading_deg is required by the schema for symmetry with the rest of
    // the position-scoring helpers, but the v1 scorer doesn't actually
    // use it (we score by reach margin + headwind alignment + length, not
    // by the aircraft's current heading). Accept it and ignore.
    let _heading_deg = args.get("heading_deg").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let wind_dir_deg = args
        .get("wind_dir_deg")
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    let wind_kt = args.get("wind_kt").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let max_candidates = args
        .get("max_candidates")
        .and_then(|v| v.as_i64())
        .unwrap_or(DEFAULT_DEAD_STICK_CANDIDATES)
        .clamp(1, MAX_DEAD_STICK_CANDIDATES);
    let glide_ratio = ctx.config.lock().performance.glide_ratio.max(1.0);

    if ctx.apt_dat_cache_dir.is_none() {
        return "error: apt.dat parquet cache is not configured".to_string();
    }
    if let Err(e) = ensure_runway_conn(ctx) {
        return format!("error: could not open apt.dat parquet cache: {}", e);
    }
    let guard = ctx.runway_conn.lock().unwrap();
    let conn = guard.as_ref().unwrap();

    // Filter out closed and absurdly short runways. Both runway ends
    // are unioned so the LLM sees each landable direction independently.
    let sql = format!("\
        WITH params AS ( \
            SELECT ?::DOUBLE AS lat, ?::DOUBLE AS lon, \
                   ?::DOUBLE AS alt_agl, ?::DOUBLE AS glide_ratio \
        ), \
        runway_ends AS ( \
            SELECT airport_ident, le_ident AS rwy_ident, \
                   le_latitude_deg AS thr_lat, le_longitude_deg AS thr_lon, \
                   le_heading_degT AS course_deg, length_ft, surface \
            FROM runways WHERE closed = 0 AND length_ft >= {min_len} \
              AND le_latitude_deg IS NOT NULL AND le_longitude_deg IS NOT NULL \
            UNION ALL \
            SELECT airport_ident, he_ident, he_latitude_deg, he_longitude_deg, \
                   he_heading_degT, length_ft, surface \
            FROM runways WHERE closed = 0 AND length_ft >= {min_len} \
              AND he_latitude_deg IS NOT NULL AND he_longitude_deg IS NOT NULL \
        ), \
        scored AS ( \
            SELECT re.*, \
                   3440.065 * 2 * ASIN(SQRT( \
                       POWER(SIN(RADIANS((re.thr_lat - p.lat)/2)), 2) + \
                       COS(RADIANS(p.lat)) * COS(RADIANS(re.thr_lat)) * \
                         POWER(SIN(RADIANS((re.thr_lon - p.lon)/2)), 2) \
                   )) AS dist_nm, \
                   (p.alt_agl * p.glide_ratio / {ft_per_nm}) AS reach_nm \
            FROM runway_ends re CROSS JOIN params p \
        ) \
        SELECT airport_ident, rwy_ident, course_deg, length_ft, COALESCE(surface, '') AS surface, \
               dist_nm, (reach_nm - dist_nm) AS margin_nm, thr_lat, thr_lon \
        FROM scored \
        WHERE dist_nm <= reach_nm \
        ORDER BY margin_nm DESC \
        LIMIT ?",
        min_len = MIN_DEAD_STICK_LENGTH_FT,
        ft_per_nm = NM_TO_FT,
    );
    let mut stmt = match conn.prepare(&sql) {
        Ok(s) => s,
        Err(e) => return format!("error: {}", e),
    };
    let mut rows = match stmt
        .query(duckdb::params![lat, lon, alt_agl_ft, glide_ratio, max_candidates])
    {
        Ok(r) => r,
        Err(e) => return format!("error: {}", e),
    };
    let mut candidates: Vec<DeadStickCandidate> = Vec::new();
    loop {
        let row = match rows.next() {
            Ok(Some(r)) => r,
            Ok(None) => break,
            Err(e) => return format!("error: {}", e),
        };
        let airport_ident: String = row.get(0).unwrap_or_default();
        let rwy_ident: String = row.get(1).unwrap_or_default();
        let course_deg: f64 = row.get(2).unwrap_or(0.0);
        let length_ft: f64 = row.get(3).unwrap_or(0.0);
        let surface: String = row.get(4).unwrap_or_default();
        let dist_nm: f64 = row.get(5).unwrap_or(0.0);
        let margin_nm: f64 = row.get(6).unwrap_or(0.0);
        let thr_lat: f64 = row.get(7).unwrap_or(0.0);
        let thr_lon: f64 = row.get(8).unwrap_or(0.0);
        let headwind_kt = if wind_kt.abs() > 0.01 {
            // Headwind on landing = wind_kt * cos(wind_dir - course).
            // Positive when wind blows from runway course direction (into
            // the nose on landing). cos((wind_from) - course): if wind is
            // from 90° and runway course is 90°, headwind is wind_kt.
            wind_kt * (((wind_dir_deg - course_deg).to_radians()).cos())
        } else {
            0.0
        };
        // Recommend the pattern side that puts the aircraft's current
        // position on the inside of the pattern. Project (aircraft −
        // threshold) onto the runway's right vector (course + 90°): a
        // positive runway-frame y means aircraft is right of course →
        // right traffic; negative → left traffic. Right beats published
        // convention with no engine: glide path is what makes the field.
        let lat_rad = lat.to_radians();
        let dlat_ft = (lat - thr_lat) * FT_PER_DEG_LAT;
        let dlon_ft = (lon - thr_lon) * FT_PER_DEG_LAT * lat_rad.cos();
        // forward = (sin(course), cos(course)) in (east, north); right
        // = forward rotated +90° = (cos(course), -sin(course)).
        let course_rad = course_deg.to_radians();
        let runway_y_ft = dlon_ft * course_rad.cos() - dlat_ft * course_rad.sin();
        let side = if runway_y_ft >= 0.0 { "right" } else { "left" };
        let score = margin_nm
            + SCORE_HEADWIND_PER_KT * headwind_kt
            + SCORE_LENGTH_PER_FT * length_ft;
        candidates.push(DeadStickCandidate {
            airport_ident,
            rwy_ident,
            course_deg,
            length_ft,
            surface,
            dist_nm,
            margin_nm,
            headwind_kt,
            side,
            score,
        });
    }
    drop(rows);
    drop(stmt);
    drop(guard);

    if candidates.is_empty() {
        return format!(
            "0 reachable runways within glide (alt_agl={:.0}ft, glide_ratio={:.1} → reach≈{:.1}NM)",
            alt_agl_ft,
            glide_ratio,
            alt_agl_ft * glide_ratio / NM_TO_FT
        );
    }
    candidates
        .sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap_or(std::cmp::Ordering::Equal));
    candidates.truncate(max_candidates as usize);

    let mut lines = vec![
        "airport_ident\trunway_ident\tdist_nm\tmargin_nm\tcourse_deg\tlength_ft\tsurface\theadwind_kt\tside\tscore"
            .to_string(),
    ];
    for c in &candidates {
        lines.push(format!(
            "{}\t{}\t{:.2}\t{:.2}\t{:.0}\t{:.0}\t{}\t{:.1}\t{}\t{:.3}",
            c.airport_ident,
            c.rwy_ident,
            c.dist_nm,
            c.margin_nm,
            c.course_deg,
            c.length_ft,
            c.surface,
            c.headwind_kt,
            c.side,
            c.score
        ));
    }
    lines.join("\n")
}

pub fn tool_engage_dead_stick_landing(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let airport = match arg_str(args, "airport_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let runway_ident = match arg_str(args, "runway_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let side = args
        .get("side")
        .and_then(|v| v.as_str())
        .unwrap_or(DEFAULT_TRAFFIC_SIDE)
        .to_string();

    // Snapshot + glide_ratio in one pass each so the on-ground / reach
    // checks don't reacquire locks they already held.
    let (snap_opt, glide_ratio) = {
        let snap_opt = ctx.pilot.lock().latest_snapshot.clone();
        let glide_ratio = ctx.config.lock().performance.glide_ratio.max(1.0);
        (snap_opt, glide_ratio)
    };
    let Some(snap) = snap_opt else {
        return "error: no aircraft state yet — call get_status first so the control loop publishes a snapshot".to_string();
    };
    if snap.state.on_ground {
        return "error: aircraft is on the ground; dead-stick landing requires being airborne".to_string();
    }

    let config = match runway_config(ctx, &airport, &runway_ident, &side) {
        Ok(cfg) => cfg,
        Err(e) => return format!("error: {}", e),
    };

    // Reach validation: refuse runways that aren't physically makeable
    // in the still-air glide.
    let alt_agl_ft = snap.state.alt_agl_ft.max(0.0);
    let reach_ft = alt_agl_ft * glide_ratio;
    let dist_ft = (snap.state.position_ft - config.airport.runway.threshold_ft).length();
    let reach_nm = reach_ft / NM_TO_FT;
    let dist_nm = dist_ft / NM_TO_FT;
    if dist_ft > reach_ft * REACH_BUFFER_FACTOR {
        return format!(
            "error: runway {}/{} not reachable in glide (need {:.1} NM, have {:.1} NM at AGL={:.0}ft, glide_ratio={:.1}). Pick a closer candidate from find_dead_stick_candidates.",
            airport, runway_ident, dist_nm, reach_nm, alt_agl_ft, glide_ratio
        );
    }

    let profile = crate::core::dead_stick_profile::DeadStickLandingProfile::new(
        config.clone(),
        RunwayFrame::new(config.airport.runway.clone()),
        &snap.state,
    );
    let entry_phase = profile.phase;
    let displaced = match engage_on_runway(ctx, config, Box::new(profile)) {
        Ok(d) => d,
        Err(e) => return e,
    };
    format!(
        "engaged dead_stick_landing {} runway {} (entry phase {}; reach {:.1}/{:.1} NM){}",
        airport,
        runway_ident,
        entry_phase.value(),
        dist_nm,
        reach_nm,
        displaced
    )
}

pub fn tool_takeoff_checklist(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    let pilot = ctx.pilot.lock();
    let snap = pilot.latest_snapshot.clone();
    drop(pilot);
    let Some(snap) = snap else {
        return "error: no aircraft state yet — call get_status first so the control loop publishes a snapshot.".to_string();
    };
    let state = &snap.state;
    let mut lines: Vec<String> =
        vec!["TAKEOFF CHECKLIST — address every [ACTION] item before taking off:".to_string()];
    let mut action_needed = false;

    match parking_brake_ratio(ctx) {
        None => {
            lines.push(
                "  [?]      parking brake: state unavailable — call set_parking_brake(engaged=False) to be safe".to_string()
            );
            action_needed = true;
        }
        Some(v) if v >= 0.5 => {
            lines.push(format!(
                "  [ACTION] parking brake: SET (ratio={:.2}) — call set_parking_brake(engaged=False)",
                v
            ));
            action_needed = true;
        }
        Some(v) => {
            lines.push(format!("  [OK]     parking brake: released (ratio={:.2})", v));
        }
    }

    if state.flap_index <= 10 {
        lines.push(format!("  [OK]     flaps: {} deg", state.flap_index));
    } else {
        lines.push(format!(
            "  [ACTION] flaps: {} deg — call set_flaps(degrees=10) or set_flaps(degrees=0) for normal takeoff",
            state.flap_index
        ));
        action_needed = true;
    }

    if state.gear_down {
        lines.push("  [OK]     gear: down".to_string());
    } else {
        lines.push("  [ACTION] gear: up — extend the gear before takeoff".to_string());
        action_needed = true;
    }

    if state.on_ground {
        lines.push("  [OK]     on ground".to_string());
    } else {
        lines.push("  [ERROR]  not on ground — a takeoff starts on the runway".to_string());
        action_needed = true;
    }

    {
        let pilot = ctx.pilot.lock();
        let runway_course = pilot.runway_frame.runway.course_deg;
        drop(pilot);
        let heading_err =
            crate::types::wrap_degrees_180(runway_course - state.heading_deg).abs();
        if heading_err <= 15.0 {
            lines.push(format!(
                "  [OK]     aligned with runway course {:.0}° (heading {:.0}°, Δ={:.0}°)",
                runway_course, state.heading_deg, heading_err
            ));
        } else {
            lines.push(format!(
                "  [ACTION] heading {:.0}° is {:.0}° off runway course {:.0}° — run engage_line_up and wait for it to finish (aircraft stopped, pointed down the runway) before taking off",
                state.heading_deg, heading_err, runway_course
            ));
            action_needed = true;
        }
    }

    if state.gs_kt > 3.0 {
        lines.push(format!(
            "  [ACTION] still moving at {:.1} kt ground speed — wait for engage_line_up to finish and the aircraft to stop before taking off",
            state.gs_kt
        ));
        action_needed = true;
    }

    if state.ias_kt > 5.0 {
        lines.push(format!(
            "  [?]      already rolling at {:.1} kt IAS — double-check you meant to call this",
            state.ias_kt
        ));
    }

    let active: Vec<&str> = snap
        .active_profiles
        .iter()
        .map(|s| s.as_str())
        .filter(|p| matches!(*p, "pattern_fly" | "plan"))
        .collect();
    if !active.is_empty() {
        lines.push(format!("  [INFO]   already engaged: {}", active.join(", ")));
    }

    lines.push("  [REMINDER] identify the runway you are on (get_status for lat/lon + heading, then sql_query with the 'What is the closest runway?' example) before committing to the roll".to_string());
    lines.push("  [REMINDER] acknowledge takeoff clearance on the radio (broadcast_on_radio) if ATC has issued one".to_string());
    lines.push("".to_string());
    if action_needed {
        lines.push("One or more items need action. Fix them, then re-run this checklist or proceed with the takeoff.".to_string());
    } else {
        lines.push("All items OK. Fly the takeoff clearance (fly: takeoff(rwy).then(...)) to begin the roll.".to_string());
    }
    lines.join("\n")
}

pub fn tool_disengage_profile(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let name = match arg_str(args, "name") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let mut pilot = ctx.pilot.lock();
    let added = match pilot.disengage_profile(&name) {
        Ok(a) => a,
        Err(e) => return format!("error: {e}"),
    };
    if added.is_empty() && !pilot.has_profile(&name) {
        return format!("no profile named {:?} was active", name);
    }
    format!(
        "disengaged {}; re-added idle profiles: {}",
        name,
        if added.is_empty() { "none".to_string() } else { added.join(", ") }
    )
}

pub fn tool_list_profiles(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    ctx.pilot.lock().list_profile_names().join(", ")
}

/// The last flight phase is a post-touchdown ground phase. Used only to
/// word errors for airborne-only commands; which profile is flying is the
/// profiles' business.
fn landed(ctx: &ToolContext) -> bool {
    matches!(
        ctx.pilot.lock().phase(),
        FlightPhase::Rollout | FlightPhase::RunwayExit | FlightPhase::TaxiClear
    )
}

pub fn tool_tune_radio(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let radio = match arg_str(args, "radio") {
        Ok(s) => s.to_ascii_lowercase(),
        Err(e) => return format!("error: {}", e),
    };
    let freq = match arg_f64(args, "frequency_mhz") {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    let Some(bridge) = ctx.bridge.as_ref() else {
        return "error: aircraft systems bridge unavailable".to_string();
    };
    let dref = match radio_dataref(&radio) {
        Ok(d) => d,
        Err(e) => return format!("error: {}", e),
    };
    let freq_khz = (freq * 1000.0).round() as i64;
    if let Err(e) = bridge.write_dataref_values(&[(dref.to_string(), freq_khz as f64)]) {
        return format!("error: {}", e);
    }
    if let Some(bus) = &ctx.bus {
        bus.push_radio(format!("[{} tuned to {:.3}]", radio.to_uppercase(), freq));
    }
    format!(
        "tuned {} to {:.3} MHz (set dataref={} value={})",
        radio, freq, dref, freq_khz
    )
}

pub fn tool_set_parking_brake(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let engaged = match arg_bool(args, "engaged") {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    let Some(bridge) = ctx.bridge.as_ref() else {
        return "error: aircraft systems bridge unavailable".to_string();
    };
    if let Err(e) = write_parking_brake(ctx, bridge, if engaged { 1.0 } else { 0.0 }) {
        return format!("error: {}", e);
    }
    if engaged { "parking brake engaged".to_string() } else { "parking brake released".to_string() }
}

const VALID_FLAP_SETTINGS: [i32; 4] = [0, 10, 20, 30];

pub fn tool_set_flaps(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let degrees = match arg_i64(args, "degrees") {
        Ok(v) => v as i32,
        Err(e) => return format!("error: {}", e),
    };
    let Some(bridge) = ctx.bridge.as_ref() else {
        return "error: aircraft systems bridge unavailable".to_string();
    };
    if !VALID_FLAP_SETTINGS.contains(&degrees) {
        return format!(
            "error: invalid flap setting {} — valid settings are {}",
            degrees,
            VALID_FLAP_SETTINGS.iter().map(|s| s.to_string()).collect::<Vec<_>>().join(", ")
        );
    }
    let ratio = degrees as f64 / 30.0;
    if let Err(e) = bridge.write_dataref_values(&[(FLAP_HANDLE_REQUEST_RATIO.name.to_string(), ratio)]) {
        return format!("error: {}", e);
    }
    let mut pilot = ctx.pilot.lock();
    pilot.flaps.note_written(degrees);
    let warning = if pilot.pattern_fly_mut().is_some() {
        " (note: the pattern manages flaps per phase — it will set them again at its next flap change)"
    } else {
        ""
    };
    format!("flaps set to {}°{}", degrees, warning)
}

pub fn tool_broadcast_on_radio(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let radio = match arg_str(args, "radio") {
        Ok(s) => s.to_ascii_lowercase(),
        Err(e) => return format!("error: {}", e),
    };
    let message = match arg_str(args, "message") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let dref = match radio_dataref(&radio) {
        Ok(d) => d,
        Err(e) => return format!("error: {}", e),
    };
    let freq_mhz = ctx.bridge.as_ref().and_then(|b| b.get_dataref_value(dref)).map(|khz| khz / 1000.0);
    let freq_label = freq_mhz.map(|f| format!("{f:.3}")).unwrap_or_else(|| "---".to_string());
    let line = format!("[{} {}] {}", radio.to_uppercase(), freq_label, message);
    if let Some(bus) = &ctx.bus {
        bus.push_radio(line.clone());
    } else {
        println!("{}", line);
    }
    {
        let t_sim = ctx.bridge.as_ref().and_then(|b| b.get_dataref_value(crate::sim::datarefs::SIM_TIME_S.name));
        ctx.radio_log.lock().push((t_sim.unwrap_or(0.0), message.to_string()));
        let mut br = ctx.recent_broadcasts.lock();
        br.push(line.clone());
        if br.len() > 16 {
            let excess = br.len() - 16;
            br.drain(..excess);
        }
    }
    if let Some(atc) = &ctx.sim_atc {
        atc.hear(crate::llm::sim_atc::RadioCall {
            freq_mhz,
            message: message.clone(),
            status_json: tool_get_status(ctx, &Map::new()),
        });
    }
    format!("broadcast on {}: {}", radio, message)
}

/// Produce the `(legs, names)` fed into `TaxiProfile::new`, inserting a
/// synthetic "pullout" leg from the aircraft's current position to the
/// graph's first node when they are not already essentially coincident.
///
/// The planner produces a route that starts at the taxi-graph node nearest
/// to the aircraft. At a gate or a hold-short the aircraft can easily be
/// 50–150 ft away from that node, oriented at any heading. Without a
/// pullout leg the nose-wheel controller sees a giant crosstrack error
/// immediately and tries to cut diagonally across the ramp. With one,
/// the first leg is "drive from here to node X" which the controllers
/// handle cleanly: crosstrack starts at zero, heading error drives the
/// rotation, proximity triggers the advance to leg 1 of the real route.
///
/// Returned `pullout_ft` is `None` when no pullout was needed, or `Some(d)`
/// where `d` is the distance of the inserted leg in feet. Whether the
/// lead-in is allowed at all (no backtaxi from a runway start) is decided
/// by `taxi_route::plan_from`.
pub struct TaxiLegsPlan {
    pub legs: Vec<StraightLeg>,
    pub names: Vec<String>,
    pub pullout_ft: Option<f64>,
}

pub fn build_taxi_legs_with_pullout(
    aircraft_pos_ft: crate::types::Vec2,
    plan_legs: &[taxi_route::TaxiLeg],
    georef: GeoReference,
) -> TaxiLegsPlan {
    const PULLOUT_THRESHOLD_FT: f64 = 20.0;

    let mut legs_ft: Vec<StraightLeg> = plan_legs
        .iter()
        .map(|l| StraightLeg {
            start_ft: geodetic_offset_ft(l.from_lat, l.from_lon, georef),
            end_ft: geodetic_offset_ft(l.to_lat, l.to_lon, georef),
        })
        .collect();
    let mut names: Vec<String> = plan_legs.iter().map(|l| l.taxiway_name.clone()).collect();

    let Some(first) = legs_ft.first().copied() else {
        return TaxiLegsPlan { legs: legs_ft, names, pullout_ft: None };
    };
    let d_ft = (first.start_ft - aircraft_pos_ft).length();
    if d_ft <= PULLOUT_THRESHOLD_FT {
        return TaxiLegsPlan { legs: legs_ft, names, pullout_ft: None };
    }
    legs_ft.insert(
        0,
        StraightLeg {
            start_ft: aircraft_pos_ft,
            end_ft: first.start_ft,
        },
    );
    names.insert(0, String::new());
    TaxiLegsPlan { legs: legs_ft, names, pullout_ft: Some(d_ft) }
}

pub fn tool_engage_line_up(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let bridge = match ctx.bridge.as_ref() {
        Some(b) => b.clone(),
        None => return "error: aircraft systems bridge unavailable; engage_line_up needs the georef".to_string(),
    };
    let airport = match arg_str(args, "airport_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let runway = match arg_str(args, "runway_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let intersection = args
        .get("intersection")
        .and_then(|v| v.as_str())
        .map(str::to_string);
    let (start_lat, start_lon) = match resolve_engage_start_latlon(ctx, args) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };

    if let Err(e) = ensure_runway_conn(ctx) {
        return format!("error: {}", e);
    }
    let (thr_lat, thr_lon, heading_deg) = {
        let guard = ctx.runway_conn.lock().unwrap();
        let conn = guard.as_ref().unwrap();
        let forms = taxi_route::runway_query_forms(&runway);
        let row: Result<(f64, f64, f64), _> = conn.query_row(
            "SELECT \
                 CASE WHEN le_ident = ? OR le_ident = ? \
                      THEN le_latitude_deg  ELSE he_latitude_deg  END, \
                 CASE WHEN le_ident = ? OR le_ident = ? \
                      THEN le_longitude_deg ELSE he_longitude_deg END, \
                 CASE WHEN le_ident = ? OR le_ident = ? \
                      THEN le_heading_degT  ELSE he_heading_degT  END \
             FROM runways \
             WHERE airport_ident = ? \
               AND (le_ident = ? OR le_ident = ? OR he_ident = ? OR he_ident = ?) \
               AND closed = 0 \
             LIMIT 1",
            [
                forms[0].as_str(), forms[1].as_str(),
                forms[0].as_str(), forms[1].as_str(),
                forms[0].as_str(), forms[1].as_str(),
                airport.as_str(),
                forms[0].as_str(), forms[1].as_str(),
                forms[0].as_str(), forms[1].as_str(),
            ],
            |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)),
        );
        match row {
            Ok(v) => v,
            Err(e) => return format!("error: runway {:?} not found at {}: {}", runway, airport, e),
        }
    };

    let georef = bridge.georef();
    let aircraft_pos_ft = geodetic_offset_ft(start_lat, start_lon, georef);

    // Engage on the resolved runway so later checks (the takeoff
    // alignment guard) compare against the runway we're lining up on,
    // not whatever startup-default runway was in the config.
    let config = match runway_config(ctx, &airport, &runway, "left") {
        Ok(cfg) => cfg,
        Err(e) => return format!("error: {}", e),
    };

    // Entry point: either the runway threshold (default) or the
    // on-runway side of a named intersection (e.g. taxiway C / rwy 19).
    // Heading stays the runway course in either case — the aircraft
    // still has to face down the runway for takeoff.
    let entry_point_ft = match intersection.as_deref() {
        None => geodetic_offset_ft(thr_lat, thr_lon, georef),
        Some(taxiway) => {
            if let Err(e) = ensure_runway_conn(ctx) {
                return format!("error: {}", e);
            }
            let guard = ctx.runway_conn.lock().unwrap();
            let conn = guard.as_ref().unwrap();
            let graph = match taxi_graph(ctx, conn, &airport) {
                Ok(g) => g,
                Err(e) => return format!("error: {}", e),
            };
            let x = match taxi_route::resolve_runway_intersection(
                conn, &graph, &airport, &runway, taxiway,
            ) {
                Ok(x) => x,
                Err(e) => return format!("error: {}", e),
            };
            geodetic_offset_ft(x.on_runway_lat, x.on_runway_lon, georef)
        }
    };
    let threshold_ft = entry_point_ft;

    // Short-circuit if we're already aligned on the runway centerline.
    // Otherwise the fresh line-up route drags the aircraft back toward
    // the threshold from its current on-runway position — when the
    // aircraft is near the far end of the runway this plans a 180°
    // reverse trip.
    {
        let forward = heading_to_vector(heading_deg, 1.0);
        let right = heading_to_vector(heading_deg + 90.0, 1.0);
        let offset = aircraft_pos_ft - threshold_ft;
        let along_track_ft = offset.x * forward.x + offset.y * forward.y;
        let crosstrack_ft = offset.x * right.x + offset.y * right.y;
        let heading_err =
            crate::types::wrap_degrees_180(heading_deg - start_heading_deg(&bridge)).abs();
        let on_runway = along_track_ft > -200.0 && along_track_ft < 12_000.0;
        let aligned = heading_err < 10.0 && crosstrack_ft.abs() < 30.0;
        if on_runway && aligned {
            return format!(
                "already lined up on {} runway {} — heading {:.0}° (course {:.0}°, Δ={:.0}°), \
                 {:.0} ft along the runway, {:.0} ft off centerline. no-op.",
                airport, runway, start_heading_deg(&bridge), heading_deg, heading_err,
                along_track_ft.max(0.0), crosstrack_ft.abs()
            );
        }
    }

    // Distance guard: engage_line_up plans a 2-leg straight route
    // (aircraft → entry point → 100 ft past). If the aircraft is far
    // from the entry point, that straight line cuts diagonally across
    // whatever terrain/taxiways sit between. Require taxi-graph
    // proximity of the hold-short before handing off the final
    // crossing+alignment to this tool; otherwise the caller should
    // engage_taxi to reach the hold-short first.
    {
        let approach_ft = (entry_point_ft - aircraft_pos_ft).length();
        const LINE_UP_MAX_APPROACH_FT: f64 = 300.0;
        if approach_ft > LINE_UP_MAX_APPROACH_FT {
            let taxi_hint = match intersection.as_deref() {
                Some(taxiway) => format!(
                    "call engage_taxi(destination_runway={:?}, intersection={:?}) first so you reach the hold-short by taxiway, then retry engage_line_up",
                    runway, taxiway
                ),
                None => format!(
                    "call engage_taxi(destination_runway={:?}) first so you reach the runway's hold-short by taxiway, then retry engage_line_up",
                    runway
                ),
            };
            return format!(
                "error: aircraft is {:.0} ft from the runway {} entry point (max {:.0} ft). engage_line_up only plans the final hold-short crossing + alignment; it won't pathfind across the airport. {}",
                approach_ft, runway, LINE_UP_MAX_APPROACH_FT, taxi_hint
            );
        }
    }

    // Final line-up pose: 100 ft past the pavement end along the runway
    // course. Enough for the aircraft to stop with nose aligned and wheels
    // clear of the hold-short line; leaves a full runway ahead for the
    // takeoff roll. The aircraft ends stopped via TaxiProfile's final-leg
    // ramp + hold brake latch.
    let along_course = heading_to_vector(heading_deg, 100.0);
    let aligned_ft = threshold_ft + along_course;

    let legs_ft = vec![
        StraightLeg { start_ft: aircraft_pos_ft, end_ft: threshold_ft },
        StraightLeg { start_ft: threshold_ft,     end_ft: aligned_ft },
    ];
    let names = vec!["(entering runway)".to_string(), format!("rwy {}", runway)];

    let profile = Box::new(TaxiProfile::new_line_up(legs_ft, names));
    let displaced = match engage_on_runway(ctx, config, profile) {
        Ok(d) => d,
        Err(e) => return e,
    };

    let approach_dist = (threshold_ft - aircraft_pos_ft).length();
    let at_clause = intersection
        .as_deref()
        .map(|t| format!(" at {}", t))
        .unwrap_or_default();
    format!(
        "engaged line_up {} runway {}{} — crossing {:.0} ft, final heading {:.0}°{}",
        airport,
        runway,
        at_clause,
        approach_dist,
        heading_deg,
        displaced
    )
}

fn start_heading_deg(bridge: &Arc<dyn ToolBridge>) -> f64 {
    // Prefer the bridge's live heading. `TRUE_PSI` isn't in our set of
    // state datarefs (we read HEADING_DEG = heading_true_degmag from the
    // bridge cache), so pull that. Falls back to 0 if unavailable.
    use crate::sim::datarefs::HEADING_DEG;
    bridge.get_dataref_value(HEADING_DEG.name).unwrap_or(0.0)
}

/// Heading in degrees for the plan-only tools, which may or may not have
/// a bridge. Preference: explicit `start_heading_deg` arg → bridge → 0.
/// With heading = 0 and no runway directly north-south, the
/// `forbid_backward_runway_traversals` output may not match the aircraft's
/// real orientation, but `plan_taxi_route`/`plan_park_route` are previews
/// only — the real engage-* path reads bridge heading directly.
fn bridge_heading_or_arg(ctx: &ToolContext, args: &Map<String, Value>) -> f64 {
    if let Some(h) = args.get("start_heading_deg").and_then(|v| v.as_f64()) {
        return h;
    }
    if let Some(b) = ctx.bridge.as_ref() {
        return start_heading_deg(b);
    }
    0.0
}

fn format_runway_taxi_plan_error(e: impl std::fmt::Display) -> String {
    let msg = e.to_string();
    if msg.contains("no taxi route") {
        format!(
            "error: {} — hint: engage_taxi and plan_taxi_route route only to a runway hold-short (outbound). To reach a parking spot from your current position (e.g. after landing), use engage_park instead.",
            msg
        )
    } else {
        format!("error: {}", msg)
    }
}

pub fn tool_engage_taxi(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let bridge = match ctx.bridge.as_ref() {
        Some(b) => b.clone(),
        None => {
            return "error: aircraft systems bridge unavailable; engage_taxi needs the georef".to_string();
        }
    };
    let airport = match arg_str(args, "airport_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let destination_runway = match arg_str(args, "destination_runway") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let via_taxiways = arg_string_array(args, "via_taxiways");
    let intersection = args
        .get("intersection")
        .and_then(|v| v.as_str())
        .map(str::to_string);

    let (start_lat, start_lon) = match resolve_engage_start_latlon(ctx, args) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };

    if let Err(e) = ensure_runway_conn(ctx) {
        return format!("error: {}", e);
    }
    let aircraft_heading_deg = start_heading_deg(&bridge);
    let goal = TaxiGoal::Runway(match intersection.as_deref() {
        Some(taxiway) => TaxiDestination::RunwayIntersection { runway: &destination_runway, taxiway },
        None => TaxiDestination::Runway(&destination_runway),
    });
    let start = TaxiStart { lat: start_lat, lon: start_lon, heading_deg: aircraft_heading_deg };
    let PlannedTaxi { plan, face_toward_latlon, .. } = {
        let guard = ctx.runway_conn.lock().unwrap();
        let conn = guard.as_ref().unwrap();
        match taxi_graph(ctx, conn, &airport)
            .and_then(|graph| taxi_route::plan_from(conn, &graph, start, &goal, &via_taxiways))
        {
            Ok(p) => p,
            Err(e) => return format_runway_taxi_plan_error(e),
        }
    };

    let georef = bridge.georef();
    let aircraft_pos_ft = geodetic_offset_ft(start_lat, start_lon, georef);
    let TaxiLegsPlan { legs: legs_ft, names, pullout_ft } =
        build_taxi_legs_with_pullout(aircraft_pos_ft, &plan.legs, georef);
    // Terminal pose target: stop at the hold-short node aligned with
    // the last leg's direction. Do NOT aim the aircraft at the runway
    // (via `face_toward_latlon`): when the approach heading differs
    // from the face-runway heading by more than ~90°, the nose-wheel
    // controller can't rotate a stationary aircraft, so the pose
    // phase oscillates forever and the profile never completes. The
    // turn onto the runway happens during engage_line_up. We still
    // want a pose (rather than coasting past the last leg) so the
    // aircraft stops *at* the hold-short rather than rolling through
    // it. Log the runway-facing heading for diagnostics.
    let mut final_pose: Option<crate::types::TaxiPose> = None;
    let mut face_runway_note: Option<String> = None;
    if let Some(last) = legs_ft.last().copied() {
        let approach = last.end_ft - last.start_ft;
        if approach.length() > 1.0 {
            let approach_heading = crate::types::vector_to_heading(approach);
            final_pose = Some(crate::types::TaxiPose {
                position_ft: last.end_ft,
                heading_deg: approach_heading,
            });
            if let Some((flat, flon)) = face_toward_latlon {
                let runway_point_ft = geodetic_offset_ft(flat, flon, georef);
                let to_runway = runway_point_ft - last.end_ft;
                if to_runway.length() > 1.0 {
                    let runway_heading = crate::types::vector_to_heading(to_runway);
                    let delta = crate::types::wrap_degrees_180(
                        runway_heading - approach_heading,
                    )
                    .abs();
                    face_runway_note = Some(format!(
                        "approach heading {:.0}° (runway bearing {:.0}°, Δ={:.0}°)",
                        approach_heading, runway_heading, delta
                    ));
                }
            }
        }
    }
    let pose_appended = final_pose.is_some();
    let leg_count = legs_ft.len();
    let mut profile = TaxiProfile::new(legs_ft, names);
    if let Some(pose) = final_pose {
        profile = profile.with_final_pose(pose);
    }
    let displaced = match engage(ctx, Box::new(profile)) {
        Ok(d) => d,
        Err(e) => return e,
    };
    // Release the parking brake only once engaged: a planning error or a
    // refused engage must not leave a parked aircraft with no brakes.
    let released_brake_ratio = match release_parking_brake_if_set(ctx, &bridge) {
        Ok(v) => v,
        Err(e) => return format!("error: taxi engaged, but {e}"),
    };

    let mut summary = format!(
        "engaged taxi {} — {} legs, {:.0} m",
        plan.airport_ident, leg_count, plan.total_distance_m
    );
    if let Some(twy) = &intersection {
        summary.push_str(&format!(
            " (to hold short of {} at {})",
            destination_runway, twy
        ));
    }
    if let Some(v) = released_brake_ratio {
        summary.push_str(&format!(" (released parking brake, ratio was {:.2})", v));
    }
    if let Some(d) = pullout_ft {
        summary.push_str(&format!(" (pullout: {:.0} ft from start)", d));
    }
    if pose_appended {
        summary.push_str(" +pose-target");
        if let Some(note) = face_runway_note {
            summary.push_str(&format!(" ({note})"));
        }
    }
    if !plan.taxiway_sequence.is_empty() {
        summary.push_str(&format!(" via {}", plan.taxiway_sequence.join(" -> ")));
    }
    if let Some(crossings) = format_unique_crossings(&plan.runway_crossings) {
        summary.push_str(&format!(" (crossings: {})", crossings));
    }
    summary.push_str(&displaced);
    summary
}

pub fn tool_plan_taxi_route(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let airport = match arg_str(args, "airport_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let destination_runway = match arg_str(args, "destination_runway") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let via_taxiways = arg_string_array(args, "via_taxiways");
    let (start_lat, start_lon) = match resolve_start_latlon(ctx, args) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };

    if let Err(e) = ensure_runway_conn(ctx) {
        return format!("error: {}", e);
    }
    let guard = ctx.runway_conn.lock().unwrap();
    let conn = guard.as_ref().unwrap();

    let start = TaxiStart { lat: start_lat, lon: start_lon, heading_deg: bridge_heading_or_arg(ctx, args) };
    let goal = TaxiGoal::Runway(TaxiDestination::Runway(&destination_runway));
    let plan = match taxi_graph(ctx, conn, &airport)
        .and_then(|graph| taxi_route::plan_from(conn, &graph, start, &goal, &via_taxiways))
    {
        Ok(p) => p.plan,
        Err(e) => return format_runway_taxi_plan_error(e),
    };
    format_taxi_plan(&plan)
}

pub fn tool_plan_park_route(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let airport = match arg_str(args, "airport_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let parking_name = match arg_str(args, "parking_name") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let via_taxiways = arg_string_array(args, "via_taxiways");
    let (start_lat, start_lon) = match resolve_start_latlon(ctx, args) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };

    if let Err(e) = ensure_runway_conn(ctx) {
        return format!("error: {}", e);
    }
    let guard = ctx.runway_conn.lock().unwrap();
    let conn = guard.as_ref().unwrap();
    let start = TaxiStart { lat: start_lat, lon: start_lon, heading_deg: bridge_heading_or_arg(ctx, args) };
    let PlannedTaxi { plan, parking, .. } = match taxi_graph(ctx, conn, &airport).and_then(|graph| {
        taxi_route::plan_from(conn, &graph, start, &TaxiGoal::Parking(&parking_name), &via_taxiways)
    }) {
        Ok(p) => p,
        Err(e) => return format!("error: {}", e),
    };
    let resolved = parking.expect("parking goal resolves a spot");
    let mut out = format_taxi_plan(&plan);
    out.push_str(&format!(
        "  parking: {:?} ({}, {:.6},{:.6}, heading {:.0}°T)\n",
        resolved.spot_name,
        resolved.spot_kind,
        resolved.spot_lat,
        resolved.spot_lon,
        resolved.spot_heading_true_deg,
    ));
    out
}

pub fn tool_engage_park(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    // Already taxiing there (e.g. the arrival taxi from plan_arrival)?
    // Re-planning would just restart the same route, so leave it be.
    if let Ok(requested) = arg_str(args, "parking_name") {
        let current = ctx.pilot.lock().profile_mut::<TaxiProfile>().map(|t| {
            t.destination
                .clone()
                .filter(|d| d.eq_ignore_ascii_case(requested))
                .map(|d| (d, t.finished))
        });
        match current {
            Some(Some((dest, true))) => {
                return format!("already parked at {dest:?}; no change.");
            }
            Some(Some((dest, false))) => {
                return format!(
                    "already taxiing to {dest:?} (the active taxi profile); no change. Its progress is in profile_details."
                );
            }
            _ => {}
        }
    }
    let bridge = match ctx.bridge.as_ref() {
        Some(b) => b.clone(),
        None => {
            return "error: aircraft systems bridge unavailable; engage_park needs the georef".to_string();
        }
    };
    let airport = match arg_str(args, "airport_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let parking_name = match arg_str(args, "parking_name") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let via_taxiways = arg_string_array(args, "via_taxiways");
    let (start_lat, start_lon) = match resolve_engage_start_latlon(ctx, args) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    let route = match build_park_route(
        ctx,
        &bridge,
        &airport,
        &parking_name,
        &via_taxiways,
        start_lat,
        start_lon,
        start_heading_deg(&bridge),
    ) {
        Ok(r) => r,
        Err(e) => return format!("error: {e}"),
    };
    let displaced = match engage(ctx, Box::new(route.profile)) {
        Ok(d) => d,
        Err(e) => return e,
    };
    // Release the parking brake only once engaged: a planning error or a
    // refused engage must not leave a parked aircraft with no brakes.
    let released_brake_ratio = match release_parking_brake_if_set(ctx, &bridge) {
        Ok(v) => v,
        Err(e) => return format!("error: park engaged, but {e}"),
    };

    let mut summary = format!("engaged park {} → {}", airport, route.headline);
    if let Some(v) = released_brake_ratio {
        summary.push_str(&format!(" (released parking brake, ratio was {:.2})", v));
    }
    summary.push_str(&route.details);
    summary.push_str(&displaced);
    summary
}

/// A planned taxi to a parking spot, ready to engage.
struct ParkRoute {
    profile: TaxiProfile,
    /// `"Spot" (kind) — N legs, M m`
    headline: String,
    /// Pullout, pose, via and crossing notes, each with a leading space.
    details: String,
}

/// Plan a taxi from (`start_lat`, `start_lon`) heading `start_heading_deg`
/// to `parking_name`, ending in a pose aligned with the spot's painted
/// heading. Shared by engage_park (from the aircraft's position) and
/// plan_arrival (from the end of a runway exit, before touchdown).
#[allow(clippy::too_many_arguments)]
fn build_park_route(
    ctx: &ToolContext,
    bridge: &Arc<dyn ToolBridge>,
    airport: &str,
    parking_name: &str,
    via_taxiways: &[String],
    start_lat: f64,
    start_lon: f64,
    start_heading_deg: f64,
) -> std::result::Result<ParkRoute, String> {
    ensure_runway_conn(ctx).map_err(|e| e.to_string())?;
    let start = TaxiStart { lat: start_lat, lon: start_lon, heading_deg: start_heading_deg };
    let PlannedTaxi { plan, parking, .. } = {
        let guard = ctx.runway_conn.lock().unwrap();
        let conn = guard.as_ref().unwrap();
        let graph = taxi_graph(ctx, conn, airport).map_err(|e| e.to_string())?;
        taxi_route::plan_from(conn, &graph, start, &TaxiGoal::Parking(parking_name), via_taxiways)
            .map_err(|e| e.to_string())?
    };
    let resolved = parking.expect("parking goal resolves a spot");

    let georef = bridge.georef();
    let start_pos_ft = geodetic_offset_ft(start_lat, start_lon, georef);
    let TaxiLegsPlan { legs: mut legs_ft, mut names, pullout_ft } =
        build_taxi_legs_with_pullout(start_pos_ft, &plan.legs, georef);

    // Parking spots sit off the network: add a leg from the last taxi
    // node toward the 1300 spot, or the aircraft would stop at the node,
    // typically 30-60 ft short. It ends at a lead-in point back along the
    // spot's heading when the turn onto the spot needs room, so the pose
    // phase rolls in along the heading instead of arriving sideways.
    let spot_pos_ft = geodetic_offset_ft(resolved.spot_lat, resolved.spot_lon, georef);
    let final_pose = crate::types::TaxiPose {
        position_ft: spot_pos_ft,
        heading_deg: resolved.spot_heading_true_deg,
    };
    if let Some(last) = legs_ft.last().copied() {
        let turn_radius_ft = ctx.config.lock().ground_handling.min_turn_radius_ft();
        let end_ft = TaxiProfile::pose_lead_in_point(last.end_ft, final_pose, turn_radius_ft)
            .unwrap_or(spot_pos_ft);
        if (end_ft - last.end_ft).length() > 1.0 {
            legs_ft.push(StraightLeg { start_ft: last.end_ft, end_ft });
            names.push(format!("(to {})", resolved.spot_name));
        }
    }
    let leg_count = legs_ft.len();
    let profile = TaxiProfile::new(legs_ft, names)
        .with_final_pose(final_pose)
        .parking_at(resolved.spot_name.clone());

    let headline = format!(
        "{:?} ({}) — {} legs, {:.0} m",
        resolved.spot_name, resolved.spot_kind, leg_count, plan.total_distance_m
    );
    let mut details = String::new();
    if let Some(d) = pullout_ft {
        details.push_str(&format!(" (pullout: {:.0} ft from start)", d));
    }
    details.push_str(&format!(
        " +pose-target (heading {:.0}°T)",
        resolved.spot_heading_true_deg
    ));
    if !plan.taxiway_sequence.is_empty() {
        details.push_str(&format!(" via {}", plan.taxiway_sequence.join(" -> ")));
    }
    if let Some(crossings) = format_unique_crossings(&plan.runway_crossings) {
        details.push_str(&format!(" (crossings: {})", crossings));
    }
    Ok(ParkRoute { profile, headline, details })
}

pub fn tool_choose_runway_exit(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let taxiway = match args.get("taxiway_name") {
        Some(Value::Null) | None => None,
        Some(Value::String(s)) => Some(s.clone()),
        Some(other) => return format!("error: taxiway_name must be a string or null, got {other}"),
    };

    // Must have a plan flying a pattern (before touchdown) or landing_rollout (after)
    // engaged to set a preference. Check first so the error message is
    // unambiguous (validation-before-engage would give confusing "runway
    // not found" errors for a pilot who hasn't even engaged a profile yet).
    let Some((touchdown_x, arrival_request)) = ctx.pilot.lock().landing_plan_mut().map(|h| {
        (
            h.runway_frame().touchdown_runway_x_ft(),
            h.arrival().map(|a| (a.parking_name.clone(), a.via_taxiways.clone())),
        )
    }) else {
        return NO_PATTERN.to_string();
    };

    // Resolve the selected taxi edge into its exact graph path now;
    // the rollout needs it to fly the exit autonomously.
    let mut exit_plan = None;
    let mut arrival = None;
    let mut note = String::new();
    if let Some(name) = &taxiway {
        let (airport, runway) = match active_runway(ctx) {
            Ok(v) => v,
            Err(e) => return format!("error: {e}"),
        };
        let detail = match runway_exits_detail(ctx, &airport, &runway) {
            Ok(v) => v,
            Err(e) => return format!("error: cannot resolve runway exit geometry: {e}"),
        };
        let earliest_x = touchdown_x.max(ground_stationing_ft(ctx, &detail).unwrap_or(f64::NEG_INFINITY));
        let exit = match select_named_exit(&detail, name, earliest_x) {
            Ok(e) => e,
            Err(e) => return format!("error: {e} at {airport} runway {runway}"),
        };
        note = format!(" (intersection at {:.0} ft)", exit.stationing_ft);
        // An arrival plan's taxi route starts where the old exit ended;
        // re-plan it from the new one.
        if let Some((parking, via)) = &arrival_request {
            match arrival_from_exit(ctx, &airport, exit, parking, via) {
                Ok(a) => {
                    note.push_str(&format!("; arrival re-planned: taxi to {}", a.summary));
                    arrival = Some(a);
                }
                Err(e) => return format!("error: can't re-plan the arrival taxi from {name}: {e}"),
            }
        }
        exit_plan = Some(exit.plan());
    } else if let Some((parking, _)) = &arrival_request {
        note = format!("; arrival plan to {parking} dropped (no exit to taxi from)");
    }

    let display = match &taxiway {
        Some(t) => format!("preferred runway exit set to {:?}{note}", t),
        None => format!("preferred runway exit cleared{note}"),
    };
    match ctx.pilot.lock().landing_plan_mut().map(|h| h.set_landing_plan(exit_plan, arrival)) {
        Some(Ok(())) => display,
        Some(Err(e)) => format!("error: {e}"),
        None => NO_PATTERN.to_string(),
    }
}

pub fn tool_plan_arrival(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let parking_name = match arg_str(args, "parking_name") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let exit_name = match args.get("exit_taxiway") {
        Some(Value::Null) | None => None,
        Some(Value::String(s)) if !s.is_empty() => Some(s.clone()),
        Some(Value::String(_)) => None,
        Some(other) => return format!("error: exit_taxiway must be a string or null, got {other}"),
    };
    let via_taxiways = arg_string_array(args, "via_taxiways");

    if landed(ctx) {
        return "error: already on the ground after landing; use choose_runway_exit for an exit ahead, or engage_park once stopped".to_string();
    }
    let Some(touchdown_x) = ctx
        .pilot
        .lock()
        .landing_plan_mut()
        .map(|h| h.runway_frame().touchdown_runway_x_ft())
    else {
        return "error: no pattern is being flown; plan_arrival is for an upcoming landing (fly a plan with join(p) first)".to_string();
    };
    let (airport, runway) = match active_runway(ctx) {
        Ok(v) => v,
        Err(e) => return format!("error: {e}"),
    };
    let detail = match runway_exits_detail(ctx, &airport, &runway) {
        Ok(v) => v,
        Err(e) => return format!("error: cannot resolve runway exit geometry: {e}"),
    };
    let aircraft_x = ground_stationing_ft(ctx, &detail).unwrap_or(f64::NEG_INFINITY);
    let exit = match &exit_name {
        Some(name) => match select_named_exit(&detail, name, touchdown_x.max(aircraft_x)) {
            Ok(e) => e,
            Err(e) => return format!("error: {e} at {airport} runway {runway}"),
        },
        None => {
            let min_rollout_ft = ctx.config.lock().post_landing.min_rollout_past_touchdown_ft;
            let earliest_x = (touchdown_x + min_rollout_ft).max(aircraft_x);
            match detail.exits.iter().find(|e| e.stationing_ft >= earliest_x) {
                Some(e) => e,
                None => {
                    return format!(
                        "error: no exit off {airport} runway {runway} at least {earliest_x:.0} ft from the threshold ({min_rollout_ft:.0} ft past touchdown); pass exit_taxiway (see list_runway_exits)"
                    )
                }
            }
        }
    };
    let arrival = match arrival_from_exit(ctx, &airport, exit, &parking_name, &via_taxiways) {
        Ok(a) => a,
        Err(e) => return format!("error: {e}"),
    };
    let reply = format!(
        "arrival planned: exit {} (intersection at {:.0} ft{}), then taxi to {}. After touchdown the aircraft rolls out, takes the exit, and taxis to parking on its own; if the exit is missed it stops on the runway and you're woken (completed: landing_rollout) to choose an exit ahead or engage_park.",
        exit.name,
        exit.stationing_ft,
        if exit_name.is_none() { ", auto-selected" } else { "" },
        arrival.summary,
    );
    let plan = exit.plan();
    match ctx.pilot.lock().landing_plan_mut().map(|h| h.set_landing_plan(Some(plan), Some(arrival))) {
        Some(Ok(())) => reply,
        Some(Err(e)) => format!("error: {e}"),
        None => NO_PATTERN.to_string(),
    }
}

/// The airport and runway the active pattern is anchored to.
fn active_runway(ctx: &ToolContext) -> std::result::Result<(String, String), String> {
    let cfg = ctx.config.lock();
    match (cfg.airport.airport.clone(), cfg.airport.runway.id.clone()) {
        (Some(airport), Some(runway)) => Ok((airport, runway)),
        _ => Err("active pattern has no airport/runway identity".to_string()),
    }
}

/// Resolve a taxiway name to one of its intersections with the runway. A
/// taxiway can meet the runway more than once: take the first
/// intersection at or past `earliest_x` (the touchdown point or the
/// aircraft, whichever is farther). An exit that is entirely behind that
/// point can't be made, so it's an error naming the exits still ahead.
fn select_named_exit<'a>(
    detail: &'a RunwayExitInfo,
    name: &str,
    earliest_x: f64,
) -> std::result::Result<&'a RunwayExitCandidate, String> {
    let names_of = |exits: &mut dyn Iterator<Item = &RunwayExitCandidate>| {
        let mut names: Vec<String> = exits.map(|exit| exit.name.clone()).collect();
        names.sort();
        names.dedup();
        names
    };
    let matches: Vec<&RunwayExitCandidate> =
        detail.exits.iter().filter(|exit| exit.name == name).collect();
    if matches.is_empty() {
        let valid = names_of(&mut detail.exits.iter());
        return Err(format!("unknown exit {:?}; valid: {:?}", name, valid));
    }
    if let Some(exit) = matches.iter().find(|exit| exit.stationing_ft >= earliest_x) {
        return Ok(exit);
    }
    let ahead = names_of(&mut detail.exits.iter().filter(|e| e.stationing_ft >= earliest_x));
    Err(format!(
        "exit {:?} meets the runway only at {} ft from the threshold, behind the touchdown point / aircraft ({:.0} ft), so it can't be made; exits ahead: {:?}",
        name,
        matches
            .iter()
            .map(|e| format!("{:.0}", e.stationing_ft))
            .collect::<Vec<_>>()
            .join(", "),
        earliest_x,
        ahead
    ))
}

/// Plan the taxi from `exit`'s last node to `parking_name`.
fn arrival_from_exit(
    ctx: &ToolContext,
    airport: &str,
    exit: &RunwayExitCandidate,
    parking_name: &str,
    via_taxiways: &[String],
) -> std::result::Result<ArrivalPlan, String> {
    let bridge = ctx
        .bridge
        .as_ref()
        .ok_or("aircraft systems bridge unavailable; planning a taxi route needs the georef")?
        .clone();
    let route = build_park_route(
        ctx,
        &bridge,
        airport,
        parking_name,
        via_taxiways,
        exit.end_lat_deg,
        exit.end_lon_deg,
        exit.end_heading_deg,
    )?;
    Ok(ArrivalPlan {
        parking_name: parking_name.to_string(),
        via_taxiways: via_taxiways.to_vec(),
        summary: format!("{}{}", route.headline, route.details),
        taxi: Box::new(route.profile),
    })
}

/// Shared core of list_runway_exits / choose_runway_exit validation.
/// Returns (name, stationing_ft) pairs sorted by stationing, along with
/// the runway's overall length. Errors as a String so either caller can
/// surface it verbatim.
/// Geometry + exits packet returned by the shared runway-exit
/// helper. The caller can project an arbitrary lat/lon onto the runway
/// centerline (via `geodetic_offset_ft(.., georef).dot(forward)`) to
/// compare positions against the exit stationings.
#[derive(Debug, Clone)]
struct RunwayExitCandidate {
    name: String,
    stationing_ft: f64,
    path_runway_ft: Vec<Vec2>,
    /// Last node of the exit (first node clear of the active zone) and
    /// the heading the aircraft has when it gets there.
    end_lat_deg: f64,
    end_lon_deg: f64,
    end_heading_deg: f64,
}

impl RunwayExitCandidate {
    fn plan(&self) -> RunwayExitPlan {
        RunwayExitPlan {
            taxiway_name: self.name.clone(),
            stationing_ft: self.stationing_ft,
            path_runway_ft: self.path_runway_ft.clone(),
        }
    }
}

struct RunwayExitInfo {
    length_ft: f64,
    georef: GeoReference,
    forward: Vec2,
    exits: Vec<RunwayExitCandidate>,
}

fn runway_exits_detail(
    ctx: &ToolContext,
    airport: &str,
    runway: &str,
) -> std::result::Result<RunwayExitInfo, String> {
    if let Err(e) = ensure_runway_conn(ctx) {
        return Err(e.to_string());
    }
    let guard = ctx.runway_conn.lock().unwrap();
    let conn = guard.as_ref().unwrap();

    let forms = taxi_route::runway_query_forms(runway);
    type RunwayRow = (f64, f64, f64, f64, f64, f64, f64, f64);
    let thresh: Result<RunwayRow, _> = conn.query_row(
        "SELECT \
             CASE WHEN le_ident = ? OR le_ident = ? THEN le_latitude_deg  ELSE he_latitude_deg  END, \
             CASE WHEN le_ident = ? OR le_ident = ? THEN le_longitude_deg ELSE he_longitude_deg END, \
             CASE WHEN le_ident = ? OR le_ident = ? THEN le_heading_degT  ELSE he_heading_degT  END, \
             length_ft, \
             le_latitude_deg, le_longitude_deg, he_latitude_deg, he_longitude_deg \
         FROM runways \
         WHERE airport_ident = ? \
           AND (le_ident = ? OR le_ident = ? OR he_ident = ? OR he_ident = ?) \
           AND closed = 0 \
         LIMIT 1",
        [
            forms[0].as_str(), forms[1].as_str(),
            forms[0].as_str(), forms[1].as_str(),
            forms[0].as_str(), forms[1].as_str(),
            airport,
            forms[0].as_str(), forms[1].as_str(), forms[0].as_str(), forms[1].as_str(),
        ],
        |r| {
            Ok((
                r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?,
                r.get(4)?, r.get(5)?, r.get(6)?, r.get(7)?,
            ))
        },
    );
    let (thr_lat, thr_lon, course, length_ft, le_lat, le_lon, he_lat, he_lon) = match thresh {
        Ok(v) => v,
        Err(e) => return Err(format!("runway {:?} not found at {}: {}", runway, airport, e)),
    };

    let georef = GeoReference {
        threshold_lat_deg: thr_lat,
        threshold_lon_deg: thr_lon,
    };
    let forward = heading_to_vector(course, 1.0);
    let right = heading_to_vector(course + 90.0, 1.0);

    let graph = match taxi_graph(ctx, conn, airport) {
        Ok(g) => g,
        Err(e) => return Err(e.to_string()),
    };
    let mut exits = Vec::new();
    for path in taxi_route::runway_exit_paths(&graph, runway, (le_lat, le_lon), (he_lat, he_lon)) {
        let path_runway_ft: Vec<Vec2> = path
            .nodes
            .iter()
            .map(|node| {
                let offset = geodetic_offset_ft(node.latitude_deg, node.longitude_deg, georef);
                Vec2::new(offset.dot(forward), offset.dot(right))
            })
            .collect();
        let Some(first) = path_runway_ft.first() else { continue };
        if first.x < 0.0 || first.x > length_ft + 500.0 {
            continue;
        }
        let [.., prev, last] = path.nodes.as_slice() else { continue };
        exits.push(RunwayExitCandidate {
            name: path.taxiway_name.clone(),
            stationing_ft: first.x,
            path_runway_ft,
            end_lat_deg: last.latitude_deg,
            end_lon_deg: last.longitude_deg,
            end_heading_deg: crate::types::initial_bearing_deg(
                prev.latitude_deg,
                prev.longitude_deg,
                last.latitude_deg,
                last.longitude_deg,
            ),
        });
    }
    exits.sort_by(|a, b| {
        a.stationing_ft
            .partial_cmp(&b.stationing_ft)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    Ok(RunwayExitInfo {
        length_ft,
        georef,
        forward,
        exits,
    })
}

/// The aircraft's runway stationing, but only once it's rolling on the
/// ground. Airborne in the pattern its projection onto the runway line
/// (e.g. abeam the far end on downwind) says nothing about which exits it
/// can still make.
fn ground_stationing_ft(ctx: &ToolContext, detail: &RunwayExitInfo) -> Option<f64> {
    let on_ground = ctx
        .pilot
        .lock()
        .latest_snapshot
        .as_ref()
        .is_some_and(|s| s.state.on_ground);
    if on_ground { aircraft_stationing_ft(ctx, detail) } else { None }
}

/// The aircraft's current position projected onto the runway centerline,
/// in feet from the landing threshold. `None` without a live bridge.
fn aircraft_stationing_ft(ctx: &ToolContext, detail: &RunwayExitInfo) -> Option<f64> {
    let (lat, lon) = live_latlon(ctx)?;
    let p = geodetic_offset_ft(lat, lon, detail.georef);
    Some(p.x * detail.forward.x + p.y * detail.forward.y)
}

pub fn tool_list_runway_exits(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let airport = match arg_str(args, "airport_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let runway = match arg_str(args, "runway_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let detail = match runway_exits_detail(ctx, &airport, &runway) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    if detail.exits.is_empty() {
        return format!(
            "no taxiway exits annotated for runway {} at {} in apt.dat",
            runway, airport
        );
    }

    // Project the aircraft's current lat/lon onto the runway centerline
    // to annotate each exit with its distance relative to where the
    // aircraft is right now. Without this the LLM has no signal for
    // which exits are ahead vs already passed, and has been observed
    // picking exits that were already behind the aircraft.
    let current_st = aircraft_stationing_ft(ctx, &detail);

    let mut out = format!(
        "exits for {} runway {} (length {:.0} ft, threshold at 0):\n",
        airport, runway, detail.length_ft
    );
    if let Some(cur) = current_st {
        out.push_str(&format!(
            "aircraft currently at {:.0} ft along-track on runway centerline.\n",
            cur
        ));
    }
    for exit in &detail.exits {
        let name = &exit.name;
        let st = exit.stationing_ft;
        match current_st {
            Some(cur) => {
                let delta = st - cur;
                let note = if delta > 50.0 {
                    format!("{:.0} ft ahead", delta)
                } else if delta < -50.0 {
                    format!("{:.0} ft behind — already passed", -delta)
                } else {
                    "abeam".to_string()
                };
                out.push_str(&format!("  {}\t{:.0} ft\t({})\n", name, st, note));
            }
            None => out.push_str(&format!("  {}\t{:.0} ft\n", name, st)),
        }
    }
    out
}

/// Start position for a tool that *engages* a ground profile. The profile
/// drives from wherever the aircraft really is, so an explicit
/// `start_lat`/`start_lon` far from the live position (e.g. a node ahead,
/// to dodge a "behind you" error) would leave it chasing a leg it isn't
/// on. Refuse those.
fn resolve_engage_start_latlon(
    ctx: &ToolContext,
    args: &Map<String, Value>,
) -> Result<(f64, f64)> {
    const MAX_START_OFFSET_FT: f64 = 150.0;
    let (lat, lon) = resolve_start_latlon(ctx, args)?;
    if let Some((live_lat, live_lon)) = live_latlon(ctx) {
        let off_ft = crate::types::haversine_m(lat, lon, live_lat, live_lon) * M_TO_FT;
        if off_ft > MAX_START_OFFSET_FT {
            return Err(anyhow!(
                "start_lat/start_lon is {off_ft:.0} ft from the aircraft's actual position; this taxis from where the aircraft really is, so omit start_lat/start_lon (they're for plan_* previews)"
            ));
        }
    }
    Ok((lat, lon))
}

fn resolve_start_latlon(
    ctx: &ToolContext,
    args: &Map<String, Value>,
) -> Result<(f64, f64)> {
    let lat = args.get("start_lat").and_then(|v| v.as_f64());
    let lon = args.get("start_lon").and_then(|v| v.as_f64());
    match (lat, lon) {
        (Some(a), Some(b)) => Ok((a, b)),
        _ => live_latlon(ctx).ok_or_else(|| {
            anyhow!("aircraft position unavailable; pass start_lat and start_lon, or check the aircraft systems bridge")
        }),
    }
}

/// The aircraft's live position from the bridge, if it has one yet.
fn live_latlon(ctx: &ToolContext) -> Option<(f64, f64)> {
    let b = ctx.bridge.as_deref()?;
    Some((b.get_dataref_value(LATITUDE_DEG.name)?, b.get_dataref_value(LONGITUDE_DEG.name)?))
}

fn format_taxi_plan(plan: &taxi_route::TaxiPlan) -> String {
    let mut out = String::new();
    out.push_str(&format!(
        "taxi plan {} — distance {:.0} m across {} legs\n",
        plan.airport_ident,
        plan.total_distance_m,
        plan.legs.len(),
    ));
    if !plan.taxiway_sequence.is_empty() {
        out.push_str(&format!(
            "  taxiways: {}\n",
            plan.taxiway_sequence.join(" -> ")
        ));
    }
    if let Some(crossings) = format_unique_crossings(&plan.runway_crossings) {
        out.push_str(&format!("  runway zones: {}\n", crossings));
    }
    out.push_str(&format!(
        "  start_node={} destination_node={}\n",
        plan.start_node, plan.destination_node
    ));
    out.push_str("legs (from_lat,from_lon -> to_lat,to_lon  taxiway  distance_m):\n");
    for leg in &plan.legs {
        let name = if leg.taxiway_name.is_empty() {
            "(connector)"
        } else {
            leg.taxiway_name.as_str()
        };
        out.push_str(&format!(
            "  {:.6},{:.6} -> {:.6},{:.6}  {}  {:.1}\n",
            leg.from_lat, leg.from_lon, leg.to_lat, leg.to_lon, name, leg.distance_m,
        ));
    }
    out
}

pub fn tool_sql_query(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let query = match arg_str(args, "query") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    if ctx.apt_dat_cache_dir.is_none() {
        return "error: apt.dat parquet cache is not configured (set --apt-dat-path or place apt.dat at the standard X-Plane 12 location)".to_string();
    }
    if let Err(e) = ensure_runway_conn(ctx) {
        return format!("error: could not open apt.dat parquet cache: {}", e);
    }
    let guard = ctx.runway_conn.lock().unwrap();
    let conn = guard.as_ref().unwrap();
    let mut stmt = match conn.prepare(&query) {
        Ok(s) => s,
        Err(e) => return format!("error: {}", e),
    };
    let mut rows = match stmt.query([]) {
        Ok(r) => r,
        Err(e) => return format!("error: {}", e),
    };
    // duckdb-rs requires the query to have run before column metadata is
    // available; pull it off the `Rows` handle, not the prepared statement.
    let column_count = rows.as_ref().map(|r| r.column_count()).unwrap_or(0);
    let column_names: Vec<String> = rows
        .as_ref()
        .map(|r| {
            (0..column_count)
                .map(|i| match r.column_name(i) {
                    Ok(s) => s.to_string(),
                    Err(_) => String::new(),
                })
                .collect()
        })
        .unwrap_or_default();
    let mut out: Vec<String> = Vec::new();
    let mut has_more = false;
    let mut count = 0;
    loop {
        let row = match rows.next() {
            Ok(Some(r)) => r,
            Ok(None) => break,
            Err(e) => return format!("error: {}", e),
        };
        if count >= SQL_QUERY_MAX_ROWS {
            has_more = true;
            break;
        }
        let mut fields: Vec<String> = Vec::with_capacity(column_count);
        for i in 0..column_count {
            let value: duckdb::types::Value = match row.get(i) {
                Ok(v) => v,
                Err(e) => return format!("error: {}", e),
            };
            fields.push(duckdb_value_to_str(&value));
        }
        out.push(fields.join("\t"));
        count += 1;
    }
    if out.is_empty() {
        return "0 rows".to_string();
    }
    let mut lines = vec![column_names.join("\t")];
    lines.extend(out);
    if has_more {
        lines.push(format!(
            "(truncated at {} rows; add LIMIT/WHERE to narrow)",
            SQL_QUERY_MAX_ROWS
        ));
    }
    lines.join("\n")
}

fn duckdb_value_to_str(value: &duckdb::types::Value) -> String {
    use duckdb::types::Value as V;
    match value {
        V::Null => "".to_string(),
        V::Boolean(b) => b.to_string(),
        V::TinyInt(v) => v.to_string(),
        V::SmallInt(v) => v.to_string(),
        V::Int(v) => v.to_string(),
        V::BigInt(v) => v.to_string(),
        V::HugeInt(v) => v.to_string(),
        V::UTinyInt(v) => v.to_string(),
        V::USmallInt(v) => v.to_string(),
        V::UInt(v) => v.to_string(),
        V::UBigInt(v) => v.to_string(),
        V::Float(v) => v.to_string(),
        V::Double(v) => v.to_string(),
        V::Text(s) => s.clone(),
        V::Blob(b) => format!("<blob {} bytes>", b.len()),
        other => format!("{:?}", other),
    }
}

// ---------- mission intent / clearance (TUI transparency) ----------

pub fn tool_set_goal(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let kind = match arg_str(args, "kind") {
        Ok(s) => s.trim().to_string(),
        Err(e) => return format!("error: {}", e),
    };
    if kind.is_empty() {
        return "error: kind must be a non-empty short label (e.g. 'land', 'taxi', 'cruise')".to_string();
    }
    let target = args
        .get("target")
        .and_then(|v| v.as_str())
        .map(|s| s.trim())
        .filter(|s| !s.is_empty())
        .map(str::to_string);
    let notes = args
        .get("notes")
        .and_then(|v| v.as_str())
        .map(|s| s.trim())
        .filter(|s| !s.is_empty())
        .map(str::to_string);
    let goal = MissionGoal {
        kind: kind.clone(),
        target: target.clone(),
        notes: notes.clone(),
    };
    ctx.pilot.lock().set_mission_goal(goal);
    let target_part = target
        .as_deref()
        .map(|t| format!(" target={}", t))
        .unwrap_or_default();
    let notes_part = notes
        .as_deref()
        .map(|n| format!(" notes={:?}", n))
        .unwrap_or_default();
    format!("goal recorded: kind={}{}{}", kind, target_part, notes_part)
}

pub fn tool_clear_goal(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    ctx.pilot.lock().clear_mission_goal();
    "goal cleared".to_string()
}

pub fn tool_record_clearance(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let text = match arg_str(args, "text") {
        Ok(s) => s.trim().to_string(),
        Err(e) => return format!("error: {}", e),
    };
    if text.is_empty() {
        return "error: text must be a non-empty clearance description".to_string();
    }
    let source = args
        .get("source")
        .and_then(|v| v.as_str())
        .map(ClearanceSource::from_str)
        .unwrap_or(ClearanceSource::Tower);
    let freq_mhz = args.get("freq_mhz").and_then(|v| v.as_f64());
    let clearance = ActiveClearance {
        text: text.clone(),
        source: source.clone(),
        freq_mhz,
        issued_at: chrono::Utc::now(),
    };
    ctx.pilot.lock().record_clearance(clearance);
    let freq_part = freq_mhz
        .map(|f| format!(" freq={:.3}", f))
        .unwrap_or_default();
    format!(
        "clearance recorded: source={}{} text={:?}",
        source.label(),
        freq_part,
        text,
    )
}

pub fn tool_clear_clearance(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    ctx.pilot.lock().clear_active_clearance();
    "clearance cleared".to_string()
}

pub fn tool_mission_complete(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let Some(tx) = ctx.mission_complete_tx.as_ref() else {
        return "error: mission_complete is only available in the eval harness".to_string();
    };
    let success = match arg_bool(args, "success") {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    let summary = match arg_str(args, "summary") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    try_send_outcome(tx, MissionOutcome::from_llm(success, summary.clone()));
    format!(
        "mission_complete recorded (success={}). The harness will shut down after the current turn.",
        success,
    )
}

// ---------- dispatch ----------

pub fn dispatch_tool(name: &str, arguments: &str, ctx: &ToolContext) -> ToolResult {
    if name.is_empty() {
        return ToolResult::from_output("error: tool call missing name".to_string());
    }
    let args_json = if arguments.is_empty() { "{}" } else { arguments };
    let args: Value = match serde_json::from_str(args_json) {
        Ok(v) => v,
        Err(e) => {
            return ToolResult::from_output(format!("error: invalid arguments JSON: {}", e));
        }
    };
    let Value::Object(map) = args else {
        return ToolResult::from_output("error: arguments must be an object".to_string());
    };
    let output = match name {
        "fly" => tool_fly(ctx, &map),
        "get_status" => tool_get_status(ctx, &map),
        "sleep" => tool_sleep(ctx, &map),
        "find_dead_stick_candidates" => tool_find_dead_stick_candidates(ctx, &map),
        "engage_dead_stick_landing" => tool_engage_dead_stick_landing(ctx, &map),
        "takeoff_checklist" => tool_takeoff_checklist(ctx, &map),
        "disengage_profile" => tool_disengage_profile(ctx, &map),
        "list_profiles" => tool_list_profiles(ctx, &map),
        "tune_radio" => tool_tune_radio(ctx, &map),
        "broadcast_on_radio" => tool_broadcast_on_radio(ctx, &map),
        "set_parking_brake" => tool_set_parking_brake(ctx, &map),
        "set_flaps" => tool_set_flaps(ctx, &map),
        "sql_query" => tool_sql_query(ctx, &map),
        "plan_taxi_route" => tool_plan_taxi_route(ctx, &map),
        "engage_taxi" => tool_engage_taxi(ctx, &map),
        "plan_park_route" => tool_plan_park_route(ctx, &map),
        "engage_park" => tool_engage_park(ctx, &map),
        "choose_runway_exit" => tool_choose_runway_exit(ctx, &map),
        "plan_arrival" => tool_plan_arrival(ctx, &map),
        "list_runway_exits" => tool_list_runway_exits(ctx, &map),
        "engage_line_up" => tool_engage_line_up(ctx, &map),
        "set_goal" => tool_set_goal(ctx, &map),
        "clear_goal" => tool_clear_goal(ctx, &map),
        "record_clearance" => tool_record_clearance(ctx, &map),
        "clear_clearance" => tool_clear_clearance(ctx, &map),
        "mission_complete" => tool_mission_complete(ctx, &map),
        other => format!("error: unknown tool {:?}", other),
    };
    ToolResult::from_output(output)
}

/// Tools that query, acknowledge or wait rather than act: the operator's
/// compact log leaves them out unless they fail.
const QUIET_TOOLS: &[&str] = &[
    "sleep",
    "get_status",
    "sql_query",
    "set_goal",
    "clear_goal",
    "record_clearance",
    "clear_clearance",
    "list_profiles",
    "list_runway_exits",
    "plan_taxi_route",
    "plan_park_route",
    "takeoff_checklist",
    "tune_radio",
    // The radio pane shows transmissions.
    "broadcast_on_radio",
];

pub fn is_quiet_tool(name: &str) -> bool {
    QUIET_TOOLS.contains(&name)
}

// ---------- tool schemas ----------

/// How far ahead a `fly(preview=true)` looks.
const PREVIEW_HORIZON_S: f64 = 900.0;

pub const FLY_DESCRIPTION: &str = "Fly a plan described in Python with the pilot vocabulary (system prompt, \"Flying with plans\"). The code runs once and builds or amends the plan; the autopilot then flies it. A lateral step or .then() chain replaces the remaining steps; a lone climb/descend/maintain/speed applies at once; edits to `p` amend the live pattern in place; remain()/prefer()/report() add constraints and reports; release(\"base\") opens an atc(\"base\") gate. print() output comes back in the result, so `print(place(\"the VA\"))` checks a reference without changing anything. Returns the plan as the autopilot reads it (resolved places, where triggers fall relative to the runway, warnings) or an error naming the line. Completes immediately; the plan's progress arrives as heartbeats (\"plan: done: ...\", \"report due ...\", \"constraint c1 not met ...\") and in status.plan. preview=true flies the change ahead in a simple simulation from the current state (traffic moving on, no wind) and reports the timeline, touchdown, closest traffic and problems without changing anything: use it before answering \"can you ...?\" or accepting an unusual request.";

/// Build the list of tool schemas advertised to the LLM. Set
/// `include_mission_complete=true` only when running under the eval
/// harness — the live X-Plane runtime should not advertise it, since
/// there's no outcome channel to receive the result on.
pub fn tool_schemas(include_mission_complete: bool) -> Vec<ToolDef> {
    fn schema(name: &str, description: &str, properties: Value, required: &[&str]) -> ToolDef {
        ToolDef {
            name: name.to_string(),
            description: description.to_string(),
            parameters: json!({
                "type": "object",
                "properties": properties,
                "required": required,
                "additionalProperties": false,
            }),
        }
    }
    let mut schemas = vec![
        schema(
            "fly",
            FLY_DESCRIPTION,
            json!({
                "code": {
                    "type": "string",
                    "description": "Python using the pilot vocabulary (see the system prompt). Builds or amends the plan; runs once."
                },
                "airport": {
                    "type": ["string", "null"],
                    "description": "Airport ident whose runways and places the code refers to. null = the plan's airport, else the pilot's current reference airport, else the nearest airport."
                },
                "preview": {
                    "type": ["boolean", "null"],
                    "description": "true = simulate the change from now and report what would happen, without flying it. null/false = fly it."
                }
            }),
            &["code", "airport", "preview"],
        ),
        schema(
            "get_status",
            "Return a JSON snapshot of aircraft state, phase, and active profiles. Completes immediately. Heartbeat messages already embed this payload — only call get_status when you have changed state within the same turn and need fresh values.",
            json!({}),
            &[],
        ),
        schema(
            "sleep",
            "End this turn; the pilot keeps flying whatever profiles are active. You wake on operator/ATC messages, on state-change heartbeats (phase change, profile engage/disengage, `completed: taxi` / `completed: line_up`), or on the idle-cadence heartbeat (default every 30 s). Pass suppress_idle_heartbeat_s=N to skip idle heartbeats for the next N seconds when state is stable for a while (capped at 600 s; state-change heartbeats still fire immediately). Preconditions: refuses if the aircraft is moving (>0.5 kt) while only idle_* profiles are active — engage a real profile first; do not route around the refusal. Completes immediately (yields the turn).",
            json!({
                "suppress_idle_heartbeat_s": {
                    "type": ["number", "null"],
                    "description": "Optional. Suppress the idle-cadence heartbeat for this many seconds of wall time. Phase changes, profile changes, crash edges, and inbound operator/ATC messages still wake you immediately. Capped server-side at 600 s. Pass null to leave the default idle cadence unchanged."
                }
            }),
            &["suppress_idle_heartbeat_s"],
        ),
        schema(
            "find_dead_stick_candidates",
            "Find runways reachable in a still-air glide from the current position. Returns a TSV with one row per runway end (both directions of every paved-or-grass runway ≥1500 ft within glide range), scored by reach margin in NM, with optional headwind alignment if wind is known. Uses the configured aircraft glide_ratio. The top of the list is the best dead-stick choice; bias toward margin_nm > 1 NM and longer / paved surfaces, and prefer the end with the most positive headwind_kt. Call this BEFORE engage_dead_stick_landing so you (and the operator on the radio) know the options. Pass the `side` value from the chosen row straight through to engage_dead_stick_landing — it's picked so the aircraft is already on the inside of the pattern, which avoids the centerline-crossing maneuver that costs altitude in an engine-out. Ignore published traffic-side conventions for dead-stick; glide path beats procedure when there's no engine. Returns '0 reachable runways' when nothing is in glide — at that point the answer is an off-airport landing, not this tool.",
            json!({
                "lat": {"type": "number", "description": "Current latitude in degrees."},
                "lon": {"type": "number", "description": "Current longitude in degrees."},
                "alt_agl_ft": {"type": "number", "description": "Current altitude above ground level in feet (used with the configured glide_ratio to compute reach in NM)."},
                "heading_deg": {"type": "number", "description": "Current heading in degrees true. Reserved for future scoring; not used in v1."},
                "wind_dir_deg": {"type": ["number", "null"], "description": "Optional wind FROM direction in degrees true (e.g. 270 = wind from the west). Used to compute headwind alignment on each runway end. Default 0."},
                "wind_kt": {"type": ["number", "null"], "description": "Optional wind speed in knots. Default 0 (no wind weighting). Pass values from ATIS/METAR or a current AWOS broadcast."},
                "max_candidates": {"type": ["integer", "null"], "description": "Maximum rows to return. Default 5; capped at 25."}
            }),
            &["lat", "lon", "alt_agl_ft", "heading_deg", "wind_dir_deg", "wind_kt", "max_candidates"],
        ),
        schema(
            "engage_dead_stick_landing",
            "Engage the engine-out / dead-stick landing profile on the named runway. Owns all three axes. Glides at best-glide speed (vbg) toward the Low Key fix (abeam touchdown on the pattern side at ~1000 ft AGL), then runs Downwind → Base → Final → Roundout → Flare → Rollout with throttle locked at zero throughout. There is NO go-around — a powerless aircraft can't climb. Use when the operator declares engine failure or pulls power to idle and asks you to land. Refuses if the aircraft is on the ground or if the chosen runway is outside still-air glide range (use find_dead_stick_candidates to enumerate reachable options first). Bias toward candidates with margin_nm > 1 so you have buffer for headwinds and pattern maneuvering. After engaging, do NOT fly a plan — talk to ATC, brief the operator, and let the autopilot fly. At touchdown it hands off to landing_rollout, which keeps the throttle closed and brakes to a stop on the centerline.",
            json!({
                "airport_ident": {"type": "string", "description": "ICAO airport code of the chosen runway (e.g. 'KSEA')."},
                "runway_ident": {"type": "string", "description": "Runway end identifier (e.g. '16L', '34R')."},
                "side": {"type": ["string", "null"], "description": "Traffic pattern side: 'left' or 'right'. Pass the value from the corresponding row of `find_dead_stick_candidates` — it's picked to keep the aircraft on the inside of the pattern (no centerline crossing, no extra turning under glide). Defaults to 'left' if omitted, which is rarely what you want for a dead-stick."}
            }),
            &["airport_ident", "runway_ident", "side"],
        ),
        schema(
            "takeoff_checklist",
            "Return a takeoff-readiness checklist with each item marked [OK], [ACTION], [ERROR], or [REMINDER]. Reads live state (parking brake, flaps, gear, on-ground, active profiles). Completes immediately. Run before taking off and address every [ACTION] item — the most common miss is a set parking brake, which also makes a takeoff plan refuse.",
            json!({}),
            &[],
        ),
        schema(
            "disengage_profile",
            "Remove the profile with the given name; orphaned axes fall back to idle profiles. Completes immediately.",
            json!({"name": {"type": "string", "description": "Profile name, e.g. 'plan' or 'taxi'."}}),
            &["name"],
        ),
        schema(
            "list_profiles",
            "Return a comma-separated list of currently active profile names. Completes immediately.",
            json!({}),
            &[],
        ),
        schema(
            "tune_radio",
            "Tune a COM radio to a frequency in MHz. Use com1 as primary (tower, ground, CTAF, departure, approach, ATIS); com2 as monitor/secondary. Completes immediately. Call before broadcast_on_radio when switching facilities.",
            json!({
                "radio": {"type": "string", "description": "Radio name: 'com1' or 'com2'."},
                "frequency_mhz": {"type": "number", "description": "Frequency in MHz, e.g. 118.30."}
            }),
            &["radio", "frequency_mhz"],
        ),
        schema(
            "broadcast_on_radio",
            "Transmit a text message over a COM radio. This is the ONLY way your words reach ATC or anyone outside the cockpit — plain-text replies are visible to the operator only and are not transmitted. Use for every clearance acknowledgment, readback, position call, or external radio call, in standard aviation phraseology. A readback satisfies ATC; the corresponding tool call satisfies the aircraft — both are required. Completes immediately (the message is queued for transmission on the next sim tick).",
            json!({
                "radio": {"type": "string", "description": "Radio name: 'com1' or 'com2'."},
                "message": {"type": "string", "description": "The exact words to transmit, e.g. 'Seattle Tower, Cessna 123AB, runway 16L cleared for takeoff'."}
            }),
            &["radio", "message"],
        ),
        schema(
            "set_parking_brake",
            "Engage or release the parking brake. Unlike the toe brakes, the parking brake holds its state without continuous input — this is the right tool for 'set the brake and hold it'. Completes immediately. Note: engage_taxi and engage_park auto-release the brake if set, so there is no need to release it before calling those tools.",
            json!({"engaged": {"type": "boolean", "description": "True to engage (set) the parking brake, false to release it."}}),
            &["engaged"],
        ),
        schema(
            "set_flaps",
            "Set the flap handle position. Valid C172 settings: 0, 10, 20, or 30 degrees. Completes immediately. Preconditions: while a plan is flying a pattern, the pattern manages flaps per flight phase and will override this setting at its next flap change — use this tool outside the pattern or during ground ops.",
            json!({"degrees": {"type": "integer", "description": "Flap setting in degrees: 0, 10, 20, or 30."}}),
            &["degrees"],
        ),
        schema(
            "sql_query",
            SQL_QUERY_DESCRIPTION,
            json!({"query": {"type": "string", "description": "A single SQL statement to execute."}}),
            &["query"],
        ),
        schema(
            "engage_line_up",
            "Cross the hold-short onto the runway, turn to runway heading, and stop with the nose aligned for takeoff — the 'line up and wait' clearance in ATC phraseology. Displaces any active three-axis profile. Preconditions: aircraft at or near the hold-short for this runway (errors out if more than ~300 ft from the entry point). Completes when the aircraft is stopped aligned on the centerline — a state-change heartbeat fires 'completed: line_up'. Next: fly the takeoff (takeoff(rwy).then(...)). If ATC already issued the takeoff clearance ('cleared for takeoff runway X'), fly it immediately on the completed heartbeat without waiting for more operator prompts; not while active_profiles still lists line_up.",
            json!({
                "airport_ident": {
                    "type": "string",
                    "description": "Airport identifier as stored in apt.dat (e.g. 'KSFO')."
                },
                "runway_ident": {
                    "type": "string",
                    "description": "Runway to line up on. Either end ident works (e.g. '12' or '30')."
                },
                "intersection": {
                    "type": ["string", "null"],
                    "description": "Taxiway name of an intersection-departure entry point (e.g. 'C' for 'cleared for takeoff runway 19 at Charlie'). Null = line up at the runway's full-length threshold."
                },
                "start_lat": {
                    "type": ["number", "null"],
                    "description": "Starting latitude. If null, the current aircraft position is used."
                },
                "start_lon": {
                    "type": ["number", "null"],
                    "description": "Starting longitude. If null, the current aircraft position is used."
                }
            }),
            &["airport_ident", "runway_ident", "intersection", "start_lat", "start_lon"],
        ),
        schema(
            "engage_taxi",
            "Engage the ground-taxi autopilot to a runway hold-short. OUTBOUND ONLY — for taxiing from a parking spot or ramp to the hold-short before takeoff. To taxi to a parking spot after landing, use engage_park, NOT this tool with the runway you just landed on (that route does not exist by design — you don't taxi back onto the runway you rolled out of). Plans the route (same logic as plan_taxi_route), then takes control: nose-wheel steering tracks the leg centerline, ground speed targets ~15 kt on straights / ~5 kt through sharp turns / 0 at the hold-short, where the aircraft stops and holds on the toe brakes (the parking brake is not set, since line-up comes next). Displaces any active three-axis profile. Auto-releases the parking brake if set — no need to call set_parking_brake(False) first. Preconditions: aircraft on the ground. For intersection departures, pass the intersection taxiway so the planner routes to that specific hold-short; otherwise routes to the full-length threshold. Completes on arrival at the hold-short — a state-change heartbeat fires 'completed: taxi'. Next: engage_line_up. Preview a route before committing with plan_taxi_route.",
            json!({
                "airport_ident": {
                    "type": "string",
                    "description": "Airport identifier as stored in apt.dat (e.g. 'KSFO')."
                },
                "destination_runway": {
                    "type": "string",
                    "description": "Runway you want to hold short of — either end ident works (e.g. '31' or '13')."
                },
                "via_taxiways": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Ordered list of taxiway names from the clearance, e.g. ['A', 'D']. Pass [] to let the planner pick the shortest route."
                },
                "intersection": {
                    "type": ["string", "null"],
                    "description": "When ATC clears an intersection departure ('hold short of 19 at Charlie'), pass the taxiway name here (e.g. 'C'). The planner routes to the hold-short at that specific intersection instead of the full-length threshold. Null = hold short of the full-length threshold."
                },
                "start_lat": {
                    "type": ["number", "null"],
                    "description": "Starting latitude. If null, the current aircraft position is used."
                },
                "start_lon": {
                    "type": ["number", "null"],
                    "description": "Starting longitude. If null, the current aircraft position is used."
                }
            }),
            &["airport_ident", "destination_runway", "via_taxiways", "intersection", "start_lat", "start_lon"],
        ),
        schema(
            "plan_taxi_route",
            "Plan a taxi route across an airport surface. Returns the ordered waypoint legs (lat/lon pairs plus the taxiway each segment rides), total distance, and any runway active-zones the path crosses (arrival / departure / ILS-critical). PLANNING ONLY — does not engage the autopilot. Pass taxiway names in order through via_taxiways (e.g. ['A','D']); the planner runs a constrained Dijkstra that traverses them in order, allows unnamed connector edges at the start/end (for gate lead-ins and runway lead-outs), and resolves destination_runway to the taxi node nearest the runway threshold. With via_taxiways=[] returns the shortest overall route and reports which taxiways it actually used. Preconditions: airport must have a taxi network in apt.dat (small uncontrolled strips error out). Completes immediately. Next: engage_taxi to actually fly the route. Use sql_query against taxi_nodes / taxi_edges for ad-hoc exploration.",
            json!({
                "airport_ident": {
                    "type": "string",
                    "description": "Airport identifier as stored in apt.dat (e.g. 'KSFO')."
                },
                "destination_runway": {
                    "type": "string",
                    "description": "Runway you want to hold short of — either end ident works (e.g. '31' or '13')."
                },
                "via_taxiways": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Ordered list of taxiway names from the clearance, e.g. ['A', 'D']. Pass [] to let the planner pick the shortest route and report which taxiways it used."
                },
                "start_lat": {
                    "type": ["number", "null"],
                    "description": "Starting latitude. If null, the current aircraft position is used."
                },
                "start_lon": {
                    "type": ["number", "null"],
                    "description": "Starting longitude. If null, the current aircraft position is used."
                }
            }),
            &["airport_ident", "destination_runway", "via_taxiways", "start_lat", "start_lon"],
        ),
        schema(
            "engage_park",
            "Taxi to a parking spot and stop with the nose aligned to the spot's painted heading. INBOUND counterpart to engage_taxi — this is the tool for getting from a completed post-landing runway exit (or any other ground position) to parking. Do not call engage_taxi with the runway you just landed on as a workaround for parking; that route doesn't exist by design. Plans a taxi route to the taxi-network node nearest the 1300 parking spot, appends a short lead-in leg to the spot's exact lat/lon, and drives to a final pose at the spot's heading. Auto-releases the parking brake if set. Preconditions: parking spot must exist in apt.dat, and if landing_rollout is still active the aircraft must be stopped on the runway centerline with no exit left to fly; calling while it is rolling out or partway through an exit is rejected so parking guidance cannot cut across the runway environment. Completes by stopping at the spot aligned to the painted heading (within ~12 ft / 8°) and setting the parking brake. Calling it again for the spot the active taxi is already headed to is a no-op. Find candidates with sql_query against parking_spots (filter by airport_ident; match `categories` against aircraft class, e.g. LIKE '%props%' for a single-engine piston, or operation_type='general_aviation' for a GA ramp).",
            json!({
                "airport_ident": {
                    "type": "string",
                    "description": "Airport identifier as stored in apt.dat (e.g. 'KSFO')."
                },
                "parking_name": {
                    "type": "string",
                    "description": "Exact (case-insensitive) name of the 1300 parking spot, e.g. 'Gate A4', 'Ramp 1', 'GA Tie-Down 3'. Query parking_spots for candidates."
                },
                "via_taxiways": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Ordered list of taxiway names if you want a specific routing, e.g. ['B', 'A']. Pass [] for shortest route."
                },
                "start_lat": {
                    "type": ["number", "null"],
                    "description": "Starting latitude. If null, the current aircraft position is used."
                },
                "start_lon": {
                    "type": ["number", "null"],
                    "description": "Starting longitude. If null, the current aircraft position is used."
                }
            }),
            &["airport_ident", "parking_name", "via_taxiways", "start_lat", "start_lon"],
        ),
        schema(
            "plan_park_route",
            "Preview the taxi route to a parking spot without engaging the autopilot. Same output shape as plan_taxi_route plus the resolved spot coordinates and heading. Preconditions: same as engage_park. Completes immediately. Next: engage_park to commit.",
            json!({
                "airport_ident": {
                    "type": "string",
                    "description": "Airport identifier as stored in apt.dat (e.g. 'KSFO')."
                },
                "parking_name": {
                    "type": "string",
                    "description": "Exact (case-insensitive) name of the 1300 parking spot."
                },
                "via_taxiways": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Ordered list of taxiway names, or [] for shortest route."
                },
                "start_lat": {
                    "type": ["number", "null"],
                    "description": "Starting latitude. If null, the current aircraft position is used."
                },
                "start_lon": {
                    "type": ["number", "null"],
                    "description": "Starting longitude. If null, the current aircraft position is used."
                }
            }),
            &["airport_ident", "parking_name", "via_taxiways", "start_lat", "start_lon"],
        ),
        schema(
            "choose_runway_exit",
            "Record a preferred runway-exit taxiway for the post-landing rollout. The rollout controller slows on centerline, follows the exit's actual apt.dat taxi-network geometry, and stops at the first node beyond the runway active zone before landing_rollout releases. Pass null to clear any prior preference. If the taxiway meets the runway more than once, the first intersection past both the touchdown point and the aircraft is used; the reply gives its stationing. An exit that is entirely behind the touchdown point (or the aircraft) is rejected with the list of exits still ahead. Preconditions: a plan flying a pattern (before touchdown) or landing_rollout (after); call on final or any time before touchdown, or after stopping on the runway to pick an exit still ahead. Rejected once the aircraft has started turning onto an exit. Completes immediately. If the selected exit is missed, the aircraft remains on centerline and brakes to a stop short of the runway end instead of cutting back toward it.",
            json!({
                "taxiway_name": {
                    "type": ["string", "null"],
                    "description": "Name of the taxiway exit you want to take, e.g. 'A5'. Pass null to clear the current preference (the rollout will then brake to a stop on the runway centerline)."
                }
            }),
            &["taxiway_name"],
        ),
        schema(
            "plan_arrival",
            "Plan the whole arrival before touchdown in one call: pick the runway exit and the taxi route from that exit to a parking spot. After touchdown the aircraft rolls out, follows the exit off the runway, and taxis straight to the spot (stopping aligned with its painted heading) with no further tool calls. Use this as the normal way to land and park; list_runway_exits / choose_runway_exit / engage_park remain for exceptions and ATC changes. With exit_taxiway null, the first exit at least post_landing.min_rollout_past_touchdown_ft (default 800 ft) past the touchdown point is chosen; if a named taxiway meets the runway more than once, the first intersection past touchdown is used. If the exit is missed (too fast at the intersection), the aircraft stops on the runway centerline, the arrival plan is put on hold, and a `completed: landing_rollout` heartbeat wakes you to choose an exit ahead or engage_park. Calling choose_runway_exit afterwards re-plans the taxi from the new exit. Preconditions: a plan flying a pattern, not yet touched down (rejected once landing_rollout has taken over); parking spot must exist in apt.dat. Find spots with sql_query against parking_spots. Completes immediately.",
            json!({
                "parking_name": {
                    "type": "string",
                    "description": "Parking spot name from the parking_spots table (case-insensitive), e.g. 'FBO Parking'."
                },
                "exit_taxiway": {
                    "type": ["string", "null"],
                    "description": "Runway exit taxiway, e.g. 'A5', or null to auto-select."
                },
                "via_taxiways": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Taxiways to follow in order from the exit to parking, e.g. ['A', 'C']. [] for the shortest route."
                }
            }),
            &["parking_name", "exit_taxiway", "via_taxiways"],
        ),
        schema(
            "list_runway_exits",
            "List the taxiway exits for a landing runway: each candidate taxiway name plus its approximate stationing in feet from the landing threshold (a taxiway that meets the runway more than once is listed once per intersection), so you can pick an exit appropriate for your rollout distance. Uses the 1204 active-zone annotations attached to taxi_edges. Completes immediately. Next: choose_runway_exit.",
            json!({
                "airport_ident": {
                    "type": "string",
                    "description": "Airport identifier as stored in apt.dat."
                },
                "runway_ident": {
                    "type": "string",
                    "description": "Runway end ident, e.g. '28L' or '10R'. Exits are matched against 1204 active-zone entries naming this runway."
                }
            }),
            &["airport_ident", "runway_ident"],
        ),
        schema(
            "set_goal",
            "Declare your top-level mission goal so the human supervisor sees what you are trying to do. Surfaced verbatim on the AI Pilot Console's Mission Intent pane until cleared or replaced. Call this at the start of every mission and any time the goal changes (diversion, alternate, hold). Does not affect aircraft control — pure transparency.",
            json!({
                "kind": {
                    "type": "string",
                    "description": "Short label for the goal kind, e.g. 'land', 'takeoff', 'cruise', 'taxi', 'hold', 'divert'."
                },
                "target": {
                    "type": ["string", "null"],
                    "description": "The destination/object: airport+runway, waypoint, fix, holding pattern. Null when not applicable. Example: 'KEMT runway 19'."
                },
                "notes": {
                    "type": ["string", "null"],
                    "description": "One-line clarifier the operator might need. Null when none."
                }
            }),
            &["kind", "target", "notes"],
        ),
        schema(
            "clear_goal",
            "Remove the currently declared mission goal. Use when the previous goal is fulfilled and you have not yet declared the next one (typically rare — prefer calling set_goal again to replace it).",
            json!({}),
            &[],
        ),
        schema(
            "record_clearance",
            "Pin the active ATC clearance on the AI Pilot Console's Mission Intent pane. The radio buffer keeps the full transmission history; this single line is the operating contract the supervisor needs to see at a glance. Call immediately after every ATC clearance you readback (cleared to land, cleared T&G, cleared takeoff, line up and wait, cleared as filed, etc.) so the supervisor can verify what you're authorized to do. Does not change aircraft control.",
            json!({
                "text": {
                    "type": "string",
                    "description": "Concise readback-style summary of the clearance, e.g. 'cleared touch-and-go runway 19, report midfield'."
                },
                "source": {
                    "type": ["string", "null"],
                    "description": "Issuing controller: 'tower', 'ground', 'approach', 'departure', 'center', 'unicom', or any short label. Null defaults to 'tower'."
                },
                "freq_mhz": {
                    "type": ["number", "null"],
                    "description": "Frequency in MHz (e.g. 124.30). Null when unknown."
                }
            }),
            &["text", "source", "freq_mhz"],
        ),
        schema(
            "clear_clearance",
            "Remove the pinned ATC clearance (e.g. once it has been satisfied, cancelled, or superseded). Prefer calling record_clearance with the new clearance text instead — that replaces the pin in one step.",
            json!({}),
            &[],
        ),
    ];
    if include_mission_complete {
        schemas.push(schema(
            "mission_complete",
            "Eval-harness only. Report that you have finished the assigned mission; the harness records the outcome and shuts the run down after this turn. Call exactly once, when the aircraft has reached the end state described in the initial briefing — success=true if accomplished, false if giving up or you believe it cannot be completed. If you find yourself calling sleep repeatedly with the aircraft stopped or at the briefed end state (e.g. parked with brake set, only idle_* profiles, no pending work), call mission_complete instead — sleep waits for more work, mission_complete ends the run. Preconditions: do NOT call mid-flight or while a profile is still actively driving the aircraft to the end state. Completes by ending the run.",
            json!({
                "success": {
                    "type": "boolean",
                    "description": "true if the assigned mission was accomplished, false if you are giving up."
                },
                "summary": {
                    "type": "string",
                    "description": "One- or two-sentence recap of what you did and what state the aircraft is in."
                }
            }),
            &["success", "summary"],
        ));
    }
    schemas
}

// SQL query description notes:
//   * Source is X-Plane's apt.dat parsed into three zstd GeoParquet tables
//     (airports, runways, comms). Endpoints are 8-decimal; headings and
//     lengths are derived from endpoint geodesy.
//   * "ALWAYS prefix spatial functions with ST_" — the LLM was observed
//     emitting bare POINT(...) which fails on DuckDB.
//   * ST_Distance_Sphere-only-accepts-POINT warning + crosstrack /
//     along-track example — the LLM tried to pass a LINESTRING and fail.
const SQL_QUERY_DESCRIPTION: &str = "Run a read-only SQL query against the runway/airport/comms/taxi \
database. Authoritative source for runway and airport facts — \
never guess a runway identifier, airport code, course, length, elevation, or ATC frequency; \
query for it. DuckDB with the spatial extension; results are tab-separated with a header row, \
truncated to 50 rows. Completes immediately.

View: airports (one row per airport / seaplane base / heliport, worldwide)
  ident VARCHAR                -- airport identifier as published in apt.dat, e.g. 'KSEA', 'EGLL'
  name VARCHAR
  elevation_ft DOUBLE
  icao_code VARCHAR            -- may be NULL for non-ICAO airports
  iata_code VARCHAR            -- may be NULL
  faa_code VARCHAR             -- may be NULL (US airports only)
  latitude_deg DOUBLE          -- airport reference point (ARP); falls back to runway centroid
  longitude_deg DOUBLE
  arp GEOMETRY                 -- POINT(longitude, latitude), NULL when no coords available

View: runways (one row per physical land runway)
  id BIGINT
  airport_ident VARCHAR        -- joins to airports.ident
  length_ft DOUBLE             -- great-circle distance between the two runway ends
  width_ft DOUBLE
  surface VARCHAR              -- 'ASP', 'CON', 'TRF', 'DIRT', 'GRV', 'WATER', 'SNOW', 'TRANS', 'LAKE', 'UNK'
  lighted TINYINT              -- 0 or 1
  closed TINYINT               -- 0 or 1 (always 0 — apt.dat does not encode a closed flag at runway level)
  le_ident VARCHAR             -- low-numbered-end identifier, e.g. '16L'
  le_latitude_deg DOUBLE       -- pavement-end latitude at the low end (the displaced threshold is inside)
  le_longitude_deg DOUBLE
  le_elevation_ft DOUBLE       -- inherited from airport elevation (apt.dat has no per-end elevation)
  le_heading_degT DOUBLE       -- initial bearing from le threshold toward he
  le_displaced_threshold_ft DOUBLE -- offset inside the runway to the landing threshold
  he_ident VARCHAR             -- high-numbered-end identifier, e.g. '34R'
  he_latitude_deg DOUBLE
  he_longitude_deg DOUBLE
  he_elevation_ft DOUBLE
  he_heading_degT DOUBLE
  he_displaced_threshold_ft DOUBLE
  centerline GEOMETRY          -- LINESTRING from le threshold to he threshold
  le_threshold GEOMETRY        -- POINT(le_longitude_deg, le_latitude_deg)
  he_threshold GEOMETRY        -- POINT(he_longitude_deg, he_latitude_deg)

View: comms (one row per ATC frequency entry)
  airport_ident VARCHAR        -- joins to airports.ident
  code SMALLINT                -- raw apt.dat row code (1050..=1056)
  kind VARCHAR                 -- 'ATIS' | 'UNICOM' | 'CD' | 'GND' | 'TWR' | 'APP' | 'DEP'
  freq_mhz DOUBLE              -- e.g. 118.200, 127.275
  label VARCHAR                -- free-form label ('TWR', 'NORCAL APP', 'AWOS 1', ...)

View: taxi_nodes (one row per taxi route network node — apt.dat row 1201)
  airport_ident VARCHAR        -- joins to airports.ident
  node_id INTEGER              -- unique *within* the airport only
  latitude_deg DOUBLE
  longitude_deg DOUBLE
  usage VARCHAR                -- 'init' | 'end' | 'both' (where a route may start/end)
  point GEOMETRY               -- POINT(longitude, latitude)

View: taxi_edges (one row per taxiway segment — apt.dat row 1202)
  airport_ident VARCHAR        -- joins to airports.ident
  from_node INTEGER            -- joins to taxi_nodes.node_id
  to_node INTEGER
  direction VARCHAR            -- 'twoway' | 'oneway' (oneway traverses from_node → to_node only)
  category VARCHAR             -- raw apt.dat token ('taxiway_E', 'runway', ...); width class letter is the suffix
  name VARCHAR                 -- taxiway name as painted, e.g. 'A', 'A2', 'B' (may be empty for connectors)
  active_zones VARCHAR         -- semicolon-delimited 'kind:runways' entries for runway conflicts (1204 rows),
                               -- e.g. 'departure:10L,28R;ils:28L'. Empty when none.

View: parking_spots (one row per startup/parking location — apt.dat row 1300)
  airport_ident VARCHAR        -- joins to airports.ident
  name VARCHAR                 -- free-form ('Gate A4', 'Ramp 1', 'GA Tie-Down 3')
  kind VARCHAR                 -- 'gate' | 'tie_down' | 'hangar' | 'misc'
  categories VARCHAR           -- pipe-separated aircraft classes accepted here,
                               -- e.g. 'heavy|jets|turboprops|props|helos'
  heading_true_deg DOUBLE      -- true heading aircraft should face when parked
  latitude_deg DOUBLE
  longitude_deg DOUBLE
  icao_category VARCHAR        -- optional 1301 ICAO aircraft category (A..F)
  operation_type VARCHAR       -- optional 1301: 'none' | 'general_aviation' | 'airline' | 'cargo' | 'military'
  airlines VARCHAR             -- optional 1301 comma-separated 3-letter airline codes
  location GEOMETRY            -- POINT(longitude, latitude)

Use taxi_nodes/taxi_edges for exploratory queries ('what taxiways connect at node 42?',
'which edges conflict with runway 31 on departure?'); for actual route planning call
`plan_taxi_route`.

Spatial functions: ST_Point(lon, lat) — LONGITUDE FIRST — and ST_Distance_Sphere(p1, p2)
which returns meters along the great circle. Other ST_* functions (ST_Distance, ST_DWithin,
ST_AsGeoJSON) are available. ALWAYS prefix with ST_ (bare POINT fails). ST_Distance_Sphere
accepts two POINTs only — it errors on a LINESTRING. For point-to-centerline distance
(crosstrack offset) use the flat-earth projection shown in the closest-runway query below.

Common queries:

  -- 'What is the closest runway?' / 'Where am I?' / 'What runway am I on?'
  -- — call get_status first to get your lat/lon AND heading, then run this
  -- with all three substituted in. Returns the ONE runway whose body
  -- (treated as a line segment between thresholds) is closest to you, plus
  -- the geometry you need to decide whether you're actually *on* it.
  --
  -- IMPORTANT details baked into this query (do not skip any of them):
  --   * miles_from_centerline is the PERPENDICULAR distance from the
  --     aircraft to the runway centerline, in statute miles. Computed in a
  --     local flat-earth projection rooted at the le threshold (accurate
  --     to sub-meter over a single runway). A runway surface is ~50-150 ft
  --     wide, so miles_from_centerline < ~0.02 (~100 ft) means you are
  --     laterally on (or on the extended line of) the runway. Larger
  --     values mean you're on a taxiway / ramp / parallel runway adjacent
  --     to it.
  --   * feet_from_threshold is the Euclidean distance to the *nearer* of
  --     the two thresholds. Combined with length_ft this tells you where
  --     along the runway you are: near zero = at a threshold, ~length_ft/2
  --     = midfield, > length_ft = past the far end / off the pavement.
  --   * active_ident is computed in SQL from cos(heading - end_heading).
  --     cos handles angular wraparound automatically (heading 0.6 is next
  --     to 360, cos ~ +1, not across from it). Do NOT compute this column
  --     in your head — read it from the query result.
  --   * ORDER BY uses point-to-segment distance (not point-to-threshold),
  --     so sitting at midfield on a 10000 ft runway still correctly picks
  --     that runway as closest even though both thresholds are 5000 ft
  --     away. No bounding box; DuckDB scans 43k rows in milliseconds.
  WITH me AS (SELECT <lat> AS lat_a, <lon> AS lon_a, <hdg> AS hdg_a),
  proj AS (
    SELECT
      r.airport_ident, r.le_ident, r.he_ident,
      r.length_ft, r.surface,
      r.le_heading_degT, r.he_heading_degT,
      (r.he_longitude_deg - r.le_longitude_deg)
        * 111320.0 * cos(radians(r.le_latitude_deg)) AS rx,
      (r.he_latitude_deg - r.le_latitude_deg) * 111320.0 AS ry,
      (m.lon_a - r.le_longitude_deg)
        * 111320.0 * cos(radians(r.le_latitude_deg)) AS ax,
      (m.lat_a - r.le_latitude_deg) * 111320.0 AS ay,
      ST_Distance_Sphere(ST_Point(r.le_longitude_deg, r.le_latitude_deg),
                         ST_Point(m.lon_a, m.lat_a)) AS le_dist_m,
      ST_Distance_Sphere(ST_Point(r.he_longitude_deg, r.he_latitude_deg),
                         ST_Point(m.lon_a, m.lat_a)) AS he_dist_m,
      m.hdg_a AS hdg_a
    FROM runways r, me m
    WHERE r.closed = 0
      AND r.le_latitude_deg IS NOT NULL
      AND r.he_latitude_deg IS NOT NULL
  ),
  metrics AS (
    SELECT *,
      CASE WHEN (rx*rx + ry*ry) > 0
           THEN (ax*rx + ay*ry) / (rx*rx + ry*ry)
           ELSE 0 END AS t,
      CASE WHEN (rx*rx + ry*ry) > 0
           THEN abs(ax*ry - ay*rx) / sqrt(rx*rx + ry*ry)
           ELSE sqrt(ax*ax + ay*ay) END AS crosstrack_m
    FROM proj
  )
  SELECT
    airport_ident, le_ident, he_ident, length_ft, surface,
    le_heading_degT, he_heading_degT,
    crosstrack_m / 1609.344 AS miles_from_centerline,
    LEAST(le_dist_m, he_dist_m) * 3.28084 AS feet_from_threshold,
    CASE
      WHEN cos(radians(le_heading_degT - hdg_a)) >
           cos(radians(he_heading_degT - hdg_a))
      THEN le_ident ELSE he_ident
    END AS active_ident
  FROM metrics
  ORDER BY
    CASE
      WHEN t < 0 THEN sqrt(ax*ax + ay*ay)
      WHEN t > 1 THEN sqrt((ax-rx)*(ax-rx) + (ay-ry)*(ay-ry))
      ELSE crosstrack_m
    END
  LIMIT 1;

  -- Interpreting the row: you are ON the returned runway if
  -- miles_from_centerline < ~0.02 AND feet_from_threshold < length_ft.
  -- Otherwise you're near it but not on it (taxiway/ramp/extended
  -- centerline on approach). active_ident is the end you are pointed at —
  -- use that as the runway in a plan: apt.runway(ident).

  -- All active runways at a known airport
  SELECT airport_ident, le_ident, he_ident, length_ft, surface,
         le_heading_degT, he_heading_degT
  FROM runways WHERE airport_ident = 'KSEA' AND closed = 0;

  -- Nearest paved runway at least 5000 ft long (both ends checked)
  SELECT airport_ident, le_ident, he_ident, length_ft, surface,
         LEAST(
           ST_Distance_Sphere(ST_Point(le_longitude_deg, le_latitude_deg),
                              ST_Point(<lon>, <lat>)),
           ST_Distance_Sphere(ST_Point(he_longitude_deg, he_latitude_deg),
                              ST_Point(<lon>, <lat>))
         ) / 1852.0 AS dist_nm
  FROM runways
  WHERE closed = 0 AND length_ft >= 5000 AND surface IN ('ASP', 'CONC', 'CON')
    AND le_latitude_deg IS NOT NULL AND he_latitude_deg IS NOT NULL
  ORDER BY dist_nm
  LIMIT 5;

  -- Crosstrack (off-centerline) and along-track offsets, in feet, of an
  -- aircraft from a specified runway's centerline. Uses a local flat-earth
  -- projection rooted at the low-end threshold (1 degree of latitude ≈
  -- 364000 ft; scale longitude by cos(lat)). Accurate to better than a
  -- foot over any single runway.
  --   crosstrack_ft = signed perpendicular distance, positive = RIGHT of
  --                   centerline when facing from le toward he.
  --   alongtrack_ft = position along centerline, 0 at le threshold, and
  --                   approximately length_ft at he threshold.
  SELECT
    (ax*dy - ay*dx) / SQRT(dx*dx + dy*dy) AS crosstrack_ft,
    (ax*dx + ay*dy) / SQRT(dx*dx + dy*dy) AS alongtrack_ft
  FROM (
    SELECT
      (he_longitude_deg - le_longitude_deg) * 364000 * COS(RADIANS(le_latitude_deg)) AS dx,
      (he_latitude_deg  - le_latitude_deg ) * 364000                                 AS dy,
      (<lon>            - le_longitude_deg) * 364000 * COS(RADIANS(le_latitude_deg)) AS ax,
      (<lat>            - le_latitude_deg )  * 364000                                AS ay
    FROM runways
    WHERE airport_ident = 'KEMT' AND le_ident = '1'
  );

  -- ATC frequencies at an airport
  SELECT kind, freq_mhz, label
  FROM comms
  WHERE airport_ident = 'KSFO'
  ORDER BY code;

  -- Nearest airport by ARP to a point (fast with the Hilbert-sorted parquet)
  SELECT ident, name,
         ST_Distance_Sphere(arp, ST_Point(<lon>, <lat>)) / 1852.0 AS dist_nm
  FROM airports
  WHERE arp IS NOT NULL
  ORDER BY dist_nm
  LIMIT 5;
";

#[cfg(test)]
mod quiet_tool_tests {
    #[test]
    fn every_quiet_tool_is_a_real_tool() {
        let names: Vec<String> = super::tool_schemas(true).into_iter().map(|t| t.name).collect();
        for q in super::QUIET_TOOLS {
            assert!(names.iter().any(|n| n == q), "{q} isn't a tool");
        }
    }
}
