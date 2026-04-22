//! Tool handlers + JSON schemas for the LLM. Mirrors sim_pilot/llm/tools.py.
//!
//! Each tool is dispatched from a parsed `function_call` item returned by the
//! Responses API. Handlers mutate `PilotCore` state (engaging/disengaging
//! profiles, arming pattern-fly triggers) or talk to the X-Plane bridge
//! (radio, parking brake, flaps). The SQL query tool uses DuckDB with the
//! spatial extension loaded so the agent can do real geospatial lookups.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use anyhow::{anyhow, Result};
use parking_lot::Mutex as PLMutex;
use serde_json::{json, Map, Value};

use crate::bus::SimBus;
use crate::config::ConfigBundle;
use crate::core::mission_manager::{PilotCore, StatusSnapshot};
use crate::core::profiles::{
    AltitudeHoldProfile, ExtendMode, HeadingHoldProfile, PatternClearanceGate, PatternFlyProfile,
    PatternLeg, SpeedHoldProfile, TakeoffProfile, TaxiProfile,
};
use crate::data::airspace_query::{self, AircraftSnapshot, AirspaceSource};
use crate::data::parquet::{
    AIRPORTS_FILE, AIRSPACES_FILE, COMMS_FILE, PARKING_SPOTS_FILE, RUNWAYS_FILE, TAXI_EDGES_FILE,
    TAXI_NODES_FILE,
};
use crate::guidance::runway_geometry::RunwayFrame;
use crate::guidance::taxi_route::{self, TaxiDestination};
use crate::sim::datarefs::{
    COM1_FREQUENCY_HZ_833, COM2_FREQUENCY_HZ_833, FLAP_HANDLE_REQUEST_RATIO, LATITUDE_DEG,
    LONGITUDE_DEG, PARKING_BRAKE_RATIO,
};
use crate::sim::xplane_bridge::{geodetic_offset_ft, GeoReference};
use crate::types::{heading_to_vector, FlightPhase, Runway, StraightLeg, TrafficSide, Vec2};

pub const SQL_QUERY_MAX_ROWS: usize = 50;

/// Write-capable view over the bridge. The real `XPlaneWebBridge` is one
/// implementation; tests use a fake that records writes. The abstraction
/// keeps `tools.rs` testable without needing a live X-Plane.
pub trait ToolBridge: Send + Sync {
    fn georef(&self) -> GeoReference;
    fn get_dataref_value(&self, name: &str) -> Option<f64>;
    fn write_dataref_values(&self, updates: &[(String, f64)]) -> Result<()>;
}

pub struct ToolContext {
    pub pilot: Arc<PLMutex<PilotCore>>,
    pub bridge: Option<Arc<dyn ToolBridge>>,
    pub config: Arc<PLMutex<ConfigBundle>>,
    pub recent_broadcasts: Arc<PLMutex<Vec<String>>>,
    /// Directory holding the three apt.dat-derived parquet files
    /// (airports / runways / comms). Populated on startup by
    /// `data::parquet::resolve`; `None` only when no apt.dat was locatable.
    pub apt_dat_cache_dir: Option<PathBuf>,
    pub bus: Option<SimBus>,
    pub runway_conn: Arc<Mutex<Option<duckdb::Connection>>>,
    /// Handle into the live heartbeat pump. Only `sleep(suppress_idle_heartbeat_s=...)`
    /// needs it today. `None` when heartbeats are disabled or in tests.
    pub heartbeat_pump: Option<Arc<crate::live_runner::HeartbeatPump>>,
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
            apt_dat_cache_dir: None,
            bus: None,
            runway_conn: Arc::new(Mutex::new(None)),
            heartbeat_pump: None,
        }
    }
}

// ---------- helpers ----------

fn format_displaced(displaced: &[String]) -> String {
    if displaced.is_empty() {
        String::new()
    } else {
        format!(" (displaced: {})", displaced.join(", "))
    }
}

fn pilot_reference_label(ctx: &ToolContext) -> String {
    let cfg = ctx.config.lock();
    let mut parts: Vec<String> = Vec::new();
    if let Some(a) = &cfg.airport.airport {
        parts.push(format!("airport={}", a));
    }
    if let Some(r) = &cfg.airport.runway.id {
        parts.push(format!("runway={}", r));
    }
    parts.push(format!("course={:.0}deg", cfg.airport.runway.course_deg));
    parts.join(" ")
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
        "alt_agl_ft": round_f64(state.alt_agl_ft, 1),
        "ias_kt": round_f64(state.ias_kt, 1),
        "gs_kt": round_f64(state.gs_kt, 1),
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
    match phase {
        Some(FlightPhase::Preflight)
        | Some(FlightPhase::TakeoffRoll)
        | Some(FlightPhase::Rollout)
        | Some(FlightPhase::TaxiClear) => false,
        _ => true,
    }
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

/// Release the parking brake if set. Returns `Some(prior_ratio)` when a
/// release actually happened, so callers can note it in their summary;
/// returns `None` when nothing needed doing.
fn release_parking_brake_if_set(
    ctx: &ToolContext,
    bridge: &Arc<dyn ToolBridge>,
) -> Result<Option<f64>, String> {
    match parking_brake_ratio(ctx) {
        Some(v) if v >= 0.5 => bridge
            .write_dataref_values(&[(PARKING_BRAKE_RATIO.name.to_string(), 0.0)])
            .map(|_| Some(v))
            .map_err(|e| format!("releasing parking brake: {}", e)),
        _ => Ok(None),
    }
}

// ---------- individual tool handlers ----------

pub fn tool_get_status(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    let pilot = ctx.pilot.lock();
    let snap = pilot.latest_snapshot.clone();
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
    // Once a real profile (pattern_fly, taxi, park, takeoff, any hold)
    // owns the aircraft, sleep is the right thing to do even at speed.
    let snapshot = ctx.pilot.lock().latest_snapshot.clone();
    if let Some(snap) = snapshot {
        let only_idle = snap
            .active_profiles
            .iter()
            .all(|n| n.starts_with("idle_"));
        if only_idle && snap.state.gs_kt > SLEEP_STATIONARY_GS_KT {
            return format!(
                "error: aircraft is moving at {:.1} kt groundspeed with only idle profiles active — sleep is only allowed when a control profile is flying the aircraft, or when stationary. Engage a control profile (engage_pattern_fly / engage_park / engage_taxi) or bring the aircraft to a stop before sleeping.",
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

pub fn tool_engage_heading_hold(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let heading = match arg_f64(args, "heading_deg") {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    let direction = args
        .get("turn_direction")
        .and_then(|v| v.as_str())
        .map(|s| s.to_string());
    let max_bank = ctx.config.lock().limits.max_bank_enroute_deg;
    let profile = match HeadingHoldProfile::new(heading, max_bank, direction.as_deref()) {
        Ok(p) => p,
        Err(e) => return format!("error: {}", e),
    };
    let heading = profile.heading_deg;
    let displaced = ctx.pilot.lock().engage_profile(Box::new(profile));
    let direction_note = direction
        .as_deref()
        .map(|d| format!(" via {}", d))
        .unwrap_or_default();
    format!(
        "engaged heading_hold heading={:.1}deg{}{}",
        heading,
        direction_note,
        format_displaced(&displaced)
    )
}

pub fn tool_engage_altitude_hold(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let altitude = match arg_f64(args, "altitude_ft") {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    let displaced = ctx
        .pilot
        .lock()
        .engage_profile(Box::new(AltitudeHoldProfile::new(altitude)));
    format!(
        "engaged altitude_hold altitude={:.0}ft{}",
        altitude,
        format_displaced(&displaced)
    )
}

pub fn tool_engage_speed_hold(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let speed = match arg_f64(args, "speed_kt") {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    let displaced = ctx
        .pilot
        .lock()
        .engage_profile(Box::new(SpeedHoldProfile::new(speed)));
    format!(
        "engaged speed_hold speed={:.0}kt{}",
        speed,
        format_displaced(&displaced)
    )
}

pub fn tool_engage_cruise(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let heading = match arg_f64(args, "heading_deg") { Ok(v) => v, Err(e) => return format!("error: {}", e) };
    let altitude = match arg_f64(args, "altitude_ft") { Ok(v) => v, Err(e) => return format!("error: {}", e) };
    let speed = match arg_f64(args, "speed_kt") { Ok(v) => v, Err(e) => return format!("error: {}", e) };
    let max_bank = ctx.config.lock().limits.max_bank_enroute_deg;
    let profiles: Vec<Box<dyn crate::core::profiles::GuidanceProfile>> = vec![
        Box::new(HeadingHoldProfile::new(heading, max_bank, None).unwrap()),
        Box::new(AltitudeHoldProfile::new(altitude)),
        Box::new(SpeedHoldProfile::new(speed)),
    ];
    let displaced = ctx.pilot.lock().engage_profiles(profiles);
    format!(
        "engaged cruise heading={:.0}deg alt={:.0}ft speed={:.0}kt{}",
        heading,
        altitude,
        speed,
        format_displaced(&displaced)
    )
}

/// Touchdown zone length (feet) used when the runway is installed dynamically
/// from a query — `RunwayFrame::touchdown_runway_x_ft` halves this and then
/// clamps to `[500, length/3]` to produce the aim-point. Returning 1200 here
/// targets an aim point 600 ft past the landing threshold, which leaves room
/// for the roundout/flare float without overrunning on short strips.
fn synthesize_touchdown_zone_ft(_length_ft: f64) -> f64 {
    1200.0
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

fn lookup_runway_for_pattern(
    ctx: &ToolContext,
    airport_ident: &str,
    runway_ident: &str,
    side: &str,
) -> Result<(Runway, f64)> {
    let bridge = ctx
        .bridge
        .as_ref()
        .ok_or_else(|| anyhow!("no X-Plane bridge available; cannot compute world-frame threshold"))?;
    ensure_runway_conn(ctx)?;
    let guard = ctx.runway_conn.lock().unwrap();
    let conn = guard.as_ref().unwrap();
    let mut stmt = conn.prepare(
        "SELECT le_ident, he_ident, \
                le_latitude_deg, le_longitude_deg, le_heading_degT, le_elevation_ft, le_displaced_threshold_ft, \
                he_latitude_deg, he_longitude_deg, he_heading_degT, he_elevation_ft, he_displaced_threshold_ft, \
                length_ft \
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
    let le_lat: Option<f64> = row.get(2)?;
    let le_lon: Option<f64> = row.get(3)?;
    let le_hdg: Option<f64> = row.get(4)?;
    let le_elev: Option<f64> = row.get(5)?;
    let le_disp: Option<f64> = row.get(6)?;
    let he_lat: Option<f64> = row.get(7)?;
    let he_lon: Option<f64> = row.get(8)?;
    let he_hdg: Option<f64> = row.get(9)?;
    let he_elev: Option<f64> = row.get(10)?;
    let he_disp: Option<f64> = row.get(11)?;
    let length_ft: Option<f64> = row.get(12)?;

    let (threshold_lat, threshold_lon, course_hdg, runway_elev, displaced_ft) =
        if Some(runway_ident.to_string()) == le_ident {
            (le_lat, le_lon, le_hdg, le_elev, le_disp)
        } else if Some(runway_ident.to_string()) == he_ident {
            (he_lat, he_lon, he_hdg, he_elev, he_disp)
        } else {
            return Err(anyhow!(
                "runway lookup returned {:?}/{:?} for query {:?}; internal mismatch",
                le_ident,
                he_ident,
                runway_ident
            ));
        };
    let (Some(lat), Some(lon), Some(course)) = (threshold_lat, threshold_lon, course_hdg) else {
        return Err(anyhow!(
            "runway {}/{} has no threshold coordinates or course in the database",
            airport_ident,
            runway_ident
        ));
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
    let pavement_end_ft = geodetic_offset_ft(lat, lon, bridge.georef());
    let field_elev = runway_elev.unwrap_or(0.0);
    let resolved_length = length_ft.unwrap_or(5000.0);
    Ok((
        Runway {
            id: Some(runway_ident.to_string()),
            threshold_ft: pavement_end_ft,
            course_deg: course,
            length_ft: resolved_length,
            touchdown_zone_ft: synthesize_touchdown_zone_ft(resolved_length),
            displaced_threshold_ft: displaced_ft.unwrap_or(0.0),
            traffic_side,
        },
        field_elev,
    ))
}

fn install_runway_in_pilot_core(
    ctx: &ToolContext,
    airport_ident: &str,
    runway: Runway,
    field_elevation_ft: f64,
) {
    {
        let mut cfg = ctx.config.lock();
        cfg.airport.airport = Some(airport_ident.to_string());
        cfg.airport.field_elevation_ft = field_elevation_ft;
        cfg.airport.runway = runway.clone();
    }
    let new_config = ctx.config.lock().clone();
    let mut pilot = ctx.pilot.lock();
    pilot.config = new_config;
    pilot.runway_frame = RunwayFrame::new(runway);
}

pub fn tool_engage_pattern_fly(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let airport = match arg_str(args, "airport_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let runway_ident = match arg_str(args, "runway_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let side = match arg_str(args, "side") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let start_phase = match arg_str(args, "start_phase") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let (runway, field_elev) = match lookup_runway_for_pattern(ctx, &airport, &runway_ident, &side) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    install_runway_in_pilot_core(ctx, &airport, runway, field_elev);

    let new_config = ctx.config.lock().clone();
    let runway_frame = ctx.pilot.lock().runway_frame.clone();
    let mut profile = PatternFlyProfile::new(new_config, runway_frame);
    match FlightPhase::from_str(&start_phase.to_ascii_lowercase()) {
        Some(p) => profile.phase = p,
        None => {
            let valid: Vec<&'static str> = FlightPhase::all().iter().map(|p| p.value()).collect();
            return format!("error: unknown start_phase {:?}; valid values are {:?}", start_phase, valid);
        }
    }
    let displaced = ctx.pilot.lock().engage_profile(Box::new(profile));
    format!(
        "engaged pattern_fly {}{}",
        pilot_reference_label(ctx),
        format_displaced(&displaced)
    )
}

pub fn tool_engage_takeoff(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let pb = parking_brake_ratio(ctx);
    if let Some(v) = pb {
        if v >= 0.5 {
            return format!(
                "error: parking brake is SET (ratio={:.2}) — call set_parking_brake(engaged=False) first, then retry engage_takeoff",
                v
            );
        }
    }

    let airport = match arg_str(args, "airport_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let runway_ident = match arg_str(args, "runway_ident") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };

    // Resolve runway from apt.dat and install it into pilot core. This
    // gives runway_frame the correct threshold position, course, and
    // length — not whatever was grabbed at startup from the aircraft's
    // boot heading. The rollout centerline controller tracks that
    // course to keep the aircraft on centerline during the roll.
    match lookup_runway_for_pattern(ctx, &airport, &runway_ident, "left") {
        Ok((rwy, field_elev)) => install_runway_in_pilot_core(ctx, &airport, rwy, field_elev),
        Err(e) => return format!("error: {}", e),
    }

    // Verify aircraft is actually on the runway we just installed, not
    // just happening to be pointed in its direction. Position check
    // catches the "I forgot to taxi/line up" case where the old
    // heading-only guard was a false positive.
    let (start_along_ft, usable_length_ft) = {
        let pilot = ctx.pilot.lock();
        let Some(snap) = pilot.latest_snapshot.as_ref() else {
            return "error: no aircraft state yet — call get_status first so the control loop publishes a snapshot".to_string();
        };
        let runway_frame = &pilot.runway_frame;
        let pos_rwy = runway_frame.to_runway_frame(snap.state.position_ft);
        let along = pos_rwy.x;
        let cross = pos_rwy.y;
        let length = runway_frame.runway.length_ft;
        let course = runway_frame.runway.course_deg;
        let heading_err = crate::types::wrap_degrees_180(course - snap.state.heading_deg).abs();

        if cross.abs() > 100.0 {
            return format!(
                "error: aircraft is {:.0} ft off runway {} centerline (max 100 ft). Taxi onto the runway first — call engage_line_up runway={} to move into position.",
                cross.abs(), runway_ident, runway_ident
            );
        }
        if along < -100.0 || along > length {
            return format!(
                "error: aircraft is at along-track {:.0} ft on runway {} (runway extends 0..{:.0} ft). Position is not on the runway — use engage_taxi or engage_line_up to get here first.",
                along, runway_ident, length
            );
        }
        if heading_err > 15.0 {
            return format!(
                "error: aircraft heading {:.0}° is not aligned with runway {} course {:.0}° (Δ={:.0}° > 15°). Call engage_line_up runway={} and wait for it to finish before engage_takeoff.",
                snap.state.heading_deg, runway_ident, course, heading_err, runway_ident
            );
        }
        if snap.state.gs_kt > 3.0 {
            return format!(
                "error: aircraft moving at {:.1} kt — stop on the runway centerline before engage_takeoff (usually means engage_line_up is still in progress)",
                snap.state.gs_kt
            );
        }
        let start_along = along.max(0.0);
        (start_along, length - start_along)
    };

    // Guard against intersection departures with insufficient remaining
    // runway. 1000 ft is a generous C172-at-sea-level floor; below this
    // there's no realistic chance of reaching Vr with margin, so refuse
    // rather than let the profile roll off the end. (Density-altitude
    // aware limit is a follow-up.)
    const MIN_USABLE_LENGTH_FT: f64 = 1000.0;
    if usable_length_ft < MIN_USABLE_LENGTH_FT {
        return format!(
            "error: only {:.0} ft of runway {} available from current position (min {:.0} ft). \
             Not enough room to accelerate to Vr. Taxi back to a position with more runway ahead.",
            usable_length_ft, runway_ident, MIN_USABLE_LENGTH_FT
        );
    }

    let new_config = ctx.config.lock().clone();
    let runway_frame = ctx.pilot.lock().runway_frame.clone();
    let profile = Box::new(TakeoffProfile::new_with_usable_length(
        new_config,
        runway_frame,
        usable_length_ft,
        start_along_ft,
    ));
    let displaced = ctx.pilot.lock().engage_profile(profile);
    format!(
        "engaged takeoff {} runway {} ({:.0} ft usable ahead){}",
        airport,
        runway_ident,
        usable_length_ft,
        format_displaced(&displaced)
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
        vec!["TAKEOFF CHECKLIST — address every [ACTION] item before engage_takeoff:".to_string()];
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
        lines.push("  [ERROR]  not on ground — you cannot engage_takeoff from the air".to_string());
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
                "  [ACTION] heading {:.0}° is {:.0}° off runway course {:.0}° — run engage_line_up and wait for it to finish (aircraft stopped, pointed down the runway) before engage_takeoff",
                state.heading_deg, heading_err, runway_course
            ));
            action_needed = true;
        }
    }

    if state.gs_kt > 3.0 {
        lines.push(format!(
            "  [ACTION] still moving at {:.1} kt ground speed — wait for engage_line_up to finish and the aircraft to stop before engage_takeoff",
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
        .filter(|p| *p == "takeoff" || *p == "pattern_fly")
        .collect();
    if !active.is_empty() {
        lines.push(format!("  [INFO]   already engaged: {}", active.join(", ")));
    }

    lines.push("  [REMINDER] identify the runway you are on (get_status for lat/lon + heading, then sql_query with the 'What is the closest runway?' example) before committing to the roll".to_string());
    lines.push("  [REMINDER] acknowledge takeoff clearance on the radio (broadcast_on_radio) if ATC has issued one".to_string());
    lines.push("".to_string());
    if action_needed {
        lines.push("One or more items need action. Fix them, then re-run this checklist or proceed to engage_takeoff.".to_string());
    } else {
        lines.push("All items OK. Call engage_takeoff() to begin the roll.".to_string());
    }
    lines.join("\n")
}

pub fn tool_disengage_profile(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let name = match arg_str(args, "name") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let mut pilot = ctx.pilot.lock();
    let added = pilot.disengage_profile(&name);
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

fn with_pattern_profile<R>(ctx: &ToolContext, f: impl FnOnce(&mut PatternFlyProfile) -> R) -> Option<R> {
    let mut pilot = ctx.pilot.lock();
    pilot.find_profile_mut("pattern_fly", |p| {
        // SAFETY-ish: we downcast by casting through Any. We use a
        // declarative trait method instead — `pattern_metadata()` is
        // always set on PatternFlyProfile — but mutating access needs
        // a concrete reference. The Python version uses isinstance().
        // Here we rely on the fact that only PatternFlyProfile
        // registers as "pattern_fly", and transmute to the concrete
        // type via a helper.
        let ptr = p as *mut dyn crate::core::profiles::GuidanceProfile as *mut u8;
        let concrete = ptr as *mut PatternFlyProfile;
        // SAFETY: `pilot.find_profile_mut` guarantees the pointer points
        // to a boxed profile whose `name()` is "pattern_fly", and no
        // other profile shares that name — so the vtable target is
        // `PatternFlyProfile`.
        unsafe { f(&mut *concrete) }
    })
}

pub fn tool_extend_pattern_leg(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let leg_str = match arg_str(args, "leg") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let Some(leg) = PatternLeg::from_str(&leg_str) else {
        return format!(
            "error: unknown leg {:?} (expected one of: crosswind, downwind)",
            leg_str
        );
    };
    let feet = match arg_f64(args, "extension_ft") {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    let mode_str = arg_str(args, "mode").unwrap_or("add").to_string();
    let mode = match mode_str.to_ascii_lowercase().as_str() {
        "add" => ExtendMode::Add,
        "set" => ExtendMode::Set,
        other => return format!("error: unknown mode {:?} (expected 'add' or 'set')", other),
    };
    let result = with_pattern_profile(ctx, |p| p.extend_leg(leg, feet, mode));
    match result {
        Some(Ok(total)) => format!(
            "{} leg extension now {:.0}ft (mode={})",
            leg.as_str(),
            total,
            match mode {
                ExtendMode::Add => "add",
                ExtendMode::Set => "set",
            }
        ),
        Some(Err(msg)) => format!("error: {}", msg),
        None => "error: pattern_fly profile is not active; call engage_pattern_fly first".to_string(),
    }
}

pub fn tool_execute_pattern_turn(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let leg_str = match arg_str(args, "leg") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let Some(leg) = PatternLeg::from_str(&leg_str) else {
        return format!(
            "error: unknown leg {:?} (expected one of: crosswind, downwind, base, final)",
            leg_str
        );
    };
    match with_pattern_profile(ctx, |p| p.execute_turn(leg)) {
        Some(_) => format!("turn to {} triggered", leg.as_str()),
        None => "error: pattern_fly profile is not active; call engage_pattern_fly first".to_string(),
    }
}

pub fn tool_set_pattern_clearance(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let gate_str = match arg_str(args, "gate") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let Some(gate) = PatternClearanceGate::from_str(&gate_str) else {
        return format!(
            "error: unknown gate {:?} (expected one of: turn_crosswind, turn_downwind, turn_base, turn_final, land)",
            gate_str
        );
    };
    let granted = match args.get("granted").and_then(|v| v.as_bool()) {
        Some(b) => b,
        None => return "error: missing required bool argument 'granted'".to_string(),
    };
    let runway_id = args
        .get("runway_id")
        .and_then(|v| v.as_str())
        .map(|s| s.to_string());
    match with_pattern_profile(ctx, |p| p.set_clearance(gate, granted, runway_id.clone())) {
        Some(_) => match (gate, granted) {
            (PatternClearanceGate::Land, true) => format!(
                "cleared to land{}",
                runway_id
                    .as_ref()
                    .map(|r| format!(" runway={}", r))
                    .unwrap_or_default()
            ),
            (PatternClearanceGate::Land, false) => "landing clearance revoked".to_string(),
            (_, true) => format!("{} clearance granted", gate.as_str()),
            (_, false) => format!("{} clearance revoked — ATC will call", gate.as_str()),
        },
        None => "error: pattern_fly profile is not active; call engage_pattern_fly first".to_string(),
    }
}

pub fn tool_go_around(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    match with_pattern_profile(ctx, |p| p.go_around()) {
        Some(_) => "go_around triggered".to_string(),
        None => "error: pattern_fly profile is not active; call engage_pattern_fly first".to_string(),
    }
}

pub fn tool_execute_touch_and_go(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    match with_pattern_profile(ctx, |p| p.execute_touch_and_go()) {
        Some(_) => "touch-and-go armed; landing will transition to takeoff_roll instead of rollout".to_string(),
        None => "error: pattern_fly profile is not active; call engage_pattern_fly first".to_string(),
    }
}

pub fn tool_join_pattern(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let runway = match arg_str(args, "runway_id") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    let pilot = ctx.pilot.lock();
    if !pilot.has_profile("pattern_fly") {
        return "error: pattern_fly profile is not active; engage_pattern_fly(airport_ident, runway_ident) first".to_string();
    }
    format!("pattern entry acknowledged runway={}", runway)
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
        return "error: no X-Plane bridge available (running in simple backend?)".to_string();
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
        return "error: no X-Plane bridge available (running in simple backend?)".to_string();
    };
    let val = if engaged { 1.0 } else { 0.0 };
    if let Err(e) = bridge.write_dataref_values(&[(PARKING_BRAKE_RATIO.name.to_string(), val)]) {
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
        return "error: no X-Plane bridge available (running in simple backend?)".to_string();
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
    let warning = if ctx.pilot.lock().has_profile("pattern_fly") {
        " (note: pattern_fly is active and manages flaps per phase — it may override this setting on the next tick)"
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
    let freq_label = ctx
        .bridge
        .as_ref()
        .and_then(|b| b.get_dataref_value(dref))
        .map(|khz| format!("{:.3}", khz / 1000.0))
        .unwrap_or_else(|| "---".to_string());
    let line = format!("[{} {}] {}", radio.to_uppercase(), freq_label, message);
    if let Some(bus) = &ctx.bus {
        bus.push_radio(line.clone());
    } else {
        println!("{}", line);
    }
    {
        let mut br = ctx.recent_broadcasts.lock();
        br.push(line.clone());
        if br.len() > 16 {
            let excess = br.len() - 16;
            br.drain(..excess);
        }
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
/// where `d` is the distance of the inserted leg in feet.
pub fn build_taxi_legs_with_pullout(
    aircraft_pos_ft: crate::types::Vec2,
    aircraft_heading_deg: f64,
    plan_legs: &[taxi_route::TaxiLeg],
    georef: GeoReference,
) -> Result<(Vec<StraightLeg>, Vec<String>, Option<f64>), String> {
    const PULLOUT_THRESHOLD_FT: f64 = 20.0;
    // Reject pullouts that require more than ~135° of turn — that means
    // the first graph node is behind the aircraft, which on a runway is
    // a backtaxi. `forbid_backward_runway_traversals` doesn't help here
    // because the pullout lead-in is a synthetic segment, not a graph
    // edge. 90° is too tight (blocks legitimate perpendicular joins).
    const PULLOUT_MAX_TURN_DEG: f64 = 135.0;

    let mut legs_ft: Vec<StraightLeg> = plan_legs
        .iter()
        .map(|l| StraightLeg {
            start_ft: geodetic_offset_ft(l.from_lat, l.from_lon, georef),
            end_ft: geodetic_offset_ft(l.to_lat, l.to_lon, georef),
        })
        .collect();
    let mut names: Vec<String> = plan_legs.iter().map(|l| l.taxiway_name.clone()).collect();

    let Some(first) = legs_ft.first().copied() else {
        return Ok((legs_ft, names, None));
    };
    let offset = first.start_ft - aircraft_pos_ft;
    let d_ft = offset.length();
    if d_ft <= PULLOUT_THRESHOLD_FT {
        return Ok((legs_ft, names, None));
    }
    let pullout_heading_deg = crate::types::vector_to_heading(offset);
    let turn_deg = crate::types::wrap_degrees_180(
        pullout_heading_deg - aircraft_heading_deg,
    )
    .abs();
    if turn_deg > PULLOUT_MAX_TURN_DEG {
        return Err(format!(
            "pullout lead-in to first taxi node would require a {:.0}° turn from current heading {:.0}° (max {:.0}°) — aircraft is probably on a runway or facing backward; reposition before engaging",
            turn_deg, aircraft_heading_deg, PULLOUT_MAX_TURN_DEG,
        ));
    }
    legs_ft.insert(
        0,
        StraightLeg {
            start_ft: aircraft_pos_ft,
            end_ft: first.start_ft,
        },
    );
    names.insert(0, String::new());
    Ok((legs_ft, names, Some(d_ft)))
}

pub fn tool_engage_line_up(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let bridge = match ctx.bridge.as_ref() {
        Some(b) => b.clone(),
        None => return "error: no X-Plane bridge available; engage_line_up needs the georef".to_string(),
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
    let (start_lat, start_lon) = match resolve_start_latlon(ctx, args) {
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

    // Install the resolved runway into PilotCore so engage_takeoff's
    // alignment guard compares heading against the runway we're lining
    // up on, not whatever startup-default runway was in the config.
    match lookup_runway_for_pattern(ctx, &airport, &runway, "left") {
        Ok((rwy, field_elev)) => install_runway_in_pilot_core(ctx, &airport, rwy, field_elev),
        Err(e) => return format!("error: {}", e),
    }

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
            let graph = match taxi_route::load_graph(conn, &airport) {
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
    let displaced = ctx.pilot.lock().engage_profile(profile);

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
        format_displaced(&displaced)
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

pub fn tool_engage_taxi(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let bridge = match ctx.bridge.as_ref() {
        Some(b) => b.clone(),
        None => {
            return "error: no X-Plane bridge available; engage_taxi needs the georef".to_string();
        }
    };
    let released_brake_ratio = match release_parking_brake_if_set(ctx, &bridge) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
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

    let (start_lat, start_lon) = match resolve_start_latlon(ctx, args) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };

    if let Err(e) = ensure_runway_conn(ctx) {
        return format!("error: {}", e);
    }
    let aircraft_heading_deg = start_heading_deg(&bridge);
    let (plan, face_toward_latlon) = {
        let guard = ctx.runway_conn.lock().unwrap();
        let conn = guard.as_ref().unwrap();
        let graph = match taxi_route::load_graph(conn, &airport) {
            Ok(g) => g,
            Err(e) => return format!("error: {}", e),
        };
        if graph.nodes.is_empty() {
            return format!(
                "error: airport {} has no taxi network in apt.dat",
                airport
            );
        }
        let start_node = match taxi_route::nearest_node(&graph, start_lat, start_lon) {
            Ok(id) => id,
            Err(e) => return format!("error: {}", e),
        };
        let dest_spec = match intersection.as_deref() {
            Some(taxiway) => TaxiDestination::RunwayIntersection {
                runway: &destination_runway,
                taxiway,
            },
            None => TaxiDestination::Runway(&destination_runway),
        };
        let dest = match taxi_route::resolve_destination(conn, &graph, &airport, &dest_spec) {
            Ok(d) => d,
            Err(e) => return format!("error: {}", e),
        };
        let allowed: std::collections::HashSet<&str> =
            via_taxiways.iter().map(|s| s.as_str()).collect();
        let forbidden_directional = taxi_route::forbid_backward_runway_traversals(
            &graph, aircraft_heading_deg, &allowed,
        );
        let dest_nodes = dest.all_node_ids();
        let plan = match taxi_route::plan(
            &graph,
            start_node,
            &dest_nodes,
            &via_taxiways,
            &dest.forbidden_edges,
            &forbidden_directional,
        ) {
            Ok(p) => p,
            Err(e) => return format!("error: {}", e),
        };
        let face = dest.face_toward_for(plan.destination_node);
        (plan, face)
    };

    let georef = bridge.georef();
    let aircraft_pos_ft = geodetic_offset_ft(start_lat, start_lon, georef);
    let (legs_ft, names, pullout_ft) = match build_taxi_legs_with_pullout(
        aircraft_pos_ft,
        aircraft_heading_deg,
        &plan.legs,
        georef,
    ) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
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
    let displaced = ctx.pilot.lock().engage_profile(Box::new(profile));

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
    summary.push_str(&format_displaced(&displaced));
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

    let graph = match taxi_route::load_graph(conn, &airport) {
        Ok(g) => g,
        Err(e) => return format!("error: {}", e),
    };
    if graph.nodes.is_empty() {
        return format!(
            "error: airport {} has no taxi network in apt.dat",
            airport
        );
    }
    let start_node = match taxi_route::nearest_node(&graph, start_lat, start_lon) {
        Ok(id) => id,
        Err(e) => return format!("error: {}", e),
    };
    let dest = match taxi_route::resolve_destination(
        conn,
        &graph,
        &airport,
        &TaxiDestination::Runway(&destination_runway),
    ) {
        Ok(d) => d,
        Err(e) => return format!("error: {}", e),
    };
    let heading_deg = bridge_heading_or_arg(ctx, args);
    let allowed: std::collections::HashSet<&str> =
        via_taxiways.iter().map(|s| s.as_str()).collect();
    let forbidden_directional = taxi_route::forbid_backward_runway_traversals(
        &graph, heading_deg, &allowed,
    );
    let dest_nodes = dest.all_node_ids();
    let plan = match taxi_route::plan(
        &graph,
        start_node,
        &dest_nodes,
        &via_taxiways,
        &dest.forbidden_edges,
        &forbidden_directional,
    ) {
        Ok(p) => p,
        Err(e) => return format!("error: {}", e),
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
    let graph = match taxi_route::load_graph(conn, &airport) {
        Ok(g) => g,
        Err(e) => return format!("error: {}", e),
    };
    if graph.nodes.is_empty() {
        return format!(
            "error: airport {} has no taxi network in apt.dat",
            airport
        );
    }
    let start_node = match taxi_route::nearest_node(&graph, start_lat, start_lon) {
        Ok(id) => id,
        Err(e) => return format!("error: {}", e),
    };
    let resolved = match taxi_route::resolve_parking_destination(
        conn, &graph, &airport, &parking_name,
    ) {
        Ok(r) => r,
        Err(e) => return format!("error: {}", e),
    };
    let heading_deg = bridge_heading_or_arg(ctx, args);
    let allowed: std::collections::HashSet<&str> =
        via_taxiways.iter().map(|s| s.as_str()).collect();
    let forbidden_directional = taxi_route::forbid_backward_runway_traversals(
        &graph, heading_deg, &allowed,
    );
    let plan = match taxi_route::plan(
        &graph,
        start_node,
        &[resolved.nearest_node],
        &via_taxiways,
        &std::collections::HashSet::new(),
        &forbidden_directional,
    ) {
        Ok(p) => p,
        Err(e) => return format!("error: {}", e),
    };
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
    let bridge = match ctx.bridge.as_ref() {
        Some(b) => b.clone(),
        None => {
            return "error: no X-Plane bridge available; engage_park needs the georef".to_string();
        }
    };
    let released_brake_ratio = match release_parking_brake_if_set(ctx, &bridge) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
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
    let (start_lat, start_lon) = match resolve_start_latlon(ctx, args) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };

    if let Err(e) = ensure_runway_conn(ctx) {
        return format!("error: {}", e);
    }
    let aircraft_heading_deg = start_heading_deg(&bridge);
    let (plan, resolved) = {
        let guard = ctx.runway_conn.lock().unwrap();
        let conn = guard.as_ref().unwrap();
        let graph = match taxi_route::load_graph(conn, &airport) {
            Ok(g) => g,
            Err(e) => return format!("error: {}", e),
        };
        if graph.nodes.is_empty() {
            return format!(
                "error: airport {} has no taxi network in apt.dat",
                airport
            );
        }
        let start_node = match taxi_route::nearest_node(&graph, start_lat, start_lon) {
            Ok(id) => id,
            Err(e) => return format!("error: {}", e),
        };
        let resolved = match taxi_route::resolve_parking_destination(
            conn, &graph, &airport, &parking_name,
        ) {
            Ok(r) => r,
            Err(e) => return format!("error: {}", e),
        };
        let allowed: std::collections::HashSet<&str> =
            via_taxiways.iter().map(|s| s.as_str()).collect();
        let forbidden_directional = taxi_route::forbid_backward_runway_traversals(
            &graph, aircraft_heading_deg, &allowed,
        );
        let plan = match taxi_route::plan(
            &graph,
            start_node,
            &[resolved.nearest_node],
            &via_taxiways,
            &std::collections::HashSet::new(),
            &forbidden_directional,
        ) {
            Ok(p) => p,
            Err(e) => return format!("error: {}", e),
        };
        (plan, resolved)
    };

    let georef = bridge.georef();
    let aircraft_pos_ft = geodetic_offset_ft(start_lat, start_lon, georef);
    let (mut legs_ft, mut names, pullout_ft) = match build_taxi_legs_with_pullout(
        aircraft_pos_ft,
        aircraft_heading_deg,
        &plan.legs,
        georef,
    ) {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };

    // Append a short lead-in leg from the last taxi-graph node to the
    // actual 1300 spot coordinates. Parking spots sit off the network, so
    // without this the aircraft would stop at the nearest taxiway node —
    // typically 30-60 ft short of the actual gate / tie-down.
    let spot_pos_ft = geodetic_offset_ft(resolved.spot_lat, resolved.spot_lon, georef);
    if let Some(last) = legs_ft.last().copied() {
        let lead_in = spot_pos_ft - last.end_ft;
        if lead_in.length() > 1.0 {
            legs_ft.push(StraightLeg {
                start_ft: last.end_ft,
                end_ft: spot_pos_ft,
            });
            names.push(format!("(to {})", resolved.spot_name));
        }
    }

    let final_pose = crate::types::TaxiPose {
        position_ft: spot_pos_ft,
        heading_deg: resolved.spot_heading_true_deg,
    };
    let leg_count = legs_ft.len();
    let profile = TaxiProfile::new(legs_ft, names).with_final_pose(final_pose);
    let displaced = ctx.pilot.lock().engage_profile(Box::new(profile));

    let mut summary = format!(
        "engaged park {} → {:?} ({}) — {} legs, {:.0} m",
        plan.airport_ident,
        resolved.spot_name,
        resolved.spot_kind,
        leg_count,
        plan.total_distance_m
    );
    if let Some(v) = released_brake_ratio {
        summary.push_str(&format!(" (released parking brake, ratio was {:.2})", v));
    }
    if let Some(d) = pullout_ft {
        summary.push_str(&format!(" (pullout: {:.0} ft from start)", d));
    }
    summary.push_str(&format!(
        " +pose-target (heading {:.0}°T)",
        resolved.spot_heading_true_deg
    ));
    if !plan.taxiway_sequence.is_empty() {
        summary.push_str(&format!(" via {}", plan.taxiway_sequence.join(" -> ")));
    }
    if let Some(crossings) = format_unique_crossings(&plan.runway_crossings) {
        summary.push_str(&format!(" (crossings: {})", crossings));
    }
    summary.push_str(&format_displaced(&displaced));
    summary
}

pub fn tool_choose_runway_exit(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let taxiway = match args.get("taxiway_name") {
        Some(Value::Null) | None => None,
        Some(Value::String(s)) => Some(s.clone()),
        Some(other) => return format!("error: taxiway_name must be a string or null, got {other}"),
    };

    // Must have pattern_fly engaged to set a preference. Check first so
    // the error message is unambiguous (validation-before-engage would
    // give confusing "runway not found" errors for a pilot who hasn't
    // even engaged a profile yet).
    if !ctx.pilot.lock().has_profile("pattern_fly") {
        return "error: pattern_fly profile is not active; call engage_pattern_fly first".to_string();
    }

    // Validate the name against the real exit list for the active
    // runway. Without this, list_runway_exits could return ["A", "12/30"]
    // and the LLM would happily call choose_runway_exit("E") — the
    // profile would silently accept the bogus name and the rollout
    // controller would never find the exit. Errors from the exit lookup
    // (e.g. runway not present in apt.dat cache) fall through — the
    // preference is best-effort in that case.
    if let Some(name) = &taxiway {
        let (airport, runway) = {
            let cfg = ctx.config.lock();
            (cfg.airport.airport.clone(), cfg.airport.runway.id.clone())
        };
        if let (Some(airport), Some(runway)) = (airport, runway) {
            if let Ok(exits) = runway_exits_for(ctx, &airport, &runway) {
                if !exits.is_empty() && !exits.iter().any(|(n, _)| n == name) {
                    let valid: Vec<String> = exits.iter().map(|(n, _)| n.clone()).collect();
                    return format!(
                        "error: unknown exit {:?} at {} runway {}; valid: {:?}",
                        name, airport, runway, valid
                    );
                }
            }
        }
    }

    let display = taxiway
        .as_deref()
        .map(|t| format!("preferred runway exit set to {:?}", t))
        .unwrap_or_else(|| "preferred runway exit cleared".to_string());
    match with_pattern_profile(ctx, |p| p.set_preferred_exit(taxiway)) {
        Some(_) => display,
        None => "error: pattern_fly profile is not active; call engage_pattern_fly first".to_string(),
    }
}

/// Shared core of list_runway_exits / choose_runway_exit validation.
/// Returns (name, stationing_ft) pairs sorted by stationing, along with
/// the runway's overall length. Errors as a String so either caller can
/// surface it verbatim.
fn runway_exits_for(
    ctx: &ToolContext,
    airport: &str,
    runway: &str,
) -> std::result::Result<Vec<(String, f64)>, String> {
    let (_length_ft, exits) = runway_exits_with_length(ctx, airport, runway)?;
    Ok(exits)
}

/// Geometry + exits packet returned by the shared runway-exit
/// helper. The caller can project an arbitrary lat/lon onto the runway
/// centerline (via `geodetic_offset_ft(.., georef).dot(forward)`) to
/// compare positions against the exit stationings.
struct RunwayExitInfo {
    length_ft: f64,
    georef: GeoReference,
    forward: Vec2,
    exits: Vec<(String, f64)>,
}

fn runway_exits_with_length(
    ctx: &ToolContext,
    airport: &str,
    runway: &str,
) -> std::result::Result<(f64, Vec<(String, f64)>), String> {
    runway_exits_detail(ctx, airport, runway).map(|d| (d.length_ft, d.exits))
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
    let thresh: Result<(f64, f64, f64, f64), _> = conn.query_row(
        "SELECT \
             CASE WHEN le_ident = ? OR le_ident = ? THEN le_latitude_deg  ELSE he_latitude_deg  END, \
             CASE WHEN le_ident = ? OR le_ident = ? THEN le_longitude_deg ELSE he_longitude_deg END, \
             CASE WHEN le_ident = ? OR le_ident = ? THEN le_heading_degT  ELSE he_heading_degT  END, \
             length_ft \
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
        |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?)),
    );
    let (thr_lat, thr_lon, course, length_ft) = match thresh {
        Ok(v) => v,
        Err(e) => return Err(format!("runway {:?} not found at {}: {}", runway, airport, e)),
    };

    let georef = GeoReference {
        threshold_lat_deg: thr_lat,
        threshold_lon_deg: thr_lon,
    };
    let forward = heading_to_vector(course, 1.0);

    let graph = match taxi_route::load_graph(conn, airport) {
        Ok(g) => g,
        Err(e) => return Err(e.to_string()),
    };
    let mut filtered: Vec<(String, f64)> = Vec::new();
    for e in &graph.edges {
        if !taxi_route::edge_conflicts_with_runway(e, runway) {
            continue;
        }
        if e.name.is_empty() {
            continue;
        }
        let Some(n1) = graph.nodes.get(&e.from_node) else { continue };
        let Some(n2) = graph.nodes.get(&e.to_node) else { continue };
        let p1 = geodetic_offset_ft(n1.latitude_deg, n1.longitude_deg, georef);
        let p2 = geodetic_offset_ft(n2.latitude_deg, n2.longitude_deg, georef);
        let s1 = p1.x * forward.x + p1.y * forward.y;
        let s2 = p2.x * forward.x + p2.y * forward.y;
        let stationing = s1.min(s2).max(0.0);
        if stationing > length_ft + 500.0 {
            continue;
        }
        filtered.push((e.name.clone(), stationing));
    }
    filtered.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.partial_cmp(&b.1).unwrap()));
    filtered.dedup_by(|a, b| a.0 == b.0);
    filtered.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    Ok(RunwayExitInfo {
        length_ft,
        georef,
        forward,
        exits: filtered,
    })
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
    let current_st = ctx.bridge.as_deref().and_then(|b| {
        let lat = b.get_dataref_value(LATITUDE_DEG.name)?;
        let lon = b.get_dataref_value(LONGITUDE_DEG.name)?;
        let p = geodetic_offset_ft(lat, lon, detail.georef);
        Some(p.x * detail.forward.x + p.y * detail.forward.y)
    });

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
    for (name, st) in &detail.exits {
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

fn resolve_start_latlon(
    ctx: &ToolContext,
    args: &Map<String, Value>,
) -> Result<(f64, f64)> {
    let lat = args.get("start_lat").and_then(|v| v.as_f64());
    let lon = args.get("start_lon").and_then(|v| v.as_f64());
    match (lat, lon) {
        (Some(a), Some(b)) => Ok((a, b)),
        _ => {
            let bridge = ctx
                .bridge
                .as_ref()
                .ok_or_else(|| anyhow!(
                    "aircraft position unavailable; pass start_lat and start_lon or connect to X-Plane"
                ))?;
            let a = bridge
                .get_dataref_value(LATITUDE_DEG.name)
                .ok_or_else(|| anyhow!("aircraft latitude not yet available"))?;
            let b = bridge
                .get_dataref_value(LONGITUDE_DEG.name)
                .ok_or_else(|| anyhow!("aircraft longitude not yet available"))?;
            Ok((a, b))
        }
    }
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

// ---------- dispatch ----------

pub fn dispatch_tool(call: &Value, ctx: &ToolContext) -> String {
    let Some(name) = call.get("name").and_then(|v| v.as_str()) else {
        return "error: tool call missing name".to_string();
    };
    let args_json = call.get("arguments").and_then(|v| v.as_str()).unwrap_or("{}");
    let args: Value = if args_json.is_empty() {
        Value::Object(Map::new())
    } else {
        match serde_json::from_str(args_json) {
            Ok(v) => v,
            Err(e) => return format!("error: invalid arguments JSON: {}", e),
        }
    };
    let Value::Object(map) = args else {
        return "error: arguments must be an object".to_string();
    };
    match name {
        "get_status" => tool_get_status(ctx, &map),
        "sleep" => tool_sleep(ctx, &map),
        "engage_heading_hold" => tool_engage_heading_hold(ctx, &map),
        "engage_altitude_hold" => tool_engage_altitude_hold(ctx, &map),
        "engage_speed_hold" => tool_engage_speed_hold(ctx, &map),
        "engage_cruise" => tool_engage_cruise(ctx, &map),
        "engage_pattern_fly" => tool_engage_pattern_fly(ctx, &map),
        "engage_takeoff" => tool_engage_takeoff(ctx, &map),
        "takeoff_checklist" => tool_takeoff_checklist(ctx, &map),
        "disengage_profile" => tool_disengage_profile(ctx, &map),
        "list_profiles" => tool_list_profiles(ctx, &map),
        "extend_pattern_leg" => tool_extend_pattern_leg(ctx, &map),
        "execute_pattern_turn" => tool_execute_pattern_turn(ctx, &map),
        "set_pattern_clearance" => tool_set_pattern_clearance(ctx, &map),
        "go_around" => tool_go_around(ctx, &map),
        "execute_touch_and_go" => tool_execute_touch_and_go(ctx, &map),
        "join_pattern" => tool_join_pattern(ctx, &map),
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
        "list_runway_exits" => tool_list_runway_exits(ctx, &map),
        "engage_line_up" => tool_engage_line_up(ctx, &map),
        other => format!("error: unknown tool {:?}", other),
    }
}

// ---------- tool schemas ----------

pub fn tool_schemas() -> Vec<Value> {
    fn schema(name: &str, description: &str, properties: Value, required: &[&str]) -> Value {
        json!({
            "type": "function",
            "name": name,
            "description": description,
            "strict": true,
            "parameters": {
                "type": "object",
                "properties": properties,
                "required": required,
                "additionalProperties": false,
            }
        })
    }
    vec![
        schema(
            "get_status",
            "Return a JSON snapshot of aircraft state, phase, and active profiles.",
            json!({}),
            &[],
        ),
        schema(
            "sleep",
            "End this turn explicitly. The pilot continues flying the active profiles; you wake up when the next operator/ATC message arrives, when a flight-phase or profile change fires a state-change heartbeat, or when the idle-cadence heartbeat fires (default every 30 s). Pass suppress_idle_heartbeat_s=N to skip the idle cadence for the next N seconds when you're confident the current state is stable for a while (capped at 600 s; state-change heartbeats still fire immediately). Use null when you have no reason to extend the default cadence.",
            json!({
                "suppress_idle_heartbeat_s": {
                    "type": ["number", "null"],
                    "description": "Optional. Suppress the idle-cadence heartbeat for this many seconds of wall time. Phase changes, profile changes, crash edges, and inbound operator/ATC messages still wake you immediately. Capped server-side at 600 s. Pass null to leave the default idle cadence unchanged."
                }
            }),
            &["suppress_idle_heartbeat_s"],
        ),
        schema(
            "engage_heading_hold",
            "Engage heading-hold on the lateral axis. Displaces any other lateral-axis profile. By default takes the shortest-path turn to the target heading. If the operator or ATC specifies a turn direction (e.g. 'turn right to 290'), pass turn_direction='right' or 'left' to force that direction even when the other way is shorter; the direction lock clears automatically once within 5 degrees of target so the autopilot will not overshoot.",
            json!({
                "heading_deg": {"type": "number", "description": "Target heading in degrees true, 0-360."},
                "turn_direction": {
                    "type": ["string", "null"],
                    "enum": ["left", "right", null],
                    "description": "Optional forced turn direction: 'left', 'right', or null for shortest-path (default)."
                }
            }),
            &["heading_deg", "turn_direction"],
        ),
        schema(
            "engage_altitude_hold",
            "Engage altitude-hold on the vertical axis using TECS. Displaces any other vertical-axis profile.",
            json!({"altitude_ft": {"type": "number", "description": "Target altitude in feet MSL."}}),
            &["altitude_ft"],
        ),
        schema(
            "engage_speed_hold",
            "Engage speed-hold on the speed axis. Displaces any other speed-axis profile.",
            json!({"speed_kt": {"type": "number", "description": "Target indicated airspeed in knots."}}),
            &["speed_kt"],
        ),
        schema(
            "engage_cruise",
            "Atomically install heading_hold + altitude_hold + speed_hold in a single tool call. Use this when transitioning out of a three-axis profile (takeoff, pattern_fly) into a steady cross-country leg — engaging the three single-axis holds separately briefly leaves the vertical and speed axes uncovered between calls, while this installs them under one lock so the control loop never sees an intermediate state. Displaces any lateral-, vertical-, or speed-axis profile currently engaged (including takeoff and pattern_fly, which own all three).",
            json!({
                "heading_deg": {"type": "number", "description": "Target true heading, 0-360."},
                "altitude_ft": {"type": "number", "description": "Target altitude MSL in feet."},
                "speed_kt": {"type": "number", "description": "Target indicated airspeed in knots."}
            }),
            &["heading_deg", "altitude_ft", "speed_kt"],
        ),
        schema(
            "engage_pattern_fly",
            "Engage the deterministic mission pilot anchored at a specific runway. Owns all three axes. The tool looks up the runway in the database, anchors the pattern geometry at its real threshold, and positions the phase machine at start_phase. All four arguments are REQUIRED — if you don't know which runway you're on, call get_status + sql_query first. Use start_phase='takeoff_roll' to start on the ground for takeoff, or start_phase='pattern_entry' to join an existing pattern from cruise. Typical takeoff call: engage_pattern_fly(airport_ident='KSEA', runway_ident='16L', side='left', start_phase='takeoff_roll'). Typical join-from-cruise call: engage_pattern_fly(airport_ident='KSEA', runway_ident='16L', side='left', start_phase='pattern_entry').",
            json!({
                "airport_ident": {"type": "string", "description": "ICAO airport code (e.g. 'KSEA')."},
                "runway_ident": {"type": "string", "description": "Runway end identifier (e.g. '16L', '34R')."},
                "side": {"type": "string", "description": "Traffic pattern side: 'left' (standard US) or 'right'."},
                "start_phase": {"type": "string", "description": "Initial phase for the phase machine. 'takeoff_roll' if starting on the ground, 'pattern_entry' if joining from cruise, 'downwind' if already on downwind, etc."}
            }),
            &["airport_ident", "runway_ident", "side", "start_phase"],
        ),
        schema(
            "engage_takeoff",
            "Start the takeoff sequence on the specified airport + runway: resolve the runway's threshold and course from apt.dat, verify the aircraft is actually on that runway (within 100 ft of centerline, inside the runway's along-track length, heading within 15° of course, ground speed < 3 kt), then go full power, hold centerline via the rollout controller, rotate at Vr, and climb at Vy along the runway track. Owns all three axes. Does NOT auto-disengage — once you're safely airborne, transition by engaging another profile (engage_heading_hold, engage_altitude_hold, engage_pattern_fly, etc.), which will displace this one via axis-ownership conflict. REFUSES to engage if the parking brake is set, the aircraft is not on the specified runway, or heading is off course — the error message names the specific check. Call takeoff_checklist first to see a human-readable status summary.",
            json!({
                "airport_ident": {"type": "string", "description": "Airport identifier as stored in apt.dat (e.g. 'KSFO')."},
                "runway_ident": {"type": "string", "description": "Runway end ident you're departing on (e.g. '12', '28R')."}
            }),
            &["airport_ident", "runway_ident"],
        ),
        schema(
            "takeoff_checklist",
            "Return a takeoff-readiness checklist with each item marked [OK], [ACTION], [ERROR], or [REMINDER]. Reads live state (parking brake, flaps, gear, on-ground, active profiles). Call this before engage_takeoff and address every [ACTION] item — the most common miss is a set parking brake, which will also cause engage_takeoff to refuse.",
            json!({}),
            &[],
        ),
        schema(
            "disengage_profile",
            "Remove the profile with the given name. Orphaned axes fall back to idle profiles.",
            json!({"name": {"type": "string", "description": "Profile name, e.g. 'heading_hold'."}}),
            &["name"],
        ),
        schema(
            "list_profiles",
            "Return a comma-separated list of currently active profile names.",
            json!({}),
            &[],
        ),
        schema(
            "extend_pattern_leg",
            "Lengthen a traffic-pattern leg. Use for ATC instructions like 'extend your downwind 2 miles' or 'extend crosswind'. Only 'crosswind' and 'downwind' are extensible. Crosswind extension widens the effective downwind offset (the pattern is flown further out to the side); downwind extension pushes the base-turn point further past the threshold. Requires pattern_fly to be active. mode='add' (default) accumulates onto the current extension; mode='set' replaces it with the exact value.",
            json!({
                "leg": {"type": "string", "enum": ["crosswind", "downwind"], "description": "Which leg to extend."},
                "extension_ft": {"type": "number", "description": "Extension in feet. With mode='add' adds to the current extension; with mode='set' replaces it (clamped to >= 0)."},
                "mode": {"type": ["string", "null"], "enum": ["add", "set", null], "description": "How to apply the value. 'add' (default) or 'set'. null means 'add'."}
            }),
            &["leg", "extension_ft", "mode"],
        ),
        schema(
            "execute_pattern_turn",
            "Force an immediate pattern-phase turn, bypassing the automatic geometric trigger. Use for ATC instructions like 'turn base now', 'turn final', 'turn crosswind early'. For 'base' in the DOWNWIND phase the base leg is rebuilt dynamically from the aircraft's current runway-frame position, so it works correctly even after an extended downwind. Valid legs: crosswind (InitialClimb->Crosswind), downwind (Crosswind->Downwind), base (Downwind->Base), final (Base->Final). Requires pattern_fly to be active.",
            json!({
                "leg": {"type": "string", "enum": ["crosswind", "downwind", "base", "final"], "description": "The turn to execute immediately."}
            }),
            &["leg"],
        ),
        schema(
            "set_pattern_clearance",
            "Grant or revoke an ATC clearance gate for a pattern-phase transition. Revoking a gate holds the aircraft on its current leg until ATC calls it (e.g. ATC says 'I'll call your base' => set_pattern_clearance(gate='turn_base', granted=false)). Granting re-enables the automatic transition. The 'land' gate records (but does not enforce) landing authorization — the LLM is responsible for calling go_around if landing isn't cleared. Defaults: all gates granted. Use runway_id only when gate='land'.",
            json!({
                "gate": {"type": "string", "enum": ["turn_crosswind", "turn_downwind", "turn_base", "turn_final", "land"], "description": "Which clearance gate to set."},
                "granted": {"type": "boolean", "description": "true to grant (auto-transition enabled); false to revoke (aircraft holds on current leg until execute_pattern_turn is called or clearance is granted)."},
                "runway_id": {"type": ["string", "null"], "description": "Runway identifier for land clearance (e.g. '16L'). Only meaningful when gate='land' and granted=true. Pass null otherwise."}
            }),
            &["gate", "granted", "runway_id"],
        ),
        schema(
            "go_around",
            "Command an immediate go-around. Requires pattern_fly to be active.",
            json!({}),
            &[],
        ),
        schema(
            "execute_touch_and_go",
            "Declare that the upcoming landing is a touch-and-go. Must be called during BASE or FINAL (before the wheels touch). On touchdown the phase machine will skip ROLLOUT (braking) and transition directly to TAKEOFF_ROLL: full throttle, flaps retract to 10°, no brakes. The aircraft re-accelerates, rotates, and flies another pattern. The flag auto-clears on TAKEOFF_ROLL → ROTATE so the next approach defaults to a normal full-stop landing unless you call execute_touch_and_go again. Requires pattern_fly to be active.",
            json!({}),
            &[],
        ),
        schema(
            "join_pattern",
            "Acknowledge a pattern join instruction. Requires pattern_fly to be active.",
            json!({"runway_id": {"type": "string", "description": "Runway identifier for the pattern."}}),
            &["runway_id"],
        ),
        schema(
            "tune_radio",
            "Tune a COM radio to a frequency in MHz.",
            json!({
                "radio": {"type": "string", "description": "Radio name: 'com1' or 'com2'."},
                "frequency_mhz": {"type": "number", "description": "Frequency in MHz, e.g. 118.30."}
            }),
            &["radio", "frequency_mhz"],
        ),
        schema(
            "broadcast_on_radio",
            "Transmit a text message over a COM radio. This is the ONLY way your words reach ATC or anyone outside the cockpit — plain-text replies are visible to the operator only and are not transmitted. Always use this tool to acknowledge clearances, read back ATC instructions, make position calls, or make any external radio call. Use standard aviation phraseology. Typically com1 is the active comm radio.",
            json!({
                "radio": {"type": "string", "description": "Radio name: 'com1' or 'com2'."},
                "message": {"type": "string", "description": "The exact words to transmit, e.g. 'Seattle Tower, Cessna 123AB, runway 16L cleared for takeoff'."}
            }),
            &["radio", "message"],
        ),
        schema(
            "set_parking_brake",
            "Engage or release the parking brake. Unlike the toe brakes, the parking brake holds its state without continuous input, so this is the right tool for 'set the brake and hold it'.",
            json!({"engaged": {"type": "boolean", "description": "True to engage (set) the parking brake, false to release it."}}),
            &["engaged"],
        ),
        schema(
            "set_flaps",
            "Set the flap handle position. Valid settings for the C172 are 0, 10, 20, or 30 degrees. Note: when pattern_fly is active, it manages flaps automatically per flight phase and will override this setting on the next tick. Use this tool when flying with single-axis profiles (heading_hold, altitude_hold, speed_hold) or during ground ops.",
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
            ENGAGE_LINE_UP_DESCRIPTION,
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
                    "description": "Starting latitude. If null, the current aircraft position from X-Plane is used."
                },
                "start_lon": {
                    "type": ["number", "null"],
                    "description": "Starting longitude. If null, the current aircraft position from X-Plane is used."
                }
            }),
            &["airport_ident", "runway_ident", "intersection", "start_lat", "start_lon"],
        ),
        schema(
            "engage_taxi",
            ENGAGE_TAXI_DESCRIPTION,
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
                    "description": "Starting latitude. If null, the current aircraft position from X-Plane is used."
                },
                "start_lon": {
                    "type": ["number", "null"],
                    "description": "Starting longitude. If null, the current aircraft position from X-Plane is used."
                }
            }),
            &["airport_ident", "destination_runway", "via_taxiways", "intersection", "start_lat", "start_lon"],
        ),
        schema(
            "plan_taxi_route",
            PLAN_TAXI_ROUTE_DESCRIPTION,
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
                    "description": "Starting latitude. If null, the current aircraft position from X-Plane is used."
                },
                "start_lon": {
                    "type": ["number", "null"],
                    "description": "Starting longitude. If null, the current aircraft position from X-Plane is used."
                }
            }),
            &["airport_ident", "destination_runway", "via_taxiways", "start_lat", "start_lon"],
        ),
        schema(
            "engage_park",
            ENGAGE_PARK_DESCRIPTION,
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
                    "description": "Starting latitude. If null, the current aircraft position from X-Plane is used."
                },
                "start_lon": {
                    "type": ["number", "null"],
                    "description": "Starting longitude. If null, the current aircraft position from X-Plane is used."
                }
            }),
            &["airport_ident", "parking_name", "via_taxiways", "start_lat", "start_lon"],
        ),
        schema(
            "plan_park_route",
            PLAN_PARK_ROUTE_DESCRIPTION,
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
                    "description": "Starting latitude. If null, the current aircraft position from X-Plane is used."
                },
                "start_lon": {
                    "type": ["number", "null"],
                    "description": "Starting longitude. If null, the current aircraft position from X-Plane is used."
                }
            }),
            &["airport_ident", "parking_name", "via_taxiways", "start_lat", "start_lon"],
        ),
        schema(
            "choose_runway_exit",
            CHOOSE_RUNWAY_EXIT_DESCRIPTION,
            json!({
                "taxiway_name": {
                    "type": ["string", "null"],
                    "description": "Name of the taxiway exit you want to take, e.g. 'A5'. Pass null to clear the current preference (the rollout will then take the first available exit)."
                }
            }),
            &["taxiway_name"],
        ),
        schema(
            "list_runway_exits",
            LIST_RUNWAY_EXITS_DESCRIPTION,
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
    ]
}

const ENGAGE_LINE_UP_DESCRIPTION: &str = "Cross the hold-short onto the \
runway, turn to runway heading, and stop with the nose aligned for takeoff. \
This is the 'line up and wait' clearance in ATC phraseology. Mirrors \
engage_taxi internally — same nose-wheel and ground-speed controllers, \
same sharp-turn slowdown — but the goal is a pose on the runway rather \
than a hold-short.

Use when ATC says 'line up runway X and wait' or 'cleared for takeoff \
runway X' and the aircraft is currently stopped at the hold-short. After \
this completes and the aircraft is aligned, call engage_takeoff to begin \
the takeoff roll.

Requires a live X-Plane bridge (needs the georef to convert the runway \
threshold lat/lon into runway-frame feet). Displaces any active \
three-axis profile.";

const ENGAGE_TAXI_DESCRIPTION: &str = "Engage the ground-taxi autopilot. \
Plans the route via the same logic as plan_taxi_route, then takes control \
of the aircraft: nose-wheel steering tracks the leg centerline, ground \
speed targets ~15 kt on straights / ~5 kt through sharp turns / 0 at the \
final node where the aircraft parks with hold brake applied. Displaces \
any currently-engaged three-axis profile (pattern_fly, takeoff, cruise).

If the parking brake is set when this is called, it is released \
automatically — no need to call set_parking_brake(engaged=False) first. \
Requires the aircraft to be on the ground and a live X-Plane bridge \
(needs the georef anchor to convert the planned lat/lon waypoints into \
the pilot's runway-frame feet).

For clearances where you want to preview the route before committing, \
call plan_taxi_route first — its output has the same summary plus \
leg-by-leg waypoints.";

const ENGAGE_PARK_DESCRIPTION: &str = "Taxi to a parking spot and stop with \
the nose aligned to the spot's painted heading. Internally this plans a \
taxi route to the taxi-network node nearest the 1300 parking spot, appends \
a short lead-in leg to the spot's exact lat/lon, and drives TaxiProfile \
to a final pose at the spot's heading.

Use after rollout has cleared the runway and the aircraft is stopped \
clear. Requires a live X-Plane bridge (needs the georef to convert the \
parking lat/lon into runway-frame feet). Displaces any active three-axis \
profile. If the parking brake is set, it's released automatically.

For candidates, query the `parking_spots` view via sql_query filtered by \
airport_ident and matching `categories` against your aircraft class \
(e.g. categories LIKE '%props%' for a single-engine piston, or \
operation_type = 'general_aviation' for a GA ramp).";

const PLAN_PARK_ROUTE_DESCRIPTION: &str = "Preview the taxi route to a \
parking spot without engaging the autopilot. Same output shape as \
plan_taxi_route plus the resolved spot coordinates and heading. Use to \
sanity-check a route before calling engage_park.";

const CHOOSE_RUNWAY_EXIT_DESCRIPTION: &str = "Record a preferred runway-exit \
taxiway for the post-landing rollout. The rollout controller slows to \
turnoff speed by the exit's stationing and, once clear of the hold-short \
line, the pattern profile auto-releases — the aircraft stops clear of the \
runway with parking brake set, ready for engage_park.

Call on final (or any time before touchdown) with the taxiway name you \
want to use, e.g. 'A5'. If the chosen exit is un-makable (aircraft still \
fast at that stationing), the rollout falls back to the next available \
exit and logs it. Passing null clears any prior preference.";

const LIST_RUNWAY_EXITS_DESCRIPTION: &str = "List the taxiway exits for a \
landing runway. Returns each candidate taxiway name plus its approximate \
stationing in feet from the runway's landing threshold, so you can pick \
an exit appropriate for your rollout distance. Uses the 1204 active-zone \
annotations attached to taxi_edges.";

const PLAN_TAXI_ROUTE_DESCRIPTION: &str = "Plan a taxi route across an airport \
surface. Returns the ordered sequence of waypoint legs (lat/lon pairs plus the \
taxiway each segment rides), total distance, and any runway active-zones the \
path crosses (arrival / departure / ILS-critical). PLANNING ONLY in this phase \
— no autopilot engagement.

Use when the operator or ATC issues a taxi clearance like 'taxi via Alpha, \
Delta, hold short runway 31'. Pass the taxiway names in order through \
via_taxiways (e.g. ['A', 'D']). The planner runs a constrained Dijkstra over \
the apt.dat taxi network: it requires the path to traverse the given taxiways \
in order, allows unnamed connector edges at the start/end to cover gate \
lead-ins and runway lead-outs, and resolves destination_runway to the taxi \
node nearest the runway threshold.

If via_taxiways is [] the planner returns the shortest overall route and \
reports which taxiways it actually used so you can echo them back to the \
user.

If the airport has no taxi network in apt.dat (small uncontrolled strips), \
this tool errors. Use sql_query against the `taxi_nodes` / `taxi_edges` \
tables for ad-hoc exploration.";

// Derived from sim_pilot/llm/tools.py::SQL_QUERY_DESCRIPTION. Rust-side
// departures from the Python version:
//   * Source is X-Plane's apt.dat parsed into three zstd GeoParquet tables
//     (airports, runways, comms) instead of the ourairports CSV. Endpoints
//     are 8-decimal; headings and lengths are derived from endpoint geodesy.
//   * "ALWAYS prefix spatial functions with ST_" — the LLM was observed
//     emitting bare POINT(...) which fails on DuckDB.
//   * ST_Distance_Sphere-only-accepts-POINT warning + crosstrack /
//     along-track example — the LLM tried to pass a LINESTRING and fail.
const SQL_QUERY_DESCRIPTION: &str = "Run an arbitrary read-only SQL query against the runway/airport/comms \
database (derived from X-Plane's apt.dat). This is the AUTHORITATIVE source \
for runway and airport facts — never guess a runway identifier, airport \
code, course, length, elevation, or ATC frequency; query for it. The \
backend is DuckDB with the spatial extension loaded, so you have full ST_* \
geospatial functions available. Results are tab-separated with a header \
row, truncated to 50 rows.

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

For concrete route planning ('taxi via A, D to 31') prefer the `plan_taxi_route` \
tool — it handles the Dijkstra and destination resolution for you. Use \
taxi_nodes/taxi_edges directly for exploratory queries ('what taxiways connect \
at node 42?', 'which edges conflict with runway 31 on departure?').

Spatial functions (DuckDB spatial extension): ST_Point(lon, lat) — note that
LONGITUDE comes first — plus ST_Distance_Sphere(p1, p2) which returns meters
along the great circle. Use these for 'nearest runway', 'within N nm', etc.
Other ST_* functions (ST_Distance, ST_DWithin, ST_AsGeoJSON) are also available.
ALWAYS prefix spatial functions with ST_ (e.g. ST_Point, not POINT).
ST_Distance_Sphere ONLY accepts two POINTs. It will error on a LINESTRING.
For point-to-centerline distance (crosstrack offset) use the local flat-earth
projection shown below — do NOT pass a runway LINESTRING to ST_Distance_Sphere.

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
  -- use that as the runway_ident for engage_pattern_fly / engage_takeoff.

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
