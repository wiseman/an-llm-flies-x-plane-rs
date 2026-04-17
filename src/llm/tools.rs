//! Tool handlers + JSON schemas for the LLM. Mirrors sim_pilot/llm/tools.py.
//!
//! Each tool is dispatched from a parsed `function_call` item returned by the
//! Responses API. Handlers mutate `PilotCore` state (engaging/disengaging
//! profiles, arming pattern-fly triggers) or talk to the X-Plane bridge
//! (radio, parking brake, flaps). The SQL query tool uses DuckDB with the
//! spatial extension loaded so the agent can do real geospatial lookups.

use std::fs;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use anyhow::{anyhow, Result};
use parking_lot::Mutex as PLMutex;
use serde_json::{json, Map, Value};

use crate::bus::SimBus;
use crate::config::ConfigBundle;
use crate::core::mission_manager::{PilotCore, StatusSnapshot};
use crate::core::profiles::{
    AltitudeHoldProfile, HeadingHoldProfile, PatternFlyProfile, SpeedHoldProfile, TakeoffProfile,
    TaxiProfile,
};
use crate::data::parquet::{
    AIRPORTS_FILE, COMMS_FILE, RUNWAYS_FILE, TAXI_EDGES_FILE, TAXI_NODES_FILE,
};
use crate::guidance::runway_geometry::RunwayFrame;
use crate::guidance::taxi_route::{self, TaxiDestination};
use crate::sim::datarefs::{
    COM1_FREQUENCY_HZ_833, COM2_FREQUENCY_HZ_833, FLAP_HANDLE_REQUEST_RATIO, LATITUDE_DEG,
    LONGITUDE_DEG, PARKING_BRAKE_RATIO,
};
use crate::sim::xplane_bridge::{geodetic_offset_ft, GeoReference};
use crate::types::{heading_to_vector, FlightPhase, Runway, StraightLeg, TrafficSide};

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

pub fn build_status_payload(snapshot: Option<&StatusSnapshot>, bridge: Option<&dyn ToolBridge>) -> Value {
    let Some(snapshot) = snapshot else {
        return json!({ "status": "uninitialized" });
    };
    let state = &snapshot.state;
    let (lat_deg, lon_deg) = match bridge {
        Some(b) => (
            b.get_dataref_value(LATITUDE_DEG.name),
            b.get_dataref_value(LONGITUDE_DEG.name),
        ),
        None => (None, None),
    };
    json!({
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
    })
}

fn round_f64(v: f64, places: u32) -> f64 {
    let m = 10f64.powi(places as i32);
    (v * m).round() / m
}

fn parking_brake_ratio(ctx: &ToolContext) -> Option<f64> {
    ctx.bridge.as_ref().and_then(|b| b.get_dataref_value(PARKING_BRAKE_RATIO.name))
}

// ---------- individual tool handlers ----------

pub fn tool_get_status(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    let pilot = ctx.pilot.lock();
    let snap = pilot.latest_snapshot.clone();
    drop(pilot);
    let bridge_ref: Option<&dyn ToolBridge> = ctx.bridge.as_deref();
    serde_json::to_string(&build_status_payload(snap.as_ref(), bridge_ref)).unwrap()
}

pub fn tool_sleep(_ctx: &ToolContext, _args: &Map<String, Value>) -> String {
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
/// clamps to `[500, length/3]` to produce the aim-point. Returning 2000 here
/// targets the standard 1000 ft aim point past the threshold on any runway
/// long enough to support it; short strips fall out via the downstream clamp.
fn synthesize_touchdown_zone_ft(_length_ft: f64) -> f64 {
    2000.0
}

fn ensure_runway_conn(ctx: &ToolContext) -> Result<()> {
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
    for p in [&airports, &runways, &comms, &taxi_nodes, &taxi_edges] {
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

fn open_apt_dat_parquet(
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
           SELECT * FROM read_parquet('{taxi_edges}');",
        airports = airports.display(),
        runways = runways.display(),
        comms = comms.display(),
        taxi_nodes = dir_join(airports, TAXI_NODES_FILE).display(),
        taxi_edges = dir_join(airports, TAXI_EDGES_FILE).display(),
    ))?;
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
    // The CSV's le_/he_ lat/lon is the physical pavement end. The real
    // landing threshold is that far *inside* the runway (displaced_threshold
    // metres/feet toward the far end), so we shift the runway-frame origin
    // along the runway heading by the displaced-threshold distance. This
    // matters: at KEMT runway 1 the displaced threshold is 289 ft, so the
    // aim point and TDZ would otherwise sit 289 ft earlier than real.
    let pavement_end_ft = geodetic_offset_ft(lat, lon, bridge.georef());
    let threshold_ft = pavement_end_ft
        + heading_to_vector(course, displaced_ft.unwrap_or(0.0));
    let field_elev = runway_elev.unwrap_or(0.0);
    let resolved_length = length_ft.unwrap_or(5000.0);
    Ok((
        Runway {
            id: Some(runway_ident.to_string()),
            threshold_ft,
            course_deg: course,
            length_ft: resolved_length,
            touchdown_zone_ft: synthesize_touchdown_zone_ft(resolved_length),
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

pub fn tool_engage_takeoff(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    let pb = parking_brake_ratio(ctx);
    if let Some(v) = pb {
        if v >= 0.5 {
            return format!(
                "error: parking brake is SET (ratio={:.2}) — call set_parking_brake(engaged=False) first, then retry engage_takeoff",
                v
            );
        }
    }

    // Alignment + stopped gate. The LLM was observed calling
    // engage_takeoff ~1 s after engage_line_up, which displaced the
    // in-progress line-up at ~90° off runway heading and started the
    // takeoff roll in the wrong direction. These two checks refuse the
    // engage until the aircraft is actually on the centerline pointed
    // down the runway.
    {
        let pilot = ctx.pilot.lock();
        if let Some(snap) = pilot.latest_snapshot.as_ref() {
            let runway_course = pilot.runway_frame.runway.course_deg;
            let heading_err = crate::types::wrap_degrees_180(runway_course - snap.state.heading_deg).abs();
            if heading_err > 15.0 {
                return format!(
                    "error: aircraft heading {:.0}° is not aligned with runway course {:.0}° (Δ={:.0}° > 15°) — wait for engage_line_up to finish (aircraft needs to be stopped on the runway centerline, nose pointed down the runway) before calling engage_takeoff. Poll get_status until heading matches runway course and gs_kt is near zero.",
                    snap.state.heading_deg, runway_course, heading_err
                );
            }
            if snap.state.gs_kt > 3.0 {
                return format!(
                    "error: aircraft is moving at {:.1} kt — stop on the runway centerline before engage_takeoff (usually means engage_line_up is still in progress — wait for it to finish)",
                    snap.state.gs_kt
                );
            }
        }
    }

    let new_config = ctx.config.lock().clone();
    let runway_frame = ctx.pilot.lock().runway_frame.clone();
    let profile = Box::new(TakeoffProfile::new(new_config, runway_frame));
    let displaced = ctx.pilot.lock().engage_profile(profile);
    format!(
        "engaged takeoff {}{}",
        pilot_reference_label(ctx),
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

    lines.push("  [REMINDER] identify the runway you are on (get_status for lat/lon + heading, then sql_query with the 'What runway am I on?' example) before committing to the roll".to_string());
    lines.push("  [REMINDER] acknowledge takeoff clearance on the radio (broadcast_on_radio) if ATC has issued one".to_string());
    lines.push("".to_string());
    if action_needed {
        lines.push("One or more items need action. Fix them, then re-run this checklist or proceed to engage_takeoff.".to_string());
    } else {
        lines.push("All items OK. Call engage_takeoff() to begin the roll.".to_string());
    }
    lines.join("\n")
}

pub fn tool_engage_approach(_ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let runway = args.get("runway_id").and_then(|v| v.as_str()).unwrap_or("");
    format!("error: engage_approach not yet implemented (requested runway={})", runway)
}

pub fn tool_engage_route_follow(_ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    "error: engage_route_follow not yet implemented".to_string()
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

pub fn tool_extend_downwind(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let ext = match arg_f64(args, "extension_ft") {
        Ok(v) => v,
        Err(e) => return format!("error: {}", e),
    };
    match with_pattern_profile(ctx, |p| {
        p.extend_downwind(ext);
        p.pattern_extension_ft
    }) {
        Some(total) => format!(
            "extended downwind by {:.0}ft; total extension={:.0}ft",
            ext, total
        ),
        None => "error: pattern_fly profile is not active".to_string(),
    }
}

pub fn tool_turn_base_now(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    match with_pattern_profile(ctx, |p| p.turn_base_now()) {
        Some(_) => "turn_base_now triggered".to_string(),
        None => "error: pattern_fly profile is not active".to_string(),
    }
}

pub fn tool_go_around(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    match with_pattern_profile(ctx, |p| p.go_around()) {
        Some(_) => "go_around triggered".to_string(),
        None => "error: pattern_fly profile is not active".to_string(),
    }
}

pub fn tool_execute_touch_and_go(ctx: &ToolContext, _args: &Map<String, Value>) -> String {
    match with_pattern_profile(ctx, |p| p.execute_touch_and_go()) {
        Some(_) => "touch-and-go armed; landing will transition to takeoff_roll instead of rollout".to_string(),
        None => "error: pattern_fly profile is not active".to_string(),
    }
}

pub fn tool_cleared_to_land(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let runway = match arg_str(args, "runway_id") {
        Ok(s) => s.to_string(),
        Err(e) => return format!("error: {}", e),
    };
    match with_pattern_profile(ctx, |p| p.cleared_to_land(Some(&runway))) {
        Some(_) => format!("cleared to land runway={}", runway),
        None => "error: pattern_fly profile is not active".to_string(),
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
    let dref = match radio.as_str() {
        "com1" => COM1_FREQUENCY_HZ_833.name,
        "com2" => COM2_FREQUENCY_HZ_833.name,
        other => return format!("error: unknown radio {:?} (expected com1, com2)", other),
    };
    let freq_khz = (freq * 1000.0).round() as i64;
    if let Err(e) = bridge.write_dataref_values(&[(dref.to_string(), freq_khz as f64)]) {
        return format!("error: {}", e);
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
    if radio != "com1" && radio != "com2" {
        return format!("error: unknown radio {:?} (expected com1, com2)", radio);
    }
    let line = format!("[BROADCAST {}] {}", radio, message);
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
    plan_legs: &[taxi_route::TaxiLeg],
    georef: GeoReference,
) -> (Vec<StraightLeg>, Vec<String>, Option<f64>) {
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
        return (legs_ft, names, None);
    };
    let offset = first.start_ft - aircraft_pos_ft;
    let d_ft = offset.length();
    if d_ft <= PULLOUT_THRESHOLD_FT {
        return (legs_ft, names, None);
    }
    legs_ft.insert(
        0,
        StraightLeg {
            start_ft: aircraft_pos_ft,
            end_ft: first.start_ft,
        },
    );
    names.insert(0, String::new());
    (legs_ft, names, Some(d_ft))
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
    let (start_lat, start_lon) = match (
        args.get("start_lat").and_then(|v| v.as_f64()),
        args.get("start_lon").and_then(|v| v.as_f64()),
    ) {
        (Some(lat), Some(lon)) => (lat, lon),
        _ => {
            let lat = bridge.get_dataref_value(LATITUDE_DEG.name);
            let lon = bridge.get_dataref_value(LONGITUDE_DEG.name);
            match (lat, lon) {
                (Some(lat), Some(lon)) => (lat, lon),
                _ => return "error: aircraft position unavailable; pass start_lat and start_lon or wait for a dataref update".to_string(),
            }
        }
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
    let threshold_ft = geodetic_offset_ft(thr_lat, thr_lon, georef);
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

    let profile = Box::new(TaxiProfile::new(legs_ft, names));
    let displaced = ctx.pilot.lock().engage_profile(profile);

    let approach_dist = (threshold_ft - aircraft_pos_ft).length();
    format!(
        "engaged line_up {} runway {} — crossing {:.0} ft, final heading {:.0}°{}",
        airport,
        runway,
        approach_dist,
        heading_deg,
        format_displaced(&displaced)
    )
}

pub fn tool_engage_taxi(ctx: &ToolContext, args: &Map<String, Value>) -> String {
    let bridge = match ctx.bridge.as_ref() {
        Some(b) => b.clone(),
        None => {
            return "error: no X-Plane bridge available; engage_taxi needs the georef".to_string();
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
    let via_taxiways: Vec<String> = args
        .get("via_taxiways")
        .and_then(|v| v.as_array())
        .map(|a| a.iter().filter_map(|v| v.as_str().map(str::to_string)).collect())
        .unwrap_or_default();

    let (start_lat, start_lon) = match (
        args.get("start_lat").and_then(|v| v.as_f64()),
        args.get("start_lon").and_then(|v| v.as_f64()),
    ) {
        (Some(lat), Some(lon)) => (lat, lon),
        _ => {
            let lat = bridge.get_dataref_value(LATITUDE_DEG.name);
            let lon = bridge.get_dataref_value(LONGITUDE_DEG.name);
            match (lat, lon) {
                (Some(lat), Some(lon)) => (lat, lon),
                _ => {
                    return "error: aircraft position unavailable; pass start_lat and start_lon or wait for a dataref update".to_string();
                }
            }
        }
    };

    if let Err(e) = ensure_runway_conn(ctx) {
        return format!("error: {}", e);
    }
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
        let dest = match taxi_route::resolve_destination(
            conn,
            &graph,
            &airport,
            &TaxiDestination::Runway(&destination_runway),
        ) {
            Ok(d) => d,
            Err(e) => return format!("error: {}", e),
        };
        let face = dest.face_toward_latlon;
        let plan = match taxi_route::plan(
            &graph,
            start_node,
            dest.node,
            &via_taxiways,
            &dest.forbidden_edges,
        ) {
            Ok(p) => p,
            Err(e) => return format!("error: {}", e),
        };
        (plan, face)
    };

    let georef = bridge.georef();
    let aircraft_pos_ft = geodetic_offset_ft(start_lat, start_lon, georef);
    let (mut legs_ft, mut names, pullout_ft) =
        build_taxi_legs_with_pullout(aircraft_pos_ft, &plan.legs, georef);
    // Face-runway orientation leg: once we reach the hold-short node the
    // taxi sequencer still has us pointed down the last leg (typically
    // parallel to the runway along taxiway A). Append a short segment
    // that starts at the hold-short and points at the runway pavement
    // just beyond, so the final stop-ramp pivots the nose 60–90° to
    // face the runway.
    //
    // 25 ft is the working tuning: most of the pivot completes in the
    // first ~20 ft of the leg while the alignment-limit creep speed
    // (4 kt) + full nose-wheel + pivot brake carve the aircraft around.
    // Once advance_if_reached fires at 30 ft from the leg end, the
    // stop-ramp brings the aircraft to rest within a few more feet.
    // Shorter than this the pivot doesn't complete; longer, the
    // aircraft rolls materially past the painted hold-short line.
    let mut face_leg_appended = false;
    if let (Some((flat, flon)), Some(last)) = (face_toward_latlon, legs_ft.last().copied()) {
        let runway_point_ft = geodetic_offset_ft(flat, flon, georef);
        let to_runway = runway_point_ft - last.end_ft;
        let d = to_runway.length();
        if d > 1.0 {
            let unit = to_runway * (1.0 / d);
            let face_end = last.end_ft + unit * 25.0;
            legs_ft.push(StraightLeg { start_ft: last.end_ft, end_ft: face_end });
            names.push("(face runway)".to_string());
            face_leg_appended = true;
        }
    }
    let leg_count = legs_ft.len();
    let profile = Box::new(TaxiProfile::new(legs_ft, names));
    let displaced = ctx.pilot.lock().engage_profile(profile);

    let mut summary = format!(
        "engaged taxi {} — {} legs, {:.0} m",
        plan.airport_ident, leg_count, plan.total_distance_m
    );
    if let Some(d) = pullout_ft {
        summary.push_str(&format!(" (pullout: {:.0} ft from start)", d));
    }
    if face_leg_appended {
        summary.push_str(" +face-runway");
    }
    if !plan.taxiway_sequence.is_empty() {
        summary.push_str(&format!(" via {}", plan.taxiway_sequence.join(" -> ")));
    }
    if !plan.runway_crossings.is_empty() {
        let mut seen = std::collections::HashSet::new();
        let unique: Vec<&str> = plan
            .runway_crossings
            .iter()
            .filter(|z| seen.insert(z.as_str()))
            .map(String::as_str)
            .collect();
        summary.push_str(&format!(" (crossings: {})", unique.join("; ")));
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
    let via_taxiways: Vec<String> = args
        .get("via_taxiways")
        .and_then(|v| v.as_array())
        .map(|a| a.iter().filter_map(|v| v.as_str().map(str::to_string)).collect())
        .unwrap_or_default();
    let start_lat = args.get("start_lat").and_then(|v| v.as_f64());
    let start_lon = args.get("start_lon").and_then(|v| v.as_f64());

    let (start_lat, start_lon) = match (start_lat, start_lon) {
        (Some(lat), Some(lon)) => (lat, lon),
        _ => match ctx
            .bridge
            .as_ref()
            .and_then(|b| Some((b.get_dataref_value(LATITUDE_DEG.name)?, b.get_dataref_value(LONGITUDE_DEG.name)?)))
        {
            Some(v) => v,
            None => {
                return "error: aircraft position unavailable; pass start_lat and start_lon or connect to X-Plane".to_string();
            }
        },
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
    let plan = match taxi_route::plan(
        &graph,
        start_node,
        dest.node,
        &via_taxiways,
        &dest.forbidden_edges,
    ) {
        Ok(p) => p,
        Err(e) => return format!("error: {}", e),
    };
    format_taxi_plan(&plan)
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
    if !plan.runway_crossings.is_empty() {
        let mut seen = std::collections::HashSet::new();
        let unique: Vec<&str> = plan
            .runway_crossings
            .iter()
            .filter(|z| seen.insert(z.as_str()))
            .map(String::as_str)
            .collect();
        out.push_str(&format!("  runway zones: {}\n", unique.join("; ")));
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
        "engage_approach" => tool_engage_approach(ctx, &map),
        "engage_route_follow" => tool_engage_route_follow(ctx, &map),
        "disengage_profile" => tool_disengage_profile(ctx, &map),
        "list_profiles" => tool_list_profiles(ctx, &map),
        "extend_downwind" => tool_extend_downwind(ctx, &map),
        "turn_base_now" => tool_turn_base_now(ctx, &map),
        "go_around" => tool_go_around(ctx, &map),
        "execute_touch_and_go" => tool_execute_touch_and_go(ctx, &map),
        "cleared_to_land" => tool_cleared_to_land(ctx, &map),
        "join_pattern" => tool_join_pattern(ctx, &map),
        "tune_radio" => tool_tune_radio(ctx, &map),
        "broadcast_on_radio" => tool_broadcast_on_radio(ctx, &map),
        "set_parking_brake" => tool_set_parking_brake(ctx, &map),
        "set_flaps" => tool_set_flaps(ctx, &map),
        "sql_query" => tool_sql_query(ctx, &map),
        "plan_taxi_route" => tool_plan_taxi_route(ctx, &map),
        "engage_taxi" => tool_engage_taxi(ctx, &map),
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
            "End this turn explicitly. The pilot continues flying the active profiles; you wake up when the next operator/ATC message arrives.",
            json!({}),
            &[],
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
            "Start the takeoff sequence: full power, accelerate on the runway centerline, rotate at Vr, then hold a straight-ahead climb at Vy along the runway track. Owns all three axes. Does NOT auto-disengage — once you're safely airborne, transition by engaging another profile (engage_heading_hold, engage_altitude_hold, engage_pattern_fly, etc.), which will displace this one via axis-ownership conflict. REFUSES to engage if the parking brake is set; call takeoff_checklist first to see what needs to be fixed.",
            json!({}),
            &[],
        ),
        schema(
            "takeoff_checklist",
            "Return a takeoff-readiness checklist with each item marked [OK], [ACTION], [ERROR], or [REMINDER]. Reads live state (parking brake, flaps, gear, on-ground, active profiles). Call this before engage_takeoff and address every [ACTION] item — the most common miss is a set parking brake, which will also cause engage_takeoff to refuse.",
            json!({}),
            &[],
        ),
        schema(
            "engage_approach",
            "Engage a final-approach profile for the given runway. Not yet implemented.",
            json!({"runway_id": {"type": "string", "description": "Runway identifier, e.g. '16L'."}}),
            &["runway_id"],
        ),
        schema(
            "engage_route_follow",
            "Engage route-follow guidance along a list of waypoints. Not yet implemented.",
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
            "extend_downwind",
            "Extend the downwind leg of the traffic pattern. Requires pattern_fly to be active.",
            json!({"extension_ft": {"type": "number", "description": "Additional downwind distance in feet."}}),
            &["extension_ft"],
        ),
        schema(
            "turn_base_now",
            "Trigger the base turn immediately. Requires pattern_fly to be active and phase DOWNWIND.",
            json!({}),
            &[],
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
            "cleared_to_land",
            "Record a cleared-to-land clearance. Requires pattern_fly to be active.",
            json!({"runway_id": {"type": "string", "description": "Runway identifier, e.g. '16L'."}}),
            &["runway_id"],
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
                "start_lat": {
                    "type": ["number", "null"],
                    "description": "Starting latitude. If null, the current aircraft position from X-Plane is used."
                },
                "start_lon": {
                    "type": ["number", "null"],
                    "description": "Starting longitude. If null, the current aircraft position from X-Plane is used."
                }
            }),
            &["airport_ident", "runway_ident", "start_lat", "start_lon"],
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

Requires the aircraft to be on the ground with the parking brake \
RELEASED. Requires a live X-Plane bridge (needs the georef anchor to \
convert the planned lat/lon waypoints into the pilot's runway-frame feet).

For clearances where you want to preview the route before committing, \
call plan_taxi_route first — its output has the same summary plus \
leg-by-leg waypoints.";

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

  -- 'What runway am I on?' / 'Where am I?' — call get_status first to get
  -- your lat/lon AND heading, then run this with all three substituted in.
  -- IMPORTANT details baked into this query (do not skip any of them):
  --   * each runway row stores BOTH ends (le_* and he_*); the LEAST() of the
  --     two spatial distances is your distance to the nearest threshold on
  --     that runway, which matters when you're sitting at the high-numbered
  --     end (e.g. 34R, whose threshold is in the he_* columns).
  --   * active_ident is computed in SQL from cos(heading - end_heading).
  --     cos handles angular wraparound automatically: heading 0.6 is next
  --     to 360 (cos ~ +1), not across from it. Do NOT compute this column
  --     in your head — read it from the query result.
  --   * no bounding box. ORDER BY the spatial distance over the whole
  --     table. DuckDB scans 43k rows in milliseconds.
  SELECT airport_ident, le_ident, he_ident, length_ft, surface,
         le_heading_degT, he_heading_degT,
         LEAST(
           ST_Distance_Sphere(ST_Point(le_longitude_deg, le_latitude_deg),
                              ST_Point(<lon>, <lat>)),
           ST_Distance_Sphere(ST_Point(he_longitude_deg, he_latitude_deg),
                              ST_Point(<lon>, <lat>))
         ) AS dist_m,
         CASE
           WHEN cos(radians(le_heading_degT - <hdg>)) >
                cos(radians(he_heading_degT - <hdg>))
           THEN le_ident
           ELSE he_ident
         END AS active_ident
  FROM runways
  WHERE closed = 0
    AND le_latitude_deg IS NOT NULL
    AND he_latitude_deg IS NOT NULL
  ORDER BY dist_m
  LIMIT 5;

  -- The runway you are on is the top row's active_ident column, provided
  -- dist_m is small (< ~100 m if you're actually on a runway). Do not pick
  -- le_ident or he_ident yourself — always read active_ident.

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

// keep fs import alive for future disk-based config overrides
#[allow(dead_code)]
fn _keep_fs_in_scope(_: &dyn Fn() -> std::io::Result<()>) -> Option<fs::File> {
    None
}
