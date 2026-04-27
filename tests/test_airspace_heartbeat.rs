//! Integration test: end-to-end airspace injection into the status
//! payload that drives the LLM heartbeat. Stands up a minimal parquet
//! fixture with two airspaces, constructs a ToolContext with a FakeBridge
//! providing lat/lon datarefs, and verifies that `build_status_payload`
//! emits the right `airspace` dict for inside / under / projected
//! scenarios. Phase gating and the on-ground gate are exercised too.
//!
//! Fixture geography, in decimal degrees for quick reference:
//!   SF Bravo (class B): lat 37.50..37.833, lon -122.5..-122.167,
//!                       floor 1500, ceiling 10000
//!   Oakland Delta (D): lat 37.667..37.75, lon -122.333..-122.25,
//!                       floor GND,  ceiling 2500

use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;

use anyhow::Result;
use parking_lot::Mutex as PLMutex;
use tempfile::TempDir;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::{PilotCore, StatusSnapshot};
use xplane_pilot::data::{airspace, apt_dat, parquet};
use xplane_pilot::llm::tools::{build_status_payload, airspace_source, ToolBridge, ToolContext};
use xplane_pilot::sim::xplane_bridge::GeoReference;
use xplane_pilot::types::{ActuatorCommands, AircraftState, FlightPhase, Vec2};

struct FakeBridge {
    values: PLMutex<HashMap<String, f64>>,
}

impl FakeBridge {
    fn new(lat: f64, lon: f64) -> Arc<Self> {
        let mut v = HashMap::new();
        v.insert("sim/flightmodel/position/latitude".into(), lat);
        v.insert("sim/flightmodel/position/longitude".into(), lon);
        Arc::new(FakeBridge {
            values: PLMutex::new(v),
        })
    }
}

impl ToolBridge for FakeBridge {
    fn georef(&self) -> GeoReference {
        GeoReference {
            threshold_lat_deg: 0.0,
            threshold_lon_deg: 0.0,
        }
    }
    fn get_dataref_value(&self, name: &str) -> Option<f64> {
        self.values.lock().get(name).copied()
    }
    fn write_dataref_values(&self, _updates: &[(String, f64)]) -> Result<()> {
        Ok(())
    }
}

const AIRSPACE_FIXTURE: &str = "\
AC B
AN SAN FRANCISCO
AL 1500 MSL
AH 10000 MSL
DP 37:30:00 N 122:30:00 W
DP 37:50:00 N 122:30:00 W
DP 37:50:00 N 122:10:00 W
DP 37:30:00 N 122:10:00 W

AC D
AN OAKLAND
AL GND
AH 2500 MSL
DP 37:40:00 N 122:20:00 W
DP 37:45:00 N 122:20:00 W
DP 37:45:00 N 122:15:00 W
DP 37:40:00 N 122:15:00 W
";

const MINIMAL_APT: &str = "1     10 0 0 KSFO stub\n99\n";

fn build_fixture() -> (TempDir, std::path::PathBuf) {
    let dir = TempDir::new().unwrap();
    let apt = apt_dat::parse(MINIMAL_APT.as_bytes()).unwrap();
    let airspaces = airspace::parse(AIRSPACE_FIXTURE.as_bytes()).unwrap();
    parquet::write_cache(&apt, &airspaces, dir.path()).unwrap();
    let p = dir.path().to_path_buf();
    (dir, p)
}

fn make_ctx(bridge: Arc<dyn ToolBridge>, cache_dir: &Path) -> ToolContext {
    let cfg = load_default_config_bundle();
    let pilot = Arc::new(PLMutex::new(PilotCore::new(cfg.clone())));
    let mut ctx = ToolContext::new(pilot, cfg);
    ctx.bridge = Some(bridge);
    ctx.apt_dat_cache_dir = Some(cache_dir.to_path_buf());
    ctx
}

fn snapshot(
    _lat: f64,
    _lon: f64,
    alt_msl_ft: f64,
    track_deg: f64,
    gs_kt: f64,
    phase: FlightPhase,
    on_ground: bool,
) -> StatusSnapshot {
    let mut state = AircraftState::synthetic_default();
    state.alt_msl_ft = alt_msl_ft;
    state.track_deg = track_deg;
    state.heading_deg = track_deg;
    state.gs_kt = gs_kt;
    state.vs_fpm = 0.0;
    state.on_ground = on_ground;
    state.position_ft = Vec2::new(0.0, 0.0);
    StatusSnapshot {
        t_sim: 0.0,
        active_profiles: Vec::new(),
        phase: Some(phase),
        state,
        last_commands: ActuatorCommands {
            aileron: 0.0,
            elevator: 0.0,
            rudder: 0.0,
            throttle: 0.0,
            flaps: None,
            gear_down: Some(true),
            brakes: 0.0,
            pivot_brake: 0.0,
        },
        last_guidance: None,
        go_around_reason: None,
        airport_ident: None,
        runway_id: None,
        field_elevation_ft: None,
        debug_lines: Vec::new(),
        completed_profiles: Vec::new(),
        profile_mode_line_suffixes: Vec::new(),
        mission_goal: None,
        active_clearance: None,
        transition_hint: None,
        lateral_owner_idx: None,
        vertical_owner_idx: None,
        speed_owner_idx: None,
    }
}

#[test]
fn inside_class_b_populates_heartbeat_airspace() {
    let (_tmp, dir) = build_fixture();
    let bridge = FakeBridge::new(37.70, -122.30);
    let ctx = make_ctx(bridge.clone(), &dir);
    let src = airspace_source(&ctx).expect("airspace source present");
    let snap = snapshot(37.70, -122.30, 2000.0, 0.0, 0.0, FlightPhase::Cruise, false);
    let out = build_status_payload(Some(&snap), ctx.bridge.as_deref(), Some(&src));
    let a = out.get("airspace").expect("airspace field present");
    let inside = a.get("inside").unwrap().as_array().unwrap();
    let names: Vec<&str> = inside.iter().map(|e| e["name"].as_str().unwrap()).collect();
    assert!(names.contains(&"SAN FRANCISCO"));
    assert!(names.contains(&"OAKLAND"));
}

#[test]
fn below_bravo_shelf_reports_under_with_margin() {
    let (_tmp, dir) = build_fixture();
    // 1000 ft inside the Bravo footprint (lon -122.25 is inside the
    // -122.5 to -122.167 range, lat 37.55 is inside 37.50 to 37.833)
    // but well clear of Oakland Delta's footprint.
    let bridge = FakeBridge::new(37.55, -122.25);
    let ctx = make_ctx(bridge.clone(), &dir);
    let src = airspace_source(&ctx).unwrap();
    let snap = snapshot(37.55, -122.25, 1000.0, 0.0, 0.0, FlightPhase::Cruise, false);
    let out = build_status_payload(Some(&snap), ctx.bridge.as_deref(), Some(&src));
    let under = out["airspace"]["under"].as_array().unwrap();
    let sf = under
        .iter()
        .find(|e| e["name"].as_str() == Some("SAN FRANCISCO"))
        .expect("SF Bravo should be UNDER");
    assert_eq!(sf["vertical_clearance_ft"].as_f64().unwrap(), 500.0);
    assert_eq!(sf["bottom_ft_msl"].as_f64().unwrap(), 1500.0);
    assert_eq!(sf["top_ft_msl"].as_f64().unwrap(), 10000.0);
    // Inside should be empty at this point/altitude.
    assert!(out["airspace"]["inside"].as_array().unwrap().is_empty());
}

#[test]
fn projected_entry_appears_in_through_120s() {
    let (_tmp, dir) = build_fixture();
    // West of Bravo's west edge, heading due east at 120 kt. In 120 s we
    // travel ~4 nm east, enough to penetrate the -122.5 boundary. At 3000
    // ft we'll land in the 1500-10000 vertical slab.
    let bridge = FakeBridge::new(37.70, -122.55);
    let ctx = make_ctx(bridge.clone(), &dir);
    let src = airspace_source(&ctx).unwrap();
    let snap = snapshot(37.70, -122.55, 3000.0, 90.0, 120.0, FlightPhase::Cruise, false);
    let out = build_status_payload(Some(&snap), ctx.bridge.as_deref(), Some(&src));
    let through = out["airspace"]["through_120s"].as_array().unwrap();
    let names: Vec<&str> = through
        .iter()
        .map(|e| e["name"].as_str().unwrap())
        .collect();
    assert!(names.contains(&"SAN FRANCISCO"), "got {:?}", names);
    let sf = through
        .iter()
        .find(|e| e["name"].as_str() == Some("SAN FRANCISCO"))
        .unwrap();
    assert_eq!(sf["class"].as_str().unwrap(), "B");
    assert_eq!(sf["t_sec"].as_i64().unwrap(), 120);
}

#[test]
fn on_ground_suppresses_airspace_field() {
    let (_tmp, dir) = build_fixture();
    let bridge = FakeBridge::new(37.70, -122.30);
    let ctx = make_ctx(bridge.clone(), &dir);
    let src = airspace_source(&ctx).unwrap();
    let snap = snapshot(
        37.70,
        -122.30,
        50.0,
        0.0,
        0.0,
        FlightPhase::Preflight,
        true,
    );
    let out = build_status_payload(Some(&snap), ctx.bridge.as_deref(), Some(&src));
    assert!(out.get("airspace").is_none());
}

#[test]
fn takeoff_roll_suppresses_airspace_field() {
    let (_tmp, dir) = build_fixture();
    let bridge = FakeBridge::new(37.70, -122.30);
    let ctx = make_ctx(bridge.clone(), &dir);
    let src = airspace_source(&ctx).unwrap();
    let snap = snapshot(
        37.70,
        -122.30,
        50.0,
        10.0,
        30.0,
        FlightPhase::TakeoffRoll,
        false,
    );
    let out = build_status_payload(Some(&snap), ctx.bridge.as_deref(), Some(&src));
    assert!(out.get("airspace").is_none());
}

#[test]
fn no_source_means_no_airspace_field() {
    let (_tmp, dir) = build_fixture();
    let bridge = FakeBridge::new(37.70, -122.30);
    let ctx = make_ctx(bridge.clone(), &dir);
    let snap = snapshot(37.70, -122.30, 2000.0, 0.0, 0.0, FlightPhase::Cruise, false);
    // Pass None for the airspace source: the heartbeat payload should
    // simply omit the key.
    let out = build_status_payload(Some(&snap), ctx.bridge.as_deref(), None);
    assert!(out.get("airspace").is_none());
    // Sanity: the non-airspace fields are still there.
    assert!(out.get("lat_deg").is_some());
    assert!(out.get("phase").is_some());
}
