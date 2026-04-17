//! Port of tests/test_tool_dispatch.py. Covers the full tool surface the LLM
//! uses, backed by a `FakeBridge` for writes that would otherwise hit X-Plane.

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::Result;
use parking_lot::Mutex as PLMutex;
use serde_json::json;
use tempfile::TempDir;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::{PilotCore, StatusSnapshot};
use xplane_pilot::core::profiles::{HeadingHoldProfile, PatternFlyProfile, TakeoffProfile};
use xplane_pilot::llm::tools::{dispatch_tool, ToolBridge, ToolContext};
use xplane_pilot::sim::xplane_bridge::GeoReference;
use xplane_pilot::types::{ActuatorCommands, AircraftState, Vec2};

struct FakeBridge {
    writes: PLMutex<Vec<HashMap<String, f64>>>,
    values: PLMutex<HashMap<String, f64>>,
    georef: GeoReference,
}

impl FakeBridge {
    fn new(initial: &[(&str, f64)], lat: f64, lon: f64) -> Arc<Self> {
        let mut v = HashMap::new();
        for (k, val) in initial {
            v.insert((*k).to_string(), *val);
        }
        Arc::new(FakeBridge {
            writes: PLMutex::new(Vec::new()),
            values: PLMutex::new(v),
            georef: GeoReference {
                threshold_lat_deg: lat,
                threshold_lon_deg: lon,
            },
        })
    }

    fn writes(&self) -> Vec<HashMap<String, f64>> {
        self.writes.lock().clone()
    }
}

impl ToolBridge for FakeBridge {
    fn georef(&self) -> GeoReference { self.georef }
    fn get_dataref_value(&self, name: &str) -> Option<f64> {
        self.values.lock().get(name).copied()
    }
    fn write_dataref_values(&self, updates: &[(String, f64)]) -> Result<()> {
        let mut map = HashMap::new();
        for (k, v) in updates {
            map.insert(k.clone(), *v);
            self.values.lock().insert(k.clone(), *v);
        }
        self.writes.lock().push(map);
        Ok(())
    }
}

fn make_ctx(bridge: Option<Arc<dyn ToolBridge>>, cache_dir: Option<PathBuf>) -> ToolContext {
    let cfg = load_default_config_bundle();
    let pilot = Arc::new(PLMutex::new(PilotCore::new(cfg.clone())));
    let mut ctx = ToolContext::new(pilot, cfg);
    ctx.bridge = bridge;
    ctx.apt_dat_cache_dir = cache_dir;
    ctx
}

fn call(name: &str, args: serde_json::Value) -> serde_json::Value {
    json!({
        "name": name,
        "call_id": "call_test",
        "arguments": args.to_string(),
    })
}

/// Base airports/runways used by every test fixture. Each tuple is:
///   (airport_ident, elevation_ft, width_m, surface_code,
///    le_ident, le_lat, le_lon, le_disp_m,
///    he_ident, he_lat, he_lon, he_disp_m)
const BASE_RUNWAYS: &[(
    &str,
    u32,
    f64,
    u8,
    &str,
    f64,
    f64,
    f64,
    &str,
    f64,
    f64,
    f64,
)] = &[
    ("KSEA", 432, 45.72, 1, "16L", 47.4638, -122.308, 0.0, "34R", 47.4312, -122.308, 0.0),
    ("KSEA", 432, 45.72, 2, "16C", 47.4638, -122.311, 0.0, "34C", 47.4380, -122.311, 0.0),
    ("KSEA", 432, 45.72, 2, "16R", 47.4638, -122.318, 0.0, "34L", 47.4405, -122.318, 0.0),
    ("KBFI",  21, 60.96, 1, "14R", 47.5301, -122.3015, 0.0, "32L", 47.5111, -122.2876, 0.0),
    ("KJFK",  12, 45.72, 1, "04L", 40.6235,  -73.7944, 0.0, "22R", 40.6432,  -73.7821, 0.0),
];

/// Assemble an apt.dat-style string covering the base runways plus any
/// additional raw apt.dat snippets the caller wants to stack on. Each extra
/// snippet must be a full airport block (row 1 followed by row 100s).
fn synthesize_apt_dat(extra_blocks: &[String]) -> String {
    let mut out = String::from("A\n1200 test fixture\n\n");
    let mut last_ident: Option<&str> = None;
    for (ident, elev, width_m, surf, le_i, le_lat, le_lon, le_d, he_i, he_lat, he_lon, he_d) in
        BASE_RUNWAYS
    {
        if last_ident != Some(*ident) {
            out.push_str(&format!("1 {} 0 0 {} {} Test Airport\n", elev, ident, ident));
            last_ident = Some(*ident);
        }
        out.push_str(&format!(
            "100 {width_m:.2} {surf} 0 0.25 0 2 0 \
             {le_i} {le_lat:.6} {le_lon:.6} {le_d:.2} 0 0 0 0 0 \
             {he_i} {he_lat:.6} {he_lon:.6} {he_d:.2} 0 0 0 0 0\n"
        ));
    }
    for block in extra_blocks {
        out.push_str(block);
        if !block.ends_with('\n') {
            out.push('\n');
        }
    }
    out.push_str("99\n");
    out
}

/// Build the three apt.dat-derived parquet files into `dir`. Returns `dir`
/// unchanged for ergonomic chaining at the call site.
fn build_fake_parquet(dir: &std::path::Path, extra_blocks: &[String]) -> PathBuf {
    let apt = synthesize_apt_dat(extra_blocks);
    let parsed = xplane_pilot::data::apt_dat::parse(apt.as_bytes())
        .expect("apt.dat fixture parses");
    xplane_pilot::data::parquet::write_cache(&parsed, dir)
        .expect("parquet cache writes");
    dir.to_path_buf()
}

// ---------- basic dispatch ----------

#[test]
fn unknown_tool_returns_error() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&json!({"name": "not_a_tool", "arguments": "{}"}), &ctx);
    assert!(r.starts_with("error:"));
    assert!(r.contains("unknown tool"));
}

#[test]
fn invalid_arguments_json_returns_error() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&json!({"name": "engage_heading_hold", "arguments": "not-json"}), &ctx);
    assert!(r.starts_with("error:"));
}

#[test]
fn missing_required_arg_returns_error() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("engage_heading_hold", json!({})), &ctx);
    assert!(r.starts_with("error:"));
}

// ---------- profiles ----------

#[test]
fn engage_heading_hold_engages_profile() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("engage_heading_hold", json!({"heading_deg": 270.0})), &ctx);
    assert!(r.contains("heading_hold"));
    let names = ctx.pilot.lock().list_profile_names();
    assert!(names.contains(&"heading_hold".to_string()));
    assert!(!names.contains(&"idle_lateral".to_string()));
}

#[test]
fn engage_cruise_installs_three_holds_atomically() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(
        &call(
            "engage_cruise",
            json!({"heading_deg": 130.0, "altitude_ft": 2500.0, "speed_kt": 95.0}),
        ),
        &ctx,
    );
    assert!(r.contains("engaged cruise"));
    let mut names = ctx.pilot.lock().list_profile_names();
    names.sort();
    assert_eq!(
        names,
        vec![
            "altitude_hold".to_string(),
            "heading_hold".to_string(),
            "speed_hold".to_string()
        ]
    );
}

#[test]
fn engage_cruise_from_takeoff_displaces_three_axis_profile() {
    let ctx = make_ctx(None, None);
    // Engage takeoff directly — the tool now requires airport/runway args
    // and a real snapshot; this test only cares about the axis-displace
    // behaviour of engage_cruise, so bypass the tool surface.
    {
        let mut p = ctx.pilot.lock();
        let cfg = ctx.config.lock().clone();
        let rf = p.runway_frame.clone();
        p.engage_profile(Box::new(TakeoffProfile::new(cfg, rf)));
    }
    assert_eq!(ctx.pilot.lock().list_profile_names(), vec!["takeoff"]);
    let r = dispatch_tool(
        &call(
            "engage_cruise",
            json!({"heading_deg": 180.0, "altitude_ft": 3000.0, "speed_kt": 100.0}),
        ),
        &ctx,
    );
    assert!(r.contains("displaced"));
    assert!(r.contains("takeoff"));
}

#[test]
fn execute_touch_and_go_arms_pattern_fly_flag() {
    let ctx = make_ctx(None, None);
    {
        let mut p = ctx.pilot.lock();
        let cfg = ctx.config.lock().clone();
        let rf = p.runway_frame.clone();
        p.engage_profile(Box::new(PatternFlyProfile::new(cfg, rf)));
    }
    let r = dispatch_tool(&call("execute_touch_and_go", json!({})), &ctx);
    assert!(r.contains("touch-and-go armed"));
}

#[test]
fn execute_touch_and_go_without_pattern_fly_errors() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("execute_touch_and_go", json!({})), &ctx);
    assert!(r.starts_with("error:"));
    assert!(r.contains("pattern_fly"));
}

#[test]
fn engage_pattern_fly_without_required_args_errors() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("engage_pattern_fly", json!({})), &ctx);
    assert!(r.starts_with("error:"));
    assert!(!ctx.pilot.lock().has_profile("pattern_fly"));
}

#[test]
fn engage_takeoff_refuses_when_parking_brake_set() {
    let bridge = FakeBridge::new(
        &[("sim/cockpit2/controls/parking_brake_ratio", 1.0)],
        47.4638,
        -122.308,
    );
    let ctx = make_ctx(Some(bridge as Arc<dyn ToolBridge>), None);
    let r = dispatch_tool(
        &call(
            "engage_takeoff",
            json!({"airport_ident": "KSEA", "runway_ident": "16L"}),
        ),
        &ctx,
    );
    assert!(r.starts_with("error:"));
    assert!(r.contains("parking brake"));
    assert!(r.contains("set_parking_brake"));
}

#[test]
fn engage_takeoff_requires_airport_and_runway_args() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("engage_takeoff", json!({})), &ctx);
    assert!(r.starts_with("error:"));
    // One of the required-arg complaints must surface.
    assert!(
        r.contains("airport_ident") || r.contains("missing") || r.contains("required"),
        "got: {}",
        r
    );
}

#[test]
fn engage_takeoff_refuses_when_not_on_runway() {
    use xplane_pilot::core::mission_manager::StatusSnapshot;
    use xplane_pilot::types::{ActuatorCommands, AircraftState, Vec2};

    let dir = TempDir::new().unwrap();
    build_fake_parquet(dir.path(), &[]);
    // Georef anchored at KSEA 16L threshold. Aircraft spawned well off
    // the runway — crosstrack check must catch it.
    let bridge = FakeBridge::new(
        &[("sim/cockpit2/controls/parking_brake_ratio", 0.0)],
        47.4638,
        -122.308,
    );
    let ctx = make_ctx(
        Some(bridge.clone() as Arc<dyn ToolBridge>),
        Some(dir.path().to_path_buf()),
    );
    // Stage a snapshot 500 ft east of the threshold (far off-centerline
    // for a runway heading ~180°).
    let mut state = AircraftState::synthetic_default();
    state.on_ground = true;
    state.position_ft = Vec2::new(500.0, 0.0);
    state.heading_deg = 180.0;
    state.gs_kt = 0.0;
    ctx.pilot.lock().latest_snapshot = Some(StatusSnapshot {
        t_sim: 0.0,
        active_profiles: Vec::new(),
        phase: None,
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
    });
    let r = dispatch_tool(
        &call(
            "engage_takeoff",
            json!({"airport_ident": "KSEA", "runway_ident": "16L"}),
        ),
        &ctx,
    );
    assert!(r.starts_with("error:"), "got: {}", r);
    // Position check is the one that should fail.
    assert!(
        r.contains("centerline") || r.contains("along-track"),
        "expected a position-check error, got: {}",
        r
    );
}

#[test]
fn disengage_profile_readds_idles() {
    let ctx = make_ctx(None, None);
    {
        let mut p = ctx.pilot.lock();
        p.engage_profile(Box::new(HeadingHoldProfile::new(270.0, 25.0, None).unwrap()));
    }
    let r = dispatch_tool(&call("disengage_profile", json!({"name": "heading_hold"})), &ctx);
    assert!(r.contains("heading_hold"));
    assert!(ctx.pilot.lock().list_profile_names().contains(&"idle_lateral".to_string()));
}

#[test]
fn list_profiles_returns_comma_separated() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("list_profiles", json!({})), &ctx);
    assert!(r.contains("idle_lateral"));
    assert!(r.contains("idle_vertical"));
    assert!(r.contains("idle_speed"));
}

#[test]
fn engage_approach_returns_not_implemented() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("engage_approach", json!({"runway_id": "16L"})), &ctx);
    assert!(r.contains("not yet implemented"));
}

#[test]
fn extend_downwind_with_pattern_fly_succeeds() {
    let ctx = make_ctx(None, None);
    {
        let mut p = ctx.pilot.lock();
        let cfg = ctx.config.lock().clone();
        let rf = p.runway_frame.clone();
        p.engage_profile(Box::new(PatternFlyProfile::new(cfg, rf)));
    }
    let r = dispatch_tool(&call("extend_downwind", json!({"extension_ft": 1500.0})), &ctx);
    assert!(r.contains("extended downwind"));
}

#[test]
fn turn_base_now_sets_profile_trigger() {
    let ctx = make_ctx(None, None);
    {
        let mut p = ctx.pilot.lock();
        let cfg = ctx.config.lock().clone();
        let rf = p.runway_frame.clone();
        p.engage_profile(Box::new(PatternFlyProfile::new(cfg, rf)));
    }
    dispatch_tool(&call("turn_base_now", json!({})), &ctx);
}

#[test]
fn go_around_without_pattern_fly_errors() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("go_around", json!({})), &ctx);
    assert!(r.starts_with("error:"));
}

// ---------- runway lookups via sql_query ----------

fn make_ctx_with_parquet(cache_dir: &std::path::Path) -> (ToolContext, Arc<FakeBridge>) {
    let bridge = FakeBridge::new(&[], 47.4638, -122.308);
    let ctx = make_ctx(Some(bridge.clone() as Arc<dyn ToolBridge>), Some(cache_dir.to_path_buf()));
    (ctx, bridge)
}

#[test]
fn engage_pattern_fly_for_takeoff_roll() {
    let dir = TempDir::new().unwrap();
    build_fake_parquet(dir.path(), &[]);
    let (ctx, _bridge) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(
        &call(
            "engage_pattern_fly",
            json!({
                "airport_ident": "KSEA",
                "runway_ident": "16L",
                "side": "left",
                "start_phase": "takeoff_roll",
            }),
        ),
        &ctx,
    );
    assert!(!r.contains("error"), "got {}", r);
    assert_eq!(ctx.pilot.lock().list_profile_names(), vec!["pattern_fly"]);
}

#[test]
fn engage_pattern_fly_anchors_at_pavement_end_and_exposes_displacement() {
    // Runway "09" pavement end coincides with the georef anchor
    // (47.4638, -122.308) and extends due east, so pavement_end_ft is
    // (0, 0). The runway frame anchors at the pavement end — NOT at
    // the displaced landing threshold — and stores the displacement
    // separately so takeoff and landing can use the same frame
    // consistently.
    let dir = TempDir::new().unwrap();
    let disp_ft = 88.0_f64 * 3.280_839_895_013_123;
    let extra = [
        "1 0 0 0 KTEST KTEST Test Airport\n\
         100 22.86 1 0 0.25 0 2 0 \
         09 47.463800 -122.308000 88.00 0 0 0 0 0 \
         27 47.463800 -122.287000 0.00 0 0 0 0 0\n"
            .to_string(),
    ];
    build_fake_parquet(dir.path(), &extra);
    let (ctx, _bridge) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(
        &call(
            "engage_pattern_fly",
            json!({
                "airport_ident": "KTEST",
                "runway_ident": "09",
                "side": "left",
                "start_phase": "pattern_entry",
            }),
        ),
        &ctx,
    );
    assert!(!r.contains("error"), "got {}", r);
    let pilot = ctx.pilot.lock();
    let threshold = pilot.runway_frame.runway.threshold_ft;
    // Threshold = pavement end (georef anchor), not displacement-shifted.
    assert!(threshold.x.abs() < 1.0, "east: got {}", threshold.x);
    assert!(threshold.y.abs() < 1.0, "north: got {}", threshold.y);
    // Displacement is exposed as its own field.
    assert!(
        (pilot.runway_frame.runway.displaced_threshold_ft - disp_ft).abs() < 1.0,
        "displaced_threshold_ft: got {}, want {}",
        pilot.runway_frame.runway.displaced_threshold_ft,
        disp_ft
    );
    // Aim point = displaced_threshold + 1000 ft: that's where the aircraft
    // actually wants to touch down on a standard runway with apt.dat
    // displacement.
    let aim = pilot.runway_frame.touchdown_runway_x_ft();
    assert!(
        (aim - (disp_ft + 1000.0)).abs() < 1.0,
        "aim: got {}, want {}",
        aim,
        disp_ft + 1000.0
    );
}

#[test]
fn engage_pattern_fly_unknown_airport_errors() {
    let dir = TempDir::new().unwrap();
    build_fake_parquet(dir.path(), &[]);
    let (ctx, _bridge) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(
        &call(
            "engage_pattern_fly",
            json!({
                "airport_ident": "NOWHERE",
                "runway_ident": "16L",
                "side": "left",
                "start_phase": "pattern_entry",
            }),
        ),
        &ctx,
    );
    assert!(r.starts_with("error:"));
    assert!(r.contains("not found"));
}

#[test]
fn engage_pattern_fly_invalid_start_phase() {
    let dir = TempDir::new().unwrap();
    build_fake_parquet(dir.path(), &[]);
    let (ctx, _bridge) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(
        &call(
            "engage_pattern_fly",
            json!({
                "airport_ident": "KSEA",
                "runway_ident": "16L",
                "side": "left",
                "start_phase": "barrel_roll",
            }),
        ),
        &ctx,
    );
    assert!(r.starts_with("error:"));
    assert!(r.contains("unknown start_phase"));
}

#[test]
fn join_pattern_without_pattern_fly_errors() {
    let dir = TempDir::new().unwrap();
    build_fake_parquet(dir.path(), &[]);
    let (ctx, _bridge) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(&call("join_pattern", json!({"runway_id": "30"})), &ctx);
    assert!(r.starts_with("error:"));
    assert!(r.contains("engage_pattern_fly"));
}

// ---------- radio/brake ----------

#[test]
fn tune_radio_without_bridge_errors() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("tune_radio", json!({"radio": "com1", "frequency_mhz": 118.3})), &ctx);
    assert!(r.starts_with("error:"));
}

#[test]
fn tune_radio_writes_dataref() {
    let bridge = FakeBridge::new(&[], 47.4638, -122.308);
    let ctx = make_ctx(Some(bridge.clone() as Arc<dyn ToolBridge>), None);
    let r = dispatch_tool(&call("tune_radio", json!({"radio": "com1", "frequency_mhz": 118.30})), &ctx);
    assert!(r.contains("118.300 MHz"));
    let writes = bridge.writes();
    assert_eq!(writes.len(), 1);
    let (name, val) = writes[0].iter().next().unwrap();
    assert!(name.contains("com1_frequency_hz_833"));
    assert_eq!(*val, 118_300.0);
}

#[test]
fn broadcast_appends_to_recent_and_logs() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("broadcast_on_radio", json!({"radio": "com1", "message": "hello"})), &ctx);
    assert!(r.contains("hello"));
    let broadcasts = ctx.recent_broadcasts.lock();
    assert_eq!(broadcasts.len(), 1);
    assert!(broadcasts[0].contains("[BROADCAST com1] hello"));
}

#[test]
fn set_parking_brake_engage_writes_1() {
    let bridge = FakeBridge::new(&[], 47.4638, -122.308);
    let ctx = make_ctx(Some(bridge.clone() as Arc<dyn ToolBridge>), None);
    let r = dispatch_tool(&call("set_parking_brake", json!({"engaged": true})), &ctx);
    assert!(r.contains("engaged"));
    let writes = bridge.writes();
    assert_eq!(writes[0]["sim/cockpit2/controls/parking_brake_ratio"], 1.0);
}

#[test]
fn set_parking_brake_release_writes_0() {
    let bridge = FakeBridge::new(&[], 47.4638, -122.308);
    let ctx = make_ctx(Some(bridge.clone() as Arc<dyn ToolBridge>), None);
    let r = dispatch_tool(&call("set_parking_brake", json!({"engaged": false})), &ctx);
    assert!(r.contains("released"));
    let writes = bridge.writes();
    assert_eq!(writes[0]["sim/cockpit2/controls/parking_brake_ratio"], 0.0);
}

// ---------- takeoff checklist ----------

fn seed_snapshot(ctx: &ToolContext, flap_index: i32, on_ground: bool) {
    let state = AircraftState {
        t_sim: 0.0,
        dt: 0.2,
        position_ft: Vec2::ZERO,
        alt_msl_ft: 314.0,
        alt_agl_ft: 0.0,
        pitch_deg: 0.0,
        roll_deg: 0.0,
        heading_deg: 0.0,
        track_deg: 0.0,
        p_rad_s: 0.0,
        q_rad_s: 0.0,
        r_rad_s: 0.0,
        ias_kt: 0.0,
        tas_kt: 0.0,
        gs_kt: 0.0,
        vs_fpm: 0.0,
        ground_velocity_ft_s: Vec2::ZERO,
        flap_index,
        gear_down: true,
        on_ground,
        throttle_pos: 0.0,
        runway_id: None,
        runway_dist_remaining_ft: None,
        runway_x_ft: Some(0.0),
        runway_y_ft: Some(0.0),
        centerline_error_ft: Some(0.0),
        threshold_abeam: false,
        distance_to_touchdown_ft: None,
        stall_margin: 2.0,
    };
    let names = ctx.pilot.lock().list_profile_names();
    ctx.pilot.lock().latest_snapshot = Some(StatusSnapshot {
        t_sim: 0.0,
        active_profiles: names,
        phase: None,
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
    });
}

#[test]
fn checklist_flags_set_parking_brake() {
    let bridge = FakeBridge::new(&[("sim/cockpit2/controls/parking_brake_ratio", 1.0)], 47.4638, -122.308);
    let ctx = make_ctx(Some(bridge as Arc<dyn ToolBridge>), None);
    seed_snapshot(&ctx, 0, true);
    let r = dispatch_tool(&call("takeoff_checklist", json!({})), &ctx);
    assert!(r.contains("[ACTION] parking brake: SET"));
    assert!(r.contains("set_parking_brake(engaged=False)"));
    assert!(r.contains("need action"));
}

#[test]
fn checklist_passes_when_brake_released_and_flaps_ok() {
    let bridge = FakeBridge::new(&[("sim/cockpit2/controls/parking_brake_ratio", 0.0)], 47.4638, -122.308);
    let ctx = make_ctx(Some(bridge as Arc<dyn ToolBridge>), None);
    seed_snapshot(&ctx, 0, true);
    let r = dispatch_tool(&call("takeoff_checklist", json!({})), &ctx);
    assert!(r.contains("[OK]     parking brake: released"));
    assert!(r.contains("[OK]     flaps: 0 deg"));
    assert!(r.contains("[OK]     on ground"));
    assert!(r.contains("All items OK"));
}

#[test]
fn checklist_flags_too_many_flaps() {
    let bridge = FakeBridge::new(&[("sim/cockpit2/controls/parking_brake_ratio", 0.0)], 47.4638, -122.308);
    let ctx = make_ctx(Some(bridge as Arc<dyn ToolBridge>), None);
    seed_snapshot(&ctx, 20, true);
    let r = dispatch_tool(&call("takeoff_checklist", json!({})), &ctx);
    assert!(r.contains("[ACTION] flaps: 20 deg"));
}

#[test]
fn checklist_flags_not_on_ground() {
    let bridge = FakeBridge::new(&[("sim/cockpit2/controls/parking_brake_ratio", 0.0)], 47.4638, -122.308);
    let ctx = make_ctx(Some(bridge as Arc<dyn ToolBridge>), None);
    seed_snapshot(&ctx, 0, false);
    let r = dispatch_tool(&call("takeoff_checklist", json!({})), &ctx);
    assert!(r.contains("[ERROR]  not on ground"));
}

#[test]
fn checklist_no_snapshot_returns_error() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("takeoff_checklist", json!({})), &ctx);
    assert!(r.starts_with("error:"));
}

// ---------- status ----------

#[test]
fn get_status_before_any_update_returns_uninitialized() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("get_status", json!({})), &ctx);
    let v: serde_json::Value = serde_json::from_str(&r).unwrap();
    assert_eq!(v, json!({"status": "uninitialized"}));
}

#[test]
fn sleep_returns_string() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("sleep", json!({})), &ctx);
    assert!(!r.is_empty());
}

// ---------- sql_query ----------

#[test]
fn sql_select_returns_tab_separated_rows_with_header() {
    let dir = TempDir::new().unwrap();
    build_fake_parquet(dir.path(), &[]);
    let (ctx, _) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(
        &call(
            "sql_query",
            json!({"query": "SELECT airport_ident, le_ident, length_ft FROM runways WHERE airport_ident='KSEA' ORDER BY length_ft DESC"}),
        ),
        &ctx,
    );
    let lines: Vec<&str> = r.lines().collect();
    assert_eq!(lines[0], "airport_ident\tle_ident\tlength_ft");
    assert_eq!(lines.len(), 4);
    // length_ft is now derived from endpoint haversine rather than a baked
    // constant, so match structurally instead of on the exact string.
    let top: Vec<&str> = lines[1].split('\t').collect();
    assert_eq!(top[0], "KSEA");
    assert_eq!(top[1], "16L");
    let length: f64 = top[2].parse().unwrap();
    assert!((length - 11_900.0).abs() < 50.0, "length {}", length);
}

#[test]
fn sql_empty_result_returns_zero_rows_message() {
    let dir = TempDir::new().unwrap();
    build_fake_parquet(dir.path(), &[]);
    let (ctx, _) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(
        &call(
            "sql_query",
            json!({"query": "SELECT * FROM runways WHERE airport_ident='NOWHERE'"}),
        ),
        &ctx,
    );
    assert_eq!(r, "0 rows");
}

#[test]
fn sql_invalid_returns_error() {
    let dir = TempDir::new().unwrap();
    build_fake_parquet(dir.path(), &[]);
    let (ctx, _) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(
        &call("sql_query", json!({"query": "SELECT * FROM does_not_exist"})),
        &ctx,
    );
    assert!(r.starts_with("error:"));
}

#[test]
fn sql_missing_csv_path_returns_error() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("sql_query", json!({"query": "SELECT 1"})), &ctx);
    assert!(r.starts_with("error:"));
    assert!(r.contains("not configured"));
}

#[test]
fn sql_row_cap_truncates_large_results() {
    let dir = TempDir::new().unwrap();
    let mut extra: Vec<String> = Vec::new();
    for i in 0..60u32 {
        // Give each synthetic airport a distinct lat/lon so the dataset
        // looks plausible, not that it affects this test.
        let lat = 40.0 + (i as f64) * 0.001;
        extra.push(format!(
            "1 500 0 0 K{i:03} K{i:03} Test\n\
             100 30.48 1 0 0.25 0 2 0 \
             09 {lat:.4} -100.0 0 0 0 0 0 0 \
             27 {lat:.4} -99.99 0 0 0 0 0 0\n"
        ));
    }
    build_fake_parquet(dir.path(), &extra);
    let (ctx, _) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(
        &call("sql_query", json!({"query": "SELECT airport_ident FROM runways"})),
        &ctx,
    );
    let lines: Vec<&str> = r.lines().collect();
    assert_eq!(lines.len(), 52); // header + 50 rows + truncation notice
    assert!(lines.last().unwrap().contains("truncated"));
}

// ---------- plan_taxi_route ----------

fn build_taxi_fixture_parquet(dir: &std::path::Path) -> PathBuf {
    // A tiny synthetic KTEST that has both runway data and a taxi network,
    // enough to exercise the tool end-to-end.
    //
    //   gate (start) -- connector --> N1 == A ==> N2 == D ==> N3 (near 09 thr)
    //                                                \== C ==> N4
    // Runway 09 threshold sits right at node 3; destination resolution
    // will therefore land the plan at node 3, which is only reachable via
    // A then D. A parallel C taxiway routes through node 4 (slightly
    // longer; used by the constrained test below).
    let extra = vec![
        "1 0 0 0 KTEST KTEST Test Airport\n\
         100 22.86 1 0 0.25 0 2 0 \
         09 37.000100 -121.997500 0.00 0 0 0 0 0 \
         27 37.000100 -121.987500 0.00 0 0 0 0 0\n\
         1201 37.000200 -122.001000 both 1\n\
         1201 37.000200 -121.999500 both 2\n\
         1201 37.000100 -121.997500 both 3\n\
         1201 37.000300 -121.998500 both 4\n\
         1201 37.000500 -122.002000 both 10\n\
         1202 10 1 twoway taxiway_E \n\
         1202 1 2 twoway taxiway_E A\n\
         1202 2 3 twoway taxiway_E D\n\
         1202 2 4 twoway taxiway_E C\n\
         1202 4 3 twoway taxiway_E C\n"
            .to_string(),
    ];
    build_fake_parquet(dir, &extra)
}

#[test]
fn plan_taxi_route_shortest_returns_formatted_plan() {
    let dir = TempDir::new().unwrap();
    build_taxi_fixture_parquet(dir.path());
    let (ctx, _bridge) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(
        &call(
            "plan_taxi_route",
            json!({
                "airport_ident": "KTEST",
                "destination_runway": "09",
                "via_taxiways": [],
                "start_lat": 37.000500,
                "start_lon": -122.002000,
            }),
        ),
        &ctx,
    );
    assert!(!r.starts_with("error"), "got: {}", r);
    assert!(r.contains("taxi plan KTEST"), "output: {}", r);
    assert!(r.contains("A -> D"), "output: {}", r);
    assert!(r.contains("legs"));
}

#[test]
fn plan_taxi_route_constrained_honors_via_taxiways() {
    let dir = TempDir::new().unwrap();
    build_taxi_fixture_parquet(dir.path());
    let (ctx, _bridge) = make_ctx_with_parquet(dir.path());
    // Force A then C (which routes via N4 instead of the shorter D leg) —
    // destination_runway 09 still resolves, and the plan takes the longer path.
    let r = dispatch_tool(
        &call(
            "plan_taxi_route",
            json!({
                "airport_ident": "KTEST",
                "destination_runway": "09",
                "via_taxiways": ["A", "C"],
                "start_lat": 37.000500,
                "start_lon": -122.002000,
            }),
        ),
        &ctx,
    );
    assert!(!r.starts_with("error"), "got: {}", r);
    assert!(r.contains("A -> C"), "plan did not honor via list: {}", r);
}

#[test]
fn plan_taxi_route_airport_without_network_errors() {
    let dir = TempDir::new().unwrap();
    build_fake_parquet(dir.path(), &[]); // Base fixture has no taxi rows.
    let (ctx, _bridge) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(
        &call(
            "plan_taxi_route",
            json!({
                "airport_ident": "KSEA",
                "destination_runway": "16L",
                "via_taxiways": [],
                "start_lat": 47.46,
                "start_lon": -122.30,
            }),
        ),
        &ctx,
    );
    assert!(r.starts_with("error"));
    assert!(r.contains("no taxi network"));
}

// ---------- engage_taxi ----------

#[test]
fn engage_taxi_swaps_in_taxi_profile_and_reports_summary() {
    let dir = TempDir::new().unwrap();
    build_taxi_fixture_parquet(dir.path());
    // Georef anchor = runway 09 threshold (well inside KTEST) so the
    // geodetic → local-feet conversion lands reasonable values.
    let bridge = FakeBridge::new(
        &[
            ("sim/flightmodel/position/latitude", 37.000500),
            ("sim/flightmodel/position/longitude", -122.002000),
        ],
        37.000100,
        -121.997500,
    );
    let ctx = make_ctx(
        Some(bridge.clone() as Arc<dyn ToolBridge>),
        Some(dir.path().to_path_buf()),
    );
    let r = dispatch_tool(
        &call(
            "engage_taxi",
            json!({
                "airport_ident": "KTEST",
                "destination_runway": "09",
                "via_taxiways": ["A", "D"],
                "start_lat": 37.000500,
                "start_lon": -122.002000,
            }),
        ),
        &ctx,
    );
    assert!(!r.starts_with("error"), "got: {}", r);
    assert!(r.contains("engaged taxi KTEST"), "got: {}", r);
    assert!(r.contains("A -> D"), "got: {}", r);
    let names = ctx.pilot.lock().list_profile_names();
    assert_eq!(names, vec!["taxi"]);
}

#[test]
fn engage_taxi_prepends_pullout_leg_when_far_from_nearest_node() {
    use xplane_pilot::sim::xplane_bridge::{geodetic_offset_ft, GeoReference};
    use xplane_pilot::types::Vec2;

    let dir = TempDir::new().unwrap();
    build_taxi_fixture_parquet(dir.path());
    // Start the aircraft 0.001° (~100 m) south-west of the nearest node,
    // which is well past the 20 ft pullout threshold.
    let start_lat = 37.000500 - 0.0010;
    let start_lon = -122.002000 - 0.0010;
    let bridge = FakeBridge::new(&[], 37.000100, -121.997500);
    let ctx = make_ctx(
        Some(bridge.clone() as Arc<dyn ToolBridge>),
        Some(dir.path().to_path_buf()),
    );
    let r = dispatch_tool(
        &call(
            "engage_taxi",
            json!({
                "airport_ident": "KTEST",
                "destination_runway": "09",
                "via_taxiways": [],
                "start_lat": start_lat,
                "start_lon": start_lon,
            }),
        ),
        &ctx,
    );
    assert!(!r.starts_with("error"), "got: {}", r);
    assert!(
        r.contains("pullout:"),
        "summary should mention pullout: {}",
        r
    );

    // Direct check: the first leg of the engaged profile should begin at
    // the aircraft's position, not at a graph node.
    let georef = GeoReference {
        threshold_lat_deg: 37.000100,
        threshold_lon_deg: -121.997500,
    };
    let expected_start: Vec2 = geodetic_offset_ft(start_lat, start_lon, georef);
    let (legs, _names, pullout) = xplane_pilot::llm::tools::build_taxi_legs_with_pullout(
        expected_start,
        &[], // empty stand-in; we exercise the full path via the dispatch above
        georef,
    );
    // Empty plan → empty legs list, no pullout.
    assert!(legs.is_empty());
    assert!(pullout.is_none());
}

#[test]
fn engage_line_up_builds_two_leg_route_to_runway_threshold() {
    let dir = TempDir::new().unwrap();
    build_taxi_fixture_parquet(dir.path());
    // Georef anchors at runway 09 threshold so the expected end-pose
    // offsets are easy to reason about.
    let bridge = FakeBridge::new(
        &[
            ("sim/flightmodel/position/latitude", 37.000800),
            ("sim/flightmodel/position/longitude", -121.998500),
        ],
        37.000100,
        -121.997500,
    );
    let ctx = make_ctx(
        Some(bridge.clone() as Arc<dyn ToolBridge>),
        Some(dir.path().to_path_buf()),
    );
    let r = dispatch_tool(
        &call(
            "engage_line_up",
            json!({
                "airport_ident": "KTEST",
                "runway_ident": "09",
                "start_lat": 37.000800,
                "start_lon": -121.998500,
            }),
        ),
        &ctx,
    );
    assert!(!r.starts_with("error"), "got: {}", r);
    assert!(r.contains("engaged line_up KTEST"), "got: {}", r);
    assert!(r.contains("runway 09"), "got: {}", r);
    // Verify the taxi profile is engaged (replaces the idle trio).
    assert_eq!(ctx.pilot.lock().list_profile_names(), vec!["taxi"]);
}

#[test]
fn engage_line_up_unknown_runway_errors() {
    let dir = TempDir::new().unwrap();
    build_taxi_fixture_parquet(dir.path());
    let bridge = FakeBridge::new(&[], 37.000100, -121.997500);
    let ctx = make_ctx(
        Some(bridge.clone() as Arc<dyn ToolBridge>),
        Some(dir.path().to_path_buf()),
    );
    let r = dispatch_tool(
        &call(
            "engage_line_up",
            json!({
                "airport_ident": "KTEST",
                "runway_ident": "99",
                "start_lat": 37.000800,
                "start_lon": -121.998500,
            }),
        ),
        &ctx,
    );
    assert!(r.starts_with("error"));
    assert!(r.contains("runway"));
}

#[test]
fn engage_taxi_without_bridge_errors() {
    let dir = TempDir::new().unwrap();
    build_taxi_fixture_parquet(dir.path());
    let ctx = make_ctx(None, Some(dir.path().to_path_buf()));
    let r = dispatch_tool(
        &call(
            "engage_taxi",
            json!({
                "airport_ident": "KTEST",
                "destination_runway": "09",
                "via_taxiways": [],
                "start_lat": 37.000500,
                "start_lon": -122.002000,
            }),
        ),
        &ctx,
    );
    assert!(r.starts_with("error"));
    assert!(r.contains("bridge"));
}

// ---------- spatial extension ----------

#[test]
fn spatial_function_finds_nearest_runways() {
    let dir = TempDir::new().unwrap();
    build_fake_parquet(dir.path(), &[]);
    let (ctx, _) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(
        &call(
            "sql_query",
            json!({"query":
                "SELECT airport_ident, le_ident \
                 FROM runways \
                 WHERE closed = 0 AND le_latitude_deg IS NOT NULL \
                 ORDER BY ST_Distance_Sphere(ST_Point(le_longitude_deg, le_latitude_deg), ST_Point(-122.31, 47.46)) \
                 LIMIT 3"}),
        ),
        &ctx,
    );
    let lines: Vec<&str> = r.lines().collect();
    assert_eq!(lines[0], "airport_ident\tle_ident");
    let airports: Vec<&str> = lines[1..4].iter().map(|l| l.split('\t').next().unwrap()).collect();
    assert_eq!(airports, vec!["KSEA", "KSEA", "KSEA"]);
}

#[test]
fn spatial_least_of_both_ends_picks_closer_high_numbered_threshold() {
    let dir = TempDir::new().unwrap();
    build_fake_parquet(dir.path(), &[]);
    let (ctx, _) = make_ctx_with_parquet(dir.path());
    let r = dispatch_tool(
        &call(
            "sql_query",
            json!({"query":
                "SELECT airport_ident, le_ident, he_ident, \
                   LEAST( \
                     ST_Distance_Sphere(ST_Point(le_longitude_deg, le_latitude_deg), ST_Point(-122.308039, 47.431388)), \
                     ST_Distance_Sphere(ST_Point(he_longitude_deg, he_latitude_deg), ST_Point(-122.308039, 47.431388)) \
                   ) AS dist_m \
                 FROM runways \
                 WHERE closed = 0 AND le_latitude_deg IS NOT NULL AND he_latitude_deg IS NOT NULL \
                 ORDER BY dist_m LIMIT 1"}),
        ),
        &ctx,
    );
    let lines: Vec<&str> = r.lines().collect();
    assert_eq!(lines[0], "airport_ident\tle_ident\the_ident\tdist_m");
    let fields: Vec<&str> = lines[1].split('\t').collect();
    assert_eq!(fields[0], "KSEA");
    assert_eq!(fields[1], "16L");
    assert_eq!(fields[2], "34R");
    let dist_m: f64 = fields[3].parse().unwrap();
    assert!(dist_m < 100.0, "dist_m was {}", dist_m);
}
