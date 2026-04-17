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
use xplane_pilot::core::profiles::{HeadingHoldProfile, PatternFlyProfile};
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

fn make_ctx(bridge: Option<Arc<dyn ToolBridge>>, runway_csv: Option<PathBuf>) -> ToolContext {
    let cfg = load_default_config_bundle();
    let pilot = Arc::new(PLMutex::new(PilotCore::new(cfg.clone())));
    let mut ctx = ToolContext::new(pilot, cfg);
    ctx.bridge = bridge;
    ctx.runway_csv_path = runway_csv;
    ctx
}

fn call(name: &str, args: serde_json::Value) -> serde_json::Value {
    json!({
        "name": name,
        "call_id": "call_test",
        "arguments": args.to_string(),
    })
}

const RUNWAY_CSV_HEADER: &str = "id,airport_ref,airport_ident,length_ft,width_ft,surface,lighted,closed,le_ident,le_latitude_deg,le_longitude_deg,le_elevation_ft,le_heading_degT,le_displaced_threshold_ft,he_ident,he_latitude_deg,he_longitude_deg,he_elevation_ft,he_heading_degT,he_displaced_threshold_ft";

fn build_fake_runway_csv(path: &std::path::Path, extra: &[&str]) {
    let mut rows: Vec<String> = vec![
        "1,1,KSEA,11900,150,ASP,1,0,16L,47.4638,-122.308,432,180.0,,34R,47.4312,-122.308,347,360.0,".into(),
        "2,1,KSEA,9426,150,CONC-F,1,0,16C,47.4638,-122.311,430,180.0,,34C,47.438,-122.311,363,360.0,".into(),
        "3,1,KSEA,9426,150,CON,1,0,16R,47.4638,-122.318,430,180.0,,34L,47.4405,-122.318,363,360.0,".into(),
        "4,2,KBFI,10007,200,ASP,1,0,14R,47.5301,-122.3015,21,142.0,,32L,47.5111,-122.2876,21,322.0,".into(),
        "5,3,KJFK,12079,150,ASP,1,0,04L,40.6235,-73.7944,12,40.0,,22R,40.6432,-73.7821,12,220.0,".into(),
    ];
    for e in extra {
        rows.push(e.to_string());
    }
    let body = format!("{}\n{}\n", RUNWAY_CSV_HEADER, rows.join("\n"));
    std::fs::write(path, body).unwrap();
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
    dispatch_tool(&call("engage_takeoff", json!({})), &ctx);
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
    let r = dispatch_tool(&call("engage_takeoff", json!({})), &ctx);
    assert!(r.starts_with("error:"));
    assert!(r.contains("parking brake"));
    assert!(r.contains("set_parking_brake"));
}

#[test]
fn engage_takeoff_succeeds_after_parking_brake_released() {
    let bridge = FakeBridge::new(
        &[("sim/cockpit2/controls/parking_brake_ratio", 1.0)],
        47.4638,
        -122.308,
    );
    let ctx = make_ctx(Some(bridge.clone() as Arc<dyn ToolBridge>), None);
    let refused = dispatch_tool(&call("engage_takeoff", json!({})), &ctx);
    assert!(refused.starts_with("error:"));
    dispatch_tool(
        &call("set_parking_brake", json!({"engaged": false})),
        &ctx,
    );
    let ok = dispatch_tool(&call("engage_takeoff", json!({})), &ctx);
    assert!(ok.contains("takeoff"));
}

#[test]
fn engage_takeoff_allows_when_no_bridge() {
    let ctx = make_ctx(None, None);
    let r = dispatch_tool(&call("engage_takeoff", json!({})), &ctx);
    assert!(r.contains("takeoff"));
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

fn make_ctx_with_csv(csv_path: &std::path::Path) -> (ToolContext, Arc<FakeBridge>) {
    let bridge = FakeBridge::new(&[], 47.4638, -122.308);
    let ctx = make_ctx(Some(bridge.clone() as Arc<dyn ToolBridge>), Some(csv_path.to_path_buf()));
    (ctx, bridge)
}

#[test]
fn engage_pattern_fly_for_takeoff_roll() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("runways.csv");
    build_fake_runway_csv(&path, &[]);
    let (ctx, _bridge) = make_ctx_with_csv(&path);
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
fn engage_pattern_fly_offsets_threshold_by_displaced_threshold() {
    // Runway 1 pavement end coincides with the georef anchor (47.4638, -122.308),
    // so `pavement_end_ft` becomes (0, 0) and the resulting runway-frame
    // threshold should equal the displacement vector along the runway heading.
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("runways.csv");
    let disp_ft = 289.0_f64;
    let heading_deg = 25.0_f64;
    let extra = [
        "99,99,KTEST,4000,75,ASP,1,0,\
         1,47.4638,-122.308,0,25.0,289,\
         19,47.4749,-122.3009,0,205.0,0",
    ];
    build_fake_runway_csv(&path, &extra);
    let (ctx, _bridge) = make_ctx_with_csv(&path);
    let r = dispatch_tool(
        &call(
            "engage_pattern_fly",
            json!({
                "airport_ident": "KTEST",
                "runway_ident": "1",
                "side": "left",
                "start_phase": "pattern_entry",
            }),
        ),
        &ctx,
    );
    assert!(!r.contains("error"), "got {}", r);
    let pilot = ctx.pilot.lock();
    let threshold = pilot.runway_frame.runway.threshold_ft;
    let expected_east = disp_ft * heading_deg.to_radians().sin();
    let expected_north = disp_ft * heading_deg.to_radians().cos();
    assert!(
        (threshold.x - expected_east).abs() < 1.0,
        "east: got {}, want {}",
        threshold.x,
        expected_east
    );
    assert!(
        (threshold.y - expected_north).abs() < 1.0,
        "north: got {}, want {}",
        threshold.y,
        expected_north
    );
    // Aim point is 1000 ft past threshold on any reasonably long runway.
    assert_eq!(pilot.runway_frame.touchdown_runway_x_ft(), 1000.0);
}

#[test]
fn engage_pattern_fly_unknown_airport_errors() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("runways.csv");
    build_fake_runway_csv(&path, &[]);
    let (ctx, _bridge) = make_ctx_with_csv(&path);
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
    let path = dir.path().join("runways.csv");
    build_fake_runway_csv(&path, &[]);
    let (ctx, _bridge) = make_ctx_with_csv(&path);
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
    let path = dir.path().join("runways.csv");
    build_fake_runway_csv(&path, &[]);
    let (ctx, _bridge) = make_ctx_with_csv(&path);
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
        },
        last_guidance: None,
        go_around_reason: None,
        airport_ident: None,
        runway_id: None,
        field_elevation_ft: None,
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
    let path = dir.path().join("runways.csv");
    build_fake_runway_csv(&path, &[]);
    let (ctx, _) = make_ctx_with_csv(&path);
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
    assert_eq!(
        lines[1].split('\t').collect::<Vec<_>>(),
        vec!["KSEA", "16L", "11900"]
    );
}

#[test]
fn sql_empty_result_returns_zero_rows_message() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("runways.csv");
    build_fake_runway_csv(&path, &[]);
    let (ctx, _) = make_ctx_with_csv(&path);
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
    let path = dir.path().join("runways.csv");
    build_fake_runway_csv(&path, &[]);
    let (ctx, _) = make_ctx_with_csv(&path);
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
    let path = dir.path().join("runways.csv");
    let mut extra = Vec::new();
    for i in 0..60 {
        extra.push(format!(
            "{},{},K{:03},5000,100,ASP,1,0,09,40.0,-100.0,500,90.0,,27,40.0,-100.0,500,270.0,",
            100 + i,
            i,
            i
        ));
    }
    let extra_refs: Vec<&str> = extra.iter().map(|s| s.as_str()).collect();
    build_fake_runway_csv(&path, &extra_refs);
    let (ctx, _) = make_ctx_with_csv(&path);
    let r = dispatch_tool(
        &call("sql_query", json!({"query": "SELECT airport_ident FROM runways"})),
        &ctx,
    );
    let lines: Vec<&str> = r.lines().collect();
    assert_eq!(lines.len(), 52); // header + 50 rows + truncation notice
    assert!(lines.last().unwrap().contains("truncated"));
}

// ---------- spatial extension ----------

#[test]
fn spatial_function_finds_nearest_runways() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("runways.csv");
    build_fake_runway_csv(&path, &[]);
    let (ctx, _) = make_ctx_with_csv(&path);
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
    let path = dir.path().join("runways.csv");
    build_fake_runway_csv(&path, &[]);
    let (ctx, _) = make_ctx_with_csv(&path);
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
