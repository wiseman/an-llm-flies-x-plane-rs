//! Port of tests/test_status_display.py — the headless status panel formatter.

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::{PilotCore, StatusSnapshot};
use xplane_pilot::core::profiles::{AltitudeHoldProfile, HeadingHoldProfile, SpeedHoldProfile};
use xplane_pilot::sim::simple_dynamics::SimpleAircraftModel;
use xplane_pilot::tui::format_snapshot_display;
use xplane_pilot::types::{
    ActuatorCommands, AircraftState, GuidanceTargets, LateralMode, Vec2, VerticalMode,
};

fn make_state(heading_deg: f64, ias_kt: f64, on_ground: bool) -> AircraftState {
    AircraftState {
        t_sim: 42.0,
        dt: 0.2,
        position_ft: Vec2::ZERO,
        alt_msl_ft: 500.0,
        alt_agl_ft: 0.0,
        pitch_deg: 0.0,
        roll_deg: 0.0,
        heading_deg,
        track_deg: heading_deg,
        p_rad_s: 0.0,
        q_rad_s: 0.0,
        r_rad_s: 0.0,
        ias_kt,
        tas_kt: ias_kt,
        gs_kt: ias_kt,
        vs_fpm: 0.0,
        ground_velocity_ft_s: Vec2::ZERO,
        flap_index: 0,
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
    }
}

fn make_snapshot(
    state: AircraftState,
    profiles: &[&str],
    throttle: f64,
    guidance: Option<GuidanceTargets>,
) -> StatusSnapshot {
    StatusSnapshot {
        t_sim: 42.0,
        active_profiles: profiles.iter().map(|s| s.to_string()).collect(),
        phase: None,
        state,
        last_commands: ActuatorCommands {
            aileron: 0.0,
            elevator: 0.0,
            rudder: 0.0,
            throttle,
            flaps: None,
            gear_down: Some(true),
            brakes: 0.0,
            pivot_brake: 0.0,
        },
        last_guidance: guidance,
        go_around_reason: None,
        airport_ident: None,
        runway_id: None,
        field_elevation_ft: None,
        debug_lines: Vec::new(),
    }
}

#[test]
fn none_snapshot_shows_waiting_placeholder() {
    assert!(format_snapshot_display(None).contains("waiting for first pilot tick"));
}

#[test]
fn idle_has_no_target_heading() {
    let snap = make_snapshot(
        make_state(0.0, 0.0, true),
        &["idle_lateral", "idle_vertical", "idle_speed"],
        0.0,
        Some(GuidanceTargets {
            lateral_mode: LateralMode::BankHold,
            vertical_mode: VerticalMode::PitchHold,
            target_bank_deg: Some(0.0),
            target_pitch_deg: Some(0.0),
            throttle_limit: Some((0.0, 0.0)),
            ..Default::default()
        }),
    );
    let out = format_snapshot_display(Some(&snap));
    assert!(out.contains("idle_lateral, idle_vertical, idle_speed"));
    assert!(out.contains("throttle"));
    assert!(out.contains("0.00"));
    assert!(out.contains('—'));
}

#[test]
fn heading_hold_shows_target_heading() {
    let guidance = GuidanceTargets {
        lateral_mode: LateralMode::TrackHold,
        vertical_mode: VerticalMode::Tecs,
        target_bank_deg: Some(5.0),
        target_heading_deg: Some(270.0),
        target_track_deg: Some(270.0),
        target_altitude_ft: Some(3000.0),
        target_speed_kt: Some(95.0),
        throttle_limit: Some((0.1, 0.9)),
        ..Default::default()
    };
    let snap = make_snapshot(
        make_state(265.0, 95.0, false),
        &["heading_hold", "altitude_hold", "speed_hold"],
        0.55,
        Some(guidance),
    );
    let out = format_snapshot_display(Some(&snap));
    assert!(out.contains("heading_hold, altitude_hold, speed_hold"));
    assert!(out.contains("throttle"));
    assert!(out.contains("0.55"));
    assert!(out.contains("265°"));
    assert!(out.contains("270°"));
    assert!(out.contains("2500 AGL"));
    assert!(out.contains("95 kt"));
    assert!(out.contains("airborne"));
}

#[test]
fn shows_flap_position() {
    let mut state = make_state(0.0, 0.0, true);
    state.flap_index = 20;
    let snap = make_snapshot(state, &["pattern_fly"], 0.45, None);
    let out = format_snapshot_display(Some(&snap));
    assert!(out.contains("flaps 20°"));
}

#[test]
fn runway_line_shows_runway_x_and_y() {
    let mut state = make_state(0.0, 0.0, true);
    state.runway_x_ft = Some(1234.0);
    state.runway_y_ft = Some(-567.0);
    let snap = make_snapshot(state, &["pattern_fly"], 0.5, None);
    let out = format_snapshot_display(Some(&snap));
    assert!(out.contains("rwy x+1234 y-567"));
}

#[test]
fn runway_line_handles_missing_runway_frame_coords() {
    let mut state = make_state(0.0, 0.0, true);
    state.runway_x_ft = None;
    state.runway_y_ft = None;
    let snap = make_snapshot(state, &["idle_lateral"], 0.0, None);
    let out = format_snapshot_display(Some(&snap));
    assert!(out.contains("rwy —"));
}

#[test]
fn shows_ground_vs_airborne() {
    let on_ground = format_snapshot_display(Some(&make_snapshot(
        make_state(0.0, 0.0, true),
        &["idle_lateral"],
        0.0,
        None,
    )));
    assert!(on_ground.contains("on ground"));
    let airborne = format_snapshot_display(Some(&make_snapshot(
        make_state(0.0, 0.0, false),
        &["idle_lateral"],
        0.0,
        None,
    )));
    assert!(airborne.contains("airborne"));
}

#[test]
fn live_pilot_update_wires_guidance_into_snapshot() {
    let cfg = load_default_config_bundle();
    let mut pilot = PilotCore::new(cfg.clone());
    pilot.engage_profile(Box::new(HeadingHoldProfile::new(270.0, 25.0, None).unwrap()));
    pilot.engage_profile(Box::new(AltitudeHoldProfile::new(3000.0)));
    pilot.engage_profile(Box::new(SpeedHoldProfile::new(90.0)));
    let model = SimpleAircraftModel::new(cfg, Vec2::ZERO);
    let raw = model.initial_state();
    pilot.update(&raw, 0.2);
    let snap = pilot.latest_snapshot.as_ref().unwrap();
    let g = snap.last_guidance.as_ref().unwrap();
    assert_eq!(g.target_heading_deg, Some(270.0));
    assert_eq!(g.target_altitude_ft, Some(3000.0));
    assert_eq!(g.target_speed_kt, Some(90.0));
}
