//! Port of tests/test_mode_transitions.py.

mod common;
use common::state_with;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mode_manager::{
    ModeManager, ModeManagerUpdate, PatternClearances, PatternTriggers,
};
use xplane_pilot::core::safety_monitor::SafetyStatus;
use xplane_pilot::guidance::pattern_manager::build_pattern_geometry;
use xplane_pilot::guidance::route_manager::RouteManager;
use xplane_pilot::guidance::runway_geometry::RunwayFrame;
use xplane_pilot::types::FlightPhase;

fn fixtures() -> (
    xplane_pilot::config::ConfigBundle,
    ModeManager,
    xplane_pilot::guidance::pattern_manager::PatternGeometry,
    RouteManager,
    SafetyStatus,
) {
    let config = load_default_config_bundle();
    let mode_manager = ModeManager::new(config.clone());
    let rf = RunwayFrame::new(config.airport.runway.clone());
    let pattern = build_pattern_geometry(&rf, config.pattern.downwind_offset_ft, 0.0, 0.0);
    let route = RouteManager::new(vec![]);
    let safe = SafetyStatus {
        request_go_around: false,
        reason: None,
        bank_limit_deg: config.limits.max_bank_pattern_deg,
    };
    (config, mode_manager, pattern, route, safe)
}

fn update(
    mm: &ModeManager,
    phase: FlightPhase,
    state: &xplane_pilot::types::AircraftState,
    rm: &RouteManager,
    p: &xplane_pilot::guidance::pattern_manager::PatternGeometry,
    s: &SafetyStatus,
    turn_base_now: bool,
    force_go_around: bool,
    stay_in_pattern: bool,
    touch_and_go: bool,
) -> FlightPhase {
    let triggers = PatternTriggers {
        turn_base: turn_base_now,
        ..Default::default()
    };
    let clearances = PatternClearances::default();
    mm.update(&ModeManagerUpdate {
        phase,
        state,
        route_manager: rm,
        pattern: p,
        safety_status: s,
        triggers: &triggers,
        clearances: &clearances,
        force_go_around,
        stay_in_pattern,
        touch_and_go,
    })
}

#[test]
fn downwind_does_not_skip_directly_to_flare() {
    let (_cfg, mm, pattern, rm, safe) = fixtures();
    let state = state_with(|s| {
        s.alt_agl_ft = 8.0;
        s.runway_x_ft = Some(pattern.base_turn_x_ft + 500.0);
        s.runway_y_ft = Some(pattern.downwind_y_ft);
    });
    assert_eq!(
        update(&mm, FlightPhase::Downwind, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Downwind
    );
}

#[test]
fn pattern_sequence_advances_one_phase_at_a_time() {
    let (cfg, mm, pattern, rm, safe) = fixtures();
    let downwind_state = state_with(|s| {
        s.runway_x_ft = Some(pattern.base_turn_x_ft - 10.0);
        s.runway_y_ft = Some(pattern.downwind_y_ft);
        s.track_deg = 180.0;
    });
    assert_eq!(
        update(&mm, FlightPhase::Downwind, &downwind_state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Base
    );
    let base_state = state_with(|s| {
        s.runway_x_ft = Some(-1500.0);
        s.runway_y_ft = Some(40.0);
        s.track_deg = 0.0;
    });
    assert_eq!(
        update(&mm, FlightPhase::Base, &base_state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Final
    );
    let final_state = state_with(|s| {
        s.alt_agl_ft = cfg.flare.roundout_height_ft - 1.0;
        s.runway_x_ft = Some(-300.0);
    });
    assert_eq!(
        update(&mm, FlightPhase::Final, &final_state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Roundout
    );
    let roundout_state = state_with(|s| {
        s.alt_agl_ft = cfg.flare.flare_start_ft - 1.0;
        s.runway_x_ft = Some(50.0);
    });
    assert_eq!(
        update(&mm, FlightPhase::Roundout, &roundout_state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Flare
    );
}

#[test]
fn slow_downwind_can_turn_base_soon_after_abeam() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let state = state_with(|s| {
        s.runway_x_ft = Some(500.0);
        s.runway_y_ft = Some(pattern.downwind_y_ft);
        s.threshold_abeam = true;
        s.gs_kt = 35.0;
        s.ias_kt = 65.0;
    });
    assert_eq!(
        update(&mm, FlightPhase::Downwind, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Base
    );
}

#[test]
fn normal_speed_downwind_does_not_turn_base_at_abeam() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let state = state_with(|s| {
        s.runway_x_ft = Some(500.0);
        s.runway_y_ft = Some(pattern.downwind_y_ft);
        s.threshold_abeam = true;
        s.gs_kt = 80.0;
        s.ias_kt = 80.0;
    });
    assert_eq!(
        update(&mm, FlightPhase::Downwind, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Downwind
    );
}

#[test]
fn takeoff_roll_rotates_at_vr_when_on_centerline() {
    let (cfg, mm, pattern, rm, safe) = fixtures();
    let vr = cfg.performance.vr_kt;
    let state = state_with(|s| {
        s.on_ground = true;
        s.alt_agl_ft = 0.0;
        s.runway_x_ft = Some(2000.0);
        s.runway_y_ft = Some(0.0);
        s.centerline_error_ft = Some(5.0);
        s.ias_kt = vr + 1.0;
        s.gs_kt = vr + 1.0;
    });
    assert_eq!(
        update(&mm, FlightPhase::TakeoffRoll, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Rotate
    );
}

#[test]
fn takeoff_roll_does_not_rotate_before_vr() {
    let (cfg, mm, pattern, rm, safe) = fixtures();
    let vr = cfg.performance.vr_kt;
    let state = state_with(|s| {
        s.on_ground = true;
        s.ias_kt = vr - 5.0;
        s.gs_kt = vr - 5.0;
        s.centerline_error_ft = Some(0.0);
    });
    assert_eq!(
        update(&mm, FlightPhase::TakeoffRoll, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::TakeoffRoll
    );
}

#[test]
fn takeoff_roll_rotates_with_moderate_crosswind_centerline_error() {
    let (cfg, mm, pattern, rm, safe) = fixtures();
    let vr = cfg.performance.vr_kt;
    let state = state_with(|s| {
        s.on_ground = true;
        s.ias_kt = vr + 1.0;
        s.gs_kt = vr + 1.0;
        s.centerline_error_ft = Some(40.0);
    });
    assert_eq!(
        update(&mm, FlightPhase::TakeoffRoll, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Rotate
    );
}

#[test]
fn takeoff_roll_does_not_rotate_when_way_off_centerline_on_ground() {
    let (cfg, mm, pattern, rm, safe) = fixtures();
    let vr = cfg.performance.vr_kt;
    let state = state_with(|s| {
        s.on_ground = true;
        s.ias_kt = vr + 1.0;
        s.gs_kt = vr + 1.0;
        s.centerline_error_ft = Some(200.0);
    });
    assert_eq!(
        update(&mm, FlightPhase::TakeoffRoll, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::TakeoffRoll
    );
}

#[test]
fn airborne_bailout_rotates_even_when_way_off_centerline() {
    let (cfg, mm, pattern, rm, safe) = fixtures();
    let vr = cfg.performance.vr_kt;
    let state = state_with(|s| {
        s.on_ground = false;
        s.alt_agl_ft = 5.0;
        s.centerline_error_ft = Some(500.0);
        s.ias_kt = vr + 20.0;
        s.gs_kt = vr + 20.0;
    });
    assert_eq!(
        update(&mm, FlightPhase::TakeoffRoll, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Rotate
    );
}

#[test]
fn preflight_on_ground_goes_to_takeoff_roll() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let state = state_with(|s| {
        s.on_ground = true;
        s.ias_kt = 0.0;
        s.gs_kt = 0.0;
        s.alt_agl_ft = 0.0;
    });
    assert_eq!(
        update(&mm, FlightPhase::Preflight, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::TakeoffRoll
    );
}

#[test]
fn preflight_airborne_above_vr_skips_straight_to_initial_climb() {
    let (cfg, mm, pattern, rm, safe) = fixtures();
    let vr = cfg.performance.vr_kt;
    let state = state_with(|s| {
        s.on_ground = false;
        s.ias_kt = vr + 40.0;
        s.gs_kt = vr + 40.0;
        s.alt_agl_ft = 300.0;
    });
    assert_eq!(
        update(&mm, FlightPhase::Preflight, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::InitialClimb
    );
}

#[test]
fn initial_climb_below_crosswind_altitude_stays_in_initial_climb_with_stay_in_pattern() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let state = state_with(|s| {
        s.alt_agl_ft = 500.0;
        s.vs_fpm = 800.0;
        s.on_ground = false;
    });
    assert_eq!(
        update(&mm, FlightPhase::InitialClimb, &state, &rm, &pattern, &safe, false, false, true, false),
        FlightPhase::InitialClimb
    );
}

#[test]
fn initial_climb_above_crosswind_altitude_turns_crosswind() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let state = state_with(|s| {
        s.alt_agl_ft = 750.0;
        s.vs_fpm = 800.0;
        s.on_ground = false;
    });
    assert_eq!(
        update(&mm, FlightPhase::InitialClimb, &state, &rm, &pattern, &safe, false, false, true, false),
        FlightPhase::Crosswind
    );
}

#[test]
fn initial_climb_without_stay_in_pattern_uses_enroute_flow() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let state = state_with(|s| {
        s.alt_agl_ft = 450.0;
        s.vs_fpm = 800.0;
        s.on_ground = false;
    });
    assert_eq!(
        update(&mm, FlightPhase::InitialClimb, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::EnrouteClimb
    );
}

#[test]
fn crosswind_holds_until_heading_captured_and_at_offset() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let state = state_with(|s| {
        s.alt_agl_ft = 900.0;
        s.runway_x_ft = Some(5000.0);
        s.runway_y_ft = Some(-500.0);
        s.heading_deg = 270.0;
        s.track_deg = 270.0;
        s.on_ground = false;
    });
    assert_eq!(
        update(&mm, FlightPhase::Crosswind, &state, &rm, &pattern, &safe, false, false, true, false),
        FlightPhase::Crosswind
    );
}

#[test]
fn crosswind_transitions_to_downwind_when_captured_and_at_offset() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let state = state_with(|s| {
        s.alt_agl_ft = 950.0;
        s.runway_x_ft = Some(5000.0);
        s.runway_y_ft = Some(-3500.0);
        s.heading_deg = 270.0;
        s.track_deg = 270.0;
        s.on_ground = false;
    });
    assert_eq!(
        update(&mm, FlightPhase::Crosswind, &state, &rm, &pattern, &safe, false, false, true, false),
        FlightPhase::Downwind
    );
}

#[test]
fn touch_and_go_on_ground_goes_to_takeoff_roll() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let state = state_with(|s| {
        s.alt_agl_ft = 5.0;
        s.on_ground = true;
        s.ias_kt = 55.0;
    });
    assert_eq!(
        update(&mm, FlightPhase::Flare, &state, &rm, &pattern, &safe, false, false, false, true),
        FlightPhase::TakeoffRoll
    );
}

#[test]
fn ground_contact_on_final_goes_to_rollout_even_with_stale_agl() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let state = state_with(|s| {
        s.alt_agl_ft = 35.0;
        s.on_ground = true;
        s.ias_kt = 50.0;
    });
    assert_eq!(
        update(&mm, FlightPhase::Final, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Rollout
    );
}

#[test]
fn go_around_persists_even_above_400_agl() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let state = state_with(|s| {
        s.alt_agl_ft = 600.0;
        s.vs_fpm = 500.0;
        s.on_ground = false;
    });
    assert_eq!(
        update(&mm, FlightPhase::GoAround, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::GoAround
    );
}

#[test]
fn base_turn_nominal_point_matches_downwind_offset() {
    let (cfg, _, pattern, _, _) = fixtures();
    assert_eq!(pattern.base_turn_x_ft, -cfg.pattern.downwind_offset_ft);
}

#[test]
fn base_does_not_transition_at_start_of_leg() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let base_start = pattern.base_leg.start_ft;
    let state = state_with(|s| {
        s.position_ft = base_start;
        s.runway_x_ft = Some(pattern.base_leg.start_ft.x);
        s.runway_y_ft = Some(pattern.base_leg.start_ft.y);
        s.track_deg = 180.0;
        s.alt_agl_ft = 900.0;
    });
    assert_eq!(
        update(&mm, FlightPhase::Base, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Base
    );
}

#[test]
fn base_transitions_near_end_of_leg() {
    let (_, mm, pattern, rm, safe) = fixtures();
    let leg = pattern.base_leg;
    let delta = leg.end_ft - leg.start_ft;
    let near_end = leg.start_ft + delta * 0.9;
    let state = state_with(|s| {
        s.position_ft = near_end;
        s.runway_x_ft = Some(near_end.x);
        s.runway_y_ft = Some(near_end.y);
        s.track_deg = 90.0;
        s.alt_agl_ft = 600.0;
    });
    assert_eq!(
        update(&mm, FlightPhase::Base, &state, &rm, &pattern, &safe, false, false, false, false),
        FlightPhase::Final
    );
}
