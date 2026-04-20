//! Port of tests/test_live_intents.py.

mod common;
use common::state_with;

use approx::assert_abs_diff_eq;
use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::core::mode_manager::{
    ModeManager, ModeManagerUpdate, PatternClearances, PatternTriggers,
};
use xplane_pilot::core::profiles::{ExtendMode, PatternFlyProfile, PatternLeg};
use xplane_pilot::core::safety_monitor::SafetyStatus;
use xplane_pilot::guidance::pattern_manager::build_pattern_geometry;
use xplane_pilot::guidance::route_manager::RouteManager;
use xplane_pilot::guidance::runway_geometry::RunwayFrame;
use xplane_pilot::live_runner::bootstrap_config_from_sample;
use xplane_pilot::sim::xplane_bridge::{BootstrapSample, PositionSample};
use xplane_pilot::types::FlightPhase;

#[test]
fn extend_downwind_rebuilds_pattern_geometry() {
    let cfg = load_default_config_bundle();
    let mut pilot = PilotCore::new(cfg.clone());
    let rf = pilot.runway_frame.clone();
    pilot.engage_profile(Box::new(PatternFlyProfile::new(cfg, rf)));
    let original_ext = pilot.find_profile_mut("pattern_fly", |p| {
        let ptr = p as *mut dyn xplane_pilot::core::profiles::GuidanceProfile as *mut u8;
        let concrete = ptr as *mut PatternFlyProfile;
        unsafe { (*concrete).pattern.extension_ft }
    }).unwrap();
    let original_base = pilot.find_profile_mut("pattern_fly", |p| {
        let ptr = p as *mut dyn xplane_pilot::core::profiles::GuidanceProfile as *mut u8;
        let concrete = ptr as *mut PatternFlyProfile;
        unsafe { (*concrete).pattern.base_turn_x_ft }
    }).unwrap();
    pilot.find_profile_mut("pattern_fly", |p| {
        let ptr = p as *mut dyn xplane_pilot::core::profiles::GuidanceProfile as *mut u8;
        let concrete = ptr as *mut PatternFlyProfile;
        unsafe {
            (*concrete)
                .extend_leg(PatternLeg::Downwind, 2400.0, ExtendMode::Add)
                .unwrap()
        }
    });
    let (new_ext, new_base) = pilot.find_profile_mut("pattern_fly", |p| {
        let ptr = p as *mut dyn xplane_pilot::core::profiles::GuidanceProfile as *mut u8;
        let concrete = ptr as *mut PatternFlyProfile;
        unsafe { ((*concrete).pattern.extension_ft, (*concrete).pattern.base_turn_x_ft) }
    }).unwrap();
    assert_abs_diff_eq!(new_ext, original_ext + 2400.0, epsilon = 1e-6);
    assert!(new_base < original_base);
}

#[test]
fn turn_base_now_override_skips_waiting_for_nominal_turn_point() {
    let cfg = load_default_config_bundle();
    let mm = ModeManager::new(cfg.clone());
    let rf = RunwayFrame::new(cfg.airport.runway.clone());
    let pattern = build_pattern_geometry(&rf, cfg.pattern.downwind_offset_ft, 0.0, 0.0);
    let state = state_with(|s| {
        s.runway_x_ft = Some(1200.0);
        s.runway_y_ft = Some(pattern.downwind_y_ft);
        s.gs_kt = 80.0;
        s.ias_kt = 80.0;
    });
    let route = RouteManager::new(vec![]);
    let safe = SafetyStatus {
        request_go_around: false,
        reason: None,
        bank_limit_deg: cfg.limits.max_bank_pattern_deg,
    };
    let triggers = PatternTriggers { turn_base: true, ..Default::default() };
    let clearances = PatternClearances::default();
    assert_eq!(
        mm.update(&ModeManagerUpdate {
            phase: FlightPhase::Downwind,
            state: &state,
            route_manager: &route,
            pattern: &pattern,
            safety_status: &safe,
            triggers: &triggers,
            clearances: &clearances,
            force_go_around: false,
            stay_in_pattern: false,
            touch_and_go: false,
        }),
        FlightPhase::Base
    );
}

#[test]
fn force_go_around_override_preempts_normal_sequence() {
    let cfg = load_default_config_bundle();
    let mm = ModeManager::new(cfg.clone());
    let rf = RunwayFrame::new(cfg.airport.runway.clone());
    let pattern = build_pattern_geometry(&rf, cfg.pattern.downwind_offset_ft, 0.0, 0.0);
    let state = state_with(|s| {
        s.alt_agl_ft = 300.0;
        s.runway_x_ft = Some(-2000.0);
        s.runway_y_ft = Some(10.0);
    });
    let route = RouteManager::new(vec![]);
    let safe = SafetyStatus {
        request_go_around: false,
        reason: None,
        bank_limit_deg: cfg.limits.max_bank_final_deg,
    };
    let triggers = PatternTriggers::default();
    let clearances = PatternClearances::default();
    assert_eq!(
        mm.update(&ModeManagerUpdate {
            phase: FlightPhase::Final,
            state: &state,
            route_manager: &route,
            pattern: &pattern,
            safety_status: &safe,
            triggers: &triggers,
            clearances: &clearances,
            force_go_around: true,
            stay_in_pattern: false,
            touch_and_go: false,
        }),
        FlightPhase::GoAround
    );
}

#[test]
fn bootstrap_on_runway() {
    let cfg = load_default_config_bundle();
    let b = bootstrap_config_from_sample(
        cfg,
        BootstrapSample {
            posi: PositionSample {
                lat_deg: 47.449,
                lon_deg: -122.309,
                altitude_msl_m: 132.0,
                roll_deg: 0.0,
                pitch_deg: 0.0,
                heading_deg: 163.0,
            },
            alt_agl_ft: 0.5,
            on_ground: true,
        },
    );
    assert_eq!(b.airport.airport, None);
    assert_eq!(b.airport.runway.id, None);
    assert_eq!(b.airport.runway.course_deg, 163.0);
    assert_abs_diff_eq!(b.airport.field_elevation_ft, 132.0 * 3.280839895013123 - 0.5, epsilon = 1e-3);
}

#[test]
fn bootstrap_airborne_in_cruise() {
    let cfg = load_default_config_bundle();
    let b = bootstrap_config_from_sample(
        cfg,
        BootstrapSample {
            posi: PositionSample {
                lat_deg: 47.0,
                lon_deg: -122.0,
                altitude_msl_m: 1524.0,
                roll_deg: 0.0,
                pitch_deg: 0.0,
                heading_deg: 270.0,
            },
            alt_agl_ft: 3500.0,
            on_ground: false,
        },
    );
    assert_eq!(b.airport.runway.course_deg, 270.0);
    assert_abs_diff_eq!(b.airport.field_elevation_ft, 1500.0, epsilon = 1.0);
}

#[test]
fn bootstrap_airborne_over_ocean() {
    let cfg = load_default_config_bundle();
    let b = bootstrap_config_from_sample(
        cfg,
        BootstrapSample {
            posi: PositionSample {
                lat_deg: 30.0,
                lon_deg: -140.0,
                altitude_msl_m: 1524.0,
                roll_deg: 0.0,
                pitch_deg: 0.0,
                heading_deg: 90.0,
            },
            alt_agl_ft: 5000.0,
            on_ground: false,
        },
    );
    assert_eq!(b.airport.runway.course_deg, 90.0);
    assert_abs_diff_eq!(b.airport.field_elevation_ft, 0.0, epsilon = 1.0);
    assert_eq!(b.airport.airport, None);
    assert_eq!(b.airport.runway.id, None);
}
