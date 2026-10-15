mod common;
use common::state_with;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::safety_monitor::{CteBand, SafetyMonitor};
use xplane_pilot::types::FlightPhase;

fn monitor() -> SafetyMonitor {
    SafetyMonitor::new(load_default_config_bundle())
}

#[test]
fn unstable_lateral_alignment_requests_go_around() {
    let mon = monitor();
    let state = state_with(|s| {
        s.alt_agl_ft = 150.0;
        s.centerline_error_ft = Some(400.0);
        s.stall_margin = 1.35;
        s.vs_fpm = -400.0;
        s.alt_msl_ft = 650.0;
        s.ias_kt = 65.0;
        s.gs_kt = 65.0;
    });
    let status = mon.evaluate(&state, FlightPhase::Final);
    assert!(status.request_go_around);
    let reason = status.reason.unwrap();
    assert!(reason.contains("unstable_lateral"), "reason was {}", reason);
    assert!(reason.contains("cle=400ft"));
    assert!(reason.contains("agl=150ft"));
    assert!(reason.contains("limit=300ft"));
}

#[test]
fn low_stall_margin_requests_go_around() {
    let mon = monitor();
    let state = state_with(|s| {
        s.alt_agl_ft = 40.0;
        s.stall_margin = 1.05;
        s.centerline_error_ft = Some(0.0);
    });
    let status = mon.evaluate(&state, FlightPhase::Final);
    assert!(status.request_go_around);
    let reason = status.reason.unwrap();
    assert!(reason.contains("low_energy"));
    assert!(reason.contains("stall_margin=1.05"));
}

#[test]
fn centerline_limit_scales_with_altitude() {
    let mon = monitor();
    let high = state_with(|s| {
        s.alt_agl_ft = 199.0;
        s.centerline_error_ft = Some(300.0);
        s.stall_margin = 2.0;
    });
    assert!(!mon.evaluate(&high, FlightPhase::Final).request_go_around);

    let low = state_with(|s| {
        s.alt_agl_ft = 40.0;
        s.centerline_error_ft = Some(100.0);
        s.stall_margin = 2.0;
    });
    let r = mon.evaluate(&low, FlightPhase::Final);
    assert!(r.request_go_around);
    assert!(r.reason.unwrap().contains("unstable_lateral"));
}

#[test]
fn centerline_limit_has_30_foot_floor_near_ground() {
    let mon = monitor();
    let state = state_with(|s| {
        s.alt_agl_ft = 5.0;
        s.centerline_error_ft = Some(25.0);
        s.stall_margin = 2.0;
    });
    assert!(!mon.evaluate(&state, FlightPhase::Final).request_go_around);
}

#[test]
fn on_ground_short_circuits_all_checks() {
    let mon = monitor();
    let state = state_with(|s| {
        s.alt_agl_ft = 35.0;
        s.stall_margin = 1.05;
        s.centerline_error_ft = Some(500.0);
        s.vs_fpm = -1500.0;
        s.on_ground = true;
    });
    let status = mon.evaluate(&state, FlightPhase::Final);
    assert!(!status.request_go_around);
    assert!(status.reason.is_none());
}

#[test]
fn landing_checks_use_radio_agl_when_available() {
    let mon = monitor();
    let state = state_with(|s| {
        s.alt_agl_ft = 500.0;
        s.alt_radio_agl_ft = Some(40.0);
        s.centerline_error_ft = Some(100.0);
        s.stall_margin = 2.0;
    });
    let status = mon.evaluate(&state, FlightPhase::Final);
    assert!(status.request_go_around);
    assert!(status.reason.unwrap().contains("agl=40ft"));
    assert_eq!(mon.risk_summary(&state, FlightPhase::Final).cte_band, CteBand::Trigger);
}

#[test]
fn high_radio_agl_suppresses_premature_field_agl_check() {
    let mon = monitor();
    let state = state_with(|s| {
        s.alt_agl_ft = 40.0;
        s.alt_radio_agl_ft = Some(500.0);
        s.centerline_error_ft = Some(400.0);
        s.stall_margin = 2.0;
    });
    assert!(!mon.evaluate(&state, FlightPhase::Final).request_go_around);
    assert_eq!(mon.risk_summary(&state, FlightPhase::Final).cte_band, CteBand::Safe);
}

#[test]
fn still_airborne_with_too_little_runway_left_goes_around() {
    let mon = monitor();
    // Default fixture: 21000 ft runway → go around with < 1500 ft left.
    assert_eq!(mon.long_landing_limit_x_ft(), 19_500.0);
    let floating = |x: f64| {
        state_with(|s| {
            s.alt_agl_ft = 8.0;
            s.runway_x_ft = Some(x);
            s.runway_y_ft = Some(0.0);
            s.centerline_error_ft = Some(0.0);
            s.stall_margin = 1.4;
            s.vs_fpm = -100.0;
        })
    };
    assert!(!mon.evaluate(&floating(5000.0), FlightPhase::Flare).request_go_around);
    let status = mon.evaluate(&floating(19_600.0), FlightPhase::Flare);
    assert!(status.request_go_around);
    assert!(status.reason.unwrap().contains("long_landing"));
}

#[test]
fn long_landing_limit_stays_past_the_aim_point_on_short_runways() {
    let mut cfg = load_default_config_bundle();
    cfg.airport.runway.length_ft = 1500.0;
    cfg.airport.runway.touchdown_zone_ft = 1200.0;
    let mon = SafetyMonitor::new(cfg);
    // Aim point is clamped to length/3 = 500; the limit can't sit on it.
    assert_eq!(mon.long_landing_limit_x_ft(), 1000.0);
}

#[test]
fn kwhp_landing_1900_ft_down_the_runway_is_not_a_go_around() {
    // The live case: 4124 ft runway, still 9 ft up at x=1873 with ~2250 ft
    // of runway ahead. Plenty to stop a C172.
    let mut cfg = load_default_config_bundle();
    cfg.airport.runway.length_ft = 4124.0;
    cfg.airport.runway.touchdown_zone_ft = 1200.0;
    let mon = SafetyMonitor::new(cfg);
    assert_eq!(mon.long_landing_limit_x_ft(), 2624.0);
    let state = state_with(|s| {
        s.alt_agl_ft = 9.0;
        s.runway_x_ft = Some(1873.0);
        s.runway_y_ft = Some(0.0);
        s.centerline_error_ft = Some(0.0);
        s.stall_margin = 1.4;
        s.vs_fpm = -150.0;
    });
    assert!(!mon.evaluate(&state, FlightPhase::Flare).request_go_around);
}
