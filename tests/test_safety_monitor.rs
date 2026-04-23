mod common;
use common::state_with;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::safety_monitor::SafetyMonitor;
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
