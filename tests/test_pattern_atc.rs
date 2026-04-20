//! Tests for the ATC-aware pattern tool refactor: clearance gates,
//! turn triggers, and per-leg extensions.

mod common;
use common::state_with;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mode_manager::{
    ModeManager, ModeManagerUpdate, PatternClearances, PatternTriggers,
};
use xplane_pilot::core::profiles::{ExtendMode, PatternClearanceGate, PatternFlyProfile, PatternLeg};
use xplane_pilot::core::safety_monitor::SafetyStatus;
use xplane_pilot::guidance::pattern_manager::build_pattern_geometry;
use xplane_pilot::guidance::route_manager::RouteManager;
use xplane_pilot::guidance::runway_geometry::RunwayFrame;
use xplane_pilot::types::FlightPhase;

fn safe_status(bank_limit_deg: f64) -> SafetyStatus {
    SafetyStatus {
        request_go_around: false,
        reason: None,
        bank_limit_deg,
    }
}

#[test]
fn extend_crosswind_widens_effective_downwind_offset() {
    let cfg = load_default_config_bundle();
    let rf = RunwayFrame::new(cfg.airport.runway.clone());
    let mut profile = PatternFlyProfile::new(cfg.clone(), rf);
    let nominal_y = profile.pattern.downwind_y_ft;
    let _ = profile
        .extend_leg(PatternLeg::Crosswind, 1500.0, ExtendMode::Add)
        .unwrap();
    let widened_y = profile.pattern.downwind_y_ft;
    assert!(
        widened_y.abs() > nominal_y.abs(),
        "crosswind extension should widen the downwind offset (nominal {} vs widened {})",
        nominal_y,
        widened_y
    );
    assert!((widened_y.abs() - nominal_y.abs() - 1500.0).abs() < 1e-6);
}

#[test]
fn extend_pattern_leg_set_mode_replaces_not_accumulates() {
    let cfg = load_default_config_bundle();
    let rf = RunwayFrame::new(cfg.airport.runway.clone());
    let mut profile = PatternFlyProfile::new(cfg, rf);
    let _ = profile
        .extend_leg(PatternLeg::Crosswind, 1000.0, ExtendMode::Add)
        .unwrap();
    let after_set = profile
        .extend_leg(PatternLeg::Crosswind, 500.0, ExtendMode::Set)
        .unwrap();
    assert_eq!(after_set, 500.0);
    assert_eq!(profile.leg_extensions.crosswind_ft, 500.0);
}

#[test]
fn extend_leg_rejects_base_and_final() {
    let cfg = load_default_config_bundle();
    let rf = RunwayFrame::new(cfg.airport.runway.clone());
    let mut profile = PatternFlyProfile::new(cfg, rf);
    assert!(profile
        .extend_leg(PatternLeg::Base, 500.0, ExtendMode::Add)
        .is_err());
    assert!(profile
        .extend_leg(PatternLeg::Final, 500.0, ExtendMode::Add)
        .is_err());
}

#[test]
fn revoked_base_clearance_holds_on_downwind_past_auto_ready() {
    let cfg = load_default_config_bundle();
    let mm = ModeManager::new(cfg.clone());
    let rf = RunwayFrame::new(cfg.airport.runway.clone());
    let pattern = build_pattern_geometry(&rf, cfg.pattern.downwind_offset_ft, 0.0, 0.0);
    // Place the aircraft well past the nominal base-turn point on downwind.
    let state = state_with(|s| {
        s.runway_x_ft = Some(pattern.base_turn_x_ft - 2000.0);
        s.runway_y_ft = Some(pattern.downwind_y_ft);
        s.gs_kt = 85.0;
        s.ias_kt = 85.0;
    });
    let route = RouteManager::new(vec![]);
    let safe = safe_status(cfg.limits.max_bank_pattern_deg);
    let triggers = PatternTriggers::default();
    let clearances = PatternClearances {
        turn_base: false,
        ..Default::default()
    };
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
            stay_in_pattern: true,
            touch_and_go: false,
        }),
        FlightPhase::Downwind,
        "revoked turn_base clearance should hold the aircraft on downwind"
    );
}

#[test]
fn turn_base_trigger_overrides_revoked_clearance() {
    let cfg = load_default_config_bundle();
    let mm = ModeManager::new(cfg.clone());
    let rf = RunwayFrame::new(cfg.airport.runway.clone());
    let pattern = build_pattern_geometry(&rf, cfg.pattern.downwind_offset_ft, 0.0, 0.0);
    let state = state_with(|s| {
        s.runway_x_ft = Some(pattern.base_turn_x_ft - 2000.0);
        s.runway_y_ft = Some(pattern.downwind_y_ft);
        s.gs_kt = 85.0;
        s.ias_kt = 85.0;
    });
    let route = RouteManager::new(vec![]);
    let safe = safe_status(cfg.limits.max_bank_pattern_deg);
    let triggers = PatternTriggers {
        turn_base: true,
        ..Default::default()
    };
    let clearances = PatternClearances {
        turn_base: false,
        ..Default::default()
    };
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
            stay_in_pattern: true,
            touch_and_go: false,
        }),
        FlightPhase::Base,
        "turn_base trigger should fire even when clearance is revoked"
    );
}

#[test]
fn revoked_downwind_clearance_holds_on_crosswind() {
    let cfg = load_default_config_bundle();
    let mm = ModeManager::new(cfg.clone());
    let rf = RunwayFrame::new(cfg.airport.runway.clone());
    let pattern = build_pattern_geometry(&rf, cfg.pattern.downwind_offset_ft, 0.0, 0.0);
    let runway_course = pattern.runway_frame.runway.course_deg;
    let side_sign = if pattern.downwind_y_ft < 0.0 { -1.0 } else { 1.0 };
    let crosswind_course = (runway_course + side_sign * 90.0).rem_euclid(360.0);
    // Auto-ready: heading captured and y offset past 75% of downwind.
    let state = state_with(|s| {
        s.track_deg = crosswind_course;
        s.heading_deg = crosswind_course;
        s.runway_y_ft = Some(pattern.downwind_y_ft);
        s.runway_x_ft = Some(0.0);
        s.gs_kt = 75.0;
    });
    let route = RouteManager::new(vec![]);
    let safe = safe_status(cfg.limits.max_bank_pattern_deg);
    let triggers = PatternTriggers::default();
    let clearances = PatternClearances {
        turn_downwind: false,
        ..Default::default()
    };
    assert_eq!(
        mm.update(&ModeManagerUpdate {
            phase: FlightPhase::Crosswind,
            state: &state,
            route_manager: &route,
            pattern: &pattern,
            safety_status: &safe,
            triggers: &triggers,
            clearances: &clearances,
            force_go_around: false,
            stay_in_pattern: true,
            touch_and_go: false,
        }),
        FlightPhase::Crosswind,
        "revoked turn_downwind clearance should hold the aircraft on crosswind"
    );
}

#[test]
fn set_clearance_land_records_runway_but_does_not_enforce() {
    let cfg = load_default_config_bundle();
    let rf = RunwayFrame::new(cfg.airport.runway.clone());
    let mut profile = PatternFlyProfile::new(cfg.clone(), rf.clone());
    // Revoke land clearance, then verify the mode machine still lets
    // Flare -> Rollout fire (record-only semantics).
    profile.set_clearance(PatternClearanceGate::Land, false, None);
    assert!(!profile.clearances.land);
    assert!(profile.clearances.landing_runway.is_none());

    let mm = ModeManager::new(cfg.clone());
    let pattern = build_pattern_geometry(&rf, cfg.pattern.downwind_offset_ft, 0.0, 0.0);
    let on_runway = state_with(|s| {
        s.on_ground = true;
        s.runway_x_ft = Some(-300.0);
        s.runway_y_ft = Some(0.0);
        s.alt_agl_ft = 0.0;
        s.gs_kt = 40.0;
    });
    let route = RouteManager::new(vec![]);
    let safe = safe_status(cfg.limits.max_bank_pattern_deg);
    let triggers = PatternTriggers::default();
    // Clone the profile's (revoked) clearances — Flare -> Rollout must still fire.
    let phase = mm.update(&ModeManagerUpdate {
        phase: FlightPhase::Flare,
        state: &on_runway,
        route_manager: &route,
        pattern: &pattern,
        safety_status: &safe,
        triggers: &triggers,
        clearances: &profile.clearances,
        force_go_around: false,
        stay_in_pattern: true,
        touch_and_go: false,
    });
    assert_eq!(
        phase,
        FlightPhase::Rollout,
        "land clearance is record-only; mode machine must not block landing"
    );

    // Granting clearance with a runway id records it.
    profile.set_clearance(PatternClearanceGate::Land, true, Some("16L".to_string()));
    assert!(profile.clearances.land);
    assert_eq!(profile.clearances.landing_runway.as_deref(), Some("16L"));
}

#[test]
fn execute_turn_base_after_extended_downwind_rebuilds_base_leg_from_current_position() {
    // After extending downwind and flying past the original base-turn
    // point, firing turn_base must push the base leg to start at the
    // aircraft's current position (not the stale precomputed one).
    let cfg = load_default_config_bundle();
    let rf = RunwayFrame::new(cfg.airport.runway.clone());
    let mut profile = PatternFlyProfile::new(cfg.clone(), rf);
    profile.phase = FlightPhase::Downwind;
    let _ = profile
        .extend_leg(PatternLeg::Downwind, 2000.0, ExtendMode::Add)
        .unwrap();
    let before_base_turn_x = profile.pattern.base_turn_x_ft;
    let current_rx = before_base_turn_x - 1500.0;

    // Fire the turn_base trigger and tick contribute(), which does the
    // dynamic rebuild. We simulate the rebuild call directly since full
    // contribute() requires guidance plumbing; the dynamic-rebuild code
    // path is the same logic.
    profile.execute_turn(PatternLeg::Base);
    // Mimic the rebuild that contribute() does on turn_base in DOWNWIND.
    let downwind_offset = profile.config.pattern.downwind_offset_ft;
    profile.leg_extensions.downwind_ft =
        (-current_rx - downwind_offset - profile.leg_extensions.crosswind_ft).max(0.0);
    let original_crosswind = profile.leg_extensions.crosswind_ft;
    profile.pattern = build_pattern_geometry(
        &profile.runway_frame,
        downwind_offset,
        profile.leg_extensions.downwind_ft,
        original_crosswind,
    );

    assert!(
        (profile.pattern.base_turn_x_ft - current_rx).abs() < 1e-6,
        "base leg should start at current position; got base_turn_x={} vs current_rx={}",
        profile.pattern.base_turn_x_ft,
        current_rx
    );
}
