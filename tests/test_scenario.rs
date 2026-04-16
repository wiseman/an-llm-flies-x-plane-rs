//! Port of tests/test_scenario.py — the end-to-end deterministic pattern mission.

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::sim::scenario::ScenarioRunner;
use xplane_pilot::types::{FlightPhase, Vec2};

#[test]
fn nominal_mission_completes_takeoff_to_rollout() {
    let result = ScenarioRunner::new(load_default_config_bundle()).run();
    assert!(result.success, "scenario did not mark success; final={:?} history_len={}", result.final_phase, result.history.len());
    for p in [
        FlightPhase::TakeoffRoll,
        FlightPhase::Rotate,
        FlightPhase::InitialClimb,
        FlightPhase::Crosswind,
        FlightPhase::Downwind,
        FlightPhase::Base,
        FlightPhase::Final,
        FlightPhase::Flare,
    ] {
        assert!(result.phases_seen.contains(&p), "missing phase {:?}", p);
    }
    assert!(matches!(result.final_phase, FlightPhase::Rollout | FlightPhase::TaxiClear));
    // Takeoff-originated missions stay in the pattern.
    for p in [
        FlightPhase::EnrouteClimb,
        FlightPhase::Cruise,
        FlightPhase::Descent,
        FlightPhase::PatternEntry,
    ] {
        assert!(!result.phases_seen.contains(&p), "unexpected phase {:?}", p);
    }
}

#[test]
fn crosswind_mission_still_tracks_and_lands() {
    let config = load_default_config_bundle();
    let runway_length = config.airport.runway.length_ft;
    let max_bank_final = config.limits.max_bank_final_deg;
    let mut runner = ScenarioRunner::new(config);
    runner.wind_vector_kt = Vec2::new(10.0, 0.0);
    let result = runner.run();
    assert!(matches!(result.final_phase, FlightPhase::Rollout | FlightPhase::TaxiClear));
    let td_x = result.touchdown_runway_x_ft.expect("touchdown x");
    assert!(td_x >= 0.0);
    assert!(td_x <= runway_length / 2.0);
    let td_y = result.touchdown_centerline_ft.unwrap_or(9999.0);
    assert!(td_y.abs() < 20.0);
    assert!(result.max_final_bank_deg.abs() < max_bank_final + 0.5);
}
