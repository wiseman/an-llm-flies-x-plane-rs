//! End-to-end deterministic pattern mission.

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::core::profiles::{PatternFlyProfile, RunwayExitPlan};
use xplane_pilot::guidance::runway_geometry::RunwayFrame;
use xplane_pilot::sim::scenario::ScenarioRunner;
use xplane_pilot::sim::simple_dynamics::SimpleAircraftModel;
use xplane_pilot::types::{heading_to_vector, FlightPhase, Vec2, KT_TO_FPS};

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

#[test]
fn selected_exit_drives_rollout_clear_of_runway() {
    let config = load_default_config_bundle();
    let frame = RunwayFrame::new(config.airport.runway.clone());
    let model = SimpleAircraftModel::new(config.clone(), Vec2::ZERO);
    let mut raw = model.initial_state_at(
        frame.to_world_frame(Vec2::new(2000.0, 0.0)),
        frame.runway.course_deg,
        config.airport.field_elevation_ft,
    );
    raw.ias_kt = 30.0;
    raw.ground_velocity_ft_s =
        heading_to_vector(frame.runway.course_deg, raw.ias_kt * KT_TO_FPS);

    let mut profile = PatternFlyProfile::new(config.clone(), frame.clone());
    profile.phase = FlightPhase::Rollout;
    profile.set_preferred_exit(Some(RunwayExitPlan {
        taxiway_name: "E".to_string(),
        stationing_ft: 3000.0,
        path_runway_ft: vec![
            Vec2::new(3000.0, 0.0),
            Vec2::new(2950.0, 100.0),
            Vec2::new(2920.0, 140.0),
        ],
    }));
    let mut pilot = PilotCore::new(config);
    pilot.engage_profile(Box::new(profile));

    let mut first_runway_departure_x = None;
    for _ in 0..1000 {
        let (_, commands) = pilot.update(&raw, 0.2);
        model.step(&mut raw, &commands, 0.2);
        let position = frame.to_runway_frame(raw.position_ft);
        if first_runway_departure_x.is_none() && position.y.abs() >= 30.0 {
            first_runway_departure_x = Some(position.x);
        }
        if pilot.phase() == FlightPhase::TaxiClear {
            break;
        }
    }

    let runway_position = frame.to_runway_frame(raw.position_ft);
    assert_eq!(
        pilot.phase(),
        FlightPhase::TaxiClear,
        "state: pos={runway_position:?} ias={} heading={}",
        raw.ias_kt,
        raw.heading_deg
    );
    assert!(
        first_runway_departure_x.is_some_and(|x| x >= 2940.0),
        "left runway before the graph intersection: {first_runway_departure_x:?}"
    );
    assert!(
        runway_position.distance_to(Vec2::new(2920.0, 140.0)) < 25.0,
        "did not stop at active-zone boundary: {runway_position:?}"
    );
    assert!(!pilot.has_profile("pattern_fly"));
}
