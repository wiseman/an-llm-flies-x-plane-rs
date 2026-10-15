//! End-to-end deterministic pattern mission.

mod common;

use common::exit;
use xplane_pilot::config::{load_default_config_bundle, ConfigBundle};
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::core::profiles::TaxiProfile;
use xplane_pilot::core::landing_rollout::{ArrivalPlan, LandingRolloutProfile, RunwayExitPlan};
use xplane_pilot::guidance::runway_geometry::RunwayFrame;
use xplane_pilot::sim::scenario::ScenarioRunner;
use xplane_pilot::sim::simple_dynamics::SimpleAircraftModel;
use xplane_pilot::sim::simple_dynamics::DynamicsState;
use xplane_pilot::types::{
    heading_to_vector, wrap_degrees_180, FlightPhase, StraightLeg, TaxiPose, Vec2, KT_TO_FPS,
};

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
fn short_runway_mission_lands_in_the_first_third() {
    // KWHP-sized: 4124 ft with the synthesized 1200 ft touchdown zone
    // (aim point 600 ft). No go-around should be needed, and touchdown
    // belongs in the first third.
    let mut config = load_default_config_bundle();
    config.airport.runway.length_ft = 4124.0;
    config.airport.runway.touchdown_zone_ft = 1200.0;
    let result = ScenarioRunner::new(config).run();
    let td_x = result.touchdown_runway_x_ft.expect("touchdown x");
    eprintln!("short runway touchdown x={td_x:.0} phases={:?}", result.phases_seen);
    assert!(!result.phases_seen.contains(&FlightPhase::GoAround), "went around");
    assert!(result.success, "touchdown x={td_x:.0} outside the zone");
    assert!(td_x <= 4124.0 / 3.0, "touchdown x={td_x:.0} past the first third");
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
    assert!(result.success, "crosswind landing outside the touchdown zone: x={td_x:.0}");
    let td_y = result.touchdown_centerline_ft.unwrap_or(9999.0);
    assert!(td_y.abs() < 20.0);
    assert!(result.max_final_bank_deg.abs() < max_bank_final + 0.5);
}

#[test]
fn selected_exit_drives_rollout_clear_of_runway() {
    let exit = exit("E", &[(3000.0, 0.0), (2950.0, 100.0), (2920.0, 140.0)]);
    let mut r = start_rollout(load_default_config_bundle(), Some(exit), 2000.0, 30.0);

    let mut first_runway_departure_x = None;
    for _ in 0..1000 {
        r.step();
        let position = r.runway_position();
        if first_runway_departure_x.is_none() && position.y.abs() >= 30.0 {
            first_runway_departure_x = Some(position.x);
        }
        if r.pilot.phase() == FlightPhase::TaxiClear {
            break;
        }
    }

    let runway_position = r.runway_position();
    assert_eq!(
        r.pilot.phase(),
        FlightPhase::TaxiClear,
        "state: pos={runway_position:?} ias={} heading={}",
        r.raw.ias_kt,
        r.raw.heading_deg
    );
    assert!(
        first_runway_departure_x.is_some_and(|x| x >= 2940.0),
        "left runway before the graph intersection: {first_runway_departure_x:?}"
    );
    assert!(
        runway_position.distance_to(Vec2::new(2920.0, 140.0)) < 25.0,
        "did not stop at active-zone boundary: {runway_position:?}"
    );
    assert!(!r.pilot.has_profile("landing_rollout"));
}

// ---------- closed-loop landing rollout on the simple ground model ----------

/// A 5000 ft runway so rollouts finish in a reasonable number of steps.
fn short_runway_config() -> ConfigBundle {
    let mut config = load_default_config_bundle();
    config.airport.runway.length_ft = 5000.0;
    config
}

struct Rollout {
    pilot: PilotCore,
    model: SimpleAircraftModel,
    frame: RunwayFrame,
    raw: DynamicsState,
}

/// Start a landing rollout `x_ft` down the centerline at `ias_kt`.
fn start_rollout(config: ConfigBundle, exit: Option<RunwayExitPlan>, x_ft: f64, ias_kt: f64) -> Rollout {
    let frame = RunwayFrame::new(config.airport.runway.clone());
    let profile = LandingRolloutProfile::new(config.clone(), frame, exit);
    start_rollout_with(config, profile, x_ft, ias_kt)
}

fn start_rollout_with(
    config: ConfigBundle,
    profile: LandingRolloutProfile,
    x_ft: f64,
    ias_kt: f64,
) -> Rollout {
    let frame = RunwayFrame::new(config.airport.runway.clone());
    let model = SimpleAircraftModel::new(config.clone(), Vec2::ZERO);
    let mut raw = model.initial_state_at(
        frame.to_world_frame(Vec2::new(x_ft, 0.0)),
        frame.runway.course_deg,
        config.airport.field_elevation_ft,
    );
    raw.ias_kt = ias_kt;
    raw.ground_velocity_ft_s = heading_to_vector(frame.runway.course_deg, ias_kt * KT_TO_FPS);
    let mut pilot = PilotCore::new(config);
    pilot.engage_profile(Box::new(profile)).unwrap();
    Rollout { pilot, model, frame, raw }
}

impl Rollout {
    fn step(&mut self) {
        let (_, commands) = self.pilot.update(&self.raw, 0.2);
        self.model.step(&mut self.raw, &commands, 0.2);
    }

    /// Step until stopped for 2 s, landing_rollout releases, or `max`.
    fn run_until_settled(&mut self, max: usize) {
        let mut stopped = 0;
        for _ in 0..max {
            self.step();
            if !self.pilot.has_profile("landing_rollout") {
                return;
            }
            stopped = if self.raw.ias_kt < 0.05 { stopped + 1 } else { 0 };
            if stopped >= 10 {
                return;
            }
        }
    }

    fn runway_position(&self) -> Vec2 {
        self.frame.to_runway_frame(self.raw.position_ft)
    }

    fn try_engage_taxi(&mut self) -> Result<Vec<String>, xplane_pilot::core::mission_manager::PreemptionRefused> {
        let here = self.raw.position_ft;
        let taxi = TaxiProfile::new(
            vec![StraightLeg { start_ft: here, end_ft: here + Vec2::new(100.0, 0.0) }],
            vec!["A".to_string()],
        );
        self.pilot.engage_profile(Box::new(taxi))
    }
}

#[test]
fn landing_without_exit_stops_on_centerline_and_hands_over_to_taxi() {
    let mut r = start_rollout(short_runway_config(), None, 1000.0, 45.0);
    assert!(r.try_engage_taxi().is_err(), "rolling rollout must not be preemptible");
    r.run_until_settled(2000);

    let pos = r.runway_position();
    assert_eq!(r.raw.ias_kt, 0.0, "should be stopped at {pos:?}");
    assert!(pos.y.abs() < 10.0, "drifted off centerline: {pos:?}");
    assert!(pos.x < 5000.0, "rolled off the end: {pos:?}");
    assert_eq!(r.pilot.list_profile_names(), vec!["landing_rollout"]);
    assert_eq!(r.pilot.phase(), FlightPhase::Rollout);
    let displaced = r.try_engage_taxi().expect("stopped rollout hands over");
    assert_eq!(displaced, vec!["landing_rollout"]);
}

#[test]
fn missed_exit_stops_short_of_runway_end_and_hands_over_to_taxi() {
    // Exit B is 80 ft ahead of an aircraft still doing 50 kt: too fast
    // to commit at the intersection.
    let exit = exit("B", &[(1080.0, 0.0), (1030.0, 100.0)]);
    let mut r = start_rollout(short_runway_config(), Some(exit), 1000.0, 50.0);
    r.run_until_settled(3000);

    let pos = r.runway_position();
    assert_eq!(r.raw.ias_kt, 0.0, "should be stopped at {pos:?}");
    assert!(pos.y.abs() < 10.0, "cut back toward the missed exit: {pos:?}");
    assert!(pos.x > 1140.0 && pos.x < 5000.0, "stop point {pos:?}");
    assert_eq!(r.pilot.phase(), FlightPhase::Rollout);
    r.try_engage_taxi().expect("stopped after missed exit hands over");
}

/// Roll onto exit E and report (phase, distance from the final node).
fn fly_exit_with_creep(creep_kt: f64) -> (FlightPhase, f64) {
    let mut config = short_runway_config();
    config.post_landing.exit_creep_speed_kt = creep_kt;
    let end = Vec2::new(3000.0, -120.0);
    let exit = exit("E", &[(3000.0, 0.0), (3000.0, -60.0), (end.x, end.y)]);
    let mut r = start_rollout(config, Some(exit), 2600.0, 15.0);
    let mut refused_mid_exit = false;
    for _ in 0..3000 {
        r.step();
        if !r.pilot.has_profile("landing_rollout") {
            break;
        }
        if r.pilot.phase() == FlightPhase::RunwayExit && !refused_mid_exit {
            assert!(r.try_engage_taxi().is_err(), "mid-exit rollout must not be preemptible");
            refused_mid_exit = true;
        }
    }
    assert!(refused_mid_exit, "never reached the exit");
    (r.pilot.phase(), r.runway_position().distance_to(end))
}

#[test]
fn exit_creeps_to_its_final_node_and_releases() {
    let (phase, miss_ft) = fly_exit_with_creep(2.0);
    assert_eq!(phase, FlightPhase::TaxiClear, "stopped {miss_ft:.1} ft from the node");
    assert!(miss_ft < 15.0, "stopped {miss_ft:.1} ft from the node");
}

#[test]
fn arrival_plan_rolls_out_exits_and_parks_without_further_input() {
    let config = short_runway_config();
    let frame = RunwayFrame::new(config.airport.runway.clone());
    let w = |p: Vec2| frame.to_world_frame(p);
    let exit_end = Vec2::new(3000.0, -120.0);
    let exit = exit("E", &[(3000.0, 0.0), (3000.0, -60.0), (exit_end.x, exit_end.y)]);
    // A parallel taxiway back toward the threshold, ending at a spot
    // that faces runway heading.
    let spot = Vec2::new(2500.0, -120.0);
    let spot_heading = frame.runway.course_deg;
    let pose = TaxiPose { position_ft: w(spot), heading_deg: spot_heading };
    let lead_in = TaxiProfile::pose_lead_in_point(
        w(exit_end),
        pose,
        config.ground_handling.min_turn_radius_ft(),
    )
    .unwrap_or(pose.position_ft);
    let taxi = TaxiProfile::new(
        vec![StraightLeg { start_ft: w(exit_end), end_ft: lead_in }],
        vec!["P".to_string()],
    )
    .with_final_pose(pose);
    let rollout = LandingRolloutProfile::new(config.clone(), frame.clone(), Some(exit)).with_arrival(
        Some(ArrivalPlan {
            parking_name: "Spot 1".to_string(),
            via_taxiways: vec![],
            taxi: Box::new(taxi),
            summary: "Spot 1".to_string(),
        }),
    );
    let mut r = start_rollout_with(config, rollout, 1500.0, 45.0);

    let mut saw_taxi = false;
    let mut settled = 0;
    for _ in 0..6000 {
        r.step();
        saw_taxi |= r.pilot.has_profile("taxi");
        settled = if saw_taxi && r.raw.ias_kt < 0.05 { settled + 1 } else { 0 };
        if settled >= 10 {
            break;
        }
    }
    let pos = r.runway_position();
    assert!(
        saw_taxi,
        "never handed off to the taxi; profiles {:?} at {pos:?}",
        r.pilot.list_profile_names()
    );
    assert!(
        pos.distance_to(spot) < 30.0,
        "parked {:.0} ft from the spot at {pos:?}",
        pos.distance_to(spot)
    );
    let heading_err = wrap_degrees_180(r.raw.heading_deg - spot_heading).abs();
    assert!(heading_err < 30.0, "heading off by {heading_err:.0}°");
}
