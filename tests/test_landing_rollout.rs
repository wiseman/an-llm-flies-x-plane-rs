//! LandingRolloutProfile: rollout, runway exit, stop, and preemption.

mod common;

use approx::assert_abs_diff_eq;
use common::exit;
use xplane_pilot::config::{load_default_config_bundle, ConfigBundle};
use xplane_pilot::core::landing_rollout::{ExitOption, LandingPlanHolder, LandingRolloutProfile};
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::core::profiles::{GuidanceProfile, PatternFlyProfile, TaxiProfile};
use xplane_pilot::guidance::runway_geometry::RunwayFrame;
use xplane_pilot::types::{AircraftState, FlightPhase, LateralMode, StraightLeg, Vec2};

fn setup() -> (ConfigBundle, RunwayFrame) {
    let cfg = load_default_config_bundle();
    let frame = RunwayFrame::new(cfg.airport.runway.clone());
    (cfg, frame)
}

fn on_ground(frame: &RunwayFrame, x: f64, y: f64, gs_kt: f64) -> AircraftState {
    AircraftState {
        runway_x_ft: Some(x),
        runway_y_ft: Some(y),
        position_ft: frame.to_world_frame(Vec2::new(x, y)),
        gs_kt,
        on_ground: true,
        ..AircraftState::synthetic_default()
    }
}

#[test]
fn selected_runway_exit_slows_without_leaving_centerline_early() {
    let (cfg, frame) = setup();
    let mut p = LandingRolloutProfile::new(
        cfg,
        frame.clone(),
        Some(exit("A5", &[(3000.0, 0.0), (2950.0, 100.0)])),
    );
    let tick = p.contribute(&on_ground(&frame, 2800.0, 0.0, 15.0), 0.2);
    assert_eq!(tick.contribution.lateral_mode, Some(LateralMode::RolloutCenterline));
    assert!(tick.contribution.target_path.is_none());
    // Braking for the turn only as it comes up: still at turnoff speed
    // 200 ft out, close to turn speed at the turn.
    assert_eq!(tick.contribution.target_speed_kt, Some(15.0));
    let near = p.contribute(&on_ground(&frame, 2975.0, 0.0, 8.0), 0.2).contribution.target_speed_kt.unwrap();
    assert!((5.0..7.5).contains(&near), "{near}");
}

#[test]
fn selected_runway_exit_commits_to_exact_graph_leg_at_intersection() {
    let (cfg, frame) = setup();
    let mut p = LandingRolloutProfile::new(
        cfg,
        frame.clone(),
        Some(exit("E", &[(3000.0, 0.0), (2950.0, -100.0), (2920.0, -140.0)])),
    );
    let tick = p.contribute(&on_ground(&frame, 2990.0, 0.0, 5.0), 0.2);
    assert_eq!(p.phase, FlightPhase::RunwayExit);
    assert_eq!(tick.contribution.lateral_mode, Some(LateralMode::TaxiFollow));
    let path = tick.contribution.target_path.expect("actual exit graph leg");
    let start = frame.to_runway_frame(path.start_ft);
    let end = frame.to_runway_frame(path.end_ft);
    assert_abs_diff_eq!(start.x, 3000.0, epsilon = 1e-6);
    assert_abs_diff_eq!(start.y, 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(end.x, 2950.0, epsilon = 1e-6);
    assert_abs_diff_eq!(end.y, -100.0, epsilon = 1e-6);
}

#[test]
fn missed_runway_exit_never_commands_a_cutback() {
    let (cfg, frame) = setup();
    let runway_length = frame.runway.length_ft;
    let turnoff_speed = cfg.post_landing.turnoff_speed_kt;
    let mut p = LandingRolloutProfile::new(
        cfg,
        frame.clone(),
        Some(exit("B", &[(3000.0, 0.0), (2950.0, 100.0)])),
    );
    let tick = p.contribute(&on_ground(&frame, 3070.0, 0.0, 15.0), 0.2);
    assert_eq!(p.phase, FlightPhase::Rollout);
    assert_eq!(tick.contribution.lateral_mode, Some(LateralMode::RolloutCenterline));
    assert!(tick.contribution.target_path.is_none());
    assert_eq!(tick.contribution.target_speed_kt, Some(turnoff_speed));

    let tick = p.contribute(&on_ground(&frame, runway_length - 100.0, 0.0, 15.0), 0.2);
    assert_eq!(tick.contribution.lateral_mode, Some(LateralMode::RolloutCenterline));
    assert_eq!(tick.contribution.target_speed_kt, Some(0.0));
}

#[test]
fn a_missed_exit_falls_back_to_the_next_one_it_can_make() {
    let (cfg, frame) = setup();
    let fallbacks = vec![
        // Too close to slow for once B is missed at 3070 ft.
        ExitOption { exit: exit("C", &[(3200.0, 0.0), (3150.0, 100.0)]), arrival: None },
        ExitOption { exit: exit("D", &[(4500.0, 0.0), (4450.0, 100.0)]), arrival: None },
    ];
    let mut p = LandingRolloutProfile::new(cfg, frame.clone(), Some(exit("B", &[(3000.0, 0.0), (2950.0, 100.0)])))
        .with_fallback_exits(fallbacks);
    p.contribute(&on_ground(&frame, 3070.0, 0.0, 30.0), 0.2);
    assert_eq!(p.preferred_exit().map(|e| e.taxiway_name.as_str()), Some("D"));
    let events = p.take_events();
    assert_eq!(events.len(), 1, "{events:?}");
    assert!(events[0].contains("missed exit B") && events[0].contains("taking D"), "{events:?}");

    // D is flown like any chosen exit.
    let tick = p.contribute(&on_ground(&frame, 4495.0, 0.0, 5.0), 0.2);
    assert_eq!(p.phase, FlightPhase::RunwayExit);
    assert_eq!(tick.contribution.lateral_mode, Some(LateralMode::TaxiFollow));

    // Missing the last one says so, once.
    let mut p = LandingRolloutProfile::new(load_default_config_bundle(), frame.clone(), Some(exit("B", &[(3000.0, 0.0), (2950.0, 100.0)])));
    p.contribute(&on_ground(&frame, 3070.0, 0.0, 30.0), 0.2);
    p.contribute(&on_ground(&frame, 3100.0, 0.0, 28.0), 0.2);
    let events = p.take_events();
    assert_eq!(events.len(), 1, "{events:?}");
    assert!(events[0].contains("no planned exit ahead"), "{events:?}");
}

#[test]
fn stopped_rollout_holds_runway_but_is_preemptible() {
    let (cfg, frame) = setup();
    let mut p = LandingRolloutProfile::new(cfg, frame.clone(), None);
    p.contribute(&on_ground(&frame, 3000.0, 0.0, 12.0), 0.2);
    assert!(p.preemption().is_err(), "still rolling");

    let tick = p.contribute(&on_ground(&frame, 3000.0, 0.0, 0.0), 0.2);
    // Keeps the brakes on rather than releasing to idle on the runway,
    // but must not deadlock a ground taxi profile.
    assert!(tick.hand_off.is_none());
    assert_eq!(p.phase, FlightPhase::Rollout);
    assert!(p.holding_on_runway());
    assert!(p.preemption().is_ok());
    assert!(p.is_complete(), "stopped on the runway wakes the LLM");
}

#[test]
fn stopped_after_missed_exit_is_preemptible() {
    let (cfg, frame) = setup();
    let x = frame.runway.length_ft - 250.0;
    let mut p = LandingRolloutProfile::new(
        cfg,
        frame.clone(),
        Some(exit("B", &[(3000.0, 0.0), (2950.0, 100.0)])),
    );
    p.contribute(&on_ground(&frame, x, 0.0, 0.0), 0.2);
    assert_eq!(p.phase, FlightPhase::Rollout);
    assert!(p.preemption().is_ok());
}

#[test]
fn exit_final_leg_keeps_creeping_until_node_captured() {
    let (cfg, frame) = setup();
    let mut p = LandingRolloutProfile::new(
        cfg,
        frame.clone(),
        Some(exit("E", &[(3000.0, 0.0), (3000.0, -100.0)])),
    );
    p.contribute(&on_ground(&frame, 2995.0, 0.0, 5.0), 0.2);
    assert_eq!(p.phase, FlightPhase::RunwayExit);
    // Stopped 12 ft short of the final node, outside its 8 ft capture
    // radius: guidance must still ask for forward motion.
    let tick = p.contribute(&on_ground(&frame, 3000.0, -88.0, 0.0), 0.2);
    assert_eq!(p.phase, FlightPhase::RunwayExit);
    let speed = tick.contribution.target_speed_kt.unwrap();
    assert!(speed >= 2.0, "target speed {speed} would stall short of the node");
    assert!(p.preemption().is_err(), "mid-exit must not be preemptible");
    // Once captured it stops and releases to idle.
    let tick = p.contribute(&on_ground(&frame, 3000.0, -95.0, 0.5), 0.2);
    assert_eq!(p.phase, FlightPhase::TaxiClear);
    assert!(tick.hand_off.is_some());
}

#[test]
fn preferred_exit_cannot_change_once_turning_off() {
    let (cfg, frame) = setup();
    let mut p = LandingRolloutProfile::new(cfg, frame.clone(), None);
    p.set_preferred_exit(Some(exit("E", &[(3000.0, 0.0), (3000.0, -100.0)]))).unwrap();
    p.contribute(&on_ground(&frame, 2995.0, 0.0, 5.0), 0.2);
    assert_eq!(p.phase, FlightPhase::RunwayExit);
    let err = p
        .set_preferred_exit(Some(exit("F", &[(4000.0, 0.0), (4000.0, -100.0)])))
        .unwrap_err();
    assert!(err.contains('E'), "{err}");
    assert!(p.set_preferred_exit(None).is_err());
    assert_eq!(p.preferred_exit().unwrap().taxiway_name, "E");
}

#[test]
fn engine_out_rollout_keeps_throttle_closed_and_refuses_exits() {
    let (cfg, frame) = setup();
    let mut p = LandingRolloutProfile::new(cfg, frame.clone(), None).without_power();
    let tick = p.contribute(&on_ground(&frame, 3000.0, 0.0, 30.0), 0.2);
    assert_eq!(tick.contribution.throttle_limit, Some((0.0, 0.0)));
    assert!(p
        .set_preferred_exit(Some(exit("E", &[(4000.0, 0.0), (4000.0, -100.0)])))
        .is_err());
}

#[test]
fn pattern_fly_hands_off_to_landing_rollout_at_touchdown() {
    let (cfg, frame) = setup();
    let mut pilot = PilotCore::new(cfg.clone());
    let mut pattern = PatternFlyProfile::new(cfg, frame.clone());
    pattern.phase = FlightPhase::Flare;
    pattern.preferred_exit = Some(exit("E", &[(3000.0, 0.0), (3000.0, -100.0)]));
    pilot.engage_profile(Box::new(pattern)).unwrap();

    let raw = xplane_pilot::sim::simple_dynamics::SimpleAircraftModel::new(
        pilot.config.clone(),
        Vec2::ZERO,
    )
    .initial_state_at(
        frame.to_world_frame(Vec2::new(1200.0, 0.0)),
        frame.runway.course_deg,
        pilot.config.airport.field_elevation_ft,
    );
    let mut raw = raw;
    raw.on_ground = true;
    raw.ias_kt = 55.0;
    // The hand-off waits for the wheels to stay down (bounce debounce).
    pilot.update(&raw, 0.2);
    assert_eq!(pilot.list_profile_names(), vec!["pattern_fly"], "not yet settled");
    for _ in 0..5 {
        pilot.update(&raw, 0.2);
    }

    assert_eq!(pilot.list_profile_names(), vec!["landing_rollout"]);
    assert_eq!(pilot.phase(), FlightPhase::Rollout);
    let carried = pilot
        .profile_mut::<LandingRolloutProfile>().map(|r| r.preferred_exit().cloned())
        .unwrap();
    assert_eq!(carried.unwrap().taxiway_name, "E");
}

#[test]
fn rolling_landing_rollout_refuses_ground_taxi_engage() {
    let (cfg, frame) = setup();
    let mut pilot = PilotCore::new(cfg.clone());
    let mut rollout = LandingRolloutProfile::new(cfg, frame.clone(), None);
    rollout.contribute(&on_ground(&frame, 2000.0, 0.0, 30.0), 0.2);
    pilot.engage_profile(Box::new(rollout)).unwrap();

    let taxi = TaxiProfile::new(
        vec![StraightLeg { start_ft: Vec2::ZERO, end_ft: Vec2::new(0.0, 100.0) }],
        vec!["A".to_string()],
    );
    let err = pilot.engage_profile(Box::new(taxi)).unwrap_err();
    assert_eq!(err.profile, "landing_rollout");
    assert!(err.reason.contains("rolling"), "{}", err.reason);
    assert_eq!(pilot.list_profile_names(), vec!["landing_rollout"]);
    assert!(pilot.disengage_profile("landing_rollout").is_err());
}

#[test]
fn leaving_the_runway_is_judged_from_its_width() {
    let (cfg, mut frame) = setup();
    frame.runway.width_ft = 50.0;
    let margin = cfg.post_landing.runway_clear_margin_ft;
    let mut p = LandingRolloutProfile::new(cfg, frame.clone(), None);
    // Inside half-width + margin: still on the runway.
    p.contribute(&on_ground(&frame, 3000.0, 25.0 + margin - 1.0, 8.0), 0.2);
    assert_eq!(p.phase, FlightPhase::Rollout);
    // Just past it: off the runway surface.
    p.contribute(&on_ground(&frame, 3000.0, 25.0 + margin + 1.0, 8.0), 0.2);
    assert_eq!(p.phase, FlightPhase::RunwayExit);
}

#[test]
fn touching_down_beside_the_runway_and_rolling_back_on_is_not_clear() {
    // The live KWHP case: touchdown 62 ft left of centerline (75 ft wide
    // runway), rolled back onto the pavement, stopped 12 ft off centerline.
    let (cfg, mut frame) = setup();
    frame.runway.width_ft = 75.0;
    let mut p = LandingRolloutProfile::new(cfg, frame.clone(), None);
    p.contribute(&on_ground(&frame, 900.0, -62.0, 60.0), 0.2);
    assert_eq!(p.phase, FlightPhase::RunwayExit);
    p.contribute(&on_ground(&frame, 1200.0, -9.0, 49.0), 0.2);
    assert_eq!(p.phase, FlightPhase::Rollout, "back on the runway");
    let tick = p.contribute(&on_ground(&frame, 1581.0, -12.4, 0.0), 0.2);
    assert_eq!(p.phase, FlightPhase::Rollout);
    assert!(tick.hand_off.is_none(), "must not release to idle on the runway");
    assert!(p.holding_on_runway());
}

#[test]
fn a_bounce_stays_with_pattern_fly_and_can_go_around() {
    let (cfg, frame) = setup();
    let field_elev = cfg.airport.field_elevation_ft;
    let mut pattern = PatternFlyProfile::new(cfg, frame.clone());
    pattern.phase = FlightPhase::Flare;
    let touch = AircraftState { alt_msl_ft: field_elev, alt_agl_ft: 0.0, ..on_ground(&frame, 1200.0, 0.0, 55.0) };
    let bounced = AircraftState {
        on_ground: false,
        alt_agl_ft: 12.0,
        alt_msl_ft: field_elev + 12.0,
        ..touch.clone()
    };
    let tick = pattern.contribute(&touch, 0.2);
    assert!(tick.hand_off.is_none());
    let tick = pattern.contribute(&bounced, 0.2);
    assert!(tick.hand_off.is_none());
    assert_eq!(pattern.phase, FlightPhase::Flare, "bounce returns to the flare");
    pattern.go_around();
    pattern.contribute(&bounced, 0.2);
    assert_eq!(pattern.phase, FlightPhase::GoAround);
}

#[test]
fn landing_rollout_is_preemptible_while_airborne() {
    let (cfg, frame) = setup();
    let mut p = LandingRolloutProfile::new(cfg, frame.clone(), None);
    p.contribute(&on_ground(&frame, 1500.0, 0.0, 50.0), 0.2);
    assert!(p.preemption().is_err());
    let airborne = AircraftState { on_ground: false, ..on_ground(&frame, 1600.0, 0.0, 50.0) };
    p.contribute(&airborne, 0.2);
    assert!(p.preemption().is_ok());
}

#[test]
fn touchdown_far_off_centerline_hands_straight_through_to_idle() {
    // Stopped already >125 ft off centerline when the hand-off happens:
    // the rollout's own first-tick hand-off must not be dropped.
    let (cfg, frame) = setup();
    let rollout = LandingRolloutProfile::new(cfg, frame.clone(), None);
    let state = on_ground(&frame, 1500.0, -200.0, 0.0);
    let tick = xplane_pilot::core::landing_rollout::touchdown_handoff(rollout, &state, 0.2);
    let names: Vec<&str> = tick.hand_off.as_ref().unwrap().iter().map(|p| p.name()).collect();
    assert_eq!(names, vec!["idle_lateral", "idle_vertical", "idle_speed"]);
}
