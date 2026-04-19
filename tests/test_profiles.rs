//! Port of tests/test_profiles.py — profile composition and PatternFlyProfile behavior.

use std::collections::BTreeSet;

use approx::assert_abs_diff_eq;
use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::core::profiles::{
    build_rotate_guidance, build_takeoff_roll_guidance, AltitudeHoldProfile, GuidanceProfile,
    HeadingHoldProfile, PatternFlyProfile, SpeedHoldProfile, TakeoffProfile, TaxiProfile,
};
use xplane_pilot::sim::simple_dynamics::SimpleAircraftModel;
use xplane_pilot::types::{
    heading_to_vector, AircraftState, FlightPhase, LateralMode, StraightLeg, Vec2, VerticalMode,
    KT_TO_FPS,
};

fn make_pilot() -> (xplane_pilot::config::ConfigBundle, PilotCore) {
    let cfg = load_default_config_bundle();
    let pilot = PilotCore::new(cfg.clone());
    (cfg, pilot)
}

#[test]
fn new_pilot_starts_with_three_idle_profiles() {
    let (_, pilot) = make_pilot();
    let names: BTreeSet<String> = pilot.list_profile_names().into_iter().collect();
    let expected: BTreeSet<String> =
        ["idle_lateral", "idle_vertical", "idle_speed"].iter().map(|s| s.to_string()).collect();
    assert_eq!(names, expected);
}

#[test]
fn engaging_heading_hold_displaces_idle_lateral_only() {
    let (_, mut pilot) = make_pilot();
    let displaced = pilot.engage_profile(Box::new(
        HeadingHoldProfile::new(270.0, 25.0, None).unwrap(),
    ));
    assert_eq!(displaced, vec!["idle_lateral"]);
    let names: BTreeSet<String> = pilot.list_profile_names().into_iter().collect();
    let expected: BTreeSet<String> =
        ["idle_vertical", "idle_speed", "heading_hold"].iter().map(|s| s.to_string()).collect();
    assert_eq!(names, expected);
}

#[test]
fn engaging_pattern_fly_displaces_all_three_idle_profiles() {
    let (cfg, mut pilot) = make_pilot();
    let rf = pilot.runway_frame.clone();
    let displaced = pilot.engage_profile(Box::new(PatternFlyProfile::new(cfg, rf)));
    let displaced_set: BTreeSet<String> = displaced.into_iter().collect();
    let expected: BTreeSet<String> =
        ["idle_lateral", "idle_vertical", "idle_speed"].iter().map(|s| s.to_string()).collect();
    assert_eq!(displaced_set, expected);
    assert_eq!(pilot.list_profile_names(), vec!["pattern_fly"]);
}

#[test]
fn disengage_pattern_fly_readds_three_idle_profiles() {
    let (cfg, mut pilot) = make_pilot();
    let rf = pilot.runway_frame.clone();
    pilot.engage_profile(Box::new(PatternFlyProfile::new(cfg, rf)));
    let added: BTreeSet<String> = pilot.disengage_profile("pattern_fly").into_iter().collect();
    let expected: BTreeSet<String> =
        ["idle_lateral", "idle_vertical", "idle_speed"].iter().map(|s| s.to_string()).collect();
    assert_eq!(added, expected);
}

#[test]
fn disengage_unknown_profile_is_noop() {
    let (_, mut pilot) = make_pilot();
    assert_eq!(pilot.disengage_profile("no_such"), Vec::<String>::new());
}

#[test]
fn alt_and_speed_hold_compose_without_conflict() {
    let (_, mut pilot) = make_pilot();
    pilot.engage_profile(Box::new(AltitudeHoldProfile::new(3000.0)));
    pilot.engage_profile(Box::new(SpeedHoldProfile::new(95.0)));
    let names: BTreeSet<String> = pilot.list_profile_names().into_iter().collect();
    let expected: BTreeSet<String> =
        ["idle_lateral", "altitude_hold", "speed_hold"].iter().map(|s| s.to_string()).collect();
    assert_eq!(names, expected);
}

#[test]
fn engage_profiles_atomically_replaces_pattern_fly() {
    let (cfg, mut pilot) = make_pilot();
    let rf = pilot.runway_frame.clone();
    pilot.engage_profile(Box::new(PatternFlyProfile::new(cfg.clone(), rf)));
    let displaced = pilot.engage_profiles(vec![
        Box::new(HeadingHoldProfile::new(90.0, 25.0, None).unwrap()),
        Box::new(AltitudeHoldProfile::new(3000.0)),
        Box::new(SpeedHoldProfile::new(95.0)),
    ]);
    assert!(displaced.contains(&"pattern_fly".to_string()));
    let names: BTreeSet<String> = pilot.list_profile_names().into_iter().collect();
    let expected: BTreeSet<String> =
        ["heading_hold", "altitude_hold", "speed_hold"].iter().map(|s| s.to_string()).collect();
    assert_eq!(names, expected);
}

#[test]
fn takeoff_roll_guidance_goes_full_power() {
    let (cfg, pilot) = make_pilot();
    let g = build_takeoff_roll_guidance(&cfg, &pilot.runway_frame);
    assert_eq!(g.throttle_limit, Some((1.0, 1.0)));
    assert_eq!(g.lateral_mode, LateralMode::RolloutCenterline);
    assert_eq!(g.target_pitch_deg, Some(0.0));
    assert_eq!(g.target_speed_kt, Some(cfg.performance.vr_kt));
}

#[test]
fn takeoff_roll_guidance_commands_takeoff_flaps() {
    let (cfg, pilot) = make_pilot();
    let g = build_takeoff_roll_guidance(&cfg, &pilot.runway_frame);
    assert_eq!(g.flaps_cmd, Some(10));
}

fn rotate_state(config: &xplane_pilot::config::ConfigBundle, on_ground: bool, track_override: Option<f64>) -> AircraftState {
    let course = config.airport.runway.course_deg;
    let track = track_override.unwrap_or(course);
    AircraftState {
        t_sim: 0.0,
        dt: 0.2,
        position_ft: Vec2::ZERO,
        alt_msl_ft: config.airport.field_elevation_ft + if on_ground { 0.0 } else { 20.0 },
        alt_agl_ft: if on_ground { 0.0 } else { 20.0 },
        pitch_deg: if on_ground { 3.0 } else { 8.0 },
        roll_deg: 0.0,
        heading_deg: track,
        track_deg: track,
        p_rad_s: 0.0,
        q_rad_s: 0.0,
        r_rad_s: 0.0,
        ias_kt: config.performance.vr_kt + 2.0,
        tas_kt: config.performance.vr_kt + 2.0,
        gs_kt: config.performance.vr_kt + 2.0,
        vs_fpm: if on_ground { 0.0 } else { 200.0 },
        ground_velocity_ft_s: Vec2::ZERO,
        flap_index: 0,
        gear_down: true,
        on_ground,
        throttle_pos: 1.0,
        runway_id: config.airport.runway.id.clone(),
        runway_dist_remaining_ft: None,
        runway_x_ft: Some(1800.0),
        runway_y_ft: Some(10.0),
        centerline_error_ft: Some(10.0),
        threshold_abeam: false,
        distance_to_touchdown_ft: None,
        stall_margin: 1.5,
    }
}

#[test]
fn rotate_guidance_on_ground_uses_rollout_centerline() {
    let (cfg, pilot) = make_pilot();
    let state = rotate_state(&cfg, true, None);
    let g = build_rotate_guidance(
        &cfg,
        &pilot.runway_frame,
        &state,
        cfg.limits.max_bank_enroute_deg,
    );
    assert_eq!(g.throttle_limit, Some((1.0, 1.0)));
    assert_eq!(g.target_pitch_deg, Some(8.0));
    assert_eq!(g.target_speed_kt, Some(cfg.performance.vy_kt));
    assert_eq!(g.lateral_mode, LateralMode::RolloutCenterline);
    assert_eq!(g.target_track_deg, Some(cfg.airport.runway.course_deg));
}

#[test]
fn rotate_guidance_airborne_banks_toward_runway_course() {
    let (cfg, pilot) = make_pilot();
    let state = rotate_state(&cfg, false, Some(30.0));
    let g = build_rotate_guidance(
        &cfg,
        &pilot.runway_frame,
        &state,
        cfg.limits.max_bank_enroute_deg,
    );
    assert_eq!(g.lateral_mode, LateralMode::TrackHold);
    assert_eq!(g.target_track_deg, Some(cfg.airport.runway.course_deg));
    assert!(g.target_bank_deg.unwrap() < 0.0);
}

#[test]
fn takeoff_profile_owns_all_three_axes() {
    let (cfg, pilot) = make_pilot();
    let profile = TakeoffProfile::new(cfg, pilot.runway_frame.clone());
    let owns: BTreeSet<_> = xplane_pilot::core::profiles::GuidanceProfile::owns(&profile).into_iter().collect();
    use xplane_pilot::core::profiles::Axis;
    let expected: BTreeSet<_> = [Axis::Lateral, Axis::Vertical, Axis::Speed].into_iter().collect();
    assert_eq!(owns, expected);
}

#[test]
fn takeoff_starts_in_preflight_and_advances_on_acceleration() {
    let (cfg, mut pilot) = make_pilot();
    let rf = pilot.runway_frame.clone();
    pilot.engage_profile(Box::new(TakeoffProfile::new(cfg.clone(), rf)));
    let model = SimpleAircraftModel::new(cfg, Vec2::ZERO);
    let mut raw_state = model.initial_state();

    let mut phases_seen: BTreeSet<FlightPhase> = BTreeSet::new();
    for _ in 0..200 {
        let (_, commands) = pilot.update(&raw_state, 0.2);
        phases_seen.insert(pilot.phase());
        let mut stepped = raw_state.clone();
        model.step(&mut stepped, &commands, 0.2);
        raw_state = stepped;
        if pilot.phase() == FlightPhase::InitialClimb {
            break;
        }
    }
    assert!(phases_seen.contains(&FlightPhase::TakeoffRoll));
    assert!(phases_seen.contains(&FlightPhase::Rotate));
    assert!(phases_seen.contains(&FlightPhase::InitialClimb));
}

#[test]
fn takeoff_profile_initial_climb_uses_l1_and_banks_toward_centerline() {
    // Aircraft 3000 ft past the threshold, 400 ft offset to the "right"
    // of the runway centerline (positive y in runway frame), climbing on
    // runway heading. The new L1-follower-backed initial_climb guidance
    // should command a LEFT bank (negative) to recover the centerline.
    let (cfg, pilot) = make_pilot();
    let rf = pilot.runway_frame.clone();
    let mut profile = TakeoffProfile::new(cfg.clone(), rf.clone());
    // Pin the phase — without this the ModeManager would start in
    // Preflight and need us to drive through takeoff_roll/rotate before
    // it settles on InitialClimb.
    profile.phase = FlightPhase::InitialClimb;

    let course = cfg.airport.runway.course_deg;
    let position_ft = rf.to_world_frame(Vec2::new(3000.0, 400.0));
    let vy = cfg.performance.vy_kt;
    let state = AircraftState {
        position_ft,
        alt_msl_ft: cfg.airport.field_elevation_ft + 400.0,
        alt_agl_ft: 400.0,
        heading_deg: course,
        track_deg: course,
        ias_kt: vy,
        tas_kt: vy,
        gs_kt: vy,
        ground_velocity_ft_s: heading_to_vector(course, vy * KT_TO_FPS),
        on_ground: false,
        ..AircraftState::synthetic_default()
    };
    use xplane_pilot::core::profiles::GuidanceProfile;
    let tick = profile.contribute(&state, 0.2);
    let g = tick.contribution;
    assert_eq!(g.lateral_mode, Some(LateralMode::PathFollow));
    assert!(g.target_path.is_some(), "target_path should be set for L1 follow");
    let bank = g.target_bank_deg.expect("L1 should command a bank");
    assert!(
        bank < -0.5,
        "aircraft offset right of centerline should bank left (negative), got {}",
        bank
    );
}

#[test]
fn shortest_path_default_goes_left_from_060_to_290() {
    let mut profile = HeadingHoldProfile::new(290.0, 25.0, None).unwrap();
    let state = AircraftState {
        track_deg: 60.0,
        heading_deg: 60.0,
        ground_velocity_ft_s: heading_to_vector(60.0, 90.0 * KT_TO_FPS),
        ..AircraftState::synthetic_default()
    };
    use xplane_pilot::core::profiles::GuidanceProfile;
    let tick = profile.contribute(&state, 0.2);
    assert!(tick.contribution.target_bank_deg.unwrap() < 0.0);
}

#[test]
fn forced_right_direction_banks_right_even_when_left_is_shorter() {
    let mut profile = HeadingHoldProfile::new(290.0, 25.0, Some("right")).unwrap();
    let state = AircraftState {
        track_deg: 60.0,
        heading_deg: 60.0,
        ground_velocity_ft_s: heading_to_vector(60.0, 90.0 * KT_TO_FPS),
        ..AircraftState::synthetic_default()
    };
    use xplane_pilot::core::profiles::GuidanceProfile;
    let tick = profile.contribute(&state, 0.2);
    assert!(tick.contribution.target_bank_deg.unwrap() > 0.0);
}

#[test]
fn invalid_direction_raises() {
    assert!(HeadingHoldProfile::new(270.0, 25.0, Some("backwards")).is_err());
}

#[test]
fn altitude_hold_contribution_is_regime_free() {
    // After the limit-cycle fix, AltitudeHoldProfile no longer switches
    // between discrete throttle/override regimes at ±150 ft. Its
    // contribution is identical whether the aircraft is far below, far
    // above, or near the target — TECS decides the actual thrust.
    use xplane_pilot::core::profiles::GuidanceProfile;
    let mut profile = AltitudeHoldProfile::new(2000.0);
    let far_below = AircraftState {
        alt_msl_ft: 1387.0,
        ..AircraftState::synthetic_default()
    };
    let far_above = AircraftState {
        alt_msl_ft: 3000.0,
        ..AircraftState::synthetic_default()
    };
    let near_target = AircraftState {
        alt_msl_ft: 1950.0,
        ..AircraftState::synthetic_default()
    };
    for state in [far_below, far_above, near_target] {
        let tick = profile.contribute(&state, 0.2);
        assert_eq!(tick.contribution.tecs_phase_override, None);
        assert_eq!(tick.contribution.throttle_limit, Some((0.0, 1.0)));
        assert_eq!(
            tick.contribution.target_altitude_ft,
            Some(2000.0)
        );
    }
}

#[test]
fn pattern_fly_route_has_only_pattern_entry_waypoint() {
    let (cfg, pilot) = make_pilot();
    let profile = PatternFlyProfile::new(cfg, pilot.runway_frame.clone());
    let names: Vec<String> = profile.route_manager.waypoints.iter().map(|w| w.name.clone()).collect();
    assert_eq!(names, vec!["pattern_entry_start"]);
}

#[test]
fn pattern_fly_go_around_targets_runway_course() {
    let (cfg, mut pilot) = make_pilot();
    let mut profile = PatternFlyProfile::new(cfg.clone(), pilot.runway_frame.clone());
    let state = AircraftState {
        track_deg: 200.0,
        heading_deg: 200.0,
        ..AircraftState::synthetic_default()
    };
    let g = profile.guidance_for_phase(&state, FlightPhase::GoAround);
    assert_eq!(g.lateral_mode, LateralMode::TrackHold);
    assert_abs_diff_eq!(g.target_track_deg.unwrap(), cfg.airport.runway.course_deg, epsilon = 1e-3);
    let _ = pilot.list_profile_names();
}

#[test]
fn flap_schedule_per_phase_follows_c172_sop() {
    let (cfg, pilot) = make_pilot();
    let mut profile = PatternFlyProfile::new(cfg, pilot.runway_frame.clone());
    let state = AircraftState {
        runway_x_ft: Some(-3000.0),
        runway_y_ft: Some(0.0),
        ..AircraftState::synthetic_default()
    };
    let cases = [
        (FlightPhase::PatternEntry, 0),
        (FlightPhase::Downwind, 10),
        (FlightPhase::Base, 20),
        (FlightPhase::Final, 30),
    ];
    for (phase, expected) in cases {
        let g = profile.guidance_for_phase(&state, phase);
        assert_eq!(g.flaps_cmd, Some(expected), "phase {:?}", phase);
    }
}

#[test]
fn altitude_hold_end_to_end_climb_through_pilot() {
    let (_, mut pilot) = make_pilot();
    pilot.engage_profile(Box::new(AltitudeHoldProfile::new(2000.0)));
    pilot.engage_profile(Box::new(SpeedHoldProfile::new(80.0)));
    let raw = xplane_pilot::sim::simple_dynamics::DynamicsState {
        position_ft: Vec2::ZERO,
        altitude_ft: 1387.0,
        heading_deg: 0.0,
        roll_deg: 0.0,
        pitch_deg: 0.0,
        ias_kt: 80.0,
        throttle_pos: 0.5,
        on_ground: false,
        time_s: 10.0,
        p_rad_s: 0.0,
        q_rad_s: 0.0,
        r_rad_s: 0.0,
        ground_velocity_ft_s: Vec2::ZERO,
        vertical_speed_ft_s: 0.0,
        flap_index: 0,
        gear_down: true,
    };
    // Run enough ticks for TECS to integrate up toward its climb
    // authority — the old regime switch floored throttle at 0.7
    // instantly; the new continuous loop needs a few ticks to wind up
    // from the cruise trim (0.58) via the integrator.
    let mut last_throttle = 0.0;
    let mut last_elevator = 0.0;
    for _ in 0..20 {
        let (_, cmds) = pilot.update(&raw, 0.2);
        last_throttle = cmds.throttle;
        last_elevator = cmds.elevator;
    }
    // With 613 ft of alt error sustained, throttle should rise above
    // the cruise trim (0.58) under TECS integral authority and elevator
    // should be pushing pitch up. Hard values depend on gains; we only
    // assert that the control loop is responding in the right direction.
    assert!(
        last_throttle > 0.6,
        "throttle did not climb above trim: {}",
        last_throttle
    );
    assert!(last_elevator > 0.0, "elevator was {}", last_elevator);
    let _ = VerticalMode::Tecs;
}

// ---------- TaxiProfile ----------

fn taxi_state(pos: Vec2, heading_deg: f64, gs_kt: f64) -> AircraftState {
    let mut s = AircraftState::synthetic_default();
    s.on_ground = true;
    s.position_ft = pos;
    s.heading_deg = heading_deg;
    s.track_deg = heading_deg;
    s.ias_kt = gs_kt;
    s.gs_kt = gs_kt;
    s.alt_agl_ft = 0.0;
    s.ground_velocity_ft_s = heading_to_vector(heading_deg, gs_kt * KT_TO_FPS);
    s
}

/// Three legs: straight east, sharp left (~90°) to north, straight north.
fn three_leg_ninety_deg() -> (Vec<StraightLeg>, Vec<String>) {
    let legs = vec![
        StraightLeg {
            start_ft: Vec2::new(0.0, 0.0),
            end_ft: Vec2::new(1000.0, 0.0),
        },
        StraightLeg {
            start_ft: Vec2::new(1000.0, 0.0),
            end_ft: Vec2::new(1000.0, 1000.0),
        },
        StraightLeg {
            start_ft: Vec2::new(1000.0, 1000.0),
            end_ft: Vec2::new(1000.0, 2000.0),
        },
    ];
    let names = vec!["A".into(), "D".into(), "D".into()];
    (legs, names)
}

#[test]
fn taxi_profile_emits_taxi_follow_and_taxi_modes() {
    let (legs, names) = three_leg_ninety_deg();
    let mut p = TaxiProfile::new(legs, names);
    let state = taxi_state(Vec2::new(10.0, 0.0), 90.0, 5.0);
    let tick = p.contribute(&state, 0.2);
    assert_eq!(tick.contribution.lateral_mode, Some(LateralMode::TaxiFollow));
    assert_eq!(tick.contribution.vertical_mode, Some(VerticalMode::Taxi));
    let leg = tick.contribution.target_path.expect("target_path");
    assert_eq!(leg.end_ft, Vec2::new(1000.0, 0.0));
}

#[test]
fn taxi_profile_advances_leg_on_proximity() {
    let (legs, names) = three_leg_ninety_deg();
    let mut p = TaxiProfile::new(legs, names);
    // Close to the end of leg 0 (within 30 ft advance distance).
    let state = taxi_state(Vec2::new(980.0, 0.0), 90.0, 5.0);
    let tick = p.contribute(&state, 0.2);
    let leg = tick.contribution.target_path.expect("target_path");
    assert_eq!(
        leg.start_ft,
        Vec2::new(1000.0, 0.0),
        "did not advance to leg 1; target start={:?}",
        leg.start_ft
    );
    assert_eq!(p.current_idx, 1);
}

#[test]
fn taxi_profile_slows_when_approaching_sharp_turn() {
    let (legs, names) = three_leg_ninety_deg();
    let mut p = TaxiProfile::new(legs, names);
    // Well before the turn — should cruise.
    let cruise = taxi_state(Vec2::new(200.0, 0.0), 90.0, 15.0);
    let t1 = p.contribute(&cruise, 0.2);
    assert_eq!(t1.contribution.target_speed_kt, Some(10.0));
    // Close to the turn (within 180 ft lookahead) — should command turn speed.
    let approach = taxi_state(Vec2::new(900.0, 0.0), 90.0, 12.0);
    let t2 = p.contribute(&approach, 0.2);
    assert_eq!(t2.contribution.target_speed_kt, Some(5.0));
}

#[test]
fn taxi_profile_creeps_when_heading_far_off_leg() {
    let (legs, names) = three_leg_ninety_deg();
    let mut p = TaxiProfile::new(legs, names);
    // Leg 0 points due east (heading 90°). Aircraft pointed due north (0°)
    // — that's 90° off, so the alignment limiter clamps target speed to
    // the creep floor (4 kt — enough forward motion to beat nose-wheel
    // tire scrub on a live X-Plane C172).
    let pivot = taxi_state(Vec2::new(0.0, 0.0), 0.0, 1.0);
    let tick = p.contribute(&pivot, 0.2);
    let target = tick.contribution.target_speed_kt.unwrap();
    assert!(
        (target - 4.0).abs() < 1e-9,
        "expected 4 kt creep, got {}",
        target
    );
    // A moderate heading offset (~25°) drops us to turn speed, not creep.
    let moderate = taxi_state(Vec2::new(10.0, 0.0), 65.0, 4.0);
    let tick = p.contribute(&moderate, 0.2);
    assert_eq!(tick.contribution.target_speed_kt, Some(5.0));
    // Aligned to the leg heading — cruise.
    let aligned = taxi_state(Vec2::new(10.0, 0.0), 90.0, 10.0);
    let tick = p.contribute(&aligned, 0.2);
    assert_eq!(tick.contribution.target_speed_kt, Some(10.0));
}

#[test]
fn taxi_profile_enters_pose_phase_after_last_leg_when_pose_is_set() {
    use xplane_pilot::types::TaxiPose;
    let (legs, names) = three_leg_ninety_deg();
    let pose = TaxiPose {
        position_ft: legs.last().unwrap().end_ft,
        heading_deg: 90.0, // face east; aircraft will pivot to match
    };
    let mut p = TaxiProfile::new(legs, names).with_final_pose(pose);
    p.current_idx = 3; // jump past all legs to exercise pose phase

    // Far from target, heading mismatched → approach phase.
    let approach_state = taxi_state(Vec2::new(1000.0, 1970.0), 0.0, 3.0);
    let tick = p.contribute(&approach_state, 0.2);
    assert_eq!(
        tick.contribution.lateral_mode,
        Some(LateralMode::TaxiPose),
        "expected TaxiPose mode"
    );
    assert!(tick.contribution.target_waypoint.is_some());
    assert_eq!(tick.contribution.target_heading_deg, Some(90.0));
    let spd = tick.contribution.target_speed_kt.unwrap();
    assert!(spd > p.pose_creep_speed_kt, "approach should command > creep: {}", spd);

    // Near target, heading still off → align phase: target speed = 0
    // (differential braking pivots the aircraft in place; forward
    // motion would drag it off the target pose).
    let align_state = taxi_state(Vec2::new(1000.0, 1995.0), 0.0, 2.0);
    let tick = p.contribute(&align_state, 0.2);
    assert_eq!(tick.contribution.target_speed_kt, Some(0.0));
    assert!(!p.finished);

    // Within both position and heading tolerance → done.
    let done_state = taxi_state(Vec2::new(1002.0, 1998.0), 90.0, 0.5);
    let tick = p.contribute(&done_state, 0.2);
    assert!(p.finished);
    assert_eq!(tick.contribution.target_speed_kt, Some(0.0));
}

#[test]
fn taxi_profile_ramps_speed_to_zero_on_final_leg_and_finishes() {
    let (legs, names) = three_leg_ninety_deg();
    let mut p = TaxiProfile::new(legs, names);
    p.current_idx = 2;
    // 30 ft from the final end: ramps to 0.25 × cruise = 3.75 kt.
    let near = taxi_state(Vec2::new(1000.0, 1970.0), 0.0, 5.0);
    let tick = p.contribute(&near, 0.2);
    let v = tick.contribution.target_speed_kt.unwrap();
    assert!(v > 0.0 && v < 15.0, "speed was {}", v);
    // At the end: profile flips to finished and targets 0 kt.
    let at_end = taxi_state(Vec2::new(1000.0, 2005.0), 0.0, 2.0);
    let tick = p.contribute(&at_end, 0.2);
    assert!(p.finished);
    assert_eq!(tick.contribution.target_speed_kt, Some(0.0));
}
