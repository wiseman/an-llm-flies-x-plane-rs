//! Integration tests for taxi behavior on the simple simulator's ground
//! model (kinematic nose wheel, split brakes, rolling and static
//! friction). The loop here is:
//!
//!   DynamicsState → PilotCore::update → ActuatorCommands → SimpleAircraftModel::step → DynamicsState
//!
//! which is the same shape as `ScenarioRunner`.

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::core::profiles::TaxiProfile;
use xplane_pilot::sim::simple_dynamics::{DynamicsState, SimpleAircraftModel};
use xplane_pilot::types::{StraightLeg, TaxiPose, Vec2};

fn run_until_stopped(
    pilot: &mut PilotCore,
    dyn_model: &SimpleAircraftModel,
    state: &mut DynamicsState,
    max_steps: usize,
) -> usize {
    let dt = 0.2;
    let mut stopped_ticks = 0;
    let trace = std::env::var("TAXI_TRACE").is_ok();
    for step in 0..max_steps {
        let (_, cmds) = pilot.update(state, dt);
        dyn_model.step(state, &cmds, dt);
        if trace && step % 25 == 0 {
            let snap = pilot.latest_snapshot.as_ref();
            let dbg = snap
                .map(|s| s.debug_lines.join("; "))
                .unwrap_or_else(|| "(no snapshot)".into());
            eprintln!(
                "step {:4}  pos=({:7.1},{:7.1})  hdg={:6.1}  gs={:4.1}  throttle={:.2}  brake={:.2}  pivot={:+.2}  rudder={:+.2}  {}",
                step,
                state.position_ft.x,
                state.position_ft.y,
                state.heading_deg,
                state.ias_kt,
                cmds.throttle,
                cmds.brakes,
                cmds.pivot_brake,
                cmds.rudder,
                dbg,
            );
        }
        // "Settled" = essentially stopped AND no residual yaw rate for a
        // couple of seconds running. Without the yaw-rate check the loop
        // declares victory while the aircraft is still pivoting in place
        // on differential braking.
        if state.ias_kt < 0.05 && state.r_rad_s.to_degrees().abs() < 0.5 {
            stopped_ticks += 1;
            if stopped_ticks >= 10 {
                return step + 1;
            }
        } else {
            stopped_ticks = 0;
        }
    }
    max_steps
}

#[test]
fn straight_leg_taxi_arrives_near_end_aligned() {
    let cfg = load_default_config_bundle();
    let mut pilot = PilotCore::new(cfg.clone());
    let dyn_model = SimpleAircraftModel::new(cfg, Vec2::ZERO);

    // Single east-bound leg from origin to (500, 0). Heading 90° = east.
    let legs = vec![StraightLeg {
        start_ft: Vec2::new(0.0, 0.0),
        end_ft: Vec2::new(500.0, 0.0),
    }];
    let names = vec!["A".into()];
    pilot.engage_profile(Box::new(TaxiProfile::new(legs, names))).unwrap();

    let mut state = dyn_model.initial_state_at(Vec2::new(0.0, 0.0), 90.0, dyn_model.config.airport.field_elevation_ft);
    // 90 s should be more than enough for 500 ft of taxi with the
    // 10 kt cruise target and a trailing ramp-down.
    let steps = run_until_stopped(&mut pilot, &dyn_model, &mut state, 450);
    assert!(steps < 450, "taxi did not settle within 90 s");

    // Final position within 40 ft of the intended end, still roughly
    // aligned east (heading ≈ 90°).
    let d = (state.position_ft - Vec2::new(500.0, 0.0)).length();
    assert!(
        d < 40.0,
        "ended {:.1} ft off from target end: pos={:?}",
        d,
        state.position_ft
    );
    let hdg_err = (state.heading_deg - 90.0).abs();
    assert!(
        hdg_err < 10.0,
        "heading drifted: {:.1}° (expected ~90°)",
        state.heading_deg
    );
}

#[test]
fn pose_target_parks_at_position_and_heading() {
    // Two-leg path heading east, then a pose target that requires a
    // ~90° left pivot to face north.
    let cfg = load_default_config_bundle();
    let mut pilot = PilotCore::new(cfg.clone());
    let dyn_model = SimpleAircraftModel::new(cfg, Vec2::ZERO);

    let hold_short = Vec2::new(400.0, 0.0);
    let legs = vec![StraightLeg {
        start_ft: Vec2::new(0.0, 0.0),
        end_ft: hold_short,
    }];
    let names = vec!["A".into()];
    let profile = TaxiProfile::new(legs, names).with_final_pose(TaxiPose {
        position_ft: hold_short,
        heading_deg: 0.0, // face north
    });
    pilot.engage_profile(Box::new(profile)).unwrap();

    let mut state = dyn_model.initial_state_at(Vec2::new(0.0, 0.0), 90.0, dyn_model.config.airport.field_elevation_ft);
    let steps = run_until_stopped(&mut pilot, &dyn_model, &mut state, 900);
    if steps >= 900 {
        let snap = pilot.latest_snapshot.clone();
        if let Some(s) = snap {
            for line in &s.debug_lines {
                eprintln!("debug: {}", line);
            }
            eprintln!(
                "state: pos=({:.1},{:.1}) heading={:.1} gs={:.2} throttle={:.2} brake={:.2} pivot={:.2}",
                state.position_ft.x,
                state.position_ft.y,
                state.heading_deg,
                state.ias_kt,
                state.throttle_pos,
                s.last_commands.brakes,
                s.last_commands.pivot_brake,
            );
        }
    }
    assert!(steps < 900, "pose-target did not settle");

    // A sideways arrival re-approaches along the heading, then settles
    // at the spot within the 8° heading tolerance.
    let pos_err = (state.position_ft - hold_short).length();
    let hdg_err = (state.heading_deg - 0.0).abs().min((state.heading_deg - 360.0).abs());
    eprintln!("90° pose: pos_err {pos_err:.1} ft, hdg_err {hdg_err:.1}°, steps {steps}");
    assert!(pos_err < 15.0, "position error {:.1} ft", pos_err);
    assert!(hdg_err < 10.0, "heading error {:.1}°", hdg_err);
}

#[test]
fn parking_lead_in_off_spot_heading_still_aligns_and_sets_brake() {
    // The live KWHP case: the lead-in leg from the last taxi node meets
    // the spot ~18° off its painted heading.
    let cfg = load_default_config_bundle();
    let mut pilot = PilotCore::new(cfg.clone());
    let dyn_model = SimpleAircraftModel::new(cfg, Vec2::ZERO);
    let spot = Vec2::new(300.0, 0.0);
    let spot_heading = 72.0; // lead-in runs due east (90°)
    let pose = TaxiPose { position_ft: spot, heading_deg: spot_heading };
    let lead_in = TaxiProfile::pose_lead_in_point(
        Vec2::ZERO,
        pose,
        dyn_model.config.ground_handling.min_turn_radius_ft(),
    )
    .expect("18° off needs a lead-in");
    let profile = TaxiProfile::new(
        vec![StraightLeg { start_ft: Vec2::ZERO, end_ft: lead_in }],
        vec!["(to spot)".into()],
    )
    .with_final_pose(pose)
    .parking_at("Spot 1");
    pilot.engage_profile(Box::new(profile)).unwrap();

    let mut state = dyn_model.initial_state_at(
        Vec2::new(0.0, 0.0),
        90.0,
        dyn_model.config.airport.field_elevation_ft,
    );
    let steps = run_until_stopped(&mut pilot, &dyn_model, &mut state, 900);
    assert!(steps < 900, "did not settle");
    let pos_err = (state.position_ft - spot).length();
    let hdg_err = xplane_pilot::types::wrap_degrees_180(state.heading_deg - spot_heading).abs();
    eprintln!("18° lead-in: pos_err {pos_err:.1} ft, hdg_err {hdg_err:.1}°, steps {steps}");
    assert!(pos_err < 15.0, "position error {:.1} ft", pos_err);
    assert!(hdg_err < 10.0, "heading error {:.1}°", hdg_err);
    assert_eq!(pilot.parking_brake.value(), Some(1.0), "parking brake not set on arrival");
    assert_eq!(state.parking_brake, 1.0, "parking brake command didn't reach the sim");
    let snap = pilot.latest_snapshot.as_ref().unwrap();
    // X-Plane releases the parking brake while toe brakes are applied.
    assert_eq!(snap.last_commands.brakes, 0.0, "toe brakes must be off once parked");
    assert_eq!(snap.last_commands.throttle, 0.0);
    assert_eq!(pilot.list_profile_names(), vec!["taxi"]);
}

#[test]
fn pullout_from_off_node_start_curves_onto_leg() {
    // Aircraft starts 100 ft south of a taxiway that heads east,
    // facing east already but laterally offset. The controller should
    // curve it onto the leg and drive to the end.
    let cfg = load_default_config_bundle();
    let mut pilot = PilotCore::new(cfg.clone());
    let dyn_model = SimpleAircraftModel::new(cfg, Vec2::ZERO);

    let legs = vec![StraightLeg {
        start_ft: Vec2::new(0.0, 0.0),
        end_ft: Vec2::new(600.0, 0.0),
    }];
    let names = vec!["A".into()];
    pilot.engage_profile(Box::new(TaxiProfile::new(legs, names))).unwrap();

    let start = Vec2::new(0.0, -100.0);
    let mut state = dyn_model.initial_state_at(start, 90.0, dyn_model.config.airport.field_elevation_ft);
    let steps = run_until_stopped(&mut pilot, &dyn_model, &mut state, 700);
    assert!(steps < 700, "taxi did not settle from off-axis start");

    // Should have worked its way onto the leg line (y ≈ 0) and ended
    // somewhere near the leg end in x.
    assert!(
        state.position_ft.y.abs() < 30.0,
        "still off the line: y={:.1}",
        state.position_ft.y
    );
    let x_err = (state.position_ft.x - 600.0).abs();
    assert!(x_err < 50.0, "ended {:.1} ft off the end in x", x_err);
}

#[test]
fn parking_converges_from_any_arrival_angle() {
    // Route-builder geometry (a leg to the lead-in point) for spot
    // headings from straight ahead to fully reversed, both sides. Each
    // must park on the spot without the old relax-after-circling
    // fallback; a bad arrival re-approaches along the heading instead.
    let cfg = load_default_config_bundle();
    let radius = cfg.ground_handling.min_turn_radius_ft();
    for turn in [0.0, 45.0, 90.0, 135.0, 180.0, -45.0, -90.0, -135.0] {
        let mut pilot = PilotCore::new(cfg.clone());
        let dyn_model = SimpleAircraftModel::new(cfg.clone(), Vec2::ZERO);
        let spot = Vec2::new(400.0, 0.0);
        let pose = TaxiPose { position_ft: spot, heading_deg: (90.0 + turn + 360.0) % 360.0 };
        let end = TaxiProfile::pose_lead_in_point(Vec2::ZERO, pose, radius).unwrap_or(spot);
        let profile = TaxiProfile::new(
            vec![StraightLeg { start_ft: Vec2::ZERO, end_ft: end }],
            vec!["A".into()],
        )
        .with_final_pose(pose)
        .parking_at("Spot");
        pilot.engage_profile(Box::new(profile)).unwrap();
        let mut state =
            dyn_model.initial_state_at(Vec2::ZERO, 90.0, dyn_model.config.airport.field_elevation_ft);
        let steps = run_until_stopped(&mut pilot, &dyn_model, &mut state, 2000);
        let pos_err = (state.position_ft - spot).length();
        let hdg_err =
            xplane_pilot::types::wrap_degrees_180(state.heading_deg - pose.heading_deg).abs();
        eprintln!("turn {turn:+4.0}°: pos_err {pos_err:.1} ft, hdg_err {hdg_err:.1}°, steps {steps}");
        assert!(steps < 2000, "turn {turn}: did not settle");
        assert!(pos_err < 6.0, "turn {turn}: position error {pos_err:.1} ft");
        assert!(hdg_err < 8.0, "turn {turn}: heading error {hdg_err:.1}°");
        assert_eq!(pilot.parking_brake.value(), Some(1.0), "turn {turn}: brake not set");
    }
}
