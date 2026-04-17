//! Integration tests for taxi behavior using `TaxiDynamicsModel` as the
//! physics model. The loop here is:
//!
//!   DynamicsState → PilotCore::update → ActuatorCommands → TaxiDynamics::step → DynamicsState
//!
//! which is the same shape as `ScenarioRunner` except the physics is the
//! separate taxi model (no runway-course attractor, real split-brake,
//! speed-scheduled nose-wheel).

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::core::profiles::TaxiProfile;
use xplane_pilot::sim::taxi_dynamics::TaxiDynamicsModel;
use xplane_pilot::types::{StraightLeg, TaxiPose, Vec2};

fn run_until_stopped(
    pilot: &mut PilotCore,
    dyn_model: &TaxiDynamicsModel,
    state: &mut xplane_pilot::sim::simple_dynamics::DynamicsState,
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
    let dyn_model = TaxiDynamicsModel::new(cfg);

    // Single east-bound leg from origin to (500, 0). Heading 90° = east.
    let legs = vec![StraightLeg {
        start_ft: Vec2::new(0.0, 0.0),
        end_ft: Vec2::new(500.0, 0.0),
    }];
    let names = vec!["A".into()];
    pilot.engage_profile(Box::new(TaxiProfile::new(legs, names)));

    let mut state = dyn_model.initial_state(Vec2::new(0.0, 0.0), 90.0);
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
    let dyn_model = TaxiDynamicsModel::new(cfg);

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
    pilot.engage_profile(Box::new(profile));

    let mut state = dyn_model.initial_state(Vec2::new(0.0, 0.0), 90.0);
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

    // The pose phase is allowed 10 ft of position tolerance and 5° of
    // heading tolerance. We allow a little slop on top of that for the
    // coast-to-stop after `finished` latches.
    let pos_err = (state.position_ft - hold_short).length();
    assert!(pos_err < 30.0, "position error {:.1} ft", pos_err);
    let hdg_err = (state.heading_deg - 0.0).abs().min((state.heading_deg - 360.0).abs());
    assert!(hdg_err < 15.0, "heading error {:.1}°", hdg_err);
}

#[test]
fn pullout_from_off_node_start_curves_onto_leg() {
    // Aircraft starts 100 ft south of a taxiway that heads east,
    // facing east already but laterally offset. The controller should
    // curve it onto the leg and drive to the end.
    let cfg = load_default_config_bundle();
    let mut pilot = PilotCore::new(cfg.clone());
    let dyn_model = TaxiDynamicsModel::new(cfg);

    let legs = vec![StraightLeg {
        start_ft: Vec2::new(0.0, 0.0),
        end_ft: Vec2::new(600.0, 0.0),
    }];
    let names = vec!["A".into()];
    pilot.engage_profile(Box::new(TaxiProfile::new(legs, names)));

    let start = Vec2::new(0.0, -100.0);
    let mut state = dyn_model.initial_state(start, 90.0);
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
