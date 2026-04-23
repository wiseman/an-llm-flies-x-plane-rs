use xplane_pilot::control::centerline_rollout::CenterlineRolloutController;

fn upd(
    c: &mut CenterlineRolloutController,
    cle: f64,
    track_err: f64,
    yaw_rate: f64,
    gs: f64,
    dt: f64,
) -> f64 {
    c.update(cle, track_err, yaw_rate, gs, dt)
}

#[test]
fn right_of_centerline_commands_left_rudder() {
    let mut c = CenterlineRolloutController::new();
    assert!(upd(&mut c, 50.0, 0.0, 0.0, 80.0, 0.1) < 0.0);
}

#[test]
fn left_of_centerline_commands_right_rudder() {
    let mut c = CenterlineRolloutController::new();
    assert!(upd(&mut c, -50.0, 0.0, 0.0, 80.0, 0.1) > 0.0);
}

#[test]
fn heading_left_of_runway_commands_right_rudder() {
    let mut c = CenterlineRolloutController::new();
    assert!(upd(&mut c, 0.0, 10.0, 0.0, 80.0, 0.1) > 0.0);
}

#[test]
fn heading_right_of_runway_commands_left_rudder() {
    let mut c = CenterlineRolloutController::new();
    assert!(upd(&mut c, 0.0, -10.0, 0.0, 80.0, 0.1) < 0.0);
}

#[test]
fn yaw_rate_damping_opposes_current_yaw() {
    let mut c = CenterlineRolloutController::new();
    assert!(upd(&mut c, 0.0, 0.0, 5.0, 80.0, 0.1) < 0.0);
}

#[test]
fn damping_reduces_rudder_when_already_turning_toward_target() {
    // Aircraft is right of centerline (want left rudder, negative) and yawing left at 3 deg/s
    // Fresh controller to isolate from integrator state
    let mut damped = CenterlineRolloutController::new();
    let damped_out = damped.update(15.0, 0.0, -3.0, 80.0, 0.1);
    let mut undamped = CenterlineRolloutController::new();
    let undamped_out = undamped.update(15.0, 0.0, 0.0, 80.0, 0.1);
    assert!(damped_out > undamped_out);
}

#[test]
fn integrator_accumulates_for_steady_state_bias() {
    let mut c = CenterlineRolloutController::new();
    let first = c.update(0.0, 10.0, 0.0, 80.0, 0.1);
    let mut last = first;
    for _ in 0..19 {
        last = c.update(0.0, 10.0, 0.0, 80.0, 0.1);
    }
    assert!(last > first);
    assert!(last > 0.0);
}

#[test]
fn reset_clears_integrator() {
    let mut c = CenterlineRolloutController::new();
    for _ in 0..20 {
        c.update(0.0, 10.0, 0.0, 80.0, 0.1);
    }
    let before = c.update(0.0, 0.0, 0.0, 80.0, 0.1);
    assert!(before > 0.0);
    c.reset();
    let after = c.update(0.0, 0.0, 0.0, 80.0, 0.1);
    assert!(after.abs() < 1e-5);
}

#[test]
fn moderate_error_does_not_saturate_rudder() {
    let mut c = CenterlineRolloutController::new();
    let rudder = c.update(0.0, 25.0, 0.0, 80.0, 0.1);
    assert!(rudder.abs() < 0.95);
}
