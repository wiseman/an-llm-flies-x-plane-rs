use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::control::tecs_lite::TECSLite;
use xplane_pilot::types::FlightPhase;

#[test]
fn below_target_altitude_and_slow_commands_more_pitch_and_power() {
    let mut tecs = TECSLite::new(load_default_config_bundle().controllers.tecs);
    let (pitch, throttle) = tecs.update(
        FlightPhase::EnrouteClimb,
        3000.0,
        74.0,
        2200.0,
        400.0,
        66.0,
        0.2,
        (0.75, 1.0),
    );
    assert!(pitch > 6.0, "pitch was {}", pitch);
    assert!(throttle > 0.85, "throttle was {}", throttle);
}

#[test]
fn above_target_and_fast_commands_descent_and_power_reduction() {
    let mut tecs = TECSLite::new(load_default_config_bundle().controllers.tecs);
    let (pitch, throttle) = tecs.update(
        FlightPhase::Descent,
        1500.0,
        85.0,
        2500.0,
        -300.0,
        100.0,
        0.2,
        (0.1, 0.6),
    );
    assert!(pitch < 0.0, "pitch was {}", pitch);
    assert!(throttle < 0.35, "throttle was {}", throttle);
}
