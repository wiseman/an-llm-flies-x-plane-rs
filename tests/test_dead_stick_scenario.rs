//! End-to-end deterministic dead-stick scenarios on the simple backend.
//!
//! Wires up `SimpleAircraftModel` + `PilotCore` + `DeadStickLandingProfile` at
//! a chosen airborne starting state, then drives the same update/step loop the
//! `ScenarioRunner` uses, watching for touchdown and recording throttle commands.
//! Asserts *control behavior* — phase progression, throttle pinned at zero,
//! touching down on the runway — not glide physics.
//!
//! Scope: only the "engaged on short final" case is exercised here. The
//! simple-dynamics drag model (linear in IAS, tuned for powered flight)
//! decays airspeed to zero within ~30 seconds of throttle-off flight, so
//! sustained gliding through a full pattern (Downwind → Base → Final) is
//! not faithfully modeled in this backend. Those scenarios are validated
//! live in X-Plane.

use xplane_pilot::config::{load_default_config_bundle, ConfigBundle};
use xplane_pilot::core::dead_stick_profile::DeadStickLandingProfile;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::guidance::pattern_manager::build_pattern_geometry;
use xplane_pilot::guidance::runway_geometry::RunwayFrame;
use xplane_pilot::sim::simple_dynamics::{DynamicsState, SimpleAircraftModel};
use xplane_pilot::types::{heading_to_vector, FlightPhase, Vec2, KT_TO_FPS};

#[derive(Debug)]
struct DeadStickResult {
    final_phase: FlightPhase,
    duration_s: f64,
    phases_seen: Vec<FlightPhase>,
    touchdown_runway_x_ft: Option<f64>,
    touchdown_centerline_ft: Option<f64>,
    /// True iff every actuator throttle command across the run was ≤ 0.001.
    /// Catches a profile bug that lets throttle leak above zero.
    throttle_stayed_zero: bool,
    max_throttle_cmd: f64,
}

/// Spawn an airborne aircraft at the given runway-frame position, glide it
/// to a stop with the dead-stick profile engaged, and return summary
/// telemetry. `starting_speed_kt` is the IAS at engage time; `pitch_deg`
/// is the trim pitch (TECS will adjust quickly).
fn run_dead_stick_scenario(
    config: ConfigBundle,
    runway_frame_pos_ft: Vec2,
    altitude_agl_ft: f64,
    heading_deg: f64,
    starting_speed_kt: f64,
    pitch_deg: f64,
    max_time_s: f64,
) -> DeadStickResult {
    let model = SimpleAircraftModel::new(config.clone(), Vec2::ZERO);
    let runway_frame = RunwayFrame::new(config.airport.runway.clone());
    let mut pilot = PilotCore::new(config.clone());

    // World-frame starting position from the runway-frame point the
    // caller supplied.
    let world_pos = runway_frame.to_world_frame(runway_frame_pos_ft);

    // Seed ground_velocity from heading × IAS so the state estimator
    // computes a non-zero gs / track on the very first tick. Without
    // this, mode_manager's adaptive base-turn relief (which scales with
    // gs shortfall vs downwind_speed_kt) thinks the aircraft is barely
    // moving and fires a base turn at engage time.
    let initial_ground_velocity =
        heading_to_vector(heading_deg, starting_speed_kt * KT_TO_FPS);
    let mut raw_state = DynamicsState {
        position_ft: world_pos,
        altitude_ft: config.airport.field_elevation_ft + altitude_agl_ft,
        heading_deg,
        roll_deg: 0.0,
        pitch_deg,
        ias_kt: starting_speed_kt,
        throttle_pos: 0.0,
        on_ground: false,
        time_s: 0.0,
        p_rad_s: 0.0,
        q_rad_s: 0.0,
        r_rad_s: 0.0,
        ground_velocity_ft_s: initial_ground_velocity,
        vertical_speed_ft_s: 0.0,
        flap_index: 0,
        gear_down: true,
    };

    // First update: estimate state so the dead-stick profile sees real
    // runway-frame coords on its decide_entry_phase call.
    let dt = 0.2;
    let (estimated, _commands) = pilot.update(&raw_state, dt);

    // Engage the profile against the freshly-estimated state — that's
    // what live_runner does too (the bridge state runs through the
    // estimator before any tool reads it).
    let profile = DeadStickLandingProfile::new(config.clone(), runway_frame.clone(), &estimated);
    pilot.engage_profile(Box::new(profile));

    let mut phases_seen: Vec<FlightPhase> = Vec::new();
    let mut touchdown_runway_x_ft: Option<f64> = None;
    let mut touchdown_centerline_ft: Option<f64> = None;
    let mut prev_airborne = !raw_state.on_ground;
    let mut throttle_stayed_zero = true;
    let mut max_throttle_cmd = 0.0_f64;

    while raw_state.time_s <= max_time_s {
        let (_estimated, commands) = pilot.update(&raw_state, dt);
        let phase = pilot.phase();
        if phases_seen.last() != Some(&phase) {
            phases_seen.push(phase);
        }
        max_throttle_cmd = max_throttle_cmd.max(commands.throttle);
        if commands.throttle > 0.001 {
            throttle_stayed_zero = false;
        }

        model.step(&mut raw_state, &commands, dt);

        if prev_airborne && raw_state.on_ground && touchdown_runway_x_ft.is_none() {
            let td = runway_frame.to_runway_frame(raw_state.position_ft);
            touchdown_runway_x_ft = Some(td.x);
            touchdown_centerline_ft = Some(td.y);
        }
        prev_airborne = !raw_state.on_ground;

        if phase == FlightPhase::TaxiClear {
            break;
        }
        if matches!(
            phase,
            FlightPhase::Rollout | FlightPhase::RunwayExit | FlightPhase::TaxiClear
        ) && raw_state.ground_velocity_ft_s.length() < 1.0
        {
            break;
        }
    }

    DeadStickResult {
        final_phase: pilot.phase(),
        duration_s: raw_state.time_s,
        phases_seen,
        touchdown_runway_x_ft,
        touchdown_centerline_ft,
        throttle_stayed_zero,
        max_throttle_cmd,
    }
}

#[test]
fn dead_stick_engaged_on_final_lands() {
    let config = load_default_config_bundle();
    let perf = config.performance.clone();
    let runway_length = config.airport.runway.length_ft;

    // 3000 ft short of threshold, on extended centerline, 600 AGL,
    // heading = runway course, vbg-ish IAS.
    let result = run_dead_stick_scenario(
        config,
        Vec2::new(-3000.0, 0.0),
        600.0,
        0.0, // runway course in default fixture is 360° == 0°
        perf.vbg_kt,
        2.0,
        300.0,
    );

    assert!(
        matches!(
            result.final_phase,
            FlightPhase::Rollout | FlightPhase::RunwayExit | FlightPhase::TaxiClear
        ),
        "expected to finish on the ground; got {:?} after {:.0}s, phases={:?}",
        result.final_phase,
        result.duration_s,
        result.phases_seen
    );
    let td_x = result
        .touchdown_runway_x_ft
        .expect("expected touchdown to be recorded");
    assert!(
        (0.0..=runway_length).contains(&td_x),
        "touchdown x = {:.0} outside runway [0, {:.0}]; phases={:?}",
        td_x,
        runway_length,
        result.phases_seen
    );
    let td_y = result.touchdown_centerline_ft.unwrap_or(9999.0);
    assert!(
        td_y.abs() < 75.0,
        "touchdown {:.0} ft off centerline (allowed ±75 ft); phases={:?}",
        td_y,
        result.phases_seen
    );
    assert!(
        result.throttle_stayed_zero,
        "throttle leaked above zero (max {:.3}); profile must keep throttle pinned",
        result.max_throttle_cmd
    );
}

#[test]
fn dead_stick_engaged_on_downwind_progresses_through_pattern() {
    let config = load_default_config_bundle();
    let pattern = build_pattern_geometry(
        &RunwayFrame::new(config.airport.runway.clone()),
        config.pattern.downwind_offset_ft,
        0.0,
        0.0,
    );
    let perf = config.performance.clone();
    let pattern_alt_agl = config.pattern.altitude_agl_ft;
    let runway_length = config.airport.runway.length_ft;

    // Mid-downwind on the pattern side. With the default left-traffic
    // 16/36 fixture, downwind_y_ft is negative; mid_x is between the
    // join point (positive) and the base-turn x (negative) — start a
    // bit toward the join so we see Downwind → Base → Final.
    let mid_x = (pattern.join_point_runway_ft.x + pattern.base_turn_x_ft) * 0.5;
    let recip = (config.airport.runway.course_deg + 180.0).rem_euclid(360.0);

    let result = run_dead_stick_scenario(
        config,
        Vec2::new(mid_x, pattern.downwind_y_ft),
        pattern_alt_agl,
        recip,
        perf.vbg_kt,
        0.0,
        600.0,
    );

    assert!(
        matches!(
            result.final_phase,
            FlightPhase::Rollout | FlightPhase::RunwayExit | FlightPhase::TaxiClear
        ),
        "expected ground state; got {:?} after {:.0}s, phases={:?}",
        result.final_phase,
        result.duration_s,
        result.phases_seen
    );
    // Phase progression: must have visited Base + Final somewhere along
    // the way (proves we didn't skip ahead from Downwind to landing).
    for required in [FlightPhase::Base, FlightPhase::Final] {
        assert!(
            result.phases_seen.contains(&required),
            "expected to see {:?} in phases_seen={:?}",
            required,
            result.phases_seen
        );
    }
    let td_x = result
        .touchdown_runway_x_ft
        .expect("expected touchdown to be recorded");
    assert!(
        (0.0..=runway_length).contains(&td_x),
        "touchdown x = {:.0} outside runway [0, {:.0}]; phases={:?}",
        td_x,
        runway_length,
        result.phases_seen
    );
    assert!(
        result.throttle_stayed_zero,
        "throttle leaked above zero (max {:.3}); profile must keep throttle pinned",
        result.max_throttle_cmd
    );
}

#[test]
fn dead_stick_engaged_at_altitude_glides_through_descent_to_landing() {
    let config = load_default_config_bundle();
    let pattern = build_pattern_geometry(
        &RunwayFrame::new(config.airport.runway.clone()),
        config.pattern.downwind_offset_ft,
        0.0,
        0.0,
    );
    let perf = config.performance.clone();
    let runway_length = config.airport.runway.length_ft;

    // Spawn 3 NM (~18,000 ft) "behind" the runway on the pattern side at
    // 5000 ft AGL, heading roughly toward the Low Key fix. The simple-
    // dynamics glide ratio is generous enough that the aircraft can
    // make the field from this position even with the meandering
    // descent → low-key → pattern routing.
    let runway_frame = RunwayFrame::new(config.airport.runway.clone());
    let low_key_world = runway_frame.to_world_frame(Vec2::new(
        runway_frame.touchdown_runway_x_ft(),
        pattern.downwind_y_ft,
    ));
    let start_world = runway_frame.to_world_frame(Vec2::new(
        pattern.join_point_runway_ft.x + 18_000.0,
        pattern.downwind_y_ft,
    ));
    let toward_low_key = low_key_world - start_world;
    let heading_deg = (toward_low_key.x.atan2(toward_low_key.y)).to_degrees();
    let heading_deg = ((heading_deg + 360.0) % 360.0) as f64;
    // Convert back to runway-frame coords for the harness signature.
    let start_runway = runway_frame.to_runway_frame(start_world);

    let result = run_dead_stick_scenario(
        config,
        start_runway,
        5000.0,
        heading_deg,
        perf.vbg_kt,
        0.0,
        1500.0,
    );

    // Must have visited Descent (the start phase) and Final (the end
    // phase). Don't pin Downwind/Base because the entry geometry can
    // pick up at PatternEntry/Downwind directly when the aircraft is
    // already aligned at low-key altitude.
    for required in [FlightPhase::Descent, FlightPhase::Final] {
        assert!(
            result.phases_seen.contains(&required),
            "expected {:?} in phases_seen={:?}",
            required,
            result.phases_seen
        );
    }
    assert!(
        matches!(
            result.final_phase,
            FlightPhase::Rollout | FlightPhase::RunwayExit | FlightPhase::TaxiClear
        ),
        "expected to finish on the ground; got {:?} after {:.0}s, phases={:?}",
        result.final_phase,
        result.duration_s,
        result.phases_seen
    );
    let td_x = result
        .touchdown_runway_x_ft
        .expect("expected touchdown to be recorded");
    assert!(
        (0.0..=runway_length).contains(&td_x),
        "touchdown x = {:.0} outside runway [0, {:.0}]; phases={:?}",
        td_x,
        runway_length,
        result.phases_seen
    );
    assert!(
        result.throttle_stayed_zero,
        "throttle leaked above zero (max {:.3}); profile must keep throttle pinned",
        result.max_throttle_cmd
    );
}
