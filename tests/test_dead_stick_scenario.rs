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
use xplane_pilot::core::dead_stick_profile::{
    DeadStickLandingProfile, DEAD_STICK_PATTERN_OFFSET_FT,
};
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
    /// Maximum IAS recorded during the pre-Final portion of the run
    /// (Descent through Base). The dead-stick profile must hold close to
    /// vbg through these phases — this catches a regression where TECS
    /// or some other controller pushes the aircraft well above vbg and
    /// burns the only energy we have.
    max_pre_final_ias_kt: f64,
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
    let mut max_pre_final_ias_kt = 0.0_f64;

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
        if matches!(
            phase,
            FlightPhase::Descent
                | FlightPhase::PatternEntry
                | FlightPhase::Downwind
                | FlightPhase::Base
        ) {
            max_pre_final_ias_kt = max_pre_final_ias_kt.max(raw_state.ias_kt);
        }
        if std::env::var("DEAD_STICK_TRACE").is_ok() && (raw_state.time_s as i32) % 5 == 0 {
            let rf = pilot.runway_frame.to_runway_frame(raw_state.position_ft);
            eprintln!(
                "t={:5.1} ph={:13?} rfx={:6.0} rfy={:6.0} agl={:5.0} ias={:4.1} hdg={:5.1} bank={:5.1}",
                raw_state.time_s,
                phase,
                rf.x, rf.y,
                raw_state.altitude_ft - pilot.config.airport.field_elevation_ft,
                raw_state.ias_kt,
                raw_state.heading_deg,
                raw_state.roll_deg,
            );
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
        max_pre_final_ias_kt,
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
    // The on-final scenario starts at 600 ft AGL, well below short
    // final, so the speed target is already vref — no pre-final IAS
    // budget assertion applies here.
}

#[test]
fn dead_stick_engaged_on_downwind_progresses_through_pattern() {
    let config = load_default_config_bundle();
    // Use the dead-stick controller's pattern offset, not the config's
    // standard powered-pattern offset — the dead-stick profile builds
    // its geometry against the tighter dead-stick value, so spawning
    // the aircraft on the config-sized downwind would put it on a
    // leg the controller doesn't see.
    let pattern = build_pattern_geometry(
        &RunwayFrame::new(config.airport.runway.clone()),
        DEAD_STICK_PATTERN_OFFSET_FT,
        0.0,
        0.0,
    );
    let perf = config.performance.clone();
    let runway_length = config.airport.runway.length_ft;
    let vbg_kt = perf.vbg_kt;

    // Mid-downwind on the pattern side. With the default left-traffic
    // 16/36 fixture, downwind_y_ft is negative; mid_x is between the
    // join point (positive) and the base-turn x (negative) — start a
    // bit toward the join so we see Downwind → Base → Final.
    let mid_x = (pattern.join_point_runway_ft.x + pattern.base_turn_x_ft) * 0.5;
    let recip = (config.airport.runway.course_deg + 180.0).rem_euclid(360.0);

    // Start a bit above pattern altitude. A standard 6000-ft-offset
    // pattern is ~18 kft of horizontal travel from mid-downwind to the
    // threshold; with the C172's published 9:1 glide that needs ~2000
    // ft of altitude to make the field. Standard pattern altitude
    // alone (1000 AGL) is intentionally not enough — operationally
    // dead-stick aircraft arrive at the pattern high, not low.
    let result = run_dead_stick_scenario(
        config,
        Vec2::new(mid_x, pattern.downwind_y_ft),
        2200.0,
        recip,
        vbg_kt,
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
    // Dead-stick must hold close to vbg through the pattern — never let
    // the aircraft accelerate well above best glide. A few knots over
    // is fine while the controller settles, but anything north of
    // vbg + 8 kt means the speed loop is broken.
    assert!(
        result.max_pre_final_ias_kt <= vbg_kt + 8.0,
        "pre-final IAS reached {:.1} (vbg {:.1} + 8 kt budget); dead-stick lost speed control",
        result.max_pre_final_ias_kt,
        vbg_kt
    );
}

#[test]
fn dead_stick_engaged_at_altitude_glides_through_descent_to_landing() {
    let config = load_default_config_bundle();
    let pattern = build_pattern_geometry(
        &RunwayFrame::new(config.airport.runway.clone()),
        DEAD_STICK_PATTERN_OFFSET_FT,
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

    let vbg_kt = perf.vbg_kt;
    let result = run_dead_stick_scenario(
        config,
        start_runway,
        5000.0,
        heading_deg,
        vbg_kt,
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
    assert!(
        result.max_pre_final_ias_kt <= vbg_kt + 8.0,
        "pre-final IAS reached {:.1} (vbg {:.1} + 8 kt budget); dead-stick lost speed control",
        result.max_pre_final_ias_kt,
        vbg_kt
    );
}

/// Aircraft starts on the *opposite* side of the runway from the pattern
/// (wrong-side entry). Mirrors the X-Plane test where dead-stick was
/// engaged at KWHP rwy 30 with the aircraft NE of the runway centerline
/// — the side opposite the left-traffic downwind. The dead-stick descent
/// must cross over the centerline, intercept the downwind axis, and
/// fly Downwind → Base → Final to the runway with a reasonable altitude
/// budget.
///
/// Starts ~3000 ft AGL above field, slightly upwind of the threshold,
/// 10,000 ft to the WRONG side of centerline (right of course = right
/// side, while pattern is left). Heading roughly northbound (along
/// runway course). Glide budget is the same `find_dead_stick_candidates`
/// would see in the live tool: AGL × glide_ratio.
#[test]
fn dead_stick_engaged_on_wrong_side_of_runway() {
    // Override the test-fixture runway to a real-world short length —
    // KWHP rwy 30 is 4124 ft. The default 21,000 ft fixture hides
    // float-long bugs because everything fits in the box; using
    // KWHP's length makes the simple-sim test a faithful proxy for
    // the live X-Plane test.
    let mut config = load_default_config_bundle();
    config.airport.runway.length_ft = 4124.0;
    let perf = config.performance.clone();
    let runway_length = config.airport.runway.length_ft;
    let vbg_kt = perf.vbg_kt;

    // Default fixture: course 360°, left traffic. So +y in runway frame
    // is "right of course" — the WRONG side. Pattern is at -1500 (the
    // dead-stick-tightened offset).
    let start_runway = Vec2::new(600.0, 10_000.0);
    let result = run_dead_stick_scenario(
        config,
        start_runway,
        3000.0,
        0.0, // heading north, roughly aligned with course
        vbg_kt,
        0.0,
        900.0,
    );

    println!(
        "wrong-side dead-stick: final_phase={:?} duration={:.0}s phases={:?} td_x={:?} td_y={:?}",
        result.final_phase,
        result.duration_s,
        result.phases_seen,
        result.touchdown_runway_x_ft,
        result.touchdown_centerline_ft,
    );
    assert!(
        matches!(
            result.final_phase,
            FlightPhase::Rollout | FlightPhase::RunwayExit | FlightPhase::TaxiClear
        ),
        "wrong-side dead-stick failed to reach the ground in a landed phase; got {:?} after {:.0}s, phases={:?}",
        result.final_phase,
        result.duration_s,
        result.phases_seen
    );
    let td_x = result
        .touchdown_runway_x_ft
        .expect("expected touchdown to be recorded");
    assert!(
        (0.0..=runway_length).contains(&td_x),
        "wrong-side dead-stick touched down x={:.0} ft, outside runway [0, {:.0}] — landed off airport. phases={:?}",
        td_x,
        runway_length,
        result.phases_seen,
    );
    assert!(
        result.throttle_stayed_zero,
        "throttle leaked above zero (max {:.3})",
        result.max_throttle_cmd
    );
    assert!(
        result.max_pre_final_ias_kt <= vbg_kt + 8.0,
        "pre-final IAS reached {:.1} (vbg {:.1} + 8 kt budget)",
        result.max_pre_final_ias_kt,
        vbg_kt
    );
}
