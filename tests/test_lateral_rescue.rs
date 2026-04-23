//! L1 far/near behavior.

mod common;

use xplane_pilot::guidance::lateral::{distance_to_leg_segment, L1PathFollower};
use xplane_pilot::types::{heading_to_vector, AircraftState, StraightLeg, Vec2, KT_TO_FPS};

fn make_state(position_ft: Vec2, track_deg: f64, gs_kt: f64) -> AircraftState {
    AircraftState {
        t_sim: 0.0,
        dt: 0.2,
        position_ft,
        alt_msl_ft: 3000.0,
        alt_agl_ft: 2500.0,
        pitch_deg: 0.0,
        roll_deg: 0.0,
        heading_deg: track_deg,
        track_deg,
        p_rad_s: 0.0,
        q_rad_s: 0.0,
        r_rad_s: 0.0,
        ias_kt: gs_kt,
        tas_kt: gs_kt,
        gs_kt,
        vs_fpm: 0.0,
        ground_velocity_ft_s: heading_to_vector(track_deg, gs_kt * KT_TO_FPS),
        flap_index: 0,
        gear_down: true,
        on_ground: false,
        throttle_pos: 0.6,
        runway_id: None,
        runway_dist_remaining_ft: None,
        runway_x_ft: None,
        runway_y_ft: None,
        centerline_error_ft: None,
        threshold_abeam: false,
        distance_to_touchdown_ft: None,
        stall_margin: 1.8,
    }
}

fn wrap(a: f64, b: f64) -> f64 {
    ((a - b + 180.0).rem_euclid(360.0)) - 180.0
}

#[test]
fn distance_perpendicular_when_projection_inside_segment() {
    let leg = StraightLeg { start_ft: Vec2::ZERO, end_ft: Vec2::new(10000.0, 0.0) };
    let state = make_state(Vec2::new(5000.0, 500.0), 0.0, 80.0);
    assert!((distance_to_leg_segment(&state, leg) - 500.0).abs() < 0.1);
}

#[test]
fn distance_to_start_when_projection_behind() {
    let leg = StraightLeg { start_ft: Vec2::ZERO, end_ft: Vec2::new(10000.0, 0.0) };
    let state = make_state(Vec2::new(-3000.0, 4000.0), 0.0, 80.0);
    assert!((distance_to_leg_segment(&state, leg) - 5000.0).abs() < 0.1);
}

#[test]
fn far_perpendicular_from_leg_commands_direct_to_start() {
    let leg = StraightLeg {
        start_ft: Vec2::new(-37534.0, 31607.0),
        end_ft: Vec2::new(-32596.0, 31262.0),
    };
    let state = make_state(Vec2::new(-1911.0, 7507.0), 320.0, 63.0);
    let follower = L1PathFollower::new();
    let (desired, _) = follower.follow_leg(&state, leg, 25.0);
    assert!((desired - 304.1).abs() < 2.0, "desired was {}", desired);
}

#[test]
fn close_to_leg_uses_normal_l1() {
    let leg = StraightLeg { start_ft: Vec2::ZERO, end_ft: Vec2::new(10000.0, 0.0) };
    let state = make_state(Vec2::new(5000.0, 500.0), 90.0, 80.0);
    let (desired, _) = L1PathFollower::new().follow_leg(&state, leg, 25.0);
    assert!(wrap(desired, 90.0).abs() < 80.0);
}

#[test]
fn past_end_on_centerline_uses_normal_l1_not_rescue() {
    let leg = StraightLeg { start_ft: Vec2::new(0.0, -10000.0), end_ft: Vec2::new(0.0, 600.0) };
    let state = make_state(Vec2::new(0.0, 3611.0), 0.0, 30.0);
    let (desired, _) = L1PathFollower::new().follow_leg(&state, leg, 10.0);
    assert!(wrap(desired, 0.0).abs() < 10.0, "desired was {}", desired);
}

#[test]
fn large_cross_track_rescue_threshold() {
    let leg = StraightLeg { start_ft: Vec2::ZERO, end_ft: Vec2::new(10000.0, 0.0) };
    let follower = L1PathFollower::new();
    // under threshold
    let close = make_state(Vec2::new(5000.0, 2000.0), 90.0, 80.0);
    let (close_t, _) = follower.follow_leg(&close, leg, 25.0);
    // over threshold
    let far = make_state(Vec2::new(5000.0, 5000.0), 90.0, 80.0);
    let (far_t, _) = follower.follow_leg(&far, leg, 25.0);
    assert!((far_t - 225.0).abs() < 1.0, "far was {}", far_t);
    assert_ne!(close_t.round(), far_t.round());
    let _ = common::default_state();
}
