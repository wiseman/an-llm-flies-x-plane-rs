//! Raw dynamics snapshot → normalized `AircraftState`.
//! Mirrors sim_pilot/core/state_estimator.py.

use crate::config::ConfigBundle;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::sim::simple_dynamics::DynamicsState;
use crate::types::{vector_to_heading, AircraftState, KT_TO_FPS};

pub fn estimate_aircraft_state(
    raw: &DynamicsState,
    config: &ConfigBundle,
    runway_frame: &RunwayFrame,
    dt: f64,
) -> AircraftState {
    let runway_position = runway_frame.to_runway_frame(raw.position_ft);
    let ground_speed_kt = raw.ground_velocity_ft_s.length() / KT_TO_FPS;
    let track_deg = if ground_speed_kt > 1.0 {
        vector_to_heading(raw.ground_velocity_ft_s)
    } else {
        raw.heading_deg
    };
    let runway_length = runway_frame.runway.length_ft;
    let runway_dist_remaining_ft = if (0.0..=runway_length).contains(&runway_position.x) {
        Some(runway_length - runway_position.x)
    } else {
        None
    };
    let touchdown = runway_frame.touchdown_point_ft();
    let distance_to_touchdown_ft = raw.position_ft.distance_to(touchdown);
    AircraftState {
        t_sim: raw.time_s,
        dt,
        position_ft: raw.position_ft,
        alt_msl_ft: raw.altitude_ft,
        alt_agl_ft: (raw.altitude_ft - config.airport.field_elevation_ft).max(0.0),
        pitch_deg: raw.pitch_deg,
        roll_deg: raw.roll_deg,
        heading_deg: raw.heading_deg,
        track_deg,
        p_rad_s: raw.p_rad_s,
        q_rad_s: raw.q_rad_s,
        r_rad_s: raw.r_rad_s,
        ias_kt: raw.ias_kt,
        tas_kt: raw.ias_kt,
        gs_kt: ground_speed_kt,
        vs_fpm: raw.vertical_speed_ft_s * 60.0,
        ground_velocity_ft_s: raw.ground_velocity_ft_s,
        flap_index: raw.flap_index,
        gear_down: raw.gear_down,
        on_ground: raw.on_ground,
        throttle_pos: raw.throttle_pos,
        runway_id: runway_frame.runway.id.clone(),
        runway_dist_remaining_ft,
        runway_x_ft: Some(runway_position.x),
        runway_y_ft: Some(runway_position.y),
        centerline_error_ft: Some(runway_position.y),
        threshold_abeam: runway_position.x.abs() <= config.pattern.abeam_window_ft,
        distance_to_touchdown_ft: Some(distance_to_touchdown_ft),
        stall_margin: raw.ias_kt / config.performance.vso_landing_kt,
    }
}
