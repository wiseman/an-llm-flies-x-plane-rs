//! Point-mass aircraft dynamics for the offline `simple` backend.

use crate::config::ConfigBundle;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::types::{
    clamp, heading_to_vector, wrap_degrees_180, wrap_degrees_360, ActuatorCommands, Vec2, G_FT_S2,
    KT_TO_FPS,
};

#[derive(Debug, Clone)]
pub struct DynamicsState {
    pub position_ft: Vec2,
    pub altitude_ft: f64,
    pub heading_deg: f64,
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub ias_kt: f64,
    pub throttle_pos: f64,
    pub on_ground: bool,
    pub time_s: f64,
    pub p_rad_s: f64,
    pub q_rad_s: f64,
    pub r_rad_s: f64,
    pub ground_velocity_ft_s: Vec2,
    pub vertical_speed_ft_s: f64,
    pub flap_index: i32,
    pub gear_down: bool,
}

pub struct SimpleAircraftModel {
    pub config: ConfigBundle,
    pub wind_vector_kt: Vec2,
    pub wind_vector_ft_s: Vec2,
    pub runway_frame: RunwayFrame,
}

impl SimpleAircraftModel {
    pub fn new(config: ConfigBundle, wind_vector_kt: Vec2) -> Self {
        let wind_vector_ft_s = wind_vector_kt * KT_TO_FPS;
        let runway_frame = RunwayFrame::new(config.airport.runway.clone());
        Self {
            config,
            wind_vector_kt,
            wind_vector_ft_s,
            runway_frame,
        }
    }

    pub fn initial_state(&self) -> DynamicsState {
        DynamicsState {
            position_ft: self.runway_frame.runway.threshold_ft,
            altitude_ft: self.config.airport.field_elevation_ft,
            heading_deg: self.runway_frame.runway.course_deg,
            roll_deg: 0.0,
            pitch_deg: 0.0,
            ias_kt: 0.0,
            throttle_pos: 0.0,
            on_ground: true,
            time_s: 0.0,
            p_rad_s: 0.0,
            q_rad_s: 0.0,
            r_rad_s: 0.0,
            ground_velocity_ft_s: Vec2::ZERO,
            vertical_speed_ft_s: 0.0,
            flap_index: 0,
            gear_down: true,
        }
    }

    pub fn step(&self, state: &mut DynamicsState, commands: &ActuatorCommands, dt: f64) {
        let target_throttle = clamp(commands.throttle, 0.0, 1.0);
        state.throttle_pos += clamp(target_throttle - state.throttle_pos, -1.2 * dt, 1.2 * dt);

        let mut roll_rate_deg_s = state.p_rad_s.to_degrees();
        let mut roll_accel = (72.0 * commands.aileron) - (3.0 * roll_rate_deg_s) - (0.6 * state.roll_deg);
        if state.on_ground {
            roll_accel -= 1.5 * state.roll_deg;
        }
        roll_rate_deg_s += roll_accel * dt;
        state.roll_deg = clamp(state.roll_deg + roll_rate_deg_s * dt, -45.0, 45.0);
        state.p_rad_s = roll_rate_deg_s.to_radians();

        let mut pitch_rate_deg_s = state.q_rad_s.to_degrees();
        let pitch_authority = if state.on_ground { 18.0 } else { 42.0 };
        let pitch_accel =
            pitch_authority * commands.elevator - 2.5 * pitch_rate_deg_s - 0.8 * state.pitch_deg;
        pitch_rate_deg_s += pitch_accel * dt;
        let pitch_limit_up = if state.on_ground { 12.0 } else { 15.0 };
        state.pitch_deg = clamp(state.pitch_deg + pitch_rate_deg_s * dt, -10.0, pitch_limit_up);
        state.q_rad_s = pitch_rate_deg_s.to_radians();

        let thrust = 10.5 * state.throttle_pos;
        let drag = 0.085 * state.ias_kt + 0.025 * state.roll_deg.abs() + 0.1 * state.pitch_deg.max(0.0);
        let brake = commands.brakes * 24.0;
        state.ias_kt = (state.ias_kt + (thrust - drag - brake) * dt).max(0.0);
        let airspeed_ft_s = (state.ias_kt * KT_TO_FPS).max(1.0);

        let mut yaw_rate_deg_s;
        if state.on_ground {
            yaw_rate_deg_s = state.r_rad_s.to_degrees();
            let runway_error_deg =
                wrap_degrees_180(self.runway_frame.runway.course_deg - state.heading_deg);
            let yaw_accel =
                16.0 * commands.rudder - 3.0 * yaw_rate_deg_s + 0.8 * runway_error_deg;
            yaw_rate_deg_s += yaw_accel * dt;
            state.vertical_speed_ft_s = 0.0;
            state.altitude_ft = self.config.airport.field_elevation_ft;
            if state.ias_kt >= self.config.performance.vr_kt
                && state.pitch_deg >= 4.0
                && state.throttle_pos >= 0.7
            {
                state.on_ground = false;
                state.altitude_ft += 1.0;
            }
        } else {
            let desired_turn_rate_deg_s =
                (G_FT_S2 * state.roll_deg.to_radians().tan() / airspeed_ft_s.max(80.0))
                    .to_degrees();
            yaw_rate_deg_s = state.r_rad_s.to_degrees();
            yaw_rate_deg_s += (desired_turn_rate_deg_s - yaw_rate_deg_s) * (2.0 * dt).min(1.0);
            let climb_efficiency = clamp(
                (state.ias_kt - (self.config.performance.vr_kt - 5.0)) / 20.0,
                0.0,
                1.2,
            );
            let flight_path_deg = state.pitch_deg - 2.0;
            state.vertical_speed_ft_s = airspeed_ft_s * flight_path_deg.to_radians().sin() * climb_efficiency
                - (0.25f64 - state.throttle_pos).max(0.0) * 12.0;
            state.altitude_ft += state.vertical_speed_ft_s * dt;
            if state.altitude_ft <= self.config.airport.field_elevation_ft {
                state.altitude_ft = self.config.airport.field_elevation_ft;
                state.on_ground = true;
                state.vertical_speed_ft_s = 0.0;
                state.pitch_deg = state.pitch_deg.min(4.0);
                yaw_rate_deg_s *= 0.4;
            }
        }

        state.r_rad_s = yaw_rate_deg_s.to_radians();
        state.heading_deg = wrap_degrees_360(state.heading_deg + yaw_rate_deg_s * dt);

        let velocity_ft_s = if state.on_ground {
            let gs = state.ias_kt * KT_TO_FPS;
            heading_to_vector(state.heading_deg, gs)
        } else {
            let air_velocity = heading_to_vector(state.heading_deg, airspeed_ft_s);
            air_velocity + self.wind_vector_ft_s
        };

        state.ground_velocity_ft_s = velocity_ft_s;
        state.position_ft = state.position_ft + velocity_ft_s * dt;
        state.time_s += dt;
        if let Some(flaps) = commands.flaps {
            state.flap_index = flaps;
        }
        if let Some(gear) = commands.gear_down {
            state.gear_down = gear;
        }
    }
}
