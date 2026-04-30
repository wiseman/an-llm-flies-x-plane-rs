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
        self.initial_state_at(
            self.runway_frame.runway.threshold_ft,
            self.runway_frame.runway.course_deg,
            self.config.airport.field_elevation_ft,
        )
    }

    /// Build a zero-velocity on-ground initial state at an arbitrary
    /// world-frame position and heading — used to spawn at a parking
    /// spot rather than the runway threshold.
    pub fn initial_state_at(
        &self,
        position_ft: Vec2,
        heading_deg: f64,
        field_elevation_ft: f64,
    ) -> DynamicsState {
        DynamicsState {
            position_ft,
            altitude_ft: field_elevation_ft,
            heading_deg,
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
        // Drag = parasitic (∝ IAS, scaled by throttle and flap) +
        // induced (∝ 1/IAS, throttle-independent — set by aircraft
        // weight, not engine state). The coefficients balance two
        // operating regimes: powered cruise (throttle 1, IAS ~110) and
        // engine-off glide. The parasitic-throttle scale preserves the
        // original powered-flight equilibrium; induced is added on top
        // and tuned (with the flap multipliers) against X-Plane
        // settled-glide measurements:
        //     IAS 68 flap  0  → drag 2.35 kt/s  (L/D 9.0)
        //     IAS 61 flap  0  → drag 2.38 kt/s  (L/D 9.2)
        //     IAS 68 flap 30  → drag 3.41 kt/s  (L/D 6.0)
        //     IAS 61 flap 30  → drag 3.21 kt/s  (L/D 6.6)
        const PARA_DRAG_K: f64 = 0.085;
        const PARA_DRAG_IDLE_FRACTION: f64 = 0.3;
        const INDUCED_DRAG_K: f64 = 42.0;
        const INDUCED_IAS_FLOOR: f64 = 40.0;
        let para_throttle_scale = PARA_DRAG_IDLE_FRACTION
            + (1.0 - PARA_DRAG_IDLE_FRACTION) * state.throttle_pos.clamp(0.0, 1.0);
        let flap_drag_factor = match state.flap_index {
            i if i <= 0 => 1.00,
            i if i <= 10 => 1.18,
            i if i <= 20 => 1.40,
            _ => 1.62,
        };
        let parasitic_drag = PARA_DRAG_K * state.ias_kt * para_throttle_scale * flap_drag_factor;
        let induced_drag = INDUCED_DRAG_K / state.ias_kt.max(INDUCED_IAS_FLOOR);
        let drag = parasitic_drag
            + induced_drag
            + 0.025 * state.roll_deg.abs()
            + 0.1 * state.pitch_deg.max(0.0);
        let brake = commands.brakes * 24.0;
        // Gravity-along-flight-path component: descending converts PE→KE
        // (aircraft accelerates), climbing converts KE→PE (decelerates).
        // Without this term, throttle-pinned-to-zero glide is impossible —
        // IAS decays to 0 since drag has nothing to balance against.
        // Magnitude is small in normal climbs/descents (~1 kt/s at 3°
        // flight path), so existing powered scenarios are largely
        // unaffected. Only applies when airborne — ground roll doesn't
        // have a flight path.
        let glide_accel_kt_s = if !state.on_ground {
            let flight_path_deg = state.pitch_deg - 2.0;
            -G_FT_S2 * flight_path_deg.to_radians().sin() / KT_TO_FPS
        } else {
            0.0
        };
        state.ias_kt = (state.ias_kt + (thrust - drag - brake + glide_accel_kt_s) * dt).max(0.0);
        let airspeed_ft_s = (state.ias_kt * KT_TO_FPS).max(1.0);

        let mut yaw_rate_deg_s;
        if state.on_ground {
            yaw_rate_deg_s = state.r_rad_s.to_degrees();
            // Runway crown / rudder-centering bias only applies when
            // the wheels are actually on the reference runway — on the
            // ramp this would fight rudder input during taxi.
            let on_runway = {
                let rf_pt = self.runway_frame.to_runway_frame(state.position_ft);
                let length = self.runway_frame.runway.length_ft;
                rf_pt.x >= -50.0 && rf_pt.x <= length + 50.0 && rf_pt.y.abs() <= 75.0
            };
            let runway_restoring = if on_runway {
                0.8 * wrap_degrees_180(
                    self.runway_frame.runway.course_deg - state.heading_deg,
                )
            } else {
                0.0
            };
            // Differential braking: positive pivot_brake = more right
            // brake = nose swings right, matching the live bridge's
            // sign convention. Tapered so the effect dominates at taxi
            // speed and fades as aero forces take over.
            let pivot = clamp(commands.pivot_brake, -1.0, 1.0);
            let pivot_speed_factor =
                (1.0 - (state.ias_kt / 40.0).clamp(0.0, 1.0)).max(0.25);
            let pivot_yaw_accel = 48.0 * pivot * pivot_speed_factor;
            // Nose-wheel steering: rudder pedals are mechanically
            // linked to the nose wheel on the ground; fades above
            // ~60 kt as the nose wheel unloads on rotation.
            let nose_wheel_gain = {
                let speed_factor = (1.0 - (state.ias_kt / 60.0).clamp(0.0, 1.0)).max(0.0);
                120.0 * speed_factor
            };
            let forcing = (16.0 + nose_wheel_gain) * commands.rudder
                + runway_restoring
                + pivot_yaw_accel;
            // Exact-exponential damping step — stable for any dt,
            // unlike explicit Euler (k*dt>2 sign-flips each tick).
            let k = 12.0_f64;
            let decay = (-k * dt).exp();
            let steady_state = forcing / k;
            yaw_rate_deg_s = yaw_rate_deg_s * decay + steady_state * (1.0 - decay);
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

        // On the ground, relax ground velocity toward `heading × ias`
        // with a tire-grip time constant so heading rotations produce
        // brief side-slip (gs > ias during turns). The TaxiProfile's
        // brake-to-target-speed logic relies on that gap — with
        // instantaneous tracking, gs == ias and the brake never fires
        // during taxi turns.
        let velocity_ft_s = if state.on_ground {
            let target_velocity = heading_to_vector(state.heading_deg, state.ias_kt * KT_TO_FPS);
            // τ=0.35 s: perpendicular slip decays to 5% in ~1 s.
            let tau = 0.35;
            let alpha = (-dt / tau).exp();
            state.ground_velocity_ft_s * alpha + target_velocity * (1.0 - alpha)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::load_default_config_bundle;
    use crate::types::ActuatorCommands;

    fn zero_cmd() -> ActuatorCommands {
        ActuatorCommands {
            aileron: 0.0,
            elevator: 0.0,
            rudder: 0.0,
            throttle: 0.0,
            flaps: None,
            gear_down: Some(true),
            brakes: 0.0,
            pivot_brake: 0.0,
        }
    }

    #[test]
    fn pivot_brake_rotates_stationary_aircraft_on_ground() {
        let cfg = load_default_config_bundle();
        let model = SimpleAircraftModel::new(cfg.clone(), Vec2::ZERO);
        let mut state = model.initial_state_at(
            Vec2::new(500.0, 500.0),
            0.0,
            cfg.airport.field_elevation_ft,
        );
        let mut cmd = zero_cmd();
        cmd.brakes = 0.5;
        cmd.pivot_brake = 1.0;
        let dt = 0.2;
        for _ in 0..25 {
            model.step(&mut state, &cmd, dt);
        }
        assert!(
            state.heading_deg > 5.0 && state.heading_deg < 180.0,
            "expected pivot to swing heading clockwise; got {}",
            state.heading_deg
        );
        assert!(
            state.ias_kt < 2.0,
            "pivot with brakes set shouldn't accelerate forward; ias={}",
            state.ias_kt
        );
    }

    #[test]
    fn rudder_steers_nose_wheel_at_taxi_speed() {
        let cfg = load_default_config_bundle();
        let model = SimpleAircraftModel::new(cfg.clone(), Vec2::ZERO);
        let mut state = model.initial_state_at(
            Vec2::new(500.0, 500.0),
            0.0,
            cfg.airport.field_elevation_ft,
        );
        state.ias_kt = 5.0;
        let mut cmd = zero_cmd();
        cmd.rudder = 1.0;
        cmd.throttle = 0.1;
        cmd.brakes = 0.2;
        for _ in 0..25 {
            model.step(&mut state, &cmd, 0.2);
        }
        assert!(
            state.heading_deg > 30.0 && state.heading_deg < 180.0,
            "expected strong rudder-driven turn at taxi speed; got heading={}",
            state.heading_deg
        );
    }

    #[test]
    fn side_slip_keeps_gs_above_ias_when_ias_drops() {
        let cfg = load_default_config_bundle();
        let model = SimpleAircraftModel::new(cfg.clone(), Vec2::ZERO);
        let mut state = model.initial_state_at(
            Vec2::new(500.0, 500.0),
            0.0,
            cfg.airport.field_elevation_ft,
        );
        state.ias_kt = 2.0;
        state.ground_velocity_ft_s = Vec2::new(0.0, 10.0 * KT_TO_FPS);

        let cmd = zero_cmd();
        model.step(&mut state, &cmd, 0.2);
        let gs_kt = state.ground_velocity_ft_s.length() / KT_TO_FPS;
        assert!(
            gs_kt > state.ias_kt + 2.0,
            "with ias dropped to 2, gs should still reflect prior ground motion; gs={:.2} ias={:.2}",
            gs_kt, state.ias_kt
        );
    }

    #[test]
    fn side_slip_decays_within_a_few_seconds() {
        let cfg = load_default_config_bundle();
        let model = SimpleAircraftModel::new(cfg.clone(), Vec2::ZERO);
        let mut state = model.initial_state_at(
            Vec2::new(500.0, 500.0),
            0.0,
            cfg.airport.field_elevation_ft,
        );
        state.ias_kt = 10.0;
        state.ground_velocity_ft_s = Vec2::new(0.0, 10.0 * KT_TO_FPS);
        state.heading_deg = 90.0;

        let mut cmd = zero_cmd();
        cmd.throttle = 0.2;
        for _ in 0..15 {
            model.step(&mut state, &cmd, 0.2);
        }
        let hdg_unit = Vec2::new(
            state.heading_deg.to_radians().sin(),
            state.heading_deg.to_radians().cos(),
        );
        let perp = state.ground_velocity_ft_s.y * hdg_unit.x
            - state.ground_velocity_ft_s.x * hdg_unit.y;
        assert!(
            perp.abs() < 1.0,
            "lateral slip should decay to sub-ft/s after 3 s; perp={:.3}",
            perp
        );
    }

    #[test]
    fn airborne_velocity_follows_heading_instantly() {
        // Side-slip is a ground-physics effect; airborne velocity
        // must track heading so pattern turns don't accumulate slip.
        let cfg = load_default_config_bundle();
        let model = SimpleAircraftModel::new(cfg.clone(), Vec2::ZERO);
        let mut aloft = model.initial_state_at(
            Vec2::new(0.0, 0.0),
            90.0,
            cfg.airport.field_elevation_ft + 1000.0,
        );
        aloft.on_ground = false;
        aloft.altitude_ft = cfg.airport.field_elevation_ft + 1000.0;
        aloft.ias_kt = 80.0;
        aloft.ground_velocity_ft_s = Vec2::new(-10.0, 80.0 * KT_TO_FPS);
        let mut cmd = zero_cmd();
        cmd.throttle = 0.6;
        model.step(&mut aloft, &cmd, 0.2);
        let hdg_unit = Vec2::new(
            aloft.heading_deg.to_radians().sin(),
            aloft.heading_deg.to_radians().cos(),
        );
        let perp = aloft.ground_velocity_ft_s.y * hdg_unit.x
            - aloft.ground_velocity_ft_s.x * hdg_unit.y;
        assert!(perp.abs() < 0.1, "airborne lateral carry-over should be zero; perp={}", perp);
    }

    #[test]
    fn pivot_brake_negative_rotates_opposite_direction() {
        let cfg = load_default_config_bundle();
        let model = SimpleAircraftModel::new(cfg.clone(), Vec2::ZERO);
        let mut state = model.initial_state_at(
            Vec2::new(500.0, 500.0),
            90.0,
            cfg.airport.field_elevation_ft,
        );
        let mut cmd = zero_cmd();
        cmd.brakes = 0.5;
        cmd.pivot_brake = -1.0;
        for _ in 0..25 {
            model.step(&mut state, &cmd, 0.2);
        }
        let went_left = state.heading_deg < 90.0 || state.heading_deg > 270.0;
        assert!(
            went_left,
            "expected negative pivot to turn left; ended heading={}",
            state.heading_deg
        );
    }
}
