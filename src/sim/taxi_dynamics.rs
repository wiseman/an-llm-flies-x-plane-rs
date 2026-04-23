//! Ground dynamics for taxi scenarios.
//!
//! ## Why a separate model from `SimpleAircraftModel`
//!
//! `simple_dynamics::SimpleAircraftModel` is tuned for takeoff / pattern /
//! landing. Three things make it unsuitable for offline taxi testing:
//!
//! 1. **Runway-course attractor.** On the ground it adds
//!    `0.8 * wrap(runway_course − heading)` to the yaw acceleration,
//!    which pulls the aircraft back toward the runway heading. Great
//!    during takeoff roll and rollout — actively fighting any taxi
//!    clearance that routes the aircraft onto a taxiway pointed
//!    somewhere else.
//! 2. **No differential braking.** It reads only `commands.brakes` (the
//!    symmetric component) and ignores `commands.pivot_brake`, so the
//!    controller's tight-turn pivot braking is invisible in the sim.
//! 3. **Nose-wheel authority is speed-independent.** Rudder is applied
//!    as a raw yaw torque term, so the aircraft can pivot on the nose
//!    wheel at zero forward speed — which a real C172 cannot.
//!
//! `TaxiDynamicsModel` models each of those honestly:
//!
//! - yaw comes from three distinct sources — nose-wheel (proportional to
//!   ground speed), differential braking (works at any speed), and yaw
//!   rate damping — with no runway-course bias;
//! - brakes are split into left / right via the same
//!   `brakes + pivot_brake` decomposition the X-Plane bridge uses, so
//!   the dynamics see exactly what the live sim would;
//! - nose-wheel yaw authority drops to zero as ground speed drops to
//!   zero, making differential braking the only way to pivot in place.
//!
//! Simplifications kept from the simple model:
//!
//! - No lateral skid. Ground velocity is always aligned with heading; the
//!   aircraft can't slide sideways. Safe at taxi speeds on dry pavement.
//! - Pitch / roll held at zero. We're on the ground.
//! - Altitude pinned to field elevation.

use crate::config::ConfigBundle;
use crate::sim::simple_dynamics::DynamicsState;
use crate::types::{
    clamp, heading_to_vector, wrap_degrees_360, ActuatorCommands, Vec2, KT_TO_FPS,
};

pub struct TaxiDynamicsModel {
    pub config: ConfigBundle,
    /// Distance from nose wheel to the main gear (feet). C172 is ~5.2 ft.
    /// Shows up in the nose-wheel kinematic: commanded yaw rate =
    /// `v_fps × tan(nose_deflection) / wheelbase`.
    pub wheelbase_ft: f64,
    /// Peak nose-wheel steering angle (deg) at |rudder| = 1. C172 without
    /// tiller is ~10°; larger aircraft rely on a separate tiller for up
    /// to 70°.
    pub max_nose_deflection_deg: f64,
    /// Forward acceleration at `throttle_pos = 1.0` (ft/s²). Tuned so a
    /// C172 takes ~6 s to reach 10 kt from a standing start — matches
    /// what the live sim showed before we broke free of the breakaway
    /// knot.
    pub thrust_accel_at_full: f64,
    /// Rolling drag: deceleration (ft/s²) per knot of current speed.
    pub rolling_resistance_per_kt: f64,
    /// Deceleration (ft/s²) at full symmetric brake.
    pub brake_authority_fps2: f64,
    /// Yaw acceleration (deg/s²) per unit of brake asymmetry (right − left).
    /// At full pivot brake (one wheel at 0.4, other at 0) this is 0.4 ×
    /// this constant = initial yaw authority with no forward speed.
    pub differential_brake_yaw_accel: f64,
    /// Yaw rate damping (1/s). Higher = more aggressive damping.
    pub yaw_damping: f64,
    /// Rate (1/s) at which actual yaw rate tracks the nose-wheel
    /// commanded yaw rate. Higher = more immediate tire response.
    pub nose_wheel_responsiveness: f64,
    /// Below this speed (kt) and below a thrust threshold, static
    /// friction pins the aircraft at zero — no creep under idle.
    pub static_friction_kt: f64,
}

impl TaxiDynamicsModel {
    pub fn new(config: ConfigBundle) -> Self {
        Self {
            config,
            wheelbase_ft: 5.2,
            max_nose_deflection_deg: 10.0,
            thrust_accel_at_full: 8.5,
            rolling_resistance_per_kt: 0.15,
            brake_authority_fps2: 20.0,
            differential_brake_yaw_accel: 50.0,
            yaw_damping: 2.5,
            nose_wheel_responsiveness: 3.0,
            static_friction_kt: 0.1,
        }
    }

    pub fn initial_state(&self, position_ft: Vec2, heading_deg: f64) -> DynamicsState {
        DynamicsState {
            position_ft,
            altitude_ft: self.config.airport.field_elevation_ft,
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

    pub fn step(&self, state: &mut DynamicsState, cmd: &ActuatorCommands, dt: f64) {
        // Throttle lag — the C172 carb + lever combo isn't instantaneous.
        let target = clamp(cmd.throttle, 0.0, 1.0);
        state.throttle_pos += clamp(target - state.throttle_pos, -1.2 * dt, 1.2 * dt);

        // Split symmetric + pivot into left / right the same way the
        // X-Plane bridge does: left = brakes − pivot, right = brakes +
        // pivot, each clamped to [0,1]. Preserves differential authority
        // even at full symmetric hold brake.
        let brake_sym = clamp(cmd.brakes, 0.0, 1.0);
        let pivot = clamp(cmd.pivot_brake, -1.0, 1.0);
        let brake_left = (brake_sym - pivot).clamp(0.0, 1.0);
        let brake_right = (brake_sym + pivot).clamp(0.0, 1.0);
        let brake_total = brake_left + brake_right; // 0..2
        let brake_asymmetry = brake_right - brake_left; // −1..1 (+ = yaw right)

        // Forward dynamics: thrust minus rolling resistance minus brakes,
        // clamped so the aircraft can't roll backward.
        let thrust = self.thrust_accel_at_full * state.throttle_pos;
        let rolling = self.rolling_resistance_per_kt * state.ias_kt;
        let braking = self.brake_authority_fps2 * (brake_total * 0.5);
        let net_accel = thrust - rolling - braking;
        let mut new_speed_kt = state.ias_kt + net_accel * dt;

        // Static friction: if we're essentially stopped and thrust is
        // below what it takes to overcome brakes + rolling drag, latch
        // at zero. Prevents the ground-speed controller's low-throttle
        // creep from drifting the aircraft when we mean to hold.
        if state.ias_kt.abs() < self.static_friction_kt && thrust < braking + rolling + 0.5 {
            new_speed_kt = 0.0;
        }
        if new_speed_kt < 0.0 {
            new_speed_kt = 0.0;
        }
        state.ias_kt = new_speed_kt;

        // Yaw dynamics. Three contributions summed into yaw acceleration:
        let yaw_rate_deg_s = state.r_rad_s.to_degrees();

        // 1. Nose-wheel: kinematic yaw rate
        //    ω = v_fps × tan(δ_nose) / wheelbase
        //    → zero at zero ground speed. First-order filter toward it.
        //    Crucial: when at rest the nose-wheel produces *no* torque at
        //    all (it's just pointing, not rotating the aircraft), so the
        //    convergence term has to vanish too — otherwise it acts as
        //    extra yaw damping that fights the differential brake pivot.
        let nose_deflection_deg = self.max_nose_deflection_deg * clamp(cmd.rudder, -1.0, 1.0);
        let nose_term = if state.ias_kt > 0.01 {
            let v_fps = state.ias_kt * KT_TO_FPS;
            let commanded_nose_yaw_rate =
                (v_fps * nose_deflection_deg.to_radians().tan() / self.wheelbase_ft).to_degrees();
            (commanded_nose_yaw_rate - yaw_rate_deg_s) * self.nose_wheel_responsiveness
        } else {
            0.0
        };

        // 2. Differential brake: torque moment that works at any speed.
        //    Positive asymmetry (right wheel with more brake) yaws right.
        let diff_term = self.differential_brake_yaw_accel * brake_asymmetry;

        // 3. Damping on the residual yaw rate (stabilizes oscillation).
        let damp_term = -self.yaw_damping * yaw_rate_deg_s;

        let yaw_accel = nose_term + diff_term + damp_term;
        let new_yaw_rate = yaw_rate_deg_s + yaw_accel * dt;
        state.r_rad_s = new_yaw_rate.to_radians();
        state.heading_deg = wrap_degrees_360(state.heading_deg + new_yaw_rate * dt);

        // Position advances along heading (no lateral skid).
        let v_fps = state.ias_kt * KT_TO_FPS;
        let velocity = heading_to_vector(state.heading_deg, v_fps);
        state.ground_velocity_ft_s = velocity;
        state.position_ft = state.position_ft + velocity * dt;

        // Hold the aircraft on the ground.
        state.altitude_ft = self.config.airport.field_elevation_ft;
        state.vertical_speed_ft_s = 0.0;
        state.on_ground = true;
        state.pitch_deg = 0.0;
        state.roll_deg = 0.0;

        if let Some(flaps) = cmd.flaps {
            state.flap_index = flaps;
        }
        if let Some(gear) = cmd.gear_down {
            state.gear_down = gear;
        }
        state.time_s += dt;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::load_default_config_bundle;
    use crate::types::ActuatorCommands;

    fn mk() -> TaxiDynamicsModel {
        TaxiDynamicsModel::new(load_default_config_bundle())
    }

    fn idle_cmd() -> ActuatorCommands {
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
    fn full_throttle_accelerates() {
        let m = mk();
        let mut s = m.initial_state(Vec2::ZERO, 0.0);
        let mut cmd = idle_cmd();
        cmd.throttle = 1.0;
        for _ in 0..50 {
            m.step(&mut s, &cmd, 0.2);
        }
        assert!(s.ias_kt > 5.0, "expected to be moving: {}", s.ias_kt);
    }

    #[test]
    fn full_brake_stops_and_static_friction_latches() {
        let m = mk();
        let mut s = m.initial_state(Vec2::ZERO, 0.0);
        s.ias_kt = 10.0;
        let mut cmd = idle_cmd();
        cmd.brakes = 1.0;
        for _ in 0..50 {
            m.step(&mut s, &cmd, 0.2);
        }
        assert_eq!(s.ias_kt, 0.0);
    }

    #[test]
    fn nose_wheel_at_rest_does_not_pivot() {
        let m = mk();
        let mut s = m.initial_state(Vec2::ZERO, 0.0);
        let mut cmd = idle_cmd();
        cmd.rudder = 1.0; // full right
        cmd.brakes = 1.0; // hold still
        let hdg_before = s.heading_deg;
        for _ in 0..50 {
            m.step(&mut s, &cmd, 0.2);
        }
        // Within ~1°. The static-friction branch holds speed at 0 so the
        // kinematic nose-wheel yaw rate is zero; any rotation must come
        // from the damping term on residual yaw (zero here).
        assert!((s.heading_deg - hdg_before).abs() < 1.0, "heading drifted: {}", s.heading_deg);
    }

    #[test]
    fn differential_brake_pivots_at_rest() {
        let m = mk();
        let mut s = m.initial_state(Vec2::ZERO, 0.0);
        let mut cmd = idle_cmd();
        // Full pivot brake on the right wheel — should yaw right.
        cmd.pivot_brake = 0.5;
        for _ in 0..20 {
            m.step(&mut s, &cmd, 0.2);
        }
        assert!(
            s.r_rad_s.to_degrees() > 1.0,
            "expected right yaw rate, got {} deg/s",
            s.r_rad_s.to_degrees()
        );
    }

    #[test]
    fn nose_wheel_turn_while_moving() {
        let m = mk();
        let mut s = m.initial_state(Vec2::ZERO, 0.0);
        s.ias_kt = 5.0;
        s.throttle_pos = 0.3;
        let mut cmd = idle_cmd();
        cmd.throttle = 0.3;
        cmd.rudder = 1.0;
        for _ in 0..10 {
            m.step(&mut s, &cmd, 0.2);
        }
        assert!(
            s.heading_deg > 5.0,
            "expected right turn, got heading {}",
            s.heading_deg
        );
    }

    #[test]
    fn ground_velocity_aligns_with_heading_no_skid() {
        let m = mk();
        let mut s = m.initial_state(Vec2::ZERO, 45.0);
        s.ias_kt = 5.0;
        let cmd = idle_cmd();
        m.step(&mut s, &cmd, 0.2);
        // Velocity vector should point at heading 45° — i.e. equal x and y.
        let vx = s.ground_velocity_ft_s.x;
        let vy = s.ground_velocity_ft_s.y;
        assert!((vx - vy).abs() < 1e-6, "vx={} vy={}", vx, vy);
    }
}
