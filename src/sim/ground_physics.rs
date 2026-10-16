//! Wheel physics for the offline simulator: what the aircraft does while
//! its weight is on the gear.
//!
//! - **Longitudinal:** thrust minus aero drag (passed in), rolling
//!   friction, and brakes, with a static-friction latch so a stopped
//!   aircraft stays put until thrust beats breakaway.
//! - **Yaw:** the nose wheel steers kinematically (`v·tan δ / wheelbase`,
//!   so zero yaw at a standstill), capped by tire grip at speed;
//!   differential braking pivots at any speed; the rudder adds
//!   aerodynamic yaw that grows with dynamic pressure.
//!
//! Brakes split into left/right as `brakes ∓ pivot_brake`, the same
//! decomposition the X-Plane bridge uses. The parking brake is a floor
//! on both wheels.

use crate::config::GroundHandlingConfig;
use crate::types::{clamp, ActuatorCommands, G_FT_S2, KT_TO_FPS};

/// Ground-roll inputs the wheel model needs beyond the actuator commands.
pub struct GroundStep {
    /// Current wheel speed (kt).
    pub speed_kt: f64,
    /// Current yaw rate (deg/s).
    pub yaw_rate_deg_s: f64,
    /// Propulsive acceleration (kt/s).
    pub thrust_kt_s: f64,
    /// Aerodynamic drag deceleration (kt/s), already computed by the
    /// airframe model.
    pub aero_drag_kt_s: f64,
    /// Parking brake ratio, applied to both wheels.
    pub parking_brake: f64,
}

/// Result of one ground step: new wheel speed and yaw rate.
pub struct GroundOutcome {
    pub speed_kt: f64,
    pub yaw_rate_deg_s: f64,
}

pub fn step_ground(
    cfg: &GroundHandlingConfig,
    input: &GroundStep,
    commands: &ActuatorCommands,
    dt: f64,
) -> GroundOutcome {
    let brake_sym = clamp(commands.brakes, 0.0, 1.0);
    let pivot = clamp(commands.pivot_brake, -1.0, 1.0);
    let parking = clamp(input.parking_brake, 0.0, 1.0);
    let brake_left = (brake_sym - pivot).clamp(parking, 1.0);
    let brake_right = (brake_sym + pivot).clamp(parking, 1.0);
    let braking = cfg.brake_decel_kt_s * 0.5 * (brake_left + brake_right);
    let asymmetry = brake_right - brake_left; // + yaws right

    // Longitudinal. Friction and brakes only oppose motion, so they can
    // stop the aircraft but never push it backward.
    let speed = input.speed_kt.max(0.0);
    let resist = cfg.rolling_friction_kt_s + cfg.rolling_drag_per_kt * speed + braking;
    let new_speed = if speed < cfg.static_friction_kt
        && input.thrust_kt_s <= braking + cfg.breakaway_kt_s
    {
        0.0
    } else {
        (speed + (input.thrust_kt_s - input.aero_drag_kt_s - resist) * dt).max(0.0)
    };

    // Yaw. The nose wheel drags the yaw rate toward its kinematic rate
    // only while rolling; at rest it just points.
    let v_fps = new_speed * KT_TO_FPS;
    let r = input.yaw_rate_deg_s;
    let nose_rate = if v_fps > 0.01 {
        let delta = (cfg.max_nosewheel_deg * clamp(commands.rudder, -1.0, 1.0)).to_radians();
        let kinematic = v_fps * delta.tan() / cfg.wheelbase_ft;
        let grip_limit = cfg.max_lateral_accel_g * G_FT_S2 / v_fps;
        Some(clamp(kinematic, -grip_limit, grip_limit).to_degrees())
    } else {
        None
    };
    let q_ratio = (new_speed / cfg.rudder_reference_kt).powi(2);
    let forcing = cfg.differential_brake_yaw_accel * asymmetry
        + cfg.aero_rudder_yaw_accel * q_ratio * clamp(commands.rudder, -1.0, 1.0);
    // r' = k_nw (ω_nw − r) + forcing − c r, integrated exactly so large
    // gains stay stable at the 0.2 s step.
    let (k_nw, target) = match nose_rate {
        Some(w) => (cfg.nose_wheel_responsiveness, w),
        None => (0.0, 0.0),
    };
    let k = k_nw + cfg.yaw_damping;
    let steady = (k_nw * target + forcing) / k;
    let decay = (-k * dt).exp();
    let yaw_rate = r * decay + steady * (1.0 - decay);

    GroundOutcome { speed_kt: new_speed, yaw_rate_deg_s: yaw_rate }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd() -> ActuatorCommands {
        ActuatorCommands {
            aileron: 0.0,
            elevator: 0.0,
            rudder: 0.0,
            slip: 0.0,
            throttle: 0.0,
            flaps: None,
            gear_down: Some(true),
            brakes: 0.0,
            pivot_brake: 0.0,
            parking_brake: None,
        }
    }

    fn input(speed_kt: f64, thrust_kt_s: f64) -> GroundStep {
        GroundStep {
            speed_kt,
            yaw_rate_deg_s: 0.0,
            thrust_kt_s,
            aero_drag_kt_s: 0.0,
            parking_brake: 0.0,
        }
    }

    #[test]
    fn parking_brake_holds_against_taxi_power() {
        let cfg = GroundHandlingConfig::default();
        let mut s = input(0.0, 2.5);
        s.parking_brake = 1.0;
        let out = step_ground(&cfg, &s, &cmd(), 0.2);
        assert_eq!(out.speed_kt, 0.0);
    }

    #[test]
    fn idle_rolling_aircraft_coasts_to_a_stop_and_stays() {
        let cfg = GroundHandlingConfig::default();
        let mut s = input(5.0, 0.0);
        for _ in 0..100 {
            let out = step_ground(&cfg, &s, &cmd(), 0.2);
            s.speed_kt = out.speed_kt;
        }
        assert_eq!(s.speed_kt, 0.0);
    }

    #[test]
    fn thrust_below_breakaway_does_not_creep() {
        let cfg = GroundHandlingConfig::default();
        let out = step_ground(&cfg, &input(0.0, cfg.breakaway_kt_s * 0.9), &cmd(), 0.2);
        assert_eq!(out.speed_kt, 0.0);
        let out = step_ground(&cfg, &input(0.0, cfg.breakaway_kt_s * 2.0), &cmd(), 0.2);
        assert!(out.speed_kt > 0.0);
    }

    #[test]
    fn full_brake_holds_against_taxi_power() {
        let cfg = GroundHandlingConfig::default();
        let mut c = cmd();
        c.brakes = 1.0;
        assert_eq!(step_ground(&cfg, &input(0.0, 3.0), &c, 0.2).speed_kt, 0.0);
    }

    #[test]
    fn nose_wheel_cannot_turn_a_stopped_aircraft() {
        let cfg = GroundHandlingConfig::default();
        let mut c = cmd();
        c.rudder = 1.0;
        let out = step_ground(&cfg, &input(0.0, 0.0), &c, 0.2);
        assert_eq!(out.yaw_rate_deg_s, 0.0);
    }

    #[test]
    fn differential_brake_pivots_at_rest() {
        let cfg = GroundHandlingConfig::default();
        let mut c = cmd();
        c.pivot_brake = 0.4;
        let mut s = input(0.0, 0.0);
        for _ in 0..10 {
            s.yaw_rate_deg_s = step_ground(&cfg, &s, &c, 0.2).yaw_rate_deg_s;
        }
        assert!(s.yaw_rate_deg_s > 1.0, "{}", s.yaw_rate_deg_s);
    }

    #[test]
    fn tire_grip_caps_yaw_rate_at_rollout_speed() {
        let cfg = GroundHandlingConfig::default();
        let mut c = cmd();
        c.rudder = 1.0;
        let mut s = input(50.0, 0.0);
        s.aero_drag_kt_s = -cfg.rolling_friction_kt_s - cfg.rolling_drag_per_kt * 50.0;
        for _ in 0..50 {
            s.yaw_rate_deg_s = step_ground(&cfg, &s, &c, 0.2).yaw_rate_deg_s;
        }
        // 0.4 g at 50 kt ≈ 8.8°/s of grip-limited turn, plus a little
        // aero rudder; nowhere near the ~160°/s pure kinematic rate.
        assert!(s.yaw_rate_deg_s < 20.0, "{}", s.yaw_rate_deg_s);
        assert!(s.yaw_rate_deg_s > 5.0, "{}", s.yaw_rate_deg_s);
    }
}
