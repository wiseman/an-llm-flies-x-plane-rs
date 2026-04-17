//! Ground speed controller for taxiing. No Python counterpart (the
//! Python pilot does not taxi).
//!
//! On the ground, TECS is the wrong tool: there's no pitch/altitude
//! coupling. Instead, a proportional throttle-and-brake controller tracks
//! a target ground speed. Three regimes:
//!
//! - `err > deadband` → advance throttle (≥ breakaway when stationary) to
//!   accelerate.
//! - `err < -deadband` → release throttle and apply proportional brake.
//! - `|err| ≤ deadband` → idle throttle, no brake; inertia carries us.
//!
//! Target `0` kt with the aircraft essentially stopped (< 0.5 kt) latches
//! a hard hold brake so the aircraft doesn't creep on a ramp slope.

use crate::types::clamp;

#[derive(Debug, Clone)]
pub struct GroundSpeedController {
    /// Throttle added per knot of (target - current) when accelerating.
    pub kp_throttle: f64,
    /// Brake applied per knot of (current - target) when decelerating.
    pub kp_brake: f64,
    /// Throttle held when the speed is already at target (idle on ground).
    pub idle_throttle: f64,
    /// ±kt band around the target where we neither add throttle nor brake.
    pub deadband_kt: f64,
    /// Hard brake applied when the commanded speed is zero and the
    /// aircraft is essentially stopped.
    pub hold_brake: f64,
    /// Minimum throttle used to break static friction from a standstill.
    pub breakaway_throttle: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundSpeedCommand {
    pub throttle: f64,
    /// Combined brake command applied symmetrically to both main wheels.
    /// 0 = no brake, 1 = full brake.
    pub brake: f64,
}

impl GroundSpeedController {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update(&self, target_kt: f64, current_kt: f64) -> GroundSpeedCommand {
        // Full stop: latch the hold brake once we're at rest so the aircraft
        // doesn't drift on ramp slopes.
        if target_kt <= 0.01 && current_kt.abs() < 0.5 {
            return GroundSpeedCommand {
                throttle: 0.0,
                brake: self.hold_brake,
            };
        }
        let err = target_kt - current_kt;
        if err < -self.deadband_kt {
            let brake = clamp(-err * self.kp_brake, 0.0, 1.0);
            return GroundSpeedCommand { throttle: 0.0, brake };
        }
        if err > self.deadband_kt {
            let mut throttle = clamp(self.idle_throttle + err * self.kp_throttle, 0.0, 1.0);
            if current_kt < 0.5 {
                throttle = throttle.max(self.breakaway_throttle);
            }
            return GroundSpeedCommand { throttle, brake: 0.0 };
        }
        GroundSpeedCommand {
            throttle: self.idle_throttle,
            brake: 0.0,
        }
    }
}

impl Default for GroundSpeedController {
    fn default() -> Self {
        Self {
            // Tuned against the C172 ground model: at 15 kt target from rest,
            // the aircraft reaches target within ~6 s without overshoot.
            kp_throttle: 0.06,
            kp_brake: 0.12,
            idle_throttle: 0.0,
            deadband_kt: 0.5,
            hold_brake: 0.8,
            breakaway_throttle: 0.35,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctrl() -> GroundSpeedController {
        GroundSpeedController::new()
    }

    #[test]
    fn stationary_with_positive_target_applies_breakaway_throttle() {
        let c = ctrl();
        let cmd = c.update(15.0, 0.0);
        assert!(cmd.brake == 0.0);
        assert!(cmd.throttle >= c.breakaway_throttle, "throttle {}", cmd.throttle);
    }

    #[test]
    fn target_zero_and_stopped_latches_hold_brake() {
        let c = ctrl();
        let cmd = c.update(0.0, 0.1);
        assert_eq!(cmd.throttle, 0.0);
        assert_eq!(cmd.brake, c.hold_brake);
    }

    #[test]
    fn target_zero_while_rolling_applies_proportional_brake_not_hold() {
        let c = ctrl();
        let cmd = c.update(0.0, 10.0);
        assert_eq!(cmd.throttle, 0.0);
        assert!(cmd.brake > 0.0 && cmd.brake <= 1.0);
        // 10 kt × 0.12 = 1.2 → clamp to 1.0.
        assert!((cmd.brake - 1.0).abs() < 1e-9);
    }

    #[test]
    fn within_deadband_coasts_with_idle_throttle_and_no_brake() {
        let c = ctrl();
        let cmd = c.update(15.0, 15.2);
        assert_eq!(cmd.brake, 0.0);
        assert_eq!(cmd.throttle, c.idle_throttle);
    }

    #[test]
    fn above_target_brakes_proportionally_to_overshoot() {
        let c = ctrl();
        let slow = c.update(15.0, 17.0); // 2 kt over
        let hard = c.update(15.0, 22.0); // 7 kt over
        assert!(hard.brake > slow.brake);
        assert_eq!(slow.throttle, 0.0);
        assert_eq!(hard.throttle, 0.0);
    }

    #[test]
    fn moving_below_target_uses_proportional_throttle_not_breakaway() {
        let c = ctrl();
        let cmd = c.update(15.0, 5.0);
        // err=10, throttle = err * kp_throttle = 0.6, clamp to 1.0. Not
        // forced to breakaway because we're already moving.
        assert!(cmd.throttle > c.breakaway_throttle);
        assert_eq!(cmd.brake, 0.0);
    }
}
