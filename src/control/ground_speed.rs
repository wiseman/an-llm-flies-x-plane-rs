//! Ground speed controller for taxiing.
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
    /// Sized so that ~4 kt of deficit saturates the throttle to 1.0 — the
    /// C172 nose wheel fully deflected can scrub enough that nothing less
    /// than full thrust breaks free, and artificially capping the output
    /// leaves the aircraft frozen on a ramp. If the aircraft is close to
    /// target, the proportional term naturally gives a small nudge.
    pub kp_throttle: f64,
    /// Brake applied per knot of (current - target) when decelerating.
    pub kp_brake: f64,
    /// Throttle held when we're inside the deadband (coasting at target).
    /// Zero on the ground because there is no headwind/drag we need to
    /// actively fight; friction will bring us back to target and the
    /// proportional term re-engages then.
    pub idle_throttle: f64,
    /// ±kt band around the target where we neither add throttle nor brake.
    pub deadband_kt: f64,
    /// Hard brake applied when the commanded speed is zero and the
    /// aircraft is essentially stopped. 1.0 = full brake; no need to leave
    /// slack — if the aircraft is at rest and asked to stay that way, we
    /// want the firmest possible hold.
    pub hold_brake: f64,
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
            // err × kp_throttle naturally saturates at 1.0 for big errors
            // (e.g. 5 kt deficit → 1.25 → clamps to full throttle, which
            // is what we want when the aircraft is stationary and the
            // nose wheel is scrubbing). Small errors give a small nudge.
            let throttle = clamp(self.idle_throttle + err * self.kp_throttle, 0.0, 1.0);
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
            // Tuned so ~4 kt of deficit saturates throttle to 1.0. The
            // aircraft physics then decide how much power it actually
            // needs to break free: if scrub is light, the proportional
            // term fades naturally as speed builds; if scrub is heavy
            // (nose wheel fully deflected), the controller sits at full
            // throttle until the aircraft finally starts rolling.
            kp_throttle: 0.25,
            kp_brake: 0.12,
            idle_throttle: 0.0,
            deadband_kt: 0.5,
            hold_brake: 1.0,
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
    fn stationary_with_positive_target_saturates_throttle() {
        let c = ctrl();
        let cmd = c.update(15.0, 0.0);
        assert_eq!(cmd.brake, 0.0);
        // 15 kt of deficit → 15 × 0.25 = 3.75 → clamps to 1.0 full throttle.
        assert!((cmd.throttle - 1.0).abs() < 1e-9, "throttle {}", cmd.throttle);
    }

    #[test]
    fn small_positive_error_gives_small_throttle_nudge() {
        let c = ctrl();
        // 1 kt of deficit from cruise: 1 × 0.25 = 0.25.
        let cmd = c.update(15.0, 14.0);
        assert_eq!(cmd.brake, 0.0);
        assert!(
            (cmd.throttle - 0.25).abs() < 1e-9,
            "throttle {}",
            cmd.throttle
        );
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
    fn moving_well_below_target_saturates_throttle() {
        let c = ctrl();
        let cmd = c.update(15.0, 5.0);
        // 10 kt deficit × 0.25 = 2.5 → clamps to 1.0 full throttle.
        assert!((cmd.throttle - 1.0).abs() < 1e-9);
        assert_eq!(cmd.brake, 0.0);
    }
}
