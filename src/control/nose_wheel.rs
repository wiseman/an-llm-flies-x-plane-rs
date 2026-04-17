//! Nose-wheel steering controller for taxiing. No Python counterpart.
//!
//! On the ground, rudder input is coupled to the nose wheel in most
//! X-Plane aircraft, so a single rudder command steers the aircraft. The
//! controller superposes three terms:
//!
//! - `-kp_crosstrack * crosstrack_ft`: pulls the aircraft back toward the
//!   leg centerline. Positive crosstrack = right of leg → left rudder
//!   (negative).
//! - `+kp_heading * heading_error_deg`: aligns heading with the leg
//!   bearing. Positive heading error = leg bearing is clockwise of current
//!   heading → right rudder.
//! - `-kp_yaw_rate * yaw_rate_deg_s`: damps the yaw rate so the aircraft
//!   doesn't fishtail on sharp inputs.
//!
//! Sign conventions match `centerline_rollout.rs`: positive rudder output
//! is right rudder. Output is clamped to [-1, 1].

use crate::types::clamp;

#[derive(Debug, Clone)]
pub struct NoseWheelController {
    /// Rudder per foot of crosstrack error. At the default the crosstrack
    /// term alone saturates the rudder at ~12 ft — tighter than a 20 ft
    /// band so the aircraft actually pulls onto the leg instead of
    /// S-curving indefinitely at low gain.
    pub kp_crosstrack: f64,
    /// Rudder per degree of heading error. Saturates at ~20° alone.
    pub kp_heading: f64,
    /// Rudder damping on yaw rate (deg/s). Sized to blunt the nose-wheel
    /// snap but not fight the steady-state turn toward the leg.
    pub kp_yaw_rate: f64,
    /// Rudder deflection threshold above which a tight-turn differential
    /// brake kicks in (|rudder| in [0, 1]).
    pub pivot_rudder_threshold: f64,
    /// Ground speed (kt) below which pivot-brake authority is at full
    /// magnitude.
    pub pivot_low_speed_kt: f64,
    /// Ground speed (kt) above which pivot-brake authority fades to zero.
    pub pivot_high_speed_kt: f64,
    /// Maximum pivot-brake fraction applied to the inside wheel. 0.4 is
    /// enough to tighten a C172 turn radius meaningfully without locking
    /// the wheel.
    pub pivot_max_brake: f64,
}

/// Paired output from `NoseWheelController::update`. `pivot_brake` is
/// signed (positive = right wheel gets extra brake) so the caller can
/// drop it straight into `ActuatorCommands.pivot_brake`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NoseWheelCommand {
    pub rudder: f64,
    pub pivot_brake: f64,
}

impl NoseWheelController {
    pub fn new() -> Self {
        Self::default()
    }

    /// `crosstrack_ft`: signed distance off the leg centerline (positive =
    /// right of leg when facing leg direction).
    /// `heading_error_deg`: `wrap(-180..180, leg_bearing - current_heading)`
    /// — positive means we need to turn right.
    /// `yaw_rate_deg_s`: positive when yawing right.
    /// `gs_kt`: current ground speed. Used to schedule pivot-brake
    /// authority (only active below a few knots where nose-wheel alone
    /// can't produce a tight-enough turn).
    pub fn update(
        &self,
        crosstrack_ft: f64,
        heading_error_deg: f64,
        yaw_rate_deg_s: f64,
        gs_kt: f64,
    ) -> NoseWheelCommand {
        let lateral = -crosstrack_ft * self.kp_crosstrack;
        let heading = heading_error_deg * self.kp_heading;
        let damping = -yaw_rate_deg_s * self.kp_yaw_rate;
        let rudder = clamp(lateral + heading + damping, -1.0, 1.0);

        // Pivot-brake: couples to the rudder command. Engages when the
        // rudder is pushing hard (|rudder| past pivot_rudder_threshold)
        // AND the aircraft is below pivot_high_speed_kt. At pivot_low_
        // speed_kt we hit full authority. The sign follows the rudder so
        // the *inside* wheel brakes; the outside wheel + engine thrust
        // then pivot the nose around.
        let speed_factor = clamp(
            (self.pivot_high_speed_kt - gs_kt)
                / (self.pivot_high_speed_kt - self.pivot_low_speed_kt).max(1e-6),
            0.0,
            1.0,
        );
        let rudder_factor = clamp(
            (rudder.abs() - self.pivot_rudder_threshold)
                / (1.0 - self.pivot_rudder_threshold).max(1e-6),
            0.0,
            1.0,
        );
        let magnitude = self.pivot_max_brake * speed_factor * rudder_factor;
        let pivot_brake = magnitude * rudder.signum();

        NoseWheelCommand { rudder, pivot_brake }
    }
}

impl Default for NoseWheelController {
    fn default() -> Self {
        Self {
            kp_crosstrack: 0.08,
            kp_heading: 0.05,
            kp_yaw_rate: 0.08,
            // Pivot-brake engages at 0.7 rudder and fades to full at 1.0.
            // Full authority below 2 kt, tapering out by 6 kt — above
            // that, nose-wheel has enough momentum + slip to turn on its
            // own and differential braking would just slow the aircraft.
            // Max 0.4 brake is a pivot, not a lockup.
            pivot_rudder_threshold: 0.7,
            pivot_low_speed_kt: 2.0,
            pivot_high_speed_kt: 6.0,
            pivot_max_brake: 0.4,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctrl() -> NoseWheelController {
        NoseWheelController::new()
    }

    #[test]
    fn zero_error_produces_zero_rudder_and_no_pivot_brake() {
        let c = ctrl();
        let cmd = c.update(0.0, 0.0, 0.0, 5.0);
        assert_eq!(cmd.rudder, 0.0);
        assert_eq!(cmd.pivot_brake, 0.0);
    }

    #[test]
    fn right_of_centerline_commands_left_rudder() {
        let c = ctrl();
        let cmd = c.update(10.0, 0.0, 0.0, 5.0);
        assert!(cmd.rudder < 0.0, "rudder={}", cmd.rudder);
    }

    #[test]
    fn leg_bearing_clockwise_of_heading_commands_right_rudder() {
        let c = ctrl();
        let cmd = c.update(0.0, 10.0, 0.0, 5.0);
        assert!(cmd.rudder > 0.0, "rudder={}", cmd.rudder);
    }

    #[test]
    fn yaw_rate_is_damped() {
        let c = ctrl();
        let undamped = c.update(0.0, 10.0, 0.0, 5.0).rudder;
        let damped = c.update(0.0, 10.0, 3.0, 5.0).rudder;
        assert!(damped < undamped, "undamped={} damped={}", undamped, damped);
    }

    #[test]
    fn rudder_saturates_at_plus_minus_one() {
        let c = ctrl();
        let very_right = c.update(-500.0, 100.0, 0.0, 5.0).rudder;
        let very_left = c.update(500.0, -100.0, 0.0, 5.0).rudder;
        assert!((very_right - 1.0).abs() < 1e-9);
        assert!((very_left - -1.0).abs() < 1e-9);
    }

    #[test]
    fn opposing_crosstrack_and_heading_terms_partially_cancel() {
        let c = ctrl();
        let both = c.update(-10.0, -10.0, 0.0, 5.0).rudder;
        let only_crosstrack = c.update(-10.0, 0.0, 0.0, 5.0).rudder;
        assert!(both.abs() < only_crosstrack.abs(), "both={}", both);
    }

    #[test]
    fn pivot_brake_engages_at_low_speed_with_saturated_rudder() {
        let c = ctrl();
        // Huge heading error saturates rudder to +1.0. At ground speed 2 kt
        // (the "full authority" knee), pivot_brake hits its max magnitude.
        let cmd = c.update(0.0, 100.0, 0.0, 2.0);
        assert!((cmd.rudder - 1.0).abs() < 1e-9);
        assert!(
            (cmd.pivot_brake - c.pivot_max_brake).abs() < 1e-9,
            "pivot_brake={}",
            cmd.pivot_brake
        );
    }

    #[test]
    fn pivot_brake_sign_follows_rudder() {
        let c = ctrl();
        let right = c.update(0.0, 100.0, 0.0, 2.0); // saturate right
        let left = c.update(0.0, -100.0, 0.0, 2.0); // saturate left
        assert!(right.pivot_brake > 0.0);
        assert!(left.pivot_brake < 0.0);
    }

    #[test]
    fn pivot_brake_fades_out_above_high_speed_threshold() {
        let c = ctrl();
        // Rudder saturated but ground speed above fade-out point —
        // pivot_brake must be zero so we don't slow the aircraft on
        // straight high-speed taxi.
        let cmd = c.update(0.0, 100.0, 0.0, 8.0);
        assert!((cmd.rudder - 1.0).abs() < 1e-9);
        assert_eq!(cmd.pivot_brake, 0.0);
    }

    #[test]
    fn pivot_brake_quiet_below_rudder_threshold() {
        let c = ctrl();
        // 3° heading error → rudder 0.15, well below the 0.7 pivot
        // threshold — no pivot brake even at low speed.
        let cmd = c.update(0.0, 3.0, 0.0, 2.0);
        assert!(cmd.rudder.abs() < 0.7);
        assert_eq!(cmd.pivot_brake, 0.0);
    }
}
