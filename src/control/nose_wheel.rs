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
    pub fn update(
        &self,
        crosstrack_ft: f64,
        heading_error_deg: f64,
        yaw_rate_deg_s: f64,
    ) -> f64 {
        let lateral = -crosstrack_ft * self.kp_crosstrack;
        let heading = heading_error_deg * self.kp_heading;
        let damping = -yaw_rate_deg_s * self.kp_yaw_rate;
        clamp(lateral + heading + damping, -1.0, 1.0)
    }
}

impl Default for NoseWheelController {
    fn default() -> Self {
        Self {
            // Crosstrack and heading gains sized to saturate rudder at
            // ~12 ft crosstrack or ~20° of heading error so the nose wheel
            // actually pulls onto the leg from a perpendicular pullout.
            //
            // kp_yaw_rate 0.08 damps turns without reversing them — a
            // healthy PD ratio here is ~0.5 × (peak yaw rate × P term at
            // typical error). At 30° heading error the P term is 1.5; at
            // a 10°/s yaw rate the damping is 0.8 — net rudder is a
            // moderate +0.7, i.e. we're still turning but the peak rate
            // is blunted. An earlier tuning (0.25) swung the output
            // negative as soon as yaw rate passed ~6°/s, which produced
            // bang-bang behaviour and visible overshoot on live taxi.
            kp_crosstrack: 0.08,
            kp_heading: 0.05,
            kp_yaw_rate: 0.08,
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
    fn zero_error_produces_zero_rudder() {
        let c = ctrl();
        assert_eq!(c.update(0.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn right_of_centerline_commands_left_rudder() {
        let c = ctrl();
        let r = c.update(10.0, 0.0, 0.0);
        assert!(r < 0.0, "r={}", r);
    }

    #[test]
    fn leg_bearing_clockwise_of_heading_commands_right_rudder() {
        let c = ctrl();
        let r = c.update(0.0, 10.0, 0.0);
        assert!(r > 0.0, "r={}", r);
    }

    #[test]
    fn yaw_rate_is_damped() {
        let c = ctrl();
        // Right-rudder command from heading error, damped by a right yaw rate.
        let undamped = c.update(0.0, 10.0, 0.0);
        let damped = c.update(0.0, 10.0, 3.0);
        assert!(damped < undamped, "undamped={} damped={}", undamped, damped);
    }

    #[test]
    fn rudder_saturates_at_plus_minus_one() {
        let c = ctrl();
        let very_right = c.update(-500.0, 100.0, 0.0);
        let very_left = c.update(500.0, -100.0, 0.0);
        assert!((very_right - 1.0).abs() < 1e-9);
        assert!((very_left - -1.0).abs() < 1e-9);
    }

    #[test]
    fn opposing_crosstrack_and_heading_terms_partially_cancel() {
        let c = ctrl();
        // 10 ft left of centerline (need right rudder) + 10° heading
        // overshot to the right (need left rudder) should partially cancel.
        let both = c.update(-10.0, -10.0, 0.0);
        let only_crosstrack = c.update(-10.0, 0.0, 0.0);
        assert!(both.abs() < only_crosstrack.abs(), "both={}", both);
    }
}
