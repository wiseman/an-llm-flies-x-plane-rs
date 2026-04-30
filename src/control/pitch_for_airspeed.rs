//! Pitch-for-airspeed controller. Drives commanded pitch purely from
//! airspeed error so the aircraft can hold a target IAS when there is no
//! throttle authority left to spend (engine-out, dead-stick).
//!
//! Sign convention: too fast → pitch up to bleed speed; too slow →
//! pitch down to gain speed. The pitch trim biases the steady-state
//! attitude toward a typical glide pitch so the controller doesn't have
//! to lean entirely on the integrator at equilibrium.

use crate::types::clamp;

#[derive(Debug, Clone)]
pub struct PitchForAirspeedController {
    /// Proportional gain, deg of pitch per kt of `(ias - target)` error.
    pub kp: f64,
    /// Integral gain, deg per (kt · s) of accumulated error.
    pub ki: f64,
    /// Steady-state pitch bias, deg. ~ the natural glide pitch at vbg.
    pub trim_deg: f64,
    pub integrator_limit: f64,
    /// Pitch authority bounds, deg. Default lower bound is -12°: a
    /// C172 at flap 30°/vbg in dead-stick glide settles around -10°,
    /// and clamping shallower leaves the controller saturated, IAS
    /// bleeds below vbg, and induced drag pins the aircraft in a
    /// slow-descent regime it can't recover from.
    pub pitch_limit: (f64, f64),
    pub integrator: f64,
}

impl PitchForAirspeedController {
    pub fn new() -> Self {
        Self {
            kp: 0.6,
            ki: 0.05,
            trim_deg: -1.5,
            integrator_limit: 60.0,
            pitch_limit: (-12.0, 12.0),
            integrator: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integrator = 0.0;
    }

    /// Returns commanded pitch in degrees.
    pub fn update(&mut self, target_speed_kt: f64, ias_kt: f64, dt: f64) -> f64 {
        if dt <= 0.0 {
            return clamp(self.trim_deg, self.pitch_limit.0, self.pitch_limit.1);
        }
        let speed_excess_kt = ias_kt - target_speed_kt;
        let next_int = clamp(
            self.integrator + speed_excess_kt * dt,
            -self.integrator_limit,
            self.integrator_limit,
        );
        let unclamped = self.trim_deg + self.kp * speed_excess_kt + self.ki * next_int;
        let clamped = clamp(unclamped, self.pitch_limit.0, self.pitch_limit.1);
        let saturated_high = unclamped > self.pitch_limit.1 && speed_excess_kt > 0.0;
        let saturated_low = unclamped < self.pitch_limit.0 && speed_excess_kt < 0.0;
        if !saturated_high && !saturated_low {
            self.integrator = next_int;
        }
        clamped
    }
}

impl Default for PitchForAirspeedController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn too_fast_pitches_up() {
        let mut c = PitchForAirspeedController::new();
        let pitch = c.update(68.0, 90.0, 0.2);
        assert!(pitch > 5.0, "expected pitch up, got {pitch}");
    }

    #[test]
    fn too_slow_pitches_down() {
        let mut c = PitchForAirspeedController::new();
        let pitch = c.update(68.0, 55.0, 0.2);
        assert!(pitch < -3.0, "expected pitch down, got {pitch}");
    }

    #[test]
    fn at_target_returns_trim() {
        let mut c = PitchForAirspeedController::new();
        let pitch = c.update(68.0, 68.0, 0.2);
        assert!((pitch - c.trim_deg).abs() < 0.1, "got {pitch}");
    }

    #[test]
    fn anti_windup_freezes_integrator_when_saturated() {
        let mut c = PitchForAirspeedController::new();
        for _ in 0..2000 {
            c.update(68.0, 120.0, 0.2);
        }
        assert!(
            c.integrator.abs() < c.integrator_limit,
            "integrator wound up to {} (limit {})",
            c.integrator,
            c.integrator_limit
        );
    }
}
