//! L1 path follower + direct-to.

use crate::types::{
    clamp, course_between, wrap_degrees_180, wrap_degrees_360, AircraftState, StraightLeg, Vec2,
    Waypoint, KT_TO_FPS,
};

pub fn leg_projection(state: &AircraftState, leg: StraightLeg) -> (f64, f64, f64) {
    let path = leg.end_ft - leg.start_ft;
    let rel = state.position_ft - leg.start_ft;
    let path_length = path.length().max(1.0);
    let path_unit = path.normalized();
    let along_track = rel.dot(path_unit);
    let cross_track = -path_unit.cross(rel);
    (along_track, cross_track, path_length)
}

pub fn distance_to_leg_segment(state: &AircraftState, leg: StraightLeg) -> f64 {
    let (along, cross, length) = leg_projection(state, leg);
    if along < 0.0 {
        (state.position_ft - leg.start_ft).length()
    } else if along > length {
        (state.position_ft - leg.end_ft).length()
    } else {
        cross.abs()
    }
}

#[derive(Debug, Clone)]
pub struct L1PathFollower {
    pub lookahead_time_s: f64,
    pub bank_gain: f64,
    pub max_cross_track_ft: f64,
}

impl L1PathFollower {
    pub fn new() -> Self {
        Self {
            lookahead_time_s: 10.0,
            bank_gain: 1.6,
            max_cross_track_ft: 2500.0,
        }
    }

    /// Returns `(desired_track_deg, bank_cmd_deg)` for following `leg`.
    pub fn follow_leg(
        &self,
        state: &AircraftState,
        leg: StraightLeg,
        max_bank_deg: f64,
    ) -> (f64, f64) {
        let (along_track_ft, cross_track_ft, _) = leg_projection(state, leg);
        if cross_track_ft.abs() > self.max_cross_track_ft {
            let desired_track = course_between(state.position_ft, leg.start_ft);
            let track_error = wrap_degrees_180(desired_track - state.track_deg);
            let bank_cmd = clamp(track_error * self.bank_gain, -max_bank_deg, max_bank_deg);
            return (desired_track, bank_cmd);
        }
        let ground_speed_ft_s = (state.gs_kt * KT_TO_FPS).max(55.0);
        let lookahead_ft = (ground_speed_ft_s * self.lookahead_time_s).max(800.0);
        let course_deg = course_between(leg.start_ft, leg.end_ft);
        let intercept_deg = cross_track_ft.atan2(lookahead_ft).to_degrees();
        let mut desired_track_deg = wrap_degrees_360(course_deg - intercept_deg);
        let mut track_error_deg = wrap_degrees_180(desired_track_deg - state.track_deg);
        let mut bank_cmd_deg = clamp(
            track_error_deg * self.bank_gain - clamp(cross_track_ft / 180.0, -8.0, 8.0),
            -max_bank_deg,
            max_bank_deg,
        );
        if along_track_ft < -200.0 {
            desired_track_deg = course_between(state.position_ft, leg.start_ft);
            track_error_deg = wrap_degrees_180(desired_track_deg - state.track_deg);
            bank_cmd_deg = clamp(track_error_deg * self.bank_gain, -max_bank_deg, max_bank_deg);
        }
        let _ = Vec2::ZERO; // keep Vec2 in scope for potential debug callers
        (desired_track_deg, bank_cmd_deg)
    }

    pub fn direct_to(
        &self,
        state: &AircraftState,
        waypoint: &Waypoint,
        max_bank_deg: f64,
    ) -> (f64, f64) {
        let desired_track = course_between(state.position_ft, waypoint.position_ft);
        let track_error = wrap_degrees_180(desired_track - state.track_deg);
        let bank_cmd = clamp(track_error * self.bank_gain, -max_bank_deg, max_bank_deg);
        (desired_track, bank_cmd)
    }
}

impl Default for L1PathFollower {
    fn default() -> Self {
        Self::new()
    }
}
