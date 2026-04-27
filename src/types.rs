//! Core numeric + mission types shared across the pilot layers.
//!
//! Units stay in feet, knots, and degrees throughout; the sim backend
//! converts at the edges.

use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, Mul, Sub};

pub const NM_TO_FT: f64 = 6076.12;
pub const KT_TO_FPS: f64 = 1.687_809_857_101_195_7;
pub const G_FT_S2: f64 = 32.174;
pub const EARTH_RADIUS_M: f64 = 6_371_008.8;

pub fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let p1 = lat1.to_radians();
    let p2 = lat2.to_radians();
    let dphi = (lat2 - lat1).to_radians();
    let dlam = (lon2 - lon1).to_radians();
    let a = (dphi * 0.5).sin().powi(2) + p1.cos() * p2.cos() * (dlam * 0.5).sin().powi(2);
    2.0 * EARTH_RADIUS_M * a.sqrt().asin()
}

pub fn initial_bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let p1 = lat1.to_radians();
    let p2 = lat2.to_radians();
    let dlam = (lon2 - lon1).to_radians();
    let x = dlam.sin() * p2.cos();
    let y = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dlam.cos();
    x.atan2(y).to_degrees().rem_euclid(360.0)
}

#[inline]
pub fn clamp(value: f64, lower: f64, upper: f64) -> f64 {
    if value < lower {
        lower
    } else if value > upper {
        upper
    } else {
        value
    }
}

pub fn wrap_degrees_360(angle_deg: f64) -> f64 {
    let m = angle_deg.rem_euclid(360.0);
    if m == 360.0 { 0.0 } else { m }
}

pub fn wrap_degrees_180(angle_deg: f64) -> f64 {
    let wrapped = wrap_degrees_360(angle_deg + 180.0) - 180.0;
    if wrapped == -180.0 { 180.0 } else { wrapped }
}

pub fn heading_to_vector(heading_deg: f64, magnitude: f64) -> Vec2 {
    let r = heading_deg.to_radians();
    Vec2::new(r.sin() * magnitude, r.cos() * magnitude)
}

pub fn vector_to_heading(v: Vec2) -> f64 {
    if v.length() <= 1e-9 {
        return 0.0;
    }
    wrap_degrees_360(v.x.atan2(v.y).to_degrees())
}

pub fn course_between(start_ft: Vec2, end_ft: Vec2) -> f64 {
    vector_to_heading(end_ft - start_ft)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FlightPhase {
    Preflight,
    TakeoffRoll,
    Rotate,
    InitialClimb,
    Crosswind,
    EnrouteClimb,
    Cruise,
    Descent,
    PatternEntry,
    Downwind,
    Base,
    Final,
    Roundout,
    Flare,
    Rollout,
    /// Between Rollout and TaxiClear: the aircraft has left the landing
    /// runway's surface and is on the exit taxiway but not yet past the
    /// hold-short line. Rollout still owns brakes/rudder; once fully
    /// clear, PatternFlyProfile hands off and the phase advances to
    /// TaxiClear.
    RunwayExit,
    TaxiClear,
    GoAround,
}

impl FlightPhase {
    pub fn value(self) -> &'static str {
        match self {
            FlightPhase::Preflight => "preflight",
            FlightPhase::TakeoffRoll => "takeoff_roll",
            FlightPhase::Rotate => "rotate",
            FlightPhase::InitialClimb => "initial_climb",
            FlightPhase::Crosswind => "crosswind",
            FlightPhase::EnrouteClimb => "enroute_climb",
            FlightPhase::Cruise => "cruise",
            FlightPhase::Descent => "descent",
            FlightPhase::PatternEntry => "pattern_entry",
            FlightPhase::Downwind => "downwind",
            FlightPhase::Base => "base",
            FlightPhase::Final => "final",
            FlightPhase::Roundout => "roundout",
            FlightPhase::Flare => "flare",
            FlightPhase::Rollout => "rollout",
            FlightPhase::RunwayExit => "runway_exit",
            FlightPhase::TaxiClear => "taxi_clear",
            FlightPhase::GoAround => "go_around",
        }
    }

    #[allow(clippy::should_implement_trait)]
    pub fn from_str(s: &str) -> Option<FlightPhase> {
        Some(match s {
            "preflight" => FlightPhase::Preflight,
            "takeoff_roll" => FlightPhase::TakeoffRoll,
            "rotate" => FlightPhase::Rotate,
            "initial_climb" => FlightPhase::InitialClimb,
            "crosswind" => FlightPhase::Crosswind,
            "enroute_climb" => FlightPhase::EnrouteClimb,
            "cruise" => FlightPhase::Cruise,
            "descent" => FlightPhase::Descent,
            "pattern_entry" => FlightPhase::PatternEntry,
            "downwind" => FlightPhase::Downwind,
            "base" => FlightPhase::Base,
            "final" => FlightPhase::Final,
            "roundout" => FlightPhase::Roundout,
            "flare" => FlightPhase::Flare,
            "rollout" => FlightPhase::Rollout,
            "runway_exit" => FlightPhase::RunwayExit,
            "taxi_clear" => FlightPhase::TaxiClear,
            "go_around" => FlightPhase::GoAround,
            _ => return None,
        })
    }

    pub fn all() -> &'static [FlightPhase] {
        &[
            FlightPhase::Preflight,
            FlightPhase::TakeoffRoll,
            FlightPhase::Rotate,
            FlightPhase::InitialClimb,
            FlightPhase::Crosswind,
            FlightPhase::EnrouteClimb,
            FlightPhase::Cruise,
            FlightPhase::Descent,
            FlightPhase::PatternEntry,
            FlightPhase::Downwind,
            FlightPhase::Base,
            FlightPhase::Final,
            FlightPhase::Roundout,
            FlightPhase::Flare,
            FlightPhase::Rollout,
            FlightPhase::RunwayExit,
            FlightPhase::TaxiClear,
            FlightPhase::GoAround,
        ]
    }
}

impl fmt::Display for FlightPhase {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.value())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrafficSide {
    Left,
    Right,
}

impl TrafficSide {
    #[allow(clippy::should_implement_trait)]
    pub fn from_str(s: &str) -> Option<TrafficSide> {
        match s.to_ascii_lowercase().as_str() {
            "left" => Some(TrafficSide::Left),
            "right" => Some(TrafficSide::Right),
            _ => None,
        }
    }

    pub fn sign(self) -> f64 {
        match self {
            TrafficSide::Left => -1.0,
            TrafficSide::Right => 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LateralMode {
    BankHold,
    TrackHold,
    PathFollow,
    CenterlineIntercept,
    RolloutCenterline,
    /// Ground taxi lateral control: follow `target_path` (leg start→end in
    /// runway-frame feet) via `NoseWheelController` on rudder; aileron is
    /// locked to zero.
    TaxiFollow,
    /// Ground taxi pose target: steer toward `target_waypoint.position_ft`
    /// until close, then pivot to match `target_heading_deg`. Used for
    /// precise stop-and-face-direction endings (hold-short, parking
    /// spot). No crosstrack — this isn't tracking a line.
    TaxiPose,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VerticalMode {
    PitchHold,
    AltitudeHold,
    Tecs,
    GlidepathTrack,
    FlareTrack,
    /// Ground taxi: `GroundSpeedController` drives throttle and brake from
    /// `target_speed_kt`. Elevator is held to zero pitch.
    Taxi,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };

    pub fn dot(self, other: Vec2) -> f64 {
        self.x * other.x + self.y * other.y
    }

    pub fn cross(self, other: Vec2) -> f64 {
        self.x * other.y - self.y * other.x
    }

    pub fn length(self) -> f64 {
        self.x.hypot(self.y)
    }

    pub fn normalized(self) -> Vec2 {
        let m = self.length();
        if m <= 1e-9 {
            Vec2::ZERO
        } else {
            Vec2::new(self.x / m, self.y / m)
        }
    }

    pub fn distance_to(self, other: Vec2) -> f64 {
        (self - other).length()
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Mul<f64> for Vec2 {
    type Output = Vec2;
    fn mul(self, s: f64) -> Vec2 {
        Vec2::new(self.x * s, self.y * s)
    }
}

impl Mul<Vec2> for f64 {
    type Output = Vec2;
    fn mul(self, v: Vec2) -> Vec2 {
        v * self
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Waypoint {
    pub name: String,
    pub position_ft: Vec2,
    pub altitude_ft: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Runway {
    pub id: Option<String>,
    /// Runway-frame origin in world-frame feet. Always set to the
    /// **pavement end** — the lat/lon apt.dat row 100 gives for the
    /// runway end — NOT the displaced landing threshold. That keeps
    /// the runway frame consistent between takeoff (which uses the
    /// full runway from x=0) and landing (which aims past the
    /// displaced threshold via `touchdown_runway_x_ft`).
    pub threshold_ft: Vec2,
    pub course_deg: f64,
    pub length_ft: f64,
    pub touchdown_zone_ft: f64,
    /// Displacement (feet) from the pavement end to the painted landing
    /// threshold. Non-zero when apt.dat row 100 encodes a displaced
    /// threshold — the aircraft must touch down past this point on
    /// landing, but can use the full runway for takeoff.
    pub displaced_threshold_ft: f64,
    pub traffic_side: TrafficSide,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StraightLeg {
    pub start_ft: Vec2,
    pub end_ft: Vec2,
}

/// Explicit position + heading target for the terminal phase of a taxi
/// (e.g. the hold-short line — stop at P, nose pointed at the runway).
/// Different object from `StraightLeg`: a leg is a path to follow, a
/// pose is a state to reach.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TaxiPose {
    pub position_ft: Vec2,
    pub heading_deg: f64,
}

impl StraightLeg {
    pub fn course_deg(&self) -> f64 {
        course_between(self.start_ft, self.end_ft)
    }

    /// Signed perpendicular distance from `pos_ft` to this leg, in feet.
    /// Positive means right of the leg's direction of travel; negative
    /// means left. Returns `None` for a degenerate (zero-length) leg.
    pub fn crosstrack_ft(&self, pos_ft: Vec2) -> Option<f64> {
        let leg_vec = self.end_ft - self.start_ft;
        let leg_len = leg_vec.length();
        if leg_len < 1e-6 {
            return None;
        }
        let leg_dir = leg_vec * (1.0 / leg_len);
        let leg_right = Vec2::new(leg_dir.y, -leg_dir.x);
        let offset = pos_ft - self.start_ft;
        Some(offset.dot(leg_right))
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Glidepath {
    pub slope_deg: f64,
    pub threshold_crossing_height_ft: f64,
    pub aimpoint_ft_from_threshold: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct GuidanceTargets {
    pub lateral_mode: LateralMode,
    pub vertical_mode: VerticalMode,
    pub target_bank_deg: Option<f64>,
    pub target_heading_deg: Option<f64>,
    pub target_track_deg: Option<f64>,
    pub target_path: Option<StraightLeg>,
    pub target_waypoint: Option<Waypoint>,
    pub target_altitude_ft: Option<f64>,
    pub target_speed_kt: Option<f64>,
    pub target_pitch_deg: Option<f64>,
    pub glidepath: Option<Glidepath>,
    pub throttle_limit: Option<(f64, f64)>,
    pub flaps_cmd: Option<i32>,
    pub gear_down: Option<bool>,
    pub brakes: f64,
    pub tecs_phase_override: Option<FlightPhase>,
}

impl Default for GuidanceTargets {
    fn default() -> Self {
        Self {
            lateral_mode: LateralMode::BankHold,
            vertical_mode: VerticalMode::PitchHold,
            target_bank_deg: None,
            target_heading_deg: None,
            target_track_deg: None,
            target_path: None,
            target_waypoint: None,
            target_altitude_ft: None,
            target_speed_kt: None,
            target_pitch_deg: None,
            glidepath: None,
            throttle_limit: None,
            flaps_cmd: None,
            gear_down: None,
            brakes: 0.0,
            tecs_phase_override: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ActuatorCommands {
    pub aileron: f64,
    pub elevator: f64,
    pub rudder: f64,
    pub throttle: f64,
    pub flaps: Option<i32>,
    pub gear_down: Option<bool>,
    /// Symmetric brake applied to both mains. Used for speed control
    /// (ground-speed overshoot), runway rollout, and the parked-stop hold.
    pub brakes: f64,
    /// Asymmetric component layered on top of `brakes` for tight-turn
    /// differential braking: positive = extra right-main, negative = extra
    /// left-main. Wheels resolve as
    ///   left_wheel  = clamp(brakes - pivot_brake, 0, 1)
    ///   right_wheel = clamp(brakes + pivot_brake, 0, 1)
    /// so differential authority survives a full symmetric hold brake
    /// (without this, hold_brake = 1.0 saturates both wheels at 1.0 and
    /// pivot disappears, which breaks pivot-in-place at pose targets).
    /// Coupled to the nose-wheel controller — kicks in when the rudder is
    /// saturated and the aircraft is below ~5 kt, where nose-wheel alone
    /// can't produce a tight-enough pivot on a C172.
    pub pivot_brake: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct AircraftState {
    pub t_sim: f64,
    pub dt: f64,
    pub position_ft: Vec2,
    pub alt_msl_ft: f64,
    pub alt_agl_ft: f64,
    pub pitch_deg: f64,
    pub roll_deg: f64,
    pub heading_deg: f64,
    pub track_deg: f64,
    pub p_rad_s: f64,
    pub q_rad_s: f64,
    pub r_rad_s: f64,
    pub ias_kt: f64,
    pub tas_kt: f64,
    pub gs_kt: f64,
    pub vs_fpm: f64,
    pub ground_velocity_ft_s: Vec2,
    pub flap_index: i32,
    pub gear_down: bool,
    pub on_ground: bool,
    pub throttle_pos: f64,
    pub runway_id: Option<String>,
    pub runway_dist_remaining_ft: Option<f64>,
    pub runway_x_ft: Option<f64>,
    pub runway_y_ft: Option<f64>,
    pub centerline_error_ft: Option<f64>,
    pub threshold_abeam: bool,
    pub distance_to_touchdown_ft: Option<f64>,
    pub stall_margin: f64,
}

impl AircraftState {
    /// Builder-style factory for tests: produces a plausible mid-pattern
    /// default that callers override selectively.
    pub fn synthetic_default() -> Self {
        AircraftState {
            t_sim: 0.0,
            dt: 0.2,
            position_ft: Vec2::ZERO,
            alt_msl_ft: 1500.0,
            alt_agl_ft: 1000.0,
            pitch_deg: 0.0,
            roll_deg: 0.0,
            heading_deg: 0.0,
            track_deg: 0.0,
            p_rad_s: 0.0,
            q_rad_s: 0.0,
            r_rad_s: 0.0,
            ias_kt: 80.0,
            tas_kt: 80.0,
            gs_kt: 80.0,
            vs_fpm: 0.0,
            ground_velocity_ft_s: heading_to_vector(0.0, 80.0 * KT_TO_FPS),
            flap_index: 0,
            gear_down: true,
            on_ground: false,
            throttle_pos: 0.5,
            runway_id: Some("36".to_string()),
            runway_dist_remaining_ft: None,
            runway_x_ft: Some(0.0),
            runway_y_ft: Some(0.0),
            centerline_error_ft: Some(0.0),
            threshold_abeam: false,
            distance_to_touchdown_ft: Some(2000.0),
            stall_margin: 1.5,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_degrees_180_handles_negative_180() {
        assert_eq!(wrap_degrees_180(-180.0), 180.0);
    }

    #[test]
    fn wrap_degrees_180_wraps_into_band() {
        assert_eq!(wrap_degrees_180(190.0), -170.0);
        assert_eq!(wrap_degrees_180(-190.0), 170.0);
    }

    #[test]
    fn heading_to_vector_matches_compass_conventions() {
        let v = heading_to_vector(90.0, 1.0);
        assert!((v.x - 1.0).abs() < 1e-9);
        assert!(v.y.abs() < 1e-9);
    }
}
