//! Guidance profiles composed by `PilotCore`. Mirrors core/profiles.py.
//!
//! Each profile owns a set of axes (lateral / vertical / speed) and emits a
//! `ProfileContribution` per tick. `PilotCore` merges per-axis contributions
//! from all active profiles into a single `GuidanceTargets`. The LLM engages
//! profiles; axis-ownership conflicts auto-displace older profiles.
//!
//! Unlike the Python version where `PatternFlyProfile.contribute()` can
//! directly call `pilot.engage_profile(...)` to swap itself for three
//! single-axis holds mid-tick, Rust profiles return a `hand_off` request and
//! `PilotCore` applies the swap after all contributions are collected. That
//! avoids reentrant `&mut` borrows on the pilot from inside one of its
//! profiles.

use std::collections::BTreeSet;

use crate::config::ConfigBundle;
use crate::core::mode_manager::{
    ModeManager, ModeManagerUpdate, PatternClearances, PatternTriggers,
};
use crate::core::safety_monitor::SafetyMonitor;
use crate::guidance::lateral::L1PathFollower;
use crate::guidance::pattern_manager::{
    build_pattern_geometry, glidepath_target_altitude_ft_default, PatternGeometry,
};
use crate::guidance::route_manager::RouteManager;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::types::{
    clamp, wrap_degrees_180, wrap_degrees_360, AircraftState, FlightPhase, Glidepath,
    GuidanceTargets, LateralMode, StraightLeg, TrafficSide, VerticalMode, Waypoint,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Axis {
    Lateral,
    Vertical,
    Speed,
}

#[derive(Debug, Clone, Default)]
pub struct ProfileContribution {
    pub lateral_mode: Option<LateralMode>,
    pub vertical_mode: Option<VerticalMode>,
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
    pub brakes: Option<f64>,
    pub tecs_phase_override: Option<FlightPhase>,
}

/// Returned by each profile's `contribute()`.
pub struct ProfileTick {
    pub contribution: ProfileContribution,
    /// If set, `PilotCore` will engage these profiles after all contributions
    /// are merged — used by `PatternFlyProfile`'s handoff to the three holds.
    pub hand_off: Option<Vec<Box<dyn GuidanceProfile>>>,
}

impl ProfileTick {
    pub fn contribution_only(c: ProfileContribution) -> Self {
        Self { contribution: c, hand_off: None }
    }
}

pub trait GuidanceProfile: Send {
    fn name(&self) -> &'static str;
    fn owns(&self) -> BTreeSet<Axis>;
    fn contribute(&mut self, state: &AircraftState, dt: f64) -> ProfileTick;

    /// For `PatternFlyProfile` observers that want to expose extra metadata on
    /// the `StatusSnapshot`. Default: no extras.
    fn pattern_metadata(&self) -> Option<PatternMetadata> {
        None
    }

    /// Report that the profile has completed its mission — the airplane
    /// has finished taxiing to the hold-short, the line-up is parked on
    /// the centerline, etc. Used by `HeartbeatPump` to wake the LLM the
    /// moment an automated sequence finishes, instead of waiting out the
    /// idle heartbeat interval. The profile stays engaged (holding the
    /// brake, etc.); only the completion edge is a signal. Default false.
    fn is_complete(&self) -> bool {
        false
    }

    /// Optional one-line debug string surfaced in the status pane. Profiles
    /// with non-trivial internal state (e.g. `TaxiProfile`'s leg sequencer)
    /// use this to expose the values a human would need to diagnose why the
    /// aircraft is doing what it is — current leg index, crosstrack /
    /// heading-error feeding the controllers, target speed, distance to the
    /// upcoming node. Default: no debug line (most profiles have all their
    /// state in the already-rendered guidance targets).
    fn debug_line(&self, _state: &AircraftState) -> Option<String> {
        None
    }

    /// Short, single-line suffix appended to the profile's name in the
    /// TUI mode line (e.g. "taxi  via A→B · leg 3/8 · 80ft"). Default:
    /// no suffix. Must not include profile-change signals — the
    /// heartbeat diffs active profile names, not these summaries, so
    /// suffix churn is fine.
    fn mode_line_suffix(&self, _state: &AircraftState) -> Option<String> {
        None
    }
}

#[derive(Debug, Clone)]
pub struct PatternMetadata {
    pub last_go_around_reason: Option<String>,
    pub airport_ident: Option<String>,
    pub runway_id: Option<String>,
    pub field_elevation_ft: Option<f64>,
    pub phase: FlightPhase,
}

// ---------- idle profiles ----------

pub struct IdleLateralProfile;
impl GuidanceProfile for IdleLateralProfile {
    fn name(&self) -> &'static str { "idle_lateral" }
    fn owns(&self) -> BTreeSet<Axis> { single(Axis::Lateral) }
    fn contribute(&mut self, _: &AircraftState, _: f64) -> ProfileTick {
        ProfileTick::contribution_only(ProfileContribution {
            lateral_mode: Some(LateralMode::BankHold),
            target_bank_deg: Some(0.0),
            ..Default::default()
        })
    }
}

pub struct IdleVerticalProfile;
impl GuidanceProfile for IdleVerticalProfile {
    fn name(&self) -> &'static str { "idle_vertical" }
    fn owns(&self) -> BTreeSet<Axis> { single(Axis::Vertical) }
    fn contribute(&mut self, _: &AircraftState, _: f64) -> ProfileTick {
        ProfileTick::contribution_only(ProfileContribution {
            vertical_mode: Some(VerticalMode::PitchHold),
            target_pitch_deg: Some(0.0),
            throttle_limit: Some((0.0, 0.0)),
            ..Default::default()
        })
    }
}

pub struct IdleSpeedProfile {
    pub default_speed_kt: f64,
}
impl IdleSpeedProfile {
    pub fn new(default_speed_kt: f64) -> Self { Self { default_speed_kt } }
}
impl GuidanceProfile for IdleSpeedProfile {
    fn name(&self) -> &'static str { "idle_speed" }
    fn owns(&self) -> BTreeSet<Axis> { single(Axis::Speed) }
    fn contribute(&mut self, _: &AircraftState, _: f64) -> ProfileTick {
        ProfileTick::contribution_only(ProfileContribution {
            target_speed_kt: Some(self.default_speed_kt),
            ..Default::default()
        })
    }
}

// ---------- single-axis holds ----------

pub struct HeadingHoldProfile {
    pub heading_deg: f64,
    pub max_bank_deg: f64,
    pub direction_lock: Option<String>,
}

impl HeadingHoldProfile {
    pub fn new(heading_deg: f64, max_bank_deg: f64, turn_direction: Option<&str>) -> Result<Self, String> {
        let normalized = turn_direction.and_then(|d| {
            let lower = d.to_ascii_lowercase();
            if lower.is_empty() { None } else { Some(lower) }
        });
        if let Some(d) = &normalized {
            if d != "left" && d != "right" {
                return Err(format!("turn_direction must be None, 'left', or 'right'; got {:?}", d));
            }
        }
        Ok(Self {
            heading_deg: heading_deg.rem_euclid(360.0),
            max_bank_deg,
            direction_lock: normalized,
        })
    }
}

impl GuidanceProfile for HeadingHoldProfile {
    fn name(&self) -> &'static str { "heading_hold" }
    fn owns(&self) -> BTreeSet<Axis> { single(Axis::Lateral) }
    fn contribute(&mut self, state: &AircraftState, _dt: f64) -> ProfileTick {
        let raw_error = (self.heading_deg - state.track_deg).rem_euclid(360.0);
        let short_error = if raw_error > 180.0 { raw_error - 360.0 } else { raw_error };
        if short_error.abs() < 5.0 {
            self.direction_lock = None;
        }
        let effective_error = match self.direction_lock.as_deref() {
            Some("right") => if raw_error > 0.0 { raw_error } else { 0.0 },
            Some("left") => if raw_error > 0.0 { raw_error - 360.0 } else { 0.0 },
            _ => short_error,
        };
        let target_bank = clamp(effective_error * 0.35, -self.max_bank_deg, self.max_bank_deg);
        ProfileTick::contribution_only(ProfileContribution {
            lateral_mode: Some(LateralMode::TrackHold),
            target_track_deg: Some(self.heading_deg),
            target_heading_deg: Some(self.heading_deg),
            target_bank_deg: Some(target_bank),
            ..Default::default()
        })
    }
}


pub struct AltitudeHoldProfile {
    pub altitude_ft: f64,
}

impl AltitudeHoldProfile {
    pub fn new(altitude_ft: f64) -> Self { Self { altitude_ft } }
}

impl GuidanceProfile for AltitudeHoldProfile {
    fn name(&self) -> &'static str { "altitude_hold" }
    fn owns(&self) -> BTreeSet<Axis> { single(Axis::Vertical) }
    fn contribute(&mut self, _state: &AircraftState, _dt: f64) -> ProfileTick {
        // Let TECS command the throttle it needs within the physical
        // envelope. A previous revision switched between three discrete
        // throttle regimes at ±150 ft of altitude error, each with its
        // own phase-override trim value — classic hysteresis-free mode
        // switching that limit-cycles when steady-state authority sits
        // near a threshold.
        let c = ProfileContribution {
            vertical_mode: Some(VerticalMode::Tecs),
            target_altitude_ft: Some(self.altitude_ft),
            throttle_limit: Some((0.0, 1.0)),
            ..Default::default()
        };
        ProfileTick::contribution_only(c)
    }
}

pub struct SpeedHoldProfile {
    pub speed_kt: f64,
}

impl SpeedHoldProfile {
    pub fn new(speed_kt: f64) -> Self { Self { speed_kt } }
}

impl GuidanceProfile for SpeedHoldProfile {
    fn name(&self) -> &'static str { "speed_hold" }
    fn owns(&self) -> BTreeSet<Axis> { single(Axis::Speed) }
    fn contribute(&mut self, _: &AircraftState, _: f64) -> ProfileTick {
        ProfileTick::contribution_only(ProfileContribution {
            target_speed_kt: Some(self.speed_kt),
            ..Default::default()
        })
    }
}

// ---------- guidance builders shared by takeoff/pattern profiles ----------

pub fn build_takeoff_roll_guidance(config: &ConfigBundle, runway_frame: &RunwayFrame) -> GuidanceTargets {
    GuidanceTargets {
        lateral_mode: LateralMode::RolloutCenterline,
        vertical_mode: VerticalMode::PitchHold,
        target_bank_deg: Some(0.0),
        target_heading_deg: Some(runway_frame.runway.course_deg),
        target_pitch_deg: Some(0.0),
        target_speed_kt: Some(config.performance.vr_kt),
        throttle_limit: Some((1.0, 1.0)),
        flaps_cmd: Some(10),
        ..Default::default()
    }
}

pub fn build_rotate_guidance(
    config: &ConfigBundle,
    runway_frame: &RunwayFrame,
    state: &AircraftState,
    bank_limit_deg: f64,
) -> GuidanceTargets {
    let target_course = runway_frame.runway.course_deg;
    if state.on_ground {
        return GuidanceTargets {
            lateral_mode: LateralMode::RolloutCenterline,
            vertical_mode: VerticalMode::PitchHold,
            target_bank_deg: Some(0.0),
            target_heading_deg: Some(target_course),
            target_track_deg: Some(target_course),
            target_pitch_deg: Some(8.0),
            target_speed_kt: Some(config.performance.vy_kt),
            throttle_limit: Some((1.0, 1.0)),
            ..Default::default()
        };
    }
    let track_error = wrap_degrees_180(target_course - state.track_deg);
    let target_bank = clamp(track_error * 0.35, -bank_limit_deg, bank_limit_deg);
    GuidanceTargets {
        lateral_mode: LateralMode::TrackHold,
        vertical_mode: VerticalMode::PitchHold,
        target_bank_deg: Some(target_bank),
        target_heading_deg: Some(target_course),
        target_track_deg: Some(target_course),
        target_pitch_deg: Some(8.0),
        target_speed_kt: Some(config.performance.vy_kt),
        throttle_limit: Some((1.0, 1.0)),
        ..Default::default()
    }
}

fn guidance_to_contribution(g: GuidanceTargets) -> ProfileContribution {
    ProfileContribution {
        lateral_mode: Some(g.lateral_mode),
        vertical_mode: Some(g.vertical_mode),
        target_bank_deg: g.target_bank_deg,
        target_heading_deg: g.target_heading_deg,
        target_track_deg: g.target_track_deg,
        target_path: g.target_path,
        target_waypoint: g.target_waypoint,
        target_altitude_ft: g.target_altitude_ft,
        target_speed_kt: g.target_speed_kt,
        target_pitch_deg: g.target_pitch_deg,
        glidepath: g.glidepath,
        throttle_limit: g.throttle_limit,
        flaps_cmd: g.flaps_cmd,
        gear_down: g.gear_down,
        brakes: Some(g.brakes),
        tecs_phase_override: g.tecs_phase_override,
    }
}

// ---------- takeoff profile ----------

pub struct TakeoffProfile {
    pub config: ConfigBundle,
    pub runway_frame: RunwayFrame,
    pub phase: FlightPhase,
    /// Along-track distance ahead of the aircraft at engage time —
    /// i.e. how much pavement is actually available for the takeoff roll.
    /// For a full-length departure this is `runway_frame.runway.length_ft`;
    /// for an intersection departure (e.g. "runway 19 at Charlie" at KEMT)
    /// it's the remaining distance from the intersection to the far end.
    /// Surfaced in debug output and drives the late-acceleration abort
    /// heuristic in `contribute`.
    pub usable_length_ft: f64,
    mode_manager: ModeManager,
    safety_monitor: SafetyMonitor,
    lateral_guidance: L1PathFollower,
    pattern_stub: PatternGeometry,
    route_stub: RouteManager,
    /// Along-track position at engage time — used as the zero-point for
    /// "how far have we rolled" in debug/abort math, since the aircraft
    /// may be partway down the runway for an intersection departure.
    start_along_ft: f64,
}

impl TakeoffProfile {
    pub fn new(config: ConfigBundle, runway_frame: RunwayFrame) -> Self {
        let usable = runway_frame.runway.length_ft;
        Self::new_with_usable_length(config, runway_frame, usable, 0.0)
    }

    /// Intersection-departure constructor. `usable_length_ft` is the
    /// runway distance from the aircraft's current position to the far
    /// end (full-length ≈ threshold to threshold minus current
    /// along-track). `start_along_ft` is the current along-track so the
    /// debug line can compute "rolled / remaining" honestly.
    pub fn new_with_usable_length(
        config: ConfigBundle,
        runway_frame: RunwayFrame,
        usable_length_ft: f64,
        start_along_ft: f64,
    ) -> Self {
        let pattern_stub = build_pattern_geometry(
            &runway_frame,
            config.pattern.downwind_offset_ft,
            config.pattern.default_extension_ft,
            0.0,
        );
        Self {
            mode_manager: ModeManager::new(config.clone()),
            safety_monitor: SafetyMonitor::new(config.clone()),
            lateral_guidance: L1PathFollower::new(),
            pattern_stub,
            route_stub: RouteManager::new(vec![]),
            phase: FlightPhase::Preflight,
            usable_length_ft,
            start_along_ft,
            config,
            runway_frame,
        }
    }
}

impl GuidanceProfile for TakeoffProfile {
    fn name(&self) -> &'static str { "takeoff" }
    fn owns(&self) -> BTreeSet<Axis> {
        let mut s = BTreeSet::new();
        s.insert(Axis::Lateral);
        s.insert(Axis::Vertical);
        s.insert(Axis::Speed);
        s
    }
    fn pattern_metadata(&self) -> Option<PatternMetadata> {
        Some(PatternMetadata {
            last_go_around_reason: None,
            airport_ident: None,
            runway_id: self.runway_frame.runway.id.clone(),
            field_elevation_ft: Some(self.config.airport.field_elevation_ft),
            phase: self.phase,
        })
    }
    fn contribute(&mut self, state: &AircraftState, _dt: f64) -> ProfileTick {
        let safety = self.safety_monitor.evaluate(state, self.phase);
        let empty_triggers = PatternTriggers::default();
        let default_clearances = PatternClearances::default();
        self.phase = self.mode_manager.update(&ModeManagerUpdate {
            phase: self.phase,
            state,
            route_manager: &self.route_stub,
            pattern: &self.pattern_stub,
            safety_status: &safety,
            triggers: &empty_triggers,
            clearances: &default_clearances,
            force_go_around: false,
            stay_in_pattern: false,
            touch_and_go: false,
        });
        let bank_limit = self.safety_monitor.bank_limit_deg(self.phase);
        let guidance = match self.phase {
            FlightPhase::TakeoffRoll => build_takeoff_roll_guidance(&self.config, &self.runway_frame),
            FlightPhase::Rotate => build_rotate_guidance(&self.config, &self.runway_frame, state, bank_limit),
            FlightPhase::InitialClimb => {
                // L1 path follower anchored on the extended runway
                // centerline (pavement-end -> 30_000 ft downtrack) so the
                // climbout tracks the line, not just the heading — same
                // behavior pattern_fly uses for the pattern legs, which
                // means crosstrack offset feeds into bank and crosswind
                // is handled implicitly by the resulting crab.
                let leg = self.runway_frame.departure_leg(30_000.0);
                let (desired_track, bank_cmd) =
                    self.lateral_guidance.follow_leg(state, leg, bank_limit);
                GuidanceTargets {
                    lateral_mode: LateralMode::PathFollow,
                    vertical_mode: VerticalMode::Tecs,
                    target_bank_deg: Some(bank_cmd),
                    target_track_deg: Some(desired_track),
                    target_heading_deg: Some(self.runway_frame.runway.course_deg),
                    target_path: Some(leg),
                    target_altitude_ft: Some(self.config.cruise_altitude_ft()),
                    target_speed_kt: Some(self.config.performance.vy_kt),
                    throttle_limit: Some((0.9, 1.0)),
                    ..Default::default()
                }
            }
            FlightPhase::EnrouteClimb | FlightPhase::Cruise => {
                let target_course = self.runway_frame.runway.course_deg;
                let track_error = wrap_degrees_180(target_course - state.track_deg);
                let target_bank = clamp(track_error * 0.35, -bank_limit, bank_limit);
                GuidanceTargets {
                    lateral_mode: LateralMode::TrackHold,
                    vertical_mode: VerticalMode::Tecs,
                    target_bank_deg: Some(target_bank),
                    target_track_deg: Some(target_course),
                    target_heading_deg: Some(target_course),
                    target_altitude_ft: Some(self.config.cruise_altitude_ft()),
                    target_speed_kt: Some(self.config.performance.vy_kt),
                    throttle_limit: Some((0.9, 1.0)),
                    ..Default::default()
                }
            }
            _ => GuidanceTargets {
                lateral_mode: LateralMode::BankHold,
                vertical_mode: VerticalMode::PitchHold,
                target_bank_deg: Some(0.0),
                target_pitch_deg: Some(0.0),
                throttle_limit: Some((0.0, 0.0)),
                ..Default::default()
            },
        };
        let guidance = self.safety_monitor.apply_limits(guidance, self.phase);
        ProfileTick::contribution_only(guidance_to_contribution(guidance))
    }

    fn debug_line(&self, state: &AircraftState) -> Option<String> {
        // Only interesting during the roll — after rotate the runway
        // distance is no longer load-bearing.
        if !matches!(
            self.phase,
            FlightPhase::Preflight | FlightPhase::TakeoffRoll
        ) {
            return None;
        }
        let along = self.runway_frame.to_runway_frame(state.position_ft).x;
        let rolled = (along - self.start_along_ft).max(0.0);
        let remaining = (self.usable_length_ft - rolled).max(0.0);
        let vr = self.config.performance.vr_kt;
        let mut line = format!(
            "takeoff: usable={:.0}ft rolled={:.0}ft remaining={:.0}ft ias={:.0}kt vr={:.0}kt",
            self.usable_length_ft, rolled, remaining, state.ias_kt, vr
        );
        // Abort heuristic: if we've consumed ≥70% of usable length and
        // still aren't within 10% of Vr, something's wrong (headwind
        // drop, underpower, wrong aircraft weight) and the pilot should
        // abort rather than force a rotation with no margin.
        if self.usable_length_ft > 0.0
            && rolled >= 0.7 * self.usable_length_ft
            && state.ias_kt < 0.9 * vr
        {
            line.push_str(" [ABORT-ADVISED: low speed past 70% of usable runway]");
        }
        Some(line)
    }
}

// ---------- pattern fly profile ----------

/// Which pattern leg a turn trigger applies to. Used by `execute_turn`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PatternLeg {
    Crosswind,
    Downwind,
    Base,
    Final,
}

impl PatternLeg {
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "crosswind" => Some(Self::Crosswind),
            "downwind" => Some(Self::Downwind),
            "base" => Some(Self::Base),
            "final" => Some(Self::Final),
            _ => None,
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Crosswind => "crosswind",
            Self::Downwind => "downwind",
            Self::Base => "base",
            Self::Final => "final",
        }
    }
}

/// Which gate `set_clearance` controls. "turn_X" gates the auto-transition
/// into phase X; "land" records (but does not enforce) landing
/// authorization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PatternClearanceGate {
    TurnCrosswind,
    TurnDownwind,
    TurnBase,
    TurnFinal,
    Land,
}

impl PatternClearanceGate {
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "turn_crosswind" => Some(Self::TurnCrosswind),
            "turn_downwind" => Some(Self::TurnDownwind),
            "turn_base" => Some(Self::TurnBase),
            "turn_final" => Some(Self::TurnFinal),
            "land" => Some(Self::Land),
            _ => None,
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            Self::TurnCrosswind => "turn_crosswind",
            Self::TurnDownwind => "turn_downwind",
            Self::TurnBase => "turn_base",
            Self::TurnFinal => "turn_final",
            Self::Land => "land",
        }
    }
}

/// `Add` accumulates onto the existing extension; `Set` replaces it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtendMode {
    Add,
    Set,
}

/// Accumulated leg extensions applied to the pattern geometry. Crosswind
/// widens the effective downwind offset (pushing the whole pattern
/// further out laterally); downwind pushes the base turn further past
/// the threshold. Both compose additively in `build_pattern_geometry`.
#[derive(Debug, Clone, Default)]
pub struct PatternLegExtensions {
    pub crosswind_ft: f64,
    pub downwind_ft: f64,
}

pub struct PatternFlyProfile {
    pub config: ConfigBundle,
    pub runway_frame: RunwayFrame,
    pub clearances: PatternClearances,
    pub triggers: PatternTriggers,
    pub leg_extensions: PatternLegExtensions,
    pub force_go_around_trigger: bool,
    pub touch_and_go_trigger: bool,
    pub phase: FlightPhase,
    pub last_go_around_reason: Option<String>,
    pub mode_manager: ModeManager,
    pub safety_monitor: SafetyMonitor,
    pub lateral_guidance: L1PathFollower,
    pub pattern: PatternGeometry,
    pub route_manager: RouteManager,
    /// When set on a tick, PilotCore should swap pattern_fly out for these
    /// three single-axis holds.
    handoff_request: Option<Vec<Box<dyn GuidanceProfile>>>,
    /// LLM-set preferred runway-exit taxiway for post-landing rollout.
    /// The rollout logic aims the brake profile to reach turnoff speed by
    /// the time the aircraft reaches this taxiway's stationing; when the
    /// wheels are past the hold-short line the profile auto-hands off to
    /// idle and stops the aircraft clear of the runway. `None` = take the
    /// first available exit.
    pub preferred_exit: Option<String>,
}

impl PatternFlyProfile {
    pub fn new(config: ConfigBundle, runway_frame: RunwayFrame) -> Self {
        let default_downwind_ext = config.pattern.default_extension_ft;
        let pattern = build_pattern_geometry(
            &runway_frame,
            config.pattern.downwind_offset_ft,
            default_downwind_ext,
            0.0,
        );
        let route = RouteManager::new(vec![Waypoint {
            name: "pattern_entry_start".to_string(),
            position_ft: runway_frame.to_world_frame(config.airport.mission.entry_start_runway_ft),
            altitude_ft: Some(config.pattern_altitude_msl_ft()),
        }]);
        Self {
            mode_manager: ModeManager::new(config.clone()),
            safety_monitor: SafetyMonitor::new(config.clone()),
            lateral_guidance: L1PathFollower::new(),
            pattern,
            route_manager: route,
            clearances: PatternClearances::default(),
            triggers: PatternTriggers::default(),
            leg_extensions: PatternLegExtensions {
                crosswind_ft: 0.0,
                downwind_ft: default_downwind_ext,
            },
            force_go_around_trigger: false,
            touch_and_go_trigger: false,
            phase: FlightPhase::Preflight,
            last_go_around_reason: None,
            config,
            runway_frame,
            handoff_request: None,
            preferred_exit: None,
        }
    }

    pub fn go_around(&mut self) { self.force_go_around_trigger = true; }

    /// Record (or clear) the preferred runway-exit taxiway for the
    /// post-landing rollout. `None` = let the rollout pick the first
    /// available exit.
    pub fn set_preferred_exit(&mut self, taxiway: Option<String>) {
        self.preferred_exit = taxiway;
    }
    pub fn execute_touch_and_go(&mut self) { self.touch_and_go_trigger = true; }

    /// Fire the one-shot trigger for a given turn. Once set, the next
    /// `ModeManager::update` tick forces the corresponding transition
    /// (bypassing the auto-ready predicate and any revoked clearance). The
    /// dynamic-leg-rebuild side effect for `Base` (base leg must start
    /// where the aircraft is, not at a stale precomputed point) is
    /// handled in `contribute()` before `update` runs.
    pub fn execute_turn(&mut self, leg: PatternLeg) {
        match leg {
            PatternLeg::Crosswind => self.triggers.turn_crosswind = true,
            PatternLeg::Downwind => self.triggers.turn_downwind = true,
            PatternLeg::Base => self.triggers.turn_base = true,
            PatternLeg::Final => self.triggers.turn_final = true,
        }
    }

    /// Grant or revoke an ATC clearance gate. `runway_id` is meaningful
    /// only for `Land` and is ignored otherwise.
    pub fn set_clearance(
        &mut self,
        gate: PatternClearanceGate,
        granted: bool,
        runway_id: Option<String>,
    ) {
        match gate {
            PatternClearanceGate::TurnCrosswind => self.clearances.turn_crosswind = granted,
            PatternClearanceGate::TurnDownwind => self.clearances.turn_downwind = granted,
            PatternClearanceGate::TurnBase => self.clearances.turn_base = granted,
            PatternClearanceGate::TurnFinal => self.clearances.turn_final = granted,
            PatternClearanceGate::Land => {
                self.clearances.land = granted;
                self.clearances.landing_runway = if granted {
                    runway_id.or_else(|| self.runway_frame.runway.id.clone())
                } else {
                    None
                };
            }
        }
    }

    /// Extend a pattern leg by adding to (or setting) its stored
    /// extension, then rebuild geometry. Only `Crosswind` and `Downwind`
    /// have an effect — `Base`/`Final` return `Err` so the tool layer
    /// can report a clean error. `feet` for `Set` is clamped to >= 0.
    pub fn extend_leg(
        &mut self,
        leg: PatternLeg,
        feet: f64,
        mode: ExtendMode,
    ) -> Result<f64, String> {
        let slot = match leg {
            PatternLeg::Crosswind => &mut self.leg_extensions.crosswind_ft,
            PatternLeg::Downwind => &mut self.leg_extensions.downwind_ft,
            PatternLeg::Base | PatternLeg::Final => {
                return Err(format!(
                    "leg '{}' is not length-extensible (only crosswind and downwind are)",
                    leg.as_str()
                ));
            }
        };
        match mode {
            ExtendMode::Add => *slot += feet,
            ExtendMode::Set => *slot = feet.max(0.0),
        }
        let new_total = slot.max(0.0);
        *slot = new_total;
        self.rebuild_pattern();
        Ok(new_total)
    }

    fn rebuild_pattern(&mut self) {
        self.pattern = build_pattern_geometry(
            &self.runway_frame,
            self.config.pattern.downwind_offset_ft,
            self.leg_extensions.downwind_ft,
            self.leg_extensions.crosswind_ft,
        );
    }

    fn go_around_climb_settled(&self, state: &AircraftState) -> bool {
        let alt_err = (self.config.pattern_altitude_msl_ft() - state.alt_msl_ft).abs();
        alt_err < 100.0 && state.vs_fpm.abs() < 200.0
    }

    /// Build guidance for the current phase.
    pub fn guidance_for_phase(&mut self, state: &AircraftState, phase: FlightPhase) -> GuidanceTargets {
        let bank_limit = self.safety_monitor.bank_limit_deg(phase);
        match phase {
            FlightPhase::TakeoffRoll => build_takeoff_roll_guidance(&self.config, &self.runway_frame),
            FlightPhase::Rotate => build_rotate_guidance(&self.config, &self.runway_frame, state, bank_limit),
            FlightPhase::GoAround => self.go_around_guidance(state, bank_limit),
            FlightPhase::InitialClimb => self.initial_climb_guidance(state, bank_limit),
            FlightPhase::Crosswind => self.crosswind_guidance(state, bank_limit),
            FlightPhase::EnrouteClimb | FlightPhase::Cruise | FlightPhase::Descent => {
                self.enroute_guidance(state, bank_limit, phase)
            }
            FlightPhase::PatternEntry
            | FlightPhase::Downwind
            | FlightPhase::Base
            | FlightPhase::Final => self.pattern_phase_guidance(state, bank_limit, phase),
            FlightPhase::Roundout => self.roundout_guidance(state, bank_limit),
            FlightPhase::Flare => self.flare_guidance(state, bank_limit),
            FlightPhase::Rollout | FlightPhase::RunwayExit => {
                // Post-landing brake behavior:
                // - If the LLM has named a preferred exit via
                //   `choose_runway_exit`, modulate brakes so the
                //   aircraft decelerates to `turnoff_speed_kt` and holds
                //   it. "Does not stop until it clears the runway."
                // - Near the departure end of the runway (last 500 ft)
                //   apply full brakes regardless — the aircraft is not
                //   going to find an exit at this point, so stop on
                //   the runway rather than rolling off the end.
                // - If we're below turnoff target but still rolling on
                //   centerline, keep light braking so the aircraft
                //   comes to a full stop. Without this, aircraft that
                //   missed the preferred exit coast indefinitely at
                //   1–5 kt with the pilot-LLM waiting for control.
                // - Otherwise keep the legacy hard-brake so the
                //   deterministic simple-backend scenarios (no LLM in
                //   the loop) still converge to TaxiClear at 5 kt.
                let brakes = if self.preferred_exit.is_some() {
                    let target = self.config.post_landing.turnoff_speed_kt;
                    let length_ft = self.runway_frame.runway.length_ft;
                    let near_end = state
                        .runway_x_ft
                        .map(|x| x > length_ft - 500.0)
                        .unwrap_or(false);
                    if near_end && state.gs_kt > 1.0 {
                        0.75
                    } else if state.gs_kt > target + 2.0 {
                        0.75
                    } else if state.gs_kt > target {
                        0.35
                    } else if state.gs_kt > 1.0 {
                        0.25
                    } else {
                        0.0
                    }
                } else {
                    0.75
                };
                GuidanceTargets {
                    lateral_mode: LateralMode::RolloutCenterline,
                    vertical_mode: VerticalMode::PitchHold,
                    target_bank_deg: Some(0.0),
                    target_heading_deg: Some(self.runway_frame.runway.course_deg),
                    target_pitch_deg: Some(0.0),
                    throttle_limit: Some((0.0, 0.0)),
                    brakes,
                    ..Default::default()
                }
            }
            FlightPhase::TaxiClear => GuidanceTargets {
                lateral_mode: LateralMode::RolloutCenterline,
                vertical_mode: VerticalMode::PitchHold,
                target_bank_deg: Some(0.0),
                target_heading_deg: Some(self.runway_frame.runway.course_deg),
                target_pitch_deg: Some(0.0),
                throttle_limit: Some((0.0, 0.0)),
                // Full brake on arrival — we're clear of the runway and
                // need to stop. Hand-off to idle happens in `contribute()`
                // so the LLM takes over with a stopped aircraft.
                brakes: 0.75,
                ..Default::default()
            },
            _ => GuidanceTargets {
                lateral_mode: LateralMode::BankHold,
                vertical_mode: VerticalMode::PitchHold,
                target_bank_deg: Some(0.0),
                target_pitch_deg: Some(0.0),
                throttle_limit: Some((0.0, 0.0)),
                ..Default::default()
            },
        }
    }

    fn go_around_guidance(&self, state: &AircraftState, bank_limit: f64) -> GuidanceTargets {
        let course = self.runway_frame.runway.course_deg;
        let track_error = wrap_degrees_180(course - state.track_deg);
        // Higher gain than the pattern/crosswind phases (0.35): go-around
        // runs with firewalled power, so P-factor and slipstream yaw the
        // nose left and the P-only loop with the smaller gain settled too
        // far off course. 1.0 brings the closed-loop time constant down
        // from ~13 s to ~4.5 s at climb speed without saturating the
        // 25° bank limit at realistic track errors.
        let bank_cmd = clamp(track_error * 1.0, -bank_limit, bank_limit);
        let pattern_alt = self.config.pattern_altitude_msl_ft();
        // Go-around is "committed to climb" — firewall the throttle and
        // hold it until pattern altitude is captured and PatternFlyProfile
        // takes over. A previous revision dropped to (0.2, 0.55) once
        // within 100 ft of pattern alt, which would cut power exactly
        // when crossing a threshold and produce the same discrete-regime
        // bouncing as the pattern downwind bug.
        GuidanceTargets {
            lateral_mode: LateralMode::TrackHold,
            vertical_mode: VerticalMode::Tecs,
            target_bank_deg: Some(bank_cmd),
            target_track_deg: Some(course),
            target_heading_deg: Some(course),
            target_altitude_ft: Some(pattern_alt),
            target_speed_kt: Some(self.config.performance.vy_kt),
            throttle_limit: Some((0.9, 1.0)),
            tecs_phase_override: None,
            flaps_cmd: Some(10),
            gear_down: Some(true),
            ..Default::default()
        }
    }

    fn initial_climb_guidance(&self, state: &AircraftState, bank_limit: f64) -> GuidanceTargets {
        // Follow the extended runway centerline with the same L1 follower
        // the pattern legs use. Crosstrack offset accumulated during
        // rotation / early climb gets actively recovered instead of
        // leaving us parallel-to but offset-from the centerline like the
        // old P-on-track form did, and crosswind is handled implicitly
        // by the resulting crab angle.
        let leg = self.runway_frame.departure_leg(30_000.0);
        let (desired_track, bank_cmd) =
            self.lateral_guidance.follow_leg(state, leg, bank_limit);
        GuidanceTargets {
            lateral_mode: LateralMode::PathFollow,
            vertical_mode: VerticalMode::Tecs,
            target_bank_deg: Some(bank_cmd),
            target_track_deg: Some(desired_track),
            target_heading_deg: Some(self.runway_frame.runway.course_deg),
            target_path: Some(leg),
            target_altitude_ft: Some(self.config.pattern_altitude_msl_ft()),
            target_speed_kt: Some(self.config.performance.vy_kt),
            throttle_limit: Some((0.75, 1.0)),
            tecs_phase_override: Some(FlightPhase::EnrouteClimb),
            ..Default::default()
        }
    }

    fn crosswind_guidance(&self, state: &AircraftState, bank_limit: f64) -> GuidanceTargets {
        let side_sign = match self.runway_frame.runway.traffic_side {
            TrafficSide::Left => -1.0,
            TrafficSide::Right => 1.0,
        };
        let course = self.runway_frame.runway.course_deg;
        let crosswind_course = wrap_degrees_360(course + side_sign * 90.0);
        let track_error = wrap_degrees_180(crosswind_course - state.track_deg);
        let bank_cmd = clamp(track_error * 0.35, -bank_limit, bank_limit);
        GuidanceTargets {
            lateral_mode: LateralMode::TrackHold,
            vertical_mode: VerticalMode::Tecs,
            target_bank_deg: Some(bank_cmd),
            target_track_deg: Some(crosswind_course),
            target_heading_deg: Some(crosswind_course),
            target_altitude_ft: Some(self.config.pattern_altitude_msl_ft()),
            target_speed_kt: Some(self.config.performance.vy_kt),
            throttle_limit: Some((0.7, 1.0)),
            tecs_phase_override: Some(FlightPhase::EnrouteClimb),
            ..Default::default()
        }
    }

    fn enroute_guidance(
        &self,
        state: &AircraftState,
        bank_limit: f64,
        phase: FlightPhase,
    ) -> GuidanceTargets {
        let waypoint = self.route_manager.active_waypoint().cloned();
        let (desired_track, bank_cmd) = match &waypoint {
            Some(wp) => self.lateral_guidance.direct_to(state, wp, bank_limit),
            None => (state.track_deg, 0.0),
        };
        let (target_alt, target_spd, throttle_limit) = match phase {
            FlightPhase::EnrouteClimb => (
                self.config.cruise_altitude_ft(),
                self.config.performance.vy_kt,
                (0.75, 1.0),
            ),
            FlightPhase::Cruise => (
                self.config.cruise_altitude_ft(),
                self.config.performance.cruise_speed_kt,
                (0.4, 0.8),
            ),
            _ => (
                self.config.pattern_altitude_msl_ft(),
                self.config.performance.descent_speed_kt,
                (0.15, 0.6),
            ),
        };
        GuidanceTargets {
            lateral_mode: LateralMode::TrackHold,
            vertical_mode: VerticalMode::Tecs,
            target_bank_deg: Some(bank_cmd),
            target_track_deg: Some(desired_track),
            target_waypoint: waypoint,
            target_altitude_ft: Some(target_alt),
            target_speed_kt: Some(target_spd),
            throttle_limit: Some(throttle_limit),
            ..Default::default()
        }
    }

    fn pattern_phase_guidance(
        &mut self,
        state: &AircraftState,
        bank_limit: f64,
        phase: FlightPhase,
    ) -> GuidanceTargets {
        let leg = self.pattern.leg_for_phase(phase).expect("pattern leg");
        let (desired_track, bank_cmd) = if phase == FlightPhase::PatternEntry {
            let join_world = self
                .runway_frame
                .to_world_frame(self.pattern.join_point_runway_ft);
            let join_waypoint = Waypoint {
                name: "pattern_join".to_string(),
                position_ft: join_world,
                altitude_ft: Some(self.config.pattern_altitude_msl_ft()),
            };
            self.lateral_guidance.direct_to(state, &join_waypoint, bank_limit)
        } else {
            self.lateral_guidance.follow_leg(state, leg, bank_limit)
        };

        // Throttle limits are the physical envelope only (idle .. firewall).
        // Operational policy (cruise vs climb thrust) is handled by TECS
        // trim + PI authority, not by clamping the controller output. A
        // previous revision clipped Downwind to 0.55 which combined with
        // an `alt_error > 150 ft` override flipping to (0.7, 1.0) caused a
        // limit cycle when steady-state altitude sat right on the
        // threshold.
        let tecs_phase_override: Option<FlightPhase> = None;
        let (target_altitude_ft, target_speed_kt, vertical_mode, glidepath, throttle_limit, flaps_cmd) = match phase {
            FlightPhase::PatternEntry => (
                self.config.pattern_altitude_msl_ft(),
                self.config.performance.downwind_speed_kt,
                VerticalMode::Tecs,
                None,
                (0.0, 1.0),
                Some(0),
            ),
            FlightPhase::Downwind => {
                // Real VFR pattern descent: power back and start a gentle
                // descent abeam the numbers, so we arrive at the base turn
                // already partway down rather than forcing a 600 ft step
                // drop into the base target. Lerp the altitude target from
                // pattern altitude at abeam (x=0) down to `field+500` at
                // the base-turn x. Before abeam (x>0) we hold pattern
                // altitude; the clamp handles the missing-runway-frame
                // case and any overshoot past the base-turn point.
                let pattern_alt = self.config.pattern_altitude_msl_ft();
                let base_turn_intercept_alt_ft =
                    self.config.airport.field_elevation_ft + 500.0;
                let base_turn_x = self.pattern.base_turn_x_ft;
                let target_alt = match state.runway_x_ft {
                    Some(x) if base_turn_x < 0.0 => {
                        let frac = (x / base_turn_x).clamp(0.0, 1.0);
                        pattern_alt + (base_turn_intercept_alt_ft - pattern_alt) * frac
                    }
                    _ => pattern_alt,
                };
                (
                    target_alt,
                    self.config.performance.downwind_speed_kt,
                    VerticalMode::Tecs,
                    None,
                    (0.0, 1.0),
                    Some(10),
                )
            }
            FlightPhase::Base => (
                self.config.airport.field_elevation_ft + 400.0,
                self.config.performance.base_speed_kt,
                VerticalMode::Tecs,
                None,
                (0.0, 1.0),
                Some(20),
            ),
            FlightPhase::Final => {
                let final_slope = 3.0;
                let target_altitude = glidepath_target_altitude_ft_default(
                    &self.runway_frame,
                    state.runway_x_ft.unwrap_or(-3000.0),
                    self.config.airport.field_elevation_ft,
                );
                (
                    target_altitude,
                    self.config.performance.final_speed_kt,
                    VerticalMode::GlidepathTrack,
                    Some(Glidepath {
                        slope_deg: final_slope,
                        threshold_crossing_height_ft: 0.0,
                        aimpoint_ft_from_threshold: self.runway_frame.touchdown_runway_x_ft(),
                    }),
                    (0.0, 1.0),
                    Some(30),
                )
            }
            _ => unreachable!(),
        };
        // Stable display heading (non-L1) so the TUI doesn't flicker while L1
        // chatters intercept angles.
        let course = self.runway_frame.runway.course_deg;
        let side_sign = match self.runway_frame.runway.traffic_side {
            TrafficSide::Left => -1.0,
            TrafficSide::Right => 1.0,
        };
        let display_heading = match phase {
            FlightPhase::PatternEntry | FlightPhase::Downwind => wrap_degrees_360(course + 180.0),
            FlightPhase::Base => wrap_degrees_360(course - side_sign * 90.0),
            _ => course,
        };
        GuidanceTargets {
            lateral_mode: LateralMode::PathFollow,
            vertical_mode,
            target_bank_deg: Some(bank_cmd),
            target_track_deg: Some(desired_track),
            target_heading_deg: Some(display_heading),
            target_path: Some(leg),
            target_altitude_ft: Some(target_altitude_ft),
            target_speed_kt: Some(target_speed_kt),
            glidepath,
            throttle_limit: Some(throttle_limit),
            flaps_cmd,
            tecs_phase_override,
            ..Default::default()
        }
    }

    fn roundout_guidance(&self, state: &AircraftState, bank_limit: f64) -> GuidanceTargets {
        let (desired_track, bank_cmd) = self.lateral_guidance.follow_leg(
            state,
            self.pattern.final_leg,
            bank_limit.min(8.0),
        );
        // Lower base pitch + gentler ramp so the plane keeps descending
        // through the roundout rather than leveling off and floating.
        // Old form (2.5 + δ·0.12) produced 4.9° at touchdown, which held the
        // aircraft ~1400 ft down the pavement before wheels touched.
        let pitch_cmd = 1.5 + ((self.config.flare.roundout_height_ft - state.alt_agl_ft).max(0.0)) * 0.08;
        GuidanceTargets {
            lateral_mode: LateralMode::PathFollow,
            vertical_mode: VerticalMode::PitchHold,
            target_path: Some(self.pattern.final_leg),
            target_bank_deg: Some(bank_cmd),
            target_track_deg: Some(desired_track),
            target_pitch_deg: Some(pitch_cmd),
            target_speed_kt: Some(self.config.performance.vref_kt),
            throttle_limit: Some((0.0, 0.2)),
            ..Default::default()
        }
    }

    fn flare_guidance(&self, state: &AircraftState, bank_limit: f64) -> GuidanceTargets {
        let (desired_track, bank_cmd) = self.lateral_guidance.follow_leg(
            state,
            self.pattern.final_leg,
            bank_limit.min(6.0),
        );
        GuidanceTargets {
            lateral_mode: LateralMode::PathFollow,
            vertical_mode: VerticalMode::FlareTrack,
            target_path: Some(self.pattern.final_leg),
            target_bank_deg: Some(bank_cmd),
            target_track_deg: Some(desired_track),
            target_speed_kt: Some(self.config.performance.vref_kt - 2.0),
            throttle_limit: Some((0.0, 0.0)),
            brakes: 0.0,
            ..Default::default()
        }
    }
}

impl GuidanceProfile for PatternFlyProfile {
    fn name(&self) -> &'static str { "pattern_fly" }
    fn owns(&self) -> BTreeSet<Axis> {
        let mut s = BTreeSet::new();
        s.insert(Axis::Lateral);
        s.insert(Axis::Vertical);
        s.insert(Axis::Speed);
        s
    }

    fn contribute(&mut self, state: &AircraftState, _dt: f64) -> ProfileTick {
        // Dynamic leg rebuild on triggers: the subsequent leg must start
        // at the aircraft's *current* runway-frame position, not at a
        // stale precomputed point, or "extend then turn X" would leave
        // the next leg behind the aircraft.
        if self.triggers.turn_base && self.phase == FlightPhase::Downwind {
            if let Some(rx) = state.runway_x_ft {
                let downwind_offset = self.config.pattern.downwind_offset_ft;
                self.leg_extensions.downwind_ft =
                    (-rx - downwind_offset - self.leg_extensions.crosswind_ft).max(0.0);
                self.rebuild_pattern();
            }
        }
        if self.triggers.turn_downwind && self.phase == FlightPhase::Crosswind {
            if let Some(ry) = state.runway_y_ft {
                let downwind_offset = self.config.pattern.downwind_offset_ft;
                self.leg_extensions.crosswind_ft =
                    (ry.abs() - downwind_offset).max(0.0);
                self.rebuild_pattern();
            }
        }

        self.route_manager.advance_if_needed(state.position_ft);
        let safety = self.safety_monitor.evaluate(state, self.phase);
        let previous_phase = self.phase;
        let manual_go_around = self.force_go_around_trigger;
        self.phase = self.mode_manager.update(&ModeManagerUpdate {
            phase: self.phase,
            state,
            route_manager: &self.route_manager,
            pattern: &self.pattern,
            safety_status: &safety,
            triggers: &self.triggers,
            clearances: &self.clearances,
            force_go_around: self.force_go_around_trigger,
            stay_in_pattern: true,
            touch_and_go: self.touch_and_go_trigger,
        });
        // Consume one-shot triggers when their corresponding transition
        // has been taken (or is no longer applicable because the phase
        // advanced anyway).
        if previous_phase == FlightPhase::InitialClimb && self.phase != FlightPhase::InitialClimb {
            self.triggers.turn_crosswind = false;
        }
        if previous_phase == FlightPhase::Crosswind && self.phase != FlightPhase::Crosswind {
            self.triggers.turn_downwind = false;
        }
        if previous_phase == FlightPhase::Downwind && self.phase != FlightPhase::Downwind {
            self.triggers.turn_base = false;
        }
        if previous_phase == FlightPhase::Base && self.phase != FlightPhase::Base {
            self.triggers.turn_final = false;
        }
        if previous_phase == FlightPhase::TakeoffRoll && self.phase == FlightPhase::Rotate {
            self.touch_and_go_trigger = false;
        }
        if self.phase == FlightPhase::GoAround {
            self.force_go_around_trigger = false;
            if previous_phase != FlightPhase::GoAround {
                self.last_go_around_reason = Some(if manual_go_around {
                    "manual_trigger".to_string()
                } else if let Some(r) = safety.reason.as_ref() {
                    r.clone()
                } else {
                    "unknown".to_string()
                });
            }
        }
        let guidance = self.guidance_for_phase(state, self.phase);
        let guidance = self.safety_monitor.apply_limits(guidance, self.phase);

        let mut hand_off: Option<Vec<Box<dyn GuidanceProfile>>> = None;
        if self.phase == FlightPhase::GoAround && self.go_around_climb_settled(state) {
            hand_off = Some(vec![
                Box::new(HeadingHoldProfile::new(
                    self.runway_frame.runway.course_deg,
                    self.config.limits.max_bank_enroute_deg,
                    None,
                ).unwrap()),
                Box::new(AltitudeHoldProfile::new(self.config.pattern_altitude_msl_ft())),
                Box::new(SpeedHoldProfile::new(self.config.performance.vy_kt)),
            ]);
        }
        // Post-landing hand-off: once the aircraft is in TaxiClear
        // (clear of the runway and stopped), release the three-axis
        // pattern profile so the LLM can engage a taxi/park profile
        // without displacing us. Guard on `previous_phase != TaxiClear`
        // so we only queue the hand-off on the *first* tick of TaxiClear.
        //
        // Only fires when the LLM has opted into the new post-landing
        // flow via `choose_runway_exit` — deterministic simple-backend
        // scenarios (no LLM) keep the legacy "pattern_fly stays engaged
        // through TaxiClear" behavior so `ScenarioRunner` can still read
        // the phase via `pattern_metadata`.
        if self.phase == FlightPhase::TaxiClear
            && previous_phase != FlightPhase::TaxiClear
            && self.preferred_exit.is_some()
        {
            hand_off = Some(vec![
                Box::new(IdleLateralProfile),
                Box::new(IdleVerticalProfile),
                Box::new(IdleSpeedProfile::new(0.0)),
            ]);
        }
        if let Some(queued) = self.handoff_request.take() {
            hand_off = Some(queued);
        }

        ProfileTick {
            contribution: guidance_to_contribution(guidance),
            hand_off,
        }
    }

    fn pattern_metadata(&self) -> Option<PatternMetadata> {
        Some(PatternMetadata {
            last_go_around_reason: self.last_go_around_reason.clone(),
            airport_ident: self.config.airport.airport.clone(),
            runway_id: self.runway_frame.runway.id.clone(),
            field_elevation_ft: Some(self.config.airport.field_elevation_ft),
            phase: self.phase,
        })
    }
}

pub struct ApproachRunwayProfile {
    pub runway_id: String,
}

impl GuidanceProfile for ApproachRunwayProfile {
    fn name(&self) -> &'static str { "approach_runway" }
    fn owns(&self) -> BTreeSet<Axis> {
        let mut s = BTreeSet::new();
        s.insert(Axis::Lateral);
        s.insert(Axis::Vertical);
        s.insert(Axis::Speed);
        s
    }
    fn contribute(&mut self, _state: &AircraftState, _dt: f64) -> ProfileTick {
        // Stub: the Python version raises NotImplementedError. Here we yield
        // zeros so the control loop keeps running if somebody accidentally
        // engages us.
        ProfileTick::contribution_only(ProfileContribution::default())
    }
}

pub struct RouteFollowProfile {
    pub waypoints: Vec<Waypoint>,
}

impl GuidanceProfile for RouteFollowProfile {
    fn name(&self) -> &'static str { "route_follow" }
    fn owns(&self) -> BTreeSet<Axis> {
        let mut s = BTreeSet::new();
        s.insert(Axis::Lateral);
        s.insert(Axis::Vertical);
        s.insert(Axis::Speed);
        s
    }
    fn contribute(&mut self, _state: &AircraftState, _dt: f64) -> ProfileTick {
        ProfileTick::contribution_only(ProfileContribution::default())
    }
}

// ---------- taxi profile ----------

/// Ground taxi profile: follows a pre-planned sequence of straight legs at
/// target ground speed, slowing for sharp turns and stopping at the last
/// leg. Owns all three axes so engaging it cleanly displaces any cruise /
/// pattern profile.
///
/// Construction:
/// - `legs_ft[i]` is the `i`-th straight segment in runway-frame feet.
/// - `taxiway_names[i]` is the taxiway-name label (for status; not for
///   control). May be empty for unnamed connector edges.
///
/// See `src/guidance/taxi_route.rs` for the planner that produces the
/// leg list and `src/control/{ground_speed,nose_wheel}.rs` for the two
/// low-level controllers `commands_from_guidance` runs when it sees the
/// `TaxiFollow` / `Taxi` modes this profile raises.
/// Whether this `TaxiProfile` instance was engaged as a ground taxi
/// (destination = hold-short) or as a runway line-up (destination =
/// on-runway entry point + alignment pose). The distinction only
/// affects the profile's reported `name()`, which surfaces into
/// `active_profiles` / `completed_profiles` and the heartbeat.
/// Downstream consumers (LLM, TUI, logs) read the name to decide
/// "what's the aircraft currently doing?" — a line-up looks like
/// a taxi from the guidance side but means something different at
/// the mission level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaxiProfileMode {
    Taxi,
    LineUp,
}

pub struct TaxiProfile {
    pub mode: TaxiProfileMode,
    pub legs_ft: Vec<StraightLeg>,
    pub taxiway_names: Vec<String>,
    pub current_idx: usize,
    pub finished: bool,
    pub leg_advance_distance_ft: f64,
    pub turn_speed_kt: f64,
    pub cruise_speed_kt: f64,
    pub turn_lookahead_ft: f64,
    pub final_stop_distance_ft: f64,
    /// Optional terminal pose target. After the leg sequencer advances
    /// past the last `StraightLeg`, the profile switches to pose-tracking
    /// mode: steer toward `position_ft` until close, then pivot to match
    /// `heading_deg`, then stop. This is the right abstraction for
    /// hold-short ("stop here, facing this way") where line-following is
    /// fundamentally the wrong objective. `None` = old behaviour (set
    /// `finished = true` when past the last leg, aircraft coasts to stop).
    pub final_pose: Option<crate::types::TaxiPose>,
    /// Pose-target entry radius. Inside this distance we stop chasing
    /// position and start pivoting to heading.
    pub pose_pivot_radius_ft: f64,
    /// Position tolerance for declaring the pose reached.
    pub pose_position_tol_ft: f64,
    /// Heading tolerance (degrees) for declaring the pose reached.
    pub pose_heading_tol_deg: f64,
    /// Creep speed used while pivoting to target heading — the nose wheel
    /// needs some forward motion to bite, so we can't sit at 0 kt while
    /// the heading still needs to rotate.
    pub pose_creep_speed_kt: f64,
}

impl TaxiProfile {
    pub fn new(legs_ft: Vec<StraightLeg>, taxiway_names: Vec<String>) -> Self {
        Self::with_mode(TaxiProfileMode::Taxi, legs_ft, taxiway_names)
    }

    /// Construct a taxi profile that presents itself as `line_up` in
    /// status/active_profiles output. Used by `engage_line_up` so the
    /// LLM can tell at a glance whether the active ground profile is
    /// an en-route taxi or a final runway line-up.
    pub fn new_line_up(legs_ft: Vec<StraightLeg>, taxiway_names: Vec<String>) -> Self {
        Self::with_mode(TaxiProfileMode::LineUp, legs_ft, taxiway_names)
    }

    fn with_mode(
        mode: TaxiProfileMode,
        legs_ft: Vec<StraightLeg>,
        taxiway_names: Vec<String>,
    ) -> Self {
        Self {
            mode,
            legs_ft,
            taxiway_names,
            current_idx: 0,
            finished: false,
            leg_advance_distance_ft: 30.0,
            turn_speed_kt: 5.0,
            cruise_speed_kt: 20.0,
            turn_lookahead_ft: 180.0,
            final_stop_distance_ft: 60.0,
            final_pose: None,
            pose_pivot_radius_ft: 15.0,
            pose_position_tol_ft: 20.0,
            pose_heading_tol_deg: 20.0,
            pose_creep_speed_kt: 2.0,
        }
    }

    pub fn with_final_pose(mut self, pose: crate::types::TaxiPose) -> Self {
        self.final_pose = Some(pose);
        self
    }

    /// Dedup'd taxiway-name sequence for the mode-line summary. Skips
    /// synthetic/descriptive names (empty strings, "(connector)",
    /// "(entering runway)") so the displayed route matches what ATC
    /// actually called.
    fn route_summary(&self) -> Option<String> {
        let mut out: Vec<&str> = Vec::new();
        for n in &self.taxiway_names {
            let n = n.as_str();
            if n.is_empty() || n.starts_with('(') {
                continue;
            }
            if out.last().map_or(true, |last| *last != n) {
                out.push(n);
            }
        }
        if out.is_empty() {
            None
        } else {
            Some(out.join("→"))
        }
    }

    fn current_leg(&self) -> Option<StraightLeg> {
        self.legs_ft.get(self.current_idx).copied()
    }

    fn advance_if_reached(&mut self, state: &AircraftState) {
        let Some(current) = self.current_leg() else {
            return;
        };
        let dist_to_end = (state.position_ft - current.end_ft).length();
        if dist_to_end < self.leg_advance_distance_ft {
            // Always increment. If that pushes us past the last leg, the
            // contribute path checks for a pose target; absent one, it
            // sets `finished`. This split lets the pose phase take over
            // without the line-following stop-ramp fighting it.
            self.current_idx += 1;
        }
    }

    fn in_pose_phase(&self) -> bool {
        self.current_idx >= self.legs_ft.len() && self.final_pose.is_some()
    }

    /// Total remaining taxi distance in feet: aircraft → end of current
    /// leg, plus the full length of every subsequent leg, plus the final
    /// pose approach from the last leg's end when one is set.
    fn remaining_distance_ft(&self, state: &AircraftState) -> f64 {
        if self.finished {
            return 0.0;
        }
        if self.in_pose_phase() {
            let pose = self.final_pose.unwrap();
            return (pose.position_ft - state.position_ft).length();
        }
        let Some(current) = self.current_leg() else {
            return 0.0;
        };
        let mut total = (state.position_ft - current.end_ft).length();
        for leg in &self.legs_ft[self.current_idx + 1..] {
            total += (leg.end_ft - leg.start_ft).length();
        }
        if let (Some(pose), Some(last)) = (self.final_pose, self.legs_ft.last()) {
            total += (pose.position_ft - last.end_ft).length();
        }
        total
    }

    /// Target ground speed given the upcoming geometry and the aircraft's
    /// current alignment with the active leg:
    /// - if heading error > 45°, creep at ~2 kt so the nose wheel can
    ///   pivot the aircraft onto the leg before we start accelerating;
    /// - if heading error > 20°, stay at `turn_speed_kt` for the same
    ///   reason (less extreme);
    /// - if a sharp turn (>20°) is within `turn_lookahead_ft`, slow to
    ///   `turn_speed_kt` ahead of it;
    /// - on the final leg, linearly ramp speed to 0 across the last
    ///   `2 × final_stop_distance_ft` so the ground-speed controller can
    ///   bleed energy without a hard stop;
    /// - otherwise cruise.
    fn target_speed_kt(&self, state: &AircraftState) -> f64 {
        if self.finished {
            return 0.0;
        }
        let Some(current) = self.current_leg() else {
            return 0.0;
        };
        let dist_to_end = (state.position_ft - current.end_ft).length();

        // Heading-alignment limit: when we're badly off-heading (common on
        // the pullout leg from a gate, or turning onto a taxiway from a
        // perpendicular approach) the nose wheel needs time to turn the
        // aircraft before forward thrust pushes us past the intercept.
        // The floor (4 kt) is chosen so the aircraft keeps moving forward
        // even with the nose wheel fully deflected — below that, C172
        // tire scrub can stall the aircraft in place on live X-Plane
        // ground physics, which we saw stuck at the pullout-to-taxiway
        // transition.
        let leg_bearing = current.course_deg();
        let heading_err_deg = wrap_degrees_180(leg_bearing - state.heading_deg).abs();
        let alignment_limit = if heading_err_deg > 45.0 {
            4.0
        } else if heading_err_deg > 20.0 {
            self.turn_speed_kt
        } else {
            f64::INFINITY
        };

        let nominal = if self.current_idx + 1 >= self.legs_ft.len() {
            let ramp = self.final_stop_distance_ft * 2.0;
            if dist_to_end < ramp {
                (dist_to_end / ramp) * self.cruise_speed_kt
            } else {
                self.cruise_speed_kt
            }
        } else {
            let next = self.legs_ft[self.current_idx + 1];
            let cur_bearing = current.course_deg();
            let next_bearing = next.course_deg();
            let turn_deg = wrap_degrees_180(next_bearing - cur_bearing).abs();
            if turn_deg > 20.0 && dist_to_end < self.turn_lookahead_ft {
                self.turn_speed_kt
            } else {
                self.cruise_speed_kt
            }
        };
        nominal.min(alignment_limit)
    }

    pub fn active_taxiway_name(&self) -> Option<&str> {
        self.taxiway_names
            .get(self.current_idx)
            .map(String::as_str)
    }

    /// Pose-phase target speed. Three regimes:
    ///
    /// - **Parked** (both tolerances met): 0.
    /// - **Align** (inside `pose_pivot_radius_ft`, heading not yet in
    ///   tolerance): `pose_creep_speed_kt`. A C172's nose-wheel steering
    ///   has no yaw authority at zero ground speed, and live X-Plane's
    ///   differential braking alone will not rotate a stationary
    ///   aircraft — so we hold a slow creep to let the nose wheel bite.
    ///   If we overshoot the position tolerance we'll re-enter the
    ///   approach regime below and re-approach.
    /// - **Approach** (farther): ramps with distance up to cruise, BUT
    ///   throttled by how badly the aircraft is off the bearing to the
    ///   target. At ≥ 45° off we creep; at ≥ 20° we use turn speed.
    ///   Without this, a fast-moving aircraft whose heading is way off
    ///   the target bearing will orbit the pose instead of converging —
    ///   exactly the failure mode the scenario test caught.
    fn pose_target_speed(&self, state: &AircraftState, pose: crate::types::TaxiPose) -> f64 {
        let to_target = pose.position_ft - state.position_ft;
        let d = to_target.length();
        let target_hdg_err = wrap_degrees_180(pose.heading_deg - state.heading_deg).abs();
        if d < self.pose_position_tol_ft && target_hdg_err < self.pose_heading_tol_deg {
            return 0.0;
        }
        if d < self.pose_pivot_radius_ft {
            return self.pose_creep_speed_kt;
        }
        let bearing_to_target = crate::types::vector_to_heading(to_target);
        let approach_hdg_err =
            wrap_degrees_180(bearing_to_target - state.heading_deg).abs();
        if approach_hdg_err > 45.0 {
            return self.pose_creep_speed_kt;
        }
        if approach_hdg_err > 20.0 {
            return self.turn_speed_kt;
        }
        // Well-aligned with the approach direction — ramp with distance
        // from turn_speed at the pivot radius up to cruise.
        let ramp_slope = 0.25;
        (self.turn_speed_kt + (d - self.pose_pivot_radius_ft) * ramp_slope)
            .min(self.cruise_speed_kt)
    }

    fn contribute_pose(
        &mut self,
        state: &AircraftState,
        pose: crate::types::TaxiPose,
    ) -> ProfileTick {
        let d = (pose.position_ft - state.position_ft).length();
        let hdg_err = wrap_degrees_180(pose.heading_deg - state.heading_deg).abs();
        if d < self.pose_position_tol_ft && hdg_err < self.pose_heading_tol_deg {
            self.finished = true;
        }
        let target_speed = self.pose_target_speed(state, pose);
        ProfileTick::contribution_only(ProfileContribution {
            lateral_mode: Some(LateralMode::TaxiPose),
            vertical_mode: Some(VerticalMode::Taxi),
            target_waypoint: Some(crate::types::Waypoint {
                name: "hold_short".to_string(),
                position_ft: pose.position_ft,
                altitude_ft: None,
            }),
            target_heading_deg: Some(pose.heading_deg),
            target_speed_kt: Some(target_speed),
            target_pitch_deg: Some(0.0),
            throttle_limit: Some((0.0, 1.0)),
            ..Default::default()
        })
    }
}

impl GuidanceProfile for TaxiProfile {
    fn name(&self) -> &'static str {
        match self.mode {
            TaxiProfileMode::Taxi => "taxi",
            TaxiProfileMode::LineUp => "line_up",
        }
    }
    fn owns(&self) -> BTreeSet<Axis> {
        let mut s = BTreeSet::new();
        s.insert(Axis::Lateral);
        s.insert(Axis::Vertical);
        s.insert(Axis::Speed);
        s
    }
    fn contribute(&mut self, state: &AircraftState, _dt: f64) -> ProfileTick {
        if !self.finished {
            self.advance_if_reached(state);
            // Past the last leg? Decide what to do next.
            if self.current_idx >= self.legs_ft.len() && self.final_pose.is_none() {
                self.finished = true;
            }
        }
        if self.in_pose_phase() && !self.finished {
            let pose = self.final_pose.unwrap();
            return self.contribute_pose(state, pose);
        }
        // Normal line-following path.
        let leg = self.current_leg();
        let target_speed = self.target_speed_kt(state);
        ProfileTick::contribution_only(ProfileContribution {
            lateral_mode: Some(LateralMode::TaxiFollow),
            vertical_mode: Some(VerticalMode::Taxi),
            target_path: leg,
            target_speed_kt: Some(target_speed),
            target_pitch_deg: Some(0.0),
            throttle_limit: Some((0.0, 1.0)),
            ..Default::default()
        })
    }

    fn is_complete(&self) -> bool {
        self.finished
    }

    fn mode_line_suffix(&self, state: &AircraftState) -> Option<String> {
        let route = self.route_summary();
        let total_remaining = self.remaining_distance_ft(state);
        let tail = if self.in_pose_phase() {
            let pose = self.final_pose.unwrap();
            let d = (pose.position_ft - state.position_ft).length();
            let phase = if self.finished {
                "parked"
            } else if d < self.pose_pivot_radius_ft {
                "align"
            } else {
                "approach"
            };
            format!("{phase} · {:.0}ft", d)
        } else if let Some(leg) = self.current_leg() {
            let name = self
                .taxiway_names
                .get(self.current_idx)
                .map(String::as_str)
                .filter(|s| !s.is_empty())
                .unwrap_or("→");
            let leg_remaining = (state.position_ft - leg.end_ft).length();
            format!(
                "leg {}/{} {} · {:.0}ft / {:.0}ft total",
                self.current_idx + 1,
                self.legs_ft.len(),
                name,
                leg_remaining,
                total_remaining,
            )
        } else {
            "finished".to_string()
        };
        Some(match route {
            Some(r) => format!("via {r} · {tail}"),
            None => tail,
        })
    }

    fn debug_line(&self, state: &AircraftState) -> Option<String> {
        if self.in_pose_phase() {
            let pose = self.final_pose.unwrap();
            let d = (pose.position_ft - state.position_ft).length();
            let hdg_err = wrap_degrees_180(pose.heading_deg - state.heading_deg);
            let phase = if self.finished {
                "parked"
            } else if d < self.pose_pivot_radius_ft {
                "align"
            } else {
                "approach"
            };
            return Some(format!(
                "taxi pose [{}] d {:.0}ft hdg-err {:+.0}° target {:.0}kt",
                phase,
                d,
                hdg_err,
                self.pose_target_speed(state, pose),
            ));
        }
        let Some(leg) = self.current_leg() else {
            return Some(format!("taxi finished={} legs={}", self.finished, self.legs_ft.len()));
        };
        let leg_vec = leg.end_ft - leg.start_ft;
        let leg_len = leg_vec.length().max(1e-6);
        let leg_dir = leg_vec * (1.0 / leg_len);
        let leg_right = crate::types::Vec2::new(leg_dir.y, -leg_dir.x);
        let offset = state.position_ft - leg.start_ft;
        let xtrack_ft = offset.dot(leg_right);
        let leg_bearing = crate::types::vector_to_heading(leg_vec);
        let hdg_err = wrap_degrees_180(leg_bearing - state.heading_deg);
        let dist_to_end = (state.position_ft - leg.end_ft).length();
        let name = self
            .taxiway_names
            .get(self.current_idx)
            .map(String::as_str)
            .filter(|s| !s.is_empty())
            .unwrap_or("(connector)");
        Some(format!(
            "taxi leg {}/{} \"{}\" xtrack {:+.0}ft hdg-err {:+.0}° to-end {:.0}ft target {:.0}kt",
            self.current_idx + 1,
            self.legs_ft.len(),
            name,
            xtrack_ft,
            hdg_err,
            dist_to_end,
            self.target_speed_kt(state),
        ))
    }
}

// ---------- helpers ----------

fn single(axis: Axis) -> BTreeSet<Axis> {
    let mut s = BTreeSet::new();
    s.insert(axis);
    s
}
