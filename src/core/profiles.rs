//! Guidance profiles composed by `PilotCore`.
//!
//! Each profile owns a set of axes (lateral / vertical / speed) and emits a
//! `ProfileContribution` per tick. `PilotCore` merges per-axis contributions
//! from all active profiles into a single `GuidanceTargets`. The LLM engages
//! profiles; axis-ownership conflicts auto-displace older profiles.
//!
//! When a profile needs to swap itself for different profiles mid-tick (e.g.
//! `PatternFlyProfile` handing the ground roll to `LandingRolloutProfile`
//! at touchdown), it returns a `hand_off` request and `PilotCore`
//! applies the swap after all contributions are collected. That avoids
//! reentrant `&mut` borrows on the pilot from inside one of its profiles.

use std::any::Any;

use crate::config::ConfigBundle;
use crate::core::landing_rollout::{
    touchdown_debounce, touchdown_handoff, LandingPlanHolder, LandingRolloutProfile,
};
pub use crate::core::landing_rollout::{ArrivalPlan, RunwayExitPlan};
use crate::core::mode_manager::{
    ModeManager, ModeManagerUpdate, PatternClearances, PatternTriggers, BASE_TO_FINAL_FRACTION,
};
use crate::core::safety_monitor::SafetyMonitor;
use crate::core::spatial::{LabeledLeg, SpatialOverlay};
use crate::guidance::lateral::{leg_projection, L1PathFollower};
use crate::guidance::pattern_manager::{
    build_pattern_geometry, glidepath_target_altitude_ft_default, PatternGeometry,
};
use crate::guidance::route_manager::RouteManager;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::types::{
    clamp, heading_to_vector, vector_to_heading, wrap_degrees_180, wrap_degrees_360, AircraftState,
    FlightPhase, Glidepath, GuidanceTargets, LateralMode, StraightLeg, TrafficSide, Vec2,
    VerticalMode, Waypoint,
};

/// Pose-target pivot radius: outside it a taxi steers toward the target
/// position, inside it it turns to the target heading. A taxi only
/// settles on its pose inside it.
pub const POSE_PIVOT_RADIUS_FT: f64 = 15.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Axis {
    Lateral,
    Vertical,
    Speed,
}

impl Axis {
    fn bit(self) -> u8 {
        match self {
            Axis::Lateral => 0b001,
            Axis::Vertical => 0b010,
            Axis::Speed => 0b100,
        }
    }
}

/// Set of axes a guidance profile owns. A 3-bit mask, no heap. Replaces
/// the `BTreeSet<Axis>` `GuidanceProfile::owns()` used to return — at 10
/// Hz with several active profiles, the BTreeSet allocations added up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AxisSet(u8);

impl AxisSet {
    pub const NONE: Self = Self(0);
    pub const LATERAL: Self = Self(0b001);
    pub const VERTICAL: Self = Self(0b010);
    pub const SPEED: Self = Self(0b100);
    pub const ALL: Self = Self(0b111);

    pub const fn single(axis: Axis) -> Self {
        match axis {
            Axis::Lateral => Self::LATERAL,
            Axis::Vertical => Self::VERTICAL,
            Axis::Speed => Self::SPEED,
        }
    }

    pub fn contains(self, axis: Axis) -> bool {
        self.0 & axis.bit() != 0
    }

    pub fn is_disjoint(self, other: Self) -> bool {
        self.0 & other.0 == 0
    }

    pub fn iter(self) -> impl Iterator<Item = Axis> {
        [Axis::Lateral, Axis::Vertical, Axis::Speed]
            .into_iter()
            .filter(move |a| self.contains(*a))
    }

    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    pub fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Axes present in `self` but not `other`.
    pub fn difference(self, other: Self) -> Self {
        Self(self.0 & !other.0)
    }
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
    pub slip: Option<f64>,
    pub throttle_limit: Option<(f64, f64)>,
    pub flaps_cmd: Option<i32>,
    pub gear_down: Option<bool>,
    pub brakes: Option<f64>,
    pub tecs_phase_override: Option<FlightPhase>,
    /// Parking brake ratio to hold (written to the sim's parking-brake
    /// dataref). `None` leaves it alone.
    pub parking_brake: Option<f64>,
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

/// SA-Level-3 hint surfaced on the TUI's MISSION INTENT pane: the next
/// phase the autopilot expects to enter, the predicate it's waiting on
/// (in human-readable form), and an optional time-to-event estimate.
/// Profiles that own a phase machine (today: `pattern_fly`, `takeoff`)
/// override `GuidanceProfile::next_transition_hint` to expose this.
#[derive(Debug, Clone)]
pub struct TransitionHint {
    pub next_phase: FlightPhase,
    pub condition_text: String,
    pub eta_s: Option<f64>,
}

/// Lets `PilotCore::with_profile` recover a profile's concrete type from a
/// `Box<dyn GuidanceProfile>` safely. Blanket-implemented for every
/// `'static` type, so profiles get it for free.
pub trait AsAny {
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
}

impl<T: Any> AsAny for T {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
}

pub trait GuidanceProfile: Send + AsAny {
    fn name(&self) -> &'static str;
    fn owns(&self) -> AxisSet;
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

    /// SA-Level-3 transparency hint: what phase is the profile expecting
    /// next, what predicate is it waiting on, and (optionally) how long
    /// until the predicate fires. Default `None` for profiles without a
    /// phase machine. Surfaced on `StatusSnapshot::transition_hint`.
    fn next_transition_hint(&self, _state: &AircraftState) -> Option<TransitionHint> {
        None
    }

    /// Whether another profile may displace (or a tool may disengage)
    /// this one right now. `PilotCore::engage_profiles` asks every
    /// profile it would displace and refuses the whole engage if any
    /// returns `Err(reason)`. A profile handing *itself* off via
    /// `ProfileTick::hand_off` is not asked. Default: always preemptible.
    fn preemption(&self) -> Result<(), String> {
        Ok(())
    }

    /// Read-only geometry for the TUI spatial view: the runway, pattern
    /// legs, ground route and terminal pose this profile is flying
    /// against. Only called while the view is visible. Default: nothing.
    fn spatial_overlay(&self) -> Option<SpatialOverlay> {
        None
    }

    /// The landing's exit/arrival plan, for profiles that carry one.
    fn landing_plan_mut(&mut self) -> Option<&mut dyn LandingPlanHolder> {
        None
    }

    /// The traffic pattern being flown, for profiles that fly one
    /// (itself, or a plan's embedded pattern).
    fn pattern_fly_mut(&mut self) -> Option<&mut PatternFlyProfile> {
        None
    }

    /// Notable moments since the last call ("report due: 3 miles",
    /// "constraint c1 not met"). `PilotCore` drains these every tick into
    /// its event log, and the heartbeat pump wakes the LLM on new ones.
    fn take_events(&mut self) -> Vec<String> {
        Vec::new()
    }

    /// Structured status for the heartbeat payload (the plan profile's
    /// steps, constraints and open items). Default: none.
    fn status_json(&self) -> Option<serde_json::Value> {
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
    fn owns(&self) -> AxisSet { AxisSet::LATERAL }
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
    fn owns(&self) -> AxisSet { AxisSet::VERTICAL }
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
    fn owns(&self) -> AxisSet { AxisSet::SPEED }
    fn contribute(&mut self, _: &AircraftState, _: f64) -> ProfileTick {
        ProfileTick::contribution_only(ProfileContribution {
            target_speed_kt: Some(self.default_speed_kt),
            ..Default::default()
        })
    }
}

/// Three-axis idle stack — what the pilot installs when no real profile
/// owns the aircraft (initial state, post-landing hand-off, axis-orphan
/// replacement). `speed_kt` is the speed the placeholder reports as its
/// target; downstream consumers don't act on it but it shows up in the
/// status snapshot.
pub fn idle_profile_trio(speed_kt: f64) -> Vec<Box<dyn GuidanceProfile>> {
    vec![
        Box::new(IdleLateralProfile),
        Box::new(IdleVerticalProfile),
        Box::new(IdleSpeedProfile::new(speed_kt)),
    ]
}

// ---------- single-axis holds ----------

pub struct HeadingHoldProfile {
    pub heading_deg: f64,
    pub max_bank_deg: f64,
}

impl HeadingHoldProfile {
    pub fn new(heading_deg: f64, max_bank_deg: f64) -> Self {
        Self { heading_deg: heading_deg.rem_euclid(360.0), max_bank_deg }
    }
}

impl GuidanceProfile for HeadingHoldProfile {
    fn name(&self) -> &'static str { "heading_hold" }
    fn owns(&self) -> AxisSet { AxisSet::LATERAL }
    fn contribute(&mut self, state: &AircraftState, _dt: f64) -> ProfileTick {
        let error = wrap_degrees_180(self.heading_deg - state.track_deg);
        let target_bank = clamp(error * 0.35, -self.max_bank_deg, self.max_bank_deg);
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
    fn owns(&self) -> AxisSet { AxisSet::VERTICAL }
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
    fn owns(&self) -> AxisSet { AxisSet::SPEED }
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

pub(crate) fn guidance_to_contribution(g: GuidanceTargets) -> ProfileContribution {
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
        slip: g.slip,
        throttle_limit: g.throttle_limit,
        flaps_cmd: g.flaps_cmd,
        gear_down: g.gear_down,
        brakes: Some(g.brakes),
        tecs_phase_override: g.tecs_phase_override,
        parking_brake: g.parking_brake,
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
    fn owns(&self) -> AxisSet {
        AxisSet::ALL
    }
    fn spatial_overlay(&self) -> Option<SpatialOverlay> {
        Some(SpatialOverlay {
            runway: Some(self.runway_frame.runway.clone()),
            ..Default::default()
        })
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

/// Accumulated leg extensions applied to the pattern geometry. Crosswind
/// widens the effective downwind offset (pushing the whole pattern
/// further out laterally); downwind pushes the base turn further past
/// the threshold. Both compose additively in `build_pattern_geometry`.
#[derive(Debug, Clone, Default)]
pub struct PatternLegExtensions {
    pub crosswind_ft: f64,
    pub downwind_ft: f64,
}


/// Go-around reason recorded at the bottom of a planned low approach.
pub const LOW_APPROACH_REASON: &str = "planned low approach";

/// The closest a base leg comes to the threshold (a short approach).
const MIN_BASE_X_FT: f64 = 2000.0;

/// Which pattern corners to fly square: a definite turn at a steady bank
/// that starts a turn radius before the next leg and rolls out on it,
/// instead of the follower's gradual intercept that rounds the corner off.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SquareCorners {
    /// Downwind to base.
    pub into_base: bool,
    /// Base to final.
    pub into_final: bool,
}

/// Bank for a squared turn onto base (the pattern limit is 30°).
const SQUARE_BASE_BANK_DEG: f64 = 25.0;
/// Bank for a squared turn onto final (the final limit is 20°).
const SQUARE_FINAL_BANK_DEG: f64 = 20.0;
/// Within this of the new leg's course the follower takes over.
const SQUARE_ROLLOUT_DEG: f64 = 8.0;

/// A squared corner in progress: hold `bank_deg` (signed, + right) until
/// the track is within `SQUARE_ROLLOUT_DEG` of `course_deg`.
#[derive(Debug, Clone, Copy)]
struct CornerTurn {
    phase: FlightPhase,
    course_deg: f64,
    bank_deg: f64,
}

/// Distance before a leg to start a squared 90° turn onto it at
/// `bank_deg`: the turn radius (the roll-in and the early roll-out
/// roughly cancel).
fn square_lead_ft(gs_kt: f64, bank_deg: f64) -> f64 {
    crate::guidance::geo::turn_radius_ft(gs_kt.max(40.0), bank_deg)
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
    /// LLM-set preferred runway exit, carried into the
    /// `LandingRolloutProfile` that takes over at touchdown. `None` =
    /// brake to a stop on the runway centerline.
    pub preferred_exit: Option<RunwayExitPlan>,
    /// Taxi-to-parking plan to run after the exit (from `plan_arrival`).
    pub arrival: Option<ArrivalPlan>,
    /// Continuous ground contact since touchdown (debounces the hand-off).
    ground_contact_s: f64,
    /// Corners to fly square ("square your base").
    pub square: SquareCorners,
    corner: Option<CornerTurn>,
    /// Reason to record for the next go-around instead of
    /// "manual_trigger" (a planned low approach).
    go_around_reason_override: Option<String>,
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
            preferred_exit: None,
            arrival: None,
            ground_contact_s: 0.0,
            square: SquareCorners::default(),
            corner: None,
            go_around_reason_override: None,
        }
    }

    /// A copy to fly ahead in a preview sim. Leaves out the arrival taxi
    /// plan, which a preview never reaches (it stops at touchdown).
    pub fn preview_copy(&self) -> Self {
        Self {
            config: self.config.clone(),
            runway_frame: self.runway_frame.clone(),
            clearances: self.clearances.clone(),
            triggers: self.triggers.clone(),
            leg_extensions: self.leg_extensions.clone(),
            force_go_around_trigger: self.force_go_around_trigger,
            touch_and_go_trigger: self.touch_and_go_trigger,
            phase: self.phase,
            last_go_around_reason: self.last_go_around_reason.clone(),
            mode_manager: self.mode_manager.clone(),
            safety_monitor: self.safety_monitor.clone(),
            lateral_guidance: self.lateral_guidance.clone(),
            pattern: self.pattern.clone(),
            route_manager: self.route_manager.clone(),
            preferred_exit: self.preferred_exit.clone(),
            arrival: None,
            ground_contact_s: self.ground_contact_s,
            square: self.square,
            corner: self.corner,
            go_around_reason_override: self.go_around_reason_override.clone(),
        }
    }

    pub fn go_around(&mut self) { self.force_go_around_trigger = true; }

    /// Go around as planned at the bottom of a low approach (not a
    /// rejected landing): the reason says so.
    pub fn low_approach_go_around(&mut self) {
        self.force_go_around_trigger = true;
        self.go_around_reason_override = Some(LOW_APPROACH_REASON.to_string());
    }

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

    /// Set the crosswind and downwind extensions (each at least 0) and
    /// rebuild the geometry.
    pub fn set_leg_extensions(&mut self, crosswind_ft: f64, downwind_ft: f64) {
        self.leg_extensions = PatternLegExtensions { crosswind_ft: crosswind_ft.max(0.0), downwind_ft: downwind_ft.max(0.0) };
        self.rebuild_pattern();
    }

    /// Put the base leg at runway-frame `rx` (where the aircraft turned
    /// or rolled out onto it), no closer than MIN_BASE_X_FT before the
    /// threshold.
    fn place_base_at(&mut self, rx: f64) {
        let offset = self.config.pattern.downwind_offset_ft + self.leg_extensions.crosswind_ft.max(0.0);
        self.leg_extensions.downwind_ft = (-rx - offset).max(MIN_BASE_X_FT - offset);
        self.rebuild_pattern();
    }

    fn rebuild_pattern(&mut self) {
        self.pattern = build_pattern_geometry(
            &self.runway_frame,
            self.config.pattern.downwind_offset_ft,
            self.leg_extensions.downwind_ft,
            self.leg_extensions.crosswind_ft,
        );
    }

    /// Start a squared corner on the turn onto base or final, and end it
    /// on rolling out. A squared base leg is rebuilt where the turn
    /// rolled out, so the follower holds it there instead of angling back
    /// to the planned line.
    fn update_corner(&mut self, state: &AircraftState, previous_phase: FlightPhase) {
        let side = self.runway_frame.runway.traffic_side.sign();
        let course = self.runway_frame.runway.course_deg;
        if previous_phase != self.phase {
            self.corner = match (previous_phase, self.phase) {
                (FlightPhase::Downwind, FlightPhase::Base) if self.square.into_base => Some(CornerTurn {
                    phase: FlightPhase::Base,
                    course_deg: wrap_degrees_360(course - side * 90.0),
                    bank_deg: side * SQUARE_BASE_BANK_DEG,
                }),
                (FlightPhase::Base, FlightPhase::Final) if self.square.into_final => Some(CornerTurn {
                    phase: FlightPhase::Final,
                    course_deg: course,
                    bank_deg: side * SQUARE_FINAL_BANK_DEG,
                }),
                _ => None,
            };
        }
        let Some(c) = self.corner else { return };
        if c.phase != self.phase {
            self.corner = None;
        } else if wrap_degrees_180(c.course_deg - state.track_deg).abs() <= SQUARE_ROLLOUT_DEG {
            self.corner = None;
            if c.phase == FlightPhase::Base {
                if let Some(rx) = state.runway_x_ft {
                    self.place_base_at(rx);
                }
            }
        }
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
        // hold it until pattern altitude is captured. Dropping throttle
        // near the capture altitude produces discrete-regime bouncing, so
        // the floor only lifts once within 150 ft of it; held all the way,
        // the climb power carries the aircraft hundreds of feet high and
        // the go-around never settles back into the pattern.
        let near_pattern_alt = state.alt_msl_ft >= pattern_alt - 150.0;
        let throttle_floor = if near_pattern_alt { 0.0 } else { 0.9 };
        // Near pattern altitude, level off on the pattern's trim, not the
        // climb trim (85% power, 6° nose up), which holds it high.
        let tecs_phase_override = near_pattern_alt.then_some(FlightPhase::Downwind);
        GuidanceTargets {
            lateral_mode: LateralMode::TrackHold,
            vertical_mode: VerticalMode::Tecs,
            target_bank_deg: Some(bank_cmd),
            target_track_deg: Some(course),
            target_heading_deg: Some(course),
            target_altitude_ft: Some(pattern_alt),
            target_speed_kt: Some(self.config.performance.vy_kt),
            throttle_limit: Some((throttle_floor, 1.0)),
            tecs_phase_override,
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
        } else if let Some(c) = self.corner.filter(|c| c.phase == phase) {
            (c.course_deg, clamp(c.bank_deg, -bank_limit, bank_limit))
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
            FlightPhase::Downwind => (
                // Hold pattern altitude across the whole downwind. Descent
                // is deferred to base/final, so an extended downwind does
                // not smuggle an early descent into the pattern.
                self.config.pattern_altitude_msl_ft(),
                self.config.performance.downwind_speed_kt,
                VerticalMode::Tecs,
                None,
                (0.0, 1.0),
                Some(10),
            ),
            FlightPhase::Base => {
                let pattern_alt = self.config.pattern_altitude_msl_ft();
                let field_elev_ft = self.config.airport.field_elevation_ft;
                let base_turn_x = self.pattern.base_turn_x_ft;
                let base_end_alt = glidepath_target_altitude_ft_default(
                    &self.runway_frame,
                    base_turn_x,
                    field_elev_ft,
                );
                let downwind_y_abs = self.pattern.downwind_y_ft.abs();
                let y_abs = state.runway_y_ft.unwrap_or(self.pattern.downwind_y_ft).abs();
                // Reach the glidepath height where the turn to final
                // happens, not at the extended centerline, or final starts
                // a few hundred feet high.
                let frac = if downwind_y_abs > 1.0 {
                    ((1.0 - y_abs / downwind_y_abs) / BASE_TO_FINAL_FRACTION)
                        .clamp(0.0, 1.0)
                } else {
                    1.0
                };
                // Base only ever descends: with base far out (a long
                // extended downwind) the glidepath there sits above
                // pattern altitude, and climbing to it on base bleeds the
                // speed final needs.
                let base_end_alt = base_end_alt.min(pattern_alt);
                let target_alt = pattern_alt + (base_end_alt - pattern_alt) * frac;
                (
                    target_alt,
                    self.config.performance.base_speed_kt,
                    VerticalMode::Tecs,
                    None,
                    (0.0, 1.0),
                    Some(20),
                )
            }
            FlightPhase::Final => {
                let final_slope = 3.0;
                let field_elev_ft = self.config.airport.field_elevation_ft;
                let glidepath_alt = glidepath_target_altitude_ft_default(
                    &self.runway_frame,
                    state.runway_x_ft.unwrap_or(-3000.0),
                    field_elev_ft,
                );
                // PilotCore's glidepath tracking turns this into a capped
                // capture target and a power ceiling when high.
                let glidepath = Glidepath {
                    slope_deg: final_slope,
                    threshold_crossing_height_ft: 0.0,
                    aimpoint_ft_from_threshold: self.runway_frame.touchdown_runway_x_ft(),
                };
                (
                    glidepath_alt,
                    self.config.performance.final_speed_kt,
                    VerticalMode::GlidepathTrack,
                    Some(glidepath),
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
        let pitch_cmd = 1.5 + ((self.config.flare.roundout_height_ft - state.landing_agl_ft()).max(0.0)) * 0.08;
        GuidanceTargets {
            lateral_mode: LateralMode::PathFollow,
            vertical_mode: VerticalMode::PitchHold,
            target_path: Some(self.pattern.final_leg),
            target_bank_deg: Some(bank_cmd),
            target_track_deg: Some(desired_track),
            target_pitch_deg: Some(pitch_cmd),
            target_speed_kt: Some(self.config.performance.vref_kt),
            // Power to idle in the roundout, as a pilot would.
            throttle_limit: Some((0.0, 0.0)),
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

impl LandingPlanHolder for PatternFlyProfile {
    fn runway_frame(&self) -> &RunwayFrame {
        &self.runway_frame
    }

    fn preferred_exit(&self) -> Option<&RunwayExitPlan> {
        self.preferred_exit.as_ref()
    }

    fn arrival(&self) -> Option<&ArrivalPlan> {
        self.arrival.as_ref()
    }

    fn set_landing_plan(
        &mut self,
        exit: Option<RunwayExitPlan>,
        arrival: Option<ArrivalPlan>,
    ) -> Result<(), String> {
        self.preferred_exit = exit;
        self.arrival = arrival;
        Ok(())
    }
}

impl GuidanceProfile for PatternFlyProfile {
    fn landing_plan_mut(&mut self) -> Option<&mut dyn LandingPlanHolder> {
        Some(self)
    }

    fn pattern_fly_mut(&mut self) -> Option<&mut PatternFlyProfile> {
        Some(self)
    }

    fn name(&self) -> &'static str { "pattern_fly" }
    fn owns(&self) -> AxisSet {
        AxisSet::ALL
    }

    fn contribute(&mut self, state: &AircraftState, dt: f64) -> ProfileTick {
        // Dynamic leg rebuild on triggers: the subsequent leg must start
        // at the aircraft's *current* runway-frame position, not at a
        // stale precomputed point, or "extend then turn X" would leave
        // the next leg behind the aircraft.
        // Squared corners start a turn radius early, so the turn rolls out
        // on the next leg; the follower's own base-to-final point (65%
        // along base) is replaced.
        let mut clearances = self.clearances.clone();
        if self.square.into_base && self.phase == FlightPhase::Downwind && clearances.turn_base && !self.triggers.turn_base {
            if let Some(rx) = state.runway_x_ft {
                if rx <= self.pattern.base_turn_x_ft + square_lead_ft(state.gs_kt, SQUARE_BASE_BANK_DEG) {
                    self.triggers.turn_base = true;
                }
            }
        }
        if self.square.into_final && self.phase == FlightPhase::Base && clearances.turn_final {
            clearances.turn_final = false;
            if let Some(ry) = state.runway_y_ft {
                let closing = wrap_degrees_180(state.track_deg - self.runway_frame.runway.course_deg).abs() < 150.0;
                if closing && ry.abs() <= square_lead_ft(state.gs_kt, SQUARE_FINAL_BANK_DEG) {
                    self.triggers.turn_final = true;
                }
            }
        }
        // An early base (a short approach) pulls the base leg in, to no
        // closer than MIN_BASE_X_FT before the threshold.
        if self.triggers.turn_base && self.phase == FlightPhase::Downwind {
            if let Some(rx) = state.runway_x_ft {
                self.place_base_at(rx);
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
            clearances: &clearances,
            force_go_around: self.force_go_around_trigger,
            stay_in_pattern: true,
            touch_and_go: self.touch_and_go_trigger,
        });
        // Touchdown: everything on the ground belongs to
        // LandingRolloutProfile. Hand off now and let it fly this tick, so
        // there's no frame of airborne guidance on the runway. A go-around
        // trigger armed before touchdown dies with this profile.
        // Debounced: the flare continues until the wheels have stayed down
        // for TOUCHDOWN_SETTLE_S, so a bounce stays with this profile.
        self.phase = touchdown_debounce(self.phase, state.on_ground, dt, &mut self.ground_contact_s);
        if self.phase == FlightPhase::Rollout {
            let rollout = LandingRolloutProfile::new(
                self.config.clone(),
                self.runway_frame.clone(),
                self.preferred_exit.take(),
            )
            .with_go_around_reason(self.last_go_around_reason.clone())
            .with_arrival(self.arrival.take());
            return touchdown_handoff(rollout, state, dt);
        }
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
                self.last_go_around_reason = Some(if let Some(r) = self.go_around_reason_override.take() {
                    r
                } else if manual_go_around {
                    "manual_trigger".to_string()
                } else if let Some(r) = safety.reason.as_ref() {
                    r.clone()
                } else {
                    "unknown".to_string()
                });
            }
        }
        // Go-around complete (back at pattern altitude): re-enter the
        // pattern for another approach, as a pilot would at a
        // non-towered field. ATC vectors or other plans are the LLM's to
        // apply by engaging something else.
        if self.phase == FlightPhase::GoAround && self.go_around_climb_settled(state) {
            self.phase = FlightPhase::Crosswind;
            // A new circuit: drop the abandoned approach's leg extensions,
            // pending turn triggers and any armed touch-and-go.
            self.leg_extensions = PatternLegExtensions {
                crosswind_ft: 0.0,
                downwind_ft: self.config.pattern.default_extension_ft,
            };
            self.triggers = PatternTriggers::default();
            self.touch_and_go_trigger = false;
            self.rebuild_pattern();
        }
        self.update_corner(state, previous_phase);
        let guidance = self.guidance_for_phase(state, self.phase);
        let guidance = self.safety_monitor.apply_limits(guidance, self.phase);

        ProfileTick::contribution_only(guidance_to_contribution(guidance))
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

    fn spatial_overlay(&self) -> Option<SpatialOverlay> {
        let p = &self.pattern;
        let pattern_legs = vec![
            LabeledLeg::new("ENTRY", p.entry_leg),
            LabeledLeg::new("DWN", p.downwind_leg),
            LabeledLeg::new("BASE", p.base_leg),
            LabeledLeg::new("FINAL", p.final_leg),
        ];
        let active_pattern_leg = match self.phase {
            FlightPhase::PatternEntry => Some(0),
            FlightPhase::Downwind => Some(1),
            FlightPhase::Base => Some(2),
            FlightPhase::Final => Some(3),
            _ => None,
        };
        let (route, target_label) = match &self.preferred_exit {
            Some(exit) => (
                exit_route_legs(&self.runway_frame, exit),
                Some(exit.taxiway_name.clone()),
            ),
            None => (Vec::new(), None),
        };
        Some(SpatialOverlay {
            runway: Some(self.runway_frame.runway.clone()),
            pattern_legs,
            active_pattern_leg,
            route,
            target_label,
            ..Default::default()
        })
    }

    fn next_transition_hint(&self, state: &AircraftState) -> Option<TransitionHint> {
        // Distance helpers in feet → ETA seconds via current ground speed.
        // GS below ~5 kt makes ETA noisy; we still emit it so the operator
        // sees the countdown, but cap it so the UI stays readable.
        let eta_from = |distance_ft: f64| -> Option<f64> {
            if state.gs_kt < 5.0 || distance_ft <= 0.0 {
                return None;
            }
            let gs_fps = state.gs_kt * 1.6878; // kt → ft/s
            let eta = distance_ft / gs_fps;
            if eta.is_finite() && eta > 0.0 {
                Some(eta.min(900.0))
            } else {
                None
            }
        };
        match self.phase {
            FlightPhase::TakeoffRoll => Some(TransitionHint {
                next_phase: FlightPhase::Rotate,
                condition_text: format!(
                    "rotate at Vr {:.0} kt (now {:.0})",
                    self.config.performance.vr_kt, state.ias_kt
                ),
                eta_s: None,
            }),
            FlightPhase::Rotate => Some(TransitionHint {
                next_phase: FlightPhase::InitialClimb,
                condition_text: format!(
                    "wheels off and AGL ≥ {:.0} ft",
                    self.config.phase_gates.rotate_agl_ft
                ),
                eta_s: None,
            }),
            FlightPhase::InitialClimb => {
                let target_agl = (self.config.pattern.altitude_agl_ft - 300.0).max(400.0);
                Some(TransitionHint {
                    next_phase: FlightPhase::Crosswind,
                    condition_text: format!(
                        "AGL ≥ {:.0} ft (now {:.0})",
                        target_agl, state.alt_agl_ft
                    ),
                    eta_s: None,
                })
            }
            FlightPhase::Crosswind => Some(TransitionHint {
                next_phase: FlightPhase::Downwind,
                condition_text: "downwind track captured & at offset".to_string(),
                eta_s: None,
            }),
            FlightPhase::Downwind => {
                // Distance along downwind to the (extended) base-turn point.
                let rx = state.runway_x_ft.unwrap_or(0.0);
                let target_x = self.pattern.base_turn_x_ft;
                let dist_ft = (rx - target_x).max(0.0);
                Some(TransitionHint {
                    next_phase: FlightPhase::Base,
                    condition_text: "abeam base-turn point".to_string(),
                    eta_s: eta_from(dist_ft),
                })
            }
            FlightPhase::Base => {
                let leg = self.pattern.base_leg;
                let path = leg.end_ft - leg.start_ft;
                let path_len = path.length().max(1.0);
                let rel = state.position_ft - leg.start_ft;
                let along = rel.dot(path.normalized()).clamp(0.0, path_len);
                let remaining_ft = (path_len * BASE_TO_FINAL_FRACTION - along).max(0.0);
                Some(TransitionHint {
                    next_phase: FlightPhase::Final,
                    condition_text: format!(
                        "established on final / {:.0}% along base",
                        BASE_TO_FINAL_FRACTION * 100.0
                    ),
                    eta_s: eta_from(remaining_ft),
                })
            }
            FlightPhase::Final => {
                let target_agl = self.config.flare.roundout_height_ft;
                let descent_remaining_ft = (state.landing_agl_ft() - target_agl).max(0.0);
                let eta = if state.vs_fpm < -10.0 {
                    // -vs_fpm ft/min → divide by 60 → ft/s
                    let descent_fps = (-state.vs_fpm) / 60.0;
                    if descent_fps > 0.5 && descent_remaining_ft > 0.0 {
                        Some((descent_remaining_ft / descent_fps).min(900.0))
                    } else {
                        None
                    }
                } else {
                    None
                };
                Some(TransitionHint {
                    next_phase: FlightPhase::Roundout,
                    condition_text: format!(
                        "AGL ≤ {:.0} ft (now {:.0})",
                        target_agl, state.landing_agl_ft()
                    ),
                    eta_s: eta,
                })
            }
            FlightPhase::Roundout => Some(TransitionHint {
                next_phase: FlightPhase::Flare,
                condition_text: format!(
                    "AGL ≤ {:.0} ft",
                    self.config.flare.flare_start_ft
                ),
                eta_s: None,
            }),
            FlightPhase::Flare => Some(TransitionHint {
                next_phase: FlightPhase::Rollout,
                condition_text: "wheels on the ground".to_string(),
                eta_s: None,
            }),
            FlightPhase::GoAround => Some(TransitionHint {
                next_phase: FlightPhase::Crosswind,
                condition_text: format!(
                    "climb on runway heading to pattern alt {:.0} ft, then turn crosswind and fly the pattern again",
                    self.config.pattern_altitude_msl_ft()
                ),
                eta_s: None,
            }),
            _ => None,
        }
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
    /// Heading tolerance (degrees) for declaring the pose reached.
    pub pose_heading_tol_deg: f64,
    /// Creep speed used while pivoting to target heading — the nose wheel
    /// needs some forward motion to bite, so we can't sit at 0 kt while
    /// the heading still needs to rotate.
    pub pose_creep_speed_kt: f64,
    /// Nose-wheel turn radius (ft) for placing pose lead-in points.
    pub pose_turn_radius_ft: f64,
    /// Re-approach point back along the pose heading, set when the
    /// aircraft passes the spot without settling on it.
    pose_lead_in: Option<Vec2>,
    /// Dedup'd, display-ready route summary ("A→B→C"). Built once at
    /// construction from `taxiway_names` because `mode_line_suffix` runs
    /// every tick.
    route_summary: Option<String>,
    /// Where this taxi is going (e.g. a parking spot name), shown in the
    /// mode line / status so the LLM can tell taxis apart.
    pub destination: Option<String>,
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
        let route_summary = Self::build_route_summary(&taxiway_names);
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
            // Position tolerance must sit inside the pivot radius: the
            // pose only counts as reached once the aircraft has switched
            // from steering toward the spot to turning onto its heading.
            pose_heading_tol_deg: 8.0,
            pose_creep_speed_kt: 2.0,
            pose_turn_radius_ft: crate::config::GroundHandlingConfig::default().min_turn_radius_ft(),
            pose_lead_in: None,
            route_summary,
            destination: None,
        }
    }

    fn build_route_summary(taxiway_names: &[String]) -> Option<String> {
        let mut out: Vec<&str> = Vec::new();
        for n in taxiway_names {
            let n = n.as_str();
            if n.is_empty() || n.starts_with('(') {
                continue;
            }
            if out.last().is_none_or(|last| *last != n) {
                out.push(n);
            }
        }
        if out.is_empty() {
            None
        } else {
            Some(out.join("→"))
        }
    }

    /// Set the terminal pose: after the last leg, steer onto it and stop.
    /// The route should arrive roughly along the pose heading (see
    /// `pose_lead_in_point`); a nose-wheel aircraft can't turn in place.
    pub fn with_final_pose(mut self, pose: crate::types::TaxiPose) -> Self {
        self.final_pose = Some(pose);
        self
    }

    /// Where a route coming from `from` should hand over to the pose
    /// phase: back along the pose heading, far enough for a nose wheel
    /// turning at `turn_radius_ft` to swing onto the heading (the arc's
    /// tangent length) plus a straight run to settle. `None` when the
    /// approach is already about aligned. Never more than most of the
    /// distance from `from`.
    pub fn pose_lead_in_point(
        from: Vec2,
        pose: crate::types::TaxiPose,
        turn_radius_ft: f64,
    ) -> Option<Vec2> {
        const ALIGNED_DEG: f64 = 10.0;
        const SETTLE_FT: f64 = 20.0;
        let to_pose = pose.position_ft - from;
        let turn_deg = wrap_degrees_180(vector_to_heading(to_pose) - pose.heading_deg).abs();
        if turn_deg <= ALIGNED_DEG || to_pose.length() < 1.0 {
            return None;
        }
        let tangent_ft = turn_radius_ft * (turn_deg.min(170.0) / 2.0).to_radians().tan();
        let lead_in_ft = (SETTLE_FT + tangent_ft).min(0.8 * to_pose.length());
        Some(pose.position_ft - heading_to_vector(pose.heading_deg, lead_in_ft))
    }

    /// Taxi to a parking spot: label it and set the parking brake there
    /// (unlike a hold-short line, where the next step is line-up).
    pub fn parking_at(mut self, spot_name: impl Into<String>) -> Self {
        self.destination = Some(spot_name.into());
        self
    }

    /// On the pose heading, inside the pivot radius, and at the spot
    /// along the heading (within the couple of feet the aircraft coasts
    /// while braking): creeping on would only overshoot.
    fn pose_settled(&self, state: &AircraftState, pose: crate::types::TaxiPose) -> bool {
        const COAST_FT: f64 = 2.0;
        let aligned =
            wrap_degrees_180(pose.heading_deg - state.heading_deg).abs() <= self.pose_heading_tol_deg;
        aligned
            && (pose.position_ft - state.position_ft).length() < POSE_PIVOT_RADIUS_FT
            && Self::spot_ahead_ft(state, pose) <= COAST_FT
    }

    /// Pose reached: settled on it and essentially stopped.
    fn pose_reached(&self, state: &AircraftState, pose: crate::types::TaxiPose) -> bool {
        self.pose_settled(state, pose) && state.gs_kt <= self.pose_creep_speed_kt + 0.5
    }

    fn route_summary(&self) -> Option<&str> {
        self.route_summary.as_deref()
    }

    fn current_leg(&self) -> Option<StraightLeg> {
        self.legs_ft.get(self.current_idx).copied()
    }

    fn advance_if_reached(&mut self, state: &AircraftState) {
        let Some(current) = self.current_leg() else {
            return;
        };
        let dist_to_end = (state.position_ft - current.end_ft).length();
        // Also advance if past the endpoint along-track: a sideways
        // overshoot can leave Euclidean distance > threshold even
        // though the aircraft has crossed the leg end.
        let (along, _, leg_len) = leg_projection(state, current);
        let passed = (current.end_ft - current.start_ft).length() > 1.0 && along > leg_len;
        if dist_to_end < self.leg_advance_distance_ft || passed {
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

    /// How far ahead the spot is along its own heading (negative once
    /// passed).
    fn spot_ahead_ft(state: &AircraftState, pose: crate::types::TaxiPose) -> f64 {
        (pose.position_ft - state.position_ft).dot(heading_to_vector(pose.heading_deg, 1.0))
    }

    fn pose_regime(&self, state: &AircraftState, pose: crate::types::TaxiPose) -> PoseRegime {
        if self.pose_settled(state, pose) {
            PoseRegime::Settled
        } else if self.pose_lead_in.is_some() {
            PoseRegime::Reapproach
        } else {
            PoseRegime::RollIn
        }
    }

    /// Start or finish a re-approach. Passing the spot without settling
    /// on it (still turning, or wide) sets a lead-in point a full turn's
    /// width back along the pose heading; the aircraft swings round to it
    /// (turning onto the pose heading as it nears it) and rolls in again.
    /// It's done once the aircraft is back there and heading roughly
    /// along the pose.
    fn update_pose_lead_in(&mut self, state: &AircraftState, pose: crate::types::TaxiPose) {
        let back_ft = 20.0 + 4.0 * self.pose_turn_radius_ft;
        let ahead_ft = Self::spot_ahead_ft(state, pose);
        match self.pose_lead_in {
            None if ahead_ft <= 0.0 && !self.pose_settled(state, pose) => {
                self.pose_lead_in =
                    Some(pose.position_ft - heading_to_vector(pose.heading_deg, back_ft));
            }
            Some(_) => {
                let roughly_along =
                    wrap_degrees_180(pose.heading_deg - state.heading_deg).abs() <= 45.0;
                if ahead_ft >= 0.5 * back_ft && roughly_along {
                    self.pose_lead_in = None;
                }
            }
            None => {}
        }
    }

    /// Pose-phase target speed:
    ///
    /// - **Settled**: 0.
    /// - **Roll-in**: creep while turning onto the heading (a C172's nose
    ///   wheel has no yaw authority at a standstill, so it needs some
    ///   motion), then a speed that tapers to creep at the spot.
    /// - **Re-approach**: throttled by how far the aircraft is off the
    ///   bearing to the lead-in point, so it turns before it accelerates.
    fn pose_target_speed(&self, state: &AircraftState, pose: crate::types::TaxiPose) -> f64 {
        match self.pose_regime(state, pose) {
            PoseRegime::Settled => 0.0,
            PoseRegime::RollIn => {
                let hdg_err = wrap_degrees_180(pose.heading_deg - state.heading_deg).abs();
                if hdg_err > 15.0 {
                    self.pose_creep_speed_kt
                } else {
                    // Down to creep for the last 20 ft so it stops near the spot.
                    let ahead_ft = Self::spot_ahead_ft(state, pose);
                    (self.pose_creep_speed_kt + 0.1 * (ahead_ft - 20.0).max(0.0))
                        .min(self.turn_speed_kt)
                }
            }
            PoseRegime::Reapproach => {
                let to_target = self.pose_lead_in.unwrap_or(pose.position_ft) - state.position_ft;
                let off_bearing =
                    wrap_degrees_180(vector_to_heading(to_target) - state.heading_deg).abs();
                if off_bearing > 45.0 {
                    self.pose_creep_speed_kt
                } else {
                    self.turn_speed_kt
                }
            }
        }
    }

    fn contribute_pose(
        &mut self,
        state: &AircraftState,
        pose: crate::types::TaxiPose,
    ) -> ProfileTick {
        self.update_pose_lead_in(state, pose);
        if self.pose_reached(state, pose) {
            self.finished = true;
        }
        let target_speed = self.pose_target_speed(state, pose);
        let common = ProfileContribution {
            vertical_mode: Some(VerticalMode::Taxi),
            target_speed_kt: Some(target_speed),
            target_pitch_deg: Some(0.0),
            throttle_limit: Some((0.0, 1.0)),
            ..Default::default()
        };
        let contribution = match self.pose_lead_in {
            // Swing round toward the lead-in point.
            Some(lead_in) => ProfileContribution {
                lateral_mode: Some(LateralMode::TaxiPose),
                target_waypoint: Some(crate::types::Waypoint {
                    name: self.destination.clone().unwrap_or_else(|| "hold_short".to_string()),
                    position_ft: lead_in,
                    altitude_ft: None,
                }),
                target_heading_deg: Some(pose.heading_deg),
                ..common
            },
            // Roll in along the line through the spot on its heading.
            None => {
                let back_ft = Self::spot_ahead_ft(state, pose).max(0.0) + 50.0;
                ProfileContribution {
                    lateral_mode: Some(LateralMode::TaxiFollow),
                    target_path: Some(StraightLeg {
                        start_ft: pose.position_ft - heading_to_vector(pose.heading_deg, back_ft),
                        end_ft: pose.position_ft,
                    }),
                    ..common
                }
            }
        };
        ProfileTick::contribution_only(contribution)
    }
}

/// What the pose phase is doing this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PoseRegime {
    /// On the pose (or stopped abeam it, aligned): hold.
    Settled,
    /// Following the line through the spot on its heading.
    RollIn,
    /// Passed the spot unsettled; swinging round to a lead-in point.
    Reapproach,
}

impl PoseRegime {
    fn label(self) -> &'static str {
        match self {
            PoseRegime::Settled => "settled",
            PoseRegime::RollIn => "roll-in",
            PoseRegime::Reapproach => "re-approach",
        }
    }
}

impl GuidanceProfile for TaxiProfile {
    fn name(&self) -> &'static str {
        match self.mode {
            TaxiProfileMode::Taxi => "taxi",
            TaxiProfileMode::LineUp => "line_up",
        }
    }
    fn owns(&self) -> AxisSet {
        AxisSet::ALL
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
        // Parked at a pose target (a parking spot): set the parking brake,
        // as a pilot would on arrival. It stays set while this profile
        // remains engaged; the next taxi releases it.
        let parked = self.finished && self.final_pose.is_some() && self.destination.is_some();
        if parked {
            return ProfileTick::contribution_only(ProfileContribution {
                lateral_mode: Some(LateralMode::TaxiFollow),
                vertical_mode: Some(VerticalMode::PitchHold),
                target_pitch_deg: Some(0.0),
                throttle_limit: Some((0.0, 0.0)),
                parking_brake: Some(1.0),
                ..Default::default()
            });
        }
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

    fn spatial_overlay(&self) -> Option<SpatialOverlay> {
        let route = self
            .legs_ft
            .iter()
            .enumerate()
            .map(|(i, leg)| {
                let name = self.taxiway_names.get(i).map(String::as_str).unwrap_or("");
                let label = if name.starts_with('(') { "" } else { name };
                LabeledLeg::new(label, *leg)
            })
            .collect();
        let target_label = self.destination.clone().or_else(|| {
            self.final_pose.map(|_| match self.mode {
                TaxiProfileMode::LineUp => "LINE UP".to_string(),
                TaxiProfileMode::Taxi => "HOLD SHORT".to_string(),
            })
        });
        Some(SpatialOverlay {
            route,
            route_active_idx: Some(self.current_idx.min(self.legs_ft.len())),
            target_pose: self.final_pose,
            target_label,
            ..Default::default()
        })
    }

    fn mode_line_suffix(&self, state: &AircraftState) -> Option<String> {
        let route = self.route_summary();
        let total_remaining = self.remaining_distance_ft(state);
        let tail = if self.in_pose_phase() {
            let pose = self.final_pose.unwrap();
            let d = (pose.position_ft - state.position_ft).length();
            let phase = if self.finished {
                "parked"
            } else {
                self.pose_regime(state, pose).label()
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
        let body = match route {
            Some(r) => format!("via {r} · {tail}"),
            None => tail,
        };
        Some(match &self.destination {
            Some(dest) => format!("→ {dest} {body}"),
            None => body,
        })
    }

    fn debug_line(&self, state: &AircraftState) -> Option<String> {
        if self.in_pose_phase() {
            let pose = self.final_pose.unwrap();
            let d = (pose.position_ft - state.position_ft).length();
            let hdg_err = wrap_degrees_180(pose.heading_deg - state.heading_deg);
            let phase = if self.finished {
                "parked"
            } else {
                self.pose_regime(state, pose).label()
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

/// World-frame legs of a runway-exit connector, each labelled with the
/// exit's taxiway name. Shared by `pattern_fly` (planned exit) and
/// `landing_rollout` (exit being flown).
pub(crate) fn exit_route_legs(frame: &RunwayFrame, exit: &RunwayExitPlan) -> Vec<LabeledLeg> {
    (0..exit.path_runway_ft.len().saturating_sub(1))
        .filter_map(|i| exit_leg(frame, exit, i))
        .map(|leg| LabeledLeg::new(exit.taxiway_name.clone(), leg))
        .collect()
}

/// World-frame leg `leg_idx` of an exit connector.
pub(crate) fn exit_leg(frame: &RunwayFrame, exit: &RunwayExitPlan, leg_idx: usize) -> Option<StraightLeg> {
    let start = *exit.path_runway_ft.get(leg_idx)?;
    let end = *exit.path_runway_ft.get(leg_idx + 1)?;
    Some(StraightLeg { start_ft: frame.to_world_frame(start), end_ft: frame.to_world_frame(end) })
}
