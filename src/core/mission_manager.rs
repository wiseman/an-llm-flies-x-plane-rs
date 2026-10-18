//! `PilotCore` — the per-tick composer that reads dynamics, composes
//! guidance from active profiles, and emits actuator commands.
//!
//! Thread model: callers lock `PilotCore` through a single `Mutex` held by
//! the control loop on every `update()`. Tool handlers (LLM worker, TUI)
//! briefly acquire the same lock to mutate the profile stack.

use std::collections::BTreeSet;

use chrono::{DateTime, Utc};

use crate::config::ConfigBundle;
use crate::control::bank_hold::{BankController, CoordinationController};
use crate::control::centerline_rollout::CenterlineRolloutController;
use crate::control::ground_speed::GroundSpeedController;
use crate::control::nose_wheel::NoseWheelController;
use crate::control::pitch_for_airspeed::PitchForAirspeedController;
use crate::control::pitch_hold::PitchController;
use crate::control::tecs_lite::TECSLite;
use crate::core::profiles::{
    idle_profile_trio, Axis, AxisSet, GuidanceProfile, IdleLateralProfile, IdleSpeedProfile,
    IdleVerticalProfile, PatternMetadata, ProfileContribution, ProfileTick, TransitionHint,
};
use crate::core::landing_rollout::LandingPlanHolder;
use crate::core::spatial::SpatialSnapshot;
use crate::core::state_estimator::estimate_aircraft_state;
use crate::guidance::flare_profile::FlareController;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::sim::simple_dynamics::DynamicsState;
use crate::types::{
    clamp, wrap_degrees_180, ActuatorCommands, AircraftState, FlightPhase, GuidanceTargets,
    LateralMode, Vec2, VerticalMode,
};

/// LLM-declared top-level mission goal — what the pilot is trying to do
/// right now, in human-readable form. Set via the `set_goal` tool;
/// cleared via `clear_goal`. Surfaced verbatim in the TUI's MISSION
/// INTENT pane so the supervisor can see the active intent without
/// having to infer it from active profiles.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MissionGoal {
    pub kind: String,
    pub target: Option<String>,
    pub notes: Option<String>,
}

/// LLM-declared active ATC clearance, pinned on screen until cleared.
/// The radio buffer keeps the full ATC text history; this is the
/// load-bearing one-line summary of the operating contract right now.
#[derive(Debug, Clone, PartialEq)]
pub struct ActiveClearance {
    pub text: String,
    pub source: ClearanceSource,
    pub freq_mhz: Option<f64>,
    pub issued_at: DateTime<Utc>,
}

/// Which controller issued the clearance. Free-text `Other(label)` covers
/// CTAF / FBO / company / etc. without inflating the enum.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ClearanceSource {
    Tower,
    Ground,
    Approach,
    Departure,
    Center,
    Unicom,
    Other(String),
}

impl ClearanceSource {
    pub fn label(&self) -> &str {
        match self {
            ClearanceSource::Tower => "TWR",
            ClearanceSource::Ground => "GND",
            ClearanceSource::Approach => "APP",
            ClearanceSource::Departure => "DEP",
            ClearanceSource::Center => "CTR",
            ClearanceSource::Unicom => "CTAF",
            ClearanceSource::Other(s) => s.as_str(),
        }
    }

    pub fn from_str(s: &str) -> Self {
        match s.to_ascii_lowercase().as_str() {
            "tower" | "twr" => ClearanceSource::Tower,
            "ground" | "gnd" => ClearanceSource::Ground,
            "approach" | "app" => ClearanceSource::Approach,
            "departure" | "dep" => ClearanceSource::Departure,
            "center" | "ctr" | "centre" => ClearanceSource::Center,
            "unicom" | "ctaf" => ClearanceSource::Unicom,
            other => ClearanceSource::Other(other.to_string()),
        }
    }
}

/// Radio-altimeter samples over the pavement count below this height...
const SURFACE_MAX_AGL_FT: f64 = 400.0;
/// ...and this far beside its edges.
const SURFACE_SIDE_FT: f64 = 50.0;
/// A sample this far off the published elevation is terrain, not runway.
const SURFACE_MAX_OFFSET_FT: f64 = 150.0;
/// Samples spread over this much runway fit its slope (fewer: their mean).
const SURFACE_FIT_SPAN_FT: f64 = 300.0;
/// Steepest runway slope believed.
const SURFACE_MAX_SLOPE: f64 = 0.02;

/// Radio-altimeter samples of a runway's surface, as a running line fit
/// of elevation against distance down the runway.
#[derive(Debug, Clone)]
struct SurfaceElevation {
    /// The runway it belongs to (its origin in the world frame).
    runway: Vec2,
    published_ft: f64,
    n: f64,
    sx: f64,
    se: f64,
    sxx: f64,
    sxe: f64,
    min_x: f64,
    max_x: f64,
    reported: bool,
}

impl SurfaceElevation {
    fn new(runway: Vec2, published_ft: f64) -> Self {
        Self {
            runway,
            published_ft,
            n: 0.0,
            sx: 0.0,
            se: 0.0,
            sxx: 0.0,
            sxe: 0.0,
            min_x: f64::INFINITY,
            max_x: f64::NEG_INFINITY,
            reported: false,
        }
    }

    fn add(&mut self, x: f64, elevation_ft: f64) {
        self.n += 1.0;
        self.sx += x;
        self.se += elevation_ft;
        self.sxx += x * x;
        self.sxe += x * elevation_ft;
        self.min_x = self.min_x.min(x);
        self.max_x = self.max_x.max(x);
    }

    fn span_ft(&self) -> f64 {
        self.max_x - self.min_x
    }

    /// The surface elevation `x` ft down the runway.
    fn at(&self, x: f64) -> f64 {
        let (mx, me) = (self.sx / self.n, self.se / self.n);
        if self.span_ft() < SURFACE_FIT_SPAN_FT {
            return me;
        }
        let var = self.sxx / self.n - mx * mx;
        let cov = self.sxe / self.n - mx * me;
        let slope = if var > 1.0 { (cov / var).clamp(-SURFACE_MAX_SLOPE, SURFACE_MAX_SLOPE) } else { 0.0 };
        me + slope * (x - mx)
    }
}

#[derive(Debug, Clone)]
pub struct StatusSnapshot {
    pub t_sim: f64,
    pub active_profiles: Vec<String>,
    pub phase: Option<FlightPhase>,
    pub state: AircraftState,
    pub last_commands: ActuatorCommands,
    pub last_guidance: Option<GuidanceTargets>,
    pub go_around_reason: Option<String>,
    pub airport_ident: Option<String>,
    pub runway_id: Option<String>,
    pub field_elevation_ft: Option<f64>,
    /// Per-profile debug lines collected from `GuidanceProfile::debug_line`.
    /// Surfaced in the status pane and log so taxi / pattern sequencers can
    /// expose their internal state (leg index, crosstrack, target speed,
    /// etc.) for live debugging.
    pub debug_lines: Vec<String>,
    /// Names of active profiles reporting `is_complete() == true`. The
    /// heartbeat pump uses transitions in this set to wake the LLM the
    /// moment an automated sequence finishes (e.g. taxi arrives at the
    /// hold-short), instead of making it wait for the idle heartbeat.
    pub completed_profiles: Vec<String>,
    /// Parallel to `active_profiles`: a short, single-line
    /// human-readable summary of what the profile is doing right now
    /// (taxi leg progress, pattern leg, etc.). `None` entries mean
    /// "nothing extra to show." Used only for display — the heartbeat
    /// explicitly ignores this because it churns with state changes.
    pub profile_mode_line_suffixes: Vec<Option<String>>,
    /// LLM-declared mission goal (set via `set_goal`).
    pub mission_goal: Option<MissionGoal>,
    /// LLM-declared active ATC clearance (set via `record_clearance`).
    pub active_clearance: Option<ActiveClearance>,
    /// Optional next-transition hint from a profile that owns its phase
    /// machine. Today only `pattern_fly` provides this.
    pub transition_hint: Option<TransitionHint>,
    /// Index into `active_profiles` of the profile that owns each axis,
    /// computed via `GuidanceProfile::owns()` so the TUI doesn't have to
    /// substring-match profile names. Each axis always has exactly one
    /// owner (idle profiles cover orphaned axes), so `None` only occurs
    /// during the brief window between displacement and the next tick.
    pub lateral_owner_idx: Option<usize>,
    pub vertical_owner_idx: Option<usize>,
    pub speed_owner_idx: Option<usize>,
    /// The active plan's status (`GuidanceProfile::status_json`), if a
    /// profile publishes one.
    pub plan_status: Option<serde_json::Value>,
    /// Recent profile events, oldest first, each with a sequence number
    /// so the heartbeat pump can tell new ones from old.
    pub events: Vec<(u64, String)>,
    /// Nearby traffic as the pilot would hear it called: type, where in
    /// the pattern, clock position, distance, relative altitude.
    pub traffic: Vec<serde_json::Value>,
}

impl StatusSnapshot {
    /// Builder-style factory for tests: produces an empty mid-mission
    /// snapshot that callers override selectively. Mirrors
    /// `AircraftState::synthetic_default` so test fixtures don't have
    /// to spell out every field of the snapshot every time we add one.
    pub fn synthetic_default() -> Self {
        StatusSnapshot {
            t_sim: 0.0,
            active_profiles: Vec::new(),
            phase: None,
            state: AircraftState::synthetic_default(),
            last_commands: ActuatorCommands {
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
            },
            last_guidance: None,
            go_around_reason: None,
            airport_ident: None,
            runway_id: None,
            field_elevation_ft: None,
            debug_lines: Vec::new(),
            completed_profiles: Vec::new(),
            profile_mode_line_suffixes: Vec::new(),
            mission_goal: None,
            active_clearance: None,
            transition_hint: None,
            lateral_owner_idx: None,
            vertical_owner_idx: None,
            speed_owner_idx: None,
            plan_status: None,
            events: Vec::new(),
            traffic: Vec::new(),
        }
    }
}

/// An engage or disengage refused because a profile it would remove
/// reported that it can't be preempted right now.
#[derive(Debug, Clone, PartialEq)]
pub struct PreemptionRefused {
    pub profile: &'static str,
    pub reason: String,
}

impl std::fmt::Display for PreemptionRefused {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} can't be displaced yet: {}", self.profile, self.reason)
    }
}

impl std::error::Error for PreemptionRefused {}

/// A cockpit setting outside the flight controls (parking brake, flap
/// handle) that tools and profiles both set. The latest request wins: a
/// tool's, or a profile's whenever its request changes. Each change goes
/// to the sim once through `ActuatorCommands`, so a change made in the
/// cockpit isn't overwritten every tick.
#[derive(Debug, Clone, Default)]
pub struct CockpitSetting<T> {
    value: Option<T>,
    last_profile_request: Option<T>,
    unsent: bool,
}

impl<T: Copy + PartialEq> CockpitSetting<T> {
    /// The setting as last requested, or `None` if nothing has set it yet.
    pub fn value(&self) -> Option<T> {
        self.value
    }

    /// A tool wrote `value` to the sim directly (so it takes effect even
    /// while the sim is paused): record it without sending it again.
    pub fn note_written(&mut self, value: T) {
        self.value = Some(value);
        self.unsent = false;
    }

    fn profile_request(&mut self, request: Option<T>) {
        if request != self.last_profile_request {
            self.last_profile_request = request;
            if let Some(v) = request {
                self.value = Some(v);
                self.unsent = true;
            }
        }
    }

    fn take_unsent(&mut self) -> Option<T> {
        std::mem::take(&mut self.unsent).then_some(self.value).flatten()
    }
}

pub struct PilotCore {
    pub config: ConfigBundle,
    pub runway_frame: RunwayFrame,
    pub bank_controller: BankController,
    pub coordination: CoordinationController,
    pub pitch_controller: PitchController,
    pub rollout_controller: CenterlineRolloutController,
    pub tecs: TECSLite,
    pub pitch_for_airspeed: PitchForAirspeedController,
    pub flare_controller: FlareController,
    pub nose_wheel_controller: NoseWheelController,
    pub ground_speed_controller: GroundSpeedController,
    pub active_profiles: Vec<Box<dyn GuidanceProfile>>,
    pub latest_snapshot: Option<StatusSnapshot>,
    /// Last phase any active profile reported via `pattern_metadata`. Lets
    /// `phase()` keep returning the post-landing phase (Rollout, RunwayExit,
    /// TaxiClear) after `pattern_fly` auto-releases on stop, instead of
    /// falling back to `Preflight` and confusing both the LLM and the
    /// scenario tests that assert on `final_phase`.
    last_known_phase: Option<FlightPhase>,
    last_lateral_mode: Option<LateralMode>,
    last_vertical_mode: Option<VerticalMode>,
    last_tecs_phase: Option<FlightPhase>,
    /// LLM-declared mission goal — what the pilot is trying to do.
    pub mission_goal: Option<MissionGoal>,
    /// LLM-declared active ATC clearance.
    pub active_clearance: Option<ActiveClearance>,
    pub parking_brake: CockpitSetting<f64>,
    pub flaps: CockpitSetting<i32>,
    /// Ring of the last `EVENT_LOG_LEN` profile events, with sequence numbers.
    event_log: std::collections::VecDeque<(u64, String)>,
    event_seq: u64,
    /// The reference runway's surface as the radio altimeter measures
    /// it, and the published field elevation it replaces. apt.dat has
    /// one elevation per airport; the sim's pavement can sit tens of
    /// feet off it (KVNY: published 800, touchdown zone 784), which
    /// puts the glidepath that far above the real surface.
    surface: Option<SurfaceElevation>,
    /// Other aircraft, as sensed (scripted in evals). Plans read it
    /// through `PlanProfile::with_traffic`.
    pub traffic: crate::core::traffic::TrafficPicture,
}

/// How many profile events the status snapshot carries.
const EVENT_LOG_LEN: usize = 32;

impl PilotCore {
    /// Traffic within listing range, nearest first, placed against the
    /// reference runway's pattern.
    fn traffic_status(&self, state: &AircraftState) -> Vec<serde_json::Value> {
        use crate::core::traffic::{in_list_range, relative, view};
        let mut targets = self.traffic.snapshot();
        targets.retain(|t| in_list_range(state.position_ft, t));
        targets.sort_by(|a, b| {
            let da = a.position_ft.distance_to(state.position_ft);
            let db = b.position_ft.distance_to(state.position_ft);
            da.total_cmp(&db)
        });
        targets
            .iter()
            .map(|t| {
                let v = view(&self.runway_frame, t);
                serde_json::json!({
                    "id": t.id,
                    "type": t.kind,
                    "callsign": t.callsign,
                    "where": if v.where_.is_empty() { serde_json::Value::Null } else { v.where_.into() },
                    "relative": relative(state.position_ft, state.track_deg, state.alt_msl_ft, t),
                    "alt_msl_ft": (t.alt_msl_ft / 10.0).round() * 10.0,
                    "track_deg": t.track_deg.round(),
                    "gs_kt": t.gs_kt.round(),
                })
            })
            .collect()
    }

    /// Re-list traffic in the latest snapshot (after the picture changed
    /// outside a tick, e.g. an eval placing its traffic before the first).
    pub fn refresh_traffic_status(&mut self) {
        if let Some(state) = self.latest_snapshot.as_ref().map(|s| s.state.clone()) {
            let traffic = self.traffic_status(&state);
            if let Some(s) = self.latest_snapshot.as_mut() {
                s.traffic = traffic;
            }
        }
    }

    pub fn new(config: ConfigBundle) -> Self {
        let runway_frame = RunwayFrame::new(config.airport.runway.clone());
        let bank_controller = BankController::new(config.controllers.bank);
        let pitch_controller = PitchController::new(config.controllers.pitch);
        let tecs = TECSLite::new(config.controllers.tecs);
        let flare = FlareController::new(config.flare.clone());
        let idle = idle_profile_trio(config.performance.cruise_speed_kt);
        Self {
            runway_frame,
            bank_controller,
            coordination: CoordinationController::default(),
            pitch_controller,
            rollout_controller: CenterlineRolloutController::new(),
            tecs,
            pitch_for_airspeed: PitchForAirspeedController::new(),
            flare_controller: flare,
            nose_wheel_controller: NoseWheelController::new(),
            ground_speed_controller: GroundSpeedController::new(),
            active_profiles: idle,
            latest_snapshot: None,
            last_known_phase: None,
            last_lateral_mode: None,
            last_vertical_mode: None,
            last_tecs_phase: None,
            mission_goal: None,
            active_clearance: None,
            parking_brake: CockpitSetting::default(),
            flaps: CockpitSetting::default(),
            event_log: std::collections::VecDeque::new(),
            event_seq: 0,
            surface: None,
            traffic: Default::default(),
            config,
        }
    }

    pub fn set_mission_goal(&mut self, goal: MissionGoal) {
        self.mission_goal = Some(goal);
    }

    pub fn clear_mission_goal(&mut self) {
        self.mission_goal = None;
    }

    pub fn record_clearance(&mut self, clearance: ActiveClearance) {
        self.active_clearance = Some(clearance);
    }

    pub fn clear_active_clearance(&mut self) {
        self.active_clearance = None;
    }

    pub fn list_profile_names(&self) -> Vec<String> {
        self.active_profiles.iter().map(|p| p.name().to_string()).collect()
    }

    /// Engage a profile. Any active profile with an overlapping axis is
    /// displaced; returns the names of displaced profiles. Refused, with
    /// nothing changed, if a profile that would be displaced isn't
    /// preemptible right now (see `GuidanceProfile::preemption`).
    pub fn engage_profile(
        &mut self,
        profile: Box<dyn GuidanceProfile>,
    ) -> Result<Vec<String>, PreemptionRefused> {
        self.engage_profiles(vec![profile])
    }

    /// Engage several profiles atomically under one logical step. Returns the
    /// set of displaced profile names (excluding any profile that was itself
    /// reinstalled in the same call — "displaced" reflects only real
    /// removals). All-or-nothing: if any profile that would be displaced
    /// refuses preemption, nothing is engaged.
    pub fn engage_profiles(
        &mut self,
        new_profiles: Vec<Box<dyn GuidanceProfile>>,
    ) -> Result<Vec<String>, PreemptionRefused> {
        let claimed = new_profiles
            .iter()
            .fold(AxisSet::NONE, |acc, p| acc.union(p.owns()));
        self.check_preemptible(|existing| !existing.owns().is_disjoint(claimed))?;
        Ok(self.install_profiles(new_profiles))
    }

    /// Engage profiles built against a new reference runway (`config`'s
    /// airport section). The runway replaces the current one only if the
    /// engage goes through, so a refused engage leaves the active
    /// profiles measuring against the runway they were built for.
    pub fn engage_profiles_with_config(
        &mut self,
        config: ConfigBundle,
        new_profiles: Vec<Box<dyn GuidanceProfile>>,
    ) -> Result<Vec<String>, PreemptionRefused> {
        let claimed = new_profiles
            .iter()
            .fold(AxisSet::NONE, |acc, p| acc.union(p.owns()));
        self.check_preemptible(|existing| !existing.owns().is_disjoint(claimed))?;
        self.set_reference_config(config);
        Ok(self.install_profiles(new_profiles))
    }

    /// Make `config`'s runway the reference frame (the one `runway_x_ft`
    /// / `runway_y_ft` are measured against) without touching the profile
    /// stack. For a plan that moves its pattern to another runway in
    /// place.
    pub fn set_reference_config(&mut self, config: ConfigBundle) {
        self.runway_frame = RunwayFrame::new(config.airport.runway.clone());
        self.config = config;
        // Same runway: keep the measured surface over the published one.
        match &self.surface {
            Some(s) if s.runway == self.config.airport.runway.threshold_ft => {
                self.config.airport.field_elevation_ft = s.at(self.runway_frame.touchdown_runway_x_ft());
            }
            _ => self.surface = None,
        }
    }

    /// Measure the reference runway's surface: over the pavement below
    /// `SURFACE_MAX_AGL_FT` (or rolling on it) the radio altimeter gives
    /// the elevation under the aircraft. A line fitted through those
    /// samples (runways slope: KVNY 16R drops ~0.5%) gives the elevation
    /// at the aim point, which then stands in for the published one in
    /// the glidepath and the height above the field.
    fn measure_surface(&mut self, raw: &DynamicsState) {
        let Some(radio_ft) = raw.agl_ft else { return };
        if radio_ft > SURFACE_MAX_AGL_FT {
            return;
        }
        let p = self.runway_frame.to_runway_frame(raw.position_ft);
        let rw = &self.runway_frame.runway;
        if p.x < 0.0 || p.x > rw.length_ft || p.y.abs() > self.runway_frame.half_width_plus_ft(SURFACE_SIDE_FT) {
            return;
        }
        let published = self.surface.as_ref().map(|s| s.published_ft).unwrap_or(self.config.airport.field_elevation_ft);
        let elevation = raw.altitude_ft - radio_ft;
        if (elevation - published).abs() > SURFACE_MAX_OFFSET_FT {
            return;
        }
        let surface = self
            .surface
            .get_or_insert_with(|| SurfaceElevation::new(rw.threshold_ft, published));
        surface.add(p.x, elevation);
        let at_aim = surface.at(self.runway_frame.touchdown_runway_x_ft());
        if !surface.reported && surface.span_ft() >= SURFACE_FIT_SPAN_FT {
            surface.reported = true;
            self.event_seq += 1;
            self.event_log.push_back((
                self.event_seq,
                format!("runway surface at the touchdown zone measured at {at_aim:.0} ft (published field elevation {published:.0} ft)"),
            ));
            if self.event_log.len() > EVENT_LOG_LEN {
                self.event_log.pop_front();
            }
        }
        self.config.airport.field_elevation_ft = at_aim;
    }

    /// Ask every active profile matching `affected` whether it may be
    /// removed; the first refusal wins.
    fn check_preemptible(
        &self,
        affected: impl Fn(&dyn GuidanceProfile) -> bool,
    ) -> Result<(), PreemptionRefused> {
        for p in &self.active_profiles {
            if affected(p.as_ref()) {
                if let Err(reason) = p.preemption() {
                    return Err(PreemptionRefused { profile: p.name(), reason });
                }
            }
        }
        Ok(())
    }

    /// Unchecked install, used directly for profile hand-offs: a profile
    /// replacing itself has consented by definition.
    fn install_profiles(&mut self, new_profiles: Vec<Box<dyn GuidanceProfile>>) -> Vec<String> {
        let new_names: BTreeSet<&'static str> = new_profiles.iter().map(|p| p.name()).collect();
        let mut all_displaced: BTreeSet<String> = BTreeSet::new();
        for profile in new_profiles {
            let owns = profile.owns();
            let mut retained: Vec<Box<dyn GuidanceProfile>> = Vec::with_capacity(self.active_profiles.len());
            for existing in std::mem::take(&mut self.active_profiles) {
                if !existing.owns().is_disjoint(owns) {
                    all_displaced.insert(existing.name().to_string());
                } else {
                    retained.push(existing);
                }
            }
            retained.push(profile);
            self.active_profiles = retained;
        }
        let names_to_filter: BTreeSet<String> = new_names.iter().map(|s| s.to_string()).collect();
        let result: Vec<String> = all_displaced.difference(&names_to_filter).cloned().collect();
        result
    }

    /// Remove every profile named `name`, back-filling orphaned axes with
    /// idle profiles; returns the names of the idles added. Refused if the
    /// profile isn't preemptible right now.
    pub fn disengage_profile(&mut self, name: &str) -> Result<Vec<String>, PreemptionRefused> {
        self.check_preemptible(|p| p.name() == name)?;
        let mut removed_owns = AxisSet::NONE;
        let mut retained: Vec<Box<dyn GuidanceProfile>> = Vec::with_capacity(self.active_profiles.len());
        for p in std::mem::take(&mut self.active_profiles) {
            if p.name() == name {
                removed_owns = removed_owns.union(p.owns());
            } else {
                retained.push(p);
            }
        }
        if removed_owns.is_empty() {
            self.active_profiles = retained;
            return Ok(vec![]);
        }
        let covered = retained
            .iter()
            .fold(AxisSet::NONE, |acc, p| acc.union(p.owns()));
        let orphans = removed_owns.difference(covered);
        let mut added: Vec<String> = Vec::new();
        for axis in orphans.iter() {
            let idle: Box<dyn GuidanceProfile> = match axis {
                Axis::Lateral => Box::new(IdleLateralProfile),
                Axis::Vertical => Box::new(IdleVerticalProfile),
                Axis::Speed => Box::new(IdleSpeedProfile::new(self.config.performance.cruise_speed_kt)),
            };
            added.push(idle.name().to_string());
            retained.push(idle);
        }
        self.active_profiles = retained;
        Ok(added)
    }

    /// The first active profile whose concrete type is `T`.
    pub fn profile_mut<T: GuidanceProfile + 'static>(&mut self) -> Option<&mut T> {
        self.active_profiles
            .iter_mut()
            .find_map(|p| (**p).as_any_mut().downcast_mut::<T>())
    }

    /// The active profile carrying the landing's exit/arrival plan.
    pub fn landing_plan_mut(&mut self) -> Option<&mut dyn LandingPlanHolder> {
        self.active_profiles.iter_mut().find_map(|p| p.landing_plan_mut())
    }

    /// The traffic pattern being flown, if any (a plan's or a bare
    /// pattern profile's).
    pub fn pattern_fly_mut(&mut self) -> Option<&mut crate::core::profiles::PatternFlyProfile> {
        self.active_profiles.iter_mut().find_map(|p| p.pattern_fly_mut())
    }

    /// Geometry for the TUI spatial view: the first active profile's
    /// `spatial_overlay`, with the pilot's reference runway standing in
    /// when no profile names one. Read-only and built on demand, so the
    /// control loop never pays for it.
    pub fn spatial_snapshot(&self) -> SpatialSnapshot {
        let mut overlay = self
            .active_profiles
            .iter()
            .find_map(|p| p.spatial_overlay())
            .unwrap_or_default();
        let runway = overlay
            .runway
            .take()
            .unwrap_or_else(|| self.runway_frame.runway.clone());
        SpatialSnapshot {
            runway,
            field_elevation_ft: self.config.airport.field_elevation_ft,
            roundout_height_ft: self.config.flare.roundout_height_ft,
            flare_start_ft: self.config.flare.flare_start_ft,
            runway_clear_margin_ft: self.config.post_landing.runway_clear_margin_ft,
            overlay,
        }
    }

    pub fn has_profile(&self, name: &str) -> bool {
        self.active_profiles.iter().any(|p| p.name() == name)
    }

    pub fn phase(&self) -> FlightPhase {
        self.current_phase()
            .or(self.last_known_phase)
            .unwrap_or(FlightPhase::Preflight)
    }

    fn current_phase(&self) -> Option<FlightPhase> {
        for p in &self.active_profiles {
            if let Some(meta) = p.pattern_metadata() {
                return Some(meta.phase);
            }
        }
        None
    }

    pub fn update(&mut self, raw_state: &DynamicsState, dt: f64) -> (AircraftState, ActuatorCommands) {
        self.measure_surface(raw_state);
        let state = estimate_aircraft_state(raw_state, &self.config, &self.runway_frame, dt);
        let (guidance, metadata) = self.compose_guidance(&state, dt);
        let commands = self.commands_from_guidance(&state, &guidance);
        // `metadata` is captured inside `compose_guidance` BEFORE any
        // hand-off displaces the reporting profile, so its `phase` stays
        // correct for this tick even when pattern_fly auto-released.
        let phase = metadata.as_ref().map(|m| m.phase);
        if let Some(p) = phase {
            self.last_known_phase = Some(p);
        }
        let debug_lines: Vec<String> = self
            .active_profiles
            .iter()
            .filter_map(|p| p.debug_line(&state))
            .collect();
        let completed_profiles: Vec<String> = self
            .active_profiles
            .iter()
            .filter(|p| p.is_complete())
            .map(|p| p.name().to_string())
            .collect();
        let profile_mode_line_suffixes: Vec<Option<String>> = self
            .active_profiles
            .iter()
            .map(|p| p.mode_line_suffix(&state))
            .collect();
        let transition_hint = self
            .active_profiles
            .iter()
            .find_map(|p| p.next_transition_hint(&state));
        let plan_status = self.active_profiles.iter().find_map(|p| p.status_json());
        let lateral_owner_idx = self
            .active_profiles
            .iter()
            .position(|p| p.owns().contains(Axis::Lateral));
        let vertical_owner_idx = self
            .active_profiles
            .iter()
            .position(|p| p.owns().contains(Axis::Vertical));
        let speed_owner_idx = self
            .active_profiles
            .iter()
            .position(|p| p.owns().contains(Axis::Speed));
        let snapshot = StatusSnapshot {
            t_sim: state.t_sim,
            active_profiles: self.list_profile_names(),
            phase,
            state: state.clone(),
            last_commands: commands,
            last_guidance: Some(guidance),
            go_around_reason: metadata.as_ref().and_then(|m| m.last_go_around_reason.clone()),
            airport_ident: metadata.as_ref().and_then(|m| m.airport_ident.clone()),
            runway_id: metadata.as_ref().and_then(|m| m.runway_id.clone()),
            field_elevation_ft: metadata.as_ref().and_then(|m| m.field_elevation_ft),
            debug_lines,
            completed_profiles,
            profile_mode_line_suffixes,
            mission_goal: self.mission_goal.clone(),
            active_clearance: self.active_clearance.clone(),
            transition_hint,
            lateral_owner_idx,
            vertical_owner_idx,
            speed_owner_idx,
            plan_status,
            events: self.event_log.iter().cloned().collect(),
            traffic: self.traffic_status(&state),
        };
        self.latest_snapshot = Some(snapshot);
        (state, commands)
    }

    fn compose_guidance(
        &mut self,
        state: &AircraftState,
        dt: f64,
    ) -> (GuidanceTargets, Option<PatternMetadata>) {
        // Tick every profile; collect contributions and hand-off requests.
        let mut contributions: Vec<(Vec<Axis>, ProfileContribution)> =
            Vec::with_capacity(self.active_profiles.len());
        let mut handoffs: Vec<Vec<Box<dyn GuidanceProfile>>> = Vec::new();
        let mut metadata: Option<PatternMetadata> = None;
        for profile in &mut self.active_profiles {
            let owns: Vec<Axis> = profile.owns().iter().collect();
            let ProfileTick { contribution, hand_off } = profile.contribute(state, dt);
            // Capture metadata AFTER contribute(): a profile that
            // transitions phase (e.g. Final -> GoAround) updates its
            // last_go_around_reason inside contribute(). Reading
            // pattern_metadata() before would snapshot the prior tick's
            // values, leaving the GoAround transition heartbeat with
            // reason=None ("unknown") even when a real cause was set.
            if metadata.is_none() {
                metadata = profile.pattern_metadata();
            }
            for text in profile.take_events() {
                self.event_seq += 1;
                self.event_log.push_back((self.event_seq, text));
                if self.event_log.len() > EVENT_LOG_LEN {
                    self.event_log.pop_front();
                }
            }
            contributions.push((owns, contribution));
            if let Some(ho) = hand_off {
                handoffs.push(ho);
            }
        }

        let mut targets = GuidanceTargets {
            lateral_mode: LateralMode::BankHold,
            vertical_mode: VerticalMode::PitchHold,
            target_bank_deg: Some(0.0),
            target_pitch_deg: Some(0.0),
            throttle_limit: Some((0.0, 0.0)),
            ..Default::default()
        };
        for (owns, c) in contributions {
            if owns.contains(&Axis::Lateral) {
                if let Some(m) = c.lateral_mode { targets.lateral_mode = m; }
                if let Some(b) = c.target_bank_deg { targets.target_bank_deg = Some(b); }
                targets.target_heading_deg = c.target_heading_deg;
                targets.target_track_deg = c.target_track_deg;
                targets.target_path = c.target_path;
                targets.target_waypoint = c.target_waypoint.clone();
            }
            if owns.contains(&Axis::Vertical) {
                if let Some(m) = c.vertical_mode { targets.vertical_mode = m; }
                targets.target_altitude_ft = c.target_altitude_ft;
                if let Some(p) = c.target_pitch_deg { targets.target_pitch_deg = Some(p); }
                targets.glidepath = c.glidepath;
                targets.slip = c.slip;
                if let Some(t) = c.throttle_limit { targets.throttle_limit = Some(t); }
                targets.flaps_cmd = c.flaps_cmd;
                targets.gear_down = c.gear_down;
                if let Some(b) = c.brakes { targets.brakes = b; }
                if let Some(o) = c.tecs_phase_override { targets.tecs_phase_override = Some(o); }
                if let Some(p) = c.parking_brake { targets.parking_brake = Some(p); }
            }
            if owns.contains(&Axis::Speed) {
                if let Some(s) = c.target_speed_kt { targets.target_speed_kt = Some(s); }
            }
        }

        // Apply any hand-offs after all profiles have ticked.
        for batch in handoffs {
            self.install_profiles(batch);
        }
        (targets, metadata)
    }

    fn commands_from_guidance(&mut self, state: &AircraftState, guidance: &GuidanceTargets) -> ActuatorCommands {
        if self.last_lateral_mode != Some(guidance.lateral_mode) {
            self.bank_controller.reset();
            if guidance.lateral_mode == LateralMode::RolloutCenterline {
                self.rollout_controller.reset();
            }
            self.last_lateral_mode = Some(guidance.lateral_mode);
        }
        if self.last_vertical_mode != Some(guidance.vertical_mode) {
            self.pitch_controller.reset();
            if guidance.vertical_mode == VerticalMode::PitchForAirspeed {
                self.pitch_for_airspeed.reset();
            }
            if !matches!(guidance.vertical_mode, VerticalMode::Tecs | VerticalMode::GlidepathTrack)
            {
                self.last_tecs_phase = None;
            }
            self.last_vertical_mode = Some(guidance.vertical_mode);
        }

        // `taxi_pivot_brake` is produced by the nose-wheel controller when
        // the rudder is saturated at low ground speed — differential
        // braking layered on top of the symmetric `brakes` field.
        let mut taxi_pivot_brake: f64 = 0.0;
        let (aileron, rudder) = if guidance.lateral_mode == LateralMode::TaxiFollow {
            // Ground taxi: follow target_path on the nose wheel. Aileron is
            // irrelevant on the ground, so lock it to zero and reset the
            // bank controller integrator so it doesn't wind up.
            let (crosstrack_ft, heading_err_deg) =
                taxi_leg_errors(guidance.target_path, state);
            let nw = self.nose_wheel_controller.update(
                crosstrack_ft,
                heading_err_deg,
                state.r_rad_s.to_degrees(),
                state.gs_kt,
            );
            taxi_pivot_brake = nw.pivot_brake;
            self.bank_controller.reset();
            (0.0, nw.rudder)
        } else if guidance.lateral_mode == LateralMode::TaxiPose {
            // Pose-tracking terminal phase. Two-phase heading feedback:
            // when we're far from the target position, steer toward it
            // (bearing_to_target); when we're close, pivot to match the
            // target heading. No crosstrack — there's no line here.
            let (heading_err_deg, taxi_pivot) = taxi_pose_errors(guidance, state);
            let nw = self.nose_wheel_controller.update(
                0.0,
                heading_err_deg,
                state.r_rad_s.to_degrees(),
                state.gs_kt,
            );
            taxi_pivot_brake = nw.pivot_brake;
            self.bank_controller.reset();
            // If the profile reported it's done aligning, actively hold
            // position via the ground-speed controller's `target = 0`
            // hold-brake. `_taxi_pivot` is `true` while we're still
            // pivoting — kept for logging parity in the future.
            let _ = taxi_pivot;
            (0.0, nw.rudder)
        } else if guidance.lateral_mode == LateralMode::RolloutCenterline {
            let track_ref = if state.on_ground { state.heading_deg } else { state.track_deg };
            let track_error = wrap_degrees_180(self.runway_frame.runway.course_deg - track_ref);
            let rudder = self.rollout_controller.update(
                state.centerline_error_ft.unwrap_or(0.0),
                track_error,
                state.r_rad_s.to_degrees(),
                state.gs_kt,
                state.dt,
            );
            let aileron = if state.on_ground {
                self.bank_controller.reset();
                0.0
            } else {
                self.bank_controller.update(0.0, state.roll_deg, state.p_rad_s, state.dt)
            };
            (aileron, rudder)
        } else {
            let target_bank = guidance.target_bank_deg.unwrap_or(0.0);
            let aileron = self.bank_controller.update(target_bank, state.roll_deg, state.p_rad_s, state.dt);
            let rudder = self.coordination.update(target_bank, state.roll_deg, state.r_rad_s, None, state.dt);
            (aileron, rudder)
        };

        let throttle_limit = guidance.throttle_limit.unwrap_or((0.0, 1.0));
        // `taxi_brake_override` is set when we're in taxi mode so the
        // final ActuatorCommands picks up the ground-speed controller's
        // brake instead of the profile-provided `guidance.brakes`.
        let mut taxi_brake_override: Option<f64> = None;
        let slip = self.slip_command(state, guidance);
        let (pitch_cmd_deg, throttle_cmd) = if guidance.vertical_mode == VerticalMode::Taxi {
            let cmd = self.ground_speed_controller.update(
                guidance.target_speed_kt.unwrap_or(0.0),
                state.gs_kt,
            );
            taxi_brake_override = Some(cmd.brake);
            (0.0, cmd.throttle)
        } else if matches!(
            guidance.vertical_mode,
            VerticalMode::Tecs | VerticalMode::GlidepathTrack
        ) {
            let tecs_phase = guidance
                .tecs_phase_override
                .or_else(|| self.current_phase())
                .unwrap_or(FlightPhase::Cruise);
            if self.last_tecs_phase != Some(tecs_phase) {
                self.tecs.reset();
                self.last_tecs_phase = Some(tecs_phase);
            }
            let (target_altitude_ft, throttle_limit) = match guidance.glidepath {
                Some(gp) if guidance.vertical_mode == VerticalMode::GlidepathTrack => {
                    // On the path (its target not clamped for a capture)
                    // and not slipping down to it: trim out any standing
                    // offset.
                    self.tecs.tracking_path = slip.abs() <= 0.1
                        && (state.alt_msl_ft - self.glidepath_alt_ft(state, gp)).abs() < MAX_GLIDEPATH_CAPTURE_ERROR_FT;
                    self.glidepath_targets_slipping(state, gp, throttle_limit, slip.abs() > 0.1)
                }
                _ => {
                    self.tecs.tracking_path = false;
                    (guidance.target_altitude_ft.unwrap_or(state.alt_msl_ft), throttle_limit)
                }
            };
            self.tecs.update(
                tecs_phase,
                target_altitude_ft,
                guidance.target_speed_kt.unwrap_or(state.ias_kt),
                state.alt_msl_ft,
                state.vs_fpm,
                state.ias_kt,
                state.dt,
                throttle_limit,
            )
        } else if guidance.vertical_mode == VerticalMode::FlareTrack {
            let pitch_cmd = self.flare_controller.target_pitch_deg(
                state.landing_agl_ft(),
                state.vs_fpm,
                guidance.target_speed_kt.unwrap_or(state.ias_kt) - state.ias_kt,
            );
            (pitch_cmd, 0.0)
        } else if guidance.vertical_mode == VerticalMode::PitchForAirspeed {
            let target = guidance.target_speed_kt.unwrap_or(state.ias_kt);
            let pitch_cmd = self
                .pitch_for_airspeed
                .update(target, state.ias_kt, state.dt);
            (pitch_cmd, throttle_limit.1)
        } else {
            (guidance.target_pitch_deg.unwrap_or(0.0), throttle_limit.1)
        };

        let elevator = self.pitch_controller.update(pitch_cmd_deg, state.pitch_deg, state.q_rad_s, state.dt);
        let rudder = rudder + slip;

        self.parking_brake.profile_request(guidance.parking_brake);
        self.flaps.profile_request(guidance.flaps_cmd);
        // X-Plane releases the parking brake while the toe brakes are
        // pressed, so with it set leave them alone.
        let parking_brake_set = self.parking_brake.value().is_some_and(|r| r >= 0.5);
        let (brakes, pivot_brake) = if parking_brake_set {
            (0.0, 0.0)
        } else {
            (taxi_brake_override.unwrap_or(guidance.brakes), taxi_pivot_brake)
        };

        ActuatorCommands {
            aileron: clamp(aileron, -1.0, 1.0),
            elevator: clamp(elevator, -1.0, 1.0),
            rudder: clamp(rudder, -1.0, 1.0),
            slip,
            throttle: clamp(throttle_cmd, 0.0, 1.0),
            flaps: self.flaps.take_unsent(),
            gear_down: Some(true),
            brakes,
            pivot_brake: clamp(pivot_brake, -1.0, 1.0),
            parking_brake: self.parking_brake.take_unsent(),
        }
    }
}

/// How far below (or above) the aircraft a glidepath capture may
/// command: a controlled intercept instead of a dive when joining above
/// the path, or a zoom when below it.
const MAX_GLIDEPATH_CAPTURE_ERROR_FT: f64 = 25.0;
/// The same while slipping.
const SLIP_CAPTURE_ERROR_FT: f64 = 60.0;

/// Forward slip on a high approach: none within `SLIP_START_FT` of the
/// glidepath, full (`SLIP_MAX`) by `SLIP_FULL_FT` above it, and taken out
/// between `SLIP_OFF_AGL_FT` and `SLIP_GONE_AGL_FT` so it's gone before
/// the roundout.
const SLIP_START_FT: f64 = 80.0;
const SLIP_FULL_FT: f64 = 250.0;
const SLIP_MAX: f64 = 0.8;
const SLIP_OFF_AGL_FT: f64 = 250.0;
const SLIP_GONE_AGL_FT: f64 = 120.0;

impl PilotCore {
    /// The slip to hold this tick: the profile's demand, else an automatic
    /// forward slip when well above the glidepath on final. Left wing low,
    /// right rudder; the track loop banks against the rudder to hold the
    /// centerline.
    fn slip_command(&self, state: &AircraftState, guidance: &GuidanceTargets) -> f64 {
        if state.on_ground {
            return 0.0;
        }
        let fade = ((state.alt_agl_ft - SLIP_GONE_AGL_FT) / (SLIP_OFF_AGL_FT - SLIP_GONE_AGL_FT)).clamp(0.0, 1.0);
        if let Some(s) = guidance.slip {
            return s.clamp(-1.0, 1.0) * fade;
        }
        let Some(gp) = guidance.glidepath.filter(|_| guidance.vertical_mode == VerticalMode::GlidepathTrack) else {
            return 0.0;
        };
        let path_alt_ft = self.glidepath_alt_ft(state, gp);
        // Only once established: lined up and on the centerline. A slip
        // while still turning final fights the turn.
        let course = self.runway_frame.runway.course_deg;
        let lined_up = wrap_degrees_180(state.track_deg - course).abs() < 10.0
            && state.runway_y_ft.is_some_and(|y| y.abs() < 300.0);
        if !lined_up {
            return 0.0;
        }
        let above = state.alt_msl_ft - path_alt_ft;
        let amount = ((above - SLIP_START_FT) / (SLIP_FULL_FT - SLIP_START_FT)).clamp(0.0, 1.0);
        // Never slip slow: it's a way down, not a way to lose airspeed.
        // The gate sits below the power floor a slip keeps (Vref - 5), so
        // the slip doesn't switch off and on at the floor.
        let speed_ok = state.ias_kt >= self.config.performance.vref_kt - 8.0;
        if speed_ok { SLIP_MAX * amount * fade } else { 0.0 }
    }

    /// The altitude of glidepath `gp` abeam the aircraft.
    fn glidepath_alt_ft(&self, state: &AircraftState, gp: crate::types::Glidepath) -> f64 {
        let distance_to_aim_ft = gp.aimpoint_ft_from_threshold - state.runway_x_ft.unwrap_or(-3000.0);
        self.config.airport.field_elevation_ft
            + (gp.threshold_crossing_height_ft + distance_to_aim_ft * gp.slope_deg.to_radians().tan()).max(0.0)
    }

    /// TECS altitude target and throttle limits for tracking `gp` on the
    /// reference runway. Joining from above, the target sits at most
    /// `MAX_GLIDEPATH_CAPTURE_ERROR_FT` below the aircraft; that alone
    /// only steepens the descent slightly, and with power holding speed
    /// the aircraft would fly parallel to the path all the way down the
    /// runway. So power comes off as well (full authority up to 30 ft
    /// above the path, idle by 70 ft) and TECS has to pitch down to hold
    /// speed. Below Vref power comes back regardless: never trade
    /// airspeed for the glidepath.
    pub fn glidepath_targets(
        &self,
        state: &AircraftState,
        gp: crate::types::Glidepath,
        throttle_limit: (f64, f64),
    ) -> (f64, (f64, f64)) {
        self.glidepath_targets_slipping(state, gp, throttle_limit, false)
    }

    /// `glidepath_targets`; while slipping, power stays off down to 5 kt
    /// below Vref (the slip's own floor) so the nose comes down to hold
    /// speed instead of power cancelling the slip's drag.
    fn glidepath_targets_slipping(
        &self,
        state: &AircraftState,
        gp: crate::types::Glidepath,
        throttle_limit: (f64, f64),
        slipping: bool,
    ) -> (f64, (f64, f64)) {
        let path_alt_ft = self.glidepath_alt_ft(state, gp);
        // Below the path (a long final behind an extended downwind) the
        // target sits at most the same margin above the aircraft: hold
        // roughly level and let the path come down to meet it, as a pilot
        // intercepts a glideslope from below. Chasing it up with flaps
        // out trades all the airspeed for a few feet.
        // A slip's drag lets the nose come further down without speeding
        // up, so it may aim lower.
        let below = if slipping { SLIP_CAPTURE_ERROR_FT } else { MAX_GLIDEPATH_CAPTURE_ERROR_FT };
        let target_alt_ft = path_alt_ft.clamp(state.alt_msl_ft - below, state.alt_msl_ft + MAX_GLIDEPATH_CAPTURE_ERROR_FT);
        let above_path_ft = state.alt_msl_ft - path_alt_ft;
        let high_ceiling = (1.0 - (above_path_ft - 30.0) / 40.0).clamp(0.0, 1.0);
        let floor_kt = self.config.performance.vref_kt - if slipping { 5.0 } else { 0.0 };
        let slow_floor = ((floor_kt - state.ias_kt) / 5.0).clamp(0.0, 1.0);
        let max_throttle = throttle_limit.1.min(high_ceiling.max(slow_floor));
        (target_alt_ft, (throttle_limit.0.min(max_throttle), max_throttle))
    }
}

/// Compute the crosstrack and heading errors needed by
/// `NoseWheelController` given the current taxi leg and aircraft state.
/// Falls back to zeros when no leg is supplied (keeps the aircraft in
/// place instead of steering wildly).
fn taxi_leg_errors(
    target_path: Option<crate::types::StraightLeg>,
    state: &AircraftState,
) -> (f64, f64) {
    let Some(leg) = target_path else {
        return (0.0, 0.0);
    };
    let Some(crosstrack_ft) = leg.crosstrack_ft(state.position_ft) else {
        return (0.0, 0.0);
    };
    let leg_bearing = crate::types::vector_to_heading(leg.end_ft - leg.start_ft);
    let heading_err = wrap_degrees_180(leg_bearing - state.heading_deg);
    (crosstrack_ft, heading_err)
}

/// Pose-target heading feedback for `LateralMode::TaxiPose`. Returns
/// `(heading_err_deg, pivoting)`. When the aircraft is outside
/// `POSE_PIVOT_RADIUS_FT` of the target position, we feed the
/// aircraft-to-target bearing as the desired heading (steer toward the
/// point). Inside that radius we switch to the target heading itself
/// (pivot in place to face the commanded direction); `pivoting` flips
/// true so the caller can expose it in debug if desired.
fn taxi_pose_errors(guidance: &GuidanceTargets, state: &AircraftState) -> (f64, bool) {
    let target_pos = guidance
        .target_waypoint
        .as_ref()
        .map(|w| w.position_ft)
        .unwrap_or(state.position_ft);
    let target_heading = guidance.target_heading_deg.unwrap_or(state.heading_deg);
    let to_target = target_pos - state.position_ft;
    let d = to_target.length();
    if d > crate::core::profiles::POSE_PIVOT_RADIUS_FT {
        let bearing_to_target = crate::types::vector_to_heading(to_target);
        let err = wrap_degrees_180(bearing_to_target - state.heading_deg);
        (err, false)
    } else {
        let err = wrap_degrees_180(target_heading - state.heading_deg);
        (err, true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::load_default_config_bundle;

    fn raw_at(pilot: &PilotCore, x: f64, y: f64, alt_msl_ft: f64, radio_ft: Option<f64>, t: f64) -> DynamicsState {
        DynamicsState {
            position_ft: pilot.runway_frame.to_world_frame(Vec2::new(x, y)),
            altitude_ft: alt_msl_ft,
            agl_ft: radio_ft,
            heading_deg: pilot.runway_frame.runway.course_deg,
            roll_deg: 0.0,
            pitch_deg: 0.0,
            ias_kt: 62.0,
            throttle_pos: 0.2,
            on_ground: false,
            time_s: t,
            p_rad_s: 0.0,
            q_rad_s: 0.0,
            r_rad_s: 0.0,
            ground_velocity_ft_s: Vec2::ZERO,
            vertical_speed_ft_s: -5.0,
            flap_index: 30,
            gear_down: true,
            sideslip_deg: 0.0,
            parking_brake: 0.0,
        }
    }

    #[test]
    fn the_radio_altimeter_over_the_runway_sets_the_surface_elevation() {
        let mut cfg = load_default_config_bundle();
        cfg.airport.field_elevation_ft = 800.0;
        let mut pilot = PilotCore::new(cfg.clone());
        // Downwind, over terrain: not the runway.
        pilot.update(&raw_at(&pilot, 3000.0, 5500.0, 1800.0, Some(1023.0), 0.0), 0.2);
        assert_eq!(pilot.config.airport.field_elevation_ft, 800.0);
        // Short final over higher ground short of the runway (ignored),
        // then over pavement that slopes down 0.5% from 795 ft.
        let surface = |x: f64| 795.0 - 0.005 * x;
        let mut t = 1.0;
        for x in [-800.0, -400.0] {
            pilot.update(&raw_at(&pilot, x, 10.0, 807.0 + 150.0, Some(150.0), t), 0.2);
            t += 0.2;
        }
        assert_eq!(pilot.config.airport.field_elevation_ft, 800.0, "terrain short of the runway");
        let mut x = 0.0;
        while x <= 1500.0 {
            pilot.update(&raw_at(&pilot, x, 10.0, surface(x) + 60.0, Some(60.0), t), 0.2);
            t += 0.2;
            x += 50.0;
        }
        let aim = pilot.runway_frame.touchdown_runway_x_ft();
        let elev = pilot.config.airport.field_elevation_ft;
        assert!((elev - surface(aim)).abs() < 0.5, "{elev} vs {} at the aim point", surface(aim));
        let (state, _) = pilot.update(&raw_at(&pilot, 1550.0, 0.0, elev + 20.0, Some(20.0), t), 0.2);
        assert!((state.alt_agl_ft - 20.0).abs() < 0.6, "{}", state.alt_agl_ft);
        let reported = format!("measured at {:.0}", surface(aim));
        assert!(pilot.event_log.iter().any(|(_, e)| e.contains(&reported)), "{:?}", pilot.event_log);
        // The same runway re-engaged (a new plan) keeps the measurement; a
        // different runway drops it.
        pilot.set_reference_config(cfg.clone());
        assert!((pilot.config.airport.field_elevation_ft - surface(aim)).abs() < 0.5);
        let mut other = cfg;
        other.airport.runway.threshold_ft = Vec2::new(5000.0, 5000.0);
        pilot.set_reference_config(other);
        assert_eq!(pilot.config.airport.field_elevation_ft, 800.0);
    }

    #[test]
    fn spatial_snapshot_reflects_active_profile() {
        use crate::core::profiles::TaxiProfile;
        use crate::types::{StraightLeg, TaxiPose};
        let mut pilot = PilotCore::new(load_default_config_bundle());
        let idle = pilot.spatial_snapshot();
        assert_eq!(idle.runway, pilot.runway_frame.runway);
        assert!(idle.overlay.route.is_empty());
        let leg = StraightLeg { start_ft: Vec2::ZERO, end_ft: Vec2::new(0.0, 400.0) };
        let taxi = TaxiProfile::new(vec![leg], vec!["A".to_string()])
            .with_final_pose(TaxiPose { position_ft: Vec2::new(0.0, 400.0), heading_deg: 0.0 });
        pilot.engage_profile(Box::new(taxi)).unwrap();
        let sp = pilot.spatial_snapshot();
        assert_eq!(sp.overlay.route.len(), 1);
        assert_eq!(sp.overlay.route[0].label, "A");
        assert_eq!(sp.overlay.route_active_idx, Some(0));
        assert!(sp.overlay.target_pose.is_some());
        assert_eq!(sp.overlay.target_label.as_deref(), Some("HOLD SHORT"));
        assert_eq!(sp.roundout_height_ft, pilot.config.flare.roundout_height_ft);
    }

    #[test]
    fn set_and_clear_mission_goal() {
        let mut pilot = PilotCore::new(load_default_config_bundle());
        assert!(pilot.mission_goal.is_none());
        pilot.set_mission_goal(MissionGoal {
            kind: "land".to_string(),
            target: Some("KEMT runway 19".to_string()),
            notes: None,
        });
        assert_eq!(
            pilot.mission_goal.as_ref().map(|g| g.kind.as_str()),
            Some("land"),
        );
        pilot.clear_mission_goal();
        assert!(pilot.mission_goal.is_none());
    }

    #[test]
    fn record_and_clear_clearance() {
        let mut pilot = PilotCore::new(load_default_config_bundle());
        assert!(pilot.active_clearance.is_none());
        pilot.record_clearance(ActiveClearance {
            text: "cleared touch-and-go runway 19".to_string(),
            source: ClearanceSource::Tower,
            freq_mhz: Some(124.30),
            issued_at: Utc::now(),
        });
        let stored = pilot.active_clearance.as_ref().expect("clearance set");
        assert_eq!(stored.source, ClearanceSource::Tower);
        assert_eq!(stored.freq_mhz, Some(124.30));
        pilot.clear_active_clearance();
        assert!(pilot.active_clearance.is_none());
    }

    #[test]
    fn clearance_source_from_str_round_trips() {
        assert_eq!(ClearanceSource::from_str("tower"), ClearanceSource::Tower);
        assert_eq!(ClearanceSource::from_str("TWR"), ClearanceSource::Tower);
        assert_eq!(ClearanceSource::from_str("ground"), ClearanceSource::Ground);
        assert_eq!(ClearanceSource::from_str("ctaf"), ClearanceSource::Unicom);
        assert_eq!(
            ClearanceSource::from_str("clearance"),
            ClearanceSource::Other("clearance".to_string()),
        );
    }

    #[test]
    fn controller_integrators_reset_on_mode_entry() {
        let mut pilot = PilotCore::new(load_default_config_bundle());
        let state = AircraftState::synthetic_default();

        for _ in 0..20 {
            pilot.rollout_controller.update(0.0, 10.0, 0.0, 30.0, 0.2);
        }
        assert!(pilot.rollout_controller.integrator().abs() > 1.0);
        pilot.last_lateral_mode = Some(LateralMode::PathFollow);
        let rollout = GuidanceTargets {
            lateral_mode: LateralMode::RolloutCenterline,
            vertical_mode: VerticalMode::PitchHold,
            target_pitch_deg: Some(0.0),
            throttle_limit: Some((0.0, 0.0)),
            ..Default::default()
        };
        pilot.commands_from_guidance(&state, &rollout);
        assert_eq!(pilot.rollout_controller.integrator(), 0.0);

        pilot.pitch_controller.pid.integrator = 10.0;
        pilot.last_vertical_mode = Some(VerticalMode::Tecs);
        let pitch_hold = GuidanceTargets {
            vertical_mode: VerticalMode::PitchHold,
            target_pitch_deg: Some(0.0),
            throttle_limit: Some((0.0, 0.0)),
            ..Default::default()
        };
        pilot.commands_from_guidance(&state, &pitch_hold);
        assert_eq!(pilot.pitch_controller.pid.integrator, 0.0);

        pilot.tecs.integrator = 80.0;
        pilot.last_vertical_mode = Some(VerticalMode::Tecs);
        pilot.last_tecs_phase = Some(FlightPhase::Base);
        let final_tecs = GuidanceTargets {
            vertical_mode: VerticalMode::Tecs,
            target_altitude_ft: Some(state.alt_msl_ft),
            target_speed_kt: Some(state.ias_kt),
            throttle_limit: Some((0.0, 1.0)),
            tecs_phase_override: Some(FlightPhase::Final),
            ..Default::default()
        };
        pilot.commands_from_guidance(&state, &final_tecs);
        assert_eq!(pilot.tecs.integrator, 0.0);
    }
}
