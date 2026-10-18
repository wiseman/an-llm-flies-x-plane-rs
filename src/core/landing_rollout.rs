//! Post-touchdown control: rollout, runway exit, and the stop clear of the
//! runway.
//!
//! `PatternFlyProfile` and `DeadStickLandingProfile` hand off to
//! `LandingRolloutProfile` on the tick the wheels touch down. From then on
//! this profile alone owns the ground phase machine
//! (`Rollout → RunwayExit → TaxiClear`), so exit sequencing has a single
//! source of truth instead of flags layered over the airborne mode manager.
//!
//! It refuses preemption while the aircraft is rolling or following an
//! exit connector, so a taxi/park profile can't cut across the runway
//! environment mid-rollout. Once stopped with nothing left to fly (no exit,
//! or a missed one) it keeps the brakes on but accepts a ground taxi
//! profile; at `TaxiClear` it releases to idle on its own.

use crate::config::ConfigBundle;
use crate::core::profiles::{
    exit_leg, exit_route_legs, guidance_to_contribution, idle_profile_trio, AxisSet, GuidanceProfile,
    PatternMetadata, ProfileTick, TaxiProfile, TransitionHint,
};
use crate::core::spatial::SpatialOverlay;
use crate::guidance::lateral::leg_projection;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::types::{KT_TO_FPS, 
    AircraftState, FlightPhase, GuidanceTargets, LateralMode, StraightLeg, Vec2, VerticalMode,
};

/// Wheels must stay on the ground this long before an airborne landing
/// profile hands over to `LandingRolloutProfile`. A bounce inside the
/// window goes back to the flare, so go-around and everything else stay
/// available to the airborne profile.
pub const TOUCHDOWN_SETTLE_S: f64 = 1.0;

/// Track continuous ground contact through a landing. Returns the phase to
/// fly this tick: `Rollout` once settled (time to hand over), `Flare` while
/// still settling or after a bounce.
pub fn touchdown_debounce(
    phase: FlightPhase,
    on_ground: bool,
    dt: f64,
    ground_contact_s: &mut f64,
) -> FlightPhase {
    if phase != FlightPhase::Rollout {
        *ground_contact_s = 0.0;
        return phase;
    }
    if !on_ground {
        *ground_contact_s = 0.0;
        return FlightPhase::Flare;
    }
    *ground_contact_s += dt;
    if *ground_contact_s >= TOUCHDOWN_SETTLE_S {
        FlightPhase::Rollout
    } else {
        FlightPhase::Flare
    }
}

/// Hand the ground roll to `rollout`, letting it fly this tick. If it's
/// already done on its first tick (e.g. stopped well off the runway), its
/// own hand-off goes straight through instead of being dropped.
pub fn touchdown_handoff(mut rollout: LandingRolloutProfile, state: &AircraftState, dt: f64) -> ProfileTick {
    let tick = rollout.contribute(state, dt);
    let next: Vec<Box<dyn GuidanceProfile>> = match tick.hand_off {
        Some(next) => next,
        None => vec![Box::new(rollout)],
    };
    ProfileTick { contribution: tick.contribution, hand_off: Some(next) }
}

/// Resolved geometry for an autonomous runway exit.
#[derive(Debug, Clone, PartialEq)]
pub struct RunwayExitPlan {
    pub taxiway_name: String,
    /// Along-runway station of the graph node on the runway centerline.
    pub stationing_ft: f64,
    /// Ordered runway-frame taxi-network points, beginning on the runway
    /// centerline and ending at the first node outside the active zone.
    pub path_runway_ft: Vec<Vec2>,
}

/// A pre-planned arrival: once the selected exit is flown to completion,
/// the rollout hands straight off to `taxi` instead of releasing to idle.
/// Built by `plan_arrival` before touchdown, while the LLM has time.
pub struct ArrivalPlan {
    pub parking_name: String,
    pub via_taxiways: Vec<String>,
    /// Taxi route from the exit's last node to the parking spot.
    pub taxi: Box<TaxiProfile>,
    /// One-line description for status and transition hints.
    pub summary: String,
}

/// A backup exit beyond the chosen one, with its own taxi to parking,
/// taken if the chosen exit is missed.
/// Within this of an exit leg's course, the aircraft is lined up on it.
const EXIT_ALIGNED_DEG: f64 = 15.0;

pub struct ExitOption {
    pub exit: RunwayExitPlan,
    pub arrival: Option<ArrivalPlan>,
}

/// Progress along the selected exit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ExitProgress {
    /// Rolling toward the exit's intersection.
    Approaching,
    /// Turned off onto the exit connector; following leg `leg_idx`.
    Committed { leg_idx: usize },
    /// Reached the last node of the exit connector.
    Complete,
    /// Passed the intersection too fast; stay on centerline and stop.
    Missed,
}

/// The selected exit and how far along it the aircraft is. No exit means
/// brake to a stop on the centerline.
struct SelectedExit {
    plan: RunwayExitPlan,
    progress: ExitProgress,
}

impl SelectedExit {
    fn new(plan: RunwayExitPlan) -> Self {
        Self { plan, progress: ExitProgress::Approaching }
    }

    fn committed_or_complete(&self) -> bool {
        matches!(self.progress, ExitProgress::Committed { .. } | ExitProgress::Complete)
    }
}

/// A profile carrying the landing's runway-exit and arrival plan:
/// pattern_fly before touchdown (which hands it to the rollout), the
/// rollout after. Tools reach whichever is active through
/// `PilotCore::landing_plan_mut`.
pub trait LandingPlanHolder {
    fn runway_frame(&self) -> &RunwayFrame;
    fn preferred_exit(&self) -> Option<&RunwayExitPlan>;
    fn arrival(&self) -> Option<&ArrivalPlan>;
    /// Replace the exit and the arrival together: the arrival taxi starts
    /// where the exit ends. `Err` once the plan can no longer change.
    fn set_landing_plan(
        &mut self,
        exit: Option<RunwayExitPlan>,
        arrival: Option<ArrivalPlan>,
    ) -> Result<(), String>;
    /// Exits to fall back on, in runway order, if the chosen one is
    /// missed. Clearing the exit clears these too.
    fn set_fallback_exits(&mut self, fallbacks: Vec<ExitOption>);
}

pub struct LandingRolloutProfile {
    pub config: ConfigBundle,
    pub runway_frame: RunwayFrame,
    pub phase: FlightPhase,
    exit: Option<SelectedExit>,
    /// False after a dead-stick landing: no throttle, so no exit taxiing.
    power_available: bool,
    last_go_around_reason: Option<String>,
    last_gs_kt: f64,
    last_on_ground: bool,
    arrival: Option<ArrivalPlan>,
    fallbacks: std::collections::VecDeque<ExitOption>,
    /// A missed exit with no fallback left has been reported.
    miss_reported: bool,
    events: Vec<String>,
}

impl LandingRolloutProfile {
    pub fn new(config: ConfigBundle, runway_frame: RunwayFrame, exit: Option<RunwayExitPlan>) -> Self {
        Self {
            config,
            runway_frame,
            phase: FlightPhase::Rollout,
            exit: exit.map(SelectedExit::new),
            power_available: true,
            last_go_around_reason: None,
            last_gs_kt: f64::INFINITY,
            last_on_ground: true,
            arrival: None,
            fallbacks: Default::default(),
            miss_reported: false,
            events: Vec::new(),
        }
    }

    /// Exits to take, in order, if the selected one is missed.
    pub fn with_fallback_exits(mut self, fallbacks: Vec<ExitOption>) -> Self {
        self.fallbacks = fallbacks.into();
        self
    }

    /// The selected exit was missed: take the first fallback still far
    /// enough ahead to slow for, and tell the pilot either way.
    fn on_missed_exit(&mut self, state: &AircraftState) {
        let Some(missed) = self.exit.as_ref().map(|e| e.plan.taxiway_name.clone()) else { return };
        let x = state.runway_x_ft.unwrap_or(f64::INFINITY);
        let lead = self.config.post_landing.slowdown_lead_ft;
        while let Some(next) = self.fallbacks.pop_front() {
            if next.exit.stationing_ft < x + lead {
                continue;
            }
            self.events.push(format!(
                "missed exit {missed}; taking {} at {:.0} ft instead{}",
                next.exit.taxiway_name,
                next.exit.stationing_ft,
                if next.arrival.is_some() { ", taxi re-planned from there" } else { "" }
            ));
            self.exit = Some(SelectedExit::new(next.exit));
            self.arrival = next.arrival;
            return;
        }
        self.miss_reported = true;
        self.events.push(format!(
            "missed exit {missed} and no planned exit ahead; slowing on the centerline (choose_runway_exit for one ahead)"
        ));
    }

    /// Continue into a pre-planned taxi to parking after the exit.
    pub fn with_arrival(mut self, arrival: Option<ArrivalPlan>) -> Self {
        self.arrival = arrival;
        self
    }

    /// Engine-out landing: throttle stays closed and exits can't be flown.
    pub fn without_power(mut self) -> Self {
        self.power_available = false;
        self.exit = None;
        self
    }

    /// Carry the landing profile's last go-around reason into telemetry.
    pub fn with_go_around_reason(mut self, reason: Option<String>) -> Self {
        self.last_go_around_reason = reason;
        self
    }

    /// Replace (or clear) the selected exit. Refused once the aircraft has
    /// turned onto the current exit — the replacement would steer back
    /// toward runway heading from a position already on the connector —
    /// and after an engine-out landing.
    pub fn set_preferred_exit(&mut self, exit: Option<RunwayExitPlan>) -> Result<(), String> {
        if !self.power_available {
            return Err("no engine power after a dead-stick landing; the aircraft can't taxi off on its own".to_string());
        }
        if let Some(current) = self.exit.as_ref().filter(|e| e.committed_or_complete()) {
            return Err(format!(
                "already turning off the runway onto {}; the exit can't be changed until it completes",
                current.plan.taxiway_name
            ));
        }
        if exit.is_none() {
            self.fallbacks.clear();
        }
        self.exit = exit.map(SelectedExit::new);
        self.miss_reported = false;
        Ok(())
    }

    /// Stopped on the runway with no exit left to fly. The profile keeps
    /// the brakes on here but lets a ground taxi profile take over.
    pub fn holding_on_runway(&self) -> bool {
        self.phase == FlightPhase::Rollout
            && self.last_gs_kt <= self.config.post_landing.stopped_gs_kt
            && self.exit.as_ref().is_none_or(|e| e.progress == ExitProgress::Missed)
    }

    fn exit_leg(&self, plan: &RunwayExitPlan, leg_idx: usize) -> Option<StraightLeg> {
        exit_leg(&self.runway_frame, plan, leg_idx)
    }

    fn update_exit_progress(&mut self, state: &AircraftState) {
        let pl = &self.config.post_landing;
        let Some(runway_x) = state.runway_x_ft else { return };
        let Some(exit) = self.exit.as_mut() else { return };
        let plan = &exit.plan;
        if plan.path_runway_ft.len() < 2 {
            exit.progress = ExitProgress::Missed;
            return;
        }
        if exit.progress == ExitProgress::Approaching {
            if runway_x > plan.stationing_ft + pl.missed_exit_ft {
                exit.progress = ExitProgress::Missed;
                return;
            }
            if runway_x >= plan.stationing_ft - pl.intersection_capture_ft
                && state.gs_kt <= pl.turnoff_speed_kt + 5.0
            {
                exit.progress = ExitProgress::Committed { leg_idx: 0 };
            }
        }
        let leg_count = plan.path_runway_ft.len() - 1;
        while let ExitProgress::Committed { leg_idx } = exit.progress {
            let Some(leg) = exit_leg(&self.runway_frame, plan, leg_idx) else { break };
            let (along, _, leg_len) = leg_projection(state, leg);
            let reached = state.position_ft.distance_to(leg.end_ft) <= pl.leg_capture_ft;
            if !reached && along <= leg_len {
                break;
            }
            exit.progress = if leg_idx + 1 >= leg_count {
                ExitProgress::Complete
            } else {
                ExitProgress::Committed { leg_idx: leg_idx + 1 }
            };
        }
    }

    fn next_phase(&self, state: &AircraftState) -> FlightPhase {
        let stopped = state.gs_kt <= self.config.post_landing.stopped_gs_kt;
        match self.exit.as_ref().map(|e| e.progress) {
            // A selected exit carries authoritative active-zone geometry:
            // graph progress, not a lateral offset, sequences the exit.
            Some(ExitProgress::Committed { .. }) => FlightPhase::RunwayExit,
            Some(ExitProgress::Complete) => {
                if stopped { FlightPhase::TaxiClear } else { FlightPhase::RunwayExit }
            }
            None | Some(ExitProgress::Approaching | ExitProgress::Missed) => {
                // Off the runway is a position, not a one-way latch: an
                // aircraft that touches down beside the pavement and rolls
                // back onto it is in Rollout again, and only counts as
                // clear once it's stopped off the pavement or past the end.
                let lateral_off = state.runway_y_ft.map(f64::abs).unwrap_or(0.0);
                let off_runway = lateral_off > self.runway_exit_offset_ft();
                let past_end = state
                    .runway_x_ft
                    .map(|x| x > self.runway_frame.runway.length_ft)
                    .unwrap_or(false);
                if lateral_off > self.runway_clear_offset_ft() || (stopped && (off_runway || past_end)) {
                    FlightPhase::TaxiClear
                } else if off_runway {
                    FlightPhase::RunwayExit
                } else {
                    FlightPhase::Rollout
                }
            }
        }
    }

    /// Lateral offset at which the aircraft has left the runway surface.
    fn runway_exit_offset_ft(&self) -> f64 {
        self.runway_frame.half_width_plus_ft(self.config.post_landing.runway_clear_margin_ft)
    }

    /// Lateral offset at which the aircraft is past the hold-short line.
    fn runway_clear_offset_ft(&self) -> f64 {
        self.config.post_landing.hold_line_offset_ft.max(self.runway_exit_offset_ft())
    }

    fn rollout_guidance(&self, state: &AircraftState) -> GuidanceTargets {
        let pl = &self.config.post_landing;
        let runway_x = state.runway_x_ft.unwrap_or(f64::NEG_INFINITY);
        let Some(SelectedExit { plan: exit, progress }) = self.exit.as_ref() else {
            return self.braking_on_centerline();
        };
        let last_idx = exit.path_runway_ft.len().saturating_sub(2);
        let following = match *progress {
            ExitProgress::Committed { leg_idx } => Some((leg_idx, false)),
            ExitProgress::Complete => Some((last_idx, true)),
            ExitProgress::Approaching | ExitProgress::Missed => None,
        };

        if let Some((leg_idx, done)) = following {
            // The turn off the runway at turn speed; lined up on the exit,
            // taxi.
            let aligned = self.exit_leg(exit, leg_idx).is_some_and(|leg| {
                crate::types::wrap_degrees_180(leg.course_deg() - state.heading_deg).abs() <= EXIT_ALIGNED_DEG
            });
            let target_speed = if done {
                0.0
            } else if leg_idx >= last_idx {
                // Ramp down toward the last node, but keep creeping until
                // it's captured: a purely proportional ramp drops below the
                // speed controller's deadband several feet out, and the
                // aircraft would stop outside the capture radius.
                let distance = exit
                    .path_runway_ft
                    .last()
                    .map(|p| state.position_ft.distance_to(self.runway_frame.to_world_frame(*p)))
                    .unwrap_or(0.0);
                let cruise = if aligned { pl.exit_taxi_speed_kt } else { pl.exit_speed_kt };
                (distance / pl.final_stop_ramp_ft * cruise).clamp(pl.exit_creep_speed_kt, cruise)
            } else if aligned {
                pl.exit_taxi_speed_kt
            } else {
                pl.exit_speed_kt
            };
            return GuidanceTargets {
                lateral_mode: LateralMode::TaxiFollow,
                vertical_mode: VerticalMode::Taxi,
                target_path: self.exit_leg(exit, leg_idx),
                target_speed_kt: Some(target_speed),
                target_pitch_deg: Some(0.0),
                throttle_limit: Some((0.0, 1.0)),
                ..Default::default()
            };
        }

        let target_speed = if *progress == ExitProgress::Missed {
            let distance_to_end = self.runway_frame.runway.length_ft - runway_x;
            if distance_to_end <= pl.slowdown_lead_ft { 0.0 } else { pl.turnoff_speed_kt }
        } else {
            // Brake to arrive at the turn at turn speed: v² = v_turn² + 2ad.
            let d = (exit.stationing_ft - runway_x).max(0.0);
            let v_turn = pl.exit_speed_kt * KT_TO_FPS;
            let v = (v_turn * v_turn + 2.0 * pl.exit_slowdown_fps2 * d).sqrt() / KT_TO_FPS;
            v.min(pl.turnoff_speed_kt)
        };
        GuidanceTargets {
            lateral_mode: LateralMode::RolloutCenterline,
            vertical_mode: VerticalMode::Taxi,
            target_bank_deg: Some(0.0),
            target_heading_deg: Some(self.runway_frame.runway.course_deg),
            target_pitch_deg: Some(0.0),
            target_speed_kt: Some(target_speed),
            throttle_limit: Some((0.0, 1.0)),
            ..Default::default()
        }
    }

    fn braking_on_centerline(&self) -> GuidanceTargets {
        GuidanceTargets {
            lateral_mode: LateralMode::RolloutCenterline,
            vertical_mode: VerticalMode::PitchHold,
            target_bank_deg: Some(0.0),
            target_heading_deg: Some(self.runway_frame.runway.course_deg),
            target_pitch_deg: Some(0.0),
            throttle_limit: Some((0.0, 0.0)),
            brakes: self.config.post_landing.rollout_brake,
            ..Default::default()
        }
    }
}

impl LandingPlanHolder for LandingRolloutProfile {
    fn runway_frame(&self) -> &RunwayFrame {
        &self.runway_frame
    }

    fn preferred_exit(&self) -> Option<&RunwayExitPlan> {
        self.exit.as_ref().map(|e| &e.plan)
    }

    fn arrival(&self) -> Option<&ArrivalPlan> {
        self.arrival.as_ref()
    }

    fn set_landing_plan(
        &mut self,
        exit: Option<RunwayExitPlan>,
        arrival: Option<ArrivalPlan>,
    ) -> Result<(), String> {
        self.set_preferred_exit(exit)?;
        self.arrival = arrival;
        Ok(())
    }

    fn set_fallback_exits(&mut self, fallbacks: Vec<ExitOption>) {
        self.fallbacks = fallbacks.into();
    }
}

impl GuidanceProfile for LandingRolloutProfile {
    fn name(&self) -> &'static str { "landing_rollout" }

    fn landing_plan_mut(&mut self) -> Option<&mut dyn LandingPlanHolder> {
        Some(self)
    }

    fn owns(&self) -> AxisSet { AxisSet::ALL }

    fn contribute(&mut self, state: &AircraftState, _dt: f64) -> ProfileTick {
        self.last_gs_kt = state.gs_kt;
        self.last_on_ground = state.on_ground;
        if matches!(self.phase, FlightPhase::Rollout | FlightPhase::RunwayExit) {
            self.update_exit_progress(state);
            if !self.miss_reported && self.exit.as_ref().is_some_and(|e| e.progress == ExitProgress::Missed) {
                self.on_missed_exit(state);
            }
        }
        let previous_phase = self.phase;
        self.phase = self.next_phase(state);

        let mut guidance = match self.phase {
            FlightPhase::TaxiClear => self.braking_on_centerline(),
            _ => self.rollout_guidance(state),
        };
        if !self.power_available {
            guidance.throttle_limit = Some((0.0, 0.0));
        }
        // Release on the first TaxiClear tick: to the planned arrival taxi
        // when the exit was flown, otherwise to idle.
        let mut hand_off = (self.phase == FlightPhase::TaxiClear
            && previous_phase != FlightPhase::TaxiClear)
            .then(|| idle_profile_trio(0.0));
        // Arrival plan: the exit ended at the node the taxi route starts
        // from, so go straight on to parking instead of idling.
        if hand_off.is_some()
            && self.exit.as_ref().is_some_and(|e| e.progress == ExitProgress::Complete)
        {
            if let Some(arrival) = self.arrival.take() {
                hand_off = Some(vec![arrival.taxi as Box<dyn GuidanceProfile>]);
            }
        }
        ProfileTick {
            contribution: guidance_to_contribution(guidance),
            hand_off,
        }
    }

    /// Stopped on the runway with nothing left to fly (no exit, or a
    /// missed one): the rollout is done and the LLM has to act, so this is
    /// reported as a completion, which wakes it with a heartbeat.
    fn is_complete(&self) -> bool {
        self.holding_on_runway()
    }

    fn take_events(&mut self) -> Vec<String> {
        std::mem::take(&mut self.events)
    }

    fn preemption(&self) -> Result<(), String> {
        // Airborne again (a bounce after the hand-off): this ground-only
        // profile must not stand in the way of a go-around or anything else.
        if !self.last_on_ground || self.phase == FlightPhase::TaxiClear || self.holding_on_runway() {
            return Ok(());
        }
        Err(match self.exit.as_ref().filter(|e| e.committed_or_complete()) {
            Some(exit) => format!(
                "following runway exit {} off the runway; wait until it stops at the end of the exit",
                exit.plan.taxiway_name
            ),
            None => format!(
                "still rolling out on the runway ({:.0} kt); wait until the aircraft stops",
                self.last_gs_kt
            ),
        })
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
        let (route, route_active_idx, target_label) = match &self.exit {
            Some(exit) => {
                let legs = exit_route_legs(&self.runway_frame, &exit.plan);
                let idx = match exit.progress {
                    ExitProgress::Committed { leg_idx } => Some(leg_idx),
                    ExitProgress::Complete => Some(legs.len()),
                    ExitProgress::Approaching | ExitProgress::Missed => None,
                };
                (legs, idx, Some(exit.plan.taxiway_name.clone()))
            }
            None => (Vec::new(), None, None),
        };
        Some(SpatialOverlay {
            runway: Some(self.runway_frame.runway.clone()),
            route,
            route_active_idx,
            target_label,
            ..Default::default()
        })
    }

    fn next_transition_hint(&self, _state: &AircraftState) -> Option<TransitionHint> {
        let arrival_note = || {
            self.arrival
                .as_ref()
                .map(|a| format!("; then taxi to {}", a.summary))
                .unwrap_or_default()
        };
        let exit = self.exit.as_ref();
        let progress = exit.map(|e| e.progress);
        let exit = exit.map(|e| &e.plan);
        let condition_text = match (self.phase, progress, exit) {
            (FlightPhase::Rollout, Some(ExitProgress::Missed), Some(exit)) if self.holding_on_runway() => {
                let plan = self
                    .arrival
                    .as_ref()
                    .map(|a| format!("arrival plan to {} on hold; ", a.parking_name))
                    .unwrap_or_default();
                format!(
                    "missed exit {}, stopped on runway holding brakes; {plan}choose an exit ahead or engage_park",
                    exit.taxiway_name
                )
            }
            (FlightPhase::Rollout, _, _) if self.holding_on_runway() => {
                "stopped on runway, holding brakes; choose an exit ahead or engage_park".to_string()
            }
            (FlightPhase::Rollout, Some(ExitProgress::Approaching), Some(exit)) => format!(
                "reach {} intersection at {:.0} ft{}",
                exit.taxiway_name,
                exit.stationing_ft,
                arrival_note()
            ),
            (FlightPhase::Rollout, _, _) => "off the runway laterally, or stopped past its end".to_string(),
            (FlightPhase::RunwayExit, _, Some(exit)) => format!(
                "follow {} to active-zone boundary and stop{}",
                exit.taxiway_name,
                arrival_note()
            ),
            (FlightPhase::RunwayExit, _, None) => "stopped, off runway".to_string(),
            _ => return None,
        };
        let next_phase = match self.phase {
            FlightPhase::Rollout => FlightPhase::RunwayExit,
            _ => FlightPhase::TaxiClear,
        };
        Some(TransitionHint { next_phase, condition_text, eta_s: None })
    }
}
