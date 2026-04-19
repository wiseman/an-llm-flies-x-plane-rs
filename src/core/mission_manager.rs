//! `PilotCore` — the per-tick composer that reads dynamics, composes
//! guidance from active profiles, and emits actuator commands. Mirrors
//! sim_pilot/core/mission_manager.py.
//!
//! Thread model: callers lock `PilotCore` through a single `Mutex` held by
//! the control loop on every `update()`. Tool handlers (LLM worker, TUI)
//! briefly acquire the same lock to mutate the profile stack.

use std::collections::BTreeSet;

use crate::config::ConfigBundle;
use crate::control::bank_hold::{BankController, CoordinationController};
use crate::control::centerline_rollout::CenterlineRolloutController;
use crate::control::ground_speed::GroundSpeedController;
use crate::control::nose_wheel::NoseWheelController;
use crate::control::pitch_hold::PitchController;
use crate::control::tecs_lite::TECSLite;
use crate::core::profiles::{
    Axis, GuidanceProfile, IdleLateralProfile, IdleSpeedProfile, IdleVerticalProfile,
    PatternMetadata, ProfileContribution, ProfileTick,
};
use crate::core::state_estimator::estimate_aircraft_state;
use crate::guidance::flare_profile::FlareController;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::sim::simple_dynamics::DynamicsState;
use crate::types::{
    clamp, wrap_degrees_180, ActuatorCommands, AircraftState, FlightPhase, GuidanceTargets,
    LateralMode, VerticalMode,
};

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
}

pub struct PilotCore {
    pub config: ConfigBundle,
    pub runway_frame: RunwayFrame,
    pub bank_controller: BankController,
    pub coordination: CoordinationController,
    pub pitch_controller: PitchController,
    pub rollout_controller: CenterlineRolloutController,
    pub tecs: TECSLite,
    pub flare_controller: FlareController,
    pub nose_wheel_controller: NoseWheelController,
    pub ground_speed_controller: GroundSpeedController,
    pub active_profiles: Vec<Box<dyn GuidanceProfile>>,
    pub latest_snapshot: Option<StatusSnapshot>,
}

impl PilotCore {
    pub fn new(config: ConfigBundle) -> Self {
        let runway_frame = RunwayFrame::new(config.airport.runway.clone());
        let bank_controller = BankController::new(config.controllers.bank);
        let pitch_controller = PitchController::new(config.controllers.pitch);
        let tecs = TECSLite::new(config.controllers.tecs);
        let flare = FlareController::new(config.flare.clone());
        let idle: Vec<Box<dyn GuidanceProfile>> = vec![
            Box::new(IdleLateralProfile),
            Box::new(IdleVerticalProfile),
            Box::new(IdleSpeedProfile::new(config.performance.cruise_speed_kt)),
        ];
        Self {
            runway_frame,
            bank_controller,
            coordination: CoordinationController::default(),
            pitch_controller,
            rollout_controller: CenterlineRolloutController::new(),
            tecs,
            flare_controller: flare,
            nose_wheel_controller: NoseWheelController::new(),
            ground_speed_controller: GroundSpeedController::new(),
            active_profiles: idle,
            latest_snapshot: None,
            config,
        }
    }

    pub fn list_profile_names(&self) -> Vec<String> {
        self.active_profiles.iter().map(|p| p.name().to_string()).collect()
    }

    /// Engage a profile. Any active profile with an overlapping axis is
    /// displaced; returns the names of displaced profiles.
    pub fn engage_profile(&mut self, profile: Box<dyn GuidanceProfile>) -> Vec<String> {
        self.engage_profiles(vec![profile])
    }

    /// Engage several profiles atomically under one logical step. Returns the
    /// set of displaced profile names (excluding any profile that was itself
    /// reinstalled in the same call — the Python `engage_profiles` does the
    /// same de-dup so "displaced" reflects only real removals).
    pub fn engage_profiles(&mut self, new_profiles: Vec<Box<dyn GuidanceProfile>>) -> Vec<String> {
        let new_names: BTreeSet<&'static str> = new_profiles.iter().map(|p| p.name()).collect();
        let mut all_displaced: BTreeSet<String> = BTreeSet::new();
        for profile in new_profiles {
            let owns = profile.owns();
            let mut retained: Vec<Box<dyn GuidanceProfile>> = Vec::with_capacity(self.active_profiles.len());
            for existing in std::mem::take(&mut self.active_profiles) {
                if !existing.owns().is_disjoint(&owns) {
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

    pub fn disengage_profile(&mut self, name: &str) -> Vec<String> {
        let mut removed_owns: BTreeSet<Axis> = BTreeSet::new();
        let mut retained: Vec<Box<dyn GuidanceProfile>> = Vec::with_capacity(self.active_profiles.len());
        for p in std::mem::take(&mut self.active_profiles) {
            if p.name() == name {
                for a in p.owns() {
                    removed_owns.insert(a);
                }
            } else {
                retained.push(p);
            }
        }
        if removed_owns.is_empty() {
            self.active_profiles = retained;
            return vec![];
        }
        let mut covered: BTreeSet<Axis> = BTreeSet::new();
        for p in &retained {
            for a in p.owns() {
                covered.insert(a);
            }
        }
        let orphans: BTreeSet<Axis> = removed_owns.difference(&covered).cloned().collect();
        let mut added: Vec<String> = Vec::new();
        for axis in orphans {
            let idle: Box<dyn GuidanceProfile> = match axis {
                Axis::Lateral => Box::new(IdleLateralProfile),
                Axis::Vertical => Box::new(IdleVerticalProfile),
                Axis::Speed => Box::new(IdleSpeedProfile::new(self.config.performance.cruise_speed_kt)),
            };
            added.push(idle.name().to_string());
            retained.push(idle);
        }
        self.active_profiles = retained;
        added
    }

    pub fn find_profile_mut<R>(&mut self, name: &str, f: impl FnOnce(&mut dyn GuidanceProfile) -> R) -> Option<R> {
        for p in &mut self.active_profiles {
            if p.name() == name {
                return Some(f(p.as_mut()));
            }
        }
        None
    }

    pub fn has_profile(&self, name: &str) -> bool {
        self.active_profiles.iter().any(|p| p.name() == name)
    }

    pub fn phase(&self) -> FlightPhase {
        self.current_phase().unwrap_or(FlightPhase::Preflight)
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
        let state = estimate_aircraft_state(raw_state, &self.config, &self.runway_frame, dt);
        let (guidance, metadata) = self.compose_guidance(&state, dt);
        let commands = self.commands_from_guidance(&state, &guidance);
        let phase = self.current_phase();
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
            let owns: Vec<Axis> = profile.owns().into_iter().collect();
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
                if let Some(t) = c.throttle_limit { targets.throttle_limit = Some(t); }
                targets.flaps_cmd = c.flaps_cmd;
                targets.gear_down = c.gear_down;
                if let Some(b) = c.brakes { targets.brakes = b; }
                if let Some(o) = c.tecs_phase_override { targets.tecs_phase_override = Some(o); }
            }
            if owns.contains(&Axis::Speed) {
                if let Some(s) = c.target_speed_kt { targets.target_speed_kt = Some(s); }
            }
        }

        // Apply any hand-offs after all profiles have ticked.
        for batch in handoffs {
            self.engage_profiles(batch);
        }
        (targets, metadata)
    }

    fn commands_from_guidance(&mut self, state: &AircraftState, guidance: &GuidanceTargets) -> ActuatorCommands {
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
            self.tecs.update(
                tecs_phase,
                guidance.target_altitude_ft.unwrap_or(state.alt_msl_ft),
                guidance.target_speed_kt.unwrap_or(state.ias_kt),
                state.alt_msl_ft,
                state.vs_fpm,
                state.ias_kt,
                state.dt,
                throttle_limit,
            )
        } else if guidance.vertical_mode == VerticalMode::FlareTrack {
            let pitch_cmd = self.flare_controller.target_pitch_deg(
                state.alt_agl_ft,
                state.vs_fpm,
                guidance.target_speed_kt.unwrap_or(state.ias_kt) - state.ias_kt,
            );
            (pitch_cmd, 0.0)
        } else {
            (guidance.target_pitch_deg.unwrap_or(0.0), throttle_limit.1)
        };

        let elevator = self.pitch_controller.update(pitch_cmd_deg, state.pitch_deg, state.q_rad_s, state.dt);

        ActuatorCommands {
            aileron: clamp(aileron, -1.0, 1.0),
            elevator: clamp(elevator, -1.0, 1.0),
            rudder: clamp(rudder, -1.0, 1.0),
            throttle: clamp(throttle_cmd, 0.0, 1.0),
            flaps: guidance.flaps_cmd,
            gear_down: Some(true),
            brakes: taxi_brake_override.unwrap_or(guidance.brakes),
            pivot_brake: clamp(taxi_pivot_brake, -1.0, 1.0),
        }
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
    let leg_vec = leg.end_ft - leg.start_ft;
    let leg_len = leg_vec.length();
    if leg_len < 1e-6 {
        return (0.0, 0.0);
    }
    let leg_dir = leg_vec * (1.0 / leg_len);
    // Right-perpendicular to the leg direction. `Vec2` is (east, north),
    // so rotating (x, y) by -90° gives (y, -x).
    let leg_right = crate::types::Vec2::new(leg_dir.y, -leg_dir.x);
    let offset = state.position_ft - leg.start_ft;
    let crosstrack_ft = offset.dot(leg_right);
    let leg_bearing = crate::types::vector_to_heading(leg_vec);
    let heading_err = wrap_degrees_180(leg_bearing - state.heading_deg);
    (crosstrack_ft, heading_err)
}

/// Pose-target heading feedback for `LateralMode::TaxiPose`. Returns
/// `(heading_err_deg, pivoting)`. When the aircraft is outside
/// `POSE_PIVOT_SWITCH_FT` of the target position, we feed the
/// aircraft-to-target bearing as the desired heading (steer toward the
/// point). Inside that radius we switch to the target heading itself
/// (pivot in place to face the commanded direction); `pivoting` flips
/// true so the caller can expose it in debug if desired.
const POSE_PIVOT_SWITCH_FT: f64 = 15.0;
fn taxi_pose_errors(guidance: &GuidanceTargets, state: &AircraftState) -> (f64, bool) {
    let target_pos = guidance
        .target_waypoint
        .as_ref()
        .map(|w| w.position_ft)
        .unwrap_or(state.position_ft);
    let target_heading = guidance.target_heading_deg.unwrap_or(state.heading_deg);
    let to_target = target_pos - state.position_ft;
    let d = to_target.length();
    if d > POSE_PIVOT_SWITCH_FT {
        let bearing_to_target = crate::types::vector_to_heading(to_target);
        let err = wrap_degrees_180(bearing_to_target - state.heading_deg);
        (err, false)
    } else {
        let err = wrap_degrees_180(target_heading - state.heading_deg);
        (err, true)
    }
}
