//! Offline scenario runner for the simple backend. Mirrors
//! sim_pilot/sim/scenario.py.

use crate::config::ConfigBundle;
use crate::core::mission_manager::PilotCore;
use crate::core::profiles::PatternFlyProfile;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::sim::simple_dynamics::{DynamicsState, SimpleAircraftModel};
use crate::types::{FlightPhase, Vec2};

#[derive(Debug, Clone)]
pub struct ScenarioLogRow {
    pub time_s: f64,
    pub phase: FlightPhase,
    pub position_x_ft: f64,
    pub position_y_ft: f64,
    pub runway_x_ft: f64,
    pub runway_y_ft: f64,
    pub pitch_deg: f64,
    pub altitude_msl_ft: f64,
    pub altitude_agl_ft: f64,
    pub throttle_pos: f64,
    pub throttle_cmd: f64,
    pub ias_kt: f64,
    pub gs_kt: f64,
    pub heading_deg: f64,
    pub bank_deg: f64,
}

#[derive(Debug, Clone)]
pub struct ScenarioResult {
    pub success: bool,
    pub final_phase: FlightPhase,
    pub duration_s: f64,
    pub touchdown_runway_x_ft: Option<f64>,
    pub touchdown_centerline_ft: Option<f64>,
    pub touchdown_sink_fpm: Option<f64>,
    pub max_final_bank_deg: f64,
    pub phases_seen: Vec<FlightPhase>,
    pub history: Vec<ScenarioLogRow>,
}

pub struct ScenarioRunner {
    pub config: ConfigBundle,
    pub wind_vector_kt: Vec2,
    pub dt: f64,
    pub max_time_s: f64,
}

impl ScenarioRunner {
    pub fn new(config: ConfigBundle) -> Self {
        Self {
            config,
            wind_vector_kt: Vec2::ZERO,
            dt: 0.2,
            max_time_s: 5400.0,
        }
    }

    pub fn with_wind(mut self, wind: Vec2) -> Self {
        self.wind_vector_kt = wind;
        self
    }

    pub fn run(self) -> ScenarioResult {
        let model = SimpleAircraftModel::new(self.config.clone(), self.wind_vector_kt);
        let mut pilot = PilotCore::new(self.config.clone());
        let runway_frame = RunwayFrame::new(self.config.airport.runway.clone());
        pilot.engage_profile(Box::new(PatternFlyProfile::new(self.config.clone(), runway_frame.clone())));
        let mut raw_state = model.initial_state();

        let mut history: Vec<ScenarioLogRow> = Vec::new();
        let mut phases_seen: Vec<FlightPhase> = Vec::new();
        let mut touchdown_runway_x_ft: Option<f64> = None;
        let mut touchdown_centerline_ft: Option<f64> = None;
        let mut touchdown_sink_fpm: Option<f64> = None;
        let mut max_final_bank_deg: f64 = 0.0;
        let mut prev_airborne = false;

        while raw_state.time_s <= self.max_time_s {
            let (estimated, commands) = pilot.update(&raw_state, self.dt);
            let phase = pilot.phase();
            history.push(ScenarioLogRow {
                time_s: estimated.t_sim,
                phase,
                position_x_ft: estimated.position_ft.x,
                position_y_ft: estimated.position_ft.y,
                runway_x_ft: estimated.runway_x_ft.unwrap_or(0.0),
                runway_y_ft: estimated.runway_y_ft.unwrap_or(0.0),
                pitch_deg: estimated.pitch_deg,
                altitude_msl_ft: estimated.alt_msl_ft,
                altitude_agl_ft: estimated.alt_agl_ft,
                throttle_pos: estimated.throttle_pos,
                throttle_cmd: commands.throttle,
                ias_kt: estimated.ias_kt,
                gs_kt: estimated.gs_kt,
                heading_deg: estimated.heading_deg,
                bank_deg: estimated.roll_deg,
            });
            match phases_seen.last() {
                Some(&last) if last == phase => {}
                _ => phases_seen.push(phase),
            }
            if matches!(phase, FlightPhase::Final | FlightPhase::Roundout | FlightPhase::Flare) {
                max_final_bank_deg = max_final_bank_deg.max(estimated.roll_deg.abs());
            }

            let pre_step = raw_state.clone();
            let mut stepped = raw_state.clone();
            model.step(&mut stepped, &commands, self.dt);
            raw_state = stepped;

            if prev_airborne && raw_state.on_ground && touchdown_runway_x_ft.is_none() {
                let td = runway_frame.to_runway_frame(raw_state.position_ft);
                touchdown_runway_x_ft = Some(td.x);
                touchdown_centerline_ft = Some(td.y);
                touchdown_sink_fpm = Some(pre_step.vertical_speed_ft_s * 60.0);
            }
            prev_airborne = !raw_state.on_ground;

            if phase == FlightPhase::TaxiClear {
                break;
            }
            // ModeManager holds Rollout until past the runway end, so a
            // sim that brakes to a stop mid-runway would otherwise spin
            // to max_time_s.
            if phase == FlightPhase::Rollout
                && raw_state.ground_velocity_ft_s.length() < 1.0
            {
                break;
            }
        }

        let final_phase = pilot.phase();
        let runway_length = self.config.airport.runway.length_ft;
        let touchdown_in_zone = touchdown_runway_x_ft
            .map(|x| (0.0..=runway_length / 3.0).contains(&x))
            .unwrap_or(false);
        let centerline_ok = touchdown_centerline_ft
            .map(|y| y.abs() < 20.0)
            .unwrap_or(false);
        let sink_ok = touchdown_sink_fpm
            .map(|v| v.abs() < 350.0)
            .unwrap_or(false);
        let bank_ok = max_final_bank_deg <= self.config.limits.max_bank_final_deg + 0.5;
        let phase_ok = matches!(final_phase, FlightPhase::Rollout | FlightPhase::TaxiClear);

        ScenarioResult {
            success: touchdown_in_zone && centerline_ok && sink_ok && bank_ok && phase_ok,
            final_phase,
            duration_s: raw_state.time_s,
            touchdown_runway_x_ft,
            touchdown_centerline_ft,
            touchdown_sink_fpm,
            max_final_bank_deg,
            phases_seen,
            history,
        }
    }
}

// silence unused warning - DynamicsState is re-exported through the module
#[allow(dead_code)]
fn _keep_dynamics_state_in_scope(_: DynamicsState) {}
