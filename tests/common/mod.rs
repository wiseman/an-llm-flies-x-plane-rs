//! Shared test helpers — per-test `make_state` factories.
//!
//! Each integration-test binary compiles this module separately, so any
//! helper a given binary doesn't reference trips the dead-code lint.
//! The helpers here are a shared palette — not every test uses every
//! one — so disable the lint at the module level.
#![allow(dead_code)]

use tempfile::TempDir;
use xplane_pilot::core::landing_rollout::RunwayExitPlan;
use xplane_pilot::eval_runner::{EvalConfig, ResolvedStart};
use xplane_pilot::llm::backend::LlmProvider;
use xplane_pilot::llm::conversation::{IncomingMessage, PilotMode};
use xplane_pilot::types::{
    heading_to_vector, AircraftState, FlightPhase, Vec2, KT_TO_FPS,
};

pub fn default_state() -> AircraftState {
    AircraftState::synthetic_default()
}

pub fn state_with(f: impl FnOnce(&mut AircraftState)) -> AircraftState {
    let mut s = default_state();
    f(&mut s);
    s
}

pub fn ground_state_at_rotate(vr_kt: f64) -> AircraftState {
    state_with(|s| {
        s.on_ground = true;
        s.ias_kt = vr_kt + 1.0;
        s.gs_kt = vr_kt + 1.0;
        s.alt_agl_ft = 0.0;
        s.runway_x_ft = Some(2000.0);
        s.runway_y_ft = Some(0.0);
        s.centerline_error_ft = Some(0.0);
    })
}

pub fn airborne_state(track_deg: f64, ias: f64) -> AircraftState {
    state_with(|s| {
        s.on_ground = false;
        s.track_deg = track_deg;
        s.heading_deg = track_deg;
        s.ias_kt = ias;
        s.tas_kt = ias;
        s.gs_kt = ias;
        s.ground_velocity_ft_s = heading_to_vector(track_deg, ias * KT_TO_FPS);
    })
}

pub fn on_ground_at_phase(phase: FlightPhase) -> AircraftState {
    let _ = phase;
    state_with(|s| {
        s.on_ground = true;
        s.alt_agl_ft = 5.0;
        s.ias_kt = 55.0;
        s.gs_kt = 55.0;
        s.runway_x_ft = Some(1000.0);
        s.runway_y_ft = Some(0.0);
    })
}

pub fn zero_vec() -> Vec2 {
    Vec2::ZERO
}

pub fn kwhp_resolved() -> ResolvedStart {
    // Real KWHP values are close enough for the test — the exact numbers
    // don't matter, we just need a self-consistent geometry. Runway 12
    // threshold / heading pulled from apt.dat.
    ResolvedStart {
        runway_ident: "12".to_string(),
        runway_threshold_lat_deg: 34.2583,
        runway_threshold_lon_deg: -118.4129,
        runway_course_deg: 121.0,
        runway_length_ft: 4120.0,
        runway_width_ft: 75.0,
        runway_displaced_threshold_ft: 0.0,
        field_elevation_ft: 1003.0,
        // Parking spot ~400 ft north of the threshold, facing runway-ish.
        parking_lat_deg: 34.2594,
        parking_lon_deg: -118.4135,
        parking_heading_deg: 180.0,
    }
}

/// A stub-provider eval config writing into `tmp`.
pub fn make_eval_config(tmp: &TempDir, stem: &str) -> EvalConfig {
    EvalConfig {
        airport: "KWHP".to_string(),
        parking_spot: "Test Ramp".to_string(),
        apt_dat_cache_dir: tmp.path().to_path_buf(),
        llm_provider: LlmProvider::OpenAi,
        llm_model: "stub-model".to_string(),
        llm_reasoning_effort: None,
        startup_messages: vec![IncomingMessage::operator("Eval test startup.")],
        pilot_mode: PilotMode::Normal,
        max_turns: 40,
        max_output_tokens: 0,
        max_sim_seconds: 120.0,
        dt: 0.2,
        heartbeat_interval_s: 30.0,
        output_dir: tmp.path().to_path_buf(),
        session_stem: stem.to_string(),
        verbose: false,
    }
}

/// A runway exit whose path runs through `points` (runway-frame feet),
/// stationed at the first point's x.
pub fn exit(name: &str, points: &[(f64, f64)]) -> RunwayExitPlan {
    RunwayExitPlan {
        taxiway_name: name.to_string(),
        stationing_ft: points[0].0,
        path_runway_ft: points.iter().map(|&(x, y)| Vec2::new(x, y)).collect(),
    }
}
