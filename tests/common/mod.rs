//! Shared test helpers — a Rust translation of the per-test `make_state`
//! factories in the Python suite.

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
