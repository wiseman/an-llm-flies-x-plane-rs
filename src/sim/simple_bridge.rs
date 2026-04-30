//! In-process `ToolBridge` for the eval harness.
//!
//! The live X-Plane runtime uses `XPlaneWebBridge` to read state and
//! write actuator-like datarefs over a WebSocket. The eval harness runs
//! the deterministic `SimpleAircraftModel` in-process, so the bridge
//! reads position/heading/speeds directly from the shared
//! `DynamicsState` and keeps written datarefs (radio freqs, parking
//! brake, flap requests, etc.) in an in-memory map. A few derived
//! values the LLM expects — fuel mass, engine RPM, has_crashed — are
//! maintained by the control loop via `refresh_derived_datarefs`.

use std::collections::HashMap;
use std::sync::Arc;

use anyhow::Result;
use parking_lot::Mutex as PLMutex;

use crate::llm::tools::ToolBridge;
use crate::sim::datarefs::{
    COM1_FREQUENCY_HZ_833, COM2_FREQUENCY_HZ_833, ELEVATION_M, ENGINE_RPM, FLAP_HANDLE_DEPLOY_RATIO,
    FLAP_HANDLE_REQUEST_RATIO, FUEL_CAPACITY_KG, FUEL_FLOW_KG_SEC, FUEL_KG, GEAR_HANDLE_DOWN,
    HAS_CRASHED, HEADING_DEG, IAS_KT, LATITUDE_DEG, LOCAL_VX_M_S, LOCAL_VZ_M_S, LONGITUDE_DEG,
    ON_GROUND_0, P_DEG_S, PARKING_BRAKE_RATIO, PITCH_DEG, Q_DEG_S, R_DEG_S, ROLL_DEG, SIM_TIME_S,
    THROTTLE_ALL, VS_FPM, Y_AGL_M,
};
use crate::sim::simple_dynamics::DynamicsState;
use crate::sim::xplane_bridge::{
    flap_ratio_to_setting, flap_setting_to_ratio, latlon_from_offset_ft, GeoReference, M_TO_FT,
};
use crate::types::Vec2;

/// Nominal fuel-burn rate in kg per second at full throttle. The C172
/// sips about 8 gal/hr ≈ 6.7 kg/hr at cruise power; we scale linearly
/// with throttle so taxi-idle burns much less. The exact number doesn't
/// matter for the eval — it's just there so the LLM sees fuel moving.
const FUEL_BURN_KG_PER_S_AT_FULL: f64 = 6.7 / 3600.0;
/// Starting fuel load and tank capacity for the eval aircraft.
const INITIAL_FUEL_KG: f64 = 100.0;
const FUEL_CAPACITY_KG_VALUE: f64 = 120.0;

pub struct SimpleToolBridge {
    pub dynamics: Arc<PLMutex<DynamicsState>>,
    pub georef: GeoReference,
    /// Flat-earth anchor for elevation-to-position conversion is
    /// implicit in the dynamics state already; `georef` is only used to
    /// convert runway-frame feet back into geodetic degrees.
    pub field_elevation_ft: f64,
    values: PLMutex<HashMap<String, f64>>,
}

impl SimpleToolBridge {
    pub fn new(
        dynamics: Arc<PLMutex<DynamicsState>>,
        georef: GeoReference,
        field_elevation_ft: f64,
    ) -> Self {
        let mut values = HashMap::new();
        // Without these seeds `build_status_payload` elides the fields,
        // leaving the LLM blind to fuel/radio/brake state.
        values.insert(FUEL_KG.name.to_string(), INITIAL_FUEL_KG);
        values.insert(FUEL_CAPACITY_KG.name.to_string(), FUEL_CAPACITY_KG_VALUE);
        values.insert(FUEL_FLOW_KG_SEC.name.to_string(), 0.0);
        values.insert(ENGINE_RPM.name.to_string(), 0.0);
        values.insert(PARKING_BRAKE_RATIO.name.to_string(), 1.0);
        values.insert(HAS_CRASHED.name.to_string(), 0.0);
        values.insert(COM1_FREQUENCY_HZ_833.name.to_string(), 122_800_000.0 / 1_000.0);
        values.insert(COM2_FREQUENCY_HZ_833.name.to_string(), 121_500_000.0 / 1_000.0);
        Self {
            dynamics,
            georef,
            field_elevation_ft,
            values: PLMutex::new(values),
        }
    }

    /// Read-only snapshot of the in-memory dataref map. Mostly useful
    /// for tests that want to assert the LLM wrote what it should have.
    pub fn snapshot_values(&self) -> HashMap<String, f64> {
        self.values.lock().clone()
    }

    /// Called once per sim tick by the control loop: decay fuel,
    /// update engine RPM from throttle, and (if detected) latch the
    /// crash flag. `dt` is the sim timestep in seconds.
    pub fn refresh_derived_datarefs(&self, dt: f64) {
        let (throttle_pos, ias_kt, vs_ft_s, alt_ft, on_ground) = {
            let d = self.dynamics.lock();
            (d.throttle_pos, d.ias_kt, d.vertical_speed_ft_s, d.altitude_ft, d.on_ground)
        };
        let flow_kg_s = FUEL_BURN_KG_PER_S_AT_FULL * throttle_pos.max(0.0);
        let rpm = 600.0 + 2100.0 * throttle_pos.clamp(0.0, 1.0);
        let mut v = self.values.lock();
        let fuel = v
            .get(FUEL_KG.name)
            .copied()
            .unwrap_or(INITIAL_FUEL_KG)
            - flow_kg_s * dt;
        v.insert(FUEL_KG.name.to_string(), fuel.max(0.0));
        v.insert(FUEL_FLOW_KG_SEC.name.to_string(), flow_kg_s);
        v.insert(ENGINE_RPM.name.to_string(), rpm);
        // Once latched, has_crashed stays latched — matches X-Plane.
        let crashed_now = v.get(HAS_CRASHED.name).copied().unwrap_or(0.0) >= 0.5;
        let hard_landing = on_ground && ias_kt < 20.0 && vs_ft_s * 60.0 < -1500.0;
        let below_field = alt_ft < self.field_elevation_ft - 5.0;
        if !crashed_now && (hard_landing || below_field) {
            v.insert(HAS_CRASHED.name.to_string(), 1.0);
        }
    }

    /// Force the crash flag from outside (used by the control loop when
    /// the eval harness wants to treat an external condition — sim-time
    /// budget reached during an unrecoverable state — as a failure).
    pub fn set_crashed(&self) {
        self.values.lock().insert(HAS_CRASHED.name.to_string(), 1.0);
    }

    /// Feet (east, north) offset from the georef anchor → geodetic
    fn offset_to_latlon(&self, offset_ft: Vec2) -> (f64, f64) {
        latlon_from_offset_ft(offset_ft, self.georef)
    }
}

impl ToolBridge for SimpleToolBridge {
    fn georef(&self) -> GeoReference {
        self.georef
    }

    fn get_dataref_value(&self, name: &str) -> Option<f64> {
        // Fast-path the "live physics" datarefs by computing them from
        // the current DynamicsState on demand. Everything else comes
        // out of the in-memory map.
        let d = self.dynamics.lock();
        if name == LATITUDE_DEG.name || name == LONGITUDE_DEG.name {
            let (lat, lon) = self.offset_to_latlon(d.position_ft);
            return Some(if name == LATITUDE_DEG.name { lat } else { lon });
        }
        match name {
            n if n == ELEVATION_M.name => Some(d.altitude_ft / M_TO_FT),
            n if n == PITCH_DEG.name => Some(d.pitch_deg),
            n if n == ROLL_DEG.name => Some(d.roll_deg),
            n if n == HEADING_DEG.name => Some(d.heading_deg),
            n if n == IAS_KT.name => Some(d.ias_kt),
            n if n == LOCAL_VX_M_S.name => Some(d.ground_velocity_ft_s.x / M_TO_FT),
            n if n == LOCAL_VZ_M_S.name => Some(-d.ground_velocity_ft_s.y / M_TO_FT),
            n if n == VS_FPM.name => Some(d.vertical_speed_ft_s * 60.0),
            n if n == P_DEG_S.name => Some(d.p_rad_s.to_degrees()),
            n if n == Q_DEG_S.name => Some(d.q_rad_s.to_degrees()),
            n if n == R_DEG_S.name => Some(d.r_rad_s.to_degrees()),
            n if n == THROTTLE_ALL.name => Some(d.throttle_pos),
            n if n == FLAP_HANDLE_DEPLOY_RATIO.name => {
                Some(flap_setting_to_ratio(d.flap_index))
            }
            n if n == GEAR_HANDLE_DOWN.name => Some(if d.gear_down { 1.0 } else { 0.0 }),
            n if n == Y_AGL_M.name => {
                let agl_ft = d.altitude_ft - self.field_elevation_ft;
                Some(agl_ft.max(0.0) / M_TO_FT)
            }
            n if n == SIM_TIME_S.name => Some(d.time_s),
            n if n == ON_GROUND_0.name => Some(if d.on_ground { 1.0 } else { 0.0 }),
            _ => {
                drop(d);
                self.values.lock().get(name).copied()
            }
        }
    }

    fn write_dataref_values(&self, updates: &[(String, f64)]) -> Result<()> {
        let mut v = self.values.lock();
        for (name, value) in updates {
            v.insert(name.clone(), *value);
            if name == FLAP_HANDLE_REQUEST_RATIO.name {
                self.dynamics.lock().flap_index = flap_ratio_to_setting(*value);
            }
        }
        Ok(())
    }
}

