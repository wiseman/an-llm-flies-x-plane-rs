//! Configuration loading.
//!
//! The YAML files live under `config/` and are embedded via `include_str!`
//! so the compiled binary is self-contained. On-disk overrides go through
//! `load_config_bundle`. Parsing is serde + serde_yaml_ng; the Raw* structs
//! below mirror the YAML layout and then fold into the existing
//! `ConfigBundle` public API.

use std::fs;
use std::path::Path;

use anyhow::{anyhow, Context, Result};
use serde::Deserialize;

use crate::types::{Runway, TrafficSide, Vec2};

const AIRCRAFT_YAML: &str = include_str!("../config/aircraft_c172.yaml");
const AIRPORT_YAML: &str = include_str!("../config/airport_defaults.yaml");
const CONTROLLER_YAML: &str = include_str!("../config/controller_gains.yaml");
const SAFETY_YAML: &str = include_str!("../config/safety_limits.yaml");

// ---------- public API ----------

#[derive(Debug, Clone)]
pub struct PerformanceConfig {
    pub aircraft: String,
    pub vr_kt: f64,
    pub vx_kt: f64,
    pub vy_kt: f64,
    pub cruise_altitude_ft: f64,
    pub cruise_speed_kt: f64,
    pub descent_speed_kt: f64,
    pub downwind_speed_kt: f64,
    pub base_speed_kt: f64,
    pub final_speed_kt: f64,
    pub vapp_kt: f64,
    pub vref_kt: f64,
    pub vso_landing_kt: f64,
    pub vbg_kt: f64,
    pub glide_ratio: f64,
}

#[derive(Debug, Clone)]
pub struct PatternConfig {
    pub altitude_agl_ft: f64,
    pub downwind_offset_ft: f64,
    pub abeam_window_ft: f64,
    pub default_extension_ft: f64,
}

#[derive(Debug, Clone)]
pub struct FlareConfig {
    pub roundout_height_ft: f64,
    pub flare_start_ft: f64,
    pub max_flare_pitch_deg: f64,
}

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct PostLandingConfig {
    /// Target ground speed during rollout after touchdown. The pilot
    /// decelerates to this speed and holds it — the aircraft keeps
    /// rolling rather than braking to a stop on the runway.
    pub turnoff_speed_kt: f64,
}

impl Default for PostLandingConfig {
    fn default() -> Self {
        Self { turnoff_speed_kt: 15.0 }
    }
}

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct PIDGains {
    pub kp: f64,
    pub kd: f64,
    pub ki: f64,
}

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct TECSGains {
    pub kp_total: f64,
    pub ki_total: f64,
    pub kp_balance: f64,
    pub kd_balance: f64,
}

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct ControllerConfig {
    pub bank: PIDGains,
    pub pitch: PIDGains,
    pub tecs: TECSGains,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SafetyLimits {
    pub max_bank_enroute_deg: f64,
    pub max_bank_pattern_deg: f64,
    pub max_bank_final_deg: f64,
    pub max_pitch_up_deg: f64,
    pub max_pitch_down_deg: f64,
    pub min_stall_margin: f64,
    pub unstable_sink_rate_fpm: f64,
    pub unstable_centerline_error_ft: f64,
}

#[derive(Debug, Clone)]
pub struct MissionConfig {
    pub entry_start_runway_ft: Vec2,
}

#[derive(Debug, Clone)]
pub struct AirportConfig {
    pub airport: Option<String>,
    pub field_elevation_ft: f64,
    pub runway: Runway,
    pub mission: MissionConfig,
}

#[derive(Debug, Clone)]
pub struct ConfigBundle {
    pub performance: PerformanceConfig,
    pub pattern: PatternConfig,
    pub flare: FlareConfig,
    pub controllers: ControllerConfig,
    pub limits: SafetyLimits,
    pub airport: AirportConfig,
    pub post_landing: PostLandingConfig,
}

impl ConfigBundle {
    pub fn cruise_altitude_ft(&self) -> f64 {
        self.performance.cruise_altitude_ft
    }
    pub fn pattern_altitude_msl_ft(&self) -> f64 {
        self.airport.field_elevation_ft + self.pattern.altitude_agl_ft
    }
}

pub fn load_default_config_bundle() -> ConfigBundle {
    load_config_bundle_from(AIRCRAFT_YAML, AIRPORT_YAML, CONTROLLER_YAML, SAFETY_YAML)
        .expect("bundled YAML is malformed; this is a bug")
}

/// Load from on-disk files; used by the CLI when a user wants to override the
/// bundled defaults. Falls back to the embedded files for any path that's `None`.
pub fn load_config_bundle(
    aircraft: Option<&Path>,
    airport: Option<&Path>,
    controller: Option<&Path>,
    safety: Option<&Path>,
) -> Result<ConfigBundle> {
    fn read_or(p: Option<&Path>, fallback: &str) -> Result<String> {
        match p {
            Some(path) => fs::read_to_string(path)
                .with_context(|| format!("reading {}", path.display())),
            None => Ok(fallback.to_string()),
        }
    }
    load_config_bundle_from(
        &read_or(aircraft, AIRCRAFT_YAML)?,
        &read_or(airport, AIRPORT_YAML)?,
        &read_or(controller, CONTROLLER_YAML)?,
        &read_or(safety, SAFETY_YAML)?,
    )
}

// ---------- raw YAML layout ----------

#[derive(Debug, Deserialize)]
struct RawAircraft {
    aircraft: String,
    vr_kt: f64,
    vx_kt: f64,
    vy_kt: f64,
    cruise_altitude_ft: f64,
    cruise_speed_kt: f64,
    descent_speed_kt: f64,
    downwind_speed_kt: f64,
    base_speed_kt: f64,
    final_speed_kt: f64,
    vapp_kt: f64,
    vref_kt: f64,
    vso_landing_kt: f64,
    #[serde(default = "default_vbg_kt")]
    vbg_kt: f64,
    #[serde(default = "default_glide_ratio")]
    glide_ratio: f64,
    pattern: RawPattern,
    flare: RawFlare,
    #[serde(default)]
    post_landing: Option<RawPostLanding>,
}

#[derive(Debug, Deserialize)]
struct RawPostLanding {
    turnoff_speed_kt: f64,
}

fn default_vbg_kt() -> f64 {
    68.0
}

fn default_glide_ratio() -> f64 {
    9.0
}

#[derive(Debug, Deserialize)]
struct RawPattern {
    altitude_agl_ft: f64,
    downwind_offset_ft: f64,
    abeam_window_ft: f64,
    default_extension_ft: f64,
}

#[derive(Debug, Deserialize)]
struct RawFlare {
    roundout_height_ft: f64,
    flare_start_ft: f64,
    max_flare_pitch_deg: f64,
}

#[derive(Debug, Deserialize)]
struct RawAirport {
    airport: Option<String>,
    field_elevation_ft: f64,
    runway: RawRunway,
    mission: RawMission,
}

/// Runway id can be `36` (integer, unquoted) or `"16L"` (string) in the YAML.
/// `RunwayId` lets serde accept either and stringify.
#[derive(Debug, Deserialize)]
#[serde(untagged)]
enum RunwayId {
    Int(i64),
    Str(String),
}

impl RunwayId {
    fn into_string(self) -> String {
        match self {
            RunwayId::Int(n) => n.to_string(),
            RunwayId::Str(s) => s,
        }
    }
}

#[derive(Debug, Deserialize)]
struct RawRunway {
    id: RunwayId,
    course_deg: f64,
    threshold_x_ft: f64,
    threshold_y_ft: f64,
    length_ft: f64,
    touchdown_zone_ft: f64,
    traffic_side: String,
}

#[derive(Debug, Deserialize)]
struct RawMission {
    entry_start_runway_x_ft: f64,
    entry_start_runway_y_ft: f64,
}

fn load_config_bundle_from(
    aircraft_text: &str,
    airport_text: &str,
    controller_text: &str,
    safety_text: &str,
) -> Result<ConfigBundle> {
    let aircraft: RawAircraft =
        serde_yaml_ng::from_str(aircraft_text).context("parsing aircraft yaml")?;
    let airport: RawAirport =
        serde_yaml_ng::from_str(airport_text).context("parsing airport yaml")?;
    let controllers: ControllerConfig =
        serde_yaml_ng::from_str(controller_text).context("parsing controller yaml")?;
    let limits: SafetyLimits =
        serde_yaml_ng::from_str(safety_text).context("parsing safety yaml")?;

    let performance = PerformanceConfig {
        aircraft: aircraft.aircraft,
        vr_kt: aircraft.vr_kt,
        vx_kt: aircraft.vx_kt,
        vy_kt: aircraft.vy_kt,
        cruise_altitude_ft: aircraft.cruise_altitude_ft,
        cruise_speed_kt: aircraft.cruise_speed_kt,
        descent_speed_kt: aircraft.descent_speed_kt,
        downwind_speed_kt: aircraft.downwind_speed_kt,
        base_speed_kt: aircraft.base_speed_kt,
        final_speed_kt: aircraft.final_speed_kt,
        vapp_kt: aircraft.vapp_kt,
        vref_kt: aircraft.vref_kt,
        vso_landing_kt: aircraft.vso_landing_kt,
        vbg_kt: aircraft.vbg_kt,
        glide_ratio: aircraft.glide_ratio,
    };
    let pattern = PatternConfig {
        altitude_agl_ft: aircraft.pattern.altitude_agl_ft,
        downwind_offset_ft: aircraft.pattern.downwind_offset_ft,
        abeam_window_ft: aircraft.pattern.abeam_window_ft,
        default_extension_ft: aircraft.pattern.default_extension_ft,
    };
    let flare = FlareConfig {
        roundout_height_ft: aircraft.flare.roundout_height_ft,
        flare_start_ft: aircraft.flare.flare_start_ft,
        max_flare_pitch_deg: aircraft.flare.max_flare_pitch_deg,
    };
    let post_landing = aircraft
        .post_landing
        .map(|p| PostLandingConfig { turnoff_speed_kt: p.turnoff_speed_kt })
        .unwrap_or_default();

    let runway = Runway {
        id: Some(airport.runway.id.into_string()),
        threshold_ft: Vec2::new(airport.runway.threshold_x_ft, airport.runway.threshold_y_ft),
        course_deg: airport.runway.course_deg,
        length_ft: airport.runway.length_ft,
        touchdown_zone_ft: airport.runway.touchdown_zone_ft,
        // Config YAML doesn't model a displaced threshold — test configs
        // use the simple full-pavement runway. Real-runway displacement
        // comes in through `lookup_runway_for_pattern` which reads it
        // from apt.dat.
        displaced_threshold_ft: 0.0,
        traffic_side: TrafficSide::from_str(&airport.runway.traffic_side).ok_or_else(|| {
            anyhow!(
                "invalid traffic_side {:?}; expected 'left' or 'right'",
                airport.runway.traffic_side
            )
        })?,
    };
    let airport_cfg = AirportConfig {
        airport: airport.airport,
        field_elevation_ft: airport.field_elevation_ft,
        runway,
        mission: MissionConfig {
            entry_start_runway_ft: Vec2::new(
                airport.mission.entry_start_runway_x_ft,
                airport.mission.entry_start_runway_y_ft,
            ),
        },
    };

    Ok(ConfigBundle {
        performance,
        pattern,
        flare,
        controllers,
        limits,
        airport: airport_cfg,
        post_landing,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loads_built_in_bundle() {
        let cfg = load_default_config_bundle();
        assert_eq!(cfg.performance.aircraft, "c172");
        assert_eq!(cfg.airport.airport.as_deref(), Some("KTEST"));
        assert_eq!(cfg.airport.runway.id.as_deref(), Some("36"));
        assert_eq!(cfg.airport.runway.traffic_side, TrafficSide::Left);
    }

    #[test]
    fn pattern_altitude_msl_ft_is_field_plus_pattern() {
        let cfg = load_default_config_bundle();
        assert_eq!(
            cfg.pattern_altitude_msl_ft(),
            cfg.airport.field_elevation_ft + cfg.pattern.altitude_agl_ft
        );
    }

    #[test]
    fn serde_yaml_rejects_lists_that_go_into_scalar_field() {
        // Smoke-check that the YAML parser actually validates types now —
        // the old hand-rolled parser silently accepted anything.
        let bad = "aircraft: c172\nvr_kt: [1, 2, 3]\n";
        assert!(serde_yaml_ng::from_str::<RawAircraft>(bad).is_err());
    }
}
