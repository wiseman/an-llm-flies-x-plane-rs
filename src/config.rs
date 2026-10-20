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

/// Landing rollout and runway-exit tuning. Every field has a default, so
/// an override file only needs the values it changes.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(default)]
pub struct PostLandingConfig {
    /// Target ground speed during rollout after touchdown when an exit is
    /// selected. The pilot decelerates to this speed and holds it until
    /// the slowdown for the exit.
    pub turnoff_speed_kt: f64,
    /// Speed for the turn off the runway onto the exit.
    pub exit_speed_kt: f64,
    /// Speed along the exit once lined up on it.
    pub exit_taxi_speed_kt: f64,
    /// Braking (ft/s²) from turnoff speed down to the exit's turn speed,
    /// timed so the aircraft arrives at the turn at that speed rather
    /// than creeping the last stretch.
    pub exit_slowdown_fps2: f64,
    /// Floor on the exit's final-node speed ramp, so the aircraft keeps
    /// creeping until the node is captured.
    pub exit_creep_speed_kt: f64,
    /// Distance before the exit (or runway end, after a missed exit) at
    /// which rollout slows from turnoff speed.
    pub slowdown_lead_ft: f64,
    /// Distance over which speed ramps down to the exit's final node.
    pub final_stop_ramp_ft: f64,
    /// Commit to the exit within this distance before its intersection.
    pub intersection_capture_ft: f64,
    /// Declare the exit missed this far past its intersection.
    pub missed_exit_ft: f64,
    /// Capture radius for each node along the exit connector.
    pub leg_capture_ft: f64,
    /// Ground speed at or below which the aircraft counts as stopped.
    pub stopped_gs_kt: f64,
    /// Brake applied when stopping on the runway with no exit.
    pub rollout_brake: f64,
    /// Clearance beyond the runway edge (half the width) that counts as
    /// having left the runway surface.
    pub runway_clear_margin_ft: f64,
    /// Lateral offset of the hold-short line from the runway centerline;
    /// past it the aircraft is clear of the runway. 125 ft is the FAA
    /// minimum for runways serving small aircraft.
    pub hold_line_offset_ft: f64,
    /// Auto exit selection skips exits closer than this to the touchdown
    /// point (rough landing roll for the aircraft).
    pub min_rollout_past_touchdown_ft: f64,
    /// Still airborne on short final / in the flare with less runway than
    /// this left ahead: go around rather than land long. (A C172's ground
    /// roll is ~600 ft; this leaves margin for braking and a late flare.)
    pub go_around_min_runway_remaining_ft: f64,
}

impl Default for PostLandingConfig {
    fn default() -> Self {
        Self {
            turnoff_speed_kt: 15.0,
            exit_speed_kt: 5.0,
            exit_taxi_speed_kt: 10.0,
            exit_slowdown_fps2: 1.5,
            exit_creep_speed_kt: 2.0,
            slowdown_lead_ft: 300.0,
            final_stop_ramp_ft: 60.0,
            intersection_capture_ft: 20.0,
            missed_exit_ft: 60.0,
            leg_capture_ft: 8.0,
            stopped_gs_kt: 1.0,
            rollout_brake: 0.75,
            runway_clear_margin_ft: 20.0,
            hold_line_offset_ft: 125.0,
            min_rollout_past_touchdown_ft: 800.0,
            go_around_min_runway_remaining_ft: 1500.0,
        }
    }
}

/// Wheel/ground handling: the nose-wheel geometry route planning uses, and
/// the offline simple simulator's ground model. Accelerations are in kt/s
/// (the simulator integrates IAS directly).
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(default)]
pub struct GroundHandlingConfig {
    /// Nose wheel to main gear (ft). C172 ≈ 5.2.
    pub wheelbase_ft: f64,
    /// Nose-wheel deflection at full rudder pedal (deg). C172 ≈ 10.
    pub max_nosewheel_deg: f64,
    /// Tire grip limit on lateral acceleration (g). Caps the nose-wheel
    /// yaw rate at speed to `a_lat / v`.
    pub max_lateral_accel_g: f64,
    /// Rate (1/s) at which yaw rate converges on the nose-wheel's
    /// kinematic yaw rate.
    pub nose_wheel_responsiveness: f64,
    /// Constant rolling-friction deceleration while moving (kt/s).
    pub rolling_friction_kt_s: f64,
    /// Speed-proportional rolling drag (kt/s per kt).
    pub rolling_drag_per_kt: f64,
    /// Thrust (kt/s) needed to break away from a standstill.
    pub breakaway_kt_s: f64,
    /// Below this speed (kt) static friction holds the aircraft unless
    /// thrust exceeds braking plus breakaway.
    pub static_friction_kt: f64,
    /// Deceleration at full symmetric brake (kt/s).
    pub brake_decel_kt_s: f64,
    /// Yaw acceleration (deg/s²) per unit of left/right brake asymmetry.
    pub differential_brake_yaw_accel: f64,
    /// Yaw-rate damping (1/s).
    pub yaw_damping: f64,
    /// Aerodynamic rudder yaw acceleration (deg/s²) at full rudder and
    /// `rudder_reference_kt`; scales with dynamic pressure.
    pub aero_rudder_yaw_accel: f64,
    pub rudder_reference_kt: f64,
}

impl GroundHandlingConfig {
    /// Tightest turn radius on the nose wheel alone (ft).
    pub fn min_turn_radius_ft(&self) -> f64 {
        self.wheelbase_ft / self.max_nosewheel_deg.to_radians().tan().max(1e-3)
    }
}

impl Default for GroundHandlingConfig {
    fn default() -> Self {
        Self {
            wheelbase_ft: 5.2,
            max_nosewheel_deg: 10.0,
            max_lateral_accel_g: 0.4,
            nose_wheel_responsiveness: 3.0,
            rolling_friction_kt_s: 0.5,
            rolling_drag_per_kt: 0.02,
            breakaway_kt_s: 2.0,
            static_friction_kt: 0.1,
            brake_decel_kt_s: 10.0,
            differential_brake_yaw_accel: 50.0,
            yaw_damping: 2.5,
            aero_rudder_yaw_accel: 16.0,
            rudder_reference_kt: 60.0,
        }
    }
}

/// Power-off pattern energy management, as glide ratios still needed to
/// reach the aim point (distance / height). Aircraft-specific: they sit
/// between the full-flap and clean glide ratios with margin for turns
/// and flare.
#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct DeadStickConfig {
    /// `(needed_ratio, flaps)`, steepest-glide-last: the first step the
    /// needed ratio exceeds picks the flaps; below all of them, full flap.
    pub flap_glide_ratio_steps: Vec<(f64, i32)>,
    /// Turn base once the base + final glide needs this ratio.
    pub base_turn_glide_ratio: f64,
    /// The base turn leaves at least this much final to the aim point.
    pub min_final_ft: f64,
    /// Hard cap on extending downwind past the nominal base turn.
    pub max_downwind_extension_ft: f64,
}

impl Default for DeadStickConfig {
    fn default() -> Self {
        Self {
            flap_glide_ratio_steps: vec![(7.5, 0), (6.8, 10), (6.0, 20)],
            base_turn_glide_ratio: 6.0,
            min_final_ft: 1800.0,
            max_downwind_extension_ft: 4000.0,
        }
    }
}

/// Altitude / vertical-speed gates for the climb-out phase transitions.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(default)]
pub struct PhaseGateConfig {
    /// AGL at which Rotate hands over to InitialClimb.
    pub rotate_agl_ft: f64,
    /// AGL at which InitialClimb advances to EnrouteClimb when not
    /// staying in the pattern.
    pub enroute_climb_agl_ft: f64,
    /// InitialClimb won't advance (crosswind turn or enroute climb) while
    /// sinking faster than this. Negative: a descent rate.
    pub climb_gate_max_sink_fpm: f64,
}

impl Default for PhaseGateConfig {
    fn default() -> Self {
        Self {
            rotate_agl_ft: 20.0,
            enroute_climb_agl_ft: 400.0,
            climb_gate_max_sink_fpm: -300.0,
        }
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
    pub phase_gates: PhaseGateConfig,
    pub ground_handling: GroundHandlingConfig,
    pub dead_stick: DeadStickConfig,
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
    post_landing: Option<PostLandingConfig>,
    #[serde(default)]
    phase_gates: PhaseGateConfig,
    #[serde(default)]
    ground_handling: GroundHandlingConfig,
    #[serde(default)]
    dead_stick: DeadStickConfig,
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
    #[serde(default = "default_runway_width_ft")]
    width_ft: f64,
    traffic_side: String,
}

/// FAA standard width for runways serving large aircraft; only used when
/// a runway definition omits its width.
pub const DEFAULT_RUNWAY_WIDTH_FT: f64 = 150.0;

fn default_runway_width_ft() -> f64 {
    DEFAULT_RUNWAY_WIDTH_FT
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
    if aircraft.flare.roundout_height_ft <= aircraft.flare.flare_start_ft {
        return Err(anyhow!(
            "roundout_height_ft ({}) must be greater than flare_start_ft ({})",
            aircraft.flare.roundout_height_ft,
            aircraft.flare.flare_start_ft
        ));
    }
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
    let post_landing = aircraft.post_landing.unwrap_or_default();

    let runway = Runway {
        id: Some(airport.runway.id.into_string()),
        threshold_ft: Vec2::new(airport.runway.threshold_x_ft, airport.runway.threshold_y_ft),
        course_deg: airport.runway.course_deg,
        length_ft: airport.runway.length_ft,
        touchdown_zone_ft: airport.runway.touchdown_zone_ft,
        // Config YAML doesn't model a displaced threshold — test configs
        // use the simple full-pavement runway. Real-runway displacement
        // comes in through `tools::runway_config` which reads it
        // from apt.dat.
        displaced_threshold_ft: 0.0,
        width_ft: airport.runway.width_ft,
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
        phase_gates: aircraft.phase_gates,
        ground_handling: aircraft.ground_handling,
        dead_stick: aircraft.dead_stick,
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
    fn rejects_collapsed_roundout_band() {
        let invalid = AIRCRAFT_YAML.replace("roundout_height_ft: 20", "roundout_height_ft: 10");
        let err = load_config_bundle_from(
            &invalid,
            AIRPORT_YAML,
            CONTROLLER_YAML,
            SAFETY_YAML,
        )
        .unwrap_err();
        assert!(err.to_string().contains("must be greater"));
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

/// Load `KEY=value` lines from a `.env` file into the environment
/// (best effort; variables already set win).
pub fn load_dotenv(path: &std::path::Path) {
    let Ok(text) = std::fs::read_to_string(path) else { return };
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some((key, value)) = line.split_once('=') {
            let (key, value) = (key.trim(), value.trim());
            if !key.is_empty() && std::env::var_os(key).is_none() {
                std::env::set_var(key, value);
            }
        }
    }
}
