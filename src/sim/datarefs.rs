//! X-Plane dataref names resolved by the live bridge. Mirrors
//! sim_pilot/sim/datarefs.py.

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DatarefSpec {
    pub name: &'static str,
    pub index: Option<i32>,
}

impl DatarefSpec {
    pub const fn new(name: &'static str) -> Self {
        Self { name, index: None }
    }
    pub const fn at(name: &'static str, index: i32) -> Self {
        Self { name, index: Some(index) }
    }
}

pub const LATITUDE_DEG: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/latitude");
pub const LONGITUDE_DEG: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/longitude");
pub const ELEVATION_M: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/elevation");
pub const PITCH_DEG: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/theta");
pub const ROLL_DEG: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/phi");
pub const HEADING_DEG: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/psi");
pub const IAS_KT: DatarefSpec = DatarefSpec::new("sim/cockpit2/gauges/indicators/airspeed_kts_pilot");
pub const LOCAL_VX_M_S: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/local_vx");
pub const LOCAL_VZ_M_S: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/local_vz");
pub const VS_FPM: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/vh_ind_fpm");
pub const P_DEG_S: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/P");
pub const Q_DEG_S: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/Q");
pub const R_DEG_S: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/R");
pub const THROTTLE_ALL: DatarefSpec = DatarefSpec::new("sim/cockpit2/engine/actuators/throttle_ratio_all");
pub const FLAP_HANDLE_DEPLOY_RATIO: DatarefSpec = DatarefSpec::new("sim/cockpit2/controls/flap_handle_deploy_ratio");
pub const FLAP_HANDLE_REQUEST_RATIO: DatarefSpec = DatarefSpec::new("sim/cockpit2/controls/flap_handle_request_ratio");
pub const GEAR_HANDLE_DOWN: DatarefSpec = DatarefSpec::new("sim/cockpit2/controls/gear_handle_down");
pub const Y_AGL_M: DatarefSpec = DatarefSpec::new("sim/flightmodel/position/y_agl");
pub const SIM_TIME_S: DatarefSpec = DatarefSpec::new("sim/time/total_running_time_sec");
pub const ON_GROUND_0: DatarefSpec = DatarefSpec::at("sim/flightmodel2/gear/on_ground", 0);
pub const HAS_CRASHED: DatarefSpec = DatarefSpec::new("sim/flightmodel2/misc/has_crashed");

pub const YOKE_PITCH_RATIO: DatarefSpec = DatarefSpec::new("sim/joystick/yoke_pitch_ratio");
pub const YOKE_ROLL_RATIO: DatarefSpec = DatarefSpec::new("sim/joystick/yoke_roll_ratio");
pub const YOKE_HEADING_RATIO: DatarefSpec = DatarefSpec::new("sim/joystick/yoke_heading_ratio");
pub const LEFT_BRAKE_RATIO: DatarefSpec = DatarefSpec::new("sim/cockpit2/controls/left_brake_ratio");
pub const RIGHT_BRAKE_RATIO: DatarefSpec = DatarefSpec::new("sim/cockpit2/controls/right_brake_ratio");
pub const PARKING_BRAKE_RATIO: DatarefSpec = DatarefSpec::new("sim/cockpit2/controls/parking_brake_ratio");

/// Required override: writes to yoke_heading_ratio don't reach the rudder /
/// nosewheel unless this override is set. See the Python docstring for the
/// investigation notes.
pub const OVERRIDE_JOYSTICK_HEADING: DatarefSpec =
    DatarefSpec::new("sim/operation/override/override_joystick_heading");

/// Required override for writes to left_brake_ratio / right_brake_ratio to
/// take full effect. Without it X-Plane blends our writes with the pilot's
/// (un-pressed) toe-brake joystick input, capping effective deceleration at
/// ~15% of max-brake authority. Observed in live KWHP landings.
pub const OVERRIDE_TOE_BRAKES: DatarefSpec =
    DatarefSpec::new("sim/operation/override/override_toe_brakes");

pub const COM1_FREQUENCY_HZ_833: DatarefSpec =
    DatarefSpec::new("sim/cockpit2/radios/actuators/com1_frequency_hz_833");
pub const COM2_FREQUENCY_HZ_833: DatarefSpec =
    DatarefSpec::new("sim/cockpit2/radios/actuators/com2_frequency_hz_833");

pub const STATE_DATAREFS: &[DatarefSpec] = &[
    LATITUDE_DEG,
    LONGITUDE_DEG,
    ELEVATION_M,
    PITCH_DEG,
    ROLL_DEG,
    HEADING_DEG,
    IAS_KT,
    LOCAL_VX_M_S,
    LOCAL_VZ_M_S,
    VS_FPM,
    P_DEG_S,
    Q_DEG_S,
    R_DEG_S,
    THROTTLE_ALL,
    FLAP_HANDLE_DEPLOY_RATIO,
    GEAR_HANDLE_DOWN,
    Y_AGL_M,
    SIM_TIME_S,
    ON_GROUND_0,
    HAS_CRASHED,
    PARKING_BRAKE_RATIO,
    COM1_FREQUENCY_HZ_833,
    COM2_FREQUENCY_HZ_833,
];

pub const COMMAND_DATAREFS: &[DatarefSpec] = &[
    YOKE_PITCH_RATIO,
    YOKE_ROLL_RATIO,
    YOKE_HEADING_RATIO,
    THROTTLE_ALL,
    GEAR_HANDLE_DOWN,
    FLAP_HANDLE_REQUEST_RATIO,
    LEFT_BRAKE_RATIO,
    RIGHT_BRAKE_RATIO,
    OVERRIDE_JOYSTICK_HEADING,
    OVERRIDE_TOE_BRAKES,
];

pub const BOOTSTRAP_DATAREFS: &[DatarefSpec] = &[
    LATITUDE_DEG,
    LONGITUDE_DEG,
    ELEVATION_M,
    PITCH_DEG,
    ROLL_DEG,
    HEADING_DEG,
    Y_AGL_M,
    ON_GROUND_0,
];
