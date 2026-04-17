//! X-Plane 12 web API bridge (REST + WebSocket). Mirrors sim/xplane_bridge.py.
//!
//! The high-level shape is the same: one-shot REST resolves dataref IDs via
//! `/api/capabilities` + `/datarefs?filter[name]=...`, then a WebSocket on
//! `/api/v3` streams `dataref_update_values` deltas which we merge into a
//! cache; writes go out as `dataref_set_values`. Runtime behavior that
//! requires a live X-Plane — the full `XPlaneWebBridge` struct — is behind
//! the `new` constructor and left as a thin wrapper; the pure helpers
//! (`geodetic_offset_ft`, `coerce_scalar`, `select_index`, flap conversions,
//! georef) are exported so tests and tools can use them directly.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use anyhow::{anyhow, bail, Context, Result};
use serde::Deserialize;
use serde_json::{json, Value};
use tungstenite::{client::IntoClientRequest, stream::MaybeTlsStream, Message, WebSocket};

use crate::sim::datarefs::{
    DatarefSpec, BOOTSTRAP_DATAREFS, COMMAND_DATAREFS, ELEVATION_M, FLAP_HANDLE_DEPLOY_RATIO,
    FLAP_HANDLE_REQUEST_RATIO, GEAR_HANDLE_DOWN, HEADING_DEG, IAS_KT, LATITUDE_DEG,
    LEFT_BRAKE_RATIO, LOCAL_VX_M_S, LOCAL_VZ_M_S, LONGITUDE_DEG, ON_GROUND_0,
    OVERRIDE_JOYSTICK_HEADING, PITCH_DEG, P_DEG_S, Q_DEG_S, RIGHT_BRAKE_RATIO, ROLL_DEG, R_DEG_S,
    SIM_TIME_S, STATE_DATAREFS, THROTTLE_ALL, VS_FPM, YOKE_HEADING_RATIO, YOKE_PITCH_RATIO,
    YOKE_ROLL_RATIO, Y_AGL_M,
};
use crate::sim::simple_dynamics::DynamicsState;
use crate::types::{clamp, ActuatorCommands, Vec2};

pub const M_TO_FT: f64 = 3.280_839_895_013_123;
pub const EARTH_RADIUS_FT: f64 = 20_925_524.9;
pub const DEFAULT_WEB_PORT: u16 = 8086;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoReference {
    pub threshold_lat_deg: f64,
    pub threshold_lon_deg: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionSample {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub altitude_msl_m: f64,
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub heading_deg: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BootstrapSample {
    pub posi: PositionSample,
    pub alt_agl_ft: f64,
    pub on_ground: bool,
}

pub fn geodetic_offset_ft(lat_deg: f64, lon_deg: f64, georef: GeoReference) -> Vec2 {
    let lat_delta_rad = (lat_deg - georef.threshold_lat_deg).to_radians();
    let lon_delta_rad = (lon_deg - georef.threshold_lon_deg).to_radians();
    let mean_lat_rad = ((lat_deg + georef.threshold_lat_deg) * 0.5).to_radians();
    let east_ft = EARTH_RADIUS_FT * lon_delta_rad * mean_lat_rad.cos();
    let north_ft = EARTH_RADIUS_FT * lat_delta_rad;
    Vec2::new(east_ft, north_ft)
}

pub fn coerce_scalar(value: &Value) -> f64 {
    match value {
        Value::Number(n) => n.as_f64().unwrap_or(0.0),
        Value::Bool(b) => {
            if *b { 1.0 } else { 0.0 }
        }
        Value::Array(a) => a
            .first()
            .and_then(|v| v.as_f64())
            .unwrap_or(0.0),
        _ => 0.0,
    }
}

pub fn select_index(value: &Value, index: Option<i32>) -> Result<f64> {
    if let Some(arr) = value.as_array() {
        let idx = index.unwrap_or(0);
        if idx < 0 || idx as usize >= arr.len() {
            bail!("index {} out of range for array of length {}", idx, arr.len());
        }
        return Ok(arr[idx as usize].as_f64().unwrap_or(0.0));
    }
    Ok(value.as_f64().unwrap_or(0.0))
}

pub fn flap_ratio_to_setting(ratio: f64) -> i32 {
    let settings = [0, 10, 20, 30];
    let target = ratio * 30.0;
    *settings
        .iter()
        .min_by(|a, b| {
            ((**a as f64) - target)
                .abs()
                .partial_cmp(&((**b as f64) - target).abs())
                .unwrap()
        })
        .unwrap()
}

pub fn flap_setting_to_ratio(setting: i32) -> f64 {
    clamp(setting as f64 / 30.0, 0.0, 1.0)
}

// -------- REST helpers --------

#[derive(Deserialize)]
struct CapabilitiesPayload {
    api: ApiCapabilities,
}
#[derive(Deserialize)]
struct ApiCapabilities {
    versions: Vec<String>,
}
#[derive(Deserialize)]
struct DatarefLookupPayload {
    data: Vec<DatarefLookupEntry>,
}
#[derive(Deserialize)]
struct DatarefLookupEntry {
    name: String,
    id: i64,
}

fn rest_base(host: &str, port: u16) -> String {
    format!("http://{}:{}/api/v3", host, port)
}

fn check_capabilities(host: &str, port: u16, timeout_secs: u64) -> Result<()> {
    let url = format!("http://{}:{}/api/capabilities", host, port);
    let resp = ureq::get(&url)
        .timeout(std::time::Duration::from_secs(timeout_secs))
        .call()
        .with_context(|| format!("reaching X-Plane web API at {}", url))?;
    let payload: CapabilitiesPayload = resp.into_json()?;
    if !payload.api.versions.iter().any(|v| v == "v3") {
        bail!("X-Plane web API does not advertise v3 support");
    }
    Ok(())
}

fn resolve_dataref_ids(
    host: &str,
    port: u16,
    specs: &[DatarefSpec],
    timeout_secs: u64,
) -> Result<HashMap<String, i64>> {
    let base = rest_base(host, port);
    let mut query = String::new();
    for (i, s) in specs.iter().enumerate() {
        if i > 0 {
            query.push('&');
        }
        query.push_str("filter[name]=");
        query.push_str(&url_encode(s.name));
    }
    let url = format!("{}/datarefs?{}", base, query);
    let payload: DatarefLookupPayload = ureq::get(&url)
        .timeout(std::time::Duration::from_secs(timeout_secs))
        .call()?
        .into_json()?;
    let mut resolved = HashMap::new();
    for entry in payload.data {
        resolved.insert(entry.name, entry.id);
    }
    for s in specs {
        if !resolved.contains_key(s.name) {
            bail!("X-Plane did not return id for dataref {}", s.name);
        }
    }
    Ok(resolved)
}

fn url_encode(s: &str) -> String {
    let mut out = String::new();
    for byte in s.bytes() {
        match byte {
            b'-' | b'_' | b'.' | b'~' | b'a'..=b'z' | b'A'..=b'Z' | b'0'..=b'9' => {
                out.push(byte as char)
            }
            b'/' => out.push_str("%2F"),
            b'[' => out.push_str("%5B"),
            b']' => out.push_str("%5D"),
            _ => out.push_str(&format!("%{:02X}", byte)),
        }
    }
    out
}

pub fn probe_bootstrap_sample(host: &str, port: u16, timeout_secs: u64) -> Result<BootstrapSample> {
    check_capabilities(host, port, timeout_secs)?;
    let resolved = resolve_dataref_ids(host, port, BOOTSTRAP_DATAREFS, timeout_secs)?;
    let mut values: HashMap<String, f64> = HashMap::new();
    for spec in BOOTSTRAP_DATAREFS {
        let id = resolved.get(spec.name).unwrap();
        let url = format!("{}/datarefs/{}/value", rest_base(host, port), id);
        let payload: Value = ureq::get(&url)
            .timeout(std::time::Duration::from_secs(timeout_secs))
            .call()?
            .into_json()?;
        let raw = payload.get("data").unwrap_or(&Value::Null);
        values.insert(spec.name.to_string(), select_index(raw, spec.index)?);
    }
    Ok(BootstrapSample {
        posi: PositionSample {
            lat_deg: values[LATITUDE_DEG.name],
            lon_deg: values[LONGITUDE_DEG.name],
            altitude_msl_m: values[ELEVATION_M.name],
            roll_deg: values[ROLL_DEG.name],
            pitch_deg: values[PITCH_DEG.name],
            heading_deg: values[HEADING_DEG.name],
        },
        alt_agl_ft: values[Y_AGL_M.name] * M_TO_FT,
        on_ground: values[ON_GROUND_0.name] >= 0.5,
    })
}

// -------- bridge --------

pub struct XPlaneWebBridge {
    pub georef: GeoReference,
    host: String,
    port: u16,
    read_ids: Arc<Mutex<HashMap<String, i64>>>,
    command_ids: Arc<Mutex<HashMap<String, i64>>>,
    cached_values: Arc<Mutex<HashMap<String, f64>>>,
    last_flap_ratio: Arc<Mutex<f64>>,
    last_gear_down: Arc<Mutex<bool>>,
    req_id: Arc<Mutex<i64>>,
    ws: Arc<Mutex<WebSocket<MaybeTlsStream<std::net::TcpStream>>>>,
}

impl XPlaneWebBridge {
    pub fn new(georef: GeoReference, host: &str, port: u16, timeout_secs: u64) -> Result<Self> {
        check_capabilities(host, port, timeout_secs)?;
        let mut specs_vec: Vec<DatarefSpec> = Vec::new();
        for s in STATE_DATAREFS.iter().chain(COMMAND_DATAREFS.iter()) {
            if !specs_vec.iter().any(|x| x.name == s.name) {
                specs_vec.push(*s);
            }
        }
        let resolved = resolve_dataref_ids(host, port, &specs_vec, timeout_secs)?;
        let mut read_ids = HashMap::new();
        for s in STATE_DATAREFS {
            read_ids.insert(s.name.to_string(), resolved[s.name]);
        }
        let mut command_ids = HashMap::new();
        for s in COMMAND_DATAREFS {
            command_ids.insert(s.name.to_string(), resolved[s.name]);
        }

        let url = format!("ws://{}:{}/api/v3", host, port);
        let request = url.into_client_request()?;
        let (ws, _) = tungstenite::connect(request)?;

        let bridge = Self {
            georef,
            host: host.to_string(),
            port,
            read_ids: Arc::new(Mutex::new(read_ids)),
            command_ids: Arc::new(Mutex::new(command_ids)),
            cached_values: Arc::new(Mutex::new(HashMap::new())),
            last_flap_ratio: Arc::new(Mutex::new(0.0)),
            last_gear_down: Arc::new(Mutex::new(true)),
            req_id: Arc::new(Mutex::new(0)),
            ws: Arc::new(Mutex::new(ws)),
        };
        bridge.subscribe_state()?;
        bridge.wait_for_initial_snapshot(timeout_secs)?;
        let override_id = bridge.command_ids.lock().unwrap()[OVERRIDE_JOYSTICK_HEADING.name];
        bridge.send_message(json!({
            "req_id": bridge.next_req_id(),
            "type": "dataref_set_values",
            "params": {
                "datarefs": [ { "id": override_id, "value": 1 } ]
            }
        }))?;
        Ok(bridge)
    }

    pub fn host(&self) -> &str { &self.host }
    pub fn port(&self) -> u16 { self.port }

    fn next_req_id(&self) -> i64 {
        let mut id = self.req_id.lock().unwrap();
        *id += 1;
        *id
    }

    fn send_message(&self, payload: Value) -> Result<()> {
        let mut ws = self.ws.lock().unwrap();
        ws.send(Message::Text(payload.to_string()))?;
        Ok(())
    }

    fn subscribe_state(&self) -> Result<()> {
        let read_ids = self.read_ids.lock().unwrap();
        let datarefs: Vec<Value> = STATE_DATAREFS
            .iter()
            .map(|s| {
                let mut entry = json!({ "id": read_ids[s.name] });
                if let Some(idx) = s.index {
                    entry["index"] = json!(idx);
                }
                entry
            })
            .collect();
        drop(read_ids);
        self.send_message(json!({
            "req_id": self.next_req_id(),
            "type": "dataref_subscribe_values",
            "params": { "datarefs": datarefs }
        }))
    }

    fn wait_for_initial_snapshot(&self, timeout_secs: u64) -> Result<()> {
        use std::time::{Duration, Instant};
        let deadline = Instant::now() + Duration::from_secs(timeout_secs);
        let needed: std::collections::BTreeSet<String> =
            STATE_DATAREFS.iter().map(|s| s.name.to_string()).collect();
        loop {
            if Instant::now() >= deadline {
                let missing: Vec<_> = {
                    let cache = self.cached_values.lock().unwrap();
                    needed.iter().filter(|n| !cache.contains_key(*n)).collect::<Vec<_>>()
                };
                bail!("timed out waiting for initial dataref snapshot; missing {:?}", missing);
            }
            {
                let mut ws = self.ws.lock().unwrap();
                set_stream_read_timeout(ws.get_mut(), Some(Duration::from_millis(200)))?;
                if let Ok(msg) = ws.read() {
                    drop(ws); // release lock before handling
                    self.handle_message(msg)?;
                }
            }
            if { self.cached_values.lock().unwrap().len() } >= needed.len() {
                let cache = self.cached_values.lock().unwrap();
                if needed.iter().all(|n| cache.contains_key(n)) {
                    return Ok(());
                }
            }
        }
    }

    fn drain_pending(&self) -> Result<()> {
        use std::time::Duration;
        loop {
            let msg_opt = {
                let mut ws = self.ws.lock().unwrap();
                set_stream_read_timeout(ws.get_mut(), Some(Duration::from_millis(1)))?;
                match ws.read() {
                    Ok(m) => Some(m),
                    Err(tungstenite::Error::Io(ref e)) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        None
                    }
                    Err(tungstenite::Error::Io(ref e)) if e.kind() == std::io::ErrorKind::TimedOut => {
                        None
                    }
                    Err(e) => return Err(anyhow!(e)),
                }
            };
            match msg_opt {
                Some(m) => self.handle_message(m)?,
                None => return Ok(()),
            }
        }
    }

    fn handle_message(&self, msg: Message) -> Result<()> {
        let text = match msg {
            Message::Text(t) => t,
            Message::Binary(b) => String::from_utf8_lossy(&b).into_owned(),
            _ => return Ok(()),
        };
        let value: Value = serde_json::from_str(&text)?;
        let msg_type = value.get("type").and_then(|v| v.as_str()).unwrap_or("");
        match msg_type {
            "dataref_update_values" => {
                let data = value.get("data").cloned().unwrap_or(Value::Null);
                if let Some(map) = data.as_object() {
                    let id_to_name: HashMap<String, String> = self
                        .read_ids
                        .lock()
                        .unwrap()
                        .iter()
                        .map(|(name, id)| (id.to_string(), name.clone()))
                        .collect();
                    let mut cache = self.cached_values.lock().unwrap();
                    for (raw_id, raw_value) in map {
                        if let Some(name) = id_to_name.get(raw_id) {
                            cache.insert(name.clone(), coerce_scalar(raw_value));
                        }
                    }
                }
            }
            "result" => {
                if !value.get("success").and_then(|v| v.as_bool()).unwrap_or(true) {
                    bail!("X-Plane web API returned failure: {}", value);
                }
            }
            _ => {}
        }
        Ok(())
    }

    pub fn read_state(&self) -> Result<DynamicsState> {
        self.drain_pending()?;
        let cache = self.cached_values.lock().unwrap();
        for spec in STATE_DATAREFS {
            if !cache.contains_key(spec.name) {
                bail!("missing dataref value after drain: {}", spec.name);
            }
        }
        let lat_deg = cache[LATITUDE_DEG.name];
        let lon_deg = cache[LONGITUDE_DEG.name];
        let altitude_m = cache[ELEVATION_M.name];
        let local_vx = cache[LOCAL_VX_M_S.name];
        let local_vz = cache[LOCAL_VZ_M_S.name];
        let vh_ind = cache[VS_FPM.name];
        let y_agl_m = cache[Y_AGL_M.name];
        let sim_time = cache[SIM_TIME_S.name];
        let on_ground = cache[ON_GROUND_0.name] >= 0.5 || y_agl_m <= 0.5;
        let throttle = clamp(cache[THROTTLE_ALL.name], 0.0, 1.0);
        let flap_ratio = clamp(cache[FLAP_HANDLE_DEPLOY_RATIO.name], 0.0, 1.0);
        let gear_down = cache[GEAR_HANDLE_DOWN.name] >= 0.5;
        drop(cache);
        *self.last_flap_ratio.lock().unwrap() = flap_ratio;
        *self.last_gear_down.lock().unwrap() = gear_down;

        let position_ft = geodetic_offset_ft(lat_deg, lon_deg, self.georef);
        let ground_vel = Vec2::new(local_vx * M_TO_FT, -local_vz * M_TO_FT);
        let cache = self.cached_values.lock().unwrap();
        Ok(DynamicsState {
            position_ft,
            altitude_ft: altitude_m * M_TO_FT,
            heading_deg: cache[HEADING_DEG.name],
            roll_deg: cache[ROLL_DEG.name],
            pitch_deg: cache[PITCH_DEG.name],
            ias_kt: cache[IAS_KT.name],
            throttle_pos: throttle,
            on_ground,
            time_s: sim_time,
            p_rad_s: cache[P_DEG_S.name].to_radians(),
            q_rad_s: cache[Q_DEG_S.name].to_radians(),
            r_rad_s: cache[R_DEG_S.name].to_radians(),
            ground_velocity_ft_s: ground_vel,
            vertical_speed_ft_s: vh_ind / 60.0,
            flap_index: flap_ratio_to_setting(flap_ratio),
            gear_down,
        })
    }

    pub fn write_commands(&self, commands: &ActuatorCommands) -> Result<()> {
        let last_flap = *self.last_flap_ratio.lock().unwrap();
        let flap_ratio = match commands.flaps {
            Some(f) => flap_setting_to_ratio(f),
            None => last_flap,
        };
        let last_gear = *self.last_gear_down.lock().unwrap();
        let gear_down = commands.gear_down.unwrap_or(last_gear);
        let brake = clamp(commands.brakes, 0.0, 1.0);
        // Differential braking: left = brakes − pivot, right = brakes +
        // pivot (each clamped to [0,1]). This keeps an asymmetry visible
        // even at full symmetric brake — at brakes=1.0 with pivot=+0.4
        // the right wheel saturates at 1.0 but the left drops to 0.6,
        // giving the differential torque needed to pivot-in-place at a
        // pose target. The older max-max formula saturated both wheels
        // and erased the pivot.
        let pivot = clamp(commands.pivot_brake, -1.0, 1.0);
        let left_brake = clamp(brake - pivot, 0.0, 1.0);
        let right_brake = clamp(brake + pivot, 0.0, 1.0);
        let cmd_ids = self.command_ids.lock().unwrap();
        let writes = vec![
            json!({ "id": cmd_ids[YOKE_PITCH_RATIO.name], "value": clamp(commands.elevator, -1.0, 1.0) }),
            json!({ "id": cmd_ids[YOKE_ROLL_RATIO.name], "value": clamp(commands.aileron, -1.0, 1.0) }),
            json!({ "id": cmd_ids[YOKE_HEADING_RATIO.name], "value": clamp(commands.rudder, -1.0, 1.0) }),
            json!({ "id": cmd_ids[THROTTLE_ALL.name], "value": clamp(commands.throttle, 0.0, 1.0) }),
            json!({ "id": cmd_ids[GEAR_HANDLE_DOWN.name], "value": if gear_down { 1.0 } else { 0.0 } }),
            json!({ "id": cmd_ids[FLAP_HANDLE_REQUEST_RATIO.name], "value": clamp(flap_ratio, 0.0, 1.0) }),
            json!({ "id": cmd_ids[LEFT_BRAKE_RATIO.name], "value": left_brake }),
            json!({ "id": cmd_ids[RIGHT_BRAKE_RATIO.name], "value": right_brake }),
        ];
        drop(cmd_ids);
        self.send_message(json!({
            "req_id": self.next_req_id(),
            "type": "dataref_set_values",
            "params": { "datarefs": writes }
        }))
    }

    pub fn get_dataref_value(&self, name: &str) -> Option<f64> {
        self.cached_values.lock().unwrap().get(name).copied()
    }

    pub fn write_dataref_values(&self, updates: &[(String, f64)]) -> Result<()> {
        if updates.is_empty() {
            return Ok(());
        }
        let mut unresolved: Vec<DatarefSpec> = Vec::new();
        {
            let cmd = self.command_ids.lock().unwrap();
            let read = self.read_ids.lock().unwrap();
            for (name, _) in updates {
                if !cmd.contains_key(name) && !read.contains_key(name) {
                    unresolved.push(DatarefSpec { name: leak_str(name), index: None });
                }
            }
        }
        if !unresolved.is_empty() {
            let resolved = resolve_dataref_ids(&self.host, self.port, &unresolved, 5)?;
            let mut cmd = self.command_ids.lock().unwrap();
            for (name, id) in resolved {
                cmd.insert(name, id);
            }
        }
        let mut writes: Vec<Value> = Vec::new();
        {
            let cmd = self.command_ids.lock().unwrap();
            let read = self.read_ids.lock().unwrap();
            for (name, value) in updates {
                let id = cmd
                    .get(name)
                    .or_else(|| read.get(name))
                    .ok_or_else(|| anyhow!("failed to resolve dataref id for {}", name))?;
                writes.push(json!({ "id": id, "value": value }));
            }
        }
        self.send_message(json!({
            "req_id": self.next_req_id(),
            "type": "dataref_set_values",
            "params": { "datarefs": writes }
        }))
    }

    pub fn close(&self) {
        let override_id = self.command_ids.lock().unwrap()[OVERRIDE_JOYSTICK_HEADING.name];
        let _ = self.send_message(json!({
            "req_id": self.next_req_id(),
            "type": "dataref_set_values",
            "params": {
                "datarefs": [ { "id": override_id, "value": 0 } ]
            }
        }));
        let _ = self.ws.lock().unwrap().close(None);
    }
}

impl crate::llm::tools::ToolBridge for XPlaneWebBridge {
    fn georef(&self) -> GeoReference { self.georef }
    fn get_dataref_value(&self, name: &str) -> Option<f64> {
        self.get_dataref_value(name)
    }
    fn write_dataref_values(&self, updates: &[(String, f64)]) -> anyhow::Result<()> {
        self.write_dataref_values(updates)
    }
}

fn leak_str(s: &str) -> &'static str {
    Box::leak(s.to_string().into_boxed_str())
}

fn set_stream_read_timeout(
    stream: &mut MaybeTlsStream<std::net::TcpStream>,
    timeout: Option<std::time::Duration>,
) -> Result<()> {
    match stream {
        MaybeTlsStream::Plain(s) => s.set_read_timeout(timeout)?,
        _ => {}
    }
    Ok(())
}
