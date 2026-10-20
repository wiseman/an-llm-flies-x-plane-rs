//! Setting X-Plane up for a take: where the script's start puts the
//! aircraft (from the runway in apt.dat), the flight-init request, and a
//! worker that fires X-Plane commands (cameras, pause) off the control
//! loop.

use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Result};
use crossbeam_channel::{unbounded, Sender};
use serde_json::{json, Value};

use crate::bus::{LogKind, SimBus};
use crate::data::parquet::{AIRPORTS_FILE, COMMS_FILE, RUNWAYS_FILE};
use crate::demo::script::DemoScript;
use crate::eval_scenario::Start;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::llm::tools::{open_apt_dat_parquet, query_runway_end, RunwayEnd, ToolBridge};
use crate::sim::xplane_bridge::{
    activate_command_id, air_start_data, command_id, geodetic_offset_ft, latlon_from_offset_ft, post_flight_init,
    probe_bootstrap_sample, BootstrapSample, GeoReference, C172_PATH,
};
use crate::types::{Vec2, KT_TO_FPS, M_TO_FT, NM_TO_FT};


/// Where a take starts.
#[derive(Debug, Clone)]
pub struct StartFix {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_ft: f64,
    pub heading_deg: f64,
    pub ias_kt: f64,
    pub runway: RunwayEnd,
}

/// The script's airborne start as a lat/lon, from its runway in apt.dat.
pub fn start_fix(script: &DemoScript, apt_dat_cache_dir: &Path, pattern_agl_ft: f64) -> Result<StartFix> {
    let Start::Airborne { airborne: a } = &script.start else {
        bail!("a demo starts airborne (start: {{airborne: ...}})");
    };
    let conn = open_apt_dat_parquet(
        &apt_dat_cache_dir.join(AIRPORTS_FILE),
        &apt_dat_cache_dir.join(RUNWAYS_FILE),
        &apt_dat_cache_dir.join(COMMS_FILE),
    )?;
    let end = query_runway_end(&conn, &script.airport, &a.runway)?;
    let anchor = GeoReference { threshold_lat_deg: end.threshold_lat_deg, threshold_lon_deg: end.threshold_lon_deg };
    let frame = RunwayFrame::new(end.runway(&a.runway, Vec2::ZERO, a.side()));
    let (lat_deg, lon_deg) = latlon_from_offset_ft(frame.to_world_frame(Vec2::new(a.along_ft, a.right_ft)), anchor);
    let alt_msl_ft = a.start_alt_msl_ft(end.elevation_ft, pattern_agl_ft);
    Ok(StartFix { lat_deg, lon_deg, alt_msl_ft, heading_deg: a.heading_deg, ias_kt: a.ias_kt, runway: end })
}

/// The `/api/v3/flight` body for the take.
pub fn flight_init_body(script: &DemoScript, fix: &StartFix) -> Value {
    let x = &script.xplane;
    let mut data = air_start_data(
        x.aircraft.as_deref().unwrap_or(C172_PATH),
        fix.lat_deg,
        fix.lon_deg,
        fix.alt_msl_ft / M_TO_FT,
        fix.heading_deg,
        fix.ias_kt * KT_TO_FPS / M_TO_FT,
    );
    // X-Plane wants both fields of a local time; a missing one is today
    // or noon.
    if x.day_of_year.is_some() || x.time_in_24_hours.is_some() {
        use chrono::Datelike;
        let day = x.day_of_year.unwrap_or_else(|| chrono::Local::now().ordinal0());
        let hour = x.time_in_24_hours.unwrap_or(12.0);
        data["local_time"] = json!({"day_of_year": day, "time_in_24_hours": hour});
    }
    if let Some(w) = &x.weather {
        data["weather"] = w.clone();
    }
    json!({"data": data})
}

/// Start the flight and wait until X-Plane has the aircraft in the air
/// near the start (it takes a few seconds to load).
pub fn init_flight(host: &str, port: u16, script: &DemoScript, fix: &StartFix) -> Result<BootstrapSample> {
    post_flight_init(host, port, &flight_init_body(script, fix))?;
    let target = GeoReference { threshold_lat_deg: fix.lat_deg, threshold_lon_deg: fix.lon_deg };
    let deadline = Instant::now() + Duration::from_secs(90);
    let mut last = String::from("no sample");
    while Instant::now() < deadline {
        thread::sleep(Duration::from_secs(1));
        match probe_bootstrap_sample(host, port, 5) {
            Ok(s) => {
                let off = geodetic_offset_ft(s.posi.lat_deg, s.posi.lon_deg, target).length();
                if !s.on_ground && off < 0.3 * NM_TO_FT {
                    return Ok(s);
                }
                last = format!("{:.2} nm from the start, on ground {}", off / NM_TO_FT, s.on_ground);
            }
            Err(e) => last = e.to_string(),
        }
    }
    Err(anyhow!("X-Plane didn't put the aircraft at the start within 90 s ({last})"))
}

enum XpWork {
    Command(String, f64),
    Datarefs(Vec<(String, f64)>),
}

/// Fires X-Plane commands (ids cached) and writes datarefs on its own
/// thread, so a slow REST call never holds up the control loop.
#[derive(Clone)]
pub struct XpCommander {
    tx: Sender<XpWork>,
}

impl XpCommander {
    pub fn spawn(host: String, port: u16, bridge: Option<Arc<dyn ToolBridge>>, bus: SimBus) -> Self {
        let (tx, rx) = unbounded::<XpWork>();
        thread::Builder::new()
            .name("demo-xplane".into())
            .spawn(move || {
                let mut ids: HashMap<String, i64> = HashMap::new();
                for work in rx {
                    let (what, result) = match work {
                        XpWork::Command(name, duration_s) => {
                            let id = match ids.get(&name) {
                                Some(id) => Ok(*id),
                                None => command_id(&host, port, &name).inspect(|id| {
                                    ids.insert(name.clone(), *id);
                                }),
                            };
                            (name, id.and_then(|id| activate_command_id(&host, port, id, duration_s)))
                        }
                        XpWork::Datarefs(values) => match &bridge {
                            Some(b) => ("datarefs".to_string(), b.write_dataref_values(&values)),
                            None => continue,
                        },
                    };
                    if let Err(e) = result {
                        bus.push_log_kind(LogKind::Error, format!("[demo] X-Plane {what}: {e:#}"));
                    }
                }
            })
            .expect("spawn demo-xplane");
        Self { tx }
    }

    pub fn command(&self, name: &str, duration_s: f64) {
        let _ = self.tx.send(XpWork::Command(name.to_string(), duration_s));
    }

    pub fn write_datarefs(&self, values: Vec<(String, f64)>) {
        let _ = self.tx.send(XpWork::Datarefs(values));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_flight_init_body_carries_the_start_time_and_aircraft() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("demo/kvny-16r.yaml");
        let script = DemoScript::load(&path).unwrap();
        let fix = StartFix {
            lat_deg: 34.2,
            lon_deg: -118.5,
            alt_msl_ft: 1800.0,
            heading_deg: 355.5,
            ias_kt: 80.0,
            runway: RunwayEnd {
                threshold_lat_deg: 34.22,
                threshold_lon_deg: -118.49,
                course_deg: 175.5,
                length_ft: 8000.0,
                width_ft: 150.0,
                elevation_ft: 800.0,
                displaced_threshold_ft: 0.0,
            },
        };
        let b = flight_init_body(&script, &fix);
        let d = &b["data"];
        assert_eq!(d["aircraft"]["path"], C172_PATH);
        assert!((d["lle_air_start"]["elevation_in_meters"].as_f64().unwrap() - 548.64).abs() < 0.01);
        assert!((d["lle_air_start"]["speed_in_meters_per_second"].as_f64().unwrap() - 41.16).abs() < 0.01);
        assert_eq!(d["local_time"]["time_in_24_hours"], 16.5);
    }
}
