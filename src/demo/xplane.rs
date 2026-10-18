//! Setting X-Plane up for a take: where the script's start puts the
//! aircraft (from the runway in apt.dat), the flight-init request, and a
//! worker that fires X-Plane commands (cameras, pause) off the control
//! loop.

use std::path::Path;
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
use crate::llm::tools::{open_apt_dat_parquet, query_runway_end, RunwayEnd};
use crate::sim::xplane_bridge::{
    activate_command_for, geodetic_offset_ft, latlon_from_offset_ft, post_flight_init, probe_bootstrap_sample,
    BootstrapSample, GeoReference,
};
use crate::types::{Runway, TrafficSide, Vec2, NM_TO_FT};

const C172_PATH: &str = "Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP.acf";
const KT_TO_MPS: f64 = 0.514_444;
const FT_TO_M: f64 = 0.3048;

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
pub fn start_fix(script: &DemoScript, apt_dat_cache_dir: &Path) -> Result<StartFix> {
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
    let frame = RunwayFrame::new(Runway {
        id: Some(a.runway.clone()),
        threshold_ft: Vec2::ZERO,
        course_deg: end.course_deg,
        length_ft: end.length_ft,
        touchdown_zone_ft: crate::plan::compile::TOUCHDOWN_ZONE_FT,
        displaced_threshold_ft: end.displaced_threshold_ft,
        width_ft: end.width_ft,
        traffic_side: if a.right_ft >= 0.0 { TrafficSide::Right } else { TrafficSide::Left },
    });
    let (lat_deg, lon_deg) = latlon_from_offset_ft(frame.to_world_frame(Vec2::new(a.along_ft, a.right_ft)), anchor);
    let alt_msl_ft = a
        .alt_msl_ft
        .or(a.alt_agl_ft.map(|h| end.elevation_ft + h))
        .unwrap_or(end.elevation_ft + 1000.0);
    Ok(StartFix { lat_deg, lon_deg, alt_msl_ft, heading_deg: a.heading_deg, ias_kt: a.ias_kt, runway: end })
}

/// The `/api/v3/flight` body for the take.
pub fn flight_init_body(script: &DemoScript, fix: &StartFix) -> Value {
    let x = &script.xplane;
    let mut data = json!({
        "aircraft": {"path": x.aircraft.clone().unwrap_or_else(|| C172_PATH.to_string())},
        "lle_air_start": {
            "latitude": fix.lat_deg,
            "longitude": fix.lon_deg,
            "elevation_in_meters": fix.alt_msl_ft * FT_TO_M,
            "heading_true": fix.heading_deg,
            "speed_in_meters_per_second": fix.ias_kt * KT_TO_MPS,
            "pitch_in_degrees": 0.0
        },
        "engine_status": {"all_engines": {"running": true}}
    });
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

/// Fires X-Plane commands on its own thread, so a slow REST call never
/// holds up the control loop.
#[derive(Clone)]
pub struct XpCommander {
    tx: Sender<(String, f64)>,
}

impl XpCommander {
    pub fn spawn(host: String, port: u16, bus: SimBus) -> Self {
        let (tx, rx) = unbounded::<(String, f64)>();
        thread::Builder::new()
            .name("demo-xplane".into())
            .spawn(move || {
                for (name, duration_s) in rx {
                    if let Err(e) = activate_command_for(&host, port, &name, duration_s) {
                        bus.push_log_kind(LogKind::Error, format!("[demo] X-Plane command {name}: {e}"));
                    }
                }
            })
            .expect("spawn demo-xplane");
        Self { tx }
    }

    pub fn command(&self, name: &str, duration_s: f64) {
        let _ = self.tx.send((name.to_string(), duration_s));
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
