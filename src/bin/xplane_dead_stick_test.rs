//! Live X-Plane equivalent of `tests/test_dead_stick_scenario.rs`.
//!
//! Drives the dead-stick controller against X-Plane 12's flight model
//! with no LLM in the loop. Spawns the aircraft via the X-Plane REST
//! API at a configurable position, looks the runway up in the apt.dat
//! parquet cache, installs it into a bare `PilotCore`, engages
//! `DeadStickLandingProfile`, and runs the same control loop the live
//! pilot uses (read state → update → write actuators) until touchdown
//! or timeout. Prints a trajectory table and asserts the touchdown
//! falls inside the runway box.
//!
//! Default scenario mirrors the deterministic
//! `dead_stick_engaged_on_wrong_side_of_runway` test: KWHP rwy 30,
//! aircraft spawned ~10 kft right of the centerline (the wrong side
//! for a left-traffic pattern), 3000 ft above field, heading roughly
//! along the runway course.
//!
//! Usage (from repo root, with X-Plane 12 running):
//!     cargo run --release --bin xplane-dead-stick-test
//!     cargo run --release --bin xplane-dead-stick-test -- --airport KWHP --runway 30

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context, Result};
use clap::Parser;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::dead_stick_profile::DeadStickLandingProfile;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::data::apt_dat;
use xplane_pilot::guidance::runway_geometry::RunwayFrame;
use xplane_pilot::sim::xplane_bridge::{
    activate_command, geodetic_offset_ft, latlon_from_offset_ft, post_air_start, GeoReference,
    XPlaneWebBridge, M_TO_FT,
};
use xplane_pilot::types::{
    AircraftState, FlightPhase, Runway, TrafficSide, Vec2, KT_TO_FPS,
};

const M_PER_FT: f64 = 1.0 / M_TO_FT;
const MPS_PER_KT: f64 = KT_TO_FPS / M_TO_FT;

#[derive(Debug, Parser)]
#[command(about = "End-to-end dead-stick test against live X-Plane (no LLM)")]
struct Args {
    #[arg(long, default_value = "127.0.0.1")]
    xplane_host: String,
    #[arg(long, default_value_t = 8086)]
    xplane_port: u16,

    /// Airport ICAO ident (default: KWHP).
    #[arg(long, default_value = "KWHP")]
    airport: String,
    /// Runway ident (default: 30).
    #[arg(long, default_value = "30")]
    runway: String,
    /// Pattern traffic side: "left" or "right".
    #[arg(long, default_value = "left")]
    side: String,

    /// Spawn offset, ft along runway course (positive = past threshold
    /// in departure direction). Defaults match the wrong-side-of-runway
    /// deterministic test: 600 ft upwind of threshold.
    #[arg(long, default_value_t = 600.0)]
    spawn_x_ft: f64,

    /// Spawn offset, ft perpendicular to course (positive = right of
    /// course). The wrong-side test uses +10 000 ft for a left-traffic
    /// runway (pattern is at -1500 ft now that the dead-stick offset
    /// is tightened).
    #[arg(long, default_value_t = 10_000.0)]
    spawn_y_ft: f64,

    /// Spawn altitude AGL above field elevation (ft).
    #[arg(long, default_value_t = 3000.0)]
    spawn_alt_agl_ft: f64,

    /// Spawn IAS (kt). Needs to be airborne-realistic for a Cessna
    /// 172 — defaults to vbg.
    #[arg(long, default_value_t = 68.0)]
    spawn_ias_kt: f64,

    /// Spawn heading offset (deg) relative to the runway course.
    /// Default 0 means the aircraft starts pointing along the runway
    /// (which is north-ish for a course of 318° at KWHP rwy 30).
    #[arg(long, default_value_t = 0.0)]
    spawn_heading_offset_deg: f64,

    /// Maximum sim seconds to wait for touchdown.
    #[arg(long, default_value_t = 360.0)]
    max_time_s: f64,

    /// Control loop tick rate (Hz).
    #[arg(long, default_value_t = 10.0)]
    control_hz: f64,

    /// Telemetry print interval (seconds).
    #[arg(long, default_value_t = 3.0)]
    print_interval_s: f64,

    /// Path to apt.dat. Auto-detects an X-Plane 12 install if omitted.
    #[arg(long)]
    apt_dat_path: Option<PathBuf>,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let traffic_side = TrafficSide::from_str(&args.side)
        .ok_or_else(|| anyhow!("--side must be 'left' or 'right' (got {:?})", args.side))?;

    // Resolve apt.dat parquet cache and look the runway up.
    let apt_dat_path = args
        .apt_dat_path
        .clone()
        .or_else(apt_dat::default_apt_dat_path)
        .ok_or_else(|| {
            anyhow!(
                "no apt.dat path provided and no default X-Plane 12 install detected; \
                 pass --apt-dat-path"
            )
        })?;
    let cache = xplane_pilot::data::parquet::resolve(&apt_dat_path, None)
        .context("resolving apt.dat parquet cache")?;
    let runway_query = lookup_runway(&cache.runways(), &args.airport, &args.runway)?;
    println!(
        "[setup] resolved {}/{} -> threshold lat={:.6} lon={:.6} course={:.2}°T length={:.0} ft elev={:.0} ft",
        args.airport,
        args.runway,
        runway_query.threshold_lat,
        runway_query.threshold_lon,
        runway_query.course_deg,
        runway_query.length_ft,
        runway_query.field_elev_ft,
    );

    let georef = GeoReference {
        threshold_lat_deg: runway_query.threshold_lat,
        threshold_lon_deg: runway_query.threshold_lon,
    };
    let runway_threshold_ft =
        geodetic_offset_ft(runway_query.threshold_lat, runway_query.threshold_lon, georef);
    let mut config = load_default_config_bundle();
    let runway = Runway {
        id: Some(args.runway.clone()),
        threshold_ft: runway_threshold_ft,
        course_deg: runway_query.course_deg,
        length_ft: runway_query.length_ft,
        touchdown_zone_ft: 2000.0,
        displaced_threshold_ft: runway_query.displaced_ft,
        traffic_side,
    };
    config.airport.airport = Some(args.airport.clone());
    config.airport.field_elevation_ft = runway_query.field_elev_ft;
    config.airport.runway = runway.clone();
    let runway_frame = RunwayFrame::new(runway);

    // Spawn position: runway-frame (spawn_x_ft, spawn_y_ft) → world
    // ft → (lat, lon) via the same flat-earth geodetic helpers the
    // bridge uses, so the spawned aircraft lands exactly where the
    // runway-frame coordinates say it should.
    let spawn_world_ft = runway_frame.to_world_frame(Vec2::new(args.spawn_x_ft, args.spawn_y_ft));
    let (spawn_lat, spawn_lon) = latlon_from_offset_ft(spawn_world_ft - runway_threshold_ft, georef);
    let spawn_msl_ft = runway_query.field_elev_ft + args.spawn_alt_agl_ft;
    let spawn_heading = (runway_query.course_deg + args.spawn_heading_offset_deg).rem_euclid(360.0);
    println!(
        "[setup] spawning aircraft at lat={:.6} lon={:.6} {:.0} ft MSL ({:.0} AGL), heading {:.1}°, IAS {:.1} kt",
        spawn_lat, spawn_lon, spawn_msl_ft, args.spawn_alt_agl_ft, spawn_heading, args.spawn_ias_kt,
    );
    post_air_start(
        &args.xplane_host,
        args.xplane_port,
        spawn_lat,
        spawn_lon,
        spawn_msl_ft * M_PER_FT,
        spawn_heading,
        args.spawn_ias_kt * MPS_PER_KT,
    )
    .context("posting flight init to X-Plane")?;
    thread::sleep(Duration::from_secs(6));
    if let Err(e) = activate_command(&args.xplane_host, args.xplane_port, "sim/operation/pause_off")
    {
        eprintln!("[setup] warn: pause_off command failed: {e}");
    }

    let bridge = Arc::new(
        XPlaneWebBridge::new(georef, &args.xplane_host, args.xplane_port, 5)
            .context("opening X-Plane web bridge")?,
    );
    let mut pilot = PilotCore::new(config.clone());
    pilot.runway_frame = runway_frame.clone();

    // Take one read so the dead-stick profile gets a real state for
    // its `decide_entry_phase` call.
    let raw_state = bridge.read_state().context("initial bridge read")?;
    let dt = 1.0 / args.control_hz.max(1.0);
    let (estimated, _commands) = pilot.update(&raw_state, dt);
    println!(
        "[setup] initial state: pos=({:.0},{:.0}) ft, alt_msl={:.0}ft alt_agl={:.0}ft hdg={:.1}° ias={:.1}",
        estimated.position_ft.x,
        estimated.position_ft.y,
        estimated.alt_msl_ft,
        estimated.alt_agl_ft,
        estimated.heading_deg,
        estimated.ias_kt,
    );

    let profile = DeadStickLandingProfile::new(config.clone(), runway_frame.clone(), &estimated);
    pilot.engage_profile(Box::new(profile));
    println!("[setup] dead_stick_landing engaged; entry phase {:?}", pilot.phase());

    // Stop on Ctrl-C so we don't leave the sim driving on a half-run.
    let stop = Arc::new(AtomicBool::new(false));
    let stop_for_ctrlc = stop.clone();
    ctrlc::set_handler(move || stop_for_ctrlc.store(true, Ordering::Release))
        .context("installing Ctrl-C handler")?;

    // Control loop.
    let target_period = Duration::from_secs_f64(1.0 / args.control_hz.max(1.0));
    let print_period = Duration::from_secs_f64(args.print_interval_s.max(0.1));
    let mut last_state_time_s: Option<f64> = None;
    let mut next_print = Instant::now();
    let start_sim_t: f64 = raw_state.time_s;
    let mut touchdown_state: Option<AircraftState> = None;
    let mut prev_airborne = !raw_state.on_ground;
    let mut max_pre_final_ias_kt = 0.0_f64;

    println!();
    println!(
        "{:>6} {:>5} {:>5} {:>4} {:>6} {:>5} {:>5} {:>5} {:>4} {:>13} phase",
        "t", "MSL", "AGL", "IAS", "VS", "hdg", "pitch", "roll", "thr", "rfx,rfy"
    );

    while !stop.load(Ordering::Acquire) {
        let loop_start = Instant::now();
        let raw_state = match bridge.read_state() {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[bridge] read error: {e}");
                thread::sleep(target_period);
                continue;
            }
        };
        let dt = match last_state_time_s {
            Some(prev) => (raw_state.time_s - prev).clamp(0.02, 0.5),
            None => target_period.as_secs_f64(),
        };
        last_state_time_s = Some(raw_state.time_s);
        let (estimated, commands) = pilot.update(&raw_state, dt);

        // Track max IAS in pre-Final phases for the speed-discipline
        // assertion.
        let phase = pilot.phase();
        if matches!(
            phase,
            FlightPhase::Descent
                | FlightPhase::PatternEntry
                | FlightPhase::Downwind
                | FlightPhase::Base
        ) {
            max_pre_final_ias_kt = max_pre_final_ias_kt.max(estimated.ias_kt);
        }

        if let Err(e) = bridge.write_commands(&commands) {
            eprintln!("[bridge] write error: {e}");
        }

        if Instant::now() >= next_print {
            let rf_pt = runway_frame.to_runway_frame(estimated.position_ft);
            println!(
                "{:>6.1} {:>5.0} {:>5.0} {:>4.0} {:>6.0} {:>5.0} {:>5.1} {:>5.1} {:>4.2} {:>6.0},{:>6.0} {:?}",
                raw_state.time_s - start_sim_t,
                estimated.alt_msl_ft,
                estimated.alt_agl_ft,
                estimated.ias_kt,
                estimated.vs_fpm,
                estimated.heading_deg,
                estimated.pitch_deg,
                estimated.roll_deg,
                commands.throttle,
                rf_pt.x,
                rf_pt.y,
                phase,
            );
            next_print = Instant::now() + print_period;
        }

        // Touchdown detection: previously airborne, now on ground.
        if prev_airborne && raw_state.on_ground && touchdown_state.is_none() {
            touchdown_state = Some(estimated.clone());
            println!("[touchdown] detected — sim t={:.1}", raw_state.time_s - start_sim_t);
        }
        prev_airborne = !raw_state.on_ground;

        // Stop once we're stopped on the ground past the runway end OR
        // out of forward speed after touchdown.
        if let Some(_td) = &touchdown_state {
            if raw_state.ground_velocity_ft_s.length() < 2.0 {
                println!("[run] aircraft stopped on the ground; ending run.");
                break;
            }
        }

        if (raw_state.time_s - start_sim_t) >= args.max_time_s {
            println!("[run] max_time_s reached");
            break;
        }

        let elapsed = loop_start.elapsed();
        if elapsed < target_period {
            thread::sleep(target_period - elapsed);
        }
    }

    // Report + assert.
    println!();
    let Some(td) = touchdown_state else {
        return Err(anyhow!(
            "no touchdown observed within {:.0}s — likely crashed off airport or never reached the ground",
            args.max_time_s
        ));
    };
    let td_rf = runway_frame.to_runway_frame(td.position_ft);
    println!(
        "[result] touchdown runway-frame x={:.0} ft, y={:.0} ft (runway length {:.0} ft)",
        td_rf.x, td_rf.y, runway_query.length_ft
    );
    println!(
        "[result] max pre-final IAS = {:.1} kt (target vbg {:.1})",
        max_pre_final_ias_kt, config.performance.vbg_kt
    );

    let mut failures: Vec<String> = Vec::new();
    if !(0.0..=runway_query.length_ft).contains(&td_rf.x) {
        failures.push(format!(
            "touchdown x={:.0} ft outside runway [0, {:.0}]",
            td_rf.x, runway_query.length_ft
        ));
    }
    if td_rf.y.abs() > 75.0 {
        failures.push(format!(
            "touchdown {:.0} ft off centerline (allowed ±75 ft)",
            td_rf.y
        ));
    }
    if max_pre_final_ias_kt > config.performance.vbg_kt + 8.0 {
        failures.push(format!(
            "pre-final IAS reached {:.1} (vbg {:.1} + 8 kt budget)",
            max_pre_final_ias_kt, config.performance.vbg_kt
        ));
    }

    if failures.is_empty() {
        println!("[PASS] dead-stick to {}/{} succeeded", args.airport, args.runway);
        Ok(())
    } else {
        for f in &failures {
            println!("[FAIL] {f}");
        }
        Err(anyhow!("{} assertion(s) failed", failures.len()))
    }
}

struct RunwayQuery {
    threshold_lat: f64,
    threshold_lon: f64,
    course_deg: f64,
    length_ft: f64,
    field_elev_ft: f64,
    displaced_ft: f64,
}

fn lookup_runway(
    runways_path: &std::path::Path,
    airport: &str,
    runway_ident: &str,
) -> Result<RunwayQuery> {
    let conn = duckdb::Connection::open_in_memory()?;
    conn.execute_batch("INSTALL spatial; LOAD spatial;")?;
    let mut stmt = conn.prepare(&format!(
        "SELECT le_ident, he_ident, \
                le_latitude_deg, le_longitude_deg, le_heading_degT, le_elevation_ft, le_displaced_threshold_ft, \
                he_latitude_deg, he_longitude_deg, he_heading_degT, he_elevation_ft, he_displaced_threshold_ft, \
                length_ft \
         FROM read_parquet('{path}') \
         WHERE airport_ident = ? AND (le_ident = ? OR he_ident = ?) AND closed = 0 \
         LIMIT 1",
        path = runways_path.display()
    ))?;
    let mut rows = stmt.query([airport, runway_ident, runway_ident])?;
    let row = rows
        .next()?
        .ok_or_else(|| anyhow!("runway {} at {} not found in parquet", runway_ident, airport))?;
    let le_ident: Option<String> = row.get(0)?;
    let he_ident: Option<String> = row.get(1)?;
    let le_lat: Option<f64> = row.get(2)?;
    let le_lon: Option<f64> = row.get(3)?;
    let le_hdg: Option<f64> = row.get(4)?;
    let le_elev: Option<f64> = row.get(5)?;
    let le_disp: Option<f64> = row.get(6)?;
    let he_lat: Option<f64> = row.get(7)?;
    let he_lon: Option<f64> = row.get(8)?;
    let he_hdg: Option<f64> = row.get(9)?;
    let he_elev: Option<f64> = row.get(10)?;
    let he_disp: Option<f64> = row.get(11)?;
    let length_ft: Option<f64> = row.get(12)?;

    let (lat, lon, hdg, elev, disp) = if Some(runway_ident.to_string()) == le_ident {
        (le_lat, le_lon, le_hdg, le_elev, le_disp)
    } else if Some(runway_ident.to_string()) == he_ident {
        (he_lat, he_lon, he_hdg, he_elev, he_disp)
    } else {
        return Err(anyhow!("runway lookup mismatch for {}", runway_ident));
    };
    Ok(RunwayQuery {
        threshold_lat: lat.ok_or_else(|| anyhow!("missing threshold lat"))?,
        threshold_lon: lon.ok_or_else(|| anyhow!("missing threshold lon"))?,
        course_deg: hdg.ok_or_else(|| anyhow!("missing course"))?,
        length_ft: length_ft.unwrap_or(5000.0),
        field_elev_ft: elev.unwrap_or(0.0),
        displaced_ft: disp.unwrap_or(0.0),
    })
}

