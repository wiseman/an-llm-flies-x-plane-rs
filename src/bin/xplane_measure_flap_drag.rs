//! Measure how flap deflection changes engine-off glide performance
//! in X-Plane's flight model. Uses our own `PitchForAirspeedController`
//! (the same controller the dead-stick profile uses) to hold vbg while
//! cycling flap settings, and reads sink rate / IAS to back out the
//! drag effect of each flap deflection.
//!
//! Output is the calibration data needed to add a flap-drag term to
//! `simple_dynamics.rs` so the deterministic dead-stick tests reflect
//! how dirty/clean configuration changes the descent rate.

use std::sync::Arc;
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::control::bank_hold::BankController;
use xplane_pilot::control::pitch_for_airspeed::PitchForAirspeedController;
use xplane_pilot::control::pitch_hold::PitchController;
use xplane_pilot::sim::xplane_bridge::{
    activate_command, post_air_start, GeoReference, XPlaneWebBridge, M_TO_FT,
};
use xplane_pilot::types::{clamp, ActuatorCommands, KT_TO_FPS};

const M_PER_FT: f64 = 1.0 / M_TO_FT;
const MPS_PER_KT: f64 = KT_TO_FPS / M_TO_FT;

fn main() -> Result<()> {
    let host = "127.0.0.1";
    let port: u16 = 8086;

    // Sweep across (target_ias, flap) so we can separate parasitic
    // drag (rises with IAS) from induced drag (rises as IAS drops).
    // Including both vbg and vref lets us see the U-shape.
    let cases: &[(f64, i32)] = &[
        (61.0, 30), // vref, full flaps — Final approach config
        (68.0, 30), // vbg, full flaps
        (61.0, 0),  // vref, clean
        (68.0, 0),  // vbg, clean — sanity check vs published 9:1
    ];
    let mut results = Vec::new();
    for (target_ias, flap) in cases {
        let r = measure_glide(host, port, *flap, *target_ias)?;
        results.push((*target_ias, *flap, r));
    }

    println!();
    println!("=== drag calibration ===");
    println!(
        "{:>7} {:>5} {:>7} {:>6} {:>7} {:>7} {:>5}",
        "tgt_kt", "flap°", "IAS_kt", "GS_kt", "VS_fpm", "pitch°", "L/D"
    );
    for (target, flap, r) in &results {
        println!(
            "{:7.1} {:5} {:7.1} {:6.1} {:7.0} {:7.1} {:5.1}",
            target, flap, r.ias, r.gs, r.vs, r.pitch, r.glide_ratio
        );
    }
    Ok(())
}

#[derive(Debug)]
struct GlideMeasurement {
    ias: f64,
    gs: f64,
    vs: f64,
    pitch: f64,
    glide_ratio: f64,
}

fn measure_glide(
    host: &str,
    port: u16,
    flap_deg: i32,
    target_ias_kt: f64,
) -> Result<GlideMeasurement> {
    println!();
    println!("--- target IAS = {:.1} kt, flap = {}° ---", target_ias_kt, flap_deg);
    // Spawn over flat ocean (y_agl is meaningful; no terrain interference).
    let lat = 34.0_f64;
    let lon = -119.5_f64;
    let elev_m = 8000.0 * M_PER_FT;
    post_air_start(host, port, lat, lon, elev_m, 270.0, 68.0 * MPS_PER_KT)?;
    thread::sleep(Duration::from_secs(6));
    // X-Plane sometimes ignores the first unpause if it lands mid-load.
    for _ in 0..3 {
        activate_command(host, port, "sim/operation/pause_off")?;
        thread::sleep(Duration::from_millis(500));
    }
    thread::sleep(Duration::from_secs(2));

    let georef = GeoReference {
        threshold_lat_deg: lat,
        threshold_lon_deg: lon,
    };
    let bridge = Arc::new(XPlaneWebBridge::new(georef, host, port, 5).context("bridge open")?);
    let sanity = bridge.read_state().context("first sanity read")?;
    println!(
        "  sanity: t_sim={:.2} ias={:.1} vs_fps={:.1}",
        sanity.time_s, sanity.ias_kt, sanity.vertical_speed_ft_s
    );

    bridge.write_dataref_values(&[
        ("sim/cockpit2/engine/actuators/mixture_ratio_all".into(), 0.0),
        ("sim/cockpit2/engine/actuators/throttle_ratio_all".into(), 0.0),
        (
            "sim/cockpit2/controls/flap_handle_request_ratio".into(),
            flap_deg as f64 / 30.0,
        ),
    ])?;

    let cfg = load_default_config_bundle();
    let mut pitch_for_ias = PitchForAirspeedController::new();
    // Test-rig overrides: production limits would saturate at high flap
    // drag, so we'd measure "drag at whatever speed the saturation
    // settled at" instead of drag at the target IAS.
    pitch_for_ias.pitch_limit = (-60.0, 15.0);
    pitch_for_ias.integrator_limit = 200.0;
    pitch_for_ias.kp = 0.8;
    let mut pitch_ctrl = PitchController::new(cfg.controllers.pitch);
    let mut bank_ctrl = BankController::new(cfg.controllers.bank);
    let target_ias = target_ias_kt;

    // Settle: hold vbg + wings-level + zero throttle for ~50 s so the
    // glide stabilizes after spawn / flap change. The flapped-glide
    // equilibrium IAS is well off the spawn IAS (68 kt), so the
    // pitch-for-airspeed loop needs significant time to converge.
    let settle_s = 50.0;
    let sample_s = 25.0;
    let dt = 0.1;
    let mut t = 0.0;
    let mut ias_samples = Vec::new();
    let mut gs_samples = Vec::new();
    let mut vs_samples = Vec::new();
    let mut pitch_samples = Vec::new();
    while t < settle_s + sample_s {
        let state = bridge.read_state()?;
        let pitch_cmd_deg =
            pitch_for_ias.update(target_ias, state.ias_kt, dt);
        let elevator =
            pitch_ctrl.update(pitch_cmd_deg, state.pitch_deg, state.q_rad_s, dt);
        let aileron =
            bank_ctrl.update(0.0, state.roll_deg, state.p_rad_s, dt);
        let cmds = ActuatorCommands {
            aileron: clamp(aileron, -1.0, 1.0),
            elevator: clamp(elevator, -1.0, 1.0),
            rudder: 0.0,
            throttle: 0.0,
            flaps: Some(flap_deg),
            gear_down: Some(true),
            brakes: 0.0,
            pivot_brake: 0.0,
        };
        bridge.write_commands(&cmds)?;

        if t >= settle_s {
            let ias = state.ias_kt;
            let gs = state.ground_velocity_ft_s.length() / KT_TO_FPS;
            let vs = state.vertical_speed_ft_s * 60.0; // ft/s -> fpm
            let pitch = state.pitch_deg;
            ias_samples.push(ias);
            gs_samples.push(gs);
            vs_samples.push(vs);
            pitch_samples.push(pitch);
        }

        thread::sleep(Duration::from_secs_f64(dt));
        t += dt;
    }

    let mean = |v: &[f64]| v.iter().copied().sum::<f64>() / v.len().max(1) as f64;
    let ias = mean(&ias_samples);
    let gs = mean(&gs_samples);
    let vs = mean(&vs_samples);
    let pitch = mean(&pitch_samples);
    let sink_fps = (-vs / 60.0).max(0.1);
    let fwd_fps = gs * KT_TO_FPS;
    let glide_ratio = fwd_fps / sink_fps;
    println!(
        "tgt={:5.1}  flap={:3}°  IAS={:5.1}  GS={:5.1}  VS={:6.0}  pitch={:5.1}  L/D≈{:4.1}",
        target_ias_kt, flap_deg, ias, gs, vs, pitch, glide_ratio
    );
    Ok(GlideMeasurement {
        ias,
        gs,
        vs,
        pitch,
        glide_ratio,
    })
}
