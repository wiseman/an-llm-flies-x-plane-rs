//! Demo-video helpers:
//!
//!   demo audition "Skyhawk 2SP, extend downwind" [--speaker pilot] [--demo demo/x.yaml]
//!       Render one line through the radio voice and play it.
//!   demo check demo/kvny-16r.yaml
//!       Resolve every cue, render the ATC lines, and check OBS scenes
//!       and X-Plane command names.
//!   demo discover [sim/view sim/replay ...]
//!       List X-Plane commands (and datarefs) under those prefixes.
//!   demo broll --take output/sim_pilot-STEM --demo demo/x.yaml [--pass cut]
//!       With X-Plane replaying the take: record it again with the pass's
//!       camera cuts, timed to the take.
//!   demo mixdown --take output/sim_pilot-STEM --demo demo/x.yaml
//!       Rebuild the radio track (ATC and pilot) from the take's timeline.
//!
//! A take itself runs in sim-pilot: `scripts/demo/take.sh demo/x.yaml`.

use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

use anyhow::{anyhow, Context, Result};
use clap::{Parser, Subcommand, ValueEnum};
use parking_lot::Mutex as PLMutex;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::data::{apt_dat, parquet as data_parquet};
use xplane_pilot::demo::obs::ObsClient;
use xplane_pilot::demo::phraseology::spoken_form;
use xplane_pilot::demo::radio_voice::{play, Voices};
use xplane_pilot::demo::script::{AudioSetup, DemoScript, TtsBackend};
use xplane_pilot::demo::xplane::start_fix;
use xplane_pilot::eval_scenario::MessageSchedule;
use xplane_pilot::llm::tools::{DuckAirports, ToolContext};
use xplane_pilot::plan::gazetteer::GAZETTEER;
use xplane_pilot::plan::resolve::Resolver;
use xplane_pilot::sim::xplane_bridge::GeoReference;

#[derive(Parser)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
    #[arg(long, global = true, default_value = "127.0.0.1")]
    xplane_host: String,
    #[arg(long, global = true, default_value_t = 8086)]
    xplane_port: u16,
}

#[derive(Clone, Copy, ValueEnum)]
enum Who {
    Atc,
    Pilot,
}

#[derive(Subcommand)]
enum Cmd {
    /// Speak a line through the radio voice.
    Audition {
        text: String,
        #[arg(long, value_enum, default_value_t = Who::Atc)]
        speaker: Who,
        /// Use this script's voices (default: Kokoro).
        #[arg(long)]
        demo: Option<PathBuf>,
        /// Override the backend: elevenlabs, kokoro or say.
        #[arg(long)]
        backend: Option<String>,
        /// Override the voice (ElevenLabs voice id, Kokoro or `say` name).
        #[arg(long)]
        voice: Option<String>,
        /// Speak the text as written, not in radio form.
        #[arg(long)]
        raw: bool,
    },
    /// Check a script before a take.
    Check { demo: PathBuf },
    /// List X-Plane commands and datarefs under these prefixes.
    Discover {
        #[arg(default_values_t = ["sim/view".to_string(), "sim/replay".to_string()])]
        prefixes: Vec<String>,
    },
    /// Record a B-roll pass over X-Plane's replay of a take.
    Broll {
        /// The take's files without extension (output/sim_pilot-STEM).
        #[arg(long)]
        take: PathBuf,
        #[arg(long)]
        demo: PathBuf,
        /// Pass name (default: the script's first).
        #[arg(long)]
        pass: Option<String>,
        /// Leave OBS alone (record by hand).
        #[arg(long)]
        no_obs: bool,
    },
    /// Rebuild the take's radio audio as one WAV aligned to the recording.
    Mixdown {
        #[arg(long)]
        take: PathBuf,
        #[arg(long)]
        demo: PathBuf,
        /// Output WAV (default: <take>.radio.wav).
        #[arg(long)]
        out: Option<PathBuf>,
    },
}

fn main() -> Result<()> {
    load_dotenv(Path::new(".env"));
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Audition { text, speaker, demo, backend, voice, raw } => {
            let audio = match demo {
                Some(p) => DemoScript::load(&p)?.audio,
                None => AudioSetup::default(),
            };
            let mut v = match speaker {
                Who::Atc => audio.atc,
                Who::Pilot => audio.pilot,
            };
            if let Some(b) = backend {
                v.backend = match b.as_str() {
                    "elevenlabs" => TtsBackend::Elevenlabs,
                    "kokoro" => TtsBackend::Kokoro,
                    "say" => TtsBackend::Say,
                    other => return Err(anyhow!("backend {other:?}: elevenlabs, kokoro or say")),
                };
            }
            if let Some(name) = voice {
                v.voice = name;
            }
            let spoken = if raw { text } else { spoken_form(&text) };
            println!("say: {spoken}");
            let t = Instant::now();
            let clip = Voices::default().render(&v, &spoken)?;
            println!("{} ({:?} {}) in {:.2} s", clip.display(), v.backend, v.voice, t.elapsed().as_secs_f64());
            play(&clip)?;
        }
        Cmd::Check { demo } => check(&demo, &cli.xplane_host, cli.xplane_port)?,
        Cmd::Discover { prefixes } => discover(&cli.xplane_host, cli.xplane_port, &prefixes)?,
        Cmd::Broll { take, demo, pass, no_obs } => broll(&take, &demo, pass.as_deref(), !no_obs, &cli.xplane_host, cli.xplane_port)?,
        Cmd::Mixdown { take, demo, out } => mixdown(&take, &demo, out)?,
    }
    Ok(())
}

fn check(path: &Path, host: &str, port: u16) -> Result<()> {
    let script = DemoScript::load(path)?;
    println!("{}: {} cues", script.id, script.cues.len());

    // Cues resolve against the real apt.dat and place list.
    let cache = resolve_apt_dat_cache()?;
    let fix = start_fix(&script, &cache)?;
    println!("start: {:.5},{:.5} {:.0} ft heading {:.0}", fix.lat_deg, fix.lon_deg, fix.alt_msl_ft, fix.heading_deg);
    let cfg = load_default_config_bundle();
    let mut ctx = ToolContext::new(Arc::new(PLMutex::new(PilotCore::new(cfg.clone()))), cfg);
    ctx.apt_dat_cache_dir = Some(cache);
    let airports = DuckAirports { ctx: &ctx };
    let georef = GeoReference { threshold_lat_deg: fix.lat_deg, threshold_lon_deg: fix.lon_deg };
    let mut r = Resolver::new(georef, &GAZETTEER, &airports);
    MessageSchedule::with_order(&script.messages(), &mut r, script.sequential).context("resolving cues")?;
    println!("cues: ok");

    // ATC audio, rendered into the cache.
    let mut voices = Voices::default();
    for c in &script.cues {
        if let Some(text) = &c.msg.atc {
            let spoken = c.act.say.clone().unwrap_or_else(|| spoken_form(text));
            let t = Instant::now();
            match voices.render(&script.audio.atc, &spoken) {
                Ok(p) => println!("audio {:.2}s {} <- {spoken}", t.elapsed().as_secs_f64(), p.display()),
                Err(e) => println!("audio FAILED for {spoken:?}: {e:#}"),
            }
        }
    }

    // OBS scenes.
    let url = std::env::var("OBS_WS_URL").unwrap_or_else(|_| "ws://127.0.0.1:4455".into());
    match ObsClient::connect(&url, std::env::var("OBS_WS_PASSWORD").ok().as_deref()) {
        Ok(mut obs) => {
            let scenes = obs.scene_names()?;
            for (alias, name) in &script.obs.scenes {
                let ok = scenes.contains(name);
                println!("obs scene {alias} -> {name:?}: {}", if ok { "ok" } else { "MISSING" });
            }
        }
        Err(e) => println!("obs: not reachable ({e:#})"),
    }

    // X-Plane commands the script uses.
    let mut names: Vec<String> = Vec::new();
    names.extend(script.xplane.initial_camera.clone());
    let acts = script.cues.iter().map(|c| &c.act).chain(script.hooks.on_fly.iter()).chain(script.hooks.on_touchdown.iter());
    for a in acts {
        names.extend(a.camera.clone());
        names.extend(a.xplane_commands.iter().map(|c| c.name.clone()));
    }
    for p in &script.broll {
        for s in &p.shots {
            names.push(s.camera.clone());
            names.extend(s.commands.iter().map(|c| c.name.clone()));
        }
    }
    names.extend(["sim/operation/pause_on".to_string(), "sim/operation/pause_off".to_string()]);
    names.sort();
    names.dedup();
    match xplane_list(host, port, "commands") {
        Ok(all) => {
            for n in &names {
                println!("xplane command {n}: {}", if all.contains(n) { "ok" } else { "MISSING" });
            }
        }
        Err(e) => println!("xplane: not reachable ({e:#})"),
    }
    Ok(())
}

fn discover(host: &str, port: u16, prefixes: &[String]) -> Result<()> {
    for kind in ["commands", "datarefs"] {
        let all = xplane_list(host, port, kind)?;
        for name in all.iter().filter(|n| prefixes.iter().any(|p| n.starts_with(p.as_str()))) {
            println!("{kind:8} {name}");
        }
    }
    Ok(())
}

/// Every command or dataref name X-Plane knows.
fn xplane_list(host: &str, port: u16, kind: &str) -> Result<Vec<String>> {
    let url = format!("http://{host}:{port}/api/v3/{kind}");
    let v: serde_json::Value = ureq::get(&url).call().map_err(|e| anyhow!("{url}: {e}"))?.into_json()?;
    Ok(v["data"].as_array().into_iter().flatten().filter_map(|c| c["name"].as_str().map(String::from)).collect())
}

/// One line of a take's `.demo.jsonl` timeline.
struct TimelineEvent {
    wall: chrono::DateTime<chrono::Utc>,
    kind: String,
    v: serde_json::Value,
}

fn read_timeline(take: &Path) -> Result<Vec<TimelineEvent>> {
    let path = take.with_extension("demo.jsonl");
    let text = fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
    let mut out = Vec::new();
    for line in text.lines().filter(|l| !l.trim().is_empty()) {
        let v: serde_json::Value = serde_json::from_str(line)?;
        let wall = v["wall_utc"].as_str().and_then(|w| chrono::DateTime::parse_from_rfc3339(w).ok()).map(|w| w.with_timezone(&chrono::Utc));
        if let Some(wall) = wall {
            out.push(TimelineEvent { wall, kind: v["kind"].as_str().unwrap_or("").to_string(), v });
        }
    }
    Ok(out)
}

/// The take's 1 Hz track: (wall time, lat, lon).
fn read_track(take: &Path) -> Result<Vec<(chrono::DateTime<chrono::Utc>, f64, f64)>> {
    let path = take.with_extension("csv");
    let text = fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
    let mut out = Vec::new();
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 4 {
            continue;
        }
        let (Ok(wall), Ok(lat), Ok(lon)) = (chrono::DateTime::parse_from_rfc3339(f[0]), f[2].parse::<f64>(), f[3].parse::<f64>()) else {
            continue;
        };
        out.push((wall.with_timezone(&chrono::Utc), lat, lon));
    }
    if out.is_empty() {
        return Err(anyhow!("no track points in {}", path.display()));
    }
    Ok(out)
}

/// When a B-roll shot happens in the take, by wall clock.
fn shot_time(at: &str, timeline: &[TimelineEvent], track_start: chrono::DateTime<chrono::Utc>) -> Option<chrono::DateTime<chrono::Utc>> {
    let find = |kind: &str, cue: Option<&str>| {
        timeline
            .iter()
            .find(|e| e.kind == kind && cue.is_none_or(|c| e.v["cue"].as_str() == Some(c)))
            .map(|e| e.wall)
    };
    if at == "start" {
        return find("roll", None).or(Some(track_start));
    }
    if let Some(id) = at.strip_prefix("cue:") {
        return find("cue", Some(id));
    }
    at.strip_prefix("event:").and_then(|ev| find(ev, None))
}

fn dataref_id(host: &str, port: u16, name: &str) -> Result<i64> {
    let url = format!("http://{host}:{port}/api/v3/datarefs");
    let v: serde_json::Value = ureq::get(&url).query("filter[name]", name).call().map_err(|e| anyhow!("{e}"))?.into_json()?;
    v["data"][0]["id"].as_i64().ok_or_else(|| anyhow!("no dataref {name}"))
}

fn dataref_value(host: &str, port: u16, id: i64) -> Result<f64> {
    let url = format!("http://{host}:{port}/api/v3/datarefs/{id}/value");
    let v: serde_json::Value = ureq::get(&url).call().map_err(|e| anyhow!("{e}"))?.into_json()?;
    v["data"].as_f64().ok_or_else(|| anyhow!("dataref {id} has no number"))
}

/// Re-record a take from X-Plane's replay with a pass's camera cuts.
/// Where the replay is in the take comes from matching the aircraft's
/// position to the take's track, so it doesn't depend on how replay
/// keeps time.
fn broll(take: &Path, demo: &Path, pass: Option<&str>, use_obs: bool, host: &str, port: u16) -> Result<()> {
    use xplane_pilot::sim::xplane_bridge::{activate_command_for, geodetic_offset_ft};
    let script = DemoScript::load(demo)?;
    let pass = match pass {
        Some(name) => script.broll.iter().find(|p| p.name == name).ok_or_else(|| anyhow!("no B-roll pass {name:?}"))?,
        None => script.broll.first().ok_or_else(|| anyhow!("the script has no broll passes"))?,
    };
    let timeline = read_timeline(take)?;
    let track = read_track(take)?;
    let mut shots: Vec<(chrono::DateTime<chrono::Utc>, &xplane_pilot::demo::script::BrollShot)> = pass
        .shots
        .iter()
        .filter_map(|s| {
            let at = shot_time(&s.at, &timeline, track[0].0)?;
            Some((at + chrono::Duration::milliseconds((s.offset_s * 1000.0) as i64), s))
        })
        .collect();
    shots.sort_by_key(|(t, _)| *t);
    println!("pass {}: {} shots", pass.name, shots.len());
    println!("Put X-Plane in replay at the start of the take, paused. Press Enter, then play at 1x.");
    let mut line = String::new();
    std::io::stdin().read_line(&mut line)?;

    let mut obs = if use_obs {
        let url = std::env::var("OBS_WS_URL").unwrap_or_else(|_| "ws://127.0.0.1:4455".into());
        match ObsClient::connect(&url, std::env::var("OBS_WS_PASSWORD").ok().as_deref()) {
            Ok(mut c) => {
                if let Some(scene) = script.obs.scenes.get("xplane") {
                    let _ = c.request("SetCurrentProgramScene", serde_json::json!({"sceneName": scene}));
                }
                c.request("StartRecord", serde_json::json!({}))?;
                Some(c)
            }
            Err(e) => {
                println!("OBS not reachable ({e:#}); record by hand");
                None
            }
        }
    } else {
        None
    };
    let (lat_id, lon_id) = (
        dataref_id(host, port, "sim/flightmodel/position/latitude")?,
        dataref_id(host, port, "sim/flightmodel/position/longitude")?,
    );
    let mut fired = 0;
    let mut idx = 0usize;
    let mut log = Vec::new();
    let last = track.len() - 1;
    let started = Instant::now();
    loop {
        let (lat, lon) = (dataref_value(host, port, lat_id)?, dataref_value(host, port, lon_id)?);
        let here = GeoReference { threshold_lat_deg: lat, threshold_lon_deg: lon };
        // Nearest track point, moving forward from the last match.
        let window = idx..(idx + 30).min(last + 1);
        idx = window
            .min_by(|&a, &b| {
                let da = geodetic_offset_ft(track[a].1, track[a].2, here).length();
                let db = geodetic_offset_ft(track[b].1, track[b].2, here).length();
                da.total_cmp(&db)
            })
            .unwrap_or(idx);
        let tau = track[idx].0;
        log.push(serde_json::json!({"wall_utc": chrono::Utc::now().to_rfc3339(), "take_wall_utc": tau.to_rfc3339()}));
        while fired < shots.len() && shots[fired].0 <= tau {
            let s = shots[fired].1;
            println!("{:6.1}s  {}", started.elapsed().as_secs_f64(), s.camera);
            activate_command_for(host, port, &s.camera, 0.0)?;
            for c in &s.commands {
                activate_command_for(host, port, &c.name, c.duration_s)?;
            }
            fired += 1;
        }
        if idx >= last {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(200));
    }
    if let Some(c) = obs.as_mut() {
        let _ = c.request("StopRecord", serde_json::json!({}));
    }
    let out = take.with_extension(format!("broll.{}.json", pass.name));
    fs::write(&out, serde_json::to_string_pretty(&serde_json::json!({"pass": pass.name, "sync": log}))?)?;
    println!("done; replay-to-take sync in {}", out.display());
    Ok(())
}

/// The take's radio as one WAV: each transmission's clip (from the TTS
/// cache) placed where it started, measured from the recording's start
/// (or the roll).
fn mixdown(take: &Path, demo: &Path, out: Option<PathBuf>) -> Result<()> {
    let script = DemoScript::load(demo)?;
    let timeline = read_timeline(take)?;
    let t0 = timeline
        .iter()
        .find(|e| e.kind == "record" && e.v["action"] == "start")
        .or_else(|| timeline.iter().find(|e| e.kind == "roll"))
        .map(|e| e.wall)
        .ok_or_else(|| anyhow!("the timeline has no record start or roll"))?;
    let mut voices = Voices::default();
    let mut clips: Vec<(PathBuf, i64)> = Vec::new();
    for e in &timeline {
        let voice = match e.kind.as_str() {
            "atc_audio_start" => &script.audio.atc,
            "pilot_audio_start" => &script.audio.pilot,
            _ => continue,
        };
        let spoken = e.v["spoken"].as_str().unwrap_or_default();
        let ms = (e.wall - t0).num_milliseconds();
        if ms < 0 || spoken.is_empty() {
            continue;
        }
        clips.push((voices.render(voice, spoken)?, ms));
    }
    if clips.is_empty() {
        return Err(anyhow!("no transmissions in the timeline"));
    }
    let out = out.unwrap_or_else(|| take.with_extension("radio.wav"));
    let mut cmd = std::process::Command::new("ffmpeg");
    cmd.args(["-hide_banner", "-loglevel", "error", "-y"]);
    for (p, _) in &clips {
        cmd.arg("-i").arg(p);
    }
    let mut graph = String::new();
    for (i, (_, ms)) in clips.iter().enumerate() {
        graph.push_str(&format!("[{i}:a]adelay={ms}:all=1[d{i}];"));
    }
    for i in 0..clips.len() {
        graph.push_str(&format!("[d{i}]"));
    }
    graph.push_str(&format!("amix=inputs={}:normalize=0:dropout_transition=0", clips.len()));
    cmd.args(["-filter_complex", &graph]).arg(&out);
    if !cmd.status()?.success() {
        return Err(anyhow!("ffmpeg mixdown failed"));
    }
    println!("{} transmissions -> {}", clips.len(), out.display());
    Ok(())
}

fn resolve_apt_dat_cache() -> Result<PathBuf> {
    let apt = apt_dat::default_apt_dat_path().ok_or_else(|| anyhow!("apt.dat not found (X-Plane 12 at the standard location)"))?;
    let airspace = xplane_pilot::data::airspace::default_airspace_txt_path().filter(|p| p.exists());
    Ok(data_parquet::resolve(&apt, airspace.as_deref())?.dir)
}

fn load_dotenv(path: &Path) {
    let Ok(text) = fs::read_to_string(path) else { return };
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some((k, v)) = line.split_once('=') {
            let (k, v) = (k.trim(), v.trim());
            if !k.is_empty() && std::env::var_os(k).is_none() {
                std::env::set_var(k, v);
            }
        }
    }
}
