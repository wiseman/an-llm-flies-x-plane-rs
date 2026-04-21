//! `sim-pilot` CLI. Mirrors sim_pilot/__main__.py.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{anyhow, Context, Result};
use chrono::Local;
use clap::{Parser, ValueEnum};

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::data::{apt_dat, parquet as data_parquet};
use xplane_pilot::live_runner::{run_live_xplane, LiveRunConfig};
use xplane_pilot::llm::conversation::PilotMode;
use xplane_pilot::sim::{
    logging::write_scenario_log_csv, plotting::write_scenario_plots, scenario::ScenarioRunner,
};
use xplane_pilot::types::Vec2;

#[derive(Parser, Debug)]
#[command(about = "Deterministic offline sim / live X-Plane MVP bridge")]
struct Cli {
    #[arg(long, value_enum, default_value_t = Backend::Xplane)]
    backend: Backend,

    #[arg(long)]
    scenario_name: Option<String>,

    #[arg(long, default_value_t = 0.0)]
    crosswind_kt: f64,

    #[arg(long, default_value_t = 0.0)]
    wind_x_kt: f64,

    #[arg(long, default_value_t = 0.0)]
    wind_y_kt: f64,

    #[arg(long)]
    log_csv: Option<PathBuf>,

    #[arg(long)]
    plots_dir: Option<PathBuf>,

    #[arg(long, default_value = "127.0.0.1")]
    xplane_host: String,

    #[arg(long, default_value_t = 8086)]
    xplane_port: u16,

    #[arg(long = "pilot-llm-model", default_value = "gpt-5.4-2026-03-05")]
    pilot_llm_model: String,

    /// Pilot persona that selects which system prompt the LLM runs with at
    /// startup. Can be swapped at runtime via the TUI `/mode <name>` command.
    /// `normal` is the default operational prompt; `realistic` layers on
    /// real-world ATC phraseology, readbacks, and procedural discipline.
    #[arg(long = "pilot-mode", value_enum, default_value_t = CliPilotMode::Normal)]
    pilot_mode: CliPilotMode,

    /// Optional reasoning effort hint passed to the pilot LLM (e.g. "low",
    /// "medium", "high"). When unset, no reasoning field is sent and the
    /// model uses its provider default.
    #[arg(long = "pilot-llm-reasoning-effort")]
    pilot_llm_reasoning_effort: Option<String>,

    /// ATC/operator message sent through the LLM at startup. Repeat for multiple messages.
    #[arg(long = "atc-message")]
    atc_message: Vec<String>,

    /// Skip the TUI. The session runs headless, logging to stdout and
    /// reading any startup messages from `--atc-message`. Useful for
    /// scripted or non-interactive runs.
    #[arg(long)]
    headless: bool,

    #[arg(long, default_value_t = 10.0)]
    control_hz: f64,

    #[arg(long, default_value_t = 2.0)]
    status_interval_s: f64,

    /// Guidance profile to engage at startup: "idle" or "pattern_fly".
    #[arg(long, default_value = "idle")]
    engage_profile: String,

    /// Path to an X-Plane 12 apt.dat (typically under Global Scenery/Global
    /// Airports/Earth nav data/). When set — or when the default X-Plane 12
    /// install is detected — the file is parsed into a per-user zstd parquet
    /// cache under ~/.cache/sim_pilot/apt-<hash>/, transparently rebuilt
    /// whenever apt.dat's mtime advances. The cache backs the LLM's
    /// sql_query tool.
    #[arg(long)]
    apt_dat_path: Option<PathBuf>,

    /// Path to an X-Plane 12 airspace.txt (typically under Resources/default
    /// data/airspaces/). When set — or when the default X-Plane 12 install
    /// is detected — the file is parsed into an `airspaces.parquet` inside
    /// the same apt.dat cache directory. Feeds the heartbeat's
    /// inside/over/under/through airspace awareness. Class Q (danger) is
    /// filtered out at parse time.
    #[arg(long)]
    airspace_txt_path: Option<PathBuf>,

    /// Path to a transcript log file. Defaults to output/sim_pilot-YYYYMMDD-HHMMSS.log.
    #[arg(long)]
    log_file: Option<PathBuf>,

    /// Disable the default transcript log file.
    #[arg(long)]
    no_log_file: bool,

    /// Disable the default flight-track recorder (output/sim_pilot-<ts>.csv
    /// streamed once per second, and output/sim_pilot-<ts>.kml written on
    /// clean shutdown).
    #[arg(long)]
    no_track: bool,

    /// Idle-period before the heartbeat pump wakes the LLM (seconds). The
    /// pump also wakes immediately when automation state changes — a
    /// flight-phase transition, profile engage/disengage, or profile
    /// completion (e.g. taxi reaches hold-short) — so this only gates the
    /// fallback "nothing is happening, check in" cadence.
    #[arg(long, default_value_t = 30.0)]
    heartbeat_interval: f64,

    /// Disable the heartbeat pump entirely.
    #[arg(long)]
    no_heartbeat: bool,

    /// Disable the push-to-talk voice transcription feature.
    #[arg(long)]
    no_voice: bool,
}

#[derive(Clone, Copy, Debug, ValueEnum)]
enum Backend {
    Simple,
    Xplane,
}

#[derive(Clone, Copy, Debug, ValueEnum)]
enum CliPilotMode {
    Normal,
    Realistic,
}

impl From<CliPilotMode> for PilotMode {
    fn from(m: CliPilotMode) -> Self {
        match m {
            CliPilotMode::Normal => PilotMode::Normal,
            CliPilotMode::Realistic => PilotMode::Realistic,
        }
    }
}

fn resolve_scenario_name(explicit: Option<&str>, wind: Vec2) -> String {
    if let Some(name) = explicit {
        return name.to_string();
    }
    if wind.x.abs() <= 1e-9 && wind.y.abs() <= 1e-9 {
        return "takeoff_to_pattern_landing".to_string();
    }
    if wind.x.abs() > 1e-9 && wind.y.abs() <= 1e-9 {
        return format!("takeoff_to_pattern_landing_crosswind_{:.1}kt", wind.x);
    }
    format!(
        "takeoff_to_pattern_landing_wind_x_{:.1}_y_{:.1}_kt",
        wind.x, wind.y
    )
}

/// Resolve (and build-if-needed) the apt.dat-derived parquet cache. An
/// explicit `--apt-dat-path` pins the source; otherwise we auto-detect the
/// standard X-Plane 12 install. The airspace.txt path is handled the same
/// way — when present (explicit flag or autodetected), its airspaces are
/// baked into `airspaces.parquet` in the same cache directory. Returns
/// `None` only when no apt.dat is locatable — the LLM's runway-lookup
/// tools will then error cleanly.
fn resolve_apt_dat_cache(
    apt_dat: Option<PathBuf>,
    airspace_txt: Option<PathBuf>,
) -> Result<Option<PathBuf>> {
    let explicit_apt = apt_dat.is_some();
    let apt = apt_dat.or_else(apt_dat::default_apt_dat_path);
    let Some(apt) = apt else { return Ok(None) };
    if !apt.exists() {
        if explicit_apt {
            return Err(anyhow!("apt.dat not found at {}", apt.display()));
        }
        return Ok(None);
    }
    println!("apt_dat_path={}", apt.display());

    let explicit_airspace = airspace_txt.is_some();
    let airspace = airspace_txt.or_else(xplane_pilot::data::airspace::default_airspace_txt_path);
    let airspace = match airspace {
        Some(p) if p.exists() => Some(p),
        Some(p) if explicit_airspace => {
            return Err(anyhow!("airspace.txt not found at {}", p.display()));
        }
        _ => None,
    };
    if let Some(p) = &airspace {
        println!("airspace_txt_path={}", p.display());
    }

    let cache = data_parquet::resolve(&apt, airspace.as_deref())
        .with_context(|| format!("building apt.dat parquet cache from {}", apt.display()))?;
    Ok(Some(cache.dir))
}

fn default_session_stem() -> String {
    format!("sim_pilot-{}", Local::now().format("%Y%m%d-%H%M%S"))
}

fn resolve_log_file_path(
    explicit: Option<PathBuf>,
    disabled: bool,
    stem: &str,
) -> Option<PathBuf> {
    if disabled {
        return None;
    }
    if let Some(p) = explicit {
        return Some(p);
    }
    Some(PathBuf::from("output").join(format!("{}.log", stem)))
}

fn resolve_track_paths(disabled: bool, stem: &str) -> Option<(PathBuf, PathBuf)> {
    if disabled {
        return None;
    }
    let dir = PathBuf::from("output");
    Some((
        dir.join(format!("{}.csv", stem)),
        dir.join(format!("{}.kml", stem)),
    ))
}

fn load_dotenv(path: &Path) {
    if !path.is_file() {
        return;
    }
    let Ok(text) = fs::read_to_string(path) else {
        return;
    };
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some((key, value)) = line.split_once('=') {
            let key = key.trim();
            let value = value.trim();
            if !key.is_empty() && std::env::var_os(key).is_none() {
                std::env::set_var(key, value);
            }
        }
    }
}

fn main() -> Result<()> {
    // Best-effort .env loader so OPENAI_API_KEY can live in a gitignored file.
    load_dotenv(Path::new(".env"));

    let args = Cli::parse();
    let config = load_default_config_bundle();
    match args.backend {
        Backend::Xplane => {
            if args.log_csv.is_some() || args.plots_dir.is_some() {
                return Err(anyhow!(
                    "Live X-Plane runs do not yet support --log-csv or --plots-dir."
                ));
            }
            let session_stem = default_session_stem();
            let log_file =
                resolve_log_file_path(args.log_file, args.no_log_file, &session_stem);
            let track_paths = resolve_track_paths(args.no_track, &session_stem);
            println!("backend=xplane");
            println!("xplane_host={}", args.xplane_host);
            println!("xplane_port={}", args.xplane_port);
            println!("pilot_llm_model={}", args.pilot_llm_model);
            let pilot_mode: PilotMode = args.pilot_mode.into();
            println!("pilot_mode={}", pilot_mode.label());
            if let Some(e) = &args.pilot_llm_reasoning_effort {
                println!("pilot_llm_reasoning_effort={}", e);
            }
            if let Some(p) = &log_file {
                println!("log_file={}", p.display());
            }
            if let Some((csv, kml)) = &track_paths {
                println!("track_csv={}", csv.display());
                println!("track_kml={}", kml.display());
            }
            let apt_dat_cache_dir =
                resolve_apt_dat_cache(args.apt_dat_path, args.airspace_txt_path)?;
            if let Some(p) = &apt_dat_cache_dir {
                println!("apt_dat_cache_dir={}", p.display());
            }
            run_live_xplane(
                config,
                LiveRunConfig {
                    xplane_host: args.xplane_host,
                    xplane_port: args.xplane_port,
                    llm_model: args.pilot_llm_model,
                    llm_reasoning_effort: args.pilot_llm_reasoning_effort,
                    atc_messages: args.atc_message,
                    interactive: !args.headless,
                    control_hz: args.control_hz,
                    status_interval_s: args.status_interval_s,
                    engage_profile: args.engage_profile,
                    apt_dat_cache_dir,
                    log_file_path: log_file,
                    track_paths,
                    session_stem,
                    heartbeat_interval_s: args.heartbeat_interval,
                    heartbeat_enabled: !args.no_heartbeat,
                    voice_enabled: !args.no_voice,
                    pilot_mode,
                },
            )?;
        }
        Backend::Simple => {
            let wind = Vec2::new(args.wind_x_kt + args.crosswind_kt, args.wind_y_kt);
            let scenario_name = resolve_scenario_name(args.scenario_name.as_deref(), wind);
            let mut runner = ScenarioRunner::new(config.clone());
            runner.wind_vector_kt = wind;
            let result = runner.run();
            println!("scenario_name={}", scenario_name);
            println!("success={}", result.success);
            println!("final_phase={}", result.final_phase.value());
            println!("duration_s={:.1}", result.duration_s);
            println!("wind_x_kt={:.1}", wind.x);
            println!("wind_y_kt={:.1}", wind.y);
            if let Some(v) = result.touchdown_runway_x_ft {
                println!("touchdown_runway_x_ft={:.1}", v);
            }
            if let Some(v) = result.touchdown_centerline_ft {
                println!("touchdown_centerline_ft={:.1}", v);
            }
            if let Some(v) = result.touchdown_sink_fpm {
                println!("touchdown_sink_fpm={:.1}", v);
            }
            if let Some(path) = args.log_csv {
                let written = write_scenario_log_csv(&result, &path)?;
                println!("log_csv={}", written.display());
            }
            if let Some(dir) = args.plots_dir {
                let plots = write_scenario_plots(&result, &config, &scenario_name, &dir)?;
                println!("plot_overview_svg={}", plots.overview_svg.display());
                println!("plot_ground_path_svg={}", plots.ground_path_svg.display());
            }
        }
    }
    Ok(())
}
