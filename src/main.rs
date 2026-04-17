//! `sim-pilot` CLI. Mirrors sim_pilot/__main__.py.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{anyhow, Context, Result};
use chrono::Local;
use clap::{Parser, ValueEnum};

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::data::{apt_dat, parquet as data_parquet};
use xplane_pilot::live_runner::{run_live_xplane, LiveRunConfig};
use xplane_pilot::sim::{
    logging::write_scenario_log_csv, plotting::write_scenario_plots, scenario::ScenarioRunner,
};
use xplane_pilot::types::Vec2;

#[derive(Parser, Debug)]
#[command(about = "Deterministic offline sim / live X-Plane MVP bridge")]
struct Cli {
    #[arg(long, value_enum, default_value_t = Backend::Simple)]
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

    #[arg(long, default_value = "gpt-5.4-2026-03-05")]
    llm_model: String,

    /// ATC/operator message sent through the LLM at startup. Repeat for multiple messages.
    #[arg(long = "atc-message")]
    atc_message: Vec<String>,

    /// Read ATC/operator messages from the TUI during a live X-Plane run.
    #[arg(long)]
    interactive_atc: bool,

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

    /// Path to a transcript log file. Defaults to output/sim_pilot-YYYYMMDD-HHMMSS.log.
    #[arg(long)]
    log_file: Option<PathBuf>,

    /// Disable the default transcript log file.
    #[arg(long)]
    no_log_file: bool,

    /// Idle-period before the heartbeat pump wakes the LLM (seconds).
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
/// standard X-Plane 12 install. Returns `None` only when no apt.dat is
/// locatable — the LLM's runway-lookup tools will then error cleanly.
fn resolve_apt_dat_cache(apt_dat: Option<PathBuf>) -> Result<Option<PathBuf>> {
    let explicit = apt_dat.is_some();
    let apt = apt_dat.or_else(apt_dat::default_apt_dat_path);
    let Some(apt) = apt else { return Ok(None) };
    if !apt.exists() {
        if explicit {
            return Err(anyhow!("apt.dat not found at {}", apt.display()));
        }
        return Ok(None);
    }
    println!("apt_dat_path={}", apt.display());
    let cache = data_parquet::resolve(&apt)
        .with_context(|| format!("building apt.dat parquet cache from {}", apt.display()))?;
    Ok(Some(cache.dir))
}

fn resolve_log_file_path(explicit: Option<PathBuf>, disabled: bool) -> Option<PathBuf> {
    if disabled {
        return None;
    }
    if let Some(p) = explicit {
        return Some(p);
    }
    let ts = Local::now().format("%Y%m%d-%H%M%S");
    Some(PathBuf::from("output").join(format!("sim_pilot-{}.log", ts)))
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
            let log_file = resolve_log_file_path(args.log_file, args.no_log_file);
            println!("backend=xplane");
            println!("xplane_host={}", args.xplane_host);
            println!("xplane_port={}", args.xplane_port);
            println!("llm_model={}", args.llm_model);
            if let Some(p) = &log_file {
                println!("log_file={}", p.display());
            }
            let apt_dat_cache_dir = resolve_apt_dat_cache(args.apt_dat_path)?;
            if let Some(p) = &apt_dat_cache_dir {
                println!("apt_dat_cache_dir={}", p.display());
            }
            run_live_xplane(
                config,
                LiveRunConfig {
                    xplane_host: args.xplane_host,
                    xplane_port: args.xplane_port,
                    llm_model: args.llm_model,
                    atc_messages: args.atc_message,
                    interactive: args.interactive_atc,
                    control_hz: args.control_hz,
                    status_interval_s: args.status_interval_s,
                    engage_profile: args.engage_profile,
                    apt_dat_cache_dir,
                    log_file_path: log_file,
                    heartbeat_interval_s: args.heartbeat_interval,
                    heartbeat_enabled: !args.no_heartbeat,
                    voice_enabled: !args.no_voice,
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
