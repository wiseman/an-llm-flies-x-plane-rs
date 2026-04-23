//! CSV scenario log writer.

use std::path::{Path, PathBuf};

use anyhow::Result;

use crate::sim::scenario::ScenarioResult;

pub fn write_scenario_log_csv(result: &ScenarioResult, path: impl AsRef<Path>) -> Result<PathBuf> {
    let path = path.as_ref().to_path_buf();
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() && parent != Path::new(".") {
            std::fs::create_dir_all(parent)?;
        }
    }
    let mut wtr = csv::Writer::from_path(&path)?;
    wtr.write_record([
        "time_s",
        "phase",
        "position_x_ft",
        "position_y_ft",
        "runway_x_ft",
        "runway_y_ft",
        "pitch_deg",
        "altitude_msl_ft",
        "altitude_agl_ft",
        "throttle_pos",
        "throttle_cmd",
        "ias_kt",
        "gs_kt",
        "heading_deg",
        "bank_deg",
    ])?;
    for row in &result.history {
        wtr.write_record([
            format!("{:.1}", row.time_s),
            row.phase.value().to_string(),
            format!("{:.3}", row.position_x_ft),
            format!("{:.3}", row.position_y_ft),
            format!("{:.3}", row.runway_x_ft),
            format!("{:.3}", row.runway_y_ft),
            format!("{:.3}", row.pitch_deg),
            format!("{:.3}", row.altitude_msl_ft),
            format!("{:.3}", row.altitude_agl_ft),
            format!("{:.3}", row.throttle_pos),
            format!("{:.3}", row.throttle_cmd),
            format!("{:.3}", row.ias_kt),
            format!("{:.3}", row.gs_kt),
            format!("{:.3}", row.heading_deg),
            format!("{:.3}", row.bank_deg),
        ])?;
    }
    wtr.flush()?;
    Ok(path)
}
