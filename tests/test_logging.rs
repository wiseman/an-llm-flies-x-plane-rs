//! Port of tests/test_logging.py.

use tempfile::TempDir;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::sim::logging::write_scenario_log_csv;
use xplane_pilot::sim::scenario::ScenarioRunner;

#[test]
fn log_writer_emits_requested_columns() {
    let result = ScenarioRunner::new(load_default_config_bundle()).run();
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("flight_log.csv");
    let written = write_scenario_log_csv(&result, &path).unwrap();
    assert_eq!(written, path);
    let mut rdr = csv::Reader::from_path(&written).unwrap();
    let headers = rdr.headers().unwrap().clone();
    let expected = [
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
    ];
    let got: Vec<&str> = headers.iter().collect();
    assert_eq!(got, expected);
    let first = rdr.records().next().unwrap().unwrap();
    assert!(matches!(&first[1], "takeoff_roll" | "rotate"));
}
