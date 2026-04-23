use tempfile::TempDir;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::sim::plotting::write_scenario_plots;
use xplane_pilot::sim::scenario::ScenarioRunner;

#[test]
fn plot_writer_emits_svg_files() {
    let config = load_default_config_bundle();
    let result = ScenarioRunner::new(config.clone()).run();
    let dir = TempDir::new().unwrap();
    let plots = write_scenario_plots(&result, &config, "takeoff_to_pattern_landing", dir.path()).unwrap();
    assert!(plots.overview_svg.exists());
    assert!(plots.ground_path_svg.exists());
    let overview = std::fs::read_to_string(&plots.overview_svg).unwrap();
    let ground = std::fs::read_to_string(&plots.ground_path_svg).unwrap();
    assert!(overview.contains("<svg"));
    assert!(overview.contains("Scenario: takeoff_to_pattern_landing"));
    assert!(overview.contains("phase by time"));
    assert!(overview.contains("downwind"));
    assert!(ground.contains("<svg"));
    let rwy_id = config.airport.runway.id.as_deref().unwrap();
    assert!(ground.contains(&format!("Runway {} threshold", rwy_id)));
    assert!(ground.contains("north-up world frame"));
    assert!(ground.contains("north up"));
    assert!(ground.contains("takeoff roll"));
}
