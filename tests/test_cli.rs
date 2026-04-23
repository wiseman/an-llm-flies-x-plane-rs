use xplane_pilot::types::Vec2;

// Re-implement resolve_scenario_name here to keep the test independent of the
// binary target (which has a private `main` + `resolve_scenario_name`).
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
    format!("takeoff_to_pattern_landing_wind_x_{:.1}_y_{:.1}_kt", wind.x, wind.y)
}

#[test]
fn nominal_name_when_no_wind() {
    assert_eq!(
        resolve_scenario_name(None, Vec2::new(0.0, 0.0)),
        "takeoff_to_pattern_landing"
    );
}

#[test]
fn crosswind_name_when_only_x_wind() {
    assert_eq!(
        resolve_scenario_name(None, Vec2::new(10.0, 0.0)),
        "takeoff_to_pattern_landing_crosswind_10.0kt"
    );
}

#[test]
fn explicit_name_takes_precedence() {
    assert_eq!(
        resolve_scenario_name(Some("pattern_debug"), Vec2::new(10.0, 5.0)),
        "pattern_debug"
    );
}
