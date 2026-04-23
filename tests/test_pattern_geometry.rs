use approx::assert_abs_diff_eq;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::guidance::pattern_manager::{
    build_pattern_geometry, glidepath_target_altitude_ft, glidepath_target_altitude_ft_default,
};
use xplane_pilot::guidance::runway_geometry::RunwayFrame;

#[test]
fn downwind_line_generated_on_correct_side() {
    let config = load_default_config_bundle();
    let rf = RunwayFrame::new(config.airport.runway.clone());
    let pattern = build_pattern_geometry(&rf, config.pattern.downwind_offset_ft, 0.0, 0.0);
    let start = rf.to_runway_frame(pattern.downwind_leg.start_ft);
    let end = rf.to_runway_frame(pattern.downwind_leg.end_ft);
    assert_abs_diff_eq!(start.y, -config.pattern.downwind_offset_ft, epsilon = 1e-6);
    assert_abs_diff_eq!(end.y, -config.pattern.downwind_offset_ft, epsilon = 1e-6);
}

#[test]
fn base_turn_point_moves_after_extension() {
    let config = load_default_config_bundle();
    let rf = RunwayFrame::new(config.airport.runway.clone());
    let nominal = build_pattern_geometry(&rf, config.pattern.downwind_offset_ft, 0.0, 0.0);
    let extended = build_pattern_geometry(&rf, config.pattern.downwind_offset_ft, 2000.0, 0.0);
    assert!(extended.base_turn_x_ft < nominal.base_turn_x_ft);
    assert_abs_diff_eq!(
        extended.base_turn_x_ft - nominal.base_turn_x_ft,
        -2000.0,
        epsilon = 1e-6
    );
}

#[test]
fn final_intercept_stays_on_centerline() {
    let config = load_default_config_bundle();
    let rf = RunwayFrame::new(config.airport.runway.clone());
    let pattern = build_pattern_geometry(&rf, config.pattern.downwind_offset_ft, 0.0, 0.0);
    let base_end = rf.to_runway_frame(pattern.base_leg.end_ft);
    let final_start = rf.to_runway_frame(pattern.final_leg.start_ft);
    let final_end = rf.to_runway_frame(pattern.final_leg.end_ft);
    assert_abs_diff_eq!(base_end.y, 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(final_start.y, 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(final_end.y, 0.0, epsilon = 1e-6);
    assert!(final_start.x < base_end.x);
    assert!(base_end.x < final_end.x);
}

#[test]
fn altitude_at_aim_point_is_ground_level_by_default() {
    let config = load_default_config_bundle();
    let rf = RunwayFrame::new(config.airport.runway.clone());
    let aim_x = rf.touchdown_runway_x_ft();
    let target = glidepath_target_altitude_ft_default(
        &rf,
        aim_x,
        config.airport.field_elevation_ft,
    );
    assert_abs_diff_eq!(target, config.airport.field_elevation_ft, epsilon = 1e-6);
}

#[test]
fn altitude_rises_before_the_aim_point() {
    let config = load_default_config_bundle();
    let rf = RunwayFrame::new(config.airport.runway.clone());
    let aim_x = rf.touchdown_runway_x_ft();
    let target = glidepath_target_altitude_ft_default(
        &rf,
        aim_x - 2000.0,
        config.airport.field_elevation_ft,
    );
    let expected = 2000.0 * 3.0 / 57.2958;
    assert_abs_diff_eq!(target - config.airport.field_elevation_ft, expected, epsilon = 0.1);
}

#[test]
fn altitude_clamps_at_ground_level() {
    let config = load_default_config_bundle();
    let rf = RunwayFrame::new(config.airport.runway.clone());
    let aim_x = rf.touchdown_runway_x_ft();
    let target = glidepath_target_altitude_ft_default(
        &rf,
        aim_x + 5000.0,
        config.airport.field_elevation_ft,
    );
    assert_abs_diff_eq!(target, config.airport.field_elevation_ft, epsilon = 1e-6);
}

#[test]
fn non_zero_aim_point_height_adds_offset() {
    let config = load_default_config_bundle();
    let rf = RunwayFrame::new(config.airport.runway.clone());
    let aim_x = rf.touchdown_runway_x_ft();
    let target = glidepath_target_altitude_ft(&rf, aim_x, config.airport.field_elevation_ft, 3.0, 50.0);
    assert_abs_diff_eq!(target - config.airport.field_elevation_ft, 50.0, epsilon = 1e-6);
}
