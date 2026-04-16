//! Port of tests/test_xplane_bridge.py — exercises the pure helpers only.

use approx::assert_abs_diff_eq;
use serde_json::json;

use xplane_pilot::sim::datarefs::{COMMAND_DATAREFS, OVERRIDE_JOYSTICK_HEADING};
use xplane_pilot::sim::xplane_bridge::{
    coerce_scalar, flap_ratio_to_setting, flap_setting_to_ratio, geodetic_offset_ft, select_index,
    GeoReference,
};

#[test]
fn positive_lat_and_lon_delta_produces_east_north_feet() {
    let georef = GeoReference { threshold_lat_deg: 34.0, threshold_lon_deg: -118.0 };
    let p = geodetic_offset_ft(34.001, -117.999, georef);
    assert!(p.x > 0.0);
    assert!(p.y > 0.0);
}

#[test]
fn same_point_is_origin() {
    let georef = GeoReference { threshold_lat_deg: 47.449, threshold_lon_deg: -122.309 };
    let p = geodetic_offset_ft(47.449, -122.309, georef);
    assert_abs_diff_eq!(p.x, 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(p.y, 0.0, epsilon = 1e-6);
}

#[test]
fn flap_round_trip_matches_nearest_setting() {
    for s in [0, 10, 20, 30] {
        let r = flap_setting_to_ratio(s);
        assert_eq!(flap_ratio_to_setting(r), s);
    }
}

#[test]
fn flap_ratio_outside_bounds_clamps_to_nearest_setting() {
    assert_eq!(flap_ratio_to_setting(-0.1), 0);
    assert_eq!(flap_ratio_to_setting(1.5), 30);
}

#[test]
fn override_joystick_heading_is_in_command_datarefs() {
    assert!(COMMAND_DATAREFS.iter().any(|s| s.name == OVERRIDE_JOYSTICK_HEADING.name));
}

#[test]
fn scalar_coerces_to_f64() {
    assert_eq!(coerce_scalar(&json!(1)), 1.0);
    assert_eq!(coerce_scalar(&json!(0.5)), 0.5);
}

#[test]
fn list_coerces_first_element() {
    assert_eq!(coerce_scalar(&json!([1.0, 2.0])), 1.0);
    assert_eq!(coerce_scalar(&json!([])), 0.0);
}

#[test]
fn select_index_returns_requested_element() {
    assert_eq!(select_index(&json!([10.0, 20.0, 30.0]), Some(1)).unwrap(), 20.0);
    assert_eq!(select_index(&json!(42.0), None).unwrap(), 42.0);
}

#[test]
fn select_index_rejects_out_of_range() {
    assert!(select_index(&json!([1.0]), Some(5)).is_err());
}
