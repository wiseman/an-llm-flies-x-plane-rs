//! End-to-end tests for intersection-departure support using the real
//! KEMT (El Monte) taxi network extracted from X-Plane's apt.dat. KEMT
//! has taxiways A-F and a single runway 01/19; taxiway "C" crosses the
//! runway near the middle, so "runway 19 at Charlie" leaves ~2700 ft of
//! usable pavement — enough to take off in a C172 but not the full
//! 3995 ft.
//!
//! Fixture file: `tests/fixtures/kemt_apt.dat` (a raw slice of the
//! Global Airports apt.dat, version-controlled so the tests are
//! hermetic and don't depend on an X-Plane install).

use std::collections::HashMap;
use std::sync::Arc;

use anyhow::Result;
use parking_lot::Mutex as PLMutex;
use tempfile::TempDir;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::guidance::taxi_route::{self, TaxiDestination};
use xplane_pilot::llm::tools::{ensure_runway_conn, ToolBridge, ToolContext};
use xplane_pilot::sim::xplane_bridge::GeoReference;

const KEMT_FIXTURE: &str = include_str!("fixtures/kemt_apt.dat");

/// KEMT ARP (airport reference point) — used as the FakeBridge georef
/// origin so geodetic-offset math has a sensible local frame.
const KEMT_LAT: f64 = 34.086008;
const KEMT_LON: f64 = -118.034844;

struct FakeBridge {
    values: PLMutex<HashMap<String, f64>>,
    georef: GeoReference,
}

impl FakeBridge {
    fn new(lat: f64, lon: f64) -> Arc<Self> {
        Arc::new(FakeBridge {
            values: PLMutex::new(HashMap::new()),
            georef: GeoReference {
                threshold_lat_deg: lat,
                threshold_lon_deg: lon,
            },
        })
    }
    fn set(&self, name: &str, v: f64) {
        self.values.lock().insert(name.to_string(), v);
    }
}

impl ToolBridge for FakeBridge {
    fn georef(&self) -> GeoReference {
        self.georef
    }
    fn get_dataref_value(&self, name: &str) -> Option<f64> {
        self.values.lock().get(name).copied()
    }
    fn write_dataref_values(&self, updates: &[(String, f64)]) -> Result<()> {
        let mut v = self.values.lock();
        for (k, val) in updates {
            v.insert(k.clone(), *val);
        }
        Ok(())
    }
}

/// Build a parquet cache containing just KEMT, return the cache dir.
fn kemt_cache() -> TempDir {
    let parsed = xplane_pilot::data::apt_dat::parse(KEMT_FIXTURE.as_bytes())
        .expect("KEMT fixture parses");
    let dir = TempDir::new().expect("tmp dir");
    xplane_pilot::data::parquet::write_cache(&parsed, &[], dir.path())
        .expect("parquet cache writes");
    dir
}

fn make_ctx(bridge: Arc<dyn ToolBridge>, cache: &TempDir) -> ToolContext {
    let cfg = load_default_config_bundle();
    let pilot = Arc::new(PLMutex::new(PilotCore::new(cfg.clone())));
    let mut ctx = ToolContext::new(pilot, cfg);
    ctx.bridge = Some(bridge);
    ctx.apt_dat_cache_dir = Some(cache.path().to_path_buf());
    ctx
}

// ---------- resolver ----------

#[test]
fn intersection_resolver_identifies_on_runway_and_hold_short_nodes() {
    let cache = kemt_cache();
    let bridge = FakeBridge::new(KEMT_LAT, KEMT_LON);
    let ctx = make_ctx(bridge.clone(), &cache);
    ensure_runway_conn(&ctx).expect("open parquet");

    let guard = ctx.runway_conn.lock().unwrap();
    let conn = guard.as_ref().unwrap();
    let graph = taxi_route::load_graph(conn, "KEMT").expect("load graph");
    let x = taxi_route::resolve_runway_intersection(conn, &graph, "KEMT", "19", "C")
        .expect("resolve C/19");

    // Truth values extracted from the KEMT block at extraction time
    // (see the 1201 rows in tests/fixtures/kemt_apt.dat):
    //   node 48 is on the runway (lat 34.088360, lon -118.033519)
    //   node 53 is the hold-short (lat 34.088220, lon -118.033171)
    assert_eq!(x.on_runway_node, 48);
    assert!((x.on_runway_lat - 34.088360).abs() < 1e-5);
    assert!((x.on_runway_lon - (-118.033519)).abs() < 1e-5);
    assert_eq!(x.hold_short_node, 53);
}

// ---------- TaxiDestination::RunwayIntersection ----------

#[test]
fn intersection_hold_short_differs_from_full_length_hold_short() {
    let cache = kemt_cache();
    let bridge = FakeBridge::new(KEMT_LAT, KEMT_LON);
    let ctx = make_ctx(bridge.clone(), &cache);
    ensure_runway_conn(&ctx).expect("open parquet");

    let guard = ctx.runway_conn.lock().unwrap();
    let conn = guard.as_ref().unwrap();
    let graph = taxi_route::load_graph(conn, "KEMT").expect("load graph");

    let full_length = taxi_route::resolve_destination(
        conn,
        &graph,
        "KEMT",
        &TaxiDestination::Runway("19"),
    )
    .expect("resolve full-length");
    let at_c = taxi_route::resolve_destination(
        conn,
        &graph,
        "KEMT",
        &TaxiDestination::RunwayIntersection {
            runway: "19",
            taxiway: "C",
        },
    )
    .expect("resolve C/19");

    assert_ne!(
        full_length.node, at_c.node,
        "intersection hold-short should not equal the full-length hold-short"
    );
    // Charlie's hold-short is node 53.
    assert_eq!(at_c.node, 53);
    // Both should forbid the full set of runway-crossing edges.
    assert!(!at_c.forbidden_edges.is_empty());
    // face_toward_latlon should point at the on-runway node (node 48).
    let (flat, flon) = at_c.face_toward_latlon.expect("face_toward set");
    assert!((flat - 34.088360).abs() < 1e-5);
    assert!((flon - (-118.033519)).abs() < 1e-5);
}

// ---------- engage_line_up with intersection ----------

#[test]
fn engage_line_up_at_intersection_uses_on_runway_point_not_threshold() {
    let cache = kemt_cache();
    let bridge = FakeBridge::new(KEMT_LAT, KEMT_LON);
    // Park the aircraft at node 45 (taxiway A / Charlie junction) so the
    // lineup's 2-leg route is from there → entry point → past entry.
    bridge.set(
        xplane_pilot::sim::datarefs::LATITUDE_DEG.name,
        34.088150,
    );
    bridge.set(
        xplane_pilot::sim::datarefs::LONGITUDE_DEG.name,
        -118.032996,
    );
    let ctx = make_ctx(bridge.clone(), &cache);

    let r = xplane_pilot::llm::tools::dispatch_tool(
        &serde_json::json!({
            "name": "engage_line_up",
            "call_id": "c1",
            "arguments": serde_json::json!({
                "airport_ident": "KEMT",
                "runway_ident": "19",
                "intersection": "C",
                "start_lat": null,
                "start_lon": null,
            }).to_string(),
        }),
        &ctx,
    );
    assert!(
        r.starts_with("engaged line_up KEMT runway 19 at C"),
        "got: {r}"
    );
    // 19 full-length threshold → C is ~950 ft south; C → aircraft
    // at node 45 is a short taxi, so reported crossing distance is
    // small (≪ full-runway 3800 ft).
    let crossing = r
        .split("crossing ")
        .nth(1)
        .and_then(|s| s.split(" ft").next())
        .and_then(|s| s.parse::<f64>().ok())
        .expect("parse crossing ft");
    assert!(
        crossing < 800.0,
        "intersection lineup should be short; got {crossing} ft (would be ~3000+ ft if threshold was used)"
    );
}

// ---------- line-up distance guard ----------

#[test]
fn engage_line_up_rejects_when_aircraft_is_far_from_entry_point() {
    // Park the aircraft at node 38 — the west-ramp end of taxiway A,
    // ~1700 ft from the B/19 intersection (node 35). engage_line_up
    // intersection=B should refuse rather than plan a straight-line
    // diagonal across the ramp + taxiway onto the runway.
    let cache = kemt_cache();
    let bridge = FakeBridge::new(KEMT_LAT, KEMT_LON);
    // Node 38 lat/lon from the fixture.
    bridge.set(
        xplane_pilot::sim::datarefs::LATITUDE_DEG.name,
        34.081092,
    );
    bridge.set(
        xplane_pilot::sim::datarefs::LONGITUDE_DEG.name,
        -118.037669,
    );
    let ctx = make_ctx(bridge.clone(), &cache);

    let r = xplane_pilot::llm::tools::dispatch_tool(
        &serde_json::json!({
            "name": "engage_line_up",
            "call_id": "c1",
            "arguments": serde_json::json!({
                "airport_ident": "KEMT",
                "runway_ident": "19",
                "intersection": "B",
                "start_lat": null,
                "start_lon": null,
            }).to_string(),
        }),
        &ctx,
    );
    assert!(r.starts_with("error:"), "expected error, got: {r}");
    assert!(
        r.contains("engage_taxi") && r.contains("intersection"),
        "error should tell the LLM to call engage_taxi with intersection first: {r}"
    );
}

// ---------- TakeoffProfile usable length & debug line ----------

#[test]
fn takeoff_profile_debug_line_shows_rolled_and_remaining() {
    use xplane_pilot::core::profiles::{GuidanceProfile, TakeoffProfile};
    use xplane_pilot::guidance::runway_geometry::RunwayFrame;
    use xplane_pilot::types::{AircraftState, Runway, TrafficSide, Vec2};

    let rwy = Runway {
        id: Some("19".to_string()),
        threshold_ft: Vec2 { x: 0.0, y: 0.0 },
        course_deg: 190.0,
        length_ft: 3995.0,
        touchdown_zone_ft: 300.0,
        displaced_threshold_ft: 0.0,
        traffic_side: TrafficSide::Left,
    };
    let runway_frame = RunwayFrame::new(rwy);
    let cfg = load_default_config_bundle();
    // Simulate an intersection departure at along-track 1000 ft on a
    // 3995-ft runway: usable = 2995 ft, start_along = 1000 ft.
    let profile =
        TakeoffProfile::new_with_usable_length(cfg, runway_frame.clone(), 2995.0, 1000.0);

    // Move the aircraft to along-track 2000 ft = 1000 ft rolled from
    // start, 1995 ft remaining. In runway-frame coords the aircraft's
    // position_ft is 2000 along the course; we need to project it back
    // to world frame. runway_frame.to_runway_frame is linear around
    // threshold; the inverse for our rwy (threshold at origin, course
    // 190° measured from north) is: world = 2000 * unit_forward.
    let forward = xplane_pilot::types::heading_to_vector(190.0, 2000.0);
    let mut st = AircraftState::synthetic_default();
    st.position_ft = forward;
    st.ias_kt = 40.0;

    let line = profile.debug_line(&st).expect("debug line present");
    assert!(line.contains("usable=2995ft"), "got: {line}");
    // 1000 ft rolled, 1995 ft remaining.
    assert!(
        line.contains("rolled=1000ft") || line.contains("rolled=999ft")
            || line.contains("rolled=1001ft"),
        "got: {line}"
    );
    assert!(line.contains("remaining="), "got: {line}");
    // 40 kt at 1000/2995 ≈ 33% rolled — abort advisory should NOT fire.
    assert!(!line.contains("ABORT"), "got: {line}");
}

#[test]
fn takeoff_profile_abort_advisory_fires_past_70pct_below_vr() {
    use xplane_pilot::core::profiles::{GuidanceProfile, TakeoffProfile};
    use xplane_pilot::guidance::runway_geometry::RunwayFrame;
    use xplane_pilot::types::{AircraftState, Runway, TrafficSide, Vec2};

    let rwy = Runway {
        id: Some("19".to_string()),
        threshold_ft: Vec2 { x: 0.0, y: 0.0 },
        course_deg: 190.0,
        length_ft: 3995.0,
        touchdown_zone_ft: 300.0,
        displaced_threshold_ft: 0.0,
        traffic_side: TrafficSide::Left,
    };
    let runway_frame = RunwayFrame::new(rwy);
    let cfg = load_default_config_bundle();
    let vr = cfg.performance.vr_kt;
    let profile =
        TakeoffProfile::new_with_usable_length(cfg, runway_frame.clone(), 2000.0, 0.0);

    // At 1600 ft along (80% of the 2000-ft usable length) with IAS
    // at 60% of Vr, the abort heuristic should light up.
    let forward = xplane_pilot::types::heading_to_vector(190.0, 1600.0);
    let mut st = AircraftState::synthetic_default();
    st.position_ft = forward;
    st.ias_kt = 0.6 * vr;

    let line = profile.debug_line(&st).expect("debug line present");
    assert!(line.contains("ABORT-ADVISED"), "got: {line}");
}
