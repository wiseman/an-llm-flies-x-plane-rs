//! Taxi routing on the real KVNY (Van Nuys) taxi network.
//!
//! KVNY has parallel runways 16L/34R and 16R/34L. Taxiway G runs east to
//! west straight across both; H is a one-way high-speed exit that leaves
//! 16R about 22 m south of G and runs southwest to A. Shortest-path
//! routing from the 16L exit at G to the Signature West ramp would cross
//! 16R on G, turn down the runway to H and leave on it: a taxi along a
//! runway nobody cleared (live take, 2026-10-19).
//!
//! Fixture file: `tests/fixtures/kvny_apt.dat` (the KVNY block of the
//! Global Airports apt.dat with pavement, lighting and sign rows dropped;
//! airport, runway, comm, taxi-network and parking rows are verbatim).

use std::sync::Arc;

use parking_lot::Mutex as PLMutex;
use tempfile::TempDir;

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::guidance::taxi_route::{self, TaxiGoal, TaxiStart};
use xplane_pilot::llm::tools::{ensure_runway_conn, ToolContext};

const KVNY_FIXTURE: &str = include_str!("fixtures/kvny_apt.dat");

/// The end of the 16L exit at G that plan_arrival planned the taxi from in
/// the 2026-10-19 take (output/sim_pilot-20261019-155704.log): G node 165,
/// between 16L and 16R, heading down G's last segment (166 -> 165).
const G_EXIT_END: TaxiStart = TaxiStart { lat: 34.210235, lon: -118.489739, heading_deg: 265.4 };

fn kvny_ctx() -> (TempDir, ToolContext) {
    let parsed = xplane_pilot::data::apt_dat::parse(KVNY_FIXTURE.as_bytes()).expect("KVNY fixture parses");
    let dir = TempDir::new().expect("tmp dir");
    xplane_pilot::data::parquet::write_cache(&parsed, &[], dir.path()).expect("parquet cache writes");
    let cfg = load_default_config_bundle();
    let pilot = Arc::new(PLMutex::new(PilotCore::new(cfg.clone())));
    let mut ctx = ToolContext::new(pilot, cfg);
    ctx.apt_dat_cache_dir = Some(dir.path().to_path_buf());
    ensure_runway_conn(&ctx).expect("open parquet");
    (dir, ctx)
}

fn plan_to_signature(ctx: &ToolContext, via: &[String]) -> taxi_route::PlannedTaxi {
    let guard = ctx.runway_conn.lock().unwrap();
    let conn = guard.as_ref().unwrap();
    let graph = taxi_route::load_graph(conn, "KVNY").expect("load graph");
    taxi_route::plan_from(conn, &graph, G_EXIT_END, &TaxiGoal::Parking("Signature West FBO Parking"), via)
        .expect("plan to Signature West")
}

#[test]
fn taxi_from_the_g_exit_crosses_16r_instead_of_taxiing_down_it() {
    let (_dir, ctx) = kvny_ctx();
    let planned = plan_to_signature(&ctx, &[]);
    let plan = &planned.plan;
    eprintln!(
        "route: {} ({} legs, {:.0} m), crossings {:?}",
        plan.taxiway_sequence.join(" -> "),
        plan.legs.len(),
        plan.total_distance_m,
        plan.runway_crossings
    );
    assert!(
        !plan.legs.iter().any(|l| l.on_runway || l.taxiway_name == "16R/34L"),
        "route runs along runway 16R/34L: {:?}",
        plan.taxiway_sequence
    );
    assert!(
        plan.legs.iter().any(|l| l.active_zones.contains("departure:16R,34L")),
        "route should still cross 16R: {:?}",
        plan.runway_crossings
    );
    assert_eq!(plan.taxiway_sequence.first().map(String::as_str), Some("G"), "{:?}", plan.taxiway_sequence);
}

#[test]
fn naming_the_runway_in_the_clearance_allows_taxiing_on_it() {
    // "Taxi via G, runway 16R, H, A": the runway leg is part of the
    // clearance, so the planner takes it.
    let (_dir, ctx) = kvny_ctx();
    let via: Vec<String> = ["G", "16R", "H", "A"].iter().map(|s| s.to_string()).collect();
    let planned = plan_to_signature(&ctx, &via);
    assert!(
        planned.plan.legs.iter().any(|l| l.on_runway && l.taxiway_name == "16R/34L"),
        "{:?}",
        planned.plan.taxiway_sequence
    );
    assert_eq!(planned.via_runways, vec!["16R".to_string(), "34L".to_string()]);
}
