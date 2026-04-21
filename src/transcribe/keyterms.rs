//! Dynamic vocabulary builder for Deepgram keyterm biasing.
//!
//! Built fresh on every PTT hold. Pulls proper nouns from the apt.dat
//! parquet views (airports within 30 NM, comm facility labels, runway
//! identifiers, taxiway names) plus a tiny phase-aware phraseology set.
//! Deduplicated, capped at 100 terms.

use std::collections::HashSet;
use std::sync::Arc;

use anyhow::Result;
use parking_lot::Mutex as PLMutex;

use crate::core::mission_manager::{PilotCore, StatusSnapshot};
use crate::llm::tools::{ensure_runway_conn, ToolBridge, ToolContext};
use crate::sim::datarefs::{LATITUDE_DEG, LONGITUDE_DEG};
use crate::types::FlightPhase;

/// Soft cap. Deepgram accepts many but returns diminishing benefit past ~100.
const MAX_TERMS: usize = 100;

/// Search radius for "nearby airport" hint harvesting.
const NEARBY_AIRPORTS_RADIUS_NM: f64 = 30.0;
const NEARBY_AIRPORTS_LIMIT: i64 = 15;

/// Words at or below this length are discarded from split labels — "at",
/// "of", "to", "la" add no value and crowd out real proper nouns.
const MIN_WORD_LEN: usize = 3;

/// Build the keyterm list for the current flight state. `atc_mode` toggles
/// ATC-leaning static terms on (Tab key) vs operator-leaning terms
/// (Space key).
///
/// All DuckDB work happens after the PilotCore lock is released — the
/// snapshot is cloned under the lock and then dropped. Best-effort: any
/// sub-query that fails is skipped silently; we still want *some* hints
/// even if, say, the parquet cache isn't populated.
pub fn build_keyterms(
    pilot: &Arc<PLMutex<PilotCore>>,
    bridge: Option<&dyn ToolBridge>,
    ctx: &ToolContext,
    atc_mode: bool,
) -> Vec<String> {
    let snapshot: Option<StatusSnapshot> = pilot.lock().latest_snapshot.clone();
    let aircraft = pilot.lock().config.performance.aircraft.clone();

    // Two-tier output so the 100-term cap drops the least-specific
    // entries first. Hand-curated baseline, proper-noun phrases, and
    // identifiers (ICAOs, runway/taxiway names) are high-priority and
    // go out first. Component words split out of multi-word labels
    // (e.g. "Whiteman" from "Whiteman Tower") are lower-priority backup
    // — useful when the compound phrase mishears but not worth
    // crowding out a more specific term.
    let mut high = DedupList::new();
    let mut low = DedupList::new();

    for t in static_baseline(atc_mode) {
        high.push(t.to_string());
    }
    if !aircraft.is_empty() {
        high.push(aircraft);
    }

    if let Some(snap) = snapshot.as_ref() {
        for t in phase_terms(snap.phase) {
            high.push(t.to_string());
        }
    }

    // Fail-soft on the whole database path.
    if let Err(e) = append_database_terms(&mut high, &mut low, snapshot.as_ref(), bridge, ctx) {
        if let Some(bus) = &ctx.bus {
            bus.push_log(format!("voice: keyterm db skipped ({e})"));
        }
    }

    merge_tiers(high, low, MAX_TERMS)
}

fn merge_tiers(high: DedupList, low: DedupList, cap: usize) -> Vec<String> {
    let mut out = DedupList::new();
    for t in high.items {
        out.push(t);
    }
    for t in low.items {
        out.push(t);
    }
    out.into_vec(cap)
}

fn append_database_terms(
    high: &mut DedupList,
    low: &mut DedupList,
    snapshot: Option<&StatusSnapshot>,
    bridge: Option<&dyn ToolBridge>,
    ctx: &ToolContext,
) -> Result<()> {
    ensure_runway_conn(ctx)?;
    let guard = ctx.runway_conn.lock().unwrap();
    let conn = guard
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("runway_conn not initialized"))?;

    // 1. Nearby airports — use live bridge lat/lon first, falling back to
    //    the configured airport position via the parquet.
    let (lat, lon) = match bridge {
        Some(b) => (
            b.get_dataref_value(LATITUDE_DEG.name),
            b.get_dataref_value(LONGITUDE_DEG.name),
        ),
        None => (None, None),
    };
    if let (Some(lat), Some(lon)) = (lat, lon) {
        if let Ok(rows) = query_nearby_airports(conn, lat, lon) {
            for (ident, name) in rows {
                high.push(ident);
                push_phrase_and_words(high, low, &name);
            }
        }
    }

    // 2/3. Current-airport comms + runways + taxiways.
    let airport = snapshot
        .and_then(|s| s.airport_ident.clone())
        .or_else(|| ctx.config.lock().airport.airport.clone());
    if let Some(airport) = airport {
        if let Ok(labels) = query_comm_labels(conn, &airport) {
            for label in labels {
                push_phrase_and_words(high, low, &label);
            }
        }
        if let Ok(idents) = query_runway_idents(conn, &airport) {
            for r in idents {
                high.push(r);
            }
        }
        if let Ok(names) = query_taxiway_names(conn, &airport) {
            for n in names {
                high.push(n);
            }
        }
    }
    Ok(())
}

fn query_nearby_airports(
    conn: &duckdb::Connection,
    lat: f64,
    lon: f64,
) -> Result<Vec<(String, String)>> {
    let mut stmt = conn.prepare(
        "SELECT ident, name \
         FROM airports \
         WHERE arp IS NOT NULL \
           AND ST_Distance_Sphere(arp, ST_Point(?, ?)) <= ? \
         ORDER BY ST_Distance_Sphere(arp, ST_Point(?, ?)) \
         LIMIT ?",
    )?;
    let radius_m = NEARBY_AIRPORTS_RADIUS_NM * 1852.0;
    let mut rows = stmt.query(duckdb::params![
        lon,
        lat,
        radius_m,
        lon,
        lat,
        NEARBY_AIRPORTS_LIMIT
    ])?;
    let mut out = Vec::new();
    while let Some(row) = rows.next()? {
        let ident: Option<String> = row.get(0)?;
        let name: Option<String> = row.get(1)?;
        out.push((ident.unwrap_or_default(), name.unwrap_or_default()));
    }
    Ok(out)
}

fn query_comm_labels(conn: &duckdb::Connection, airport: &str) -> Result<Vec<String>> {
    let mut stmt = conn.prepare(
        "SELECT label FROM comms WHERE airport_ident = ? AND label IS NOT NULL",
    )?;
    let mut rows = stmt.query([airport])?;
    let mut out = Vec::new();
    while let Some(row) = rows.next()? {
        let label: Option<String> = row.get(0)?;
        if let Some(l) = label {
            out.push(l);
        }
    }
    Ok(out)
}

fn query_runway_idents(conn: &duckdb::Connection, airport: &str) -> Result<Vec<String>> {
    let mut stmt = conn.prepare(
        "SELECT le_ident, he_ident FROM runways WHERE airport_ident = ? AND closed = 0",
    )?;
    let mut rows = stmt.query([airport])?;
    let mut out = Vec::new();
    while let Some(row) = rows.next()? {
        let le: Option<String> = row.get(0)?;
        let he: Option<String> = row.get(1)?;
        if let Some(r) = le {
            out.push(r);
        }
        if let Some(r) = he {
            out.push(r);
        }
    }
    Ok(out)
}

fn query_taxiway_names(conn: &duckdb::Connection, airport: &str) -> Result<Vec<String>> {
    let mut stmt = conn.prepare(
        "SELECT DISTINCT name FROM taxi_edges WHERE airport_ident = ? AND name != ''",
    )?;
    let mut rows = stmt.query([airport])?;
    let mut out = Vec::new();
    while let Some(row) = rows.next()? {
        let n: Option<String> = row.get(0)?;
        if let Some(n) = n {
            out.push(n);
        }
    }
    Ok(out)
}

/// Split a label into its compound-phrase form (high priority) and its
/// component-word backups (low priority).
///
/// Deepgram's `keyterm` parameter biases multi-word phrases as
/// first-class entries (see docs: "Use an encoded space to combine
/// multiple keyterms into a single space-delimited value"), so
/// "SoCal Approach" as a single keyterm is more valuable than two
/// separate "SoCal" and "Approach" entries. The component words are
/// still useful backup when the compound mishears, but they're the
/// first thing we drop when we hit the term cap.
///
/// Phrases keep short connector words ("El" in "El Monte") because
/// they're part of the proper noun. Component-word backups still
/// filter sub-3-char fillers to avoid over-biasing common speech.
/// Single-word labels go straight to `high` (no backup tier).
fn push_phrase_and_words(high: &mut DedupList, low: &mut DedupList, label: &str) {
    let all_words: Vec<&str> = label
        .split(|c: char| !c.is_alphanumeric())
        .filter(|w| !w.is_empty())
        .collect();
    match all_words.len() {
        0 => {}
        1 => {
            if all_words[0].chars().count() >= MIN_WORD_LEN {
                high.push(all_words[0].to_string());
            }
        }
        _ => {
            high.push(all_words.join(" "));
            for w in all_words {
                if w.chars().count() >= MIN_WORD_LEN {
                    low.push(w.to_string());
                }
            }
        }
    }
}

fn static_baseline(atc_mode: bool) -> &'static [&'static str] {
    if atc_mode {
        &[
            "runway", "heading", "altitude", "squawk", "ident",
            "cleared", "takeoff", "landing", "approach", "departure",
            "tower", "ground", "unicom", "CTAF",
            "downwind", "base", "final", "crosswind", "upwind",
            "traffic", "pattern", "taxi", "hold", "short",
            "line", "wait", "contact", "Mayday",
        ]
    } else {
        &[
            "runway", "heading", "altitude", "flaps", "throttle",
            "taxi", "takeoff", "landing", "downwind", "base",
            "final", "squawk", "pattern",
        ]
    }
}

fn phase_terms(phase: Option<FlightPhase>) -> &'static [&'static str] {
    use FlightPhase::*;
    match phase {
        Some(Preflight) | Some(TaxiClear) => {
            &["taxi", "hold", "short", "ready", "ATIS"]
        }
        Some(TakeoffRoll) | Some(Rotate) | Some(InitialClimb) | Some(Crosswind) => {
            &["cleared", "takeoff", "rotate", "climb"]
        }
        Some(EnrouteClimb) | Some(Cruise) | Some(Descent) => {
            &["cruise", "squawk", "ident", "descend", "climb", "contact"]
        }
        Some(PatternEntry) | Some(Downwind) | Some(Base) | Some(Final)
        | Some(Roundout) | Some(Flare) => {
            &["cleared", "land", "final", "traffic", "pattern", "Mayday"]
        }
        Some(Rollout) | Some(RunwayExit) | Some(GoAround) | None => &[],
    }
}

/// Order-preserving case-insensitive deduping list.
struct DedupList {
    seen: HashSet<String>,
    items: Vec<String>,
}

impl DedupList {
    fn new() -> Self {
        Self {
            seen: HashSet::new(),
            items: Vec::new(),
        }
    }

    fn push(&mut self, term: String) {
        let term = term.trim().to_string();
        if term.is_empty() {
            return;
        }
        let key = term.to_ascii_lowercase();
        if self.seen.insert(key) {
            self.items.push(term);
        }
    }

    fn into_vec(self, cap: usize) -> Vec<String> {
        let mut items = self.items;
        items.truncate(cap);
        items
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Returns (high, low) tiers.
    fn run_push(label: &str) -> (Vec<String>, Vec<String>) {
        let mut h = DedupList::new();
        let mut l = DedupList::new();
        push_phrase_and_words(&mut h, &mut l, label);
        (h.into_vec(100), l.into_vec(100))
    }

    #[test]
    fn multi_word_label_puts_phrase_in_high_and_component_words_in_low() {
        let (high, low) = run_push("SoCal Approach");
        assert_eq!(high, vec!["SoCal Approach".to_string()]);
        assert!(low.contains(&"SoCal".to_string()));
        assert!(low.contains(&"Approach".to_string()));
    }

    #[test]
    fn phrase_keeps_short_connectors_low_tier_still_filters_them() {
        // "El Monte" → phrase goes high (keep "El"); solo "El" is
        // dropped from low because it's sub-MIN_WORD_LEN.
        let (high, low) = run_push("El Monte");
        assert_eq!(high, vec!["El Monte".to_string()]);
        assert_eq!(low, vec!["Monte".to_string()]);
    }

    #[test]
    fn single_word_label_goes_only_to_high() {
        let (high, low) = run_push("ATIS");
        assert_eq!(high, vec!["ATIS".to_string()]);
        assert!(low.is_empty());
    }

    #[test]
    fn merge_puts_high_first_and_caps_low_first() {
        let mut h = DedupList::new();
        h.push("A".to_string());
        h.push("B".to_string());
        h.push("C".to_string());
        let mut l = DedupList::new();
        l.push("X".to_string());
        l.push("Y".to_string());
        // Cap 4 → keep all of high (3), plus one of low.
        let out = merge_tiers(h, l, 4);
        assert_eq!(out, vec!["A", "B", "C", "X"]);
    }

    #[test]
    fn merge_dedups_low_against_high() {
        let mut h = DedupList::new();
        h.push("SoCal Approach".to_string());
        h.push("SoCal".to_string()); // already promoted somewhere
        let mut l = DedupList::new();
        l.push("SoCal".to_string()); // dup — should be skipped
        l.push("Approach".to_string());
        let out = merge_tiers(h, l, 100);
        // "SoCal" appears once (from high), not twice.
        assert_eq!(
            out,
            vec![
                "SoCal Approach".to_string(),
                "SoCal".to_string(),
                "Approach".to_string(),
            ]
        );
    }

    #[test]
    fn dedup_is_case_insensitive_and_preserves_first_casing() {
        let mut d = DedupList::new();
        d.push("Whiteman".to_string());
        d.push("whiteman".to_string());
        d.push("WHITEMAN".to_string());
        d.push("SoCal".to_string());
        let v = d.into_vec(10);
        assert_eq!(v, vec!["Whiteman".to_string(), "SoCal".to_string()]);
    }

    #[test]
    fn static_baseline_switches_on_atc_mode() {
        assert!(static_baseline(true).contains(&"cleared"));
        assert!(!static_baseline(false).contains(&"cleared"));
    }

    #[test]
    fn phase_terms_for_final_include_cleared_land() {
        let t = phase_terms(Some(FlightPhase::Final));
        assert!(t.contains(&"cleared"));
        assert!(t.contains(&"land"));
    }

    /// End-to-end: seed an in-memory DuckDB matching the schema that
    /// `open_apt_dat_parquet` exposes, then verify the DB-path branch
    /// emits the expected proper nouns.
    #[test]
    fn db_query_path_emits_airport_comm_runway_taxiway_terms() {
        let conn = duckdb::Connection::open_in_memory().unwrap();
        conn.execute_batch("INSTALL spatial; LOAD spatial;").unwrap();
        conn.execute_batch(
            "CREATE TABLE airports_base(ident VARCHAR, name VARCHAR, longitude_deg DOUBLE, latitude_deg DOUBLE);
             CREATE VIEW airports AS SELECT ident, name,
                 CASE WHEN latitude_deg IS NULL OR longitude_deg IS NULL
                      THEN NULL ELSE ST_Point(longitude_deg, latitude_deg) END AS arp
                 FROM airports_base;
             CREATE TABLE comms(airport_ident VARCHAR, label VARCHAR);
             CREATE TABLE runways(airport_ident VARCHAR, le_ident VARCHAR, he_ident VARCHAR, closed INTEGER);
             CREATE TABLE taxi_edges(airport_ident VARCHAR, name VARCHAR);",
        )
        .unwrap();
        conn.execute_batch(
            "INSERT INTO airports_base VALUES
                ('KWHP', 'Whiteman', -118.413, 34.259),
                ('KVNY', 'Van Nuys', -118.490, 34.209);
             INSERT INTO comms VALUES
                ('KWHP', 'Whiteman Tower'),
                ('KWHP', 'SoCal Approach');
             INSERT INTO runways VALUES
                ('KWHP', '12', '30', 0),
                ('KVNY', '16R', '34L', 0);
             INSERT INTO taxi_edges VALUES
                ('KWHP', 'A'),
                ('KWHP', 'alpha'),
                ('KWHP', '');",
        )
        .unwrap();

        // Center on KWHP so both airports fall within 30 NM.
        let rows = query_nearby_airports(&conn, 34.26, -118.41).unwrap();
        assert!(rows.iter().any(|(i, n)| i == "KWHP" && n == "Whiteman"));
        assert!(rows.iter().any(|(i, _)| i == "KVNY"));

        let labels = query_comm_labels(&conn, "KWHP").unwrap();
        assert!(labels.iter().any(|l| l == "Whiteman Tower"));
        assert!(labels.iter().any(|l| l == "SoCal Approach"));

        let rws = query_runway_idents(&conn, "KWHP").unwrap();
        assert!(rws.contains(&"12".to_string()));
        assert!(rws.contains(&"30".to_string()));

        let twys = query_taxiway_names(&conn, "KWHP").unwrap();
        assert!(twys.contains(&"A".to_string()));
        assert!(twys.contains(&"alpha".to_string()));
        assert!(!twys.iter().any(|n| n.is_empty()));
    }
}
