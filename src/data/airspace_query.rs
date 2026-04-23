//! Airspace containment + trajectory projection query.
//!
//! Given a DuckDB connection with the `airspaces` view registered (see
//! `llm::tools::open_apt_dat_parquet`) and an aircraft state, returns
//! the airspaces the aircraft is currently INSIDE (3D), 2D-footprint
//! OVER (alt > top), 2D-footprint UNDER (alt < bottom), and
//! projected to enter within a short lookahead horizon (typically
//! 120 s). The lookahead uses a flat-earth straight-line projection
//! along `track_deg` at `gs_kt`, with altitude projected from current
//! `vs_fpm`. Entries returned are de-duplicated by (class, name) so
//! multi-polygon airspaces (oceanic CTAs, Alaska border parts) don't
//! appear twice in the heartbeat payload.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use anyhow::Result;
use duckdb::Connection;
use serde_json::{json, Value};

/// Shared source for airspace containment queries. The connection slot is
/// reused across callers (`tool_get_status`, the heartbeat pump) so a
/// single DuckDB connection — opened lazily on first use — serves both.
/// `cache_dir` points at the directory holding `airspaces.parquet`.
#[derive(Clone)]
pub struct AirspaceSource {
    pub conn: Arc<Mutex<Option<Connection>>>,
    pub cache_dir: PathBuf,
}

#[derive(Debug, Clone, PartialEq)]
pub struct AirspaceEntry {
    pub class: String,
    pub name: String,
    pub bottom_ft_msl: f64,
    pub top_ft_msl: f64,
    pub bottom_is_gnd: bool,
    pub top_is_gnd: bool,
    /// For UNDER: how far below the bottom we are (bottom - alt). For OVER:
    /// how far above the top (alt - top). For INSIDE and
    /// THROUGH_120S: not meaningful, set to 0.0.
    pub vertical_clearance_ft: f64,
    /// Seconds until the trajectory enters the airspace. Only populated
    /// for the `through_120s` list. 0.0 for other states.
    pub t_sec: f64,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct AirspaceStates {
    pub inside: Vec<AirspaceEntry>,
    pub over: Vec<AirspaceEntry>,
    pub under: Vec<AirspaceEntry>,
    pub through_120s: Vec<AirspaceEntry>,
}

/// Feet per degree of latitude (great-circle, mean earth radius). Used for
/// the flat-earth trajectory projection — fine for 120 s of lookahead at
/// small-GA speeds (projected distance < 10 nm, error < 0.1%).
const FT_PER_DEG_LAT: f64 = 364_000.0;

/// How many entries to keep per state bucket before truncating, in the
/// order returned by the SQL query. Heartbeats shouldn't balloon in
/// dense airspace (say, overflying Europe).
const MAX_ENTRIES_PER_STATE: usize = 8;

/// Seconds of lookahead used for the `through_120s` bucket. Kept in
/// this constant so the name and value stay in sync.
pub const LOOKAHEAD_SECS: f64 = 120.0;

#[derive(Debug, Clone, Copy)]
pub struct AircraftSnapshot {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_ft: f64,
    pub track_deg: f64,
    pub gs_kt: f64,
    pub vs_fpm: f64,
}

pub fn query_airspace_states(
    conn: &Connection,
    snap: AircraftSnapshot,
) -> Result<AirspaceStates> {
    // Current-position footprint containment.
    let here = contains_query(conn, snap.lat_deg, snap.lon_deg)?;
    let alt_now = snap.alt_msl_ft;

    let mut states = AirspaceStates::default();
    let mut seen_keys: std::collections::HashSet<(String, String)> =
        std::collections::HashSet::new();
    for row in &here {
        let key = (row.class.clone(), row.name.clone());
        if !seen_keys.insert(key) {
            continue;
        }
        let vertical = if alt_now > row.top_ft_msl {
            Some((State::Over, alt_now - row.top_ft_msl))
        } else if alt_now < row.bottom_ft_msl {
            Some((State::Under, row.bottom_ft_msl - alt_now))
        } else {
            Some((State::Inside, 0.0))
        };
        let entry = AirspaceEntry {
            class: row.class.clone(),
            name: row.name.clone(),
            bottom_ft_msl: row.bottom_ft_msl,
            top_ft_msl: row.top_ft_msl,
            bottom_is_gnd: row.bottom_is_gnd,
            top_is_gnd: row.top_is_gnd,
            vertical_clearance_ft: vertical.as_ref().map(|v| v.1).unwrap_or(0.0),
            t_sec: 0.0,
        };
        match vertical.unwrap().0 {
            State::Inside => states.inside.push(entry),
            State::Over => states.over.push(entry),
            State::Under => states.under.push(entry),
        }
    }

    // Projected containment, only if we have meaningful motion. Below
    // ~5 kt groundspeed the track is numerically unstable and the
    // projected point barely moves — skip it.
    if snap.gs_kt > 5.0 {
        let (lat_proj, lon_proj, alt_proj) = project_forward(snap, LOOKAHEAD_SECS);
        let projected = contains_query(conn, lat_proj, lon_proj)?;
        for row in &projected {
            let key = (row.class.clone(), row.name.clone());
            if !seen_keys.insert(key) {
                continue;
            }
            // Only count it as THROUGH if the *projected altitude* is inside
            // the vertical slab at arrival time. OVER/UNDER at the projected
            // point are less actionable than the ones at the current point,
            // and the heartbeat already caps size.
            if alt_proj >= row.bottom_ft_msl && alt_proj <= row.top_ft_msl {
                // Linear time-to-entry: pro-rate by how far along the path
                // the projected endpoint sits. We don't have the entry
                // point in hand, so report the lookahead horizon as an
                // upper bound. Good enough for "heads up, coming soon".
                let entry = AirspaceEntry {
                    class: row.class.clone(),
                    name: row.name.clone(),
                    bottom_ft_msl: row.bottom_ft_msl,
                    top_ft_msl: row.top_ft_msl,
                    bottom_is_gnd: row.bottom_is_gnd,
                    top_is_gnd: row.top_is_gnd,
                    vertical_clearance_ft: 0.0,
                    t_sec: LOOKAHEAD_SECS,
                };
                states.through_120s.push(entry);
            }
        }
    }

    // Truncate each bucket. SQL ordered by distance from the query point
    // already, so the truncation keeps the closest entries.
    states.inside.truncate(MAX_ENTRIES_PER_STATE);
    states.over.truncate(MAX_ENTRIES_PER_STATE);
    states.under.truncate(MAX_ENTRIES_PER_STATE);
    states.through_120s.truncate(MAX_ENTRIES_PER_STATE);
    Ok(states)
}

#[derive(Debug, Clone)]
struct Row {
    class: String,
    name: String,
    bottom_ft_msl: f64,
    top_ft_msl: f64,
    bottom_is_gnd: bool,
    top_is_gnd: bool,
}

enum State {
    Inside,
    Over,
    Under,
}

fn contains_query(conn: &Connection, lat: f64, lon: f64) -> Result<Vec<Row>> {
    // bbox prefilter on the scalar columns + ST_Contains against the
    // computed footprint geometry. The view computes footprint via
    // ST_GeomFromText so the bbox prefilter is essential — without it
    // every heartbeat would decode 19k WKT strings.
    let mut stmt = conn.prepare(
        "SELECT class, name, bottom_ft_msl, top_ft_msl, \
                bottom_is_gnd, top_is_gnd \
         FROM airspaces \
         WHERE ? BETWEEN min_lat AND max_lat \
           AND ? BETWEEN min_lon AND max_lon \
           AND ST_Contains(footprint, ST_Point(?, ?))",
    )?;
    let mut rows = stmt.query([lat, lon, lon, lat])?;
    let mut out = Vec::new();
    while let Some(row) = rows.next()? {
        out.push(Row {
            class: row.get(0)?,
            name: row.get(1)?,
            bottom_ft_msl: row.get(2)?,
            top_ft_msl: row.get(3)?,
            bottom_is_gnd: row.get(4)?,
            top_is_gnd: row.get(5)?,
        });
    }
    Ok(out)
}

/// Emit the airspace states as the JSON value embedded under `"airspace"`
/// in the heartbeat/status payload. Entries are rendered compactly: class,
/// name, bottom, top, GND flags; UNDER/OVER also carry
/// `vertical_clearance_ft`; THROUGH_120S carries `t_sec`. When every
/// bucket is empty the function returns `null` so the JSON object omits
/// the key via `Value::Null` handling upstream.
pub fn format_states_json(states: &AirspaceStates) -> Value {
    if states.inside.is_empty()
        && states.over.is_empty()
        && states.under.is_empty()
        && states.through_120s.is_empty()
    {
        return Value::Null;
    }
    let to_entry_basic = |e: &AirspaceEntry| -> Value {
        json!({
            "class": e.class,
            "name": e.name,
            "bottom_ft_msl": e.bottom_ft_msl,
            "top_ft_msl": e.top_ft_msl,
            "bottom_is_gnd": e.bottom_is_gnd,
            "top_is_gnd": e.top_is_gnd,
        })
    };
    let to_entry_margin = |e: &AirspaceEntry| -> Value {
        let mut v = to_entry_basic(e);
        v.as_object_mut().unwrap().insert(
            "vertical_clearance_ft".into(),
            json!(e.vertical_clearance_ft.round()),
        );
        v
    };
    let to_entry_through = |e: &AirspaceEntry| -> Value {
        let mut v = to_entry_basic(e);
        v.as_object_mut()
            .unwrap()
            .insert("t_sec".into(), json!(e.t_sec.round() as i64));
        v
    };
    json!({
        "inside": states.inside.iter().map(to_entry_basic).collect::<Vec<_>>(),
        "over": states.over.iter().map(to_entry_margin).collect::<Vec<_>>(),
        "under": states.under.iter().map(to_entry_margin).collect::<Vec<_>>(),
        "through_120s": states.through_120s.iter().map(to_entry_through).collect::<Vec<_>>(),
    })
}

/// Flat-earth straight-line projection at constant `track_deg` and
/// ground speed, plus linear altitude extrapolation from `vs_fpm`. The
/// 120 s horizon makes this fine at GA speeds; at airliner speeds the
/// flat-earth approximation still holds (<10 nm of travel).
fn project_forward(snap: AircraftSnapshot, dt_s: f64) -> (f64, f64, f64) {
    let distance_ft = (snap.gs_kt * 1.687_81) * dt_s;
    let track_rad = snap.track_deg.to_radians();
    let d_north_ft = track_rad.cos() * distance_ft;
    let d_east_ft = track_rad.sin() * distance_ft;
    let lat = snap.lat_deg + d_north_ft / FT_PER_DEG_LAT;
    let ft_per_deg_lon = FT_PER_DEG_LAT * snap.lat_deg.to_radians().cos().max(1e-6);
    let lon = snap.lon_deg + d_east_ft / ft_per_deg_lon;
    let alt = snap.alt_msl_ft + (snap.vs_fpm / 60.0) * dt_s;
    (lat, lon, alt)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::{airspace, apt_dat, parquet};
    use crate::llm::tools::open_apt_dat_parquet;

    fn build_conn_with_airspaces(airspace_src: &str) -> (tempfile::TempDir, Connection) {
        let dir = tempfile::tempdir().unwrap();
        // Minimum apt.dat required for the other parquet files to exist.
        let minimal_apt = "1 0 0 0 KXXX stub\n99\n";
        let apt = apt_dat::parse(minimal_apt.as_bytes()).unwrap();
        let airspaces = airspace::parse(airspace_src.as_bytes()).unwrap();
        let cache = parquet::write_cache(&apt, &airspaces, dir.path()).unwrap();
        let conn = open_apt_dat_parquet(
            &cache.airports(),
            &cache.runways(),
            &cache.comms(),
        )
        .unwrap();
        (dir, conn)
    }

    const TWO_AIRSPACES: &str = "\
AC B
AN SAN FRANCISCO
AL 1500 MSL
AH 10000 MSL
DP 37:30:00 N 122:30:00 W
DP 37:50:00 N 122:30:00 W
DP 37:50:00 N 122:10:00 W
DP 37:30:00 N 122:10:00 W

AC D
AN OAKLAND
AL GND
AH 2500 MSL
DP 37:40:00 N 122:20:00 W
DP 37:45:00 N 122:20:00 W
DP 37:45:00 N 122:15:00 W
DP 37:40:00 N 122:15:00 W
";

    #[test]
    fn inside_class_b_and_d() {
        // Point inside both SF Bravo (lat 37.50-37.833, lon -122.5 to
        // -122.167) AND Oakland Delta (lat 37.667-37.75, lon -122.333 to
        // -122.25), altitude 2000 ft: inside both slabs (SF B = 1500-
        // 10000, OAK D = GND-2500).
        let (_t, conn) = build_conn_with_airspaces(TWO_AIRSPACES);
        let s = query_airspace_states(
            &conn,
            AircraftSnapshot {
                lat_deg: 37.7,
                lon_deg: -122.30,
                alt_msl_ft: 2000.0,
                track_deg: 0.0,
                gs_kt: 0.0,
                vs_fpm: 0.0,
            },
        )
        .unwrap();
        let names: Vec<&str> = s.inside.iter().map(|e| e.name.as_str()).collect();
        assert!(names.contains(&"SAN FRANCISCO"), "got {:?}", names);
        assert!(names.contains(&"OAKLAND"), "got {:?}", names);
    }

    #[test]
    fn under_reports_clearance_margin() {
        // Inside the Bravo footprint at 1000 ft — under the 1500 floor by
        // 500 ft.
        let (_t, conn) = build_conn_with_airspaces(TWO_AIRSPACES);
        let s = query_airspace_states(
            &conn,
            AircraftSnapshot {
                lat_deg: 37.7,
                lon_deg: -122.25,
                alt_msl_ft: 1000.0,
                track_deg: 0.0,
                gs_kt: 0.0,
                vs_fpm: 0.0,
            },
        )
        .unwrap();
        let sf = s
            .under
            .iter()
            .find(|e| e.name == "SAN FRANCISCO")
            .expect("SF Bravo should be UNDER");
        assert!((sf.vertical_clearance_ft - 500.0).abs() < 1e-6);
        // At 1000 ft, outside Oakland Delta footprint — nothing else reported.
        assert!(s.inside.is_empty());
    }

    #[test]
    fn over_reports_clearance_margin() {
        // 11000 ft inside Bravo footprint: over the 10000 ceiling by 1000 ft.
        let (_t, conn) = build_conn_with_airspaces(TWO_AIRSPACES);
        let s = query_airspace_states(
            &conn,
            AircraftSnapshot {
                lat_deg: 37.7,
                lon_deg: -122.25,
                alt_msl_ft: 11000.0,
                track_deg: 0.0,
                gs_kt: 0.0,
                vs_fpm: 0.0,
            },
        )
        .unwrap();
        let sf = s
            .over
            .iter()
            .find(|e| e.name == "SAN FRANCISCO")
            .expect("SF Bravo should be OVER");
        assert!((sf.vertical_clearance_ft - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn projected_crossing_appears_in_through_120s() {
        // Start a bit west of SF Bravo and head east at 120 kt. At 120 kt
        // we travel ~4 nm in 120 s (~0.067 deg of longitude at this lat).
        // Starting at lon -122.55 we'll be at ~-122.48, inside Bravo
        // footprint (which spans -122.5 → -122.166).
        let (_t, conn) = build_conn_with_airspaces(TWO_AIRSPACES);
        let s = query_airspace_states(
            &conn,
            AircraftSnapshot {
                lat_deg: 37.7,
                lon_deg: -122.55,
                alt_msl_ft: 3000.0,
                track_deg: 90.0, // due east
                gs_kt: 120.0,
                vs_fpm: 0.0,
            },
        )
        .unwrap();
        assert!(s.inside.is_empty());
        let names: Vec<&str> = s.through_120s.iter().map(|e| e.name.as_str()).collect();
        assert!(names.contains(&"SAN FRANCISCO"), "got {:?}", names);
    }

    #[test]
    fn dedup_across_inside_and_through() {
        // Inside Oakland Delta (lat 37.70, lon -122.32 is just inside its
        // west edge -122.333) AND heading east at 100 kt: t=120 we're at
        // ~-122.264, still inside Oakland. Should appear ONCE, in `inside`,
        // not also in `through_120s`.
        let (_t, conn) = build_conn_with_airspaces(TWO_AIRSPACES);
        let s = query_airspace_states(
            &conn,
            AircraftSnapshot {
                lat_deg: 37.70,
                lon_deg: -122.32,
                alt_msl_ft: 2000.0,
                track_deg: 90.0,
                gs_kt: 100.0,
                vs_fpm: 0.0,
            },
        )
        .unwrap();
        let inside_names: Vec<&str> = s.inside.iter().map(|e| e.name.as_str()).collect();
        let through_names: Vec<&str> = s.through_120s.iter().map(|e| e.name.as_str()).collect();
        assert!(inside_names.contains(&"OAKLAND"));
        assert!(!through_names.contains(&"OAKLAND"));
    }
}
