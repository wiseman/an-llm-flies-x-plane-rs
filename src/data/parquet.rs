//! apt.dat → zstd GeoParquet cache. No Python counterpart.
//!
//! Builds three parquet files (`airports.parquet`, `runways.parquet`,
//! `comms.parquet`) in a per-apt.dat cache directory under
//! `~/.cache/sim_pilot/apt-<hash>/`. The full global build takes ~0.8 s once
//! (parse + sort + write); subsequent runs open the cache in <10 ms. The
//! cache is transparently rebuilt when apt.dat's mtime advances.
//!
//! Row order in each parquet is spatially clustered via `ST_Hilbert` at
//! write time, so DuckDB's parquet row-group bounding-box statistics give
//! efficient predicate pushdown on later spatial filters.
//!
//! Only scalar columns are stored in the parquet files (lat/lon doubles,
//! idents, surface, etc.). The GEOMETRY columns the LLM queries —
//! `airports.arp`, `runways.centerline` / `le_threshold` / `he_threshold`
//! — are added back as computed expressions in the runtime views that
//! `llm/tools.rs` opens. That keeps the parquet files portable and avoids
//! a duckdb-rs crash on `SELECT *` against GEOMETRY columns round-tripped
//! through parquet.

use std::collections::hash_map::DefaultHasher;
use std::fs;
use std::hash::{Hash, Hasher};
use std::path::{Path, PathBuf};
use std::time::SystemTime;

use anyhow::{anyhow, Context, Result};
use duckdb::{params, Connection};

use crate::data::{airspace, apt_dat};

/// Names of the parquet files inside a cache directory. Exposed so
/// `llm/tools.rs` can create views without hardcoding strings.
pub const AIRPORTS_FILE: &str = "airports.parquet";
pub const RUNWAYS_FILE: &str = "runways.parquet";
pub const COMMS_FILE: &str = "comms.parquet";
pub const TAXI_NODES_FILE: &str = "taxi_nodes.parquet";
pub const TAXI_EDGES_FILE: &str = "taxi_edges.parquet";
pub const AIRSPACES_FILE: &str = "airspaces.parquet";

pub struct AptDatCache {
    pub dir: PathBuf,
}

impl AptDatCache {
    pub fn airports(&self) -> PathBuf {
        self.dir.join(AIRPORTS_FILE)
    }
    pub fn runways(&self) -> PathBuf {
        self.dir.join(RUNWAYS_FILE)
    }
    pub fn comms(&self) -> PathBuf {
        self.dir.join(COMMS_FILE)
    }
    pub fn taxi_nodes(&self) -> PathBuf {
        self.dir.join(TAXI_NODES_FILE)
    }
    pub fn taxi_edges(&self) -> PathBuf {
        self.dir.join(TAXI_EDGES_FILE)
    }
    pub fn airspaces(&self) -> PathBuf {
        self.dir.join(AIRSPACES_FILE)
    }
    pub fn apt_dat_files(&self) -> [PathBuf; 5] {
        [
            self.airports(),
            self.runways(),
            self.comms(),
            self.taxi_nodes(),
            self.taxi_edges(),
        ]
    }
    pub fn apt_dat_files_exist(&self) -> bool {
        self.apt_dat_files().iter().all(|p| p.exists())
    }
}

/// Return a cache for `apt_dat`, rebuilding it if missing or older than
/// any source file. If `airspace_txt` is `Some`, its rows are parsed into
/// `airspaces.parquet` alongside the apt.dat outputs and its mtime is
/// included in the freshness check; otherwise `airspaces.parquet` is not
/// produced. This is the single entry point callers outside this module
/// should use.
pub fn resolve(apt_dat: &Path, airspace_txt: Option<&Path>) -> Result<AptDatCache> {
    if !apt_dat.exists() {
        return Err(anyhow!("apt.dat not found at {}", apt_dat.display()));
    }
    if let Some(p) = airspace_txt {
        if !p.exists() {
            return Err(anyhow!("airspace.txt not found at {}", p.display()));
        }
    }
    let cache = cache_for(apt_dat)?;
    if cache_is_fresh(apt_dat, airspace_txt, &cache)? {
        return Ok(cache);
    }
    fs::create_dir_all(&cache.dir)
        .with_context(|| format!("creating {}", cache.dir.display()))?;
    build(apt_dat, airspace_txt, &cache)
        .with_context(|| format!("building apt.dat parquet cache at {}", cache.dir.display()))?;
    Ok(cache)
}

/// Bump this whenever the parquet schema changes (column names / types /
/// added or removed files) in an incompatible way. Folded into the cache
/// directory hash, so old caches are transparently ignored and rebuilt
/// rather than producing SQL errors against stale columns.
const SCHEMA_VERSION: u64 = 2;

fn cache_for(apt_dat: &Path) -> Result<AptDatCache> {
    let home = std::env::var_os("HOME")
        .ok_or_else(|| anyhow!("HOME is unset; cannot locate cache directory"))?;
    let canon = fs::canonicalize(apt_dat).unwrap_or_else(|_| apt_dat.to_path_buf());
    let mut hasher = DefaultHasher::new();
    SCHEMA_VERSION.hash(&mut hasher);
    canon.hash(&mut hasher);
    let h = hasher.finish();
    let dir = PathBuf::from(home)
        .join(".cache/sim_pilot")
        .join(format!("apt-{:016x}", h));
    Ok(AptDatCache { dir })
}

fn cache_is_fresh(
    apt_dat: &Path,
    airspace_txt: Option<&Path>,
    cache: &AptDatCache,
) -> Result<bool> {
    if !cache.apt_dat_files_exist() {
        return Ok(false);
    }
    if airspace_txt.is_some() && !cache.airspaces().exists() {
        return Ok(false);
    }
    let mut sources = vec![apt_dat];
    if let Some(a) = airspace_txt {
        sources.push(a);
    }
    let src_latest = sources
        .iter()
        .map(|p| {
            fs::metadata(p)
                .and_then(|m| m.modified())
                .unwrap_or(SystemTime::UNIX_EPOCH)
        })
        .max()
        .unwrap_or(SystemTime::UNIX_EPOCH);
    let mut outs = cache.apt_dat_files().to_vec();
    if airspace_txt.is_some() {
        outs.push(cache.airspaces());
    }
    for f in &outs {
        let dst = fs::metadata(f)?.modified().unwrap_or(SystemTime::UNIX_EPOCH);
        if dst < src_latest {
            return Ok(false);
        }
    }
    Ok(true)
}

fn build(apt_dat: &Path, airspace_txt: Option<&Path>, cache: &AptDatCache) -> Result<()> {
    let parsed = apt_dat::parse_file(apt_dat)?;
    let airspaces = match airspace_txt {
        Some(p) => airspace::parse_file(p)?,
        None => Vec::new(),
    };
    write_parquets(&parsed, &airspaces, airspace_txt.is_some(), cache)
}

/// Public entry point for tests: feed the builder a pre-parsed apt.dat dump
/// and write parquet into an arbitrary directory. Used by
/// `tests/test_tool_dispatch.rs` to stand up a fixture without depending on
/// the filesystem layout or `HOME`. Airspaces are optional — pass `&[]` to
/// skip the airspace parquet.
pub fn write_cache(
    parsed: &apt_dat::ParsedAptDat,
    airspaces: &[airspace::ParsedAirspace],
    dir: &Path,
) -> Result<AptDatCache> {
    fs::create_dir_all(dir)?;
    let cache = AptDatCache { dir: dir.to_path_buf() };
    write_parquets(parsed, airspaces, !airspaces.is_empty(), &cache)?;
    Ok(cache)
}

fn write_parquets(
    parsed: &apt_dat::ParsedAptDat,
    airspaces: &[airspace::ParsedAirspace],
    write_airspaces: bool,
    cache: &AptDatCache,
) -> Result<()> {
    let conn = Connection::open_in_memory()?;
    conn.execute_batch("INSTALL spatial; LOAD spatial;")?;
    create_airport_table(&conn, &parsed.airports)?;
    create_runway_table(&conn, &parsed.runways)?;
    create_comm_table(&conn, &parsed.comms)?;
    create_taxi_node_table(&conn, &parsed.taxi_nodes)?;
    create_taxi_edge_table(&conn, &parsed.taxi_edges)?;
    copy_to_parquet(&conn, "airports", &cache.airports())?;
    copy_to_parquet(&conn, "runways", &cache.runways())?;
    copy_to_parquet(&conn, "comms", &cache.comms())?;
    copy_to_parquet(&conn, "taxi_nodes", &cache.taxi_nodes())?;
    copy_to_parquet(&conn, "taxi_edges", &cache.taxi_edges())?;
    if write_airspaces {
        create_airspace_table(&conn, airspaces)?;
        copy_to_parquet(&conn, "airspaces", &cache.airspaces())?;
    }
    Ok(())
}

fn copy_to_parquet(conn: &Connection, table: &str, path: &Path) -> Result<()> {
    // Write via a temp file + rename so a crashed build never leaves the
    // cache in a half-valid state. row_group_size of 4096 keeps bbox stats
    // tight enough for predicate pushdown.
    let tmp = path.with_extension("parquet.tmp");
    let sql = format!(
        "COPY {table} TO '{}' (FORMAT PARQUET, COMPRESSION zstd, ROW_GROUP_SIZE 4096);",
        tmp.display()
    );
    conn.execute_batch(&sql)?;
    fs::rename(&tmp, path)?;
    Ok(())
}

fn create_airport_table(conn: &Connection, airports: &[apt_dat::ParsedAirport]) -> Result<()> {
    conn.execute_batch(
        "CREATE TABLE airports_raw (
            ident VARCHAR,
            name VARCHAR,
            elevation_ft DOUBLE,
            icao_code VARCHAR,
            iata_code VARCHAR,
            faa_code VARCHAR,
            latitude_deg DOUBLE,
            longitude_deg DOUBLE
        );",
    )?;
    {
        let mut app = conn.appender("airports_raw")?;
        for a in airports {
            app.append_row(params![
                a.ident,
                a.name,
                a.elevation_ft,
                a.icao_code.as_deref(),
                a.iata_code.as_deref(),
                a.faa_code.as_deref(),
                a.latitude_deg,
                a.longitude_deg,
            ])?;
        }
        app.flush()?;
    }
    conn.execute_batch(
        "CREATE TABLE airports AS
         SELECT ident, name, elevation_ft, icao_code, iata_code, faa_code,
                latitude_deg, longitude_deg
         FROM airports_raw
         ORDER BY CASE WHEN latitude_deg IS NULL OR longitude_deg IS NULL THEN 1 ELSE 0 END,
                  ST_Hilbert(
                      COALESCE(ST_Point(longitude_deg, latitude_deg), ST_Point(0, 0)),
                      ST_Extent(ST_MakeEnvelope(-180, -90, 180, 90))
                  );",
    )?;
    Ok(())
}

fn create_runway_table(conn: &Connection, runways: &[apt_dat::ParsedRunway]) -> Result<()> {
    conn.execute_batch(
        "CREATE TABLE runways_raw (
            id BIGINT,
            airport_ident VARCHAR,
            length_ft DOUBLE,
            width_ft DOUBLE,
            surface VARCHAR,
            lighted TINYINT,
            closed TINYINT,
            le_ident VARCHAR,
            le_latitude_deg DOUBLE,
            le_longitude_deg DOUBLE,
            le_elevation_ft DOUBLE,
            le_heading_degT DOUBLE,
            le_displaced_threshold_ft DOUBLE,
            he_ident VARCHAR,
            he_latitude_deg DOUBLE,
            he_longitude_deg DOUBLE,
            he_elevation_ft DOUBLE,
            he_heading_degT DOUBLE,
            he_displaced_threshold_ft DOUBLE
        );",
    )?;
    {
        let mut app = conn.appender("runways_raw")?;
        for (i, r) in runways.iter().enumerate() {
            let id = (i + 1) as i64;
            app.append_row(params![
                id,
                r.airport_ident,
                r.length_ft,
                r.width_ft,
                r.surface,
                r.lighted as i8,
                r.closed as i8,
                r.le_ident,
                r.le_lat,
                r.le_lon,
                r.le_elevation_ft,
                r.le_heading_deg,
                r.le_displaced_threshold_ft,
                r.he_ident,
                r.he_lat,
                r.he_lon,
                r.he_elevation_ft,
                r.he_heading_deg,
                r.he_displaced_threshold_ft,
            ])?;
        }
        app.flush()?;
    }
    conn.execute_batch(
        "CREATE TABLE runways AS
         SELECT id, airport_ident, length_ft, width_ft, surface, lighted, closed,
                le_ident, le_latitude_deg, le_longitude_deg, le_elevation_ft,
                le_heading_degT, le_displaced_threshold_ft,
                he_ident, he_latitude_deg, he_longitude_deg, he_elevation_ft,
                he_heading_degT, he_displaced_threshold_ft
         FROM runways_raw
         ORDER BY ST_Hilbert(
                      ST_Point(
                          (le_longitude_deg + he_longitude_deg) * 0.5,
                          (le_latitude_deg  + he_latitude_deg ) * 0.5
                      ),
                      ST_Extent(ST_MakeEnvelope(-180, -90, 180, 90))
                  );",
    )?;
    Ok(())
}

fn create_taxi_node_table(conn: &Connection, nodes: &[apt_dat::ParsedTaxiNode]) -> Result<()> {
    conn.execute_batch(
        "CREATE TABLE taxi_nodes (
            airport_ident VARCHAR,
            node_id INTEGER,
            latitude_deg DOUBLE,
            longitude_deg DOUBLE,
            usage VARCHAR
        );",
    )?;
    let mut app = conn.appender("taxi_nodes")?;
    for n in nodes {
        app.append_row(params![
            n.airport_ident,
            n.node_id as i32,
            n.latitude_deg,
            n.longitude_deg,
            n.usage,
        ])?;
    }
    app.flush()?;
    Ok(())
}

fn create_taxi_edge_table(conn: &Connection, edges: &[apt_dat::ParsedTaxiEdge]) -> Result<()> {
    // active_zones is serialised as a semicolon-delimited "kind:runways"
    // string (e.g. "departure:10L,28R;ils:28L"). Empty when the edge has
    // no runway conflicts. Phase-two code can normalize into a separate
    // table if query ergonomics demand it.
    conn.execute_batch(
        "CREATE TABLE taxi_edges (
            airport_ident VARCHAR,
            from_node INTEGER,
            to_node INTEGER,
            direction VARCHAR,
            category VARCHAR,
            name VARCHAR,
            active_zones VARCHAR
        );",
    )?;
    let mut app = conn.appender("taxi_edges")?;
    for e in edges {
        let zones = if e.active_zones.is_empty() {
            String::new()
        } else {
            e.active_zones
                .iter()
                .map(|z| format!("{}:{}", z.kind, z.runways))
                .collect::<Vec<_>>()
                .join(";")
        };
        app.append_row(params![
            e.airport_ident,
            e.from_node as i32,
            e.to_node as i32,
            e.direction,
            e.category,
            e.name,
            zones,
        ])?;
    }
    app.flush()?;
    Ok(())
}

fn create_airspace_table(
    conn: &Connection,
    airspaces: &[airspace::ParsedAirspace],
) -> Result<()> {
    // Polygon stored as WKT text — no GEOMETRY columns on disk (keeps parquet
    // portable and sidesteps the duckdb-rs panic on round-tripped GEOMETRY).
    // The view in llm/tools.rs rebuilds the geometry via ST_GeomFromText.
    // bbox columns are stored as scalars for predicate pushdown before the
    // spatial contains check.
    conn.execute_batch(
        "CREATE TABLE airspaces_raw (
            id BIGINT,
            class VARCHAR,
            name VARCHAR,
            bottom_ft_msl DOUBLE,
            top_ft_msl DOUBLE,
            bottom_is_gnd BOOLEAN,
            top_is_gnd BOOLEAN,
            min_lat DOUBLE,
            max_lat DOUBLE,
            min_lon DOUBLE,
            max_lon DOUBLE,
            polygon_wkt VARCHAR
        );",
    )?;
    {
        let mut app = conn.appender("airspaces_raw")?;
        for (i, a) in airspaces.iter().enumerate() {
            let id = (i + 1) as i64;
            let wkt = polygon_wkt(&a.vertices);
            app.append_row(params![
                id,
                a.class,
                a.name,
                a.bottom_ft_msl,
                a.top_ft_msl,
                a.bottom_is_gnd,
                a.top_is_gnd,
                a.min_lat,
                a.max_lat,
                a.min_lon,
                a.max_lon,
                wkt,
            ])?;
        }
        app.flush()?;
    }
    conn.execute_batch(
        "CREATE TABLE airspaces AS
         SELECT id, class, name, bottom_ft_msl, top_ft_msl,
                bottom_is_gnd, top_is_gnd,
                min_lat, max_lat, min_lon, max_lon, polygon_wkt
         FROM airspaces_raw
         ORDER BY ST_Hilbert(
                      ST_Point(
                          (min_lon + max_lon) * 0.5,
                          (min_lat + max_lat) * 0.5
                      ),
                      ST_Extent(ST_MakeEnvelope(-180, -90, 180, 90))
                  );",
    )?;
    Ok(())
}

fn polygon_wkt(vertices: &[(f64, f64)]) -> String {
    let mut s = String::with_capacity(vertices.len() * 32);
    s.push_str("POLYGON ((");
    for (i, (lat, lon)) in vertices.iter().enumerate() {
        if i > 0 {
            s.push_str(", ");
        }
        // WKT order is (lon lat). Use a fixed-precision format so the file
        // is reproducible.
        s.push_str(&format!("{:.7} {:.7}", lon, lat));
    }
    s.push_str("))");
    s
}

fn create_comm_table(conn: &Connection, comms: &[apt_dat::ParsedComm]) -> Result<()> {
    conn.execute_batch(
        "CREATE TABLE comms (
            airport_ident VARCHAR,
            code SMALLINT,
            kind VARCHAR,
            freq_mhz DOUBLE,
            label VARCHAR
        );",
    )?;
    let mut app = conn.appender("comms")?;
    for c in comms {
        app.append_row(params![
            c.airport_ident,
            c.code as i16,
            c.kind,
            c.freq_mhz,
            c.label,
        ])?;
    }
    app.flush()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::apt_dat::parse;

    const FIXTURE: &str = "\
1     429 0 0 KSEA Seattle-Tacoma Intl
1302 datum_lat 47.449888889
1302 datum_lon -122.311777778
1054 119900 TWR
100 45.72 1 0 0.25 1 2 0 16L 47.46380000 -122.30800000 0 60 2 1 1 2 34R 47.43130000 -122.30800000 0 60 2 1 1 2
1201 47.44990 -122.31180 both 1
1201 47.44500 -122.31180 both 2
1202 1 2 twoway taxiway_E A
1204 departure 16L

1     13 0 0 KSFO San Francisco Intl
100 61.00 1 0 0.0 1 3 0 10L 37.62875970 -122.39343890 0 250 3 0 0 3 28R 37.61353400 -122.35715510 90 100 3 2 1 3
99
";

    fn build_fixture_cache() -> (tempfile::TempDir, AptDatCache) {
        let dir = tempfile::tempdir().unwrap();
        let parsed = parse(FIXTURE.as_bytes()).unwrap();
        let cache = write_cache(&parsed, &[], dir.path()).unwrap();
        (dir, cache)
    }

    #[test]
    fn parquet_cache_has_all_files() {
        let (_tmp, cache) = build_fixture_cache();
        for f in cache.apt_dat_files() {
            assert!(f.exists(), "missing {}", f.display());
        }
        // airspace parquet is opt-in; the default fixture passes none, so
        // that file should be absent.
        assert!(!cache.airspaces().exists());
    }

    #[test]
    #[ignore = "requires the real X-Plane 12 install; run with `cargo test -- --ignored`"]
    fn resolve_builds_full_cache_against_real_xplane_install() {
        let apt = crate::data::apt_dat::default_apt_dat_path()
            .expect("X-Plane 12 install detected");
        let airspace = crate::data::airspace::default_airspace_txt_path()
            .expect("airspace.txt present next to apt.dat");
        let cache = resolve(&apt, Some(airspace.as_path())).unwrap();
        for f in cache.apt_dat_files() {
            assert!(f.exists(), "missing {}", f.display());
        }
        assert!(cache.airspaces().exists(), "airspaces parquet not built");
        // Sanity-check airspace row count lands in the expected ballpark
        // (19,382 after dropping Q at parse time on the shipped apt.dat).
        let conn = Connection::open_in_memory().unwrap();
        let n: i64 = conn
            .query_row(
                &format!(
                    "SELECT COUNT(*) FROM read_parquet('{}')",
                    cache.airspaces().display()
                ),
                [],
                |r| r.get(0),
            )
            .unwrap();
        assert!(n > 10_000, "airspace count suspiciously low: {}", n);
    }

    #[test]
    fn airspaces_parquet_round_trips() {
        use crate::data::airspace;
        let dir = tempfile::tempdir().unwrap();
        let parsed = parse(FIXTURE.as_bytes()).unwrap();
        let airspace_fixture = "\
AC B
AN SAN FRANCISCO
AL 1500 MSL
AH 10000 MSL
DP 37:30:00 N 122:30:00 W
DP 37:50:00 N 122:30:00 W
DP 37:50:00 N 122:10:00 W
DP 37:30:00 N 122:10:00 W
";
        let airspaces = airspace::parse(airspace_fixture.as_bytes()).unwrap();
        let cache = write_cache(&parsed, &airspaces, dir.path()).unwrap();
        assert!(cache.airspaces().exists());

        let conn = Connection::open_in_memory().unwrap();
        conn.execute_batch("INSTALL spatial; LOAD spatial;").unwrap();
        let (class, name, floor, ceiling): (String, String, f64, f64) = conn
            .query_row(
                &format!(
                    "SELECT class, name, bottom_ft_msl, top_ft_msl \
                     FROM read_parquet('{}')",
                    cache.airspaces().display()
                ),
                [],
                |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?)),
            )
            .unwrap();
        assert_eq!(class, "B");
        assert_eq!(name, "SAN FRANCISCO");
        assert_eq!(floor, 1500.0);
        assert_eq!(ceiling, 10000.0);

        // WKT polygon should decode back to a 5-vertex closed ring (4 input
        // + 1 closure) whose bbox matches the airspace bbox columns.
        let (poly_wkt, n_points): (String, i64) = conn
            .query_row(
                &format!(
                    "SELECT polygon_wkt, \
                            ST_NPoints(ST_GeomFromText(polygon_wkt)) \
                     FROM read_parquet('{}')",
                    cache.airspaces().display()
                ),
                [],
                |r| Ok((r.get(0)?, r.get(1)?)),
            )
            .unwrap();
        assert!(poly_wkt.starts_with("POLYGON"));
        assert_eq!(n_points, 5);
    }

    #[test]
    fn airports_parquet_round_trips_scalar_schema() {
        let (_tmp, cache) = build_fixture_cache();
        let conn = Connection::open_in_memory().unwrap();
        let n: i64 = conn
            .query_row(
                &format!(
                    "SELECT COUNT(*) FROM read_parquet('{}')",
                    cache.airports().display()
                ),
                [],
                |r| r.get(0),
            )
            .unwrap();
        assert_eq!(n, 2);
        let (ident, lat, lon): (String, f64, f64) = conn
            .query_row(
                &format!(
                    "SELECT ident, latitude_deg, longitude_deg \
                     FROM read_parquet('{}') WHERE ident = 'KSEA'",
                    cache.airports().display()
                ),
                [],
                |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)),
            )
            .unwrap();
        assert_eq!(ident, "KSEA");
        assert!((lat - 47.449888889).abs() < 1e-6);
        assert!((lon - -122.311777778).abs() < 1e-6);
    }

    #[test]
    fn runways_parquet_preserves_le_he_schema() {
        let (_tmp, cache) = build_fixture_cache();
        let conn = Connection::open_in_memory().unwrap();
        conn.execute_batch("INSTALL spatial; LOAD spatial;").unwrap();
        let sql = format!(
            "SELECT airport_ident, le_ident, he_ident, length_ft, le_heading_degT \
             FROM read_parquet('{}') WHERE airport_ident = 'KSFO'",
            cache.runways().display()
        );
        let (ai, le, he, len, hdg): (String, String, String, f64, f64) = conn
            .query_row(&sql, [], |r| {
                Ok((r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?, r.get(4)?))
            })
            .unwrap();
        assert_eq!(ai, "KSFO");
        assert_eq!(le, "10L");
        assert_eq!(he, "28R");
        assert!(len > 11_000.0 && len < 12_000.0);
        assert!((hdg - 117.0).abs() < 1.5);
    }

    #[test]
    fn taxi_nodes_and_edges_round_trip() {
        let (_tmp, cache) = build_fixture_cache();
        let conn = Connection::open_in_memory().unwrap();
        let node_count: i64 = conn
            .query_row(
                &format!(
                    "SELECT COUNT(*) FROM read_parquet('{}')",
                    cache.taxi_nodes().display()
                ),
                [],
                |r| r.get(0),
            )
            .unwrap();
        assert_eq!(node_count, 2);
        let (airport, zones): (String, String) = conn
            .query_row(
                &format!(
                    "SELECT airport_ident, active_zones FROM read_parquet('{}') WHERE name = 'A'",
                    cache.taxi_edges().display()
                ),
                [],
                |r| Ok((r.get(0)?, r.get(1)?)),
            )
            .unwrap();
        assert_eq!(airport, "KSEA");
        assert_eq!(zones, "departure:16L");
    }

    #[test]
    fn comms_parquet_has_kind_and_freq() {
        let (_tmp, cache) = build_fixture_cache();
        let conn = Connection::open_in_memory().unwrap();
        let (kind, mhz): (String, f64) = conn
            .query_row(
                &format!(
                    "SELECT kind, freq_mhz FROM read_parquet('{}') WHERE airport_ident = 'KSEA'",
                    cache.comms().display()
                ),
                [],
                |r| Ok((r.get(0)?, r.get(1)?)),
            )
            .unwrap();
        assert_eq!(kind, "TWR");
        assert!((mhz - 119.9).abs() < 1e-6);
    }
}
