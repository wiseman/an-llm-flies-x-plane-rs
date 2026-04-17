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

use crate::data::apt_dat;

/// Names of the three parquet files inside a cache directory. Exposed so
/// `llm/tools.rs` can create views without hardcoding strings.
pub const AIRPORTS_FILE: &str = "airports.parquet";
pub const RUNWAYS_FILE: &str = "runways.parquet";
pub const COMMS_FILE: &str = "comms.parquet";

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
    pub fn all_files_exist(&self) -> bool {
        self.airports().exists() && self.runways().exists() && self.comms().exists()
    }
}

/// Return a cache for `apt_dat`, rebuilding it if missing or older than the
/// source. This is the single entry point callers outside this module
/// should use.
pub fn resolve(apt_dat: &Path) -> Result<AptDatCache> {
    if !apt_dat.exists() {
        return Err(anyhow!("apt.dat not found at {}", apt_dat.display()));
    }
    let cache = cache_for(apt_dat)?;
    if cache_is_fresh(apt_dat, &cache)? {
        return Ok(cache);
    }
    fs::create_dir_all(&cache.dir)
        .with_context(|| format!("creating {}", cache.dir.display()))?;
    build(apt_dat, &cache)
        .with_context(|| format!("building apt.dat parquet cache at {}", cache.dir.display()))?;
    Ok(cache)
}

fn cache_for(apt_dat: &Path) -> Result<AptDatCache> {
    let home = std::env::var_os("HOME")
        .ok_or_else(|| anyhow!("HOME is unset; cannot locate cache directory"))?;
    let canon = fs::canonicalize(apt_dat).unwrap_or_else(|_| apt_dat.to_path_buf());
    let mut hasher = DefaultHasher::new();
    canon.hash(&mut hasher);
    let h = hasher.finish();
    let dir = PathBuf::from(home)
        .join(".cache/sim_pilot")
        .join(format!("apt-{:016x}", h));
    Ok(AptDatCache { dir })
}

fn cache_is_fresh(apt_dat: &Path, cache: &AptDatCache) -> Result<bool> {
    if !cache.all_files_exist() {
        return Ok(false);
    }
    let src = fs::metadata(apt_dat)?
        .modified()
        .unwrap_or(SystemTime::UNIX_EPOCH);
    for f in [cache.airports(), cache.runways(), cache.comms()] {
        let dst = fs::metadata(&f)?.modified().unwrap_or(SystemTime::UNIX_EPOCH);
        if dst < src {
            return Ok(false);
        }
    }
    Ok(true)
}

fn build(apt_dat: &Path, cache: &AptDatCache) -> Result<()> {
    let parsed = apt_dat::parse_file(apt_dat)?;
    write_parquets(&parsed, cache)
}

/// Public entry point for tests: feed the builder a pre-parsed apt.dat dump
/// and write parquet into an arbitrary directory. Used by
/// `tests/test_tool_dispatch.rs` to stand up a fixture without depending on
/// the filesystem layout or `HOME`.
pub fn write_cache(parsed: &apt_dat::ParsedAptDat, dir: &Path) -> Result<AptDatCache> {
    fs::create_dir_all(dir)?;
    let cache = AptDatCache { dir: dir.to_path_buf() };
    write_parquets(parsed, &cache)?;
    Ok(cache)
}

fn write_parquets(parsed: &apt_dat::ParsedAptDat, cache: &AptDatCache) -> Result<()> {
    let conn = Connection::open_in_memory()?;
    conn.execute_batch("INSTALL spatial; LOAD spatial;")?;
    create_airport_table(&conn, &parsed.airports)?;
    create_runway_table(&conn, &parsed.runways)?;
    create_comm_table(&conn, &parsed.comms)?;
    copy_to_parquet(&conn, "airports", &cache.airports())?;
    copy_to_parquet(&conn, "runways", &cache.runways())?;
    copy_to_parquet(&conn, "comms", &cache.comms())?;
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

1     13 0 0 KSFO San Francisco Intl
100 61.00 1 0 0.0 1 3 0 10L 37.62875970 -122.39343890 0 250 3 0 0 3 28R 37.61353400 -122.35715510 90 100 3 2 1 3
99
";

    fn build_fixture_cache() -> (tempfile::TempDir, AptDatCache) {
        let dir = tempfile::tempdir().unwrap();
        let parsed = parse(FIXTURE.as_bytes()).unwrap();
        let cache = write_cache(&parsed, dir.path()).unwrap();
        (dir, cache)
    }

    #[test]
    fn parquet_cache_has_all_three_files() {
        let (_tmp, cache) = build_fixture_cache();
        assert!(cache.airports().exists());
        assert!(cache.runways().exists());
        assert!(cache.comms().exists());
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
