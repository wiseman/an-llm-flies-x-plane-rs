//! apt.dat parser + derived-CSV cache. No Python counterpart.
//!
//! X-Plane ships runway truth in `Global Scenery/Global Airports/Earth nav
//! data/apt.dat` (~360 MB, ~31k airports, ~38k runways). This module parses
//! the subset the pilot cares about — airport headers (rows 1 / 16 / 17),
//! land runways (row 100), airport metadata (row 1302) — and exports a CSV
//! whose column names match what `llm/tools.rs` already queries via DuckDB.
//!
//! apt.dat's row 100 does not store per-end heading, runway length, or per-end
//! elevation. These are derived from the two end coordinates (great-circle
//! distance, initial bearing) and the airport elevation from the row-1 header.
//! Spec: X-Plane apt.dat 1100 / 1200 File Format Specification.

use std::collections::hash_map::DefaultHasher;
use std::fs::{self, File};
use std::hash::{Hash, Hasher};
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::time::SystemTime;

use anyhow::{anyhow, Context, Result};

const METERS_TO_FEET: f64 = 3.280_839_895_013_123;
const EARTH_RADIUS_M: f64 = 6_371_008.8;

#[derive(Debug, Clone, PartialEq)]
pub struct ParsedAirport {
    pub ident: String,
    pub name: String,
    pub elevation_ft: f64,
    pub icao_code: Option<String>,
    pub iata_code: Option<String>,
    pub faa_code: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ParsedRunway {
    pub airport_ident: String,
    pub le_ident: String,
    pub he_ident: String,
    pub length_ft: f64,
    pub width_ft: f64,
    pub surface: String,
    pub lighted: u8,
    pub closed: u8,
    pub le_lat: f64,
    pub le_lon: f64,
    pub he_lat: f64,
    pub he_lon: f64,
    pub le_heading_deg: f64,
    pub he_heading_deg: f64,
    pub le_displaced_threshold_ft: f64,
    pub he_displaced_threshold_ft: f64,
    pub le_elevation_ft: f64,
    pub he_elevation_ft: f64,
}

pub fn parse<R: BufRead>(reader: R) -> Result<(Vec<ParsedAirport>, Vec<ParsedRunway>)> {
    let mut airports = Vec::new();
    let mut runways = Vec::new();
    let mut cur: Option<ParsedAirport> = None;

    for (i, line) in reader.lines().enumerate() {
        let line = line.with_context(|| format!("reading line {}", i + 1))?;
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        if trimmed == "99" {
            break;
        }
        let mut iter = trimmed.splitn(2, char::is_whitespace);
        let code = iter.next().unwrap_or("");
        let rest = iter.next().unwrap_or("").trim_start();

        match code {
            "1" | "16" | "17" => {
                if let Some(a) = cur.take() {
                    airports.push(a);
                }
                cur = parse_airport_header(rest);
            }
            "100" => {
                if let Some(a) = cur.as_ref() {
                    if let Some(rw) = parse_runway_100(rest, a) {
                        runways.push(rw);
                    }
                }
            }
            "1302" => {
                if let Some(a) = cur.as_mut() {
                    apply_1302(rest, a);
                }
            }
            _ => {}
        }
    }
    if let Some(a) = cur.take() {
        airports.push(a);
    }
    Ok((airports, runways))
}

pub fn parse_file(path: &Path) -> Result<(Vec<ParsedAirport>, Vec<ParsedRunway>)> {
    let f = File::open(path).with_context(|| format!("opening {}", path.display()))?;
    parse(BufReader::new(f))
}

fn parse_airport_header(rest: &str) -> Option<ParsedAirport> {
    // rest is everything after the "1" / "16" / "17" row code:
    //   <elev_ft> <deprecated> <deprecated> <ICAO> <name ...>
    let mut parts = rest.splitn(5, char::is_whitespace).filter(|s| !s.is_empty());
    let elev: f64 = parts.next()?.parse().ok()?;
    let _dep1 = parts.next()?;
    let _dep2 = parts.next()?;
    let ident = parts.next()?.to_string();
    let name = parts.next().unwrap_or("").trim().to_string();
    Some(ParsedAirport {
        ident,
        name,
        elevation_ft: elev,
        icao_code: None,
        iata_code: None,
        faa_code: None,
    })
}

fn apply_1302(rest: &str, airport: &mut ParsedAirport) {
    let mut parts = rest.splitn(2, char::is_whitespace);
    let key = parts.next().unwrap_or("");
    let value = parts.next().unwrap_or("").trim().to_string();
    if value.is_empty() {
        return;
    }
    match key {
        "icao_code" => airport.icao_code = Some(value),
        "iata_code" => airport.iata_code = Some(value),
        "faa_code" => airport.faa_code = Some(value),
        _ => {}
    }
}

fn parse_runway_100(rest: &str, airport: &ParsedAirport) -> Option<ParsedRunway> {
    // Fields after the "100" row code (25 total):
    //   0 width_m   1 surface   2 shoulder  3 smoothness
    //   4 cl_lights 5 edge_lights 6 signs
    //   7..16  end1: designator lat lon displaced_m blast_m marking approach tdz reil
    //   16..25 end2: same layout
    let f: Vec<&str> = rest.split_whitespace().collect();
    if f.len() < 25 {
        return None;
    }
    let width_m: f64 = f[0].parse().ok()?;
    let surface: i32 = f[1].parse().ok()?;
    let edge_lights: i32 = f[5].parse().ok()?;

    let le_ident = f[7].to_string();
    let le_lat: f64 = f[8].parse().ok()?;
    let le_lon: f64 = f[9].parse().ok()?;
    let le_disp_m: f64 = f[10].parse().ok()?;

    let he_ident = f[16].to_string();
    let he_lat: f64 = f[17].parse().ok()?;
    let he_lon: f64 = f[18].parse().ok()?;
    let he_disp_m: f64 = f[19].parse().ok()?;

    let length_m = haversine_m(le_lat, le_lon, he_lat, he_lon);
    let le_heading = initial_bearing_deg(le_lat, le_lon, he_lat, he_lon);
    let he_heading = (le_heading + 180.0).rem_euclid(360.0);

    Some(ParsedRunway {
        airport_ident: airport.ident.clone(),
        le_ident,
        he_ident,
        length_ft: length_m * METERS_TO_FEET,
        width_ft: width_m * METERS_TO_FEET,
        surface: surface_code_to_str(surface).to_string(),
        lighted: u8::from(edge_lights > 0),
        closed: 0,
        le_lat,
        le_lon,
        he_lat,
        he_lon,
        le_heading_deg: le_heading,
        he_heading_deg: he_heading,
        le_displaced_threshold_ft: le_disp_m * METERS_TO_FEET,
        he_displaced_threshold_ft: he_disp_m * METERS_TO_FEET,
        le_elevation_ft: airport.elevation_ft,
        he_elevation_ft: airport.elevation_ft,
    })
}

fn surface_code_to_str(code: i32) -> &'static str {
    match code {
        1 => "ASP",
        2 => "CON",
        3 => "TRF",
        4 => "DIRT",
        5 => "GRV",
        12 => "LAKE",
        13 => "WATER",
        14 => "SNOW",
        15 => "TRANS",
        20..=47 => "ASP",
        50..=57 => "CON",
        _ => "UNK",
    }
}

fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let p1 = lat1.to_radians();
    let p2 = lat2.to_radians();
    let dphi = (lat2 - lat1).to_radians();
    let dlam = (lon2 - lon1).to_radians();
    let a = (dphi * 0.5).sin().powi(2) + p1.cos() * p2.cos() * (dlam * 0.5).sin().powi(2);
    2.0 * EARTH_RADIUS_M * a.sqrt().asin()
}

fn initial_bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let p1 = lat1.to_radians();
    let p2 = lat2.to_radians();
    let dlam = (lon2 - lon1).to_radians();
    let x = dlam.sin() * p2.cos();
    let y = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dlam.cos();
    x.atan2(y).to_degrees().rem_euclid(360.0)
}

pub const RUNWAY_CSV_HEADER: &str = "id,airport_ref,airport_ident,length_ft,width_ft,surface,lighted,closed,le_ident,le_latitude_deg,le_longitude_deg,le_elevation_ft,le_heading_degT,le_displaced_threshold_ft,he_ident,he_latitude_deg,he_longitude_deg,he_elevation_ft,he_heading_degT,he_displaced_threshold_ft";

pub fn write_runways_csv<W: Write>(mut writer: W, runways: &[ParsedRunway]) -> Result<()> {
    writeln!(writer, "{}", RUNWAY_CSV_HEADER)?;
    for (i, r) in runways.iter().enumerate() {
        writeln!(
            writer,
            "{id},0,{ident},{length:.1},{width:.1},{surface},{lit},{closed},\
             {le_i},{le_lat:.8},{le_lon:.8},{le_elev:.1},{le_h:.2},{le_d:.2},\
             {he_i},{he_lat:.8},{he_lon:.8},{he_elev:.1},{he_h:.2},{he_d:.2}",
            id = i + 1,
            ident = r.airport_ident,
            length = r.length_ft,
            width = r.width_ft,
            surface = r.surface,
            lit = r.lighted,
            closed = r.closed,
            le_i = r.le_ident,
            le_lat = r.le_lat,
            le_lon = r.le_lon,
            le_elev = r.le_elevation_ft,
            le_h = r.le_heading_deg,
            le_d = r.le_displaced_threshold_ft,
            he_i = r.he_ident,
            he_lat = r.he_lat,
            he_lon = r.he_lon,
            he_elev = r.he_elevation_ft,
            he_h = r.he_heading_deg,
            he_d = r.he_displaced_threshold_ft,
        )?;
    }
    Ok(())
}

fn cache_dir() -> Option<PathBuf> {
    std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".cache/sim_pilot"))
}

fn cached_csv_path_for(apt_dat: &Path) -> Option<PathBuf> {
    let canon = fs::canonicalize(apt_dat).unwrap_or_else(|_| apt_dat.to_path_buf());
    let mut hasher = DefaultHasher::new();
    canon.hash(&mut hasher);
    let h = hasher.finish();
    cache_dir().map(|d| d.join(format!("runways-{:016x}.csv", h)))
}

/// Return a runways CSV derived from `apt_dat`. The first call parses the
/// 360 MB source and writes a small cached copy; subsequent calls reuse the
/// cache as long as its mtime is at or after apt.dat's.
pub fn resolve_runways_csv(apt_dat: &Path) -> Result<PathBuf> {
    let cache = cached_csv_path_for(apt_dat)
        .ok_or_else(|| anyhow!("HOME is unset; cannot locate cache directory"))?;
    if cache_is_fresh(apt_dat, &cache)? {
        return Ok(cache);
    }
    if let Some(parent) = cache.parent() {
        fs::create_dir_all(parent)?;
    }
    let (_airports, runways) = parse_file(apt_dat)?;
    let tmp = cache.with_extension("csv.tmp");
    {
        let file =
            File::create(&tmp).with_context(|| format!("creating {}", tmp.display()))?;
        write_runways_csv(file, &runways)?;
    }
    fs::rename(&tmp, &cache)?;
    Ok(cache)
}

fn cache_is_fresh(apt_dat: &Path, cache: &Path) -> Result<bool> {
    if !cache.exists() {
        return Ok(false);
    }
    let src = fs::metadata(apt_dat)?
        .modified()
        .unwrap_or(SystemTime::UNIX_EPOCH);
    let dst = fs::metadata(cache)?
        .modified()
        .unwrap_or(SystemTime::UNIX_EPOCH);
    Ok(dst >= src)
}

/// Best-effort autodetect of the default X-Plane 12 apt.dat on the current
/// host. Returns None if no candidate exists; callers fall back to
/// --runway-csv-path.
pub fn default_apt_dat_path() -> Option<PathBuf> {
    let home = std::env::var_os("HOME")?;
    let candidates = [
        PathBuf::from(&home).join(
            "Library/Application Support/Steam/steamapps/common/X-Plane 12/\
             Global Scenery/Global Airports/Earth nav data/apt.dat",
        ),
        PathBuf::from("/Applications/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat"),
    ];
    candidates.into_iter().find(|p| p.exists())
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = "\
A
1200 Generated by WorldEditor

1     429 0 0 KSEA Seattle-Tacoma Intl
1302 city Seattle
1302 icao_code KSEA
1302 iata_code SEA
100 45.72 1 0 0.25 1 2 0 16L 47.46380000 -122.30800000 0 60 2 1 1 2 34R 47.43130000 -122.30800000 0 60 2 1 1 2
100 45.72 2 0 0.25 1 2 0 16R 47.46380000 -122.31800000 0 60 2 1 1 2 34L 47.44050000 -122.31800000 0 60 2 1 1 2
101 49 1 08 35.04420900 -106.59855700 26 35.04420911 -106.59855711

1     21 0 0 KBFI Boeing Field
102 H1 47.53918 -122.30722 0.0 20.00 20.00 2 0 0 0.00 0

1     13 0 0 KSFO San Francisco Intl
1302 icao_code KSFO
100 61.00 1 0 0.0 1 3 0 10L 37.62875970 -122.39343890 0 250 3 0 0 3 28R 37.61353400 -122.35715510 90 100 3 2 1 3
99
";

    fn parse_fixture() -> (Vec<ParsedAirport>, Vec<ParsedRunway>) {
        parse(FIXTURE.as_bytes()).unwrap()
    }

    #[test]
    fn parses_three_airports_and_three_runways() {
        let (airports, runways) = parse_fixture();
        let idents: Vec<&str> = airports.iter().map(|a| a.ident.as_str()).collect();
        assert_eq!(idents, ["KSEA", "KBFI", "KSFO"]);
        // KBFI only had a helipad (102) — no land runways.
        let rw_airports: Vec<&str> = runways.iter().map(|r| r.airport_ident.as_str()).collect();
        assert_eq!(rw_airports, ["KSEA", "KSEA", "KSFO"]);
    }

    #[test]
    fn airport_metadata_captured() {
        let (airports, _) = parse_fixture();
        let ksea = &airports[0];
        assert_eq!(ksea.icao_code.as_deref(), Some("KSEA"));
        assert_eq!(ksea.iata_code.as_deref(), Some("SEA"));
        assert_eq!(ksea.elevation_ft, 429.0);
        assert_eq!(ksea.name, "Seattle-Tacoma Intl");
        assert!(airports[1].icao_code.is_none());
    }

    #[test]
    fn surface_code_mapping() {
        let (_, runways) = parse_fixture();
        assert_eq!(runways[0].surface, "ASP");
        assert_eq!(runways[1].surface, "CON");
        assert_eq!(surface_code_to_str(20), "ASP");
        assert_eq!(surface_code_to_str(55), "CON");
        assert_eq!(surface_code_to_str(99), "UNK");
    }

    #[test]
    fn derived_heading_and_length_for_ksea_16l() {
        let (_, runways) = parse_fixture();
        let r = &runways[0];
        assert_eq!(r.le_ident, "16L");
        assert_eq!(r.he_ident, "34R");
        // Same longitude, end1 north of end2 → due south.
        assert!((r.le_heading_deg - 180.0).abs() < 0.01, "le heading was {}", r.le_heading_deg);
        assert!((r.he_heading_deg - 0.0).abs() < 0.01, "he heading was {}", r.he_heading_deg);
        // ~2.17 min of latitude at this lat ≈ 11,870 ft.
        assert!(
            (r.length_ft - 11_875.0).abs() < 100.0,
            "length was {}",
            r.length_ft
        );
        // Width 45.72 m = 150 ft.
        assert!((r.width_ft - 150.0).abs() < 0.5, "width was {}", r.width_ft);
        assert_eq!(r.lighted, 1);
        assert_eq!(r.closed, 0);
        assert_eq!(r.le_elevation_ft, 429.0);
    }

    #[test]
    fn derived_heading_for_ksfo_10l_matches_known() {
        let (_, runways) = parse_fixture();
        let r = runways.iter().find(|r| r.le_ident == "10L").unwrap();
        // KSFO 10L/28R is charted as 117°/297°. Great-circle bearing between
        // the published threshold coords lands there within ~1°.
        assert!(
            (r.le_heading_deg - 117.0).abs() < 1.5,
            "10L heading was {}",
            r.le_heading_deg
        );
        assert!((r.he_displaced_threshold_ft - 295.0).abs() < 1.0);
    }

    #[test]
    fn ignores_water_runways_and_helipads() {
        let (_, runways) = parse_fixture();
        // The 101 (water) in KSEA block and 102 (helipad) under KBFI must not
        // appear as land runways.
        assert!(runways.iter().all(|r| r.le_ident != "08"));
        assert!(runways.iter().all(|r| r.le_ident != "H1"));
    }

    #[test]
    fn csv_emits_expected_header_and_row_count() {
        let (_, runways) = parse_fixture();
        let mut buf: Vec<u8> = Vec::new();
        write_runways_csv(&mut buf, &runways).unwrap();
        let text = String::from_utf8(buf).unwrap();
        let mut lines = text.lines();
        assert_eq!(lines.next().unwrap(), RUNWAY_CSV_HEADER);
        assert_eq!(lines.count(), runways.len());
    }

    #[test]
    fn csv_row_has_correct_column_count() {
        let (_, runways) = parse_fixture();
        let mut buf: Vec<u8> = Vec::new();
        write_runways_csv(&mut buf, &runways).unwrap();
        let text = String::from_utf8(buf).unwrap();
        for line in text.lines() {
            assert_eq!(line.matches(',').count(), 19, "row={}", line);
        }
    }
}
