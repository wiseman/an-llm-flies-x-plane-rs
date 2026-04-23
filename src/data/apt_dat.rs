//! apt.dat streaming parser.
//!
//! X-Plane ships all runway and comm truth in `Global Scenery/Global
//! Airports/Earth nav data/apt.dat` (~360 MB, ~38k airports, ~38k runways,
//! ~28k comm frequencies). This module parses the subset the pilot cares
//! about in one streaming pass:
//!
//! - rows 1 / 16 / 17 (airport / seaplane base / heliport header)
//! - row 100 (land runway)
//! - row 1302 (airport identification metadata — ICAO / IATA / FAA /
//!   datum_lat / datum_lon)
//! - rows 1050..=1056 (comm frequencies: ATIS, UNICOM, CD, GND, TWR, APP, DEP)
//! - row 1300 (startup/parking locations — gate, tie-down, hangar, misc)
//!   plus optional 1301 operation metadata
//!
//! apt.dat's row 100 does not store per-end heading, runway length, or
//! per-end elevation. These are derived from the two end coordinates
//! (great-circle distance, initial bearing) and the airport elevation from
//! the row-1 header. Only ~44% of airports have `datum_lat`/`datum_lon` in
//! row 1302, so airports missing those are backfilled with the centroid of
//! their runway endpoints (rounded to the airport centre of mass) so every
//! airport has a lat/lon downstream.
//!
//! Spec: X-Plane apt.dat 1100 / 1200 File Format Specification.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

use crate::types::{haversine_m, initial_bearing_deg};

const METERS_TO_FEET: f64 = 3.280_839_895_013_123;

#[derive(Debug, Clone, PartialEq)]
pub struct ParsedAirport {
    pub ident: String,
    pub name: String,
    pub elevation_ft: f64,
    pub icao_code: Option<String>,
    pub iata_code: Option<String>,
    pub faa_code: Option<String>,
    /// Airport reference point. Populated from row 1302 `datum_lat`/
    /// `datum_lon` when present, otherwise backfilled with the centroid of
    /// the airport's runway endpoints. `None` only for airports with neither
    /// 1302 metadata nor any land runway.
    pub latitude_deg: Option<f64>,
    pub longitude_deg: Option<f64>,
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

#[derive(Debug, Clone, PartialEq)]
pub struct ParsedComm {
    pub airport_ident: String,
    /// Raw apt.dat row code (1050..=1056) — retained for callers that want
    /// exact provenance.
    pub code: u16,
    /// Friendly kind derived from `code`: ATIS / UNICOM / CD / GND / TWR /
    /// APP / DEP.
    pub kind: &'static str,
    pub freq_mhz: f64,
    /// Free-form descriptive label from the apt.dat row (may contain
    /// spaces), e.g. "NORCAL APP", "AWOS 1".
    pub label: String,
}

/// A single node on an airport's taxi route network (apt.dat row 1201).
#[derive(Debug, Clone, PartialEq)]
pub struct ParsedTaxiNode {
    pub airport_ident: String,
    /// Node id as written in apt.dat — unique *within* the airport block
    /// only; composite key is (airport_ident, node_id).
    pub node_id: u32,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    /// Usage flag: "init" (path can start here), "end" (path can end
    /// here), or "both".
    pub usage: String,
}

/// A single edge on an airport's taxi route network (apt.dat row 1202),
/// plus any 1204 active-zone rows attached to it.
#[derive(Debug, Clone, PartialEq)]
pub struct ParsedTaxiEdge {
    pub airport_ident: String,
    pub from_node: u32,
    pub to_node: u32,
    /// "twoway" or "oneway". When "oneway", traversal is `from_node` →
    /// `to_node` only.
    pub direction: String,
    /// Raw apt.dat category token ("taxiway_E", "taxiway_F", "runway",
    /// "taxiway"). The trailing letter is the ICAO width class.
    pub category: String,
    /// Taxiway name as painted on pavement ("A", "A2", "B", "L", …). May
    /// be empty for some runway-threshold connector edges.
    pub name: String,
    /// 1204 rows attached to this edge — one per (usage, runway_list)
    /// tuple. Usage is "arrival" / "departure" / "ils"; runway list is
    /// comma-separated raw from apt.dat (e.g. "01L,01R,19L,19R").
    pub active_zones: Vec<ParsedActiveZone>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ParsedActiveZone {
    pub kind: String,
    pub runways: String,
}

/// A startup / parking location (apt.dat row 1300) plus any attached 1301
/// operation metadata. apt.dat only pairs one 1301 with the preceding 1300
/// — so we fold the 1301 fields directly onto the spot instead of keeping
/// a separate struct.
#[derive(Debug, Clone, PartialEq)]
pub struct ParsedParkingSpot {
    pub airport_ident: String,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    /// True heading the aircraft should face when parked, in degrees.
    pub heading_true_deg: f64,
    /// Kind token: "gate", "tie_down", "hangar", or "misc".
    pub kind: String,
    /// Pipe-separated categories from apt.dat, preserved raw:
    /// e.g. "heavy|jets|turboprops|props|helos". Empty string if absent.
    pub categories: String,
    /// Free-form spot name ("Ramp 1", "Gate A4", "GA Tie-Down 3").
    pub name: String,
    /// ICAO airline-operations category from 1301 (single token like "A",
    /// "B" through "F", or a pipe list). None when no 1301 follows.
    pub icao_category: Option<String>,
    /// Operation type from 1301: "none", "general_aviation", "airline",
    /// "cargo", or "military". None when no 1301 follows.
    pub operation_type: Option<String>,
    /// Comma-separated 3-letter airline codes from 1301 (free-form, may be
    /// empty even when 1301 is present).
    pub airlines: Option<String>,
}

#[derive(Debug, Clone, Default)]
pub struct ParsedAptDat {
    pub airports: Vec<ParsedAirport>,
    pub runways: Vec<ParsedRunway>,
    pub comms: Vec<ParsedComm>,
    pub taxi_nodes: Vec<ParsedTaxiNode>,
    pub taxi_edges: Vec<ParsedTaxiEdge>,
    pub parking_spots: Vec<ParsedParkingSpot>,
}

pub fn parse<R: BufRead>(reader: R) -> Result<ParsedAptDat> {
    let mut out = ParsedAptDat::default();
    let mut cur: Option<ParsedAirport> = None;
    // Index of the last 1202 edge pushed, so any 1204 rows that follow can
    // attach to it. Reset between airports.
    let mut last_edge_idx: Option<usize> = None;
    // Index of the last 1300 parking spot pushed, for 1301 attachment.
    let mut last_parking_idx: Option<usize> = None;

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
                    out.airports.push(a);
                }
                cur = parse_airport_header(rest);
                last_edge_idx = None;
                last_parking_idx = None;
            }
            "100" => {
                if let Some(a) = cur.as_ref() {
                    if let Some(rw) = parse_runway_100(rest, a) {
                        out.runways.push(rw);
                    }
                }
            }
            "1302" => {
                if let Some(a) = cur.as_mut() {
                    apply_1302(rest, a);
                }
            }
            "1050" | "1051" | "1052" | "1053" | "1054" | "1055" | "1056" => {
                if let Some(a) = cur.as_ref() {
                    if let Some(comm) = parse_comm_row(code, rest, &a.ident) {
                        out.comms.push(comm);
                    }
                }
            }
            "1201" => {
                if let Some(a) = cur.as_ref() {
                    if let Some(node) = parse_taxi_node(rest, &a.ident) {
                        out.taxi_nodes.push(node);
                    }
                }
            }
            "1202" => {
                if let Some(a) = cur.as_ref() {
                    if let Some(edge) = parse_taxi_edge(rest, &a.ident) {
                        out.taxi_edges.push(edge);
                        last_edge_idx = Some(out.taxi_edges.len() - 1);
                    } else {
                        last_edge_idx = None;
                    }
                }
            }
            "1204" => {
                if let Some(idx) = last_edge_idx {
                    if let Some(zone) = parse_active_zone(rest) {
                        out.taxi_edges[idx].active_zones.push(zone);
                    }
                }
            }
            "1300" => {
                if let Some(a) = cur.as_ref() {
                    if let Some(spot) = parse_parking_spot(rest, &a.ident) {
                        out.parking_spots.push(spot);
                        last_parking_idx = Some(out.parking_spots.len() - 1);
                    } else {
                        last_parking_idx = None;
                    }
                }
            }
            "1301" => {
                if let Some(idx) = last_parking_idx {
                    apply_1301(rest, &mut out.parking_spots[idx]);
                }
            }
            _ => {}
        }
    }
    if let Some(a) = cur.take() {
        out.airports.push(a);
    }
    backfill_airport_arps(&mut out.airports, &out.runways);
    Ok(out)
}

pub fn parse_file(path: &Path) -> Result<ParsedAptDat> {
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
        latitude_deg: None,
        longitude_deg: None,
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
        "datum_lat" => airport.latitude_deg = value.parse().ok(),
        "datum_lon" => airport.longitude_deg = value.parse().ok(),
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

fn parse_comm_row(code: &str, rest: &str, airport_ident: &str) -> Option<ParsedComm> {
    // Modern apt.dat (1200+) encodes comm frequencies at 8.33 kHz resolution
    // as a 6-digit integer equal to MHz * 1000 (e.g. "127275" = 127.275 MHz,
    // "120500" = 120.500 MHz). Label is free-form text (possibly with spaces).
    let mut parts = rest.splitn(2, char::is_whitespace);
    let freq_token = parts.next()?;
    let label = parts.next().unwrap_or("").trim().to_string();
    let freq_khz: u64 = freq_token.parse().ok()?;
    let freq_mhz = freq_khz as f64 / 1000.0;
    let code_u: u16 = code.parse().ok()?;
    let kind = comm_code_to_kind(code_u)?;
    Some(ParsedComm {
        airport_ident: airport_ident.to_string(),
        code: code_u,
        kind,
        freq_mhz,
        label,
    })
}

fn parse_taxi_node(rest: &str, airport_ident: &str) -> Option<ParsedTaxiNode> {
    // 1201 <lat> <lon> <usage> <node_id> [<name ...>]
    let mut parts = rest.split_whitespace();
    let lat: f64 = parts.next()?.parse().ok()?;
    let lon: f64 = parts.next()?.parse().ok()?;
    let usage = parts.next()?.to_string();
    let node_id: u32 = parts.next()?.parse().ok()?;
    Some(ParsedTaxiNode {
        airport_ident: airport_ident.to_string(),
        node_id,
        latitude_deg: lat,
        longitude_deg: lon,
        usage,
    })
}

fn parse_taxi_edge(rest: &str, airport_ident: &str) -> Option<ParsedTaxiEdge> {
    // 1202 <from> <to> <direction> <category> [<name ...>]
    // The name may contain whitespace (e.g. "A 2" is unusual but possible);
    // splitn keeps everything after the category as the full name.
    let mut parts = rest.splitn(5, char::is_whitespace);
    let from_node: u32 = parts.next()?.parse().ok()?;
    let to_node: u32 = parts.next()?.parse().ok()?;
    let direction = parts.next()?.to_string();
    let category = parts.next()?.to_string();
    let name = parts.next().unwrap_or("").trim().to_string();
    Some(ParsedTaxiEdge {
        airport_ident: airport_ident.to_string(),
        from_node,
        to_node,
        direction,
        category,
        name,
        active_zones: Vec::new(),
    })
}

fn parse_parking_spot(rest: &str, airport_ident: &str) -> Option<ParsedParkingSpot> {
    // 1300 <lat> <lon> <heading_true> <type> <categories> <name ...>
    // `categories` is a single pipe-separated token (no whitespace), so
    // splitn(6, ws) keeps `name` — which may contain whitespace — intact.
    let mut parts = rest.splitn(6, char::is_whitespace).filter(|s| !s.is_empty());
    let lat: f64 = parts.next()?.parse().ok()?;
    let lon: f64 = parts.next()?.parse().ok()?;
    let heading: f64 = parts.next()?.parse().ok()?;
    let kind = parts.next()?.to_string();
    let categories = parts.next().unwrap_or("").to_string();
    let name = parts.next().unwrap_or("").trim().to_string();
    Some(ParsedParkingSpot {
        airport_ident: airport_ident.to_string(),
        latitude_deg: lat,
        longitude_deg: lon,
        heading_true_deg: heading.rem_euclid(360.0),
        kind,
        categories,
        name,
        icao_category: None,
        operation_type: None,
        airlines: None,
    })
}

fn apply_1301(rest: &str, spot: &mut ParsedParkingSpot) {
    // 1301 <icao_aircraft_category> <operation_type> [airline_codes ...]
    let mut parts = rest.splitn(3, char::is_whitespace).filter(|s| !s.is_empty());
    if let Some(cat) = parts.next() {
        spot.icao_category = Some(cat.to_string());
    }
    if let Some(op) = parts.next() {
        spot.operation_type = Some(op.to_string());
    }
    if let Some(airlines) = parts.next() {
        let trimmed = airlines.trim();
        if !trimmed.is_empty() {
            spot.airlines = Some(trimmed.to_string());
        }
    }
}

fn parse_active_zone(rest: &str) -> Option<ParsedActiveZone> {
    // 1204 <kind> <runway1,runway2,...>
    let mut parts = rest.splitn(2, char::is_whitespace);
    let kind = parts.next()?.to_string();
    let runways = parts.next()?.trim().to_string();
    if kind.is_empty() || runways.is_empty() {
        return None;
    }
    Some(ParsedActiveZone { kind, runways })
}

fn comm_code_to_kind(code: u16) -> Option<&'static str> {
    match code {
        1050 => Some("ATIS"),   // also AWOS / ASOS — disambiguate via label
        1051 => Some("UNICOM"), // CTAF at non-towered fields
        1052 => Some("CD"),     // Clearance Delivery
        1053 => Some("GND"),    // Ground
        1054 => Some("TWR"),    // Tower
        1055 => Some("APP"),    // Approach
        1056 => Some("DEP"),    // Departure
        _ => None,
    }
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

/// Populate `latitude_deg`/`longitude_deg` for airports that came out of
/// the parser without them (no row 1302 `datum_lat`/`datum_lon`) by taking
/// the mean of the runway endpoint coordinates at that airport. apt.dat's
/// row 1302 only covers the ~44% of airports in the gateway — the remainder
/// are small strips where the centroid of the runway endpoints is a fine
/// stand-in for the ARP (worst-case offset is a few hundred metres, well
/// inside the scale at which 'which airport am I closest to' queries care).
fn backfill_airport_arps(airports: &mut [ParsedAirport], runways: &[ParsedRunway]) {
    use std::collections::HashMap;
    let mut sums: HashMap<&str, (f64, f64, usize)> = HashMap::new();
    for r in runways {
        let entry = sums.entry(r.airport_ident.as_str()).or_insert((0.0, 0.0, 0));
        entry.0 += r.le_lat + r.he_lat;
        entry.1 += r.le_lon + r.he_lon;
        entry.2 += 2;
    }
    for a in airports.iter_mut() {
        if a.latitude_deg.is_some() && a.longitude_deg.is_some() {
            continue;
        }
        if let Some(&(lat_sum, lon_sum, n)) = sums.get(a.ident.as_str()) {
            if n > 0 {
                a.latitude_deg = Some(lat_sum / n as f64);
                a.longitude_deg = Some(lon_sum / n as f64);
            }
        }
    }
}

/// Best-effort autodetect of the default X-Plane 12 apt.dat on the current
/// host. Returns None if no candidate exists.
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
1302 datum_lat 47.449888889
1302 datum_lon -122.311777778
1050 127275 ATIS
1054 119900 TWR
100 45.72 1 0 0.25 1 2 0 16L 47.46380000 -122.30800000 0 60 2 1 1 2 34R 47.43130000 -122.30800000 0 60 2 1 1 2
100 45.72 2 0 0.25 1 2 0 16R 47.46380000 -122.31800000 0 60 2 1 1 2 34L 47.44050000 -122.31800000 0 60 2 1 1 2
101 49 1 08 35.04420900 -106.59855700 26 35.04420911 -106.59855711

1     21 0 0 KBFI Boeing Field
102 H1 47.53918 -122.30722 0.0 20.00 20.00 2 0 0 0.00 0

1     13 0 0 KSFO San Francisco Intl
1302 icao_code KSFO
100 61.00 1 0 0.0 1 3 0 10L 37.62875970 -122.39343890 0 250 3 0 0 3 28R 37.61353400 -122.35715510 90 100 3 2 1 3
1055 128325 NORCAL APP
1055 134500 NORCAL APP
1200
1201 37.634770 -122.394253 both 1
1201 37.635023 -122.393136 both 2
1201 37.615613 -122.371783 both 3
1202 1 2 twoway taxiway_E A
1202 2 3 twoway taxiway_E B
1204 departure 10L,28R
1204 ils 28R
1202 3 1 oneway taxiway_F Z
1300 37.615930 -122.371120 295.9 gate jets|turboprops|props Gate A4
1301 D airline UAL,DAL
1300 37.616120 -122.370450 115.1 tie_down props|helos GA Tie-Down 1
99
";

    fn parse_fixture() -> ParsedAptDat {
        parse(FIXTURE.as_bytes()).unwrap()
    }

    #[test]
    fn parses_three_airports_and_three_runways() {
        let data = parse_fixture();
        let idents: Vec<&str> = data.airports.iter().map(|a| a.ident.as_str()).collect();
        assert_eq!(idents, ["KSEA", "KBFI", "KSFO"]);
        // KBFI only had a helipad (102) — no land runways.
        let rw_airports: Vec<&str> =
            data.runways.iter().map(|r| r.airport_ident.as_str()).collect();
        assert_eq!(rw_airports, ["KSEA", "KSEA", "KSFO"]);
    }

    #[test]
    fn airport_metadata_captured() {
        let data = parse_fixture();
        let ksea = &data.airports[0];
        assert_eq!(ksea.icao_code.as_deref(), Some("KSEA"));
        assert_eq!(ksea.iata_code.as_deref(), Some("SEA"));
        assert_eq!(ksea.elevation_ft, 429.0);
        assert_eq!(ksea.name, "Seattle-Tacoma Intl");
        // Datum comes straight from 1302 when present.
        assert_eq!(ksea.latitude_deg, Some(47.449888889));
        assert_eq!(ksea.longitude_deg, Some(-122.311777778));
        assert!(data.airports[1].icao_code.is_none());
    }

    #[test]
    fn airport_without_datum_gets_centroid_fallback() {
        let data = parse_fixture();
        // KSFO has no 1302 datum_lat/datum_lon in the fixture; the single
        // runway's endpoint centroid should fill it in.
        let ksfo = data.airports.iter().find(|a| a.ident == "KSFO").unwrap();
        let lat = ksfo.latitude_deg.expect("KSFO lat backfilled");
        let lon = ksfo.longitude_deg.expect("KSFO lon backfilled");
        let expected_lat = (37.62875970 + 37.61353400) * 0.5;
        let expected_lon = (-122.39343890 + -122.35715510) * 0.5;
        assert!((lat - expected_lat).abs() < 1e-9);
        assert!((lon - expected_lon).abs() < 1e-9);
    }

    #[test]
    fn airport_with_no_runways_and_no_datum_has_no_coords() {
        let data = parse_fixture();
        // KBFI in the fixture has only a helipad (row 102), no land runways,
        // and no 1302 datum. Fallback has nothing to work with.
        let kbfi = data.airports.iter().find(|a| a.ident == "KBFI").unwrap();
        assert!(kbfi.latitude_deg.is_none());
        assert!(kbfi.longitude_deg.is_none());
    }

    #[test]
    fn surface_code_mapping() {
        let data = parse_fixture();
        assert_eq!(data.runways[0].surface, "ASP");
        assert_eq!(data.runways[1].surface, "CON");
        assert_eq!(surface_code_to_str(20), "ASP");
        assert_eq!(surface_code_to_str(55), "CON");
        assert_eq!(surface_code_to_str(99), "UNK");
    }

    #[test]
    fn derived_heading_and_length_for_ksea_16l() {
        let data = parse_fixture();
        let r = &data.runways[0];
        assert_eq!(r.le_ident, "16L");
        assert_eq!(r.he_ident, "34R");
        assert!((r.le_heading_deg - 180.0).abs() < 0.01);
        assert!((r.he_heading_deg - 0.0).abs() < 0.01);
        assert!((r.length_ft - 11_875.0).abs() < 100.0, "length was {}", r.length_ft);
        assert!((r.width_ft - 150.0).abs() < 0.5);
        assert_eq!(r.lighted, 1);
        assert_eq!(r.closed, 0);
        assert_eq!(r.le_elevation_ft, 429.0);
    }

    #[test]
    fn derived_heading_for_ksfo_10l_matches_known() {
        let data = parse_fixture();
        let r = data.runways.iter().find(|r| r.le_ident == "10L").unwrap();
        assert!((r.le_heading_deg - 117.0).abs() < 1.5);
        assert!((r.he_displaced_threshold_ft - 295.0).abs() < 1.0);
    }

    #[test]
    fn ignores_water_runways_and_helipads() {
        let data = parse_fixture();
        assert!(data.runways.iter().all(|r| r.le_ident != "08"));
        assert!(data.runways.iter().all(|r| r.le_ident != "H1"));
    }

    #[test]
    fn parses_taxi_nodes_and_edges() {
        let data = parse_fixture();
        let ksfo_nodes: Vec<_> =
            data.taxi_nodes.iter().filter(|n| n.airport_ident == "KSFO").collect();
        assert_eq!(ksfo_nodes.len(), 3);
        assert_eq!(ksfo_nodes[0].node_id, 1);
        assert!((ksfo_nodes[0].latitude_deg - 37.634770).abs() < 1e-9);
        assert_eq!(ksfo_nodes[0].usage, "both");

        let ksfo_edges: Vec<_> =
            data.taxi_edges.iter().filter(|e| e.airport_ident == "KSFO").collect();
        assert_eq!(ksfo_edges.len(), 3);
        assert_eq!(ksfo_edges[0].name, "A");
        assert_eq!(ksfo_edges[0].direction, "twoway");
        assert_eq!(ksfo_edges[2].name, "Z");
        assert_eq!(ksfo_edges[2].direction, "oneway");
    }

    #[test]
    fn active_zones_attach_to_prior_edge() {
        let data = parse_fixture();
        // 1204 rows in the fixture follow the "B" edge (index 1 among KSFO
        // edges), not "A" or "Z".
        let edge_b = data
            .taxi_edges
            .iter()
            .find(|e| e.airport_ident == "KSFO" && e.name == "B")
            .unwrap();
        assert_eq!(edge_b.active_zones.len(), 2);
        assert_eq!(edge_b.active_zones[0].kind, "departure");
        assert_eq!(edge_b.active_zones[0].runways, "10L,28R");
        assert_eq!(edge_b.active_zones[1].kind, "ils");
        let edge_a = data
            .taxi_edges
            .iter()
            .find(|e| e.airport_ident == "KSFO" && e.name == "A")
            .unwrap();
        assert!(edge_a.active_zones.is_empty());
    }

    #[test]
    fn parses_parking_spots_with_and_without_1301() {
        let data = parse_fixture();
        let ksfo: Vec<_> = data
            .parking_spots
            .iter()
            .filter(|p| p.airport_ident == "KSFO")
            .collect();
        assert_eq!(ksfo.len(), 2);

        let gate = &ksfo[0];
        assert_eq!(gate.name, "Gate A4");
        assert_eq!(gate.kind, "gate");
        assert_eq!(gate.categories, "jets|turboprops|props");
        assert!((gate.heading_true_deg - 295.9).abs() < 1e-6);
        assert_eq!(gate.icao_category.as_deref(), Some("D"));
        assert_eq!(gate.operation_type.as_deref(), Some("airline"));
        assert_eq!(gate.airlines.as_deref(), Some("UAL,DAL"));

        let tie = &ksfo[1];
        assert_eq!(tie.name, "GA Tie-Down 1");
        assert_eq!(tie.kind, "tie_down");
        assert_eq!(tie.categories, "props|helos");
        assert!(tie.icao_category.is_none());
        assert!(tie.operation_type.is_none());
        assert!(tie.airlines.is_none());
    }

    #[test]
    fn parking_1301_does_not_leak_across_airports() {
        // Synthesize: airport A has 1300 without 1301, then airport B starts
        // and its first row is 1301 — which must NOT attach to airport A's
        // spot.
        let fixture = "\
1     10 0 0 KAAA A
1302 icao_code KAAA
1300 10.0 20.0 0 gate jets Spot 1

1     10 0 0 KBBB B
1301 C airline XXX
99
";
        let data = parse(fixture.as_bytes()).unwrap();
        assert_eq!(data.parking_spots.len(), 1);
        assert!(data.parking_spots[0].icao_category.is_none());
    }

    #[test]
    fn parses_comm_rows_with_kind_and_mhz() {
        let data = parse_fixture();
        let ksea: Vec<_> = data.comms.iter().filter(|c| c.airport_ident == "KSEA").collect();
        assert_eq!(ksea.len(), 2);
        let atis = ksea.iter().find(|c| c.code == 1050).unwrap();
        assert_eq!(atis.kind, "ATIS");
        assert!((atis.freq_mhz - 127.275).abs() < 1e-9);
        assert_eq!(atis.label, "ATIS");
        let twr = ksea.iter().find(|c| c.code == 1054).unwrap();
        assert_eq!(twr.kind, "TWR");
        assert!((twr.freq_mhz - 119.900).abs() < 1e-9);

        // Multi-word label preserved.
        let ksfo_app: Vec<_> = data
            .comms
            .iter()
            .filter(|c| c.airport_ident == "KSFO" && c.code == 1055)
            .collect();
        assert_eq!(ksfo_app.len(), 2);
        assert!(ksfo_app.iter().all(|c| c.label == "NORCAL APP"));
    }
}
