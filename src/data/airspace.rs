//! airspace.txt streaming parser. No Python counterpart.
//!
//! X-Plane 12 ships a global airspace dataset at
//! `Resources/default data/airspaces/airspace.txt` (~18 MB, ~24k
//! airspaces). The format is a minimal OpenAir subset: only five record
//! types appear in the shipped file —
//!
//!   AC <class>         airspace class / type
//!   AN <name>          free-form name
//!   AL <alt>           lower limit ("GND" or "<n> MSL")
//!   AH <alt>           upper limit ("GND" or "<n> MSL")
//!   DP HH:MM:SS N ddd:mm:ss W
//!                      polygon vertex (DMS lat + DMS lon)
//!
//! No arcs, circles, or `V X=` centers appear in the shipped file, so
//! every airspace is a closed lat/lon polygon. Classes observed globally:
//! R (restricted, 7572), Q (danger, 4635), CTR (2854), C (2820), D (2688),
//! P (prohibited, 2120), A (725), B (603).
//!
//! This parser skips AC=Q (danger areas — advisory only, and most are in
//! Europe where they'd flood the heartbeat) and the stray AC=BRDR rows
//! that appear in the sibling `borders.txt` (political boundaries).
//! Vertices are normalized to signed decimal degrees; GND floors and
//! ceilings collapse to 0 ft MSL with `floor_is_gnd` / `ceiling_is_gnd`
//! flags preserved for display.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use anyhow::{anyhow, Context, Result};

/// Airspace classes the parser accepts. AC=Q is dropped. Any other token
/// is logged and the block is dropped too.
const ACCEPTED_CLASSES: &[&str] = &["A", "B", "C", "D", "CTR", "R", "P"];

#[derive(Debug, Clone, PartialEq)]
pub struct ParsedAirspace {
    /// Class token as it appears in the file (A/B/C/D/CTR/R/P).
    pub class: String,
    /// Free-form name from the `AN` row.
    pub name: String,
    /// Lower limit in feet MSL. 0.0 when the row was `GND`.
    pub floor_ft_msl: f64,
    /// Upper limit in feet MSL. 0.0 when the row was `GND`.
    pub ceiling_ft_msl: f64,
    pub floor_is_gnd: bool,
    pub ceiling_is_gnd: bool,
    /// Polygon vertices as (lat, lon) decimal degrees, North and East
    /// positive. The polygon is closed: the last vertex equals the first.
    pub vertices: Vec<(f64, f64)>,
    /// Axis-aligned bounding box, cached for cheap prefiltering before
    /// `ST_Contains`.
    pub min_lat: f64,
    pub max_lat: f64,
    pub min_lon: f64,
    pub max_lon: f64,
}

pub fn parse_file(path: &Path) -> Result<Vec<ParsedAirspace>> {
    let f = File::open(path).with_context(|| format!("opening {}", path.display()))?;
    parse(BufReader::new(f))
}

pub fn parse<R: BufRead>(reader: R) -> Result<Vec<ParsedAirspace>> {
    let mut out: Vec<ParsedAirspace> = Vec::new();
    let mut cur: Option<Builder> = None;

    for (lineno, line) in reader.lines().enumerate() {
        let lineno = lineno + 1;
        let line = line.with_context(|| format!("read error at line {}", lineno))?;
        let trimmed = line.trim();
        if trimmed.is_empty() {
            if let Some(b) = cur.take() {
                if let Some(a) = b.finish() {
                    out.push(a);
                }
            }
            continue;
        }
        let Some((tag, rest)) = trimmed.split_once(' ') else {
            continue;
        };
        let rest = rest.trim();
        match tag {
            "AC" => {
                if let Some(b) = cur.take() {
                    if let Some(a) = b.finish() {
                        out.push(a);
                    }
                }
                if ACCEPTED_CLASSES.iter().any(|c| c.eq_ignore_ascii_case(rest)) {
                    cur = Some(Builder::new(rest.to_string()));
                }
                // Other classes (Q, BRDR, unknown) are silently dropped.
            }
            "AN" => {
                if let Some(b) = cur.as_mut() {
                    b.name = rest.to_string();
                }
            }
            "AL" => {
                if let Some(b) = cur.as_mut() {
                    let (ft, is_gnd) = parse_altitude(rest)
                        .with_context(|| format!("parsing AL at line {}", lineno))?;
                    b.floor_ft_msl = ft;
                    b.floor_is_gnd = is_gnd;
                }
            }
            "AH" => {
                if let Some(b) = cur.as_mut() {
                    let (ft, is_gnd) = parse_altitude(rest)
                        .with_context(|| format!("parsing AH at line {}", lineno))?;
                    b.ceiling_ft_msl = ft;
                    b.ceiling_is_gnd = is_gnd;
                }
            }
            "DP" => {
                if let Some(b) = cur.as_mut() {
                    let (lat, lon) = parse_dp(rest)
                        .with_context(|| format!("parsing DP at line {}", lineno))?;
                    b.vertices.push((lat, lon));
                }
            }
            _ => { /* unknown tag, ignore */ }
        }
    }
    if let Some(b) = cur.take() {
        if let Some(a) = b.finish() {
            out.push(a);
        }
    }
    Ok(out)
}

struct Builder {
    class: String,
    name: String,
    floor_ft_msl: f64,
    ceiling_ft_msl: f64,
    floor_is_gnd: bool,
    ceiling_is_gnd: bool,
    vertices: Vec<(f64, f64)>,
}

impl Builder {
    fn new(class: String) -> Self {
        Self {
            class,
            name: String::new(),
            floor_ft_msl: 0.0,
            ceiling_ft_msl: 0.0,
            floor_is_gnd: false,
            ceiling_is_gnd: false,
            vertices: Vec::new(),
        }
    }
    fn finish(mut self) -> Option<ParsedAirspace> {
        if self.vertices.len() < 3 {
            return None;
        }
        // Close the ring if the last vertex isn't the first.
        let first = self.vertices[0];
        let last = *self.vertices.last().unwrap();
        if (first.0 - last.0).abs() > 1e-9 || (first.1 - last.1).abs() > 1e-9 {
            self.vertices.push(first);
        }
        let mut min_lat = f64::INFINITY;
        let mut max_lat = f64::NEG_INFINITY;
        let mut min_lon = f64::INFINITY;
        let mut max_lon = f64::NEG_INFINITY;
        for &(lat, lon) in &self.vertices {
            if lat < min_lat { min_lat = lat; }
            if lat > max_lat { max_lat = lat; }
            if lon < min_lon { min_lon = lon; }
            if lon > max_lon { max_lon = lon; }
        }
        Some(ParsedAirspace {
            class: self.class,
            name: self.name,
            floor_ft_msl: self.floor_ft_msl,
            ceiling_ft_msl: self.ceiling_ft_msl,
            floor_is_gnd: self.floor_is_gnd,
            ceiling_is_gnd: self.ceiling_is_gnd,
            vertices: self.vertices,
            min_lat,
            max_lat,
            min_lon,
            max_lon,
        })
    }
}

fn parse_altitude(s: &str) -> Result<(f64, bool)> {
    let s = s.trim();
    if s.eq_ignore_ascii_case("GND") {
        return Ok((0.0, true));
    }
    // "<n> MSL" — split on whitespace, first token is feet.
    let mut it = s.split_whitespace();
    let num = it
        .next()
        .ok_or_else(|| anyhow!("empty altitude"))?
        .parse::<f64>()
        .with_context(|| format!("non-numeric altitude {:?}", s))?;
    let unit = it.next().unwrap_or("MSL");
    if !unit.eq_ignore_ascii_case("MSL") {
        return Err(anyhow!("unsupported altitude reference {:?}", unit));
    }
    Ok((num, false))
}

/// Parse a DP row payload like " 26:58:59 N 080:19:42 W" into signed
/// decimal (lat, lon). Seconds field may carry a fractional part.
fn parse_dp(s: &str) -> Result<(f64, f64)> {
    // Tokens: LAT_DMS N|S LON_DMS E|W
    let toks: Vec<&str> = s.split_whitespace().collect();
    if toks.len() != 4 {
        return Err(anyhow!("DP expected 4 tokens, got {}: {:?}", toks.len(), s));
    }
    let lat_val = parse_dms(toks[0])?;
    let lat_sign = match toks[1].to_ascii_uppercase().as_str() {
        "N" => 1.0,
        "S" => -1.0,
        other => return Err(anyhow!("expected N/S, got {:?}", other)),
    };
    let lon_val = parse_dms(toks[2])?;
    let lon_sign = match toks[3].to_ascii_uppercase().as_str() {
        "E" => 1.0,
        "W" => -1.0,
        other => return Err(anyhow!("expected E/W, got {:?}", other)),
    };
    Ok((lat_sign * lat_val, lon_sign * lon_val))
}

fn parse_dms(s: &str) -> Result<f64> {
    let mut it = s.split(':');
    let d: f64 = it
        .next()
        .ok_or_else(|| anyhow!("empty DMS"))?
        .parse()
        .with_context(|| format!("degrees not numeric in {:?}", s))?;
    let m: f64 = it
        .next()
        .ok_or_else(|| anyhow!("missing minutes in {:?}", s))?
        .parse()
        .with_context(|| format!("minutes not numeric in {:?}", s))?;
    let sec: f64 = it
        .next()
        .ok_or_else(|| anyhow!("missing seconds in {:?}", s))?
        .parse()
        .with_context(|| format!("seconds not numeric in {:?}", s))?;
    Ok(d + m / 60.0 + sec / 3600.0)
}

/// Best-effort autodetect of the default X-Plane 12 airspace.txt on the
/// current host, alongside the apt.dat autodetect.
pub fn default_airspace_txt_path() -> Option<PathBuf> {
    let home = std::env::var_os("HOME")?;
    let candidates = [
        PathBuf::from(&home).join(
            "Library/Application Support/Steam/steamapps/common/X-Plane 12/\
             Resources/default data/airspaces/airspace.txt",
        ),
        PathBuf::from(
            "/Applications/X-Plane 12/Resources/default data/airspaces/airspace.txt",
        ),
    ];
    candidates.into_iter().find(|p| p.exists())
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = "\
AC D
AN GWINN
AL GND
AH 2500 MSL
DP 26:58:59 N 080:19:42 W
DP 26:58:55 N 080:18:49 W
DP 26:49:59 N 080:19:42 W

AC Q
AN EUROPE DANGER
AL GND
AH 5000 MSL
DP 50:00:00 N 010:00:00 E
DP 51:00:00 N 010:00:00 E
DP 51:00:00 N 011:00:00 E

AC B
AN SAN FRANCISCO
AL 1500 MSL
AH 10000 MSL
DP 37:30:00 N 122:30:00 W
DP 37:50:00 N 122:30:00 W
DP 37:50:00 N 122:10:00 W
DP 37:30:00 N 122:10:00 W

AC BRDR
AN Nevada
DP 39:00:00 N 120:00:00 W
DP 40:00:00 N 120:00:00 W
";

    #[test]
    fn parses_accepted_classes_only() {
        let v = parse(FIXTURE.as_bytes()).unwrap();
        let names: Vec<&str> = v.iter().map(|a| a.name.as_str()).collect();
        assert_eq!(names, vec!["GWINN", "SAN FRANCISCO"]);
    }

    #[test]
    fn gnd_floor_flagged_and_zero() {
        let v = parse(FIXTURE.as_bytes()).unwrap();
        let gwinn = v.iter().find(|a| a.name == "GWINN").unwrap();
        assert!(gwinn.floor_is_gnd);
        assert_eq!(gwinn.floor_ft_msl, 0.0);
        assert!(!gwinn.ceiling_is_gnd);
        assert_eq!(gwinn.ceiling_ft_msl, 2500.0);
    }

    #[test]
    fn dms_to_decimal_signs() {
        let v = parse(FIXTURE.as_bytes()).unwrap();
        let sf = v.iter().find(|a| a.name == "SAN FRANCISCO").unwrap();
        // 37:30:00 N => 37.5, 122:30:00 W => -122.5
        let first = sf.vertices[0];
        assert!((first.0 - 37.5).abs() < 1e-9);
        assert!((first.1 - -122.5).abs() < 1e-9);
    }

    #[test]
    fn polygon_is_closed() {
        let v = parse(FIXTURE.as_bytes()).unwrap();
        let sf = v.iter().find(|a| a.name == "SAN FRANCISCO").unwrap();
        assert_eq!(sf.vertices.first(), sf.vertices.last());
        assert_eq!(sf.vertices.len(), 5); // 4 input + 1 closure
    }

    #[test]
    fn bbox_correct() {
        let v = parse(FIXTURE.as_bytes()).unwrap();
        let sf = v.iter().find(|a| a.name == "SAN FRANCISCO").unwrap();
        assert!((sf.min_lat - 37.5).abs() < 1e-9);
        assert!((sf.max_lat - 37.833333333).abs() < 1e-6);
        assert!((sf.max_lon - -122.166666667).abs() < 1e-6);
        assert!((sf.min_lon - -122.5).abs() < 1e-9);
    }

    #[test]
    fn rejects_unknown_altitude_unit() {
        let bad = "AC D\nAN X\nAL 500 AGL\nAH 1000 MSL\nDP 0:0:0 N 0:0:0 E\n";
        let r = parse(bad.as_bytes());
        assert!(r.is_err());
    }
}
