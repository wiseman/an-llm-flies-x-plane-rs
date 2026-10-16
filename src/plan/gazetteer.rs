//! Hardcoded place lists (`config/places/*.yaml`, generated from OSM by
//! `scripts/places/build_places.py`): the roads, landmarks and local
//! nicknames controllers use, until geography comes from OSM directly.

use anyhow::{Context, Result};
use once_cell::sync::Lazy;
use serde::Deserialize;

const EMBEDDED: &[(&str, &str)] = &[
    ("san_fernando_valley", include_str!("../../config/places/san_fernando_valley.yaml")),
    ("santa_monica", include_str!("../../config/places/santa_monica.yaml")),
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlaceKind {
    Road,
    Line,
    Point,
    Area,
}

impl PlaceKind {
    pub fn is_line(self) -> bool {
        matches!(self, PlaceKind::Road | PlaceKind::Line)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            PlaceKind::Road => "road",
            PlaceKind::Line => "line",
            PlaceKind::Point => "point",
            PlaceKind::Area => "area",
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct Place {
    pub name: String,
    pub kind: PlaceKind,
    #[serde(default)]
    pub aliases: Vec<String>,
    /// `[lat, lon]` pairs: a polyline for roads/lines, one point for points.
    #[serde(default)]
    pub coords: Vec<[f64; 2]>,
    /// Areas: center and rough radius.
    #[serde(default)]
    pub center: Option<[f64; 2]>,
    #[serde(default)]
    pub radius_ft: Option<f64>,
    /// Filled from the file's `region:` on load.
    #[serde(skip)]
    pub region: String,
}

impl Place {
    /// Representative `[lat, lon]`: the area center, the point, or the
    /// middle vertex of a line.
    pub fn anchor(&self) -> [f64; 2] {
        if let Some(c) = self.center {
            return c;
        }
        self.coords.get(self.coords.len() / 2).copied().unwrap_or([0.0, 0.0])
    }

    fn matches(&self, query: &str) -> bool {
        let q = normalize(query);
        normalize(&self.name) == q || self.aliases.iter().any(|a| normalize(a) == q)
    }
}

#[derive(Debug, Deserialize)]
struct PlaceFile {
    region: String,
    /// Airports the region's places are near (documentation only; every
    /// list is searched).
    #[serde(default)]
    #[allow(dead_code)]
    airports: Vec<String>,
    places: Vec<Place>,
}

#[derive(Debug, Clone, Default)]
pub struct Gazetteer {
    pub places: Vec<Place>,
}

/// Every embedded place list, merged.
pub static GAZETTEER: Lazy<Gazetteer> =
    Lazy::new(|| Gazetteer::from_embedded().expect("embedded place lists parse"));

impl Gazetteer {
    pub fn from_embedded() -> Result<Self> {
        let mut places = Vec::new();
        for (ident, text) in EMBEDDED {
            let file: PlaceFile = serde_yaml_ng::from_str(text)
                .with_context(|| format!("parsing config/places/{ident}.yaml"))?;
            for mut p in file.places {
                p.region = file.region.clone();
                places.push(p);
            }
        }
        Ok(Self { places })
    }

    pub fn find(&self, query: &str) -> Option<&Place> {
        self.places.iter().find(|p| p.matches(query))
    }
}

/// Case-, article- and punctuation-insensitive form of a name, so "the
/// 118", "118" and "CA 118" can share aliases without listing every
/// spelling.
pub fn normalize(s: &str) -> String {
    let lower = s.to_ascii_lowercase();
    let cleaned: String = lower
        .chars()
        .filter(|c| !matches!(c, '.' | '\''))
        .map(|c| if c.is_ascii_alphanumeric() { c } else { ' ' })
        .collect();
    cleaned
        .split_whitespace()
        .filter(|w| *w != "the")
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_lists_parse_and_aliases_resolve() {
        let g = &*GAZETTEER;
        assert!(g.places.len() >= 60);
        let mut names: Vec<String> = g.places.iter().map(|p| normalize(&p.name)).collect();
        names.sort();
        let n = names.len();
        names.dedup();
        assert_eq!(n, names.len(), "duplicate place names");
        // No alias may name two places.
        let mut seen = std::collections::HashMap::new();
        for p in &g.places {
            for a in p.aliases.iter().chain(std::iter::once(&p.name)) {
                if let Some(other) = seen.insert(normalize(a), p.name.clone()) {
                    assert_eq!(other, p.name, "alias {a:?} names both {other} and {}", p.name);
                }
            }
        }
        assert_eq!(g.find("the shoreline").unwrap().name, "Shoreline");
        assert_eq!(g.find("the 210").unwrap().name, "I-210");
        assert_eq!(g.find("the 118").unwrap().name, "CA-118");
        assert_eq!(g.find("The VA").unwrap().name, "VA Sepulveda Ambulatory Care Center");
        assert_eq!(g.find("the flood basin").unwrap().name, "Sepulveda Basin");
        assert_eq!(g.find("us-101").unwrap().kind, PlaceKind::Road);
        assert!(g.find("Eiffel Tower").is_none());
    }
}
