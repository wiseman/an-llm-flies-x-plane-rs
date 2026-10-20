//! Resolve plan references (`ir::Ref`) to geometry in world-frame feet.

use std::collections::HashMap;

use anyhow::{anyhow, bail, Result};

use crate::guidance::geo::Polyline;
use crate::plan::gazetteer::{Gazetteer, Place, PlaceKind};
use crate::plan::ir::Ref;
use crate::sim::xplane_bridge::{geodetic_offset_ft, GeoReference};
use crate::types::{heading_to_vector, Vec2, NM_TO_FT};

/// Margin around an airport's runways for "north of the field" and
/// "over the field".
const AIRPORT_MARGIN_FT: f64 = 1500.0;
/// How far an extended centerline reaches beyond each runway end.
const EXTENDED_CENTERLINE_FT: f64 = 10.0 * NM_TO_FT;

/// One runway end from apt.dat.
#[derive(Debug, Clone)]
pub struct RunwayInfo {
    pub ident: String,
    /// Pavement end (apt.dat row 100), the runway frame's origin.
    pub lat: f64,
    pub lon: f64,
    /// The opposite pavement end.
    pub far_lat: f64,
    pub far_lon: f64,
    pub course_deg: f64,
    pub length_ft: f64,
    pub width_ft: f64,
    pub displaced_threshold_ft: f64,
}

#[derive(Debug, Clone)]
pub struct AirportData {
    pub ident: String,
    pub elevation_ft: f64,
    pub lat: f64,
    pub lon: f64,
    pub runways: Vec<RunwayInfo>,
}

impl AirportData {
    pub fn runway(&self, ident: &str) -> Option<&RunwayInfo> {
        let want = ident.trim().to_ascii_uppercase();
        self.runways.iter().find(|r| r.ident.eq_ignore_ascii_case(&want))
    }
}

/// Where airport data comes from: DuckDB over apt.dat in the tools,
/// a fixture in tests.
pub trait AirportSource {
    fn airport(&self, ident: &str) -> Result<AirportData>;
}

#[derive(Debug, Clone, PartialEq)]
pub enum Geom {
    Point(Vec2),
    Line(Polyline),
    /// A closed ring (not repeated at the end) plus its center.
    Area { center: Vec2, ring: Vec<Vec2> },
}

impl Geom {
    pub fn kind(&self) -> &'static str {
        match self {
            Geom::Point(_) => "point",
            Geom::Line(_) => "line",
            Geom::Area { .. } => "area",
        }
    }

    /// The point a "direct to" or "abeam" uses.
    pub fn anchor(&self) -> Vec2 {
        match self {
            Geom::Point(p) => *p,
            Geom::Line(l) => l.centroid(),
            Geom::Area { center, .. } => *center,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Feature {
    pub label: String,
    pub geom: Geom,
}

pub struct Resolver<'a> {
    pub georef: GeoReference,
    pub gazetteer: &'a Gazetteer,
    pub airports: &'a dyn AirportSource,
    cache: HashMap<String, AirportData>,
}

impl<'a> Resolver<'a> {
    pub fn new(georef: GeoReference, gazetteer: &'a Gazetteer, airports: &'a dyn AirportSource) -> Self {
        Self { georef, gazetteer, airports, cache: HashMap::new() }
    }

    pub fn airport(&mut self, ident: &str) -> Result<&AirportData> {
        let key = ident.trim().to_ascii_uppercase();
        if !self.cache.contains_key(&key) {
            let data = self.airports.airport(&key)?;
            self.cache.insert(key.clone(), data);
        }
        Ok(&self.cache[&key])
    }

    fn ft(&self, lat: f64, lon: f64) -> Vec2 {
        geodetic_offset_ft(lat, lon, self.georef)
    }

    pub fn resolve(&mut self, r: &Ref) -> Result<Feature> {
        match r {
            Ref::LatLon { lat, lon } => Ok(Feature {
                label: format!("{lat:.5},{lon:.5}"),
                geom: Geom::Point(self.ft(*lat, *lon)),
            }),
            Ref::Place { name } => {
                let place = self
                    .gazetteer
                    .find(name)
                    .ok_or_else(|| anyhow!("unknown place {name:?}"))?
                    .clone();
                Ok(Feature { label: place.name.clone(), geom: self.place_geom(&place)? })
            }
            Ref::Airport { ident } => {
                let apt = self.airport(ident)?.clone();
                let mut pts: Vec<Vec2> = Vec::new();
                for rw in &apt.runways {
                    pts.push(self.ft(rw.lat, rw.lon));
                    pts.push(self.ft(rw.far_lat, rw.far_lon));
                }
                if pts.is_empty() {
                    pts.push(self.ft(apt.lat, apt.lon));
                }
                let (mut lo, mut hi) = (pts[0], pts[0]);
                for p in &pts {
                    lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
                    hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
                }
                let m = AIRPORT_MARGIN_FT;
                let ring = vec![
                    Vec2::new(lo.x - m, lo.y - m),
                    Vec2::new(hi.x + m, lo.y - m),
                    Vec2::new(hi.x + m, hi.y + m),
                    Vec2::new(lo.x - m, hi.y + m),
                ];
                Ok(Feature {
                    label: apt.ident.clone(),
                    geom: Geom::Area { center: (lo + hi) * 0.5, ring },
                })
            }
            Ref::Runway { airport, runway, part } => {
                let apt = self.airport(airport)?.clone();
                let rw = apt
                    .runway(runway)
                    .ok_or_else(|| anyhow!("runway {runway} not found at {airport}"))?
                    .clone();
                let origin = self.ft(rw.lat, rw.lon);
                let fwd = heading_to_vector(rw.course_deg, 1.0);
                let at = |x: f64| origin + fwd * x;
                let label = format!("{airport} {runway} {}", part.replace('_', " "));
                let geom = match part.as_str() {
                    "threshold" | "numbers" => Geom::Point(at(rw.displaced_threshold_ft)),
                    "approach_end" | "pavement_end" => Geom::Point(origin),
                    "aim_point" => Geom::Point(at(aim_point_x(&rw))),
                    "midfield" => Geom::Point(at(rw.length_ft * 0.5)),
                    "departure_end" | "end" => Geom::Point(at(rw.length_ft)),
                    "centerline" => Geom::Line(Polyline::new(vec![origin, at(rw.length_ft)])),
                    "extended_centerline" => Geom::Line(Polyline::new(vec![
                        at(-EXTENDED_CENTERLINE_FT),
                        at(rw.length_ft + EXTENDED_CENTERLINE_FT),
                    ])),
                    other => bail!("unknown runway part {other:?}"),
                };
                Ok(Feature { label, geom })
            }
            Ref::RunwayFrame { airport, runway, along_ft, right_ft } => {
                let apt = self.airport(airport)?.clone();
                let rw = apt
                    .runway(runway)
                    .ok_or_else(|| anyhow!("runway {runway} not found at {airport}"))?
                    .clone();
                let origin = self.ft(rw.lat, rw.lon);
                let p = origin
                    + heading_to_vector(rw.course_deg, *along_ft)
                    + heading_to_vector(rw.course_deg + 90.0, *right_ft);
                Ok(Feature {
                    label: format!("{airport} {runway} +{:.1}/{:+.1} nm", along_ft / NM_TO_FT, right_ft / NM_TO_FT),
                    geom: Geom::Point(p),
                })
            }
            Ref::Intersection { a, b } => {
                let fa = self.resolve(a)?;
                let fb = self.resolve(b)?;
                let (Geom::Line(la), Geom::Line(lb)) = (&fa.geom, &fb.geom) else {
                    bail!("an intersection needs two lines; got {} and {}", fa.geom.kind(), fb.geom.kind());
                };
                let p = polyline_intersection(la, lb)
                    .or_else(|| polyline_junction(la, lb, JUNCTION_GAP_FT))
                    .ok_or_else(|| anyhow!("{} and {} don't meet", fa.label, fb.label))?;
                Ok(Feature { label: format!("{}/{} interchange", fa.label, fb.label), geom: Geom::Point(p) })
            }
        }
    }

    fn place_geom(&self, place: &Place) -> Result<Geom> {
        match place.kind {
            PlaceKind::Road | PlaceKind::Line => {
                if place.coords.len() < 2 {
                    bail!("{} has fewer than two points", place.name);
                }
                Ok(Geom::Line(Polyline::new(
                    place.coords.iter().map(|c| self.ft(c[0], c[1])).collect(),
                )))
            }
            PlaceKind::Point => {
                let c = place.coords.first().ok_or_else(|| anyhow!("{} has no coordinates", place.name))?;
                Ok(Geom::Point(self.ft(c[0], c[1])))
            }
            PlaceKind::Area => {
                let c = place.center.ok_or_else(|| anyhow!("{} has no center", place.name))?;
                let center = self.ft(c[0], c[1]);
                let r = place.radius_ft.unwrap_or(1000.0);
                let ring = (0..24)
                    .map(|i| center + heading_to_vector(i as f64 * 15.0, r))
                    .collect();
                Ok(Geom::Area { center, ring })
            }
        }
    }
}

/// The landing aim point's distance past the pavement end, as the
/// pattern flies it (`Runway::touchdown_x_ft` with the tool layer's
/// synthesized 1200 ft touchdown zone).
pub fn aim_point_x(rw: &RunwayInfo) -> f64 {
    crate::types::touchdown_x_ft(rw.displaced_threshold_ft, crate::plan::compile::TOUCHDOWN_ZONE_FT, rw.length_ft)
}

/// How far apart two roads' lines may end and still count as meeting:
/// a freeway that ends at another (the 118 at the 210) only touches it at
/// an endpoint, and simplified map lines leave a gap there.
const JUNCTION_GAP_FT: f64 = 2000.0;

/// Where one line ends at the other: the closest approach of either
/// line's endpoints to the other line, when within `max_gap_ft`.
pub fn polyline_junction(a: &Polyline, b: &Polyline, max_gap_ft: f64) -> Option<Vec2> {
    let ends = |l: &Polyline| [l.points.first().copied(), l.points.last().copied()].into_iter().flatten().collect::<Vec<_>>();
    let mut best: Option<(f64, Vec2)> = None;
    for (from, to) in [(a, b), (b, a)] {
        for e in ends(from) {
            if let Some(n) = to.nearest(e) {
                if n.distance_ft <= max_gap_ft && best.is_none_or(|(d, _)| n.distance_ft < d) {
                    best = Some((n.distance_ft, (e + n.point) * 0.5));
                }
            }
        }
    }
    best.map(|(_, p)| p)
}

/// First point where two polylines cross.
pub fn polyline_intersection(a: &Polyline, b: &Polyline) -> Option<Vec2> {
    for (a0, a1) in a.segments() {
        for (b0, b1) in b.segments() {
            if let Some(t) = crate::guidance::geo::segment_intersection_t(a0, a1, b0, b1) {
                return Some(a0 + (a1 - a0) * t);
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::fixtures::{kvny_georef, KvnyFixture};
    use crate::plan::gazetteer::GAZETTEER;

    #[test]
    fn resolves_places_runways_and_interchanges() {
        let src = KvnyFixture;
        let mut r = Resolver::new(kvny_georef(), &GAZETTEER, &src);
        let road = r.resolve(&Ref::Place { name: "the 118".into() }).unwrap();
        let Geom::Line(l) = &road.geom else { panic!("118 should be a line") };
        // The 118 runs east-west about 3.5 nm north of the field.
        let n = l.nearest(Vec2::ZERO).unwrap();
        assert!(n.point.y > 3.0 * NM_TO_FT && n.point.y < 4.5 * NM_TO_FT, "{:?}", n.point);

        let thr = r
            .resolve(&Ref::Runway { airport: "KVNY".into(), runway: "16R".into(), part: "threshold".into() })
            .unwrap();
        let Geom::Point(p) = thr.geom else { panic!() };
        assert!(p.y > 1500.0 && p.y < 4000.0, "16R (displaced) threshold north of the ARP: {p:?}");

        let ix = r
            .resolve(&Ref::Intersection {
                a: Box::new(Ref::Place { name: "I-405".into() }),
                b: Box::new(Ref::Place { name: "US-101".into() }),
            })
            .unwrap();
        let Geom::Point(ix) = ix.geom else { panic!() };
        // South-east of the field, near the Sepulveda Basin.
        assert!(ix.y < -2.0 * NM_TO_FT && ix.x > 0.0, "{ix:?}");

        // The 118 ends at the 210 (east of the field, north of Whiteman):
        // the lines only touch at an endpoint.
        let ix = r
            .resolve(&Ref::Intersection {
                a: Box::new(Ref::Place { name: "the 210".into() }),
                b: Box::new(Ref::Place { name: "the 118".into() }),
            })
            .unwrap();
        let Geom::Point(ix) = ix.geom else { panic!() };
        assert!(ix.x > 3.0 * NM_TO_FT && ix.y > 3.0 * NM_TO_FT, "{ix:?}");
        // Parallel roads miles apart still don't meet.
        assert!(r
            .resolve(&Ref::Intersection {
                a: Box::new(Ref::Place { name: "the 118".into() }),
                b: Box::new(Ref::Place { name: "the 101".into() }),
            })
            .is_err());

        assert!(r.resolve(&Ref::Place { name: "Eiffel Tower".into() }).is_err());
    }
}
