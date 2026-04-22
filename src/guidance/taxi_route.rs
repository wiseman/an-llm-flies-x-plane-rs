//! Taxi-route planner over an airport's apt.dat taxi network. No Python
//! counterpart (the Python pilot does not taxi).
//!
//! Phase 1: planning only — produce a `TaxiPlan` the LLM can return to the
//! user or feed into a future `engage_taxi` tool. No control-loop
//! engagement, no nose-wheel steering, no hold-short enforcement.
//!
//! ## Routing semantics
//!
//! `plan()` runs a constrained Dijkstra where the search state is
//! `(node_id, stage_index)` and `stage_index` walks the caller's
//! `via_taxiways` list in order:
//!
//! - `stage = -1` (encoded as 0 here, with real stages offset by one)
//!   means "before the first named taxiway" — the algorithm allows any
//!   connector edge, which covers the common case of taxiing off a ramp
//!   onto the first named taxiway.
//! - `stage = i` (0..len) means "currently on `via_taxiways[i]`" — only
//!   edges named `via_taxiways[i]` (stay) or `via_taxiways[i+1]`
//!   (advance) are legal.
//! - After the last stage, any edge is legal again so the path can exit
//!   onto the runway hold-short / gate lead-out.
//!
//! If `via_taxiways` is empty the search degenerates to plain shortest
//! path, which is useful for "taxi to runway 31 via the shortest route".

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

use anyhow::{anyhow, bail, Context, Result};
use duckdb::Connection;

const EARTH_RADIUS_M: f64 = 6_371_008.8;

#[derive(Debug, Clone)]
pub struct TaxiNode {
    pub node_id: i64,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub usage: String,
}

#[derive(Debug, Clone)]
pub struct TaxiEdge {
    pub from_node: i64,
    pub to_node: i64,
    pub direction: String,
    /// Raw apt.dat 1202 category ("runway", "taxiway_A".."taxiway_F").
    /// `"runway"` tags pavement that is the runway surface itself — these
    /// edges are what backtaxi-on-runway detection keys on.
    pub category: String,
    pub name: String,
    pub active_zones: String,
}

pub struct TaxiGraph {
    pub airport_ident: String,
    pub nodes: HashMap<i64, TaxiNode>,
    pub edges: Vec<TaxiEdge>,
    /// Adjacency list: node → list of (neighbour_node, edge_index).
    adj: HashMap<i64, Vec<(i64, usize)>>,
}

#[derive(Debug, Clone)]
pub struct TaxiLeg {
    pub from_node: i64,
    pub to_node: i64,
    pub from_lat: f64,
    pub from_lon: f64,
    pub to_lat: f64,
    pub to_lon: f64,
    pub taxiway_name: String,
    pub distance_m: f64,
    pub active_zones: String,
}

#[derive(Debug, Clone)]
pub struct TaxiPlan {
    pub airport_ident: String,
    pub start_node: i64,
    pub destination_node: i64,
    pub legs: Vec<TaxiLeg>,
    pub total_distance_m: f64,
    /// Ordered, de-duped list of taxiway names the plan actually traverses
    /// ("A → D → hold_31"). Useful for echoing the clearance back to the
    /// user.
    pub taxiway_sequence: Vec<String>,
    /// Runway active-zone crossings encountered along the way. Each entry
    /// is the raw `"departure:01L,01R"` / `"ils:28L"` token from
    /// apt.dat's 1204 row, attached to the leg that triggered it.
    pub runway_crossings: Vec<String>,
}

#[derive(Debug)]
pub enum TaxiDestination<'a> {
    /// Resolve to the taxi node nearest the runway's threshold. For a
    /// runway printed as "13/31" either end's ident ("13" or "31") works.
    Runway(&'a str),
    /// Resolve to the hold-short node at the specific taxiway/runway
    /// intersection — e.g. ("19", "C") for "hold short of 19 at Charlie"
    /// at KEMT. Useful for intersection-departure clearances where the
    /// full-length threshold is the wrong target.
    RunwayIntersection {
        runway: &'a str,
        taxiway: &'a str,
    },
    /// Nearest taxi node to a raw lat/lon (e.g. a gate).
    Coord(f64, f64),
    /// Explicit node id, when the caller already knows the destination.
    NodeId(i64),
}

pub fn load_graph(conn: &Connection, airport_ident: &str) -> Result<TaxiGraph> {
    let mut nodes: HashMap<i64, TaxiNode> = HashMap::new();
    let mut stmt = conn.prepare(
        "SELECT node_id, latitude_deg, longitude_deg, usage \
         FROM taxi_nodes WHERE airport_ident = ?",
    )?;
    let mut rows = stmt.query([airport_ident])?;
    while let Some(row) = rows.next()? {
        let id: i64 = row.get::<_, i32>(0)? as i64;
        nodes.insert(
            id,
            TaxiNode {
                node_id: id,
                latitude_deg: row.get(1)?,
                longitude_deg: row.get(2)?,
                usage: row.get(3)?,
            },
        );
    }
    drop(rows);
    drop(stmt);

    let mut edges: Vec<TaxiEdge> = Vec::new();
    let mut stmt = conn.prepare(
        "SELECT from_node, to_node, direction, category, name, active_zones \
         FROM taxi_edges WHERE airport_ident = ?",
    )?;
    let mut rows = stmt.query([airport_ident])?;
    while let Some(row) = rows.next()? {
        edges.push(TaxiEdge {
            from_node: row.get::<_, i32>(0)? as i64,
            to_node: row.get::<_, i32>(1)? as i64,
            direction: row.get(2)?,
            category: row.get(3)?,
            name: row.get(4)?,
            active_zones: row.get(5)?,
        });
    }

    let mut adj: HashMap<i64, Vec<(i64, usize)>> = HashMap::new();
    for (idx, e) in edges.iter().enumerate() {
        adj.entry(e.from_node).or_default().push((e.to_node, idx));
        if e.direction == "twoway" {
            adj.entry(e.to_node).or_default().push((e.from_node, idx));
        }
    }
    Ok(TaxiGraph {
        airport_ident: airport_ident.to_string(),
        nodes,
        edges,
        adj,
    })
}

/// Nearest node to `(lat, lon)` that is attached to at least one taxi edge.
///
/// apt.dat row 1201 emits nodes for both the aircraft taxi network (row
/// 1202 edges) and the separate ground-vehicle network (row 1206), and the
/// node-id space is shared. Unrestricted nearest-node would happily snap
/// the aircraft to a baggage-cart-only node and the planner would report
/// "no route". Filtering by `graph.adj` (which is built from 1202 edges
/// only) avoids that trap.
pub fn nearest_node(graph: &TaxiGraph, lat: f64, lon: f64) -> Result<i64> {
    let mut best: Option<(i64, f64)> = None;
    for (id, neighbors) in &graph.adj {
        if neighbors.is_empty() {
            continue;
        }
        let Some(n) = graph.nodes.get(id) else {
            continue;
        };
        let d = haversine_m(lat, lon, n.latitude_deg, n.longitude_deg);
        if best.as_ref().map(|(_, bd)| d < *bd).unwrap_or(true) {
            best = Some((*id, d));
        }
    }
    best.map(|(id, _)| id)
        .ok_or_else(|| anyhow!("airport {} has no aircraft-taxiable nodes", graph.airport_ident))
}

/// One valid stopping node for a runway hold-short. A full-length runway
/// hold-short can have two or more: one on each side of the runway plus
/// any connector hold-short nodes. Each carries its own `face_toward`
/// (the node on the other side of the forbidden edge leaving it) because
/// the runway-facing heading is side-specific.
#[derive(Debug, Clone)]
pub struct HoldShortCandidate {
    pub node: i64,
    pub face_toward_latlon: Option<(f64, f64)>,
}

/// The resolved taxi goal plus any edges the planner must refuse to cross
/// to reach it. For a runway destination, `forbidden_edges` contains every
/// 1202 edge whose 1204 active-zone list names the target runway —
/// crossing those is what would put the aircraft onto the runway. For a
/// coord / node destination, the set is empty.
///
/// `face_toward_latlon`, when set, names a world point just inside the
/// active zone (the node on the other side of the forbidden edge leaving
/// the hold-short node). Callers that actually execute the taxi can use
/// it to append a short orientation leg so the aircraft ends up *facing*
/// the runway at the hold-short line instead of parked parallel to it
/// along the taxiway.
///
/// `alt_nodes` carries the remaining valid hold-short candidates (those
/// not chosen as the primary `node`). Pass primary + alts into `plan()`
/// so it picks whichever is cheapest from the aircraft's start — without
/// this, the planner may detour around the whole airport just to reach a
/// geometrically-closer-to-threshold node on the far side of the runway.
#[derive(Debug, Clone, Default)]
pub struct DestinationResolution {
    pub node: i64,
    pub forbidden_edges: HashSet<usize>,
    pub face_toward_latlon: Option<(f64, f64)>,
    pub alt_nodes: Vec<HoldShortCandidate>,
}

impl DestinationResolution {
    /// Primary + alternate candidate node ids, for feeding into `plan`.
    pub fn all_node_ids(&self) -> Vec<i64> {
        let mut out = Vec::with_capacity(1 + self.alt_nodes.len());
        out.push(self.node);
        out.extend(self.alt_nodes.iter().map(|c| c.node));
        out
    }

    /// Look up the face-toward target for whichever candidate the planner
    /// actually terminated at. Returns the primary's face when `chosen`
    /// matches the primary, otherwise searches `alt_nodes`.
    pub fn face_toward_for(&self, chosen: i64) -> Option<(f64, f64)> {
        if chosen == self.node {
            return self.face_toward_latlon;
        }
        self.alt_nodes
            .iter()
            .find(|c| c.node == chosen)
            .and_then(|c| c.face_toward_latlon)
    }
}

pub fn resolve_destination(
    conn: &Connection,
    graph: &TaxiGraph,
    airport_ident: &str,
    dest: &TaxiDestination,
) -> Result<DestinationResolution> {
    match *dest {
        TaxiDestination::NodeId(id) => {
            if graph.nodes.contains_key(&id) {
                Ok(DestinationResolution {
                    node: id,
                    forbidden_edges: HashSet::new(),
                    face_toward_latlon: None,
                    alt_nodes: Vec::new(),
                })
            } else {
                Err(anyhow!("node {} not in taxi graph for {}", id, airport_ident))
            }
        }
        TaxiDestination::Coord(lat, lon) => Ok(DestinationResolution {
            node: nearest_node(graph, lat, lon)?,
            forbidden_edges: HashSet::new(),
            face_toward_latlon: None,
            alt_nodes: Vec::new(),
        }),
        TaxiDestination::Runway(runway_ident) => resolve_runway_hold_short(
            conn,
            graph,
            airport_ident,
            runway_ident,
        ),
        TaxiDestination::RunwayIntersection { runway, taxiway } => {
            resolve_runway_intersection_hold_short(
                conn,
                graph,
                airport_ident,
                runway,
                taxiway,
            )
        }
    }
}

/// Resolved intersection of a taxiway with a runway. `on_runway_node` is
/// the endpoint sitting on the runway centerline (takeoff-roll start
/// point for an intersection departure); `hold_short_node` is the
/// corresponding off-runway endpoint.
#[derive(Debug, Clone, Copy)]
pub struct RunwayIntersection {
    pub on_runway_node: i64,
    pub on_runway_lat: f64,
    pub on_runway_lon: f64,
    pub hold_short_node: i64,
    pub hold_short_lat: f64,
    pub hold_short_lon: f64,
}

/// Find the edge named `taxiway` that crosses the active zone of
/// `runway_ident`, and split its endpoints into "on runway" vs
/// "hold short" by perpendicular distance to the runway centerline
/// (read from the `runways` view). Used by both
/// `resolve_runway_intersection_hold_short` (for engage_taxi) and
/// the line-up handler (for the takeoff-roll anchor).
pub fn resolve_runway_intersection(
    conn: &Connection,
    graph: &TaxiGraph,
    airport_ident: &str,
    runway_ident: &str,
    taxiway: &str,
) -> Result<RunwayIntersection> {
    // 1. Look up the runway's two threshold endpoints so we have a line
    //    segment to measure perpendicular distance against.
    let forms = runway_query_forms(runway_ident);
    let (le_lat, le_lon, he_lat, he_lon): (f64, f64, f64, f64) = conn
        .query_row(
            "SELECT le_latitude_deg, le_longitude_deg, \
                    he_latitude_deg, he_longitude_deg \
             FROM runways \
             WHERE airport_ident = ? \
               AND (le_ident = ? OR le_ident = ? OR he_ident = ? OR he_ident = ?) \
               AND closed = 0 \
             LIMIT 1",
            [
                airport_ident,
                forms[0].as_str(),
                forms[1].as_str(),
                forms[0].as_str(),
                forms[1].as_str(),
            ],
            |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?)),
        )
        .with_context(|| {
            format!(
                "runway {:?} not found at airport {}",
                runway_ident, airport_ident
            )
        })?;

    // 2. Among edges named `taxiway`, pick one whose active-zone list
    //    names this runway. That's the crossing edge — one endpoint is
    //    on the runway, the other is the hold-short.
    let crossing: &TaxiEdge = graph
        .edges
        .iter()
        .find(|e| {
            e.name.eq_ignore_ascii_case(taxiway) && edge_conflicts_with_runway(e, runway_ident)
        })
        .ok_or_else(|| {
            anyhow!(
                "no edge named {:?} crosses runway {:?} at {} \
                 (apt.dat has no 1204 active-zone annotation for that intersection)",
                taxiway,
                runway_ident,
                airport_ident
            )
        })?;

    let from_node = graph
        .nodes
        .get(&crossing.from_node)
        .ok_or_else(|| anyhow!("crossing edge from_node missing"))?;
    let to_node = graph
        .nodes
        .get(&crossing.to_node)
        .ok_or_else(|| anyhow!("crossing edge to_node missing"))?;

    // 3. On-runway = closer to the runway centerline segment.
    let d_from = point_to_segment_m(
        from_node.latitude_deg,
        from_node.longitude_deg,
        le_lat,
        le_lon,
        he_lat,
        he_lon,
    );
    let d_to = point_to_segment_m(
        to_node.latitude_deg,
        to_node.longitude_deg,
        le_lat,
        le_lon,
        he_lat,
        he_lon,
    );
    let (on_runway, hold_short) = if d_from <= d_to {
        (from_node, to_node)
    } else {
        (to_node, from_node)
    };

    Ok(RunwayIntersection {
        on_runway_node: on_runway.node_id,
        on_runway_lat: on_runway.latitude_deg,
        on_runway_lon: on_runway.longitude_deg,
        hold_short_node: hold_short.node_id,
        hold_short_lat: hold_short.latitude_deg,
        hold_short_lon: hold_short.longitude_deg,
    })
}

/// Resolved parking destination: the 1300 spot's truth coordinates plus
/// the nearest-on-network taxi node the planner should terminate at.
///
/// Parking spots aren't graph nodes — they're free-floating lat/lons
/// alongside the taxiway network. `tool_engage_park` plans to
/// `nearest_node` and then appends a short lead-in leg from the node to
/// `spot_lat/lon` so the aircraft rolls up to the spot itself, with a
/// final pose aligned to `spot_heading_true_deg`.
#[derive(Debug, Clone)]
pub struct ParkingResolution {
    pub nearest_node: i64,
    pub spot_lat: f64,
    pub spot_lon: f64,
    pub spot_heading_true_deg: f64,
    pub spot_kind: String,
    pub spot_name: String,
}

/// Look up a parking spot by exact (case-insensitive) name at the given
/// airport, then snap to the nearest aircraft-taxi-network node via
/// `nearest_node` (which already filters out 1206-only nodes).
///
/// Name match is exact-case-insensitive. Two spots with the same name at
/// the same airport is rare but possible — the first one DuckDB returns
/// wins. Callers wanting disambiguation can pass a more specific string
/// or fall back to sql_query.
pub fn resolve_parking_destination(
    conn: &Connection,
    graph: &TaxiGraph,
    airport_ident: &str,
    parking_name: &str,
) -> Result<ParkingResolution> {
    let (lat, lon, hdg, kind, name): (f64, f64, f64, String, String) = conn
        .query_row(
            "SELECT latitude_deg, longitude_deg, heading_true_deg, kind, name \
             FROM parking_spots \
             WHERE airport_ident = ? AND lower(name) = lower(?) \
             LIMIT 1",
            [airport_ident, parking_name],
            |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?, r.get(4)?)),
        )
        .with_context(|| {
            format!(
                "parking spot {:?} not found at airport {}",
                parking_name, airport_ident
            )
        })?;
    let nearest_node = nearest_node(graph, lat, lon)?;
    Ok(ParkingResolution {
        nearest_node,
        spot_lat: lat,
        spot_lon: lon,
        spot_heading_true_deg: hdg,
        spot_kind: kind,
        spot_name: name,
    })
}

fn resolve_runway_intersection_hold_short(
    conn: &Connection,
    graph: &TaxiGraph,
    airport_ident: &str,
    runway_ident: &str,
    taxiway: &str,
) -> Result<DestinationResolution> {
    let x = resolve_runway_intersection(conn, graph, airport_ident, runway_ident, taxiway)?;
    // Full runway forbidden-edge set — same as `resolve_runway_hold_short`
    // — so Dijkstra won't sneak across any other intersection to reach
    // this hold-short.
    let forbidden_edges: HashSet<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| edge_conflicts_with_runway(e, runway_ident))
        .map(|(i, _)| i)
        .collect();
    Ok(DestinationResolution {
        node: x.hold_short_node,
        forbidden_edges,
        face_toward_latlon: Some((x.on_runway_lat, x.on_runway_lon)),
        alt_nodes: Vec::new(),
    })
}

/// Perpendicular distance in meters from (plat, plon) to the great-circle
/// segment between (alat, alon) and (blat, blon), using a local flat-earth
/// approximation. Accurate to sub-meter at taxi-network scales.
fn point_to_segment_m(plat: f64, plon: f64, alat: f64, alon: f64, blat: f64, blon: f64) -> f64 {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * alat.to_radians().cos();
    let ax = (alon - plon) * m_per_deg_lon;
    let ay = (alat - plat) * m_per_deg_lat;
    let bx = (blon - plon) * m_per_deg_lon;
    let by = (blat - plat) * m_per_deg_lat;
    let dx = bx - ax;
    let dy = by - ay;
    let len_sq = dx * dx + dy * dy;
    if len_sq < 1e-9 {
        return (ax * ax + ay * ay).sqrt();
    }
    let t = ((-ax) * dx + (-ay) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);
    let cx = ax + t * dx;
    let cy = ay + t * dy;
    (cx * cx + cy * cy).sqrt()
}

/// "Taxi to runway X" = hold short of X. Picks a node on the approach
/// side of the runway and returns the set of 1202 edges the planner must
/// not traverse (those with a 1204 active zone naming X). Without the
/// forbidden set Dijkstra would happily cut across the runway and park
/// on the far grass.
///
/// Candidate selection: of the nodes incident on at least one forbidden
/// edge, pick the one closest to the runway's threshold lat/lon from the
/// runways table. That's the node painted with the hold-short line for
/// the threshold — on the far end of a pavement edge that otherwise
/// enters the runway's active zone.
///
/// Fallback: when apt.dat doesn't annotate any edge for this runway
/// (small fields), resolve to the plain nearest taxi node to the
/// threshold and return no forbidden edges.
fn resolve_runway_hold_short(
    conn: &Connection,
    graph: &TaxiGraph,
    airport_ident: &str,
    runway_ident: &str,
) -> Result<DestinationResolution> {
    let forms = runway_query_forms(runway_ident);
    let (lat, lon): (f64, f64) = conn
        .query_row(
            "SELECT \
                 CASE WHEN le_ident = ? OR le_ident = ? \
                      THEN le_latitude_deg  ELSE he_latitude_deg  END, \
                 CASE WHEN le_ident = ? OR le_ident = ? \
                      THEN le_longitude_deg ELSE he_longitude_deg END \
             FROM runways \
             WHERE airport_ident = ? \
               AND (le_ident = ? OR le_ident = ? OR he_ident = ? OR he_ident = ?) \
               AND closed = 0 \
             LIMIT 1",
            [
                forms[0].as_str(),
                forms[1].as_str(),
                forms[0].as_str(),
                forms[1].as_str(),
                airport_ident,
                forms[0].as_str(),
                forms[1].as_str(),
                forms[0].as_str(),
                forms[1].as_str(),
            ],
            |r| Ok((r.get(0)?, r.get(1)?)),
        )
        .with_context(|| {
            format!(
                "runway {:?} not found at airport {}",
                runway_ident, airport_ident
            )
        })?;

    let forbidden_edges: HashSet<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| edge_conflicts_with_runway(e, runway_ident))
        .map(|(i, _)| i)
        .collect();

    if forbidden_edges.is_empty() {
        return Ok(DestinationResolution {
            node: nearest_node(graph, lat, lon)?,
            forbidden_edges,
            face_toward_latlon: None,
            alt_nodes: Vec::new(),
        });
    }

    // A node is a real hold-short candidate only if it has at least one
    // non-forbidden adjacent edge — i.e. it sits at the *boundary* of the
    // active zone with a path leading away from the runway. Nodes whose
    // every adjacent edge is forbidden live entirely inside the active
    // zone (e.g. the far end of the runway-crossing edge, or the node
    // that is the runway centerline itself) and are unreachable without
    // crossing.
    let candidate_ids: HashSet<i64> = forbidden_edges
        .iter()
        .flat_map(|&i| [graph.edges[i].from_node, graph.edges[i].to_node])
        .filter(|id| {
            graph
                .adj
                .get(id)
                .map(|adj| {
                    adj.iter()
                        .any(|(_, eid)| !forbidden_edges.contains(eid))
                })
                .unwrap_or(false)
        })
        .collect();

    // Each boundary node gets its own face-toward target, computed from a
    // forbidden edge incident on it — that edge's far endpoint sits on the
    // runway side, so aiming there faces the aircraft toward the runway.
    let face_toward = |node_id: i64| -> Option<(f64, f64)> {
        forbidden_edges.iter().find_map(|&i| {
            let e = &graph.edges[i];
            let other = if e.from_node == node_id {
                Some(e.to_node)
            } else if e.to_node == node_id {
                Some(e.from_node)
            } else {
                None
            }?;
            let n = graph.nodes.get(&other)?;
            Some((n.latitude_deg, n.longitude_deg))
        })
    };

    // Rank by distance to the threshold; primary = nearest, rest ride
    // along as `alt_nodes`. The planner picks whichever is cheapest by
    // path cost, so a candidate on the aircraft's side of the runway
    // wins over a slightly-nearer one that would require looping around.
    let mut ranked: Vec<(i64, f64)> = candidate_ids
        .iter()
        .filter_map(|id| {
            graph.nodes.get(id).map(|n| {
                (*id, haversine_m(lat, lon, n.latitude_deg, n.longitude_deg))
            })
        })
        .collect();
    ranked.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal));
    if ranked.is_empty() {
        return Err(anyhow!(
            "runway {:?} at {} has active-zone edges but no reachable hold-short node (every boundary node is inside the zone)",
            runway_ident,
            airport_ident
        ));
    }
    let primary = ranked[0].0;
    let primary_face = face_toward(primary);
    let alt_nodes: Vec<HoldShortCandidate> = ranked[1..]
        .iter()
        .map(|(id, _)| HoldShortCandidate {
            node: *id,
            face_toward_latlon: face_toward(*id),
        })
        .collect();
    let _ = face_toward;

    Ok(DestinationResolution {
        node: primary,
        forbidden_edges,
        face_toward_latlon: primary_face,
        alt_nodes,
    })
}

/// True when any 1204 active-zone entry on this edge names the given
/// runway (matched against both the exact ident and its zero-padded or
/// -unpadded alternate form).
pub fn edge_conflicts_with_runway(edge: &TaxiEdge, runway: &str) -> bool {
    if edge.active_zones.is_empty() {
        return false;
    }
    let forms = runway_query_forms(runway);
    for entry in edge.active_zones.split(';') {
        let Some((_kind, runways)) = entry.split_once(':') else {
            continue;
        };
        for r in runways.split(',') {
            let r = r.trim();
            if forms.iter().any(|f| f == r) {
                return true;
            }
        }
    }
    false
}

/// Build the directional-forbid set that keeps a taxi plan from
/// backtaxiing on a runway.
///
/// For every `category == "runway"` edge whose `name` is not in
/// `allowed_runway_names` (i.e. the LLM did not explicitly name the
/// runway in `via_taxiways`), we look at the edge's bearing from
/// `from_node → to_node` and compare it to `aircraft_heading_deg`. The
/// direction whose bearing is within 90° of the heading is "forward" —
/// rolling out along the runway — and stays legal. The opposite
/// direction is the backtaxi and gets added to the returned set as a
/// forbidden `(from_node, to_node)` traversal.
///
/// Plain forbidden `HashSet<usize>` can't express this — a runway edge
/// is `twoway` and the edge index is the same for both traversal
/// directions — so the planner takes this directional set separately.
pub fn forbid_backward_runway_traversals(
    graph: &TaxiGraph,
    aircraft_heading_deg: f64,
    allowed_runway_names: &HashSet<&str>,
) -> HashSet<(i64, i64)> {
    let mut out: HashSet<(i64, i64)> = HashSet::new();
    for e in &graph.edges {
        if e.category != "runway" {
            continue;
        }
        if allowed_runway_names.contains(e.name.as_str()) {
            continue;
        }
        let Some(n1) = graph.nodes.get(&e.from_node) else {
            continue;
        };
        let Some(n2) = graph.nodes.get(&e.to_node) else {
            continue;
        };
        let bearing_forward = initial_bearing_deg(
            n1.latitude_deg, n1.longitude_deg,
            n2.latitude_deg, n2.longitude_deg,
        );
        let delta = crate::types::wrap_degrees_180(bearing_forward - aircraft_heading_deg).abs();
        if delta < 90.0 {
            // from→to aligns with the aircraft heading → reverse (to→from)
            // is the backtaxi direction.
            out.insert((e.to_node, e.from_node));
        } else {
            // to→from is the forward (heading-aligned) direction; forbid
            // from→to.
            out.insert((e.from_node, e.to_node));
        }
    }
    out
}

/// Initial bearing from (lat1, lon1) to (lat2, lon2) in degrees true,
/// 0..360 (0 = due north).
fn initial_bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let p1 = lat1.to_radians();
    let p2 = lat2.to_radians();
    let dlam = (lon2 - lon1).to_radians();
    let y = dlam.sin() * p2.cos();
    let x = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dlam.cos();
    let b = y.atan2(x).to_degrees();
    (b + 360.0) % 360.0
}

/// Constrained shortest-path planner. `via_taxiways` empty → plain
/// Dijkstra. `forbidden_edges` (usually supplied by `resolve_destination`
/// for runway goals) are never relaxed — that's how we stop *at* a
/// hold-short line instead of cutting across the runway.
///
/// `forbidden_directional` forbids specific `(from_node, to_node)`
/// traversals — used by `forbid_backward_runway_traversals` to allow
/// rolling forward along a runway while still blocking a U-turn-and-
/// backtaxi on the same pavement.
///
/// `destination_nodes` is a non-empty list of acceptable goals — the
/// planner terminates at whichever is cheapest from `start_node` and
/// records it in the returned `TaxiPlan.destination_node`. For a full-
/// length runway hold-short, pass every valid hold-short node on both
/// sides of the runway; Dijkstra's optimality guarantees the one chosen
/// is the minimum-cost, which avoids the "3 m closer to threshold but
/// 4 km further by taxiway" trap.
pub fn plan(
    graph: &TaxiGraph,
    start_node: i64,
    destination_nodes: &[i64],
    via_taxiways: &[String],
    forbidden_edges: &HashSet<usize>,
    forbidden_directional: &HashSet<(i64, i64)>,
) -> Result<TaxiPlan> {
    if !graph.nodes.contains_key(&start_node) {
        bail!("start node {} not in taxi graph for {}", start_node, graph.airport_ident);
    }
    if destination_nodes.is_empty() {
        bail!(
            "plan() called with empty destination_nodes for {}",
            graph.airport_ident
        );
    }
    for d in destination_nodes {
        if !graph.nodes.contains_key(d) {
            bail!(
                "destination node {} not in taxi graph for {}",
                d,
                graph.airport_ident
            );
        }
    }
    let dest_set: HashSet<i64> = destination_nodes.iter().copied().collect();

    // Stage semantics: stage 0 = "before first named taxiway"; stage i in
    // 1..=len = "on via_taxiways[i-1]"; stage len+1 = "past last taxiway".
    // This keeps indices non-negative so we can fit them into a u8.
    let target_stage = (via_taxiways.len() + 1) as u8;

    // Priority queue entry: (dist_m, node, stage).
    #[derive(Copy, Clone)]
    struct Entry {
        dist: f64,
        node: i64,
        stage: u8,
    }
    impl Eq for Entry {}
    impl PartialEq for Entry {
        fn eq(&self, o: &Self) -> bool {
            self.dist == o.dist
        }
    }
    impl PartialOrd for Entry {
        fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
            // Min-heap via reversed comparison.
            o.dist.partial_cmp(&self.dist)
        }
    }
    impl Ord for Entry {
        fn cmp(&self, o: &Self) -> Ordering {
            self.partial_cmp(o).unwrap_or(Ordering::Equal)
        }
    }

    type StateKey = (i64, u8);
    let mut best: HashMap<StateKey, f64> = HashMap::new();
    let mut came_from: HashMap<StateKey, (StateKey, usize)> = HashMap::new();
    let mut heap: BinaryHeap<Entry> = BinaryHeap::new();

    let start_key = (start_node, 0u8);
    best.insert(start_key, 0.0);
    heap.push(Entry {
        dist: 0.0,
        node: start_node,
        stage: 0,
    });

    let mut final_key: Option<StateKey> = None;
    while let Some(Entry { dist, node, stage }) = heap.pop() {
        let key: StateKey = (node, stage);
        if best.get(&key).copied().unwrap_or(f64::INFINITY) < dist {
            continue;
        }
        if dest_set.contains(&node) && stage >= target_stage.saturating_sub(1) {
            final_key = Some(key);
            break;
        }
        let Some(adj) = graph.adj.get(&node) else {
            continue;
        };
        for &(neighbor, edge_idx) in adj {
            if forbidden_edges.contains(&edge_idx) {
                continue;
            }
            if forbidden_directional.contains(&(node, neighbor)) {
                continue;
            }
            let edge = &graph.edges[edge_idx];
            let Some(next_stage) = transition(stage, edge, via_taxiways) else {
                continue;
            };
            let seg_m = haversine_m(
                graph.nodes[&node].latitude_deg,
                graph.nodes[&node].longitude_deg,
                graph.nodes[&neighbor].latitude_deg,
                graph.nodes[&neighbor].longitude_deg,
            );
            let nd = dist + seg_m;
            let nk = (neighbor, next_stage);
            if nd < *best.get(&nk).unwrap_or(&f64::INFINITY) {
                best.insert(nk, nd);
                came_from.insert(nk, (key, edge_idx));
                heap.push(Entry {
                    dist: nd,
                    node: neighbor,
                    stage: next_stage,
                });
            }
        }
    }

    let Some(end) = final_key else {
        bail!(
            "no taxi route from node {} to any of {:?} via {:?}",
            start_node,
            destination_nodes,
            via_taxiways
        );
    };
    let destination_node = end.0;

    // Reconstruct the leg list by walking came_from back to the start.
    let mut legs_rev: Vec<TaxiLeg> = Vec::new();
    let mut total_m = 0.0;
    let mut cursor = end;
    let mut crossings: Vec<String> = Vec::new();
    while let Some(&(prev, edge_idx)) = came_from.get(&cursor) {
        let edge = &graph.edges[edge_idx];
        let from = &graph.nodes[&prev.0];
        let to = &graph.nodes[&cursor.0];
        let seg_m = haversine_m(
            from.latitude_deg,
            from.longitude_deg,
            to.latitude_deg,
            to.longitude_deg,
        );
        total_m += seg_m;
        if !edge.active_zones.is_empty() {
            crossings.push(edge.active_zones.clone());
        }
        legs_rev.push(TaxiLeg {
            from_node: prev.0,
            to_node: cursor.0,
            from_lat: from.latitude_deg,
            from_lon: from.longitude_deg,
            to_lat: to.latitude_deg,
            to_lon: to.longitude_deg,
            taxiway_name: edge.name.clone(),
            distance_m: seg_m,
            active_zones: edge.active_zones.clone(),
        });
        cursor = prev;
    }
    legs_rev.reverse();
    crossings.reverse();

    let mut taxiway_sequence: Vec<String> = Vec::new();
    for leg in &legs_rev {
        if leg.taxiway_name.is_empty() {
            continue;
        }
        if taxiway_sequence.last() != Some(&leg.taxiway_name) {
            taxiway_sequence.push(leg.taxiway_name.clone());
        }
    }
    Ok(TaxiPlan {
        airport_ident: graph.airport_ident.clone(),
        start_node,
        destination_node,
        legs: legs_rev,
        total_distance_m: total_m,
        taxiway_sequence,
        runway_crossings: crossings,
    })
}

/// Two candidate forms for a runway ident, with and without a leading
/// zero on the numeric prefix. "01L" → ["01L", "1L"]; "1L" → ["1L", "01L"];
/// "31" → ["31", "31"] (no zero-pad change possible).
pub fn runway_query_forms(ident: &str) -> [String; 2] {
    let digits: String = ident.chars().take_while(|c| c.is_ascii_digit()).collect();
    let suffix: &str = &ident[digits.len()..];
    let alt = if let Some(stripped) = digits.strip_prefix('0') {
        if stripped.is_empty() {
            ident.to_string()
        } else {
            format!("{stripped}{suffix}")
        }
    } else if digits.len() == 1 {
        format!("0{digits}{suffix}")
    } else {
        ident.to_string()
    };
    [ident.to_string(), alt]
}

fn transition(stage: u8, edge: &TaxiEdge, via: &[String]) -> Option<u8> {
    if via.is_empty() {
        return Some(stage);
    }
    let last_stage = (via.len()) as u8; // stage = len = "past last taxiway"
    match stage {
        0 => {
            // Before the sequence starts: any edge stays at 0; edges named
            // via[0] advance to stage 1.
            if edge.name == via[0] {
                Some(1)
            } else {
                Some(0)
            }
        }
        s if s <= last_stage => {
            // Currently on via[s-1]. Edges named via[s-1] stay; edges
            // named via[s] (if any) advance; other named edges invalid.
            let cur = &via[(s as usize) - 1];
            let next = if (s as usize) < via.len() {
                Some(&via[s as usize])
            } else {
                None
            };
            if &edge.name == cur {
                Some(s)
            } else if next.is_some() && Some(&edge.name) == next {
                Some(s + 1)
            } else if s == last_stage {
                // Past the sequence — any edge is legal (lead-out).
                Some(s)
            } else if edge.name.is_empty() {
                // Unnamed connector mid-sequence — allow but don't advance.
                Some(s)
            } else {
                None
            }
        }
        _ => None,
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

#[cfg(test)]
mod tests {
    use super::*;

    fn mk_taxiway_edge(
        from_node: i64,
        to_node: i64,
        name: &str,
        active_zones: &str,
    ) -> TaxiEdge {
        TaxiEdge {
            from_node,
            to_node,
            direction: "twoway".into(),
            category: "taxiway_E".into(),
            name: name.into(),
            active_zones: active_zones.into(),
        }
    }

    fn mk_runway_edge(
        from_node: i64,
        to_node: i64,
        name: &str,
        active_zones: &str,
    ) -> TaxiEdge {
        TaxiEdge {
            from_node,
            to_node,
            direction: "twoway".into(),
            category: "runway".into(),
            name: name.into(),
            active_zones: active_zones.into(),
        }
    }

    fn tiny_graph() -> TaxiGraph {
        // Layout (lat, lon in deg, small spacing):
        //   N1 --A-- N2 --D-- N3
        //   N1 -------------- N3  (via connector edge, no name)
        //   N2 --C-- N4 --D-- N3  (alternative D route)
        let mk = |id, lat, lon| TaxiNode {
            node_id: id,
            latitude_deg: lat,
            longitude_deg: lon,
            usage: "both".to_string(),
        };
        let mut nodes = HashMap::new();
        nodes.insert(1, mk(1, 37.0000, -122.0000));
        nodes.insert(2, mk(2, 37.0010, -122.0000));
        nodes.insert(3, mk(3, 37.0020, -122.0000));
        nodes.insert(4, mk(4, 37.0015, -122.0005));
        let edges = vec![
            mk_taxiway_edge(1, 2, "A", ""),
            mk_taxiway_edge(2, 3, "D", ""),
            mk_taxiway_edge(1, 3, "", ""),
            mk_taxiway_edge(2, 4, "C", "ils:31"),
            mk_taxiway_edge(4, 3, "D", ""),
        ];
        let mut adj: HashMap<i64, Vec<(i64, usize)>> = HashMap::new();
        for (i, e) in edges.iter().enumerate() {
            adj.entry(e.from_node).or_default().push((e.to_node, i));
            adj.entry(e.to_node).or_default().push((e.from_node, i));
        }
        TaxiGraph {
            airport_ident: "KTEST".into(),
            nodes,
            edges,
            adj,
        }
    }

    #[test]
    fn unconstrained_picks_shortest_path() {
        let g = tiny_graph();
        let plan = plan(&g, 1, &[3], &[], &HashSet::new(), &HashSet::new()).unwrap();
        // N1→N3 directly via the connector is shortest.
        assert_eq!(plan.legs.len(), 1);
        assert_eq!(plan.legs[0].from_node, 1);
        assert_eq!(plan.legs[0].to_node, 3);
    }

    #[test]
    fn constrained_forces_a_then_d() {
        let g = tiny_graph();
        let plan = plan(&g, 1, &[3], &["A".into(), "D".into()], &HashSet::new(), &HashSet::new()).unwrap();
        // Must go N1 -A- N2 -D- N3 (not the shorter connector, not via C/N4).
        assert_eq!(plan.legs.len(), 2);
        assert_eq!(plan.legs[0].taxiway_name, "A");
        assert_eq!(plan.legs[1].taxiway_name, "D");
        assert_eq!(plan.taxiway_sequence, vec!["A", "D"]);
    }

    #[test]
    fn impossible_constraint_errors() {
        let g = tiny_graph();
        let err = plan(&g, 1, &[3], &["NOPE".into()], &HashSet::new(), &HashSet::new()).unwrap_err();
        assert!(err.to_string().contains("no taxi route"));
    }

    #[test]
    fn runway_crossings_surface_in_plan() {
        let g = tiny_graph();
        // Force the long path through N4 via C — it has active_zones="ils:31".
        let plan = plan(&g, 1, &[3], &["A".into(), "C".into(), "D".into()], &HashSet::new(), &HashSet::new()).unwrap();
        assert!(plan.runway_crossings.iter().any(|z| z.contains("ils:31")));
    }

    /// A graph shaped like the KWHP failure: start at node 1; runway 31's
    /// hold-short is node 2; the edge from 2 to 3 is the runway crossing
    /// (1204 departure 31). Without forbidden edges Dijkstra picks the
    /// short path 1 → 2 → 3 (20 ft total). With node 2 as the goal and
    /// the crossing edge forbidden, the path stops at 2 (10 ft).
    fn hold_short_graph() -> TaxiGraph {
        let mk = |id, lat, lon| TaxiNode {
            node_id: id,
            latitude_deg: lat,
            longitude_deg: lon,
            usage: "both".to_string(),
        };
        let mut nodes = HashMap::new();
        nodes.insert(1, mk(1, 37.0000, -122.0000));
        nodes.insert(2, mk(2, 37.0001, -122.0000));
        nodes.insert(3, mk(3, 37.0002, -122.0000));
        let edges = vec![
            mk_taxiway_edge(1, 2, "A", ""),
            mk_runway_edge(2, 3, "31/13", "departure:31,13;arrival:31,13"),
        ];
        let mut adj: HashMap<i64, Vec<(i64, usize)>> = HashMap::new();
        for (i, e) in edges.iter().enumerate() {
            adj.entry(e.from_node).or_default().push((e.to_node, i));
            adj.entry(e.to_node).or_default().push((e.from_node, i));
        }
        TaxiGraph {
            airport_ident: "KHOLD".into(),
            nodes,
            edges,
            adj,
        }
    }

    #[test]
    fn edge_conflicts_with_runway_matches_zero_padded_ident() {
        let e = mk_taxiway_edge(0, 1, "x", "departure:01L,19R;ils:01L");
        assert!(edge_conflicts_with_runway(&e, "01L"));
        assert!(edge_conflicts_with_runway(&e, "1L"));
        assert!(edge_conflicts_with_runway(&e, "19R"));
        assert!(!edge_conflicts_with_runway(&e, "28L"));
    }

    #[test]
    fn forbidden_edges_prevent_runway_crossing() {
        let g = hold_short_graph();
        // Without forbidden edges the planner will gladly walk 1 → 2 → 3
        // (past the hold-short) if asked for node 3.
        let p_cross = plan(&g, 1, &[3], &[], &HashSet::new(), &HashSet::new()).unwrap();
        assert_eq!(p_cross.legs.len(), 2);

        // With the runway-crossing edge forbidden and node 2 as goal, the
        // plan stops at node 2.
        let forbidden: HashSet<usize> = [1].into_iter().collect();
        let p_hold = plan(&g, 1, &[2], &[], &forbidden, &HashSet::new()).unwrap();
        assert_eq!(p_hold.legs.len(), 1);
        assert_eq!(p_hold.legs[0].to_node, 2);

        // Asking for node 3 with the crossing forbidden is infeasible.
        let err = plan(&g, 1, &[3], &[], &forbidden, &HashSet::new()).unwrap_err();
        assert!(err.to_string().contains("no taxi route"));
    }

    /// Regression: at KVNY, the full-length 16L hold-short has two valid
    /// boundary nodes — one 40.4 m from the threshold (west side of the
    /// runway) and one 43.3 m from the threshold (east side). The old
    /// resolver picked the west-side node by a 3 m margin, which then
    /// required a 4 km detour around the airport (south via P, north via
    /// A) because the aircraft started on the east ramp and 16L itself
    /// couldn't be crossed. The new resolver returns both candidates and
    /// the planner picks whichever is cheapest from the start.
    ///
    /// Synthetic graph:
    ///
    ///     start (1) -- clean(short) -- east_hs (2) --+
    ///                                                 |-- [forbidden] -- rwy_mid (3)
    ///     start (1) -- clean(LONG ) -- west_hs (4) --+
    ///
    /// Both hold-short nodes are equidistant to `rwy_mid` (= what the
    /// resolver uses as the threshold proxy in the real code). The
    /// connection to `start` differs by ~100× in length.
    #[test]
    fn plan_picks_reachable_hold_short_over_nearer_but_unreachable() {
        let mk = |id, lat, lon| TaxiNode {
            node_id: id,
            latitude_deg: lat,
            longitude_deg: lon,
            usage: "both".to_string(),
        };
        let mut nodes = HashMap::new();
        nodes.insert(1, mk(1, 37.0000, -122.0000));
        nodes.insert(2, mk(2, 37.0000, -122.0001)); // east hold-short: ~9 m from start
        nodes.insert(3, mk(3, 37.0000, -122.0002)); // "threshold" proxy
        nodes.insert(4, mk(4, 37.0010, -122.0003)); // west hold-short: ~111 m from start on a
                                                     // detour
        let edges = vec![
            mk_taxiway_edge(1, 2, "A", ""),       // clean, short
            mk_taxiway_edge(2, 3, "C", "departure:16L"), // east crossing (forbidden)
            mk_taxiway_edge(3, 4, "C", "departure:16L"), // west crossing (forbidden)
            mk_taxiway_edge(1, 4, "D", ""),       // clean, long detour
        ];
        let mut adj: HashMap<i64, Vec<(i64, usize)>> = HashMap::new();
        for (i, e) in edges.iter().enumerate() {
            adj.entry(e.from_node).or_default().push((e.to_node, i));
            adj.entry(e.to_node).or_default().push((e.from_node, i));
        }
        let g = TaxiGraph {
            airport_ident: "KSYN".into(),
            nodes,
            edges,
            adj,
        };
        // Both node 2 and node 4 border forbidden edges with one clean
        // neighbour — both are valid hold-short candidates. Give both to
        // the planner and require it to pick node 2 (cheap) over node 4
        // (100× more expensive).
        let forbidden: HashSet<usize> = [1, 2].into_iter().collect();
        let plan = plan(&g, 1, &[4, 2], &[], &forbidden, &HashSet::new()).unwrap();
        assert_eq!(
            plan.destination_node, 2,
            "planner should pick the reachable-cheap hold-short, not the geometric-nearer one"
        );
        assert_eq!(plan.legs.len(), 1);
        assert_eq!(plan.legs[0].to_node, 2);
    }

    /// Regression: at KWHP the taxiway-B edge crossing runway 12/30 had
    /// 1204 departure:12,30 on it, and the node on the *runway* side also
    /// had the runway pavement edge (also 1204:12) as its only other
    /// neighbour. Picking the "closest" candidate blindly picked the
    /// runway-side node — unreachable because every adjacent edge was
    /// forbidden. The boundary filter fixes this.
    #[test]
    fn hold_short_picker_skips_nodes_with_only_forbidden_edges() {
        // Graph: 1 --[clean]-- 2 --[rwy-cross]-- 3 --[rwy-pavement]-- 4.
        // Both edges incident on node 3 are forbidden — it's *inside* the
        // runway active zone, not the hold-short.
        let mk = |id, lat, lon| TaxiNode {
            node_id: id,
            latitude_deg: lat,
            longitude_deg: lon,
            usage: "both".to_string(),
        };
        let mut nodes = HashMap::new();
        nodes.insert(1, mk(1, 37.0000, -122.0000));
        nodes.insert(2, mk(2, 37.0001, -122.0000));
        nodes.insert(3, mk(3, 37.0002, -122.0000));
        nodes.insert(4, mk(4, 37.0003, -122.0000));
        let edges = vec![
            mk_taxiway_edge(1, 2, "B", ""),
            mk_taxiway_edge(2, 3, "B", "departure:12,30"),
            mk_runway_edge(3, 4, "12/30", "departure:12,30"),
        ];
        let mut adj: HashMap<i64, Vec<(i64, usize)>> = HashMap::new();
        for (i, e) in edges.iter().enumerate() {
            adj.entry(e.from_node).or_default().push((e.to_node, i));
            adj.entry(e.to_node).or_default().push((e.from_node, i));
        }
        // Nodes incident on forbidden edges: {2, 3, 4}. Node 3 has only
        // forbidden neighbours; nodes 2 and 4 each have one clean
        // neighbour (2→1 on B, and 4 has only 4→3 — forbidden — so 4 is
        // also not a valid hold-short). Only node 2 qualifies.
        let forbidden: HashSet<usize> = [1, 2].into_iter().collect();
        let valid: HashSet<i64> = forbidden
            .iter()
            .flat_map(|&i| [edges[i].from_node, edges[i].to_node])
            .filter(|id| {
                adj.get(id)
                    .map(|a| a.iter().any(|(_, e)| !forbidden.contains(e)))
                    .unwrap_or(false)
            })
            .collect();
        assert_eq!(valid, [2].into_iter().collect::<HashSet<_>>());
    }

    /// KWHP-shaped regression: aircraft landed mid-runway; `engage_park`
    /// used to route it backward along the runway surface to reach a
    /// taxiway exit on the approach-end side. With
    /// `forbid_backward_runway_traversals` in place the planner must
    /// instead continue forward to the departure-end exit.
    #[test]
    fn backward_runway_traversal_is_forbidden_by_default() {
        // Four nodes on runway 12/30 (roughly east-west), split into
        // three runway-surface segments. Node 2 is where the aircraft
        // stopped. Taxiway A exits north at node 1 (behind); taxiway B
        // exits north at node 3 (forward, departure-end side).
        //
        //         taxi_A (N0)                         taxi_B (N5)
        //           |                                    |
        //   (west)  1 === 2 === 3 === 4  (east, rwy 30)
        //    rwy 12 thr       aircraft
        let mk = |id, lat, lon| TaxiNode {
            node_id: id,
            latitude_deg: lat,
            longitude_deg: lon,
            usage: "both".to_string(),
        };
        // Runway course 90° (east). Landing heading = 90°, so forward
        // along the runway is +longitude.
        let mut nodes = HashMap::new();
        nodes.insert(0, mk(0, 37.0010, -122.0030));  // taxi A gate end
        nodes.insert(1, mk(1, 37.0000, -122.0030));  // rwy 12 thr node
        nodes.insert(2, mk(2, 37.0000, -122.0020));  // aircraft here
        nodes.insert(3, mk(3, 37.0000, -122.0010));
        nodes.insert(4, mk(4, 37.0000, -122.0000));  // rwy 30 thr node
        nodes.insert(5, mk(5, 37.0010, -122.0010));  // taxi B gate end
        let edges = vec![
            mk_runway_edge(1, 2, "12/30", "departure:12,30"),  // 0
            mk_runway_edge(2, 3, "12/30", "departure:12,30"),  // 1
            mk_runway_edge(3, 4, "12/30", "departure:12,30"),  // 2
            mk_taxiway_edge(0, 1, "A", "departure:12,30"),     // 3
            mk_taxiway_edge(3, 5, "B", "departure:12,30"),     // 4
        ];
        let mut adj: HashMap<i64, Vec<(i64, usize)>> = HashMap::new();
        for (i, e) in edges.iter().enumerate() {
            adj.entry(e.from_node).or_default().push((e.to_node, i));
            adj.entry(e.to_node).or_default().push((e.from_node, i));
        }
        let g = TaxiGraph {
            airport_ident: "KWHP-like".into(),
            nodes,
            edges,
            adj,
        };

        // Shortest path from 2 to gate node 0: 2→1→0 via runway backward
        // (~10 m of runway) then taxi A. Without the directional forbid,
        // the planner takes it.
        let unguarded = plan(&g, 2, &[0], &[], &HashSet::new(), &HashSet::new()).unwrap();
        assert_eq!(unguarded.legs.len(), 2);
        assert_eq!(unguarded.legs[0].to_node, 1);
        assert_eq!(unguarded.legs[0].taxiway_name, "12/30");

        // Heading 90° (east, landing direction). Backward = toward node 1
        // = traversal (2→1) on edge 0, (3→2) on edge 1, (4→3) on edge 2.
        let fbd = forbid_backward_runway_traversals(&g, 90.0, &HashSet::new());
        assert!(fbd.contains(&(2, 1)));
        assert!(fbd.contains(&(3, 2)));
        assert!(fbd.contains(&(4, 3)));
        assert!(!fbd.contains(&(1, 2)));  // forward still allowed
        assert!(!fbd.contains(&(2, 3)));

        // With the directional forbid set, the planner must go forward
        // (2→3) on the runway and exit at taxiway B to reach node 5.
        let plan5 = plan(&g, 2, &[5], &[], &HashSet::new(), &fbd).unwrap();
        assert_eq!(plan5.legs.len(), 2);
        assert_eq!(plan5.legs[0].from_node, 2);
        assert_eq!(plan5.legs[0].to_node, 3);
        assert_eq!(plan5.legs[1].to_node, 5);

        // Destination node 0 is now unreachable without a backtaxi — the
        // only runway-surface traversals heading west are forbidden, and
        // there is no parallel taxiway. Planner must error out.
        let err = plan(&g, 2, &[0], &[], &HashSet::new(), &fbd).unwrap_err();
        assert!(err.to_string().contains("no taxi route"));

        // Escape hatch: naming the runway in via_taxiways lifts the
        // restriction for that runway.
        let allowed: HashSet<&str> = ["12/30"].into_iter().collect();
        let fbd2 = forbid_backward_runway_traversals(&g, 90.0, &allowed);
        assert!(fbd2.is_empty());
    }
}
