//! Taxi-route planner over an airport's apt.dat taxi network. See `plan`
//! for the Dijkstra-over-stages routing contract.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

use anyhow::{anyhow, bail, Context, Result};
use duckdb::Connection;

use crate::types::{haversine_m, initial_bearing_deg, wrap_degrees_180, M_TO_FT};

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

/// Ordered taxi-network nodes that carry an aircraft from a runway
/// centerline onto the selected exit and beyond the runway active zone.
/// The first node is on the runway; the last node is connected to a
/// non-conflicting taxi edge and is therefore the safe handoff point.
#[derive(Debug, Clone)]
pub struct RunwayExitPath {
    pub taxiway_name: String,
    pub nodes: Vec<TaxiNode>,
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

    Ok(TaxiGraph::new(airport_ident, nodes, edges))
}

impl TaxiGraph {
    /// A graph over `nodes` and `edges`, with adjacency honouring each
    /// edge's direction.
    pub fn new(
        airport_ident: &str,
        nodes: HashMap<i64, TaxiNode>,
        edges: impl Into<Vec<TaxiEdge>>,
    ) -> Self {
        let edges = edges.into();
        let mut adj: HashMap<i64, Vec<(i64, usize)>> = HashMap::new();
        for (idx, e) in edges.iter().enumerate() {
            adj.entry(e.from_node).or_default().push((e.to_node, idx));
            if e.direction == "twoway" {
                adj.entry(e.to_node).or_default().push((e.from_node, idx));
            }
        }
        Self { airport_ident: airport_ident.to_string(), nodes, edges, adj }
    }
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

/// A plan whose start node is already the destination has no legs, but
/// the aircraft may still be some way from that node (e.g. rolling down
/// the runway to the hold-short ahead). Give it a zero-length leg at the
/// start node so the caller's lead-in from the aircraft has a target.
pub fn ensure_start_leg(plan: &mut TaxiPlan, graph: &TaxiGraph) {
    if !plan.legs.is_empty() {
        return;
    }
    if let Some(n) = graph.nodes.get(&plan.start_node) {
        plan.legs.push(TaxiLeg {
            from_node: n.node_id,
            to_node: n.node_id,
            from_lat: n.latitude_deg,
            from_lon: n.longitude_deg,
            to_lat: n.latitude_deg,
            to_lon: n.longitude_deg,
            taxiway_name: String::new(),
            distance_m: 0.0,
            active_zones: String::new(),
        });
    }
}

/// Taxi start node for an aircraft at (`lat`, `lon`) heading
/// `heading_deg`. On a runway (`on_runway`) the nearest node can be
/// behind it, and reaching it would mean a backtaxi; prefer the nearest
/// node ahead along its line of travel instead (within 60° of the
/// heading and 40 m of the heading line — the runway's own nodes and
/// turnoffs, not a parallel taxiway across the grass), so the aircraft
/// rolls forward to the next turnoff. Elsewhere, the nearest.
pub fn start_node(graph: &TaxiGraph, lat: f64, lon: f64, heading_deg: f64, on_runway: bool) -> Result<i64> {
    const AHEAD_HALF_ANGLE_DEG: f64 = 60.0;
    const MAX_LATERAL_M: f64 = 40.0;
    if on_runway {
        let ahead = graph
            .adj
            .iter()
            .filter(|(_, neighbors)| !neighbors.is_empty())
            .filter_map(|(id, _)| graph.nodes.get(id))
            .filter_map(|n| {
                let bearing = initial_bearing_deg(lat, lon, n.latitude_deg, n.longitude_deg);
                let off = wrap_degrees_180(bearing - heading_deg);
                let distance_m = haversine_m(lat, lon, n.latitude_deg, n.longitude_deg);
                let lateral_m = distance_m * off.to_radians().sin().abs();
                (off.abs() <= AHEAD_HALF_ANGLE_DEG && lateral_m <= MAX_LATERAL_M)
                    .then_some((n.node_id, distance_m))
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal));
        if let Some((id, _)) = ahead {
            return Ok(id);
        }
    }
    nearest_node(graph, lat, lon)
}

/// Where a taxi starts: the aircraft's position and heading, or a
/// planned point such as the end of a runway exit.
#[derive(Debug, Clone, Copy)]
pub struct TaxiStart {
    pub lat: f64,
    pub lon: f64,
    pub heading_deg: f64,
}

/// What a taxi goes to.
pub enum TaxiGoal<'a> {
    Runway(TaxiDestination<'a>),
    Parking(&'a str),
}

/// A planned taxi and what its start and goal resolved to.
pub struct PlannedTaxi {
    pub plan: TaxiPlan,
    /// The start is on runway pavement.
    pub on_runway: bool,
    /// Runway goals: a point just inside the active zone past the chosen
    /// hold-short, for orientation notes.
    pub face_toward_latlon: Option<(f64, f64)>,
    /// Parking goals: the resolved spot.
    pub parking: Option<ParkingResolution>,
}

/// On a runway, a lead-in to the start node needing more than this much
/// turn means the node is behind the aircraft: a backtaxi.
const RUNWAY_MAX_LEAD_IN_TURN_DEG: f64 = 135.0;
/// Lead-ins shorter than this are not checked (the aircraft is at the node).
const LEAD_IN_MIN_FT: f64 = 20.0;

/// The one taxi-planning pipeline behind every taxi tool, so previews
/// and engages plan the same route. Picks the start node (on a runway,
/// one ahead rather than a backtaxi), resolves the goal, plans with the
/// via constraints and without running backward along a runway, and
/// gives a plan that starts at its destination a start leg.
pub fn plan_from(
    conn: &Connection,
    graph: &TaxiGraph,
    start: TaxiStart,
    goal: &TaxiGoal,
    via_taxiways: &[String],
) -> Result<PlannedTaxi> {
    let airport = graph.airport_ident.as_str();
    if graph.nodes.is_empty() {
        bail!("airport {} has no taxi network in apt.dat", airport);
    }
    let on_runway = position_on_runway(conn, airport, start.lat, start.lon);
    let start_id = start_node(graph, start.lat, start.lon, start.heading_deg, on_runway)?;
    if on_runway {
        let n = &graph.nodes[&start_id];
        let lead_in_ft = haversine_m(start.lat, start.lon, n.latitude_deg, n.longitude_deg) * M_TO_FT;
        let turn_deg = wrap_degrees_180(
            initial_bearing_deg(start.lat, start.lon, n.latitude_deg, n.longitude_deg) - start.heading_deg,
        )
        .abs();
        if lead_in_ft > LEAD_IN_MIN_FT && turn_deg > RUNWAY_MAX_LEAD_IN_TURN_DEG {
            bail!(
                "lead-in to the first taxi node would need a {:.0}° turn from heading {:.0}° on runway pavement (max {:.0}°): that's a backtaxi down the runway. Reposition before engaging",
                turn_deg, start.heading_deg, RUNWAY_MAX_LEAD_IN_TURN_DEG
            );
        }
    }
    let (destination_nodes, forbidden_edges, hold_short, parking) = match goal {
        TaxiGoal::Runway(spec) => {
            let d = resolve_destination(conn, graph, airport, spec)?;
            (d.all_node_ids(), d.forbidden_edges.clone(), Some(d), None)
        }
        TaxiGoal::Parking(name) => {
            let r = resolve_parking_destination(conn, graph, airport, name)?;
            (vec![r.nearest_node], HashSet::new(), None, Some(r))
        }
    };
    let allowed: HashSet<&str> = via_taxiways.iter().map(String::as_str).collect();
    let forbidden_directional = forbid_backward_runway_traversals(graph, start.heading_deg, &allowed);
    let mut plan = plan(
        graph,
        start_id,
        &destination_nodes,
        via_taxiways,
        &forbidden_edges,
        &forbidden_directional,
    )?;
    ensure_start_leg(&mut plan, graph);
    let face_toward_latlon = hold_short.and_then(|d| d.face_toward_for(plan.destination_node));
    Ok(PlannedTaxi { plan, on_runway, face_toward_latlon, parking })
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
/// `alt_nodes` carries the remaining valid hold-short candidates in the
/// same threshold cluster as `node` (e.g. the pavement-end hold-short on
/// the opposite side of the runway). Pass primary + alts into `plan()`
/// so it picks whichever is cheapest from the aircraft's start — without
/// this, the planner may detour around the whole airport just to reach a
/// geometrically-closer-to-threshold node on the far side of the runway.
/// Candidates outside the threshold cluster (mid-field crossings, the
/// opposite threshold) are excluded: "taxi to runway X" without an
/// `intersection` argument must end at X's approach end.
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

/// Taxi edges that can serve as a runway exit: named-or-not taxiway edges
/// carrying an active-zone annotation for `runway_ident`.
fn is_runway_exit_edge(edge: &TaxiEdge, runway_ident: &str) -> bool {
    edge.category != "runway" && edge_conflicts_with_runway(edge, runway_ident)
}

/// Resolve every named exit off `runway_ident`, one entry per (taxiway,
/// intersection). A taxiway that meets the runway at several places (say
/// at both ends) yields one path per intersection, each with its own
/// centerline node, so callers can choose by stationing instead of
/// getting whichever junction happens to sit closest to the centerline.
/// `runway_start`/`runway_end` are the runway's endpoint lat/lons.
pub fn runway_exit_paths(
    graph: &TaxiGraph,
    runway_ident: &str,
    runway_start: (f64, f64),
    runway_end: (f64, f64),
) -> Vec<RunwayExitPath> {
    let mut names: Vec<&str> = graph
        .edges
        .iter()
        .filter(|edge| !edge.name.is_empty() && is_runway_exit_edge(edge, runway_ident))
        .map(|edge| edge.name.as_str())
        .collect();
    names.sort_unstable();
    names.dedup();
    names
        .into_iter()
        .flat_map(|name| {
            resolve_runway_exit_paths_in_graph(graph, runway_ident, name, runway_start, runway_end)
                .unwrap_or_default()
        })
        .collect()
}

/// Resolve the actual apt.dat centerline paths for one named runway exit.
///
/// A single named exit may contain several consecutive 1202 edges inside
/// the runway active zone. In particular, angled exits often run *backward*
/// in runway stationing as they leave the centerline. Treating the smallest
/// endpoint station as the intersection invents a shortcut across unpaved
/// ground. This resolver instead splits the named, runway-conflicting
/// edges into connected pieces (one per intersection), starts each at its
/// node closest to the runway centerline, and follows it to the first node
/// that connects to the ordinary (non-conflicting) taxi network.
fn resolve_runway_exit_paths_in_graph(
    graph: &TaxiGraph,
    runway_ident: &str,
    taxiway: &str,
    runway_start: (f64, f64),
    runway_end: (f64, f64),
) -> Result<Vec<RunwayExitPath>> {
    let candidate_edges: HashSet<usize> = graph
        .edges
        .iter()
        .enumerate()
        .filter(|(_, edge)| {
            edge.name.eq_ignore_ascii_case(taxiway) && is_runway_exit_edge(edge, runway_ident)
        })
        .map(|(idx, _)| idx)
        .collect();
    if candidate_edges.is_empty() {
        bail!(
            "no edge named {:?} crosses runway {:?} at {}",
            taxiway,
            runway_ident,
            graph.airport_ident
        );
    }

    let centerline_distance_m = |node: &TaxiNode| {
        point_to_segment_m(
            node.latitude_deg,
            node.longitude_deg,
            runway_start.0,
            runway_start.1,
            runway_end.0,
            runway_end.1,
        )
    };

    // Split the candidate edges into connected components; each one is a
    // separate place where the taxiway meets the runway.
    let mut seeds: Vec<i64> = candidate_edges
        .iter()
        .flat_map(|idx| {
            let edge = &graph.edges[*idx];
            [edge.from_node, edge.to_node]
        })
        .collect();
    seeds.sort_unstable();
    seeds.dedup();
    let mut assigned: HashSet<i64> = HashSet::new();
    let mut components: Vec<Vec<i64>> = Vec::new();
    for seed in seeds {
        if !assigned.insert(seed) {
            continue;
        }
        let mut component = vec![seed];
        let mut queue = VecDeque::from([seed]);
        while let Some(node) = queue.pop_front() {
            for (next, edge_idx) in graph.adj.get(&node).into_iter().flatten() {
                if candidate_edges.contains(edge_idx) && assigned.insert(*next) {
                    component.push(*next);
                    queue.push_back(*next);
                }
            }
        }
        components.push(component);
    }

    let mut paths = Vec::new();
    let mut last_err = None;
    for component in components {
        let runway_node = component
            .iter()
            .filter_map(|node_id| graph.nodes.get(node_id))
            .min_by(|a, b| {
                centerline_distance_m(a)
                    .partial_cmp(&centerline_distance_m(b))
                    .unwrap_or(Ordering::Equal)
            })
            .map(|node| node.node_id);
        let Some(runway_node) = runway_node else {
            last_err = Some(anyhow!("runway exit {:?} has no graph nodes", taxiway));
            continue;
        };
        match trace_runway_exit(graph, runway_ident, taxiway, &candidate_edges, runway_node) {
            Ok(path) => paths.push(path),
            Err(e) => last_err = Some(e),
        }
    }
    if paths.is_empty() {
        return Err(last_err
            .unwrap_or_else(|| anyhow!("runway exit {:?} has no graph nodes", taxiway)));
    }
    Ok(paths)
}

/// Breadth-first walk along `candidate_edges` from `runway_node` to the
/// nearest node that also touches a non-active-zone taxi edge.
fn trace_runway_exit(
    graph: &TaxiGraph,
    runway_ident: &str,
    taxiway: &str,
    candidate_edges: &HashSet<usize>,
    runway_node: i64,
) -> Result<RunwayExitPath> {
    let is_clear_node = |node_id: i64| {
        node_id != runway_node
            && graph.edges.iter().any(|edge| {
                (edge.from_node == node_id || edge.to_node == node_id)
                    && !edge_conflicts_with_runway(edge, runway_ident)
            })
    };

    let mut queue = VecDeque::from([runway_node]);
    let mut visited = HashSet::from([runway_node]);
    let mut parent: HashMap<i64, i64> = HashMap::new();
    let mut clear_node = None;
    while let Some(node) = queue.pop_front() {
        if is_clear_node(node) {
            clear_node = Some(node);
            break;
        }
        for (next, edge_idx) in graph.adj.get(&node).into_iter().flatten() {
            if !candidate_edges.contains(edge_idx) || !visited.insert(*next) {
                continue;
            }
            parent.insert(*next, node);
            queue.push_back(*next);
        }
    }
    let clear_node = clear_node.ok_or_else(|| {
        anyhow!(
            "runway exit {:?} at {} does not reach a non-active taxi edge",
            taxiway,
            graph.airport_ident
        )
    })?;

    let mut node_ids = vec![clear_node];
    let mut cursor = clear_node;
    while cursor != runway_node {
        cursor = *parent.get(&cursor).ok_or_else(|| {
            anyhow!("runway exit {:?} path reconstruction failed", taxiway)
        })?;
        node_ids.push(cursor);
    }
    node_ids.reverse();
    let nodes = node_ids
        .into_iter()
        .map(|id| {
            graph
                .nodes
                .get(&id)
                .cloned()
                .ok_or_else(|| anyhow!("runway exit {:?} node {} is missing", taxiway, id))
        })
        .collect::<Result<Vec<_>>>()?;

    Ok(RunwayExitPath {
        taxiway_name: taxiway.to_string(),
        nodes,
    })
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

/// True when `(lat, lon)` lies on the paved footprint of any runway at
/// the given airport — perpendicular distance to the centerline segment
/// no greater than half the runway width. `plan_from` uses it to decide
/// whether a big-turn lead-in risks a runway backtaxi (only possible when
/// the aircraft is actually on a runway) versus an innocent ramp
/// pirouette.
///
/// Returns `false` if the airport has no runways in the parquet view,
/// or if the query errors. The caller treats that as "not on a runway"
/// and skips the backtaxi guard — the alternative (refusing every taxi
/// at an airport with no runway data) is worse than the risk of missing
/// the guard in an edge case.
pub fn position_on_runway(
    conn: &Connection,
    airport_ident: &str,
    lat: f64,
    lon: f64,
) -> bool {
    let mut stmt = match conn.prepare(
        "SELECT le_latitude_deg, le_longitude_deg, \
                he_latitude_deg, he_longitude_deg, width_ft \
         FROM runways \
         WHERE airport_ident = ? AND closed = 0",
    ) {
        Ok(s) => s,
        Err(_) => return false,
    };
    let rows = stmt.query_map([airport_ident], |r| {
        Ok((
            r.get::<_, f64>(0)?,
            r.get::<_, f64>(1)?,
            r.get::<_, f64>(2)?,
            r.get::<_, f64>(3)?,
            r.get::<_, f64>(4)?,
        ))
    });
    let rows = match rows {
        Ok(r) => r,
        Err(_) => return false,
    };
    for row in rows.flatten() {
        let (le_lat, le_lon, he_lat, he_lon, width_ft) = row;
        let d_m = point_to_segment_m(lat, lon, le_lat, le_lon, he_lat, he_lon);
        let half_width_m = width_ft * 0.3048 * 0.5;
        if d_m <= half_width_m {
            return true;
        }
    }
    false
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
    // along as `alt_nodes` so the planner can pick whichever candidate
    // on the threshold cluster has the cheapest path cost (e.g. the
    // hold-short on the *aircraft's* side of the pavement end when the
    // geometrically-closest one would require looping around the runway).
    //
    // Hard-clamp alt_nodes to a tight window around the primary: without
    // an `intersection` argument, "taxi to runway X" means the approach
    // end, and every boundary node of X's active zone is a candidate —
    // including mid-runway taxiway crossings and the opposite threshold's
    // hold-shorts. Those are hundreds-to-thousands of ft from the
    // requested threshold, so they win on path cost and silently convert
    // the clearance into an intersection departure. The cluster of
    // legitimate threshold-end hold-shorts (both sides of pavement plus
    // any approach hold-short) sits within ~150 ft of the threshold
    // reference point, so a 500 ft cushion covers them while excluding
    // mid-field.
    const THRESHOLD_CLUSTER_CUSHION_M: f64 = 152.4; // 500 ft
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
    let primary_dist_m = ranked[0].1;
    let alt_cutoff_m = primary_dist_m + THRESHOLD_CLUSTER_CUSHION_M;
    let alt_nodes: Vec<HoldShortCandidate> = ranked[1..]
        .iter()
        .take_while(|(_, d)| *d <= alt_cutoff_m)
        .map(|(id, _)| HoldShortCandidate {
            node: *id,
            face_toward_latlon: face_toward(*id),
        })
        .collect();

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
            Some(self.cmp(o))
        }
    }
    impl Ord for Entry {
        fn cmp(&self, o: &Self) -> Ordering {
            // Min-heap via reversed comparison.
            o.dist.partial_cmp(&self.dist).unwrap_or(Ordering::Equal)
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

#[cfg(test)]
mod tests {
    use super::*;

    fn mk_edge(
        from_node: i64,
        to_node: i64,
        name: &str,
        category: &str,
        active_zones: &str,
    ) -> TaxiEdge {
        TaxiEdge {
            from_node,
            to_node,
            direction: "twoway".into(),
            category: category.into(),
            name: name.into(),
            active_zones: active_zones.into(),
        }
    }

    fn mk_taxiway_edge(from: i64, to: i64, name: &str, active_zones: &str) -> TaxiEdge {
        mk_edge(from, to, name, "taxiway_E", active_zones)
    }

    fn mk_runway_edge(from: i64, to: i64, name: &str, active_zones: &str) -> TaxiEdge {
        mk_edge(from, to, name, "runway", active_zones)
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
        TaxiGraph::new("KTEST", nodes, edges)
    }

    #[test]
    fn runway_exit_path_starts_on_centerline_and_preserves_reversed_geometry() {
        // Eastbound runway. Exit E reaches the centerline at N66, then
        // angles backward through N31 and N32 before joining taxiway A.
        // Choosing the minimum station would incorrectly call N32 the
        // runway intersection.
        let mk = |id, lat, lon| TaxiNode {
            node_id: id,
            latitude_deg: lat,
            longitude_deg: lon,
            usage: "both".to_string(),
        };
        let mut nodes = HashMap::new();
        nodes.insert(29, mk(29, 37.0005, -122.0003));
        nodes.insert(32, mk(32, 37.0004, -122.0002));
        nodes.insert(31, mk(31, 37.0003, -122.0001));
        nodes.insert(66, mk(66, 37.0000, -122.0000));
        nodes.insert(67, mk(67, 37.0000, -121.9990));
        let active = "departure:09,27;arrival:09,27";
        let edges = vec![
            mk_taxiway_edge(31, 66, "E", active),
            mk_taxiway_edge(32, 31, "E", active),
            mk_taxiway_edge(29, 32, "A", ""),
            mk_runway_edge(66, 67, "09/27", active),
        ];
        let graph = TaxiGraph::new("KTEST", nodes, edges);

        let exits = resolve_runway_exit_paths_in_graph(
            &graph,
            "09",
            "E",
            (37.0, -122.01),
            (37.0, -121.99),
        )
        .unwrap();
        assert_eq!(exits.len(), 1);
        let exit = &exits[0];

        assert_eq!(exit.nodes.iter().map(|n| n.node_id).collect::<Vec<_>>(), [66, 31, 32]);
        assert!(exit.nodes[1].longitude_deg < exit.nodes[0].longitude_deg);
        assert!(exit.nodes[2].longitude_deg < exit.nodes[1].longitude_deg);
    }

    #[test]
    fn taxiway_meeting_runway_twice_yields_one_exit_per_intersection() {
        // Parallel taxiway A joins runway 09 at both ends. Each junction
        // is its own exit; neither may be dropped in favour of the other.
        let mk = |id, lat, lon| TaxiNode {
            node_id: id,
            latitude_deg: lat,
            longitude_deg: lon,
            usage: "both".to_string(),
        };
        let mut nodes = HashMap::new();
        nodes.insert(10, mk(10, 37.0000, -122.0095));
        nodes.insert(11, mk(11, 37.0003, -122.0095));
        nodes.insert(20, mk(20, 37.0000, -121.9905));
        nodes.insert(21, mk(21, 37.0003, -121.9905));
        nodes.insert(30, mk(30, 37.0003, -122.0000));
        let active = "departure:09,27;arrival:09,27";
        let edges = vec![
            mk_taxiway_edge(10, 11, "A", active),
            mk_taxiway_edge(11, 30, "A", ""),
            mk_taxiway_edge(30, 21, "A", ""),
            mk_taxiway_edge(21, 20, "A", active),
            mk_runway_edge(10, 20, "09/27", active),
        ];
        let graph = TaxiGraph::new("KTEST", nodes, edges);

        let exits = runway_exit_paths(&graph, "09", (37.0, -122.01), (37.0, -121.99));
        let mut starts: Vec<(i64, i64)> = exits
            .iter()
            .map(|e| (e.nodes[0].node_id, e.nodes.last().unwrap().node_id))
            .collect();
        starts.sort();
        assert_eq!(starts, [(10, 11), (20, 21)]);
        assert!(exits.iter().all(|e| e.taxiway_name == "A"));
    }

    #[test]
    fn start_node_on_runway_prefers_the_next_node_ahead() {
        // Eastbound on the runway centerline, just past node 1 (30 m
        // behind); node 2 is 150 m ahead.
        let mk = |id, lat, lon| TaxiNode {
            node_id: id,
            latitude_deg: lat,
            longitude_deg: lon,
            usage: "both".to_string(),
        };
        let mut nodes = HashMap::new();
        nodes.insert(1, mk(1, 37.0, -122.000338));
        nodes.insert(2, mk(2, 37.0, -121.998313));
        nodes.insert(3, mk(3, 37.003, -121.999));
        let edges = vec![
            mk_taxiway_edge(1, 3, "B", ""),
            mk_taxiway_edge(2, 3, "C", ""),
        ];
        let graph = TaxiGraph::new("KTEST", nodes, edges);
        let (lat, lon) = (37.0, -122.0);
        assert_eq!(nearest_node(&graph, lat, lon).unwrap(), 1, "nearest is behind");
        assert_eq!(start_node(&graph, lat, lon, 90.0, true).unwrap(), 2, "on runway: roll ahead");
        assert_eq!(start_node(&graph, lat, lon, 90.0, false).unwrap(), 1, "off runway: nearest");

        // A parallel-taxiway node off to the side, closer than node 2 and
        // inside the 60° cone, must not win: that lead-in crosses grass.
        let mut graph = graph;
        graph.nodes.insert(4, mk(4, 37.0008, -121.9991));
        graph.edges.push(mk_taxiway_edge(4, 3, "A", ""));
        let idx = graph.edges.len() - 1;
        graph.adj.entry(4).or_default().push((3, idx));
        graph.adj.entry(3).or_default().push((4, idx));
        assert_eq!(start_node(&graph, lat, lon, 90.0, true).unwrap(), 2, "parallel taxiway skipped");
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

    /// 1 --A-- 2 --[runway 31/13 crossing, 1204-active]-- 3. With the
    /// crossing edge forbidden, a plan for node 2 must stop there; an
    /// unconstrained plan for node 3 cuts straight across.
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
        TaxiGraph::new("KHOLD", nodes, edges)
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

    /// Two hold-short candidates equidistant from the "threshold" proxy
    /// but with ~100× different path costs from the start. The planner
    /// must pick the cheap one.
    ///
    ///     start (1) -- clean(short) -- east_hs (2) --+
    ///                                                 |-- [forbidden] -- rwy_mid (3)
    ///     start (1) -- clean(LONG ) -- west_hs (4) --+
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
        let g = TaxiGraph::new("KSYN", nodes, edges);
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

    /// A node whose every adjacent edge is forbidden is unreachable —
    /// it sits *inside* the runway active zone, not on its boundary.
    /// The picker must skip such nodes even if they're geometrically
    /// closest.
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
        let edges = [
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

    /// An aircraft stopped mid-runway must not be routed backward along
    /// the runway surface to reach the nearest taxiway exit — the
    /// planner has to continue forward to the departure-end exit even
    /// when that's the longer graph path.
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
        let g = TaxiGraph::new("KTEST-rwy", nodes, edges);

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
