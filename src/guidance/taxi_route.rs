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
use std::collections::{BinaryHeap, HashMap};

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
        "SELECT from_node, to_node, direction, name, active_zones \
         FROM taxi_edges WHERE airport_ident = ?",
    )?;
    let mut rows = stmt.query([airport_ident])?;
    while let Some(row) = rows.next()? {
        edges.push(TaxiEdge {
            from_node: row.get::<_, i32>(0)? as i64,
            to_node: row.get::<_, i32>(1)? as i64,
            direction: row.get(2)?,
            name: row.get(3)?,
            active_zones: row.get(4)?,
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

pub fn resolve_destination(
    conn: &Connection,
    graph: &TaxiGraph,
    airport_ident: &str,
    dest: &TaxiDestination,
) -> Result<i64> {
    match *dest {
        TaxiDestination::NodeId(id) => {
            if graph.nodes.contains_key(&id) {
                Ok(id)
            } else {
                Err(anyhow!("node {} not in taxi graph for {}", id, airport_ident))
            }
        }
        TaxiDestination::Coord(lat, lon) => nearest_node(graph, lat, lon),
        TaxiDestination::Runway(runway_ident) => {
            // apt.dat row 100 sometimes drops the leading zero on single-
            // digit runways ("1L" instead of "01L"), while ATC and the taxi
            // network often retain it. Try both forms.
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
            nearest_node(graph, lat, lon)
        }
    }
}

/// Constrained shortest-path planner. `via_taxiways` empty → plain Dijkstra.
pub fn plan(
    graph: &TaxiGraph,
    start_node: i64,
    destination_node: i64,
    via_taxiways: &[String],
) -> Result<TaxiPlan> {
    if !graph.nodes.contains_key(&start_node) {
        bail!("start node {} not in taxi graph for {}", start_node, graph.airport_ident);
    }
    if !graph.nodes.contains_key(&destination_node) {
        bail!(
            "destination node {} not in taxi graph for {}",
            destination_node,
            graph.airport_ident
        );
    }

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
        if node == destination_node && stage >= target_stage.saturating_sub(1) {
            final_key = Some(key);
            break;
        }
        let Some(adj) = graph.adj.get(&node) else {
            continue;
        };
        for &(neighbor, edge_idx) in adj {
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
            "no taxi route from node {} to node {} via {:?}",
            start_node,
            destination_node,
            via_taxiways
        );
    };

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
fn runway_query_forms(ident: &str) -> [String; 2] {
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
            TaxiEdge {
                from_node: 1,
                to_node: 2,
                direction: "twoway".into(),
                name: "A".into(),
                active_zones: String::new(),
            },
            TaxiEdge {
                from_node: 2,
                to_node: 3,
                direction: "twoway".into(),
                name: "D".into(),
                active_zones: String::new(),
            },
            TaxiEdge {
                from_node: 1,
                to_node: 3,
                direction: "twoway".into(),
                name: String::new(),
                active_zones: String::new(),
            },
            TaxiEdge {
                from_node: 2,
                to_node: 4,
                direction: "twoway".into(),
                name: "C".into(),
                active_zones: "ils:31".into(),
            },
            TaxiEdge {
                from_node: 4,
                to_node: 3,
                direction: "twoway".into(),
                name: "D".into(),
                active_zones: String::new(),
            },
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
        let plan = plan(&g, 1, 3, &[]).unwrap();
        // N1→N3 directly via the connector is shortest.
        assert_eq!(plan.legs.len(), 1);
        assert_eq!(plan.legs[0].from_node, 1);
        assert_eq!(plan.legs[0].to_node, 3);
    }

    #[test]
    fn constrained_forces_a_then_d() {
        let g = tiny_graph();
        let plan = plan(&g, 1, 3, &["A".into(), "D".into()]).unwrap();
        // Must go N1 -A- N2 -D- N3 (not the shorter connector, not via C/N4).
        assert_eq!(plan.legs.len(), 2);
        assert_eq!(plan.legs[0].taxiway_name, "A");
        assert_eq!(plan.legs[1].taxiway_name, "D");
        assert_eq!(plan.taxiway_sequence, vec!["A", "D"]);
    }

    #[test]
    fn impossible_constraint_errors() {
        let g = tiny_graph();
        let err = plan(&g, 1, 3, &["NOPE".into()]).unwrap_err();
        assert!(err.to_string().contains("no taxi route"));
    }

    #[test]
    fn runway_crossings_surface_in_plan() {
        let g = tiny_graph();
        // Force the long path through N4 via C — it has active_zones="ils:31".
        let plan = plan(&g, 1, 3, &["A".into(), "C".into(), "D".into()]).unwrap();
        assert!(plan.runway_crossings.iter().any(|z| z.contains("ils:31")));
    }
}
