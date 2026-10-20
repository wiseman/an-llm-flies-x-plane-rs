//! Plane geometry for plan conditions: polylines (roads, shorelines),
//! crossings between ticks, side-of-line on curving features, abeam and
//! angle-behind-the-wing tests. All coordinates are world-frame feet
//! (x east, y north).

use crate::types::{heading_to_vector, vector_to_heading, wrap_degrees_180, Vec2, G_FT_S2, KT_TO_FPS, NM_TO_FT};

/// How far a side-of-line ray is cast before giving up and falling back
/// to the nearest point. 30 nm covers anything near an airport.
const SIDE_RAY_FT: f64 = 30.0 * NM_TO_FT;

/// The closest point to `p` on segment `a`–`b`, and how far along it
/// (0 at `a`, 1 at `b`).
pub fn closest_on_segment(p: Vec2, a: Vec2, b: Vec2) -> (f64, Vec2) {
    let ab = b - a;
    let len2 = ab.dot(ab);
    let t = if len2 < 1e-9 { 0.0 } else { ((p - a).dot(ab) / len2).clamp(0.0, 1.0) };
    (t, a + ab * t)
}

/// Radius of a coordinated turn at `gs_kt` and `bank_deg`.
pub fn turn_radius_ft(gs_kt: f64, bank_deg: f64) -> f64 {
    let v = gs_kt * KT_TO_FPS;
    v * v / (G_FT_S2 * bank_deg.to_radians().tan())
}

#[derive(Debug, Clone, PartialEq)]
pub struct Polyline {
    pub points: Vec<Vec2>,
}

/// Closest point on a polyline to a query point.
#[derive(Debug, Clone, Copy)]
pub struct Nearest {
    pub distance_ft: f64,
    pub point: Vec2,
    pub segment: usize,
    /// Positive when the query point is right of the polyline's direction
    /// of travel at the closest segment, negative when left.
    pub signed_offset_ft: f64,
}

impl Polyline {
    pub fn new(points: Vec<Vec2>) -> Self {
        Self { points }
    }

    pub fn segments(&self) -> impl Iterator<Item = (Vec2, Vec2)> + '_ {
        self.points.windows(2).map(|w| (w[0], w[1]))
    }

    pub fn nearest(&self, p: Vec2) -> Option<Nearest> {
        let mut best: Option<Nearest> = None;
        for (i, (a, b)) in self.segments().enumerate() {
            let (_, q) = closest_on_segment(p, a, b);
            let d = p.distance_to(q);
            if best.is_none_or(|n| d < n.distance_ft) {
                // ab × ap > 0 means p is left of a→b (x east, y north).
                let side = if (b - a).cross(p - a) > 0.0 { -1.0 } else { 1.0 };
                best = Some(Nearest { distance_ft: d, point: q, segment: i, signed_offset_ft: side * d });
            }
        }
        if best.is_none() && self.points.len() == 1 {
            let q = self.points[0];
            best = Some(Nearest { distance_ft: p.distance_to(q), point: q, segment: 0, signed_offset_ft: 0.0 });
        }
        best
    }

    /// Whether the move `from → to` crosses the polyline.
    pub fn crossed_by(&self, from: Vec2, to: Vec2) -> bool {
        self.segments().any(|(a, b)| segments_intersect(from, to, a, b))
    }

    /// Whether `p` is on the `compass_deg` side of the line ("north of the
    /// 101" is `compass_deg = 0`). Casts a ray from `p` the opposite way:
    /// an odd number of hits means the line lies between `p` and that
    /// direction. A ray that misses (the line doesn't extend that far)
    /// falls back to comparing `p` with the line's nearest point.
    pub fn is_on_side(&self, p: Vec2, compass_deg: f64) -> bool {
        let back = p + heading_to_vector(compass_deg + 180.0, SIDE_RAY_FT);
        let hits = self.segments().filter(|(a, b)| segments_intersect(p, back, *a, *b)).count();
        if hits > 0 {
            return hits % 2 == 1;
        }
        match self.nearest(p) {
            Some(n) => (p - n.point).dot(heading_to_vector(compass_deg, 1.0)) > 0.0,
            None => false,
        }
    }

    /// Mean course of the polyline (first to last point).
    pub fn overall_course_deg(&self) -> f64 {
        match (self.points.first(), self.points.last()) {
            (Some(a), Some(b)) => vector_to_heading(*b - *a),
            _ => 0.0,
        }
    }

    /// Course of the segment nearest `p`, oriented so it's within 90° of
    /// `prefer_course_deg` (polyline direction is arbitrary).
    pub fn local_course_deg(&self, p: Vec2, prefer_course_deg: f64) -> Option<f64> {
        let n = self.nearest(p)?;
        let (a, b) = self.segments().nth(n.segment)?;
        let c = vector_to_heading(b - a);
        Some(if wrap_degrees_180(c - prefer_course_deg).abs() > 90.0 { (c + 180.0) % 360.0 } else { c })
    }

    /// Centroid of the vertices; good enough for "abeam the reservoir".
    pub fn centroid(&self) -> Vec2 {
        if self.points.is_empty() {
            return Vec2::ZERO;
        }
        let sum = self.points.iter().fold(Vec2::ZERO, |acc, p| acc + *p);
        sum * (1.0 / self.points.len() as f64)
    }
}

/// Carrot point for flying a circle of `radius_ft` about `center_ft`:
/// tangent on the circle, blending inward or outward when off it. Fly it
/// with `L1PathFollower::direct_to`.
pub fn orbit_target_world_ft(
    aircraft_pos_ft: Vec2,
    center_ft: Vec2,
    radius_ft: f64,
    lookahead_ft: f64,
    ccw: bool,
) -> Vec2 {
    let radial = aircraft_pos_ft - center_ft;
    let radial_len = radial.length().max(1.0);
    let radial_unit = radial * (1.0 / radial_len);
    // Rotate radial 90° (CCW for ccw orbit, CW otherwise) to get the
    // tangent direction at the aircraft's projection on the circle.
    let tangent = if ccw {
        Vec2::new(-radial_unit.y, radial_unit.x)
    } else {
        Vec2::new(radial_unit.y, -radial_unit.x)
    };
    let inward = Vec2::new(-radial_unit.x, -radial_unit.y);
    let inward_weight = ((radial_len - radius_ft) / radius_ft.max(1.0)).tanh();
    let tangent_weight = (1.0 - inward_weight.abs()).max(0.0);
    let desired_dir = (tangent * tangent_weight + inward * inward_weight).normalized();
    aircraft_pos_ft + desired_dir * lookahead_ft.max(1.0)
}

/// Polygon membership (ray casting). `ring` need not be closed.
pub fn point_in_polygon(p: Vec2, ring: &[Vec2]) -> bool {
    let mut inside = false;
    let n = ring.len();
    if n < 3 {
        return false;
    }
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (ring[i], ring[j]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Where segment `p0→p1` crosses segment `q0→q1`, as a fraction along
/// `p0→p1`.
pub fn segment_intersection_t(p0: Vec2, p1: Vec2, q0: Vec2, q1: Vec2) -> Option<f64> {
    let r = p1 - p0;
    let s = q1 - q0;
    let denom = r.cross(s);
    if denom.abs() < 1e-9 {
        return None;
    }
    let t = (q0 - p0).cross(s) / denom;
    let u = (q0 - p0).cross(r) / denom;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some(t)
}

pub fn segments_intersect(p1: Vec2, p2: Vec2, q1: Vec2, q2: Vec2) -> bool {
    let d1 = (p2 - p1).cross(q1 - p1);
    let d2 = (p2 - p1).cross(q2 - p1);
    let d3 = (q2 - q1).cross(p1 - q1);
    let d4 = (q2 - q1).cross(p2 - q1);
    ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0)) && d1 != 0.0 && d3 != 0.0
}

/// Distance of `point` ahead of the aircraft along its track (negative
/// once it's behind). Abeam is the sign change.
pub fn along_track_ft(position: Vec2, track_deg: f64, point: Vec2) -> f64 {
    (point - position).dot(heading_to_vector(track_deg, 1.0))
}

/// How far behind the wing `point` is: 0° when it's straight off the
/// wingtip, 45° when it's 45° behind (the AFH base-turn cue), negative
/// while it's still ahead of the wing.
pub fn angle_behind_wing_deg(position: Vec2, track_deg: f64, point: Vec2) -> f64 {
    let bearing = vector_to_heading(point - position);
    wrap_degrees_180(bearing - track_deg).abs() - 90.0
}

/// A closed path (a rectangular course), as a ring of points in the
/// order flown, with the distance along it at each point.
#[derive(Debug, Clone, PartialEq)]
pub struct ClosedPath {
    pub points: Vec<Vec2>,
    /// Distance from `points[0]` to `points[i]` along the path.
    pub cum_ft: Vec<f64>,
    pub length_ft: f64,
}

/// Closest point on a `ClosedPath`.
#[derive(Debug, Clone, Copy)]
pub struct PathNearest {
    pub segment: usize,
    pub along_ft: f64,
    pub distance_ft: f64,
}

impl ClosedPath {
    pub fn new(points: Vec<Vec2>) -> Self {
        let mut cum_ft = Vec::with_capacity(points.len());
        let mut s = 0.0;
        for i in 0..points.len() {
            cum_ft.push(s);
            s += points[i].distance_to(points[(i + 1) % points.len()]);
        }
        Self { points, cum_ft, length_ft: s }
    }

    fn segment(&self, i: usize) -> (Vec2, Vec2) {
        (self.points[i], self.points[(i + 1) % self.points.len()])
    }

    /// Closest point to `p`, searching the segments from `hint - 2` to
    /// `hint + 40` (the way flown), or all of them without a hint.
    pub fn nearest(&self, p: Vec2, hint: Option<usize>) -> PathNearest {
        let n = self.points.len();
        let (start, count) = match hint {
            Some(h) if n > 45 => (h + n - 2, 43),
            _ => (0, n),
        };
        let mut best = PathNearest { segment: 0, along_ft: 0.0, distance_ft: f64::INFINITY };
        for i in (0..count).map(|k| (start + k) % n) {
            let (a, b) = self.segment(i);
            let (t, q) = closest_on_segment(p, a, b);
            let d = p.distance_to(q);
            if d < best.distance_ft {
                best = PathNearest { segment: i, along_ft: self.cum_ft[i] + t * a.distance_to(b), distance_ft: d };
            }
        }
        best
    }

    /// The point `s` along the path (wrapping).
    pub fn point_at(&self, s: f64) -> Vec2 {
        let s = s.rem_euclid(self.length_ft.max(1.0));
        let i = match self.cum_ft.binary_search_by(|c| c.partial_cmp(&s).unwrap()) {
            Ok(i) => i,
            Err(i) => i - 1,
        };
        let (a, b) = self.segment(i);
        let len = a.distance_to(b).max(1e-9);
        a + (b - a) * ((s - self.cum_ft[i]) / len)
    }

    /// Signed distance from `from_ft` to `to_ft` going forward, in
    /// (-length/2, length/2].
    pub fn progress(&self, from_ft: f64, to_ft: f64) -> f64 {
        let l = self.length_ft.max(1.0);
        let d = (to_ft - from_ft).rem_euclid(l);
        if d > l / 2.0 { d - l } else { d }
    }
}

/// A rectangle: center, unit axis `u` (the long side when there is
/// one), half-lengths along `u` and its left perpendicular `v`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect {
    pub center: Vec2,
    pub u: Vec2,
    pub half_u: f64,
    pub half_v: f64,
}

impl Rect {
    pub fn v(&self) -> Vec2 {
        Vec2::new(-self.u.y, self.u.x)
    }

}

fn convex_hull(pts: &[Vec2]) -> Vec<Vec2> {
    let mut p: Vec<Vec2> = pts.to_vec();
    p.sort_by(|a, b| a.x.partial_cmp(&b.x).unwrap().then(a.y.partial_cmp(&b.y).unwrap()));
    if p.len() < 3 {
        return p;
    }
    let mut lower: Vec<Vec2> = Vec::new();
    for q in &p {
        while lower.len() >= 2 && (lower[lower.len() - 1] - lower[lower.len() - 2]).cross(*q - lower[lower.len() - 2]) <= 0.0 {
            lower.pop();
        }
        lower.push(*q);
    }
    let mut upper: Vec<Vec2> = Vec::new();
    for q in p.iter().rev() {
        while upper.len() >= 2 && (upper[upper.len() - 1] - upper[upper.len() - 2]).cross(*q - upper[upper.len() - 2]) <= 0.0 {
            upper.pop();
        }
        upper.push(*q);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

fn rect_along(pts: &[Vec2], u: Vec2) -> Rect {
    let v = Vec2::new(-u.y, u.x);
    let (mut lo_u, mut hi_u, mut lo_v, mut hi_v) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for p in pts {
        lo_u = lo_u.min(p.dot(u));
        hi_u = hi_u.max(p.dot(u));
        lo_v = lo_v.min(p.dot(v));
        hi_v = hi_v.max(p.dot(v));
    }
    let center = u * ((lo_u + hi_u) / 2.0) + v * ((lo_v + hi_v) / 2.0);
    let (hu, hv) = ((hi_u - lo_u) / 2.0, (hi_v - lo_v) / 2.0);
    if hv > hu {
        // Keep `u` on the long side.
        Rect { center, u: v, half_u: hv, half_v: hu }
    } else {
        Rect { center, u, half_u: hu, half_v: hv }
    }
}

/// The smallest rectangle around `ring` (any orientation), or the one
/// squared to north when that is nearly as small (a round area, or a
/// field on the street grid).
pub fn bounding_rect(ring: &[Vec2]) -> Rect {
    let hull = convex_hull(ring);
    let area = |r: &Rect| r.half_u * r.half_v;
    let north = rect_along(&hull, Vec2::new(0.0, 1.0));
    let mut best = north;
    for i in 0..hull.len() {
        let e = hull[(i + 1) % hull.len()] - hull[i];
        if e.length() < 1e-6 {
            continue;
        }
        let r = rect_along(&hull, e.normalized());
        if area(&r) < area(&best) {
            best = r;
        }
    }
    if area(&north) <= area(&best) * 1.08 { north } else { best }
}

/// The track of a rectangular course: `rect` grown by `offset_ft` on
/// every side, corners rounded to `radius_ft`, flown with left turns
/// (`left = true`, counterclockwise) or right turns. Starts at the
/// beginning of the first long side.
pub fn rectangular_course(rect: &Rect, offset_ft: f64, radius_ft: f64, left: bool) -> ClosedPath {
    let (u, v) = (rect.u, rect.v());
    let (a, b) = (rect.half_u + offset_ft, rect.half_v + offset_ft);
    let r = radius_ft.min(a * 0.9).min(b * 0.9).max(1.0);
    let mut pts = Vec::new();
    // Counterclockwise: along -v side going +u, then the +u side, ...
    for (k, (su, sv)) in [(1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0)].iter().enumerate() {
        let c = rect.center + u * (su * (a - r)) + v * (sv * (b - r));
        let start = -90.0 + 90.0 * k as f64;
        for j in 0..=18 {
            let th = (start + 5.0 * j as f64).to_radians();
            pts.push(c + u * (r * th.cos()) + v * (r * th.sin()));
        }
    }
    // Start at the beginning of the first side, not mid-corner.
    let n = pts.len();
    pts.rotate_left(n - 1);
    if !left {
        // Reversed, the first side starts at the second-to-last point.
        pts.reverse();
        pts.rotate_left(n - 2);
    }
    ClosedPath::new(pts)
}

/// L1 nonlinear guidance toward `carrot`: the lateral acceleration
/// 2·V²·sin(η)/L that puts the aircraft on a path through it (exact on
/// a circle), as a bank angle. η is measured from the ground track, so
/// the bank steepens with groundspeed (a tailwind) on the same circle.
/// Returns (course to the carrot, bank, + right).
pub fn l1_bank_toward(pos: Vec2, track_deg: f64, gs_kt: f64, carrot: Vec2, max_bank_deg: f64) -> (f64, f64) {
    let to = carrot - pos;
    let l = to.length().max(1.0);
    let course = vector_to_heading(to);
    let eta = wrap_degrees_180(course - track_deg).to_radians();
    let s = if eta.abs() > std::f64::consts::FRAC_PI_2 { eta.signum() } else { eta.sin() };
    let v = gs_kt.max(30.0) * KT_TO_FPS;
    let accel = 2.0 * v * v * s / l;
    let bank = (accel / G_FT_S2).atan().to_degrees();
    (course, bank.clamp(-max_bank_deg, max_bank_deg))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wavy_east_west() -> Polyline {
        // A road running east, bending north in the middle.
        Polyline::new(vec![
            Vec2::new(-10_000.0, 0.0),
            Vec2::new(0.0, 0.0),
            Vec2::new(2_000.0, 3_000.0),
            Vec2::new(10_000.0, 3_000.0),
        ])
    }

    #[test]
    fn crossing_detected_between_ticks() {
        let road = wavy_east_west();
        assert!(road.crossed_by(Vec2::new(-5_000.0, -100.0), Vec2::new(-5_000.0, 100.0)));
        assert!(!road.crossed_by(Vec2::new(-5_000.0, 100.0), Vec2::new(-5_000.0, 300.0)));
        assert!(road.crossed_by(Vec2::new(5_000.0, 3_100.0), Vec2::new(5_000.0, 2_900.0)));
    }

    #[test]
    fn side_of_curving_line_follows_the_bend() {
        let road = wavy_east_west();
        // (5000, 1500) is south of the road where it has bent north, even
        // though it's north of the road's western half.
        assert!(!road.is_on_side(Vec2::new(5_000.0, 1_500.0), 0.0));
        assert!(road.is_on_side(Vec2::new(5_000.0, 1_500.0), 180.0));
        assert!(road.is_on_side(Vec2::new(-5_000.0, 1_500.0), 0.0));
        // Beyond the road's east end the nearest point decides.
        assert!(road.is_on_side(Vec2::new(20_000.0, 4_000.0), 0.0));
    }

    #[test]
    fn signed_offset_is_right_positive() {
        let road = Polyline::new(vec![Vec2::new(0.0, 0.0), Vec2::new(0.0, 10_000.0)]); // northbound
        let n = road.nearest(Vec2::new(500.0, 5_000.0)).unwrap();
        assert!((n.signed_offset_ft - 500.0).abs() < 1e-6);
        let n = road.nearest(Vec2::new(-500.0, 5_000.0)).unwrap();
        assert!((n.signed_offset_ft + 500.0).abs() < 1e-6);
    }

    #[test]
    fn abeam_and_behind_wing() {
        let pos = Vec2::new(0.0, 0.0);
        let point = Vec2::new(-3_000.0, 1_000.0); // ahead-left, northbound flight
        assert!(along_track_ft(pos, 0.0, point) > 0.0);
        assert!(angle_behind_wing_deg(pos, 0.0, point) < 0.0);
        let behind = Vec2::new(-3_000.0, -3_000.0);
        assert!((angle_behind_wing_deg(pos, 0.0, behind) - 45.0).abs() < 1e-6);
    }

    #[test]
    fn polygon_membership() {
        let sq = [Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(10.0, 10.0), Vec2::new(0.0, 10.0)];
        assert!(point_in_polygon(Vec2::new(5.0, 5.0), &sq));
        assert!(!point_in_polygon(Vec2::new(15.0, 5.0), &sq));
    }

    #[test]
    fn bounding_rect_squares_round_areas_to_north_and_follows_tilted_ones() {
        let circle: Vec<Vec2> = (0..24)
            .map(|k| heading_to_vector(k as f64 * 15.0, 2000.0) + Vec2::new(500.0, 500.0))
            .collect();
        let r = bounding_rect(&circle);
        assert!(wrap_degrees_180(vector_to_heading(r.u)).abs() % 90.0 < 1.0, "{:?}", r.u);
        assert!((r.half_u - 2000.0).abs() < 50.0 && (r.half_v - 2000.0).abs() < 50.0);
        // A 4000 x 1000 field tilted to 030.
        let (u, v) = (heading_to_vector(30.0, 1.0), heading_to_vector(120.0, 1.0));
        let field = vec![u * -2000.0 + v * -500.0, u * 2000.0 + v * -500.0, u * 2000.0 + v * 500.0, u * -2000.0 + v * 500.0];
        let r = bounding_rect(&field);
        assert!(wrap_degrees_180(vector_to_heading(r.u) - 30.0).abs() % 180.0 < 1.0, "{}", vector_to_heading(r.u));
        assert!((r.half_u - 2000.0).abs() < 1.0 && (r.half_v - 500.0).abs() < 1.0);
    }

    #[test]
    fn rectangular_course_runs_offset_from_the_edges_with_left_or_right_turns() {
        let rect = Rect { center: Vec2::ZERO, u: Vec2::new(1.0, 0.0), half_u: 3000.0, half_v: 2000.0 };
        let left = rectangular_course(&rect, 1500.0, 1000.0, true);
        // The middle of each side is the offset out from the field edge.
        for p in [Vec2::new(0.0, -3500.0), Vec2::new(4500.0, 0.0), Vec2::new(0.0, 3500.0), Vec2::new(-4500.0, 0.0)] {
            assert!(left.nearest(p, None).distance_ft < 1.0, "{p:?}");
        }
        // Straights plus four quarter circles.
        let expect = 2.0 * (2.0 * (4500.0 - 1000.0) + 2.0 * (3500.0 - 1000.0)) + 2.0 * std::f64::consts::PI * 1000.0;
        assert!((left.length_ft - expect).abs() < 30.0, "{} vs {expect}", left.length_ft);
        // Left: the first side runs east along the south edge; right: west.
        let course = |c: &ClosedPath| vector_to_heading(c.points[1] - c.points[0]);
        assert!((course(&left) - 90.0).abs() < 1.0, "{}", course(&left));
        let right = rectangular_course(&rect, 1500.0, 1000.0, false);
        assert!(wrap_degrees_180(course(&right) - 270.0).abs() < 1.0, "{}", course(&right));
        // Progress wraps around the start.
        assert!((left.progress(left.length_ft - 100.0, 50.0) - 150.0).abs() < 1e-6);
    }

    #[test]
    fn l1_banks_toward_the_carrot_and_harder_with_more_groundspeed() {
        let (_, b) = l1_bank_toward(Vec2::ZERO, 0.0, 90.0, Vec2::new(500.0, 800.0), 45.0);
        assert!(b > 10.0, "{b}");
        let (_, b2) = l1_bank_toward(Vec2::ZERO, 0.0, 110.0, Vec2::new(500.0, 800.0), 45.0);
        assert!(b2 > b);
        let (_, l) = l1_bank_toward(Vec2::ZERO, 0.0, 90.0, Vec2::new(-500.0, 800.0), 45.0);
        assert!((l + b).abs() < 1e-9);
    }
}
