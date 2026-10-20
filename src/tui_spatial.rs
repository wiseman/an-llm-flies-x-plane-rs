//! Spatial "targeting computer" view for the interactive TUI.
//!
//! An animated, braille-resolution display in the spirit of a fighter
//! targeting computer, chosen automatically from the flight phase:
//!
//! - **Approach** (final / roundout / flare): a perspective tunnel down
//!   the glidepath. Gates sit at fixed points along the path and stream
//!   toward the viewer as the range closes, over a converging ground grid.
//!   Lock brackets on the aim point tighten as glidepath and centerline
//!   errors shrink. A side-profile strip underneath shows the 3° path,
//!   roundout / flare heights, the runway and the recent altitude trail.
//! - **Pattern / Scope** (pattern legs, go-around, enroute, idle): a
//!   heading-up radar scope centred on own ship with a sweeping beam,
//!   rotating compass rose, pattern legs, and marching chevrons on the
//!   active leg.
//! - **Runway** (takeoff roll, rollout, runway exit): runway-up plan view
//!   with distance-remaining markers, the selected exit connector, and a
//!   scanning bar.
//! - **Taxi** (taxi / line-up / park): heading-up scope with the taxi
//!   route, the terminal pose (hold-short line or parking spot), and the
//!   nearby runway for context.
//!
//! Colour carries meaning: green for own ship and its trail, cyan for
//! guidance, blue for context and scan, pale ice for runways, magenta for
//! lock-on / designated targets, amber for caution and red for warnings.
//! `NO_COLOR` drops to the terminal's default colour.
//!
//! Every animation is a pure function of a frame counter the TUI bumps
//! once per redraw (its existing 10 Hz loop), so a test can render any
//! frame deterministically into a `TestBackend` buffer.

use std::collections::VecDeque;

use ratatui::buffer::Buffer;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::symbols::Marker;
use ratatui::text::{Line, Span};
use ratatui::widgets::canvas::{Canvas, Context, Line as CanvasLine};
use ratatui::widgets::{Block, Borders, Paragraph, Widget};

use crate::core::mission_manager::StatusSnapshot;
use crate::core::spatial::{LabeledLeg, PlanShape, ShapeGeom, ShapeRole, SpatialOverlay, SpatialSnapshot};
use crate::guidance::pattern_manager::glidepath_target_altitude_ft;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::types::{
    heading_to_vector, vector_to_heading, wrap_degrees_180, wrap_degrees_360, FlightPhase,
    Runway, Vec2, KT_TO_FPS, NM_TO_FT as FT_PER_NM,
};


type P = (f64, f64);

// ---------------------------------------------------------------------------
// View selection
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewKind {
    Approach,
    Pattern,
    Runway,
    Taxi,
    Scope,
}

impl ViewKind {
    pub fn label(self) -> &'static str {
        match self {
            ViewKind::Approach => "APPROACH",
            ViewKind::Pattern => "PATTERN",
            ViewKind::Runway => "RUNWAY",
            ViewKind::Taxi => "TAXI",
            ViewKind::Scope => "SCOPE",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        Some(match s.trim().to_ascii_lowercase().as_str() {
            "approach" | "appr" | "final" | "landing" => ViewKind::Approach,
            "pattern" | "pat" | "circuit" => ViewKind::Pattern,
            "runway" | "rwy" | "rollout" | "takeoff" => ViewKind::Runway,
            "taxi" | "ground" => ViewKind::Taxi,
            "scope" | "radar" | "map" | "enroute" => ViewKind::Scope,
            _ => return None,
        })
    }
}

/// Auto-select from the flight phase, or stay pinned on one view.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ViewMode {
    #[default]
    Auto,
    Pinned(ViewKind),
}

/// Parsed argument of the `/view` command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewCommand {
    Toggle,
    On,
    Off,
    /// Turn the view on in this mode.
    Mode(ViewMode),
}

pub const VIEW_COMMAND_HELP: &str =
    "on | off | auto | approach | pattern | runway | taxi | scope";

pub fn parse_view_command(arg: &str) -> Result<ViewCommand, String> {
    let a = arg.trim().to_ascii_lowercase();
    match a.as_str() {
        "" | "toggle" => Ok(ViewCommand::Toggle),
        "on" => Ok(ViewCommand::On),
        "off" => Ok(ViewCommand::Off),
        "auto" => Ok(ViewCommand::Mode(ViewMode::Auto)),
        other => ViewKind::parse(other)
            .map(|k| ViewCommand::Mode(ViewMode::Pinned(k)))
            .ok_or_else(|| format!("unknown view {other:?} — expected {VIEW_COMMAND_HELP}")),
    }
}

/// Pick the view for the current phase and active geometry.
pub fn select_view(snap: &StatusSnapshot, sp: &SpatialSnapshot) -> ViewKind {
    use FlightPhase::*;
    let ov = &sp.overlay;
    let ground_route =
        ov.pattern_legs.is_empty() && (!ov.route.is_empty() || ov.target_pose.is_some());
    match snap.phase {
        Some(Final | Roundout | Flare) => ViewKind::Approach,
        Some(TakeoffRoll | Rotate | Rollout | RunwayExit | TaxiClear) => ViewKind::Runway,
        Some(PatternEntry | Crosswind | Downwind | Base | GoAround | InitialClimb) => {
            ViewKind::Pattern
        }
        Some(Preflight)
            if snap.state.on_ground
                && RunwayFrame::new(sp.runway.clone())
                    .on_runway_surface(snap.state.position_ft, sp.runway_clear_margin_ft) =>
        {
            ViewKind::Runway
        }
        Some(EnrouteClimb | Cruise | Descent)
            if !ov.pattern_legs.is_empty()
                && snap.state.position_ft.distance_to(runway_midpoint(&sp.runway))
                    < 5.0 * FT_PER_NM =>
        {
            ViewKind::Pattern
        }
        _ if ground_route && snap.state.on_ground => ViewKind::Taxi,
        _ => ViewKind::Scope,
    }
}

fn runway_midpoint(runway: &Runway) -> Vec2 {
    runway.threshold_ft + heading_to_vector(runway.course_deg, runway.length_ft * 0.5)
}

// ---------------------------------------------------------------------------
// Trail
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrailPoint {
    pub t: f64,
    pub pos: Vec2,
    pub agl_ft: f64,
}

/// Short position/height history kept by the TUI (not the pilot).
#[derive(Debug, Clone, Default)]
pub struct Trail {
    points: VecDeque<TrailPoint>,
}

impl Trail {
    pub const MAX_POINTS: usize = 600;
    pub const MIN_DT_S: f64 = 0.5;

    /// Record a sample. Skips samples closer than `MIN_DT_S` in sim time
    /// or under 3 ft of movement; clears on a sim reset (time going
    /// backwards) or a teleport (a jump over 3000 ft in under 5 s).
    pub fn push(&mut self, t: f64, pos: Vec2, agl_ft: f64) {
        if let Some(last) = self.points.back().copied() {
            if t < last.t - 0.5 {
                self.points.clear();
            } else {
                let dt = t - last.t;
                let moved = pos.distance_to(last.pos);
                if moved > 3000.0 && dt < 5.0 {
                    self.points.clear();
                } else if dt < Self::MIN_DT_S || moved < 3.0 {
                    return;
                }
            }
        }
        self.points.push_back(TrailPoint { t, pos, agl_ft });
        while self.points.len() > Self::MAX_POINTS {
            self.points.pop_front();
        }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &TrailPoint> {
        self.points.iter()
    }

    pub fn clear(&mut self) {
        self.points.clear();
    }
}

// ---------------------------------------------------------------------------
// Palette
// ---------------------------------------------------------------------------

/// Semantic colours. Hues mean one thing each: green = own ship, cyan =
/// guidance, blue = context / scan, ice = runway, magenta = lock-on /
/// designated target, amber = caution, red = warning.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Palette {
    pub ship: Color,
    /// Own-ship trail, newest to oldest.
    pub trail: [Color; 3],
    pub guide: Color,
    pub guide_dim: Color,
    pub grid: Color,
    pub grid_dim: Color,
    pub runway: Color,
    pub runway_dim: Color,
    pub lock: Color,
    pub lock_dim: Color,
    pub caution: Color,
    pub warn: Color,
    /// Plan constraints (stay south of the 101).
    pub constraint: Color,
    pub text: Color,
    pub text_dim: Color,
    /// Radar beam afterglow, brightest first.
    pub sweep: [Color; 4],
}

impl Palette {
    pub fn color() -> Self {
        Palette {
            ship: Color::Rgb(110, 255, 140),
            trail: [
                Color::Rgb(70, 200, 100),
                Color::Rgb(40, 130, 65),
                Color::Rgb(25, 80, 42),
            ],
            guide: Color::Rgb(0, 230, 255),
            guide_dim: Color::Rgb(0, 125, 165),
            grid: Color::Rgb(40, 95, 150),
            grid_dim: Color::Rgb(26, 56, 92),
            runway: Color::Rgb(215, 238, 255),
            runway_dim: Color::Rgb(120, 150, 180),
            lock: Color::Rgb(255, 60, 200),
            lock_dim: Color::Rgb(135, 30, 105),
            caution: Color::Rgb(255, 176, 0),
            warn: Color::Rgb(255, 55, 55),
            constraint: Color::Rgb(190, 120, 255),
            text: Color::Rgb(205, 228, 248),
            text_dim: Color::Rgb(95, 130, 165),
            sweep: [
                Color::Rgb(120, 205, 255),
                Color::Rgb(55, 125, 195),
                Color::Rgb(34, 78, 130),
                Color::Rgb(22, 48, 82),
            ],
        }
    }

    /// Terminal default colour everywhere (shapes and labels still carry
    /// the information).
    pub fn mono() -> Self {
        let r = Color::Reset;
        Palette {
            ship: r,
            trail: [r; 3],
            guide: r,
            guide_dim: r,
            grid: r,
            grid_dim: r,
            runway: r,
            runway_dim: r,
            lock: r,
            lock_dim: r,
            caution: r,
            warn: r,
            constraint: r,
            text: r,
            text_dim: r,
            sweep: [r; 4],
        }
    }

    /// `NO_COLOR` (any non-empty value) selects `mono()`.
    pub fn from_env() -> Self {
        match std::env::var_os("NO_COLOR") {
            Some(v) if !v.is_empty() => Self::mono(),
            _ => Self::color(),
        }
    }

    /// The dim half of a blinking warning.
    fn warn_dim(&self) -> Color {
        match self.warn {
            Color::Rgb(r, g, b) => Color::Rgb(r / 2, g / 2, b / 2),
            c => c,
        }
    }

    fn lock_color(&self, lock: Lock, frame: u64) -> Color {
        match lock {
            Lock::Locked => self.lock,
            Lock::Tracking => {
                if pulse(frame, 10) > 0.5 { self.guide } else { self.guide_dim }
            }
            Lock::Lost => self.warn,
        }
    }
}

// ---------------------------------------------------------------------------
// Geometry (pure)
// ---------------------------------------------------------------------------

/// Axis-aligned canvas bounds: `x = [left, right]`, `y = [bottom, top]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub x: [f64; 2],
    pub y: [f64; 2],
}

impl Bounds {
    pub fn centered(cx: f64, cy: f64, half_w: f64, half_h: f64) -> Self {
        Bounds { x: [cx - half_w, cx + half_w], y: [cy - half_h, cy + half_h] }
    }

    pub fn width(&self) -> f64 {
        self.x[1] - self.x[0]
    }

    pub fn height(&self) -> f64 {
        self.y[1] - self.y[0]
    }

    pub fn center(&self) -> P {
        ((self.x[0] + self.x[1]) * 0.5, (self.y[0] + self.y[1]) * 0.5)
    }

    pub fn contains(&self, p: P) -> bool {
        p.0 >= self.x[0] && p.0 <= self.x[1] && p.1 >= self.y[0] && p.1 <= self.y[1]
    }

    /// Expand (never shrink) about the centre so one braille dot spans the
    /// same distance horizontally and vertically: a `cols × rows` cell
    /// area is `2·cols × 4·rows` dots, and terminal cells are about twice
    /// as tall as wide, so dots come out square.
    pub fn fit_aspect(self, cols: u16, rows: u16) -> Self {
        let dots_w = f64::from(cols.max(1)) * 2.0;
        let dots_h = f64::from(rows.max(1)) * 4.0;
        let upd = (self.width().max(1e-9) / dots_w).max(self.height().max(1e-9) / dots_h);
        let (cx, cy) = self.center();
        Bounds::centered(cx, cy, upd * dots_w * 0.5, upd * dots_h * 0.5)
    }

    fn inset(&self, eps_frac: f64) -> Self {
        let ex = self.width() * eps_frac;
        let ey = self.height() * eps_frac;
        Bounds { x: [self.x[0] + ex, self.x[1] - ex], y: [self.y[0] + ey, self.y[1] - ey] }
    }
}

/// Liang–Barsky clip of segment `a→b` to `bounds`. ratatui's canvas drops
/// any line with an endpoint outside the bounds, so every line goes
/// through this first.
pub fn clip_segment(a: P, b: P, bounds: &Bounds) -> Option<(P, P)> {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let mut t0 = 0.0_f64;
    let mut t1 = 1.0_f64;
    let checks = [
        (-dx, a.0 - bounds.x[0]),
        (dx, bounds.x[1] - a.0),
        (-dy, a.1 - bounds.y[0]),
        (dy, bounds.y[1] - a.1),
    ];
    for (p, q) in checks {
        if p == 0.0 {
            if q < 0.0 {
                return None;
            }
        } else {
            let r = q / p;
            if p < 0.0 {
                if r > t1 {
                    return None;
                }
                t0 = t0.max(r);
            } else {
                if r < t0 {
                    return None;
                }
                t1 = t1.min(r);
            }
        }
    }
    Some(((a.0 + t0 * dx, a.1 + t0 * dy), (a.0 + t1 * dx, a.1 + t1 * dy)))
}

/// Clip segment `a→b` to the disc of radius `r` centred on the origin
/// (the radar scope face).
pub fn clip_segment_circle(a: P, b: P, r: f64) -> Option<(P, P)> {
    let d = (b.0 - a.0, b.1 - a.1);
    let qa = d.0 * d.0 + d.1 * d.1;
    let c = a.0 * a.0 + a.1 * a.1 - r * r;
    if qa < 1e-12 {
        return (c <= 0.0).then_some((a, b));
    }
    let qb = 2.0 * (a.0 * d.0 + a.1 * d.1);
    let disc = qb * qb - 4.0 * qa * c;
    if disc < 0.0 {
        return None;
    }
    let s = disc.sqrt();
    let t_in = ((-qb - s) / (2.0 * qa)).max(0.0);
    let t_out = ((-qb + s) / (2.0 * qa)).min(1.0);
    if t_in >= t_out {
        return None;
    }
    Some((
        (a.0 + t_in * d.0, a.1 + t_in * d.1),
        (a.0 + t_out * d.0, a.1 + t_out * d.1),
    ))
}

/// First ladder value ≥ `required`, or the largest.
pub fn pick_range(required: f64, ladder: &[f64]) -> f64 {
    ladder
        .iter()
        .copied()
        .find(|r| *r >= required)
        .unwrap_or_else(|| ladder.last().copied().unwrap_or(required))
}

pub const SCOPE_RANGES_NM: [f64; 10] = [0.5, 1.0, 2.0, 3.0, 5.0, 10.0, 20.0, 40.0, 80.0, 160.0];
pub const TAXI_RANGES_FT: [f64; 7] = [150.0, 300.0, 600.0, 1200.0, 2500.0, 5000.0, 10000.0];

/// Plan-view projection from world feet (x east, y north) to canvas units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MapFrame {
    /// Own ship at the origin, `heading_deg` pointing up.
    HeadingUp { center: Vec2, heading_deg: f64 },
    /// Runway frame: `+x` down the runway (to the right on screen), right
    /// of centerline is down.
    RunwayUp { origin: Vec2, course_deg: f64 },
}

impl MapFrame {
    pub fn project(&self, p: Vec2) -> P {
        match *self {
            MapFrame::HeadingUp { center, heading_deg } => {
                let off = p - center;
                let fwd = heading_to_vector(heading_deg, 1.0);
                let right = heading_to_vector(heading_deg + 90.0, 1.0);
                (off.dot(right), off.dot(fwd))
            }
            MapFrame::RunwayUp { origin, course_deg } => {
                let off = p - origin;
                let fwd = heading_to_vector(course_deg, 1.0);
                let right = heading_to_vector(course_deg + 90.0, 1.0);
                (off.dot(fwd), -off.dot(right))
            }
        }
    }

    /// Screen-space unit vector for a world heading.
    pub fn dir(&self, heading_deg: f64) -> P {
        let o = self.project(Vec2::ZERO);
        let p = self.project(heading_to_vector(heading_deg, 1.0));
        (p.0 - o.0, p.1 - o.1)
    }
}

/// Glidepath height above field elevation at runway-frame `runway_x_ft`.
pub fn glidepath_height_agl_ft(frame: &RunwayFrame, runway_x_ft: f64, slope_deg: f64) -> f64 {
    glidepath_target_altitude_ft(frame, runway_x_ft, 0.0, slope_deg, 0.0)
}

/// Perspective camera for the approach tunnel. Sits at the aircraft
/// (runway frame: x along the runway, y right of centerline, z height
/// above the field), looks down the runway course, pitched down by the
/// glidepath angle — so on a perfect approach the aim point sits dead
/// centre.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ApproachCamera {
    pub eye: [f64; 3],
    pub pitch_down_deg: f64,
    pub focal: f64,
}

pub const CAMERA_NEAR_FT: f64 = 25.0;

impl ApproachCamera {
    /// (depth, right, up) in camera space.
    pub fn to_camera(&self, p: [f64; 3]) -> [f64; 3] {
        let t = self.pitch_down_deg.to_radians();
        let (dx, dy, dz) = (p[0] - self.eye[0], p[1] - self.eye[1], p[2] - self.eye[2]);
        [dx * t.cos() - dz * t.sin(), dy, dx * t.sin() + dz * t.cos()]
    }

    fn screen(&self, c: [f64; 3]) -> P {
        (self.focal * c[1] / c[0], self.focal * c[2] / c[0])
    }

    pub fn project(&self, p: [f64; 3]) -> Option<P> {
        let c = self.to_camera(p);
        (c[0] >= CAMERA_NEAR_FT).then(|| self.screen(c))
    }

    /// Project a 3-D segment, clipping it at the near plane first.
    pub fn project_segment(&self, a: [f64; 3], b: [f64; 3]) -> Option<(P, P)> {
        let mut ca = self.to_camera(a);
        let mut cb = self.to_camera(b);
        if ca[0] < CAMERA_NEAR_FT && cb[0] < CAMERA_NEAR_FT {
            return None;
        }
        let lerp = |p: [f64; 3], q: [f64; 3]| {
            let t = (CAMERA_NEAR_FT - p[0]) / (q[0] - p[0]);
            [CAMERA_NEAR_FT, p[1] + t * (q[1] - p[1]), p[2] + t * (q[2] - p[2])]
        };
        if ca[0] < CAMERA_NEAR_FT {
            ca = lerp(ca, cb);
        } else if cb[0] < CAMERA_NEAR_FT {
            cb = lerp(cb, ca);
        }
        Some((self.screen(ca), self.screen(cb)))
    }

    /// Where the velocity vector (along-runway, right, up; any units)
    /// points on screen, or `None` when it points behind the camera.
    pub fn flight_path_marker(&self, v: [f64; 3]) -> Option<P> {
        let t = self.pitch_down_deg.to_radians();
        let depth = v[0] * t.cos() - v[2] * t.sin();
        if depth <= 1e-6 {
            return None;
        }
        let up = v[0] * t.sin() + v[2] * t.cos();
        Some((self.focal * v[1] / depth, self.focal * up / depth))
    }
}

/// Runway-up view window `[lo, hi]` along the runway: covers the aircraft,
/// the exit connector, and a stretch of runway ahead.
pub fn runway_window(ac_x: f64, route_xs: &[f64], length_ft: f64) -> (f64, f64) {
    let (mut lo, mut hi) = (ac_x, ac_x + 1500.0);
    for &x in route_xs {
        lo = lo.min(x);
        hi = hi.max(x);
    }
    let lo = (lo - 600.0).max((-300.0_f64).min(ac_x - 200.0));
    let hi = (hi + 600.0).min((length_ft + 300.0).max(ac_x + 200.0));
    if hi - lo < 1500.0 {
        let mid = 0.5 * (lo + hi);
        (mid - 750.0, mid + 750.0)
    } else {
        (lo, hi)
    }
}

// ---------------------------------------------------------------------------
// Lock-on + animation (pure)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lock {
    Locked,
    Tracking,
    Lost,
}

impl Lock {
    pub fn label(self) -> &'static str {
        match self {
            Lock::Locked => "TGT LOCK",
            Lock::Tracking => "TRACKING",
            Lock::Lost => "NO LOCK",
        }
    }
}

/// Approach error as a fraction of a full-scale needle: vertical against
/// ±0.7° of glidepath, lateral against ±2.5° of localizer, both measured
/// from the aim point (floored at 400 ft so the needles don't go
/// hypersensitive over the numbers). 1.0 = full deflection.
pub fn approach_error_norm(vert_dev_ft: f64, lat_dev_ft: f64, dist_to_aim_ft: f64) -> f64 {
    let d = dist_to_aim_ft.abs().max(400.0);
    let v = vert_dev_ft / (d * 0.7_f64.to_radians().tan());
    let h = lat_dev_ft / (d * 2.5_f64.to_radians().tan());
    v.abs().max(h.abs())
}

/// Terminal-pose error: 40 ft or 20° of heading is one unit.
pub fn pose_error_norm(dist_ft: f64, heading_err_deg: f64) -> f64 {
    (dist_ft / 40.0).max(heading_err_deg.abs() / 20.0)
}

pub fn lock_from_error(err: f64) -> Lock {
    if err < 0.3 {
        Lock::Locked
    } else if err < 1.0 {
        Lock::Tracking
    } else {
        Lock::Lost
    }
}

/// Square wave: true for the first `on` frames of every `period`.
pub fn blink(frame: u64, period: u64, on: u64) -> bool {
    period == 0 || frame % period < on
}

/// Triangle wave in [0, 1] with the given period (frames).
pub fn pulse(frame: u64, period: u64) -> f64 {
    if period < 2 {
        return 1.0;
    }
    let half = period as f64 / 2.0;
    let ph = (frame % period) as f64;
    if ph <= half { ph / half } else { (period as f64 - ph) / half }
}

/// Deterministic pseudo-random value in [-1, 1] for (frame, salt).
pub fn jitter(frame: u64, salt: u64) -> f64 {
    let mut z = frame.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ salt.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z ^= z >> 31;
    z = z.wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^= z >> 29;
    (z % 2001) as f64 / 1000.0 - 1.0
}

/// Extra bracket size (as a fraction of the base size): tight and
/// breathing when locked, widening with error, jittering when lost.
pub fn bracket_spread(err: f64, frame: u64) -> f64 {
    let base = err.clamp(0.0, 2.0) * 0.6;
    match lock_from_error(err) {
        Lock::Locked => 0.06 * pulse(frame, 10),
        Lock::Tracking => base + 0.08 * pulse(frame, 14),
        Lock::Lost => base + 0.2 * jitter(frame, 7).abs(),
    }
}

/// Radar beam angle (degrees clockwise from screen-up): one turn every
/// 60 frames (6 s at the TUI's 10 Hz).
pub fn sweep_angle_deg(frame: u64) -> f64 {
    (frame % 60) as f64 * 6.0
}

/// Afterglow of a contact at screen bearing `target_deg` given the beam
/// at `sweep_deg`: 1.0 just painted, fading linearly to 0 over 180°.
pub fn afterglow(sweep_deg: f64, target_deg: f64) -> f64 {
    let behind = wrap_degrees_360(sweep_deg - target_deg);
    (1.0 - behind / 180.0).max(0.0)
}

/// Positions along a path of `length` where chevrons sit, marching
/// forward `step` per frame with `spacing` between them.
pub fn march_offsets(length: f64, spacing: f64, frame: u64, step: f64) -> Vec<f64> {
    if !(length > 0.0 && spacing > 0.0) {
        return Vec::new();
    }
    let mut out = Vec::new();
    let mut s = (frame as f64 * step).rem_euclid(spacing);
    while s < length && out.len() < 64 {
        out.push(s);
        s += spacing;
    }
    out
}

// ---------------------------------------------------------------------------
// Drawing helper
// ---------------------------------------------------------------------------

struct Pen<'p, 'c> {
    ctx: &'p mut Context<'c>,
    b: Bounds,
    clip_b: Bounds,
    cols: u16,
    rows: u16,
    /// Clip map content to the scope face (radius about the origin).
    clip_r: Option<f64>,
    /// Cells already holding a label: (row, first col, last col).
    labels: Vec<(i64, i64, i64)>,
    /// Low-priority labels printed only where they don't collide.
    deferred: Vec<(f64, f64, String, Style)>,
}

impl<'p, 'c> Pen<'p, 'c> {
    fn new(ctx: &'p mut Context<'c>, b: Bounds, cols: u16, rows: u16) -> Self {
        Pen {
            ctx,
            b,
            clip_b: b.inset(1e-6),
            cols,
            rows,
            clip_r: None,
            labels: Vec::new(),
            deferred: Vec::new(),
        }
    }

    /// Canvas units per braille dot.
    fn dot(&self) -> f64 {
        self.b.width() / (f64::from(self.cols.max(1)) * 2.0)
    }

    fn dot_y(&self) -> f64 {
        self.b.height() / (f64::from(self.rows.max(1)) * 4.0)
    }

    fn layer(&mut self) {
        self.ctx.layer();
    }

    fn raw(&mut self, a: P, b: P, c: Color) {
        if let Some((p, q)) = clip_segment(a, b, &self.clip_b) {
            self.ctx.draw(&CanvasLine::new(p.0, p.1, q.0, q.1, c));
        }
    }

    fn line(&mut self, a: P, b: P, c: Color) {
        match self.clip_r {
            Some(r) => {
                if let Some((p, q)) = clip_segment_circle(a, b, r) {
                    self.raw(p, q, c);
                }
            }
            None => self.raw(a, b, c),
        }
    }

    fn dashed(&mut self, a: P, b: P, c: Color, dash: f64, gap: f64, phase: f64) {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1e-9 || dash <= 0.0 {
            return;
        }
        let period = dash + gap.max(0.0);
        let (ux, uy) = (dx / len, dy / len);
        let mut s = -(phase.rem_euclid(period));
        let mut n = 0;
        while s < len && n < 400 {
            let s0 = s.max(0.0);
            let s1 = (s + dash).min(len);
            if s1 > s0 {
                self.line((a.0 + ux * s0, a.1 + uy * s0), (a.0 + ux * s1, a.1 + uy * s1), c);
            }
            s += period;
            n += 1;
        }
    }

    fn ring(&mut self, r: f64, c: Color, dashed: bool) {
        let n = 96;
        for i in 0..n {
            if dashed && i % 2 == 1 {
                continue;
            }
            let a0 = i as f64 / n as f64 * std::f64::consts::TAU;
            let a1 = (i + 1) as f64 / n as f64 * std::f64::consts::TAU;
            self.raw((r * a0.sin(), r * a0.cos()), (r * a1.sin(), r * a1.cos()), c);
        }
    }

    /// Print a label, nudged to fit inside the canvas. Skipped (returns
    /// false) when it would overlap a label already printed.
    fn text(&mut self, x: f64, y: f64, s: &str, style: Style) -> bool {
        let n = s.chars().count() as f64;
        let cols = f64::from(self.cols.max(2));
        let rows = f64::from(self.rows.max(2));
        let col_w = self.b.width() / (cols - 1.0);
        let row_h = self.b.height() / (rows - 1.0);
        let mut x = x.max(self.b.x[0]);
        let mut col = ((x - self.b.x[0]) / col_w).floor();
        if col + n > cols {
            col = (cols - n).max(0.0);
            x = self.b.x[0] + col * col_w;
        }
        let y = y.clamp(self.b.y[0], self.b.y[1]);
        let row = ((self.b.y[1] - y) / row_h).floor() as i64;
        let (c0, c1) = (col as i64, (col + n) as i64);
        if self.labels.iter().any(|(r, a, b)| *r == row && c0 <= *b && *a <= c1) {
            return false;
        }
        self.labels.push((row, c0, c1));
        self.ctx.print(x, y, Span::styled(s.to_string(), style));
        true
    }

    /// Queue a low-priority label for `flush_deferred`.
    fn text_low(&mut self, x: f64, y: f64, s: &str, style: Style) {
        let col_w = self.b.width() / (f64::from(self.cols.max(2)) - 1.0);
        let n = s.chars().count() as f64;
        self.deferred.push((x - n * 0.5 * col_w, y, s.to_string(), style));
    }

    fn flush_deferred(&mut self) {
        for (x, y, s, style) in std::mem::take(&mut self.deferred) {
            self.text(x, y, &s, style);
        }
    }

    /// Text centred horizontally on `x`.
    fn text_c(&mut self, x: f64, y: f64, s: &str, style: Style) -> bool {
        let col_w = self.b.width() / (f64::from(self.cols.max(2)) - 1.0);
        let n = s.chars().count() as f64;
        self.text(x - n * 0.5 * col_w, y, s, style)
    }

    /// Four corner brackets around `c`.
    fn brackets(&mut self, c: P, hw: f64, hh: f64, color: Color) {
        let aw = hw * 0.4;
        let ah = hh * 0.4;
        for (sx, sy) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
            let corner = (c.0 + sx * hw, c.1 + sy * hh);
            self.raw(corner, (corner.0 - sx * aw, corner.1), color);
            self.raw(corner, (corner.0, corner.1 - sy * ah), color);
        }
    }

    /// Delta-wing own-ship symbol at `at`, nose along unit vector `dir`.
    fn ownship(&mut self, at: P, dir: P, size: f64, color: Color) {
        let (fx, fy) = dir;
        let (rx, ry) = (fy, -fx);
        let pt = |f: f64, r: f64| (at.0 + fx * f * size + rx * r * size, at.1 + fy * f * size + ry * r * size);
        let nose = pt(1.0, 0.0);
        let lw = pt(-0.6, -0.8);
        let rw = pt(-0.6, 0.8);
        let notch = pt(-0.25, 0.0);
        self.raw(nose, lw, color);
        self.raw(lw, notch, color);
        self.raw(notch, rw, color);
        self.raw(rw, nose, color);
    }

    /// Small chevron with its tip at `at`, pointing along unit `dir`.
    fn chevron(&mut self, at: P, dir: P, size: f64, color: Color) {
        let (fx, fy) = dir;
        let (rx, ry) = (fy, -fx);
        let back = (at.0 - fx * size, at.1 - fy * size);
        self.line(at, (back.0 + rx * size, back.1 + ry * size), color);
        self.line(at, (back.0 - rx * size, back.1 - ry * size), color);
    }

    /// Path line with chevrons marching along it in the direction of travel.
    fn marching(&mut self, a: P, b: P, line: Color, chev: Color, frame: u64) {
        self.line(a, b, line);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1e-9 {
            return;
        }
        let dir = (dx / len, dy / len);
        let d = self.dot();
        for s in march_offsets(len, 7.0 * d, frame, 1.0 * d) {
            self.chevron((a.0 + dir.0 * s, a.1 + dir.1 * s), dir, 1.6 * d, chev);
        }
    }

    fn trail(&mut self, pts: &[P], pal: &Palette) {
        let n = pts.len();
        if n < 2 {
            return;
        }
        for i in 1..n {
            let age = (n - 1 - i) as f64 / n as f64;
            let c = if age < 0.25 {
                pal.trail[0]
            } else if age < 0.6 {
                pal.trail[1]
            } else {
                pal.trail[2]
            };
            self.line(pts[i - 1], pts[i], c);
        }
    }
}

fn fg(c: Color) -> Style {
    Style::default().fg(c)
}

fn bold(c: Color) -> Style {
    Style::default().fg(c).add_modifier(Modifier::BOLD)
}

fn render_canvas<F>(buf: &mut Buffer, area: Rect, b: Bounds, paint: F)
where
    F: Fn(&mut Pen),
{
    if area.width < 2 || area.height < 2 {
        return;
    }
    let (cols, rows) = (area.width, area.height);
    Canvas::default()
        .x_bounds(b.x)
        .y_bounds(b.y)
        .marker(Marker::Braille)
        .paint(|ctx| {
            let mut pen = Pen::new(ctx, b, cols, rows);
            paint(&mut pen);
        })
        .render(area, buf);
}

// ---------------------------------------------------------------------------
// Readout helpers
// ---------------------------------------------------------------------------

fn kv(spans: &mut Vec<Span<'static>>, pal: &Palette, k: &str, v: String, style: Style) {
    if !spans.is_empty() {
        spans.push(Span::raw("  "));
    }
    spans.push(Span::styled(format!("{k} "), fg(pal.text_dim)));
    spans.push(Span::styled(v, style));
}

pub fn fmt_range(ft: f64) -> String {
    if ft.abs() >= 0.25 * FT_PER_NM {
        format!("{:.2}NM", ft / FT_PER_NM)
    } else {
        format!("{:.0}FT", ft)
    }
}

fn fmt_lr(y_ft: f64) -> String {
    if y_ft.abs() < 0.5 {
        "CTR".to_string()
    } else if y_ft > 0.0 {
        format!("R{:.0}", y_ft)
    } else {
        format!("L{:.0}", -y_ft)
    }
}

fn phase_tag(snap: &StatusSnapshot) -> String {
    match snap.phase {
        Some(p) => p.value().replace('_', " ").to_ascii_uppercase(),
        None => snap
            .lateral_owner_idx
            .and_then(|i| snap.active_profiles.get(i))
            .map(|n| n.replace('_', " ").to_ascii_uppercase())
            .unwrap_or_else(|| "IDLE".to_string()),
    }
}

fn with_target(actual: f64, target: Option<f64>) -> String {
    match target {
        Some(t) if (t - actual).abs() >= 1.0 => format!("{actual:.0}→{t:.0}"),
        _ => format!("{actual:.0}"),
    }
}

fn runway_label(runway: &Runway) -> String {
    runway.id.clone().unwrap_or_else(|| format!("{:03.0}", wrap_degrees_360(runway.course_deg)))
}

// ---------------------------------------------------------------------------
// Widget
// ---------------------------------------------------------------------------

/// Everything a view needs to draw one frame.
pub struct SpatialWidget<'a> {
    pub snapshot: Option<&'a StatusSnapshot>,
    pub spatial: Option<&'a SpatialSnapshot>,
    pub trail: &'a Trail,
    pub mode: ViewMode,
    pub frame: u64,
    pub palette: Palette,
}

struct ViewCtx<'a> {
    snap: &'a StatusSnapshot,
    sp: &'a SpatialSnapshot,
    trail: &'a Trail,
    frame: u64,
    pal: Palette,
}

impl Widget for SpatialWidget<'_> {
    fn render(self, area: Rect, buf: &mut Buffer) {
        let pal = self.palette;
        let block = Block::default().borders(Borders::ALL).border_style(fg(pal.grid));
        let inner = block.inner(area);
        let (Some(snap), Some(sp)) = (self.snapshot, self.spatial) else {
            block
                .title(Line::from(Span::styled(" TARGETING ", bold(pal.text_dim))))
                .render(area, buf);
            let cursor = if blink(self.frame, 10, 5) { "_" } else { " " };
            Paragraph::new(Line::from(Span::styled(
                format!("AWAITING TELEMETRY{cursor}"),
                fg(pal.text_dim),
            )))
            .render(inner, buf);
            return;
        };
        let kind = match self.mode {
            ViewMode::Pinned(k) => k,
            ViewMode::Auto => select_view(snap, sp),
        };
        let mode_tag = match self.mode {
            ViewMode::Auto => "AUTO",
            ViewMode::Pinned(_) => "PIN",
        };
        let pip = if blink(self.frame, 10, 6) { "◆" } else { "◇" };
        let title = Line::from(vec![
            Span::styled(format!(" {pip} "), fg(pal.lock)),
            Span::styled("TARGETING ", bold(pal.text)),
            Span::styled(format!("// {} ", kind.label()), bold(pal.guide)),
            Span::styled(format!("{mode_tag} "), fg(pal.text_dim)),
        ]);
        if inner.width < 24 || inner.height < 5 {
            block.title(title).render(area, buf);
            Paragraph::new(Line::from(Span::styled("ENLARGE PANE", fg(pal.text_dim))))
                .render(inner, buf);
            return;
        }
        let ctx = ViewCtx { snap, sp, trail: self.trail, frame: self.frame, pal };
        let rows = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Length(1), Constraint::Min(1)])
            .split(inner);
        let (readout, status) = match kind {
            ViewKind::Approach => draw_approach(buf, rows[1], &ctx),
            ViewKind::Runway => draw_runway(buf, rows[1], &ctx),
            ViewKind::Pattern | ViewKind::Taxi | ViewKind::Scope => {
                draw_scope(buf, rows[1], &ctx, kind)
            }
        };
        Paragraph::new(readout).render(rows[0], buf);
        let mut block = block.title(title);
        if let Some(s) = status {
            block = block.title(s.right_aligned());
        }
        block.render(area, buf);
    }
}

// ---------------------------------------------------------------------------
// Approach: perspective tunnel + side profile
// ---------------------------------------------------------------------------

/// Runway-frame approach state derived from the snapshot.
#[derive(Debug, Clone)]
pub struct ApproachState {
    pub frame: RunwayFrame,
    pub ax: f64,
    pub ay: f64,
    pub agl: f64,
    pub slope_deg: f64,
    pub aim_x: f64,
    pub gp_h: f64,
    pub vdev: f64,
    pub dist_to_aim: f64,
    pub err: f64,
    pub lock: Lock,
}

pub fn approach_state(snap: &StatusSnapshot, sp: &SpatialSnapshot) -> ApproachState {
    let frame = RunwayFrame::new(sp.runway.clone());
    let rf = frame.to_runway_frame(snap.state.position_ft);
    let slope_deg = snap
        .last_guidance
        .as_ref()
        .and_then(|g| g.glidepath.as_ref())
        .map(|g| g.slope_deg)
        .filter(|s| *s > 0.5 && *s < 10.0)
        .unwrap_or(3.0);
    let aim_x = frame.touchdown_runway_x_ft();
    let agl = snap.state.alt_agl_ft;
    let gp_h = glidepath_height_agl_ft(&frame, rf.x, slope_deg);
    let vdev = agl - gp_h;
    let dist_to_aim = aim_x - rf.x;
    let err = approach_error_norm(vdev, rf.y, dist_to_aim);
    ApproachState {
        frame,
        ax: rf.x,
        ay: rf.y,
        agl,
        slope_deg,
        aim_x,
        gp_h,
        vdev,
        dist_to_aim,
        err,
        lock: lock_from_error(err),
    }
}

fn draw_approach(buf: &mut Buffer, area: Rect, v: &ViewCtx) -> (Line<'static>, Option<Line<'static>>) {
    let pal = v.pal;
    let st = approach_state(v.snap, v.sp);
    let s = &v.snap.state;
    let lock_c = pal.lock_color(st.lock, v.frame);

    let mut spans = Vec::new();
    kv(&mut spans, &pal, "RNG", fmt_range(st.dist_to_aim), bold(pal.text));
    kv(&mut spans, &pal, "HAT", format!("{:.0}", st.agl), bold(pal.text));
    let gp_c = match st.lock {
        Lock::Locked => pal.text,
        Lock::Tracking => pal.caution,
        Lock::Lost => pal.warn,
    };
    let hilo = if st.vdev >= 0.0 { "HI" } else { "LO" };
    kv(&mut spans, &pal, "GP", format!("{:+.0} {hilo}", st.vdev), bold(gp_c));
    kv(&mut spans, &pal, "LOC", fmt_lr(st.ay), bold(gp_c));
    kv(&mut spans, &pal, "GS", format!("{:.0}", s.gs_kt), fg(pal.text));
    kv(&mut spans, &pal, "VS", format!("{:+.0}", s.vs_fpm), fg(pal.text));

    let strip_h = if area.height >= 16 { 6 } else { 0 };
    let parts = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(4), Constraint::Length(strip_h)])
        .split(area);
    draw_tunnel(buf, parts[0], v, &st);
    if strip_h > 0 {
        draw_profile_strip(buf, parts[1], v, &st);
    }
    let status = Line::from(Span::styled(format!(" {} ", st.lock.label()), bold(lock_c)));
    (Line::from(spans), Some(status))
}

fn draw_tunnel(buf: &mut Buffer, area: Rect, v: &ViewCtx, st: &ApproachState) {
    let (cols, rows) = (area.width, area.height);
    let half_w = 100.0;
    let half_h = half_w * f64::from(rows.max(1)) * 4.0 / (f64::from(cols.max(1)) * 2.0);
    let b = Bounds::centered(0.0, 0.0, half_w, half_h);
    let cam = ApproachCamera {
        eye: [st.ax, st.ay, st.agl.max(2.0)],
        pitch_down_deg: st.slope_deg,
        focal: half_w / 24.0_f64.to_radians().tan(),
    };
    let pal = v.pal;
    let frame = v.frame;
    let rw = &v.sp.runway;
    let s = &v.snap.state;
    render_canvas(buf, area, b, |pen| {
        let d = pen.dot();
        // Layer 0: scanline + horizon + ground grid.
        let scan_y = b.y[1] - (frame % 24) as f64 / 24.0 * b.height();
        pen.dashed((b.x[0], scan_y), (b.x[1], scan_y), pal.grid_dim, 2.0 * d, 3.0 * d, 0.0);
        if let Some(h) = cam.project([st.ax + 1.0e7, st.ay, 0.0]) {
            pen.raw((b.x[0], h.1), (b.x[1], h.1), pal.grid);
        }
        for off in [-3000.0, -1000.0, -350.0, 350.0, 1000.0, 3000.0] {
            if let Some((p, q)) =
                cam.project_segment([st.ax, off, 0.0], [st.ax + 40_000.0, off, 0.0])
            {
                pen.raw(p, q, pal.grid_dim);
            }
        }
        // Cross lines every 1000 ft, world-fixed so they stream past;
        // stop once successive lines crowd within ~3 dots on screen.
        let spacing = 1000.0;
        let k0 = ((st.ax + CAMERA_NEAR_FT * 2.0) / spacing).ceil() as i64;
        let mut last_y: Option<f64> = None;
        for k in k0..k0 + 24 {
            let x = k as f64 * spacing;
            let Some((p, q)) = cam.project_segment([x, -3000.0, 0.0], [x, 3000.0, 0.0]) else {
                continue;
            };
            if last_y.is_some_and(|ly| (ly - p.1).abs() < 3.0 * pen.dot_y()) {
                break;
            }
            last_y = Some(p.1);
            let c = if x - st.ax < 3000.0 { pal.grid } else { pal.grid_dim };
            pen.raw(p, q, c);
        }
        pen.layer();

        // Layer 1: runway + extended centerline.
        let hw = rw.width_ft * 0.5;
        let len = rw.length_ft;
        let seg = |pen: &mut Pen, a: [f64; 3], bb: [f64; 3], c: Color| {
            if let Some((p, q)) = cam.project_segment(a, bb) {
                pen.raw(p, q, c);
            }
        };
        let mut x = st.ax.max(-60_000.0);
        while x < -1.0 {
            seg(pen, [x, 0.0, 0.0], [(x + 300.0).min(0.0), 0.0, 0.0], pal.guide_dim);
            x += 600.0;
        }
        seg(pen, [0.0, -hw, 0.0], [len, -hw, 0.0], pal.runway);
        seg(pen, [0.0, hw, 0.0], [len, hw, 0.0], pal.runway);
        seg(pen, [0.0, -hw, 0.0], [0.0, hw, 0.0], pal.runway);
        seg(pen, [len, -hw, 0.0], [len, hw, 0.0], pal.runway);
        let thr = rw.displaced_threshold_ft;
        if thr > 1.0 {
            seg(pen, [thr, -hw, 0.0], [thr, hw, 0.0], pal.runway);
        }
        let mut cx = thr + 150.0;
        while cx < len - 150.0 {
            seg(pen, [cx, 0.0, 0.0], [cx + 120.0, 0.0, 0.0], pal.runway_dim);
            cx += 250.0;
        }
        for side in [-0.45, 0.45] {
            seg(pen, [st.aim_x, side * hw, 0.0], [st.aim_x + 150.0, side * hw, 0.0], pal.runway);
        }
        pen.layer();

        // Layer 2: glidepath gates streaming toward the viewer.
        let gate_s = if st.dist_to_aim < 6000.0 { 500.0 } else { 1000.0 };
        let k_max = ((st.aim_x - st.ax - CAMERA_NEAR_FT * 2.5) / gate_s).floor() as i64;
        let mut prev: Option<[[f64; 3]; 4]> = None;
        for (n, k) in (1..=k_max).rev().take(10).enumerate() {
            let gx = st.aim_x - k as f64 * gate_s;
            let gh = glidepath_height_agl_ft(&st.frame, gx, st.slope_deg);
            let ghw = (gh * 0.8).clamp(25.0, 140.0);
            let ghh = ghw * 0.6;
            let corners = [
                [gx, -ghw, gh - ghh],
                [gx, ghw, gh - ghh],
                [gx, ghw, gh + ghh],
                [gx, -ghw, gh + ghh],
            ];
            let c = if n == 0 {
                if pulse(frame, 8) > 0.5 { pal.guide } else { pal.guide_dim }
            } else if n < 5 {
                pal.guide_dim
            } else {
                pal.grid
            };
            for i in 0..4 {
                seg(pen, corners[i], corners[(i + 1) % 4], c);
            }
            if let Some(p) = prev {
                for i in 0..4 {
                    seg(pen, p[i], corners[i], pal.grid_dim);
                }
            }
            prev = Some(corners);
        }
        pen.layer();

        // Layer 3: boresight, flight-path marker, lock brackets.
        pen.raw((-7.0 * d, 0.0), (-3.0 * d, 0.0), pal.text_dim);
        pen.raw((3.0 * d, 0.0), (7.0 * d, 0.0), pal.text_dim);
        pen.raw((0.0, -2.0 * d), (0.0, -4.0 * d), pal.text_dim);
        if s.gs_kt >= 5.0 {
            let rel = (s.track_deg - rw.course_deg).to_radians();
            let gs = s.gs_kt * KT_TO_FPS;
            let vel = [gs * rel.cos(), gs * rel.sin(), s.vs_fpm / 60.0];
            if let Some(fpm) = cam.flight_path_marker(vel) {
                let inside = b.inset(0.05).contains(fpm);
                let p = (fpm.0.clamp(b.x[0] + 4.0 * d, b.x[1] - 4.0 * d), fpm.1.clamp(b.y[0] + 3.0 * d, b.y[1] - 3.0 * d));
                let c = if inside { pal.ship } else { pal.caution };
                let r = 1.5 * d;
                pen.raw((p.0 - r, p.1), (p.0, p.1 + r), c);
                pen.raw((p.0, p.1 + r), (p.0 + r, p.1), c);
                pen.raw((p.0 + r, p.1), (p.0, p.1 - r), c);
                pen.raw((p.0, p.1 - r), (p.0 - r, p.1), c);
                pen.raw((p.0 - 4.0 * r, p.1), (p.0 - r, p.1), c);
                pen.raw((p.0 + r, p.1), (p.0 + 4.0 * r, p.1), c);
                pen.raw((p.0, p.1 + r), (p.0, p.1 + 2.5 * r), c);
            }
        }
        if let Some(aim) = cam.project([st.aim_x, 0.0, 0.0]) {
            let spread = bracket_spread(st.err, frame);
            let (jx, jy) = if st.lock == Lock::Lost {
                (jitter(frame, 1) * 2.0 * d, jitter(frame, 2) * 2.0 * d)
            } else {
                (0.0, 0.0)
            };
            let c = pal.lock_color(st.lock, frame);
            let show = st.lock != Lock::Lost || blink(frame, 6, 4);
            if show {
                let hw = 6.0 * d * (1.0 + 2.0 * spread);
                let hh = 4.0 * d * (1.0 + 2.0 * spread);
                pen.brackets((aim.0 + jx, aim.1 + jy), hw, hh, c);
                pen.text_c(aim.0, aim.1 - hh - 2.0 * pen.dot_y(), &fmt_range(st.dist_to_aim), fg(c));
            }
        }
        pen.text(b.x[0], b.y[1], &format!("RWY {} {:.1}°", runway_label(rw), st.slope_deg), fg(pal.text_dim));
        let lbl = st.lock.label();
        if st.lock != Lock::Lost || blink(frame, 6, 4) {
            pen.text(b.x[1], b.y[0], lbl, bold(pal.lock_color(st.lock, frame)));
        }
    });
}

fn draw_profile_strip(buf: &mut Buffer, area: Rect, v: &ViewCtx, st: &ApproachState) {
    let pal = v.pal;
    let sp = v.sp;
    let span = (st.aim_x - st.ax).max(1500.0);
    let x_left = st.ax - 0.15 * span;
    let x_right = st.aim_x.max(st.ax) + 0.35 * span;
    let gp_left = glidepath_height_agl_ft(&st.frame, x_left, st.slope_deg);
    let top = st.agl.max(gp_left).max(sp.roundout_height_ft * 2.0).max(80.0) * 1.15;
    let rows = f64::from(area.height.max(1));
    let bottom = -top * 1.6 / (rows * 4.0 - 1.6).max(1.0) * 1.0;
    let b = Bounds { x: [x_left, x_right], y: [bottom, top] };
    let pts: Vec<P> = v
        .trail
        .iter()
        .map(|t| (st.frame.to_runway_frame(t.pos).x, t.agl_ft))
        .collect();
    let rw = &sp.runway;
    render_canvas(buf, area, b, |pen| {
        let dy = pen.dot_y();
        let dx = pen.dot();
        pen.raw((x_left, 0.0), (x_right, 0.0), pal.grid_dim);
        // Glidepath.
        pen.line((x_left, gp_left), (st.aim_x, 0.0), pal.guide);
        // Roundout / flare heights, if they resolve to at least ~2 dots.
        for (h, lbl) in [(sp.roundout_height_ft, "RO"), (sp.flare_start_ft, "FL")] {
            if h / dy >= 2.0 {
                let x0 = st.aim_x - h / st.slope_deg.to_radians().tan() * 2.5;
                pen.dashed((x0, h), (st.aim_x + 600.0, h), pal.guide_dim, 2.0 * dx, 2.0 * dx, 0.0);
                pen.text(x0 - 5.0 * dx, h, &format!("{lbl}{h:.0}"), fg(pal.text_dim));
            }
        }
        pen.layer();
        // Runway slab.
        pen.raw((0.0, 0.0), (rw.length_ft, 0.0), pal.runway);
        pen.raw((0.0, -dy), (rw.length_ft, -dy), pal.runway_dim);
        let thr = rw.displaced_threshold_ft;
        pen.raw((thr, 0.0), (thr, 2.0 * dy), pal.runway);
        pen.raw((st.aim_x, -dy), (st.aim_x, 2.5 * dy), pal.guide);
        pen.trail(&pts, &pal);
        pen.layer();
        // Deviation tick and own ship.
        let c = match st.lock {
            Lock::Locked => pal.ship,
            Lock::Tracking => pal.caution,
            Lock::Lost => pal.warn,
        };
        pen.raw((st.ax, st.gp_h), (st.ax, st.agl), c);
        let r = 1.2 * dx;
        let ry = 1.2 * dy;
        pen.raw((st.ax - r, st.agl), (st.ax, st.agl + ry), pal.ship);
        pen.raw((st.ax, st.agl + ry), (st.ax + r, st.agl), pal.ship);
        pen.raw((st.ax + r, st.agl), (st.ax, st.agl - ry), pal.ship);
        pen.raw((st.ax, st.agl - ry), (st.ax - r, st.agl), pal.ship);
        pen.text(x_left, b.y[0], "PROFILE", fg(pal.text_dim));
        pen.text(thr, b.y[0], "THR", fg(pal.text_dim));
        pen.text(x_right, top, &format!("{:.0}FT", top), fg(pal.text_dim));
    });
}

// ---------------------------------------------------------------------------
// Runway-up plan view
// ---------------------------------------------------------------------------

fn draw_runway(buf: &mut Buffer, area: Rect, v: &ViewCtx) -> (Line<'static>, Option<Line<'static>>) {
    let pal = v.pal;
    let sp = v.sp;
    let rw = &sp.runway;
    let s = &v.snap.state;
    let frame = RunwayFrame::new(rw.clone());
    let mf = MapFrame::RunwayUp { origin: rw.threshold_ft, course_deg: rw.course_deg };
    let ac = mf.project(s.position_ft);
    let ov = &sp.overlay;
    let route: Vec<(P, P)> = ov
        .route
        .iter()
        .map(|l| (mf.project(l.leg.start_ft), mf.project(l.leg.end_ft)))
        .collect();
    let route_xs: Vec<f64> = route.iter().flat_map(|(a, b)| [a.0, b.0]).collect();
    let (lo, hi) = runway_window(ac.0, &route_xs, rw.length_ft);
    let max_y = route
        .iter()
        .flat_map(|(a, b)| [a.1.abs(), b.1.abs()])
        .fold(ac.1.abs() * 1.4, f64::max);
    let half_h = (rw.width_ft * 2.0).max(max_y * 1.3).max(120.0);
    let b = Bounds { x: [lo, hi], y: [-half_h, half_h] }.fit_aspect(area.width, area.height);
    let trail: Vec<P> = v.trail.iter().map(|t| mf.project(t.pos)).collect();
    let fr = v.frame;
    let active = ov.route_active_idx;
    let exit_end = route.last().map(|(_, e)| *e);
    let exit_start_x = route.first().map(|(a, _)| a.0);
    let label = ov.target_label.clone();
    let rel = wrap_degrees_180(s.heading_deg - rw.course_deg).to_radians();
    let ship_dir = (rel.cos(), -rel.sin());

    render_canvas(buf, area, b, |pen| {
        let d = pen.dot();
        let dy = pen.dot_y();
        // Layer 0: scanning bar with afterglow.
        let scan_x = b.x[0] + (fr % 40) as f64 / 40.0 * b.width();
        for (i, c) in pal.sweep.iter().take(3).enumerate() {
            let x = scan_x - i as f64 * 2.5 * d;
            pen.raw((x, b.y[0]), (x, b.y[1]), *c);
        }
        pen.layer();
        // Layer 1: runway.
        let hw = rw.width_ft * 0.5;
        let len = rw.length_ft;
        pen.raw((0.0, hw), (len, hw), pal.runway);
        pen.raw((0.0, -hw), (len, -hw), pal.runway);
        pen.raw((0.0, -hw), (0.0, hw), pal.runway);
        pen.raw((len, -hw), (len, hw), pal.runway);
        let thr = rw.displaced_threshold_ft;
        if thr > 1.0 {
            pen.raw((thr, -hw), (thr, hw), pal.runway_dim);
        }
        if rw.width_ft / dy >= 6.0 {
            pen.dashed((thr + 100.0, 0.0), (len - 100.0, 0.0), pal.runway_dim, 120.0, 130.0, 0.0);
        }
        let aim = frame.touchdown_runway_x_ft();
        pen.raw((aim, hw * 0.5), (aim + 150.0, hw * 0.5), pal.runway_dim);
        pen.raw((aim, -hw * 0.5), (aim + 150.0, -hw * 0.5), pal.runway_dim);
        // Distance-remaining markers every 1000 ft.
        let mut k = 1;
        while (k as f64) * 1000.0 < len {
            let x = len - k as f64 * 1000.0;
            pen.raw((x, hw + dy), (x, hw + 3.0 * dy), pal.grid);
            pen.text_c(x, -hw - 3.0 * dy, &k.to_string(), fg(pal.text_dim));
            k += 1;
        }
        pen.text(-2.0 * d, hw + 6.0 * dy, &format!("RWY {}", runway_label(rw)), fg(pal.text_dim));
        pen.layer();
        // Layer 2: exit route, trail.
        if let Some(x0) = exit_start_x {
            pen.dashed((x0, -hw), (x0, hw), pal.guide_dim, 1.0 * d, 1.0 * d, 0.0);
        }
        for (i, (a, bb)) in route.iter().enumerate() {
            match active {
                Some(ai) if i < ai => pen.line(*a, *bb, pal.grid),
                Some(ai) if i == ai => pen.marching(*a, *bb, pal.guide_dim, pal.guide, fr),
                _ => pen.marching(*a, *bb, pal.guide_dim, pal.guide_dim, fr),
            }
        }
        pen.trail(&trail, &pal);
        pen.layer();
        // Layer 3: designated exit, own ship.
        if let Some(e) = exit_end {
            let done = active.is_some_and(|ai| ai >= route.len());
            let c = if done || pulse(fr, 12) > 0.5 { pal.lock } else { pal.lock_dim };
            let sz = 3.0 * d * (1.0 + 0.3 * pulse(fr, 12));
            pen.brackets(e, sz, sz, c);
            if let Some(l) = &label {
                pen.text(e.0 + sz + d, e.1, &format!("CLR {l}"), fg(c));
            }
        }
        pen.ownship(ac, ship_dir, (3.5 * d).max(18.0), pal.ship);
    });

    let mut spans = Vec::new();
    kv(&mut spans, &pal, "", phase_tag(v.snap), bold(pal.guide));
    kv(&mut spans, &pal, "GS", format!("{:.0}", s.gs_kt), bold(pal.text));
    kv(&mut spans, &pal, "IAS", format!("{:.0}", s.ias_kt), fg(pal.text));
    kv(&mut spans, &pal, "REM", format!("{:.0}", (rw.length_ft - ac.0).max(0.0)), fg(pal.text));
    kv(&mut spans, &pal, "CL", fmt_lr(-ac.1), fg(pal.text));
    if let (Some(x0), Some(l)) = (exit_start_x, &ov.target_label) {
        kv(&mut spans, &pal, "EXIT", format!("{l} {:+.0}", x0 - ac.0), bold(pal.guide));
    }
    let status = Line::from(Span::styled(
        format!(" HDG {:03.0} ", wrap_degrees_360(s.heading_deg)),
        fg(pal.text_dim),
    ));
    (Line::from(spans), Some(status))
}

// ---------------------------------------------------------------------------
// Heading-up scope: pattern, taxi, enroute
// ---------------------------------------------------------------------------

/// How far the view must reach to show the plan's active, target and
/// trigger shapes (up to 10 nm).
fn plan_reach_ft(ov: &SpatialOverlay, pos: Vec2) -> f64 {
    ov.plan
        .iter()
        .filter(|sh| matches!(sh.role, ShapeRole::Active | ShapeRole::Target | ShapeRole::Trigger))
        .map(|sh| plan_shape_reach_ft(sh, pos))
        .fold(0.0, f64::max)
        .min(10.0 * FT_PER_NM)
}

/// Scope radius in feet for the view.
pub fn scope_range_ft(kind: ViewKind, snap: &StatusSnapshot, sp: &SpatialSnapshot) -> f64 {
    let pos = snap.state.position_ft;
    let ov = &sp.overlay;
    match kind {
        ViewKind::Taxi => {
            let mut req: f64 = 120.0;
            let from = ov.route_active_idx.unwrap_or(0);
            for l in ov.route.iter().skip(from).take(2) {
                req = req.max(pos.distance_to(l.leg.end_ft));
            }
            if let Some(p) = ov.target_pose {
                req = req.max(pos.distance_to(p.position_ft));
            }
            pick_range(req.min(5000.0) * 1.1, &TAXI_RANGES_FT)
        }
        ViewKind::Pattern => {
            let rw = &sp.runway;
            let mut req = pos.distance_to(rw.threshold_ft).max(
                pos.distance_to(rw.threshold_ft + heading_to_vector(rw.course_deg, rw.length_ft)),
            );
            for l in &ov.pattern_legs {
                if l.label == "FINAL" {
                    continue;
                }
                req = req.max(pos.distance_to(l.leg.start_ft)).max(pos.distance_to(l.leg.end_ft));
            }
            req = req.max(plan_reach_ft(ov, pos));
            pick_range((req * 1.1 / FT_PER_NM).min(10.0), &SCOPE_RANGES_NM) * FT_PER_NM
        }
        _ => {
            let speed_nm = (snap.state.gs_kt / 60.0 * 3.0).max(1.0);
            let rwy_nm = pos.distance_to(sp.runway.threshold_ft) / FT_PER_NM;
            let mut req = speed_nm;
            if rwy_nm < 40.0 {
                req = req.max(rwy_nm * 1.15);
            }
            if let Some(w) = snap.last_guidance.as_ref().and_then(|g| g.target_waypoint.as_ref()) {
                req = req.max(pos.distance_to(w.position_ft) / FT_PER_NM * 1.15);
            }
            req = req.max(plan_reach_ft(ov, pos) / FT_PER_NM * 1.15);
            pick_range(req, &SCOPE_RANGES_NM) * FT_PER_NM
        }
    }
}

fn fmt_scope_range(r_ft: f64, kind: ViewKind) -> String {
    if kind == ViewKind::Taxi {
        format!("{:.0}FT", r_ft)
    } else {
        let nm = r_ft / FT_PER_NM;
        if nm < 1.0 { format!("{nm:.1}NM") } else { format!("{nm:.0}NM") }
    }
}

fn leg_proj(mf: &MapFrame, l: &LabeledLeg) -> (P, P) {
    (mf.project(l.leg.start_ft), mf.project(l.leg.end_ft))
}

/// How far from `pos` the view must reach to show `sh`: all of a circle
/// or closed course, the nearest part of a road, the point.
fn plan_shape_reach_ft(sh: &PlanShape, pos: Vec2) -> f64 {
    match &sh.geom {
        ShapeGeom::Circle { center, radius_ft } => pos.distance_to(*center) + radius_ft,
        ShapeGeom::Ring(pts) => pts.iter().map(|p| pos.distance_to(*p)).fold(0.0, f64::max),
        g => pos.distance_to(g.nearest_point(pos)),
    }
}

/// The plan's geometry: the step being flown (guide), what it flies to
/// (lock, with a leader), what a turn waits for (caution, marching),
/// and constraints (violet, dashed; red and blinking when broken).
/// Labels sit at the shape's nearest point, or on the scope edge toward
/// it when it's out of range.
fn draw_plan(pen: &mut Pen, mf: &MapFrame, ov: &SpatialOverlay, pal: &Palette, fr: u64, r: f64, pos: Vec2) {
    let d = pen.dot();
    let march = (fr as f64) * 0.6 * d;
    for sh in &ov.plan {
        let color = match sh.role {
            ShapeRole::Active => pal.guide,
            ShapeRole::Target => pal.lock,
            ShapeRole::Trigger => pal.caution,
            ShapeRole::Constraint => pal.constraint,
            ShapeRole::Violating => {
                if pulse(fr, 6) > 0.5 { pal.warn } else { pal.warn_dim() }
            }
        };
        let seg = |pen: &mut Pen, a: P, b: P| match sh.role {
            ShapeRole::Active | ShapeRole::Target => pen.line(a, b, color),
            ShapeRole::Trigger => pen.dashed(a, b, color, 2.5 * d, 1.5 * d, -march),
            ShapeRole::Constraint | ShapeRole::Violating => pen.dashed(a, b, color, 1.5 * d, 1.5 * d, 0.0),
        };
        let pts: Vec<P> = match &sh.geom {
            ShapeGeom::Point(p) => {
                let c = mf.project(*p);
                let k = 2.0 * d;
                let diamond = [(c.0, c.1 + k), (c.0 + k, c.1), (c.0, c.1 - k), (c.0 - k, c.1)];
                for i in 0..4 {
                    pen.line(diamond[i], diamond[(i + 1) % 4], color);
                }
                Vec::new()
            }
            ShapeGeom::Polyline(v) => v.iter().map(|p| mf.project(*p)).collect(),
            ShapeGeom::Ring(v) => v.iter().chain(v.first()).map(|p| mf.project(*p)).collect(),
            ShapeGeom::Circle { center, radius_ft } => (0..=48)
                .map(|i| mf.project(*center + heading_to_vector(i as f64 * 7.5, *radius_ft)))
                .collect(),
        };
        for w in pts.windows(2) {
            seg(pen, w[0], w[1]);
        }
        let near = mf.project(sh.geom.nearest_point(pos));
        if sh.role == ShapeRole::Target {
            pen.dashed((0.0, 0.0), near, pal.lock_dim, 1.0 * d, 2.0 * d, 0.0);
        }
        if sh.label.is_empty() {
            continue;
        }
        let dist = (near.0 * near.0 + near.1 * near.1).sqrt();
        let at = if dist < r * 0.9 {
            (near.0, near.1 + 2.0 * pen.dot_y())
        } else {
            let k = r * 0.8 / dist.max(1.0);
            (near.0 * k, near.1 * k)
        };
        pen.text_low(at.0, at.1, &sh.label, bold(color));
    }
}

fn draw_scope(
    buf: &mut Buffer,
    area: Rect,
    v: &ViewCtx,
    kind: ViewKind,
) -> (Line<'static>, Option<Line<'static>>) {
    let pal = v.pal;
    let s = &v.snap.state;
    let sp = v.sp;
    let ov = &sp.overlay;
    let fr = v.frame;
    let heading = s.heading_deg;
    let mf = MapFrame::HeadingUp { center: s.position_ft, heading_deg: heading };
    let r = scope_range_ft(kind, v.snap, sp);
    let b = Bounds::centered(0.0, 0.0, r * 1.1, r * 1.1).fit_aspect(area.width, area.height);
    let trail: Vec<P> = v.trail.iter().map(|t| mf.project(t.pos)).collect();
    let rw = &sp.runway;
    let rframe = RunwayFrame::new(rw.clone());
    let rwc = |x: f64, y: f64| mf.project(rframe.to_world_frame(Vec2::new(x, y)));
    let guidance = v.snap.last_guidance.as_ref();
    let target_path = guidance.and_then(|g| g.target_path).map(|l| (mf.project(l.start_ft), mf.project(l.end_ft)));
    let waypoint = guidance.and_then(|g| g.target_waypoint.clone());
    let bug = guidance.and_then(|g| g.target_track_deg.or(g.target_heading_deg));
    let compact = area.width < 34;
    let sweep = sweep_angle_deg(fr);
    let pose_state = ov.target_pose.map(|p| {
        let dist = s.position_ft.distance_to(p.position_ft);
        let herr = wrap_degrees_180(p.heading_deg - s.heading_deg);
        (p, dist, pose_error_norm(dist, herr))
    });

    render_canvas(buf, area, b, |pen| {
        let d = pen.dot();
        let dy = pen.dot_y();
        // Layer 0: radar beam with afterglow.
        for i in 0..4 {
            let a = (sweep - i as f64 * 5.0).to_radians();
            let c = pal.sweep[i];
            pen.raw((0.0, 0.0), (r * a.sin(), r * a.cos()), c);
        }
        pen.layer();
        // Layer 1: rings, compass rose.
        pen.ring(r, pal.grid, false);
        pen.ring(r * 2.0 / 3.0, pal.grid_dim, true);
        pen.ring(r / 3.0, pal.grid_dim, true);
        for deg in (0..360).step_by(10) {
            let a = (deg as f64 - heading).to_radians();
            let major = deg % 30 == 0;
            let inner = if major { r * 0.93 } else { r * 0.965 };
            pen.raw((inner * a.sin(), inner * a.cos()), (r * a.sin(), r * a.cos()), pal.grid);
            if major && (!compact || deg % 90 == 0) {
                let lbl = match deg {
                    0 => "N".to_string(),
                    90 => "E".to_string(),
                    180 => "S".to_string(),
                    270 => "W".to_string(),
                    _ => (deg / 10).to_string(),
                };
                let lr = r * 0.85;
                let c = if deg == 0 { pal.text } else { pal.text_dim };
                pen.text_low(lr * a.sin(), lr * a.cos(), &lbl, fg(c));
            }
        }
        pen.text_low(r * 0.8, -r * 0.98, &fmt_scope_range(r, kind), fg(pal.text_dim));
        pen.layer();

        // Layer 2: context + guidance, clipped to the scope face.
        pen.clip_r = Some(r);
        // Runway.
        let hw = rw.width_ft * 0.5;
        let corners = [rwc(0.0, -hw), rwc(rw.length_ft, -hw), rwc(rw.length_ft, hw), rwc(0.0, hw)];
        let rwy_glow = afterglow(sweep, vector_to_heading(Vec2::new(corners[0].0, corners[0].1)));
        let rwy_c = if kind == ViewKind::Scope && rwy_glow < 0.35 { pal.runway_dim } else { pal.runway };
        for i in 0..4 {
            pen.line(corners[i], corners[(i + 1) % 4], rwy_c);
        }
        if kind != ViewKind::Taxi {
            pen.dashed(rwc(-3.0 * FT_PER_NM, 0.0), rwc(0.0, 0.0), pal.grid, 3.0 * d, 3.0 * d, 0.0);
        }
        let rwy_lbl = rwc(-(4.0 * d).max(250.0), 0.0);
        if kind != ViewKind::Scope && (rwy_lbl.0.powi(2) + rwy_lbl.1.powi(2)).sqrt() < r * 0.9 {
            pen.text_c(rwy_lbl.0, rwy_lbl.1, &runway_label(rw), fg(pal.runway_dim));
        }

        match kind {
            ViewKind::Pattern => {
                let mut active_seg = None;
                for (i, l) in ov.pattern_legs.iter().enumerate() {
                    let (a, bb) = leg_proj(&mf, l);
                    if Some(i) == ov.active_pattern_leg {
                        active_seg = Some((a, bb));
                        pen.marching(a, bb, pal.guide_dim, pal.guide, fr);
                    } else {
                        pen.line(a, bb, pal.grid);
                    }
                    let m = ((a.0 + bb.0) * 0.5, (a.1 + bb.1) * 0.5);
                    if (m.0 * m.0 + m.1 * m.1).sqrt() < r * 0.85 {
                        let c = if Some(i) == ov.active_pattern_leg { pal.guide } else { pal.text_dim };
                        pen.text_c(m.0, m.1 + 2.0 * dy, &l.label, fg(c));
                    }
                }
                if let Some((a, bb)) = target_path {
                    let same = active_seg.is_some_and(|(x, y)| {
                        (x.0 - a.0).abs() + (x.1 - a.1).abs() + (y.0 - bb.0).abs() + (y.1 - bb.1).abs() < 1.0
                    });
                    if !same {
                        pen.marching(a, bb, pal.guide_dim, pal.guide, fr);
                    }
                }
                for l in &ov.route {
                    let (a, bb) = leg_proj(&mf, l);
                    pen.line(a, bb, pal.guide_dim);
                }
            }
            ViewKind::Taxi => {
                let ai = ov.route_active_idx.unwrap_or(0);
                let mut last_lbl = "";
                for (i, l) in ov.route.iter().enumerate() {
                    let (a, bb) = leg_proj(&mf, l);
                    if i < ai {
                        pen.dashed(a, bb, pal.grid, 1.5 * d, 1.5 * d, 0.0);
                    } else if i == ai {
                        pen.marching(a, bb, pal.guide_dim, pal.guide, fr);
                    } else {
                        pen.marching(a, bb, pal.guide_dim, pal.guide_dim, fr);
                    }
                    if !l.label.is_empty() && l.label != last_lbl && i >= ai {
                        let m = ((a.0 + bb.0) * 0.5, (a.1 + bb.1) * 0.5);
                        if (m.0 * m.0 + m.1 * m.1).sqrt() < r * 0.9 {
                            pen.text_c(m.0 + 2.0 * d, m.1, &l.label, bold(pal.guide));
                        }
                    }
                    last_lbl = &l.label;
                }
            }
            _ => {
                if let Some((a, bb)) = target_path {
                    pen.marching(a, bb, pal.guide_dim, pal.guide, fr);
                }
            }
        }
        draw_plan(pen, &mf, ov, &pal, fr, r, s.position_ft);
        pen.trail(&trail, &pal);
        pen.clip_r = None;
        if let Some(w) = ov.plan_waiting.first() {
            let (x, y) = (pen.b.x[0], pen.b.y[1]);
            pen.text(x, y, &w.to_uppercase(), bold(pal.caution));
        }
        pen.layer();

        // Layer 3: targets, bugs, own ship.
        match kind {
            ViewKind::Pattern => {
                let xs = corners.iter().map(|c| c.0);
                let ys = corners.iter().map(|c| c.1);
                let (x0, x1) = xs.fold((f64::MAX, f64::MIN), |(lo, hi), x| (lo.min(x), hi.max(x)));
                let (y0, y1) = ys.fold((f64::MAX, f64::MIN), |(lo, hi), y| (lo.min(y), hi.max(y)));
                let c = ((x0 + x1) * 0.5, (y0 + y1) * 0.5);
                if (c.0 * c.0 + c.1 * c.1).sqrt() < r {
                    let grow = 1.0 + 0.25 * pulse(fr, 12);
                    let hw = ((x1 - x0) * 0.5 + 2.0 * d) * grow;
                    let hh = ((y1 - y0) * 0.5 + 2.0 * d) * grow;
                    let col = if pulse(fr, 12) > 0.5 { pal.lock } else { pal.lock_dim };
                    pen.brackets(c, hw, hh, col);
                }
            }
            ViewKind::Taxi => {
                if let Some((p, dist, err)) = pose_state {
                    let at = mf.project(p.position_ft);
                    let dir = mf.dir(p.heading_deg);
                    let perp = (dir.1, -dir.0);
                    let bar = (20.0_f64).max(3.0 * d);
                    let lock = if err < 0.5 { Lock::Locked } else { Lock::Tracking };
                    let c = pal.lock_color(lock, fr);
                    pen.raw((at.0 - perp.0 * bar, at.1 - perp.1 * bar), (at.0 + perp.0 * bar, at.1 + perp.1 * bar), pal.lock);
                    let tip = (at.0 + dir.0 * bar * 1.4, at.1 + dir.1 * bar * 1.4);
                    pen.raw(at, tip, pal.lock);
                    pen.chevron(tip, dir, bar * 0.4, pal.lock);
                    let spread = bracket_spread(err.min(1.9), fr);
                    let sz = (bar * 1.3) * (1.0 + spread);
                    pen.brackets(at, sz, sz, c);
                    let tag = ov.target_label.clone().unwrap_or_else(|| "POSE".to_string());
                    let txt = if lock == Lock::Locked { format!("{tag} LOCK") } else { format!("{tag} {dist:.0}FT") };
                    pen.text(at.0 + sz + d, at.1 + sz, &txt, bold(c));
                }
            }
            _ => {
                let rd = (rwc(0.0, 0.0).0.powi(2) + rwc(0.0, 0.0).1.powi(2)).sqrt();
                if rd > r * 0.95 {
                    let p = rwc(0.0, 0.0);
                    let a = p.0.atan2(p.1);
                    let edge = (r * 0.97 * a.sin(), r * 0.97 * a.cos());
                    let dir = (a.sin(), a.cos());
                    pen.chevron(edge, dir, 2.0 * d, pal.guide);
                    let li = (r * 0.75 * a.sin(), r * 0.75 * a.cos());
                    pen.text_c(li.0, li.1, &format!("{} {:.0}NM", runway_label(rw), rd / FT_PER_NM), fg(pal.guide));
                } else {
                    let p = rwc(rw.length_ft * 0.5, 0.0);
                    let sz = 2.5 * d;
                    let c = if rwy_glow > 0.6 { pal.lock } else { pal.lock_dim };
                    pen.brackets(p, sz, sz, c);
                    pen.text(p.0 + sz + d, p.1, &format!("{} {:.1}NM", runway_label(rw), rd / FT_PER_NM), fg(pal.text_dim));
                }
                if let Some(w) = &waypoint {
                    let p = mf.project(w.position_ft);
                    if (p.0 * p.0 + p.1 * p.1).sqrt() < r {
                        let sz = 2.0 * d;
                        pen.raw((p.0 - sz, p.1), (p.0, p.1 + sz), pal.guide);
                        pen.raw((p.0, p.1 + sz), (p.0 + sz, p.1), pal.guide);
                        pen.raw((p.0 + sz, p.1), (p.0, p.1 - sz), pal.guide);
                        pen.raw((p.0, p.1 - sz), (p.0 - sz, p.1), pal.guide);
                        pen.text(p.0 + sz + d, p.1, &w.name.to_ascii_uppercase(), fg(pal.guide));
                    }
                }
            }
        }
        // Heading / track bug on the ring.
        if let Some(t) = bug {
            let a = (t - heading).to_radians();
            let tip = (r * 0.99 * a.sin(), r * 0.99 * a.cos());
            pen.chevron(tip, (a.sin(), a.cos()), 2.0 * d, pal.guide);
        }
        // Lubber line + predicted track.
        pen.raw((0.0, r), (-1.5 * d, r + 2.5 * dy), pal.ship);
        pen.raw((0.0, r), (1.5 * d, r + 2.5 * dy), pal.ship);
        if s.gs_kt > 2.0 {
            let a = (s.track_deg - heading).to_radians();
            let lead = (s.gs_kt * KT_TO_FPS * 60.0).min(r);
            pen.dashed((0.0, 0.0), (lead * a.sin(), lead * a.cos()), pal.trail[1], 1.5 * d, 1.5 * d, 0.0);
        }
        let ship_size = if kind == ViewKind::Taxi { (3.0 * d).max(18.0) } else { 3.0 * d };
        pen.ownship((0.0, 0.0), (0.0, 1.0), ship_size, pal.ship);
        pen.text_c(0.0, b.y[1], &format!("{:03.0}", wrap_degrees_360(heading)), bold(pal.ship));
        pen.flush_deferred();
    });

    // Readouts.
    let g = guidance;
    let mut spans = Vec::new();
    match kind {
        ViewKind::Taxi => {
            let owner = v
                .snap
                .lateral_owner_idx
                .and_then(|i| v.snap.active_profiles.get(i).map(|n| (i, n)));
            let name = owner.map(|(_, n)| n.replace('_', " ").to_ascii_uppercase()).unwrap_or_else(|| "TAXI".to_string());
            kv(&mut spans, &pal, "", name, bold(pal.guide));
            kv(&mut spans, &pal, "GS", with_target(s.gs_kt, g.and_then(|g| g.target_speed_kt)), bold(pal.text));
            if let Some((_, dist, _)) = pose_state {
                kv(&mut spans, &pal, "TGT", format!("{dist:.0}FT"), fg(pal.text));
            }
            if let Some(suffix) = owner.and_then(|(i, _)| v.snap.profile_mode_line_suffixes.get(i).cloned().flatten()) {
                spans.push(Span::raw("  "));
                spans.push(Span::styled(suffix.to_ascii_uppercase(), fg(pal.text_dim)));
            }
        }
        _ => {
            kv(&mut spans, &pal, "", phase_tag(v.snap), bold(pal.guide));
            kv(&mut spans, &pal, "ALT", with_target(s.alt_msl_ft, g.and_then(|g| g.target_altitude_ft)), bold(pal.text));
            kv(&mut spans, &pal, "IAS", with_target(s.ias_kt, g.and_then(|g| g.target_speed_kt)), fg(pal.text));
            kv(&mut spans, &pal, "HDG", with_target(wrap_degrees_360(s.heading_deg), bug.map(wrap_degrees_360)), fg(pal.text));
            kv(&mut spans, &pal, "VS", format!("{:+.0}", s.vs_fpm), fg(pal.text));
            if let Some(h) = &v.snap.transition_hint {
                let eta = h.eta_s.map(|e| format!(" {}:{:02}", (e / 60.0) as u32, (e % 60.0) as u32)).unwrap_or_default();
                kv(&mut spans, &pal, "NEXT", format!("{}{eta}", h.next_phase.value().replace('_', " ").to_ascii_uppercase()), fg(pal.guide));
            }
        }
    }
    let status = Line::from(Span::styled(format!(" RNG {} ", fmt_scope_range(r, kind)), fg(pal.text_dim)));
    (Line::from(spans), Some(status))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::spatial::{PlanShape, ShapeGeom, ShapeRole};
    use crate::core::spatial::SpatialOverlay;
    use crate::guidance::pattern_manager::build_pattern_geometry;
    use crate::types::{Glidepath, GuidanceTargets, StraightLeg, TaxiPose, TrafficSide};
    use ratatui::backend::TestBackend;
    use ratatui::Terminal;

    fn runway() -> Runway {
        Runway {
            id: Some("31".to_string()),
            threshold_ft: Vec2::new(1000.0, -2000.0),
            course_deg: 310.0,
            length_ft: 5000.0,
            touchdown_zone_ft: 2000.0,
            displaced_threshold_ft: 0.0,
            width_ft: 100.0,
            traffic_side: TrafficSide::Left,
        }
    }

    fn spatial(overlay: SpatialOverlay) -> SpatialSnapshot {
        SpatialSnapshot {
            runway: runway(),
            field_elevation_ft: 500.0,
            roundout_height_ft: 30.0,
            flare_start_ft: 10.0,
            runway_clear_margin_ft: 20.0,
            overlay,
        }
    }

    fn pattern_overlay(active: Option<usize>) -> SpatialOverlay {
        let p = build_pattern_geometry(&RunwayFrame::new(runway()), 4000.0, 0.0, 0.0);
        SpatialOverlay {
            pattern_legs: vec![
                LabeledLeg::new("ENTRY", p.entry_leg),
                LabeledLeg::new("DWN", p.downwind_leg),
                LabeledLeg::new("BASE", p.base_leg),
                LabeledLeg::new("FINAL", p.final_leg),
            ],
            active_pattern_leg: active,
            ..Default::default()
        }
    }

    fn snap_at(phase: Option<FlightPhase>, rx: f64, ry: f64, agl: f64, on_ground: bool) -> StatusSnapshot {
        let f = RunwayFrame::new(runway());
        let mut s = StatusSnapshot::synthetic_default();
        s.phase = phase;
        s.state.position_ft = f.to_world_frame(Vec2::new(rx, ry));
        s.state.alt_agl_ft = agl;
        s.state.alt_msl_ft = agl + 500.0;
        s.state.heading_deg = 310.0;
        s.state.track_deg = 310.0;
        s.state.gs_kt = if on_ground { 15.0 } else { 70.0 };
        s.state.vs_fpm = if on_ground { 0.0 } else { -370.0 };
        s.state.on_ground = on_ground;
        s
    }

    fn render(widget: SpatialWidget, w: u16, h: u16) -> Buffer {
        let mut term = Terminal::new(TestBackend::new(w, h)).unwrap();
        term.draw(|f| f.render_widget(widget, f.area())).unwrap();
        term.backend().buffer().clone()
    }

    fn dump(buf: &Buffer) {
        if std::env::var_os("SPATIAL_DUMP").is_some() {
            println!("{}", buffer_text(buf));
        }
    }

    fn buffer_text(buf: &Buffer) -> String {
        let mut out = String::new();
        for y in 0..buf.area.height {
            for x in 0..buf.area.width {
                out.push_str(buf[(x, y)].symbol());
            }
            out.push('\n');
        }
        out
    }

    fn trail_along(f: &RunwayFrame, xs: &[(f64, f64, f64)]) -> Trail {
        let mut t = Trail::default();
        for (i, (x, y, h)) in xs.iter().enumerate() {
            t.push(i as f64, f.to_world_frame(Vec2::new(*x, *y)), *h);
        }
        t
    }

    // ---- view selection ----

    #[test]
    fn select_view_by_phase() {
        let sp = spatial(pattern_overlay(Some(1)));
        let cases = [
            (FlightPhase::Final, ViewKind::Approach),
            (FlightPhase::Roundout, ViewKind::Approach),
            (FlightPhase::Flare, ViewKind::Approach),
            (FlightPhase::Downwind, ViewKind::Pattern),
            (FlightPhase::Base, ViewKind::Pattern),
            (FlightPhase::GoAround, ViewKind::Pattern),
            (FlightPhase::Crosswind, ViewKind::Pattern),
            (FlightPhase::Rollout, ViewKind::Runway),
            (FlightPhase::RunwayExit, ViewKind::Runway),
            (FlightPhase::TakeoffRoll, ViewKind::Runway),
        ];
        for (phase, want) in cases {
            let s = snap_at(Some(phase), -3000.0, 0.0, 500.0, false);
            assert_eq!(select_view(&s, &sp), want, "{phase:?}");
        }
    }

    #[test]
    fn select_view_taxi_and_fallbacks() {
        let taxi = spatial(SpatialOverlay {
            route: vec![LabeledLeg::new("A", StraightLeg { start_ft: Vec2::ZERO, end_ft: Vec2::new(0.0, 500.0) })],
            route_active_idx: Some(0),
            ..Default::default()
        });
        let s = snap_at(None, -800.0, -600.0, 0.0, true);
        assert_eq!(select_view(&s, &taxi), ViewKind::Taxi);
        let empty = spatial(SpatialOverlay::default());
        assert_eq!(select_view(&s, &empty), ViewKind::Scope);
        let cruise = snap_at(Some(FlightPhase::Cruise), -60000.0, 0.0, 4000.0, false);
        assert_eq!(select_view(&cruise, &spatial(pattern_overlay(None))), ViewKind::Scope);
        let near = snap_at(Some(FlightPhase::Descent), -6000.0, 3000.0, 1500.0, false);
        assert_eq!(select_view(&near, &spatial(pattern_overlay(None))), ViewKind::Pattern);
        let lined_up = snap_at(Some(FlightPhase::Preflight), 10.0, 0.0, 0.0, true);
        assert_eq!(select_view(&lined_up, &empty), ViewKind::Runway);
    }

    #[test]
    fn parse_view_command_variants() {
        assert_eq!(parse_view_command(""), Ok(ViewCommand::Toggle));
        assert_eq!(parse_view_command(" ON "), Ok(ViewCommand::On));
        assert_eq!(parse_view_command("off"), Ok(ViewCommand::Off));
        assert_eq!(parse_view_command("auto"), Ok(ViewCommand::Mode(ViewMode::Auto)));
        assert_eq!(
            parse_view_command("final"),
            Ok(ViewCommand::Mode(ViewMode::Pinned(ViewKind::Approach)))
        );
        assert_eq!(
            parse_view_command("radar"),
            Ok(ViewCommand::Mode(ViewMode::Pinned(ViewKind::Scope)))
        );
        assert!(parse_view_command("banana").is_err());
    }

    // ---- geometry ----

    #[test]
    fn fit_aspect_makes_square_dots_and_only_expands() {
        let b = Bounds { x: [0.0, 100.0], y: [0.0, 100.0] }.fit_aspect(80, 20);
        // 160 × 80 dots → x span must be twice y span.
        assert!((b.width() / b.height() - 2.0).abs() < 1e-9);
        assert!(b.width() >= 100.0 && b.height() >= 100.0);
        assert_eq!(b.center(), (50.0, 50.0));
        let tall = Bounds { x: [0.0, 400.0], y: [0.0, 10.0] }.fit_aspect(80, 20);
        assert!((tall.width() - 400.0).abs() < 1e-9);
        assert!((tall.height() - 200.0).abs() < 1e-9);
    }

    #[test]
    fn clip_segment_cases() {
        let b = Bounds { x: [0.0, 10.0], y: [0.0, 10.0] };
        assert_eq!(clip_segment((1.0, 1.0), (2.0, 2.0), &b), Some(((1.0, 1.0), (2.0, 2.0))));
        let (p, q) = clip_segment((-5.0, 5.0), (15.0, 5.0), &b).unwrap();
        assert_eq!(p, (0.0, 5.0));
        assert_eq!(q, (10.0, 5.0));
        assert_eq!(clip_segment((-5.0, -5.0), (-1.0, 20.0), &b), None);
        assert_eq!(clip_segment((11.0, 0.0), (11.0, 10.0), &b), None);
    }

    #[test]
    fn clip_segment_circle_cases() {
        let (p, q) = clip_segment_circle((-20.0, 0.0), (20.0, 0.0), 10.0).unwrap();
        assert!((p.0 + 10.0).abs() < 1e-9 && (q.0 - 10.0).abs() < 1e-9);
        assert_eq!(clip_segment_circle((-20.0, 15.0), (20.0, 15.0), 10.0), None);
        assert_eq!(clip_segment_circle((1.0, 1.0), (2.0, 2.0), 10.0), Some(((1.0, 1.0), (2.0, 2.0))));
    }

    #[test]
    fn pick_range_ladder() {
        assert_eq!(pick_range(0.2, &SCOPE_RANGES_NM), 0.5);
        assert_eq!(pick_range(2.5, &SCOPE_RANGES_NM), 3.0);
        assert_eq!(pick_range(500.0, &SCOPE_RANGES_NM), 160.0);
        assert_eq!(pick_range(700.0, &TAXI_RANGES_FT), 1200.0);
    }

    #[test]
    fn heading_up_projection() {
        let mf = MapFrame::HeadingUp { center: Vec2::new(100.0, 100.0), heading_deg: 90.0 };
        // Due east of own ship while heading east → straight up.
        let p = mf.project(Vec2::new(200.0, 100.0));
        assert!(p.0.abs() < 1e-9 && (p.1 - 100.0).abs() < 1e-9);
        // Due south while heading east → to the right.
        let q = mf.project(Vec2::new(100.0, 0.0));
        assert!((q.0 - 100.0).abs() < 1e-9 && q.1.abs() < 1e-9);
        let d = mf.dir(0.0);
        assert!((d.0 + 1.0).abs() < 1e-9 && d.1.abs() < 1e-9, "north points left");
    }

    #[test]
    fn runway_up_projection() {
        let rw = runway();
        let mf = MapFrame::RunwayUp { origin: rw.threshold_ft, course_deg: rw.course_deg };
        let f = RunwayFrame::new(rw);
        let p = mf.project(f.to_world_frame(Vec2::new(1000.0, 50.0)));
        assert!((p.0 - 1000.0).abs() < 1e-6, "{p:?}");
        assert!((p.1 + 50.0).abs() < 1e-6, "right of centerline draws below");
    }

    #[test]
    fn glidepath_height_math() {
        let f = RunwayFrame::new(runway());
        let aim = f.touchdown_runway_x_ft();
        assert!(glidepath_height_agl_ft(&f, aim, 3.0).abs() < 1e-9);
        let h = glidepath_height_agl_ft(&f, aim - FT_PER_NM, 3.0);
        assert!((h - FT_PER_NM * 3.0_f64.to_radians()).abs() < 1.0, "≈318 ft/NM, got {h}");
        assert_eq!(glidepath_height_agl_ft(&f, aim + 500.0, 3.0), 0.0);
    }

    #[test]
    fn approach_camera_centres_aim_on_glidepath() {
        let f = RunwayFrame::new(runway());
        let aim = f.touchdown_runway_x_ft();
        let x = aim - 3000.0;
        let h = glidepath_height_agl_ft(&f, x, 3.0);
        let cam = ApproachCamera { eye: [x, 0.0, h], pitch_down_deg: 3.0, focal: 200.0 };
        let p = cam.project([aim, 0.0, 0.0]).unwrap();
        assert!(p.0.abs() < 1e-6 && p.1.abs() < 0.05, "{p:?}");
        // High on the path → aim point drops below centre; right of
        // centerline → aim point appears to the left.
        let high = ApproachCamera { eye: [x, 40.0, h + 100.0], ..cam };
        let q = high.project([aim, 0.0, 0.0]).unwrap();
        assert!(q.1 < 0.0 && q.0 < 0.0, "{q:?}");
        // Behind the camera.
        assert!(cam.project([x - 100.0, 0.0, 0.0]).is_none());
        // Segment straddling the near plane is clipped, not dropped.
        assert!(cam.project_segment([x - 500.0, 10.0, 0.0], [x + 500.0, 10.0, 0.0]).is_some());
        // Flying down the glidepath: FPM on the boresight.
        let t = 3.0_f64.to_radians();
        let fpm = cam.flight_path_marker([t.cos(), 0.0, -t.sin()]).unwrap();
        assert!(fpm.0.abs() < 1e-9 && fpm.1.abs() < 1e-9);
    }

    #[test]
    fn runway_window_covers_aircraft_and_exit() {
        let (lo, hi) = runway_window(2000.0, &[3200.0, 3300.0], 5000.0);
        assert!(lo <= 1400.0 && hi >= 3900.0);
        let (lo, hi) = runway_window(0.0, &[], 5000.0);
        assert!(lo <= -200.0 && hi >= 1500.0);
        let (lo, hi) = runway_window(4950.0, &[], 5000.0);
        assert!(hi - lo >= 1500.0 - 1e-9 && lo < 4950.0 && hi > 4950.0);
    }

    // ---- lock + animation ----

    #[test]
    fn lock_tightens_with_error() {
        assert_eq!(lock_from_error(approach_error_norm(5.0, 3.0, 6000.0)), Lock::Locked);
        assert_eq!(lock_from_error(approach_error_norm(50.0, 0.0, 6000.0)), Lock::Tracking);
        assert_eq!(lock_from_error(approach_error_norm(200.0, 0.0, 6000.0)), Lock::Lost);
        assert_eq!(lock_from_error(approach_error_norm(0.0, 400.0, 6000.0)), Lock::Lost);
        // Same deviation is worse closer in, down to the 400 ft floor.
        assert!(approach_error_norm(20.0, 0.0, 1000.0) > approach_error_norm(20.0, 0.0, 6000.0));
        assert_eq!(approach_error_norm(20.0, 0.0, 10.0), approach_error_norm(20.0, 0.0, 400.0));
        for f in 0..40 {
            let locked = bracket_spread(0.1, f);
            let tracking = bracket_spread(0.7, f);
            let lost = bracket_spread(1.6, f);
            assert!(locked < tracking && tracking < lost, "frame {f}");
        }
        assert_eq!(lock_from_error(pose_error_norm(5.0, 3.0)), Lock::Locked);
        assert_eq!(lock_from_error(pose_error_norm(200.0, 3.0)), Lock::Lost);
    }

    #[test]
    fn animation_primitives_are_periodic_and_bounded() {
        assert!(blink(0, 6, 4) && blink(3, 6, 4) && !blink(4, 6, 4) && blink(6, 6, 4));
        assert_eq!(pulse(0, 10), 0.0);
        assert_eq!(pulse(5, 10), 1.0);
        assert_eq!(pulse(13, 10), pulse(3, 10));
        for f in 0..500 {
            let j = jitter(f, 3);
            assert!((-1.0..=1.0).contains(&j));
            assert_eq!(j, jitter(f, 3));
        }
        assert_eq!(sweep_angle_deg(0), 0.0);
        assert_eq!(sweep_angle_deg(15), 90.0);
        assert_eq!(sweep_angle_deg(60), 0.0);
        assert_eq!(afterglow(90.0, 90.0), 1.0);
        assert!(afterglow(90.0, 45.0) > afterglow(90.0, 0.0));
        assert_eq!(afterglow(90.0, 100.0), 0.0, "ahead of the beam is dark");
    }

    #[test]
    fn march_offsets_advance_and_wrap() {
        let a = march_offsets(100.0, 20.0, 0, 3.0);
        assert_eq!(a, vec![0.0, 20.0, 40.0, 60.0, 80.0]);
        let b = march_offsets(100.0, 20.0, 2, 3.0);
        assert_eq!(b[0], 6.0);
        let c = march_offsets(100.0, 20.0, 7, 3.0);
        assert!((c[0] - 1.0).abs() < 1e-9, "21 wraps to 1");
        assert!(march_offsets(0.0, 20.0, 0, 1.0).is_empty());
        assert!(march_offsets(10.0, 0.0, 0, 1.0).is_empty());
    }

    #[test]
    fn trail_samples_dedups_and_resets() {
        let mut t = Trail::default();
        t.push(0.0, Vec2::ZERO, 0.0);
        t.push(0.1, Vec2::new(50.0, 0.0), 0.0); // too soon
        t.push(1.0, Vec2::new(1.0, 0.0), 0.0); // too little movement
        t.push(1.0, Vec2::new(60.0, 0.0), 0.0);
        assert_eq!(t.len(), 2);
        t.push(2.0, Vec2::new(10_000.0, 0.0), 0.0); // teleport
        assert_eq!(t.len(), 1);
        t.push(0.0, Vec2::ZERO, 0.0); // sim reset
        assert_eq!(t.len(), 1);
        for i in 0..(Trail::MAX_POINTS + 50) {
            t.push(1.0 + i as f64, Vec2::new(10.0 * i as f64, 0.0), 0.0);
        }
        assert_eq!(t.len(), Trail::MAX_POINTS);
    }

    // ---- rendering ----

    fn final_fixture() -> (StatusSnapshot, SpatialSnapshot, Trail) {
        let f = RunwayFrame::new(runway());
        let aim = f.touchdown_runway_x_ft();
        let x = aim - 4000.0;
        let gp = glidepath_height_agl_ft(&f, x, 3.0);
        let mut s = snap_at(Some(FlightPhase::Final), x, 15.0, gp + 12.0, false);
        s.last_guidance = Some(GuidanceTargets {
            glidepath: Some(Glidepath {
                slope_deg: 3.0,
                threshold_crossing_height_ft: 0.0,
                aimpoint_ft_from_threshold: aim,
            }),
            target_speed_kt: Some(65.0),
            ..Default::default()
        });
        let pts: Vec<(f64, f64, f64)> = (0..12)
            .map(|i| {
                let xi = x - 2400.0 + i as f64 * 200.0;
                (xi, 15.0, glidepath_height_agl_ft(&f, xi, 3.0) + 30.0 - 1.5 * i as f64)
            })
            .collect();
        let trail = trail_along(&f, &pts);
        (s, spatial(pattern_overlay(Some(3))), trail)
    }

    fn widget<'a>(s: &'a StatusSnapshot, sp: &'a SpatialSnapshot, t: &'a Trail, frame: u64) -> SpatialWidget<'a> {
        SpatialWidget {
            snapshot: Some(s),
            spatial: Some(sp),
            trail: t,
            mode: ViewMode::Auto,
            frame,
            palette: Palette::color(),
        }
    }

    #[test]
    fn renders_final_approach_frames() {
        let (s, sp, t) = final_fixture();
        for frame in [0, 7, 33] {
            let buf = render(widget(&s, &sp, &t, frame), 72, 26);
            let text = buffer_text(&buf);
            assert!(text.contains("APPROACH"), "{text}");
            assert!(text.contains("RNG"));
            assert!(text.contains("PROFILE"));
            assert!(text.contains("TGT LOCK") || text.contains("TRACKING"), "{text}");
        }
        if std::env::var_os("SPATIAL_DUMP").is_some() {
            println!("{}", buffer_text(&render(widget(&s, &sp, &t, 7), 72, 26)));
        }
    }

    #[test]
    fn renders_pattern_downwind() {
        let f = RunwayFrame::new(runway());
        let mut s = snap_at(Some(FlightPhase::Downwind), 2500.0, -4000.0, 1000.0, false);
        s.state.heading_deg = 130.0;
        s.state.track_deg = 128.0;
        let sp = spatial(pattern_overlay(Some(1)));
        s.last_guidance = Some(GuidanceTargets {
            target_path: Some(sp.overlay.pattern_legs[1].leg),
            target_altitude_ft: Some(1500.0),
            target_speed_kt: Some(80.0),
            ..Default::default()
        });
        let t = trail_along(&f, &[(4000.0, -3500.0, 950.0), (3500.0, -3900.0, 980.0), (3000.0, -4000.0, 1000.0)]);
        for frame in [0, 21] {
            let text = buffer_text(&render(widget(&s, &sp, &t, frame), 64, 24));
            assert!(text.contains("PATTERN") && text.contains("DOWNWIND"), "{text}");
        }
        if std::env::var_os("SPATIAL_DUMP").is_some() {
            println!("{}", buffer_text(&render(widget(&s, &sp, &t, 9), 64, 24)));
        }
    }

    #[test]
    fn renders_rollout_with_exit() {
        let f = RunwayFrame::new(runway());
        let s = snap_at(Some(FlightPhase::Rollout), 2400.0, 2.0, 0.0, true);
        let path = [Vec2::new(3200.0, 0.0), Vec2::new(3300.0, 80.0), Vec2::new(3320.0, 250.0)];
        let route = path
            .windows(2)
            .map(|w| LabeledLeg::new("B", StraightLeg { start_ft: f.to_world_frame(w[0]), end_ft: f.to_world_frame(w[1]) }))
            .collect();
        let sp = spatial(SpatialOverlay {
            route,
            route_active_idx: None,
            target_label: Some("B".into()),
            ..Default::default()
        });
        let t = trail_along(&f, &[(1200.0, 0.0, 0.0), (1800.0, 1.0, 0.0), (2300.0, 2.0, 0.0)]);
        for frame in [0, 19] {
            let text = buffer_text(&render(widget(&s, &sp, &t, frame), 70, 18));
            assert!(text.contains("RUNWAY") && text.contains("EXIT"), "{text}");
            assert!(text.contains("CLR B"), "{text}");
        }
        dump(&render(widget(&s, &sp, &t, 12), 70, 18));
    }

    #[test]
    fn renders_taxi_with_pose() {
        let f = RunwayFrame::new(runway());
        let a = f.to_world_frame(Vec2::new(-200.0, -900.0));
        let b = f.to_world_frame(Vec2::new(-200.0, -300.0));
        let c = f.to_world_frame(Vec2::new(-60.0, -150.0));
        let mut s = snap_at(None, -200.0, -700.0, 0.0, true);
        s.state.heading_deg = 40.0;
        s.active_profiles = vec!["taxi".into()];
        s.lateral_owner_idx = Some(0);
        s.profile_mode_line_suffixes = vec![Some("via A · leg 1/2 · 400ft".into())];
        let sp = spatial(SpatialOverlay {
            route: vec![
                LabeledLeg::new("A", StraightLeg { start_ft: a, end_ft: b }),
                LabeledLeg::new("", StraightLeg { start_ft: b, end_ft: c }),
            ],
            route_active_idx: Some(0),
            target_pose: Some(TaxiPose { position_ft: c, heading_deg: 310.0 }),
            target_label: Some("HOLD SHORT".into()),
            ..Default::default()
        });
        let t = trail_along(&f, &[(-200.0, -950.0, 0.0), (-200.0, -820.0, 0.0)]);
        for frame in [0, 5, 11] {
            let text = buffer_text(&render(widget(&s, &sp, &t, frame), 60, 22));
            assert!(text.contains("TAXI") && text.contains("HOLD SHORT"), "{text}");
        }
        dump(&render(widget(&s, &sp, &t, 4), 60, 22));
    }

    #[test]
    fn renders_the_plan_trigger_constraint_and_what_it_waits_for() {
        let f = RunwayFrame::new(runway());
        let mut s = snap_at(Some(FlightPhase::Downwind), -2000.0, 4000.0, 1000.0, false);
        s.state.heading_deg = 130.0;
        s.state.track_deg = 130.0;
        let road = vec![f.to_world_frame(Vec2::new(-7000.0, 9000.0)), f.to_world_frame(Vec2::new(-7000.0, -9000.0))];
        let fence = vec![f.to_world_frame(Vec2::new(3000.0, 9000.0)), f.to_world_frame(Vec2::new(3000.0, -9000.0))];
        let mut ov = pattern_overlay(Some(1));
        ov.plan = vec![
            PlanShape { label: "BASE @ NORDHOFF".into(), role: ShapeRole::Trigger, geom: ShapeGeom::Polyline(road) },
            PlanShape { label: "SOUTH OF US-101".into(), role: ShapeRole::Constraint, geom: ShapeGeom::Polyline(fence) },
            PlanShape {
                label: "360".into(),
                role: ShapeRole::Active,
                geom: ShapeGeom::Circle { center: f.to_world_frame(Vec2::new(-2000.0, 5000.0)), radius_ft: 1000.0 },
            },
        ];
        ov.plan_waiting = vec!["waiting for ATC: base".into()];
        let sp = spatial(ov);
        let t = Trail::default();
        for frame in [0, 7] {
            let text = buffer_text(&render(widget(&s, &sp, &t, frame), 80, 30));
            assert!(text.contains("BASE @ NORDHOFF"), "{text}");
            assert!(text.contains("WAITING FOR ATC: BASE"), "{text}");
        }
        dump(&render(widget(&s, &sp, &t, 3), 80, 30));
    }

    #[test]
    fn renders_scope_and_degenerate_cases() {
        let mut s = snap_at(Some(FlightPhase::Cruise), -60000.0, 20000.0, 5000.0, false);
        s.last_guidance = Some(GuidanceTargets { target_heading_deg: Some(280.0), ..Default::default() });
        let sp = spatial(SpatialOverlay::default());
        let t = Trail::default();
        let text = buffer_text(&render(widget(&s, &sp, &t, 3), 60, 20));
        assert!(text.contains("SCOPE") && text.contains("31"), "{text}");
        dump(&render(widget(&s, &sp, &t, 3), 60, 20));
        // Every view, pinned, at tiny and odd sizes: must not panic.
        let (fs, fsp, ft) = final_fixture();
        for kind in [ViewKind::Approach, ViewKind::Pattern, ViewKind::Runway, ViewKind::Taxi, ViewKind::Scope] {
            for (w, h) in [(10, 4), (26, 7), (30, 12), (120, 40)] {
                let mut wg = widget(&fs, &fsp, &ft, 5);
                wg.mode = ViewMode::Pinned(kind);
                render(wg, w, h);
                let mut wg = widget(&s, &sp, &t, 5);
                wg.mode = ViewMode::Pinned(kind);
                wg.palette = Palette::mono();
                render(wg, w, h);
            }
        }
        // No telemetry yet.
        let text = buffer_text(&render(
            SpatialWidget { snapshot: None, spatial: None, trail: &t, mode: ViewMode::Auto, frame: 0, palette: Palette::color() },
            40,
            8,
        ));
        assert!(text.contains("AWAITING TELEMETRY"));
    }
}
