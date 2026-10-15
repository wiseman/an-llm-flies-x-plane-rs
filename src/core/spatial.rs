//! Read-only geometry the TUI's spatial view draws.
//!
//! Profiles expose what they are flying against (runway, pattern legs,
//! taxi route, exit connector, terminal pose) through
//! `GuidanceProfile::spatial_overlay`. `PilotCore::spatial_snapshot`
//! folds the first overlay it finds together with the pilot's reference
//! runway and flare heights. It is built on demand (only while the view
//! is on), so the control loop and headless runs never pay for it.

use crate::types::{Runway, StraightLeg, TaxiPose};

/// A straight segment with an optional display label (pattern leg name,
/// taxiway name). World-frame feet.
#[derive(Debug, Clone, PartialEq)]
pub struct LabeledLeg {
    pub label: String,
    pub leg: StraightLeg,
}

impl LabeledLeg {
    pub fn new(label: impl Into<String>, leg: StraightLeg) -> Self {
        Self { label: label.into(), leg }
    }
}

/// What one profile contributes to the spatial view. Every field is
/// optional / empty by default.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SpatialOverlay {
    /// Runway the profile is flying against (pattern, landing, rollout,
    /// takeoff). `None` lets the pilot's reference runway stand in.
    pub runway: Option<Runway>,
    /// Traffic-pattern legs (entry, downwind, base, final).
    pub pattern_legs: Vec<LabeledLeg>,
    /// Index into `pattern_legs` of the leg being flown, if any.
    pub active_pattern_leg: Option<usize>,
    /// Ground path: taxi route legs, or the runway-exit connector.
    pub route: Vec<LabeledLeg>,
    /// Index into `route` of the leg being followed. `Some(route.len())`
    /// means past the last leg (pose phase); `None` means not started.
    pub route_active_idx: Option<usize>,
    /// Terminal pose: hold-short line, parking spot, line-up point.
    pub target_pose: Option<TaxiPose>,
    /// Short label for the route destination / pose ("HOLD SHORT", a
    /// parking spot name, an exit taxiway).
    pub target_label: Option<String>,
}

/// Everything the spatial view needs beyond `StatusSnapshot`.
#[derive(Debug, Clone, PartialEq)]
pub struct SpatialSnapshot {
    /// The profile's runway, or the pilot's reference runway.
    pub runway: Runway,
    pub field_elevation_ft: f64,
    pub roundout_height_ft: f64,
    pub flare_start_ft: f64,
    /// Beyond the runway's half-width plus this, the aircraft is off it.
    pub runway_clear_margin_ft: f64,
    pub overlay: SpatialOverlay,
}
