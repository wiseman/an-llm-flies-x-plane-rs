//! Pattern geometry + glidepath math.

use crate::guidance::runway_geometry::RunwayFrame;
use crate::types::{
    clamp, wrap_degrees_180, wrap_degrees_360, FlightPhase, StraightLeg, TrafficSide, Vec2,
};

#[derive(Debug, Clone)]
pub struct PatternGeometry {
    pub runway_frame: RunwayFrame,
    pub entry_leg: StraightLeg,
    pub downwind_leg: StraightLeg,
    pub base_leg: StraightLeg,
    pub final_leg: StraightLeg,
    pub join_point_runway_ft: Vec2,
    pub base_turn_x_ft: f64,
    pub downwind_y_ft: f64,
    /// Downwind extension applied at build time (pushes the base-turn point
    /// further from the runway). "Extend downwind" cumulates here.
    pub extension_ft: f64,
    /// Crosswind extension applied at build time (widens the effective
    /// downwind offset — i.e. pushes the pattern further out to the side).
    /// "Extend crosswind" cumulates here.
    pub crosswind_extension_ft: f64,
}

impl PatternGeometry {
    pub fn reciprocal_course_deg(&self) -> f64 {
        wrap_degrees_360(self.runway_frame.runway.course_deg + 180.0)
    }

    pub fn leg_for_phase(&self, phase: FlightPhase) -> Option<StraightLeg> {
        Some(match phase {
            FlightPhase::PatternEntry => self.entry_leg,
            FlightPhase::Downwind => self.downwind_leg,
            FlightPhase::Base => self.base_leg,
            FlightPhase::Final => self.final_leg,
            _ => return None,
        })
    }

    pub fn is_established_on_downwind(
        &self,
        runway_x_ft: Option<f64>,
        runway_y_ft: Option<f64>,
        track_deg: f64,
    ) -> bool {
        let (Some(x), Some(y)) = (runway_x_ft, runway_y_ft) else { return false };
        let altitude_band_ok = (y - self.downwind_y_ft).abs() <= 350.0;
        let track_ok = wrap_degrees_180(track_deg - self.reciprocal_course_deg()).abs() <= 20.0;
        altitude_band_ok && track_ok && x <= self.join_point_runway_ft.x
    }

    pub fn base_turn_ready(&self, runway_x_ft: Option<f64>) -> bool {
        runway_x_ft.is_some_and(|x| x <= self.base_turn_x_ft)
    }

    pub fn is_established_on_final(
        &self,
        runway_x_ft: Option<f64>,
        runway_y_ft: Option<f64>,
        track_deg: f64,
    ) -> bool {
        let (Some(x), Some(y)) = (runway_x_ft, runway_y_ft) else { return false };
        x <= -500.0
            && y.abs() <= 120.0
            && wrap_degrees_180(track_deg - self.runway_frame.runway.course_deg).abs() <= 15.0
    }
}

/// Build pattern geometry with independent downwind and crosswind
/// extensions. `crosswind_extension_ft` widens the effective downwind
/// offset (pushes the pattern further out laterally), which is how
/// "extend crosswind" is modelled — you fly further perpendicular to the
/// runway before the downwind turn. `downwind_extension_ft` moves the
/// base-turn point further past the threshold along the runway axis, as
/// before. Both default to 0 and compose additively.
pub fn build_pattern_geometry(
    runway_frame: &RunwayFrame,
    downwind_offset_ft: f64,
    downwind_extension_ft: f64,
    crosswind_extension_ft: f64,
) -> PatternGeometry {
    let side_sign = runway_frame.runway.traffic_side.sign();
    let effective_offset_ft = downwind_offset_ft + crosswind_extension_ft.max(0.0);
    let downwind_y_ft = side_sign * effective_offset_ft;
    let join_x_ft = effective_offset_ft * 1.1;
    let base_turn_x_ft = -(effective_offset_ft + downwind_extension_ft);
    let final_start_x_ft = -(10000.0f64).max(effective_offset_ft * 2.5);

    let entry_start_rf = Vec2::new(
        join_x_ft + effective_offset_ft,
        downwind_y_ft + side_sign * effective_offset_ft,
    );
    let join_point_rf = Vec2::new(join_x_ft, downwind_y_ft);
    let downwind_end_rf = Vec2::new(base_turn_x_ft, downwind_y_ft);
    let base_end_rf = Vec2::new(base_turn_x_ft, 0.0);

    let entry_leg = StraightLeg {
        start_ft: runway_frame.to_world_frame(entry_start_rf),
        end_ft: runway_frame.to_world_frame(join_point_rf),
    };
    let downwind_leg = StraightLeg {
        start_ft: runway_frame.to_world_frame(join_point_rf),
        end_ft: runway_frame.to_world_frame(downwind_end_rf),
    };
    let base_leg = StraightLeg {
        start_ft: runway_frame.to_world_frame(downwind_end_rf),
        end_ft: runway_frame.to_world_frame(base_end_rf),
    };
    let final_leg = runway_frame.final_leg(final_start_x_ft);
    PatternGeometry {
        runway_frame: runway_frame.clone(),
        entry_leg,
        downwind_leg,
        base_leg,
        final_leg,
        join_point_runway_ft: join_point_rf,
        base_turn_x_ft,
        downwind_y_ft,
        extension_ft: downwind_extension_ft,
        crosswind_extension_ft,
    }
}

/// Target altitude along the 3° (or other) glidepath at a given runway-frame
/// `runway_x_ft`, clamped at field elevation behind the threshold. The
/// ground-level aim-point default makes threshold crossing fall out as
/// `aim_x * tan(slope)`.
pub fn glidepath_target_altitude_ft(
    runway_frame: &RunwayFrame,
    runway_x_ft: f64,
    field_elevation_ft: f64,
    slope_deg: f64,
    aim_point_height_agl_ft: f64,
) -> f64 {
    let slope_rad = clamp(slope_deg / 57.2958, 0.0, 0.2);
    let distance_to_aimpoint = runway_frame.touchdown_runway_x_ft() - runway_x_ft;
    let path_height = aim_point_height_agl_ft + distance_to_aimpoint * slope_rad;
    field_elevation_ft + path_height.max(0.0)
}

pub fn glidepath_target_altitude_ft_default(
    runway_frame: &RunwayFrame,
    runway_x_ft: f64,
    field_elevation_ft: f64,
) -> f64 {
    glidepath_target_altitude_ft(runway_frame, runway_x_ft, field_elevation_ft, 3.0, 0.0)
}

/// Same as `glidepath_target_altitude_ft` but shifts the aim point
/// `lead_ft` feet *before* `touchdown_runway_x_ft`. TECS tracks a 3°
/// line that hits 0 AGL at the leading point so the aircraft is already
/// touched-down-ready when it enters the roundout/flare float — prevents
/// the "high over the numbers, lands long" pattern where TECS converges
/// on the 3° line only as the plane crosses the aim.
pub fn glidepath_target_altitude_leading_ft(
    runway_frame: &RunwayFrame,
    runway_x_ft: f64,
    field_elevation_ft: f64,
    slope_deg: f64,
    lead_ft: f64,
) -> f64 {
    let slope_rad = clamp(slope_deg / 57.2958, 0.0, 0.2);
    let effective_aim_x = runway_frame.touchdown_runway_x_ft() - lead_ft;
    let distance_to_aim = effective_aim_x - runway_x_ft;
    let path_height = distance_to_aim * slope_rad;
    field_elevation_ft + path_height.max(0.0)
}

// Unused-name suppression: keeps `TrafficSide` in scope for downstream
// re-exports even when no local code names it.
#[allow(dead_code)]
fn _traffic_side_reference(_s: TrafficSide) {}
