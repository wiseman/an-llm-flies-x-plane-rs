//! Runway-relative coordinate frame. Mirrors guidance/runway_geometry.py.

use crate::types::{heading_to_vector, Runway, StraightLeg, Vec2};

#[derive(Debug, Clone)]
pub struct RunwayFrame {
    pub runway: Runway,
}

impl RunwayFrame {
    pub fn new(runway: Runway) -> Self {
        Self { runway }
    }

    pub fn forward(&self) -> Vec2 {
        heading_to_vector(self.runway.course_deg, 1.0)
    }

    pub fn right(&self) -> Vec2 {
        heading_to_vector(self.runway.course_deg + 90.0, 1.0)
    }

    pub fn to_runway_frame(&self, point_ft: Vec2) -> Vec2 {
        let delta = point_ft - self.runway.threshold_ft;
        Vec2::new(delta.dot(self.forward()), delta.dot(self.right()))
    }

    pub fn to_world_frame(&self, point_ft: Vec2) -> Vec2 {
        self.runway.threshold_ft + self.forward() * point_ft.x + self.right() * point_ft.y
    }

    /// Aim-point x in the runway frame — i.e. how far past x=0 (the
    /// pavement end, frame origin) the aircraft should touch down on
    /// landing.
    ///
    /// Composed of two parts:
    /// - `displaced_threshold_ft`: skip past any displaced threshold so
    ///   the aim point falls on usable landing pavement (the runway
    ///   markings in apt.dat put the landing threshold this far inside
    ///   the pavement end).
    /// - `clamp(tdz/2, 500, length/3)`: the standard 1000 ft TDZ aim
    ///   (via `touchdown_zone_ft = 2000`), clamped for short runways.
    pub fn touchdown_runway_x_ft(&self) -> f64 {
        let half_tdz = self.runway.touchdown_zone_ft * 0.5;
        let aim_from_landing_threshold = half_tdz.max(500.0).min(self.runway.length_ft / 3.0);
        self.runway.displaced_threshold_ft + aim_from_landing_threshold
    }

    pub fn touchdown_point_ft(&self) -> Vec2 {
        self.to_world_frame(Vec2::new(self.touchdown_runway_x_ft(), 0.0))
    }

    pub fn departure_leg(&self, length_ft: f64) -> StraightLeg {
        StraightLeg {
            start_ft: self.runway.threshold_ft,
            end_ft: self.to_world_frame(Vec2::new(length_ft, 0.0)),
        }
    }

    pub fn final_leg(&self, start_x_ft: f64) -> StraightLeg {
        StraightLeg {
            start_ft: self.to_world_frame(Vec2::new(start_x_ft, 0.0)),
            end_ft: self.to_world_frame(Vec2::new(self.touchdown_runway_x_ft(), 0.0)),
        }
    }
}
