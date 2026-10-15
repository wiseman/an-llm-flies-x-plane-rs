//! Runway-relative coordinate frame.

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

    /// Half the runway width plus `margin_ft`: past this lateral offset
    /// the aircraft is off the runway.
    pub fn half_width_plus_ft(&self, margin_ft: f64) -> f64 {
        self.runway.width_ft * 0.5 + margin_ft
    }

    /// On the runway pavement, give or take `margin_ft` all round.
    pub fn on_runway_surface(&self, position_ft: Vec2, margin_ft: f64) -> bool {
        let p = self.to_runway_frame(position_ft);
        p.x >= -margin_ft
            && p.x <= self.runway.length_ft + margin_ft
            && p.y.abs() <= self.half_width_plus_ft(margin_ft)
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
        self.runway.touchdown_x_ft()
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
