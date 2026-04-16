//! Waypoint sequencer. Mirrors guidance/route_manager.py.

use crate::types::{Vec2, Waypoint};

#[derive(Debug, Clone)]
pub struct RouteManager {
    pub waypoints: Vec<Waypoint>,
    pub active_index: usize,
    pub switch_radius_ft: f64,
}

impl RouteManager {
    pub fn new(waypoints: Vec<Waypoint>) -> Self {
        Self {
            waypoints,
            active_index: 0,
            switch_radius_ft: 2500.0,
        }
    }

    pub fn active_waypoint(&self) -> Option<&Waypoint> {
        self.waypoints.get(self.active_index)
    }

    pub fn advance_if_needed(&mut self, position_ft: Vec2) {
        let Some(wp) = self.waypoints.get(self.active_index) else {
            return;
        };
        if position_ft.distance_to(wp.position_ft) <= self.switch_radius_ft
            && self.active_index < self.waypoints.len() - 1
        {
            self.active_index += 1;
        }
    }

    pub fn is_complete(&self) -> bool {
        self.active_waypoint().is_none()
    }
}
