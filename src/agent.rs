//! Crowd agent state, configuration, and navigation pipeline.
//!
//! Direct Rust port of RecastNavigation's `dtCrowdAgent`.
//! Represents a single pedestrian or character navigating through a crowd.

use glam::Vec2;
use serde::{Deserialize, Serialize};

/// State of an agent within the crowd simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum CrowdAgentState {
    /// Inactive / unused agent slot.
    #[default]
    Invalid,
    /// Active in the crowd but stationary or waiting for a navigation command.
    Waiting,
    /// Actively moving along a corridor or towards a navigation target.
    Walking,
    /// Actively controlled and simulating in the crowd.
    Active,
}

/// Configuration parameters for an individual crowd agent.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CrowdAgentParams {
    /// Collision radius of the agent in meters (default: 0.5m).
    pub radius: f32,
    /// Physical height of the agent in meters (default: 2.0m).
    pub height: f32,
    /// Maximum acceleration in m/s^2 (default: 8.0 m/s^2).
    pub max_acceleration: f32,
    /// Maximum movement speed in m/s (default: 3.5 m/s).
    pub max_speed: f32,
    /// Proximity query range for finding neighbor agents (default: radius * 8.0).
    pub collision_query_range: f32,
    /// Weight applied to neighbor separation force (default: 2.0).
    pub separation_weight: f32,
    /// Weight applied to boundary avoidance force (default: 1.5).
    pub boundary_avoidance_weight: f32,
    /// Index of obstacle avoidance parameter profile to use (default: 0).
    pub avoidance_profile: usize,
}

impl Default for CrowdAgentParams {
    fn default() -> Self {
        let radius = 0.5;
        Self {
            radius,
            height: 2.0,
            max_acceleration: 8.0,
            max_speed: 3.5,
            collision_query_range: radius * 8.0,
            separation_weight: 2.0,
            boundary_avoidance_weight: 1.5,
            avoidance_profile: 0,
        }
    }
}

/// An individual crowd agent in the simulation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CrowdAgent {
    /// Unique agent slot identifier.
    pub id: usize,
    /// Current lifecycle state of the agent.
    pub state: CrowdAgentState,
    /// Current 2D world position.
    pub position: Vec2,
    /// Current linear velocity vector.
    pub velocity: Vec2,
    /// Desired velocity vector pointing toward target or corridor corner.
    pub desired_velocity: Vec2,
    /// Ultimate target position in world space.
    pub target_position: Option<Vec2>,
    /// List of waypoints / corners along the navigation corridor.
    pub corridor_corners: Vec<Vec2>,
    /// Current corner index in `corridor_corners`.
    pub corridor_index: usize,
    /// Agent physical and behavioral configuration parameters.
    pub params: CrowdAgentParams,
}

impl CrowdAgent {
    /// Creates a new crowd agent at the given position with specified parameters.
    pub fn new(id: usize, position: Vec2, params: CrowdAgentParams) -> Self {
        Self {
            id,
            state: CrowdAgentState::Waiting,
            position,
            velocity: Vec2::ZERO,
            desired_velocity: Vec2::ZERO,
            target_position: None,
            corridor_corners: Vec::new(),
            corridor_index: 0,
            params,
        }
    }

    /// Sets a direct target position for the agent.
    pub fn set_target(&mut self, target: Vec2) {
        self.target_position = Some(target);
        self.corridor_corners = vec![target];
        self.corridor_index = 0;
        self.state = CrowdAgentState::Walking;
    }

    /// Sets a corridor of waypoints / path corners for the agent to follow.
    pub fn set_corridor(&mut self, corners: Vec<Vec2>) {
        if let Some(&last) = corners.last() {
            self.target_position = Some(last);
        }
        self.corridor_corners = corners;
        self.corridor_index = 0;
        self.state = if self.corridor_corners.is_empty() {
            CrowdAgentState::Waiting
        } else {
            CrowdAgentState::Walking
        };
    }

    /// Returns the current active navigation corner or waypoint.
    #[inline]
    pub fn current_corner(&self) -> Option<Vec2> {
        self.corridor_corners.get(self.corridor_index).copied()
    }

    /// Advances to the next corner along the corridor.
    pub fn advance_corner(&mut self) -> bool {
        if self.corridor_index + 1 < self.corridor_corners.len() {
            self.corridor_index += 1;
            true
        } else {
            false
        }
    }

    /// Computes the straight-line distance to the current active corner.
    #[inline]
    pub fn distance_to_current_corner(&self) -> Option<f32> {
        self.current_corner().map(|c| self.position.distance(c))
    }

    /// Computes the straight-line distance to the final target position.
    #[inline]
    pub fn distance_to_target(&self) -> Option<f32> {
        self.target_position.map(|t| self.position.distance(t))
    }

    /// Checks if the agent has reached its target within a threshold distance.
    #[inline]
    pub fn has_reached_target(&self, threshold: f32) -> bool {
        self.distance_to_target().is_some_and(|d| d <= threshold)
    }

    /// Clears any active target or path and puts the agent into waiting state.
    pub fn stop(&mut self) {
        self.target_position = None;
        self.corridor_corners.clear();
        self.corridor_index = 0;
        self.desired_velocity = Vec2::ZERO;
        if self.state != CrowdAgentState::Invalid {
            self.state = CrowdAgentState::Waiting;
        }
    }
}
