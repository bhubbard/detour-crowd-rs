//! # detour-crowd-rs
//!
//! Pure Rust port of DetourCrowd from RecastNavigation.
//! Features Reciprocal Velocity Obstacles (RVO), adaptive velocity space sampling,
//! spatial hashing proximity grid for $O(1)$ neighbor queries, corridor navigation,
//! and boundary / separation steering forces.

pub mod agent;
pub mod avoidance;
pub mod crowd;
pub mod error;
pub mod proximity_grid;

// Re-exports for convenience
pub use agent::{CrowdAgent, CrowdAgentParams, CrowdAgentState};
pub use avoidance::{
    ObstacleAvoidanceParams, ObstacleAvoidanceQuery, ObstacleCircle, ObstacleSegment,
};
pub use crowd::Crowd;
pub use error::{CrowdError, Result};
pub use proximity_grid::ProximityGrid;
