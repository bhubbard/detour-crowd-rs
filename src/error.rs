//! Error types for detour-crowd-rs.

use thiserror::Error;

/// Result alias for detour crowd operations.
pub type Result<T> = std::result::Result<T, CrowdError>;

/// Errors that can occur within the crowd simulation and avoidance systems.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum CrowdError {
    #[error("Agent with ID {0} not found")]
    AgentNotFound(usize),

    #[error("Agent pool full (maximum capacity {0} reached)")]
    MaxAgentsReached(usize),

    #[error("Corridor has no remaining valid corners / waypoints")]
    CorridorEmpty,

    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),

    #[error("Target unreachable or invalid")]
    InvalidTarget,
}
