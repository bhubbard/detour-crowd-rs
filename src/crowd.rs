//! Crowd management and steering pipeline.
//!
//! Direct Rust port and optimization of RecastNavigation's `dtCrowd`.
//! Coordinates agents, runs the proximity grid, computes boundary avoidance
//! and neighbor separation forces, and integrates the RVO avoidance query.

use glam::Vec2;
use serde::{Deserialize, Serialize};

use crate::agent::{CrowdAgent, CrowdAgentParams, CrowdAgentState};
use crate::avoidance::{
    ObstacleAvoidanceParams, ObstacleAvoidanceQuery, ObstacleCircle, ObstacleSegment,
};
use crate::error::{CrowdError, Result};
use crate::proximity_grid::ProximityGrid;

/// Main crowd simulation manager coordinating agents, navigation, and obstacle avoidance.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Crowd {
    agents: Vec<CrowdAgent>,
    max_agents: usize,
    proximity_grid: ProximityGrid,
    boundary_segments: Vec<ObstacleSegment>,
    avoidance_params: Vec<ObstacleAvoidanceParams>,
}

impl Crowd {
    /// Creates a new crowd manager with a maximum agent capacity and default spatial grid.
    pub fn new(max_agents: usize, cell_size: f32) -> Self {
        Self {
            agents: Vec::with_capacity(max_agents),
            max_agents,
            proximity_grid: ProximityGrid::new(cell_size, 1024),
            boundary_segments: Vec::new(),
            avoidance_params: vec![ObstacleAvoidanceParams::default()],
        }
    }

    /// Adds a new agent to the crowd simulation.
    pub fn add_agent(&mut self, position: Vec2, params: CrowdAgentParams) -> Result<usize> {
        // Reuse an inactive / invalid slot if one exists
        for (idx, agent) in self.agents.iter_mut().enumerate() {
            if agent.state == CrowdAgentState::Invalid {
                *agent = CrowdAgent::new(idx, position, params);
                agent.state = CrowdAgentState::Active;
                return Ok(idx);
            }
        }

        if self.agents.len() >= self.max_agents {
            return Err(CrowdError::MaxAgentsReached(self.max_agents));
        }

        let id = self.agents.len();
        let mut agent = CrowdAgent::new(id, position, params);
        agent.state = CrowdAgentState::Active;
        self.agents.push(agent);
        Ok(id)
    }

    /// Removes an agent from the crowd, marking its slot as invalid.
    pub fn remove_agent(&mut self, id: usize) -> Result<()> {
        let agent = self.agents.get_mut(id).ok_or(CrowdError::AgentNotFound(id))?;
        if agent.state == CrowdAgentState::Invalid {
            return Err(CrowdError::AgentNotFound(id));
        }
        agent.stop();
        agent.state = CrowdAgentState::Invalid;
        Ok(())
    }

    /// Returns a reference to an agent by ID.
    pub fn agent(&self, id: usize) -> Option<&CrowdAgent> {
        let a = self.agents.get(id)?;
        if a.state != CrowdAgentState::Invalid {
            Some(a)
        } else {
            None
        }
    }

    /// Returns a mutable reference to an agent by ID.
    pub fn agent_mut(&mut self, id: usize) -> Option<&mut CrowdAgent> {
        let a = self.agents.get_mut(id)?;
        if a.state != CrowdAgentState::Invalid {
            Some(a)
        } else {
            None
        }
    }

    /// Returns all currently active agents.
    pub fn active_agents(&self) -> impl Iterator<Item = &CrowdAgent> {
        self.agents
            .iter()
            .filter(|a| a.state != CrowdAgentState::Invalid)
    }

    /// Number of active agents currently simulating.
    pub fn active_agent_count(&self) -> usize {
        self.agents
            .iter()
            .filter(|a| a.state != CrowdAgentState::Invalid)
            .count()
    }

    /// Sets the destination target for an agent.
    pub fn set_agent_target(&mut self, id: usize, target: Vec2) -> Result<()> {
        let agent = self.agents.get_mut(id).ok_or(CrowdError::AgentNotFound(id))?;
        if agent.state == CrowdAgentState::Invalid {
            return Err(CrowdError::AgentNotFound(id));
        }
        agent.set_target(target);
        Ok(())
    }

    /// Sets a corridor of waypoints for an agent.
    pub fn set_agent_corridor(&mut self, id: usize, corners: Vec<Vec2>) -> Result<()> {
        let agent = self.agents.get_mut(id).ok_or(CrowdError::AgentNotFound(id))?;
        if agent.state == CrowdAgentState::Invalid {
            return Err(CrowdError::AgentNotFound(id));
        }
        agent.set_corridor(corners);
        Ok(())
    }

    /// Adds a static boundary line segment (wall / navmesh edge) to the crowd.
    pub fn add_boundary_segment(&mut self, p: Vec2, q: Vec2) {
        self.boundary_segments.push(ObstacleSegment::new(p, q));
    }

    /// Clears all static boundary segments.
    pub fn clear_boundary_segments(&mut self) {
        self.boundary_segments.clear();
    }

    /// Returns the static boundary segments.
    pub fn boundary_segments(&self) -> &[ObstacleSegment] {
        &self.boundary_segments
    }

    /// Sets or updates an obstacle avoidance parameter profile.
    pub fn set_avoidance_params(&mut self, index: usize, params: ObstacleAvoidanceParams) {
        if index >= self.avoidance_params.len() {
            self.avoidance_params.resize(index + 1, ObstacleAvoidanceParams::default());
        }
        self.avoidance_params[index] = params;
    }

    /// Advances the entire crowd simulation by one timestep `dt` (in seconds).
    pub fn update(&mut self, dt: f32) {
        if dt <= 1e-4 {
            return;
        }

        // 1. Clear and populate proximity grid with active agents
        self.proximity_grid.clear();
        for agent in &self.agents {
            if agent.state != CrowdAgentState::Invalid {
                self.proximity_grid
                    .insert_circle(agent.id, agent.position, agent.params.radius);
            }
        }

        // 2. Query neighbors for all active agents
        let num_agents = self.agents.len();
        let mut neighbors_by_agent: Vec<Vec<ObstacleCircle>> = vec![Vec::new(); num_agents];
        let mut query_results = Vec::with_capacity(32);

        for (i, agent) in self.agents.iter().enumerate() {
            if agent.state == CrowdAgentState::Invalid {
                continue;
            }

            self.proximity_grid.query_circle(
                agent.position,
                agent.params.collision_query_range,
                &mut query_results,
            );

            let mut neighbors = Vec::with_capacity(query_results.len());
            for &other_id in &query_results {
                if other_id != i
                    && let Some(other) = self.agents.get(other_id)
                        && other.state != CrowdAgentState::Invalid {
                            neighbors.push(ObstacleCircle::new(
                                other.position,
                                other.params.radius,
                                other.velocity,
                            ));
                        }
            }
            neighbors_by_agent[i] = neighbors;
        }

        // 3. Update navigation corridors and compute raw desired velocity
        for agent in &mut self.agents {
            if agent.state == CrowdAgentState::Walking {
                let arrive_dist = agent.params.radius * 0.5;

                // Check arrival at current corner
                if let Some(corner) = agent.current_corner() {
                    let to_corner = corner - agent.position;
                    let dist = to_corner.length();

                    if dist <= arrive_dist {
                        // Advance to next corner or stop if at final target
                        if !agent.advance_corner() {
                            agent.velocity = Vec2::ZERO;
                            agent.desired_velocity = Vec2::ZERO;
                            agent.state = CrowdAgentState::Active;
                            continue;
                        }
                    }

                    // Recalculate desired velocity towards active corner
                    if let Some(active_corner) = agent.current_corner() {
                        let to_active = active_corner - agent.position;
                        let dist = to_active.length();
                        if dist > 1e-4 {
                            agent.desired_velocity = (to_active / dist) * agent.params.max_speed;
                        } else {
                            agent.desired_velocity = Vec2::ZERO;
                        }
                    }
                }
            } else {
                agent.desired_velocity = Vec2::ZERO;
            }
        }

        // 4. Calculate boundary avoidance and neighbor separation steering forces
        let mut safe_velocities = vec![Vec2::ZERO; num_agents];

        for (i, agent) in self.agents.iter().enumerate() {
            if agent.state == CrowdAgentState::Invalid {
                continue;
            }

            let neighbors = &neighbors_by_agent[i];
            let mut sep_force = Vec2::ZERO;

            // Separation force among close neighbors
            for neighbor in neighbors {
                let to_agent = agent.position - neighbor.position;
                let dist = to_agent.length();
                let comfort_dist = (agent.params.radius + neighbor.radius) * 1.5;

                if dist < comfort_dist && dist > 1e-4 {
                    let weight = 1.0 - (dist / comfort_dist);
                    sep_force += (to_agent / dist) * weight;
                }
            }

            // Boundary avoidance steering force
            let mut boundary_force = Vec2::ZERO;
            let boundary_range = agent.params.radius * 2.0;

            for seg in &self.boundary_segments {
                let seg_vec = seg.q - seg.p;
                let seg_len_sq = seg_vec.length_squared();
                if seg_len_sq > 1e-4 {
                    let t = ((agent.position - seg.p).dot(seg_vec) / seg_len_sq).clamp(0.0, 1.0);
                    let closest = seg.p + seg_vec * t;
                    let to_agent = agent.position - closest;
                    let dist = to_agent.length();

                    if dist < boundary_range && dist > 1e-4 {
                        let weight = 1.0 - (dist / boundary_range);
                        boundary_force += (to_agent / dist) * weight;
                    }
                }
            }

            // Blend separation & boundary steering with desired velocity
            let blended_des_vel = agent.desired_velocity
                + sep_force * (agent.params.separation_weight * agent.params.max_speed)
                + boundary_force * (agent.params.boundary_avoidance_weight * agent.params.max_speed);

            let capped_des_vel = blended_des_vel.clamp_length_max(agent.params.max_speed);

            // Obstacle Avoidance Query: RVO sampling
            let profile_idx = agent.params.avoidance_profile.min(self.avoidance_params.len() - 1);
            let avoidance_query = ObstacleAvoidanceQuery::new(self.avoidance_params[profile_idx]);

            let safe_vel = avoidance_query.sample_velocity(
                agent.position,
                agent.params.radius,
                agent.velocity,
                capped_des_vel,
                agent.params.max_speed,
                neighbors,
                &self.boundary_segments,
            );

            safe_velocities[i] = safe_vel;
        }

        // 5. Integrate velocity and position respecting acceleration limits
        for (i, agent) in self.agents.iter_mut().enumerate() {
            if agent.state == CrowdAgentState::Invalid {
                continue;
            }

            let target_vel = safe_velocities[i];
            let max_accel_step = agent.params.max_acceleration * dt;
            let dv = target_vel - agent.velocity;

            let clamped_dv = if dv.length() > max_accel_step {
                dv.normalize() * max_accel_step
            } else {
                dv
            };

            agent.velocity += clamped_dv;
            agent.position += agent.velocity * dt;
        }
    }
}
