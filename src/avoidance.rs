//! Local obstacle avoidance: Reciprocal Velocity Obstacles (RVO) and sampling-based query.
//!
//! Direct Rust port and enhancement of RecastNavigation's `dtObstacleAvoidanceQuery`.
//! Evaluates candidate velocities in velocity space against nearby dynamic agent circles
//! and static segment boundaries (walls / navmesh edges), minimizing a penalty function
//! balancing collision safety, target trajectory alignment, and smooth acceleration.

use glam::Vec2;
use serde::{Deserialize, Serialize};

/// Dynamic circular obstacle representing an agent or moving collider.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ObstacleCircle {
    /// World position of the obstacle center.
    pub position: Vec2,
    /// Collision radius.
    pub radius: f32,
    /// Current velocity vector of the obstacle.
    pub velocity: Vec2,
}

impl ObstacleCircle {
    /// Creates a new dynamic circular obstacle.
    pub fn new(position: Vec2, radius: f32, velocity: Vec2) -> Self {
        Self {
            position,
            radius,
            velocity,
        }
    }
}

/// Static line segment obstacle representing a wall, barrier, or navmesh boundary.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ObstacleSegment {
    /// Start point of the segment.
    pub p: Vec2,
    /// End point of the segment.
    pub q: Vec2,
}

impl ObstacleSegment {
    /// Creates a new static line segment obstacle.
    pub fn new(p: Vec2, q: Vec2) -> Self {
        Self { p, q }
    }
}

/// Parameters configuring the velocity sampling and penalty calculation.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ObstacleAvoidanceParams {
    /// Weight given to time-of-impact penalty (default: 2.5).
    pub weight_toi: f32,
    /// Weight given to deviation from desired velocity (default: 2.0).
    pub weight_des_vel: f32,
    /// Weight given to deviation from current velocity for smooth motion (default: 0.75).
    pub weight_cur_vel: f32,
    /// Weight given to side preference / lane formation (default: 0.75).
    pub weight_side: f32,
    /// Forward anticipation horizon in seconds (default: 2.5s).
    pub time_horizon: f32,
    /// Number of sample rings in polar velocity space (default: 7).
    pub pattern_rings: usize,
    /// Number of sample sectors per ring (default: 8).
    pub pattern_sectors: usize,
    /// Number of adaptive sampling depth iterations (default: 2).
    pub adaptive_depth: usize,
}

impl Default for ObstacleAvoidanceParams {
    fn default() -> Self {
        Self {
            weight_toi: 10.0,
            weight_des_vel: 2.0,
            weight_cur_vel: 0.75,
            weight_side: 0.75,
            time_horizon: 2.5,
            pattern_rings: 7,
            pattern_sectors: 8,
            adaptive_depth: 2,
        }
    }
}

/// Obstacle Avoidance Query engine.
#[derive(Debug, Clone, Default)]
pub struct ObstacleAvoidanceQuery {
    pub params: ObstacleAvoidanceParams,
}

impl ObstacleAvoidanceQuery {
    /// Creates a new avoidance query engine with the given parameters.
    pub fn new(params: ObstacleAvoidanceParams) -> Self {
        Self { params }
    }

    /// Evaluates candidate velocity space to find the optimal velocity for an agent.
    ///
    /// # Arguments
    /// * `pos` - Agent current world position.
    /// * `radius` - Agent collision radius.
    /// * `cur_vel` - Agent current velocity.
    /// * `des_vel` - Desired velocity towards navigation target.
    /// * `max_speed` - Maximum agent speed.
    /// * `circles` - List of nearby dynamic obstacles (other crowd agents).
    /// * `segments` - List of nearby static wall/boundary segments.
    #[allow(clippy::too_many_arguments)]
    pub fn sample_velocity(
        &self,
        pos: Vec2,
        radius: f32,
        cur_vel: Vec2,
        des_vel: Vec2,
        max_speed: f32,
        circles: &[ObstacleCircle],
        segments: &[ObstacleSegment],
    ) -> Vec2 {
        if max_speed <= 1e-4 {
            return Vec2::ZERO;
        }

        let mut best_vel = des_vel.clamp_length_max(max_speed);
        let mut min_penalty = self.evaluate_velocity_penalty(
            pos, radius, best_vel, cur_vel, des_vel, max_speed, circles, segments,
        );

        // If desired velocity is completely clear, take it directly
        if min_penalty < 1e-4 {
            return best_vel;
        }

        // Test stopping (zero velocity) as a fallback baseline
        let stop_penalty = self.evaluate_velocity_penalty(
            pos,
            radius,
            Vec2::ZERO,
            cur_vel,
            des_vel,
            max_speed,
            circles,
            segments,
        );
        if stop_penalty < min_penalty {
            min_penalty = stop_penalty;
            best_vel = Vec2::ZERO;
        }

        // Polar velocity sampling around origin and desired heading
        let rings = self.params.pattern_rings.max(3);
        let sectors = self.params.pattern_sectors.max(4);

        for r in 1..=rings {
            let speed_frac = (r as f32) / (rings as f32);
            let speed = speed_frac * max_speed;

            for s in 0..sectors {
                let angle = (s as f32) * std::f32::consts::TAU / (sectors as f32);
                let (sin_a, cos_a) = angle.sin_cos();
                let cand_vel = Vec2::new(cos_a, sin_a) * speed;

                let penalty = self.evaluate_velocity_penalty(
                    pos, radius, cand_vel, cur_vel, des_vel, max_speed, circles, segments,
                );

                if penalty < min_penalty {
                    min_penalty = penalty;
                    best_vel = cand_vel;
                }
            }
        }

        // Adaptive refinement pass around best candidate
        let mut search_radius = max_speed * 0.5;
        for _ in 0..self.params.adaptive_depth {
            let center = best_vel;
            search_radius *= 0.5;

            for s in 0..sectors {
                let angle = (s as f32) * std::f32::consts::TAU / (sectors as f32);
                let (sin_a, cos_a) = angle.sin_cos();
                let offset = Vec2::new(cos_a, sin_a) * search_radius;
                let cand_vel = (center + offset).clamp_length_max(max_speed);

                let penalty = self.evaluate_velocity_penalty(
                    pos, radius, cand_vel, cur_vel, des_vel, max_speed, circles, segments,
                );

                if penalty < min_penalty {
                    min_penalty = penalty;
                    best_vel = cand_vel;
                }
            }
        }

        best_vel
    }

    /// Evaluates the total penalty of a candidate velocity $\vec{v}$.
    #[allow(clippy::too_many_arguments)]
    fn evaluate_velocity_penalty(
        &self,
        pos: Vec2,
        radius: f32,
        cand_vel: Vec2,
        cur_vel: Vec2,
        des_vel: Vec2,
        max_speed: f32,
        circles: &[ObstacleCircle],
        segments: &[ObstacleSegment],
    ) -> f32 {
        let toi = self.compute_time_of_impact(pos, radius, cand_vel, cur_vel, circles, segments);

        // 1. Time-to-impact penalty: collision sooner than time_horizon incurs high penalty
        let toi_penalty = if toi < self.params.time_horizon {
            (self.params.time_horizon - toi) / (toi + 0.05)
        } else {
            0.0
        };

        // 2. Desired velocity deviation penalty
        let des_diff = (cand_vel - des_vel).length() / max_speed;

        // 3. Current velocity deviation penalty (smoothness)
        let cur_diff = (cand_vel - cur_vel).length() / max_speed;

        // 4. Side preference penalty (favors passing on right when meeting oncoming agents)
        let mut side_penalty = 0.0;
        let cand_speed = cand_vel.length();
        if cand_speed > 1e-4 && des_vel.length() > 1e-4 {
            let des_dir = des_vel.normalize();
            let cand_dir = cand_vel / cand_speed;
            // 2D perp dot product (cross product z-component)
            let side = des_dir.x * cand_dir.y - des_dir.y * cand_dir.x;
            if side < 0.0 {
                // Moving to the left is penalized slightly relative to right
                side_penalty = -side;
            }
        }

        self.params.weight_toi * toi_penalty
            + self.params.weight_des_vel * des_diff
            + self.params.weight_cur_vel * cur_diff
            + self.params.weight_side * side_penalty
    }

    /// Computes minimum time of impact (TOI) between agent moving at `cand_vel` and all obstacles.
    fn compute_time_of_impact(
        &self,
        pos: Vec2,
        radius: f32,
        cand_vel: Vec2,
        cur_vel: Vec2,
        circles: &[ObstacleCircle],
        segments: &[ObstacleSegment],
    ) -> f32 {
        let mut min_toi = self.params.time_horizon;

        // Dynamic obstacle circles (agents) using Reciprocal Velocity Obstacles (RVO) formulation
        for circle in circles {
            let combined_radius = radius + circle.radius;
            let rel_pos = circle.position - pos;
            let dist_sq = rel_pos.length_squared();

            if dist_sq < combined_radius * combined_radius {
                // Already penetrating or overlapping
                return 0.0;
            }

            // RVO relative velocity: 2 * v_cand - v_cur - v_other
            let rel_vel = cand_vel * 2.0 - cur_vel - circle.velocity;
            let speed_sq = rel_vel.length_squared();

            if speed_sq < 1e-6 {
                continue;
            }

            // Ray-circle intersection: (rel_pos - rel_vel * t)^2 = combined_radius^2
            let a = speed_sq;
            let b = -2.0 * rel_pos.dot(rel_vel);
            let c = dist_sq - combined_radius * combined_radius;

            let discriminant = b * b - 4.0 * a * c;
            if discriminant >= 0.0 {
                let sqrt_d = discriminant.sqrt();
                let t1 = (-b - sqrt_d) / (2.0 * a);
                if t1 > 0.0 && t1 < min_toi {
                    min_toi = t1;
                }
            }
        }

        // Static obstacle segments (walls / boundaries)
        for seg in segments {
            if let Some(toi) = time_of_impact_segment(pos, radius, cand_vel, seg.p, seg.q)
                && toi < min_toi {
                    min_toi = toi;
                }
        }

        min_toi
    }
}

/// Computes time of impact between a moving circle (agent) and a static line segment.
fn time_of_impact_segment(
    pos: Vec2,
    radius: f32,
    vel: Vec2,
    p: Vec2,
    q: Vec2,
) -> Option<f32> {
    let seg = q - p;
    let seg_len_sq = seg.length_squared();
    if seg_len_sq < 1e-6 {
        return None;
    }

    // Closest point on segment to pos
    let t = ((pos - p).dot(seg) / seg_len_sq).clamp(0.0, 1.0);
    let closest = p + seg * t;
    let to_closest = closest - pos;
    let dist_sq = to_closest.length_squared();

    if dist_sq < radius * radius {
        return Some(0.0);
    }

    let speed_sq = vel.length_squared();
    if speed_sq < 1e-6 {
        return None;
    }

    // Check intersection of ray pos + vel * t with capsule around segment of radius `radius`
    // Project ray onto segment normal
    let normal = Vec2::new(-seg.y, seg.x).normalize();
    let dist_normal = (pos - p).dot(normal);
    let vel_normal = vel.dot(normal);

    if vel_normal.abs() < 1e-6 {
        return None;
    }

    let target_dist = if dist_normal > 0.0 { radius } else { -radius };
    let t_hit = (target_dist - dist_normal) / vel_normal;

    if t_hit > 0.0 {
        let hit_pos = pos + vel * t_hit;
        let proj = ((hit_pos - p).dot(seg) / seg_len_sq).clamp(0.0, 1.0);
        let pt_on_seg = p + seg * proj;
        if hit_pos.distance_squared(pt_on_seg) <= (radius * 1.05) * (radius * 1.05) {
            return Some(t_hit);
        }
    }

    None
}
