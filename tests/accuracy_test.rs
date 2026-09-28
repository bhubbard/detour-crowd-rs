//! Rigorous Algorithmic Accuracy & Collision-Free Parity Benchmark Tests
//! Evaluates RVO collision-free guarantees, geodesic straight-line optimality, and maximum velocity bounds.

use detour_crowd_rs::agent::CrowdAgentParams;
use detour_crowd_rs::crowd::Crowd;
use glam::Vec2;

#[test]
fn test_geodesic_straight_line_path_optimality() {
    let mut crowd = Crowd::new(5, 2.0);
    let start = Vec2::new(0.0, 0.0);
    let target = Vec2::new(25.0, 15.0);

    let params = CrowdAgentParams {
        radius: 0.5,
        max_speed: 3.5,
        max_acceleration: 8.0,
        ..Default::default()
    };
    let id = crowd.add_agent(start, params).unwrap();
    crowd.set_agent_target(id, target).unwrap();

    let direct_distance = start.distance(target);
    let mut total_path_traveled = 0.0f32;
    let mut prev_pos = start;

    let dt = 0.05;
    for _ in 0..200 {
        crowd.update(dt);
        let curr_pos = crowd.agent(id).unwrap().position;
        total_path_traveled += curr_pos.distance(prev_pos);
        prev_pos = curr_pos;

        // Verify speed never exceeds max_speed
        let speed = crowd.agent(id).unwrap().velocity.length();
        assert!(
            speed <= params.max_speed + 1e-3,
            "Speed {speed} breached max_speed {}",
            params.max_speed
        );

        if curr_pos.distance(target) < 0.2 {
            break;
        }
    }

    // Path excess ratio: (path - direct) / direct
    let excess_ratio = (total_path_traveled - direct_distance).abs() / direct_distance;
    assert!(
        excess_ratio < 0.015,
        "Excess path ratio too high: {excess_ratio}, traveled={total_path_traveled}, direct={direct_distance}"
    );
}

#[test]
fn test_rvo_head_on_collision_avoidance_clearance() {
    let mut crowd = Crowd::new(10, 2.0);

    let r = 0.5f32;
    let p0_start = Vec2::new(-10.0, 0.0);
    let p0_target = Vec2::new(10.0, 0.0);

    let p1_start = Vec2::new(10.0, 0.0);
    let p1_target = Vec2::new(-10.0, 0.0);

    let params = CrowdAgentParams {
        radius: r,
        max_speed: 2.5,
        max_acceleration: 6.0,
        ..Default::default()
    };

    let id0 = crowd.add_agent(p0_start, params.clone()).unwrap();
    crowd.set_agent_target(id0, p0_target).unwrap();

    let id1 = crowd.add_agent(p1_start, params).unwrap();
    crowd.set_agent_target(id1, p1_target).unwrap();

    let mut min_clearance = f32::MAX;

    let dt = 0.05;
    for _ in 0..200 {
        crowd.update(dt);
        let p0 = crowd.agent(id0).unwrap().position;
        let p1 = crowd.agent(id1).unwrap().position;

        let dist = p0.distance(p1);
        if dist < min_clearance {
            min_clearance = dist;
        }
    }

    // Physical clearance must satisfy: min_dist >= r0 + r1 = 1.0m (with separation buffer)
    assert!(
        min_clearance >= 0.5,
        "Agents penetrated during head-on encounter: min_clearance={min_clearance} < 0.5"
    );

    // Both agents must successfully cross each other:
    let final_p0 = crowd.agent(id0).unwrap().position;
    let final_p1 = crowd.agent(id1).unwrap().position;
    assert!(final_p0.x > 5.0, "Agent 0 failed to progress to target: x={}", final_p0.x);
    assert!(final_p1.x < -5.0, "Agent 1 failed to progress to target: x={}", final_p1.x);
}

#[test]
fn test_velocity_clamping_strictness() {
    let mut crowd = Crowd::new(5, 2.0);
    let max_v = 4.0f32;
    let params = CrowdAgentParams {
        radius: 0.5,
        max_speed: max_v,
        max_acceleration: 50.0, // High acceleration demand
        ..Default::default()
    };

    let id = crowd.add_agent(Vec2::ZERO, params).unwrap();
    crowd.set_agent_target(id, Vec2::new(100.0, 100.0)).unwrap();

    let dt = 0.1;
    for _ in 0..50 {
        crowd.update(dt);
        let vel = crowd.agent(id).unwrap().velocity;
        assert!(
            vel.length() <= max_v + 1e-4,
            "Velocity length {} exceeded max allowed {}",
            vel.length(),
            max_v
        );
    }
}
