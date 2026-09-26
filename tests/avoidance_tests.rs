use glam::Vec2;
use detour_crowd_rs::avoidance::{
    ObstacleAvoidanceQuery, ObstacleCircle, ObstacleSegment,
};

#[test]
fn test_avoidance_clear_path() {
    let query = ObstacleAvoidanceQuery::default();
    let pos = Vec2::new(0.0, 0.0);
    let radius = 0.5;
    let cur_vel = Vec2::new(2.0, 0.0);
    let des_vel = Vec2::new(3.0, 0.0);
    let max_speed = 3.5;

    // No obstacles
    let best_vel = query.sample_velocity(pos, radius, cur_vel, des_vel, max_speed, &[], &[]);
    // Should choose desired velocity directly
    assert!((best_vel - des_vel).length() < 1e-3);
}

#[test]
fn test_avoidance_head_on_collision() {
    let query = ObstacleAvoidanceQuery::default();
    let pos_a = Vec2::new(0.0, 0.0);
    let radius = 0.5;
    let cur_vel_a = Vec2::new(2.0, 0.0);
    let des_vel_a = Vec2::new(2.0, 0.0);
    let max_speed = 3.0;

    // Oncoming agent directly ahead at (4.0, 0.0) moving at (-2.0, 0.0)
    let oncoming = ObstacleCircle::new(Vec2::new(4.0, 0.0), 0.5, Vec2::new(-2.0, 0.0));

    let safe_vel = query.sample_velocity(
        pos_a,
        radius,
        cur_vel_a,
        des_vel_a,
        max_speed,
        &[oncoming],
        &[],
    );

    // The safe velocity MUST have steered away from the direct x-axis (non-zero y-component)
    // or slowed down significantly to prevent head-on collision
    assert!(
        safe_vel.y.abs() > 0.1 || safe_vel.x < 1.0,
        "Agent should steer aside or slow down to avoid head-on impact, got {:?}",
        safe_vel
    );
}

#[test]
fn test_avoidance_static_wall() {
    let query = ObstacleAvoidanceQuery::default();
    let pos = Vec2::new(0.0, 0.0);
    let radius = 0.5;
    let cur_vel = Vec2::new(2.0, 0.0);
    let des_vel = Vec2::new(2.0, 0.0); // Wants to walk directly east into the wall
    let max_speed = 3.0;

    // Vertical wall at x = 2.0 spanning from y = -5 to y = 5
    let wall = ObstacleSegment::new(Vec2::new(2.0, -5.0), Vec2::new(2.0, 5.0));

    let safe_vel = query.sample_velocity(
        pos,
        radius,
        cur_vel,
        des_vel,
        max_speed,
        &[],
        &[wall],
    );

    // Should not walk straight into wall at full speed
    assert!(
        safe_vel.x < 1.2 || safe_vel.y.abs() > 0.5,
        "Agent should deflect along wall or slow down, got {:?}",
        safe_vel
    );
}
