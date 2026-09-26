use glam::Vec2;
use detour_crowd_rs::agent::{CrowdAgent, CrowdAgentParams, CrowdAgentState};

#[test]
fn test_agent_lifecycle_and_target() {
    let mut agent = CrowdAgent::new(0, Vec2::new(0.0, 0.0), CrowdAgentParams::default());
    assert_eq!(agent.state, CrowdAgentState::Waiting);

    agent.set_target(Vec2::new(10.0, 0.0));
    assert_eq!(agent.state, CrowdAgentState::Walking);
    assert_eq!(agent.distance_to_target(), Some(10.0));
    assert_eq!(agent.current_corner(), Some(Vec2::new(10.0, 0.0)));

    assert!(!agent.has_reached_target(0.5));

    // Move agent close to target
    agent.position = Vec2::new(9.8, 0.0);
    assert!(agent.has_reached_target(0.5));

    agent.stop();
    assert_eq!(agent.state, CrowdAgentState::Waiting);
    assert_eq!(agent.target_position, None);
}

#[test]
fn test_agent_corridor_corners() {
    let mut agent = CrowdAgent::new(1, Vec2::ZERO, CrowdAgentParams::default());
    let corners = vec![
        Vec2::new(5.0, 0.0),
        Vec2::new(5.0, 5.0),
        Vec2::new(10.0, 5.0),
    ];
    agent.set_corridor(corners);

    assert_eq!(agent.state, CrowdAgentState::Walking);
    assert_eq!(agent.current_corner(), Some(Vec2::new(5.0, 0.0)));

    // Advance to next corner
    assert!(agent.advance_corner());
    assert_eq!(agent.current_corner(), Some(Vec2::new(5.0, 5.0)));

    assert!(agent.advance_corner());
    assert_eq!(agent.current_corner(), Some(Vec2::new(10.0, 5.0)));

    // Reached final corner, cannot advance further
    assert!(!agent.advance_corner());
}
