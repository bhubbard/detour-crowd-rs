use glam::Vec2;
use detour_crowd_rs::agent::CrowdAgentParams;
use detour_crowd_rs::crowd::Crowd;

#[test]
fn test_crowd_agent_management() {
    let mut crowd = Crowd::new(5, 2.0);
    assert_eq!(crowd.active_agent_count(), 0);

    let id0 = crowd.add_agent(Vec2::ZERO, CrowdAgentParams::default()).unwrap();
    let _id1 = crowd.add_agent(Vec2::X, CrowdAgentParams::default()).unwrap();
    assert_eq!(crowd.active_agent_count(), 2);

    crowd.remove_agent(id0).unwrap();
    assert_eq!(crowd.active_agent_count(), 1);
    assert!(crowd.agent(id0).is_none());

    // Slot should be reused
    let id_new = crowd.add_agent(Vec2::Y, CrowdAgentParams::default()).unwrap();
    assert_eq!(id_new, id0);
    assert_eq!(crowd.active_agent_count(), 2);
}

#[test]
fn test_crowd_two_agents_crossing_avoidance() {
    let mut crowd = Crowd::new(10, 2.0);

    // Agent 0 starts at (-6, 0) moving to (+6, 0)
    let id0 = crowd.add_agent(Vec2::new(-6.0, 0.0), CrowdAgentParams::default()).unwrap();
    crowd.set_agent_target(id0, Vec2::new(6.0, 0.0)).unwrap();

    // Agent 1 starts at (+6, 0) moving to (-6, 0)
    let id1 = crowd.add_agent(Vec2::new(6.0, 0.0), CrowdAgentParams::default()).unwrap();
    crowd.set_agent_target(id1, Vec2::new(-6.0, 0.0)).unwrap();

    let mut min_distance = f32::MAX;

    // Simulate for 5 seconds (50 steps of 0.1s)
    let dt = 0.1;
    for _ in 0..60 {
        crowd.update(dt);

        let p0 = crowd.agent(id0).unwrap().position;
        let p1 = crowd.agent(id1).unwrap().position;
        let dist = p0.distance(p1);
        if dist < min_distance {
            min_distance = dist;
        }
    }

    // Both agents have radius 0.5 (combined radius = 1.0).
    // Thanks to RVO avoidance + separation force, they must never penetrate each other!
    assert!(
        min_distance >= 0.5,
        "Agents penetrated each other! Minimum distance was {:.3}",
        min_distance
    );

    // Verify both agents progressed towards their destinations
    let p0_final = crowd.agent(id0).unwrap().position;
    let p1_final = crowd.agent(id1).unwrap().position;
    assert!(p0_final.x > 0.0, "Agent 0 should have moved positive X");
    assert!(p1_final.x < 0.0, "Agent 1 should have moved negative X");
}

#[test]
fn test_crowd_corridor_with_boundary_walls() {
    let mut crowd = Crowd::new(10, 2.0);

    // Corridor bounded by two parallel walls: y = +1.5 and y = -1.5
    crowd.add_boundary_segment(Vec2::new(-10.0, 1.5), Vec2::new(10.0, 1.5));
    crowd.add_boundary_segment(Vec2::new(-10.0, -1.5), Vec2::new(10.0, -1.5));

    // Agent starts near the bottom wall at (-8.0, -1.2) heading to (8.0, 0.0)
    let params = CrowdAgentParams {
        radius: 0.4,
        ..Default::default()
    };
    let id = crowd.add_agent(Vec2::new(-8.0, -1.2), params).unwrap();
    crowd.set_agent_target(id, Vec2::new(8.0, 0.0)).unwrap();

    // Step simulation
    for _ in 0..40 {
        crowd.update(0.1);
        let pos = crowd.agent(id).unwrap().position;
        // Verify agent remains safely between the walls (-1.5 < y < 1.5)
        assert!(
            pos.y > -1.5 && pos.y < 1.5,
            "Agent penetrated boundary wall at y = {}",
            pos.y
        );
    }
}

#[test]
fn test_crowd_separation_among_clustering_agents() {
    let mut crowd = Crowd::new(10, 2.0);

    // Spawn 4 agents close together in a 0.2m cluster around (0, 0)
    let params = CrowdAgentParams {
        radius: 0.5,
        ..Default::default()
    };
    let id0 = crowd.add_agent(Vec2::new(-0.1, -0.1), params).unwrap();
    let _id1 = crowd.add_agent(Vec2::new(0.1, -0.1), params).unwrap();
    let _id2 = crowd.add_agent(Vec2::new(-0.1, 0.1), params).unwrap();
    let id3 = crowd.add_agent(Vec2::new(0.1, 0.1), params).unwrap();

    // Without movement targets, separation force alone should disperse them
    let initial_dist = crowd.agent(id0).unwrap().position.distance(crowd.agent(id3).unwrap().position);

    for _ in 0..20 {
        crowd.update(0.05);
    }

    let final_dist = crowd.agent(id0).unwrap().position.distance(crowd.agent(id3).unwrap().position);
    assert!(
        final_dist > initial_dist,
        "Separation force should push clustered agents apart: initial {}, final {}",
        initial_dist,
        final_dist
    );
}
