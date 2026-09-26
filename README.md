# detour-crowd-rs

[![Crates.io](https://img.shields.io/badge/crates.io-v0.1.0-orange.svg)](https://crates.io)
[![Documentation](https://docs.rs/detour-crowd-rs/badge.svg)](https://docs.rs/detour-crowd-rs)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](LICENSE)

A pure Rust port of **DetourCrowd** from [RecastNavigation](https://github.com/recastnavigation/recastnavigation). Features Reciprocal Velocity Obstacles (RVO), adaptive velocity space sampling, spatial hashing proximity grid for $O(1)$ neighbor queries, corridor navigation, and boundary / separation steering forces.

Designed for game engines and open-world games (such as Bevy RPGs) requiring real-time collision-free steering for hundreds of simultaneous pedestrians and NPCs.

---

## Key Features

- **Local Obstacle Avoidance (RVO / Detour Query)**:
  - **Reciprocal Velocity Obstacles**: Each agent reciprocally adjusts its velocity to avoid dynamic neighbors without oscillations.
  - **Velocity Space Sampling**: Multi-ring polar and adaptive refinement sampling matching Recast's `dtObstacleAvoidanceQuery`.
  - **Comprehensive Penalty Evaluation**: Evaluates time of impact (TOI) against dynamic agent circles and static segment boundaries, deviation from desired velocity, current velocity acceleration smoothness, and side preference (emergent two-way lane formation).
- **Spatial Hashing Proximity Grid (`ProximityGrid`)**:
  - Direct Rust port of Recast's `dtProximityGrid`.
  - Prime-based spatial hashing dividing continuous space into discrete cells.
  - $O(1)$ neighborhood lookups eliminating $O(N^2)$ pairwise distance checks across large crowds.
- **Corridor Navigation & Path Steering**:
  - Follows waypoints and corners along navigation corridors.
  - Smooth corner advancement and target arrival deceleration.
- **Boundary & Separation Forces**:
  - Static obstacle line segments (walls, barriers, navmesh edges) generate repulsive boundary steering forces.
  - Soft separation forces maintain comfortable personal space between agents in dense clusters.
- **Crowd Agent Pipeline**:
  - Agent states: `Invalid`, `Waiting`, `Walking`, `Active`.
  - Acceleration limits, maximum speeds, collision query ranges, and avoidance parameter profiles.

---

## Quick Start

Add to your `Cargo.toml`:

```toml
[dependencies]
detour-crowd-rs = "0.1"
glam = "0.29"
```

### Basic Crowd Simulation

```rust
use glam::Vec2;
use detour_crowd_rs::{Crowd, CrowdAgentParams};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Create a crowd supporting up to 100 agents with 2.5m grid cells
    let mut crowd = Crowd::new(100, 2.5);

    // 2. Add static boundary walls (e.g. hallway or street borders)
    crowd.add_boundary_segment(Vec2::new(-20.0, 2.0), Vec2::new(20.0, 2.0));
    crowd.add_boundary_segment(Vec2::new(-20.0, -2.0), Vec2::new(20.0, -2.0));

    // 3. Spawn agents walking towards each other
    let agent_a = crowd.add_agent(Vec2::new(-10.0, 0.0), CrowdAgentParams::default())?;
    crowd.set_agent_target(agent_a, Vec2::new(10.0, 0.0))?;

    let agent_b = crowd.add_agent(Vec2::new(10.0, 0.0), CrowdAgentParams::default())?;
    crowd.set_agent_target(agent_b, Vec2::new(-10.0, 0.0))?;

    // 4. Update simulation at 60 Hz
    let dt = 1.0 / 60.0;
    for _ in 0..300 {
        crowd.update(dt);
    }

    if let Some(agent) = crowd.agent(agent_a) {
        println!("Agent A position: {:?}", agent.position);
    }

    Ok(())
}
```

---

## Pipeline Overview

```
                      ┌──────────────────────────────────────┐
                      │        Clear & Update Grid           │
                      │     `ProximityGrid.insert_circle`    │
                      └──────────────────┬───────────────────┘
                                         │
                                         ▼
                      ┌──────────────────────────────────────┐
                      │       Query Local Neighbors          │
                      │    `ProximityGrid.query_circle`      │
                      └──────────────────┬───────────────────┘
                                         │
                                         ▼
                      ┌──────────────────────────────────────┐
                      │       Corridor / Waypoint Steering   │
                      │    Calculate desired target heading  │
                      └──────────────────┬───────────────────┘
                                         │
                                         ▼
                      ┌──────────────────────────────────────┐
                      │   Separation & Boundary Avoidance    │
                      │     Blend soft repulsive forces      │
                      └──────────────────┬───────────────────┘
                                         │
                                         ▼
                      ┌──────────────────────────────────────┐
                      │      RVO Velocity Space Sampling     │
                      │   Find optimal collision-free vel    │
                      └──────────────────┬───────────────────┘
                                         │
                                         ▼
                      ┌──────────────────────────────────────┐
                      │        Integrate State (dt)          │
                      │  Clamp acceleration & advance pos    │
                      └──────────────────────────────────────┘
```

---

## License

Dual-licensed under either:

- MIT License ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)

at your option.
