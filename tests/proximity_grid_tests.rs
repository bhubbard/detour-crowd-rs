use glam::Vec2;
use detour_crowd_rs::proximity_grid::ProximityGrid;

#[test]
fn test_proximity_grid_insert_and_query() {
    let mut grid = ProximityGrid::new(2.0, 64);
    let mut results = Vec::new();

    // Insert 3 items
    grid.insert_circle(1, Vec2::new(0.0, 0.0), 0.5);
    grid.insert_circle(2, Vec2::new(1.0, 0.0), 0.5);
    grid.insert_circle(3, Vec2::new(10.0, 10.0), 0.5); // far away

    // Query near origin with radius 2.0
    grid.query_circle(Vec2::new(0.0, 0.0), 2.0, &mut results);

    assert!(results.contains(&1));
    assert!(results.contains(&2));
    assert!(!results.contains(&3));

    // Clear grid
    grid.clear();
    grid.query_circle(Vec2::new(0.0, 0.0), 2.0, &mut results);
    assert!(results.is_empty());
}

#[test]
fn test_proximity_grid_large_scale() {
    let mut grid = ProximityGrid::new(5.0, 1024);
    let mut results = Vec::new();

    // Insert 500 agents scattered on a 100x100 grid
    for id in 0..500 {
        let x = ((id % 25) as f32) * 4.0;
        let y = ((id / 25) as f32) * 5.0;
        grid.insert_circle(id, Vec2::new(x, y), 0.5);
    }

    // Query point at (20, 20) with radius 6.0
    grid.query_circle(Vec2::new(20.0, 20.0), 6.0, &mut results);
    assert!(!results.is_empty());

    // Verify all returned results are within reasonable grid neighborhood
    for &id in &results {
        let x = ((id % 25) as f32) * 4.0;
        let y = ((id / 25) as f32) * 5.0;
        let pos = Vec2::new(x, y);
        let dist = pos.distance(Vec2::new(20.0, 20.0));
        // Cell boundary tolerance: dist should be within radius + 2 * cell_size
        assert!(dist <= 6.0 + 2.0 * 5.0);
    }
}
