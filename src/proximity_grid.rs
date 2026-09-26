//! Spatial hashing proximity grid for $O(1)$ crowd neighbor lookup.
//!
//! Direct Rust port and optimization of RecastNavigation's `dtProximityGrid`.
//! Divides 2D continuous space into discrete cells using prime-based spatial hashing,
//! allowing hundreds or thousands of agents to quickly query their neighborhood
//! without $O(N^2)$ pairwise distance checks.

use glam::Vec2;
use serde::{Deserialize, Serialize};

/// Prime numbers for spatial hashing function.
const HASH_PRIME_X: i32 = 73856093;
const HASH_PRIME_Y: i32 = 19349663;

/// A node in the spatial hash bucket linked list.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
struct GridItem {
    id: usize,
    gx: i32,
    gy: i32,
    next: Option<usize>,
}

/// 2D Proximity Grid using spatial hashing.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProximityGrid {
    cell_size: f32,
    inv_cell_size: f32,
    bucket_count: usize,
    /// Head pointers for each bucket: bucket_index -> head item index in `items`.
    buckets: Vec<Option<usize>>,
    /// Contiguous pool of item records for cache locality.
    items: Vec<GridItem>,
}

impl ProximityGrid {
    /// Creates a new proximity grid with given cell size and bucket capacity.
    ///
    /// # Arguments
    /// * `cell_size` - Size of each square grid cell in world units (meters).
    /// * `bucket_count` - Number of hash buckets (prime or power of 2 recommended, e.g. 1024 or 2048).
    pub fn new(cell_size: f32, bucket_count: usize) -> Self {
        assert!(cell_size > 0.0, "Cell size must be positive");
        let buckets_len = bucket_count.next_power_of_two().max(16);
        Self {
            cell_size,
            inv_cell_size: 1.0 / cell_size,
            bucket_count: buckets_len,
            buckets: vec![None; buckets_len],
            items: Vec::with_capacity(1024),
        }
    }

    /// Default grid tuned for typical human crowds (cell size 2.5m, 1024 buckets).
    pub fn default_crowd() -> Self {
        Self::new(2.5, 1024)
    }

    /// Cell size in world units.
    #[inline]
    pub fn cell_size(&self) -> f32 {
        self.cell_size
    }

    /// Clears all entries from the grid for a new frame.
    #[inline]
    pub fn clear(&mut self) {
        self.buckets.fill(None);
        self.items.clear();
    }

    /// Spatial hash function converting integer grid cell coordinates $(gx, gy)$
    /// into a bucket index $0 \le \text{idx} < \text{bucket\_count}$.
    #[inline]
    fn hash_cell(&self, gx: i32, gy: i32) -> usize {
        let h = (gx.wrapping_mul(HASH_PRIME_X)) ^ (gy.wrapping_mul(HASH_PRIME_Y));
        (h as usize) & (self.bucket_count - 1)
    }

    /// Inserts an agent/item with a given ID into the grid cells overlapped by its bounding box.
    pub fn insert_box(&mut self, id: usize, min_pos: Vec2, max_pos: Vec2) {
        let min_x = (min_pos.x * self.inv_cell_size).floor() as i32;
        let min_y = (min_pos.y * self.inv_cell_size).floor() as i32;
        let max_x = (max_pos.x * self.inv_cell_size).floor() as i32;
        let max_y = (max_pos.y * self.inv_cell_size).floor() as i32;

        for gy in min_y..=max_y {
            for gx in min_x..=max_x {
                let bucket = self.hash_cell(gx, gy);
                let item_idx = self.items.len();
                let prev_head = self.buckets[bucket];
                self.items.push(GridItem {
                    id,
                    gx,
                    gy,
                    next: prev_head,
                });
                self.buckets[bucket] = Some(item_idx);
            }
        }
    }

    /// Inserts an agent/item with a given ID into the grid cells overlapped by its circle.
    #[inline]
    pub fn insert_circle(&mut self, id: usize, center: Vec2, radius: f32) {
        let r = Vec2::splat(radius);
        self.insert_box(id, center - r, center + r);
    }

    /// Queries all unique item IDs within an axis-aligned bounding box.
    pub fn query_box(&self, min_pos: Vec2, max_pos: Vec2, results: &mut Vec<usize>) {
        results.clear();
        let min_x = (min_pos.x * self.inv_cell_size).floor() as i32;
        let min_y = (min_pos.y * self.inv_cell_size).floor() as i32;
        let max_x = (max_pos.x * self.inv_cell_size).floor() as i32;
        let max_y = (max_pos.y * self.inv_cell_size).floor() as i32;

        for gy in min_y..=max_y {
            for gx in min_x..=max_x {
                let bucket = self.hash_cell(gx, gy);
                let mut curr = self.buckets[bucket];
                while let Some(idx) = curr {
                    let item = self.items[idx];
                    if item.gx == gx && item.gy == gy && !results.contains(&item.id) {
                        results.push(item.id);
                    }
                    curr = item.next;
                }
            }
        }
    }

    /// Queries all unique item IDs within a circular region centered at `center` with radius `radius`.
    #[inline]
    pub fn query_circle(&self, center: Vec2, radius: f32, results: &mut Vec<usize>) {
        let r = Vec2::splat(radius);
        self.query_box(center - r, center + r, results);
    }
}
