use std::collections::HashMap;

use crate::body::collision::storage::{ColliderId, Collider};
use crate::math::Vec2;

/// Spatial hash grid used for broad-phase collision detection.
pub type SpatialGrid = HashMap<(i32, i32), Vec<ColliderId>>;

#[inline]
fn cell_coord(pos: Vec2, cell_size: f32) -> [i32; 2] {
    assert!(
        cell_size > 0.0,
        "cell_size must be greater than zero"
    );
    [
        (pos[0] / cell_size).floor() as i32,
        (pos[1] / cell_size).floor() as i32,
    ]
}

/// Build a uniform spatial grid for all colliders.
pub fn build_grid(colliders: &[Collider], cell_size: f32) -> SpatialGrid {
    let mut grid: SpatialGrid = HashMap::new();

    for collider in colliders {
        
        let aabb = collider.compute_aabb();
        
        let min = cell_coord(aabb.min, cell_size);
        let max = cell_coord(aabb.max, cell_size);

        for x in min[0]..=max[0] {
            for y in min[1]..=max[1] {
                grid.entry((x, y))
                    .or_default()
                    .push(collider.id);
            }
        }       
    }
    grid
}

/// Returns candidate colliders from the cell containing `position`
/// and the 8 neighboring cells.
pub fn get_neighbors(
    grid: &SpatialGrid,
    position: Vec2,
    cell_size: f32,
) -> Vec<ColliderId> {
    let [cx, cy] = cell_coord(position, cell_size);

    let mut neighbors = Vec::new();

    for dx in -1..=1 {
        for dy in -1..=1 {
            if let Some(indices) = grid.get(&(cx + dx, cy + dy)) {
                neighbors.extend_from_slice(indices);
            }
        }
    }
    neighbors.sort_unstable();
    neighbors.dedup();
    neighbors
}