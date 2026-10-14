use std::collections::HashMap;

use crate::math::Vec2;
use crate::fluids::sph::storage::Particle;
#[derive(Clone)]
pub struct ParticleId(pub usize);
/// Spatial grid used for neighbor search.
pub type SpatialGrid = HashMap<(i32, i32), Vec<ParticleId>>;

#[inline]
fn cell_coord(pos: Vec2, h: f32) -> (i32, i32) {
    assert!(
        h > 0.0,
        "smoothing_length must be greater than zero"
    );
    (
        (pos[0] / h).floor() as i32,
        (pos[1] / h).floor() as i32,
    )
}

/// Build a uniform grid for all particles.
pub fn build_grid(particles: &[Particle], h: f32) -> SpatialGrid {
    let mut grid: SpatialGrid = HashMap::new();

    for (i, particle) in particles.iter().enumerate() {
        let cell = cell_coord(particle.position, h);
        grid.entry(cell).or_default().push(ParticleId(i));
    }

    grid
}

/// Get candidate neighbor indices for a particle position.
///
/// Returns indices from the current cell and the 8 surrounding cells.
pub fn get_neighbors(
    grid: &SpatialGrid,
    position: Vec2,
    h: f32,
) -> Vec<ParticleId> {
    let (cx, cy) = cell_coord(position, h);

    let mut neighbors = Vec::new();

    for dx in -1..=1 {
        for dy in -1..=1 {
            if let Some(indices) = grid.get(&(cx + dx, cy + dy)) {
                neighbors.extend_from_slice(indices);
            }
        }
    }

    neighbors
}