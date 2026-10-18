use crate::fluids::sph::{
    ParticleStorage,
    SpatialGrid,
    SphConfig,
};

use crate::fluids::sph::{get_neighbors};

use crate::math::*;

pub fn compute_density(
    storage: &mut ParticleStorage,
    grid: &SpatialGrid,
    config: &SphConfig,
){
    let h = config.smoothing_length;
    let h2 = h * h;
    for i in 0..storage.particles.len() {
        let pos_i = storage.particles[i].position;
        let neighbors = get_neighbors(grid, pos_i, h);
        let mut d = SCALAR_ZERO;
        for j in neighbors {
            let pos_j = storage.particles[j.0].position;
            let mass_j = storage.particles[j.0].mass;
            let r = sub(pos_i, pos_j);
            let r2 = dot(r, r);
            if r2 < h2 {
                d += mass_j * poly6(r2,h);
            }
            
        }
        storage.particles[i].density = d.max(EPSILON);
    }
}