use crate::fluids::sph::{
    ParticleStorage,
    SpatialGrid,
    SphConfig,
};

use crate::fluids::sph::{get_neighbors};

use crate::math::*;

pub fn compute_pressure_force(
    storage: &mut ParticleStorage,
    grid: &SpatialGrid,
    config: &SphConfig,
) {
    let h = config.smoothing_length;
    let h2 = h * h;

    for i in 0..storage.particles.len() {
        let mut force = VEC2_ZERO;
        let p_i = storage.particles[i];
        let neighbors = get_neighbors(grid, p_i.position, h);
        for j in neighbors {
            let p_j = storage.particles[j.0];
            let r = sub(p_i.position, p_j.position);
            let r2 = dot(r, r);
            if r2 > EPSILON && r2 < h2{
                let den_i2 = p_i.density.max(EPSILON) * p_i.density.max(EPSILON);
                let den_j2 = p_j.density.max(EPSILON) * p_j.density.max(EPSILON);
                let pressure =
                    (p_i.pressure / den_i2) + (p_j.pressure / den_j2);
                let grad = spiky_grad(r, h);
                let f = scale( grad, -p_j.mass * pressure );

                force = add(force, f);
            } 

        }
        storage.particles[i].force = add(
            storage.particles[i].force,
            force
        );     
    }
}

pub fn compute_viscosity_force(
    storage: &mut ParticleStorage,
    grid: &SpatialGrid,
    config: &SphConfig,
) {
    let h = config.smoothing_length;
    let h2 = h * h;
    let viscosity = config.viscosity;

    for i in 0..storage.particles.len() {
        let mut force = VEC2_ZERO;
        let p_i = storage.particles[i];
        let neighbors = get_neighbors(grid, p_i.position, h);
        for j in neighbors {
            let p_j = storage.particles[j.0];
            let r = sub(p_i.position, p_j.position);
            let r2 = dot(r, r);

            if r2 > 0.0 && r2 < h2 {
                let distance = length(r);

                let kernel = viscosity_kernel(distance, h);

                let velocity_diff = sub( p_j.velocity, p_i.velocity );

                let coef = viscosity * p_j.mass * kernel / p_j.density.max(1e-6);

                let f = scale( velocity_diff, coef );

                force = add(force, f);
            }
        }

        storage.particles[i].force = add( storage.particles[i].force, force );
    }
}