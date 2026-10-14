use crate::fluids::sph::config::SphConfig;
use crate::fluids::sph::storage::ParticleStorage;

use crate::fluids::sph::emitter::*;

use crate::fluids::sph::broad_phase::{
    build_grid, 
};

use crate::fluids::sph::steps::{
    compute_density,
    compute_pressure,
    compute_pressure_force,
    compute_viscosity_force,
    integrate,
};

pub struct SphSolver;

impl SphSolver {
    pub fn step (storage: &mut ParticleStorage, emitters: &mut Vec<Emitter>, config: &SphConfig, dt: f32,){
        let grid = build_grid(&storage.particles, config.smoothing_length);
        
        for emitter in emitters.iter_mut() {
            emitter.update(dt, &mut storage.particles);
        }

        compute_density(storage, &grid, config);
        compute_pressure(storage, config);
        compute_pressure_force(storage, &grid, config);
        compute_viscosity_force(storage, &grid, config);
        integrate(storage, config ,dt);
    }
}