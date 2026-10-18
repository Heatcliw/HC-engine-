use crate::fluids::sph::{
    ParticleStorage,
    SphConfig,
};
use crate::math::*;

pub fn compute_pressure(
    storage: &mut ParticleStorage,
    config: &SphConfig,
) {
    let st = config.stiffness;
    let d = config.rest_density;

    for p in &mut storage.particles {
        p.pressure = (st * (p.density - d)).max(SCALAR_ZERO)
    }
}