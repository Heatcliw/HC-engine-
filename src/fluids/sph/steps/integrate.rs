use crate::fluids::sph::{storage::ParticleStorage, config::SphConfig};
use crate::math::*;


pub fn integrate(
    storage: &mut ParticleStorage,
    config: &SphConfig,
    dt: f32,
) {
    for particle in &mut storage.particles {

        // a = F/m + g
        let acceleration = add(
            scale(particle.force, 1.0 / particle.mass.max(EPSILON)),
        config.gravity,
        );

        particle.velocity = add(
            particle.velocity,
            scale(acceleration, dt)
        );

        particle.position = add(
            particle.position,
            scale(particle.velocity, dt)
        );

        // сброс силы для следующего шага
        particle.force = VEC2_ZERO;
    }
}