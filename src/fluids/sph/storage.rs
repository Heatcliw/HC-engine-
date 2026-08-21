use crate::math::{VEC2_ZERO, Vec2};
use crate::material::storage::MaterialId;

/// Single SPH particle.
#[derive(Debug,Clone,Copy)]
pub struct Particle {
    /// Current particle position.
    pub position: Vec2,

    /// Current particle velocity.
    pub velocity: Vec2,

    /// Accumulated force for the current simulation step.
    pub force: Vec2,

    /// Computed density.
    pub density: f32,

    /// Computed pressure.
    pub pressure: f32,

    /// Particle mass.
    pub mass: f32,

    /// lifetime Particle
    pub lifetime: f32,

    /// Material used for physical properties.
    pub material: MaterialId,
}

impl Particle {
    /// Create a new particle with zero velocity and force.
    ///
    /// Density and pressure are initialized to `0.0`
    /// and will be computed by the SPH solver.
    pub fn new(position: Vec2, mass: f32, material: MaterialId) -> Self {
        Self {
            position,
            velocity: VEC2_ZERO,
            force: VEC2_ZERO,

            density: 0.0,
            pressure: 0.0,

            lifetime: 0.0,
            mass,
            material,
        }
    }
}

/// Storage for all SPH particles.
#[derive(Debug, Default)]
pub struct ParticleStorage {
    /// Contiguous storage of particles used by the solver.
    pub particles: Vec<Particle>,
}

impl ParticleStorage {
    /// Add a particle to the simulation storage.
    pub fn add_particle(&mut self, particle: Particle) -> usize {
        let index = self.particles.len();
        self.particles.push(particle);
        index
    }

    /// Remove all particles from the storage.
    pub fn clear(&mut self) {
        self.particles.clear();
    }

    /// Return the number of particles currently stored.
    pub fn len(&self) -> usize {
        self.particles.len()
    }

    /// Return `true` if the storage contains no particles.
    pub fn is_empty(&self) -> bool {
        self.particles.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &Particle> {
        self.particles.iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Particle> {
        self.particles.iter_mut()
    }
}