use crate::math::*;
use crate::material::*;
use crate::fluids::sph::Particle;

use rand::Rng;

use crate::math::geometry::*;

pub struct Emitter {
    pub shape: Shape,
    pub direction: Vec2,

    pub speed: f32,
    pub particles_per_second: f32,
    pub spread: f32,

    pub particle_mass: f32,
    pub particle_lifetime: f32,
    pub material: MaterialId,

    pub max_particles: usize,

    pub accumulator: f32,
    pub active: bool,
}

impl Emitter {
    pub fn sample_position(&self, rng: &mut impl Rng) -> Vec2 {
        match &self.shape {
            Shape::Point(s) => s.sample_point(rng),
            Shape::MultiPoint(s) => s.sample_point(rng),
            Shape::Line(s) => s.sample_point(rng),
            Shape::Circle(s) => s.sample_point(rng),
            Shape::Ellipse(s) => s.sample_point(rng),
            Shape::Rectangle(s) => s.sample_point(rng),
            Shape::Rhombus(s) => s.sample_point(rng),
            Shape::Polygon(s) => s.sample_point(rng),
            Shape::Triangle(s) => s.sample_point(rng)
        }
    }
    
    pub fn spawn_particle(&self) -> Particle {
        let mut rng = rand::rng();

        let position = self.sample_position(&mut rng);

        let angle = rng.random_range(-self.spread..=self.spread);
        let (sin, cos) = angle.sin_cos();

        let dir = [
            self.direction[0] * cos - self.direction[1] * sin,
            self.direction[0] * sin + self.direction[1] * cos,
        ];

        let speed = self.speed * rng.random_range(0.8..1.2);
        Particle {
            position,
            velocity: scale(dir, speed),
            force: VEC2_ZERO,

            density: SCALAR_ZERO,
            pressure: SCALAR_ZERO,

            mass: self.particle_mass,
            lifetime: self.particle_lifetime,
            material: self.material,
        }
    }
    pub fn from_material(
        shape: Shape,
        material: &Material,
    ) -> Self {
        let fluid = material.fluid.as_ref();

        let viscosity_factor = fluid.map(|f| (1.0 - f.viscosity * 0.3).max(0.05)).unwrap_or(1.0);
        
        Self {
            shape,
            direction: [0.0, -1.0],

            speed: 120.0,

            // более вязкие жидкости текут медленнее
            particles_per_second: 200.0 * viscosity_factor,

            spread: 0.12,

            // масса зависит от относительной плотности
            particle_mass: material.physics.density.max(0.1),

            particle_lifetime: 1000.0,

            material: material.id,

            max_particles: 20_000,

            accumulator: 0.0,
            active: true,
        }
    }

    pub fn update(
        &mut self,
        dt: f32,
        particles: &mut Vec<Particle>,
    ) {
        if !self.active {
            return;
        }

        self.accumulator += self.particles_per_second * dt;

        while self.accumulator >= 1.0 {
            if particles.len() >= self.max_particles {
                break;
            }

            self.accumulator -= 1.0;

            particles.push(self.spawn_particle());
        }
    }
}