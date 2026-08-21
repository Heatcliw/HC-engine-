use crate::{fluids::sph::SphConfig, math::*};

pub const VEC2_ZERO: Vec2 = [0.0, 0.0];
pub const SCALAR_ZERO: f32 = 0.0;

pub const VEC2_X: Vec2 = [1.0, 0.0];


pub const MAX: f32 = std::f32::MAX;
pub const EPSILON: f32 = 1e-6;

pub const TAU: f32 = std::f32::consts::TAU;

pub const PI: f32 = std::f32::consts::PI;

pub const SQRT_2: f32 = std::f32::consts::SQRT_2;

pub static CONFIG: SphConfig = SphConfig {
    smoothing_length: 16.0,
    stiffness: 40.0,
    rest_density: 1.0,
    viscosity: 0.08,

    gravity: [0.0, -98.0], // если работаешь в пикселях
};