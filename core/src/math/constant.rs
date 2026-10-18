use crate::{math::*};

pub const VEC2_ZERO: Vec2 = Vec2 {
    x: 0.0,
    y: 0.0
};
pub const SCALAR_ZERO: f32 = 0.0;

pub const VEC2_X: Vec2 = Vec2 {
    x: 1.0,
    y: 0.0
};
pub const VEC2_Y: Vec2 = Vec2 {
    x: 0.0,
    y: 1.0
};

pub const INFINITY: f32 = std::f32::INFINITY;

pub const NEG_INFINITY:f32 = std::f32::NEG_INFINITY;

pub const MAX: f32 = std::f32::MAX;
pub const EPSILON: f32 = 1e-6;

pub const TAU: f32 = std::f32::consts::TAU;

pub const PI: f32 = std::f32::consts::PI;

pub const SQRT_2: f32 = std::f32::consts::SQRT_2;