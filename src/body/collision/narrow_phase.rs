use std::vec;

use crate::{body::collision::storage::Collider,world::*, math::*};
impl Shape {
    pub fn kind(&self) -> ShapeKind {
        match self {
            Shape::Point(_) => ShapeKind::Point,
            Shape::MultiPoint(_) => ShapeKind::MultiPoint,
            Shape::Line(_) => ShapeKind::Line,

            Shape::Circle(_) => ShapeKind::Circle,
            Shape::Ellipse(_) => ShapeKind::Ellipse,
            Shape::Rectangle(_) => ShapeKind::Rectangle,
            Shape::Rhombus(_) => ShapeKind::Rhombus,
            Shape::Polygon(_) => ShapeKind::Polygon,
            Shape::Triangle(_) => ShapeKind::Triangle,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum CollisionRelation {
    Outside,
    Touching,
    Overlapping,
    Inside,
}
#[derive(Debug, Clone)]
pub struct CollisionInfo {
    pub relation: CollisionRelation,
    pub contact: Option<Contact>,
}

#[derive(Debug, Clone)]
pub struct Contact {
    pub depth: f32,
    pub normal: Vec2,
    pub points: Vec<Vec2>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ShapePair(pub ShapeKind, pub ShapeKind);

impl ShapePair {
    pub fn new(a: ShapeKind, b: ShapeKind) -> Self {
        if (a as u8) <= (b as u8) {
            Self(a, b)
        } else {
            Self(b, a)
        }
    }
}

pub trait NarrowPhaseAlgorithm {
    fn collide(&self, a: &Collider, b: &Collider,) -> CollisionInfo;
}




