use crate::math::*;
use serde::{Serialize, Deserialize};

#[derive(
    Default,
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    Serialize,
    Deserialize,
)]
pub struct ColliderId (pub usize);
#[derive(Debug,Clone)]
pub struct Collider {
    pub id: ColliderId,
    pub name: String,
    pub transform: Transform,
    pub geometry: Shape,
    pub trigger: bool,
}

impl Collider {
    pub fn compute_aabb(&self) -> Aabb {
        match &self.geometry {
            Shape::Point(s) => s.aabb(&self.transform),
            Shape::MultiPoint(s) => s.aabb(&self.transform),
            Shape::Line(s) => s.aabb(&self.transform),
            Shape::Circle(s) => s.aabb(&self.transform),
            Shape::Rectangle(s) => s.aabb(&self.transform),
            Shape::Rhombus(s) => s.aabb(&self.transform),
            Shape::Ellipse(s) => s.aabb(&self.transform),
            Shape::Polygon(s) => s.aabb(&self.transform),
            _ => todo!(),
        }
    }
}