use crate::{body::collision::*, math::*};

pub struct CircleTriangle;

impl NarrowPhaseAlgorithm for CircleTriangle {
    fn collide(&self, a: &Collider, b: &Collider,) -> CollisionInfo {
        
    }
}