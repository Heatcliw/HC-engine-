use crate::{body::collision::*, math::*};

pub struct CircleRectangle;

impl NarrowPhaseAlgorithm for CircleRectangle {
    fn collide(&self, a: &Collider, b: &Collider,) -> CollisionInfo {
        
    }
}