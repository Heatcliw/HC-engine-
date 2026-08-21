use crate::{body::collision::*, math::*};

pub struct CircleSquare;

impl NarrowPhaseAlgorithm for CircleSquare {
    fn collide(&self, a: &Collider, b: &Collider,) -> CollisionInfo {
        
    }
}