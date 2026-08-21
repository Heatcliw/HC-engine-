use crate::{body::collision::*, math::*};
pub struct CircleCircle;

impl NarrowPhaseAlgorithm for CircleCircle {
    fn collide(&self,a: &Collider,b:&Collider) -> CollisionInfo {
        let cir_a = match &a.geometry {
            Shape::Circle(c) => c,
            _ => {
                return CollisionInfo { relation: CollisionRelation::Outside, contact: None };
            }
        };
        let cir_b = match &b.geometry {
            Shape::Circle(c) => c,
            _ => {
                return CollisionInfo { relation: CollisionRelation::Outside, contact: None };
            }
        };
        let delta = sub(b.transform.center, a.transform.center);
        let distance = length(delta);
        let sum_rad = cir_a.radius + cir_b.radius;

        if distance > sum_rad {
            return CollisionInfo { relation: CollisionRelation::Outside, contact: None };
        }

        let normal = if distance > SCALAR_ZERO 
        {
            div(delta,distance)
        } else {
            VEC2_X
        };

        let point_a = add(
            a.transform.center,
            scale(normal, cir_a.radius)
        );

        let point_b = sub(
            b.transform.center,
            scale(normal, cir_b.radius)
        );

        let point = scale(
            add(point_a, point_b),
            0.5
        );

        let contact = Contact {
            depth: sum_rad - distance,
            normal,
            points: vec![point]
        };

        if distance + cir_b.radius <= cir_a.radius - EPSILON || distance + cir_a.radius <= cir_b.radius - EPSILON
        {
            return CollisionInfo { relation: CollisionRelation::Inside, contact: Some(contact) };
        }

        if (distance - sum_rad).abs() < EPSILON 
        {
            CollisionInfo { relation: CollisionRelation::Touching, contact: Some(contact) }
        } else {
            CollisionInfo { relation: CollisionRelation::Overlapping, contact: Some(contact) }
        }
    }
}

