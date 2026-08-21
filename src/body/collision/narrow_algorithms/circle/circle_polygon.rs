use crate::{body::collision::*,world::*, math::*};

pub struct CirclePolygon;

impl NarrowPhaseAlgorithm for CirclePolygon  {
    fn collide(&self, a: &Collider, b: &Collider,) -> CollisionInfo
    {
        let cir_a = match &a.geometry {
            Shape::Circle(c) => c,
            _ => {
                return CollisionInfo { relation: CollisionRelation::Outside, contact: None };
            }
        };
        let pol_b = match &b.geometry {
            Shape::Polygon(p) => p,
            _ => {
                return CollisionInfo { relation: CollisionRelation::Outside, contact: None };
            }
        };

        let vertices = pol_b.world_vertices(&b.transform);   

        let mut closest_point = VEC2_ZERO;
        let mut min_distance = MAX;

        for i in 0..vertices.len() {
            let start = vertices[i];
            let end = vertices[(i + 1) % vertices.len()];

            let point = closest_point_segment(
                a.transform.center,
                start,
                end,
            );

            let distance = length(
                sub(a.transform.center, point)
            );

            if distance < min_distance {
                min_distance = distance;
                closest_point = point;
            }
        }

        if min_distance > cir_a.radius {
            return CollisionInfo { relation: CollisionRelation::Outside, contact: None };
        }

        let normal = normalize(sub(a.transform.center, closest_point));

        let contact = Contact {
            depth: cir_a.radius - min_distance,
            normal,
            points: vec![closest_point]
        };

        if (min_distance - cir_a.radius).abs() < EPSILON {
            CollisionInfo {
                relation: CollisionRelation::Touching,
                contact: Some(contact),
            }
        } else {
            CollisionInfo {
                relation: CollisionRelation::Overlapping,
                contact: Some(contact),
            }
        }
    }
}