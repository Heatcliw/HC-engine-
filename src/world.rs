use crate::math::*;

pub trait WorldVertices {
    fn world_vertices(&self, transform: &Transform) -> Vec<Vec2>;
}

impl WorldVertices for Polygon {
    fn world_vertices(&self, transform: &Transform) -> Vec<Vec2> {
        self.local_vectors
            .iter()
            .map(|vertex| {
                let scaled = Vec2 {
                   x: vertex.x * transform.scale.x,
                   y: vertex.y * transform.scale.y,
                };

                let rotated = rotate(
                    scaled,
                    transform.rotation
                );

                add(rotated, transform.center)
            })
            .collect()
    }
}