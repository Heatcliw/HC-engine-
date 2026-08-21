use crate::math::*;

pub trait WorldVertices {
    fn world_vertices(&self, transform: &Transform) -> Vec<Vec2>;
}

impl WorldVertices for Polygon {
    fn world_vertices(&self, transform: &Transform) -> Vec<Vec2> {
        self.local_vectors
            .iter()
            .map(|vertex| {
                let scaled = [
                    vertex[0] * transform.scale[0],
                    vertex[1] * transform.scale[1],
                ];

                let rotated = rotate(
                    scaled,
                    transform.rotation
                );

                add(rotated, transform.center)
            })
            .collect()
    }
}