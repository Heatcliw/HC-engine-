use crate::math::*;
use rand::Rng;
impl HasArea for Polygon {
    fn area(&self) -> f32 {
        let mut sum = 0.0;
        let len = self.local_vectors.len();

        for i in 0..len {
            let current = &self.local_vectors[i];
            let next = &self.local_vectors[(i + 1) % len];
            sum += gaus(*current, *next)
        }

        sum.abs() / 2.0
    }
}

impl HasPerimeter for Polygon {
    fn perimeter(&self) -> f32 {
        let mut total_perimeter = 0.0;
        let len = self.local_vectors.len();

        for i in 0..len {
            let current = self.local_vectors[i];
            let next = self.local_vectors[(i + 1) % len];
            let side_vector = sub(next, current);
            total_perimeter += length(side_vector);
        }

        total_perimeter
    }
}

impl HasDiagonal for Polygon {
    type Output = Vec<f32>;
    fn diagonals(&self) -> Self::Output {
        let count = self.local_vectors.len();
        let mut diagon = Vec::new();
        if count < 4 {
            return diagon;
        }
        for i in 0..count {
            for j in (i + 2)..count {
                if i == 0 && j == count - 1 {continue;}
                let d = sub(self.local_vectors[i], self.local_vectors[j]);
                let len = length(d);
                diagon.push(len);
            }
            
        }
        diagon
    }
}

impl HasVertices for Polygon {
    type Output = Vec<Vec2>;

    fn vertices(&self) -> Self::Output {
        self.local_vectors.clone()
    }
}

impl HasAabb for Polygon {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let mut min = [f32::INFINITY; 2];
        let mut max = [f32::NEG_INFINITY; 2];

        for vertex in &self.vertices() {
            let scaled = [
                vertex[0] * transform.scale[0],
                vertex[1] * transform.scale[1],
            ];

            let rotated = rotate(
                scaled,
                transform.rotation
            );

            let world = add(
                rotated,
                transform.center
            );

            min[0] = min[0].min(world[0]);
            min[1] = min[1].min(world[1]);

            max[0] = max[0].max(world[0]);
            max[1] = max[1].max(world[1]);
        }

        Aabb { min, max }
    }
}

impl SamplePoint for Polygon {
    fn sample_point(&self, rng: &mut impl Rng) -> Vec2 {
        let vertices = &self.local_vectors;

        if vertices.len() < 3 {
            return VEC2_ZERO;
        }

        let i = rng.random_range(1..vertices.len() - 1);

        let a = vertices[0];
        let b = vertices[i];
        let c = vertices[i + 1];

        let mut u: f32 = rng.random();
        let mut v: f32 = rng.random();

        if u + v > 1.0 {
            u = 1.0 - u;
            v = 1.0 - v;
        }

        [
            a[0] + u * (b[0] - a[0]) + v * (c[0] - a[0]),
            a[1] + u * (b[1] - a[1]) + v * (c[1] - a[1]),
        ]
    }
}

