use crate::math::*;

impl HasAabb for Point {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let p = [
            self.0[0] + transform.center[0],
            self.0[1] + transform.center[1],
        ];

        Aabb {
            min: p,
            max: p,
        }
    }
}

impl SamplePoint for Point {
    fn sample_point(&self, _: &mut impl rand::prelude::Rng) -> Vec2 {
        self.0
    }
}

impl HasAabb for MultiPoint {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let mut min = [f32::INFINITY; 2];
        let mut max = [f32::NEG_INFINITY; 2];

        for p in &self.0 {
            let x = p[0] + transform.center[0];
            let y = p[1] + transform.center[1];

            min[0] = min[0].min(x);
            min[1] = min[1].min(y);

            max[0] = max[0].max(x);
            max[1] = max[1].max(y);
        }

        Aabb { min, max }
    }
}

impl SamplePoint for MultiPoint {
    fn sample_point(&self, rng: &mut impl rand::Rng) -> Vec2 {
        self.0[rng.random_range(0..self.0.len())]
    }
}

impl HasAabb for Line {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let start = [
            self.start[0] + transform.center[0],
            self.start[1] + transform.center[1],
        ];

        let end = [
            self.end[0] + transform.center[0],
            self.end[1] + transform.center[1],
        ];

        Aabb {
            min: [
                start[0].min(end[0]),
                start[1].min(end[1]),
            ],
            max: [
                start[0].max(end[0]),
                start[1].max(end[1]),
            ],
        }
    }
}

impl SamplePoint for Line {
    fn sample_point(&self, rng: &mut impl rand::Rng) -> Vec2 {
        let i = rng.random_range(0..self.count) as f32;

        let t = if self.count > 1 {
            i / (self.count as f32 - 1.0)
        } else {
            0.0
        };

        [
            self.start[0] + (self.end[0] - self.start[0]) * t,
            self.start[1] + (self.end[1] - self.start[1]) * t,
        ]
    }
}