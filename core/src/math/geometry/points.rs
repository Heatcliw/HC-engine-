

use crate::math::*;

impl HasAabb for Point {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let p = Vec2{
            x: self.positions.x + transform.center.x,
            y: self.positions.y + transform.center.y,
        };

        Aabb {
            min: p,
            max: p,
        }
    }
}

impl SamplePoint for Point {
    fn sample_point(&self, _: &mut impl rand::prelude::Rng) -> Vec2 {
        Vec2 { 
            x: self.positions.x, 
            y: self.positions.y 
        }
    }
}

impl HasAabb for MultiPoint {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let mut min = Vec2{ x: INFINITY,y: INFINITY };
        let mut max = Vec2{ x: INFINITY,y: INFINITY };

        for p in &self.positions {
            let x = p.x + transform.center.x;
            let y = p.y + transform.center.y;

            min.x = min.x.min(x);
            min.y = min.y.min(y);

            max.x = max.x.max(x);
            max.y = max.y.max(y);
        }

        Aabb { min, max }
    }
}

impl SamplePoint for MultiPoint {
    fn sample_point(&self, rng: &mut impl rand::Rng) -> Vec2 {
        self.positions[rng.random_range(0..self.positions.len())]
    }
}

impl HasAabb for Line {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let start = Vec2 {
            x: self.start.x + transform.center.x,
            y: self.start.y + transform.center.y,
        };

        let end = Vec2 {
            x: self.end.x + transform.center.x,
            y: self.end.y + transform.center.y,
        };

        Aabb {
            min: Vec2 {
                x: start.x.min(end.x),
                y: start.y.min(end.y),
            },
            max: Vec2 {
                x: start.x.max(end.x),
                y: start.y.max(end.y),
            },
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

        Vec2 {
            x: self.start.x + (self.end.x - self.start.x) * t,
            y: self.start.y + (self.end.y - self.start.y) * t,
        }
    }
}