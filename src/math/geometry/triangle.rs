use crate::math::*;

impl HasArea for Triangle {
    fn area(&self) -> f32 {
        if self.a + self.b <= self.c
            || self.a + self.c <= self.b
            || self.b + self.c <= self.a
        {
            return SCALAR_ZERO;
        }
        
        let s = self.perimeter() * 0.5;

        (s * (s - self.a) * (s - self.b) * (s - self.c)).sqrt()
    }
}

impl HasPerimeter for Triangle {
    fn perimeter(&self) -> f32 {
        self.a + self.b + self.c
    }
}

impl HasHeight for Triangle {
    type Output = [f32; 3];
    fn heights(&self) -> Self::Output {
        let area = self.area();
        [(2.0 * area) / self.a,(2.0 * area) / self.b,(2.0 * area) / self.c ]
    }
}

impl HasVertices for Triangle {
    type Output = [Vec2; 3]; 
    fn vertices(&self) -> Self::Output{
        let a = Vec2 {x: 0.0, y: 0.0};
        let b = Vec2 {x: self.c, y: 0.0};
        
        let x = (self.b * self.b + self.c * self.c - self.a * self.a) / (2.0 * self.c);

        let y = (self.b * self.b - x * x).max(SCALAR_ZERO).sqrt();

        let c = Vec2 { x, y }; 

        [
            a,
            b,
            c
        ]
    }
}

impl HasAabb for Triangle {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let vertices = self.vertices();

        let mut min = Vec2{ x: INFINITY,y: INFINITY };
        let mut max = Vec2{ x: NEG_INFINITY,y: NEG_INFINITY };

        for vertex in vertices {
            let x = vertex.x + transform.center.x;
            let y = vertex.y + transform.center.y;

            min.x = min.x.min(x);
            min.y = min.y.min(y);

            max.x = max.x.max(x);
            max.y = max.y.max(y);
        }

        Aabb { min, max }
    }
}

impl SamplePoint for Triangle {
    fn sample_point(&self, rng: &mut impl Rng) -> Vec2 {
        let [a, b, c] = self.vertices();

        let mut u = rng.random::<f32>();
        let mut v = rng.random::<f32>();

        if u + v > 1.0 {
            u = 1.0 - u;
            v = 1.0 - v;
        }

        let w = 1.0 - u - v;

        add(
            add(scale(a, w), scale(b, u)),
            scale(c, v),
        )
    }
}

// impl SampleBoundaryPoint for Triangle {
//     fn sample_boundary_point(&self, rng: &mut impl Rng) -> Vec2;
// }