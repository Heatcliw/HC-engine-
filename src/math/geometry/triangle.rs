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
        let a:Vec2 = [0.0, 0.0];
        let b:Vec2 = [self.c, 0.0];
        
        let x = (self.b * self.b + self.c * self.c - self.a * self.a) / (2.0 * self.c);

        let y = (self.b * self.b - x * x).max(SCALAR_ZERO).sqrt();

        let c:Vec2 = [x,y]; 

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

        let mut min = [f32::INFINITY; 2];
        let mut max = [f32::NEG_INFINITY; 2];

        for vertex in vertices {
            let x = vertex[0] + transform.center[0];
            let y = vertex[1] + transform.center[1];

            min[0] = min[0].min(x);
            min[1] = min[1].min(y);

            max[0] = max[0].max(x);
            max[1] = max[1].max(y);
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