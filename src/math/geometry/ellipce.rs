use crate::math::*;


impl HasArea for Ellipse  {
    fn area(&self) -> f32 {
        PI * self.radius_x * self.radius_y
    }
}

impl HasPerimeter for Ellipse {
    fn perimeter(&self) -> f32 {
        let a = self.radius_x;
        let b = self.radius_y;
        let h = ((a - b) * (a - b))/ ((a + b) * (a + b)); 
        let sqrt4_3h = (4.0-3.0*h).sqrt();
        PI * (a + b) * (1.0 + (3.0*h)/(10.0 + sqrt4_3h) )
    }
}

impl HasDiameter for Ellipse {
    type Output = [f32; 2];
    fn diameters(&self) -> Self::Output {
        let d_x =self.radius_x * 2.0;
        let d_y = self.radius_y * 2.0;
        [d_x, d_y]
    }

}
// TODO: Take transform.rotation into account.
impl HasAabb for Ellipse {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let c = transform.center;
        let rx = self.radius_x;
        let ry = self.radius_y;

        Aabb {
            min: Vec2 { x: c.x - rx, y: c.y - ry },
            max: Vec2 { x: c.x + rx, y: c.y + ry },
        }
    }
}

impl HasRadius for Ellipse {
    type Output = f32;
    fn radius(&self) -> Self::Output {
        let theta = self.delta.y.atan2(self.delta.x);
        let cos_t = theta.cos();
        let sin_t = theta.sin();

        (self.radius_x * self.radius_y) / ((self.radius_y * cos_t).powi(2) + (self.radius_x * sin_t).powi(2)).sqrt() 
    }
}

impl HasArea for Circle {
    fn area(&self) -> f32 {
        PI * self.radius * self.radius
    }
}

impl HasPerimeter for Circle {
    fn perimeter(&self) -> f32 {
        2.0 * PI * self.radius
    }
}

impl HasDiameter for Circle {
    type Output = f32;
    fn diameters(&self) -> Self::Output {
        self.radius * 2.0
    }
}

impl HasAabb for Circle {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let c = transform.center;
        let r = self.radius;

        Aabb {
            min: Vec2 {x: c.x - r, y: c.y - r},
            max: Vec2 {x:  c.x + r, y: c.y + r},
        }
    }
}

impl HasRadius for Circle {
    type Output = f32;
    fn radius(&self) -> Self::Output {
        self.radius
    }
}

impl SamplePoint for Circle {
    fn sample_point(&self, rng: &mut impl Rng) -> Vec2 {
        let angle = rng.random_range(0.0..TAU);
        let r = self.radius * rng.random::<f32>().sqrt();

        Vec2 {
           x: angle.cos() * r,
           y: angle.sin() * r,
        }
    }
}

impl SamplePoint for Ellipse {
    fn sample_point(&self, rng: &mut impl Rng) -> Vec2 {
        let angle = rng.random_range(0.0..TAU);
        let r = rng.random::<f32>().sqrt();

        Vec2 {
           x: angle.cos() * self.radius_x * r,
           y: angle.sin() * self.radius_y * r,
        }
    }
}