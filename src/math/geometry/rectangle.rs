use crate::math::{storage::Rectangle, *};
use rand::Rng;

impl HasArea for Square {
    fn area(&self) -> f32 {
        self.side * self.side
    }
}

impl HasPerimeter for Square {
    fn perimeter(&self) -> f32 {
        self.side * 4.0
    }
}

impl HasDiagonal for Square {
    type Output = f32;
    fn diagonals(&self) -> Self::Output {
        self.side * SQRT_2
    }
}

impl HasVertices for Square {
    type Output = [Vec2; 4];
    fn vertices(&self) -> Self::Output {
        let h = self.side * 0.5;

        [
            [-h , -h ], 
            [ h , -h ],
            [ h ,  h ], 
            [-h , h ]
        ]
    }
}
impl HasAabb for Square {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let c = transform.center;
        let h = self.side * 0.5;

        Aabb {
            min: [c[0] - h, c[1] - h],
            max: [c[0] + h, c[1] + h],
        }
    }
}
impl SamplePoint for Square {
    fn sample_point(&self, rng: &mut impl Rng) -> Vec2 {
        let h = self.side * 0.5;

        [
            rng.random_range(-h..h),
            rng.random_range(-h..h),
        ]
    }
}

impl HasArea for Rectangle {
    fn area(&self) -> f32 {
        self.height * self.width
    }
}

impl HasPerimeter for Rectangle{
    fn perimeter(&self) -> f32 {
        2.0 * (self.height + self.width)
    }
}

impl HasDiagonal for Rectangle {
    type Output = f32;
    fn diagonals(&self) -> Self::Output {
        let iw2 = self.height * self.height + self.width * self.width;
        iw2.sqrt()
    }
}

impl HasVertices for Rectangle {
    type Output = [Vec2; 4];
    fn vertices(&self) -> Self::Output {
        let w = self.width * 0.5;
        let h = self.height * 0.5;

        [
            [-w , -h ], 
            [ w , -h ],
            [ w ,  h ], 
            [-w , h ]
        ]
    }
}

impl HasAabb for Rectangle {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let hw = self.width * 0.5;
        let hh = self.height * 0.5;
        let c = transform.center;

        Aabb {
            min: [c[0] - hw, c[1] - hh],
            max: [c[0] + hw, c[1] + hh],
        }
    }
}

impl SamplePoint for Rectangle {
    fn sample_point(&self, rng: &mut impl Rng) -> Vec2 {
        [
            rng.random_range(-self.width * 0.5..self.width * 0.5),
            rng.random_range(-self.height * 0.5..self.height * 0.5),
        ]
    }
}

impl HasArea for Rhombus {
    fn area(&self) -> f32 {
        let height = self.side * self.angle.sin();
        self.side * height
    }
}

impl HasPerimeter for Rhombus {
    fn perimeter(&self) -> f32 {
        self.side * 4.0
    }
}

impl HasDiagonal for Rhombus  {
    type Output = [f32; 2];
    fn diagonals(&self) -> Self::Output {
        let a_2 = 0.5 * self.angle;
        let big = 2.0 * self.side * a_2.cos();
        let small = 2.0 * self.side * a_2.sin();
        [big, small]
    }
}

impl HasVertices for Rhombus {
    type Output = [Vec2; 4];
    fn vertices(&self) -> Self::Output {
        let [d1, d2] = self.diagonals();

        [
            [-d1 * 0.5, 0.0],
            [0.0, d2 * 0.5],
            [d1 * 0.5, 0.0],
            [0.0, -d2 * 0.5],
        ]
    }
}

impl HasAabb for Rhombus {
    fn aabb(&self, transform: &Transform) -> Aabb {
        let c = transform.center;

        let half_d1 = self.side * (self.angle * 0.5).cos();
        let half_d2 = self.side * (self.angle * 0.5).sin();

        Aabb {
            min: [c[0] - half_d1, c[1] - half_d2],
            max: [c[0] + half_d1, c[1] + half_d2],
        }
    }
}

impl SamplePoint for Rhombus {
    fn sample_point(&self, rng: &mut impl Rng) -> Vec2 {
        
        let [v0, v1, v2, v3] = self.vertices();

        let (a, b, c) = if rng.random::<bool>() {
            (v0, v1, v2)
        } else {
            (v0, v2, v3)
        };

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

#[derive(Debug, Clone, Copy)]
pub enum Diagonals {
    One(f32),
    Two([f32; 2]),
}

impl Shape {
    pub fn diagonals(&self) -> Option<Diagonals> {
        match self {
            Shape::Square(square) => {
                Some(Diagonals::One(square.diagonals()))
            }

            Shape::Rectangle(rect) => {
                Some(Diagonals::One(rect.diagonals()))
            }

            Shape::Rhombus(rhombus) => {
                Some(Diagonals::Two(rhombus.diagonals()))
            }

            _ => None,
        }
    }
}