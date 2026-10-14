pub struct RectangleRectangle;

impl NarrowPhaseAlgorithm<Rectangle, Rectangle> for RectangleRectangle {
    fn collide(a: (&Rectangle, &Transform), b: (&Rectangle, &Transform)) -> CollisionInfo {

    }
}

pub struct RectangleRhombus;

impl NarrowPhaseAlgorithm<Rectangle, Rhombus> for RectangleRhombus {
    fn collide(a: (&Rectangle, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {

    }
}

pub struct RectanglePolygon;

impl NarrowPhaseAlgorithm<Rectangle, Polygon> for RectanglePolygon {
    fn collide(a: (&Rectangle, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {

    }
}

pub struct RectangleTriangle;

impl NarrowPhaseAlgorithm<Rectangle, Triangle> for RectangleTriangle {
    fn collide(a: (&Rectangle, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {

    }
}