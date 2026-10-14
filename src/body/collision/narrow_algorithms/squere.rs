pub struct SquareSquare;

impl NarrowPhaseAlgorithm<Square, Square> for SquareSquare {
    fn collide(a: (&Square, &Transform), b: (&Square, &Transform)) -> CollisionInfo {

    }
}

pub struct SquareRectangle;

impl NarrowPhaseAlgorithm<Square, Rectangle> for SquareRectangle {
    fn collide(a: (&Square, &Transform), b: (&Rectangle, &Transform)) -> CollisionInfo {

    }
}

pub struct SquareRhombus;

impl NarrowPhaseAlgorithm<Square, Rhombus> for SquareRhombus {
    fn collide(a: (&Square, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {

    }
}

pub struct SquarePolygon;

impl NarrowPhaseAlgorithm<Square, Polygon> for SquarePolygon {
    fn collide(a: (&Square, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {

    }
}

pub struct SquareTriangle;

impl NarrowPhaseAlgorithm<Square, Triangle> for SquareTriangle {
    fn collide(a: (&Square, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {

    }
}