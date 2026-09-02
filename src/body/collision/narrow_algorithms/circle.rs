pub struct CircleCircle;

impl NarrowPhaseAlgorithm<Circle, Circle> for CircleCircle {
    fn collide(a: (&Circle, &Transform), b: (&Circle, &Transform)) -> CollisionInfo {

    }
}

pub struct CircleEllipse;

impl NarrowPhaseAlgorithm<Circle, Ellipse> for CircleEllipse {
    fn collide(a: (&Circle, &Transform), b: (&Ellipse, &Transform)) -> CollisionInfo {

    }
}

pub struct CircleSquare;

impl NarrowPhaseAlgorithm<Circle, Square> for CircleSquare {
    fn collide(a: (&Circle, &Transform), b: (&Square, &Transform)) -> CollisionInfo {

    }
}

pub struct CircleRectangle;

impl NarrowPhaseAlgorithm<Circle, Rectangle> for CircleRectangle {
    fn collide(a: (&Circle, &Transform), b: (&Rectangle, &Transform)) -> CollisionInfo {

    }
}

pub struct CircleRhombus;

impl NarrowPhaseAlgorithm<Circle, Rhombus> for CircleRhombus {
    fn collide(a: (&Circle, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {

    }
}

pub struct CirclePolygon;

impl NarrowPhaseAlgorithm<Circle, Polygon> for CirclePolygon {
    fn collide(a: (&Circle, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {

    }
}

pub struct CircleTriangle;

impl NarrowPhaseAlgorithm<Circle, Triangle> for CircleTriangle {
    fn collide(a: (&Circle, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {

    }
}