pub struct EllipseEllipse;

impl NarrowPhaseAlgorithm<Ellipse, Ellipse> for EllipseEllipse {
    fn collide(a: (&Ellipse, &Transform), b: (&Ellipse, &Transform)) -> CollisionInfo {

    }
}

pub struct EllipseSquare;

impl NarrowPhaseAlgorithm<Ellipse, Square> for EllipseSquare {
    fn collide(a: (&Ellipse, &Transform), b: (&Square, &Transform)) -> CollisionInfo {

    }
}

pub struct EllipseRectangle;

impl NarrowPhaseAlgorithm<Ellipse, Rectangle> for EllipseRectangle {
    fn collide(a: (&Ellipse, &Transform), b: (&Rectangle, &Transform)) -> CollisionInfo {

    }
}

pub struct EllipseRhombus;

impl NarrowPhaseAlgorithm<Ellipse, Rhombus> for EllipseRhombus {
    fn collide(a: (&Ellipse, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {

    }
}

pub struct EllipsePolygon;

impl NarrowPhaseAlgorithm<Ellipse, Polygon> for EllipsePolygon {
    fn collide(a: (&Ellipse, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {

    }
}

pub struct EllipseTriangle;

impl NarrowPhaseAlgorithm<Ellipse, Triangle> for EllipseTriangle {
    fn collide(a: (&Ellipse, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {

    }
}