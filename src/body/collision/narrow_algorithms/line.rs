pub struct LineLine;

impl NarrowPhaseAlgorithm<Line, Line> for LineLine {
    fn collide(a: (&Line, &Transform), b: (&Line, &Transform)) -> CollisionInfo {

    }
}

pub struct LineCircle;

impl NarrowPhaseAlgorithm<Line, Circle> for LineCircle {
    fn collide(a: (&Line, &Transform), b: (&Circle, &Transform)) -> CollisionInfo {

    }
}

pub struct LineEllipse;

impl NarrowPhaseAlgorithm<Line, Ellipse> for LineEllipse {
    fn collide(a: (&Line, &Transform), b: (&Ellipse, &Transform)) -> CollisionInfo {

    }
}

pub struct LineSquare;

impl NarrowPhaseAlgorithm<Line, Square> for LineSquare {
    fn collide(a: (&Line, &Transform), b: (&Square, &Transform)) -> CollisionInfo {

    }
}

pub struct LineRectangle;

impl NarrowPhaseAlgorithm<Line, Rectangle> for LineRectangle {
    fn collide(a: (&Line, &Transform), b: (&Rectangle, &Transform)) -> CollisionInfo {

    }
}

pub struct LineRhombus;

impl NarrowPhaseAlgorithm<Line, Rhombus> for LineRhombus {
    fn collide(a: (&Line, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {

    }
}

pub struct LinePolygon;

impl NarrowPhaseAlgorithm<Line, Polygon> for LinePolygon {
    fn collide(a: (&Line, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {

    }
}

pub struct LineTriangle;

impl NarrowPhaseAlgorithm<Line, Triangle> for LineTriangle {
    fn collide(a: (&Line, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {

    }
}