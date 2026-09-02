pub struct PointPoint;

impl NarrowPhaseAlgorithm<Point, Point> for PointPoint {
    fn collide(a: (&Point, &Transform), b: (&Point, &Transform)) -> CollisionInfo {

    }
}

pub struct PointMultiPoint;

impl NarrowPhaseAlgorithm<Point, MultiPoint> for PointMultiPoint {
    fn collide(a: (&Point, &Transform), b: (&MultiPoint, &Transform)) -> CollisionInfo {

    }
}

pub struct PointLine;

impl NarrowPhaseAlgorithm<Point, Line> for PointLine {
    fn collide(a: (&Point, &Transform), b: (&Line, &Transform)) -> CollisionInfo {

    }
}

pub struct PointCircle;

impl NarrowPhaseAlgorithm<Point, Circle> for PointCircle {
    fn collide(a: (&Point, &Transform), b: (&Circle, &Transform)) -> CollisionInfo {

    }
}

pub struct PointEllipse;

impl NarrowPhaseAlgorithm<Point, Ellipse> for PointEllipse {
    fn collide(a: (&Point, &Transform), b: (&Ellipse, &Transform)) -> CollisionInfo {

    }
}

pub struct PointSquare;

impl NarrowPhaseAlgorithm<Point, Square> for PointSquare {
    fn collide(a: (&Point, &Transform), b: (&Square, &Transform)) -> CollisionInfo {

    }
}

pub struct PointRectangle;

impl NarrowPhaseAlgorithm<Point, Rectangle> for PointRectangle {
    fn collide(a: (&Point, &Transform), b: (&Rectangle, &Transform)) -> CollisionInfo {

    }
}

pub struct PointRhombus;

impl NarrowPhaseAlgorithm<Point, Rhombus> for PointRhombus {
    fn collide(a: (&Point, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {

    }
}

pub struct PointPolygon;

impl NarrowPhaseAlgorithm<Point, Polygon> for PointPolygon {
    fn collide(a: (&Point, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {

    }
}

pub struct PointTriangle;

impl NarrowPhaseAlgorithm<Point, Triangle> for PointTriangle {
    fn collide(a: (&Point, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {

    }
}