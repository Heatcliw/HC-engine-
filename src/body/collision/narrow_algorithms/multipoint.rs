pub struct MultiPointMultiPoint;

impl NarrowPhaseAlgorithm<MultiPoint, MultiPoint> for MultiPointMultiPoint {
    fn collide(a: (&MultiPoint, &Transform), b: (&MultiPoint, &Transform)) -> CollisionInfo {

    }
}

pub struct MultiPointLine;

impl NarrowPhaseAlgorithm<MultiPoint, Line> for MultiPointLine {
    fn collide(a: (&MultiPoint, &Transform), b: (&Line, &Transform)) -> CollisionInfo {

    }
}

pub struct MultiPointCircle;

impl NarrowPhaseAlgorithm<MultiPoint, Circle> for MultiPointCircle {
    fn collide(a: (&MultiPoint, &Transform), b: (&Circle, &Transform)) -> CollisionInfo {

    }
}

pub struct MultiPointEllipse;

impl NarrowPhaseAlgorithm<MultiPoint, Ellipse> for MultiPointEllipse {
    fn collide(a: (&MultiPoint, &Transform), b: (&Ellipse, &Transform)) -> CollisionInfo {

    }
}

pub struct MultiPointSquare;

impl NarrowPhaseAlgorithm<MultiPoint, Square> for MultiPointSquare {
    fn collide(a: (&MultiPoint, &Transform), b: (&Square, &Transform)) -> CollisionInfo {

    }
}

pub struct MultiPointRectangle;

impl NarrowPhaseAlgorithm<MultiPoint, Rectangle> for MultiPointRectangle {
    fn collide(a: (&MultiPoint, &Transform), b: (&Rectangle, &Transform)) -> CollisionInfo {

    }
}

pub struct MultiPointRhombus;

impl NarrowPhaseAlgorithm<MultiPoint, Rhombus> for MultiPointRhombus {
    fn collide(a: (&MultiPoint, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {

    }
}

pub struct MultiPointPolygon;

impl NarrowPhaseAlgorithm<MultiPoint, Polygon> for MultiPointPolygon {
    fn collide(a: (&MultiPoint, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {

    }
}

pub struct MultiPointTriangle;

impl NarrowPhaseAlgorithm<MultiPoint, Triangle> for MultiPointTriangle {
    fn collide(a: (&MultiPoint, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {

    }
}