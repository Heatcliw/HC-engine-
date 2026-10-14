pub struct PolygonPolygon;

impl NarrowPhaseAlgorithm<Polygon, Polygon> for PolygonPolygon {
    fn collide(a: (&Polygon, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {

    }
}

pub struct PolygonTriangle;

impl NarrowPhaseAlgorithm<Polygon, Triangle> for PolygonTriangle {
    fn collide(a: (&Polygon, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {

    }
}