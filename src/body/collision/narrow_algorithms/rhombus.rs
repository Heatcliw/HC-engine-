pub struct RhombusRhombus;

impl NarrowPhaseAlgorithm<Rhombus, Rhombus> for RhombusRhombus {
    fn collide(a: (&Rhombus, &Transform), b: (&Rhombus, &Transform)) -> CollisionInfo {

    }
}

pub struct RhombusPolygon;

impl NarrowPhaseAlgorithm<Rhombus, Polygon> for RhombusPolygon {
    fn collide(a: (&Rhombus, &Transform), b: (&Polygon, &Transform)) -> CollisionInfo {

    }
}

pub struct RhombusTriangle;

impl NarrowPhaseAlgorithm<Rhombus, Triangle> for RhombusTriangle {
    fn collide(a: (&Rhombus, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {

    }
}