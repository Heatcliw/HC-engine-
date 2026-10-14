pub struct TriangleTriangle;

impl NarrowPhaseAlgorithm<Triangle, Triangle> for TriangleTriangle {
    fn collide(a: (&Triangle, &Transform), b: (&Triangle, &Transform)) -> CollisionInfo {

    }
}