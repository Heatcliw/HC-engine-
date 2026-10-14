use crate::math::*;

pub fn closest_point_segment(point: Vec2, a: Vec2, b: Vec2,) -> Vec2 {

    let ab = sub(b, a);
    let ap = sub(point, a);

    let t = dot(ap, ab) / dot(ab, ab);

    let t = t.clamp(0.0, 1.0);

    add(
        a,
        scale(ab, t)
    )
}