use crate::math::*;

pub type Vec2 = [f32; 2];

#[inline]
pub fn add(a:Vec2,b:Vec2) -> Vec2 {
    [a[0]+b[0],a[1]+b[1]]
}

#[inline]
pub fn sub(a:Vec2,b:Vec2) -> Vec2 {
    [a[0]-b[0],a[1]-b[1]]
}

#[inline]
pub fn scale(v: Vec2, s: f32) -> Vec2 {
    [v[0]*s,v[1]*s]
}

#[inline]
pub fn div(v: Vec2, s: f32) -> Vec2 {
    [v[0]/s,v[1]/s]
}
#[inline]
pub fn dot(a:Vec2,b:Vec2)-> f32 {
    a[0] * b[0] + a[1] * b[1]
}
#[inline]
pub fn andot(a:Vec2,b:Vec2)-> f32 {
    a[0] * b[0] - a[1] * b[1]
}

#[inline]
pub fn gaus(a:Vec2,b:Vec2)-> f32 {
    a[0] * b[1] - b[0] * a[1]
}

#[inline]
pub fn length(v: Vec2) -> f32 {
    dot(v, v).sqrt()
}

#[inline]
pub fn lerp(a: Vec2, b: Vec2, t: f32) -> Vec2 {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
    ]
}

#[inline]
pub fn negative(v: Vec2) -> Vec2 {
    [-v[0], -v[1]]
}
#[inline]
pub fn negative_absciss(v: Vec2) -> Vec2 {
    [-v[0], v[1]]
}
#[inline]
pub fn negative_ordinate(v: Vec2) -> Vec2 {
    [v[0], -v[1]]
}

#[inline]
pub fn rotate(v: Vec2, angle: f32) -> Vec2 {
    let cos = angle.cos();
    let sin = angle.sin();

    [
        v[0] * cos - v[1] * sin,
        v[0] * sin + v[1] * cos,
    ]
}

#[inline]
pub fn normalize(v: Vec2) -> Vec2 {
    let len = length(v);

    if len < EPSILON {
        VEC2_ZERO
    } else {
        scale(v, 1.0 / len)
    }
}