use crate::math::*;
use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Copy)]
#[derive(Default, Serialize, Deserialize)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32
}

#[inline]
pub fn add(a:Vec2,b:Vec2) -> Vec2 {
    let x = a.x + b.x;
    let y = a.y + b.y;
    Vec2 {
        x, y
    }
}

#[inline]
pub fn sub(a:Vec2,b:Vec2) -> Vec2 {
    let x = a.x - b.x;
    let y = a.y - b.y;

    Vec2 {
        x, y
    }
}

#[inline]
pub fn scale(v: Vec2, s: f32) -> Vec2 {
    let x = v.x * s;
    let y = v.y * s;
    
    Vec2 {
        x, y
    }
}

#[inline]
pub fn div(v: Vec2, s: f32) -> Vec2 {
    let x = v.x / s;
    let y = v.y / s;

    Vec2 {
        x, y
    }
}
#[inline]
pub fn dot(a:Vec2,b:Vec2)-> f32 {
    let dot = a.x * b.x + a.y * b.y;
    dot
}
#[inline]
pub fn andot(a:Vec2,b:Vec2)-> f32 {
    let andot = a.x * b.x - a.y * b.y;
    andot
}

#[inline]
pub fn gaus(a:Vec2,b:Vec2)-> f32 {
    let gaus = a.x * b.y - b.x * a.y;
    gaus
}

#[inline]
pub fn length(v: Vec2) -> f32 {
    dot(v, v).sqrt()
}

#[inline]
pub fn lerp(a: Vec2, b: Vec2, t: f32) -> Vec2 {
    
    let x = a.x + (b.x - a.x) * t;
    let y = a.y + (b.y - a.y) * t;
    
    Vec2 {
        x, y
    }
}

#[inline]
pub fn negative(v: Vec2) -> Vec2 {
    let x = v.x * (-1.0);
    let y = v.y * (-1.0);
    Vec2 {
        x, y
    }

}
#[inline]
pub fn negative_absciss(v: Vec2) -> Vec2 {
    let x = v.x * (-1.0);
    let y = v.y;
    Vec2 {
        x, y
    }
}
#[inline]
pub fn negative_ordinate(v: Vec2) -> Vec2 {
    let x = v.x;
    let y = v.y * (-1.0);
    Vec2 {
        x, y
    }
}

#[inline]
pub fn rotate(v: Vec2, angle: f32) -> Vec2 {
    let cos = angle.cos();
    let sin = angle.sin();

    let x = v.x * cos - v.y * sin;
    let y = v.x * sin + v.y * cos;
    Vec2 {
        x, y
    }
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