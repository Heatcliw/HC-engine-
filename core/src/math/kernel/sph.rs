use crate::math::*;

const PI: f32 = std::f32::consts::PI;

/// Poly6 smoothing kernel.
/// r2 = squared distance between particles.
pub fn poly6(r2:f32,h:f32) -> f32 {
    let h2 = h * h;
    if r2 >= h2 {
        return 0.0;
    }
    let k = 4.0 / (PI * h.powi(8));
    let diff = h2 - r2;
    let diff2 = diff * diff;
    k * diff2 * diff
}

/// Gradient of the spiky kernel.
pub fn spiky_grad(r:Vec2,h:f32) -> Vec2 {
    // r² = dot(r, r)
    let len_sq = dot(r, r);
    let h2 = h * h;
    if len_sq == 0.0 || len_sq >= h2 {
        return VEC2_ZERO;
    }

    let len = len_sq.sqrt();
    let k = -30.0 / (PI * h.powi(5));
    let f = k * (h-len).powi(2) / len;
    Vec2 {
        x: r.x * f, 
        y: r.y * f
    }
}


/// Viscosity kernel weight function.
pub fn viscosity_kernel(r:f32,h:f32) -> f32 {
    if r >= h {
        return 0.0;
    }

    let k = 20.0 / (3.0 * PI * h.powi(5));
    k * (h - r)
}