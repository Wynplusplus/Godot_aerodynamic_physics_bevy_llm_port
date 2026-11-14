//! Helpers that reproduce the Godot 4 math semantics used by the original GDScript code.
//!
//! The original plugin leans on a few Godot-specific behaviours (`Vector3 * Basis`, `ease()`,
//! Euler orders, `sign(0) == 0`). They are collected here so the physics code can stay close to
//! the original source.

use bevy::math::{Mat3, Vec3};
use serde::Deserialize;

/// Godot's `sign()`: returns 0 for 0 (glam's `signum` returns 1).
pub fn sign(x: f32) -> f32 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// Godot's `remap()`. Not clamped.
pub fn remap(value: f32, from_min: f32, from_max: f32, to_min: f32, to_max: f32) -> f32 {
    to_min + (value - from_min) * (to_max - to_min) / (from_max - from_min)
}

/// Godot's `@GlobalScope.ease(x, curve)`.
pub fn ease(x: f32, curve: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    if curve > 0.0 {
        if curve < 1.0 {
            1.0 - (1.0 - x).powf(1.0 / curve)
        } else {
            x.powf(curve)
        }
    } else if curve < 0.0 {
        if x < 0.5 {
            (x * 2.0).powf(-curve) * 0.5
        } else {
            (1.0 - (1.0 - x) * 2.0).powf(-curve) * 0.5 + 0.5
        }
    } else {
        0.0
    }
}

/// `AeroMathUtils.improved_ease`: eases the magnitude and keeps the sign.
pub fn improved_ease(value: f32, curve: f32) -> f32 {
    ease(value.abs(), curve) * sign(value)
}

/// `AeroMathUtils.v3log_with_base`.
pub fn v3_log_with_base(v: Vec3, base: f32) -> Vec3 {
    v.normalize_or_zero() * ((v.length() + 1.0).ln() / base.ln())
}

/// `AeroMathUtils.max_magnitude`: component-wise max of the absolute values.
pub fn max_magnitude(a: Vec3, b: Vec3) -> Vec3 {
    a.abs().max(b.abs())
}

/// Godot's `Vector3 * Basis` operator (row-vector multiplication), i.e. `basis.transpose() * v`.
pub fn vec_times_basis(v: Vec3, basis: Mat3) -> Vec3 {
    basis.transpose() * v
}

/// Godot's `Vector3.angle_to()`.
pub fn angle_to(a: Vec3, b: Vec3) -> f32 {
    a.cross(b).length().atan2(a.dot(b))
}

/// Godot's `Vector3.move_toward()`.
pub fn vec_move_toward(from: Vec3, to: Vec3, delta: f32) -> Vec3 {
    let d = to - from;
    let len = d.length();
    if len <= delta || len == 0.0 {
        to
    } else {
        from + d / len * delta
    }
}

/// Godot's `TransformUtils.looking_at_safe`.
pub fn looking_at_safe(target: Vec3, up: Vec3, alternate_up: Vec3, default_basis: Mat3) -> Mat3 {
    if target.length_squared().abs() < f32::EPSILON || up.length_squared().abs() < f32::EPSILON {
        return default_basis;
    }
    let up = if target.cross(up).length_squared().abs() < f32::EPSILON {
        alternate_up
    } else {
        up
    };
    // Godot's Basis.looking_at(target, up): -Z points at the target.
    let z = -target.normalize();
    let x = up.cross(z).normalize();
    let y = z.cross(x);
    Mat3::from_cols(x, y, z)
}

/// Godot's `EULER_ORDER_*` enum, as it appears in `.tscn` files.
#[derive(Deserialize, Debug, Clone, Copy, PartialEq, Eq, Default)]
#[serde(from = "i32")]
pub enum EulerOrder {
    #[default]
    Xyz,
    Xzy,
    Yxz,
    Yzx,
    Zxy,
    Zyx,
}

impl From<i32> for EulerOrder {
    fn from(v: i32) -> Self {
        match v {
            1 => Self::Xzy,
            2 => Self::Yxz,
            3 => Self::Yzx,
            4 => Self::Zxy,
            5 => Self::Zyx,
            _ => Self::Xyz,
        }
    }
}

impl EulerOrder {
    /// Axis indices (0 = X, 1 = Y, 2 = Z) in the order the rotations are named.
    fn axes(self) -> [usize; 3] {
        match self {
            Self::Xyz => [0, 1, 2],
            Self::Xzy => [0, 2, 1],
            Self::Yxz => [1, 0, 2],
            Self::Yzx => [1, 2, 0],
            Self::Zxy => [2, 0, 1],
            Self::Zyx => [2, 1, 0],
        }
    }
}

fn axis_vec(i: usize) -> Vec3 {
    match i {
        0 => Vec3::X,
        1 => Vec3::Y,
        _ => Vec3::Z,
    }
}

/// Godot's `Basis.from_euler(euler, order)`.
///
/// For an order "ABC" the matrix is `R_C * R_B * R_A`, so the first named axis is applied first.
pub fn basis_from_euler(euler: Vec3, order: EulerOrder) -> Mat3 {
    let [a, b, c] = order.axes();
    let ra = Mat3::from_axis_angle(axis_vec(a), euler[a]);
    let rb = Mat3::from_axis_angle(axis_vec(b), euler[b]);
    let rc = Mat3::from_axis_angle(axis_vec(c), euler[c]);
    rc * rb * ra
}

/// Middle angle of `m = R_c(gamma) * R_b(beta) * R_a(alpha)`.
fn middle_angle(m: Mat3, a: Vec3, b: Vec3, c: Vec3) -> f32 {
    // R_b(beta) maps e_a to cos(beta) e_a + sin(beta) k e_c, and R_c keeps the e_c component.
    let k = b.cross(a).dot(c);
    ((m * a).dot(c) * k).clamp(-1.0, 1.0).asin()
}

/// Outer angle of `m = R_c(gamma) * R_b(beta) * R_a(alpha)`.
fn outer_angle(m: Mat3, a: Vec3, b: Vec3, c: Vec3) -> f32 {
    // R_c rotates the e_a component within the plane perpendicular to e_c.
    let m_a = m * a;
    let t = c.cross(a).dot(b);
    (t * m_a.dot(b)).atan2(m_a.dot(a))
}

/// Godot's `Basis.get_euler(order)`: the Euler angles that rebuild `m` with `basis_from_euler`.
pub fn euler_from_basis(m: Mat3, order: EulerOrder) -> Vec3 {
    let [a, b, c] = order.axes();
    let (ea, eb, ec) = (axis_vec(a), axis_vec(b), axis_vec(c));
    // The first angle is the outer angle of the transpose, with the axis order reversed and negated.
    let alpha = -outer_angle(m.transpose(), ec, eb, ea);
    let beta = middle_angle(m, ea, eb, ec);
    let gamma = outer_angle(m, ea, eb, ec);
    let mut out = Vec3::ZERO;
    out[a] = alpha;
    out[b] = beta;
    out[c] = gamma;
    out
}

/// Godot's default `rotation_order` for Node3D (YXZ), used for `global_rotation`.
pub fn euler_yxz(m: Mat3) -> Vec3 {
    euler_from_basis(m, EulerOrder::Yxz)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn euler_round_trips_for_every_order() {
        let orders = [EulerOrder::Xyz, EulerOrder::Xzy, EulerOrder::Yxz, EulerOrder::Yzx, EulerOrder::Zxy, EulerOrder::Zyx];
        let samples = [Vec3::new(0.3, -0.7, 1.1), Vec3::new(-1.2, 0.4, 0.2), Vec3::new(0.05, 0.9, -0.6)];
        for order in orders {
            for e in samples {
                let m = basis_from_euler(e, order);
                let back = euler_from_basis(m, order);
                let rebuilt = basis_from_euler(back, order);
                assert!((rebuilt - m).to_cols_array().iter().all(|v| v.abs() < 1e-4), "order {order:?} euler {e:?} -> {back:?}");
                assert!((back - e).abs().max_element() < 1e-4, "order {order:?}: {e:?} != {back:?}");
            }
        }
    }

    #[test]
    fn ease_matches_godot_definition() {
        assert!((ease(0.5, 1.0) - 0.5).abs() < 1e-6);
        assert!((ease(0.5, 2.0) - 0.25).abs() < 1e-6);
        assert!((improved_ease(-0.5, 2.0) + 0.25).abs() < 1e-6);
        assert_eq!(sign(0.0), 0.0);
    }

    #[test]
    fn remap_is_linear_and_unclamped() {
        assert!((remap(5.0, 0.0, 10.0, 0.0, 1.0) - 0.5).abs() < 1e-6);
        assert!((remap(20.0, 0.0, 10.0, 0.0, 1.0) - 2.0).abs() < 1e-6);
    }
}
