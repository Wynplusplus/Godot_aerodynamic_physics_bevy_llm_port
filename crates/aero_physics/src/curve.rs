//! A port of Godot's `Curve` resource, including `sample_baked()`, and of `easy_lift_curve.gd`.

use serde::Deserialize;

const TANGENT_FREE: u8 = 0;
const TANGENT_LINEAR: u8 = 1;

/// One control point, matching the `_data` entries of a Godot `.tscn` curve.
#[derive(Deserialize, Debug, Clone, Copy, PartialEq)]
#[serde(default)]
pub struct CurvePoint {
    pub x: f32,
    pub y: f32,
    pub left_tangent: f32,
    pub right_tangent: f32,
    pub left_mode: u8,
    pub right_mode: u8,
}

impl Default for CurvePoint {
    fn default() -> Self {
        Self { x: 0.0, y: 0.0, left_tangent: 0.0, right_tangent: 0.0, left_mode: TANGENT_FREE, right_mode: TANGENT_FREE }
    }
}

/// Godot `Curve`, with the domain/value limits and the baked lookup table.
#[derive(Deserialize, Debug, Clone, PartialEq)]
#[serde(default)]
pub struct Curve {
    pub min_domain: f32,
    pub max_domain: f32,
    pub min_value: f32,
    pub max_value: f32,
    pub bake_resolution: u32,
    pub points: Vec<CurvePoint>,
    #[serde(skip)]
    baked: Vec<f32>,
}

impl Default for Curve {
    fn default() -> Self {
        Self { min_domain: 0.0, max_domain: 1.0, min_value: 0.0, max_value: 1.0, bake_resolution: 100, points: Vec::new(), baked: Vec::new() }
    }
}

impl Curve {
    /// Builds a curve from `(x, y)` points with the given tangent modes, then bakes it.
    /// The domain and value limits must be set before baking.
    pub fn from_points(points: &[(f32, f32)], min_domain: f32, max_domain: f32, min_value: f32, max_value: f32, bake_resolution: u32) -> Self {
        let mut curve = Self { min_domain, max_domain, min_value, max_value, bake_resolution, ..Default::default() };
        for &(x, y) in points {
            curve.points.push(CurvePoint { x, y, left_mode: TANGENT_LINEAR, right_mode: TANGENT_LINEAR, ..Default::default() });
        }
        curve.bake();
        curve
    }

    /// Godot `Curve.add_point()` with explicit tangents and modes. Points are kept sorted by x.
    pub fn add_point(&mut self, point: CurvePoint) {
        let index = self.points.iter().position(|p| p.x > point.x).unwrap_or(self.points.len());
        self.points.insert(index, point);
    }

    /// Computes linear tangents from neighbouring points, the way Godot does for `TANGENT_LINEAR`.
    fn resolved_tangents(&self) -> Vec<(f32, f32)> {
        let n = self.points.len();
        (0..n)
            .map(|i| {
                let p = self.points[i];
                let left = if p.left_mode == TANGENT_LINEAR {
                    if i > 0 {
                        let prev = self.points[i - 1];
                        (p.y - prev.y) / (p.x - prev.x)
                    } else {
                        0.0
                    }
                } else {
                    p.left_tangent
                };
                let right = if p.right_mode == TANGENT_LINEAR {
                    if i + 1 < n {
                        let next = self.points[i + 1];
                        (next.y - p.y) / (next.x - p.x)
                    } else {
                        0.0
                    }
                } else {
                    p.right_tangent
                };
                (left, right)
            })
            .collect()
    }

    /// Cubic Hermite interpolation between control points, evaluated at absolute x.
    pub fn interpolate(&self, x: f32) -> f32 {
        let n = self.points.len();
        match n {
            0 => return 0.0,
            1 => return self.points[0].y,
            _ => {}
        }
        if x <= self.points[0].x {
            return self.points[0].y;
        }
        if x >= self.points[n - 1].x {
            return self.points[n - 1].y;
        }
        let tangents = self.resolved_tangents();
        let i = (0..n - 1).rfind(|&i| self.points[i].x <= x).unwrap_or(0);
        let p0 = self.points[i];
        let p1 = self.points[i + 1];
        let h = p1.x - p0.x;
        let t = (x - p0.x) / h;
        let m0 = tangents[i].1 * h;
        let m1 = tangents[i + 1].0 * h;
        let t2 = t * t;
        let t3 = t2 * t;
        (2.0 * t3 - 3.0 * t2 + 1.0) * p0.y + (t3 - 2.0 * t2 + t) * m0 + (-2.0 * t3 + 3.0 * t2) * p1.y + (t3 - t2) * m1
    }

    /// Godot `Curve.bake()`: samples the curve at `bake_resolution` evenly spaced points.
    pub fn bake(&mut self) {
        let res = self.bake_resolution.max(2) as usize;
        self.baked = (0..res)
            .map(|i| {
                let t = i as f32 / (res - 1) as f32;
                self.interpolate(self.min_domain + t * (self.max_domain - self.min_domain))
            })
            .collect();
    }

    /// Godot `Curve.sample_baked()`: linear interpolation over the baked table, clamped to the domain.
    pub fn sample_baked(&self, offset: f32) -> f32 {
        if self.baked.is_empty() {
            return self.interpolate(offset);
        }
        let span = self.max_domain - self.min_domain;
        if span <= 0.0 {
            return self.baked[0];
        }
        let t = ((offset - self.min_domain) / span).clamp(0.0, 1.0);
        let f = t * (self.baked.len() - 1) as f32;
        let i = (f.floor() as usize).min(self.baked.len() - 2);
        let frac = f - i as f32;
        self.baked[i] * (1.0 - frac) + self.baked[i + 1] * frac
    }
}

/// Parameters of `easy_lift_curve.gd`. The defaults match the GDScript exports.
#[derive(Deserialize, Debug, Clone, PartialEq)]
#[serde(default)]
pub struct LiftCurveParams {
    pub lift_coefficient: f32,
    pub lift_slope: f32,
    pub camber: f32,
    pub stall_region_size: f32,
    pub post_stall_lift: f32,
    pub reverse_lift_penalty: f32,
}

impl Default for LiftCurveParams {
    fn default() -> Self {
        Self { lift_coefficient: 1.2, lift_slope: 15.0, camber: 0.0, stall_region_size: 10.0, post_stall_lift: 0.6, reverse_lift_penalty: 0.3 }
    }
}

/// `LiftCurve.recalculate()`: builds the lift-vs-AoA curve over -180..180 degrees.
pub fn lift_curve(p: &LiftCurveParams) -> Curve {
    let min_domain = -180.0_f32;
    let max_domain = 180.0_f32;
    let ls = p.lift_slope;
    let lc = p.lift_coefficient;
    let camber = p.camber;
    let rlp = p.reverse_lift_penalty;
    let pss = p.post_stall_lift;
    // slope = (0.5 * (2 * lift_slope), lift_coefficient)
    let slope = (ls, lc);
    let crossover_height = 0.0_f32;

    let mut curve = Curve { min_domain, max_domain, min_value: -20.0, max_value: 20.0, bake_resolution: 36, points: Vec::new(), baked: Vec::new() };
    let lin = |x: f32, y: f32, left: u8, right: u8| CurvePoint { x, y, left_mode: left, right_mode: right, ..Default::default() };

    curve.add_point(lin(min_domain, crossover_height, TANGENT_LINEAR, TANGENT_LINEAR));
    curve.add_point(lin(
        min_domain + ls - slope.0 * camber - slope.0 * rlp,
        lc - slope.1 * camber - slope.1 * rlp,
        TANGENT_LINEAR,
        TANGENT_FREE,
    ));
    curve.add_point(lin(min_domain + 45.0, pss, TANGENT_FREE, TANGENT_FREE));
    curve.add_point(lin(-45.0, -pss, TANGENT_FREE, TANGENT_FREE));
    curve.add_point(lin(-ls + slope.0 * camber, -lc + slope.1 * camber, TANGENT_FREE, TANGENT_LINEAR));
    curve.add_point(lin(ls + slope.0 * camber, lc + slope.1 * camber, TANGENT_LINEAR, TANGENT_FREE));
    curve.add_point(lin(45.0, pss, TANGENT_FREE, TANGENT_FREE));
    curve.add_point(lin(max_domain - 45.0, -pss, TANGENT_FREE, TANGENT_FREE));
    curve.add_point(lin(
        max_domain - ls - slope.0 * camber + slope.0 * rlp,
        -lc - slope.1 * camber + slope.1 * rlp,
        TANGENT_FREE,
        TANGENT_LINEAR,
    ));
    curve.add_point(lin(max_domain, crossover_height, TANGENT_LINEAR, TANGENT_LINEAR));

    let abs_max = curve.points.iter().fold(0.0_f32, |acc, p| acc.max(p.y.abs()));
    curve.min_value = -abs_max;
    curve.max_value = abs_max;
    curve.bake();
    curve
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lift_curve_is_odd_and_bounded() {
        let c = lift_curve(&LiftCurveParams::default());
        assert!(c.sample_baked(0.0).abs() < 1e-3);
        let up = c.sample_baked(20.0);
        let down = c.sample_baked(-20.0);
        assert!(up > 0.5 && (up + down).abs() < 1e-3, "up {up} down {down}");
        assert!(c.max_value > 1.0);
    }

    #[test]
    fn baked_curve_matches_interpolation_at_points() {
        let c = Curve::from_points(&[(0.0, 0.0), (1.0, 2.0), (2.0, 1.0)], 0.0, 2.0, 0.0, 2.0, 201);
        assert!((c.sample_baked(1.0) - 2.0).abs() < 1e-3);
        assert!((c.sample_baked(0.0)).abs() < 1e-6);
        assert!((c.sample_baked(2.0) - 1.0).abs() < 1e-6);
    }
}
