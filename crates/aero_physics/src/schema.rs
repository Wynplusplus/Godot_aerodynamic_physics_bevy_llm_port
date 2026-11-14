//! The data format produced by `tools/convert_scenes.py` from the Godot `.tscn` scenes.
//!
//! Each scene is a tree of [`NodeDef`]s. Node `class` names are the Godot class of the node, or the
//! `class_name` of its script (for example `AeroBody3D` or `ManualAeroSurface3D`). Resource-valued
//! properties are inlined, with Godot property names kept as they are.

use bevy::math::{Vec2, Vec3};
use serde::{Deserialize, Deserializer};
use serde_json::{Map, Value};

use crate::curve::{Curve, LiftCurveParams, lift_curve};
use crate::godot::EulerOrder;

/// Deserialises a JSON `[x, y, z]` array into a `Vec3`.
pub fn v3<'de, D: Deserializer<'de>>(d: D) -> Result<Vec3, D::Error> {
    let a = <[f32; 3]>::deserialize(d)?;
    Ok(Vec3::from_array(a))
}

/// Deserialises a JSON `[x, y]` array into a `Vec2`.
pub fn v2<'de, D: Deserializer<'de>>(d: D) -> Result<Vec2, D::Error> {
    let a = <[f32; 2]>::deserialize(d)?;
    Ok(Vec2::from_array(a))
}

/// A whole aircraft or prop scene.
#[derive(Deserialize, Debug, Clone)]
pub struct SceneFile {
    pub name: String,
    pub root: NodeDef,
}

/// One node of a converted scene.
#[derive(Deserialize, Debug, Clone)]
pub struct NodeDef {
    pub name: String,
    #[serde(default)]
    pub class: String,
    /// Godot `Transform3D` layout: basis columns x, y, z followed by the origin.
    #[serde(default)]
    pub transform: Option<[f32; 12]>,
    #[serde(default = "yes")]
    pub visible: bool,
    #[serde(default)]
    pub props: Map<String, Value>,
    #[serde(default)]
    pub mesh: Option<MeshRef>,
    #[serde(default)]
    pub follow: Vec<FollowDef>,
    #[serde(default)]
    pub children: Vec<NodeDef>,
}

fn yes() -> bool {
    true
}

impl NodeDef {
    /// Reads a property and deserialises it. Missing or invalid values fall back to the default.
    pub fn prop<T: serde::de::DeserializeOwned + Default>(&self, key: &str) -> T {
        self.props.get(key).and_then(|v| serde_json::from_value(v.clone()).ok()).unwrap_or_default()
    }

    pub fn prop_opt<T: serde::de::DeserializeOwned>(&self, key: &str) -> Option<T> {
        self.props.get(key).and_then(|v| serde_json::from_value(v.clone()).ok())
    }

    pub fn prop_f32(&self, key: &str, default: f32) -> f32 {
        self.props.get(key).and_then(Value::as_f64).map_or(default, |v| v as f32)
    }

    pub fn prop_bool(&self, key: &str, default: bool) -> bool {
        self.props.get(key).and_then(Value::as_bool).unwrap_or(default)
    }

    pub fn prop_str(&self, key: &str) -> Option<&str> {
        self.props.get(key).and_then(Value::as_str)
    }

    /// The Godot transform as `(basis columns, origin)`.
    pub fn local_transform(&self) -> Option<(bevy::math::Mat3, Vec3)> {
        self.transform.map(|t| {
            let basis = bevy::math::Mat3::from_cols(Vec3::new(t[0], t[1], t[2]), Vec3::new(t[3], t[4], t[5]), Vec3::new(t[6], t[7], t[8]));
            (basis, Vec3::new(t[9], t[10], t[11]))
        })
    }
}

/// A set of glTF primitives drawn for this node. `file` is relative to the asset root.
#[derive(Deserialize, Debug, Clone)]
pub struct MeshRef {
    pub file: String,
    pub primitives: Vec<PrimitiveRef>,
    /// Buoyancy geometry, in the node's local space, when the node is a buoyancy mesh source.
    #[serde(default)]
    pub buoyancy_vertices: Vec<[f32; 3]>,
    #[serde(default)]
    pub buoyancy_indices: Vec<u32>,
}

#[derive(Deserialize, Debug, Clone)]
pub struct PrimitiveRef {
    pub mesh: usize,
    pub primitive: usize,
    #[serde(default)]
    pub material: Option<usize>,
}

/// A visual rotation link: `target.rotation.<axis> = sign * source.rotation.<axis> + offset`.
/// Converted from the small GDScript snippets that keep visual control surfaces in step with the
/// aero surfaces that drive them.
#[derive(Deserialize, Debug, Clone)]
pub struct FollowDef {
    pub target: String,
    pub source: String,
    /// Axis of the target node's rotation that is written: "x", "y" or "z".
    pub axis: String,
    /// Axis of the source node's rotation that is read: "x", "y" or "z".
    #[serde(default = "x_axis")]
    pub source_axis: String,
    #[serde(default = "one")]
    pub sign: f32,
    #[serde(default)]
    pub offset: f32,
}

fn one() -> f32 {
    1.0
}

fn x_axis() -> String {
    "x".to_string()
}

/// Shapes used by `CollisionShape3D` nodes (the `shape` property).
#[derive(Deserialize, Debug, Clone)]
#[serde(tag = "class")]
pub enum ShapeDef {
    BoxShape3D {
        #[serde(deserialize_with = "v3")]
        size: Vec3,
    },
    CapsuleShape3D {
        radius: f32,
        height: f32,
    },
    SphereShape3D {
        radius: f32,
    },
    ConvexPolygonShape3D {
        points: Vec<[f32; 3]>,
    },
}

// ---- Typed resource configs -------------------------------------------------------------------

/// `AeroSurfaceConfig`.
#[derive(Deserialize, Debug, Clone)]
#[serde(default)]
pub struct SurfaceConfig {
    pub chord: f32,
    pub span: f32,
}

impl Default for SurfaceConfig {
    fn default() -> Self {
        Self { chord: 1.0, span: 2.0 }
    }
}

impl SurfaceConfig {
    pub fn area(&self) -> f32 {
        self.span * self.chord
    }
}

/// `ManualAeroSurfaceConfig`.
#[derive(Deserialize, Debug, Clone)]
#[serde(default)]
pub struct ManualSurfaceConfig {
    pub lift_aoa_curve: Option<Curve>,
    pub min_drag_coefficient: f32,
    pub max_drag_coefficient: f32,
    pub drag_aoa_curve: Option<Curve>,
    pub sweep_drag_multiplier_curve: Option<Curve>,
    pub drag_at_mach_multiplier_curve: Option<Curve>,
}

impl Default for ManualSurfaceConfig {
    fn default() -> Self {
        Self {
            lift_aoa_curve: None,
            min_drag_coefficient: 0.02,
            max_drag_coefficient: 1.0,
            drag_aoa_curve: None,
            sweep_drag_multiplier_curve: None,
            drag_at_mach_multiplier_curve: None,
        }
    }
}

impl ManualSurfaceConfig {
    /// The lift curve, falling back to the default `LiftCurve.new()`.
    pub fn lift_curve(&self) -> Curve {
        self.lift_aoa_curve.clone().unwrap_or_else(|| lift_curve(&LiftCurveParams::default()))
    }

    pub fn drag_aoa_curve(&self) -> Curve {
        self.drag_aoa_curve.clone().unwrap_or_else(default_drag_aoa_curve)
    }

    pub fn sweep_drag_multiplier_curve(&self) -> Curve {
        self.sweep_drag_multiplier_curve.clone().unwrap_or_else(default_sweep_drag_curve)
    }

    pub fn drag_at_mach_multiplier_curve(&self) -> Curve {
        self.drag_at_mach_multiplier_curve.clone().unwrap_or_else(default_drag_at_mach_curve)
    }
}

/// `res://core/resources/default_drag_aoa_curve.tres`.
pub fn default_drag_aoa_curve() -> Curve {
    let mut curve = Curve::from_points(&[(-180.0, 0.0), (-90.0, 1.0), (0.0, 0.0), (90.0, 1.0), (180.0, 0.0)], -180.0, 180.0, 0.0, 1.0, 21);
    free_tangents(&mut curve);
    curve
}

/// Sets every point to free tangents with zero slope, as stored in the `.tres` defaults, then bakes.
fn free_tangents(curve: &mut Curve) {
    for p in &mut curve.points {
        *p = crate::curve::CurvePoint { x: p.x, y: p.y, ..Default::default() };
    }
    curve.bake();
}

/// `res://core/resources/default_drag_at_mach_curve.tres`.
pub fn default_drag_at_mach_curve() -> Curve {
    let mut curve = Curve::from_points(&[(0.7, 1.0), (1.0, 2.0), (2.0, 1.2)], 0.0, 2.0, 1.0, 2.0, 23);
    free_tangents(&mut curve);
    curve.points[2].left_tangent = 0.181;
    curve.bake();
    curve
}

/// `res://core/resources/default_sweep_drag_multiplier.tres`.
pub fn default_sweep_drag_curve() -> Curve {
    let mut curve = Curve::from_points(&[(0.0, 1.0), (35.0, 0.6), (90.0, 0.5)], 0.0, 90.0, 0.0, 1.0, 4);
    free_tangents(&mut curve);
    curve.points[0].right_tangent = -0.015519453;
    curve.points[1].left_tangent = -0.0035805835;
    curve.points[1].right_tangent = -0.0035805835;
    curve.bake();
    curve
}

/// `AeroBuoyancySphere3D` / `AeroBuoyancyMesh3D` settings (scalar fields only).
#[derive(Deserialize, Debug, Clone)]
#[serde(default)]
pub struct MeshBuoyancySettings {
    pub skin_friction: f32,
    pub aero_force_multiplier: f32,
}

impl Default for MeshBuoyancySettings {
    fn default() -> Self {
        Self { skin_friction: 0.02, aero_force_multiplier: 1.0 }
    }
}

/// Helper used by the spawner to read a Godot `rotation_order` (defaults to YXZ).
pub fn rotation_order_of(node: &NodeDef) -> EulerOrder {
    EulerOrder::from(node.props.get("rotation_order").and_then(Value::as_i64).unwrap_or(2) as i32)
}
