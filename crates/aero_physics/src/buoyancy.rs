//! Port of `aero_buoyancy_mesh_3d.gd`: vertex-based lift, drag, and buoyancy for an arbitrary mesh.

use std::collections::HashMap;

use bevy::math::{Mat3, Vec3};

use crate::atmosphere::AtmosphereSnapshot;
use crate::influencer::NodeState;
use crate::godot::remap;

/// Precomputed buoyancy data for one mesh.
#[derive(Debug, Clone, Default)]
pub struct MeshBuoyancy {
    /// Barycentres of the triangles around each unique vertex (mesh local space).
    pub positions: Vec<Vec3>,
    /// Area-weighted normal of each unique vertex. Its length is the vertex's "area".
    pub coefficients: Vec<Vec3>,
    /// Mean distance from each vertex to the centres of its triangles.
    pub sizes: Vec<f32>,
    pub skin_friction: f32,
    pub aero_force_multiplier: f32,
}

impl MeshBuoyancy {
    /// `AeroBuoyancyMesh3D.calculate_mesh_data()`.
    ///
    /// `vertices` and `indices` describe the triangles in the mesh's local space. Vertices with
    /// identical positions are merged, as in the original.
    pub fn from_mesh(vertices: &[Vec3], indices: &[u32], skin_friction: f32, aero_force_multiplier: f32) -> Self {
        // Merge exact duplicate positions and remap the indices.
        let mut unique: Vec<Vec3> = Vec::new();
        let mut lookup: HashMap<[u32; 3], u32> = HashMap::new();
        let mut remap_index = vec![0u32; vertices.len()];
        for (i, v) in vertices.iter().enumerate() {
            let key = [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
            let id = *lookup.entry(key).or_insert_with(|| {
                unique.push(*v);
                (unique.len() - 1) as u32
            });
            remap_index[i] = id;
        }
        let tris: Vec<u32> = indices.iter().map(|&i| remap_index[i as usize]).collect();

        // Triangles that touch each unique vertex.
        let mut touching: Vec<Vec<usize>> = vec![Vec::new(); unique.len()];
        for (slot, &v) in tris.iter().enumerate() {
            touching[v as usize].push(slot / 3);
        }

        let mut out = Self { skin_friction, aero_force_multiplier, ..Default::default() };
        for (vertex_index, triangles) in touching.iter().enumerate() {
            if triangles.is_empty() {
                continue;
            }
            let vertex = unique[vertex_index];
            let mut normal_sum = Vec3::ZERO;
            let mut position_sum = Vec3::ZERO;
            let mut distance_sum = 0.0;
            for &t in triangles {
                // As in the original, the normal is taken from the triangle's first vertex.
                let a = unique[tris[t * 3] as usize];
                let b = unique[tris[t * 3 + 1] as usize];
                let c = unique[tris[t * 3 + 2] as usize];
                // glTF triangles are counter-clockwise seen from outside, so this normal points outwards.
                // (The GDScript uses the Godot winding order, which is the reverse.)
                let side_ab = b - a;
                let side_ac = c - a;
                let triangle_normal = 0.5 * side_ab.cross(side_ac);
                normal_sum += triangle_normal / 3.0;
                let center = (a + b + c) / 3.0;
                position_sum += center;
                distance_sum += vertex.distance(center);
            }
            let count = triangles.len() as f32;
            out.coefficients.push(normal_sum);
            out.positions.push(position_sum / count);
            out.sizes.push(distance_sum / count);
        }
        out
    }

    /// Lift, drag, and buoyancy of the mesh for one substep. Returns `(force, torque)`.
    ///
    /// The result replaces, rather than adds to, the children's sum, as in the original.
    pub fn forces(&self, state: &NodeState, gravity: Vec3, body_density: f32, atmospheres: &[AtmosphereSnapshot]) -> (Vec3, Vec3) {
        let rel = state.relative_position;
        let basis: Mat3 = state.global_basis;
        let global_origin = state.global_origin;
        let lin = state.linear_velocity_substep;
        let ang = state.angular_velocity_substep;

        let mut force_sum = Vec3::ZERO;
        let mut torque_sum = Vec3::ZERO;

        for i in 0..self.positions.len() {
            let vertex_position = rel + basis * self.positions[i];
            let global_vertex_position = global_origin + basis * self.positions[i];
            let factor = basis * self.coefficients[i];
            let radius = self.sizes[i];

            // Air velocity at the vertex (negative of its motion through the air).
            let vertex_velocity = -lin + -ang.cross(vertex_position);

            let (f, t) = self.lift_drag(vertex_position, vertex_velocity, factor, body_density);
            force_sum += f;
            torque_sum += t;

            for atmosphere in atmospheres.iter().filter(|a| a.settings.per_influencer_positioning) {
                let distance = atmosphere.distance_to_surface(global_vertex_position);
                let normal = atmosphere.surface_normal();
                let contact_depth = radius * (1.0 - normal.dot(factor.normalize_or_zero()).abs());
                let density_delta = atmosphere.settings.density - body_density;
                let vertex_density = submerged_density(distance, contact_depth, density_delta);

                let (bf, bt) = self.buoyancy(vertex_position, factor, vertex_density, distance, gravity);
                let (lf, lt) = self.lift_drag(vertex_position, vertex_velocity, factor, vertex_density);
                force_sum += bf + lf;
                torque_sum += bt + lt;
            }
        }

        (force_sum, torque_sum)
    }

    fn buoyancy(&self, position: Vec3, factor: Vec3, density: f32, distance: f32, gravity: Vec3) -> (Vec3, Vec3) {
        if density == 0.0 {
            return (Vec3::ZERO, Vec3::ZERO);
        }
        let force = density * gravity.length() * distance.min(0.0) * factor;
        (force, position.cross(force))
    }

    fn lift_drag(&self, position: Vec3, velocity: Vec3, factor: Vec3, density: f32) -> (Vec3, Vec3) {
        if density == 0.0 {
            return (Vec3::ZERO, Vec3::ZERO);
        }
        let drag_direction = velocity.normalize_or_zero();
        let airspeed = velocity.length();
        let dynamic_pressure = 0.5 * density * airspeed * airspeed;

        // Pressure on the face that points into the flow.
        let mut force = factor * dynamic_pressure * drag_direction.dot(factor.normalize_or_zero()).min(0.0) * self.aero_force_multiplier;
        // Skin friction along the flow.
        force += drag_direction * factor.length() * dynamic_pressure * self.skin_friction;
        (force, position.cross(force))
    }
}

/// The density used for one vertex inside an atmosphere: a linear blend across the contact depth.
///
/// Zero depth (a flat contact) has no blend, so the vertex is either fully submerged or not.
fn submerged_density(distance: f32, contact_depth: f32, density_delta: f32) -> f32 {
    if contact_depth <= 0.0 {
        return if distance < 0.0 { density_delta.max(0.0) } else { 0.0 };
    }
    let value = remap(distance, -contact_depth, contact_depth, density_delta, 0.0);
    value.clamp(0.0, density_delta.max(0.0))
}
