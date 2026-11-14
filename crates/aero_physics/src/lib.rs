//! `aero_physics`: a Bevy port of the Godot "godot_aerodynamic_physics" plugin.
//!
//! The model is the same as the original: aerodynamic influencers (wings, drag bodies, thrusters,
//! propellers, buoyancy meshes) are children of an [`AeroBody`], and each physics step their
//! forces are summed and applied to the Avian rigid body.
//!
//! Setup:
//! 1. Add Avian's `PhysicsPlugins` (with a 60 Hz fixed timestep, as in Godot's default).
//! 2. Add [`AeroPhysicsPlugin`].
//! 3. Spawn a scene with [`spawn::spawn_scene`], or add [`AeroBody`] to a `RigidBody::Dynamic`.
//! 4. Write input strengths into [`controls::ActionInputs`] each frame.

pub mod atmosphere;
pub mod body;
pub mod buoyancy;
pub mod controls;
pub mod curve;
pub mod expr;
pub mod flight_assist;
pub mod godot;
pub mod influencer;
pub mod pid;
pub mod schema;
pub mod spawn;
pub mod units;
pub mod wheels;

use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::math::Mat3;
use bevy::transform::TransformSystems;

pub use atmosphere::{AeroAtmosphere, AtmosphereKind};
pub use body::{AeroBody, AeroSettings};
pub use controls::{ActionInputs, ControlConfig};
pub use flight_assist::FlightAssist;
pub use spawn::{AircraftPaths, NodeHook, RotationFollow, SpawnAssets, spawn_scene};
pub use units::AeroUnits;

/// Adds the aerodynamic model. Requires Avian's `PhysicsPlugins` and a fixed timestep.
pub struct AeroPhysicsPlugin;

impl Plugin for AeroPhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<AeroSettings>()
            .init_resource::<AeroUnits>()
            .init_resource::<ActionInputs>()
            .add_systems(FixedPreUpdate, (body::update_atmosphere_membership, body::update_aero_controls).chain())
            .add_systems(
                FixedPostUpdate,
                (wheels::wheel_suspension, body::aero_integrate).chain().in_set(PhysicsSystems::Prepare),
            )
            .add_systems(
                PostUpdate,
                (body::sync_influencer_transforms, apply_rotation_follow).chain().before(TransformSystems::Propagate),
            );
    }
}

/// Applies the `RotationFollow` links: `target.rotation[axis] = sign * source.rotation[axis] + offset`.
#[allow(clippy::type_complexity)]
fn apply_rotation_follow(mut set: ParamSet<(Query<&Transform>, Query<(Entity, &RotationFollow, &mut Transform)>)>) {
    let links: Vec<(Entity, RotationFollow)> = set.p1().iter().map(|(entity, follow, _)| (entity, follow.clone())).collect();
    for (target, follow) in links {
        let Ok(source) = set.p0().get(follow.source).map(|t| t.rotation) else {
            continue;
        };
        let angle = godot::euler_from_basis(Mat3::from_quat(source), follow.source_order)[follow.source_axis] * follow.sign + follow.offset;
        if let Ok((_, _, mut transform)) = set.p1().get_mut(target) {
            let mut euler = godot::euler_from_basis(Mat3::from_quat(transform.rotation), follow.target_order);
            euler[follow.axis] = angle;
            transform.rotation = Quat::from_mat3(&godot::basis_from_euler(euler, follow.target_order));
        }
    }
}
