//! `VehicleWheel3D`: suspension and ground friction for wheels on an aerodynamic body.
//!
//! Godot's `VehicleWheel3D` is driven by the VehicleBody3D solver, which this port does not use.
//! Each wheel is instead a ray cast down the body's up axis. A spring and damper push the body up
//! while the ray is shorter than the rest length, and sideways friction opposes sliding.
//! This is an approximation: it has no engine force, steering, or brake support.

use avian3d::prelude::*;
use bevy::math::{Mat3, Vec3};
use bevy::prelude::*;
use serde::Deserialize;

use crate::body::AeroBody;

/// Wheel settings and state. The field names are the Godot `VehicleWheel3D` property names.
#[derive(Deserialize, Debug, Clone)]
#[serde(default)]
pub struct Wheel {
    /// Position of the wheel's suspension mount, in the body's local space.
    #[serde(skip)]
    pub mount: Vec3,
    pub wheel_radius: f32,
    pub suspension_rest_length: f32,
    pub suspension_travel: f32,
    pub suspension_stiffness: f32,
    pub damping_compression: f32,
    pub damping_relaxation: f32,
    pub friction_slip: f32,
    #[serde(skip)]
    pub last_compression: f32,
}

impl Default for Wheel {
    fn default() -> Self {
        Self {
            mount: Vec3::ZERO,
            wheel_radius: 0.5,
            suspension_rest_length: 0.15,
            suspension_travel: 0.2,
            suspension_stiffness: 5.88,
            damping_compression: 0.88,
            damping_relaxation: 0.88,
            friction_slip: 10.5,
            last_compression: 0.0,
        }
    }
}

/// A wheel contact for this step: where the force acts, and the force itself (world space).
#[derive(Debug, Clone, Copy)]
pub struct WheelContact {
    pub point: Vec3,
    pub force: Vec3,
}

/// Ground friction coefficient that caps the sideways force, scaled by the wheel's friction slip.
const FRICTION_COEFFICIENT: f32 = 1.0;

/// Computes each wheel's contact force for this physics step. `hit` casts a ray (origin, direction,
/// max length) and returns the distance to the first solid surface, if any.
#[allow(clippy::too_many_arguments)]
pub fn wheel_contacts(
    wheels: &mut [Wheel],
    body_position: Vec3,
    body_basis: Mat3,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    mass: f32,
    dt: f32,
    hit: &mut impl FnMut(Vec3, Vec3, f32) -> Option<f32>,
) -> Vec<WheelContact> {
    let up = body_basis.y_axis.normalize_or_zero();
    let wheel_count = wheels.len().max(1);
    let mut contacts = Vec::new();
    for wheel in wheels.iter_mut() {
        let hub = body_position + body_basis * wheel.mount;
        let max_length = wheel.suspension_rest_length + wheel.suspension_travel + wheel.wheel_radius;
        let Some(distance) = hit(hub, -up, max_length) else {
            wheel.last_compression = 0.0;
            continue;
        };

        // Spring compression: how much shorter than the rest length the suspension is.
        let suspension_length = (distance - wheel.wheel_radius).max(0.0);
        let compression = wheel.suspension_rest_length - suspension_length;
        if compression <= 0.0 {
            wheel.last_compression = 0.0;
            continue;
        }
        let compression = compression.min(wheel.suspension_travel + wheel.suspension_rest_length);
        let compression_speed = (compression - wheel.last_compression) / dt;
        wheel.last_compression = compression;
        let damping = if compression_speed >= 0.0 { wheel.damping_compression } else { wheel.damping_relaxation };
        // Each wheel carries its share of the mass. The stiffness is a rate (1/s^2), so the spring
        // acceleration is `stiffness * compression` and the force is that times the wheel's share.
        let share = mass / wheel_count as f32;
        let normal_force = share * (wheel.suspension_stiffness * compression - damping * compression_speed).max(0.0);
        let contact = hub - up * distance;

        // Sideways friction: stop the contact point's sliding within one step, up to the friction limit.
        let lever = contact - body_position;
        let contact_velocity = linear_velocity + angular_velocity.cross(lever);
        let sideways = contact_velocity - up * contact_velocity.dot(up);
        let sliding_speed = sideways.length();
        let friction = if sliding_speed > 1e-4 {
            let stopping = sliding_speed * share / dt;
            let limit = FRICTION_COEFFICIENT * wheel.friction_slip / 10.5 * normal_force;
            -sideways / sliding_speed * stopping.min(limit)
        } else {
            Vec3::ZERO
        };

        contacts.push(WheelContact { point: contact, force: up * normal_force + friction });
    }
    contacts
}

/// Ray cast helper for the systems: returns the distance to the first solid hit, excluding the body.
fn cast_down(spatial: &SpatialQuery, body: Entity, origin: Vec3, direction: Vec3, max: f32) -> Option<f32> {
    let filter = SpatialQueryFilter::from_excluded_entities([body]);
    let dir = Dir3::new(direction).ok()?;
    spatial.cast_ray(origin, dir, max, true, &filter).map(|hit| hit.distance)
}

/// Updates the wheel contacts of every body with wheels, before the aero integrator runs.
pub fn wheel_suspension(
    time: Res<Time<Fixed>>,
    spatial: SpatialQuery,
    mut bodies: Query<(Entity, &mut AeroBody, &Position, &Rotation, &LinearVelocity, &AngularVelocity, &ComputedMass)>,
) {
    let dt = time.delta_secs();
    for (entity, mut body, position, rotation, linear, angular, mass) in &mut bodies {
        if body.wheels.is_empty() {
            continue;
        }
        let basis = Mat3::from_quat(rotation.0);
        let mut wheels = std::mem::take(&mut body.wheels);
        let mass_value = mass.value();
        body.wheel_contacts = wheel_contacts(&mut wheels, position.0, basis, linear.0, angular.0, mass_value, dt, &mut |origin, direction, max| {
            cast_down(&spatial, entity, origin, direction, max)
        });
        body.wheels = wheels;
    }
}
