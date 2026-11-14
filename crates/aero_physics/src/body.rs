//! Port of `core/aero_body_3d.gd`: the aerodynamic rigid body, its integrator, and the Bevy systems.

use avian3d::prelude::*;
use bevy::math::{Mat3, Mat4, Vec3};
use bevy::prelude::*;

use crate::atmosphere::{AeroAtmosphere, AtmosphereSnapshot};
use crate::controls::{ActionInputs, ControlConfig};
use crate::flight_assist::FlightAssist;
use crate::godot::{angle_to, euler_yxz, vec_times_basis};
use crate::spawn::RotationFollow;
use crate::influencer::{BodyFrame, Node, ParentFrame, calculate_forces, update_transforms, world_inverse_inertia};
use crate::units::AeroUnits;
use crate::wheels::{Wheel, WheelContact};

/// Global physics settings, the equivalent of the `physics/aerodynamics/*` project settings.
#[derive(Resource, Debug, Clone, Copy)]
pub struct AeroSettings {
    /// Substeps used when an `AeroBody` does not set its own `substeps_override`.
    pub substeps: u32,
}

impl Default for AeroSettings {
    fn default() -> Self {
        Self { substeps: 1 }
    }
}

/// The aerodynamic body. Attach it to an Avian `RigidBody::Dynamic` entity.
///
/// Public fields hold the values the GDScript exposes (air speed, angle of attack, and so on), so
/// instruments and debug drawing can read them.
#[derive(Component, Debug, Clone)]
pub struct AeroBody {
    /// Overrides the number of aerodynamic substeps (`None` uses [`AeroSettings`]).
    pub substeps_override: Option<u32>,
    /// `experimental_energy_tracking`: limits the change in kinetic energy over a step.
    pub experimental_energy_tracking: bool,
    pub show_debug: bool,
    /// The influencer tree, flattened. Indices refer to this vector.
    pub nodes: Vec<Node>,
    /// Indices of the influencers that are direct children of the body.
    pub roots: Vec<usize>,
    pub controls: ControlConfig,
    pub flight_assist: Option<FlightAssist>,
    /// Air volumes this body currently overlaps. Rebuilt every physics tick.
    pub atmospheres: Vec<AtmosphereSnapshot>,

    pub wind: Vec3,
    pub air_velocity: Vec3,
    pub local_air_velocity: Vec3,
    pub local_angular_velocity: Vec3,
    pub air_speed: f32,
    pub mach: f32,
    pub altitude: f32,
    pub air_density: f32,
    pub air_pressure: f32,
    pub angle_of_attack: f32,
    pub sideslip_angle: f32,
    pub bank_angle: f32,
    pub heading: f32,
    pub inclination: f32,
    pub current_force: Vec3,
    pub current_torque: Vec3,
    pub current_gravity: Vec3,
    pub linear_velocity_substep: Vec3,
    pub angular_velocity_substep: Vec3,
    pub linear_acceleration: Vec3,
    pub angular_acceleration: Vec3,
    pub last_linear_velocity: Vec3,
    pub last_angular_velocity: Vec3,
    pub uncommitted_last_linear_velocity: Vec3,
    pub uncommitted_last_angular_velocity: Vec3,
    pub current_substep: u32,
    /// Centre of mass in the body's local space (Godot `center_of_mass` in custom mode).
    pub center_of_mass: Vec3,
    /// `VehicleWheel3D` nodes attached to the body.
    pub wheels: Vec<Wheel>,
    /// Ground contacts of the wheels for the current step, applied by `aero_integrate`.
    pub wheel_contacts: Vec<WheelContact>,
}

/// What the integrator reads from the physics engine each step.
pub struct StepInput {
    pub step: f32,
    pub position: Vec3,
    pub center_of_mass: Vec3,
    pub basis: Mat3,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub gravity: Vec3,
    pub mass: f32,
    pub inverse_inertia_local: Mat3,
}

impl AeroBody {
    pub fn new(nodes: Vec<Node>, roots: Vec<usize>, controls: ControlConfig, flight_assist: Option<FlightAssist>) -> Self {
        Self {
            substeps_override: None,
            experimental_energy_tracking: false,
            show_debug: false,
            nodes,
            roots,
            controls,
            flight_assist,
            atmospheres: Vec::new(),
            wind: Vec3::ZERO,
            air_velocity: Vec3::ZERO,
            local_air_velocity: Vec3::ZERO,
            local_angular_velocity: Vec3::ZERO,
            air_speed: 0.0,
            mach: 0.0,
            altitude: 0.0,
            air_density: 1.225,
            air_pressure: 101_325.0,
            angle_of_attack: 0.0,
            sideslip_angle: 0.0,
            bank_angle: 0.0,
            heading: 0.0,
            inclination: 0.0,
            current_force: Vec3::ZERO,
            current_torque: Vec3::ZERO,
            current_gravity: Vec3::ZERO,
            linear_velocity_substep: Vec3::ZERO,
            angular_velocity_substep: Vec3::ZERO,
            linear_acceleration: Vec3::ZERO,
            angular_acceleration: Vec3::ZERO,
            last_linear_velocity: Vec3::ZERO,
            last_angular_velocity: Vec3::ZERO,
            uncommitted_last_linear_velocity: Vec3::ZERO,
            uncommitted_last_angular_velocity: Vec3::ZERO,
            current_substep: 0,
            center_of_mass: Vec3::ZERO,
            wheels: Vec::new(),
            wheel_contacts: Vec::new(),
        }
    }

    /// Looks up a control command by axis name, as `AeroBody3D.get_control_command()` does.
    pub fn control_command(&self, axis: &str) -> f32 {
        self.controls.control_command(axis)
    }

    /// `AeroControlComponent._physics_process()`: the control axes, then flight assist.
    pub fn update_controls(&mut self, delta: f32, inputs: &ActionInputs, linear_velocity: Vec3, basis: Mat3) {
        self.controls.update(delta, inputs);

        if let Some(fa) = self.flight_assist.as_mut() {
            let c = &mut self.controls;
            fa.control_input = Vec3::new(c.control_command("pitch"), c.control_command("yaw"), c.control_command("roll"));
            fa.throttle_command = c.control_command("throttle");
            fa.air_speed = self.air_speed;
            fa.air_density = self.air_density;
            fa.angle_of_attack = self.angle_of_attack;
            fa.altitude = self.altitude;
            fa.heading = self.heading;
            fa.bank_angle = self.bank_angle;
            fa.inclination = self.inclination;
            fa.linear_velocity = linear_velocity;
            fa.local_angular_velocity = self.local_angular_velocity;
            fa.global_basis = basis;
            fa.update(delta);

            c.set_control_command("pitch", fa.control_command.x);
            c.set_control_command("yaw", fa.control_command.y);
            c.set_control_command("roll", fa.control_command.z);
            c.set_control_command("throttle", fa.throttle_command);
        }
    }

    fn substeps(&self, default: u32) -> u32 {
        self.substeps_override.unwrap_or(default)
    }

    /// `AeroBody3D.integrator(state)`. Returns the average `(force, torque)` for the step.
    pub fn integrate(&mut self, input: &StepInput, units: &AeroUnits, default_substeps: u32) -> (Vec3, Vec3) {
        self.last_linear_velocity = self.uncommitted_last_linear_velocity;
        self.last_angular_velocity = self.uncommitted_last_angular_velocity;

        self.current_gravity = input.gravity;
        self.center_of_mass = input.center_of_mass;
        self.linear_velocity_substep = input.linear_velocity;
        self.angular_velocity_substep = input.angular_velocity;

        let (force, torque) = self.calculate_forces(input, units, default_substeps);

        self.linear_acceleration = (input.linear_velocity - self.last_linear_velocity) / input.step;
        self.angular_acceleration = (input.angular_velocity - self.last_angular_velocity) / input.step;

        self.uncommitted_last_linear_velocity = input.linear_velocity;
        self.uncommitted_last_angular_velocity = input.angular_velocity;

        (force, torque)
    }

    /// `AeroBody3D.calculate_forces(delta)`.
    fn calculate_forces(&mut self, input: &StepInput, units: &AeroUnits, default_substeps: u32) -> (Vec3, Vec3) {
        let substeps = self.substeps(default_substeps);
        if substeps == 0 {
            return (Vec3::ZERO, Vec3::ZERO);
        }

        let basis = input.basis;
        let position = input.position;
        self.altitude = AeroUnits::altitude_of(position.y);
        self.mach = units.speed_to_mach_at_altitude(self.air_speed, self.altitude);
        self.air_density = units.density_at_altitude(self.altitude);
        self.air_pressure = units.pressure_at_altitude(self.altitude);

        // Body-level volumes set the wind and density. Per-influencer volumes are handled per node.
        self.wind = Vec3::ZERO;
        for atmosphere in self.atmospheres.iter().filter(|a| !a.settings.per_influencer_positioning) {
            self.wind += atmosphere.wind_at();
            if atmosphere.settings.override_density {
                self.air_density = atmosphere.density_at(position);
            }
        }

        let euler = euler_yxz(basis);
        self.bank_angle = euler.z;
        self.heading = euler.y;
        self.inclination = euler.x;

        let sub_delta = input.step / substeps as f32;
        // Godot's get_relative_position(): the body's centre of mass, in world orientation.
        let body_relative = basis * -self.center_of_mass;
        let inverse_inertia_world = world_inverse_inertia(basis, input.inverse_inertia_local);

        let mut total_force = Vec3::ZERO;
        let mut total_torque = Vec3::ZERO;
        let mut lin = input.linear_velocity;
        let mut ang = input.angular_velocity;

        for substep in 0..substeps {
            self.current_substep = substep;

            self.air_velocity = -lin + self.wind;
            self.air_speed = self.air_velocity.length();
            self.local_air_velocity = vec_times_basis(self.air_velocity, basis);
            self.local_angular_velocity = vec_times_basis(ang, basis);
            self.angle_of_attack = angle_to(basis.y_axis, -self.air_velocity) - std::f32::consts::FRAC_PI_2;
            self.sideslip_angle = angle_to(basis.x_axis, self.air_velocity) - std::f32::consts::FRAC_PI_2;

            let frame = BodyFrame {
                wind: self.wind,
                air_density: self.air_density,
                altitude: self.altitude,
                gravity: input.gravity,
                body_basis: basis,
                body_relative,
                substep_delta: sub_delta,
                current_substep: substep,
                atmospheres: &self.atmospheres,
                units,
                control: &self.controls,
            };

            for &root in &self.roots {
                if !self.nodes[root].disabled {
                    update_transforms(&mut self.nodes, root, &frame);
                }
            }

            let parent = ParentFrame {
                global_origin: position,
                global_basis: basis,
                relative_position: body_relative,
                linear_velocity: lin,
                angular_velocity: ang,
            };
            let mut substep_force = Vec3::ZERO;
            let mut substep_torque = Vec3::ZERO;
            for &root in &self.roots {
                if self.nodes[root].disabled {
                    continue;
                }
                let (f, t) = calculate_forces(&mut self.nodes, root, parent, &frame);
                substep_force += f;
                substep_torque += t;
            }

            total_force += substep_force;
            total_torque += substep_torque;

            lin += substep_force / input.mass * sub_delta + input.gravity * sub_delta;
            ang += inverse_inertia_world * substep_torque * sub_delta;
            self.linear_velocity_substep = lin;
            self.angular_velocity_substep = ang;
        }

        let average_force = total_force / substeps as f32;
        let average_torque = total_torque / substeps as f32;
        let (average_force, average_torque) = if self.experimental_energy_tracking {
            energy_tracked(input, inverse_inertia_world, average_force, average_torque)
        } else {
            (average_force, average_torque)
        };
        self.current_force = average_force;
        self.current_torque = average_torque;
        (average_force, average_torque)
    }
}

/// `experimental_energy_tracking` (`AeroBody3D.calculate_forces`): converts the step's force and torque
/// into velocity changes that do not add more kinetic energy than the body had.
fn energy_tracked(input: &StepInput, inverse_inertia_world: Mat3, force: Vec3, torque: Vec3) -> (Vec3, Vec3) {
    let delta = input.step;
    let mass = input.mass;
    let linear = input.linear_velocity;
    let angular = input.angular_velocity;

    // Linear kinetic energy before and after the force.
    let pre_kinetic = 0.5 * mass * linear.length_squared();
    let linear_change = force * delta / mass;
    let post_kinetic = 0.5 * mass * (linear + linear_change).length_squared();

    // Rotational kinetic energy about the spin axis.
    let real_inertia = inverse_inertia_world.inverse();
    let axis = angular.normalize_or_zero();
    let inertia_about_axis = (real_inertia * axis).dot(axis);
    let pre_rotational = 0.5 * inertia_about_axis * angular.length_squared();
    let angular_change = inverse_inertia_world * (torque * delta);
    let post_rotational = 0.5 * inertia_about_axis * (angular + angular_change).length_squared();

    // Energy is clamped, to avoid integration errors when the forces are large.
    let total_post = post_kinetic + post_rotational;
    let clamped = total_post.clamp(0.0, pre_kinetic + pre_rotational);
    let angular_proportion = if total_post > 0.0 { post_rotational / total_post } else { 0.0 };

    let linear_energy = clamped * (1.0 - angular_proportion);
    let new_linear = (linear + linear_change).normalize_or_zero() * (linear_energy / (0.5 * mass)).max(0.0).sqrt();
    let new_force = (new_linear - linear) * mass / delta;

    let angular_energy = clamped * angular_proportion;
    let new_angular = if inertia_about_axis != 0.0 {
        (angular + angular_change).normalize_or_zero() * (angular_energy / (0.5 * inertia_about_axis)).max(0.0).sqrt()
    } else {
        Vec3::ZERO
    };
    let new_torque = (new_angular - angular) * inertia_about_axis / delta;

    (new_force, new_torque)
}

/// Rebuilds the list of air volumes each body overlaps, from the sensor contacts Avian reports.
pub fn update_atmosphere_membership(
    mut bodies: Query<&mut AeroBody>,
    atmospheres: Query<(Entity, &GlobalTransform, &AeroAtmosphere)>,
    sensors: Query<(&ChildOf, &CollidingEntities), With<Sensor>>,
    colliders: Query<&ColliderOf>,
) {
    for mut body in &mut bodies {
        body.atmospheres.clear();
    }
    for (parent, contacts) in &sensors {
        let Ok((atmosphere, transform, settings)) = atmospheres.get(parent.parent()) else {
            continue;
        };
        for &collider in contacts.iter() {
            let Ok(owner) = colliders.get(collider) else {
                continue;
            };
            let Ok(mut body) = bodies.get_mut(owner.body) else {
                continue;
            };
            if body.atmospheres.iter().any(|s| s.entity == atmosphere) {
                continue;
            }
            body.atmospheres.push(AtmosphereSnapshot {
                entity: atmosphere,
                position: transform.translation(),
                settings: settings.clone(),
            });
        }
    }
}

/// `AeroControlComponent._physics_process()`, run before the physics step.
pub fn update_aero_controls(
    time: Res<Time<Fixed>>,
    inputs: Res<ActionInputs>,
    mut bodies: Query<(&mut AeroBody, &LinearVelocity, &Rotation)>,
) {
    let delta = time.delta_secs();
    for (mut body, linear_velocity, rotation) in &mut bodies {
        let basis = Mat3::from_quat(rotation.0);
        body.update_controls(delta, &inputs, linear_velocity.0, basis);
    }
}

/// `AeroBody3D._integrate_forces()`: runs the aerodynamic model and applies its result to the body.
#[allow(clippy::type_complexity)]
pub fn aero_integrate(
    time: Res<Time<Fixed>>,
    gravity: Res<Gravity>,
    units: Res<AeroUnits>,
    settings: Res<AeroSettings>,
    mut bodies: Query<(Forces, &mut AeroBody, &ComputedMass, &ComputedAngularInertia, &CenterOfMass, Has<Sleeping>)>,
) {
    let step = time.delta_secs();
    for (mut forces, mut body, mass, inertia, com, sleeping) in &mut bodies {
        if sleeping || body.substeps(settings.substeps) == 0 {
            continue;
        }
        let rotation = forces.rotation();
        let input = StepInput {
            step,
            position: forces.position(),
            center_of_mass: com.0,
            basis: Mat3::from_quat(rotation),
            linear_velocity: forces.linear_velocity(),
            angular_velocity: forces.angular_velocity(),
            gravity: gravity.0,
            mass: mass.value(),
            inverse_inertia_local: Mat3::from(inertia.inverse()),
        };
        let (force, torque) = body.integrate(&input, &units, settings.substeps);
        forces.apply_force(force);
        forces.apply_torque(torque);
        for contact in &body.wheel_contacts {
            forces.apply_force_at_point(contact.force, contact.point);
        }
    }
}

/// Copies the influencer poses into the Bevy transforms of their entities, for drawing.
/// Entities driven by a `RotationFollow` link are left to that link.
pub fn sync_influencer_transforms(bodies: Query<&AeroBody>, mut transforms: Query<&mut Transform, Without<RotationFollow>>) {
    for body in &bodies {
        for node in &body.nodes {
            if let Ok(mut transform) = transforms.get_mut(node.entity) {
                let b = node.basis;
                let matrix = Mat4::from_cols(b.x_axis.extend(0.0), b.y_axis.extend(0.0), b.z_axis.extend(0.0), node.origin.extend(1.0));
                *transform = Transform::from_matrix(matrix);
            }
        }
    }
}
