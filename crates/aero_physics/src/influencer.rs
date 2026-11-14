//! The influencer tree: data model plus the recursive transform-update and force passes.
//!
//! Godot's `AeroInfluencer3D` classes call `super()` in a chain. Here the same chain is written as
//! one recursive pass per phase, with the class behaviour selected by [`Kind`]. The tree lives in
//! a flat `Vec<Node>` owned by the [`AeroBody`](crate::body::AeroBody), so the recursion needs no
//! ECS borrows.

use bevy::math::{Mat3, Quat, Vec2, Vec3};
use bevy::prelude::Entity;

use crate::controls::{ActuationConfig, ControlConfig};
use crate::curve::Curve;
use crate::atmosphere::AtmosphereSnapshot;
use crate::buoyancy::MeshBuoyancy;
use crate::godot::{EulerOrder, basis_from_euler, euler_from_basis, euler_yxz, vec_times_basis};
use crate::schema::{ManualSurfaceConfig, SurfaceConfig};
use crate::units::AeroUnits;
use crate::expr::ControlSource;

/// One influencer (the Godot `AeroInfluencer3D` and its subclasses).
#[derive(Debug, Clone)]
pub struct Node {
    /// The Bevy entity that draws this node. Its `Transform` is synced from `basis` and `origin`.
    pub entity: Entity,
    pub name: String,
    pub children: Vec<usize>,
    pub disabled: bool,
    pub omit_from_debug: bool,
    pub is_duplicate: bool,
    pub mirror_axis: u8,
    pub mirror_scaling_fix_axis: u8,
    /// Godot `Node3D.rotation_order`, used when reading `rotation.y` for cyclic control.
    pub rotation_order: EulerOrder,
    pub control_rotation_order: EulerOrder,
    pub actuation: Option<ActuationConfig>,
    /// Local transform (`transform`): `basis` and `origin`.
    pub basis: Mat3,
    pub origin: Vec3,
    /// `default_transform`: the base pose that actuation and movers build on.
    pub default_basis: Mat3,
    pub default_origin: Vec3,
    pub state: NodeState,
    pub kind: Kind,
}

/// Values computed each substep. Mirrors the fields the GDScript keeps on the node.
#[derive(Debug, Clone, Copy, Default)]
pub struct NodeState {
    pub global_basis: Mat3,
    pub global_origin: Vec3,
    pub relative_position: Vec3,
    pub linear_velocity_substep: Vec3,
    pub angular_velocity_substep: Vec3,
    pub world_air_velocity: Vec3,
    pub air_speed: f32,
    pub air_density: f32,
    pub dynamic_pressure: f32,
    pub drag_direction: Vec3,
    pub mach: f32,
    pub current_force: Vec3,
    pub current_torque: Vec3,
}

/// The class of an influencer. Subclass data is nested, matching the inheritance chain.
#[derive(Debug, Clone)]
pub enum Kind {
    /// `AeroInfluencer3D`: a pure transform node.
    Base,
    /// `AeroSurface3D`.
    Surface(SurfaceState),
    /// `ManualAeroSurface3D`.
    ManualSurface(Box<ManualSurface>),
    /// `AeroDragInfluencer3D`.
    Drag { radius: f32, drag_coefficient: f32 },
    /// `AeroThruster3D`, optionally with the jet-engine model of `AeroJetThruster3D`.
    Thruster(Box<ThrusterState>),
    /// `AeroMover3D`.
    Mover(MoverState),
    /// `AeroPropeller3D`.
    Propeller(Box<PropellerState>),
    /// `AeroCyclicPropeller3D`.
    CyclicPropeller(Box<CyclicState>),
    /// `AeroBuoyancySphere3D`.
    BuoyancySphere(SphereBuoyancy),
    /// `AeroBuoyancyMesh3D`.
    BuoyancyMesh(Box<MeshBuoyancy>),
}

#[derive(Debug, Clone, Default)]
pub struct SurfaceState {
    pub wing: SurfaceConfig,
    pub angle_of_attack: f32,
    pub sweep_angle: f32,
    pub area: f32,
    pub projected_wing_area: f32,
    pub aero_reference: f32,
    pub lift_direction: Vec3,
    pub lift_force: f32,
    pub drag_force: f32,
}

#[derive(Debug, Clone)]
pub struct ManualSurface {
    pub surface: SurfaceState,
    pub lift_curve: Curve,
    pub drag_aoa_curve: Curve,
    pub sweep_curve: Curve,
    pub mach_curve: Curve,
    pub min_drag_coefficient: f32,
    pub max_drag_coefficient: f32,
}

impl ManualSurface {
    pub fn new(wing: SurfaceConfig, config: &ManualSurfaceConfig) -> Self {
        Self {
            surface: SurfaceState { wing, ..Default::default() },
            lift_curve: config.lift_curve(),
            drag_aoa_curve: config.drag_aoa_curve(),
            sweep_curve: config.sweep_drag_multiplier_curve(),
            mach_curve: config.drag_at_mach_multiplier_curve(),
            min_drag_coefficient: config.min_drag_coefficient,
            max_drag_coefficient: config.max_drag_coefficient,
        }
    }

    fn lift_coefficient(&self, aoa: f32) -> f32 {
        let c = &self.lift_curve;
        c.sample_baked(crate::godot::remap(aoa, -std::f32::consts::PI, std::f32::consts::PI, c.min_domain, c.max_domain))
    }

    fn drag_coefficient(&self, aoa: f32) -> f32 {
        let c = &self.drag_aoa_curve;
        let sample = c.sample_baked(crate::godot::remap(aoa, -std::f32::consts::PI, std::f32::consts::PI, c.min_domain, c.max_domain));
        crate::godot::remap(sample, c.min_value, c.max_value, self.min_drag_coefficient, self.max_drag_coefficient)
    }

    fn drag_at_sweep(&self, sweep: f32) -> f32 {
        let c = &self.sweep_curve;
        c.sample_baked(crate::godot::remap(sweep, 0.0, std::f32::consts::PI, c.min_domain, c.max_domain).abs())
    }

    fn drag_at_mach(&self, mach: f32) -> f32 {
        self.mach_curve.sample_baked(mach)
    }
}

#[derive(Debug, Clone)]
pub struct JetParams {
    pub intake_area: f32,
    pub intake_fan_max_velocity: f32,
    pub exhaust_velocity: f32,
    pub max_fuel_flow: f32,
}

#[derive(Debug, Clone)]
pub struct ThrusterState {
    pub max_thrust_force: f32,
    pub throttle: Vec3,
    pub throttle_config: Option<ActuationConfig>,
    pub jet: Option<JetParams>,
}

/// `AeroMover3D` state: motors and the velocities they imply.
#[derive(Debug, Clone)]
pub struct MoverState {
    pub linear_motor: Vec3,
    pub angular_motor: Vec3,
    pub disable_lift_dissymmetry: bool,
    pub use_movement_as_velocity: bool,
    pub last_position: Vec3,
    pub last_rotation: Mat3,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

impl Default for MoverState {
    fn default() -> Self {
        Self {
            linear_motor: Vec3::ZERO,
            angular_motor: Vec3::ZERO,
            disable_lift_dissymmetry: false,
            use_movement_as_velocity: false,
            last_position: Vec3::ZERO,
            last_rotation: Mat3::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PropellerState {
    pub mover: MoverState,
    pub amount: u32,
    /// Node index of the blade used as the template for the propeller's copies.
    pub blade: Option<usize>,
    /// Node indices of every blade, including the template.
    pub instances: Vec<usize>,
    pub speed_config: Option<ActuationConfig>,
}

#[derive(Debug, Clone)]
pub struct CyclicState {
    pub propeller: PropellerState,
    pub cyclic: Vec2,
    pub cyclic_pitch: f32,
    pub cyclic_config: Option<ActuationConfig>,
}

#[derive(Debug, Clone, Copy)]
pub struct SphereBuoyancy {
    pub radius: f32,
    pub buoyancy_force: Vec3,
    pub cached_force: Vec3,
    pub cached_torque: Vec3,
}

/// Everything about the parent body that the influencer pass needs for one substep.
pub struct BodyFrame<'a> {
    pub wind: Vec3,
    pub air_density: f32,
    pub altitude: f32,
    pub gravity: Vec3,
    pub body_basis: Mat3,
    /// `relative_position` of the body's direct children: `body_basis * -center_of_mass`.
    pub body_relative: Vec3,
    pub substep_delta: f32,
    pub current_substep: u32,
    pub atmospheres: &'a [AtmosphereSnapshot],
    pub units: &'a AeroUnits,
    pub control: &'a ControlConfig,
}

/// The parent's values that a child inherits during the force pass.
#[derive(Clone, Copy)]
pub struct ParentFrame {
    pub global_origin: Vec3,
    pub global_basis: Mat3,
    pub relative_position: Vec3,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

/// Adapts the body's control axes to the `get_control_command` lookups used by actuation.
struct Control<'a>(&'a ControlConfig);

impl ControlSource for Control<'_> {
    fn control_command(&self, axis: &str) -> f32 {
        self.0.control_command(axis)
    }
}

impl Node {
    pub fn mover(&self) -> Option<&MoverState> {
        match &self.kind {
            Kind::Mover(m) => Some(m),
            Kind::Propeller(p) => Some(&p.mover),
            Kind::CyclicPropeller(c) => Some(&c.propeller.mover),
            _ => None,
        }
    }
}

fn mover_of(kind: &mut Kind) -> Option<&mut MoverState> {
    match kind {
        Kind::Mover(m) => Some(m),
        Kind::Propeller(p) => Some(&mut p.mover),
        Kind::CyclicPropeller(c) => Some(&mut c.propeller.mover),
        _ => None,
    }
}

/// `AeroMover3D._update_transform_substep()`, up to the call to `super`.
fn mover_motor(node: &mut Node, dt: f32) {
    let Node { kind, default_basis, default_origin, basis, origin, .. } = node;
    let Some(m) = mover_of(kind) else {
        return;
    };

    m.linear_velocity = Vec3::ZERO;
    m.angular_velocity = Vec3::ZERO;

    if m.use_movement_as_velocity {
        // Velocity caused by moving the node, minus whatever the motor already accounts for.
        m.linear_velocity = (*origin - m.last_position) / dt;
        m.linear_velocity -= m.linear_motor;

        let rotation_delta = *basis * m.last_rotation.inverse();
        let (axis, angle) = Quat::from_mat3(&rotation_delta).to_axis_angle();
        m.angular_velocity = vec_times_basis(axis * angle / dt, *basis) * 2.0;
        m.angular_velocity -= m.angular_motor;
    }

    // Motors only move the default transform, as in the original (the node's origin is not touched).
    *default_origin += vec_times_basis(m.linear_motor, *basis) * dt;
    if m.angular_motor.length_squared() > f32::EPSILON {
        *default_basis *= Mat3::from_axis_angle(m.angular_motor.normalize(), m.angular_motor.length() * dt);
    }

    m.linear_velocity += m.linear_motor;
    m.angular_velocity += m.angular_motor;

    m.last_position = *origin;
    m.last_rotation = *basis;
}

/// Phase 1 of a substep: update transforms bottom-up (children first), then this node's control.
pub fn update_transforms(nodes: &mut [Node], index: usize, frame: &BodyFrame) {
    mover_motor(&mut nodes[index], frame.substep_delta);

    for i in 0..nodes[index].children.len() {
        let child = nodes[index].children[i];
        if !nodes[child].disabled {
            update_transforms(nodes, child, frame);
        }
    }

    control_transform(nodes, index, frame);
}

/// `AeroInfluencer3D._update_control_transform()` and its subclass overrides.
fn control_transform(nodes: &mut [Node], index: usize, frame: &BodyFrame) {
    let control = Control(frame.control);
    let dt = frame.substep_delta;
    let body_basis = frame.body_basis;

    let node = &mut nodes[index];
    let relative_position = node.state.relative_position;
    let actuation_value = match &mut node.actuation {
        Some(actuation) => actuation.update(relative_position, body_basis, &control, dt),
        None => Vec3::ZERO,
    };
    node.basis = node.default_basis * basis_from_euler(actuation_value, node.control_rotation_order);

    // Cyclic propellers need the blades' data, so their update is applied after this borrow ends.
    let mut cyclic_update: Option<(Vec2, f32, Vec<usize>)> = None;
    match &mut node.kind {
        Kind::Thruster(t) => {
            if let Some(config) = &mut t.throttle_config {
                t.throttle = config.update(relative_position, body_basis, &control, dt);
            }
        }
        Kind::Propeller(p) => {
            if let Some(config) = &mut p.speed_config {
                p.mover.angular_motor = config.update(relative_position, body_basis, &control, dt);
            }
        }
        Kind::CyclicPropeller(c) => {
            if let Some(config) = &mut c.propeller.speed_config {
                c.propeller.mover.angular_motor = config.update(relative_position, body_basis, &control, dt);
            }
            if let Some(config) = &mut c.cyclic_config {
                let value = config.update(relative_position, body_basis, &control, dt);
                c.cyclic = Vec2::new(value.x, value.y);
            }
            let rotor_offset = euler_from_basis(node.basis, EulerOrder::Xzy).y;
            cyclic_update = Some((c.cyclic, rotor_offset, c.propeller.instances.clone()));
        }
        _ => {}
    }

    if let Some((cyclic, rotor_offset, instances)) = cyclic_update {
        let cyclic_pitch = match &nodes[index].kind {
            Kind::CyclicPropeller(c) => c.cyclic_pitch,
            _ => 0.0,
        };
        for instance in instances {
            let inst = &mut nodes[instance];
            let angular_position = rotor_offset + euler_from_basis(inst.basis, inst.rotation_order).y;
            let cyclic_effect = (angular_position.cos() * cyclic.x + angular_position.sin() * cyclic.y) * cyclic_pitch.to_radians();
            let euler = euler_yxz(inst.default_basis);
            inst.default_basis = basis_from_euler(Vec3::new(cyclic_effect, euler.y, euler.z), EulerOrder::Xyz);
        }
    }
}

/// Phase 2 of a substep: compute forces top-down, returning the summed `(force, torque)` of this
/// node and its children.
pub fn calculate_forces(nodes: &mut [Node], index: usize, parent: ParentFrame, frame: &BodyFrame) -> (Vec3, Vec3) {
    // Global transform and the velocities of this node's origin.
    let (position, basis_local) = (nodes[index].origin, nodes[index].basis);
    let global_basis = parent.global_basis * basis_local;
    let global_origin = parent.global_origin + parent.global_basis * position;
    let relative_position = parent.relative_position + parent.global_basis * position;
    let mut linear = parent.linear_velocity + parent.angular_velocity.cross(parent.global_basis * position);
    let mut angular = parent.angular_velocity;
    if let Some(m) = nodes[index].mover() {
        let (mlin, mang, disable_dissymmetry) = (m.linear_velocity, m.angular_velocity, m.disable_lift_dissymmetry);
        linear += vec_times_basis(mlin, global_basis.inverse());
        if disable_dissymmetry {
            let up = global_basis.y_axis.normalize_or_zero();
            linear = up * linear.dot(global_basis.y_axis);
        }
        angular += vec_times_basis(mang, global_basis.inverse());
    }

    {
        let state = &mut nodes[index].state;
        state.global_basis = global_basis;
        state.global_origin = global_origin;
        state.relative_position = relative_position;
        state.linear_velocity_substep = linear;
        state.angular_velocity_substep = angular;
    }

    // Air around this node. Starts from the body's wind, then per-influencer volumes override it.
    let mut air_density = frame.air_density;
    let mut world_air_velocity = -linear + frame.wind;
    for atmosphere in frame.atmospheres.iter().filter(|a| a.settings.per_influencer_positioning) {
        if !atmosphere.is_inside(global_origin) {
            continue;
        }
        if atmosphere.settings.override_density {
            air_density = atmosphere.density_at(global_origin);
        }
        if atmosphere.settings.override_wind {
            world_air_velocity = -linear + atmosphere.wind_at();
        }
    }
    let air_speed = world_air_velocity.length();
    let dynamic_pressure = 0.5 * air_density * air_speed * air_speed;
    let drag_direction = world_air_velocity.normalize_or_zero();
    let mach = frame.units.speed_to_mach_at_altitude(air_speed, frame.altitude);
    {
        let state = &mut nodes[index].state;
        state.air_density = air_density;
        state.world_air_velocity = world_air_velocity;
        state.air_speed = air_speed;
        state.dynamic_pressure = dynamic_pressure;
        state.drag_direction = drag_direction;
        state.mach = mach;
        state.current_force = Vec3::ZERO;
        state.current_torque = Vec3::ZERO;
    }

    // Children first; their sum is what this node returns, plus its own contribution.
    let child_parent = ParentFrame { global_origin, global_basis, relative_position, linear_velocity: linear, angular_velocity: angular };
    let mut force = Vec3::ZERO;
    let mut torque = Vec3::ZERO;
    for i in 0..nodes[index].children.len() {
        let child = nodes[index].children[i];
        if nodes[child].disabled {
            continue;
        }
        let (f, t) = calculate_forces(nodes, child, child_parent, frame);
        force += f;
        torque += t;
    }

    let (own_force, own_torque) = own_forces(nodes, index, frame);
    let state = &mut nodes[index].state;
    state.current_force = own_force;
    state.current_torque = own_torque;
    (force + own_force, torque + own_torque)
}

/// The class-specific force of one node.
fn own_forces(nodes: &mut [Node], index: usize, frame: &BodyFrame) -> (Vec3, Vec3) {
    let node = &mut nodes[index];
    let state = node.state;
    let rel = state.relative_position;
    let basis = state.global_basis;
    match &mut node.kind {
        Kind::Base | Kind::Mover(_) | Kind::Propeller(_) | Kind::CyclicPropeller(_) => (Vec3::ZERO, Vec3::ZERO),
        Kind::Surface(s) => {
            surface_values(s, &state, basis);
            (Vec3::ZERO, Vec3::ZERO)
        }
        Kind::ManualSurface(m) => {
            surface_values(&mut m.surface, &state, basis);
            let aero_reference = m.surface.aero_reference;
            let lift_coefficient = m.lift_coefficient(m.surface.angle_of_attack);
            let drag_coefficient = m.drag_coefficient(m.surface.angle_of_attack);
            let drag_at_sweep = m.drag_at_sweep(m.surface.sweep_angle);
            let drag_at_mach = m.drag_at_mach(state.mach);
            m.surface.lift_force = aero_reference * lift_coefficient;
            m.surface.drag_force = aero_reference * drag_coefficient * drag_at_sweep * drag_at_mach;
            let lift = m.surface.lift_direction * m.surface.lift_force;
            let drag = state.drag_direction * m.surface.drag_force;
            let f = lift + drag;
            (f, rel.cross(f))
        }
        Kind::Drag { radius, drag_coefficient } => {
            let _ = drag_coefficient;
            let area = std::f32::consts::PI * *radius * *radius;
            let f = state.dynamic_pressure * area * state.drag_direction;
            (f, rel.cross(f))
        }
        Kind::Thruster(t) => {
            let thrust = match &t.jet {
                Some(jet) => t.throttle * jet_mass_flow_force(jet, state.air_density, state.air_speed, t.throttle),
                None => t.max_thrust_force * t.throttle,
            };
            let f = basis * thrust;
            (f, rel.cross(f))
        }
        Kind::BuoyancySphere(b) => {
            if frame.current_substep == 0 {
                b.buoyancy_force = sphere_buoyancy(b.radius, state.global_origin, frame);
                b.cached_force = b.buoyancy_force;
                b.cached_torque = rel.cross(b.buoyancy_force);
            }
            (b.cached_force, b.cached_torque)
        }
        Kind::BuoyancyMesh(mesh) => mesh.forces(&state, frame.gravity, frame.air_density, frame.atmospheres),
    }
}

/// `AeroSurface3D._calculate_forces()` after `super`: angles and reference values for the wing.
fn surface_values(s: &mut SurfaceState, state: &NodeState, basis: Mat3) {
    let world_air = state.world_air_velocity;
    s.angle_of_attack = angle_between(basis.y_axis, -world_air) - std::f32::consts::FRAC_PI_2;
    s.sweep_angle = angle_between(basis.x_axis, -world_air) - std::f32::consts::FRAC_PI_2;
    s.area = s.wing.area();
    s.projected_wing_area = (s.area * s.angle_of_attack.sin()).abs();
    s.aero_reference = state.dynamic_pressure * s.area;
    let right_facing = world_air.cross(-basis.y_axis).normalize_or_zero();
    s.lift_direction = state.drag_direction.cross(right_facing).normalize_or_zero();
}

fn angle_between(a: Vec3, b: Vec3) -> f32 {
    crate::godot::angle_to(a, b)
}

/// `AeroJetThruster3D.calculate_mass_flow_acceleration_force()`.
fn jet_mass_flow_force(jet: &JetParams, air_density: f32, intake_air_velocity: f32, throttle: Vec3) -> f32 {
    let intake_mass_flow = air_density * intake_air_velocity * jet.intake_area;
    let fuel_burn = jet.max_fuel_flow * throttle.length();
    let exhaust_velocity = jet.exhaust_velocity;
    let exhaust_mass_flow = intake_mass_flow + air_density * jet.intake_fan_max_velocity * throttle.length() * jet.intake_area + fuel_burn;
    exhaust_mass_flow * exhaust_velocity - intake_mass_flow * intake_air_velocity
}

/// `AeroBuoyancySphere3D.calculate_buoyancy()`.
fn sphere_buoyancy(radius: f32, global_origin: Vec3, frame: &BodyFrame) -> Vec3 {
    let mut force = Vec3::ZERO;
    for atmosphere in frame.atmospheres.iter().filter(|a| a.settings.per_influencer_positioning) {
        let submerged_height = (-(atmosphere.distance_to_surface(global_origin) - radius)).clamp(0.0, 2.0 * radius);
        let submerged_volume = (std::f32::consts::PI * submerged_height * submerged_height) / 3.0 * (3.0 * radius - submerged_height);
        force += atmosphere.settings.density * -frame.gravity * submerged_volume;
    }
    force
}

/// The world-space inverse inertia tensor of the body, given the local one.
pub fn world_inverse_inertia(body_basis: Mat3, local_inverse: Mat3) -> Mat3 {
    body_basis * local_inverse * body_basis.transpose()
}
