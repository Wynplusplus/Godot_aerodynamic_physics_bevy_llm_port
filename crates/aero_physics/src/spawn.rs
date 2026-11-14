//! Builds an aircraft (or any converted scene) from a [`SceneFile`](crate::schema::SceneFile).

use std::collections::HashMap;

use avian3d::prelude::*;
use bevy::asset::AssetServer;
use bevy::gltf::GltfAssetLabel;
use bevy::math::{Mat3, Vec3};
use bevy::prelude::*;
use serde_json::Value;

use crate::atmosphere::AeroAtmosphere;
use crate::body::AeroBody;
use crate::buoyancy::MeshBuoyancy;
use crate::controls::{ActuationConfig, ControlConfig};
use crate::flight_assist::FlightAssist;
use crate::godot::EulerOrder;
use crate::influencer::{
    CyclicState, JetParams, Kind, ManualSurface, MoverState, Node, NodeState, PropellerState, SphereBuoyancy, SurfaceState,
    ThrusterState,
};
use crate::schema::{ManualSurfaceConfig, NodeDef, SceneFile, ShapeDef, SurfaceConfig};
use crate::wheels::Wheel;

/// Classes that are aerodynamic influencers, as they appear in the converted scenes.
pub const INFLUENCER_CLASSES: &[&str] = &[
    "AeroInfluencer3D",
    "AeroSurface3D",
    "ManualAeroSurface3D",
    "AeroDragInfluencer3D",
    "AeroThruster3D",
    "AeroJetThruster3D",
    "AeroMover3D",
    "AeroPropeller3D",
    "AeroCyclicPropeller3D",
    "AeroBuoyancySphere3D",
    "AeroBuoyancyMesh3D",
];

/// Marks the entity of a spawned aircraft. Maps Godot node paths (relative to the root) to entities,
/// so visual links can find their source and target nodes.
#[derive(Component, Debug, Clone, Default)]
pub struct AircraftPaths(pub HashMap<String, Entity>);

/// Visual link: `target.rotation[axis] = sign * source.rotation[axis] + offset`, with Godot's
/// Euler conventions (each node's `rotation_order`).
#[derive(Component, Debug, Clone)]
pub struct RotationFollow {
    pub source: Entity,
    pub source_order: EulerOrder,
    pub source_axis: usize,
    pub target_order: EulerOrder,
    pub axis: usize,
    pub sign: f32,
    pub offset: f32,
}

/// Asset access for [`spawn_scene`]. Mesh files in a scene are relative to the asset root.
pub struct SpawnAssets<'a> {
    pub server: &'a AssetServer,
    pub meshes: &'a mut Assets<Mesh>,
    pub materials: &'a mut Assets<StandardMaterial>,
}

/// Called for every node after its entity is created. Lets the application attach its own
/// components (cameras, HUD, and so on). Arguments: the node, its entity, and the root entity.
pub type NodeHook<'a> = &'a mut dyn FnMut(&NodeDef, Entity, Entity, &mut Commands);

/// A visual link as written in the scene, with root-relative node paths.
struct FollowLink {
    target: String,
    source: String,
    axis: usize,
    source_axis: usize,
    sign: f32,
    offset: f32,
}

/// Spawns `scene` as a physics body. `placement` is applied on top of the scene root's own transform,
/// as Godot does when a vehicle is instanced under a spawn marker. Returns the root entity.
pub fn spawn_scene(commands: &mut Commands, assets: &mut SpawnAssets, scene: &SceneFile, placement: Transform, hook: NodeHook) -> Entity {
    let root_def = &scene.root;
    let transform = placement.mul_transform(local_transform(root_def));
    let mass = root_def.prop_f32("mass", 1.0);
    let center_of_mass = root_def.props.get("center_of_mass").map_or(Vec3::ZERO, vec3_of);
    let can_sleep = root_def.prop_bool("can_sleep", true);

    let root = commands
        .spawn((
            Name::new(scene.name.clone()),
            transform,
            Visibility::default(),
            RigidBody::Dynamic,
            Mass(mass),
            CenterOfMass(center_of_mass),
            AircraftPaths::default(),
        ))
        .id();
    if !can_sleep {
        commands.entity(root).insert(SleepingDisabled);
    }
    hook(root_def, root, root, commands);

    let mut spawner = Spawner {
        assets,
        hook,
        root,
        nodes: Vec::new(),
        roots: Vec::new(),
        paths: HashMap::new(),
        orders: HashMap::new(),
        follows: Vec::new(),
        controls: None,
        flight_assist: None,
        substeps_override: root_def.props.get("substeps_override").and_then(Value::as_i64).filter(|v| *v >= 0).map(|v| v as u32),
        show_debug: root_def.prop_bool("show_debug", false),
        wheels: Vec::new(),
        volumes: Vec::new(),
    };
    spawner.paths.insert(String::new(), root);
    spawner.spawn_children(commands, root, None, &root_def.children, "", false, false);

    let mut body = AeroBody::new(
        std::mem::take(&mut spawner.nodes),
        std::mem::take(&mut spawner.roots),
        spawner.controls.take().unwrap_or_default(),
        spawner.flight_assist.take(),
    );
    body.substeps_override = spawner.substeps_override;
    body.experimental_energy_tracking = root_def.prop_bool("experimental_energy_tracking", false);
    body.show_debug = spawner.show_debug;
    body.wheels = std::mem::take(&mut spawner.wheels);

    // `Mass` is the total mass, so the colliders must not add their own: `NoAutoMass` on the body
    // stops descendants contributing mass. Their inertia still counts, and comes from their density.
    // One density for every body collider, chosen so the colliders weigh `mass` in total, gives the
    // inertia Godot derives from the same shapes and mass.
    let total_volume: f32 = spawner.volumes.iter().map(|(_, v)| v).sum();
    if total_volume > 0.0 {
        let density = mass / total_volume;
        for (entity, _) in &spawner.volumes {
            commands.entity(*entity).insert(ColliderDensity(density));
        }
    }
    body.center_of_mass = center_of_mass;

    let paths = std::mem::take(&mut spawner.paths);
    for link in &spawner.follows {
        let (Some(&target), Some(&source)) = (paths.get(&link.target), paths.get(&link.source)) else {
            continue;
        };
        let target_order = spawner.orders.get(&link.target).copied().unwrap_or_default();
        let source_order = spawner.orders.get(&link.source).copied().unwrap_or_default();
        commands.entity(target).insert(RotationFollow {
            source,
            source_order,
            target_order,
            axis: link.axis,
            source_axis: link.source_axis,
            sign: link.sign,
            offset: link.offset,
        });
    }
    commands.entity(root).insert((body, AircraftPaths(paths), NoAutoMass));
    root
}

struct Spawner<'a, 'b> {
    assets: &'a mut SpawnAssets<'b>,
    hook: NodeHook<'a>,
    root: Entity,
    nodes: Vec<Node>,
    roots: Vec<usize>,
    paths: HashMap<String, Entity>,
    orders: HashMap<String, EulerOrder>,
    follows: Vec<FollowLink>,
    controls: Option<ControlConfig>,
    flight_assist: Option<FlightAssist>,
    substeps_override: Option<u32>,
    show_debug: bool,
    wheels: Vec<Wheel>,
    /// Body colliders and their volumes, used to set the density so that the inertia matches the mass.
    volumes: Vec<(Entity, f32)>,
}

fn vec3_of(v: &Value) -> Vec3 {
    let a: Vec<f32> = v.as_array().map(|arr| arr.iter().filter_map(Value::as_f64).map(|x| x as f32).collect()).unwrap_or_default();
    Vec3::new(*a.first().unwrap_or(&0.0), *a.get(1).unwrap_or(&0.0), *a.get(2).unwrap_or(&0.0))
}

/// Mirror transform: flips the root axis of the origin and the basis, as `AeroInfluencer3D.generate_mirror`.
fn mirrored(basis: Mat3, origin: Vec3, axis: u8) -> (Mat3, Vec3) {
    let mut flip = Vec3::ONE;
    flip[axis as usize - 1] = -1.0;
    (Mat3::from_diagonal(flip) * basis, origin * flip)
}

/// Rows of the basis are scaled by the chosen axis, as `_mirror_scaling_fix_axis` does for duplicates.
fn fix_scaling(basis: Mat3, axis: u8) -> Mat3 {
    let mut flip = Vec3::ONE;
    flip[axis as usize - 1] = -1.0;
    Mat3::from_diagonal(flip) * basis
}

impl<'a, 'b> Spawner<'a, 'b> {
    /// Spawns `children` under `parent`. `path` is the root-relative path of `parent`.
    #[allow(clippy::too_many_arguments)]
    fn spawn_children(
        &mut self,
        commands: &mut Commands,
        parent: Entity,
        parent_index: Option<usize>,
        children: &[NodeDef],
        path: &str,
        is_duplicate: bool,
        in_atmosphere: bool,
    ) {
        for child in children {
            let child_path = join(path, &child.name);
            match child.class.as_str() {
                "AeroControlComponent" => {
                    self.controls = Some(read_controls(child));
                    self.flight_assist = read_flight_assist(child);
                }
                class if INFLUENCER_CLASSES.contains(&class) => {
                    self.spawn_influencer(commands, parent, parent_index, child, is_duplicate, path, None, true);
                }
                "AeroAtmosphere3D" | "AeroWaterAtmosphere3D" => {
                    self.spawn_atmosphere(commands, parent, child, path);
                }
                "CollisionShape3D" => {
                    self.spawn_collider(commands, parent, child, in_atmosphere, &child_path);
                }
                "VehicleWheel3D" if parent_index.is_none() => {
                    let mount = child.local_transform().map_or(Vec3::ZERO, |(_, o)| o);
                    let json = serde_json::to_value(&child.props).unwrap_or(Value::Null);
                    let mut wheel: Wheel = serde_json::from_value(json).unwrap_or_default();
                    wheel.mount = mount;
                    self.wheels.push(wheel);
                    let entity = self.spawn_plain(commands, parent, child, &child_path);
                    self.spawn_children(commands, entity, parent_index, &child.children, &child_path, is_duplicate, in_atmosphere);
                }
                _ => {
                    let entity = self.spawn_plain(commands, parent, child, &child_path);
                    self.spawn_children(commands, entity, parent_index, &child.children, &child_path, is_duplicate, in_atmosphere);
                }
            }
        }
    }

    fn run_hook(&mut self, def: &NodeDef, entity: Entity, commands: &mut Commands) {
        (self.hook)(def, entity, self.root, commands);
    }

    /// Spawns a node that the physics model does not need (a visual or a plain transform).
    fn spawn_plain(&mut self, commands: &mut Commands, parent: Entity, def: &NodeDef, path: &str) -> Entity {
        let entity = commands
            .spawn((Name::new(def.name.clone()), local_transform(def), visibility(def.visible)))
            .id();
        commands.entity(parent).add_child(entity);
        self.register(path, entity, def);
        self.add_visuals(commands, entity, def);
        self.run_hook(def, entity, commands);
        entity
    }

    fn register(&mut self, path: &str, entity: Entity, def: &NodeDef) {
        self.paths.insert(path.to_string(), entity);
        self.orders.insert(path.to_string(), crate::schema::rotation_order_of(def));
        for f in &def.follow {
            self.follows.push(FollowLink {
                target: f.target.clone(),
                source: f.source.clone(),
                axis: axis_index(&f.axis),
                source_axis: axis_index(&f.source_axis),
                sign: f.sign,
                offset: f.offset,
            });
        }
    }

    /// Draws the node's own geometry: glTF primitives, or a CSG primitive.
    fn add_visuals(&mut self, commands: &mut Commands, entity: Entity, def: &NodeDef) {
        if let Some(mesh_ref) = &def.mesh {
            let file = mesh_ref.file.clone();
            for primitive in &mesh_ref.primitives {
                let mesh = self.assets.server.load(GltfAssetLabel::Primitive { mesh: primitive.mesh, primitive: primitive.primitive }.from_asset(file.clone()));
                let material = match primitive.material {
                    Some(index) => self.assets.server.load(GltfAssetLabel::Material { index, is_scale_inverted: false }.from_asset(file.clone())),
                    None => self.assets.materials.add(StandardMaterial::default()),
                };
                let child = commands.spawn((Mesh3d(mesh), MeshMaterial3d(material), Transform::IDENTITY)).id();
                commands.entity(entity).add_child(child);
            }
        }

        let mesh = match def.class.as_str() {
            "CSGBox3D" => Some(self.assets.meshes.add(Cuboid::from_size(vec3_prop(def, "size", Vec3::ONE)))),
            "CSGCylinder3D" => Some(self.assets.meshes.add(Cylinder::new(def.prop_f32("radius", 0.5), def.prop_f32("height", 2.0)))),
            "CSGSphere3D" => Some(self.assets.meshes.add(Sphere::new(def.prop_f32("radius", 0.5)))),
            _ => None,
        };
        if let Some(mesh) = mesh {
            let color = def
                .props
                .get("albedo_color")
                .map(vec4_of)
                .map_or(Color::srgb(0.6, 0.6, 0.6), |c| Color::srgba(c.x, c.y, c.z, c.w));
            let material = self.assets.materials.add(StandardMaterial { base_color: color, ..Default::default() });
            let child = commands.spawn((Mesh3d(mesh), MeshMaterial3d(material), Transform::IDENTITY)).id();
            commands.entity(entity).add_child(child);
        }
    }

    /// Spawns an influencer and its subtree. `allow_mirror` is false for propeller blade copies.
    #[allow(clippy::too_many_arguments)]
    fn spawn_influencer(
        &mut self,
        commands: &mut Commands,
        parent: Entity,
        parent_index: Option<usize>,
        def: &NodeDef,
        is_duplicate: bool,
        path: &str,
        override_transform: Option<(Mat3, Vec3)>,
        allow_mirror: bool,
    ) -> usize {
        let (mut basis, origin) = override_transform.or_else(|| def.local_transform()).unwrap_or((Mat3::IDENTITY, Vec3::ZERO));
        let mirror_axis = def.prop_f32("mirror_axis", 0.0) as u8;
        let fix_axis = def.prop_f32("mirror_scaling_fix_axis", 0.0) as u8;
        if is_duplicate && fix_axis != 0 {
            basis = fix_scaling(basis, fix_axis);
        }

        let child_path = join(path, &def.name);
        let entity = commands
            .spawn((Name::new(def.name.clone()), transform_of(basis, origin), visibility(def.visible)))
            .id();
        commands.entity(parent).add_child(entity);
        self.register(&child_path, entity, def);
        self.run_hook(def, entity, commands);

        // Reserve the index first: the kind's blades and children refer to it.
        let index = self.nodes.len();
        let kind = self.build_kind(def);
        self.nodes.push(Node {
            entity,
            name: def.name.clone(),
            children: Vec::new(),
            disabled: def.prop_bool("disabled", false),
            omit_from_debug: def.prop_bool("omit_from_debug", false),
            is_duplicate,
            mirror_axis,
            mirror_scaling_fix_axis: fix_axis,
            rotation_order: crate::schema::rotation_order_of(def),
            control_rotation_order: EulerOrder::from(def.prop_f32("control_rotation_order", 0.0) as i32),
            actuation: read_actuation(def, "actuation_config"),
            basis,
            origin,
            default_basis: basis,
            default_origin: origin,
            state: NodeState::default(),
            kind,
        });
        match parent_index {
            Some(p) => self.nodes[p].children.push(index),
            None => self.roots.push(index),
        }

        self.add_visuals(commands, entity, def);
        self.spawn_children(commands, entity, Some(index), &def.children, &child_path, is_duplicate, false);

        if matches!(self.nodes[index].kind, Kind::Propeller(_) | Kind::CyclicPropeller(_)) {
            self.spawn_propeller_blades(commands, index, def, is_duplicate, &child_path);
        }

        // A mirrored copy is a sibling with every descendant marked as a duplicate.
        if mirror_axis != 0 && !is_duplicate && allow_mirror {
            let (mirror_basis, mirror_origin) = mirrored(basis, origin, mirror_axis);
            let mirror_def = NodeDef { name: format!("{}Mirror", def.name), ..def.clone() };
            self.spawn_influencer(commands, parent, parent_index, &mirror_def, true, path, Some((mirror_basis, mirror_origin)), false);
        }

        index
    }

    /// Creates the extra blades of a propeller: `propeller_amount - 1` rotated copies of the template.
    fn spawn_propeller_blades(&mut self, commands: &mut Commands, index: usize, def: &NodeDef, is_duplicate: bool, child_path: &str) {
        let blade_name = def.prop_str("propeller_blade").unwrap_or_default().split('/').next().unwrap_or_default().to_string();
        let Some(template) = self.nodes[index].children.iter().copied().find(|&c| self.nodes[c].name == blade_name) else {
            return;
        };
        let Some(blade_def) = def.children.iter().find(|c| c.name == blade_name).cloned() else {
            return;
        };
        let amount = match &self.nodes[index].kind {
            Kind::Propeller(p) => p.amount,
            Kind::CyclicPropeller(c) => c.propeller.amount,
            _ => 1,
        };
        let propeller_entity = self.nodes[index].entity;
        let base_basis = self.nodes[template].default_basis;
        let base_origin = self.nodes[template].default_origin;

        let mut instances = vec![template];
        for i in 1..amount {
            let rotation = Mat3::from_axis_angle(Vec3::Y, (360.0_f32 / amount as f32 * i as f32).to_radians());
            let copy = self.spawn_influencer(
                commands,
                propeller_entity,
                Some(index),
                &blade_def,
                is_duplicate,
                child_path,
                Some((rotation * base_basis, rotation * base_origin)),
                false,
            );
            instances.push(copy);
        }

        match &mut self.nodes[index].kind {
            Kind::Propeller(p) => {
                p.blade = Some(template);
                p.instances = instances;
            }
            Kind::CyclicPropeller(c) => {
                c.propeller.blade = Some(template);
                c.propeller.instances = instances;
            }
            _ => {}
        }
    }

    /// Builds the class-specific data of an influencer from its properties.
    fn build_kind(&mut self, def: &NodeDef) -> Kind {
        match def.class.as_str() {
            "AeroSurface3D" => Kind::Surface(SurfaceState { wing: wing_config(def), ..Default::default() }),
            "ManualAeroSurface3D" => {
                let manual: ManualSurfaceConfig = def.prop("manual_config");
                Kind::ManualSurface(Box::new(ManualSurface::new(wing_config(def), &manual)))
            }
            "AeroDragInfluencer3D" => Kind::Drag {
                radius: def.prop_f32("radius", 0.5),
                drag_coefficient: def.prop_f32("drag_coefficient", 0.5),
            },
            "AeroThruster3D" | "AeroJetThruster3D" => {
                let jet = (def.class == "AeroJetThruster3D").then(|| JetParams {
                    intake_area: def.prop_f32("intake_area", 0.5),
                    intake_fan_max_velocity: def.prop_f32("intake_fan_max_velocity", 300.0),
                    exhaust_velocity: def.prop_f32("exhaust_velocity", 1000.0),
                    max_fuel_flow: def.prop_f32("max_fuel_flow", 0.1),
                });
                Kind::Thruster(Box::new(ThrusterState {
                    max_thrust_force: def.prop_f32("max_thrust_force", 10000.0),
                    throttle: vec3_prop(def, "throttle", Vec3::ZERO),
                    throttle_config: Some(read_actuation(def, "throttle_control_config").unwrap_or_else(default_throttle_config)),
                    jet,
                }))
            }
            "AeroMover3D" => Kind::Mover(read_mover(def)),
            "AeroPropeller3D" => Kind::Propeller(Box::new(read_propeller(def))),
            "AeroCyclicPropeller3D" => Kind::CyclicPropeller(Box::new(CyclicState {
                propeller: read_propeller(def),
                cyclic: glam_vec2(def, "cyclic"),
                cyclic_pitch: def.prop_f32("cyclic_pitch", 15.0),
                cyclic_config: Some(read_actuation(def, "cyclic_control_config").unwrap_or_else(default_cyclic_config)),
            })),
            "AeroBuoyancySphere3D" => Kind::BuoyancySphere(SphereBuoyancy {
                radius: def.prop_f32("radius", 1.0),
                buoyancy_force: Vec3::ZERO,
                cached_force: Vec3::ZERO,
                cached_torque: Vec3::ZERO,
            }),
            "AeroBuoyancyMesh3D" => {
                let (vertices, indices) = def.mesh.as_ref().map(|m| (m.buoyancy_vertices.clone(), m.buoyancy_indices.clone())).unwrap_or_default();
                let vertices: Vec<Vec3> = vertices.iter().map(|v| Vec3::from_array(*v)).collect();
                let indices = if indices.is_empty() { (0..vertices.len() as u32).collect() } else { indices };
                let defaults = crate::schema::MeshBuoyancySettings::default();
                Kind::BuoyancyMesh(Box::new(MeshBuoyancy::from_mesh(
                    &vertices,
                    &indices,
                    def.prop_f32("skin_friction", defaults.skin_friction),
                    def.prop_f32("aero_force_multiplier", defaults.aero_force_multiplier),
                )))
            }
            _ => Kind::Base,
        }
    }

    fn spawn_atmosphere(&mut self, commands: &mut Commands, parent: Entity, def: &NodeDef, path: &str) {
        let child_path = join(path, &def.name);
        let mut settings = if def.class == "AeroWaterAtmosphere3D" { AeroAtmosphere::water() } else { AeroAtmosphere::default() };
        apply_atmosphere_props(&mut settings, def);
        let entity = commands.spawn((Name::new(def.name.clone()), local_transform(def), visibility(def.visible), settings)).id();
        commands.entity(parent).add_child(entity);
        self.register(&child_path, entity, def);
        self.run_hook(def, entity, commands);
        self.spawn_children(commands, entity, None, &def.children, &child_path, false, true);
    }

    /// Adds a collider. Inside an atmosphere it becomes a sensor, which reports overlaps instead of colliding.
    fn spawn_collider(&mut self, commands: &mut Commands, parent: Entity, def: &NodeDef, in_atmosphere: bool, path: &str) {
        let Some(shape) = def.props.get("shape").and_then(|v| serde_json::from_value::<ShapeDef>(v.clone()).ok()) else {
            return;
        };
        let collider = match shape {
            ShapeDef::BoxShape3D { size } => Collider::cuboid(size.x, size.y, size.z),
            ShapeDef::CapsuleShape3D { radius, height } => Collider::capsule(radius, (height - 2.0 * radius).max(0.0)),
            ShapeDef::SphereShape3D { radius } => Collider::sphere(radius),
            ShapeDef::ConvexPolygonShape3D { points } => {
                let points: Vec<Vec3> = points.iter().map(|p| Vec3::from_array(*p)).collect();
                match Collider::convex_hull(points.as_slice()) {
                    Some(collider) => collider,
                    None => return,
                }
            }
        };
        let volume = collider.mass_properties(1.0).mass;
        let entity = if in_atmosphere {
            commands.spawn((Name::new(def.name.clone()), local_transform(def), collider, Sensor, CollidingEntities::default())).id()
        } else {
            self.volumes.push((Entity::PLACEHOLDER, volume));
            let index = self.volumes.len() - 1;
            let entity = commands.spawn((Name::new(def.name.clone()), local_transform(def), collider)).id();
            self.volumes[index].0 = entity;
            entity
        };
        commands.entity(parent).add_child(entity);
        self.register(path, entity, def);
        self.run_hook(def, entity, commands);
    }
}

fn local_transform(def: &NodeDef) -> Transform {
    def.local_transform().map_or(Transform::IDENTITY, |(b, o)| transform_of(b, o))
}

fn read_propeller(def: &NodeDef) -> PropellerState {
    PropellerState {
        mover: read_mover(def),
        amount: def.prop_f32("propeller_amount", 2.0).max(1.0) as u32,
        blade: None,
        instances: Vec::new(),
        speed_config: Some(read_actuation(def, "propeller_speed_control_config").unwrap_or_else(default_speed_config)),
    }
}

fn axis_index(name: &str) -> usize {
    match name {
        "y" => 1,
        "z" => 2,
        _ => 0,
    }
}

fn join(base: &str, name: &str) -> String {
    if base.is_empty() { name.to_string() } else { format!("{base}/{name}") }
}

fn visibility(visible: bool) -> Visibility {
    if visible { Visibility::Inherited } else { Visibility::Hidden }
}

fn transform_of(basis: Mat3, origin: Vec3) -> Transform {
    let m = bevy::math::Mat4::from_cols(basis.x_axis.extend(0.0), basis.y_axis.extend(0.0), basis.z_axis.extend(0.0), origin.extend(1.0));
    Transform::from_matrix(m)
}

fn vec3_prop(def: &NodeDef, key: &str, default: Vec3) -> Vec3 {
    def.props.get(key).map_or(default, vec3_of)
}

fn glam_vec2(def: &NodeDef, key: &str) -> bevy::math::Vec2 {
    let v = vec3_prop(def, key, Vec3::ZERO);
    bevy::math::Vec2::new(v.x, v.y)
}

fn vec4_of(v: &Value) -> bevy::math::Vec4 {
    let a: Vec<f32> = v.as_array().map(|arr| arr.iter().filter_map(Value::as_f64).map(|x| x as f32).collect()).unwrap_or_default();
    bevy::math::Vec4::new(*a.first().unwrap_or(&1.0), *a.get(1).unwrap_or(&1.0), *a.get(2).unwrap_or(&1.0), *a.get(3).unwrap_or(&1.0))
}

fn wing_config(def: &NodeDef) -> SurfaceConfig {
    def.prop("wing_config")
}

fn read_actuation(def: &NodeDef, key: &str) -> Option<ActuationConfig> {
    let mut config: ActuationConfig = def.prop_opt(key)?;
    config.compile().ok()?;
    Some(config)
}

fn read_mover(def: &NodeDef) -> MoverState {
    MoverState {
        linear_motor: vec3_prop(def, "linear_motor", Vec3::ZERO),
        angular_motor: vec3_prop(def, "angular_motor", Vec3::ZERO),
        disable_lift_dissymmetry: def.prop_bool("disable_lift_dissymmetry", false),
        use_movement_as_velocity: def.prop_bool("use_movement_as_velocity", false),
        ..Default::default()
    }
}

fn read_controls(def: &NodeDef) -> ControlConfig {
    let mut config: ControlConfig = def.prop_opt("control_config").unwrap_or_default();
    if config.axis_configs.is_empty() {
        config = ControlConfig::default();
    }
    config
}

fn read_flight_assist(def: &NodeDef) -> Option<FlightAssist> {
    let value = def.props.get("flight_assist")?;
    serde_json::from_value::<FlightAssist>(value.clone()).ok()
}

fn apply_atmosphere_props(settings: &mut AeroAtmosphere, def: &NodeDef) {
    settings.per_influencer_positioning = def.prop_bool("per_influencer_positioning", settings.per_influencer_positioning);
    settings.wind = vec3_prop(def, "wind", settings.wind);
    settings.override_wind = def.prop_bool("override_wind", settings.override_wind);
    settings.density = def.prop_f32("density", settings.density);
    settings.override_density = def.prop_bool("override_density", settings.override_density);
    settings.temperature = def.prop_f32("temperature", settings.temperature);
    settings.override_temperature = def.prop_bool("override_temperature", settings.override_temperature);
}

/// `AeroThruster3D.create_throttle_control_config()`.
fn default_throttle_config() -> ActuationConfig {
    let mut config = ActuationConfig { use_separate_minmax: true, ..Default::default() };
    config.min_value = Vec3::new(0.0, 0.0, -1.0);
    config.max_value = Vec3::new(0.0, 0.0, 0.0);
    config.axis_configs.push(crate::controls::ActuationAxis { axis_name: "throttle".into(), contribution: -Vec3::ONE, ..Default::default() });
    config
}

/// `AeroPropeller3D.create_speed_control_config()`.
fn default_speed_config() -> ActuationConfig {
    let mut config = ActuationConfig { use_separate_minmax: true, ..Default::default() };
    config.max_value = Vec3::new(0.0, 200.0, 0.0);
    config.min_value = Vec3::ZERO;
    config.axis_configs.push(crate::controls::ActuationAxis { axis_name: "throttle".into(), contribution: Vec3::ONE, ..Default::default() });
    config
}

/// `AeroCyclicPropeller3D.create_cyclic_control_config()`.
fn default_cyclic_config() -> ActuationConfig {
    let mut config = ActuationConfig { max_value: Vec3::new(1.0, 1.0, 0.0), ..Default::default() };
    config.axis_configs.push(crate::controls::ActuationAxis { axis_name: "roll".into(), contribution: Vec3::X, ..Default::default() });
    config.axis_configs.push(crate::controls::ActuationAxis { axis_name: "pitch".into(), contribution: Vec3::Y, ..Default::default() });
    config
}

