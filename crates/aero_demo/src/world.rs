//! The environment of `demo_scene.tscn`: ground, water, a wind layer, lights, and spawn markers.

use avian3d::prelude::*;
use bevy::math::Mat3;
use bevy::prelude::*;

use aero_physics::AeroAtmosphere;

/// Where land vehicles appear (`VehicleSpawner`).
pub fn land_spawn() -> Transform {
    Transform::from_xyz(0.0, -0.5, 0.0)
}

/// Where water vehicles appear (`VehicleSpawner/WaterSpawn`).
pub fn water_spawn() -> Transform {
    // The Godot basis columns, as written in demo_scene.tscn.
    let basis = Mat3::from_cols(Vec3::new(-0.258_819, 0.0, 0.965_926), Vec3::Y, Vec3::new(-0.965_926, 0.0, -0.258_819));
    Transform::from_matrix(bevy::math::Mat4::from_cols(basis.x_axis.extend(0.0), basis.y_axis.extend(0.0), basis.z_axis.extend(0.0), Vec3::new(0.0, -0.5, 991.977).extend(1.0)))
}

/// The camera that is shown while the menu is open.
#[derive(Component)]
pub struct MenuCamera;

pub struct WorldPlugin;

impl Plugin for WorldPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(ClearColor(Color::srgb(0.646, 0.656, 0.671)))
            .insert_resource(GlobalAmbientLight { color: Color::WHITE, brightness: 400.0, ..default() })
            .add_systems(Startup, setup_world);
    }
}

fn setup_world(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>) {
    // Sun, with the rotation of the demo's DirectionalLight3D.
    let sun_basis = Mat3::from_cols(Vec3::new(-0.866_023, -0.433_016, 0.250_001), Vec3::new(0.0, 0.499_998, 0.866_027), Vec3::new(-0.500_003, 0.749_999, -0.433_01));
    commands.spawn((
        DirectionalLight { shadow_maps_enabled: true, illuminance: 12_000.0, ..default() },
        Transform::from_matrix(bevy::math::Mat4::from_cols(sun_basis.x_axis.extend(0.0), sun_basis.y_axis.extend(0.0), sun_basis.z_axis.extend(0.0), Vec4::W)),
    ));

    // Ground: a 10 km square with a collider, as the CSGBox3D with use_collision.
    let ground_size = Vec3::new(10_000.0, 1.0, 10_000.0);
    commands.spawn((
        Name::new("Ground"),
        Transform::from_xyz(0.0, -1.0, -4151.12),
        RigidBody::Static,
        Collider::cuboid(ground_size.x, ground_size.y, ground_size.z),
        Mesh3d(meshes.add(Cuboid::from_size(ground_size))),
        MeshMaterial3d(materials.add(StandardMaterial { base_color: Color::srgb(0.0, 0.490, 0.082), ..default() })),
    ));

    // Water: a per-influencer volume whose surface is at its origin height.
    let water = commands
        .spawn((Name::new("AeroWaterAtmosphere3D"), Transform::from_xyz(0.0, -1.0, 5580.07), AeroAtmosphere::water()))
        .id();
    let water_volume = Vec3::new(10_000.0, 1501.0, 10_000.0);
    let sensor = commands
        .spawn((Name::new("WaterVolume"), Transform::from_xyz(0.0, -750.0, 0.0), Collider::cuboid(water_volume.x, water_volume.y, water_volume.z), Sensor, CollidingEntities::default()))
        .id();
    let surface_mesh = commands
        .spawn((
            Name::new("WaterSurface"),
            Transform::from_xyz(0.0, -758.984, 0.0),
            Mesh3d(meshes.add(Cuboid::new(10_000.0, 1517.968, 10_000.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.0, 0.389, 0.551),
                metallic: 0.25,
                perceptual_roughness: 0.14,
                ..default()
            })),
        ))
        .id();
    commands.entity(water).add_children(&[sensor, surface_mesh]);

    // Wind layer: a body-level volume that pushes air to the -X direction at 10 m/s.
    let air = commands
        .spawn((
            Name::new("AeroAtmosphere3D"),
            Transform::from_xyz(0.0, 0.0, 5633.997),
            AeroAtmosphere { wind: Vec3::new(-10.0, 0.0, 0.0), ..default() },
        ))
        .id();
    let air_sensor = commands
        .spawn((Name::new("AirVolume"), Collider::cuboid(10_000.0, 594.688, 10_000.0), Sensor, CollidingEntities::default()))
        .id();
    commands.entity(air).add_child(air_sensor);

    // Menu camera, and a 2D camera that draws the UI on top of everything.
    commands.spawn((MenuCamera, Camera3d::default(), Transform::from_xyz(0.0, 2.0, 14.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y)));
    commands.spawn((Camera2d, Camera { order: 10, clear_color: ClearColorConfig::None, ..default() }));
}
