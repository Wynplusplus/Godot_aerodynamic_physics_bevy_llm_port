//! Headless integration tests: converted scenes spawn, fly, and float under the real Avian step.

use std::time::Duration;

use avian3d::prelude::*;
use bevy::asset::AssetPlugin;
use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use bevy::transform::TransformPlugin;

use aero_physics::atmosphere::AeroAtmosphere;
use aero_physics::schema::SceneFile;
use aero_physics::{AeroBody, AeroPhysicsPlugin, SpawnAssets, spawn_scene};

fn scene(name: &str) -> SceneFile {
    let path = format!("{}/../aero_demo/assets/aircraft/{name}.json", env!("CARGO_MANIFEST_DIR"));
    serde_json::from_str(&std::fs::read_to_string(path).expect("scene json")).expect("scene parses")
}

fn app() -> App {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, AssetPlugin::default(), TransformPlugin, PhysicsPlugins::default(), AeroPhysicsPlugin));
    app.init_asset::<Mesh>().init_asset::<StandardMaterial>();
    app.insert_resource(Time::<Fixed>::from_hz(60.0));
    app.insert_resource(Gravity(Vec3::NEG_Y * 9.8));
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(1.0 / 60.0)));
    // `App::run` does this before the first update; plugins such as Avian register resources here.
    app.finish();
    app.cleanup();
    app
}

fn water_volume(app: &mut App) {
    app.world_mut().spawn((Transform::from_xyz(0.0, -1.0, 0.0), AeroAtmosphere::water())).with_children(|p| {
        p.spawn((Transform::from_xyz(0.0, -750.0, 0.0), Collider::cuboid(10_000.0, 1501.0, 10_000.0), Sensor, CollidingEntities::default()));
    });
}

#[derive(Resource, Clone)]
struct Spawn {
    scene: SceneFile,
    at: Transform,
}

fn spawn_system(
    mut commands: Commands,
    spawn: Res<Spawn>,
    server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut assets = SpawnAssets { server: &server, meshes: &mut meshes, materials: &mut materials };
    spawn_scene(&mut commands, &mut assets, &spawn.scene, spawn.at, &mut |_, _, _, _| {});
}

/// Reads the body's aerodynamic state and its position, after one update.
fn body_state(app: &mut App) -> (usize, Vec3, Vec3, Vec3, f32) {
    let world = app.world_mut();
    let (atmospheres, force, air_speed) = {
        let body = world.query::<&AeroBody>().single(world).unwrap();
        (body.atmospheres.len(), body.current_force, body.air_speed)
    };
    let position = world.query_filtered::<&Position, With<AeroBody>>().single(world).unwrap().0;
    let velocity = world.query_filtered::<&LinearVelocity, With<AeroBody>>().single(world).unwrap().0;
    (atmospheres, position, velocity, force, air_speed)
}

#[test]
fn demo_plane_flies_stably() {
    let mut app = app();
    app.insert_resource(Spawn { scene: scene("demo_plane"), at: Transform::from_xyz(0.0, 200.0, 0.0) });
    app.add_systems(Startup, spawn_system);
    app.update();

    // Give the plane forward airspeed, as if it were already flying.
    let root = app.world_mut().query_filtered::<Entity, With<AeroBody>>().single(app.world()).unwrap();
    app.world_mut().entity_mut(root).insert(LinearVelocity(Vec3::new(0.0, 0.0, -60.0)));
    for _ in 0..300 {
        app.update();
    }

    let (_, _, velocity, force, air_speed) = body_state(&mut app);
    assert!(air_speed.is_finite() && air_speed > 1.0, "air speed {air_speed}");
    assert!(force.is_finite(), "force {force:?}");
    assert!(velocity.is_finite() && velocity.length() < 500.0, "velocity {velocity:?}");
}

#[test]
fn floaty_ball_floats_up_out_of_the_water() {
    let mut app = app();
    water_volume(&mut app);
    app.insert_resource(Spawn { scene: scene("floaty_ball"), at: Transform::from_xyz(0.0, -3.0, 0.0) });
    app.add_systems(Startup, spawn_system);
    app.update();

    let mut saw_water = false;
    let mut highest = f32::MIN;
    for _ in 0..120 {
        app.update();
        let (atmospheres, position, velocity, force, _) = body_state(&mut app);
        assert!(position.is_finite() && velocity.is_finite() && force.is_finite(), "diverged: {position:?} {velocity:?} {force:?}");
        saw_water |= atmospheres > 0;
        highest = highest.max(position.y);
    }
    assert!(saw_water, "the ball never overlapped the water volume");
    assert!(highest > -1.0, "buoyancy did not lift the ball (highest y {highest})");
}

#[test]
fn cessna_settles_on_its_wheels() {
    let mut app = app();
    // A flat ground plane with its top at y = 0.
    app.world_mut().spawn((Transform::from_xyz(0.0, -0.5, 0.0), RigidBody::Static, Collider::cuboid(400.0, 1.0, 400.0)));
    app.insert_resource(Spawn { scene: scene("cessna172"), at: Transform::IDENTITY });
    app.add_systems(Startup, spawn_system);
    app.update();

    for _ in 0..240 {
        app.update();
    }
    let (_, position, velocity, force, _) = body_state(&mut app);
    assert!(position.is_finite() && velocity.is_finite() && force.is_finite(), "diverged: {position:?} {velocity:?}");
    assert!(position.y > -0.5, "the aircraft fell through the ground: {position:?}");
    assert!(velocity.length() < 2.0, "the aircraft is still moving: {velocity:?}");
}

#[test]
fn energy_tracking_keeps_the_plane_finite() {
    let mut app = app();
    app.insert_resource(Spawn { scene: scene("demo_plane"), at: Transform::from_xyz(0.0, 200.0, 0.0) });
    app.add_systems(Startup, spawn_system);
    app.update();
    let root = app.world_mut().query_filtered::<Entity, With<AeroBody>>().single(app.world()).unwrap();
    app.world_mut().entity_mut(root).insert(LinearVelocity(Vec3::new(0.0, 0.0, -60.0)));
    app.world_mut().query::<&mut AeroBody>().single_mut(app.world_mut()).unwrap().experimental_energy_tracking = true;
    for _ in 0..240 {
        app.update();
    }
    let (_, position, velocity, force, _) = body_state(&mut app);
    assert!(position.is_finite() && velocity.is_finite() && force.is_finite(), "diverged: {position:?} {velocity:?}");
}
