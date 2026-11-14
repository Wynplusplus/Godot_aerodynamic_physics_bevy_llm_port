//! `BallLauncher` (ball_launcher.gd): fires a drag ball, which is removed after 30 seconds.

use avian3d::prelude::*;
use bevy::prelude::*;

use aero_physics::{AeroBody, SpawnAssets, spawn_scene};

use crate::hooks::BallLauncher;
use crate::input::just_pressed;
use crate::menu::load_scene;

/// Muzzle speed of the ball, in m/s, relative to the launching body.
const LAUNCH_SPEED: f32 = 400.0;
const LIFETIME_SECONDS: f32 = 30.0;

/// A launched ball. Removed after its lifetime, or when returning to the menu.
#[derive(Component)]
pub struct Ball;

/// Removes the entity once the timer finishes.
#[derive(Component)]
struct Lifetime(Timer);

pub struct BallPlugin;

impl Plugin for BallPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, (launch, expire));
    }
}

fn launch(
    keys: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    launchers: Query<(&BallLauncher, &GlobalTransform)>,
    bodies: Query<(&AeroBody, &LinearVelocity, &AngularVelocity)>,
) {
    if !just_pressed(&keys, "ui_accept") {
        return;
    }
    let Some(scene) = load_scene("drag_ball") else {
        return;
    };
    for (launcher, marker) in &launchers {
        let Ok((_, body_velocity, body_spin)) = bodies.get(launcher.body) else {
            continue;
        };
        let placement = marker.compute_transform();
        // Godot: linear_velocity = body velocity + (-global_basis.z) * launch_velocity. Bevy's forward() is -Z.
        let velocity = body_velocity.0 + placement.forward().as_vec3() * LAUNCH_SPEED;

        let mut assets = SpawnAssets { server: &server, meshes: &mut meshes, materials: &mut materials };
        // Balls get no hooks: their scene camera must not compete with the aircraft's.
        let ball = spawn_scene(&mut commands, &mut assets, &scene, placement, &mut |_, _, _, _| {});
        commands.entity(ball).insert((
            Ball,
            LinearVelocity(velocity),
            AngularVelocity(body_spin.0),
            Lifetime(Timer::from_seconds(LIFETIME_SECONDS, TimerMode::Once)),
        ));
    }
}

fn expire(time: Res<Time>, mut commands: Commands, mut balls: Query<(Entity, &mut Lifetime)>) {
    for (entity, mut lifetime) in &mut balls {
        if lifetime.0.tick(time.delta()).is_finished() {
            commands.entity(entity).despawn();
        }
    }
}
