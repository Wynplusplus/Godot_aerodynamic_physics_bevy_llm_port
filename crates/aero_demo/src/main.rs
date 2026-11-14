//! Demo scene for `aero_physics`: pick an aircraft from the menu and fly it.
//!
//! Controls (from the Godot demo): W/S pitch, A/D roll, Q/E yaw, Shift/Ctrl throttle, B brake,
//! C/V tilt, T/G collective, Enter or Space launches a ball (BallLauncher), F5 returns to the menu.

mod ball;
mod hooks;
mod hud;
mod input;
mod menu;
mod world;

use avian3d::prelude::*;
use bevy::asset::AssetPlugin;
use bevy::prelude::*;
use bevy::window::WindowResolution;

use aero_physics::AeroPhysicsPlugin;

fn main() {
    App::new()
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "aero_physics demo".into(),
                        resolution: WindowResolution::new(1280, 720),
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin { file_path: concat!(env!("CARGO_MANIFEST_DIR"), "/assets").into(), ..default() }),
        )
        // Godot's defaults: 60 Hz physics and 9.8 m/s^2 gravity.
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(Gravity(Vec3::NEG_Y * 9.8))
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(AeroPhysicsPlugin)
        .add_plugins((input::InputPlugin, world::WorldPlugin, menu::MenuPlugin, hooks::HooksPlugin, hud::HudPlugin, ball::BallPlugin))
        .run();
}
