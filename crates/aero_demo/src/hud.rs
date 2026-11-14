//! The on-screen readouts of `flight_info_label.tscn`, `floating_hud.tscn`, and the debug vectors
//! of `AeroBody3D` (`show_debug`).

use avian3d::prelude::*;
use bevy::math::{Mat3, Vec3};
use bevy::prelude::*;
use bevy::text::FontSize;

use aero_physics::godot::{v3_log_with_base, vec_times_basis};
use aero_physics::AeroBody;

use crate::hooks::ControlsText;
use crate::menu::Aircraft;

/// Text of the flight readout (IAS, ALT, Mach, AoA, G, wind).
#[derive(Component)]
struct FlightInfoText;

/// The controls text, in the top-left corner.
#[derive(Component)]
struct ControlsHud;

/// The flight-path marker: a box at the projected velocity vector.
#[derive(Component)]
struct FlightPathHud;

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, build_hud).add_systems(Update, (update_text, update_flight_path, draw_debug_vectors));
    }
}

fn build_hud(mut commands: Commands) {
    commands.spawn((
        ControlsHud,
        Text::new(""),
        TextFont { font_size: FontSize::Px(16.0), ..default() },
        TextColor(Color::srgb(0.9, 0.9, 0.9)),
        Node { position_type: PositionType::Absolute, left: px(12), top: px(12), ..default() },
    ));
    commands.spawn((
        FlightInfoText,
        Text::new(""),
        TextFont { font_size: FontSize::Px(18.0), ..default() },
        TextColor(Color::srgb(0.0, 1.0, 0.0)),
        Node { position_type: PositionType::Absolute, right: px(16), bottom: px(16), ..default() },
    ));
    commands.spawn((
        FlightPathHud,
        Node { position_type: PositionType::Absolute, width: px(22), height: px(22), border: UiRect::all(px(2)), display: Display::None, ..default() },
        BorderColor::all(Color::srgb(0.0, 1.0, 0.0)),
    ));
}

fn update_text(
    controls: Query<&ControlsText>,
    bodies: Query<(&AeroBody, &Transform), With<Aircraft>>,
    mut controls_hud: Query<&mut Text, (With<ControlsHud>, Without<FlightInfoText>)>,
    mut info: Query<&mut Text, (With<FlightInfoText>, Without<ControlsHud>)>,
) {
    if let Some(label) = controls.iter().next() {
        for mut text in &mut controls_hud {
            if text.0 != label.0 {
                text.0 = label.0.clone();
            }
        }
    }

    let Some((body, transform)) = bodies.iter().next() else {
        for mut text in &mut info {
            text.0.clear();
        }
        return;
    };
    let basis = Mat3::from_quat(transform.rotation);
    let g = vec_times_basis(body.linear_acceleration - body.current_gravity, basis).y / 9.81;
    let altitude = body.altitude.round() as i64;
    let alt = if altitude < 0 { format!("{altitude}") } else { format!(" {altitude}") };
    let text = format!(
        "IAS: {:.1}m/s\nALT:{alt}m\nM:   {:.2}\na:   {:.1}\nG:   {:.1}\nWind:{:.1}m/s",
        body.air_speed,
        body.mach,
        body.angle_of_attack.to_degrees(),
        g,
        body.wind.length()
    );
    for mut t in &mut info {
        t.0 = text.clone();
    }
}

/// Places the flight-path marker where the velocity vector points on screen.
fn update_flight_path(
    bodies: Query<(&AeroBody, &Transform, &LinearVelocity), With<Aircraft>>,
    cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    mut marker: Query<&mut Node, With<FlightPathHud>>,
) {
    let Ok(mut node) = marker.single_mut() else {
        return;
    };
    let Some((camera, camera_transform)) = cameras.iter().find(|(c, _)| c.is_active) else {
        node.display = Display::None;
        return;
    };
    let Some((_, transform, velocity)) = bodies.iter().next() else {
        node.display = Display::None;
        return;
    };
    if velocity.0.length() < 1e-3 {
        node.display = Display::None;
        return;
    }
    let point = transform.translation + velocity.0.normalize() * 1000.0;
    match camera.world_to_viewport(camera_transform, point) {
        Ok(pos) => {
            node.display = Display::Flex;
            node.left = px(pos.x - 11.0);
            node.top = px(pos.y - 11.0);
        }
        Err(_) => node.display = Display::None,
    }
}

/// Debug vectors at the centre of mass, from the influencer forces (`AeroBody3D._update_debug`).
fn draw_debug_vectors(bodies: Query<(&AeroBody, &Transform)>, mut gizmos: Gizmos) {
    const SCALE: f32 = 0.1;
    for (body, transform) in &bodies {
        if !body.show_debug {
            continue;
        }
        let basis = Mat3::from_quat(transform.rotation);
        let origin = transform.translation + basis * body.center_of_mass;

        let (mut force, mut drag, mut lift, mut thrust) = (Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
        for node in body.nodes.iter().filter(|n| !n.omit_from_debug && !n.disabled) {
            let f = node.state.current_force;
            let dir = node.state.drag_direction;
            let along = f.dot(dir);
            force += f;
            drag += along.max(0.0) * dir;
            lift += f - along * dir;
            thrust += along.min(0.0) * dir;
        }

        let draw = |gizmos: &mut Gizmos, v: Vec3, color: Color| {
            if v.length_squared() > 1e-8 {
                gizmos.arrow(origin, origin + v3_log_with_base(v, 2.0) * SCALE, color);
            }
        };
        draw(&mut gizmos, force, Color::WHITE);
        draw(&mut gizmos, lift, Color::srgb(0.0, 1.0, 1.0));
        draw(&mut gizmos, drag, Color::srgb(1.0, 0.0, 0.0));
        draw(&mut gizmos, thrust, Color::srgb(1.0, 0.0, 1.0));
        draw(&mut gizmos, body.linear_velocity_substep, Color::srgb(0.0, 0.5, 0.5));
        draw(&mut gizmos, body.angular_velocity_substep, Color::srgb(0.0, 0.8, 0.0));
    }
}
