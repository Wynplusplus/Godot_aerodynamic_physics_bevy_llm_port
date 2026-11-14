//! Demo-only node classes from the Godot demo scripts (camera rig, wind arrow, controllers).
//!
//! `aero_physics` spawns the physics and generic nodes. These hooks add the components and systems
//! for the classes in `misc_demo_assets/`.

use aero_physics::body::update_aero_controls;
use aero_physics::godot::{looking_at_safe, remap};
use aero_physics::schema::NodeDef;
use aero_physics::AeroBody;
use avian3d::prelude::*;
use bevy::math::{Mat3, Quat, Vec3};
use bevy::prelude::*;
use bevy::window::PrimaryWindow;

pub struct HooksPlugin;

impl Plugin for HooksPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(FixedPreUpdate, (airspeed_control, mouse_control).before(update_aero_controls))
            .add_systems(Update, (camera_rig_update, wind_arrow_update, select_aircraft_camera));
    }
}

/// A `Camera3D` of an aircraft. Only the one under a `CameraRotator` is switched on.
#[derive(Component)]
pub struct AircraftCamera;

/// `CameraRotator`: keeps the camera looking along the flight path, as in `camera_rotator.gd`.
#[derive(Component)]
pub struct CameraRig {
    pub body: Entity,
    pub rotation_speed: f32,
}

/// `Label3D`: the controls text shown in the corner.
#[derive(Component)]
pub struct ControlsText(pub String);

/// `WindIndicator`: points along the wind.
#[derive(Component)]
pub struct WindArrow {
    pub body: Entity,
}

/// `AirspeedControl` (from `f14.tscn`): drives a control axis from air speed.
#[derive(Component)]
pub struct AirspeedControl {
    pub body: Entity,
    pub axis: String,
    pub min_speed: f32,
    pub max_speed: f32,
}

/// `MouseControl`: aims the autopilot at the point under the mouse cursor.
#[derive(Component)]
pub struct MouseControl {
    pub body: Entity,
}

/// `BallLauncher`: fires a drag ball from this marker when Enter or Space is pressed.
#[derive(Component)]
pub struct BallLauncher {
    pub body: Entity,
}

/// Called for every node of a spawned aircraft.
pub fn on_node(def: &NodeDef, entity: Entity, root: Entity, commands: &mut Commands) {
    match def.class.as_str() {
        "Camera3D" => {
            let projection = PerspectiveProjection { fov: 75.0_f32.to_radians(), far: def.prop_f32("far", 1000.0), ..default() };
            commands.entity(entity).insert((Camera3d::default(), Projection::Perspective(projection), AircraftCamera, Camera { is_active: false, ..default() }));
        }
        "CameraRotator" => {
            commands.entity(entity).insert(CameraRig { body: root, rotation_speed: 180.0_f32.to_radians() });
        }
        "Label3D" => {
            let text = def.prop_str("text").unwrap_or_default().to_string();
            commands.entity(entity).insert(ControlsText(text));
        }
        "WindIndicator" => {
            commands.entity(entity).insert(WindArrow { body: root });
        }
        "AirspeedControl" => {
            commands.entity(entity).insert(AirspeedControl {
                body: root,
                axis: def.prop_str("axis").unwrap_or("wing_sweep").to_string(),
                min_speed: def.prop_f32("min_speed", 225.0),
                max_speed: def.prop_f32("max_speed", 275.0),
            });
        }
        "MouseControl" => {
            commands.entity(entity).insert(MouseControl { body: root });
        }
        "BallLauncher" => {
            commands.entity(entity).insert(BallLauncher { body: root });
        }
        _ => {}
    }
}

/// `AirspeedControl._physics_process()`: writes the wing sweep command from the air speed.
fn airspeed_control(controls: Query<&AirspeedControl>, mut bodies: Query<&mut AeroBody>) {
    for control in &controls {
        let Ok(mut body) = bodies.get_mut(control.body) else {
            continue;
        };
        let value = remap(body.air_speed, control.min_speed, control.max_speed, 0.0, 1.0).clamp(0.0, 1.0);
        body.controls.set_control_input(&control.axis, value);
    }
}

/// `MouseControl._ready()` and `_physics_process()`: direction-target autopilot aimed by the mouse.
fn mouse_control(
    controls: Query<&MouseControl>,
    mut bodies: Query<&mut AeroBody>,
    cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    windows: Query<&Window, With<PrimaryWindow>>,
) {
    let Some(cursor) = windows.single().ok().and_then(|w| w.cursor_position()) else {
        return;
    };
    let Some((camera, camera_transform)) = cameras.iter().find(|(c, _)| c.is_active) else {
        return;
    };
    let Ok(ray) = camera.viewport_to_world(camera_transform, cursor) else {
        return;
    };
    for control in &controls {
        let Ok(mut body) = bodies.get_mut(control.body) else {
            continue;
        };
        if let Some(fa) = body.flight_assist.as_mut() {
            fa.enable_target_direction = true;
            fa.direction_target = ray.direction.as_vec3();
        }
    }
}

/// Switches on the aircraft camera that sits under a `CameraRotator`.
fn select_aircraft_camera(rigs: Query<(), With<CameraRig>>, mut cameras: Query<(&ChildOf, &mut Camera), With<AircraftCamera>>) {
    for (parent, mut camera) in &mut cameras {
        let wanted = rigs.contains(parent.parent());
        if camera.is_active != wanted {
            camera.is_active = wanted;
        }
    }
}

/// `CameraRotator._process()`: turns the camera toward the flight path, lerping with speed.
fn camera_rig_update(time: Res<Time>, bodies: Query<(&Transform, &LinearVelocity, &AeroBody)>, mut rigs: Query<(&CameraRig, &mut Transform), Without<AeroBody>>) {
    for (rig, mut transform) in &mut rigs {
        let Ok((body_transform, linear_velocity, _)) = bodies.get(rig.body) else {
            continue;
        };
        let body_rotation = body_transform.rotation;
        let body_basis = Mat3::from_quat(body_rotation);

        let mut direction_to_look = linear_velocity.0;
        if direction_to_look.normalize_or_zero().dot(-body_basis.z_axis.normalize_or_zero()) < 0.0 {
            direction_to_look = -linear_velocity.0;
        }
        let flightpath_basis = looking_at_safe(direction_to_look, body_basis.y_axis, body_basis.z_axis, body_basis);

        let current_global = body_rotation * transform.rotation;
        let current_basis = Mat3::from_quat(current_global);

        // The camera faces forward when slow, and along the flight path when fast.
        let lerp_factor = remap(direction_to_look.length(), 10.0, 40.0, 0.0, 0.5).clamp(0.0, 0.5);
        let target = Quat::from_mat3(&body_basis).slerp(Quat::from_mat3(&flightpath_basis), lerp_factor);

        let necessary = Quat::from_mat3(&(current_basis.inverse() * Mat3::from_quat(target)));
        let (axis, angle) = godot_axis_angle(necessary);
        let amount = angle.min(rig.rotation_speed * time.delta_secs());
        let rotation_to_make = if axis.length() > 1e-6 { Quat::from_axis_angle(axis.normalize(), amount) } else { Quat::IDENTITY };

        let new_global = current_global * rotation_to_make;
        transform.rotation = body_rotation.inverse() * new_global;
    }
}

/// Godot's `Quaternion.get_axis()` and `get_angle()`.
fn godot_axis_angle(q: Quat) -> (Vec3, f32) {
    let w = q.w.clamp(-1.0, 1.0);
    let angle = 2.0 * w.acos();
    let s = (1.0 - w * w).max(0.0).sqrt();
    let axis = if s < 1e-6 { Vec3::ZERO } else { Vec3::new(q.x, q.y, q.z) / s };
    (axis, angle)
}

/// `WindIndicator._process()`: points the arrow along the wind, in world space.
fn wind_arrow_update(bodies: Query<&AeroBody>, globals: Query<&GlobalTransform>, mut arrows: Query<(&WindArrow, &ChildOf, &mut Transform)>) {
    for (arrow, parent, mut transform) in &mut arrows {
        let Ok(body) = bodies.get(arrow.body) else {
            continue;
        };
        let wanted = if body.wind.length() > 1e-6 { looking_at_safe(body.wind, Vec3::Y, Vec3::Z, Mat3::IDENTITY) } else { Mat3::IDENTITY };
        let parent_rotation = globals.get(parent.parent()).map_or(Quat::IDENTITY, |g| g.rotation());
        transform.rotation = parent_rotation.inverse() * Quat::from_mat3(&wanted);
    }
}
