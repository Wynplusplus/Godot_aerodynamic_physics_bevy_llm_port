//! Maps keys to the action names used by the scene controls (`ControlAxis` events).

use bevy::prelude::*;

use aero_physics::ActionInputs;

/// Action name and the keys that trigger it. Matches the `add_key_input` calls in the Godot demo.
const BINDINGS: &[(&str, &[KeyCode])] = &[
    ("pitch_up", &[KeyCode::KeyS]),
    ("pitch_down", &[KeyCode::KeyW]),
    ("yaw_left", &[KeyCode::KeyQ]),
    ("yaw_right", &[KeyCode::KeyE]),
    ("roll_right", &[KeyCode::KeyD]),
    ("roll_left", &[KeyCode::KeyA]),
    ("throttle_up", &[KeyCode::ShiftLeft, KeyCode::ShiftRight]),
    ("throttle_down", &[KeyCode::ControlLeft, KeyCode::ControlRight]),
    ("brake_up", &[KeyCode::KeyB]),
    ("tilt_up", &[KeyCode::KeyC]),
    ("tilt_down", &[KeyCode::KeyV]),
    ("collective_up", &[KeyCode::KeyT]),
    ("collective_down", &[KeyCode::KeyG]),
    // Godot's built-in ui_* actions, used by axes that do not name their own keys.
    ("ui_up", &[KeyCode::ArrowUp]),
    ("ui_down", &[KeyCode::ArrowDown]),
    ("ui_left", &[KeyCode::ArrowLeft]),
    ("ui_right", &[KeyCode::ArrowRight]),
    ("ui_page_up", &[KeyCode::PageUp]),
    ("ui_page_down", &[KeyCode::PageDown]),
    ("ui_accept", &[KeyCode::Enter, KeyCode::Space]),
];

pub struct InputPlugin;

impl Plugin for InputPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PreUpdate, read_keys);
    }
}

fn read_keys(keys: Res<ButtonInput<KeyCode>>, mut inputs: ResMut<ActionInputs>) {
    for (action, codes) in BINDINGS {
        let pressed = codes.iter().any(|code| keys.pressed(*code));
        inputs.0.insert(action.to_string(), if pressed { 1.0 } else { 0.0 });
    }
}

/// True when the action was pressed this frame, for one-shot actions such as launching a ball.
pub fn just_pressed(keys: &ButtonInput<KeyCode>, action: &str) -> bool {
    BINDINGS
        .iter()
        .find(|(name, _)| *name == action)
        .is_some_and(|(_, codes)| codes.iter().any(|code| keys.just_pressed(*code)))
}
