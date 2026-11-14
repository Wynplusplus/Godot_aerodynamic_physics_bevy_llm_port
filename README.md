# bevy_aero_physics

A Bevy 0.20 port of [godot_aerodynamic_physics](https://github.com/addmix/godot_aerodynamic_physics)
by Addmix, including the demo scene. The physics runs on [Avian 0.8](https://github.com/avianphysics/avian).

This port was written with the help of a large language model (Claude, by Anthropic). The original
design, physics model, scenes and models are the work of Addmix; any mistakes in the port are the
LLM's and the repository maintainer's, not the original author's.

```
crates/aero_physics   the library: aerodynamic model, Bevy components/systems, scene loader
crates/aero_demo      the demo: vehicle menu, flight camera, HUD, water and wind volumes, ball launcher
tools/convert_scenes.py   converts the Godot .tscn scenes and .glb models into crates/aero_demo/assets
PORTING_NOTES.md      how each Godot class maps onto the Rust code, and where the port differs
```

## Running

1. Generate the demo assets from a checkout of the Godot project:

   ```
   python3 tools/convert_scenes.py /path/to/godot_aerodynamic_physics/demo crates/aero_demo/assets
   ```

2. Run the demo:

   ```
   cargo run -p aero_demo --release
   ```

3. Tests (headless; they run the real physics step):

   ```
   cargo test -p aero_physics
   ```

## Controls

| Key | Action |
| --- | --- |
| W / S | pitch |
| A / D | roll |
| Q / E | yaw |
| Shift / Ctrl | throttle |
| B, C / V, T / G | brake, tilt rotor, collective (where the vehicle has them) |
| Up / Down, Enter | menu selection (menu), `ui_accept` launches a ball in flight |
| F5 | return to the menu |

Arrow keys and PageUp/PageDown drive the `ui_*` actions, which the Godot defaults also use.

## Library use

```rust
app.add_plugins((PhysicsPlugins::default(), AeroPhysicsPlugin));
app.insert_resource(Time::<Fixed>::from_hz(60.0));   // Godot's default physics rate
app.insert_resource(Gravity(Vec3::NEG_Y * 9.8));      // Godot's default gravity

// Spawn a converted scene (see tools/convert_scenes.py):
let scene: SceneFile = serde_json::from_str(&json)?;
spawn_scene(&mut commands, &mut assets, &scene, placement, &mut |def, entity, root, cmds| { /* app classes */ });

// Each frame, feed input strengths for the actions named in the scene:
inputs.0.insert("pitch_up".into(), 1.0);
```

## Credits and license

- Original project: [godot_aerodynamic_physics](https://github.com/addmix/godot_aerodynamic_physics) by Addmix (MIT).
- This Bevy port was created with an LLM (Claude, by Anthropic) and is also MIT-licensed; see `LICENSE`.
