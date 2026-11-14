# Porting notes: godot_aerodynamic_physics -> bevy_aero_physics

## Class mapping

| Godot | Rust | Notes |
| --- | --- | --- |
| `AeroBody3D` (VehicleBody3D) | `aero_physics::AeroBody` on an Avian `RigidBody::Dynamic` | Integrator in `body.rs` mirrors `calculate_forces` and the substep loop. Mass comes from `Mass`, the centre of mass from `CenterOfMass`. |
| `AeroControlComponent` | `AeroBody::controls`, `AeroBody::flight_assist` | Lives on the body instead of a child node. |
| `AeroControlConfig`, `...Axis` | `controls::ControlConfig`, `ControlAxis` | Inputs come from `ActionInputs`, not `Input`. |
| `AeroInfluencerControlConfig`, `...AxisConfig` | `controls::ActuationConfig`, `ActuationAxis` | Expressions are compiled once with `expr.rs`. |
| `FlightAssist`, `aero_PID` | `flight_assist::FlightAssist`, `pid::Pid` | Defaults are the GDScript export defaults. |
| `AeroInfluencer3D` and subclasses | `influencer::Node` + `Kind` | One flat `Vec<Node>` per body, walked recursively. `Kind::Base`, `Surface`, `ManualSurface`, `Drag`, `Thruster` (+ jet), `Mover`, `Propeller`, `CyclicPropeller`, `BuoyancySphere`, `BuoyancyMesh`. |
| `AeroSurface3D`, `ManualAeroSurface3D` | `Kind::Surface`, `Kind::ManualSurface` | Lift and drag curves are Godot `Curve`s (`curve.rs`). |
| `AeroBuoyancyMesh3D`, `AeroBuoyancySphere3D` | `buoyancy.rs`, `Kind::BuoyancySphere` | Mesh preprocessing is the same algorithm. |
| `AeroAtmosphere3D`, `AeroWaterAtmosphere3D` | `atmosphere::AeroAtmosphere`, `AtmosphereKind::Water` | Membership comes from Avian sensor contacts. |
| `AeroUnits` singleton | `units::AeroUnits` resource | ISA table and Mach curves. |
| `utils/*` | `godot.rs`, `curve.rs`, `expr.rs` | Godot semantics: `Vector3 * Basis`, `ease()`, `sign(0) == 0`, Euler orders, `Curve.sample_baked()`. |
| Demo scripts | `crates/aero_demo/src/hooks.rs`, `hud.rs`, `ball.rs`, `menu.rs` | Camera rig, wind arrow, airspeed-driven wing sweep, mouse-aimed autopilot, ball launcher, HUD text. |
| `demo_scene.tscn` | `world.rs`, `menu.rs` | Ground, water, wind layer, spawn markers, vehicle list. |

## Conventions that were kept on purpose

- Godot's tscn `transform` is 9 basis floats in column order followed by the origin.
- `Vector3 * Basis` is implemented as `basis.transpose() * v`.
- Euler order `ABC` means `R_C * R_B * R_A`; `euler_from_basis` inverts this exactly (unit tested for all six orders).
- Mirrored copies are siblings named `<name>Mirror`, with every descendant marked as a duplicate, as `generate_mirror()` does.
- Propeller blades are rotated copies of the template blade around local Y.
- Motors (`linear_motor`) update the default origin only. This is how the GDScript behaves: the node's `position` is not changed.

## Verification

- `cargo test -p aero_physics` (12 tests): unit tests for the Euler round trips in all six orders, `ease`, `remap`, the Godot-style curves, the expression evaluator, and the ISA table; and headless scene tests that run the real Avian step:
  - the demo plane flies at 60 m/s without diverging;
  - the floaty ball enters the water volume (sensor membership is reported), is lifted by buoyancy, and leaves the volume;
  - the Cessna settles on its wheels on a ground plane;
  - the demo plane stays finite with `experimental_energy_tracking` on.
- `cargo clippy --workspace --all-targets`: no warnings.
- The demo starts, creates its window, and runs without errors. Menu clicks and spawning were not exercised by hand.

## Deliberate differences

1. **Physics engine.** Avian 0.8 replaces Godot's physics server. Forces are applied with Avian's `Forces` query inside `PhysicsSystems::Prepare`, which is the equivalent of `_integrate_forces`. Gravity is applied by Avian as in Godot.
2. **Tick rate.** The app sets 60 Hz, Godot's default physics rate.
3. **Atmosphere membership.** Godot uses Area3D `body_entered`/`body_exited`. Here the sensor's `CollidingEntities` is mapped back to bodies through `ColliderOf`, which is the same overlap test.
4. **Buoyancy meshes.** Godot stores them in binary `.res` files. `tools/convert_scenes.py` takes the geometry from the glTF node that carries the same mesh (`Hull`, `Fueslage`, `Fueslage_aero`, and so on). The `.res` files were not decoded. glTF triangles are counter-clockwise, so the vertex normals are computed as `cross(b - a, c - a)`. The GDScript's `cross(c - a, b - a)` assumes Godot's clockwise winding; with glTF data it gives inward normals, which push the vertices down.
5. **Zero-depth guard.** When a buoyancy vertex's contact depth is zero, the GDScript divides by zero. The port treats the vertex as fully submerged or not, depending on the sign of the distance.
6. **Expressions.** `expression_x/y/z` are evaluated by a small evaluator (`expr.rs`) that covers arithmetic, `clamp`, `remap`, `min`, `max`, `abs`, `sign`, `pow`, `sqrt`, `lerp`, `deg_to_rad`, `rad_to_deg`, and `get_control_command`. Other Godot built-ins are not implemented; they evaluate to 0.
7. **Visual links.** The GDScript snippets that copy one node's `rotation.x` onto another (control surfaces, mirrored wings) are converted to `RotationFollow` links, which keep the Euler conventions of each node. The scripts themselves are not run.
8. **Wheels.** `VehicleWheel3D` is a ray-cast spring and damper with sideways friction (`wheels.rs`), applied at the contact points. Godot's VehicleBody3D solver is not reproduced: there is no engine force, steering, or braking, and the spring is scaled by each wheel's share of the mass (stiffness is a rate, 1/s^2).
8a. **Mass and inertia.** Avian applies an explicit `Mass` but keeps the angular inertia that the colliders' density gives. Godot scales the inertia with the mass. The port sets every body collider's density to `mass / total_volume` and puts `NoAutoMass` on the body, so the collider mass does not add to the total. Without this the floaty ball and the other bodies spun up and diverged.
9. **Not ported.** `AeroRotorHinge3D`. Upstream marks it "currently broken", it is not used by any demo, and its rotation writes are overwritten by the next substep's control update, so a faithful port would only reproduce broken behaviour. Experimental energy tracking (`experimental_energy_tracking`) is ported and covered by a test.
10. **Input.** Keyboard only. Gamepad bindings (`JOY_AXIS_*`) are not mapped because gamepad support needs libudev, which was not available when building.
11. **Mouse capture.** The Godot `MouseControl` captures the mouse. The port keeps the cursor free and aims at the cursor position.
12. **HUD.** Godot's 3D labels and quads (IAS/ALT/M/a/G readout, flight-path marker, controls text) are drawn as screen-space UI in the corners and at the velocity vector.
13. **Camera.** Field of view is 75 degrees, as Godot's `Camera3D` default.
14. **Euler convention check.** `Basis.from_euler` is assumed to be `R_C * R_B * R_A` for an order `ABC`. The decomposition and the composition are consistent with each other, and all demo axis rotations are single-axis, so the convention only matters for multi-axis control rotations.
15. **Water surface.** The base `AeroWaterAtmosphere3D.get_surface_height()` returns 0, so the water surface is flat, as in the demo.
