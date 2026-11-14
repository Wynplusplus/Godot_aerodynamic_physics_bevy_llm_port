//! The vehicle menu (`VehicleSpawner` of demo_scene.tscn), built with Bevy UI.

use bevy::prelude::*;
use bevy::text::FontSize;
use bevy::ui::{ComputedNode, UiGlobalTransform};
use bevy::window::PrimaryWindow;

use aero_physics::schema::SceneFile;
use aero_physics::{SpawnAssets, spawn_scene};

use crate::ball::Ball;
use crate::input::just_pressed;
use crate::hooks;
use crate::world::{MenuCamera, land_spawn, water_spawn};

/// One entry of the vehicle menu. `file` is the converted scene in `assets/aircraft/`.
pub struct Vehicle {
    pub name: &'static str,
    pub file: &'static str,
    pub water: bool,
}

/// The same vehicles, in the same order, as the GDScript dictionary in demo_scene.tscn.
pub const VEHICLES: &[Vehicle] = &[
    Vehicle { name: "Demo plane", file: "demo_plane", water: false },
    Vehicle { name: "Cessna 172", file: "cessna172", water: false },
    Vehicle { name: "F-14 Tomcat", file: "f14", water: false },
    Vehicle { name: "F-14 Tomcat Mouse Control", file: "f14_mouse_control", water: false },
    Vehicle { name: "AIM-9 Sidewinder", file: "aim9", water: false },
    Vehicle { name: "Quadcopter", file: "quadcopter", water: false },
    Vehicle { name: "Su-57 \"Felon\"", file: "su57", water: false },
    Vehicle { name: "Su-47 \"Berkut\"", file: "berkut", water: false },
    Vehicle { name: "B-2 Spirit", file: "b2", water: false },
    Vehicle { name: "V-22 Osprey", file: "v22", water: false },
    Vehicle { name: "UH-60 \"Blackhawk\"", file: "blackhawk", water: false },
    Vehicle { name: "PBY Catalina", file: "catalina", water: true },
    Vehicle { name: "Boat", file: "boat", water: true },
    Vehicle { name: "Sailboat", file: "sailboat", water: true },
    Vehicle { name: "Floaty ball", file: "floaty_ball", water: true },
];

/// Marks every entity that belongs to the flying vehicle. Despawned when returning to the menu.
#[derive(Component)]
pub struct Aircraft;

#[derive(Resource)]
pub struct Menu {
    pub selected: usize,
    pub open: bool,
}

#[derive(Component)]
struct MenuRoot;

#[derive(Component)]
struct VehicleButton(usize);

pub struct MenuPlugin;

impl Plugin for MenuPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Menu { selected: 0, open: true })
            .add_systems(Startup, build_menu)
            .add_systems(Update, (menu_keys, button_clicks, highlight_selection, return_to_menu, sync_menu_visibility));
    }
}

fn build_menu(mut commands: Commands) {
    let root = commands
        .spawn((
            MenuRoot,
            Node {
                position_type: PositionType::Absolute,
                width: percent(100),
                height: percent(100),
                flex_direction: FlexDirection::Column,
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                row_gap: px(6),
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.45)),
        ))
        .id();

    let title = commands
        .spawn((Text::new("Select a vehicle"), TextFont { font_size: FontSize::Px(34.0), ..default() }, TextColor(Color::WHITE)))
        .id();
    commands.entity(root).add_child(title);

    for (i, vehicle) in VEHICLES.iter().enumerate() {
        let button = commands
            .spawn((
                VehicleButton(i),
                Node { width: px(360), height: px(30), align_items: AlignItems::Center, justify_content: JustifyContent::Center, ..default() },
                BackgroundColor(Color::srgb(0.12, 0.12, 0.14)),
            ))
            .with_children(|b| {
                b.spawn((Text::new(vehicle.name), TextFont { font_size: FontSize::Px(18.0), ..default() }, TextColor(Color::WHITE)));
            })
            .id();
        commands.entity(root).add_child(button);
    }

    let hint = commands
        .spawn((
            Text::new("Up/Down to choose, Enter or click to spawn. F5 returns here."),
            TextFont { font_size: FontSize::Px(16.0), ..default() },
            TextColor(Color::srgb(0.85, 0.85, 0.85)),
        ))
        .id();
    commands.entity(root).add_child(hint);
}

fn menu_keys(mut menu: ResMut<Menu>, keys: Res<ButtonInput<KeyCode>>, mut commands: Commands, server: Res<AssetServer>, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>, mut cameras: Query<&mut Camera, With<MenuCamera>>) {
    if !menu.open {
        return;
    }
    if keys.just_pressed(KeyCode::ArrowDown) {
        menu.selected = (menu.selected + 1) % VEHICLES.len();
    }
    if keys.just_pressed(KeyCode::ArrowUp) {
        menu.selected = (menu.selected + VEHICLES.len() - 1) % VEHICLES.len();
    }
    if just_pressed(&keys, "ui_accept") {
        let index = menu.selected;
        spawn_vehicle(&mut commands, &server, &mut meshes, &mut materials, index);
        open_flight(&mut menu, &mut cameras);
    }
}

#[allow(clippy::too_many_arguments)]
fn button_clicks(
    mut menu: ResMut<Menu>,
    mouse: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    buttons: Query<(&ComputedNode, &UiGlobalTransform, &VehicleButton)>,
    mut commands: Commands,
    server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut cameras: Query<&mut Camera, With<MenuCamera>>,
) {
    if !menu.open || !mouse.just_pressed(MouseButton::Left) {
        return;
    }
    let Some(cursor) = windows.single().ok().and_then(|w| w.cursor_position()) else {
        return;
    };
    for (node, transform, button) in &buttons {
        if node.contains_point(*transform, cursor) {
            menu.selected = button.0;
            spawn_vehicle(&mut commands, &server, &mut meshes, &mut materials, button.0);
            open_flight(&mut menu, &mut cameras);
            return;
        }
    }
}

fn open_flight(menu: &mut Menu, cameras: &mut Query<&mut Camera, With<MenuCamera>>) {
    menu.open = false;
    for mut camera in cameras.iter_mut() {
        camera.is_active = false;
    }
}

fn highlight_selection(menu: Res<Menu>, mut buttons: Query<(&VehicleButton, &mut BackgroundColor)>) {
    for (button, mut color) in &mut buttons {
        color.0 = if button.0 == menu.selected { Color::srgb(0.25, 0.42, 0.7) } else { Color::srgb(0.12, 0.12, 0.14) };
    }
}

/// F5 despawns the vehicle and shows the menu again.
fn return_to_menu(
    mut menu: ResMut<Menu>,
    keys: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    aircraft: Query<Entity, With<Aircraft>>,
    balls: Query<Entity, With<Ball>>,
    mut cameras: Query<&mut Camera, With<MenuCamera>>,
) {
    if menu.open || !keys.just_pressed(KeyCode::F5) {
        return;
    }
    for entity in &aircraft {
        commands.entity(entity).despawn();
    }
    for entity in &balls {
        commands.entity(entity).despawn();
    }
    menu.open = true;
    for mut camera in cameras.iter_mut() {
        camera.is_active = true;
    }
}

fn sync_menu_visibility(menu: Res<Menu>, mut roots: Query<&mut Visibility, With<MenuRoot>>) {
    if !menu.is_changed() {
        return;
    }
    for mut visibility in &mut roots {
        *visibility = if menu.open { Visibility::Inherited } else { Visibility::Hidden };
    }
}

/// Loads a converted scene from `assets/aircraft/<file>.json`.
pub fn load_scene(file: &str) -> Option<SceneFile> {
    let path = format!("{}/assets/aircraft/{}.json", env!("CARGO_MANIFEST_DIR"), file);
    let text = match std::fs::read_to_string(&path) {
        Ok(text) => text,
        Err(err) => {
            error!("cannot read {path}: {err}");
            return None;
        }
    };
    match serde_json::from_str(&text) {
        Ok(scene) => Some(scene),
        Err(err) => {
            error!("cannot parse {path}: {err}");
            None
        }
    }
}

/// Spawns the vehicle at the spawn marker for its medium.
fn spawn_vehicle(
    commands: &mut Commands,
    server: &AssetServer,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    index: usize,
) {
    let vehicle = &VEHICLES[index];
    let Some(scene) = load_scene(vehicle.file) else {
        return;
    };
    let placement = if vehicle.water { water_spawn() } else { land_spawn() };
    let mut assets = SpawnAssets { server, meshes, materials };
    let root = spawn_scene(commands, &mut assets, &scene, placement, &mut |def, entity, root, cmds| hooks::on_node(def, entity, root, cmds));
    commands.entity(root).insert(Aircraft);
}
