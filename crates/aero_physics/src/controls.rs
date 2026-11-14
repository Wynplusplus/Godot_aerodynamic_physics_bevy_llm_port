//! Port of `core/controls/*`: body-level control axes and per-influencer actuation.

use std::collections::HashMap;

use bevy::math::{Mat3, Vec3};
use bevy::prelude::Resource;
use serde::Deserialize;

use crate::expr::{ControlSource, Expr};
use crate::godot::{improved_ease, sign, vec_move_toward, vec_times_basis};
use crate::schema::v3;

/// Strength of every named input action this frame, in `[-1, 1]` (or `[0, 1]` for buttons).
///
/// The original plugin reads `Input.get_action_strength()`. The application fills this resource
/// each frame, so the physics code does not depend on any particular input backend.
#[derive(Resource, Default, Debug, Clone)]
pub struct ActionInputs(pub HashMap<String, f32>);

impl ActionInputs {
    pub fn strength(&self, action: &str) -> f32 {
        self.0.get(action).copied().unwrap_or(0.0)
    }
}

/// `AeroControlConfigAxis`.
#[derive(Deserialize, Debug, Clone, PartialEq)]
#[serde(default)]
pub struct ControlAxis {
    pub axis_name: String,
    pub use_bindings: bool,
    pub input: f32,
    pub min_limit: f32,
    pub max_limit: f32,
    pub positive_event: String,
    pub negative_event: String,
    pub enable_smoothing: bool,
    pub smoothing_rate: f32,
    pub cumulative_positive_event: String,
    pub cumulative_negative_event: String,
    pub cumulative_rate: f32,
    pub easing: f32,
    #[serde(skip)]
    pub smoothed_input: f32,
    #[serde(skip)]
    pub command: f32,
    #[serde(skip)]
    pub cumulative_input: f32,
    #[serde(skip)]
    pub cumulative_command: f32,
}

impl Default for ControlAxis {
    fn default() -> Self {
        Self {
            axis_name: String::new(),
            use_bindings: true,
            input: 0.0,
            min_limit: -1.0,
            max_limit: 1.0,
            positive_event: String::new(),
            negative_event: String::new(),
            enable_smoothing: false,
            smoothing_rate: 1.0,
            cumulative_positive_event: String::new(),
            cumulative_negative_event: String::new(),
            cumulative_rate: 1.0,
            easing: 1.0,
            smoothed_input: 0.0,
            command: 0.0,
            cumulative_input: 0.0,
            cumulative_command: 0.0,
        }
    }
}

impl ControlAxis {
    /// `AeroControlConfigAxis._init(name, min, max, increase_event, decrease_event, use_cumulative)`.
    pub fn new(name: &str, min_limit: f32, max_limit: f32, increase: &str, decrease: &str, cumulative: bool) -> Self {
        let mut axis = Self { axis_name: name.into(), min_limit, max_limit, ..Default::default() };
        if cumulative {
            axis.cumulative_positive_event = increase.into();
            axis.cumulative_negative_event = decrease.into();
        } else {
            axis.positive_event = increase.into();
            axis.negative_event = decrease.into();
        }
        axis
    }

    fn get_axis(&self, inputs: &ActionInputs, negative: &str, positive: &str) -> f32 {
        if !self.use_bindings {
            return 0.0;
        }
        let mut input = 0.0;
        if !negative.is_empty() {
            input -= inputs.strength(negative);
        }
        if !positive.is_empty() {
            input += inputs.strength(positive);
        }
        input
    }

    /// `AeroControlConfigAxis.update(delta)`. Returns the new command.
    pub fn update(&mut self, delta: f32, inputs: &ActionInputs) -> f32 {
        let total_input = self.input + self.get_axis(inputs, &self.negative_event, &self.positive_event);
        let total_cumulative_input =
            self.cumulative_input + self.get_axis(inputs, &self.cumulative_negative_event, &self.cumulative_positive_event);

        self.cumulative_command = update_cumulative_control(
            delta,
            total_cumulative_input,
            self.cumulative_command,
            self.cumulative_rate,
            self.min_limit,
            self.max_limit,
        );
        let total_desired = (total_input + self.cumulative_command).clamp(self.min_limit, self.max_limit);

        self.smoothed_input = calculate_smoothing(self.smoothed_input, total_desired, self.enable_smoothing, self.smoothing_rate, delta);
        self.command = improved_ease(self.smoothed_input, self.easing);
        self.command
    }
}

pub fn update_cumulative_control(delta: f32, input: f32, current: f32, rate: f32, min: f32, max: f32) -> f32 {
    (current + input * rate * delta).clamp(min, max)
}

pub fn calculate_smoothing(current: f32, target: f32, enabled: bool, rate: f32, delta: f32) -> f32 {
    if enabled {
        // Godot's move_toward(current, target, rate * delta)
        let step = rate * delta;
        if (target - current).abs() <= step {
            target
        } else {
            current + step * (target - current).signum()
        }
    } else {
        target
    }
}

/// `AeroControlConfig`: the list of named control axes.
#[derive(Deserialize, Debug, Clone, PartialEq)]
#[serde(default)]
pub struct ControlConfig {
    pub axis_configs: Vec<ControlAxis>,
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            axis_configs: vec![
                ControlAxis::new("pitch", -1.0, 1.0, "ui_up", "ui_down", false),
                ControlAxis { axis_name: "yaw".into(), ..Default::default() },
                ControlAxis::new("roll", -1.0, 1.0, "ui_left", "ui_right", false),
                ControlAxis::new("throttle", 0.0, 1.0, "ui_page_up", "ui_page_down", true),
                ControlAxis::new("brake", 0.0, 1.0, "", "", false),
                ControlAxis { axis_name: "collective".into(), ..Default::default() },
            ],
        }
    }
}

impl ControlConfig {
    pub fn update(&mut self, delta: f32, inputs: &ActionInputs) {
        for axis in &mut self.axis_configs {
            axis.update(delta, inputs);
        }
    }

    fn axis(&self, name: &str) -> Option<&ControlAxis> {
        self.axis_configs.iter().find(|a| a.axis_name == name)
    }

    fn axis_mut(&mut self, name: &str) -> Option<&mut ControlAxis> {
        self.axis_configs.iter_mut().find(|a| a.axis_name == name)
    }

    pub fn set_control_input(&mut self, name: &str, value: f32) {
        if let Some(a) = self.axis_mut(name) {
            a.input = value;
        }
    }

    pub fn control_input(&self, name: &str) -> f32 {
        self.axis(name).map_or(0.0, |a| a.input)
    }

    pub fn set_control_command(&mut self, name: &str, value: f32) {
        if let Some(a) = self.axis_mut(name) {
            a.command = value;
        }
    }

    pub fn control_command(&self, name: &str) -> f32 {
        self.axis(name).map_or(0.0, |a| a.command)
    }

    pub fn set_cumulative_control_command(&mut self, name: &str, value: f32) {
        if let Some(a) = self.axis_mut(name) {
            a.cumulative_command = value;
        }
    }

    pub fn cumulative_control_command(&self, name: &str) -> f32 {
        self.axis(name).map_or(0.0, |a| a.cumulative_command)
    }
}

impl ControlSource for ControlConfig {
    fn control_command(&self, axis: &str) -> f32 {
        ControlConfig::control_command(self, axis)
    }
}

/// `AeroInfluencerControlAxisConfig`: how one control axis rotates an influencer.
#[derive(Deserialize, Debug, Clone)]
#[serde(default)]
pub struct ActuationAxis {
    pub axis_name: String,
    #[serde(deserialize_with = "v3")]
    pub contribution: Vec3,
    pub easing: f32,
    pub axis_flip_x: i32,
    pub axis_flip_y: i32,
    pub axis_flip_z: i32,
    pub expression_x: String,
    pub expression_y: String,
    pub expression_z: String,
    #[serde(skip)]
    pub expressions: [Option<Expr>; 3],
}

impl Default for ActuationAxis {
    fn default() -> Self {
        Self {
            axis_name: String::new(),
            contribution: Vec3::ZERO,
            easing: 1.0,
            axis_flip_x: 0,
            axis_flip_y: 0,
            axis_flip_z: 0,
            expression_x: String::new(),
            expression_y: String::new(),
            expression_z: String::new(),
            expressions: [None, None, None],
        }
    }
}

impl ActuationAxis {
    /// Parses the expression strings. Must be called once after deserialising.
    pub fn compile(&mut self) -> Result<(), String> {
        for (slot, text) in self.expressions.iter_mut().zip([&self.expression_x, &self.expression_y, &self.expression_z]) {
            *slot = if text.is_empty() { None } else { Some(Expr::parse(text)?) };
        }
        Ok(())
    }

    /// `get_value(command, influencer, axis_sign)`.
    pub fn value(&self, command: f32, control: &dyn ControlSource, axis_sign: Vec3) -> Vec3 {
        let mut running = self.contribution;
        for (i, expr) in self.expressions.iter().enumerate() {
            if let Some(expr) = expr {
                running[i] = expr.eval(control);
            }
        }
        let flips = Vec3::new(
            axis_flip(axis_sign, self.axis_flip_x),
            axis_flip(axis_sign, self.axis_flip_y),
            axis_flip(axis_sign, self.axis_flip_z),
        );
        running * improved_ease(command, self.easing) * flips
    }
}

fn axis_flip(sign_vec: Vec3, flip: i32) -> f32 {
    match flip {
        1 => sign_vec.x,
        2 => sign_vec.y,
        3 => sign_vec.z,
        _ => 1.0,
    }
}

/// `AeroInfluencerControlConfig`: rotates an influencer in response to control commands.
#[derive(Deserialize, Debug, Clone)]
#[serde(default)]
pub struct ActuationConfig {
    pub enable_control: bool,
    pub axis_configs: Vec<ActuationAxis>,
    pub use_separate_minmax: bool,
    #[serde(deserialize_with = "v3")]
    pub max_value: Vec3,
    #[serde(deserialize_with = "v3")]
    pub min_value: Vec3,
    pub limit_movement_speed: bool,
    pub movement_speed: f32,
    #[serde(skip)]
    pub current_value: Vec3,
}

impl Default for ActuationConfig {
    fn default() -> Self {
        Self {
            enable_control: true,
            axis_configs: Vec::new(),
            use_separate_minmax: false,
            max_value: Vec3::ZERO,
            min_value: Vec3::ZERO,
            limit_movement_speed: false,
            movement_speed: 1.0,
            current_value: Vec3::ZERO,
        }
    }
}

impl ActuationConfig {
    /// Effective minimum limit: Godot forces `min = -max` unless the separate min/max flag is set.
    pub fn min(&self) -> Vec3 {
        if self.use_separate_minmax { self.min_value } else { -self.max_value }
    }

    pub fn compile(&mut self) -> Result<(), String> {
        self.axis_configs.iter_mut().try_for_each(|a| a.compile())
    }

    /// `AeroInfluencerControlConfig.update(aero_influencer, delta)`.
    ///
    /// `relative_position` is the influencer's position relative to the body's center of mass,
    /// `body_basis` the body's global basis.
    pub fn update(&mut self, relative_position: Vec3, body_basis: Mat3, control: &dyn ControlSource, delta: f32) -> Vec3 {
        if !self.enable_control {
            return self.current_value;
        }
        let local_rel = vec_times_basis(relative_position, body_basis);
        let axis_sign = Vec3::new(sign(local_rel.x), sign(local_rel.y), sign(local_rel.z));

        let mut total = Vec3::ZERO;
        for axis in &self.axis_configs {
            let command = control.control_command(&axis.axis_name);
            total += axis.value(command, control, axis_sign);
        }
        total = total.clamp(-Vec3::ONE, Vec3::ONE);

        let min = self.min();
        let max = self.max_value;
        let mut desired = total * crate::godot::max_magnitude(min, max);
        desired = desired.clamp(min, max);

        self.current_value = if self.limit_movement_speed {
            vec_move_toward(self.current_value, desired, self.movement_speed * delta)
        } else {
            desired
        };
        self.current_value
    }
}
