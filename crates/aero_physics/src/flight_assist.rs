//! Port of `core/flight_assist.gd`: rate tracking, limiters, and autopilot holds.

use bevy::math::{Mat3, Vec3};
use serde::Deserialize;

use crate::godot::{remap, vec_times_basis};
use crate::pid::Pid;
use crate::schema::v3;

/// Flight assist configuration and state. Field names and defaults match the GDScript exports.
#[derive(Deserialize, Debug, Clone)]
#[serde(default)]
pub struct FlightAssist {
    pub enable_flight_assist: bool,
    #[serde(deserialize_with = "v3")]
    pub max_angular_rates: Vec3,
    pub enable_flight_assist_x: bool,
    pub pitch_assist_pid: Pid,
    pub enable_flight_assist_y: bool,
    pub yaw_assist_pid: Pid,
    pub enable_flight_assist_z: bool,
    pub roll_assist_pid: Pid,
    pub enable_g_limiter: bool,
    pub g_limit: f32,
    pub negative_g_limit: f32,
    pub enable_aoa_limiter: bool,
    pub aoa_limit_start: f32,
    pub aoa_limit_end: f32,
    pub enable_control_adjustment: bool,
    pub tuned_airspeed: f32,
    pub min_accounted_airspeed: f32,
    pub tuned_density: f32,
    pub min_accounted_air_density: f32,
    pub control_rotation: f32,
    pub enable_bank_angle_assist: bool,
    pub bank_angle_target: f32,
    pub bank_angle_pid: Pid,
    pub enable_heading_hold: bool,
    pub heading_target: f32,
    pub heading_pid: Pid,
    pub enable_inclination_hold: bool,
    pub inclination_target: f32,
    pub inclination_pid: Pid,
    pub enable_speed_hold: bool,
    pub speed_target: f32,
    pub speed_pid: Pid,
    pub enable_altitude_hold: bool,
    pub altitude_target: f32,
    pub altitude_pid: Pid,
    pub enable_target_direction: bool,
    pub use_velocity_vector_for_targetting: bool,
    pub disable_yaw_on_immelmann: bool,
    #[serde(deserialize_with = "v3")]
    pub direction_target: Vec3,
    pub direction_pitch_pid: Pid,
    pub direction_yaw_pid: Pid,
    pub direction_roll_pid: Pid,

    // Inputs, refreshed by the control system every physics tick.
    #[serde(skip)]
    pub air_speed: f32,
    #[serde(skip)]
    pub air_density: f32,
    #[serde(skip)]
    pub angle_of_attack: f32,
    #[serde(skip)]
    pub bank_angle: f32,
    #[serde(skip)]
    pub heading: f32,
    #[serde(skip)]
    pub inclination: f32,
    #[serde(skip)]
    pub altitude: f32,
    #[serde(skip)]
    pub linear_velocity: Vec3,
    #[serde(skip)]
    pub local_angular_velocity: Vec3,
    #[serde(skip)]
    pub global_basis: Mat3,
    // Outputs.
    #[serde(skip)]
    pub control_input: Vec3,
    #[serde(skip)]
    pub control_command: Vec3,
    #[serde(skip)]
    pub throttle_command: f32,
    #[serde(skip)]
    pub angular_rate_error: Vec3,
}

impl Default for FlightAssist {
    fn default() -> Self {
        Self {
            enable_flight_assist: true,
            max_angular_rates: Vec3::new(2.0, 1.0, 5.0),
            enable_flight_assist_x: true,
            pitch_assist_pid: Pid::clamped(0.5, 3.0, 0.0, -0.2, 0.2),
            enable_flight_assist_y: true,
            yaw_assist_pid: Pid::new(1.0, 0.0, 0.0),
            enable_flight_assist_z: true,
            roll_assist_pid: Pid::new(0.2, 0.0, 0.0),
            enable_g_limiter: true,
            g_limit: 9.0,
            negative_g_limit: 3.0,
            enable_aoa_limiter: true,
            aoa_limit_start: 22.0,
            aoa_limit_end: 25.0,
            enable_control_adjustment: true,
            tuned_airspeed: 100.0,
            min_accounted_airspeed: 75.0,
            tuned_density: 1.225,
            min_accounted_air_density: 0.1,
            control_rotation: 0.0,
            enable_bank_angle_assist: false,
            bank_angle_target: 0.0,
            bank_angle_pid: Pid::new(1.0, 0.05, 0.1),
            enable_heading_hold: false,
            heading_target: 0.0,
            heading_pid: Pid::new(10.0, 0.0, 0.0),
            enable_inclination_hold: false,
            inclination_target: 0.0,
            inclination_pid: Pid::new(10.0, 0.0, 0.0),
            enable_speed_hold: false,
            speed_target: 0.0,
            speed_pid: Pid::new(0.0, 0.4, 0.0),
            enable_altitude_hold: false,
            altitude_target: 0.0,
            altitude_pid: Pid::new(0.001, 0.0, 0.01),
            enable_target_direction: false,
            use_velocity_vector_for_targetting: false,
            disable_yaw_on_immelmann: true,
            direction_target: Vec3::ZERO,
            direction_pitch_pid: Pid::new(10.0, 0.3, 4.0),
            direction_yaw_pid: Pid::new(25.0, 0.0, 0.0),
            direction_roll_pid: Pid::new(0.8, 0.0, 0.0),
            air_speed: 0.0,
            air_density: 1.0,
            angle_of_attack: 0.0,
            bank_angle: 0.0,
            heading: 0.0,
            inclination: 0.0,
            altitude: 0.0,
            linear_velocity: Vec3::ZERO,
            local_angular_velocity: Vec3::ZERO,
            global_basis: Mat3::IDENTITY,
            control_input: Vec3::ZERO,
            control_command: Vec3::ZERO,
            throttle_command: 0.0,
            angular_rate_error: Vec3::ZERO,
        }
    }
}

const GRAVITY: f32 = 9.8;

impl FlightAssist {
    /// `FlightAssist.update(delta)`.
    pub fn update(&mut self, delta: f32) {
        if !self.enable_flight_assist {
            self.control_command = self.control_input;
            return;
        }
        self.bank_angle_hold(delta);
        self.heading_hold(delta);
        self.inclination_hold(delta);
        self.altitude_hold(delta);
        self.speed_hold(delta);
        self.target_direction(delta);
        self.flight_assist(delta);
    }

    fn bank_angle_hold(&mut self, delta: f32) {
        if !self.enable_bank_angle_assist && !self.enable_altitude_hold && !self.enable_heading_hold {
            return;
        }
        let out = self.bank_angle_pid.update(delta, self.bank_angle_target - self.bank_angle);
        self.control_input.z += out;
    }

    fn heading_hold(&mut self, delta: f32) {
        if !self.enable_heading_hold {
            return;
        }
        let out = self.heading_pid.update(delta, self.heading_target - self.heading);
        self.control_input.y += out;
    }

    fn inclination_hold(&mut self, delta: f32) {
        if !self.enable_inclination_hold {
            return;
        }
        let out = self.inclination_pid.update(delta, self.inclination_target - self.inclination);
        self.control_input.x += out;
    }

    fn speed_hold(&mut self, delta: f32) {
        if !self.enable_speed_hold {
            return;
        }
        self.throttle_command = self.speed_pid.update(delta, self.speed_target - self.air_speed);
    }

    fn altitude_hold(&mut self, delta: f32) {
        if !self.enable_altitude_hold {
            return;
        }
        let out = self.altitude_pid.update(delta, self.altitude_target - self.altitude);
        self.control_input.x += out;
    }

    /// Euler-like angles of a local direction, as used by the direction-target autopilot.
    fn direction_angles(local: Vec3) -> Vec3 {
        Vec3::new(local.y.atan2(-local.z), (-local.x).atan2(-local.z), (-local.x).atan2(local.y))
    }

    fn target_direction(&mut self, delta: f32) {
        if !self.enable_target_direction {
            return;
        }
        let basis = self.global_basis;
        let local_direction_target = vec_times_basis(self.direction_target, basis);
        let angles_to_target = Self::direction_angles(local_direction_target);
        let mut error = angles_to_target;

        let local_velocity_direction = vec_times_basis(self.linear_velocity.normalize_or_zero(), basis);
        if self.use_velocity_vector_for_targetting {
            if self.linear_velocity.length() < 1e-5 {
                return;
            }
            error = angles_to_target - Self::direction_angles(local_velocity_direction);
        }

        // Yaw is undesirable when the target is behind us; it causes oscillation.
        if self.disable_yaw_on_immelmann && error.x.abs() >= 90.0_f32.to_radians() {
            error.y = 0.0;
        }

        // Roll so that the felt G-forces line up with the body's up axis.
        let speed = self.linear_velocity.length();
        let local_desired_acceleration = local_direction_target * speed - local_velocity_direction * speed
            + vec_times_basis(Vec3::new(0.0, GRAVITY, 0.0), basis);
        error.z = -local_desired_acceleration.x;

        let pitch = self.direction_pitch_pid.update(delta, error.x);
        let yaw = self.direction_yaw_pid.update(delta, error.y);
        let roll = self.direction_roll_pid.update(delta, error.z);
        self.control_input.x += pitch;
        self.control_input.y += yaw;
        self.control_input.z += roll;
        self.control_input = self.control_input.clamp(Vec3::splat(-1.0), Vec3::ONE);
    }

    /// The rate-tracking core, including the G and AoA limiters.
    fn flight_assist(&mut self, delta: f32) {
        let mut angular_rates = self.max_angular_rates;

        if self.enable_g_limiter {
            let mut g_limited_rate = self.max_angular_rates.x;
            if self.air_speed.abs() >= 1e-5 {
                // Gravity projected onto the body's up axis.
                let zero_effort_g = self.global_basis.y_axis.dot(Vec3::NEG_Y);
                g_limited_rate = (self.g_limit + zero_effort_g) * 9.81 / self.air_speed;
                if self.control_input.x <= 0.0 {
                    g_limited_rate = (self.negative_g_limit - zero_effort_g) * 9.81 / self.air_speed;
                }
            }
            angular_rates.x = angular_rates.x.min(g_limited_rate);
        }

        self.angular_rate_error = angular_rates * self.control_input - self.local_angular_velocity;

        // Keep the commands already written by target_direction and the holds.
        self.control_command = self.control_input;
        if self.enable_flight_assist_x {
            self.control_command.x = self.pitch_assist_pid.update(delta, self.angular_rate_error.x);
        }
        if self.enable_flight_assist_y {
            self.control_command.y = self.yaw_assist_pid.update(delta, self.angular_rate_error.y);
        }
        if self.enable_flight_assist_z {
            self.control_command.z = self.roll_assist_pid.update(delta, self.angular_rate_error.z);
        }

        if self.enable_control_adjustment {
            self.control_command *= get_control_adjustment_factor(
                self.air_speed,
                self.air_density,
                self.tuned_airspeed,
                self.tuned_density,
                self.min_accounted_airspeed,
                self.min_accounted_air_density,
            );
        }

        self.control_command = self.control_command.clamp(Vec3::splat(-1.0), Vec3::ONE);

        if self.enable_aoa_limiter {
            let scale = remap(
                self.angle_of_attack.abs(),
                self.aoa_limit_end.to_radians(),
                self.aoa_limit_start.to_radians(),
                0.0,
                1.0,
            )
            .clamp(0.0, 1.0);
            self.control_command *= scale;
        }
    }
}

/// `FlightAssist.get_control_adjustment_factor()`: keeps control response constant across speed and density.
pub fn get_control_adjustment_factor(speed: f32, density: f32, tuned_speed: f32, tuned_density: f32, min_speed: f32, min_density: f32) -> f32 {
    let mut adjustment = 1.0;
    let accounted_speed = min_speed.max(speed);
    adjustment /= (accounted_speed * accounted_speed) / (tuned_speed * tuned_speed);
    let accounted_density = min_density.max(density);
    adjustment /= accounted_density / tuned_density;
    adjustment
}
