//! Port of `utils/pid.gd` (`aero_PID`).

use serde::Deserialize;

/// PID controller with optional integral clamping. Defaults match the GDScript exports.
#[derive(Deserialize, Debug, Clone, PartialEq)]
#[serde(default)]
pub struct Pid {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub clamp_integral: bool,
    pub min_integral: f32,
    pub max_integral: f32,
    #[serde(skip)]
    pub output: f32,
    #[serde(skip)]
    last_error: f32,
    #[serde(skip)]
    integral_error: f32,
}

impl Default for Pid {
    fn default() -> Self {
        Self::new(1.0, 0.2, 0.0)
    }
}

impl Pid {
    pub fn new(p: f32, i: f32, d: f32) -> Self {
        Self { p, i, d, clamp_integral: false, min_integral: -1.0, max_integral: 1.0, output: 0.0, last_error: 0.0, integral_error: 0.0 }
    }

    /// Equivalent of `aero_PID.new(p, i, d, clamp, min, max)`.
    pub fn clamped(p: f32, i: f32, d: f32, min_integral: f32, max_integral: f32) -> Self {
        Self { clamp_integral: true, min_integral, max_integral, ..Self::new(p, i, d) }
    }

    /// Runs one step with the given error and returns the new output.
    pub fn update(&mut self, delta: f32, error: f32) -> f32 {
        let derivative = (error - self.last_error) / delta;
        self.integral_error += error * delta;
        if self.clamp_integral {
            self.integral_error = self.integral_error.clamp(self.min_integral, self.max_integral);
        }
        self.last_error = error;
        self.output = self.p * error + self.i * self.integral_error + self.d * derivative;
        self.output
    }
}
