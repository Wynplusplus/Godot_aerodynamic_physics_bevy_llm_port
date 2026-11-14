//! Port of `singletons/aero_units.gd`: International Standard Atmosphere lookups.

use bevy::prelude::Resource;

use crate::curve::Curve;

/// `(altitude m, temperature K, pressure Pa, density kg/m^3)`, from http://www.braeunig.us/space/atmos.htm.
const ISA_TABLE: [(f32, f32, f32, f32); 12] = [
    (0.0, 288.0, 101325.0, 1.225),
    (2000.0, 275.0, 79495.2, 1.0065),
    (5000.0, 255.0, 54019.9, 0.7361),
    (10000.0, 223.0, 26436.3, 0.4127),
    (15000.0, 216.0, 12044.6, 0.1937),
    (20000.0, 216.0, 5474.8, 0.08803),
    (30000.0, 226.0, 1171.8, 0.01803),
    (40000.0, 251.0, 277.5, 0.003851),
    (50000.0, 270.0, 75.9, 0.0009775),
    (60000.0, 245.0, 20.3, 0.0002883),
    (70000.0, 217.0, 4.6, 0.00007424),
    (80000.0, 196.0, 0.89, 0.0000157),
];

/// Air properties as a function of altitude. Insert this resource to enable altitude-dependent air.
#[derive(Resource, Debug, Clone)]
pub struct AeroUnits {
    pub ratio_of_specific_heat: f32,
    pub temperature: Curve,
    pub pressure: Curve,
    pub density: Curve,
    pub mach: Curve,
}

impl Default for AeroUnits {
    fn default() -> Self {
        Self::new(1.402)
    }
}

impl AeroUnits {
    /// Builds the lookup curves from the ISA table. `ratio_of_specific_heat` is 1.402 for air.
    pub fn new(ratio_of_specific_heat: f32) -> Self {
        let min_alt = ISA_TABLE[0].0;
        let max_alt = ISA_TABLE[ISA_TABLE.len() - 1].0;
        let build = |values: Vec<(f32, f32)>| {
            let (lo, hi) = values.iter().fold((f32::MAX, f32::MIN), |(lo, hi), &(_, v)| (lo.min(v), hi.max(v)));
            Curve::from_points(&values, min_alt, max_alt, lo, hi, 100)
        };
        let temperature = build(ISA_TABLE.iter().map(|r| (r.0, r.1)).collect());
        let pressure = build(ISA_TABLE.iter().map(|r| (r.0, r.2)).collect());
        let density = build(ISA_TABLE.iter().map(|r| (r.0, r.3)).collect());
        let mach_values: Vec<(f32, f32)> = ISA_TABLE
            .iter()
            .map(|r| (r.0, speed_of_sound(ratio_of_specific_heat, r.2, r.3)))
            .collect();
        let mach = build(mach_values);
        Self { ratio_of_specific_heat, temperature, pressure, density, mach }
    }

    pub fn temperature_at_altitude(&self, altitude: f32) -> f32 {
        self.temperature.sample_baked(altitude)
    }

    pub fn pressure_at_altitude(&self, altitude: f32) -> f32 {
        self.pressure.sample_baked(altitude)
    }

    pub fn density_at_altitude(&self, altitude: f32) -> f32 {
        self.density.sample_baked(altitude)
    }

    /// Speed of sound at the altitude, in m/s.
    pub fn mach_at_altitude(&self, altitude: f32) -> f32 {
        self.mach.sample_baked(altitude)
    }

    /// Mach number for a true airspeed at an altitude.
    pub fn speed_to_mach_at_altitude(&self, speed: f32, altitude: f32) -> f32 {
        speed / self.mach_at_altitude(altitude)
    }

    /// The altitude of a world position. The original plugin uses `global_position.y`.
    pub fn altitude_of(position_y: f32) -> f32 {
        position_y
    }
}

/// `get_speed_of_sound_at_pressure_and_density()`.
pub fn speed_of_sound(ratio_of_specific_heat: f32, pressure: f32, density: f32) -> f32 {
    (ratio_of_specific_heat * (pressure / density)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sea_level_values() {
        let units = AeroUnits::default();
        assert!((units.density_at_altitude(0.0) - 1.225).abs() < 1e-3);
        assert!((units.mach_at_altitude(0.0) - 340.3).abs() < 0.5);
        assert!((units.speed_to_mach_at_altitude(340.3, 0.0) - 1.0).abs() < 1e-3);
    }
}
