//! Port of `core/aero_atmosphere_3d/*`: air volumes that change wind and density.
//!
//! An [`AeroAtmosphere`] is a sensor volume. Bodies that overlap it are listed in
//! `AeroBody::atmospheres`, and per-influencer volumes (such as water) are also checked at each
//! influencer's position.

use bevy::math::Vec3;
use bevy::prelude::{Component, Entity};
use serde::Deserialize;

use crate::schema::v3;

/// The kind of volume. Water overrides the surface height and uses per-influencer positioning.
#[derive(Deserialize, Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AtmosphereKind {
    #[default]
    Air,
    /// `AeroWaterAtmosphere3D`: flat water, with the surface at the volume's origin height.
    Water,
}

/// Settings of an atmosphere volume (`AeroAtmosphere3D` exports).
#[derive(Component, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct AeroAtmosphere {
    pub kind: AtmosphereKind,
    pub per_influencer_positioning: bool,
    #[serde(deserialize_with = "v3")]
    pub wind: Vec3,
    pub override_wind: bool,
    pub density: f32,
    pub override_density: bool,
    pub temperature: f32,
    pub override_temperature: bool,
}

impl Default for AeroAtmosphere {
    fn default() -> Self {
        Self {
            kind: AtmosphereKind::Air,
            per_influencer_positioning: false,
            wind: Vec3::ZERO,
            override_wind: false,
            density: 1.225,
            override_density: false,
            temperature: 288.0,
            override_temperature: false,
        }
    }
}

impl AeroAtmosphere {
    /// `AeroWaterAtmosphere3D._init()` defaults.
    pub fn water() -> Self {
        Self { kind: AtmosphereKind::Water, per_influencer_positioning: true, override_density: true, density: 1000.0, override_wind: true, ..Default::default() }
    }
}

/// A snapshot of an atmosphere's state for one physics step. Taken before the force loop so the
/// force code does not need to query the ECS.
#[derive(Debug, Clone)]
pub struct AtmosphereSnapshot {
    pub entity: Entity,
    pub position: Vec3,
    pub settings: AeroAtmosphere,
}

impl AtmosphereSnapshot {
    /// Signed distance from the surface: negative when under water. Air has no surface (always 0).
    pub fn distance_to_surface(&self, p: Vec3) -> f32 {
        match self.settings.kind {
            AtmosphereKind::Air => 0.0,
            AtmosphereKind::Water => p.y - self.position.y - 0.0,
        }
    }

    pub fn surface_normal(&self) -> Vec3 {
        Vec3::Y
    }

    pub fn is_inside(&self, p: Vec3) -> bool {
        self.distance_to_surface(p) < 0.0
    }

    pub fn wind_at(&self) -> Vec3 {
        self.settings.wind
    }

    pub fn density_at(&self, p: Vec3) -> f32 {
        match self.settings.kind {
            AtmosphereKind::Air => self.settings.density,
            AtmosphereKind::Water => {
                if self.distance_to_surface(p) <= 0.0 {
                    self.settings.density
                } else {
                    1.225
                }
            }
        }
    }
}
