//! Earth rotation: Coriolis acceleration and latitude-dependent gravity.

use crate::math;
use crate::vec3::Vec3;

/// Earth's rotation rate, rad/s.
pub const EARTH_RATE: f64 = 7.292_115e-5;

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Coriolis {
    /// rad, + north
    pub latitude: f64,
    /// Firing azimuth, rad clockwise from true north.
    pub azimuth: f64,
}

impl Coriolis {
    /// Earth's rotation vector expressed in the shot frame (x downrange, y up, z right).
    pub fn omega(&self) -> Vec3 {
        let (sl, cl) = (math::sin(self.latitude), math::cos(self.latitude));
        let (sa, ca) = (math::sin(self.azimuth), math::cos(self.azimuth));
        Vec3::new(EARTH_RATE * cl * ca, EARTH_RATE * sl, -EARTH_RATE * cl * sa)
    }

    /// Coriolis acceleration `-2 Ω × v` for ground velocity `v`.
    /// Horizontal part deflects right in the northern hemisphere; the vertical
    /// (Eötvös) part lifts eastward shots and drops westward ones.
    #[inline]
    pub fn acceleration(&self, v: Vec3) -> Vec3 {
        -2.0 * self.omega().cross(v)
    }

    /// Normal gravity at this latitude, m/s².
    pub fn gravity(&self) -> f64 {
        normal_gravity(self.latitude)
    }
}

/// WGS-84 normal gravity at sea level (International Gravity Formula 1980 series), m/s².
pub fn normal_gravity(latitude: f64) -> f64 {
    let s = math::sin(latitude);
    let s2 = math::sin(2.0 * latitude);
    9.780_327 * (1.0 + 0.005_302_4 * s * s - 0.000_005_8 * s2 * s2)
}
