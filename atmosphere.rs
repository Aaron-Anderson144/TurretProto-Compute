//! Air density and speed of sound: ISA, or measured station conditions with humidity.

use crate::math;

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Air {
    /// kg/m³
    pub density: f64,
    /// m/s
    pub speed_of_sound: f64,
}

/// Air as a function of height above the launch point (m).
pub trait Atmosphere {
    fn at(&self, height: f64) -> Air;
}

pub const R_DRY: f64 = 287.058; // J/(kg·K)
pub const R_VAPOR: f64 = 461.495; // J/(kg·K)
pub const GAMMA: f64 = 1.4;
/// ISA tropospheric lapse rate, K/m.
pub const LAPSE: f64 = 0.0065;
/// Standard gravity, m/s².
pub const G0: f64 = 9.80665;
pub const ISA_T0: f64 = 288.15; // K
pub const ISA_P0: f64 = 101_325.0; // Pa
/// ICAO sea-level density, kg/m³.
pub const ISA_RHO0: f64 = 1.225;
/// Army Standard Metro density (59 °F, 29.53 inHg, 78 % RH), kg/m³.
pub const ASM_RHO: f64 = 1.2030;

/// Saturation vapour pressure over water, Pa (Alduchov & Eskridge 1996).
pub fn saturation_vapor_pressure(temperature: f64) -> f64 {
    let tc = temperature - 273.15;
    610.94 * math::exp(17.625 * tc / (tc + 243.04))
}

/// Moist-air density and speed of sound from temperature (K), absolute
/// pressure (Pa) and relative humidity (0..1).
pub fn air(temperature: f64, pressure: f64, relative_humidity: f64) -> Air {
    let rh = relative_humidity.clamp(0.0, 1.0);
    let e = (rh * saturation_vapor_pressure(temperature)).min(pressure);
    let pd = pressure - e;
    let density = pd / (R_DRY * temperature) + e / (R_VAPOR * temperature);
    let speed_of_sound = math::sqrt(GAMMA * pressure / density);
    Air {
        density,
        speed_of_sound,
    }
}

/// ISA temperature (K) and pressure (Pa) at geometric altitude (m).
/// Troposphere model — good to 11 km, which is all a rifle needs.
pub fn isa_tp(altitude: f64) -> (f64, f64) {
    let t = ISA_T0 - LAPSE * altitude;
    let p = ISA_P0 * math::pow(t / ISA_T0, G0 / (R_DRY * LAPSE));
    (t, p)
}

/// Sea-level-corrected pressure (what a weather report gives) to absolute
/// station pressure at `altitude` m, using the ISA lapse.
pub fn sea_level_to_station(sea_level_pressure: f64, altitude: f64) -> f64 {
    let (t, _) = isa_tp(altitude);
    sea_level_pressure * math::pow(t / ISA_T0, G0 / (R_DRY * LAPSE))
}

/// ICAO standard atmosphere, dry. `station_altitude` is the launch point's
/// altitude ASL (m); heights passed to `at` are relative to it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Isa {
    pub station_altitude: f64,
}

impl Atmosphere for Isa {
    fn at(&self, height: f64) -> Air {
        let (t, p) = isa_tp(self.station_altitude + height);
        air(t, p, 0.0)
    }
}

/// Measured conditions at the launch point (Kestrel-style). Above/below the
/// launch point, temperature and pressure follow the ISA lapse from the
/// measured values; humidity is held constant.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Station {
    /// K
    pub temperature: f64,
    /// Absolute (station) pressure, Pa. Not sea-level corrected.
    pub pressure: f64,
    /// 0..1
    pub humidity: f64,
}

impl Atmosphere for Station {
    fn at(&self, height: f64) -> Air {
        let t = self.temperature - LAPSE * height;
        let p = self.pressure * math::pow(t / self.temperature, G0 / (R_DRY * LAPSE));
        air(t, p, self.humidity)
    }
}
