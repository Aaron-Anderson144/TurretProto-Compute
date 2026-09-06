//! Shooter-units front door: a JSON-friendly request in imperial units, a
//! drop/drift table out. The CLI and the wasm wrapper are both thin shells
//! over [`solve`], so the unit handling lives in exactly one place.

use core::fmt;

use serde::{Deserialize, Serialize};

use crate::atmosphere::{isa_tp, Atmosphere, Station};
use crate::coriolis::Coriolis;
use crate::drag::G7;
use crate::solver::{Projectile, Shot, Solver, Wind};
use crate::units::*;

/// Which standard atmosphere the published BC was referenced to.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum BcReference {
    #[default]
    Icao,
    Asm,
}

/// Everything about one table, in the units a shooter types. Missing fields
/// take the defaults below.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TableRequest {
    /// G7 ballistic coefficient, lb/in².
    pub bc: f64,
    pub bc_reference: BcReference,
    pub weight_gr: f64,
    pub muzzle_velocity_fps: f64,
    pub sight_height_in: f64,
    pub zero_range_yd: f64,
    pub max_range_yd: f64,
    pub step_yd: f64,
    pub wind_mph: f64,
    /// Direction the wind blows FROM, o'clock. 12 head, 3 from the right, 6 tail, 9 from the left.
    pub wind_clock: f64,
    pub temperature_f: f64,
    /// Absolute station pressure, inHg. Not the sea-level-corrected number.
    pub pressure_inhg: Option<f64>,
    /// Used to derive station pressure from ISA when `pressure_inhg` is absent.
    pub altitude_ft: Option<f64>,
    pub humidity_pct: f64,
    /// Turns on Coriolis and latitude-corrected gravity.
    pub latitude_deg: Option<f64>,
    /// Firing azimuth, degrees clockwise from true north.
    pub azimuth_deg: f64,
    /// + uphill.
    pub look_angle_deg: f64,
    pub dt_ms: f64,
}

impl Default for TableRequest {
    fn default() -> TableRequest {
        TableRequest {
            bc: 0.0,
            bc_reference: BcReference::Icao,
            weight_gr: 0.0,
            muzzle_velocity_fps: 0.0,
            sight_height_in: 2.0,
            zero_range_yd: 100.0,
            max_range_yd: 1500.0,
            step_yd: 100.0,
            wind_mph: 0.0,
            wind_clock: 3.0,
            temperature_f: 59.0,
            pressure_inhg: None,
            altitude_ft: None,
            humidity_pct: 0.0,
            latitude_deg: None,
            azimuth_deg: 0.0,
            look_angle_deg: 0.0,
            dt_ms: 1.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TableRow {
    pub range_yd: f64,
    /// + above line of sight.
    pub path_in: f64,
    pub path_moa: f64,
    pub path_mil: f64,
    /// + right.
    pub drift_in: f64,
    pub drift_moa: f64,
    pub drift_mil: f64,
    pub velocity_fps: f64,
    pub mach: f64,
    pub energy_ftlb: f64,
    pub time_s: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TableResult {
    pub zero_angle_moa: f64,
    pub zero_angle_mil: f64,
    pub station_pressure_inhg: f64,
    pub air_density_kg_m3: f64,
    pub speed_of_sound_fps: f64,
    pub rows: Vec<TableRow>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
    Invalid(&'static str),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Invalid(msg) => write!(f, "invalid input: {msg}"),
        }
    }
}

impl std::error::Error for Error {}

pub fn solve(req: &TableRequest) -> Result<TableResult, Error> {
    if !(req.bc > 0.0) {
        return Err(Error::Invalid("bc must be > 0"));
    }
    if !(req.weight_gr > 0.0) {
        return Err(Error::Invalid("weight_gr must be > 0"));
    }
    if !(req.muzzle_velocity_fps > 0.0) {
        return Err(Error::Invalid("muzzle_velocity_fps must be > 0"));
    }
    if !(req.zero_range_yd > 0.0) {
        return Err(Error::Invalid("zero_range_yd must be > 0"));
    }
    if !(req.step_yd > 0.0) {
        return Err(Error::Invalid("step_yd must be > 0"));
    }
    if !(req.max_range_yd >= 0.0) {
        return Err(Error::Invalid("max_range_yd must be >= 0"));
    }
    if !(req.temperature_f > -459.67) {
        return Err(Error::Invalid("temperature_f is below absolute zero"));
    }
    if !(req.dt_ms > 0.0) {
        return Err(Error::Invalid("dt_ms must be > 0"));
    }

    let mass = grains_to_kg(req.weight_gr);
    let projectile = match req.bc_reference {
        BcReference::Icao => Projectile::new(req.bc, mass),
        BcReference::Asm => Projectile::from_asm_bc(req.bc, mass),
    };

    let pressure = match (req.pressure_inhg, req.altitude_ft) {
        (Some(p), _) => inhg_to_pa(p),
        (None, Some(alt)) => isa_tp(ft_to_m(alt)).1,
        (None, None) => inhg_to_pa(29.92),
    };
    if !(pressure > 0.0) {
        return Err(Error::Invalid("pressure must be > 0"));
    }
    let atmosphere = Station {
        temperature: f_to_k(req.temperature_f),
        pressure,
        humidity: req.humidity_pct / 100.0,
    };

    let coriolis = req.latitude_deg.map(|lat| Coriolis {
        latitude: lat.to_radians(),
        azimuth: req.azimuth_deg.to_radians(),
    });

    let shot = Shot {
        muzzle_velocity: fps_to_mps(req.muzzle_velocity_fps),
        sight_height: in_to_m(req.sight_height_in),
        zero_range: yd_to_m(req.zero_range_yd),
        look_angle: req.look_angle_deg.to_radians(),
        wind: Wind::from_clock(mph_to_mps(req.wind_mph), req.wind_clock),
        coriolis,
        dt: req.dt_ms * 1e-3,
    };

    let solver = Solver::new(projectile, G7, atmosphere);
    let zero = solver.zero_angle(&shot);
    let air = atmosphere.at(0.0);

    let rows = solver
        .flight_with_zero(&shot, zero, yd_to_m(req.step_yd), yd_to_m(req.max_range_yd))
        .map(|p| TableRow {
            range_yd: m_to_yd(p.range),
            path_in: m_to_in(p.path),
            path_moa: p.path_moa(),
            path_mil: p.path_mil(),
            drift_in: m_to_in(p.drift),
            drift_moa: p.drift_moa(),
            drift_mil: p.drift_mil(),
            velocity_fps: mps_to_fps(p.speed),
            mach: p.mach,
            energy_ftlb: j_to_ftlbf(p.energy),
            time_s: p.time,
        })
        .collect();

    Ok(TableResult {
        zero_angle_moa: rad_to_moa(zero),
        zero_angle_mil: rad_to_mil(zero),
        station_pressure_inhg: pa_to_inhg(pressure),
        air_density_kg_m3: air.density,
        speed_of_sound_fps: mps_to_fps(air.speed_of_sound),
        rows,
    })
}
