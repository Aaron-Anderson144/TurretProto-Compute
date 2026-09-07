//! `bal` — drop/drift table from the command line.

use ballistics_core::table::{solve, BcReference, TableRequest, TableResult};
use clap::Parser;

#[derive(Parser, Debug)]
#[command(
    name = "bal",
    version,
    about = "Exterior ballistics table: point mass, RK4, G7 drag, atmosphere, wind, Coriolis",
    after_help = "Signs: path is + above the line of sight, drift is + right. Dial the opposite."
)]
struct Args {
    /// G7 ballistic coefficient, lb/in² (ICAO reference unless --asm)
    #[arg(long)]
    bc: f64,
    /// Bullet weight, grains
    #[arg(long)]
    weight: f64,
    /// Muzzle velocity, fps
    #[arg(long)]
    mv: f64,
    /// Sight height above bore, inches
    #[arg(long, default_value_t = 2.0)]
    sight_height: f64,
    /// Zero range, yards
    #[arg(long, default_value_t = 100.0)]
    zero: f64,
    /// Max range, yards
    #[arg(long, default_value_t = 1500.0)]
    range: f64,
    /// Table step, yards
    #[arg(long, default_value_t = 100.0)]
    step: f64,
    /// Wind speed, mph
    #[arg(long, default_value_t = 0.0)]
    wind: f64,
    /// Direction the wind blows FROM, o'clock (12 head, 3 from the right, 6 tail, 9 from the left)
    #[arg(long, default_value_t = 3.0)]
    wind_clock: f64,
    /// Air temperature, °F
    #[arg(long, default_value_t = 59.0)]
    temp: f64,
    /// Station (absolute) pressure, inHg — not the sea-level-corrected weather-report number
    #[arg(long)]
    pressure: Option<f64>,
    /// Altitude, ft. Derives station pressure from ISA when --pressure is not given
    #[arg(long)]
    altitude: Option<f64>,
    /// Relative humidity, %
    #[arg(long, default_value_t = 0.0)]
    humidity: f64,
    /// Latitude, degrees (+N). Turns on Coriolis; pair with --azimuth
    #[arg(long)]
    lat: Option<f64>,
    /// Firing azimuth, degrees clockwise from true north
    #[arg(long, default_value_t = 0.0)]
    azimuth: f64,
    /// Look angle to the target, degrees (+ uphill)
    #[arg(long, default_value_t = 0.0)]
    look: f64,
    /// The BC is referenced to Army Standard Metro instead of ICAO
    #[arg(long)]
    asm: bool,
    /// Integrator step, ms
    #[arg(long, default_value_t = 1.0)]
    dt: f64,
    /// Emit JSON instead of a table
    #[arg(long)]
    json: bool,
}

fn main() {
    let a = Args::parse();
    let req = TableRequest {
        bc: a.bc,
        bc_reference: if a.asm { BcReference::Asm } else { BcReference::Icao },
        weight_gr: a.weight,
        muzzle_velocity_fps: a.mv,
        sight_height_in: a.sight_height,
        zero_range_yd: a.zero,
        max_range_yd: a.range,
        step_yd: a.step,
        wind_mph: a.wind,
        wind_clock: a.wind_clock,
        temperature_f: a.temp,
        pressure_inhg: a.pressure,
        altitude_ft: a.altitude,
        humidity_pct: a.humidity,
        latitude_deg: a.lat,
        azimuth_deg: a.azimuth,
        look_angle_deg: a.look,
        dt_ms: a.dt,
    };

    let out = match solve(&req) {
        Ok(out) => out,
        Err(e) => {
            eprintln!("bal: {e}");
            std::process::exit(2);
        }
    };

    if a.json {
        println!("{}", serde_json::to_string_pretty(&out).unwrap());
    } else {
        print_table(&req, &out);
    }
}

fn print_table(req: &TableRequest, out: &TableResult) {
    println!(
        "BC {:.3} G7 ({}), {:.0} gr, MV {:.0} fps | zero {:.0} yd, sight {:.2} in -> zero angle {:.2} MOA / {:.2} mil",
        req.bc,
        match req.bc_reference {
            BcReference::Icao => "ICAO",
            BcReference::Asm => "ASM->ICAO",
        },
        req.weight_gr,
        req.muzzle_velocity_fps,
        req.zero_range_yd,
        req.sight_height_in,
        out.zero_angle_moa,
        out.zero_angle_mil
    );
    println!(
        "Air {:.1} F, {:.2} inHg station, {:.0}% RH -> rho {:.4} kg/m3, c {:.0} fps",
        req.temperature_f, out.station_pressure_inhg, req.humidity_pct, out.air_density_kg_m3, out.speed_of_sound_fps
    );
    let mut cond = format!("Wind {:.1} mph from {:.0} o'clock", req.wind_mph, req.wind_clock);
    match req.latitude_deg {
        Some(lat) => cond += &format!(" | Coriolis lat {lat:.1}, az {:.0}", req.azimuth_deg),
        None => cond += " | no Coriolis",
    }
    cond += &format!(" | look {:.1} deg", req.look_angle_deg);
    println!("{cond}");
    println!();
    println!(
        "{:>6} {:>8} {:>7} {:>7} {:>8} {:>7} {:>7} {:>6} {:>6} {:>7} {:>6}",
        "Range", "Path", "MOA", "mil", "Drift", "MOA", "mil", "Vel", "Mach", "Energy", "TOF"
    );
    println!(
        "{:>6} {:>8} {:>7} {:>7} {:>8} {:>7} {:>7} {:>6} {:>6} {:>7} {:>6}",
        "(yd)", "(in)", "", "", "(in)", "", "", "(fps)", "", "(ft-lb)", "(s)"
    );
    for r in &out.rows {
        println!(
            "{:>6.0} {:>8.1} {:>7.2} {:>7.2} {:>8.1} {:>7.2} {:>7.2} {:>6.0} {:>6.2} {:>7.0} {:>6.3}",
            r.range_yd,
            nz(r.path_in, 1),
            nz(r.path_moa, 2),
            nz(r.path_mil, 2),
            nz(r.drift_in, 1),
            nz(r.drift_moa, 2),
            nz(r.drift_mil, 2),
            r.velocity_fps,
            r.mach,
            r.energy_ftlb,
            r.time_s
        );
    }
}

/// Round to `decimals` and turn "-0.0" into "0.0".
fn nz(x: f64, decimals: i32) -> f64 {
    let s = 10f64.powi(decimals);
    let r = (x * s).round() / s;
    if r == 0.0 {
        0.0
    } else {
        r
    }
}
