//! What a fire control loop calls, and what it costs.
//!
//! `cargo run --example aim_demo -p ballistics-core`

use ballistics_core::aim::{aim, aim_hinted, aim_with_rates, AimParams, Conditions, Enu, Gun, Target};
use ballistics_core::units::*;
use ballistics_core::{Projectile, Solver, Station, G7};

const MIL: f64 = 1000.0;

fn main() {
    let solver = Solver::new(
        Projectile::new(0.42, grains_to_kg(300.0)),
        G7,
        Station {
            temperature: f_to_k(59.0),
            pressure: inhg_to_pa(29.92),
            humidity: 0.30,
        },
    );
    let gun = Gun {
        muzzle_velocity: fps_to_mps(2650.0),
        sight_height: in_to_m(2.5),
    };
    let params = AimParams::default();

    // 8 mph from the west, Licking County latitude.
    let conditions = Conditions::wind_from(mph_to_mps(8.0), 270f64.to_radians())
        .at_latitude(40.1f64.to_radians());

    println!(
        ".338 NM, G7 0.42, 300 gr, 2650 fps. 8 mph from 270, lat 40.1 N.\n\
         Gun bearing 000 to a target due north. Angles in mils.\n"
    );
    println!(
        "{:>7} {:>9} {:>8} {:>8} {:>8} {:>7} {:>6} {:>6} {:>5}",
        "range", "target", "super", "drift", "lead az", "TOF", "impact", "Mach", "cost"
    );
    println!(
        "{:>7} {:>9} {:>8} {:>8} {:>8} {:>7} {:>6} {:>6} {:>5}",
        "(m)", "", "(mil)", "(mil)", "(mil)", "(s)", "(m/s)", "", ""
    );

    for &(range, label, vel) in &[
        (300.0, "parked", Enu::ZERO),
        (600.0, "parked", Enu::ZERO),
        (900.0, "parked", Enu::ZERO),
        (1200.0, "parked", Enu::ZERO),
        (600.0, "10 m/s E", Enu::new(10.0, 0.0, 0.0)),
        (900.0, "10 m/s E", Enu::new(10.0, 0.0, 0.0)),
        (900.0, "30 m/s E", Enu::new(30.0, 0.0, 0.0)),
        (900.0, "30 m/s in", Enu::new(0.0, -30.0, 0.0)),
    ] {
        let target = Target {
            position: Enu::new(0.0, range, 0.0),
            velocity: vel,
            acceleration: Enu::ZERO,
        };
        let s = aim(&solver, &gun, &target, &conditions, &params);
        println!(
            "{:>7.0} {:>9} {:>8.2} {:>8.2} {:>8.2} {:>7.3} {:>6.0} {:>6.2} {:>5}",
            range,
            label,
            s.superelevation * MIL,
            s.drift_correction * MIL,
            (s.lead_bearing - s.drift_correction) * MIL,
            s.tof,
            s.impact_speed,
            s.impact_mach,
            s.trajectories,
        );
    }

    // What it costs to hold a track, which is the number that decides whether
    // this fits in the loop budget on the Orin.
    println!("\nTracking a 30 m/s crosser at 900 m, 50 Hz, warm started:");
    let mut target = Target {
        position: Enu::new(0.0, 900.0, 0.0),
        velocity: Enu::new(30.0, 0.0, 0.0),
        acceleration: Enu::ZERO,
    };
    let dt = 0.02;
    let mut last = aim(&solver, &gun, &target, &conditions, &params);
    println!("  first (cold) : {} integrations", last.trajectories);

    let mut total = 0u32;
    let mut worst = 0u32;
    for _ in 0..50 {
        target.position = target.at(dt);
        last = aim_hinted(&solver, &gun, &target, &conditions, &params, Some(&last));
        total += last.trajectories;
        worst = worst.max(last.trajectories);
    }
    println!(
        "  next 50 frames: {:.1} integrations/frame average, {worst} worst case",
        total as f64 / 50.0
    );

    // And the feed-forward rates the cascade controller wants.
    let (s, rates) = aim_with_rates(&solver, &gun, &target, &conditions, &params);
    println!(
        "\nFeed forward at the last frame: az {:.1} mil/s, el {:.2} mil/s (bearing {:.1} deg, elevation {:.2} deg)",
        rates.bearing * MIL,
        rates.elevation * MIL,
        s.bearing.to_degrees(),
        s.elevation.to_degrees(),
    );
}
