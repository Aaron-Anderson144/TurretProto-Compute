//! Checks that need no reference data: closed forms, sign conventions, convergence, sanity.

use approx::assert_abs_diff_eq;
use ballistics_core::atmosphere::{air, G0};
use ballistics_core::solver::ASM_TO_ICAO_BC;
use ballistics_core::units::*;
use ballistics_core::*;

fn bullet() -> Projectile {
    Projectile::new(0.42, grains_to_kg(300.0))
}

fn base_shot() -> Shot {
    Shot {
        muzzle_velocity: 800.0,
        sight_height: 0.05,
        zero_range: 100.0,
        ..Shot::default()
    }
}

/// Point at `range` m (stations 0 and `range`; the last one is the far one).
fn at<D: DragModel, A: Atmosphere>(solver: &Solver<D, A>, shot: &Shot, range: f64) -> Point {
    let p = solver.flight(shot, range, range).last().unwrap();
    assert!((p.range - range).abs() < 1e-6, "bullet never reached {range} m");
    p
}

#[test]
fn vacuum_matches_closed_form() {
    // RK4 is exact for constant acceleration, so this is tight.
    let solver = Solver::new(bullet(), NoDrag, Isa { station_altitude: 0.0 });
    let theta = 0.02_f64;
    let v = 800.0_f64;
    let shot = Shot {
        muzzle_velocity: v,
        sight_height: 0.0,
        ..Shot::default()
    };
    let mut n = 0;
    for p in solver.flight_with_zero(&shot, theta, 100.0, 1500.0) {
        let x = p.range;
        let y = x * theta.tan() - G0 * x * x / (2.0 * v * v * theta.cos().powi(2));
        assert_abs_diff_eq!(p.path, y, epsilon = 1e-7);
        assert_abs_diff_eq!(p.time, x / (v * theta.cos()), epsilon = 1e-9);
        assert_abs_diff_eq!(p.drift, 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(p.velocity.x, v * theta.cos(), epsilon = 1e-9);
        n += 1;
    }
    assert_eq!(n, 16);
}

#[test]
fn zero_puts_path_on_the_line_of_sight() {
    let solver = Solver::new(bullet(), G7, Isa { station_altitude: 0.0 });
    let shot = Shot {
        muzzle_velocity: fps_to_mps(2650.0),
        sight_height: in_to_m(2.0),
        zero_range: yd_to_m(100.0),
        ..Shot::default()
    };
    let z = solver.zero_angle(&shot);
    assert!(z > 0.0 && z < 0.01, "zero angle {z} rad");
    assert_abs_diff_eq!(solver.path_at(&shot, z, shot.zero_range), 0.0, epsilon = 1e-6);

    // Station 0 sits one sight height below the LOS, at muzzle velocity, t = 0.
    let first = solver.flight(&shot, 50.0, 100.0).next().unwrap();
    assert_abs_diff_eq!(first.path, -shot.sight_height, epsilon = 1e-12);
    assert_abs_diff_eq!(first.speed, shot.muzzle_velocity, epsilon = 1e-12);
    assert_eq!(first.time, 0.0);
    assert_eq!(first.path_moa(), 0.0);
}

#[test]
fn stations_land_where_asked() {
    let solver = Solver::new(bullet(), G7, Isa { station_altitude: 0.0 });
    let pts = solver.table(&base_shot(), 37.5, 1000.0);
    assert_eq!(pts.len(), 27); // 0, 37.5, …, 975
    for (i, p) in pts.iter().enumerate() {
        assert_abs_diff_eq!(p.range, 37.5 * i as f64, epsilon = 1e-6);
    }
    // Time and drop increase monotonically, speed decreases.
    for w in pts.windows(2) {
        assert!(w[1].time > w[0].time);
        assert!(w[1].speed < w[0].speed);
    }
    let far: Vec<_> = pts.iter().filter(|p| p.range > 150.0).collect();
    for w in far.windows(2) {
        assert!(w[1].path < w[0].path);
    }
}

#[test]
fn coriolis_signs() {
    let solver = Solver::new(bullet(), G7, Isa { station_altitude: 0.0 });
    let base = base_shot();
    let calm = at(&solver, &base, 1000.0);
    assert_abs_diff_eq!(calm.drift, 0.0, epsilon = 1e-12);

    // Northern hemisphere: horizontal Coriolis deflects right on every azimuth.
    for az in [0.0_f64, 90.0, 180.0, 270.0] {
        let shot = Shot {
            coriolis: Some(Coriolis {
                latitude: 40.0_f64.to_radians(),
                azimuth: az.to_radians(),
            }),
            ..base
        };
        let p = at(&solver, &shot, 1000.0);
        assert!(p.drift > 0.02, "az {az}: drift {} m should be right", p.drift);
    }
    // Southern hemisphere: left.
    let south = Shot {
        coriolis: Some(Coriolis {
            latitude: -40.0_f64.to_radians(),
            azimuth: 0.0,
        }),
        ..base
    };
    assert!(at(&solver, &south, 1000.0).drift < -0.02);

    // Eötvös: firing east strikes higher than firing west.
    let east = Shot {
        coriolis: Some(Coriolis {
            latitude: 40.0_f64.to_radians(),
            azimuth: 90.0_f64.to_radians(),
        }),
        ..base
    };
    let west = Shot {
        coriolis: Some(Coriolis {
            latitude: 40.0_f64.to_radians(),
            azimuth: 270.0_f64.to_radians(),
        }),
        ..base
    };
    let (e, w) = (at(&solver, &east, 1000.0), at(&solver, &west, 1000.0));
    assert!(e.path > w.path + 0.02, "east {} vs west {}", e.path, w.path);
    // Same latitude on both, so the horizontal deflection matches (to the
    // second-order coupling through the slightly different flight paths).
    assert_abs_diff_eq!(e.drift, w.drift, epsilon = 1e-5);
}

#[test]
fn wind_signs() {
    let solver = Solver::new(bullet(), G7, Isa { station_altitude: 0.0 });
    let base = base_shot();
    let calm = at(&solver, &base, 1000.0);
    let wind = mph_to_mps(10.0);

    let from_right = at(&solver, &Shot { wind: Wind::from_clock(wind, 3.0), ..base }, 1000.0);
    assert!(from_right.drift < -0.3, "from the right should push left: {}", from_right.drift);
    let from_left = at(&solver, &Shot { wind: Wind::from_clock(wind, 9.0), ..base }, 1000.0);
    assert_abs_diff_eq!(from_left.drift, -from_right.drift, epsilon = 1e-9);

    let head = at(&solver, &Shot { wind: Wind::from_clock(wind, 12.0), ..base }, 1000.0);
    let tail = at(&solver, &Shot { wind: Wind::from_clock(wind, 6.0), ..base }, 1000.0);
    assert!(head.speed < calm.speed && calm.speed < tail.speed);
    assert!(head.path < calm.path && calm.path < tail.path);
    assert_abs_diff_eq!(head.drift, 0.0, epsilon = 1e-9);

    // Clock and degrees agree.
    assert_eq!(Wind::from_clock(wind, 3.0), Wind::from_direction(wind, 90.0));
}

#[test]
fn look_angle_reduces_drop_uphill_and_downhill() {
    let solver = Solver::new(bullet(), G7, Isa { station_altitude: 0.0 });
    let base = base_shot();
    let level = at(&solver, &base, 800.0);
    let up = at(&solver, &Shot { look_angle: 30.0_f64.to_radians(), ..base }, 800.0);
    let down = at(&solver, &Shot { look_angle: -30.0_f64.to_radians(), ..base }, 800.0);
    assert!(up.path > level.path && down.path > level.path);
    // At 30° the gravity component across the LOS is cos 30°; rough rifleman's rule check.
    let ratio = (up.path + base.sight_height) / (level.path + base.sight_height);
    assert!((0.80..0.93).contains(&ratio), "ratio {ratio}");
}

#[test]
fn rk4_is_converged_at_1ms() {
    let solver = Solver::new(bullet(), G7, Isa { station_altitude: 0.0 });
    let base = Shot {
        wind: Wind::from_clock(4.0, 2.0),
        coriolis: Some(Coriolis {
            latitude: 40.0_f64.to_radians(),
            azimuth: 45.0_f64.to_radians(),
        }),
        ..base_shot()
    };
    let coarse = at(&solver, &Shot { dt: 1e-3, ..base }, 1200.0);
    let fine = at(&solver, &Shot { dt: 2.5e-4, ..base }, 1200.0);
    assert_abs_diff_eq!(coarse.path, fine.path, epsilon = 1e-4);
    assert_abs_diff_eq!(coarse.drift, fine.drift, epsilon = 1e-4);
    assert_abs_diff_eq!(coarse.speed, fine.speed, epsilon = 1e-2);
    assert_abs_diff_eq!(coarse.time, fine.time, epsilon = 1e-5);
}

#[test]
fn sanity_338_class_load() {
    // 300 gr, G7 0.42, 2650 fps, 100 yd zero — loose bounds, just catching unit blunders.
    let solver = Solver::new(bullet(), G7, Isa { station_altitude: 0.0 });
    let shot = Shot {
        muzzle_velocity: fps_to_mps(2650.0),
        sight_height: in_to_m(2.0),
        zero_range: yd_to_m(100.0),
        ..Shot::default()
    };
    let muzzle = solver.flight(&shot, yd_to_m(1000.0), 0.0).next().unwrap();
    assert!((4600.0..4750.0).contains(&j_to_ftlbf(muzzle.energy)));

    let p = at(&solver, &shot, yd_to_m(1000.0));
    let v = mps_to_fps(p.speed);
    let drop_in = m_to_in(p.path);
    assert!((1500.0..2000.0).contains(&v), "1000 yd velocity {v} fps");
    assert!((-400.0..-200.0).contains(&drop_in), "1000 yd path {drop_in} in");
    assert!((1.2..1.7).contains(&p.time), "1000 yd tof {} s", p.time);
    assert!(p.mach > 1.3 && p.mach < 1.8);
}

#[test]
fn atmosphere_reference_values() {
    let sl = Isa { station_altitude: 0.0 }.at(0.0);
    assert_abs_diff_eq!(sl.density, 1.2250, epsilon = 2e-4);
    assert_abs_diff_eq!(sl.speed_of_sound, 340.29, epsilon = 0.05);

    let km = Isa { station_altitude: 1000.0 }.at(0.0);
    assert_abs_diff_eq!(km.density, 1.1117, epsilon = 5e-4);
    assert_abs_diff_eq!(km.speed_of_sound, 336.43, epsilon = 0.1);

    // Height above the launch point is the same as station altitude.
    let a = Isa { station_altitude: 300.0 }.at(200.0);
    let b = Isa { station_altitude: 500.0 }.at(0.0);
    assert_eq!(a, b);

    // A station fed ISA conditions reproduces ISA up and down the column.
    let st = Station {
        temperature: 288.15,
        pressure: 101_325.0,
        humidity: 0.0,
    };
    for h in [-100.0, 0.0, 250.0, 800.0] {
        let x = st.at(h);
        let y = Isa { station_altitude: 0.0 }.at(h);
        assert_abs_diff_eq!(x.density, y.density, epsilon = 1e-9);
        assert_abs_diff_eq!(x.speed_of_sound, y.speed_of_sound, epsilon = 1e-9);
    }

    // Humid air is lighter and slightly faster.
    let dry = air(303.15, 101_325.0, 0.0);
    let wet = air(303.15, 101_325.0, 1.0);
    assert!(wet.density < dry.density - 0.01);
    assert!(wet.speed_of_sound > dry.speed_of_sound);
}

#[test]
fn g7_table_anchors_and_clamping() {
    assert_eq!(G7.cd(0.0), 0.1198);
    assert_eq!(G7.cd(-1.0), 0.1198);
    assert_eq!(G7.cd(0.5), 0.1194);
    assert_eq!(G7.cd(1.0), 0.3803);
    assert_eq!(G7.cd(2.0), 0.2980);
    assert_eq!(G7.cd(3.0), 0.2424);
    assert_eq!(G7.cd(5.0), 0.1618);
    assert_eq!(G7.cd(9.0), 0.1618);
    assert_abs_diff_eq!(G7.cd(1.0125), 0.5 * (0.3803 + 0.4015), epsilon = 1e-12);
    // Table is sorted by Mach.
    for w in ballistics_core::drag::G7_TABLE.windows(2) {
        assert!(w[1].0 > w[0].0);
    }
}

#[test]
fn asm_bc_converts_down_to_icao() {
    let p = Projectile::from_asm_bc(0.5, 0.02);
    assert_abs_diff_eq!(ASM_TO_ICAO_BC, 0.9820, epsilon = 1e-4);
    assert_abs_diff_eq!(p.bc, 0.4910, epsilon = 1e-4);
    assert_abs_diff_eq!(Projectile::new(1.0, 0.02).bc_si(), 703.07, epsilon = 0.01);
}

#[test]
fn table_request_round_trip() {
    use ballistics_core::table::{solve, TableRequest};
    let req = TableRequest {
        bc: 0.42,
        weight_gr: 300.0,
        muzzle_velocity_fps: 2650.0,
        wind_mph: 10.0,
        latitude_deg: Some(40.0),
        max_range_yd: 1000.0,
        ..TableRequest::default()
    };
    let out = solve(&req).unwrap();
    assert_eq!(out.rows.len(), 11);
    assert_abs_diff_eq!(out.rows[1].path_in, 0.0, epsilon = 1e-4); // 100 yd zero
    assert!(out.rows[10].drift_in < -20.0); // 10 mph from the right
    assert!(out.zero_angle_moa > 1.0 && out.zero_angle_moa < 6.0);

    // JSON in, JSON out, nothing lost.
    let json = serde_json::to_string(&req).unwrap();
    let back: TableRequest = serde_json::from_str(&json).unwrap();
    assert_eq!(back, req);
    let sparse: TableRequest =
        serde_json::from_str(r#"{"bc":0.42,"weight_gr":300,"muzzle_velocity_fps":2650}"#).unwrap();
    assert_eq!(sparse.zero_range_yd, 100.0);
    assert!(solve(&TableRequest::default()).is_err());
}
