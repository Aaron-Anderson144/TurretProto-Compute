//! Aiming solver checks. Closed forms, sign conventions, and the geometry the
//! fire control loop is going to lean on.

use approx::assert_relative_eq;
use ballistics_core::aim::{
    aim, aim_hinted, aim_with_rates, wrap_2pi, wrap_pi, AimParams, Conditions, Enu, Gun, Status,
    Target,
};
use ballistics_core::units::*;
use ballistics_core::{Projectile, Shot, Solver, Station, Wind, G7};

use std::f64::consts::PI;

fn solver() -> Solver<G7, Station> {
    Solver::new(
        Projectile::new(0.42, grains_to_kg(300.0)),
        G7,
        Station {
            temperature: f_to_k(59.0),
            pressure: inhg_to_pa(29.92),
            humidity: 0.0,
        },
    )
}

fn gun() -> Gun {
    Gun {
        muzzle_velocity: fps_to_mps(2650.0),
        sight_height: in_to_m(2.5),
    }
}

fn north(range: f64) -> Target {
    Target::stationary(Enu::new(0.0, range, 0.0))
}

// --------------------------------------------------------------------------
// frame
// --------------------------------------------------------------------------

#[test]
fn enu_spherical_round_trip() {
    for &b_deg in &[0.0, 45.0, 90.0, 179.0, 225.0, 359.0] {
        for &e_deg in &[-30.0, -5.0, 0.0, 12.0, 60.0] {
            let (b, e, r) = (b_deg * PI / 180.0, e_deg * PI / 180.0, 1234.5);
            let v = Enu::from_spherical(b, e, r);
            assert_relative_eq!(v.range(), r, epsilon = 1e-9);
            assert_relative_eq!(v.bearing(), wrap_2pi(b), epsilon = 1e-9);
            assert_relative_eq!(v.elevation(), e, epsilon = 1e-9);
        }
    }
}

#[test]
fn enu_axes_are_where_they_say() {
    // Due north, level.
    let n = Enu::from_spherical(0.0, 0.0, 100.0);
    assert_relative_eq!(n.north, 100.0, epsilon = 1e-9);
    assert_relative_eq!(n.east, 0.0, epsilon = 1e-9);

    // 090 is east.
    let e = Enu::from_spherical(90f64.to_radians(), 0.0, 100.0);
    assert_relative_eq!(e.east, 100.0, epsilon = 1e-9);
    assert_relative_eq!(e.north, 0.0, epsilon = 1e-9);

    // Straight up.
    let u = Enu::from_spherical(0.0, 90f64.to_radians(), 100.0);
    assert_relative_eq!(u.up, 100.0, epsilon = 1e-9);
}

#[test]
fn angle_wrapping() {
    assert_relative_eq!(wrap_2pi(-0.5), 2.0 * PI - 0.5, epsilon = 1e-12);
    assert_relative_eq!(wrap_2pi(0.5), 0.5, epsilon = 1e-12);
    assert_relative_eq!(wrap_pi(2.0 * PI - 0.5), -0.5, epsilon = 1e-12);
    assert_relative_eq!(wrap_pi(0.5), 0.5, epsilon = 1e-12);
}

// --------------------------------------------------------------------------
// superelevation
// --------------------------------------------------------------------------

#[test]
fn superelevation_matches_the_zero_angle_for_the_same_range() {
    // A stationary target at R, level, no wind, is the same problem as zeroing
    // the rifle at R. Two completely different root finders should land in the
    // same place.
    let s = solver();
    let g = gun();
    let p = AimParams::default();

    for &range in &[100.0, 500.0, 1000.0, 1500.0] {
        let sol = aim(&s, &g, &north(range), &Conditions::CALM, &p);
        assert!(sol.status.is_converged(), "{range} m: {:?}", sol.status);

        let shot = Shot {
            muzzle_velocity: g.muzzle_velocity,
            sight_height: g.sight_height,
            zero_range: range,
            look_angle: 0.0,
            wind: Wind::CALM,
            coriolis: None,
            dt: p.dt,
        };
        let za = s.zero_angle(&shot);
        assert_relative_eq!(sol.superelevation, za, epsilon = 5e-6);
    }
}

#[test]
fn the_solution_actually_puts_the_bullet_on_the_target() {
    // Fly the answer back through the forward solver and check the miss.
    let s = solver();
    let g = gun();
    let p = AimParams::default();

    for &(range, elev_deg) in &[(400.0, 0.0), (900.0, 0.0), (700.0, 25.0), (600.0, -20.0)] {
        let tgt = Target::stationary(Enu::from_spherical(0.0, elev_deg * PI / 180.0, range));
        let sol = aim(&s, &g, &tgt, &Conditions::CALM, &p);
        assert!(sol.status.is_converged(), "{range} m {elev_deg} deg");

        let shot = Shot {
            muzzle_velocity: g.muzzle_velocity,
            sight_height: g.sight_height,
            zero_range: range,
            look_angle: sol.intercept.elevation(),
            wind: Wind::CALM,
            coriolis: None,
            dt: p.dt,
        };
        let pt = ballistics_core::Flight::new(
            &s,
            &shot,
            sol.superelevation,
            sol.range,
            f64::INFINITY,
            sol.range,
        )
        .next()
        .expect("bullet reaches the target");

        // Miss under the tolerance the solver was asked for.
        assert!(
            pt.path.abs() <= p.elevation_tol * sol.range * 1.5,
            "{range} m {elev_deg} deg: missed by {} m",
            pt.path
        );
        assert_relative_eq!(pt.time, sol.tof, epsilon = 1e-6);
    }
}

#[test]
fn depression_gives_a_negative_gun_elevation() {
    let s = solver();
    let g = gun();
    let look = (-20f64).to_radians();
    let tgt = Target::stationary(Enu::from_spherical(0.0, look, 600.0));
    let sol = aim(&s, &g, &tgt, &Conditions::CALM, &AimParams::default());

    assert!(sol.status.is_converged());
    assert!(sol.elevation < 0.0, "gun should be depressed");
    // Still above the line of sight: gravity does not care which way you look.
    assert!(sol.superelevation > 0.0);
    assert_relative_eq!(sol.elevation, look + sol.superelevation, epsilon = 1e-12);
}

#[test]
fn superelevation_grows_with_range() {
    let s = solver();
    let g = gun();
    let p = AimParams::default();
    let mut last = f64::NEG_INFINITY;
    for &r in &[200.0, 400.0, 800.0, 1200.0, 1600.0] {
        let sol = aim(&s, &g, &north(r), &Conditions::CALM, &p);
        assert!(sol.status.is_converged(), "{r} m");
        assert!(sol.superelevation > last, "{r} m did not need more elevation");
        last = sol.superelevation;
    }
}

#[test]
fn out_of_reach_is_reported_not_guessed() {
    let s = solver();
    let sol = aim(
        &s,
        &gun(),
        &north(50_000.0),
        &Conditions::CALM,
        &AimParams::default(),
    );
    assert_eq!(sol.status, Status::Unreachable);
    assert!(!sol.status.is_usable());
    // And it works that out cheaply instead of burning the whole budget.
    assert!(sol.trajectories <= 4, "spent {} shots", sol.trajectories);
}

// --------------------------------------------------------------------------
// lead
// --------------------------------------------------------------------------

#[test]
fn crossing_target_leads_the_right_way_by_the_right_amount() {
    let s = solver();
    let g = gun();
    let p = AimParams::default();
    let v = 15.0; // m/s due east

    let tgt = Target {
        position: Enu::new(0.0, 800.0, 0.0),
        velocity: Enu::new(v, 0.0, 0.0),
        acceleration: Enu::ZERO,
    };
    let sol = aim(&s, &g, &tgt, &Conditions::CALM, &p);
    assert!(sol.status.is_converged(), "{:?}", sol.status);

    // The intercept is where the target will be at t = tof, to within the
    // fixed point's own tolerance. tof_tol is a time, so the position residual
    // is that times how fast the target is going.
    let residual = (sol.intercept - tgt.at(sol.tof)).range();
    assert!(
        residual <= 2.0 * p.tof_tol * v,
        "fixed point left {residual} m on the table"
    );
    assert_relative_eq!(sol.intercept.north, 800.0, epsilon = 1e-9);

    // Lead is to the right (bearing increases) and is exactly the geometry to
    // the point that was solved for.
    assert!(sol.lead_bearing > 0.0);
    assert_relative_eq!(
        sol.lead_bearing,
        (sol.intercept.east / sol.intercept.north).atan(),
        epsilon = 1e-12
    );

    // Calm and no Coriolis, so nothing else moved the bearing.
    assert_relative_eq!(sol.drift_correction, 0.0, epsilon = 1e-12);

    // Sanity on the size of it: about 20 mrad for this shot.
    assert!(sol.lead_bearing > 0.010 && sol.lead_bearing < 0.035);
}

#[test]
fn crossing_the_other_way_leads_the_other_way() {
    let s = solver();
    let p = AimParams::default();
    let tgt = Target {
        position: Enu::new(0.0, 800.0, 0.0),
        velocity: Enu::new(-15.0, 0.0, 0.0),
        acceleration: Enu::ZERO,
    };
    let sol = aim(&s, &gun(), &tgt, &Conditions::CALM, &p);
    assert!(sol.status.is_converged());
    assert!(sol.lead_bearing < 0.0);
}

#[test]
fn closer_target_means_shorter_flight() {
    let s = solver();
    let g = gun();
    let p = AimParams::default();

    let stat = aim(&s, &g, &north(800.0), &Conditions::CALM, &p);
    let closing = aim(
        &s,
        &g,
        &Target {
            position: Enu::new(0.0, 800.0, 0.0),
            velocity: Enu::new(0.0, -50.0, 0.0), // 50 m/s straight at the mount
            acceleration: Enu::ZERO,
        },
        &Conditions::CALM,
        &p,
    );
    assert!(closing.status.is_converged());
    assert!(closing.range < stat.range);
    assert!(closing.tof < stat.tof);
    assert!(closing.superelevation < stat.superelevation);
    // Straight in, so no bearing lead.
    assert_relative_eq!(closing.lead_bearing, 0.0, epsilon = 1e-12);
}

#[test]
fn a_climbing_target_needs_more_elevation() {
    let s = solver();
    let p = AimParams::default();
    let climbing = aim(
        &s,
        &gun(),
        &Target {
            position: Enu::new(0.0, 800.0, 0.0),
            velocity: Enu::new(0.0, 0.0, 40.0),
            acceleration: Enu::ZERO,
        },
        &Conditions::CALM,
        &p,
    );
    assert!(climbing.status.is_converged());
    assert!(climbing.intercept.up > 0.0);
    assert!(climbing.lead_elevation > 0.0);
    assert!(climbing.elevation > 0.0);
}

#[test]
fn stationary_target_needs_no_lead() {
    let s = solver();
    let sol = aim(
        &s,
        &gun(),
        &north(700.0),
        &Conditions::CALM,
        &AimParams::default(),
    );
    assert!(sol.status.is_converged());
    assert_relative_eq!(sol.lead_bearing, 0.0, epsilon = 1e-12);
    // All of the vertical offset is drop compensation.
    assert_relative_eq!(sol.lead_elevation, sol.superelevation, epsilon = 1e-12);
    assert_relative_eq!(sol.range, 700.0, epsilon = 1e-9);
}

// --------------------------------------------------------------------------
// drift
// --------------------------------------------------------------------------

#[test]
fn wind_from_the_left_is_held_off_to_the_left() {
    // Firing north. Wind from the west blows the bullet east, which is right of
    // the line of sight, so the gun goes left of the target.
    let s = solver();
    let cond = Conditions::wind_from(mph_to_mps(15.0), 270f64.to_radians());
    let sol = aim(
        &s,
        &gun(),
        &north(1000.0),
        &cond,
        &AimParams::default(),
    );
    assert!(sol.status.is_converged());
    assert!(sol.drift_correction < 0.0, "should hold left");
    assert!(sol.lead_bearing < 0.0);
    // Wrapped, so a left aim-off shows up just under 2 pi.
    assert!(sol.bearing > PI);
}

#[test]
fn wind_from_the_right_is_held_off_to_the_right() {
    let s = solver();
    let cond = Conditions::wind_from(mph_to_mps(15.0), 90f64.to_radians());
    let sol = aim(&s, &gun(), &north(1000.0), &cond, &AimParams::default());
    assert!(sol.status.is_converged());
    assert!(sol.drift_correction > 0.0, "should hold right");
}

#[test]
fn a_pure_headwind_barely_moves_the_bearing() {
    let s = solver();
    // Firing north, wind from the north.
    let cond = Conditions::wind_from(mph_to_mps(15.0), 0.0);
    let sol = aim(&s, &gun(), &north(1000.0), &cond, &AimParams::default());
    assert!(sol.status.is_converged());
    assert!(sol.drift_correction.abs() < 1e-6);
}

#[test]
fn wind_decomposition_follows_the_firing_azimuth() {
    // The same physical wind has to give the same hold whichever way the gun is
    // pointing. Wind from the west firing north is a pure left-to-right
    // crosswind; wind from the north firing east is the same crosswind.
    let s = solver();
    let g = gun();
    let p = AimParams::default();
    let speed = mph_to_mps(12.0);

    let firing_north = aim(
        &s,
        &g,
        &Target::stationary(Enu::from_spherical(0.0, 0.0, 900.0)),
        &Conditions::wind_from(speed, 270f64.to_radians()),
        &p,
    );
    let firing_east = aim(
        &s,
        &g,
        &Target::stationary(Enu::from_spherical(90f64.to_radians(), 0.0, 900.0)),
        &Conditions::wind_from(speed, 0.0),
        &p,
    );

    assert!(firing_north.status.is_converged());
    assert!(firing_east.status.is_converged());
    assert_relative_eq!(
        firing_north.drift_correction,
        firing_east.drift_correction,
        epsilon = 1e-9
    );
}

#[test]
fn coriolis_holds_left_in_the_northern_hemisphere() {
    let s = solver();
    let cond = Conditions::CALM.at_latitude(40f64.to_radians());
    let sol = aim(&s, &gun(), &north(1500.0), &cond, &AimParams::default());
    assert!(sol.status.is_converged());
    // Bullet deflects right up north, so hold left.
    assert!(sol.drift_correction < 0.0);
}

#[test]
fn coriolis_holds_right_in_the_southern_hemisphere() {
    let s = solver();
    let cond = Conditions::CALM.at_latitude((-40f64).to_radians());
    let sol = aim(&s, &gun(), &north(1500.0), &cond, &AimParams::default());
    assert!(sol.status.is_converged());
    assert!(sol.drift_correction > 0.0);
}

// --------------------------------------------------------------------------
// cost and rates
// --------------------------------------------------------------------------

#[test]
fn a_warm_start_is_cheaper_than_a_cold_one() {
    let s = solver();
    let g = gun();
    let p = AimParams::default();
    let tgt = Target {
        position: Enu::new(0.0, 900.0, 0.0),
        velocity: Enu::new(12.0, -8.0, 0.0),
        acceleration: Enu::ZERO,
    };

    let cold = aim(&s, &g, &tgt, &Conditions::CALM, &p);
    assert!(cold.status.is_converged());

    let warm = aim_hinted(&s, &g, &tgt, &Conditions::CALM, &p, Some(&cold));
    assert!(warm.status.is_converged());

    assert!(
        warm.trajectories < cold.trajectories,
        "warm {} vs cold {}",
        warm.trajectories,
        cold.trajectories
    );
    assert_relative_eq!(warm.bearing, cold.bearing, epsilon = 1e-6);
    assert_relative_eq!(warm.elevation, cold.elevation, epsilon = 1e-6);
}

#[test]
fn a_cold_solve_stays_inside_a_sane_budget() {
    let s = solver();
    let sol = aim(
        &s,
        &gun(),
        &Target {
            position: Enu::new(0.0, 1200.0, 0.0),
            velocity: Enu::new(20.0, 0.0, 5.0),
            acceleration: Enu::ZERO,
        },
        &Conditions::CALM,
        &AimParams::default(),
    );
    assert!(sol.status.is_converged());
    assert!(sol.lead_iterations <= 4, "{} lead passes", sol.lead_iterations);
    assert!(sol.trajectories <= 20, "{} integrations", sol.trajectories);
}

#[test]
fn rates_track_a_crosser() {
    let s = solver();
    let g = gun();
    let p = AimParams::default();
    let v = 15.0;
    let tgt = Target {
        position: Enu::new(0.0, 800.0, 0.0),
        velocity: Enu::new(v, 0.0, 0.0),
        acceleration: Enu::ZERO,
    };
    let (sol, rates) = aim_with_rates(&s, &g, &tgt, &Conditions::CALM, &p);
    assert!(sol.status.is_converged());

    // Line of sight rate for a crosser is about v/R. The gun rate is that plus
    // the (small) rate of change of the lead, so it should be in the same
    // neighbourhood and the same sign.
    let los_rate = v / 800.0;
    assert!(rates.bearing > 0.0);
    assert!(
        (rates.bearing - los_rate).abs() < 0.5 * los_rate,
        "{} vs los {}",
        rates.bearing,
        los_rate
    );
}

#[test]
fn a_parked_target_needs_no_slew() {
    let s = solver();
    let (sol, rates) = aim_with_rates(
        &s,
        &gun(),
        &north(700.0),
        &Conditions::CALM,
        &AimParams::default(),
    );
    assert!(sol.status.is_converged());
    assert!(rates.bearing.abs() < 1e-3);
    assert!(rates.elevation.abs() < 1e-3);
}

#[test]
fn rates_are_zero_when_there_is_no_solution() {
    let s = solver();
    let (sol, rates) = aim_with_rates(
        &s,
        &gun(),
        &north(50_000.0),
        &Conditions::CALM,
        &AimParams::default(),
    );
    assert_eq!(sol.status, Status::Unreachable);
    assert_eq!(rates.bearing, 0.0);
    assert_eq!(rates.elevation, 0.0);
}
