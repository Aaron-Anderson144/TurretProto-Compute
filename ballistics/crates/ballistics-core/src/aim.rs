//! Aiming: the inverse of [`crate::solver`].
//!
//! `solver` and `table` answer "where does the bullet go". This answers "where
//! do I point", which is what a fire control loop needs and what a drop table
//! cannot give it.
//!
//! Hand it a target's position and velocity relative to the mount and it hands
//! back gun bearing, gun elevation and time of flight. No zero, no yards, no
//! table. It allocates nothing and does not need std, so the same call works on
//! the Jetson and on the MCU.
//!
//! # Frames
//!
//! Everything on this module's boundary is [`Enu`]: east, north, up, metres,
//! measured from the sight. That is the frame a tracker already works in. The
//! shot frame the solver uses (x downrange, y up, z right) never leaves this
//! module.
//!
//! Bearings are radians clockwise from true north. Elevations are radians above
//! horizontal, + up. Solution bearings come back wrapped to [0, 2pi); lead
//! angles are signed and wrapped to (-pi, pi].
//!
//! # What it does
//!
//! ```text
//! guess time of flight
//!   -> where the target will be then                         (lead)
//!   -> bearing, elevation and slant range to there
//!   -> bore elevation that drops the bullet onto that point   (superelevation)
//!   -> time of flight to it, go round again
//! ```
//!
//! Two or three passes converge for anything short of a target moving a real
//! fraction of the bullet's speed. Warm start it from the last frame's solution
//! with [`aim_hinted`] and it is usually one pass.
//!
//! The inner superelevation solve is a safeguarded secant, not a bisection.
//! Every evaluation is a full trajectory integration, so bisection would cost
//! forty of them where the secant costs three. The safeguard is a bracket: once
//! the root is bracketed the step is false position, and any step that would
//! leave the bracket is replaced by a bisection step. It cannot run away.
//!
//! Lateral wind and Coriolis drift fall out of the same integration and are
//! applied as an aim-off afterward. That aim-off is a few milliradians, so its
//! effect back on the wind decomposition and the Coriolis azimuth is ignored.
//!
//! # What it does not do
//!
//! - Spin drift. The Python solver has it (Litz, off Miller SG). The Rust core
//!   has no SG because [`Projectile`](crate::solver::Projectile) carries no
//!   length, diameter or twist. When that lands it adds straight into
//!   `drift_correction` and nothing else here changes.
//! - Aerodynamic jump.
//! - Mount translation. A moving platform adds its own velocity to the muzzle
//!   velocity vector and [`Shot`] cannot express that, so this assumes the mount
//!   is not translating. Mount rotation is fine, that is the gimbal's problem.
//! - Terrain. There is no ground plane. A depressed solution will fly a bullet
//!   straight through a hill and report a clean intercept.
//!
//! # Example
//!
//! ```
//! use ballistics_core::aim::{aim, AimParams, Conditions, Enu, Gun, Target};
//! use ballistics_core::units::*;
//! use ballistics_core::{Projectile, Solver, Station, G7};
//!
//! let solver = Solver::new(
//!     Projectile::new(0.42, grains_to_kg(300.0)),
//!     G7,
//!     Station { temperature: f_to_k(59.0), pressure: inhg_to_pa(29.92), humidity: 0.0 },
//! );
//! let gun = Gun { muzzle_velocity: fps_to_mps(2650.0), sight_height: in_to_m(2.5) };
//!
//! // Target 800 m out on a bearing of 090, crossing left to right at 10 m/s.
//! let target = Target {
//!     position: Enu::from_spherical(90f64.to_radians(), 0.0, 800.0),
//!     velocity: Enu::new(0.0, -10.0, 0.0),
//!     acceleration: Enu::ZERO,
//! };
//!
//! let s = aim(&solver, &gun, &target, &Conditions::CALM, &AimParams::default());
//! assert!(s.status.is_usable());
//! ```

use core::f64::consts::{PI, TAU};
use core::ops::{Add, Mul, Neg, Sub};

use crate::atmosphere::Atmosphere;
use crate::coriolis::Coriolis;
use crate::drag::DragModel;
use crate::math;
use crate::solver::{Flight, Point, Shot, Solver, Wind};
use crate::vec3::Vec3;

// ---------------------------------------------------------------------------
// frame
// ---------------------------------------------------------------------------

/// A vector in the local level frame at the mount: east, north, up, metres.
///
/// Deliberately not [`Vec3`]. The solver's `Vec3` is the shot frame, where y is
/// up and z is right, and mixing the two silently is exactly the bug this type
/// exists to prevent.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Enu {
    pub east: f64,
    pub north: f64,
    pub up: f64,
}

impl Enu {
    pub const ZERO: Enu = Enu::new(0.0, 0.0, 0.0);

    pub const fn new(east: f64, north: f64, up: f64) -> Enu {
        Enu { east, north, up }
    }

    /// From bearing (rad clockwise from north), elevation (rad, + up) and slant
    /// range (m). The inverse of [`Enu::bearing`] / [`Enu::elevation`] /
    /// [`Enu::range`].
    pub fn from_spherical(bearing: f64, elevation: f64, range: f64) -> Enu {
        let (sb, cb) = (math::sin(bearing), math::cos(bearing));
        let (se, ce) = (math::sin(elevation), math::cos(elevation));
        Enu::new(range * ce * sb, range * ce * cb, range * se)
    }

    /// Slant range, m.
    #[inline]
    pub fn range(self) -> f64 {
        math::sqrt(self.east * self.east + self.north * self.north + self.up * self.up)
    }

    /// Horizontal range, m.
    #[inline]
    pub fn ground_range(self) -> f64 {
        math::sqrt(self.east * self.east + self.north * self.north)
    }

    /// Bearing, rad clockwise from true north, wrapped to [0, 2pi).
    #[inline]
    pub fn bearing(self) -> f64 {
        wrap_2pi(math::atan2(self.east, self.north))
    }

    /// Elevation above horizontal, rad, + up.
    #[inline]
    pub fn elevation(self) -> f64 {
        math::atan2(self.up, self.ground_range())
    }
}

impl Add for Enu {
    type Output = Enu;
    #[inline]
    fn add(self, o: Enu) -> Enu {
        Enu::new(self.east + o.east, self.north + o.north, self.up + o.up)
    }
}

impl Sub for Enu {
    type Output = Enu;
    #[inline]
    fn sub(self, o: Enu) -> Enu {
        Enu::new(self.east - o.east, self.north - o.north, self.up - o.up)
    }
}

impl Mul<f64> for Enu {
    type Output = Enu;
    #[inline]
    fn mul(self, k: f64) -> Enu {
        Enu::new(self.east * k, self.north * k, self.up * k)
    }
}

impl Mul<Enu> for f64 {
    type Output = Enu;
    #[inline]
    fn mul(self, v: Enu) -> Enu {
        v * self
    }
}

impl Neg for Enu {
    type Output = Enu;
    #[inline]
    fn neg(self) -> Enu {
        Enu::new(-self.east, -self.north, -self.up)
    }
}

/// Wrap an angle to [0, 2pi).
#[inline]
pub fn wrap_2pi(a: f64) -> f64 {
    let t = a % TAU;
    if t < 0.0 {
        t + TAU
    } else {
        t
    }
}

/// Wrap an angle to [-pi, pi).
#[inline]
pub fn wrap_pi(a: f64) -> f64 {
    wrap_2pi(a + PI) - PI
}

// ---------------------------------------------------------------------------
// inputs
// ---------------------------------------------------------------------------

/// The gun. What the ballistics needs that the target does not supply.
///
/// No zero range. Aiming never uses one: it solves the bore angle for the
/// target in front of it, which is what [`Shot::zero_range`] is a stand-in for
/// on the table path.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Gun {
    /// m/s at the muzzle.
    pub muzzle_velocity: f64,
    /// Sight or sensor above the bore, m, measured perpendicular to the line of
    /// sight. Positive means the sensor is above the barrel. Zero for a
    /// boresighted gun. Target positions are measured from the sensor, so this
    /// is what ties the two together.
    pub sight_height: f64,
}

impl Default for Gun {
    fn default() -> Gun {
        Gun {
            muzzle_velocity: 800.0,
            sight_height: 0.05,
        }
    }
}

/// Target state relative to the mount, in the local level frame.
///
/// This is the tracker's output. Position from the sensor, velocity and
/// acceleration relative to the mount (so a moving mount's own velocity is
/// already differenced out by the tracker, not here).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Target {
    /// m from the sensor.
    pub position: Enu,
    /// m/s relative to the mount. Zero for a static target.
    pub velocity: Enu,
    /// m/s^2. Zero unless the tracker is running a constant-acceleration model.
    pub acceleration: Enu,
}

impl Target {
    /// A target that is not going anywhere.
    pub const fn stationary(position: Enu) -> Target {
        Target {
            position,
            velocity: Enu::ZERO,
            acceleration: Enu::ZERO,
        }
    }

    /// Predicted position `t` seconds from now, constant acceleration.
    #[inline]
    pub fn at(&self, t: f64) -> Enu {
        self.position + self.velocity * t + self.acceleration * (0.5 * t * t)
    }

    /// Predicted velocity `t` seconds from now.
    #[inline]
    pub fn velocity_at(&self, t: f64) -> Enu {
        self.velocity + self.acceleration * t
    }
}

/// Conditions that do not depend on where the gun ends up pointing.
///
/// Wind lives here as a level-frame vector rather than in [`Shot`], because the
/// shot-frame decomposition changes with the firing azimuth and getting that
/// re-derived on every pass is the whole point.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Conditions {
    /// Air velocity, m/s. Where the air is GOING, not where it comes from.
    /// [`Conditions::wind_from`] builds it the way wind gets reported.
    pub wind: Enu,
    /// Mount latitude, rad, + north. `None` turns off Coriolis and leaves
    /// gravity at standard.
    pub latitude: Option<f64>,
}

impl Conditions {
    pub const CALM: Conditions = Conditions {
        wind: Enu::ZERO,
        latitude: None,
    };

    /// Wind the way it gets reported: a speed, and the bearing it blows FROM.
    /// A 5 m/s wind from due west is `wind_from(5.0, 270f64.to_radians())`.
    pub fn wind_from(speed: f64, from_bearing: f64) -> Conditions {
        Conditions {
            wind: Enu::new(
                -speed * math::sin(from_bearing),
                -speed * math::cos(from_bearing),
                0.0,
            ),
            latitude: None,
        }
    }

    /// Same conditions with Coriolis and latitude gravity switched on.
    pub fn at_latitude(mut self, latitude: f64) -> Conditions {
        self.latitude = Some(latitude);
        self
    }
}

/// Solver budget and limits. Defaults are sized for a rifle-class problem on a
/// machine that is not in a hurry; trim `max_lead_iters` and widen the
/// tolerances for a hard real-time loop, and warm start.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AimParams {
    /// Integrator step, s.
    pub dt: f64,
    /// Stop the lead loop once the time of flight moves less than this, s.
    pub tof_tol: f64,
    /// Lead passes before giving up.
    pub max_lead_iters: u32,
    /// Stop the superelevation solve once the predicted miss at the target
    /// subtends less than this angle, rad. 1e-6 rad is a thousandth of a mil.
    pub elevation_tol: f64,
    /// Trajectory integrations the superelevation solve may spend per pass.
    pub max_elevation_iters: u32,
    /// Bore elevation limits relative to the line of sight, rad. Superelevation
    /// is positive for any normal mount; it only goes negative when the sensor
    /// sits below the bore.
    pub min_superelevation: f64,
    pub max_superelevation: f64,
    /// Forward difference step used by [`aim_with_rates`], s.
    pub rate_step: f64,
}

impl Default for AimParams {
    fn default() -> AimParams {
        AimParams {
            dt: 1e-3,
            tof_tol: 1e-4,
            max_lead_iters: 6,
            elevation_tol: 1e-6,
            max_elevation_iters: 12,
            min_superelevation: -0.35,
            max_superelevation: 0.35,
            rate_step: 0.02,
        }
    }
}

// ---------------------------------------------------------------------------
// outputs
// ---------------------------------------------------------------------------

/// How the solve went. Check this before firing anything.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Status {
    /// Both loops met their tolerance.
    Converged,
    /// Ran out of lead passes. The angles are the last pass and are usually
    /// close, but the target is moving fast enough relative to the bullet that
    /// the fixed point did not settle. Caller's call.
    LeadNotConverged,
    /// The superelevation solve ran out of integrations. Same story one level
    /// down.
    ElevationNotConverged,
    /// Out of reach at `max_superelevation`, or below `min_superelevation`. The
    /// angles are the limit that got hit, not a firing solution.
    Unreachable,
}

impl Status {
    /// True only for [`Status::Converged`].
    #[inline]
    pub fn is_converged(&self) -> bool {
        matches!(self, Status::Converged)
    }

    /// True when there are real numbers attached, converged or not. False for
    /// [`Status::Unreachable`], where the angles are a limit and the impact
    /// figures are NaN.
    #[inline]
    pub fn is_usable(&self) -> bool {
        !matches!(self, Status::Unreachable)
    }
}

/// Where to point.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Solution {
    /// Gun bearing, rad clockwise from true north, [0, 2pi).
    pub bearing: f64,
    /// Gun elevation above horizontal, rad, + up.
    pub elevation: f64,
    /// Time of flight to `intercept`, s. Exact for that point: it is what the
    /// integration returned. This is the number the tracker needs back to know
    /// how far ahead it is predicting.
    pub tof: f64,
    /// The point the gun angles were solved for, m from the sensor.
    ///
    /// This is the aim point, not `target.at(tof)`. The two differ by the last
    /// pass of the fixed point, at most `tof_tol` times the target's speed
    /// (millimetres at the defaults). `bearing`, `elevation`, `range` and `tof`
    /// all belong to THIS point and are consistent with each other; if you need
    /// the residual, compare it against `target.at(tof)` yourself.
    pub intercept: Enu,
    /// Slant range to the intercept, m.
    pub range: f64,
    /// Bore elevation above the line of sight to the intercept, rad. The drop
    /// compensation on its own.
    pub superelevation: f64,
    /// Lateral aim-off for wind and Coriolis drift, rad, + right. Already
    /// included in `bearing`. Subtract it to see the pure lead.
    pub drift_correction: f64,
    /// Gun bearing minus the bearing to where the target is right now, rad,
    /// wrapped to (-pi, pi]. Target motion and drift aim-off together.
    pub lead_bearing: f64,
    /// Gun elevation minus the elevation to where the target is right now, rad.
    /// Target motion and superelevation together.
    pub lead_elevation: f64,
    /// m/s at the intercept.
    pub impact_speed: f64,
    /// Mach at the intercept. Below 1.2 the point-mass model is on thin ice.
    pub impact_mach: f64,
    /// J at the intercept.
    pub impact_energy: f64,
    pub status: Status,
    /// Lead passes used.
    pub lead_iterations: u32,
    /// Trajectory integrations spent. This is the cost; budget against it.
    pub trajectories: u32,
}

/// Slew rates that hold the solution as the target moves, for feed forward.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Rates {
    /// rad/s, + clockwise.
    pub bearing: f64,
    /// rad/s, + up.
    pub elevation: f64,
}

// ---------------------------------------------------------------------------
// solve
// ---------------------------------------------------------------------------

/// Solve the firing problem cold.
pub fn aim<D: DragModel, A: Atmosphere>(
    solver: &Solver<D, A>,
    gun: &Gun,
    target: &Target,
    conditions: &Conditions,
    params: &AimParams,
) -> Solution {
    aim_hinted(solver, gun, target, conditions, params, None)
}

/// Solve it warm, from the last frame's answer.
///
/// In a tracking loop this is the one to call. The hint seeds the time of
/// flight and the superelevation, which normally collapses the whole thing to
/// one lead pass and two or three integrations.
pub fn aim_hinted<D: DragModel, A: Atmosphere>(
    solver: &Solver<D, A>,
    gun: &Gun,
    target: &Target,
    conditions: &Conditions,
    params: &AimParams,
    hint: Option<&Solution>,
) -> Solution {
    let now = target.position;
    let bearing_now = now.bearing();
    let elevation_now = now.elevation();

    let mut tof = match hint {
        Some(h) if h.tof.is_finite() && h.tof >= 0.0 => h.tof,
        _ => {
            if gun.muzzle_velocity > 0.0 {
                now.range() / gun.muzzle_velocity
            } else {
                0.0
            }
        }
    };
    let mut theta = match hint {
        Some(h) if h.superelevation.is_finite() => h.superelevation,
        _ => 0.0,
    };

    let mut trajectories = 0u32;
    let mut status = Status::LeadNotConverged;
    let mut iters = 0u32;

    // Geometry of the pass we are reporting.
    let mut intercept = now;
    let mut los_bearing = bearing_now;
    let mut los_elevation = elevation_now;
    let mut terminal: Option<Point> = None;

    for i in 0..params.max_lead_iters.max(1) {
        iters = i + 1;

        intercept = target.at(tof);
        let range = intercept.range();
        los_bearing = intercept.bearing();
        los_elevation = intercept.elevation();

        let shot = build_shot(gun, conditions, params, los_bearing, los_elevation);
        let solved = solve_superelevation(solver, &shot, range, theta, params, &mut trajectories);

        theta = solved.theta;

        match solved.point {
            Some(pt) => {
                let settled = math::abs(pt.time - tof) <= params.tof_tol;
                tof = pt.time;
                terminal = Some(pt);
                if solved.status != Status::Converged {
                    status = solved.status;
                    break;
                }
                if settled {
                    status = Status::Converged;
                    break;
                }
            }
            None => {
                status = solved.status;
                terminal = None;
                break;
            }
        }
    }

    let (drift_correction, impact_speed, impact_mach, impact_energy) = match terminal {
        Some(p) => {
            let r = if p.range > 0.0 { p.range } else { intercept.range() };
            let correction = if r > 0.0 {
                -math::atan2(p.drift, r)
            } else {
                0.0
            };
            (correction, p.speed, p.mach, p.energy)
        }
        None => (0.0, f64::NAN, f64::NAN, f64::NAN),
    };

    let bearing = wrap_2pi(los_bearing + drift_correction);
    let elevation = los_elevation + theta;

    Solution {
        bearing,
        elevation,
        tof,
        intercept,
        range: intercept.range(),
        superelevation: theta,
        drift_correction,
        lead_bearing: wrap_pi(bearing - bearing_now),
        lead_elevation: elevation - elevation_now,
        impact_speed,
        impact_mach,
        impact_energy,
        status,
        lead_iterations: iters,
        trajectories,
    }
}

/// Solve, and also give the slew rates that keep the solution good.
///
/// Rates come from a forward difference: solve now, advance the target by
/// `params.rate_step`, solve again warm from the first answer. Forward rather
/// than central because central costs a third integration set and this feeds a
/// feed-forward term that the rate loop is going to clean up anyway.
///
/// Returns the solution for NOW. The second solve is scratch.
pub fn aim_with_rates<D: DragModel, A: Atmosphere>(
    solver: &Solver<D, A>,
    gun: &Gun,
    target: &Target,
    conditions: &Conditions,
    params: &AimParams,
) -> (Solution, Rates) {
    let s0 = aim(solver, gun, target, conditions, params);
    let h = params.rate_step;
    if !(h > 0.0) || !s0.status.is_usable() {
        return (s0, Rates::default());
    }

    let ahead = Target {
        position: target.at(h),
        velocity: target.velocity_at(h),
        acceleration: target.acceleration,
    };
    let s1 = aim_hinted(solver, gun, &ahead, conditions, params, Some(&s0));
    if !s1.status.is_usable() {
        return (s0, Rates::default());
    }

    (
        s0,
        Rates {
            bearing: wrap_pi(s1.bearing - s0.bearing) / h,
            elevation: (s1.elevation - s0.elevation) / h,
        },
    )
}

// ---------------------------------------------------------------------------
// internals
// ---------------------------------------------------------------------------

/// Build the shot frame for a given firing azimuth and look angle.
///
/// The shot frame is x horizontal along the azimuth, y up, z right, which makes
/// the rotation from the level frame:
///
/// ```text
/// x_hat = ( sin B,  cos B, 0)
/// y_hat = (     0,      0, 1)
/// z_hat = ( cos B, -sin B, 0)
/// ```
///
/// `zero_range` is set to a placeholder because nothing on this path calls
/// [`Solver::zero_angle`]; the bore angle comes from the superelevation solve.
fn build_shot(
    gun: &Gun,
    conditions: &Conditions,
    params: &AimParams,
    bearing: f64,
    look_angle: f64,
) -> Shot {
    let (sb, cb) = (math::sin(bearing), math::cos(bearing));
    let w = conditions.wind;
    let wind = Wind(Vec3::new(
        w.east * sb + w.north * cb,
        w.up,
        w.east * cb - w.north * sb,
    ));

    Shot {
        muzzle_velocity: gun.muzzle_velocity,
        sight_height: gun.sight_height,
        zero_range: 1.0,
        look_angle,
        wind,
        coriolis: conditions.latitude.map(|latitude| Coriolis {
            latitude,
            azimuth: bearing,
        }),
        dt: params.dt,
    }
}

struct Solved {
    status: Status,
    theta: f64,
    point: Option<Point>,
}

/// Bore elevation above the line of sight that puts the bullet on the line of
/// sight at `range`.
///
/// Safeguarded secant. `path` at the target range is monotone increasing in
/// theta over the direct-fire branch, so once we have one angle that shoots low
/// and one that shoots high the root is bracketed and cannot escape. A
/// trajectory that never reaches `range` counts as shooting low, which is the
/// physically right reading: it fell short.
fn solve_superelevation<D: DragModel, A: Atmosphere>(
    solver: &Solver<D, A>,
    shot: &Shot,
    range: f64,
    hint: f64,
    params: &AimParams,
    trajectories: &mut u32,
) -> Solved {
    let lo_lim = params.min_superelevation;
    let hi_lim = params.max_superelevation;
    let span = range.max(1.0);
    // Angular tolerance turned into a miss distance at this range.
    let miss_tol = params.elevation_tol * span;

    let mut theta = if hint.is_finite() {
        hint.clamp(lo_lim, hi_lim)
    } else {
        0.0
    };

    // (theta, miss) either side of the root, once we find them.
    let mut low: Option<(f64, f64)> = None; // miss < 0, shooting under
    let mut high: Option<(f64, f64)> = None; // miss > 0, shooting over
    let mut best: Option<(f64, f64, Point)> = None; // (|miss|, theta, point)

    for _ in 0..params.max_elevation_iters.max(1) {
        *trajectories += 1;
        let flown = Flight::new(solver, shot, theta, range, f64::INFINITY, range).next();

        let miss = match flown {
            Some(p) => {
                let m = p.path;
                let am = math::abs(m);
                let better = match best {
                    Some((bm, _, _)) => am < bm,
                    None => true,
                };
                if better {
                    best = Some((am, theta, p));
                }
                if am <= miss_tol {
                    return Solved {
                        status: Status::Converged,
                        theta,
                        point: Some(p),
                    };
                }
                m
            }
            // Never got there. Treat as far too low so the bracket logic pushes up.
            None => f64::NEG_INFINITY,
        };

        if miss < 0.0 {
            low = Some((theta, miss));
        } else {
            high = Some((theta, miss));
        }

        let next = match (low, high) {
            (Some((tl, ml)), Some((th, mh))) => {
                // Bracketed. False position, kept off the ends so it cannot
                // stall against one side, and bisection if anything is odd.
                let width = th - tl;
                if !(width > 0.0) || !(mh > ml) || !ml.is_finite() || !mh.is_finite() {
                    0.5 * (tl + th)
                } else {
                    let t = tl + width * (-ml) / (mh - ml);
                    t.clamp(tl + 0.01 * width, th - 0.01 * width)
                }
            }
            // Only ever shot under. Step up.
            (Some((tl, ml)), None) => {
                if ml.is_finite() {
                    // d(path)/d(theta) is about the range for small angles.
                    (tl - ml / span).clamp(tl + 1e-9, hi_lim)
                } else {
                    // Never got to the target at all, so there is no gradient
                    // to follow. Go straight to the ceiling: if that cannot
                    // reach it nothing can, and we are out in two integrations
                    // instead of creeping up on the limit for twelve.
                    hi_lim
                }
            }
            // Only ever shot over. Step down.
            (None, Some((th, mh))) => (th - mh / span).clamp(lo_lim, th - 1e-9),
            (None, None) => break,
        };

        // Pinned against a limit with no bracket: nothing left to try.
        if math::abs(next - theta) < 1e-12 {
            return match best {
                Some((_, bt, bp)) => Solved {
                    status: Status::Unreachable,
                    theta: bt,
                    point: Some(bp),
                },
                None => Solved {
                    status: Status::Unreachable,
                    theta,
                    point: None,
                },
            };
        }
        theta = next;
    }

    match best {
        Some((_, bt, bp)) => Solved {
            status: Status::ElevationNotConverged,
            theta: bt,
            point: Some(bp),
        },
        None => Solved {
            status: Status::Unreachable,
            theta,
            point: None,
        },
    }
}
