//! Point-mass trajectory: RK4 over position/velocity with drag, gravity, wind, Coriolis.

use core::f64::consts::PI;

use crate::atmosphere::{Atmosphere, ASM_RHO, G0, ISA_RHO0};
use crate::coriolis::Coriolis;
use crate::drag::DragModel;
use crate::math;
use crate::vec3::Vec3;

/// lb/in² → kg/m².
pub const BC_LB_IN2_TO_KG_M2: f64 = 0.453_592_37 / (0.0254 * 0.0254);
/// A BC quoted against Army Standard Metro is bigger than the same bullet's
/// ICAO BC by the density ratio. Multiply an ASM BC by this to get ICAO.
pub const ASM_TO_ICAO_BC: f64 = ASM_RHO / ISA_RHO0;

/// The bullet, as far as a point-mass model cares.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Projectile {
    /// Ballistic coefficient for the solver's drag model, lb/in², ICAO reference.
    pub bc: f64,
    /// kg. Only used for energy.
    pub mass: f64,
}

impl Projectile {
    pub const fn new(bc: f64, mass: f64) -> Projectile {
        Projectile { bc, mass }
    }

    /// From a BC quoted against Army Standard Metro.
    pub fn from_asm_bc(bc_asm: f64, mass: f64) -> Projectile {
        Projectile::new(bc_asm * ASM_TO_ICAO_BC, mass)
    }

    #[inline]
    pub fn bc_si(&self) -> f64 {
        self.bc * BC_LB_IN2_TO_KG_M2
    }
}

/// Wind as the air-mass velocity in the shot frame, m/s.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Wind(pub Vec3);

impl Wind {
    pub const CALM: Wind = Wind(Vec3::ZERO);

    /// `from_deg` is the direction the wind blows FROM, degrees clockwise from
    /// downrange: 0 = headwind, 90 = from the right, 180 = tailwind, 270 = from the left.
    pub fn from_direction(speed: f64, from_deg: f64) -> Wind {
        let b = from_deg.to_radians();
        Wind(Vec3::new(-speed * math::cos(b), 0.0, -speed * math::sin(b)))
    }

    /// Clock face: 12 = headwind, 3 = from the right, 6 = tailwind, 9 = from the left.
    pub fn from_clock(speed: f64, hours: f64) -> Wind {
        Wind::from_direction(speed, hours * 30.0)
    }
}

/// One shot's setup. Angles rad, lengths m, speeds m/s.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Shot {
    pub muzzle_velocity: f64,
    /// Sight above bore, measured perpendicular to the line of sight.
    pub sight_height: f64,
    /// Distance along the line of sight at which the path crosses it, level fire.
    pub zero_range: f64,
    /// Line-of-sight angle above horizontal (+ uphill).
    pub look_angle: f64,
    pub wind: Wind,
    /// `None` = flat earth, standard gravity.
    pub coriolis: Option<Coriolis>,
    /// Integrator step, s. 1 ms is converged for rifle work (see tests).
    pub dt: f64,
}

impl Default for Shot {
    fn default() -> Shot {
        Shot {
            muzzle_velocity: 800.0,
            sight_height: 0.05,
            zero_range: 100.0,
            look_angle: 0.0,
            wind: Wind::CALM,
            coriolis: None,
            dt: 1e-3,
        }
    }
}

/// Trajectory sample at one range station.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Point {
    /// Distance along the line of sight, m.
    pub range: f64,
    /// Time of flight, s.
    pub time: f64,
    /// Position in the shot frame, m.
    pub position: Vec3,
    /// Ground velocity in the shot frame, m/s.
    pub velocity: Vec3,
    /// |velocity|, m/s.
    pub speed: f64,
    /// Mach number relative to the air.
    pub mach: f64,
    /// Perpendicular offset from the line of sight, m, + above.
    pub path: f64,
    /// Cross-range offset, m, + right.
    pub drift: f64,
    /// J.
    pub energy: f64,
}

impl Point {
    #[inline]
    fn angle(offset: f64, range: f64) -> f64 {
        if range <= 0.0 {
            0.0
        } else {
            math::atan2(offset, range)
        }
    }
    /// Path as an angle, minutes (+ above LOS).
    pub fn path_moa(&self) -> f64 {
        Self::angle(self.path, self.range).to_degrees() * 60.0
    }
    /// Path as an angle, milliradians (+ above LOS).
    pub fn path_mil(&self) -> f64 {
        Self::angle(self.path, self.range) * 1e3
    }
    /// Drift as an angle, minutes (+ right).
    pub fn drift_moa(&self) -> f64 {
        Self::angle(self.drift, self.range).to_degrees() * 60.0
    }
    /// Drift as an angle, milliradians (+ right).
    pub fn drift_mil(&self) -> f64 {
        Self::angle(self.drift, self.range) * 1e3
    }
}

/// Bullet + drag curve + atmosphere. Build one, then ask it for flights.
#[derive(Clone, Copy, Debug)]
pub struct Solver<D, A> {
    pub projectile: Projectile,
    pub drag: D,
    pub atmosphere: A,
}

#[derive(Clone, Copy, Debug)]
struct State {
    p: Vec3,
    v: Vec3,
}

/// Per-shot constants the integrator needs.
#[derive(Clone, Copy, Debug)]
struct Env {
    wind: Vec3,
    coriolis: Option<Coriolis>,
    gravity: Vec3,
}

impl<D: DragModel, A: Atmosphere> Solver<D, A> {
    pub fn new(projectile: Projectile, drag: D, atmosphere: A) -> Solver<D, A> {
        Solver {
            projectile,
            drag,
            atmosphere,
        }
    }

    fn env(&self, shot: &Shot) -> Env {
        let g = shot.coriolis.map(|c| c.gravity()).unwrap_or(G0);
        Env {
            wind: shot.wind.0,
            coriolis: shot.coriolis,
            gravity: Vec3::new(0.0, -g, 0.0),
        }
    }

    /// Total acceleration: drag on the air-relative velocity, gravity, Coriolis on ground velocity.
    fn accel(&self, s: &State, env: &Env) -> Vec3 {
        let air = self.atmosphere.at(s.p.y);
        let vr = s.v - env.wind;
        let speed = vr.norm();
        let mut a = env.gravity;
        if speed > 0.0 {
            let mach = speed / air.speed_of_sound;
            let k = PI * air.density * self.drag.cd(mach) / (8.0 * self.projectile.bc_si());
            a += vr * (-k * speed);
        }
        if let Some(c) = &env.coriolis {
            a += c.acceleration(s.v);
        }
        a
    }

    fn rk4(&self, s: State, dt: f64, env: &Env) -> State {
        let a1 = self.accel(&s, env);
        let v1 = s.v;

        let s2 = State {
            p: s.p + v1 * (0.5 * dt),
            v: s.v + a1 * (0.5 * dt),
        };
        let a2 = self.accel(&s2, env);
        let v2 = s2.v;

        let s3 = State {
            p: s.p + v2 * (0.5 * dt),
            v: s.v + a2 * (0.5 * dt),
        };
        let a3 = self.accel(&s3, env);
        let v3 = s3.v;

        let s4 = State {
            p: s.p + v3 * dt,
            v: s.v + a3 * dt,
        };
        let a4 = self.accel(&s4, env);
        let v4 = s4.v;

        State {
            p: s.p + (v1 + 2.0 * v2 + 2.0 * v3 + v4) * (dt / 6.0),
            v: s.v + (a1 + 2.0 * a2 + 2.0 * a3 + a4) * (dt / 6.0),
        }
    }

    /// Bore elevation above the line of sight (rad) that puts the path on the
    /// LOS at `shot.zero_range`. Solved for level fire, no wind, no Coriolis
    /// deflection, in this solver's atmosphere and at the shot's latitude
    /// gravity — so the atmosphere you build the solver with should be the
    /// one you zeroed under. Use [`Solver::flight_with_zero`] if you want to
    /// carry a zero over from other conditions.
    pub fn zero_angle(&self, shot: &Shot) -> f64 {
        let level = Shot {
            look_angle: 0.0,
            ..*shot
        };
        let env = Env {
            wind: Vec3::ZERO,
            coriolis: None,
            gravity: self.env(shot).gravity,
        };
        let f = |theta: f64| {
            Flight::with_env(self, &level, theta, level.zero_range, f64::INFINITY, level.zero_range, env)
                .next()
                .map(|p| p.path)
                .unwrap_or(f64::NAN)
        };

        let (mut lo, mut hi) = (0.0_f64, 0.25_f64);
        if f(lo) >= 0.0 {
            return lo;
        }
        if !(f(hi) > 0.0) {
            // Can't reach the zero range with 14° of elevation — not a rifle problem.
            return hi;
        }
        for _ in 0..64 {
            let mid = 0.5 * (lo + hi);
            if f(mid) >= 0.0 {
                hi = mid;
            } else {
                lo = mid;
            }
            if hi - lo < 1e-13 {
                break;
            }
        }
        0.5 * (lo + hi)
    }

    /// Path (m, + above LOS) at LOS distance `range` for a given bore elevation,
    /// under the shot's wind and Coriolis. NaN if the bullet never gets there.
    pub fn path_at(&self, shot: &Shot, zero_angle: f64, range: f64) -> f64 {
        Flight::new(self, shot, zero_angle, range, f64::INFINITY, range)
            .next()
            .map(|p| p.path)
            .unwrap_or(f64::NAN)
    }

    /// Zero the rifle, then fly it. Yields a [`Point`] every `step` m along the
    /// line of sight, starting at 0, up to `max_range`.
    pub fn flight(&self, shot: &Shot, step: f64, max_range: f64) -> Flight<'_, D, A> {
        let z = self.zero_angle(shot);
        self.flight_with_zero(shot, z, step, max_range)
    }

    /// Same, with the zero angle supplied (e.g. computed under other conditions).
    pub fn flight_with_zero(
        &self,
        shot: &Shot,
        zero_angle: f64,
        step: f64,
        max_range: f64,
    ) -> Flight<'_, D, A> {
        Flight::new(self, shot, zero_angle, 0.0, step, max_range)
    }

    /// `flight` collected.
    #[cfg(feature = "std")]
    pub fn table(&self, shot: &Shot, step: f64, max_range: f64) -> Vec<Point> {
        self.flight(shot, step, max_range).collect()
    }
}

/// Iterator over range stations of one trajectory. Holds the integrator
/// state, so it allocates nothing and works without std.
pub struct Flight<'a, D, A> {
    solver: &'a Solver<D, A>,
    env: Env,
    los_dir: Vec3,
    los_perp: Vec3,
    sight_height: f64,
    mass: f64,
    dt: f64,
    state: State,
    time: f64,
    next_station: f64,
    step: f64,
    max_range: f64,
    done: bool,
}

impl<'a, D: DragModel, A: Atmosphere> Flight<'a, D, A> {
    pub fn new(
        solver: &'a Solver<D, A>,
        shot: &Shot,
        zero_angle: f64,
        first_station: f64,
        step: f64,
        max_range: f64,
    ) -> Flight<'a, D, A> {
        Flight::with_env(solver, shot, zero_angle, first_station, step, max_range, solver.env(shot))
    }

    fn with_env(
        solver: &'a Solver<D, A>,
        shot: &Shot,
        zero_angle: f64,
        first_station: f64,
        step: f64,
        max_range: f64,
        env: Env,
    ) -> Flight<'a, D, A> {
        let look = shot.look_angle;
        let los_dir = Vec3::new(math::cos(look), math::sin(look), 0.0);
        let los_perp = Vec3::new(-math::sin(look), math::cos(look), 0.0);
        let bore = look + zero_angle;
        let v0 = Vec3::new(math::cos(bore), math::sin(bore), 0.0) * shot.muzzle_velocity;
        Flight {
            solver,
            env,
            los_dir,
            los_perp,
            sight_height: shot.sight_height,
            mass: solver.projectile.mass,
            dt: if shot.dt > 0.0 { shot.dt } else { 1e-3 },
            state: State { p: Vec3::ZERO, v: v0 },
            time: 0.0,
            next_station: first_station,
            step: if step > 0.0 { step } else { f64::INFINITY },
            max_range,
            done: false,
        }
    }

    fn point(&self, s: State, t: f64) -> Point {
        let air = self.solver.atmosphere.at(s.p.y);
        let speed = s.v.norm();
        Point {
            range: s.p.dot(self.los_dir),
            time: t,
            position: s.p,
            velocity: s.v,
            speed,
            mach: (s.v - self.env.wind).norm() / air.speed_of_sound,
            path: s.p.dot(self.los_perp) - self.sight_height,
            drift: s.p.z,
            energy: 0.5 * self.mass * speed * speed,
        }
    }
}

impl<'a, D: DragModel, A: Atmosphere> Iterator for Flight<'a, D, A> {
    type Item = Point;

    fn next(&mut self) -> Option<Point> {
        if self.done || self.next_station > self.max_range + 1e-9 {
            return None;
        }
        loop {
            let s0 = self.state;
            let r0 = s0.p.dot(self.los_dir);
            if r0 >= self.next_station - 1e-9 {
                let pt = self.point(s0, self.time);
                self.next_station += self.step;
                return Some(pt);
            }

            let s1 = self.solver.rk4(s0, self.dt, &self.env);
            let r1 = s1.p.dot(self.los_dir);
            if r1 >= self.next_station {
                // Land on the station with a partial RK4 step from s0. Range is
                // not linear in time (drag), so refine the partial step with a
                // couple of Newton iterations. State is left at s0; the next
                // call re-steps from there.
                let target = self.next_station;
                let mut tau = self.dt * (target - r0) / (r1 - r0);
                let mut sf = self.solver.rk4(s0, tau, &self.env);
                for _ in 0..4 {
                    let vr = sf.v.dot(self.los_dir);
                    if vr <= 0.0 {
                        break;
                    }
                    let d = (target - sf.p.dot(self.los_dir)) / vr;
                    tau += d;
                    sf = self.solver.rk4(s0, tau, &self.env);
                    if math::abs(d) < 1e-13 {
                        break;
                    }
                }
                let pt = self.point(sf, self.time + tau);
                self.next_station += self.step;
                return Some(pt);
            }

            self.state = s1;
            self.time += self.dt;
            if r1 <= r0 || self.time > 120.0 || s1.v.norm() < 10.0 || s1.p.y < -10_000.0 {
                self.done = true;
                return None;
            }
        }
    }
}
