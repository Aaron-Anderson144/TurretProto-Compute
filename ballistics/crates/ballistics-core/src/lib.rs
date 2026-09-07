//! Exterior ballistics core.
//!
//! 3-DOF point-mass model integrated with fixed-step RK4. Drag comes from a
//! reference drag curve (G7 by default, or any tabulated Cd-vs-Mach) scaled by
//! the ballistic coefficient. Atmosphere is ISA or measured station conditions
//! with humidity. Wind and Coriolis are included. Gravity is latitude-corrected
//! when a latitude is supplied.
//!
//! `no_std` when built with `--no-default-features`; math goes through `libm`.
//!
//! Frame (right-handed, shooter's view):
//!   x = downrange, horizontal, along the firing azimuth
//!   y = up
//!   z = right
//!
//! Everything inside the crate is SI (m, m/s, kg, K, Pa, rad). BC is the one
//! exception: it is taken in lb/in² because that is how it is published.
//! Convert at the edges with [`units`].
//!
//! Sign conventions on outputs: `path` is + above the line of sight, `drift`
//! is + right. What you dial is the opposite sign.
//!
//! Two front doors:
//!
//! - [`solver`] and `table` go forward. Bullet and conditions in,
//!   where it lands out. That is the drop table and the CLI.
//! - [`aim`] goes backward. Target position and velocity in, gun bearing,
//!   elevation and time of flight out. That is what a fire control loop calls,
//!   and it works in east/north/up rather than the shot frame.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
// `!(x > 0.0)` is used throughout to reject NaN along with the out-of-range
// case, which is the point. `x <= 0.0` would let NaN through.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

pub mod aim;
pub mod atmosphere;
pub mod coriolis;
pub mod drag;
pub mod math;
pub mod solver;
pub mod units;
pub mod vec3;

#[cfg(feature = "table")]
pub mod table;

pub use aim::{AimParams, Conditions, Enu, Gun, Rates, Solution, Status, Target};
pub use atmosphere::{Air, Atmosphere, Isa, Station};
pub use coriolis::Coriolis;
pub use drag::{DragModel, NoDrag, TableDrag, G7};
pub use solver::{Flight, Point, Projectile, Shot, Solver, Wind};
pub use vec3::Vec3;
