//! Conversions to/from SI. The core takes SI; use these at the edges.

use core::f64::consts::PI;

pub const FT: f64 = 0.3048; // m
pub const YD: f64 = 0.9144; // m
pub const IN: f64 = 0.0254; // m
pub const GRAIN: f64 = 6.479_891e-5; // kg
pub const MPH: f64 = 0.447_04; // m/s
pub const INHG: f64 = 3386.389; // Pa
pub const FT_LBF: f64 = 1.355_817_948_331_400_4; // J

#[inline]
pub fn fps_to_mps(v: f64) -> f64 {
    v * FT
}
#[inline]
pub fn mps_to_fps(v: f64) -> f64 {
    v / FT
}
#[inline]
pub fn yd_to_m(d: f64) -> f64 {
    d * YD
}
#[inline]
pub fn m_to_yd(d: f64) -> f64 {
    d / YD
}
#[inline]
pub fn ft_to_m(d: f64) -> f64 {
    d * FT
}
#[inline]
pub fn m_to_ft(d: f64) -> f64 {
    d / FT
}
#[inline]
pub fn in_to_m(d: f64) -> f64 {
    d * IN
}
#[inline]
pub fn m_to_in(d: f64) -> f64 {
    d / IN
}
#[inline]
pub fn grains_to_kg(m: f64) -> f64 {
    m * GRAIN
}
#[inline]
pub fn kg_to_grains(m: f64) -> f64 {
    m / GRAIN
}
#[inline]
pub fn mph_to_mps(v: f64) -> f64 {
    v * MPH
}
#[inline]
pub fn mps_to_mph(v: f64) -> f64 {
    v / MPH
}
#[inline]
pub fn inhg_to_pa(p: f64) -> f64 {
    p * INHG
}
#[inline]
pub fn pa_to_inhg(p: f64) -> f64 {
    p / INHG
}
#[inline]
pub fn hpa_to_pa(p: f64) -> f64 {
    p * 100.0
}
#[inline]
pub fn f_to_k(t: f64) -> f64 {
    (t - 32.0) * 5.0 / 9.0 + 273.15
}
#[inline]
pub fn c_to_k(t: f64) -> f64 {
    t + 273.15
}
#[inline]
pub fn k_to_f(t: f64) -> f64 {
    (t - 273.15) * 9.0 / 5.0 + 32.0
}
#[inline]
pub fn j_to_ftlbf(e: f64) -> f64 {
    e / FT_LBF
}

/// Minute of angle.
#[inline]
pub fn moa_to_rad(a: f64) -> f64 {
    a / 60.0 * PI / 180.0
}
#[inline]
pub fn rad_to_moa(a: f64) -> f64 {
    a * 180.0 / PI * 60.0
}
/// Milliradian.
#[inline]
pub fn mil_to_rad(a: f64) -> f64 {
    a * 1e-3
}
#[inline]
pub fn rad_to_mil(a: f64) -> f64 {
    a * 1e3
}
