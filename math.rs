//! Thin wrappers over `libm` so the crate builds without std.
//! (`f64::sqrt`, `sin`, `pow` … are std-only; `min`/`max`/`clamp`/`to_radians` are in core.)

#[inline]
pub fn sqrt(x: f64) -> f64 {
    libm::sqrt(x)
}
#[inline]
pub fn sin(x: f64) -> f64 {
    libm::sin(x)
}
#[inline]
pub fn cos(x: f64) -> f64 {
    libm::cos(x)
}
#[inline]
pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}
#[inline]
pub fn exp(x: f64) -> f64 {
    libm::exp(x)
}
#[inline]
pub fn pow(x: f64, y: f64) -> f64 {
    libm::pow(x, y)
}
#[inline]
pub fn abs(x: f64) -> f64 {
    libm::fabs(x)
}
