//! Reference drag curves.
//!
//! Deceleration is `a = -(π ρ / (8 BC)) · Cd_ref(M) · |v| v` with BC in kg/m².
//! Cd_ref is the drag coefficient of the *reference* projectile (G7 = long
//! boat-tail); the form factor between your bullet and the reference is what
//! the BC carries, so no diameter or mass shows up in the drag term.

/// A reference drag curve: Cd of the standard projectile at a given Mach number.
pub trait DragModel {
    fn cd(&self, mach: f64) -> f64;
}

/// Piecewise-linear interpolation over a `(mach, cd)` table sorted by Mach.
/// Clamps to the end values outside the table.
pub fn interp(table: &[(f64, f64)], mach: f64) -> f64 {
    let n = table.len();
    if n == 0 {
        return 0.0;
    }
    if mach <= table[0].0 {
        return table[0].1;
    }
    if mach >= table[n - 1].0 {
        return table[n - 1].1;
    }
    let (mut lo, mut hi) = (0usize, n - 1);
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if table[mid].0 <= mach {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let (m0, c0) = table[lo];
    let (m1, c1) = table[hi];
    if m1 <= m0 {
        return c0;
    }
    c0 + (c1 - c0) * (mach - m0) / (m1 - m0)
}

/// G7 reference projectile (long boat-tail). Use with G7 BCs.
#[derive(Clone, Copy, Debug, Default)]
pub struct G7;

impl DragModel for G7 {
    #[inline]
    fn cd(&self, mach: f64) -> f64 {
        interp(G7_TABLE, mach)
    }
}

/// Any tabulated Cd-vs-Mach curve, e.g. a manufacturer's Doppler-radar data
/// for one specific bullet (then use BC = sectional density, form factor 1).
#[derive(Clone, Copy, Debug)]
pub struct TableDrag<'a> {
    pub table: &'a [(f64, f64)],
}

impl DragModel for TableDrag<'_> {
    #[inline]
    fn cd(&self, mach: f64) -> f64 {
        interp(self.table, mach)
    }
}

/// Zero drag. Exists so the integrator can be checked against the vacuum
/// closed form.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoDrag;

impl DragModel for NoDrag {
    #[inline]
    fn cd(&self, _mach: f64) -> f64 {
        0.0
    }
}

/// Standard G7 drag function, Cd vs Mach.
/// Diff this against the table in the Python solver before trusting any
/// cross-check — the tables floating around differ in the last digit here and there.
pub const G7_TABLE: &[(f64, f64)] = &[
    (0.00, 0.1198),
    (0.05, 0.1197),
    (0.10, 0.1196),
    (0.15, 0.1194),
    (0.20, 0.1193),
    (0.25, 0.1194),
    (0.30, 0.1194),
    (0.35, 0.1194),
    (0.40, 0.1193),
    (0.45, 0.1193),
    (0.50, 0.1194),
    (0.55, 0.1193),
    (0.60, 0.1194),
    (0.65, 0.1197),
    (0.70, 0.1202),
    (0.725, 0.1207),
    (0.75, 0.1215),
    (0.775, 0.1226),
    (0.80, 0.1242),
    (0.825, 0.1266),
    (0.85, 0.1306),
    (0.875, 0.1368),
    (0.90, 0.1464),
    (0.925, 0.1660),
    (0.95, 0.2054),
    (0.975, 0.2993),
    (1.00, 0.3803),
    (1.025, 0.4015),
    (1.05, 0.4043),
    (1.075, 0.4034),
    (1.10, 0.4014),
    (1.125, 0.3987),
    (1.15, 0.3955),
    (1.20, 0.3884),
    (1.25, 0.3810),
    (1.30, 0.3732),
    (1.35, 0.3657),
    (1.40, 0.3580),
    (1.50, 0.3440),
    (1.55, 0.3376),
    (1.60, 0.3315),
    (1.65, 0.3260),
    (1.70, 0.3209),
    (1.75, 0.3160),
    (1.80, 0.3117),
    (1.85, 0.3078),
    (1.90, 0.3042),
    (1.95, 0.3010),
    (2.00, 0.2980),
    (2.05, 0.2951),
    (2.10, 0.2922),
    (2.15, 0.2892),
    (2.20, 0.2864),
    (2.25, 0.2835),
    (2.30, 0.2807),
    (2.35, 0.2779),
    (2.40, 0.2752),
    (2.45, 0.2725),
    (2.50, 0.2697),
    (2.55, 0.2670),
    (2.60, 0.2643),
    (2.65, 0.2615),
    (2.70, 0.2588),
    (2.75, 0.2561),
    (2.80, 0.2533),
    (2.85, 0.2506),
    (2.90, 0.2479),
    (2.95, 0.2451),
    (3.00, 0.2424),
    (3.10, 0.2368),
    (3.20, 0.2313),
    (3.30, 0.2258),
    (3.40, 0.2205),
    (3.50, 0.2154),
    (3.60, 0.2106),
    (3.70, 0.2060),
    (3.80, 0.2017),
    (3.90, 0.1975),
    (4.00, 0.1935),
    (4.20, 0.1861),
    (4.40, 0.1793),
    (4.60, 0.1730),
    (4.80, 0.1672),
    (5.00, 0.1618),
];
