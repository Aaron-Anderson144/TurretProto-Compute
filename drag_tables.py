"""
Standard projectile drag functions.

The G7 standard projectile is a 7-caliber-long secant-ogive boat-tail form that
closely resembles modern low-drag .338 match bullets (Berger Hybrid, Sierra MK,
Hornady ELD-M). For these bullets G7 tracks true drag far better than G1, whose
flat-base reference shape diverges badly through the transonic region.

Cd values below are the standard tabulated drag coefficient of the reference
projectile as a function of Mach number. A bullet's G7 ballistic coefficient
(BC7) scales this reference curve: the retardation applied in the integrator is

    a_drag = (rho / rho_ref) * (Cd_ref(M) / BC7) * (v^2 / (2 * i_ref)) ...

but in practice we use the equivalent, cleaner BC formulation implemented in
solver.py, where BC7 carries sectional density and form factor together.

NOTE ON FIDELITY: this table is a solid, standard baseline suitable for getting
correct trajectory *shape*. For match-grade dope you should still TRUE the solver
to observed velocity + drop (see stability.true_bc). Every serious solver is
trued in the field; none is trusted raw.
"""

import numpy as np

# --- G7 standard drag function: (Mach, Cd_ref) -----------------------------
# Dense sampling through transonic (0.8-1.3 Mach) where curvature is highest and
# where a .338 NM spends its terminal, wind-sensitive flight at long range.
_G7 = np.array([
    [0.00, 0.1198], [0.05, 0.1197], [0.10, 0.1196], [0.15, 0.1194],
    [0.20, 0.1193], [0.25, 0.1194], [0.30, 0.1194], [0.35, 0.1193],
    [0.40, 0.1193], [0.45, 0.1193], [0.50, 0.1194], [0.55, 0.1193],
    [0.60, 0.1194], [0.65, 0.1197], [0.70, 0.1202], [0.725, 0.1207],
    [0.75, 0.1215], [0.775, 0.1226], [0.80, 0.1242], [0.825, 0.1266],
    [0.85, 0.1306], [0.875, 0.1368], [0.90, 0.1464], [0.925, 0.1660],
    [0.95, 0.2054], [0.975, 0.2993], [1.00, 0.3803], [1.025, 0.4015],
    [1.05, 0.4043], [1.075, 0.4034], [1.10, 0.4014], [1.125, 0.3987],
    [1.15, 0.3955], [1.20, 0.3884], [1.25, 0.3810], [1.30, 0.3732],
    [1.35, 0.3657], [1.40, 0.3580], [1.50, 0.3440], [1.55, 0.3376],
    [1.60, 0.3315], [1.65, 0.3260], [1.70, 0.3209], [1.75, 0.3160],
    [1.80, 0.3117], [1.85, 0.3078], [1.90, 0.3042], [1.95, 0.3010],
    [2.00, 0.2980], [2.05, 0.2951], [2.10, 0.2922], [2.15, 0.2892],
    [2.20, 0.2864], [2.25, 0.2835], [2.30, 0.2807], [2.35, 0.2779],
    [2.40, 0.2752], [2.45, 0.2725], [2.50, 0.2697], [2.60, 0.2643],
    [2.70, 0.2592], [2.80, 0.2545], [2.90, 0.2502], [3.00, 0.2463],
    [3.10, 0.2427], [3.20, 0.2395], [3.30, 0.2366], [3.40, 0.2339],
    [3.50, 0.2313], [3.60, 0.2290], [3.80, 0.2250], [4.00, 0.2210],
    [4.20, 0.2172], [4.40, 0.2138], [4.60, 0.2110], [4.80, 0.2090],
    [5.00, 0.2073],
])

# G1 standard drag function, provided for reference / comparison only.
_G1 = np.array([
    [0.00, 0.2629], [0.05, 0.2558], [0.10, 0.2487], [0.15, 0.2413],
    [0.20, 0.2344], [0.25, 0.2278], [0.30, 0.2214], [0.35, 0.2155],
    [0.40, 0.2104], [0.45, 0.2061], [0.50, 0.2032], [0.55, 0.2020],
    [0.60, 0.2034], [0.70, 0.2165], [0.725, 0.2230], [0.75, 0.2313],
    [0.775, 0.2417], [0.80, 0.2546], [0.825, 0.2706], [0.85, 0.2901],
    [0.875, 0.3136], [0.90, 0.3415], [0.925, 0.3734], [0.95, 0.4084],
    [0.975, 0.4448], [1.00, 0.4805], [1.025, 0.5136], [1.05, 0.5427],
    [1.075, 0.5677], [1.10, 0.5883], [1.125, 0.6053], [1.15, 0.6191],
    [1.20, 0.6393], [1.25, 0.6518], [1.30, 0.6589], [1.35, 0.6621],
    [1.40, 0.6625], [1.50, 0.6573], [1.60, 0.6461], [1.70, 0.6319],
    [1.80, 0.6157], [1.90, 0.5980], [2.00, 0.5797], [2.20, 0.5444],
    [2.40, 0.5115], [2.60, 0.4822], [2.80, 0.4566], [3.00, 0.4331],
    [3.50, 0.3859], [4.00, 0.3512], [4.50, 0.3269], [5.00, 0.3086],
])

_TABLES = {"G7": _G7, "G1": _G1}


def cd(mach: float, model: str = "G7") -> float:
    """Reference drag coefficient at a given Mach number (linear interpolation)."""
    t = _TABLES[model.upper()]
    return float(np.interp(mach, t[:, 0], t[:, 1]))


def cd_array(mach, model: str = "G7"):
    """Vectorized Cd lookup."""
    t = _TABLES[model.upper()]
    return np.interp(mach, t[:, 0], t[:, 1])


# Standard-projectile reference values used to convert a BC into a retardation.
# For the BC formulation we use the classic relation:
#   i (form factor) = Cd_bullet / Cd_ref,  BC = SD / i
# The integrator uses BC directly (see solver.drag_accel), so the only reference
# quantity needed there is Cd_ref(M) above. These constants are exposed for
# users who want to reconstruct form factors from measured Cd.
G7_REF_DIAMETER_IN = 0.284   # reference caliber the G7 SD is normalized to
