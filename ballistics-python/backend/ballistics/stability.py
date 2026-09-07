"""
Gyroscopic stability (Miller) and spin drift (Litz), plus a BC truing helper.

Spin drift is a real horizontal offset (to the right for right-hand twist) that a
point-mass model cannot produce on its own -- it comes from the bullet's yaw of
repose. Rather than run a full 6-DOF solve requiring pitch/yaw aerodynamic
coefficients that manufacturers don't publish, we use Miller's stability rule to
get SG and Litz's empirical spin-drift fit, which together match observed .338
drift to well within aiming error.
"""

import math
from .atmosphere import air_density, standard_density

MILLER_REF_VELOCITY_FPS = 2800.0


def miller_stability(
    mass_grains: float,
    diameter_in: float,
    length_in: float,
    twist_in: float,
    velocity_fps: float,
    temp_c: float,
    pressure_pa: float,
    humidity: float = 0.0,
) -> float:
    """
    Miller gyroscopic stability factor SG, corrected for muzzle velocity and the
    actual atmosphere. SG > 1.4 is comfortably stable; ~1.0-1.2 is marginal.

    twist_in : twist rate in inches per turn (e.g. 9.35 for 1:9.35")
    """
    t = twist_in / diameter_in            # twist in calibers per turn
    ell = length_in / diameter_in         # bullet length in calibers

    sg = (30.0 * mass_grains) / (
        t**2 * diameter_in**3 * ell * (1.0 + ell**2)
    )

    # Velocity correction (normalized to 2800 fps)
    sg *= (velocity_fps / MILLER_REF_VELOCITY_FPS) ** (1.0 / 3.0)

    # Atmosphere correction: denser air -> less stable. Miller's T/P form is
    # equivalent to the standard/actual density ratio.
    rho = air_density(temp_c, pressure_pa, humidity)
    sg *= standard_density() / rho
    return sg


def spin_drift(sg: float, tof_s: float, right_twist: bool = True) -> float:
    """
    Litz spin-drift approximation. Returns lateral offset in METERS.
    Positive = to the shooter's right (for right-hand twist).
    """
    drift_in = 1.25 * (sg + 1.2) * (tof_s ** 1.83)
    drift_m = drift_in * 0.0254
    return drift_m if right_twist else -drift_m


def true_bc(bc_g7: float, predicted_drop_m: float, observed_drop_m: float) -> float:
    """
    First-order BC truing. If the solver predicts `predicted_drop_m` at a range
    but you actually measured `observed_drop_m`, nudge BC by the drop ratio.
    Iterate 2-3 times in practice: solve -> true -> solve.

    Less drop than predicted -> bullet is slicker -> raise BC, and vice versa.
    """
    if observed_drop_m <= 0 or predicted_drop_m <= 0:
        return bc_g7
    # drop ~ 1/BC to first order: more observed drop -> more drag -> lower BC
    return bc_g7 * (predicted_drop_m / observed_drop_m)
