"""
Trajectory solver for the .338 Norma Magnum (and any bullet given a G7 BC).

Model: numerical point-mass integration (RK4) using the FULL G7 drag curve
(not a single averaged BC), with gravity, wind, and Coriolis integrated directly,
plus physically-based spin-drift (Litz) and gyroscopic stability (Miller) applied
as corrections. This is the same practical architecture as Applied-Ballistics-
style solvers; it is NOT a full 6-DOF rigid-body solve (that needs pitch/yaw
aero coefficients manufacturers don't publish). Accurate to well inside aiming
error through the supersonic and near-transonic band once trued.

Frames: East-North-Up (ENU), SI units internally. Azimuth is degrees clockwise
from North; the bore's horizontal direction. Windage is reported as the
cross-range (right-positive) component; elevation as vertical drop below the
line of sight. Unit conversion happens only at the API boundary.
"""

import math
from dataclasses import dataclass, field
import numpy as np

from .atmosphere import Atmosphere
from .drag_tables import cd as cd_ref
from .stability import miller_stability, spin_drift

# --- physical constants ---
G = 9.80665
OMEGA = 7.2921159e-5          # Earth's sidereal rotation rate, rad/s

# --- unit conversions ---
GRAIN_TO_KG = 6.47989e-5
IN_TO_M = 0.0254
FPS_TO_MS = 0.3048
YARD_TO_M = 0.9144
MPH_TO_MS = 0.44704

RAD_TO_MIL = 1000.0
RAD_TO_MOA = (180.0 / math.pi) * 60.0
J_TO_FTLB = 0.737562


@dataclass
class Bullet:
    name: str
    mass_grains: float
    diameter_in: float
    length_in: float
    bc_g7: float
    twist_in: float = 9.35          # 1:9.35" typical for .338 NM
    right_twist: bool = True

    @property
    def mass_kg(self) -> float:
        return self.mass_grains * GRAIN_TO_KG

    @property
    def area_m2(self) -> float:
        d = self.diameter_in * IN_TO_M
        return math.pi * 0.25 * d * d

    @property
    def form_factor_g7(self) -> float:
        # i = SD / BC, with sectional density in lb/in^2 (BC's native units)
        sd = (self.mass_grains / 7000.0) / (self.diameter_in ** 2)
        return sd / self.bc_g7


@dataclass
class Shot:
    muzzle_velocity_fps: float = 2750.0
    azimuth_deg: float = 0.0             # bearing of fire, cw from North
    latitude_deg: float = 40.0
    look_angle_deg: float = 0.0          # incline of target line (+ up)
    wind_speed_mph: float = 0.0
    wind_from_clock: float = 3.0         # o'clock the wind blows FROM (3 = right)
    sight_height_in: float = 2.0
    zero_range_yd: float = 100.0
    max_range_yd: float = 1600.0
    dt: float = 0.001


@dataclass
class RangeRow:
    range_m: float
    range_yd: float
    drop_m: float
    drop_mil: float
    drop_moa: float
    wind_m: float
    wind_mil: float
    wind_moa: float
    spin_drift_m: float
    coriolis_m: float
    velocity_ms: float
    velocity_fps: float
    mach: float
    energy_j: float
    energy_ftlb: float
    tof_s: float
    supersonic: bool


def _wind_vector_enu(shot: Shot):
    """Horizontal wind velocity vector (m/s) in ENU. Clock is direction wind
    blows FROM relative to line of fire; 12 = headwind, 3 = from right."""
    az = math.radians(shot.azimuth_deg)
    h_hat = np.array([math.sin(az), math.cos(az), 0.0])          # downrange
    r_hat = np.array([math.cos(az), -math.sin(az), 0.0])         # right
    alpha = math.radians(shot.wind_from_clock / 12.0 * 360.0)    # from-angle vs downrange
    speed = shot.wind_speed_mph * MPH_TO_MS
    # wind travels toward the opposite of where it comes from
    return -speed * (math.cos(alpha) * h_hat + math.sin(alpha) * r_hat)


def _accel(vel, atmo: Atmosphere, bullet: Bullet, wind, omega_vec):
    """Total acceleration in ENU (drag + gravity + Coriolis)."""
    v_rel = vel - wind
    speed = math.sqrt(v_rel[0] ** 2 + v_rel[1] ** 2 + v_rel[2] ** 2)
    if speed < 1e-6:
        a = np.zeros(3)
    else:
        mach = speed / atmo.mach1
        cd_bullet = bullet.form_factor_g7 * cd_ref(mach)
        k = 0.5 * atmo.density * cd_bullet * bullet.area_m2 / bullet.mass_kg
        a = -k * speed * v_rel                    # drag opposes relative velocity
    a[2] -= G                                     # gravity (Up is +)
    if omega_vec is not None:
        a = a - 2.0 * np.cross(omega_vec, vel)    # Coriolis
    return a


def _integrate(bullet: Bullet, shot: Shot, atmo: Atmosphere, bore_angle_rad: float,
               use_wind: bool, use_coriolis: bool):
    """RK4 integrate one trajectory. Returns arrays of downrange, vertical,
    windage, speed, time until max range or ground impact."""
    az = math.radians(shot.azimuth_deg)
    h_hat = np.array([math.sin(az), math.cos(az), 0.0])
    r_hat = np.array([math.cos(az), -math.sin(az), 0.0])
    up = np.array([0.0, 0.0, 1.0])

    v0 = shot.muzzle_velocity_fps * FPS_TO_MS
    # bore elevated by bore_angle_rad in the vertical plane containing h_hat
    vel = v0 * (math.cos(bore_angle_rad) * h_hat + math.sin(bore_angle_rad) * up)
    # start at bore; scope sits sight_height above -> LOS reference is +H above bore.
    pos = np.zeros(3)

    wind = _wind_vector_enu(shot) if use_wind else np.zeros(3)
    if use_coriolis:
        lat = math.radians(shot.latitude_deg)
        omega_vec = OMEGA * np.array([0.0, math.cos(lat), math.sin(lat)])
    else:
        omega_vec = None

    max_range_m = shot.max_range_yd * YARD_TO_M
    dt = shot.dt
    d_list, y_list, w_list, s_list, t_list = [], [], [], [], []

    t = 0.0
    steps = 0
    max_steps = 200000
    while steps < max_steps:
        d = float(np.dot(pos, h_hat))
        y = float(np.dot(pos, up))
        w = float(np.dot(pos, r_hat))
        s = float(math.sqrt(vel @ vel))
        d_list.append(d); y_list.append(y); w_list.append(w)
        s_list.append(s); t_list.append(t)

        if d >= max_range_m:
            break
        # stop only if the round has truly plunged (spent); at long range the
        # bullet is legitimately tens of meters below the sight line in flight.
        if y < -500.0 and t > 0.05:
            break

        k1v = _accel(vel, atmo, bullet, wind, omega_vec)
        k1x = vel
        k2v = _accel(vel + 0.5 * dt * k1v, atmo, bullet, wind, omega_vec)
        k2x = vel + 0.5 * dt * k1v
        k3v = _accel(vel + 0.5 * dt * k2v, atmo, bullet, wind, omega_vec)
        k3x = vel + 0.5 * dt * k2v
        k4v = _accel(vel + dt * k3v, atmo, bullet, wind, omega_vec)
        k4x = vel + dt * k3v

        vel = vel + (dt / 6.0) * (k1v + 2 * k2v + 2 * k3v + k4v)
        pos = pos + (dt / 6.0) * (k1x + 2 * k2x + 2 * k3x + k4x)
        t += dt
        steps += 1

    return (np.array(d_list), np.array(y_list), np.array(w_list),
            np.array(s_list), np.array(t_list))


def _solve_bore_angle(bullet: Bullet, shot: Shot, atmo: Atmosphere) -> float:
    """Find bore elevation that puts the bullet on the line of sight at the zero
    range. Zeroed in still air (no wind/Coriolis), as one does at the range.
    The LOS is H_sight above the bore at the muzzle."""
    zero_m = shot.zero_range_yd * YARD_TO_M
    h_sight = shot.sight_height_in * IN_TO_M

    def height_error(angle):
        d, y, _, _, _ = _integrate(bullet, shot, atmo, angle,
                                   use_wind=False, use_coriolis=False)
        # bullet vertical vs LOS: LOS is at +h_sight above bore origin, level.
        y_at_zero = float(np.interp(zero_m, d, y))
        return y_at_zero - h_sight

    # secant iteration; bracket around a small positive angle
    a0, a1 = 0.0, math.radians(0.5)
    f0, f1 = height_error(a0), height_error(a1)
    for _ in range(60):
        if abs(f1 - f0) < 1e-12:
            break
        a2 = a1 - f1 * (a1 - a0) / (f1 - f0)
        a0, f0 = a1, f1
        a1 = a2
        f1 = height_error(a1)
        if abs(f1) < 1e-5:
            break
    return a1


def solve(bullet: Bullet, shot: Shot, atmo: Atmosphere, ranges_yd=None):
    """Full solution. Returns (bore_angle_deg, sg, list[RangeRow])."""
    h_sight = shot.sight_height_in * IN_TO_M
    look = math.radians(shot.look_angle_deg)

    bore_angle = _solve_bore_angle(bullet, shot, atmo)

    # Full trajectory with wind + Coriolis, and a wind-only run to isolate Coriolis.
    d, y, w, s, t = _integrate(bullet, shot, atmo, bore_angle,
                               use_wind=True, use_coriolis=True)
    d2, y2, w2, s2, t2 = _integrate(bullet, shot, atmo, bore_angle,
                                    use_wind=True, use_coriolis=False)

    if ranges_yd is None:
        step = 100.0
        n = int(shot.max_range_yd // step)
        ranges_yd = [step * i for i in range(1, n + 1)]

    v0 = shot.muzzle_velocity_fps
    sg = miller_stability(bullet.mass_grains, bullet.diameter_in, bullet.length_in,
                          bullet.twist_in, v0, atmo.temp_c, atmo.pressure_pa,
                          atmo.humidity)

    rows = []
    for rg_yd in ranges_yd:
        rg_m = rg_yd * YARD_TO_M
        if rg_m > d[-1] + 1e-6:
            break
        y_r = float(np.interp(rg_m, d, y))
        w_full = float(np.interp(rg_m, d, w))
        w_nocor = float(np.interp(rg_m, d2, w2))
        y_nocor = float(np.interp(rg_m, d2, y2))
        spd = float(np.interp(rg_m, d, s))
        tof = float(np.interp(rg_m, d, t))

        # Line of sight height at this range (level or inclined)
        los_y = h_sight + rg_m * math.tan(look)
        drop = los_y - y_r                       # + = bullet below LOS (dial up)

        # windage components
        coriolis_w = w_full - w_nocor            # horizontal Coriolis part
        sd = spin_drift(sg, tof, bullet.right_twist)
        wind_total = w_full + sd                 # spin drift added on top

        # slant range for angular conversion under incline
        slant = rg_m / max(math.cos(look), 1e-6)
        drop_ang = math.atan2(drop, slant)
        wind_ang = math.atan2(wind_total, slant)

        mach = spd / atmo.mach1
        energy = 0.5 * bullet.mass_kg * spd * spd

        rows.append(RangeRow(
            range_m=rg_m, range_yd=rg_yd,
            drop_m=drop, drop_mil=drop_ang * RAD_TO_MIL, drop_moa=drop_ang * RAD_TO_MOA,
            wind_m=wind_total, wind_mil=wind_ang * RAD_TO_MIL, wind_moa=wind_ang * RAD_TO_MOA,
            spin_drift_m=sd, coriolis_m=coriolis_w,
            velocity_ms=spd, velocity_fps=spd / FPS_TO_MS,
            mach=mach, energy_j=energy, energy_ftlb=energy * J_TO_FTLB,
            tof_s=tof, supersonic=mach > 1.0,
        ))

    return math.degrees(bore_angle), sg, rows
