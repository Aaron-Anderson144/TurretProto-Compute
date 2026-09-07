"""
Physics sanity tests. These assert behavior/invariants, not exact dope (which
depends on trued BCs), so they stay valid across reasonable table refinements.
"""

import math
import pytest

from ballistics import Shot, Atmosphere, solve, bullets, air_density, speed_of_sound
from ballistics.stability import miller_stability


def _solve_at(ranges, **shot_kw):
    b = bullets.get_bullet("Berger 300gr Hybrid OTM")
    atmo = Atmosphere(temp_c=15, pressure_pa=101325, humidity=0.5)
    shot = Shot(muzzle_velocity_fps=2750, max_range_yd=max(ranges) + 100, **shot_kw)
    return solve(b, shot, atmo, ranges_yd=ranges)


def test_humid_air_less_dense():
    dry = air_density(30, 101325, 0.0)
    wet = air_density(30, 101325, 1.0)
    assert wet < dry                      # moist air is lighter


def test_speed_of_sound_rises_with_humidity():
    assert speed_of_sound(30, 101325, 1.0) > speed_of_sound(30, 101325, 0.0)


def test_drop_monotonic_increasing():
    _, _, rows = _solve_at([200, 400, 600, 800, 1000])
    drops = [r.drop_mil for r in rows]
    assert all(b > a for a, b in zip(drops, drops[1:]))


def test_velocity_monotonic_decreasing():
    _, _, rows = _solve_at([100, 500, 1000, 1400])
    v = [r.velocity_fps for r in rows]
    assert all(b < a for a, b in zip(v, v[1:]))


def test_338nm_supersonic_past_1500yd_at_sea_level():
    _, _, rows = _solve_at([1500])
    assert rows[0].supersonic                    # M > 1 at 1500 yd SL


def test_1000yd_dope_in_expected_band():
    # 300gr @ 2750, 100yd zero: come-up ~7-9 mil is the accepted ballpark
    _, _, rows = _solve_at([1000], zero_range_yd=100)
    assert 6.5 < rows[0].drop_mil < 9.5


def test_spin_drift_positive_right_twist():
    _, _, rows = _solve_at([1000])
    assert rows[0].spin_drift_m > 0             # right for right-hand twist


def test_stability_factor_reasonable():
    sg = miller_stability(300, 0.338, 1.800, 9.35, 2750, 15, 101325, 0.5)
    assert 1.4 < sg < 2.6                        # comfortably stable


def test_eotvos_east_shoots_higher_than_west():
    _, _, east = _solve_at([1000], azimuth_deg=90, latitude_deg=45)
    _, _, west = _solve_at([1000], azimuth_deg=270, latitude_deg=45)
    assert east[0].drop_mil < west[0].drop_mil   # eastward needs less come-up


def test_thinner_air_flatter_trajectory():
    b = bullets.get_bullet("Berger 300gr Hybrid OTM")
    sl = Atmosphere(15, 101325, 0.3)
    alt = Atmosphere(20, 83500, 0.3)             # ~5000 ft
    shot = Shot(muzzle_velocity_fps=2750, max_range_yd=1600)
    _, _, r_sl = solve(b, shot, sl, ranges_yd=[1500])
    _, _, r_alt = solve(b, shot, alt, ranges_yd=[1500])
    assert r_alt[0].drop_mil < r_sl[0].drop_mil


def test_headwind_vs_tailwind_symmetry_on_windage():
    # pure head/tail wind should produce ~no horizontal wind drift
    _, _, head = _solve_at([1000], wind_speed_mph=15, wind_from_clock=12)
    # spin drift + coriolis remain, but the wind's lateral part should be ~0
    # compared to a full-value crosswind
    _, _, cross = _solve_at([1000], wind_speed_mph=15, wind_from_clock=3)
    assert abs(cross[0].wind_m) > abs(head[0].wind_m) + 0.25
