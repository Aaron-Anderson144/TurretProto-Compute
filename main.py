"""
FastAPI service for the .338 Norma Magnum ballistic solver.

Inputs use shooter-friendly units (fps, yards, inches, degF, inHg, %, mph);
conversion to SI happens at this boundary. Everything below is SI.

Run:  uvicorn main:app --reload --port 8000
"""

from typing import Optional, List
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from ballistics import Shot, Atmosphere, solve, bullets
from ballistics.solver import Bullet
from ballistics.stability import true_bc

app = FastAPI(title="338 NM Ballistic Solver", version="1.0.0")

# Vite dev server + local use
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# unit conversions at the boundary
F_TO_C = lambda f: (f - 32.0) * 5.0 / 9.0
INHG_TO_PA = 3386.389


class BulletIn(BaseModel):
    name: Optional[str] = None
    mass_grains: Optional[float] = None
    diameter_in: float = 0.338
    length_in: Optional[float] = None
    bc_g7: Optional[float] = None
    twist_in: float = 9.35
    right_twist: bool = True


class SolveRequest(BaseModel):
    bullet: BulletIn
    muzzle_velocity_fps: float = 2750.0
    zero_range_yd: float = 100.0
    max_range_yd: float = 1600.0
    increment_yd: float = 25.0
    sight_height_in: float = 2.0
    # environment
    azimuth_deg: float = 0.0
    latitude_deg: float = 40.0
    look_angle_deg: float = 0.0
    wind_speed_mph: float = 0.0
    wind_from_clock: float = 3.0
    temp_f: float = 59.0
    pressure_inhg: float = 29.92
    humidity_pct: float = 50.0


class TrueRequest(BaseModel):
    bc_g7: float
    predicted_drop_moa: float
    observed_drop_moa: float


def _build_bullet(b: BulletIn) -> Bullet:
    if b.name:
        try:
            base = bullets.get_bullet(b.name)
        except KeyError:
            raise HTTPException(404, f"Unknown bullet '{b.name}'")
        # allow overriding any field the client sent
        return Bullet(
            name=base.name,
            mass_grains=b.mass_grains or base.mass_grains,
            diameter_in=b.diameter_in or base.diameter_in,
            length_in=b.length_in or base.length_in,
            bc_g7=b.bc_g7 or base.bc_g7,
            twist_in=b.twist_in or base.twist_in,
            right_twist=b.right_twist,
        )
    # fully custom bullet
    missing = [k for k in ("mass_grains", "length_in", "bc_g7")
               if getattr(b, k) is None]
    if missing:
        raise HTTPException(422, f"Custom bullet missing fields: {missing}")
    return Bullet(
        name="Custom", mass_grains=b.mass_grains, diameter_in=b.diameter_in,
        length_in=b.length_in, bc_g7=b.bc_g7, twist_in=b.twist_in,
        right_twist=b.right_twist,
    )


@app.get("/api/health")
def health():
    return {"status": "ok"}


@app.get("/api/bullets")
def get_bullets():
    return {"bullets": bullets.list_bullets()}


@app.post("/api/solve")
def post_solve(req: SolveRequest):
    bullet = _build_bullet(req.bullet)

    atmo = Atmosphere(
        temp_c=F_TO_C(req.temp_f),
        pressure_pa=req.pressure_inhg * INHG_TO_PA,
        humidity=max(0.0, min(1.0, req.humidity_pct / 100.0)),
    )
    shot = Shot(
        muzzle_velocity_fps=req.muzzle_velocity_fps,
        azimuth_deg=req.azimuth_deg,
        latitude_deg=req.latitude_deg,
        look_angle_deg=req.look_angle_deg,
        wind_speed_mph=req.wind_speed_mph,
        wind_from_clock=req.wind_from_clock,
        sight_height_in=req.sight_height_in,
        zero_range_yd=req.zero_range_yd,
        max_range_yd=req.max_range_yd,
    )

    step = max(5.0, req.increment_yd)
    n = int(req.max_range_yd // step)
    ranges = [step * i for i in range(1, n + 1)]

    bore_deg, sg, rows = solve(bullet, shot, atmo, ranges_yd=ranges)

    # supersonic range = last range still above Mach 1
    supersonic_limit_yd = None
    for r in rows:
        if r.supersonic:
            supersonic_limit_yd = r.range_yd

    return {
        "meta": {
            "bore_angle_deg": round(bore_deg, 4),
            "stability_sg": round(sg, 3),
            "air_density_kgm3": round(atmo.density, 4),
            "mach1_ms": round(atmo.mach1, 2),
            "supersonic_limit_yd": supersonic_limit_yd,
            "bullet": bullet.name,
            "form_factor_g7": round(bullet.form_factor_g7, 4),
        },
        "path": [_row_dict(r) for r in rows],
    }


@app.post("/api/true")
def post_true(req: TrueRequest):
    """Nudge BC to match observed drop. Iterate solve->true a couple times."""
    new_bc = true_bc(req.bc_g7, req.predicted_drop_moa, req.observed_drop_moa)
    return {"bc_g7": round(new_bc, 4)}


def _row_dict(r):
    return {
        "range_yd": round(r.range_yd, 1),
        "range_m": round(r.range_m, 2),
        "drop_mil": round(r.drop_mil, 3),
        "drop_moa": round(r.drop_moa, 3),
        "drop_in": round(r.drop_m / 0.0254, 2),
        "wind_mil": round(r.wind_mil, 3),
        "wind_moa": round(r.wind_moa, 3),
        "wind_in": round(r.wind_m / 0.0254, 2),
        "spin_drift_in": round(r.spin_drift_m / 0.0254, 2),
        "coriolis_in": round(r.coriolis_m / 0.0254, 2),
        "velocity_fps": round(r.velocity_fps, 1),
        "velocity_ms": round(r.velocity_ms, 1),
        "mach": round(r.mach, 3),
        "energy_ftlb": round(r.energy_ftlb, 0),
        "energy_j": round(r.energy_j, 0),
        "tof_s": round(r.tof_s, 3),
        "supersonic": r.supersonic,
    }
