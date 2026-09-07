import React, { useEffect, useMemo, useRef, useState } from "react";
import { fetchBullets, solve } from "./api.js";
import ControlPanel from "./ControlPanel.jsx";
import TrajectoryHUD from "./TrajectoryHUD.jsx";
import RangeCard from "./RangeCard.jsx";

const DEFAULTS = {
  bullet: {
    name: "Berger 300gr Hybrid OTM",
    mass_grains: 300,
    diameter_in: 0.338,
    length_in: 1.8,
    bc_g7: 0.379,
    twist_in: 9.35,
    right_twist: true,
  },
  muzzle_velocity_fps: 2750,
  zero_range_yd: 100,
  max_range_yd: 1600,
  increment_yd: 25,
  sight_height_in: 2.0,
  azimuth_deg: 90,
  latitude_deg: 40,
  look_angle_deg: 0,
  wind_speed_mph: 10,
  wind_from_clock: 3,
  temp_f: 59,
  pressure_inhg: 29.92,
  humidity_pct: 50,
};

const finite = (x) => typeof x === "number" && Number.isFinite(x);

export default function App() {
  const [params, setParams] = useState(DEFAULTS);
  const [bulletList, setBulletList] = useState([]);
  const [result, setResult] = useState(null);
  const [target, setTarget] = useState(1000);
  const [unit, setUnit] = useState("mil");
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState(null);
  const timer = useRef(null);

  useEffect(() => {
    fetchBullets().then(setBulletList).catch(() => {});
  }, []);

  // Debounced real-time solve on any input change.
  useEffect(() => {
    const b = params.bullet;
    const ok =
      finite(params.muzzle_velocity_fps) &&
      finite(params.max_range_yd) &&
      params.max_range_yd > 50 &&
      finite(b.bc_g7) &&
      finite(b.mass_grains) &&
      finite(b.length_in);
    if (!ok) return;

    clearTimeout(timer.current);
    timer.current = setTimeout(async () => {
      setBusy(true);
      setErr(null);
      try {
        setResult(await solve(params));
      } catch (e) {
        setErr(e.message);
      } finally {
        setBusy(false);
      }
    }, 200);
    return () => clearTimeout(timer.current);
  }, [params]);

  // keep target range within the solved envelope
  useEffect(() => {
    if (target > params.max_range_yd) setTarget(params.max_range_yd);
  }, [params.max_range_yd]); // eslint-disable-line

  const path = result?.path || [];
  const meta = result?.meta;

  const cur = useMemo(() => {
    if (!path.length) return null;
    return path.reduce((a, b) =>
      Math.abs(b.range_yd - target) < Math.abs(a.range_yd - target) ? b : a
    );
  }, [path, target]);

  const sgClass =
    meta && meta.stability_sg >= 1.5
      ? "super"
      : meta && meta.stability_sg >= 1.3
      ? "trans"
      : "sub";

  return (
    <div className="app">
      <div className="topbar">
        <div className="brand">
          <div className="cal">
            .338<em>NM</em>
          </div>
          <div className="sub">Exterior Ballistic Solver · G7 · Point-Mass RK4</div>
        </div>
        <div className="status-pill">
          <span className={`dot ${busy ? "busy" : ""}`} />
          {err ? "solver error" : busy ? "solving" : "solution current"}
        </div>
      </div>

      <div className="grid">
        <ControlPanel params={params} set={setParams} bulletList={bulletList} />

        <div>
          {/* key readouts */}
          <div className="readouts">
            <div className="readout">
              <div className="k">Stability SG</div>
              <div className={`v ${sgClass}`}>
                {meta ? meta.stability_sg.toFixed(2) : "—"}
              </div>
            </div>
            <div className="readout">
              <div className="k">Supersonic to</div>
              <div className="v cyan">
                {meta?.supersonic_limit_yd ?? "—"}
                <small>yd</small>
              </div>
            </div>
            <div className="readout">
              <div className="k">Bore angle</div>
              <div className="v">
                {meta ? meta.bore_angle_deg.toFixed(3) : "—"}
                <small>°</small>
              </div>
            </div>
            <div className="readout">
              <div className="k">Air density</div>
              <div className="v">
                {meta ? meta.air_density_kgm3.toFixed(3) : "—"}
                <small>kg/m³</small>
              </div>
            </div>
            <div className="readout">
              <div className="k">Mach 1</div>
              <div className="v">
                {meta ? Math.round(meta.mach1_ms / 0.3048) : "—"}
                <small>fps</small>
              </div>
            </div>
          </div>

          {/* signature: trajectory HUD */}
          <TrajectoryHUD
            path={path}
            targetRange={target}
            maxRange={params.max_range_yd}
            onScrub={setTarget}
          />

          {/* target-range firing solution */}
          <div className="range-scrub">
            <input
              type="range"
              min={Math.min(100, params.max_range_yd)}
              max={params.max_range_yd}
              step={5}
              value={target}
              onChange={(e) => setTarget(parseFloat(e.target.value))}
            />
            <div className="rv">{target} yd</div>
          </div>

          <div className="solution">
            <div className="sol-cell hero">
              <div className="k">Elevation · come-up</div>
              <div className="big">
                {cur ? (unit === "mil" ? cur.drop_mil.toFixed(2) : cur.drop_moa.toFixed(2)) : "—"}
                <small> {unit}</small>
              </div>
              <div className="alt">
                {cur
                  ? unit === "mil"
                    ? `${cur.drop_moa.toFixed(2)} moa · ${cur.drop_in.toFixed(0)} in`
                    : `${cur.drop_mil.toFixed(2)} mil · ${cur.drop_in.toFixed(0)} in`
                  : ""}
              </div>
            </div>
            <div className="sol-cell">
              <div className="k">Windage {cur && cur.wind_mil >= 0 ? "· R" : "· L"}</div>
              <div className="big wind">
                {cur ? (unit === "mil" ? Math.abs(cur.wind_mil).toFixed(2) : Math.abs(cur.wind_moa).toFixed(2)) : "—"}
                <small> {unit}</small>
              </div>
              <div className="alt">
                {cur ? `${Math.abs(cur.wind_in).toFixed(1)} in · spin ${cur.spin_drift_in.toFixed(1)}"` : ""}
              </div>
            </div>
            <div className="sol-cell">
              <div className="k">Terminal @ range</div>
              <div className="big wind" style={{ fontSize: 22 }}>
                {cur ? cur.velocity_fps.toFixed(0) : "—"}
                <small> fps · M{cur ? cur.mach.toFixed(2) : "—"}</small>
              </div>
              <div className="alt">
                {cur ? `${cur.energy_ftlb.toFixed(0)} ftlb · ${cur.tof_s.toFixed(2)} s tof` : ""}
              </div>
            </div>
          </div>

          {/* dope table */}
          <div className="panel">
            <div className="panel-hd">
              <span>Range Card</span>
              <span>
                <button
                  className="btn"
                  onClick={() => setUnit((u) => (u === "mil" ? "moa" : "mil"))}
                >
                  {unit === "mil" ? "MIL" : "MOA"}
                </button>
              </span>
            </div>
            <div className="panel-bd">
              {path.length ? (
                <RangeCard path={path} targetRange={target} onPick={setTarget} unit={unit} />
              ) : (
                <div className="note">{err ? `Error: ${err}` : "Awaiting solution…"}</div>
              )}
              <div className="note">
                <b>Model:</b> point-mass RK4 over the full G7 drag curve · humid-air
                density (Buck/CIPM) · Miller stability · Litz spin drift · Coriolis +
                Eötvös. <b>Not 6-DOF.</b> BCs are representative —{" "}
                <b>true to your observed drop</b> before relying on dope. Aerodynamic
                jump from crosswind is not modeled.
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
