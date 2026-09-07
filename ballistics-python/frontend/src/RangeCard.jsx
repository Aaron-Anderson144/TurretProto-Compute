import React from "react";
import { machBand } from "./api.js";

// Show every 100-yd line by default so the table reads like a real dope card,
// even though the solver samples finer for the plot.
export default function RangeCard({ path, targetRange, onPick, unit }) {
  const rows = path.filter(
    (p) => Math.round(p.range_yd) % 100 === 0 || p.range_yd === path[0].range_yd
  );

  const nearest = path.reduce((a, b) =>
    Math.abs(b.range_yd - targetRange) < Math.abs(a.range_yd - targetRange) ? b : a
  );

  const dropCol = unit === "mil" ? "drop_mil" : "drop_moa";
  const windCol = unit === "mil" ? "wind_mil" : "wind_moa";

  return (
    <div className="card-wrap">
      <table className="card">
        <thead>
          <tr>
            <th>Range</th>
            <th>Drop {unit}</th>
            <th>Wind {unit}</th>
            <th>Spin</th>
            <th>Vel</th>
            <th>Mach</th>
            <th>Energy</th>
            <th>TOF</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((p) => {
            const band = machBand(p.mach);
            const active = Math.round(p.range_yd) === Math.round(nearest.range_yd);
            return (
              <tr
                key={p.range_yd}
                className={`band-${band} ${active ? "active" : ""}`}
                onClick={() => onPick(p.range_yd)}
              >
                <td>{p.range_yd.toFixed(0)}</td>
                <td>{p[dropCol].toFixed(2)}</td>
                <td>{p[windCol].toFixed(2)}</td>
                <td>{p.spin_drift_in.toFixed(1)}"</td>
                <td>{p.velocity_fps.toFixed(0)}</td>
                <td className={`mach t-${band}`}>{p.mach.toFixed(2)}</td>
                <td>{p.energy_ftlb.toFixed(0)}</td>
                <td>{p.tof_s.toFixed(2)}</td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}
