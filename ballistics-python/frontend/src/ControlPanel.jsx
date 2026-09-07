import React from "react";

// A single labeled numeric field with an optional unit suffix.
function Field({ label, value, onChange, unit, step = "any", min, max, width }) {
  return (
    <div className="field">
      <label>{label}</label>
      <div className="input-wrap">
        <input
          type="number"
          className={unit ? "has-unit" : ""}
          value={value}
          step={step}
          min={min}
          max={max}
          style={width ? { width } : undefined}
          onChange={(e) =>
            onChange(e.target.value === "" ? "" : parseFloat(e.target.value))
          }
        />
        {unit && <span className="unit">{unit}</span>}
      </div>
    </div>
  );
}

export default function ControlPanel({ params, set, bulletList }) {
  const onBullet = (name) => {
    const b = bulletList.find((x) => x.name === name);
    // adopt the load's specs + typical MV, but leave the rest of the shot alone
    set((p) => ({
      ...p,
      bullet: {
        name: b.name,
        mass_grains: b.mass_grains,
        diameter_in: b.diameter_in,
        length_in: b.length_in,
        bc_g7: b.bc_g7,
        twist_in: b.twist_in,
        right_twist: true,
      },
      muzzle_velocity_fps: b.typical_mv_fps,
    }));
  };

  const b = params.bullet;
  const setBullet = (patch) =>
    set((p) => ({ ...p, bullet: { ...p.bullet, ...patch } }));

  return (
    <div className="panel">
      <div className="panel-hd">
        <span>Firing Setup</span>
        <span className="idx">.338 NM</span>
      </div>
      <div className="panel-bd">
        <div className="group">
          <div className="group-label">Projectile · G7</div>
          <div className="field full">
            <label>Load</label>
            <select
              value={b.name || ""}
              onChange={(e) => onBullet(e.target.value)}
            >
              {bulletList.map((x) => (
                <option key={x.name} value={x.name}>
                  {x.name}
                </option>
              ))}
            </select>
          </div>
          <Field label="Mass" value={b.mass_grains} unit="gr"
            onChange={(v) => setBullet({ mass_grains: v })} />
          <Field label="G7 BC" value={b.bc_g7} step="0.001"
            onChange={(v) => setBullet({ bc_g7: v })} />
          <Field label="Length" value={b.length_in} unit="in" step="0.001"
            onChange={(v) => setBullet({ length_in: v })} />
          <Field label="Twist 1:" value={b.twist_in} unit="in" step="0.05"
            onChange={(v) => setBullet({ twist_in: v })} />
        </div>

        <div className="group">
          <div className="group-label">Rifle · Zero</div>
          <Field label="Muzzle vel" value={params.muzzle_velocity_fps} unit="fps"
            onChange={(v) => set((p) => ({ ...p, muzzle_velocity_fps: v }))} />
          <Field label="Zero range" value={params.zero_range_yd} unit="yd"
            onChange={(v) => set((p) => ({ ...p, zero_range_yd: v }))} />
          <Field label="Sight ht" value={params.sight_height_in} unit="in" step="0.1"
            onChange={(v) => set((p) => ({ ...p, sight_height_in: v }))} />
          <Field label="Max range" value={params.max_range_yd} unit="yd"
            onChange={(v) => set((p) => ({ ...p, max_range_yd: v }))} />
          <Field label="Increment" value={params.increment_yd} unit="yd"
            onChange={(v) => set((p) => ({ ...p, increment_yd: v }))} />
        </div>

        <div className="group">
          <div className="group-label">Atmosphere</div>
          <Field label="Temp" value={params.temp_f} unit="°F"
            onChange={(v) => set((p) => ({ ...p, temp_f: v }))} />
          <Field label="Pressure" value={params.pressure_inhg} unit="inHg" step="0.01"
            onChange={(v) => set((p) => ({ ...p, pressure_inhg: v }))} />
          <Field label="Humidity" value={params.humidity_pct} unit="%"
            onChange={(v) => set((p) => ({ ...p, humidity_pct: v }))} />
        </div>

        <div className="group">
          <div className="group-label">Wind · Geometry</div>
          <Field label="Wind speed" value={params.wind_speed_mph} unit="mph"
            onChange={(v) => set((p) => ({ ...p, wind_speed_mph: v }))} />
          <Field label="Wind from" value={params.wind_from_clock} unit="o'clk" step="0.5" min="1" max="12"
            onChange={(v) => set((p) => ({ ...p, wind_from_clock: v }))} />
          <Field label="Azimuth" value={params.azimuth_deg} unit="°" min="0" max="360"
            onChange={(v) => set((p) => ({ ...p, azimuth_deg: v }))} />
          <Field label="Latitude" value={params.latitude_deg} unit="°" min="-90" max="90"
            onChange={(v) => set((p) => ({ ...p, latitude_deg: v }))} />
          <Field label="Look angle" value={params.look_angle_deg} unit="°" min="-60" max="60"
            onChange={(v) => set((p) => ({ ...p, look_angle_deg: v }))} />
        </div>
      </div>
    </div>
  );
}
