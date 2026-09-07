import React, { useRef, useCallback } from "react";

const W = 920,
  H = 400,
  ML = 46,
  MR = 52,
  MT = 24,
  MB = 40;
const PW = W - ML - MR;
const PH = H - MT - MB;

const regime = (m) => (m >= 1.2 ? "super" : m >= 1.0 ? "trans" : "sub");
const bandColor = {
  super: "rgba(79,209,160,0.055)",
  trans: "rgba(230,179,76,0.06)",
  sub: "rgba(229,107,87,0.07)",
};

export default function TrajectoryHUD({ path, targetRange, maxRange, onScrub }) {
  const svgRef = useRef(null);

  const handlePointer = useCallback(
    (e) => {
      if (e.buttons === 0 && e.type === "pointermove") return;
      const svg = svgRef.current;
      if (!svg) return;
      const rect = svg.getBoundingClientRect();
      const px = ((e.clientX - rect.left) / rect.width) * W;
      const frac = Math.max(0, Math.min(1, (px - ML) / PW));
      onScrub(Math.round((frac * maxRange) / 5) * 5);
    },
    [maxRange, onScrub]
  );

  if (!path || path.length < 2) {
    return (
      <div className="hud">
        <svg viewBox={`0 0 ${W} ${H}`} className="hud-svg">
          <text x={W / 2} y={H / 2} fill="#5b6672" fontFamily="JetBrains Mono" fontSize="13" textAnchor="middle">
            awaiting solution…
          </text>
        </svg>
      </div>
    );
  }

  const drops = path.map((p) => p.drop_mil);
  const vels = path.map((p) => p.velocity_fps);
  const dMin = Math.min(0, ...drops);
  const dMax = Math.max(...drops) * 1.04;
  const vMin = Math.min(...vels) * 0.96;
  const vMax = Math.max(...vels) * 1.02;

  const xS = (r) => ML + (r / maxRange) * PW;
  const yS = (d) => MT + ((d - dMin) / (dMax - dMin || 1)) * PH; // down = more drop
  const yV = (v) => MT + (1 - (v - vMin) / (vMax - vMin || 1)) * PH;

  // trajectory + velocity paths
  const arc = path.map((p, i) => `${i ? "L" : "M"}${xS(p.range_yd).toFixed(1)},${yS(p.drop_mil).toFixed(1)}`).join(" ");
  const fill = `${arc} L${xS(path[path.length - 1].range_yd).toFixed(1)},${(MT + PH).toFixed(1)} L${xS(0).toFixed(1)},${(MT + PH).toFixed(1)} Z`;
  const vLine = path.map((p, i) => `${i ? "L" : "M"}${xS(p.range_yd).toFixed(1)},${yV(p.velocity_fps).toFixed(1)}`).join(" ");

  // Mach-band boundaries along range
  const firstBelow = (thr) => {
    const p = path.find((q) => q.mach < thr);
    return p ? xS(p.range_yd) : ML + PW;
  };
  const xTrans = firstBelow(1.2);
  const xSub = firstBelow(1.0);
  const bands = [
    { x0: ML, x1: xTrans, c: bandColor.super },
    { x0: xTrans, x1: xSub, c: bandColor.trans },
    { x0: xSub, x1: ML + PW, c: bandColor.sub },
  ].filter((b) => b.x1 > b.x0 + 0.5);

  // cursor at target range
  const cur = path.reduce((a, b) => (Math.abs(b.range_yd - targetRange) < Math.abs(a.range_yd - targetRange) ? b : a));
  const cx = xS(cur.range_yd);
  const cy = yS(cur.drop_mil);
  const curColor = `var(--${regime(cur.mach)})`;

  // ticks
  const xStep = maxRange > 1400 ? 250 : maxRange > 700 ? 200 : 100;
  const xTicks = [];
  for (let r = xStep; r <= maxRange + 1; r += xStep) xTicks.push(r);
  const yTicks = 5;
  const yVals = Array.from({ length: yTicks }, (_, i) => dMin + ((dMax - dMin) * i) / (yTicks - 1));

  const corner = (x, y, sx, sy) =>
    `M${x + sx * 12},${y} L${x},${y} L${x},${y + sy * 12}`;

  return (
    <div className="hud">
      <svg
        ref={svgRef}
        viewBox={`0 0 ${W} ${H}`}
        className="hud-svg"
        onPointerDown={handlePointer}
        onPointerMove={handlePointer}
        style={{ touchAction: "none", cursor: "crosshair" }}
      >
        {/* Mach-regime background bands */}
        {bands.map((b, i) => (
          <rect key={i} x={b.x0} y={MT} width={b.x1 - b.x0} height={PH} fill={b.c} />
        ))}

        {/* grid */}
        {yVals.map((d, i) => (
          <g key={`y${i}`}>
            <line x1={ML} y1={yS(d)} x2={ML + PW} y2={yS(d)} stroke="#1e2732" strokeWidth="1" />
            <text x={ML - 8} y={yS(d) + 3} fill="#5b6672" fontFamily="JetBrains Mono" fontSize="10" textAnchor="end">
              {d.toFixed(1)}
            </text>
          </g>
        ))}
        {xTicks.map((r, i) => (
          <g key={`x${i}`}>
            <line x1={xS(r)} y1={MT} x2={xS(r)} y2={MT + PH} stroke="#1a222c" strokeWidth="1" />
            <text x={xS(r)} y={MT + PH + 15} fill="#5b6672" fontFamily="JetBrains Mono" fontSize="10" textAnchor="middle">
              {r}
            </text>
          </g>
        ))}

        {/* axis labels */}
        <text x={ML - 8} y={MT - 8} fill="#8b98a5" fontFamily="JetBrains Mono" fontSize="9.5" textAnchor="start" letterSpacing="1">
          DROP · MIL
        </text>
        <text x={ML + PW} y={MT - 8} fill="#2b90b3" fontFamily="JetBrains Mono" fontSize="9.5" textAnchor="end" letterSpacing="1">
          VEL · FPS
        </text>
        <text x={ML + PW} y={MT + PH + 15} fill="#8b98a5" fontFamily="JetBrains Mono" fontSize="9.5" textAnchor="end" letterSpacing="1">
          RANGE · YD
        </text>

        {/* velocity trace (secondary) */}
        <path d={vLine} fill="none" stroke="#2b90b3" strokeWidth="1.2" strokeDasharray="3 3" opacity="0.8" />

        {/* trajectory fill + line */}
        <path d={fill} fill="url(#arcfill)" opacity="0.5" />
        <path d={arc} fill="none" stroke="#57c4e5" strokeWidth="2" />
        <defs>
          <linearGradient id="arcfill" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#57c4e5" stopOpacity="0.16" />
            <stop offset="100%" stopColor="#57c4e5" stopOpacity="0" />
          </linearGradient>
        </defs>

        {/* cursor */}
        <line x1={cx} y1={MT} x2={cx} y2={MT + PH} stroke={curColor} strokeWidth="1" strokeDasharray="2 3" opacity="0.8" />
        <line x1={ML} y1={cy} x2={ML + PW} y2={cy} stroke={curColor} strokeWidth="0.6" strokeDasharray="2 4" opacity="0.4" />
        <circle cx={cx} cy={cy} r="5.5" fill="none" stroke={curColor} strokeWidth="1.5" />
        <circle cx={cx} cy={cy} r="1.6" fill={curColor} />
        <g fontFamily="JetBrains Mono" fontSize="11" fontWeight="600">
          <rect x={Math.min(cx + 8, W - 96)} y={cy - 30} width="88" height="22" fill="#0d1218" stroke="#33404f" rx="2" />
          <text x={Math.min(cx + 14, W - 90)} y={cy - 15} fill={curColor}>
            {cur.drop_mil.toFixed(2)} mil
          </text>
        </g>

        {/* HUD corner brackets */}
        <g stroke="#2b90b3" strokeWidth="1.4" fill="none" opacity="0.55">
          <path d={corner(ML - 6, MT - 6, 1, 1)} />
          <path d={corner(ML + PW + 6, MT - 6, -1, 1)} />
          <path d={corner(ML - 6, MT + PH + 6, 1, -1)} />
          <path d={corner(ML + PW + 6, MT + PH + 6, -1, -1)} />
        </g>
      </svg>
      <div className="legend">
        <span><i style={{ background: "var(--super)" }} /> Supersonic ≥ M1.2</span>
        <span><i style={{ background: "var(--trans)" }} /> Transonic M1.0–1.2</span>
        <span><i style={{ background: "var(--sub)" }} /> Subsonic &lt; M1.0</span>
        <span style={{ marginLeft: "auto", color: "#5b6672" }}>drag click to scrub range</span>
      </div>
    </div>
  );
}
