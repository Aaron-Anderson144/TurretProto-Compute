const BASE = "/api";

export async function fetchBullets() {
  const r = await fetch(`${BASE}/bullets`);
  if (!r.ok) throw new Error("bullets");
  return (await r.json()).bullets;
}

export async function solve(params) {
  const r = await fetch(`${BASE}/solve`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(params),
  });
  if (!r.ok) throw new Error(`solve ${r.status}`);
  return await r.json();
}

// Mach-regime classifier shared across the UI.
export function machBand(mach) {
  if (mach >= 1.2) return "super";
  if (mach >= 1.0) return "trans";
  return "sub";
}
