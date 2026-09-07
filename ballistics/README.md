# ballistics

Rust port of the .338 Norma exterior ballistics solver. Point mass, RK4, G7 drag, atmosphere with humidity, wind, Coriolis, latitude gravity.

```
crates/ballistics-core   the solver and the aiming solver. no_std with
                         --no-default-features (libm for math)
crates/ballistics-cli    `bal` - drop/drift table from the command line
crates/ballistics-wasm   wasm-bindgen wrapper, JSON in / JSON out, for the React app
```

## Run

```
cargo test --workspace                                   # 38 tests
cargo run --example aim_demo -p ballistics-core          # firing solutions and what they cost
cargo build -p ballistics-core --no-default-features     # proves the core is no_std clean
cargo run --release -p ballistics-cli -- --bc 0.42 --weight 300 --mv 2650 --wind 10 --lat 40 --azimuth 0
cargo run -p ballistics-cli -- --help
```

Add `--json` for the same table as JSON. The numbers above are placeholders, put your load in.

## Conventions

Frame: x downrange, y up, z right. SI inside the crate, BC in lb/in² because that's how it's published. `units.rs` converts at the edges.

- `path` is + above the line of sight, `drift` is + right. What you dial is the opposite sign.
- Wind is where it blows FROM. Clock: 12 head, 3 from the right, 6 tail, 9 from the left.
- Pressure is station (absolute) pressure like a Kestrel reads. If you only have the sea-level number from a weather report, `atmosphere::sea_level_to_station` converts it, or give `--altitude` and no `--pressure` to get ISA.
- BC is ICAO-referenced. `--asm` / `bc_reference: "asm"` converts an Army Standard Metro BC (×0.982).
- Zero is solved level, no wind, no Coriolis deflection, in the solver's atmosphere and at the shot's latitude gravity. `Solver::flight_with_zero` takes a zero angle you computed some other way.
- Range is distance along the line of sight, so uphill/downhill shots come out right without a rifleman's rule fudge.

## Core API

```rust
use ballistics_core::*;
use ballistics_core::units::*;

let solver = Solver::new(
    Projectile::new(0.42, grains_to_kg(300.0)),
    G7,
    Station { temperature: f_to_k(59.0), pressure: inhg_to_pa(29.92), humidity: 0.0 },
);
let shot = Shot {
    muzzle_velocity: fps_to_mps(2650.0),
    sight_height: in_to_m(2.0),
    zero_range: yd_to_m(100.0),
    wind: Wind::from_clock(mph_to_mps(10.0), 3.0),
    coriolis: Some(Coriolis { latitude: 40f64.to_radians(), azimuth: 0.0 }),
    ..Shot::default()
};
for p in solver.flight(&shot, yd_to_m(100.0), yd_to_m(1500.0)) {
    println!("{:5.0} yd  {:7.1} in  {:6.2} mil", m_to_yd(p.range), m_to_in(p.path), p.path_mil());
}
```

`flight` is an iterator that carries the integrator state, so nothing allocates — that's what makes it usable on an MCU later. `Solver::table` collects it to a `Vec` when you have std.

`DragModel` is a trait. `G7` is built in; `TableDrag` takes any `(mach, cd)` slice, e.g. Doppler-radar data for one specific bullet (then BC = sectional density). `Atmosphere` is a trait too: `Isa { station_altitude }` or `Station { temperature, pressure, humidity }`.

## Aiming

`solver` and `table` go forward: bullet and conditions in, where it lands out.
`aim` goes backward: target in, where to point out. That is what a fire control
loop needs and what a drop table cannot give it.

```rust
use ballistics_core::aim::{aim, AimParams, Conditions, Enu, Gun, Target};

let gun = Gun { muzzle_velocity: fps_to_mps(2650.0), sight_height: in_to_m(2.5) };

// 900 m due north, crossing left to right at 30 m/s.
let target = Target {
    position: Enu::new(0.0, 900.0, 0.0),
    velocity: Enu::new(30.0, 0.0, 0.0),
    acceleration: Enu::ZERO,
};
let conditions = Conditions::wind_from(mph_to_mps(8.0), 270f64.to_radians())
    .at_latitude(40.1f64.to_radians());

let s = aim(&solver, &gun, &target, &conditions, &AimParams::default());
// s.bearing, s.elevation   where to point, rad
// s.tof                    time of flight, s
// s.status                 check this before firing anything
```

Everything on that boundary is east/north/up metres from the sensor, which is
the frame a tracker already works in. The shot frame never leaves the module.

It guesses a time of flight, works out where the target will be then, solves the
bore elevation that drops the bullet onto that point, reads the real time of
flight back off the integration and goes round again. Two or three passes. The
inner superelevation solve is a safeguarded secant rather than a bisection
because every evaluation is a whole trajectory: three integrations instead of
forty, and the bracket keeps it from running away.

`aim_hinted` warm starts from the last frame's answer, which is what a tracking
loop should call. At 50 Hz on a 30 m/s crosser at 900 m that is 1.4 integrations
per frame against 5 for a cold solve. `aim_with_rates` also returns the slew
rates for the controller's feed forward.

Not in there yet: spin drift, aerodynamic jump, and mount translation (a moving
platform adds its own velocity to the muzzle velocity vector and `Shot` cannot
express that). There is no ground plane either, so a depressed solution will fly
a bullet through a hill and report a clean intercept.

## Cross-check against the Python solver

1. Diff `G7_TABLE` in `crates/ballistics-core/src/drag.rs` against the Python table first. The tables floating around differ in the last digit here and there and that shows up as a few fps at 1000 yd.
2. Dump a table from the Python solver for a load, write it into a JSON file shaped like `crates/ballistics-core/tests/fixtures/_example.json` (rename it, anything starting with `_` is skipped), and `cargo test`. `reference.rs` compares path, drift, velocity and TOF at every listed range within the tolerances you set.

Things that will produce small disagreements that aren't bugs: how the Python version handles humidity, whether it uses ISA or ASM as the reference atmosphere, whether it varies density with height along the trajectory (this one does), latitude gravity (this one uses it when a latitude is given), and dt. Check `wind_clock` convention too.

## wasm for the React app

Needs `rustup target add wasm32-unknown-unknown` and `cargo install wasm-pack`.

```
wasm-pack build crates/ballistics-wasm --target web --out-dir ../../web/pkg
```

```js
import init, { solve_json, default_request } from "./pkg/ballistics_wasm.js";
await init();
const req = { ...JSON.parse(default_request()), bc: 0.42, weight_gr: 300, muzzle_velocity_fps: 2650, wind_mph: 10 };
const { zero_angle_moa, rows } = JSON.parse(solve_json(JSON.stringify(req)));
```

Field names and units are on `TableRequest` in `crates/ballistics-core/src/table.rs`. Same struct the CLI uses, so the CLI is the test bench for the web app. With that in place the FastAPI service has nothing left to do.

## Toolchain note

Originally built on rustc 1.75, which is why clap is pinned to `~4.4` and
wasm-bindgen to 0.2.100 in `Cargo.lock`. Verified again on 1.95 with those pins
still in place: builds, tests and clippy all clean, so the pins are not holding
anything back and can go whenever someone wants to.

No MSRV is declared. 1.75 has not been re-checked since `aim` was added, so if
you need a floor, verify it before putting `rust-version` in `Cargo.toml`.

The code is not rustfmt formatted and CI does not check it. Running
`cargo fmt --all` touches about 21 sites across the workspace, so if it ever
happens it should be one deliberate commit on its own.

## Next

- Spin drift (Litz estimate: 1.25 (SG + 1.2) t^1.83 inches) and aerodynamic jump as optional post-corrections on `Point`. The Python has both; this does not, and on a .338 spin drift is 8 to 10 inches right at 1000 yd. `aim` already has the slot for it: it adds into `drift_correction` and nothing else changes. Needs SG, so `Projectile` has to start carrying length, diameter and twist.
- G1 table, for bullets that only publish G1.
- `no_std` target build: STM32H7-class part (double-precision FPU). The core already compiles without std; it needs an entry point and a way to get the request in.
