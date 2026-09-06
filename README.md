# ballistics

Rust port of the .338 Norma exterior ballistics solver. Point mass, RK4, G7 drag, atmosphere with humidity, wind, Coriolis, latitude gravity.

```
crates/ballistics-core   the solver. no_std with --no-default-features (libm for math)
crates/ballistics-cli    `bal` — drop/drift table from the command line
crates/ballistics-wasm   wasm-bindgen wrapper, JSON in / JSON out, for the React app
```

## Run

```
cargo test --workspace                                   # 13 tests
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

Built and tested here on rustc 1.75. clap is pinned to `~4.4` and wasm-bindgen to 0.2.100 in `Cargo.lock` for that reason; on a current stable toolchain both pins can go.

## Next

- Spin drift (Litz estimate: 1.25 (SG + 1.2) t^1.83 inches) and aerodynamic jump as optional post-corrections on `Point`.
- G1 table, for bullets that only publish G1.
- `no_std` target build: STM32H7-class part (double-precision FPU). The core already compiles without std; it needs an entry point and a way to get the request in.
