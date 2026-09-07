# TurretProto-Compute

Compute side of the turret prototype. Ballistics, gimbal stabilization, and the
fire control loop that is supposed to tie them together.

## Layout

```
ballistics/         Rust. no_std core + CLI + wasm. The one that ships.
ballistics-python/  Python/FastAPI + React. Older, more complete, kept as the
                    reference oracle until the Rust reaches parity.
control/matlab/     Gimbal plant and controllers. Design and tuning reference.
fcs/                Fire control. Empty. See fcs/README.md.
sim/                Closed loop harness. Empty. See sim/README.md.
docs/               Hardware notes and the code review.
```

## Where it stands

`ballistics` and `control` both work and neither one knows the other exists.
Ballistics produces drop tables for a person to read. The controller takes
setpoints from a hand written scenario script. Nothing turns a target into a
setpoint yet. That is `fcs/`.

## ballistics

Rust workspace. 3 DOF point mass, RK4, G7 drag, atmosphere with humidity, wind,
Coriolis, latitude gravity.

```
cd ballistics
cargo test --workspace
cargo build -p ballistics-core --no-default-features    # proves no_std clean
cargo run --release -p ballistics-cli -- --help
```

Full notes in `ballistics/README.md`.

`ballistics/README.md` has the aiming section. Short version: `aim()` takes a
target's position and velocity in east/north/up and returns gun bearing,
elevation and time of flight. That is the call `fcs/` is built around.

```
cargo run --example aim_demo -p ballistics-core
```

Two things to know before trusting it:

- Spin drift and Miller SG are in the Python and not yet in the Rust. On a .338
  that is 8 to 10 inches right at 1000 yd.
- `tests/reference.rs` passes vacuously. `tests/fixtures/` holds only
  `_example.json`, which the test skips. Nothing has actually been cross checked
  against the Python yet. 38 tests pass, but they are all self-consistency and
  closed-form checks, not an accuracy claim.

## ballistics-python

```
cd ballistics-python/backend
pip install -r requirements.txt
uvicorn main:app --reload --port 8000

cd ballistics-python/frontend
npm install && npm run dev
```

Has spin drift (Litz), Miller stability, BC truing and the bullet library. Do not
retire it until the Rust has parity. Use it to generate the reference fixtures.

## control/matlab

```
cd control/matlab
run_turret_stab      % single axis, point / vector / stabilize
run_turret_2axis     % coupled pan and tilt
```

`NOISE_FREE = true` is set in both run scripts, so the PID vs LADRC numbers they
print are from a noise free plant. The gyro and encoder noise the plant defines
never get exercised. Fix that before quoting any of those numbers.


