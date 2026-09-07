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

## Hardware path

From `docs/Turretnoteshardware.txt`: Jetson Orin Nano Super dev kit and sim
first, then a real IMU + 2 axis brushless gimbal + recoil actuator, then rugged
enclosures and better IMUs, then DuraCOR for environmental.

Two things worth deciding early:

- The 1 kHz rate loops do not belong in Linux userspace on the Orin. Put them on
  an MCU (STM32H7, which is the no_std target the ballistics crate already
  targets) and leave the Jetson doing tracker, ballistics and mission at 50 to
  100 Hz.
- The whole thing is a time of flight prediction problem, so IMU, camera and
  control need a common timebase. Pick one hardware sync line or PTP now.

## Building on it

`CLAUDE.md` has the conventions that will bite you: the two frames and why they
are separate types, the sign conventions, the units, and what is deliberately
not modelled. Read it before changing anything in `ballistics/`.

CI (`.github/workflows/ci.yml`) runs the Rust tests, clippy as an error, both
no_std builds, and the Python test suite. All green as of the last commit.

## License

MIT or Apache-2.0, at your option. See `LICENSE-MIT` and `LICENSE-APACHE`.

## Next

Order is in `docs/REVIEW-2026-09-07.md`. Short version:

1. Reference fixtures out of the Python so `reference.rs` actually runs. This is
   now the only thing between the solver and a real accuracy claim.
2. Turn the MATLAB noise on, fix the encoder differentiation and the dB metric,
   re-take the numbers.
3. Spin drift and Miller SG in the Rust, parity with the Python, then retire the
   FastAPI backend.
4. `fcs/`: define `AimPoint`, wire the tracker to `aim()` and `aim()` to the
   controller's rate feed forward.

Done: `aim.rs`, the inverse solver.
