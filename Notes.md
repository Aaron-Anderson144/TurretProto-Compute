# Notes for working in this repo

## What this is

Compute side of a turret prototype. Ballistics, gimbal stabilization, and the
fire control loop meant to tie them together.

```
ballistics/         Rust workspace. no_std core + CLI + wasm. The one that ships.
ballistics-python/  Older Python/FastAPI + React solver. More complete model.
                    Kept as the reference oracle until the Rust reaches parity.
                    Do not delete it, and do not "clean it up".
control/matlab/     Gimbal plant and controllers. Design and tuning reference.
fcs/                Fire control. Empty. See fcs/README.md.
sim/                Closed loop harness. Empty. See sim/README.md.
docs/               Hardware notes and the 9/7/2026 code review.
```

## Build and test

```
cd ballistics
cargo test --workspace                                        # 38 tests
cargo clippy --workspace --all-targets -- -D warnings         # clean, keep it that way
cargo build -p ballistics-core --no-default-features          # no_std, do not break this
cargo build -p ballistics-core --no-default-features --features serde
cargo run --example aim_demo -p ballistics-core               # firing solutions + cost
cargo run --release -p ballistics-cli -- --help
```

```
cd ballistics-python/backend
pip install -r requirements.txt && python -m pytest ballistics/tests/ -q   # 11 tests
```

MATLAB: `cd control/matlab`, then `run_turret_stab` or `run_turret_2axis`.

## Environment

Aaron's workstation has no cargo, no rustc and no octave. Rust work has to be
built and tested somewhere else and the checked files committed back. Do not
claim a Rust change works without having compiled it somewhere.

The repo currently lives inside OneDrive. Git and OneDrive do not get along:
OneDrive syncs `.git`, holds locks, and can dehydrate objects into cloud-only
placeholders. Moving the repo out of OneDrive is on the list.

Sandboxed shells on that machine cannot delete files. `rm` fails with
"Operation not permitted". Two consequences: git leaves a stale
`.git/index.lock` after every write, which has to be moved aside between
`git add` and `git commit`, and anything that needs deleting gets moved to a
gitignored `_to_delete/` folder for Aaron to remove himself.

## Conventions that will bite you

Frames. There are two, on purpose, and they are different types so they cannot
be mixed by accident.

- `Vec3` is the **shot frame**: x downrange along the firing azimuth, y up,
  z right. Right handed. Internal to the solver.
- `Enu` is the **level frame**: east, north, up, metres from the sensor. This is
  the boundary of `aim`, because it is the frame a tracker works in.

Bearings are radians clockwise from true north. Elevations are radians above
horizontal, + up. Solution bearings come back wrapped to [0, 2pi); lead angles
are signed and wrapped to (-pi, pi].

Signs. `path` is + above the line of sight, `drift` is + right. What you dial is
the opposite. Wind is quoted as the direction it blows FROM, but `Wind` and
`Conditions.wind` store the air's velocity vector, which is the other way round.

Units. Everything inside the crates is SI. BC is the one exception, taken in
lb/in^2 because that is how it is published. Convert at the edges with `units`.

`!(x > 0.0)` throughout is deliberate: it rejects NaN as well as the
out-of-range case. `x <= 0.0` would let NaN through. There is a crate-level
clippy allow for it in `lib.rs`.

## State of things

`ballistics-core` has two front doors. `solver`/`table` go forward (bullet in,
where it lands out). `aim` goes backward (target in, where to point out).

Known gaps, deliberate, documented in the code:

- Spin drift and Miller SG are in the Python and not in the Rust. On a .338 that
  is 8 to 10 inches right at 1000 yd. `aim::Solution.drift_correction` has the
  slot waiting; it needs `Projectile` to carry length, diameter and twist.
- `tests/reference.rs` passes vacuously. `tests/fixtures/` holds only
  `_example.json`, which the test skips. The 38 tests are self-consistency and
  closed-form checks, not an accuracy claim. Dumping a few loads out of the
  Python is the highest value thing left.
- `aim` does not model mount translation, aerodynamic jump, or terrain. There is
  no ground plane, so a depressed solution will fly through a hill and report a
  clean intercept.
- The MATLAB run scripts set `NOISE_FREE = true`, so every PID vs LADRC number
  in the repo is from a noise free plant. `CascadeController` differentiates the
  raw encoder with no filter, which is why. Do not quote those numbers.

`docs/REVIEW-2026-09-07.md` has the full findings and the build order.

## Style

Plain and direct. No em dashes. Comments explain why, not what. READMEs stay
simple with no decoration, and skip explaining the obvious.

The code is not rustfmt formatted and CI does not check it. If that ever
changes it should be one deliberate `cargo fmt --all` commit on its own, not
mixed into a change.
