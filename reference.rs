//! Cross-check against the Python solver.
//!
//! Drop one JSON file per load into `tests/fixtures/` (see `_example.json`
//! for the shape). Anything starting with `_` is ignored. With no fixtures
//! present this test passes and just says so.

use std::fs;
use std::path::PathBuf;

use ballistics_core::table::{solve, TableRequest};
use serde::Deserialize;

#[derive(Deserialize)]
struct Fixture {
    request: TableRequest,
    tolerance: Tolerance,
    points: Vec<Expected>,
}

#[derive(Deserialize)]
struct Tolerance {
    path_in: f64,
    drift_in: f64,
    velocity_fps: f64,
    time_s: f64,
}

#[derive(Deserialize)]
struct Expected {
    range_yd: f64,
    path_in: f64,
    drift_in: f64,
    velocity_fps: f64,
    time_s: f64,
}

#[test]
fn python_reference_fixtures() {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let mut checked = 0;
    for entry in fs::read_dir(&dir).unwrap() {
        let path = entry.unwrap().path();
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        if !name.ends_with(".json") || name.starts_with('_') {
            continue;
        }
        let text = fs::read_to_string(&path).unwrap();
        let fx: Fixture = serde_json::from_str(&text).unwrap_or_else(|e| panic!("{name}: {e}"));
        let out = solve(&fx.request).unwrap_or_else(|e| panic!("{name}: {e}"));
        for exp in &fx.points {
            let row = out
                .rows
                .iter()
                .find(|r| (r.range_yd - exp.range_yd).abs() < 1e-6)
                .unwrap_or_else(|| panic!("{name}: no row at {} yd — pick step_yd so the stations line up", exp.range_yd));
            let check = |what: &str, got: f64, want: f64, tol: f64| {
                assert!(
                    (got - want).abs() <= tol,
                    "{name} @ {} yd {what}: rust {got} vs python {want} (tol {tol})",
                    exp.range_yd
                );
            };
            check("path_in", row.path_in, exp.path_in, fx.tolerance.path_in);
            check("drift_in", row.drift_in, exp.drift_in, fx.tolerance.drift_in);
            check("velocity_fps", row.velocity_fps, exp.velocity_fps, fx.tolerance.velocity_fps);
            check("time_s", row.time_s, exp.time_s, fx.tolerance.time_s);
        }
        checked += 1;
    }
    if checked == 0 {
        eprintln!("no fixtures in {} — export tables from the Python solver to compare", dir.display());
    }
}
