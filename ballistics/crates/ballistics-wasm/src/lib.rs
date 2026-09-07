//! wasm-bindgen wrapper. One function: JSON `TableRequest` in, JSON `TableResult` out.
//! Field names and units are documented on `ballistics_core::table::TableRequest`.
//!
//! Build: `wasm-pack build crates/ballistics-wasm --target web`

use ballistics_core::table::{solve, TableRequest};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn solve_json(request: &str) -> Result<String, JsError> {
    let req: TableRequest = serde_json::from_str(request)?;
    let out = solve(&req)?;
    Ok(serde_json::to_string(&out)?)
}

/// The request with every default filled in, so the front end can show them.
#[wasm_bindgen]
pub fn default_request() -> String {
    serde_json::to_string(&TableRequest::default()).unwrap()
}
