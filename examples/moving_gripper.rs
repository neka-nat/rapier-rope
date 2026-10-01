//! moving/rotating endpoint grasp, interior grasp, moving pin and release.
#[allow(dead_code)]
mod attachments;
#[allow(dead_code)]
mod common;
use common::config::Config;
use std::path::Path;
fn main() -> common::Result<()> {
    let input = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/native/baseline.json");
    let config = Config::load(&input)?;
    let runs = ["endpoint_grasp", "interior_grasp", "moving_pin"]
        .into_iter()
        .map(|case| attachments::moving(&config, case))
        .collect::<common::Result<Vec<_>>>()?;
    attachments::write_result(&config, runs)
}
