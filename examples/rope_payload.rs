//! dynamic payload and independent direct-native comparison.
#[allow(dead_code)]
mod attachments;
#[allow(dead_code)]
mod common;
use common::config::Config;
use std::path::Path;
fn main() -> common::Result<()> {
    let input = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/native/baseline.json");
    let config = Config::load(&input)?;
    let run = attachments::payload(&config)?;
    attachments::write_result(&config, vec![run])
}
