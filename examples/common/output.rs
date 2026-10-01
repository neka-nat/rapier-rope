use super::Result;
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{fs, path::Path};

pub fn write_json(path: &Path, value: &impl Serialize) -> Result<()> {
    let file = fs::File::create(path)?;
    serde_json::to_writer_pretty(std::io::BufWriter::new(file), value)?;
    Ok(())
}

pub fn digest(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
