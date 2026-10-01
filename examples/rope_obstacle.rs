//! Capsule wire contact with a cuboid, saved through the public track API.
#[allow(dead_code)]
mod common;
#[allow(dead_code)]
mod recording;
fn main() -> common::Result<()> {
    recording::example("obstacle")
}
