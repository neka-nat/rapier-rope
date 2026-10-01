//! Checked snapshots and playback recording of a pinned, hanging rope.
#[allow(dead_code)]
mod common;
#[allow(dead_code)]
mod recording;
fn main() -> common::Result<()> {
    recording::example("hanging")
}
