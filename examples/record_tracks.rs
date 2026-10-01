//! Record headless rope examples for the playback viewer. Optional output directory.
#[allow(dead_code)]
mod common;
#[allow(dead_code)]
mod recording;
use common::config::Config;
use serde_json::json;
use std::path::Path;
fn main() -> common::Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() > 1 {
        return Err("usage: record_tracks [output_directory]".into());
    }
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let input = root.join("tests/fixtures/native/baseline.json");
    let config = Config::load(&input)?;
    let output = args
        .first()
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| root.join("target/tracks").join(common::precision()));
    std::fs::create_dir_all(&output)?;
    let mut runs = vec![];
    for case in recording::CASES {
        let (track, checks) = recording::run(case, &config)?;
        let path = output.join(format!("{case}.json"));
        track.write_json(std::fs::File::create(&path)?)?;
        let frames = track.frames();
        runs.push(json!({"case":case,"track_sha256":common::output::digest(&std::fs::read(path)?),"frames":frames.len(),"first_capture":frames.first().unwrap().capture,"last_capture":frames.last().unwrap().capture,"name":frames[0].ropes[0].name,"radius_m":frames[0].ropes[0].radius_m,"particles":frames[0].ropes[0].positions_m.len(),"events":track.events(),"checks":checks,"passed":checks.iter().all(|c|c.passed)}));
    }
    let passed = runs.iter().all(|r| r["passed"] == true);
    let result =
        json!({"precision":common::precision(),"configuration":config,"runs":runs,"passed":passed});
    let result_path = output.join("result.json");
    common::output::write_json(&result_path, &result)?;
    println!(
        "{}: {}",
        common::precision(),
        if passed {
            "PASS: 5 playback tracks"
        } else {
            "FAIL"
        }
    );
    if !passed {
        return Err("playback example checks failed".into());
    }
    Ok(())
}
