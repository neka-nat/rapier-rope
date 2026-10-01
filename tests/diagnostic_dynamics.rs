#[path = "../examples/common/mod.rs"]
#[allow(dead_code)]
mod common;
#[path = "../examples/recording/mod.rs"]
#[allow(dead_code)]
mod recording;
#[test]
fn baseline_tracks_round_trip_and_midpoint_contact_is_observed() {
    let config = common::config::Config::load(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/native/baseline.json"),
    )
    .unwrap();
    for case in recording::CASES {
        let (track, checks) = recording::run(case, &config).unwrap();
        assert!(checks.iter().all(|c| c.passed), "{case}: {checks:?}");
        assert_eq!(track.frames().last().unwrap().capture.step, Some(479));
        if case == "moving" {
            let release = track
                .events()
                .iter()
                .find(|e| matches!(e.operation, rapier_rope::TrackEventKind::Detach { .. }))
                .unwrap();
            assert_eq!(release.step, 240);
            assert!((release.time_s.value() - 1.0).abs() < 1e-6);
            let before = track
                .frames()
                .iter()
                .find(|f| f.capture.phase == rapier_rope::CapturePhase::BeforeStep)
                .unwrap();
            assert!(before.ropes[0].diagnostics.attachments.is_empty());
            assert_eq!(
                before.ropes[0]
                    .diagnostics
                    .impulse_metadata
                    .last_completed_step,
                Some(239)
            );
        }
    }
}
