#[path = "../examples/attachments/mod.rs"]
#[allow(dead_code)]
mod attachments;
#[path = "../examples/common/mod.rs"]
#[allow(dead_code)]
mod common;
fn config() -> common::config::Config {
    common::config::Config::load(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/native/baseline.json"),
    )
    .unwrap()
}
#[test]
fn moving_rotating_grasps_and_pin_fall_after_release() {
    for case in ["endpoint_grasp", "interior_grasp", "moving_pin"] {
        let run = attachments::moving(&config(), case).unwrap();
        assert!(run.passed(), "{case}: {:?}", run.checks);
    }
}
#[test]
fn dynamic_payload_preserves_mass_and_direct_native_two_way_response() {
    let run = attachments::payload(&config()).unwrap();
    assert!(run.passed(), "{:?}", run.checks);
}
