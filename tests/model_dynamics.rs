#[path = "../examples/common/mod.rs"]
#[allow(dead_code)]
mod common;
#[path = "../examples/model_reference/mod.rs"]
mod model_reference;

#[test]
fn domain_builder_matches_direct_native_motion_in_five_scenes() {
    let config = common::config::Config::load(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/native/baseline.json"),
    )
    .unwrap();
    for case in [
        "hanging",
        "moving",
        "obstacle",
        "payload",
        "polyline_placed",
    ] {
        for matched_native_inputs in [false, true] {
            let result = model_reference::compare(case, &config, matched_native_inputs).unwrap();
            assert!(result.passed, "{result:#?}");
            assert_eq!(result.steps, 480);
        }
    }
}
