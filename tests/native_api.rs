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
fn native_public_api_semantics() {
    let probes = common::probes::run(&config()).unwrap();
    let failures: Vec<_> = probes.checks.iter().filter(|check| !check.passed).collect();
    assert!(failures.is_empty(), "{failures:#?}");
}

#[test]
fn four_native_scenes_satisfy_fixed_baseline() {
    for case in common::scenes::CASES {
        let run = common::scenes::run(case, &config()).unwrap();
        let failures: Vec<_> = run.checks.iter().filter(|check| !check.passed).collect();
        assert!(failures.is_empty(), "{case}: {failures:#?}");
        assert_eq!(run.frames.first().unwrap().step, 0);
        assert_eq!(run.frames.last().unwrap().step, run.simulated_steps);
    }
}

#[test]
fn experiment_config_rejects_invalid_native_inputs() {
    let baseline = config();
    let mut bad = baseline.clone();
    bad.segments = 0;
    assert!(bad.validate().is_err());
    bad = baseline.clone();
    bad.radius_m = 0.0;
    assert!(bad.validate().is_err());
    bad = baseline;
    bad.dt_s = f64::NAN;
    assert!(bad.validate().is_err());
}
