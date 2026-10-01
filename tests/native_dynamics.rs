#[path = "../examples/common/mod.rs"]
#[allow(dead_code)]
mod common;
#[path = "../examples/contacts/mod.rs"]
#[allow(dead_code)]
mod contacts;

fn run(case: &str) {
    let config = common::config::Config::load(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/native/baseline.json"),
    )
    .unwrap();
    let q = contacts::ContactSettings::load().unwrap();
    let (track, result) = contacts::contact(case, &config, &q, true).unwrap();
    assert_eq!(track.frames().last().unwrap().capture.step, Some(479));
    assert_eq!(result["passed"], true, "{result:#}");
    if case != "floor" {
        let (control, result) = contacts::contact(case, &config, &q, false).unwrap();
        assert_eq!(result["contact_steps"], 0, "{result:#}");
        let current = &track
            .frames()
            .last()
            .unwrap()
            .ropes
            .last()
            .unwrap()
            .positions_m;
        let disabled = &control
            .frames()
            .last()
            .unwrap()
            .ropes
            .last()
            .unwrap()
            .positions_m;
        let max_difference = current
            .iter()
            .zip(disabled)
            .map(|(a, b)| {
                ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
            })
            .fold(0.0_f64, f64::max);
        assert!(
            max_difference >= q.min_control_shape_difference_m,
            "control difference {max_difference}"
        );
    }
}
#[test]
fn floor_capsule_contact() {
    run("floor");
}
#[test]
fn nonadjacent_self_contact_changes_motion() {
    run("self_contact");
}
#[test]
fn two_rope_contact_changes_motion() {
    run("two_ropes");
}
