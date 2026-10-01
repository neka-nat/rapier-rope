#[path = "support/registry.rs"]
#[allow(dead_code)]
mod support;
use rapier_rope::{rapier::prelude::*, *};
use support::{ID, setup};
const H: f64 = 1.0 / 240.0;
fn sample(points: Vec<Point3>) -> SampledRope {
    let mut spec = support::spec("geometry");
    spec.reference_centerline_m = points;
    spec.sampling.max_segment_length_m = 10.0;
    sample_rope(&spec).unwrap()
}
fn stamp(phase: CapturePhase, step: Option<u64>, time: f64) -> CaptureStamp {
    CaptureStamp::new(phase, step, time, H).unwrap()
}
fn recorded() -> (RopeSet, PhysicsWorld, RopeHandle, RopeTrack, RopeSnapshot) {
    let (ropes, world, rope) = setup();
    let view = ropes.centerline(ID, &world, rope).unwrap();
    let mut track = RopeTrack::new("unit", "right-handed, Y-up, SI", &world).unwrap();
    track.register_rope(&view).unwrap();
    let snapshot = view
        .snapshot(stamp(CapturePhase::Initial, None, 0.0))
        .unwrap();
    (ropes, world, rope, track, snapshot)
}
fn frame(snapshot: RopeSnapshot) -> TrackFrame {
    TrackFrame {
        capture: snapshot.capture.clone(),
        ropes: vec![snapshot],
        bodies: vec![],
    }
}

#[test]
fn straight_length_strain_and_infinite_radius_are_distinct() {
    let samples = sample(vec![[0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]);
    let d = diagnose_geometry(&samples, &[[0.0; 3], [2.0, 0.0, 0.0], [4.0, 0.0, 0.0]]).unwrap();
    assert_eq!(d.current_length_m.value(), Some(4.0));
    assert_eq!(d.max_tensile_strain.value(), Some(1.0));
    assert!(
        d.segments
            .iter()
            .all(|s| s.axial_strain.value() == Some(1.0))
    );
    assert_eq!(d.curvature[1].curvature_inv_m.value(), Some(0.0));
    assert!(
        matches!(&d.curvature[1].estimated_bend_radius_m,Observation::Undefined{reason} if reason=="straight_limit_infinite_radius")
    );
    assert!(
        matches!(&d.curvature[0].curvature_inv_m,Observation::Undefined{reason} if reason=="endpoint")
    );
}
#[test]
fn known_circle_curvature_and_polyline_length_converge_under_refinement() {
    let radius = 2.0;
    let mut errors = vec![];
    for segments in [8, 16, 32, 64] {
        let points: Vec<_> = (0..=segments)
            .map(|i| {
                let t = std::f64::consts::FRAC_PI_2 * i as f64 / segments as f64;
                [radius * t.cos(), radius * t.sin(), 0.0]
            })
            .collect();
        let d = diagnose_geometry(&sample(points.clone()), &points).unwrap();
        let delta = std::f64::consts::FRAC_PI_2 / segments as f64;
        let expected = delta / (2.0 * radius * (delta / 2.0).sin());
        assert!(
            d.curvature[1..segments]
                .iter()
                .all(|c| (c.curvature_inv_m.value().unwrap() - expected).abs() < 1e-12)
        );
        let length_error =
            (d.current_length_m.value().unwrap() - radius * std::f64::consts::FRAC_PI_2).abs();
        let curvature_error =
            (d.curvature[segments / 2].curvature_inv_m.value().unwrap() - 1.0 / radius).abs();
        errors.push((length_error, curvature_error));
    }
    for w in errors.windows(2) {
        assert!(w[1].0 < w[0].0 / 3.9);
        assert!(w[1].1 < w[0].1 / 3.9);
    }
}
#[test]
fn nonuniform_spacing_uses_mean_adjacent_current_lengths() {
    let points = vec![[0.0; 3], [1.0, 0.0, 0.0], [1.0, 2.0, 0.0]];
    let d = diagnose_geometry(&sample(points.clone()), &points).unwrap();
    let c = &d.curvature[1];
    assert_eq!(c.incoming_length_m.value(), Some(1.0));
    assert_eq!(c.outgoing_length_m.value(), Some(2.0));
    assert!((c.curvature_inv_m.value().unwrap() - std::f64::consts::PI / 3.0).abs() < 1e-14);
    assert!(
        (c.estimated_bend_radius_m.value().unwrap() - 3.0 / std::f64::consts::PI).abs() < 1e-14
    );
}
#[test]
fn collapsed_segments_are_undefined_and_nonfinite_values_are_explicit_json_states() {
    let s = sample(vec![[0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]);
    let d = diagnose_geometry(&s, &[[0.0; 3], [0.0; 3], [2.0, 0.0, 0.0]]).unwrap();
    assert!(
        matches!(&d.curvature[1].curvature_inv_m,Observation::Undefined{reason} if reason=="collapsed_adjacent_segment")
    );
    let d = diagnose_geometry(
        &s,
        &[[0.0; 3], [f64::NAN, 0.0, 0.0], [f64::INFINITY, 0.0, 0.0]],
    )
    .unwrap();
    assert!(!d.positions_finite);
    assert!(matches!(d.current_length_m, Observation::NonFinite { .. }));
    let json = serde_json::to_string(&d).unwrap();
    assert!(!json.contains("null"));
    assert!(!json.contains("NaN"));
    assert!(json.contains("non_finite"));
    let read: GeometryDiagnostics = serde_json::from_str(&json).unwrap();
    assert!(matches!(
        read.curvature[1].curvature_inv_m,
        Observation::NonFinite { .. }
    ));
}
#[test]
fn finite_coordinates_with_overflow_and_bad_count_do_not_become_zero() {
    let s = sample(vec![[0.0; 3], [1.0, 0.0, 0.0]]);
    let d = diagnose_geometry(&s, &[[f64::MAX, 0.0, 0.0], [-f64::MAX, 0.0, 0.0]]).unwrap();
    assert!(d.positions_finite);
    assert!(matches!(d.current_length_m, Observation::NonFinite { .. }));
    assert!(matches!(
        diagnose_geometry(&s, &[]),
        Err(DiagnosticError::ParticleCount { .. })
    ));
    assert!(FiniteScalar::new(f64::NAN).is_err());
    assert!(serde_json::from_str::<FiniteScalar>("1e999").is_err());
}
#[test]
fn checked_view_snapshot_is_owned_and_stale_native_state_is_rejected() {
    let (mut ropes, mut world, rope, _, snapshot) = recorded();
    let initial = snapshot.positions_m.clone();
    ropes.remove(ID, &mut world, rope).unwrap();
    assert_eq!(snapshot.positions_m, initial);
    assert!(matches!(
        ropes.centerline(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::StaleRopeHandle
    ));
    let (ropes, mut world, rope) = setup();
    let h = ropes.get(ID, &world, rope).unwrap().native_handle;
    world.soft_bodies[h].set_particle_velocity(0, Vector::splat(Real::NAN));
    assert!(matches!(
        ropes.centerline(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::NonFiniteState(_)
    ));
}
#[test]
fn unstepped_impulses_and_unprovided_capabilities_are_never_reported_as_zero() {
    let (_, _, _, _, s) = recorded();
    assert!(
        s.diagnostics
            .edge_impulses
            .iter()
            .all(|e| matches!(e.impulse_ns, Observation::Unavailable { .. }))
    );
    assert!(matches!(
        s.capabilities.axial_torsion,
        CapabilitySupport::Unsupported { .. }
    ));
    assert!(matches!(
        s.diagnostics.unprovided.average_tension_n,
        Observation::Unavailable { .. }
    ));
    assert!(matches!(
        s.diagnostics.unprovided.material_stress_pa,
        Observation::Unavailable { .. }
    ));
    assert!(matches!(
        s.diagnostics.unprovided.axial_torsion_rad,
        Observation::Unsupported { .. }
    ));
    assert!(matches!(
        s.diagnostics.unprovided.solver_converged,
        Observation::Unavailable { .. }
    ));
}
#[test]
fn native_impulses_match_exactly_and_carry_units_direction_and_capture_time() {
    let (mut ropes, mut world, rope) = setup();
    let body = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(1.0, 1.4, 0.0))
            .additional_mass(0.05),
    );
    ropes
        .prepare_attachments(
            ID,
            &mut world,
            0,
            H,
            &[
                AttachmentCommand::Pin {
                    rope,
                    location: RopeLocation::Start,
                    position_m: [0.0, 1.5, 0.0],
                },
                AttachmentCommand::Attach {
                    rope,
                    location: RopeLocation::End,
                    body,
                },
            ],
        )
        .unwrap();
    world.step();
    ropes.inspect(ID, &world, 0).unwrap();
    let s = ropes
        .centerline(ID, &world, rope)
        .unwrap()
        .snapshot(stamp(CapturePhase::AfterStep, Some(0), H))
        .unwrap();
    let native = ropes.get(ID, &world, rope).unwrap().soft_body;
    for (i, (actual, e)) in s
        .diagnostics
        .edge_impulses
        .iter()
        .zip(native.edges())
        .enumerate()
    {
        #[allow(clippy::unnecessary_cast)]
        let value = e.impulse() as f64;
        assert_eq!(actual.impulse_ns.value(), Some(value));
        assert_eq!(actual.vertices, e.vertices);
        assert_eq!(
            actual.family,
            if i < 8 {
                EdgeFamily::Structural
            } else {
                EdgeFamily::Bending
            }
        );
    }
    let a = &s.diagnostics.attachments[0];
    let n = &native.particle_attachments()[0];
    #[allow(clippy::unnecessary_cast)]
    let expected = [
        n.impulse().x as f64,
        n.impulse().y as f64,
        n.impulse().z as f64,
    ];
    assert_eq!(
        a.native_impulse_ns,
        Observation::Available(expected.map(|x| FiniteScalar::new(x).unwrap()))
    );
    assert_eq!(s.diagnostics.impulse_metadata.unit, "N s");
    assert_eq!(s.diagnostics.impulse_metadata.last_completed_step, Some(0));
    assert!(
        s.diagnostics
            .impulse_metadata
            .attachment_direction
            .contains("+ on rigid body, - on particle")
    );
    assert!(matches!(
        s.diagnostics.impulse_metadata.internal_substep_dt_s,
        Observation::Unavailable { .. }
    ));
}
#[test]
fn capture_context_validates_phase_time_dt_and_before_step_impulse_age() {
    assert!(CaptureStamp::new(CapturePhase::Initial, Some(0), 0.0, H).is_err());
    assert!(CaptureStamp::new(CapturePhase::AfterStep, None, H, H).is_err());
    assert!(CaptureStamp::new(CapturePhase::AfterStep, Some(0), f64::NAN, H).is_err());
    assert!(CaptureStamp::new(CapturePhase::AfterStep, Some(0), H, 0.0).is_err());
    assert_eq!(
        stamp(CapturePhase::BeforeStep, Some(7), 7.0 * H).last_completed_step(),
        Some(6)
    );
    let (ropes, world, rope) = setup();
    #[allow(clippy::unnecessary_cast)]
    let effective_dt = world.integration_parameters.dt as f64;
    assert_eq!(
        stamp(CapturePhase::Initial, None, 0.0).outer_dt_s.value(),
        effective_dt
    );
    let wrong = CaptureStamp::new(CapturePhase::Initial, None, 0.0, 2.0 * H).unwrap();
    assert!(
        ropes
            .centerline(ID, &world, rope)
            .unwrap()
            .snapshot(wrong)
            .is_err()
    );
}
#[test]
fn track_round_trip_retains_definition_sampling_first_last_and_events() {
    let (mut ropes, mut world, rope, mut track, s) = recorded();
    track.push_frame(frame(s)).unwrap();
    support::step(
        &mut ropes,
        &mut world,
        &[RopeCommand::Pin { rope, particle: 0 }],
    );
    track
        .record_event(TrackEvent {
            step: 0,
            time_s: FiniteScalar::new(0.0).unwrap(),
            operation: TrackEventKind::Pin {
                rope: rope.into(),
                particle: 0,
            },
        })
        .unwrap();
    let final_frame = frame(
        ropes
            .centerline(ID, &world, rope)
            .unwrap()
            .snapshot(stamp(CapturePhase::AfterStep, Some(0), H))
            .unwrap(),
    );
    let final_positions = final_frame.ropes[0].positions_m.clone();
    track.push_frame(final_frame).unwrap();
    let mut bytes = vec![];
    track.write_json(&mut bytes).unwrap();
    let read = RopeTrack::read_json(bytes.as_slice()).unwrap();
    assert_eq!(read.frames().len(), 2);
    assert_eq!(read.events().len(), 1);
    assert_eq!(read.frames()[1].ropes[0].positions_m, final_positions);
    assert_eq!(read.definitions()[0].definition, support::spec("rope"));
    assert_eq!(
        read.frames()[0].ropes[0].radius_m.value(),
        track.frames()[0].ropes[0].radius_m.value()
    );
    assert!(
        String::from_utf8(bytes)
            .unwrap()
            .contains("playback_only_not_solver_checkpoint")
    );
}
#[test]
fn reader_rejects_schema_bad_sampling_and_malformed_snapshot() {
    let (_, _, _, mut track, s) = recorded();
    track.push_frame(frame(s)).unwrap();
    let mut bytes = vec![];
    track.write_json(&mut bytes).unwrap();
    let original: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
    for field in [
        "schema",
        "sampling",
        "positions",
        "impulse_age",
        "units",
        "force",
    ] {
        let mut value = original.clone();
        match field {
            "schema" => value["schema_version"] = 99.into(),
            "sampling" => value["ropes"][0]["reference_arc_lengths_m"][1] = 99.into(),
            "positions" => value["frames"][0]["ropes"][0]["positions_m"] = serde_json::json!([]),
            "impulse_age" => {
                value["frames"][0]["ropes"][0]["diagnostics"]["impulse_metadata"]["last_completed_step"] =
                    9.into()
            }
            "units" => value["units"]["length"] = "mm".into(),
            _ => {
                value["frames"][0]["ropes"][0]["diagnostics"]["unprovided"]["average_tension_n"] =
                    serde_json::json!({"status":"available","data":[]})
            }
        }
        assert!(
            RopeTrack::read_json(serde_json::to_vec(&value).unwrap().as_slice()).is_err(),
            "{field}"
        );
    }
}
#[test]
fn invalid_snapshot_json_and_unordered_frames_events_are_rejected_atomically() {
    let (_, _, _, mut track, mut s) = recorded();
    s.positions_m[0][0] = f64::NAN;
    assert!(serde_json::to_string(&s).is_err());
    assert!(track.push_frame(frame(s)).is_err());
    assert!(track.frames().is_empty());
    let (_, _, rope, mut track, s) = recorded();
    let first = frame(s);
    track.push_frame(first.clone()).unwrap();
    assert!(track.push_frame(first).is_err());
    assert_eq!(track.frames().len(), 1);
    let bad = TrackEvent {
        step: 0,
        time_s: FiniteScalar::new(0.0).unwrap(),
        operation: TrackEventKind::Pin {
            rope: rope.into(),
            particle: 999,
        },
    };
    assert!(track.record_event(bad).is_err());
    assert!(track.events().is_empty());
    track
        .record_event(TrackEvent {
            step: 2,
            time_s: FiniteScalar::new(2.0 * H).unwrap(),
            operation: TrackEventKind::CaptureRejected {
                reason: "non-finite native state".into(),
            },
        })
        .unwrap();
    assert!(
        track
            .record_event(TrackEvent {
                step: 1,
                time_s: FiniteScalar::new(H).unwrap(),
                operation: TrackEventKind::CaptureRejected {
                    reason: "older observation".into()
                }
            })
            .is_err()
    );
}
