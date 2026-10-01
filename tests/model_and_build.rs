#![allow(clippy::unnecessary_cast)] // Same assertions compile for f32 and f64.
use rapier_rope::{rapier::prelude::*, *};

fn spec() -> RopeSpec {
    RopeSpec::new(
        "test-rope",
        vec![[0.0; 3], [1.0, 0.0, 0.0]],
        NativeRopeMaterial::new(
            0.1,
            SpringSettings::new(500.0, 1.0),
            SpringSettings::new(20.0, 0.8),
        ),
        SamplingSettings::new(1.0 / 32.0),
        CollisionSettings::new(0.005),
    )
}

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} != {expected} (tol {tolerance})"
    );
}

#[test]
fn resolution_preserves_length_mass_and_radius_in_both_precisions() {
    let mass_tol = if cfg!(feature = "f32") { 1e-5 } else { 1e-12 };
    for segments in [1, 32, 64, 128] {
        let mut input = spec();
        input.sampling.max_segment_length_m = 1.0 / segments as f64;
        let build = build_rope(&input).unwrap();
        let sample = build.sampled();
        assert_eq!(sample.reference_positions_m().len(), segments + 1);
        assert_eq!(sample.reference_length_m(), 1.0);
        assert_close(sample.particle_masses_kg().iter().sum(), 0.1, 1e-14);
        assert_close(sample.nominal_mass_kg(), 0.1, 1e-14);
        let native = SoftBody::from(build.native_builder().clone().pinned_particles([0]));
        // Pinning sets inverse mass to zero while preserving nominal mass.
        let nominal_mass: f64 = native.particles().iter().map(|p| p.mass() as f64).sum();
        assert_close(nominal_mass / 0.1, 1.0, mass_tol);
        assert_eq!(native.particles()[0].inv_mass(), 0.0);
        assert!(native.particles()[0].mass() > 0.0);
        assert_close(native.particle_radius() as f64, 0.005, 1e-8);
        assert_eq!(sample.original_vertex_indices(), &[0, segments as u32]);
    }
}

#[test]
fn corners_named_and_required_samples_survive_nonuniform_sampling() {
    let mut input = spec();
    input.reference_centerline_m = vec![[0.0; 3], [0.3, 0.0, 0.0], [0.3, 0.4, 0.0]];
    input.sampling.max_segment_length_m = 0.2;
    input.sampling.required_samples_m = vec![0.6, 0.1];
    input.named_locations = vec![
        NamedLocation::new("corner", 0.3),
        NamedLocation::new("grip", 0.5),
    ];
    let build = build_rope(&input).unwrap();
    let s = build.sampled();
    assert_close(s.reference_length_m(), 0.7, 1e-15);
    for (point, &index) in input
        .reference_centerline_m
        .iter()
        .zip(s.original_vertex_indices())
    {
        assert_eq!(*point, s.reference_positions_m()[index as usize]);
    }
    for (&expected, actual) in input
        .sampling
        .required_samples_m
        .iter()
        .zip(s.required_samples())
    {
        assert_eq!(actual.requested_arc_length_m, expected);
        assert_eq!(actual.actual_arc_length_m, expected);
    }
    assert_eq!(
        s.resolve_location(&RopeLocation::Named("corner".into()))
            .unwrap()
            .particle_index,
        s.original_vertex_indices()[1]
    );
    assert!(
        s.reference_edge_lengths_m()
            .iter()
            .all(|&v| v <= 0.2 + 1e-15)
    );
    assert_close(s.particle_masses_kg().iter().sum(), 0.07, 1e-14);
    // Sum of geometric lengths must equal the polyline length: no shortcut across the corner.
    let geometry: f64 = s
        .reference_positions_m()
        .windows(2)
        .map(|p| {
            (p[1][0] - p[0][0])
                .hypot(p[1][1] - p[0][1])
                .hypot(p[1][2] - p[0][2])
        })
        .sum();
    assert_close(geometry, 0.7, 1e-15);
}

#[test]
fn each_edge_distributes_half_its_reference_mass_to_each_endpoint() {
    let mut input = spec();
    input.reference_centerline_m = vec![[0.0; 3], [0.3, 0.0, 0.0], [0.3, 0.4, 0.0]];
    input.sampling.max_segment_length_m = 0.5;
    let s = sample_rope(&input).unwrap();
    for (&actual, expected) in s.particle_masses_kg().iter().zip([0.015, 0.035, 0.02]) {
        assert_close(actual, expected, 1e-16);
    }
}

#[test]
fn nearby_requests_merge_without_accumulating_snap_error() {
    let mut input = spec();
    input.sampling.max_segment_length_m = 0.5;
    input.sampling.merge_tolerance_m = 0.01;
    input.sampling.required_samples_m = vec![0.218, 0.209, 0.2];
    input.named_locations = vec![NamedLocation::new("near", 0.209)];
    let s = sample_rope(&input).unwrap();
    let r = s.required_samples();
    assert_eq!(r[2].actual_arc_length_m, 0.2);
    assert_eq!(r[1].actual_arc_length_m, 0.2);
    assert_eq!(r[0].actual_arc_length_m, 0.218);
    assert!(r.iter().all(|r| r.error_m <= 0.01));
    assert_eq!(s.named_locations()["near"], r[1]);
}

#[test]
fn close_original_vertices_are_never_merged() {
    let mut input = spec();
    input.reference_centerline_m = vec![[0.0; 3], [0.0001, 0.0, 0.0], [1.0, 0.0, 0.0]];
    input.sampling.merge_tolerance_m = 0.01;
    let s = sample_rope(&input).unwrap();
    assert_ne!(
        s.original_vertex_indices()[0],
        s.original_vertex_indices()[1]
    );
    assert_eq!(
        s.reference_positions_m()[s.original_vertex_indices()[1] as usize],
        [0.0001, 0.0, 0.0]
    );
}

#[test]
fn location_resolution_uses_explicit_tolerance_and_reports_actual_arc() {
    let mut input = spec();
    input.sampling.max_segment_length_m = 0.5;
    let s = sample_rope(&input).unwrap();
    let r = s
        .resolve_location(&RopeLocation::ArcLength {
            arc_length_m: 0.49,
            tolerance_m: 0.011,
        })
        .unwrap();
    assert_eq!(r.particle_index, 1);
    assert_eq!(r.actual_arc_length_m, 0.5);
    assert_close(r.error_m, 0.01, 1e-16);
    let tie = s
        .resolve_location(&RopeLocation::ArcLength {
            arc_length_m: 0.25,
            tolerance_m: 0.25,
        })
        .unwrap();
    assert_eq!(tie.particle_index, 0);
    for location in [
        RopeLocation::Named("absent".into()),
        RopeLocation::ArcLength {
            arc_length_m: 0.49,
            tolerance_m: 0.001,
        },
        RopeLocation::ArcLength {
            arc_length_m: -1e-15,
            tolerance_m: 1.0,
        },
        RopeLocation::ArcLength {
            arc_length_m: f64::NAN,
            tolerance_m: 1.0,
        },
        RopeLocation::ArcLength {
            arc_length_m: 0.5,
            tolerance_m: -1.0,
        },
    ] {
        assert_eq!(
            s.resolve_location(&location).unwrap_err().rope_name,
            "test-rope"
        );
    }
    assert_eq!(
        s.resolve_location(&RopeLocation::End)
            .unwrap()
            .particle_index,
        2
    );
}

#[test]
fn two_particle_rope_has_no_bending_edges_or_invalid_indices() {
    let mut input = spec();
    input.sampling.max_segment_length_m = 2.0;
    let build = build_rope(&input).unwrap();
    let n = build.native_builder();
    assert_eq!(n.positions.len(), 2);
    assert_eq!(n.edges, [[0, 1]]);
    assert!(n.bend_edges.is_empty());
    assert_eq!(n.wire, [[0, 1]]);
    assert_eq!(n.tension_only_edges, [0]);
    assert_eq!(n.edge_softness.len(), 1);
    let body = SoftBody::from(n.clone());
    assert_eq!(body.edges().len(), 1);
}

#[test]
fn native_configuration_maps_families_collision_and_disables_failure_models() {
    let mut input = spec();
    input.collision.groups = CollisionGroups {
        memberships: 4,
        filter: 9,
    };
    input.collision.self_contacts = true;
    input.collision.friction = 0.7;
    input.dynamics.additional_solver_iterations = 2;
    input.dynamics.additional_pgs_iterations = 5;
    let build = build_rope(&input).unwrap();
    let n = build.native_builder();
    assert_eq!(n.tension_only_edges, (0..32).collect::<Vec<_>>());
    assert_eq!(n.edge_softness.len(), 63);
    for &(index, spring) in &n.edge_softness {
        let expected = if index < 32 {
            input.material.axial
        } else {
            input.material.bending
        };
        assert_close(
            spring.natural_frequency as f64,
            expected.natural_frequency_hz,
            1e-6,
        );
        assert_close(spring.damping_ratio as f64, expected.damping_ratio, 1e-6);
    }
    assert_eq!(n.material.plastic_yield, 0.0);
    assert_eq!(n.material.edge_plastic_yield, 0.0);
    assert!(n.material.tear_strain.is_none() && n.material.tear_force.is_none());
    assert!(n.self_contacts);
    assert_eq!(n.particle_settings.additional_solver_iterations, 2);
    assert_eq!(n.particle_settings.additional_pgs_iterations, 5);
    let collider = n.collider_template.clone().unwrap().build();
    assert_eq!(collider.collision_groups().memberships.bits(), 4);
    assert_eq!(collider.collision_groups().filter.bits(), 9);
    assert_close(collider.friction() as f64, 0.7, 1e-6);
    assert_eq!(build.specification(), &input);
    input.material.axial_response = AxialResponse::TensionAndCompression;
    assert!(
        build_rope(&input)
            .unwrap()
            .native_builder()
            .tension_only_edges
            .is_empty()
    );
}

#[test]
fn rigid_placement_preserves_reference_mass_and_distances() {
    let mut input = spec();
    input.reference_centerline_m = vec![[0.0; 3], [0.3, 0.0, 0.0], [0.3, 0.4, 0.0]];
    input.sampling.max_segment_length_m = 0.5;
    input.placement = RigidPlacement {
        translation_m: [1.0, 2.0, 3.0],
        rotation_xyzw: [
            0.0,
            0.0,
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
        ],
    };
    let build = build_rope(&input).unwrap();
    for (actual, expected) in build.native_builder().positions.iter().zip([
        [1.0, 2.0, 3.0],
        [1.0, 2.3, 3.0],
        [0.6, 2.3, 3.0],
    ]) {
        for (a, e) in [actual.x, actual.y, actual.z].into_iter().zip(expected) {
            assert_close(a as f64, e, 1e-6);
        }
    }
    assert_close(build.sampled().nominal_mass_kg(), 0.07, 1e-15);
    let body = SoftBody::from(build.native_builder().clone());
    for (edge, expected) in body.edges().iter().zip([0.3, 0.4, 0.5]) {
        assert_close(edge.rest_length as f64, expected, 1e-6);
    }
}

#[test]
fn invalid_values_report_rope_and_offending_field() {
    type Edit = fn(&mut RopeSpec);
    let cases: &[(&str, Edit)] = &[
        ("name", |s| s.name = " ".into()),
        ("reference_centerline_m", |s| {
            s.reference_centerline_m.clear()
        }),
        ("reference_centerline_m[1]", |s| {
            s.reference_centerline_m[1][0] = f64::NAN
        }),
        ("material.linear_density_kg_m", |s| {
            s.material.linear_density_kg_m = 0.0
        }),
        ("material.axial.natural_frequency_hz", |s| {
            s.material.axial.natural_frequency_hz = 0.0
        }),
        ("material.bending.damping_ratio", |s| {
            s.material.bending.damping_ratio = -0.1
        }),
        ("material.linear_damping", |s| {
            s.material.linear_damping = f64::INFINITY
        }),
        ("collision.radius_m", |s| s.collision.radius_m = -1.0),
        ("collision.friction", |s| s.collision.friction = -1.0),
        ("sampling.max_segment_length_m", |s| {
            s.sampling.max_segment_length_m = 0.0
        }),
        ("sampling.merge_tolerance_m", |s| {
            s.sampling.merge_tolerance_m = -1.0
        }),
        ("sampling.max_particles", |s| s.sampling.max_particles = 1),
        ("sampling.max_native_edge_relative_error", |s| {
            s.sampling.max_native_edge_relative_error = 1.0
        }),
        ("placement.translation_m", |s| {
            s.placement.translation_m[0] = f64::INFINITY
        }),
        ("placement.rotation_xyzw", |s| {
            s.placement.rotation_xyzw[3] = 2.0
        }),
        ("sampling.required_samples_m[0]", |s| {
            s.sampling.required_samples_m = vec![1.01]
        }),
        ("named_locations[0].name", |s| {
            s.named_locations = vec![NamedLocation::new("", 0.5)]
        }),
        ("named_locations[0].arc_length_m", |s| {
            s.named_locations = vec![NamedLocation::new("grip", -0.1)]
        }),
    ];
    for &(field, edit) in cases {
        let mut input = spec();
        edit(&mut input);
        let error = build_rope(&input).unwrap_err();
        assert_eq!(error.rope_name, input.name);
        assert_eq!(error.field, field);
        assert!(error.to_string().contains(field));
    }
}

#[test]
fn duplicate_names_zero_length_and_unsupported_capabilities_are_typed_errors() {
    let mut input = spec();
    input.named_locations = vec![
        NamedLocation::new("grip", 0.2),
        NamedLocation::new("grip", 0.8),
    ];
    assert_eq!(
        build_rope(&input).unwrap_err().kind,
        RopeErrorKind::DuplicateName("grip".into())
    );
    input = spec();
    input.reference_centerline_m[1] = input.reference_centerline_m[0];
    assert_eq!(
        build_rope(&input).unwrap_err().kind,
        RopeErrorKind::ZeroLengthSegment(0)
    );
    for capability in [
        RopeCapability::AxialTorsion,
        RopeCapability::OrientationClamp,
        RopeCapability::CalibratedElasticModuli,
    ] {
        input = spec();
        input.required_capabilities.push(capability);
        assert_eq!(
            build_rope(&input).unwrap_err().kind,
            RopeErrorKind::UnsupportedCapability(capability)
        );
    }
}

#[test]
fn excessive_sampling_is_rejected_before_particle_allocation() {
    let mut input = spec();
    input.sampling.max_segment_length_m = 1e-100;
    assert_eq!(
        build_rope(&input).unwrap_err().kind,
        RopeErrorKind::SamplingLimit {
            max_particles: 65_536
        }
    );
    input = spec();
    input.sampling.max_particles = 3;
    input.sampling.max_segment_length_m = 1.0;
    input.sampling.required_samples_m = vec![0.2, 0.3, 0.4];
    assert!(matches!(
        build_rope(&input).unwrap_err().kind,
        RopeErrorKind::SamplingLimit { .. }
    ));
}

#[test]
fn native_precision_loss_and_folded_back_bend_are_rejected() {
    let mut bad_spring = spec();
    bad_spring.material.axial.natural_frequency_hz = 1e200;
    assert!(matches!(
        build_rope(&bad_spring).unwrap_err().kind,
        RopeErrorKind::PrecisionLoss(_)
    ));
    bad_spring = spec();
    bad_spring.material.bending.damping_ratio = 1e-200;
    assert!(matches!(
        build_rope(&bad_spring).unwrap_err().kind,
        RopeErrorKind::PrecisionLoss(_)
    ));
    let mut input = spec();
    input.placement.translation_m[0] = 1e20;
    assert!(matches!(
        build_rope(&input).unwrap_err().kind,
        RopeErrorKind::PrecisionLoss(_)
    ));
    input = spec();
    input.reference_centerline_m = vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0; 3]];
    input.sampling.max_segment_length_m = 1.0;
    assert!(matches!(
        build_rope(&input).unwrap_err().kind,
        RopeErrorKind::PrecisionLoss(_)
    ));
    #[cfg(feature = "f32")]
    {
        input = spec();
        input.material.linear_density_kg_m = 1e-100;
        assert!(matches!(
            build_rope(&input).unwrap_err().kind,
            RopeErrorKind::PrecisionLoss(_)
        ));
        input = spec();
        input.sampling.max_native_edge_relative_error = 1e-4;
        input.placement.translation_m[0] = 1e5;
        // Representable nonzero edges may still have an unacceptable rest-length change.
        input.sampling.max_segment_length_m = 0.1;
        assert!(matches!(
            build_rope(&input).unwrap_err().kind,
            RopeErrorKind::PrecisionLoss(_)
        ));
    }
}

#[test]
fn failed_build_leaves_caller_world_untouched_and_snapshot_is_owned() {
    let mut world = PhysicsWorld::new();
    world.insert_body(RigidBodyBuilder::fixed());
    let mut input = spec();
    let build = build_rope(&input).unwrap();
    input.collision.radius_m = 0.0;
    assert!(build_rope(&input).is_err());
    assert_eq!(world.bodies.len(), 1);
    assert_eq!(world.soft_bodies.len(), 0);
    assert_eq!(world.colliders.len(), 0);
    let (native, samples, saved) = build.into_parts();
    assert_eq!(saved.collision.radius_m, 0.005);
    let handle = world.insert_soft_body(native);
    assert_eq!(
        world.soft_bodies[handle].num_particles(),
        samples.reference_positions_m().len()
    );
}
