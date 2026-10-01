use rapier_rope::{rapier::prelude::*, *};
#[path = "support/native_harness.rs"]
#[allow(dead_code)]
mod native_probe;

const ID: WorldId = WorldId(71);
const DT: f64 = 1.0 / 240.0;
#[allow(clippy::unnecessary_cast)]
fn r(x: f64) -> Real {
    x as Real
}
pub fn y() -> HarnessSpec {
    let nodes = vec![
        JunctionSpec::new("junction", [0.0, 1.0, 0.0]),
        JunctionSpec::new("tail", [0.0, 0.5, 0.0]),
        JunctionSpec::new("left", [-0.4, 1.3, 0.0]),
        JunctionSpec::new("right", [0.4, 1.3, 0.0]),
    ];
    let spans = [
        ("stem", "tail", 0.1),
        ("left_branch", "left", 0.2),
        ("right_branch", "right", 0.3),
    ]
    .into_iter()
    .enumerate()
    .map(|(i, (name, end, density))| {
        SpanSpec::new(
            name,
            "junction",
            end,
            NativeRopeMaterial::new(
                density,
                SpringSettings::new(500.0 + i as f64 * 100.0, 1.0),
                SpringSettings::new(20.0 + i as f64, 0.8),
            ),
            SamplingSettings::new(0.125),
        )
    })
    .collect();
    HarnessSpec::new("Y", nodes, spans, CollisionSettings::new(0.005))
}
fn setup() -> (HarnessSet, PhysicsWorld, HarnessHandle) {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = r(DT);
    let mut set = HarnessSet::new(ID).unwrap();
    let h = set.insert(ID, &mut world, &y()).unwrap();
    (set, world, h)
}
fn location(name: &str) -> HarnessLocation {
    HarnessLocation::Junction(name.into())
}
fn advance(
    set: &mut HarnessSet,
    world: &mut PhysicsWorld,
    commands: &[HarnessCommand],
) -> HarnessPreparedStep {
    let step = set.next_step();
    let p = set.prepare(ID, world, step, DT, commands).unwrap();
    world.step();
    set.inspect(ID, world, step).unwrap();
    p
}
#[test]
fn shared_vertex_mass_and_per_span_native_edges_are_exactly_mapped() {
    let spec = y();
    let b = build_harness(&spec).unwrap();
    let s = b.sampled();
    let n = b.native_builder();
    assert_eq!(s.reference_positions_m().len(), 13);
    assert_eq!(s.structural_edges().len(), 12);
    assert_eq!(s.bend_edges().len(), 9);
    assert_eq!(n.wire, s.wire_segments());
    assert!(n.surface.is_empty());
    assert_eq!(n.positions.len(), 13);
    assert_eq!(n.masses.len(), 13);
    let j = s.junction_particles()["junction"];
    assert!((s.nominal_mass_kg() - 0.3).abs() < 1e-12);
    assert!((s.particle_masses_kg().iter().sum::<f64>() - 0.3).abs() < 1e-12);
    assert!((s.particle_masses_kg()[j as usize] - 0.0375).abs() < 1e-12);
    for (i, span) in spec.spans.iter().enumerate() {
        let map = &s.spans()[&span.name];
        assert_eq!(map.particle_indices()[0], j);
        assert_eq!(
            map.particle_indices().last(),
            s.junction_particles().get(&span.end)
        );
        assert_eq!(map.structural_edge_range(), 4 * i..4 * (i + 1));
        assert_eq!(map.bending_edge_range(), 3 * i..3 * (i + 1));
        let expected = SpringCoefficients::new(r(span.material.axial.natural_frequency_hz), r(1.0));
        for k in map.structural_edge_range() {
            assert_eq!(
                n.edge_softness
                    .iter()
                    .find(|(idx, _)| *idx == k as u32)
                    .unwrap()
                    .1,
                expected
            );
        }
        for k in map.bending_edge_range() {
            assert_eq!(
                n.edge_softness
                    .iter()
                    .find(|(idx, _)| *idx == (12 + k) as u32)
                    .unwrap()
                    .1,
                SpringCoefficients::new(r(span.material.bending.natural_frequency_hz), r(0.8))
            );
        }
        for edge in &s.bend_edges()[map.bending_edge_range()] {
            assert!(edge.iter().all(|v| map.particle_indices().contains(v)));
        }
    }
    assert_eq!(n.tension_only_edges, (0..12).collect::<Vec<_>>());
    assert!(n.edges.iter().chain(&n.bend_edges).all(|e| e[0] != e[1]));
}
#[test]
fn span_required_samples_corners_and_material_locations_survive_endpoint_sharing() {
    let mut spec = y();
    spec.spans[1].interior_points_m = vec![[-0.2, 1.1, 0.0]];
    spec.spans[1].sampling.required_samples_m = vec![0.31];
    spec.spans[1]
        .named_locations
        .push(NamedLocation::new("clip", 0.2));
    let b = build_harness(&spec).unwrap();
    let s = b.sampled();
    let left = &s.spans()["left_branch"];
    let named = s
        .resolve_location(&HarnessLocation::Span {
            span: "left_branch".into(),
            location: RopeLocation::Named("clip".into()),
        })
        .unwrap();
    let local = &left.reference().named_locations()["clip"];
    assert_eq!(
        named.particle_index,
        left.particle_indices()[local.particle_index as usize]
    );
    assert!((named.material.unwrap().actual_arc_length_m - 0.2).abs() < 1e-12);
    assert!(
        left.reference()
            .reference_positions_m()
            .contains(&[-0.2, 1.1, 0.0])
    );
    let required = &left.reference().required_samples()[0];
    assert!((required.actual_arc_length_m - 0.31).abs() < 1e-12);
    assert_eq!(
        s.resolve_location(&location("junction")).unwrap().material,
        None
    );
    assert!(
        s.resolve_location(&HarnessLocation::Span {
            span: "left_branch".into(),
            location: RopeLocation::ArcLength {
                arc_length_m: 0.201,
                tolerance_m: 0.00001
            }
        })
        .is_err()
    );
    assert!(
        s.resolve_location(&HarnessLocation::Span {
            span: "missing".into(),
            location: RopeLocation::Start
        })
        .is_err()
    );
}
#[test]
fn bad_graphs_and_common_body_setting_overrides_fail_before_world_mutation() {
    let mut cases = Vec::new();
    let mut s = y();
    s.spans.pop();
    cases.push(s);
    let mut s = y();
    let mut edge = s.spans[0].clone();
    edge.name = "cycle".into();
    edge.start = "left".into();
    edge.end = "right".into();
    s.spans.push(edge);
    cases.push(s);
    let mut s = y();
    s.spans[0].end = "junction".into();
    cases.push(s);
    let mut s = y();
    s.spans[0].end = "absent".into();
    cases.push(s);
    let mut s = y();
    s.junctions[1].name = "junction".into();
    cases.push(s);
    let mut s = y();
    s.spans[1].name = "stem".into();
    cases.push(s);
    let mut s = y();
    s.spans[0].collision = Some(CollisionSettings::new(0.01));
    cases.push(s);
    let mut s = y();
    let mut c = s.collision.clone();
    c.friction = 0.9;
    s.spans[0].collision = Some(c);
    cases.push(s);
    let mut s = y();
    s.spans[1].material.linear_damping = 2.0;
    cases.push(s);
    let mut s = y();
    s.junctions[1].reference_position_m = s.junctions[0].reference_position_m;
    cases.push(s);
    let mut s = y();
    s.max_particles = 12;
    cases.push(s);
    for budget in [0, 1, usize::MAX] {
        let mut s = y();
        s.spans[0].sampling.max_particles = budget;
        cases.push(s);
    }
    let mut s = y();
    s.required_capabilities
        .push(RopeCapability::OrientationClamp);
    cases.push(s);
    let mut s = y();
    s.placement.rotation_xyzw = [0.0; 4];
    cases.push(s);
    let mut s = y();
    s.spans[1].material.linear_density_kg_m = f64::INFINITY;
    cases.push(s);
    let mut world = PhysicsWorld::new();
    let mut set = HarnessSet::new(ID).unwrap();
    for s in cases {
        assert!(set.insert(ID, &mut world, &s).is_err(), "{s:?}");
        assert!(
            set.is_empty()
                && world.soft_bodies.is_empty()
                && world.bodies.is_empty()
                && world.colliders.is_empty()
        );
    }
}
#[test]
fn common_collision_settings_and_mixed_axial_response_are_supported() {
    let mut s = y();
    s.collision.self_contacts = true;
    s.spans[0].collision = Some(s.collision.clone());
    s.spans[1].material.axial_response = AxialResponse::TensionAndCompression;
    let b = build_harness(&s).unwrap();
    assert!(b.native_builder().self_contacts);
    assert_eq!(
        b.native_builder().tension_only_edges,
        vec![0, 1, 2, 3, 8, 9, 10, 11]
    );
    let bytes = serde_json::to_vec(&s).unwrap();
    assert_eq!(serde_json::from_slice::<HarnessSpec>(&bytes).unwrap(), s);
}
#[test]
fn stale_world_foreign_set_and_whole_harness_removal_preserve_external_bodies() {
    let (mut set, mut world, h) = setup();
    let other = HarnessSet::new(ID).unwrap();
    assert!(matches!(
        set.get(WorldId(9), &world, h).err().unwrap().kind,
        RopeSetErrorKind::WorldMismatch { .. }
    ));
    assert!(matches!(
        other.get(ID, &world, h).err().unwrap().kind,
        RopeSetErrorKind::ForeignSet { .. }
    ));
    let rb = world.insert_body(RigidBodyBuilder::dynamic());
    assert_eq!(set.get_by_name(ID, &world, "Y").unwrap().unwrap().handle, h);
    assert!(set.insert(ID, &mut world, &y()).is_err());
    let gone = set.remove(ID, &mut world, h).unwrap();
    assert!(gone.native_removed);
    assert_eq!(gone.samples.spans().len(), 3);
    assert!(world.soft_bodies.is_empty() && world.colliders.is_empty());
    assert_eq!(world.bodies.len(), 1);
    assert!(world.bodies.contains(rb));
    let replacement = set.insert(ID, &mut world, &y()).unwrap();
    assert_eq!(replacement.slot(), h.slot());
    assert_ne!(replacement.generation(), h.generation());
    assert!(set.get(ID, &world, h).is_err());
}
#[test]
fn topology_changes_and_external_deletion_invalidate_the_entire_harness() {
    let (mut set, mut world, h) = setup();
    let n = set.get(ID, &world, h).unwrap().native_handle;
    world.soft_bodies[n].tear_edge(0);
    assert!(matches!(
        set.get(ID, &world, h).err().unwrap().kind,
        RopeSetErrorKind::TopologyChanged(_)
    ));
    set.remove(ID, &mut world, h).unwrap();
    let h = set.insert(ID, &mut world, &y()).unwrap();
    let n = set.get(ID, &world, h).unwrap().native_handle;
    world.remove_soft_body(n);
    assert!(matches!(
        set.get(ID, &world, h).err().unwrap().kind,
        RopeSetErrorKind::MissingSoftBody(_)
    ));
    assert!(!set.remove(ID, &mut world, h).unwrap().native_removed);
}
#[test]
fn endpoint_aliases_conflict_atomically_and_release_can_precede_reacquisition() {
    let (mut set, mut world, h) = setup();
    let a = HarnessLocation::Span {
        span: "stem".into(),
        location: RopeLocation::Start,
    };
    let b = HarnessLocation::Span {
        span: "left_branch".into(),
        location: RopeLocation::Start,
    };
    let commands = [
        HarnessCommand::Pin {
            harness: h,
            location: a.clone(),
            position_m: [0.0, 1.0, 0.0],
        },
        HarnessCommand::Pin {
            harness: h,
            location: b,
            position_m: [0.0, 1.0, 0.0],
        },
    ];
    let e = set.prepare(ID, &mut world, 0, DT, &commands).unwrap_err();
    assert_eq!(e.applied_commands, 0);
    assert!(!set.get(ID, &world, h).unwrap().soft_body.particles()[0].is_pinned());
    advance(&mut set, &mut world, &commands[..1]);
    let rb = world.insert_body(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 1.0, 0.0)),
    );
    let p = advance(
        &mut set,
        &mut world,
        &[
            HarnessCommand::Unpin {
                harness: h,
                location: a,
            },
            HarnessCommand::Attach {
                harness: h,
                location: location("junction"),
                body: rb,
            },
        ],
    );
    assert_eq!(p.created_attachments.len(), 1);
    assert_eq!(p.created_attachments[0].particle, 0);
}
#[test]
fn connector_local_anchor_mass_and_detach_velocity_follow_native_contract() {
    let (mut set, mut world, h) = setup();
    let mut spec = y();
    spec.name = "second".into();
    let other = set.insert(ID, &mut world, &spec).unwrap();
    set.remove(ID, &mut world, other).unwrap();
    let (rb, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.45, 1.25, 0.0))
            .rotation(Vector::new(0.0, 0.0, 0.7))
            .can_sleep(false),
        ColliderBuilder::ball(r(0.025)).mass(r(0.05)),
    );
    let before: Vec<Point3> = set.get(ID, &world, h).unwrap().positions_m().collect();
    let p = set
        .prepare(
            ID,
            &mut world,
            0,
            DT,
            &[HarnessCommand::Attach {
                harness: h,
                location: location("right"),
                body: rb,
            }],
        )
        .unwrap();
    assert_eq!(
        before,
        set.get(ID, &world, h)
            .unwrap()
            .positions_m()
            .collect::<Vec<_>>()
    );
    let a = &p.created_attachments[0];
    let point = world.bodies[rb]
        .position()
        .transform_point(Vector::from_array(a.local_anchor_m.map(r)));
    assert!((point - Vector::from_array([0.4, 1.3, 0.0])).length() < r(1e-6));
    assert!((world.bodies[rb].mass() - r(0.05)).abs() < r(1e-6));
    assert!((set.get(ID, &world, h).unwrap().samples.nominal_mass_kg() - 0.3).abs() < 1e-12);
    world.step();
    set.inspect(ID, &world, 0).unwrap();
    let v: Vec<_> = set.get(ID, &world, h).unwrap().velocities_m_s().collect();
    set.prepare(
        ID,
        &mut world,
        1,
        DT,
        &[HarnessCommand::Detach {
            attachment: a.handle,
        }],
    )
    .unwrap();
    assert_eq!(
        v,
        set.get(ID, &world, h)
            .unwrap()
            .velocities_m_s()
            .collect::<Vec<_>>()
    );
    world.step();
    set.inspect(ID, &world, 1).unwrap();
    assert!(set.get_attachment(ID, &world, a.handle).is_err());
}

#[test]
fn graph_snapshot_preserves_span_diagnostics_without_invented_junction_curvature() {
    let (mut set, mut world, h) = setup();
    advance(&mut set, &mut world, &[]);
    let view = set.get(ID, &world, h).unwrap();
    let snap = view
        .snapshot(CaptureStamp::new(CapturePhase::AfterStep, Some(0), DT, DT).unwrap())
        .unwrap();
    assert_eq!(snap.span_geometry.len(), 3);
    assert_eq!(snap.span_particles.len(), 3);
    assert_eq!(snap.segments.len(), 12);
    assert!(matches!(
        snap.junction_curvature,
        Observation::Unsupported { .. }
    ));
    assert!(matches!(
        snap.unprovided.axial_torsion_rad,
        Observation::Unsupported { .. }
    ));
    let decoded: HarnessSnapshot =
        serde_json::from_slice(&serde_json::to_vec(&snap).unwrap()).unwrap();
    assert_eq!(decoded.positions_m, snap.positions_m);
    assert!(
        view.snapshot(CaptureStamp::new(CapturePhase::BeforeStep, Some(0), 0.0, DT).unwrap())
            .is_err()
    );
}
#[test]
fn native_y_branch_response_and_contacts_have_disabled_controls() {
    assert_eq!(native_probe::response()["passed"], true);
    let (on, yes) = native_probe::contact(true);
    let (off, no) = native_probe::contact(false);
    assert_eq!(yes["finite"], true);
    assert_eq!(no["finite"], true);
    assert!(yes["witness_steps"].as_u64().unwrap() > 0);
    assert_eq!(no["edge_witnesses"], 0);
    let delta = on
        .iter()
        .zip(off)
        .map(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2]))
        .fold(0.0, f64::max);
    assert!(delta > 0.005);
}

#[test]
fn registered_harness_transmits_a_pulled_branch_through_shared_junction() {
    let (mut set, mut world, h) = setup();
    world.gravity = Vector::ZERO;
    advance(
        &mut set,
        &mut world,
        &[
            HarnessCommand::Pin {
                harness: h,
                location: location("tail"),
                position_m: [0.0, 0.5, 0.0],
            },
            HarnessCommand::Pin {
                harness: h,
                location: location("left"),
                position_m: [-0.4, 1.3, 0.0],
            },
        ],
    );
    for k in 0..240 {
        let u = (k + 1) as f64 / 240.0;
        advance(
            &mut set,
            &mut world,
            &[HarnessCommand::MovePin {
                harness: h,
                location: location("left"),
                target_m: [-0.4 - 0.2 * u, 1.3 + 0.15 * u, 0.0],
            }],
        );
    }
    let v = set.get(ID, &world, h).unwrap();
    let p: Vec<_> = v.positions_m().collect();
    let j = p[v.samples.junction_particles()["junction"] as usize];
    let other = p[v.samples.junction_particles()["right"] as usize];
    assert!((j[0]).hypot(j[1] - 1.0).hypot(j[2]) > 0.01);
    assert!((other[0] - 0.4).hypot(other[1] - 1.3).hypot(other[2]) > 0.01);
}

#[test]
fn registered_harness_branch_contacts_have_disabled_controls() {
    fn run(enabled: bool) -> (Vec<Point3>, usize) {
        let nodes = vec![
            JunctionSpec::new("J", [0.6, 0.6, 0.0]),
            JunctionSpec::new("tail", [0.8, 0.8, 0.0]),
            JunctionSpec::new("lower", [-0.5, 0.2, 0.0]),
            JunctionSpec::new("upper", [-0.5, 0.4, 0.003]),
        ];
        let material = NativeRopeMaterial::new(
            0.1,
            SpringSettings::new(500.0, 1.0),
            SpringSettings::new(20.0, 0.8),
        );
        let mut spans = vec![
            SpanSpec::new(
                "tail",
                "J",
                "tail",
                material.clone(),
                SamplingSettings::new(0.05),
            ),
            SpanSpec::new(
                "lower",
                "J",
                "lower",
                material.clone(),
                SamplingSettings::new(0.05),
            ),
            SpanSpec::new("upper", "J", "upper", material, SamplingSettings::new(0.05)),
        ];
        spans[1].interior_points_m = vec![[0.5, 0.2, 0.0]];
        spans[2].interior_points_m = vec![[0.5, 0.4, 0.003]];
        let mut collision = CollisionSettings::new(0.01);
        collision.self_contacts = enabled;
        let spec = HarnessSpec::new("branch contact", nodes, spans, collision);
        let mut world = PhysicsWorld::new();
        world.integration_parameters.dt = r(DT);
        let mut set = HarnessSet::new(ID).unwrap();
        let h = set.insert(ID, &mut world, &spec).unwrap();
        let view = set.get(ID, &world, h).unwrap();
        let native = view.native_handle;
        let lower = &view.samples.spans()["lower"];
        let pins: Vec<_> = lower
            .reference()
            .arc_lengths_m()
            .iter()
            .zip(lower.reference().reference_positions_m())
            .map(|(&s, &p)| HarnessCommand::Pin {
                harness: h,
                location: HarnessLocation::Span {
                    span: "lower".into(),
                    location: RopeLocation::ArcLength {
                        arc_length_m: s,
                        tolerance_m: 0.0,
                    },
                },
                position_m: p,
            })
            .collect();
        let mut contacts = 0;
        for k in 0..480 {
            advance(&mut set, &mut world, if k == 0 { &pins } else { &[] });
            contacts += world.soft_bodies[native]
                .edge_contact_segments(&world.soft_bodies)
                .count();
        }
        (
            set.get(ID, &world, h).unwrap().positions_m().collect(),
            contacts,
        )
    }
    let (on, witnesses) = run(true);
    let (off, disabled) = run(false);
    assert!(witnesses > 0);
    assert_eq!(disabled, 0);
    let delta = on
        .iter()
        .zip(off)
        .map(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2]))
        .fold(0.0, f64::max);
    assert!(delta > 0.005, "{delta}");
}

#[test]
fn junction_contact_neighborhood_does_not_repulse_a_resting_y() {
    let mut spec = y();
    spec.collision.self_contacts = true;
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.dt = r(DT);
    let mut set = HarnessSet::new(ID).unwrap();
    let h = set.insert(ID, &mut world, &spec).unwrap();
    let v = set.get(ID, &world, h).unwrap();
    let n = v.native_handle;
    let before: Vec<_> = v.positions_m().collect();
    for _ in 0..8 {
        advance(&mut set, &mut world, &[]);
        assert_eq!(
            world.soft_bodies[n]
                .edge_contact_segments(&world.soft_bodies)
                .count(),
            0
        );
    }
    let after: Vec<_> = set.get(ID, &world, h).unwrap().positions_m().collect();
    assert!(
        before
            .iter()
            .zip(after)
            .all(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2]) < 1e-6)
    );
}

#[test]
fn prepared_harness_cannot_be_removed_and_post_step_failure_advances_protocol() {
    let (mut set, mut world, h) = setup();
    let target = world.insert_body(RigidBodyBuilder::dynamic());
    set.prepare(
        ID,
        &mut world,
        0,
        DT,
        &[HarnessCommand::Attach {
            harness: h,
            location: location("left"),
            body: target,
        }],
    )
    .unwrap();
    assert!(matches!(
        set.remove(ID, &mut world, h).unwrap_err().kind,
        RopeSetErrorKind::RegistryBusy { .. }
    ));
    world.step();
    world.remove_body(target);
    let err = set.inspect(ID, &world, 0).unwrap_err();
    assert_eq!(err.phase, RegistryPhase::Inspect);
    assert_eq!(set.next_step(), 1);
    set.remove(ID, &mut world, h).unwrap();
    assert!(world.soft_bodies.is_empty());
}
