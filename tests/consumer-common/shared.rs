use rapier_rope::{rapier::prelude::*, *};

#[allow(clippy::unnecessary_cast)]
fn r(value: f64) -> Real {
    value as Real
}
fn run(expected_real_bytes: usize) -> Result<(), Box<dyn std::error::Error>> {
    assert_eq!(std::mem::size_of::<Real>(), expected_real_bytes);
    let id = WorldId(101);
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = r(1.0 / 240.0);
    world.integration_parameters.num_solver_iterations = 8;
    let mut ropes = RopeSet::new(id)?;
    let mut spec = RopeSpec::new(
        "consumer cable",
        vec![[0.0, 1.5, 0.0], [1.0, 1.5, 0.0]],
        NativeRopeMaterial::new(
            0.1,
            SpringSettings::new(500.0, 1.0),
            SpringSettings::new(20.0, 0.8),
        ),
        SamplingSettings::new(1.0 / 32.0),
        CollisionSettings::new(0.005),
    );
    spec.named_locations
        .push(NamedLocation::new("connector", 1.0));
    let rope = ropes.insert(id, &mut world, &spec)?;
    // The body and its mass belong to the caller's re-exported Rapier world.
    let (body, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(1.0, 1.44, 0.0))
            .can_sleep(false),
        ColliderBuilder::ball(r(0.05)).mass(r(0.05)),
    );
    let mut track = RopeTrack::new("consumer lifecycle", "right-handed, Y-up, SI", &world)?;
    track.register_rope(&ropes.centerline(id, &world, rope)?)?;
    #[allow(clippy::useless_conversion)]
    let dt = f64::from(world.integration_parameters.dt);
    let mut attachment = None;
    for step in 0..96_u64 {
        let commands = if step == 0 {
            vec![
                AttachmentCommand::Pin {
                    rope,
                    location: RopeLocation::Start,
                    position_m: [0.0, 1.5, 0.0],
                },
                AttachmentCommand::Attach {
                    rope,
                    location: RopeLocation::Named("connector".into()),
                    body,
                },
            ]
        } else if step == 48 {
            vec![AttachmentCommand::Detach {
                attachment: attachment.unwrap(),
            }]
        } else if step == 64 {
            vec![AttachmentCommand::Unpin {
                rope,
                location: RopeLocation::Start,
            }]
        } else {
            vec![]
        };
        let before = ropes
            .centerline(id, &world, rope)?
            .velocities_m_s()
            .collect::<Vec<_>>();
        let prepared = ropes.prepare_attachments(id, &mut world, step, 1.0 / 240.0, &commands)?;
        if step == 0 {
            let created = &prepared.prepared.created_attachments[0];
            assert_eq!(created.particle, 32);
            attachment = Some(created.handle);
        }
        if step == 48 || step == 64 {
            assert_eq!(
                before,
                ropes
                    .centerline(id, &world, rope)?
                    .velocities_m_s()
                    .collect::<Vec<_>>()
            );
        }
        world.step();
        let report = ropes.inspect(id, &world, step)?;
        assert_eq!(report.ropes.len(), 1);
        let view = ropes.centerline(id, &world, rope)?;
        assert_eq!(view.name(), "consumer cable");
        if (step + 1) % 8 == 0 {
            let stamp = CaptureStamp::new(
                CapturePhase::AfterStep,
                Some(step),
                (step + 1) as f64 * dt,
                dt,
            )?;
            let snapshot = view.snapshot(stamp.clone())?;
            assert!(
                snapshot
                    .diagnostics
                    .geometry
                    .current_length_m
                    .value()
                    .unwrap()
                    > 0.0
            );
            track.push_frame(TrackFrame {
                capture: stamp,
                ropes: vec![snapshot],
                bodies: vec![],
            })?;
        }
    }
    let mut bytes = vec![];
    track.write_json(&mut bytes)?;
    let read = RopeTrack::read_json(bytes.as_slice())?;
    assert_eq!(read.frames().len(), 12);
    assert_eq!(read.frames().last().unwrap().capture.step, Some(95));
    assert_eq!(
        read.frames().last().unwrap().ropes[0].positions_m,
        track.frames().last().unwrap().ropes[0].positions_m
    );
    ropes.remove(id, &mut world, rope)?;
    assert!(ropes.get(id, &world, rope).is_err());
    assert_eq!(world.bodies.len(), 1);
    // The distributed harness API uses one body and reuses the same caller world.
    let material = NativeRopeMaterial::new(
        0.1,
        SpringSettings::new(500.0, 1.0),
        SpringSettings::new(20.0, 0.8),
    );
    let harness_spec = HarnessSpec::new(
        "consumer Y",
        vec![
            JunctionSpec::new("J", [0.0, 1.5, 0.0]),
            JunctionSpec::new("mount", [0.0, 2.0, 0.0]),
            JunctionSpec::new("a", [-0.5, 1.0, 0.0]),
            JunctionSpec::new("b", [0.5, 1.0, 0.0]),
        ],
        ["mount", "a", "b"]
            .into_iter()
            .map(|end| SpanSpec::new(end, "J", end, material.clone(), SamplingSettings::new(0.05)))
            .collect(),
        CollisionSettings::new(0.005),
    );
    let mut harnesses = HarnessSet::new(id)?;
    let harness = harnesses.insert(id, &mut world, &harness_spec)?;
    assert_eq!(world.soft_bodies.len(), 1);
    for step in 0..16 {
        let commands = if step == 0 {
            vec![HarnessCommand::Pin {
                harness,
                location: HarnessLocation::Junction("mount".into()),
                position_m: [0.0, 2.0, 0.0],
            }]
        } else {
            vec![]
        };
        harnesses.prepare(id, &mut world, step, 1.0 / 240.0, &commands)?;
        world.step();
        harnesses.inspect(id, &world, step)?;
    }
    let snapshot = harnesses
        .get(id, &world, harness)?
        .snapshot(CaptureStamp::new(
            CapturePhase::AfterStep,
            Some(15),
            16.0 * dt,
            dt,
        )?)?;
    assert_eq!(snapshot.span_geometry.len(), 3);
    assert!(matches!(
        snapshot.junction_curvature,
        Observation::Unsupported { .. }
    ));
    let decoded: HarnessSnapshot = serde_json::from_slice(&serde_json::to_vec(&snapshot)?)?;
    assert_eq!(decoded.positions_m, snapshot.positions_m);
    harnesses.remove(id, &mut world, harness)?;
    assert!(harnesses.get(id, &world, harness).is_err());
    assert!(world.soft_bodies.is_empty());
    assert_eq!(world.bodies.len(), 1);
    println!(
        "PASS: {}-bit Rapier world; insert/pin/attach/96 steps/read/detach/unpin/JSON/remove",
        expected_real_bytes * 8
    );
    Ok(())
}
