#[path = "support/registry.rs"]
mod support;
use rapier_rope::{rapier::prelude::*, *};
use support::*;

#[test]
fn prepare_step_inspect_keeps_stepping_owned_by_caller() {
    let (mut ropes, mut world, rope) = setup();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    let initial = world.soft_bodies[native].particle_position(8);
    let prepared = ropes
        .prepare(
            ID,
            &mut world,
            0,
            1.0 / 240.0,
            &[RopeCommand::Pin { rope, particle: 0 }],
        )
        .unwrap();
    assert_eq!(prepared.applied_commands, 1);
    assert_eq!(world.soft_bodies[native].particle_position(8), initial);
    for n in 0..30 {
        if n != 0 {
            ropes.prepare(ID, &mut world, n, 1.0 / 240.0, &[]).unwrap();
        }
        world.step();
        let report = ropes.inspect(ID, &world, n).unwrap();
        assert_eq!(report.ropes[0].handle, rope);
        assert_eq!(report.ropes[0].particle_count, 9);
    }
    assert!(world.soft_bodies[native].particle_position(8).y < initial.y);
    assert_eq!(ropes.next_step(), 30);
}

#[test]
fn invalid_late_command_applies_nothing_and_step_is_reusable() {
    let (mut ropes, mut world, rope) = setup();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    world.soft_bodies[native].set_particle_velocity(0, Vector::X);
    let error = ropes
        .prepare(
            ID,
            &mut world,
            0,
            1.0 / 240.0,
            &[
                RopeCommand::Pin { rope, particle: 0 },
                RopeCommand::Pin { rope, particle: 99 },
            ],
        )
        .unwrap_err();
    assert_eq!(error.applied_commands, 0);
    assert!(matches!(
        error.kind,
        RopeSetErrorKind::InvalidParticle { .. }
    ));
    assert!(!world.soft_bodies[native].particles()[0].is_pinned());
    assert_eq!(world.soft_bodies[native].particle_velocity(0), Vector::X);
    step(
        &mut ropes,
        &mut world,
        &[RopeCommand::Pin { rope, particle: 0 }],
    );
}

#[test]
fn multiple_attachments_preserve_native_order_when_an_old_slot_is_reused() {
    let (mut ropes, mut world, rope) = setup();
    let target = world.insert_body(RigidBodyBuilder::fixed());
    let prepared = step(
        &mut ropes,
        &mut world,
        &[
            RopeCommand::Attach {
                rope,
                particle: 0,
                body: target,
            },
            RopeCommand::Attach {
                rope,
                particle: 8,
                body: target,
            },
        ],
    );
    let first = prepared.created_attachments[0].handle;
    let retained = prepared.created_attachments[1].handle;
    let next = step(
        &mut ropes,
        &mut world,
        &[
            RopeCommand::Detach { attachment: first },
            RopeCommand::Attach {
                rope,
                particle: 1,
                body: target,
            },
        ],
    )
    .created_attachments[0]
        .handle;
    assert_eq!(next.slot(), first.slot());
    assert_eq!(
        ropes.get_attachment(ID, &world, retained).unwrap().particle,
        8
    );
    assert_eq!(ropes.get_attachment(ID, &world, next).unwrap().particle, 1);
    assert_eq!(
        ropes
            .get(ID, &world, rope)
            .unwrap()
            .soft_body
            .particle_attachments()
            .iter()
            .map(|a| a.particle)
            .collect::<Vec<_>>(),
        [8, 1]
    );
    step(&mut ropes, &mut world, &[]);
}

#[test]
fn invalid_attachment_pose_is_rejected_before_any_earlier_pin_is_applied() {
    let (mut ropes, mut world, rope) = setup();
    let target = world.insert_body(RigidBodyBuilder::fixed().translation(Vector::splat(Real::NAN)));
    let error = ropes
        .prepare(
            ID,
            &mut world,
            0,
            1.0 / 240.0,
            &[
                RopeCommand::Pin { rope, particle: 1 },
                RopeCommand::Attach {
                    rope,
                    particle: 0,
                    body: target,
                },
            ],
        )
        .unwrap_err();
    assert!(matches!(error.kind, RopeSetErrorKind::NonFiniteState(_)));
    assert_eq!(error.applied_commands, 0);
    assert!(!ropes.get(ID, &world, rope).unwrap().soft_body.particles()[1].is_pinned());
}

#[test]
fn timestep_and_step_errors_preserve_native_state_and_pending_pair() {
    let (mut ropes, mut world, rope) = setup();
    for h in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e-320] {
        let error = ropes.prepare(ID, &mut world, 0, h, &[]).unwrap_err();
        assert!(matches!(
            error.kind,
            RopeSetErrorKind::InvalidTimestep | RopeSetErrorKind::TimestepMismatch { .. }
        ));
    }
    assert!(matches!(
        ropes
            .prepare(ID, &mut world, 1, 1.0 / 240.0, &[])
            .unwrap_err()
            .kind,
        RopeSetErrorKind::WrongStep { .. }
    ));
    assert!(matches!(
        ropes
            .prepare(ID, &mut world, 0, 1.0 / 120.0, &[])
            .unwrap_err()
            .kind,
        RopeSetErrorKind::TimestepMismatch { .. }
    ));
    assert_eq!(
        ropes.inspect(ID, &world, 0).unwrap_err().kind,
        RopeSetErrorKind::StepNotPrepared
    );
    ropes.prepare(ID, &mut world, 0, 1.0 / 240.0, &[]).unwrap();
    assert!(matches!(
        ropes
            .prepare(ID, &mut world, 0, 1.0 / 240.0, &[])
            .unwrap_err()
            .kind,
        RopeSetErrorKind::StepAlreadyPrepared { .. }
    ));
    assert!(matches!(
        ropes.remove(ID, &mut world, rope).unwrap_err().kind,
        RopeSetErrorKind::RegistryBusy { .. }
    ));
    assert!(matches!(
        ropes
            .insert(ID, &mut world, &spec("busy"))
            .unwrap_err()
            .kind,
        RopeSetErrorKind::RegistryBusy { .. }
    ));
    assert!(matches!(
        ropes.inspect(WorldId(1), &world, 0).unwrap_err().kind,
        RopeSetErrorKind::WorldMismatch { .. }
    ));
    assert!(matches!(
        ropes.inspect(ID, &world, 1).unwrap_err().kind,
        RopeSetErrorKind::WrongStep { .. }
    ));
    world.step();
    ropes.inspect(ID, &world, 0).unwrap();
    assert_eq!(
        ropes.inspect(ID, &world, 0).unwrap_err().kind,
        RopeSetErrorKind::StepNotPrepared
    );
}

#[test]
fn duplicate_acquisition_and_wrong_release_order_are_atomic() {
    let (mut ropes, mut world, rope) = setup();
    let target = world.insert_body(RigidBodyBuilder::fixed());
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    for commands in [
        vec![
            RopeCommand::Pin { rope, particle: 0 },
            RopeCommand::Attach {
                rope,
                particle: 0,
                body: target,
            },
        ],
        vec![
            RopeCommand::Attach {
                rope,
                particle: 0,
                body: target,
            },
            RopeCommand::Pin { rope, particle: 0 },
        ],
        vec![
            RopeCommand::Pin { rope, particle: 0 },
            RopeCommand::Unpin { rope, particle: 0 },
        ],
    ] {
        let error = ropes
            .prepare(ID, &mut world, 0, 1.0 / 240.0, &commands)
            .unwrap_err();
        assert_eq!(error.applied_commands, 0);
        assert!(matches!(
            error.kind,
            RopeSetErrorKind::ConflictingCommands { .. }
        ));
        assert!(!world.soft_bodies[native].particles()[0].is_pinned());
        assert!(world.soft_bodies[native].particle_attachments().is_empty());
        assert_eq!(ropes.attachment_count(), 0);
    }
}

#[test]
fn release_then_reacquire_reuses_attachment_slot_with_new_generation() {
    let (mut ropes, mut world, rope) = setup();
    let (target, old) = attached(&mut ropes, &mut world, rope);
    let next = step(
        &mut ropes,
        &mut world,
        &[
            RopeCommand::Detach { attachment: old },
            RopeCommand::Attach {
                rope,
                particle: 0,
                body: target,
            },
        ],
    )
    .created_attachments[0]
        .handle;
    assert_eq!(next.slot(), old.slot());
    assert_ne!(next.generation(), old.generation());
    assert_eq!(
        ropes.get_attachment(ID, &world, old).unwrap_err().kind,
        RopeSetErrorKind::StaleAttachmentHandle
    );
    assert_eq!(ropes.get_attachment(ID, &world, next).unwrap().body, target);
    let prepared = step(
        &mut ropes,
        &mut world,
        &[
            RopeCommand::Detach { attachment: next },
            RopeCommand::Pin { rope, particle: 0 },
        ],
    );
    assert_eq!(prepared.applied_commands, 2);
    step(
        &mut ropes,
        &mut world,
        &[
            RopeCommand::Unpin { rope, particle: 0 },
            RopeCommand::Attach {
                rope,
                particle: 0,
                body: target,
            },
        ],
    );
    assert_eq!(ropes.attachment_count(), 1);
}

#[test]
fn missing_body_foreign_attachment_and_stale_ids_are_rejected_before_apply() {
    let (mut ropes, mut world, rope) = setup();
    assert!(matches!(
        ropes
            .prepare(
                ID,
                &mut world,
                0,
                1.0 / 240.0,
                &[
                    RopeCommand::Pin { rope, particle: 1 },
                    RopeCommand::Attach {
                        rope,
                        particle: 0,
                        body: RigidBodyHandle::invalid()
                    }
                ]
            )
            .unwrap_err()
            .kind,
        RopeSetErrorKind::MissingRigidBody(_)
    ));
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    assert!(!world.soft_bodies[native].particles()[1].is_pinned());
    let root = world.soft_bodies[native].root_body();
    assert_eq!(
        ropes
            .prepare(
                ID,
                &mut world,
                0,
                1.0 / 240.0,
                &[RopeCommand::Attach {
                    rope,
                    particle: 0,
                    body: root
                }]
            )
            .unwrap_err()
            .kind,
        RopeSetErrorKind::UnsupportedAttachmentBody(root)
    );
    let (_, attachment) = attached(&mut ropes, &mut world, rope);
    let mut other = RopeSet::new(ID).unwrap();
    assert!(matches!(
        other
            .prepare(
                ID,
                &mut world,
                0,
                1.0 / 240.0,
                &[RopeCommand::Detach { attachment }]
            )
            .unwrap_err()
            .kind,
        RopeSetErrorKind::ForeignSet { .. }
    ));
    step(
        &mut ropes,
        &mut world,
        &[RopeCommand::Detach { attachment }],
    );
    let n = ropes.next_step();
    assert_eq!(
        ropes
            .prepare(
                ID,
                &mut world,
                n,
                1.0 / 240.0,
                &[RopeCommand::Detach { attachment }]
            )
            .unwrap_err()
            .kind,
        RopeSetErrorKind::StaleAttachmentHandle
    );
    ropes.remove(ID, &mut world, rope).unwrap();
    assert_eq!(
        ropes
            .prepare(
                ID,
                &mut world,
                n,
                1.0 / 240.0,
                &[RopeCommand::Pin { rope, particle: 0 }]
            )
            .unwrap_err()
            .kind,
        RopeSetErrorKind::StaleRopeHandle
    );
}

#[test]
fn validation_of_all_registered_ropes_prevents_partial_batch_mutation() {
    let (mut ropes, mut world, first) = setup();
    let second = ropes.insert(ID, &mut world, &spec("second")).unwrap();
    let invalid = ropes.get(ID, &world, second).unwrap().native_handle;
    world.remove_soft_body(invalid).unwrap();
    let error = ropes
        .prepare(
            ID,
            &mut world,
            0,
            1.0 / 240.0,
            &[RopeCommand::Pin {
                rope: first,
                particle: 0,
            }],
        )
        .unwrap_err();
    assert_eq!(error.rope, Some(second));
    assert_eq!(error.phase, RegistryPhase::Prepare);
    assert_eq!(error.applied_commands, 0);
    assert!(!ropes.get(ID, &world, first).unwrap().soft_body.particles()[0].is_pinned());
    ropes.remove(ID, &mut world, second).unwrap();
    step(
        &mut ropes,
        &mut world,
        &[RopeCommand::Pin {
            rope: first,
            particle: 0,
        }],
    );
}

#[test]
fn inspect_failure_marks_post_step_phase_consumes_pair_and_allows_cleanup() {
    let (mut ropes, mut world, rope) = setup();
    ropes
        .prepare(
            ID,
            &mut world,
            0,
            1.0 / 240.0,
            &[RopeCommand::Pin { rope, particle: 0 }],
        )
        .unwrap();
    world.step();
    let native = world.soft_bodies.iter().next().unwrap().0;
    world.remove_soft_body(native).unwrap();
    let error = ropes.inspect(ID, &world, 0).unwrap_err();
    assert_eq!(error.phase, RegistryPhase::Inspect);
    assert_eq!(error.applied_commands, 1);
    assert_eq!(error.kind, RopeSetErrorKind::MissingSoftBody(native));
    assert_eq!(ropes.next_step(), 1);
    assert!(!ropes.remove(ID, &mut world, rope).unwrap().native_removed);
    step(&mut ropes, &mut world, &[]);
}

#[test]
fn changed_dt_and_nonfinite_particle_state_are_reported_after_step() {
    let (mut ropes, mut world, rope) = setup();
    ropes.prepare(ID, &mut world, 0, 1.0 / 240.0, &[]).unwrap();
    world.step();
    world.integration_parameters.dt = 1.0 / 120.0;
    let error = ropes.inspect(ID, &world, 0).unwrap_err();
    assert_eq!(error.phase, RegistryPhase::Inspect);
    assert!(matches!(
        error.kind,
        RopeSetErrorKind::TimestepMismatch { .. }
    ));
    world.integration_parameters.dt = 1.0 / 240.0;
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    ropes.prepare(ID, &mut world, 1, 1.0 / 240.0, &[]).unwrap();
    world.step();
    world.soft_bodies[native].set_particle_position(1, Vector::splat(Real::NAN));
    let error = ropes.inspect(ID, &world, 1).unwrap_err();
    assert_eq!(error.phase, RegistryPhase::Inspect);
    assert!(matches!(error.kind, RopeSetErrorKind::NonFiniteState(_)));
    ropes.remove(ID, &mut world, rope).unwrap();
}

#[test]
fn protocol_does_not_claim_to_detect_actual_step_call_count() {
    let (mut ropes, world, _) = setup();
    let mut world = world;
    ropes.prepare(ID, &mut world, 0, 1.0 / 240.0, &[]).unwrap();
    // Without a native counter the protocol accepts zero calls, as explicitly documented.
    assert!(ropes.inspect(ID, &world, 0).is_ok());
    ropes.prepare(ID, &mut world, 1, 1.0 / 240.0, &[]).unwrap();
    world.step();
    world.step();
    assert!(ropes.inspect(ID, &world, 1).is_ok());
}
