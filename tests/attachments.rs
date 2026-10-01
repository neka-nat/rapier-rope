#[path = "support/registry.rs"]
#[allow(dead_code)]
mod support;
use rapier_rope::{rapier::prelude::*, *};
use support::{ID, setup};

const H: f64 = 1.0 / 240.0;
#[allow(clippy::unnecessary_cast)]
fn real(value: f64) -> Real {
    value as Real
}
#[allow(clippy::unnecessary_cast)]
fn scalar(value: Real) -> f64 {
    value as f64
}
fn prepare(
    ropes: &mut RopeSet,
    world: &mut PhysicsWorld,
    commands: &[AttachmentCommand],
) -> AttachmentPreparation {
    ropes
        .prepare_attachments(ID, world, ropes.next_step(), H, commands)
        .unwrap()
}
fn finish(ropes: &mut RopeSet, world: &mut PhysicsWorld) {
    world.step();
    ropes.inspect(ID, world, ropes.next_step()).unwrap();
}
fn gripper(world: &mut PhysicsWorld) -> RigidBodyHandle {
    world.insert_body(
        RigidBodyBuilder::kinematic_position_based()
            .pose(Pose::new(Vector::new(-0.1, 1.5, 0.0), Vector::Z * 0.4)),
    )
}

#[test]
fn interior_selection_reports_actual_reference_location_and_anchor_without_teleport() {
    let (mut ropes, mut world, rope) = setup();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    let position = Vector::new(0.4, 1.2, 0.1);
    let velocity = Vector::new(1.0, 2.0, 3.0);
    world.soft_bodies[native].set_particle_position(4, position);
    world.soft_bodies[native].set_particle_velocity(4, velocity);
    let body = gripper(&mut world);
    let result = prepare(
        &mut ropes,
        &mut world,
        &[AttachmentCommand::Attach {
            rope,
            location: RopeLocation::ArcLength {
                arc_length_m: 0.51,
                tolerance_m: 0.011,
            },
            body,
        }],
    );
    let selection = &result.locations[0];
    assert_eq!(selection.command_index, 0);
    assert_eq!(selection.location.particle_index, 4);
    assert_eq!(selection.location.actual_arc_length_m, 0.5);
    assert!((selection.location.error_m - 0.01).abs() < 1e-14);
    let receipt = &result.prepared.created_attachments[0];
    let view = ropes.get_attachment(ID, &world, receipt.handle).unwrap();
    assert_eq!(view.local_anchor_m, receipt.local_anchor_m);
    assert_eq!(receipt.particle, 4);
    assert_eq!(receipt.rope, rope);
    let actual = world.soft_bodies[native].particle_attachments()[0].local_anchor;
    assert_eq!(
        actual,
        world.bodies[body]
            .position()
            .inverse_transform_point(position)
    );
    assert_eq!(world.soft_bodies[native].particle_position(4), position);
    assert_eq!(world.soft_bodies[native].particle_velocity(4), velocity);
}

#[test]
fn named_and_endpoint_selections_use_registration_reference_map() {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = real(H);
    let mut ropes = RopeSet::new(ID).unwrap();
    let mut spec = support::spec("named");
    spec.named_locations
        .push(NamedLocation::new("connector", 0.5));
    let rope = ropes.insert(ID, &mut world, &spec).unwrap();
    let body = gripper(&mut world);
    let result = prepare(
        &mut ropes,
        &mut world,
        &[
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
            AttachmentCommand::Pin {
                rope,
                location: RopeLocation::End,
                position_m: [1.0, 1.5, 0.0],
            },
        ],
    );
    assert_eq!(
        result
            .locations
            .iter()
            .map(|r| r.location.particle_index)
            .collect::<Vec<_>>(),
        vec![0, 4, 8]
    );
    assert_eq!(result.prepared.created_attachments[0].command_index, 1);
    finish(&mut ropes, &mut world);
}

#[test]
fn bad_material_locations_fail_before_any_commands_apply() {
    let (mut ropes, mut world, rope) = setup();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    for location in [
        RopeLocation::Named("missing".into()),
        RopeLocation::ArcLength {
            arc_length_m: 1.1,
            tolerance_m: 1.0,
        },
        RopeLocation::ArcLength {
            arc_length_m: 0.51,
            tolerance_m: 0.001,
        },
        RopeLocation::ArcLength {
            arc_length_m: 0.5,
            tolerance_m: f64::NAN,
        },
    ] {
        let e = ropes
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
                    AttachmentCommand::Unpin { rope, location },
                ],
            )
            .unwrap_err();
        assert_eq!(e.phase, RegistryPhase::Prepare);
        assert_eq!(e.applied_commands, 0);
        assert!(matches!(e.kind, RopeSetErrorKind::Location(_)));
        assert!(!world.soft_bodies[native].particles()[0].is_pinned());
        assert_eq!(ropes.next_step(), 0);
    }
}

#[test]
fn explicit_pin_zeros_velocity_and_sets_world_point() {
    let (mut ropes, mut world, rope) = setup();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    world.soft_bodies[native].set_particle_velocity(0, Vector::X);
    prepare(
        &mut ropes,
        &mut world,
        &[AttachmentCommand::Pin {
            rope,
            location: RopeLocation::Start,
            position_m: [0.0, 1.25, 0.0],
        }],
    );
    assert_eq!(
        world.soft_bodies[native].particle_position(0),
        Vector::new(0.0, 1.25, 0.0)
    );
    assert_eq!(world.soft_bodies[native].particle_velocity(0), Vector::ZERO);
    assert!(world.soft_bodies[native].particles()[0].is_pinned());
    assert_eq!(
        world.soft_bodies[native].particles()[0].mass(),
        real(0.00625)
    );
}

#[test]
fn moving_pin_queues_target_and_unpin_preserves_velocity_before_step() {
    let (mut ropes, mut world, rope) = setup();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    prepare(
        &mut ropes,
        &mut world,
        &[AttachmentCommand::Pin {
            rope,
            location: RopeLocation::Start,
            position_m: [0.0, 1.5, 0.0],
        }],
    );
    finish(&mut ropes, &mut world);
    let old_position = world.soft_bodies[native].particle_position(0);
    let old_velocity = world.soft_bodies[native].particle_velocity(0);
    let target = Vector::new(0.01, 1.5, 0.0);
    prepare(
        &mut ropes,
        &mut world,
        &[AttachmentCommand::MovePin {
            rope,
            location: RopeLocation::Start,
            target_m: [0.01, 1.5, 0.0],
        }],
    );
    assert_eq!(world.soft_bodies[native].particle_position(0), old_position);
    assert_eq!(world.soft_bodies[native].particle_velocity(0), old_velocity);
    assert_eq!(
        world.soft_bodies[native].particles()[0].kinematic_target(),
        Some(target)
    );
    finish(&mut ropes, &mut world);
    let current_velocity = world.soft_bodies[native].particle_velocity(0);
    assert!(current_velocity.x > 2.0);
    assert!((world.soft_bodies[native].particle_position(0) - target).length() < real(1e-6));
    prepare(
        &mut ropes,
        &mut world,
        &[AttachmentCommand::Unpin {
            rope,
            location: RopeLocation::Start,
        }],
    );
    assert_eq!(
        world.soft_bodies[native].particle_velocity(0),
        current_velocity
    );
    assert_eq!(
        world.soft_bodies[native].particles()[0].kinematic_target(),
        None
    );
    assert!(!world.soft_bodies[native].particles()[0].is_pinned());
    finish(&mut ropes, &mut world);
    assert_ne!(
        world.soft_bodies[native].particle_velocity(0),
        current_velocity
    );
}

#[test]
fn detach_preserves_current_velocity_and_invalidates_handle() {
    let (mut ropes, mut world, rope) = setup();
    let body = gripper(&mut world);
    let result = prepare(
        &mut ropes,
        &mut world,
        &[AttachmentCommand::Attach {
            rope,
            location: RopeLocation::End,
            body,
        }],
    );
    let attachment = result.prepared.created_attachments[0].handle;
    finish(&mut ropes, &mut world);
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    let velocity = Vector::new(0.2, -0.3, 0.4);
    world.soft_bodies[native].set_particle_velocity(8, velocity);
    let result = prepare(
        &mut ropes,
        &mut world,
        &[AttachmentCommand::Detach { attachment }],
    );
    assert!(result.locations.is_empty());
    assert_eq!(world.soft_bodies[native].particle_velocity(8), velocity);
    assert!(world.soft_bodies[native].particle_attachments().is_empty());
    assert!(matches!(
        ropes
            .get_attachment(ID, &world, attachment)
            .unwrap_err()
            .kind,
        RopeSetErrorKind::StaleAttachmentHandle
    ));
}

#[test]
fn target_must_be_pinned_and_entire_target_batch_is_validated() {
    let (mut ropes, mut world, rope) = setup();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    let e = ropes
        .prepare_attachments(
            ID,
            &mut world,
            0,
            H,
            &[AttachmentCommand::MovePin {
                rope,
                location: RopeLocation::Start,
                target_m: [0.1, 1.5, 0.0],
            }],
        )
        .unwrap_err();
    assert!(matches!(e.kind, RopeSetErrorKind::NotPinned { .. }));
    for value in [
        f64::NAN,
        f64::INFINITY,
        f64::MAX,
        scalar(Real::MAX).sqrt() * 2.0,
    ] {
        let e = ropes
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
                    AttachmentCommand::Pin {
                        rope,
                        location: RopeLocation::End,
                        position_m: [value, 1.5, 0.0],
                    },
                ],
            )
            .unwrap_err();
        assert!(matches!(e.kind, RopeSetErrorKind::InvalidTarget(_)));
        assert_eq!(e.applied_commands, 0);
        assert!(!world.soft_bodies[native].particles()[0].is_pinned());
    }
}

#[test]
fn duplicate_targets_and_target_before_release_are_rejected() {
    let (mut ropes, mut world, rope) = setup();
    prepare(
        &mut ropes,
        &mut world,
        &[AttachmentCommand::Pin {
            rope,
            location: RopeLocation::Start,
            position_m: [0.0, 1.5, 0.0],
        }],
    );
    finish(&mut ropes, &mut world);
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    let old_target = world.soft_bodies[native].particles()[0].kinematic_target();
    let moving = AttachmentCommand::MovePin {
        rope,
        location: RopeLocation::Start,
        target_m: [0.1, 1.5, 0.0],
    };
    for commands in [
        vec![moving.clone(), moving.clone()],
        vec![
            moving.clone(),
            AttachmentCommand::Unpin {
                rope,
                location: RopeLocation::Start,
            },
        ],
    ] {
        let e = ropes
            .prepare_attachments(ID, &mut world, 1, H, &commands)
            .unwrap_err();
        assert!(matches!(
            e.kind,
            RopeSetErrorKind::ConflictingCommands { .. }
        ));
        assert_eq!(
            world.soft_bodies[native].particles()[0].kinematic_target(),
            old_target
        );
    }
}

#[test]
fn conflicting_grasps_and_pin_attach_are_rejected_atomically() {
    let (mut ropes, mut world, rope) = setup();
    let first = gripper(&mut world);
    let second = gripper(&mut world);
    let attach = AttachmentCommand::Attach {
        rope,
        location: RopeLocation::Start,
        body: first,
    };
    let pin = AttachmentCommand::Pin {
        rope,
        location: RopeLocation::Start,
        position_m: [0.0, 1.5, 0.0],
    };
    for commands in [
        vec![
            attach.clone(),
            AttachmentCommand::Attach {
                rope,
                location: RopeLocation::Start,
                body: second,
            },
        ],
        vec![pin.clone(), attach.clone()],
        vec![attach, pin],
    ] {
        let e = ropes
            .prepare_attachments(ID, &mut world, 0, H, &commands)
            .unwrap_err();
        assert!(matches!(
            e.kind,
            RopeSetErrorKind::ConflictingCommands { .. }
        ));
        assert_eq!(e.applied_commands, 0);
        assert_eq!(ropes.attachment_count(), 0);
    }
}

#[test]
fn external_multiple_attachments_are_detected_before_release() {
    let (mut ropes, mut world, rope) = setup();
    let first = gripper(&mut world);
    let second = gripper(&mut world);
    let result = prepare(
        &mut ropes,
        &mut world,
        &[AttachmentCommand::Attach {
            rope,
            location: RopeLocation::Start,
            body: first,
        }],
    );
    let attachment = result.prepared.created_attachments[0].handle;
    finish(&mut ropes, &mut world);
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    world.soft_bodies[native].attach_particle(0, second, &world.bodies);
    let e = ropes
        .prepare_attachments(
            ID,
            &mut world,
            1,
            H,
            &[AttachmentCommand::Detach { attachment }],
        )
        .unwrap_err();
    assert!(matches!(e.kind, RopeSetErrorKind::ConstraintChanged(_)));
    assert_eq!(world.soft_bodies[native].particle_attachments().len(), 2);
}

#[test]
fn missing_body_and_stale_material_rope_fail_without_pin_mutation() {
    let (mut ropes, mut world, rope) = setup();
    let body = gripper(&mut world);
    world.remove_body(body);
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    let e = ropes
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
        .unwrap_err();
    assert!(matches!(e.kind, RopeSetErrorKind::MissingRigidBody(_)));
    assert!(!world.soft_bodies[native].particles()[0].is_pinned());
    ropes.remove(ID, &mut world, rope).unwrap();
    let e = ropes
        .prepare_attachments(
            ID,
            &mut world,
            0,
            H,
            &[AttachmentCommand::Unpin {
                rope,
                location: RopeLocation::End,
            }],
        )
        .unwrap_err();
    assert_eq!(e.phase, RegistryPhase::Prepare);
    assert!(matches!(e.kind, RopeSetErrorKind::StaleRopeHandle));
}

#[test]
fn release_then_regrasp_uses_current_anchor_and_new_generation() {
    let (mut ropes, mut world, rope) = setup();
    let first = gripper(&mut world);
    let second = gripper(&mut world);
    let result = prepare(
        &mut ropes,
        &mut world,
        &[AttachmentCommand::Attach {
            rope,
            location: RopeLocation::Start,
            body: first,
        }],
    );
    let old = result.prepared.created_attachments[0].handle;
    finish(&mut ropes, &mut world);
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    let position = world.soft_bodies[native].particle_position(0);
    let velocity = world.soft_bodies[native].particle_velocity(0);
    let result = prepare(
        &mut ropes,
        &mut world,
        &[
            AttachmentCommand::Detach { attachment: old },
            AttachmentCommand::Attach {
                rope,
                location: RopeLocation::Start,
                body: second,
            },
        ],
    );
    let new = result.prepared.created_attachments[0].handle;
    assert_ne!(old, new);
    assert_eq!(result.prepared.created_attachments[0].command_index, 1);
    assert_eq!(world.soft_bodies[native].particle_position(0), position);
    assert_eq!(world.soft_bodies[native].particle_velocity(0), velocity);
}

#[test]
fn combined_target_distances_are_checked_before_pin_mutation() {
    let (mut ropes, mut world, rope) = setup();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    let large = scalar(Real::MAX).sqrt() * 0.9;
    let e = ropes
        .prepare_attachments(
            ID,
            &mut world,
            0,
            H,
            &[
                AttachmentCommand::Pin {
                    rope,
                    location: RopeLocation::Start,
                    position_m: [large, 0.0, 0.0],
                },
                AttachmentCommand::Pin {
                    rope,
                    location: RopeLocation::End,
                    position_m: [-large, 0.0, 0.0],
                },
            ],
        )
        .unwrap_err();
    assert!(matches!(
        e.kind,
        RopeSetErrorKind::InvalidTarget("target distance overflow")
    ));
    assert_eq!(e.applied_commands, 0);
    assert!(!world.soft_bodies[native].particles()[0].is_pinned());
}

#[test]
fn finite_target_with_overflowing_motion_velocity_is_rejected() {
    let (mut ropes, mut world, rope) = setup();
    prepare(
        &mut ropes,
        &mut world,
        &[AttachmentCommand::Pin {
            rope,
            location: RopeLocation::Start,
            position_m: [0.0, 1.5, 0.0],
        }],
    );
    finish(&mut ropes, &mut world);
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    let old = world.soft_bodies[native].particles()[0].kinematic_target();
    let e = ropes
        .prepare_attachments(
            ID,
            &mut world,
            1,
            H,
            &[AttachmentCommand::MovePin {
                rope,
                location: RopeLocation::Start,
                target_m: [scalar(Real::MAX).sqrt() * 0.01, 0.0, 0.0],
            }],
        )
        .unwrap_err();
    assert!(matches!(
        e.kind,
        RopeSetErrorKind::InvalidTarget("target velocity overflow")
    ));
    assert_eq!(
        world.soft_bodies[native].particles()[0].kinematic_target(),
        old
    );
}

#[test]
fn nonfinite_external_pin_target_and_kinematic_gripper_target_are_rejected() {
    let (mut ropes, mut world, rope) = setup();
    let body = gripper(&mut world);
    world.bodies[body]
        .set_next_kinematic_position(Pose::new(Vector::new(Real::NAN, 0.0, 0.0), Vector::ZERO));
    let e = ropes
        .prepare_attachments(
            ID,
            &mut world,
            0,
            H,
            &[AttachmentCommand::Attach {
                rope,
                location: RopeLocation::Start,
                body,
            }],
        )
        .unwrap_err();
    assert!(matches!(e.kind, RopeSetErrorKind::NonFiniteState(_)));
    assert_eq!(ropes.attachment_count(), 0);
    let current_pose = *world.bodies[body].position();
    world.bodies[body].set_next_kinematic_position(current_pose);
    prepare(
        &mut ropes,
        &mut world,
        &[AttachmentCommand::Pin {
            rope,
            location: RopeLocation::Start,
            position_m: [0.0, 1.5, 0.0],
        }],
    );
    finish(&mut ropes, &mut world);
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    world.soft_bodies[native].set_particle_kinematic_target(0, Vector::splat(Real::NAN));
    let e = ropes
        .prepare_attachments(ID, &mut world, 1, H, &[])
        .unwrap_err();
    assert!(matches!(e.kind, RopeSetErrorKind::NonFiniteState(_)));
}
