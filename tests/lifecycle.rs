#[path = "support/registry.rs"]
mod support;
use rapier_rope::{rapier::prelude::*, *};
use support::*;

#[test]
fn lookup_preserves_definition_maps_and_native_handle() {
    let (ropes, world, handle) = setup();
    let view = ropes.get(ID, &world, handle).unwrap();
    assert_eq!(view.specification, &spec("rope"));
    assert_eq!(view.samples.arc_lengths_m().len(), 9);
    assert_eq!(view.soft_body.num_particles(), 9);
    assert_eq!(
        ropes
            .get_by_name(ID, &world, "rope")
            .unwrap()
            .unwrap()
            .handle,
        handle
    );
    assert!(ropes.get_by_name(ID, &world, "absent").unwrap().is_none());
}

#[test]
fn removal_invalidates_generations_even_after_slot_reuse() {
    let (mut ropes, mut world, old) = setup();
    let native = ropes.get(ID, &world, old).unwrap().native_handle;
    assert!(ropes.remove(ID, &mut world, old).unwrap().native_removed);
    assert!(!world.soft_bodies.contains(native));
    assert_eq!(
        ropes.get(ID, &world, old).err().unwrap().kind,
        RopeSetErrorKind::StaleRopeHandle
    );
    let new = ropes.insert(ID, &mut world, &spec("rope")).unwrap();
    assert_eq!(new.slot(), old.slot());
    assert!(new.generation() > old.generation());
    assert_eq!(
        ropes.remove(ID, &mut world, old).unwrap_err().kind,
        RopeSetErrorKind::StaleRopeHandle
    );
    assert!(ropes.get(ID, &world, new).is_ok());
}

#[test]
fn world_and_registry_mismatches_do_not_mutate_either_world() {
    let (mut ropes, mut world, rope) = setup();
    let mut other = RopeSet::new(ID).unwrap();
    let error = other.remove(ID, &mut world, rope).unwrap_err();
    assert!(matches!(error.kind, RopeSetErrorKind::ForeignSet { .. }));
    assert_eq!(world.soft_bodies.len(), 1);
    let mut other_world = PhysicsWorld::new();
    let wrong = WorldId(18);
    assert!(matches!(
        ropes
            .insert(wrong, &mut other_world, &spec("second"))
            .unwrap_err()
            .kind,
        RopeSetErrorKind::WorldMismatch { .. }
    ));
    assert!(matches!(
        ropes.get(wrong, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::WorldMismatch { .. }
    ));
    assert!(matches!(
        ropes.remove(wrong, &mut world, rope).unwrap_err().kind,
        RopeSetErrorKind::WorldMismatch { .. }
    ));
    assert!(other_world.soft_bodies.is_empty());
    assert_eq!(world.soft_bodies.len(), 1);
}

#[test]
fn invalid_definition_and_duplicate_names_are_atomic() {
    let (mut ropes, mut world, _) = setup();
    let before = (
        world.soft_bodies.len(),
        world.bodies.len(),
        world.colliders.len(),
    );
    assert!(matches!(
        ropes
            .insert(ID, &mut world, &spec("rope"))
            .unwrap_err()
            .kind,
        RopeSetErrorKind::DuplicateRopeName(_)
    ));
    let mut invalid = spec("bad");
    invalid.collision.radius_m = 0.0;
    assert!(matches!(
        ropes.insert(ID, &mut world, &invalid).unwrap_err().kind,
        RopeSetErrorKind::Definition(_)
    ));
    assert_eq!(
        (
            world.soft_bodies.len(),
            world.bodies.len(),
            world.colliders.len()
        ),
        before
    );
    assert_eq!(ropes.len(), 1);
}

#[test]
fn repeated_insert_attach_remove_reclaims_native_assets_and_preserves_external_bodies() {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = 1.0 / 240.0;
    let external = world.insert_body(RigidBodyBuilder::fixed());
    let (_, collider) = world.insert(RigidBodyBuilder::fixed(), ColliderBuilder::ball(0.01));
    let baseline = (world.bodies.len(), world.colliders.len());
    let mut ropes = RopeSet::new(ID).unwrap();
    for _ in 0..40 {
        let rope = ropes.insert(ID, &mut world, &spec("repeat")).unwrap();
        let native = ropes.get(ID, &world, rope).unwrap().native_handle;
        let prepared = step(
            &mut ropes,
            &mut world,
            &[RopeCommand::Attach {
                rope,
                particle: 0,
                body: external,
            }],
        );
        let attachment = prepared.created_attachments[0].handle;
        ropes.remove(ID, &mut world, rope).unwrap();
        assert!(!world.soft_bodies.contains(native));
        assert!(world.bodies.contains(external) && world.colliders.contains(collider));
        assert_eq!((world.bodies.len(), world.colliders.len()), baseline);
        assert!(world.soft_bodies.is_empty() && ropes.is_empty());
        assert_eq!(ropes.attachment_count(), 0);
        assert_eq!(
            ropes
                .get_attachment(ID, &world, attachment)
                .unwrap_err()
                .kind,
            RopeSetErrorKind::StaleAttachmentHandle
        );
    }
}

#[test]
fn external_soft_body_deletion_is_reported_then_metadata_can_be_removed() {
    let (mut ropes, mut world, rope) = setup();
    let (_, attachment) = attached(&mut ropes, &mut world, rope);
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    world.remove_soft_body(native).unwrap();
    assert_eq!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::MissingSoftBody(native)
    );
    assert!(matches!(
        ropes
            .get_attachment(ID, &world, attachment)
            .unwrap_err()
            .kind,
        RopeSetErrorKind::MissingSoftBody(_)
    ));
    let removed = ropes.remove(ID, &mut world, rope).unwrap();
    assert!(!removed.native_removed);
    assert!(ropes.is_empty());
    assert_eq!(ropes.attachment_count(), 0);
}

#[test]
fn external_proxy_and_collider_deletion_never_reads_old_maps() {
    let (mut ropes, mut world, rope) = setup();
    let view = ropes.get(ID, &world, rope).unwrap();
    let native = view.native_handle;
    let root = view.soft_body.root_body();
    world.remove_body(root).unwrap();
    assert_eq!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::MissingSoftBody(native)
    );
    assert!(!ropes.remove(ID, &mut world, rope).unwrap().native_removed);
    let rope = ropes.insert(ID, &mut world, &spec("other")).unwrap();
    let collider = ropes.get(ID, &world, rope).unwrap().soft_body.clusters()[0]
        .meshes()
        .next()
        .unwrap()
        .collider();
    world.remove_collider(collider).unwrap();
    assert!(matches!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::MissingCollider(_) | RopeSetErrorKind::TopologyChanged(_)
    ));
    ropes.remove(ID, &mut world, rope).unwrap();
    assert!(world.soft_bodies.is_empty() && world.bodies.is_empty() && world.colliders.is_empty());
}

#[test]
fn missing_attachment_body_and_external_constraint_edits_are_rejected() {
    let (mut ropes, mut world, rope) = setup();
    let (target, _) = attached(&mut ropes, &mut world, rope);
    world.remove_body(target).unwrap();
    assert_eq!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::MissingRigidBody(target)
    );
    ropes.remove(ID, &mut world, rope).unwrap();
    let rope = ropes.insert(ID, &mut world, &spec("pin-edit")).unwrap();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    world.soft_bodies[native].set_particle_pinned(1, true);
    assert!(matches!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::ConstraintChanged(_)
    ));
    ropes.remove(ID, &mut world, rope).unwrap();
    let rope = ropes
        .insert(ID, &mut world, &spec("attachment-edit"))
        .unwrap();
    let (target, _) = attached(&mut ropes, &mut world, rope);
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    world.soft_bodies[native].attach_particle(0, target, &world.bodies);
    assert!(matches!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::ConstraintChanged(_)
    ));
    ropes.remove(ID, &mut world, rope).unwrap();
    assert!(world.bodies.contains(target));
}

#[test]
fn external_tearing_and_replaced_native_topology_are_detected() {
    let (mut ropes, mut world, rope) = setup();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    world.soft_bodies[native].tear_edge(0);
    assert!(matches!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::TopologyChanged(_)
    ));
    // Clear the pending rope through normal removal; tear execution is not managed by .
    ropes.remove(ID, &mut world, rope).unwrap();
    let rope = ropes.insert(ID, &mut world, &spec("replacement")).unwrap();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    let saved = world.soft_bodies[native].clone();
    world.soft_bodies[native] = SoftBody::from(SoftBodyBuilder::rope(Vector::ZERO, Vector::X, 3));
    assert!(matches!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::TopologyChanged(_)
    ));
    assert!(matches!(
        ropes.remove(ID, &mut world, rope).unwrap_err().kind,
        RopeSetErrorKind::TopologyChanged(_)
    ));
    world.soft_bodies[native] = saved;
    ropes.remove(ID, &mut world, rope).unwrap();
}

#[test]
fn live_cluster_change_is_detected_and_foreign_proxy_removal_is_refused() {
    let (mut ropes, mut world, rope) = setup();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    let cluster = world.add_soft_body_cluster(native, &[0, 1]).unwrap();
    assert!(matches!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::TopologyChanged(_)
    ));
    assert!(matches!(
        ropes.remove(ID, &mut world, rope).unwrap_err().kind,
        RopeSetErrorKind::TopologyChanged(_)
    ));
    world.remove_soft_body_cluster(native, cluster).unwrap();
    // Dead cluster slots also change the captured inventory; caller restores via native removal.
    world.remove_soft_body(native).unwrap();
    ropes.remove(ID, &mut world, rope).unwrap();
}

#[test]
fn collider_ownership_changes_are_detected_and_removal_preserves_foreign_assets() {
    let (mut ropes, mut world, rope) = setup();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    let root = world.soft_bodies[native].root_body();
    let added = world.insert_collider(ColliderBuilder::ball(0.02), Some(root));
    assert!(matches!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::TopologyChanged(_)
    ));
    assert!(matches!(
        ropes.remove(ID, &mut world, rope).unwrap_err().kind,
        RopeSetErrorKind::TopologyChanged(_)
    ));
    assert!(world.colliders.contains(added) && world.soft_bodies.contains(native));
    world.remove_collider(added).unwrap();
    let collider = world.soft_bodies[native].clusters()[0]
        .meshes()
        .next()
        .unwrap()
        .collider();
    let other = world.insert_body(RigidBodyBuilder::fixed());
    world
        .colliders
        .set_parent(collider, Some(other), &mut world.bodies);
    assert!(matches!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::TopologyChanged(_)
    ));
    assert!(matches!(
        ropes.remove(ID, &mut world, rope).unwrap_err().kind,
        RopeSetErrorKind::TopologyChanged(_)
    ));
    assert!(world.bodies.contains(other) && world.colliders.contains(collider));
    world
        .colliders
        .set_parent(collider, Some(root), &mut world.bodies);
    ropes.remove(ID, &mut world, rope).unwrap();
    assert!(world.bodies.contains(other));
}

#[test]
fn executed_native_tear_invalidates_reference_maps_and_caller_can_clean_up_pieces() {
    let (mut ropes, mut world, rope) = setup();
    let native = ropes.get(ID, &world, rope).unwrap().native_handle;
    assert!(world.tear_soft_body(native, &[3], &[]).is_some());
    let error = ropes.get(ID, &world, rope).err().unwrap();
    assert!(matches!(
        error.kind,
        RopeSetErrorKind::TopologyChanged(_)
            | RopeSetErrorKind::MissingProxy(_)
            | RopeSetErrorKind::MissingCollider(_)
            | RopeSetErrorKind::MissingSoftBody(_)
    ));
    let native_pieces: Vec<_> = world.soft_bodies.iter().map(|(h, _)| h).collect();
    for piece in native_pieces {
        world.remove_soft_body(piece).unwrap();
    }
    assert!(!ropes.remove(ID, &mut world, rope).unwrap().native_removed);
    assert!(
        ropes.is_empty()
            && world.soft_bodies.is_empty()
            && world.bodies.is_empty()
            && world.colliders.is_empty()
    );
}

#[test]
fn replacing_native_sets_cannot_silently_abandon_registered_assets() {
    let (mut ropes, mut world, rope) = setup();
    let saved_soft_bodies = std::mem::replace(&mut world.soft_bodies, SoftBodySet::new());
    assert!(matches!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::MissingSoftBody(_)
    ));
    assert!(matches!(
        ropes.remove(ID, &mut world, rope).unwrap_err().kind,
        RopeSetErrorKind::TopologyChanged(_)
    ));
    assert_eq!(ropes.len(), 1);
    assert!(!world.bodies.is_empty() && !world.colliders.is_empty());
    world.soft_bodies = saved_soft_bodies;
    let saved_bodies = std::mem::replace(&mut world.bodies, RigidBodySet::new());
    assert!(matches!(
        ropes.get(ID, &world, rope).err().unwrap().kind,
        RopeSetErrorKind::MissingProxy(_)
    ));
    assert!(matches!(
        ropes.remove(ID, &mut world, rope).unwrap_err().kind,
        RopeSetErrorKind::TopologyChanged(_)
    ));
    assert_eq!(ropes.len(), 1);
    assert!(!world.colliders.is_empty());
    world.bodies = saved_bodies;
    ropes.remove(ID, &mut world, rope).unwrap();
    assert!(
        ropes.is_empty()
            && world.soft_bodies.is_empty()
            && world.bodies.is_empty()
            && world.colliders.is_empty()
    );
}
