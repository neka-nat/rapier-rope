//! World-bound registration, lifetime checks and a caller-driven step protocol.
use crate::{
    build_rope,
    command::*,
    error::RopeError,
    id::{Arena, *},
    model::RopeSpec,
    rapier::prelude::*,
    sampling::SampledRope,
};
use std::{collections::BTreeMap, fmt};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegistryPhase {
    Access,
    Prepare,
    Inspect,
}

#[derive(Clone, Debug, PartialEq)]
pub enum RopeSetErrorKind {
    WorldMismatch {
        expected: WorldId,
        actual: WorldId,
    },
    ForeignSet {
        expected: RopeSetId,
        actual: RopeSetId,
    },
    StaleRopeHandle,
    StaleAttachmentHandle,
    DuplicateRopeName(String),
    Definition(Box<RopeError>),
    Location(Box<RopeError>),
    InvalidTarget(&'static str),
    MissingSoftBody(SoftBodyHandle),
    MissingProxy(RigidBodyHandle),
    MissingCollider(ColliderHandle),
    MissingRigidBody(RigidBodyHandle),
    TopologyChanged(&'static str),
    ConstraintChanged(&'static str),
    InvalidParticle {
        particle: u32,
        count: usize,
    },
    AlreadyConstrained {
        particle: u32,
    },
    NotPinned {
        particle: u32,
    },
    ConflictingCommands {
        particle: u32,
    },
    UnsupportedAttachmentBody(RigidBodyHandle),
    InvalidTimestep,
    TimestepMismatch {
        requested_s: f64,
        world_s: f64,
    },
    WrongStep {
        expected: u64,
        actual: u64,
    },
    StepAlreadyPrepared {
        step: u64,
    },
    StepNotPrepared,
    RegistryBusy {
        step: u64,
    },
    NonFiniteState(&'static str),
    WorldQuarantined,
    IdExhausted,
    ApplyFailed(&'static str),
}

/// Inspect failures do not roll back a world that the caller may have advanced.
#[derive(Clone, Debug, PartialEq)]
pub struct RopeSetError {
    pub phase: RegistryPhase,
    pub rope: Option<RopeHandle>,
    pub kind: RopeSetErrorKind,
    /// Zero for all validation failures. Apply failures retain the completed count.
    pub applied_commands: usize,
}

impl RopeSetError {
    pub(crate) fn new(
        phase: RegistryPhase,
        rope: Option<RopeHandle>,
        kind: RopeSetErrorKind,
    ) -> Self {
        Self {
            phase,
            rope,
            kind,
            applied_commands: 0,
        }
    }
}
impl fmt::Display for RopeSetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "rope registry {:?}, {:?}: {:?} ({} commands applied)",
            self.phase, self.rope, self.kind, self.applied_commands
        )
    }
}
impl std::error::Error for RopeSetError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match &self.kind {
            RopeSetErrorKind::Definition(error) | RopeSetErrorKind::Location(error) => {
                Some(error.as_ref())
            }
            _ => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct MeshStamp {
    proxy: RigidBodyHandle,
    collider: ColliderHandle,
    enabled: bool,
    indices: Vec<[u32; 3]>,
    particles: Vec<u32>,
}

#[derive(Debug)]
struct RopeEntry {
    native: SoftBodyHandle,
    name: String,
    rope_reference: Option<(RopeSpec, SampledRope)>,
    particle_count: usize,
    version: u32,
    edges: Vec<([u32; 2], SoftBodyEdgeKind)>,
    proxies: Vec<RigidBodyHandle>,
    meshes: Vec<MeshStamp>,
    pinned: Vec<bool>,
    attachments: Vec<AttachmentHandle>,
}

#[derive(Debug)]
struct AttachmentEntry {
    rope: RopeHandle,
    particle: u32,
    body: RigidBodyHandle,
    local_anchor: Vector,
}

/// A checked borrow. No mutable native access is provided by the registry.
pub struct RopeView<'a> {
    pub handle: RopeHandle,
    pub native_handle: SoftBodyHandle,
    pub specification: &'a RopeSpec,
    pub samples: &'a SampledRope,
    pub soft_body: &'a SoftBody,
}

#[derive(Clone, Debug)]
pub struct AttachmentView {
    pub handle: AttachmentHandle,
    pub rope: RopeHandle,
    pub particle: u32,
    pub body: RigidBodyHandle,
    pub local_anchor_m: [f64; 3],
}

#[derive(Clone, Debug)]
pub struct RemovedRope {
    /// False means the native body had already been deleted externally.
    pub native_removed: bool,
    pub specification: RopeSpec,
    pub samples: SampledRope,
}

#[derive(Clone, Debug)]
pub struct RopeStepObservation {
    pub handle: RopeHandle,
    pub native_handle: SoftBodyHandle,
    pub particle_count: usize,
    pub topology_version: u32,
    pub attachment_count: usize,
}

#[derive(Clone, Debug)]
pub struct StepReport {
    pub step: u64,
    pub dt_s: f64,
    pub applied_commands: usize,
    pub ropes: Vec<RopeStepObservation>,
}

#[derive(Debug)]
struct PendingStep {
    step: u64,
    dt: Real,
    applied_commands: usize,
}

/// Owns registration metadata, never the world or time advancement.
///
/// Step numbers begin at zero. A successful prepare must be paired with inspect.
/// A matching inspect consumes the step even when it reports an integrity failure.
/// Wrong world/step calls preserve the pending step. This protocol cannot prove
/// that `world.step()` was called, or how many times the caller invoked it.
///
/// ```
/// use rapier_rope::{CollisionSettings, NativeRopeMaterial, RopeCommand,
///     RopeSet, RopeSpec, SamplingSettings, SpringSettings, WorldId};
/// let id = WorldId(1);
/// let mut world = rapier_rope::rapier::prelude::PhysicsWorld::new();
/// world.integration_parameters.dt = 1.0 / 240.0;
/// let spec = RopeSpec::new("cable", vec![[0.0, 1.5, 0.0], [1.0, 1.5, 0.0]],
///     NativeRopeMaterial::new(0.1, SpringSettings::new(500.0, 1.0),
///         SpringSettings::new(20.0, 0.8)),
///     SamplingSettings::new(1.0 / 32.0), CollisionSettings::new(0.005));
/// let mut ropes = RopeSet::new(id)?;
/// let rope = ropes.insert(id, &mut world, &spec)?;
/// ropes.prepare(id, &mut world, 0, 1.0 / 240.0,
///     &[RopeCommand::Pin { rope, particle: 0 }])?;
/// world.step();
/// let report = ropes.inspect(id, &world, 0)?;
/// assert_eq!(report.ropes[0].handle, rope);
/// assert_eq!(ropes.get(id, &world, rope)?.samples.reference_length_m(), 1.0);
/// ropes.remove(id, &mut world, rope)?;
/// assert!(world.soft_bodies.is_empty());
/// # Ok::<(), rapier_rope::RopeSetError>(())
/// ```
#[derive(Debug)]
pub struct RopeSet {
    id: RopeSetId,
    world_id: WorldId,
    ropes: Arena<RopeEntry>,
    attachments: Arena<AttachmentEntry>,
    names: BTreeMap<String, RopeHandle>,
    next_step: u64,
    pending: Option<PendingStep>,
}

fn mesh_stamps(body: &SoftBody) -> Vec<MeshStamp> {
    body.clusters()
        .iter()
        .flat_map(|c| c.meshes().map(move |m| (c.proxy(), m)))
        .map(|(proxy, m)| MeshStamp {
            proxy,
            collider: m.collider(),
            enabled: m.collision_enabled(),
            indices: m.indices().to_vec(),
            particles: match m.binding() {
                SoftMeshMapping::Direct { particles } => particles.clone(),
                SoftMeshMapping::Skinned { .. } => Vec::new(),
            },
        })
        .collect()
}

#[allow(clippy::unnecessary_cast)]
fn scalar(value: Real) -> f64 {
    value as f64
}

impl RopeSet {
    pub(crate) fn pending_step(&self) -> Option<u64> {
        self.pending.as_ref().map(|p| p.step)
    }
    pub fn new(world_id: WorldId) -> Result<Self, RopeSetError> {
        let id = RopeSetId::fresh().ok_or_else(|| {
            RopeSetError::new(RegistryPhase::Access, None, RopeSetErrorKind::IdExhausted)
        })?;
        Ok(Self {
            id,
            world_id,
            ropes: Arena::default(),
            attachments: Arena::default(),
            names: BTreeMap::new(),
            next_step: 0,
            pending: None,
        })
    }
    pub fn id(&self) -> RopeSetId {
        self.id
    }
    pub fn world_id(&self) -> WorldId {
        self.world_id
    }
    pub fn len(&self) -> usize {
        self.ropes.len()
    }
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Registered IDs only; use `get` to verify their current native integrity.
    /// Remove each rope explicitly before dropping the registry; Drop cannot borrow a world.
    pub fn handles(&self) -> impl Iterator<Item = RopeHandle> + '_ {
        self.ropes.iter().map(|(slot, generation, _)| RopeHandle {
            set: self.id,
            slot,
            generation,
        })
    }
    pub fn attachment_count(&self) -> usize {
        self.attachments.len()
    }
    pub fn next_step(&self) -> u64 {
        self.next_step
    }

    fn world(&self, id: WorldId, phase: RegistryPhase) -> Result<(), RopeSetError> {
        if id != self.world_id {
            return Err(RopeSetError::new(
                phase,
                None,
                RopeSetErrorKind::WorldMismatch {
                    expected: self.world_id,
                    actual: id,
                },
            ));
        }
        Ok(())
    }
    fn idle(&self) -> Result<(), RopeSetError> {
        if let Some(pending) = &self.pending {
            return Err(RopeSetError::new(
                RegistryPhase::Access,
                None,
                RopeSetErrorKind::RegistryBusy { step: pending.step },
            ));
        }
        Ok(())
    }
    fn entry(&self, handle: RopeHandle, phase: RegistryPhase) -> Result<&RopeEntry, RopeSetError> {
        if handle.set != self.id {
            return Err(RopeSetError::new(
                phase,
                Some(handle),
                RopeSetErrorKind::ForeignSet {
                    expected: self.id,
                    actual: handle.set,
                },
            ));
        }
        self.ropes
            .get(handle.slot, handle.generation)
            .ok_or_else(|| {
                RopeSetError::new(phase, Some(handle), RopeSetErrorKind::StaleRopeHandle)
            })
    }
    fn attachment(
        &self,
        handle: AttachmentHandle,
        phase: RegistryPhase,
    ) -> Result<&AttachmentEntry, RopeSetError> {
        if handle.set != self.id {
            return Err(RopeSetError::new(
                phase,
                None,
                RopeSetErrorKind::ForeignSet {
                    expected: self.id,
                    actual: handle.set,
                },
            ));
        }
        self.attachments
            .get(handle.slot, handle.generation)
            .ok_or_else(|| RopeSetError::new(phase, None, RopeSetErrorKind::StaleAttachmentHandle))
    }

    /// Validate the definition and duplicate name before creating native objects.
    pub fn insert(
        &mut self,
        id: WorldId,
        world: &mut PhysicsWorld,
        spec: &RopeSpec,
    ) -> Result<RopeHandle, RopeSetError> {
        self.world(id, RegistryPhase::Access)?;
        self.idle()?;
        if self.names.contains_key(&spec.name) {
            return Err(RopeSetError::new(
                RegistryPhase::Access,
                None,
                RopeSetErrorKind::DuplicateRopeName(spec.name.clone()),
            ));
        }
        if !self.ropes.can_insert(1) {
            return Err(RopeSetError::new(
                RegistryPhase::Access,
                None,
                RopeSetErrorKind::IdExhausted,
            ));
        }
        let (builder, samples, specification) = build_rope(spec)
            .map_err(|e| {
                RopeSetError::new(
                    RegistryPhase::Access,
                    None,
                    RopeSetErrorKind::Definition(Box::new(e)),
                )
            })?
            .into_parts();
        self.insert_native(
            id,
            world,
            spec.name.clone(),
            builder,
            Some((specification, samples)),
        )
    }

    // Shared native lifetime/constraint checks for ropes and branched graphs.
    // A graph has no fabricated RopeSpec or global arc-length ordering.
    pub(crate) fn insert_graph(
        &mut self,
        id: WorldId,
        world: &mut PhysicsWorld,
        name: String,
        builder: SoftBodyBuilder,
    ) -> Result<RopeHandle, RopeSetError> {
        self.insert_native(id, world, name, builder, None)
    }

    fn insert_native(
        &mut self,
        id: WorldId,
        world: &mut PhysicsWorld,
        name: String,
        builder: SoftBodyBuilder,
        rope_reference: Option<(RopeSpec, SampledRope)>,
    ) -> Result<RopeHandle, RopeSetError> {
        self.world(id, RegistryPhase::Access)?;
        self.idle()?;
        if self.names.contains_key(&name) {
            return Err(RopeSetError::new(
                RegistryPhase::Access,
                None,
                RopeSetErrorKind::DuplicateRopeName(name),
            ));
        }
        if !self.ropes.can_insert(1) {
            return Err(RopeSetError::new(
                RegistryPhase::Access,
                None,
                RopeSetErrorKind::IdExhausted,
            ));
        }
        let native = world.insert_soft_body(builder);
        let body = &world.soft_bodies[native];
        let entry = RopeEntry {
            native,
            name: name.clone(),
            rope_reference,
            particle_count: body.num_particles(),
            version: body.topology_version(),
            edges: body.edges().iter().map(|e| (e.vertices, e.kind)).collect(),
            proxies: body.clusters().iter().map(|c| c.proxy()).collect(),
            meshes: mesh_stamps(body),
            pinned: vec![false; body.num_particles()],
            attachments: Vec::new(),
        };
        let (slot, generation) = self
            .ropes
            .insert(entry)
            .expect("capacity checked before native insertion");
        let handle = RopeHandle {
            set: self.id,
            slot,
            generation,
        };
        self.names.insert(name, handle);
        Ok(handle)
    }

    /// Remove owned native objects and metadata, preserving external rigid bodies.
    /// Already-deleted native ropes can be explicitly unregistered by this method.
    /// Changed proxy/collider ownership is rejected instead of deleting foreign assets.
    pub fn remove(
        &mut self,
        id: WorldId,
        world: &mut PhysicsWorld,
        handle: RopeHandle,
    ) -> Result<RemovedRope, RopeSetError> {
        self.world(id, RegistryPhase::Access)?;
        if self
            .entry(handle, RegistryPhase::Access)?
            .rope_reference
            .is_none()
        {
            return Err(RopeSetError::new(
                RegistryPhase::Access,
                Some(handle),
                RopeSetErrorKind::InvalidTarget("not a rope definition"),
            ));
        }
        let (native_removed, entry) = self.remove_entry(id, world, handle)?;
        let (specification, samples) = entry.rope_reference.expect("rope checked before removal");
        Ok(RemovedRope {
            native_removed,
            specification,
            samples,
        })
    }

    pub(crate) fn remove_graph(
        &mut self,
        id: WorldId,
        world: &mut PhysicsWorld,
        handle: RopeHandle,
    ) -> Result<bool, RopeSetError> {
        self.remove_entry(id, world, handle)
            .map(|(removed, _)| removed)
    }

    fn remove_entry(
        &mut self,
        id: WorldId,
        world: &mut PhysicsWorld,
        handle: RopeHandle,
    ) -> Result<(bool, RopeEntry), RopeSetError> {
        self.world(id, RegistryPhase::Access)?;
        self.idle()?;
        let entry = self.entry(handle, RegistryPhase::Access)?;
        if let Some(body) = world.soft_bodies.get(entry.native) {
            if body
                .clusters()
                .iter()
                .map(|c| c.proxy())
                .collect::<Vec<_>>()
                != entry.proxies
                || mesh_stamps(body).iter().any(|m| {
                    world.colliders.contains(m.collider)
                        && !entry
                            .meshes
                            .iter()
                            .any(|owned| owned.collider == m.collider && owned.proxy == m.proxy)
                })
            {
                return Err(RopeSetError::new(
                    RegistryPhase::Access,
                    Some(handle),
                    RopeSetErrorKind::TopologyChanged("native ownership changed; removal refused"),
                ));
            }
            self.check_ownership(world, handle, entry, RegistryPhase::Access, false)?;
        } else if entry.proxies.iter().any(|&p| world.bodies.contains(p))
            || entry
                .meshes
                .iter()
                .any(|m| m.enabled && world.colliders.contains(m.collider))
        {
            return Err(RopeSetError::new(
                RegistryPhase::Access,
                Some(handle),
                RopeSetErrorKind::TopologyChanged(
                    "native body missing while registered assets remain; removal refused",
                ),
            ));
        }
        let native_removed = world.remove_soft_body(entry.native).is_some();
        let entry = self
            .ropes
            .remove(handle.slot, handle.generation)
            .expect("handle checked before removal");
        for attachment in &entry.attachments {
            self.attachments
                .remove(attachment.slot, attachment.generation);
        }
        self.names.remove(&entry.name);
        Ok((native_removed, entry))
    }

    fn check_ownership(
        &self,
        world: &PhysicsWorld,
        handle: RopeHandle,
        entry: &RopeEntry,
        phase: RegistryPhase,
        require_assets: bool,
    ) -> Result<(), RopeSetError> {
        let fail = |kind| RopeSetError::new(phase, Some(handle), kind);
        for &proxy in &entry.proxies {
            if let Some(rb) = world.bodies.get(proxy) {
                if rb.soft_body() != Some(entry.native) {
                    return Err(fail(RopeSetErrorKind::TopologyChanged(
                        "proxy ownership changed",
                    )));
                }
                // A foreign collider added to a proxy must not be deleted by remove.
                if rb.colliders().iter().any(|&c| {
                    !entry
                        .meshes
                        .iter()
                        .any(|m| m.collider == c && m.proxy == proxy)
                }) {
                    return Err(fail(RopeSetErrorKind::TopologyChanged(
                        "unmanaged collider attached to proxy",
                    )));
                }
            } else if require_assets {
                return Err(fail(RopeSetErrorKind::MissingProxy(proxy)));
            } else if entry
                .meshes
                .iter()
                .any(|m| m.proxy == proxy && m.enabled && world.colliders.contains(m.collider))
            {
                return Err(fail(RopeSetErrorKind::TopologyChanged(
                    "proxy missing while colliders remain; removal refused",
                )));
            }
        }
        for mesh in &entry.meshes {
            if !mesh.enabled {
                continue;
            }
            if let Some(collider) = world.colliders.get(mesh.collider) {
                if collider.parent() != Some(mesh.proxy) {
                    return Err(fail(RopeSetErrorKind::TopologyChanged(
                        "collider parent changed",
                    )));
                }
            } else if require_assets {
                return Err(fail(RopeSetErrorKind::MissingCollider(mesh.collider)));
            }
        }
        Ok(())
    }

    fn validate<'a>(
        &self,
        world: &'a PhysicsWorld,
        handle: RopeHandle,
        entry: &RopeEntry,
        phase: RegistryPhase,
    ) -> Result<&'a SoftBody, RopeSetError> {
        let fail = |kind| RopeSetError::new(phase, Some(handle), kind);
        let body = world
            .soft_bodies
            .get(entry.native)
            .ok_or_else(|| fail(RopeSetErrorKind::MissingSoftBody(entry.native)))?;
        self.check_ownership(world, handle, entry, phase, true)?;
        if body.topology_version() != entry.version
            || body.has_pending_tears()
            || body.num_particles() != entry.particle_count
            || body
                .edges()
                .iter()
                .map(|e| (e.vertices, e.kind))
                .ne(entry.edges.iter().copied())
            || body
                .clusters()
                .iter()
                .map(|c| c.proxy())
                .ne(entry.proxies.iter().copied())
            || mesh_stamps(body) != entry.meshes
        {
            return Err(fail(RopeSetErrorKind::TopologyChanged(
                "particle/edge/mesh topology or version changed",
            )));
        }
        if body
            .particles()
            .iter()
            .map(|p| p.is_pinned())
            .ne(entry.pinned.iter().copied())
        {
            return Err(fail(RopeSetErrorKind::ConstraintChanged(
                "pin state changed outside registry",
            )));
        }
        for &id in &entry.attachments {
            let expected = self.attachment(id, phase)?;
            let rb = world
                .bodies
                .get(expected.body)
                .ok_or_else(|| fail(RopeSetErrorKind::MissingRigidBody(expected.body)))?;
            if !rb.translation().is_finite()
                || !rb.linvel().is_finite()
                || !rb.angvel().is_finite()
                || !rb
                    .position()
                    .transform_point(expected.local_anchor)
                    .is_finite()
                || !finite_attachment_target(rb, expected.local_anchor)
            {
                return Err(fail(RopeSetErrorKind::NonFiniteState(
                    "attachment body pose/velocity",
                )));
            }
        }
        if body.particle_attachments().len() != entry.attachments.len() {
            return Err(fail(RopeSetErrorKind::ConstraintChanged(
                "attachment count changed outside registry",
            )));
        }
        for (actual, &id) in body.particle_attachments().iter().zip(&entry.attachments) {
            let expected = self.attachment(id, phase)?;
            if actual.particle != expected.particle
                || actual.body != expected.body
                || actual.local_anchor != expected.local_anchor
            {
                return Err(fail(RopeSetErrorKind::ConstraintChanged(
                    "attachment identity/order/anchor changed outside registry",
                )));
            }
        }
        if !body
            .particle_positions()
            .chain(body.particle_velocities())
            .all(|p| p.is_finite())
            || !body
                .particles()
                .iter()
                .all(|p| p.kinematic_target().is_none_or(|t| t.is_finite()))
            || !body.edges().iter().all(|e| e.impulse().is_finite())
            || !body
                .particle_attachments()
                .iter()
                .all(|a| a.impulse().is_finite())
        {
            return Err(fail(RopeSetErrorKind::NonFiniteState(
                "particle or native impulse",
            )));
        }
        if !world.quarantine().is_empty() {
            return Err(fail(RopeSetErrorKind::WorldQuarantined));
        }
        Ok(body)
    }

    pub fn get<'a>(
        &'a self,
        id: WorldId,
        world: &'a PhysicsWorld,
        handle: RopeHandle,
    ) -> Result<RopeView<'a>, RopeSetError> {
        self.world(id, RegistryPhase::Access)?;
        let entry = self.entry(handle, RegistryPhase::Access)?;
        let soft_body = self.validate(world, handle, entry, RegistryPhase::Access)?;
        let (specification, samples) = entry.rope_reference.as_ref().ok_or_else(|| {
            RopeSetError::new(
                RegistryPhase::Access,
                Some(handle),
                RopeSetErrorKind::InvalidTarget("not a rope definition"),
            )
        })?;
        Ok(RopeView {
            handle,
            native_handle: entry.native,
            specification,
            samples,
            soft_body,
        })
    }
    pub(crate) fn get_graph<'a>(
        &self,
        id: WorldId,
        world: &'a PhysicsWorld,
        handle: RopeHandle,
    ) -> Result<(SoftBodyHandle, &'a SoftBody), RopeSetError> {
        self.world(id, RegistryPhase::Access)?;
        let entry = self.entry(handle, RegistryPhase::Access)?;
        Ok((
            entry.native,
            self.validate(world, handle, entry, RegistryPhase::Access)?,
        ))
    }

    pub fn get_by_name<'a>(
        &'a self,
        id: WorldId,
        world: &'a PhysicsWorld,
        name: &str,
    ) -> Result<Option<RopeView<'a>>, RopeSetError> {
        self.world(id, RegistryPhase::Access)?;
        self.names
            .get(name)
            .map(|&h| self.get(id, world, h))
            .transpose()
    }
    pub fn get_attachment(
        &self,
        id: WorldId,
        world: &PhysicsWorld,
        handle: AttachmentHandle,
    ) -> Result<AttachmentView, RopeSetError> {
        self.world(id, RegistryPhase::Access)?;
        let entry = self.attachment(handle, RegistryPhase::Access)?;
        self.get_graph(id, world, entry.rope)?;
        Ok(AttachmentView {
            handle,
            rope: entry.rope,
            particle: entry.particle,
            body: entry.body,
            local_anchor_m: [
                scalar(entry.local_anchor.x),
                scalar(entry.local_anchor.y),
                scalar(entry.local_anchor.z),
            ],
        })
    }

    fn validate_all(&self, world: &PhysicsWorld, phase: RegistryPhase) -> Result<(), RopeSetError> {
        for (slot, generation, entry) in self.ropes.iter() {
            self.validate(
                world,
                RopeHandle {
                    set: self.id,
                    slot,
                    generation,
                },
                entry,
                phase,
            )?;
        }
        if !world.quarantine().is_empty() {
            return Err(RopeSetError::new(
                phase,
                None,
                RopeSetErrorKind::WorldQuarantined,
            ));
        }
        Ok(())
    }

    /// `world.integration_parameters.dt` must already equal `h` in native precision.
    /// Validation failure applies zero commands and leaves the step reusable.
    #[allow(clippy::unnecessary_cast)]
    pub fn prepare(
        &mut self,
        id: WorldId,
        world: &mut PhysicsWorld,
        step: u64,
        h: f64,
        commands: &[RopeCommand],
    ) -> Result<PreparedStep, RopeSetError> {
        let phase = RegistryPhase::Prepare;
        let fail = |kind| RopeSetError::new(phase, None, kind);
        self.world(id, phase)?;
        if let Some(p) = &self.pending {
            return Err(fail(RopeSetErrorKind::StepAlreadyPrepared { step: p.step }));
        }
        if step != self.next_step {
            return Err(fail(RopeSetErrorKind::WrongStep {
                expected: self.next_step,
                actual: step,
            }));
        }
        if step == u64::MAX {
            return Err(fail(RopeSetErrorKind::IdExhausted));
        }
        let dt = h as Real;
        if !h.is_finite() || h <= 0.0 || !dt.is_finite() || dt <= 0.0 || !(1.0 / dt).is_finite() {
            return Err(fail(RopeSetErrorKind::InvalidTimestep));
        }
        if world.integration_parameters.dt != dt {
            return Err(fail(RopeSetErrorKind::TimestepMismatch {
                requested_s: scalar(dt),
                world_s: scalar(world.integration_parameters.dt),
            }));
        }
        self.validate_all(world, phase)?;
        // Prospective states allow release -> acquisition without mutating the world.
        let mut states: BTreeMap<(RopeHandle, u32), (bool, bool)> = BTreeMap::new();
        let mut operations: BTreeMap<(RopeHandle, u32), (bool, bool)> = BTreeMap::new();
        let mut targets: BTreeMap<(RopeHandle, u32), Vector> = BTreeMap::new();
        let mut attach_count = 0;
        for command in commands {
            let (rope, particle, release) = match *command {
                RopeCommand::Pin { rope, particle }
                | RopeCommand::PinAt { rope, particle, .. }
                | RopeCommand::MovePin { rope, particle, .. }
                | RopeCommand::Attach { rope, particle, .. } => (rope, particle, false),
                RopeCommand::Unpin { rope, particle } => (rope, particle, true),
                RopeCommand::Detach { attachment } => {
                    let a = self.attachment(attachment, phase)?;
                    (a.rope, a.particle, true)
                }
            };
            let fail = |kind| RopeSetError::new(phase, Some(rope), kind);
            let entry = self.entry(rope, phase)?;
            if particle as usize >= entry.pinned.len() {
                return Err(fail(RopeSetErrorKind::InvalidParticle {
                    particle,
                    count: entry.pinned.len(),
                }));
            }
            let (released, acquired) = operations.entry((rope, particle)).or_default();
            let moving = matches!(command, RopeCommand::MovePin { .. });
            if (!moving && *acquired)
                || (release && (*released || targets.contains_key(&(rope, particle))))
            {
                return Err(fail(RopeSetErrorKind::ConflictingCommands { particle }));
            }
            if release {
                *released = true;
            } else if !moving {
                *acquired = true;
            }
            let state = states.entry((rope, particle)).or_insert_with(|| {
                (
                    entry.pinned[particle as usize],
                    entry.attachments.iter().any(|&id| {
                        self.attachment(id, phase)
                            .is_ok_and(|a| a.particle == particle)
                    }),
                )
            });
            match *command {
                RopeCommand::Pin { .. } | RopeCommand::PinAt { .. } => {
                    if state.0 || state.1 {
                        return Err(fail(RopeSetErrorKind::AlreadyConstrained { particle }));
                    }
                    state.0 = true;
                }
                RopeCommand::MovePin { .. } => {
                    if !state.0 {
                        return Err(fail(RopeSetErrorKind::NotPinned { particle }));
                    }
                }
                RopeCommand::Unpin { .. } => {
                    if !state.0 {
                        return Err(fail(RopeSetErrorKind::NotPinned { particle }));
                    }
                    state.0 = false;
                }
                RopeCommand::Attach { body, .. } => {
                    if state.0 || state.1 {
                        return Err(fail(RopeSetErrorKind::AlreadyConstrained { particle }));
                    }
                    let rb = world
                        .bodies
                        .get(body)
                        .ok_or_else(|| fail(RopeSetErrorKind::MissingRigidBody(body)))?;
                    if rb.soft_body().is_some() {
                        return Err(fail(RopeSetErrorKind::UnsupportedAttachmentBody(body)));
                    }
                    let position =
                        world.soft_bodies[entry.native].particle_position(particle as usize);
                    if !rb.position().inverse_transform_point(position).is_finite()
                        || !rb.linvel().is_finite()
                        || !rb.angvel().is_finite()
                        || !finite_attachment_target(
                            rb,
                            rb.position().inverse_transform_point(position),
                        )
                    {
                        return Err(fail(RopeSetErrorKind::NonFiniteState(
                            "attachment body/anchor",
                        )));
                    }
                    state.1 = true;
                    attach_count += 1;
                }
                RopeCommand::Detach { .. } => {
                    state.1 = false;
                }
            }
            if let RopeCommand::PinAt {
                position_m: point, ..
            }
            | RopeCommand::MovePin {
                target_m: point, ..
            } = *command
            {
                if targets.contains_key(&(rope, particle)) {
                    return Err(fail(RopeSetErrorKind::ConflictingCommands { particle }));
                }
                let target = checked_target(point).map_err(fail)?;
                let native = &world.soft_bodies[entry.native];
                // Bound squared distances as well as coordinates: native vector
                // arithmetic must not overflow before the next step.
                if native
                    .particle_positions()
                    .any(|p| !(target - p).length_squared().is_finite())
                    || targets.iter().any(|(&(other_rope, _), &p)| {
                        other_rope == rope && !(target - p).length_squared().is_finite()
                    })
                {
                    return Err(fail(RopeSetErrorKind::InvalidTarget(
                        "target distance overflow",
                    )));
                }
                if moving {
                    let velocity = (target - native.particle_position(particle as usize)) / dt;
                    if !velocity.is_finite() || !velocity.length_squared().is_finite() {
                        return Err(fail(RopeSetErrorKind::InvalidTarget(
                            "target velocity overflow",
                        )));
                    }
                }
                targets.insert((rope, particle), target);
            }
        }
        if !self.attachments.can_insert(attach_count) {
            return Err(fail(RopeSetErrorKind::IdExhausted));
        }
        let mut result = PreparedStep {
            step,
            dt_s: scalar(dt),
            applied_commands: 0,
            created_attachments: Vec::new(),
        };
        // Exclusive &mut world + no callback between validation and apply. The current
        // primitive operations cannot fail after validation, apart from invariant bugs.
        for (command_index, command) in commands.iter().enumerate() {
            let (rope, particle) = match *command {
                RopeCommand::Pin { rope, particle }
                | RopeCommand::PinAt { rope, particle, .. }
                | RopeCommand::MovePin { rope, particle, .. }
                | RopeCommand::Unpin { rope, particle }
                | RopeCommand::Attach { rope, particle, .. } => (rope, particle),
                RopeCommand::Detach { attachment } => {
                    let a = self.attachment(attachment, phase)?;
                    (a.rope, a.particle)
                }
            };
            let entry = self
                .ropes
                .get_mut(rope.slot, rope.generation)
                .expect("validated rope");
            let body = world
                .soft_bodies
                .get_mut(entry.native)
                .expect("validated native body");
            match *command {
                RopeCommand::Pin { .. } | RopeCommand::Unpin { .. } => {
                    let pin = matches!(command, RopeCommand::Pin { .. });
                    body.set_particle_pinned(particle as usize, pin);
                    entry.pinned[particle as usize] = pin;
                }
                RopeCommand::PinAt { position_m, .. } => {
                    body.set_particle_position(
                        particle as usize,
                        checked_target(position_m).expect("validated target"),
                    );
                    body.set_particle_pinned(particle as usize, true);
                    entry.pinned[particle as usize] = true;
                }
                RopeCommand::MovePin { target_m, .. } => {
                    body.set_particle_kinematic_target(
                        particle as usize,
                        checked_target(target_m).expect("validated target"),
                    );
                }
                RopeCommand::Attach { body: target, .. } => {
                    body.attach_particle(particle as usize, target, &world.bodies);
                    let a = body
                        .particle_attachments()
                        .last()
                        .expect("native attach adds one item");
                    let (slot, generation) = self
                        .attachments
                        .insert(AttachmentEntry {
                            rope,
                            particle,
                            body: target,
                            local_anchor: a.local_anchor,
                        })
                        .expect("attachment capacity validated");
                    let handle = AttachmentHandle {
                        set: self.id,
                        slot,
                        generation,
                    };
                    entry.attachments.push(handle);
                    result.created_attachments.push(CreatedAttachment {
                        command_index,
                        handle,
                        rope,
                        particle,
                        body: target,
                        local_anchor_m: [
                            scalar(a.local_anchor.x),
                            scalar(a.local_anchor.y),
                            scalar(a.local_anchor.z),
                        ],
                    });
                }
                RopeCommand::Detach { attachment } => {
                    if !body.detach_particle(particle as usize) {
                        let mut error = RopeSetError::new(
                            phase,
                            Some(rope),
                            RopeSetErrorKind::ApplyFailed("native attachment disappeared"),
                        );
                        error.applied_commands = result.applied_commands;
                        return Err(error);
                    }
                    self.attachments
                        .remove(attachment.slot, attachment.generation);
                    entry.attachments.retain(|&a| a != attachment);
                }
            }
            result.applied_commands += 1;
        }
        self.pending = Some(PendingStep {
            step,
            dt,
            applied_commands: result.applied_commands,
        });
        Ok(result)
    }

    /// Observe after caller stepping. A matching attempt completes the protocol even
    /// on error, allowing explicit cleanup; it never restores native state.
    pub fn inspect(
        &mut self,
        id: WorldId,
        world: &PhysicsWorld,
        step: u64,
    ) -> Result<StepReport, RopeSetError> {
        let phase = RegistryPhase::Inspect;
        let fail = |kind| RopeSetError::new(phase, None, kind);
        self.world(id, phase)?;
        let pending = self
            .pending
            .as_ref()
            .ok_or_else(|| fail(RopeSetErrorKind::StepNotPrepared))?;
        if step != pending.step {
            return Err(fail(RopeSetErrorKind::WrongStep {
                expected: pending.step,
                actual: step,
            }));
        }
        let pending = self.pending.take().expect("pending step checked");
        self.next_step = step + 1;
        let checked = (|| {
            if world.integration_parameters.dt != pending.dt {
                return Err(fail(RopeSetErrorKind::TimestepMismatch {
                    requested_s: scalar(pending.dt),
                    world_s: scalar(world.integration_parameters.dt),
                }));
            }
            self.validate_all(world, phase)?;
            let ropes = self
                .ropes
                .iter()
                .map(|(slot, generation, entry)| RopeStepObservation {
                    handle: RopeHandle {
                        set: self.id,
                        slot,
                        generation,
                    },
                    native_handle: entry.native,
                    particle_count: entry.pinned.len(),
                    topology_version: entry.version,
                    attachment_count: entry.attachments.len(),
                })
                .collect();
            Ok(StepReport {
                step,
                dt_s: scalar(pending.dt),
                applied_commands: pending.applied_commands,
                ropes,
            })
        })();
        checked.map_err(|mut error: RopeSetError| {
            error.applied_commands = pending.applied_commands;
            error
        })
    }
}

#[allow(clippy::unnecessary_cast)]
fn checked_target(point: [f64; 3]) -> Result<Vector, RopeSetErrorKind> {
    let target = Vector::new(point[0] as Real, point[1] as Real, point[2] as Real);
    if !point.into_iter().all(f64::is_finite)
        || !target.is_finite()
        || !target.length_squared().is_finite()
    {
        return Err(RopeSetErrorKind::InvalidTarget(
            "world point is non-finite or overflows native arithmetic",
        ));
    }
    Ok(target)
}

fn finite_attachment_target(body: &RigidBody, anchor: Vector) -> bool {
    body.body_type() != RigidBodyType::KinematicPositionBased
        || (body.next_position().translation.is_finite()
            && body.next_position().rotation.is_finite()
            && body.next_position().transform_point(anchor).is_finite())
}
