//! Connected tree harnesses: one native body, shared junction particles.
//! Span material coordinates remain independent; there is no global arc length.
//!
//! ```
//! use rapier_rope::{rapier::prelude::*, *};
//! let material = NativeRopeMaterial::new(0.1, SpringSettings::new(500.0, 1.0),
//!     SpringSettings::new(20.0, 0.8));
//! let spec = HarnessSpec::new("Y", vec![
//!     JunctionSpec::new("J", [0.0, 1.0, 0.0]),
//!     JunctionSpec::new("mount", [0.0, 1.5, 0.0]),
//!     JunctionSpec::new("a", [-0.5, 0.8, 0.0]),
//!     JunctionSpec::new("b", [0.5, 0.8, 0.0]),
//! ], ["mount", "a", "b"].into_iter().map(|end|
//!     SpanSpec::new(end, "J", end, material.clone(), SamplingSettings::new(0.05))
//! ).collect(), CollisionSettings::new(0.005));
//! let id = WorldId(1);
//! let mut world = PhysicsWorld::new();
//! world.integration_parameters.dt = 1.0 / 240.0;
//! let mut harnesses = HarnessSet::new(id)?;
//! let harness = harnesses.insert(id, &mut world, &spec)?;
//! harnesses.prepare(id, &mut world, 0, 1.0 / 240.0, &[HarnessCommand::Pin {
//!     harness, location: HarnessLocation::Junction("mount".into()),
//!     position_m: [0.0, 1.5, 0.0],
//! }])?;
//! world.step();
//! harnesses.inspect(id, &world, 0)?;
//! assert_eq!(world.soft_bodies.len(), 1);
//! harnesses.remove(id, &mut world, harness)?;
//! # Ok::<(), HarnessError>(())
//! ```
use crate::{
    AttachmentHandle, CollisionSettings, HarnessHandle, NativeDynamicsSettings, NativeRopeMaterial,
    Point3, RegistryPhase, ResolvedLocation, RigidPlacement, RopeCapability, RopeCommand,
    RopeError, RopeErrorKind, RopeHandle, RopeLocation, RopeSet, RopeSetError, RopeSetErrorKind,
    RopeSetId, RopeSpec, SampledRope, SamplingSettings, WorldId, build_rope, builder::convert,
    rapier::prelude::*,
};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, fmt, ops::Range};

/// Named graph vertex. Degree-one vertices are named terminal endpoints too.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JunctionSpec {
    pub name: String,
    pub reference_position_m: Point3,
}
impl JunctionSpec {
    pub fn new(name: impl Into<String>, reference_position_m: Point3) -> Self {
        Self {
            name: name.into(),
            reference_position_m,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpanSpec {
    pub name: String,
    pub start: String,
    pub end: String,
    /// Interior vertices only; endpoints come from the named graph vertices.
    pub interior_points_m: Vec<Point3>,
    pub material: NativeRopeMaterial,
    pub sampling: SamplingSettings,
    pub named_locations: Vec<crate::NamedLocation>,
    /// A requested override must equal the common body collision settings.
    pub collision: Option<CollisionSettings>,
}
impl SpanSpec {
    pub fn new(
        name: impl Into<String>,
        start: impl Into<String>,
        end: impl Into<String>,
        material: NativeRopeMaterial,
        sampling: SamplingSettings,
    ) -> Self {
        Self {
            name: name.into(),
            start: start.into(),
            end: end.into(),
            interior_points_m: vec![],
            material,
            sampling,
            named_locations: vec![],
            collision: None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HarnessSpec {
    pub name: String,
    pub junctions: Vec<JunctionSpec>,
    pub spans: Vec<SpanSpec>,
    pub placement: RigidPlacement,
    pub collision: CollisionSettings,
    pub dynamics: NativeDynamicsSettings,
    /// Global budget after endpoint sharing, in addition to each span's budget.
    pub max_particles: usize,
    pub required_capabilities: Vec<RopeCapability>,
}
impl HarnessSpec {
    pub fn new(
        name: impl Into<String>,
        junctions: Vec<JunctionSpec>,
        spans: Vec<SpanSpec>,
        collision: CollisionSettings,
    ) -> Self {
        Self {
            name: name.into(),
            junctions,
            spans,
            placement: Default::default(),
            collision,
            dynamics: Default::default(),
            max_particles: 65_536,
            required_capabilities: vec![],
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum HarnessLocation {
    Junction(String),
    Span {
        span: String,
        location: RopeLocation,
    },
}

#[derive(Clone, Debug)]
pub struct SampledSpan {
    reference: SampledRope,
    particles: Vec<u32>,
    structural: Range<usize>,
    bending: Range<usize>,
}
impl SampledSpan {
    /// Reference sample indices are span-local. Use particle_indices for native IDs.
    pub fn reference(&self) -> &SampledRope {
        &self.reference
    }
    pub fn particle_indices(&self) -> &[u32] {
        &self.particles
    }
    /// Range in the harness structural-edge list.
    pub fn structural_edge_range(&self) -> Range<usize> {
        self.structural.clone()
    }
    /// Range in the harness bend-edge list (not concatenated native indices).
    pub fn bending_edge_range(&self) -> Range<usize> {
        self.bending.clone()
    }
    /// Native/global particle index with span-local reference arc lengths.
    pub fn resolve_location(&self, location: &RopeLocation) -> Result<ResolvedLocation, RopeError> {
        let mut r = self.reference.resolve_location(location)?;
        r.particle_index = self.particles[r.particle_index as usize];
        Ok(r)
    }
}

#[derive(Clone, Debug)]
pub struct SampledHarness {
    name: String,
    positions: Vec<Point3>,
    masses: Vec<f64>,
    edges: Vec<[u32; 2]>,
    bends: Vec<[u32; 2]>,
    junctions: BTreeMap<String, u32>,
    spans: BTreeMap<String, SampledSpan>,
    nominal_mass: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResolvedHarnessLocation {
    pub particle_index: u32,
    /// Present only for a span request; these arcs belong to that span.
    pub material: Option<ResolvedLocation>,
}
impl SampledHarness {
    pub fn reference_positions_m(&self) -> &[Point3] {
        &self.positions
    }
    pub fn particle_masses_kg(&self) -> &[f64] {
        &self.masses
    }
    pub fn structural_edges(&self) -> &[[u32; 2]] {
        &self.edges
    }
    pub fn bend_edges(&self) -> &[[u32; 2]] {
        &self.bends
    }
    pub fn wire_segments(&self) -> &[[u32; 2]] {
        &self.edges
    }
    pub fn junction_particles(&self) -> &BTreeMap<String, u32> {
        &self.junctions
    }
    pub fn spans(&self) -> &BTreeMap<String, SampledSpan> {
        &self.spans
    }
    pub fn nominal_mass_kg(&self) -> f64 {
        self.nominal_mass
    }
    pub fn resolve_location(
        &self,
        location: &HarnessLocation,
    ) -> Result<ResolvedHarnessLocation, RopeError> {
        match location {
            HarnessLocation::Junction(name) => self
                .junctions
                .get(name)
                .map(|&particle_index| ResolvedHarnessLocation {
                    particle_index,
                    material: None,
                })
                .ok_or_else(|| {
                    RopeError::new(
                        &self.name,
                        "location.junction",
                        RopeErrorKind::UnknownLocation(name.clone()),
                    )
                }),
            HarnessLocation::Span { span, location } => {
                let s = self.spans.get(span).ok_or_else(|| {
                    RopeError::new(
                        &self.name,
                        "location.span",
                        RopeErrorKind::UnknownLocation(span.clone()),
                    )
                })?;
                let material = s.resolve_location(location)?;
                Ok(ResolvedHarnessLocation {
                    particle_index: material.particle_index,
                    material: Some(material),
                })
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct HarnessBuild {
    specification: HarnessSpec,
    samples: SampledHarness,
    native: SoftBodyBuilder,
}
impl HarnessBuild {
    pub fn specification(&self) -> &HarnessSpec {
        &self.specification
    }
    pub fn sampled(&self) -> &SampledHarness {
        &self.samples
    }
    pub fn native_builder(&self) -> &SoftBodyBuilder {
        &self.native
    }
    pub fn into_parts(self) -> (SoftBodyBuilder, SampledHarness, HarnessSpec) {
        (self.native, self.samples, self.specification)
    }
}

fn invalid(spec: &HarnessSpec, field: impl Into<String>, reason: &'static str) -> RopeError {
    RopeError::new(&spec.name, field, RopeErrorKind::InvalidValue(reason))
}
fn root(parents: &mut [usize], mut i: usize) -> usize {
    while parents[i] != i {
        parents[i] = parents[parents[i]];
        i = parents[i];
    }
    i
}

/// Reuse validated per-span sampling/conversion, then merge only named endpoints.
/// No zero-length weld edges or bending constraints between distinct spans.
pub fn build_harness(spec: &HarnessSpec) -> Result<HarnessBuild, RopeError> {
    if spec.name.trim().is_empty() || spec.junctions.len() < 2 || spec.spans.is_empty() {
        return Err(invalid(
            spec,
            "graph",
            "requires a name, at least two vertices and one span",
        ));
    }
    if !(2..=u32::MAX as usize / 2).contains(&spec.max_particles)
        || spec.junctions.len() > spec.max_particles
    {
        return Err(invalid(
            spec,
            "max_particles",
            "invalid global particle budget",
        ));
    }
    let mut junctions = BTreeMap::new();
    let mut positions = Vec::new();
    for j in &spec.junctions {
        if j.name.trim().is_empty() || !j.reference_position_m.iter().all(|x| x.is_finite()) {
            return Err(invalid(
                spec,
                "junctions",
                "names and positions must be valid",
            ));
        }
        if junctions
            .insert(j.name.clone(), positions.len() as u32)
            .is_some()
        {
            return Err(RopeError::new(
                &spec.name,
                "junctions.name",
                RopeErrorKind::DuplicateName(j.name.clone()),
            ));
        }
        positions.push(j.reference_position_m);
    }
    let mut parents: Vec<_> = (0..positions.len()).collect();
    let mut names = BTreeMap::new();
    // Check the entire topology before sampling or allocating any native body.
    for s in &spec.spans {
        if s.name.trim().is_empty() || names.insert(&s.name, ()).is_some() {
            return Err(invalid(spec, "spans.name", "blank or duplicate span name"));
        }
        if !(2..=u32::MAX as usize / 2).contains(&s.sampling.max_particles) {
            return Err(invalid(
                spec,
                format!("spans.{}.sampling.max_particles", s.name),
                "invalid span particle budget",
            ));
        }
        let a = *junctions
            .get(&s.start)
            .ok_or_else(|| invalid(spec, "spans.start", "unknown vertex"))?
            as usize;
        let b = *junctions
            .get(&s.end)
            .ok_or_else(|| invalid(spec, "spans.end", "unknown vertex"))? as usize;
        let (a, b) = (root(&mut parents, a), root(&mut parents, b));
        if a == b {
            return Err(invalid(
                spec,
                "graph",
                "cycles, self-loops and parallel spans are unsupported",
            ));
        }
        parents[b] = a;
        if s.collision.as_ref().is_some_and(|c| c != &spec.collision) {
            return Err(invalid(
                spec,
                format!("spans.{}.collision", s.name),
                "collision settings must be common to the body",
            ));
        }
        if s.material.linear_damping != spec.spans[0].material.linear_damping {
            return Err(invalid(
                spec,
                format!("spans.{}.linear_damping", s.name),
                "linear damping is a common body setting",
            ));
        }
    }
    let first_root = root(&mut parents, 0);
    if (0..parents.len()).any(|i| root(&mut parents, i) != first_root) {
        return Err(invalid(
            spec,
            "graph",
            "disconnected graphs and isolated vertices are unsupported",
        ));
    }
    let mut masses = vec![0.0; positions.len()];
    let mut native_positions = vec![Vector::ZERO; positions.len()];
    let mut edges = Vec::new();
    let mut bends = Vec::new();
    let mut softness = Vec::new();
    let mut tension = Vec::new();
    let mut span_map = BTreeMap::new();
    let mut native = None;
    let mut first_spec = None;
    let mut nominal_mass = 0.0;
    for span in &spec.spans {
        let (start, end) = (junctions[&span.start], junctions[&span.end]);
        let mut points = vec![positions[start as usize]];
        points.extend(&span.interior_points_m);
        points.push(positions[end as usize]);
        let mut rope = RopeSpec::new(
            format!("{}/{}", spec.name, span.name),
            points,
            span.material.clone(),
            span.sampling.clone(),
            spec.collision.clone(),
        );
        rope.placement = spec.placement;
        rope.dynamics = spec.dynamics.clone();
        rope.named_locations = span.named_locations.clone();
        rope.required_capabilities = spec.required_capabilities.clone();
        rope.sampling.max_particles = rope.sampling.max_particles.min(spec.max_particles);
        let (local, reference, _) = build_rope(&rope)?.into_parts();
        let count = reference.reference_positions_m().len();
        if positions
            .len()
            .checked_add(count - 2)
            .is_none_or(|n| n > spec.max_particles)
        {
            return Err(RopeError::new(
                &spec.name,
                "max_particles",
                RopeErrorKind::SamplingLimit {
                    max_particles: spec.max_particles,
                },
            ));
        }
        let mut particles = vec![start];
        for i in 1..count - 1 {
            particles.push(positions.len() as u32);
            positions.push(reference.reference_positions_m()[i]);
            masses.push(0.0);
            native_positions.push(local.positions[i]);
        }
        particles.push(end);
        for (i, &p) in particles.iter().enumerate() {
            native_positions[p as usize] = local.positions[i];
            masses[p as usize] += reference.particle_masses_kg()[i];
        }
        let sr = edges.len()..edges.len() + local.edges.len();
        let br = bends.len()..bends.len() + local.bend_edges.len();
        edges.extend(
            local
                .edges
                .iter()
                .map(|e| [particles[e[0] as usize], particles[e[1] as usize]]),
        );
        bends.extend(
            local
                .bend_edges
                .iter()
                .map(|e| [particles[e[0] as usize], particles[e[1] as usize]]),
        );
        for &(i, s) in &local.edge_softness {
            if (i as usize) < local.edges.len() {
                softness.push((false, (sr.start + i as usize) as u32, s));
            } else {
                softness.push((true, (br.start + i as usize - local.edges.len()) as u32, s));
            }
        }
        tension.extend(
            local
                .tension_only_edges
                .iter()
                .map(|&i| sr.start as u32 + i),
        );
        nominal_mass += reference.nominal_mass_kg();
        span_map.insert(
            span.name.clone(),
            SampledSpan {
                reference,
                particles,
                structural: sr,
                bending: br,
            },
        );
        if native.is_none() {
            native = Some(local);
            first_spec = Some(rope);
        }
    }
    let first = first_spec.expect("nonempty tree");
    let native_masses = masses
        .iter()
        .enumerate()
        .map(|(i, &m)| {
            let n = convert(&first, &format!("harness.masses[{i}]"), m)?;
            if !m.is_finite() || m <= 0.0 || !(1.0 / n).is_finite() {
                return Err(invalid(
                    spec,
                    "particle_masses",
                    "mass arithmetic overflows or underflows",
                ));
            }
            Ok(n)
        })
        .collect::<Result<Vec<_>, RopeError>>()?;
    let sum: Real = native_masses.iter().sum();
    let weighted = native_positions
        .iter()
        .zip(&native_masses)
        .fold(Vector::ZERO, |a, (&p, &m)| a + p * m);
    if !nominal_mass.is_finite()
        || !sum.is_finite()
        || !weighted.is_finite()
        || !(weighted / sum).is_finite()
    {
        return Err(invalid(
            spec,
            "rest_center_of_mass",
            "combined mass arithmetic overflows",
        ));
    }
    let mut native = native.expect("nonempty tree");
    native.positions = native_positions;
    native.masses = native_masses;
    native.edges = edges.clone();
    native.bend_edges = bends.clone();
    // Keep the computational surface empty: native wire contacts are polylines.
    native = native.wire(edges.clone());
    native.edge_softness = softness
        .into_iter()
        .map(|(bend, i, s)| (i + if bend { edges.len() as u32 } else { 0 }, s))
        .collect();
    native.tension_only_edges = tension;
    Ok(HarnessBuild {
        specification: spec.clone(),
        samples: SampledHarness {
            name: spec.name.clone(),
            positions,
            masses,
            edges,
            bends,
            junctions,
            spans: span_map,
            nominal_mass,
        },
        native,
    })
}

impl HarnessHandle {
    fn inner(self) -> RopeHandle {
        RopeHandle {
            set: self.set,
            slot: self.slot,
            generation: self.generation,
        }
    }
    fn from_inner(h: RopeHandle) -> Self {
        Self {
            set: h.set,
            slot: h.slot,
            generation: h.generation,
        }
    }
}
/// Same validation kinds as the rope registry, with a distinct harness identity.
#[derive(Clone, Debug, PartialEq)]
pub struct HarnessError {
    pub phase: RegistryPhase,
    pub harness: Option<HarnessHandle>,
    pub kind: RopeSetErrorKind,
    pub applied_commands: usize,
}
impl From<RopeSetError> for HarnessError {
    fn from(e: RopeSetError) -> Self {
        Self {
            phase: e.phase,
            harness: e.rope.map(HarnessHandle::from_inner),
            kind: e.kind,
            applied_commands: e.applied_commands,
        }
    }
}
impl fmt::Display for HarnessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "harness registry {:?}, {:?}: {:?} ({} commands applied)",
            self.phase, self.harness, self.kind, self.applied_commands
        )
    }
}
impl std::error::Error for HarnessError {}

#[derive(Clone, Debug, PartialEq)]
pub enum HarnessCommand {
    Pin {
        harness: HarnessHandle,
        location: HarnessLocation,
        position_m: Point3,
    },
    MovePin {
        harness: HarnessHandle,
        location: HarnessLocation,
        target_m: Point3,
    },
    Unpin {
        harness: HarnessHandle,
        location: HarnessLocation,
    },
    Attach {
        harness: HarnessHandle,
        location: HarnessLocation,
        body: RigidBodyHandle,
    },
    Detach {
        attachment: AttachmentHandle,
    },
}
#[derive(Clone, Debug)]
pub struct HarnessAttachment {
    pub command_index: usize,
    pub handle: AttachmentHandle,
    pub harness: HarnessHandle,
    pub particle: u32,
    pub body: RigidBodyHandle,
    pub local_anchor_m: Point3,
}
#[derive(Clone, Debug)]
pub struct HarnessSelection {
    pub command_index: usize,
    pub harness: HarnessHandle,
    pub location: ResolvedHarnessLocation,
}
#[derive(Clone, Debug)]
pub struct HarnessPreparedStep {
    pub step: u64,
    pub dt_s: f64,
    pub applied_commands: usize,
    pub created_attachments: Vec<HarnessAttachment>,
    pub locations: Vec<HarnessSelection>,
}
#[derive(Clone, Debug)]
pub struct HarnessStepObservation {
    pub handle: HarnessHandle,
    pub native_handle: SoftBodyHandle,
    pub particle_count: usize,
    pub topology_version: u32,
    pub attachment_count: usize,
}
#[derive(Clone, Debug)]
pub struct HarnessStepReport {
    pub step: u64,
    pub dt_s: f64,
    pub applied_commands: usize,
    pub harnesses: Vec<HarnessStepObservation>,
}
#[derive(Clone, Debug)]
pub struct RemovedHarness {
    pub native_removed: bool,
    pub specification: HarnessSpec,
    pub samples: SampledHarness,
}
pub struct HarnessView<'a> {
    pub handle: HarnessHandle,
    pub native_handle: SoftBodyHandle,
    pub specification: &'a HarnessSpec,
    pub samples: &'a SampledHarness,
    pub soft_body: &'a SoftBody,
    pub(crate) world: &'a PhysicsWorld,
    pub(crate) next_step: u64,
    pub(crate) prepared: Option<u64>,
}
#[derive(Clone, Debug)]
pub struct HarnessAttachmentView {
    pub handle: AttachmentHandle,
    pub harness: HarnessHandle,
    pub particle: u32,
    pub body: RigidBodyHandle,
    pub local_anchor_m: Point3,
}

/// Owns checked graph metadata and the existing native lifecycle machinery.
/// It never steps the caller's world. Remove entire harnesses before Drop.
#[derive(Debug)]
pub struct HarnessSet {
    registry: RopeSet,
    entries: BTreeMap<HarnessHandle, (HarnessSpec, SampledHarness)>,
}
impl HarnessSet {
    pub fn new(id: WorldId) -> Result<Self, HarnessError> {
        Ok(Self {
            registry: RopeSet::new(id)?,
            entries: BTreeMap::new(),
        })
    }
    pub fn id(&self) -> RopeSetId {
        self.registry.id()
    }
    pub fn world_id(&self) -> WorldId {
        self.registry.world_id()
    }
    pub fn len(&self) -> usize {
        self.entries.len()
    }
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
    pub fn handles(&self) -> impl Iterator<Item = HarnessHandle> + '_ {
        self.entries.keys().copied()
    }
    pub fn next_step(&self) -> u64 {
        self.registry.next_step()
    }
    pub fn attachment_count(&self) -> usize {
        self.registry.attachment_count()
    }
    pub fn insert(
        &mut self,
        id: WorldId,
        world: &mut PhysicsWorld,
        spec: &HarnessSpec,
    ) -> Result<HarnessHandle, HarnessError> {
        let (builder, samples, specification) = build_harness(spec)
            .map_err(|e| HarnessError {
                phase: RegistryPhase::Access,
                harness: None,
                kind: RopeSetErrorKind::Definition(Box::new(e)),
                applied_commands: 0,
            })?
            .into_parts();
        let h = HarnessHandle::from_inner(self.registry.insert_graph(
            id,
            world,
            spec.name.clone(),
            builder,
        )?);
        self.entries.insert(h, (specification, samples));
        Ok(h)
    }
    pub fn get<'a>(
        &'a self,
        id: WorldId,
        world: &'a PhysicsWorld,
        h: HarnessHandle,
    ) -> Result<HarnessView<'a>, HarnessError> {
        let (native_handle, soft_body) = self.registry.get_graph(id, world, h.inner())?;
        let (specification, samples) = self.entries.get(&h).expect("validated harness metadata");
        Ok(HarnessView {
            handle: h,
            native_handle,
            specification,
            samples,
            soft_body,
            world,
            next_step: self.next_step(),
            prepared: self.registry.pending_step(),
        })
    }
    pub fn get_by_name<'a>(
        &'a self,
        id: WorldId,
        world: &'a PhysicsWorld,
        name: &str,
    ) -> Result<Option<HarnessView<'a>>, HarnessError> {
        // Validate world even when the name is missing.
        if id != self.world_id() {
            return Err(HarnessError {
                phase: RegistryPhase::Access,
                harness: None,
                kind: RopeSetErrorKind::WorldMismatch {
                    expected: self.world_id(),
                    actual: id,
                },
                applied_commands: 0,
            });
        }
        self.entries
            .iter()
            .find(|(_, e)| e.0.name == name)
            .map(|(&h, _)| self.get(id, world, h))
            .transpose()
    }
    pub fn remove(
        &mut self,
        id: WorldId,
        world: &mut PhysicsWorld,
        h: HarnessHandle,
    ) -> Result<RemovedHarness, HarnessError> {
        let native_removed = self.registry.remove_graph(id, world, h.inner())?;
        let (specification, samples) = self.entries.remove(&h).expect("validated harness metadata");
        Ok(RemovedHarness {
            native_removed,
            specification,
            samples,
        })
    }
    pub fn get_attachment(
        &self,
        id: WorldId,
        world: &PhysicsWorld,
        h: AttachmentHandle,
    ) -> Result<HarnessAttachmentView, HarnessError> {
        let a = self.registry.get_attachment(id, world, h)?;
        Ok(HarnessAttachmentView {
            handle: a.handle,
            harness: HarnessHandle::from_inner(a.rope),
            particle: a.particle,
            body: a.body,
            local_anchor_m: a.local_anchor_m,
        })
    }
    /// Resolve all locations before the same atomic validate/prepare protocol used
    /// for ropes. Two span endpoint aliases at a junction target one particle.
    pub fn prepare(
        &mut self,
        id: WorldId,
        world: &mut PhysicsWorld,
        step: u64,
        h: f64,
        commands: &[HarnessCommand],
    ) -> Result<HarnessPreparedStep, HarnessError> {
        let mut native = Vec::new();
        let mut locations = Vec::new();
        for (i, c) in commands.iter().enumerate() {
            let (handle, location) = match c {
                HarnessCommand::Pin {
                    harness, location, ..
                }
                | HarnessCommand::MovePin {
                    harness, location, ..
                }
                | HarnessCommand::Unpin { harness, location }
                | HarnessCommand::Attach {
                    harness, location, ..
                } => (*harness, location),
                HarnessCommand::Detach { attachment } => {
                    native.push(RopeCommand::Detach {
                        attachment: *attachment,
                    });
                    continue;
                }
            };
            let view = self.get(id, world, handle).map_err(|mut e| {
                e.phase = RegistryPhase::Prepare;
                e
            })?;
            let location = view
                .samples
                .resolve_location(location)
                .map_err(|e| HarnessError {
                    phase: RegistryPhase::Prepare,
                    harness: Some(handle),
                    kind: RopeSetErrorKind::Location(Box::new(e)),
                    applied_commands: 0,
                })?;
            let particle = location.particle_index;
            let rope = handle.inner();
            native.push(match *c {
                HarnessCommand::Pin { position_m, .. } => RopeCommand::PinAt {
                    rope,
                    particle,
                    position_m,
                },
                HarnessCommand::MovePin { target_m, .. } => RopeCommand::MovePin {
                    rope,
                    particle,
                    target_m,
                },
                HarnessCommand::Unpin { .. } => RopeCommand::Unpin { rope, particle },
                HarnessCommand::Attach { body, .. } => RopeCommand::Attach {
                    rope,
                    particle,
                    body,
                },
                HarnessCommand::Detach { .. } => unreachable!(),
            });
            locations.push(HarnessSelection {
                command_index: i,
                harness: handle,
                location,
            });
        }
        let p = self.registry.prepare(id, world, step, h, &native)?;
        Ok(HarnessPreparedStep {
            step: p.step,
            dt_s: p.dt_s,
            applied_commands: p.applied_commands,
            locations,
            created_attachments: p
                .created_attachments
                .into_iter()
                .map(|a| HarnessAttachment {
                    command_index: a.command_index,
                    handle: a.handle,
                    harness: HarnessHandle::from_inner(a.rope),
                    particle: a.particle,
                    body: a.body,
                    local_anchor_m: a.local_anchor_m,
                })
                .collect(),
        })
    }
    pub fn inspect(
        &mut self,
        id: WorldId,
        world: &PhysicsWorld,
        step: u64,
    ) -> Result<HarnessStepReport, HarnessError> {
        let r = self.registry.inspect(id, world, step)?;
        Ok(HarnessStepReport {
            step: r.step,
            dt_s: r.dt_s,
            applied_commands: r.applied_commands,
            harnesses: r
                .ropes
                .into_iter()
                .map(|h| HarnessStepObservation {
                    handle: HarnessHandle::from_inner(h.handle),
                    native_handle: h.native_handle,
                    particle_count: h.particle_count,
                    topology_version: h.topology_version,
                    attachment_count: h.attachment_count,
                })
                .collect(),
        })
    }
}
