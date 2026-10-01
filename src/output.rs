//! Renderer-independent checked views, owned snapshots and experimental playback JSON.
//! Tracks are not checkpoints and contain no complete solver restart state.
//!
//! ```
//! use rapier_rope::{rapier::prelude::*, *};
//! let id = WorldId(1);
//! let mut world = PhysicsWorld::new();
//! world.integration_parameters.dt = 1.0 / 240.0;
//! let mut ropes = RopeSet::new(id)?;
//! let spec = RopeSpec::new("cable", vec![[0.0, 1.5, 0.0], [1.0, 1.5, 0.0]],
//!     NativeRopeMaterial::new(0.1, SpringSettings::new(500.0, 1.0),
//!         SpringSettings::new(20.0, 0.8)),
//!     SamplingSettings::new(1.0 / 32.0), CollisionSettings::new(0.005));
//! let rope = ropes.insert(id, &mut world, &spec)?;
//! let view = ropes.centerline(id, &world, rope)?;
//! let mut track = RopeTrack::new("example", "right-handed, Y-up, SI", &world)?;
//! track.register_rope(&view)?;
//! let capture = CaptureStamp::new(CapturePhase::Initial, None, 0.0, 1.0 / 240.0)?;
//! track.push_frame(TrackFrame { capture: capture.clone(),
//!     ropes: vec![view.snapshot(capture)?], bodies: vec![] })?;
//! let mut json = Vec::new();
//! track.write_json(&mut json)?;
//! let playback = RopeTrack::read_json(json.as_slice())?;
//! assert_eq!(playback.frames()[0].ropes[0].name, "cable");
//! # Ok::<(), OutputError>(())
//! ```
use crate::{
    AttachmentHandle, Point3, ResolvedLocation, RopeHandle, RopeSet, RopeSetError, RopeSpec,
    RopeView, WorldId, diagnostics::*, rapier::prelude::*, sample_rope,
};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    fmt,
    io::{Read, Write},
};

pub const TRACK_SCHEMA_VERSION: u32 = 1;
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CapturePhase {
    Initial,
    BeforeStep,
    AfterStep,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CaptureStamp {
    pub phase: CapturePhase,
    /// Zero-based caller step index; absent only for the initial, unstepped state.
    pub step: Option<u64>,
    pub time_s: FiniteScalar,
    pub outer_dt_s: FiniteScalar,
}
impl CaptureStamp {
    #[allow(clippy::unnecessary_cast)]
    pub fn new(
        phase: CapturePhase,
        step: Option<u64>,
        time_s: f64,
        dt_s: f64,
    ) -> Result<Self, OutputError> {
        let stamp = Self {
            phase,
            step,
            time_s: FiniteScalar::new(time_s)?,
            outer_dt_s: FiniteScalar::new(scalar(dt_s as Real))?,
        };
        stamp.validate()?;
        Ok(stamp)
    }
    pub(crate) fn validate(&self) -> Result<(), OutputError> {
        if (self.phase == CapturePhase::Initial) != self.step.is_none()
            || self.time_s.value() < 0.0
            || self.outer_dt_s.value() <= 0.0
        {
            return Err(OutputError::Invalid("invalid capture phase/step/time/dt"));
        }
        Ok(())
    }
    pub fn last_completed_step(&self) -> Option<u64> {
        match self.phase {
            CapturePhase::Initial => None,
            CapturePhase::BeforeStep => self.step.and_then(|s| s.checked_sub(1)),
            CapturePhase::AfterStep => self.step,
        }
    }
}

#[derive(Debug)]
pub enum OutputError {
    Registry(Box<RopeSetError>),
    Diagnostic(DiagnosticError),
    Invalid(&'static str),
    Io(std::io::Error),
    Json(serde_json::Error),
}
impl fmt::Display for OutputError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "rope output: {self:?}")
    }
}
impl std::error::Error for OutputError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Registry(e) => Some(e.as_ref()),
            Self::Diagnostic(e) => Some(e),
            Self::Io(e) => Some(e),
            Self::Json(e) => Some(e),
            Self::Invalid(_) => None,
        }
    }
}
impl From<RopeSetError> for OutputError {
    fn from(e: RopeSetError) -> Self {
        Self::Registry(Box::new(e))
    }
}
impl From<DiagnosticError> for OutputError {
    fn from(e: DiagnosticError) -> Self {
        Self::Diagnostic(e)
    }
}
impl From<std::io::Error> for OutputError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}
impl From<serde_json::Error> for OutputError {
    fn from(e: serde_json::Error) -> Self {
        Self::Json(e)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecordedId {
    pub set: u64,
    pub slot: u32,
    pub generation: u64,
}
impl From<RopeHandle> for RecordedId {
    fn from(h: RopeHandle) -> Self {
        Self {
            set: h.set_id().value(),
            slot: h.slot(),
            generation: h.generation(),
        }
    }
}
impl From<AttachmentHandle> for RecordedId {
    fn from(h: AttachmentHandle) -> Self {
        Self {
            set: h.set_id().value(),
            slot: h.slot(),
            generation: h.generation(),
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecordedBodyId {
    pub index: u32,
    pub generation: u32,
}
impl From<RigidBodyHandle> for RecordedBodyId {
    fn from(h: RigidBodyHandle) -> Self {
        let (index, generation) = h.into_raw_parts();
        Self { index, generation }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NativeImpulseMetadata {
    pub unit: String,
    pub interval: String,
    /// Caller-declared capture context. This does not prove world.step() was called.
    pub capture: CaptureStamp,
    pub last_completed_step: Option<u64>,
    pub internal_substep_dt_s: Measurement,
    pub edge_direction: String,
    pub attachment_direction: String,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EdgeFamily {
    Structural,
    Bending,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EdgeImpulseObservation {
    pub native_index: u32,
    pub vertices: [u32; 2],
    pub family: EdgeFamily,
    pub impulse_ns: Measurement,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AttachmentObservation {
    pub particle: u32,
    pub body: RecordedBodyId,
    pub local_anchor_m: [FiniteScalar; 3],
    pub world_anchor_m: [FiniteScalar; 3],
    pub position_error_m: Measurement,
    pub native_impulse_ns: Observation<[FiniteScalar; 3]>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StateFiniteness {
    pub particle_positions: bool,
    pub particle_velocities: bool,
    pub native_impulses: bool,
    pub attachment_poses: bool,
    pub quarantine_empty: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RopeDiagnostics {
    pub geometry: GeometryDiagnostics,
    pub state_finiteness: StateFiniteness,
    pub impulse_metadata: NativeImpulseMetadata,
    pub edge_impulses: Vec<EdgeImpulseObservation>,
    pub attachments: Vec<AttachmentObservation>,
    pub unprovided: UnprovidedObservations,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RopeSnapshot {
    pub identity: RecordedId,
    pub name: String,
    pub capture: CaptureStamp,
    #[serde(serialize_with = "serialize_points")]
    pub positions_m: Vec<Point3>,
    #[serde(serialize_with = "serialize_points")]
    pub velocities_m_s: Vec<Point3>,
    pub segments: Vec<[u32; 2]>,
    pub radius_m: FiniteScalar,
    pub pinned: Vec<bool>,
    #[serde(serialize_with = "serialize_locations")]
    pub named_locations: BTreeMap<String, ResolvedLocation>,
    pub capabilities: RopeCapabilities,
    pub diagnostics: RopeDiagnostics,
}
pub(crate) fn serialize_points<S: serde::Serializer>(
    points: &[Point3],
    serializer: S,
) -> Result<S::Ok, S::Error> {
    if !points.iter().flatten().copied().all(f64::is_finite) {
        return Err(serde::ser::Error::custom(
            "non-finite point; use an observation status instead",
        ));
    }
    points.serialize(serializer)
}
fn serialize_locations<S: serde::Serializer>(
    locations: &BTreeMap<String, ResolvedLocation>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    if locations.values().any(|l| {
        ![l.requested_arc_length_m, l.actual_arc_length_m, l.error_m]
            .into_iter()
            .all(f64::is_finite)
    }) {
        return Err(serde::ser::Error::custom("non-finite material location"));
    }
    locations.serialize(serializer)
}
#[allow(clippy::unnecessary_cast)]
fn scalar(x: Real) -> f64 {
    x as f64
}
pub(crate) fn xyz(x: Vector) -> Point3 {
    [scalar(x.x), scalar(x.y), scalar(x.z)]
}
pub(crate) fn finite_point(x: Vector) -> Result<[FiniteScalar; 3], DiagnosticError> {
    Ok([
        FiniteScalar::new(scalar(x.x))?,
        FiniteScalar::new(scalar(x.y))?,
        FiniteScalar::new(scalar(x.z))?,
    ])
}

/// A checked borrow; positions/velocities are read from the caller's live world.
pub struct CenterlineView<'a> {
    rope: RopeView<'a>,
    world: &'a PhysicsWorld,
    next_step: u64,
    prepared: Option<u64>,
}
impl<'a> CenterlineView<'a> {
    pub fn name(&self) -> &str {
        &self.rope.specification.name
    }
    pub fn specification(&self) -> &RopeSpec {
        self.rope.specification
    }
    pub fn identity(&self) -> RecordedId {
        self.rope.handle.into()
    }
    pub fn positions_m(&self) -> impl ExactSizeIterator<Item = Point3> + '_ {
        self.rope.soft_body.particle_positions().map(xyz)
    }
    pub fn velocities_m_s(&self) -> impl ExactSizeIterator<Item = Point3> + '_ {
        self.rope.soft_body.particle_velocities().map(xyz)
    }
    pub fn segments(&self) -> &[[u32; 2]] {
        self.rope.samples.wire_segments()
    }
    pub fn radius_m(&self) -> f64 {
        scalar(self.rope.soft_body.particle_radius())
    }
    pub fn named_locations(&self) -> &BTreeMap<String, ResolvedLocation> {
        self.rope.samples.named_locations()
    }
    pub fn snapshot(&self, capture: CaptureStamp) -> Result<RopeSnapshot, OutputError> {
        capture.validate()?;
        let correct_phase = match capture.phase {
            CapturePhase::Initial => self.next_step == 0,
            CapturePhase::BeforeStep => capture.step == self.prepared && self.prepared.is_some(),
            CapturePhase::AfterStep => {
                self.prepared.is_none()
                    && capture.step.and_then(|s| s.checked_add(1)) == Some(self.next_step)
            }
        };
        if !correct_phase {
            return Err(OutputError::Invalid(
                "capture phase/step does not match registry protocol",
            ));
        }
        #[allow(clippy::unnecessary_cast)]
        let native_dt = capture.outer_dt_s.value() as Real;
        if native_dt != self.world.integration_parameters.dt {
            return Err(OutputError::Invalid("capture dt differs from world dt"));
        }
        let positions: Vec<_> = self.positions_m().collect();
        let geometry = diagnose_geometry(self.rope.samples, &positions)?;
        // Before-step commands can create/reset attachments. Their zero impulse
        // must not masquerade as an observation from the previous native substep.
        let observed = capture.phase == CapturePhase::AfterStep;
        let edge_impulses = self
            .rope
            .soft_body
            .edges()
            .iter()
            .enumerate()
            .map(|(i, e)| EdgeImpulseObservation {
                native_index: i as u32,
                vertices: e.vertices,
                family: if i < self.segments().len() {
                    EdgeFamily::Structural
                } else {
                    EdgeFamily::Bending
                },
                impulse_ns: if observed {
                    Measurement::from_number(scalar(e.impulse()))
                } else {
                    Observation::Unavailable {
                        reason: "impulses are provided only after an inspected caller step; commands may reset them".into(),
                    }
                },
            })
            .collect();
        let attachments = self
            .rope
            .soft_body
            .particle_attachments()
            .iter()
            .map(|a| {
                let body = &self.world.bodies[a.body];
                let anchor = body.position().transform_point(a.local_anchor);
                Ok(AttachmentObservation {
                    particle: a.particle,
                    body: a.body.into(),
                    local_anchor_m: finite_point(a.local_anchor)?,
                    world_anchor_m: finite_point(anchor)?,
                    position_error_m: Measurement::from_number(scalar(
                        (anchor - self.rope.soft_body.particle_position(a.particle as usize))
                            .length(),
                    )),
                    native_impulse_ns: if observed {
                        Observation::Available(finite_point(a.impulse())?)
                    } else {
                        Observation::Unavailable {
                            reason: "impulses are provided only after an inspected caller step; commands may reset them".into(),
                        }
                    },
                })
            })
            .collect::<Result<Vec<_>, DiagnosticError>>()?;
        Ok(RopeSnapshot { identity: self.identity(), name: self.name().into(), capture: capture.clone(), positions_m: positions,
            velocities_m_s: self.velocities_m_s().collect(), segments: self.segments().to_vec(), radius_m: FiniteScalar::new(self.radius_m())?,
            pinned: self.rope.soft_body.particles().iter().map(|p|p.is_pinned()).collect(), named_locations: self.named_locations().clone(), capabilities: RopeCapabilities::default(),
            diagnostics: RopeDiagnostics { geometry, state_finiteness: StateFiniteness { particle_positions:true,particle_velocities:true,native_impulses:true,attachment_poses:true,quarantine_empty:true },
                impulse_metadata: NativeImpulseMetadata { unit:"N s".into(), interval:"native last internal substep, not outer-step sum".into(), last_completed_step:capture.last_completed_step(), capture,
                    internal_substep_dt_s: Observation::Unavailable { reason:"not independently exposed for this coupled world".into() },
                    edge_direction:"0.36 distance edges: positive scalar pulls vertex[0] toward vertex[1] and vertex[1] toward vertex[0] along the solver's last-substep gradient; current geometry is not that saved gradient".into(),
                    attachment_direction:"0.36 native world-space vector: + on rigid body, - on particle".into() },
                edge_impulses, attachments, unprovided: UnprovidedObservations::default() } })
    }
}
impl RopeSet {
    pub fn centerline<'a>(
        &'a self,
        id: WorldId,
        world: &'a PhysicsWorld,
        rope: RopeHandle,
    ) -> Result<CenterlineView<'a>, RopeSetError> {
        Ok(CenterlineView {
            rope: self.get(id, world, rope)?,
            world,
            next_step: self.next_step(),
            prepared: self.pending_step(),
        })
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WorldRecordingSettings {
    pub outer_dt_s: FiniteScalar,
    pub gravity_m_s2: [FiniteScalar; 3],
    pub num_solver_iterations: usize,
    pub num_internal_pgs_iterations: usize,
    pub integration_parameters_debug: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecordedRopeDefinition {
    pub identity: RecordedId,
    pub definition: RopeSpec,
    pub reference_arc_lengths_m: Vec<FiniteScalar>,
    pub particle_masses_kg: Vec<FiniteScalar>,
    pub segments: Vec<[u32; 2]>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum DisplayShape {
    Sphere { radius_m: FiniteScalar },
    Cuboid { half_extents_m: [FiniteScalar; 3] },
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DisplayObject {
    pub name: String,
    /// e.g. collider, marker or freefall_control; never a material frame.
    pub role: String,
    pub body: Option<RecordedBodyId>,
    pub shape: DisplayShape,
    pub translation_m: [FiniteScalar; 3],
    pub rotation_xyzw: [FiniteScalar; 4],
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BodyPoseSnapshot {
    pub identity: RecordedBodyId,
    pub translation_m: [FiniteScalar; 3],
    pub rotation_xyzw: [FiniteScalar; 4],
}
impl BodyPoseSnapshot {
    pub fn capture(world: &PhysicsWorld, body: RigidBodyHandle) -> Result<Self, OutputError> {
        let rb = world
            .bodies
            .get(body)
            .ok_or(OutputError::Invalid("missing display body"))?;
        let q = rb.rotation();
        Ok(Self {
            identity: body.into(),
            translation_m: finite_point(rb.translation())?,
            rotation_xyzw: [
                FiniteScalar::new(scalar(q.x))?,
                FiniteScalar::new(scalar(q.y))?,
                FiniteScalar::new(scalar(q.z))?,
                FiniteScalar::new(scalar(q.w))?,
            ],
        })
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TrackFrame {
    pub capture: CaptureStamp,
    pub ropes: Vec<RopeSnapshot>,
    pub bodies: Vec<BodyPoseSnapshot>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum TrackEventKind {
    Pin {
        rope: RecordedId,
        particle: u32,
    },
    Unpin {
        rope: RecordedId,
        particle: u32,
    },
    Attach {
        rope: RecordedId,
        particle: u32,
        attachment: RecordedId,
        body: RecordedBodyId,
        local_anchor_m: [FiniteScalar; 3],
    },
    Detach {
        rope: RecordedId,
        particle: u32,
        attachment: RecordedId,
        body: RecordedBodyId,
    },
    CaptureRejected {
        reason: String,
    },
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TrackEvent {
    pub step: u64,
    pub time_s: FiniteScalar,
    pub operation: TrackEventKind,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct TrackData {
    schema_version: u32,
    purpose: String,
    scene: String,
    coordinate_system: String,
    units: BTreeMap<String, String>,
    precision: String,
    rapier_version: String,
    package_version: String,
    world_settings: WorldRecordingSettings,
    ropes: Vec<RecordedRopeDefinition>,
    display_objects: Vec<DisplayObject>,
    events: Vec<TrackEvent>,
    frames: Vec<TrackFrame>,
}

/// Experimental playback format. Accessors borrow immutable data; validation
/// runs before insertion, writing and reading. No complete solver state is stored.
pub struct RopeTrack {
    data: TrackData,
}
impl RopeTrack {
    pub fn new(
        scene: &str,
        coordinate_system: &str,
        world: &PhysicsWorld,
    ) -> Result<Self, OutputError> {
        let data = TrackData {
            schema_version: TRACK_SCHEMA_VERSION,
            purpose: "playback_only_not_solver_checkpoint".into(),
            scene: scene.into(),
            coordinate_system: coordinate_system.into(),
            units: BTreeMap::from([
                ("length".into(), "m".into()),
                ("time".into(), "s".into()),
                ("mass".into(), "kg".into()),
                ("velocity".into(), "m/s".into()),
                ("curvature".into(), "1/m".into()),
                ("impulse".into(), "N s".into()),
            ]),
            precision: if cfg!(feature = "f32") { "f32" } else { "f64" }.into(),
            rapier_version: "0.36.0".into(),
            package_version: env!("CARGO_PKG_VERSION").into(),
            world_settings: WorldRecordingSettings {
                outer_dt_s: FiniteScalar::new(scalar(world.integration_parameters.dt))?,
                gravity_m_s2: finite_point(world.gravity)?,
                num_solver_iterations: world.integration_parameters.num_solver_iterations,
                num_internal_pgs_iterations: world
                    .integration_parameters
                    .num_internal_pgs_iterations,
                integration_parameters_debug: format!("{:#?}", world.integration_parameters),
            },
            ropes: vec![],
            display_objects: vec![],
            events: vec![],
            frames: vec![],
        };
        let track = Self { data };
        track.validate()?;
        Ok(track)
    }
    pub fn register_rope(&mut self, view: &CenterlineView<'_>) -> Result<(), OutputError> {
        if !self.data.frames.is_empty()
            || self
                .data
                .ropes
                .iter()
                .any(|r| r.identity == view.identity())
        {
            return Err(OutputError::Invalid("duplicate or late rope registration"));
        }
        self.data.ropes.push(RecordedRopeDefinition {
            identity: view.identity(),
            definition: view.specification().clone(),
            reference_arc_lengths_m: view
                .rope
                .samples
                .arc_lengths_m()
                .iter()
                .map(|&x| FiniteScalar::new(x).expect("validated reference sample"))
                .collect(),
            particle_masses_kg: view
                .rope
                .samples
                .particle_masses_kg()
                .iter()
                .map(|&x| FiniteScalar::new(x).expect("validated reference sample"))
                .collect(),
            segments: view.segments().to_vec(),
        });
        Ok(())
    }
    pub fn add_display_object(&mut self, object: DisplayObject) -> Result<(), OutputError> {
        validate_object(&object)?;
        if !self.data.frames.is_empty() {
            return Err(OutputError::Invalid("late display object registration"));
        }
        self.data.display_objects.push(object);
        Ok(())
    }
    pub fn record_event(&mut self, event: TrackEvent) -> Result<(), OutputError> {
        self.validate_event(&event)?;
        if self
            .data
            .events
            .last()
            .is_some_and(|old| event.time_s.value() < old.time_s.value() || event.step < old.step)
        {
            return Err(OutputError::Invalid("event time/step went backwards"));
        }
        self.data.events.push(event);
        Ok(())
    }
    pub fn push_frame(&mut self, frame: TrackFrame) -> Result<(), OutputError> {
        self.validate_frame(&frame)?;
        if let Some(old) = self.data.frames.last() {
            validate_frame_order(old, &frame)?;
        }
        self.data.frames.push(frame);
        Ok(())
    }
    pub fn frames(&self) -> &[TrackFrame] {
        &self.data.frames
    }
    pub fn events(&self) -> &[TrackEvent] {
        &self.data.events
    }
    pub fn definitions(&self) -> &[RecordedRopeDefinition] {
        &self.data.ropes
    }
    pub fn scene(&self) -> &str {
        &self.data.scene
    }
    pub fn precision(&self) -> &str {
        &self.data.precision
    }
    pub fn coordinate_system(&self) -> &str {
        &self.data.coordinate_system
    }
    pub fn world_settings(&self) -> &WorldRecordingSettings {
        &self.data.world_settings
    }
    pub fn display_objects(&self) -> &[DisplayObject] {
        &self.data.display_objects
    }
    pub fn units(&self) -> &BTreeMap<String, String> {
        &self.data.units
    }
    pub fn schema_version(&self) -> u32 {
        self.data.schema_version
    }
    pub fn rapier_version(&self) -> &str {
        &self.data.rapier_version
    }
    pub fn package_version(&self) -> &str {
        &self.data.package_version
    }
    pub fn write_json(&self, writer: impl Write) -> Result<(), OutputError> {
        self.validate()?;
        serde_json::to_writer_pretty(writer, &self.data)?;
        Ok(())
    }
    pub fn read_json(reader: impl Read) -> Result<Self, OutputError> {
        let data = serde_json::from_reader(reader)?;
        let track = Self { data };
        track.validate()?;
        Ok(track)
    }
    fn validate_event(&self, event: &TrackEvent) -> Result<(), OutputError> {
        if event.time_s.value() < 0.0 {
            return Err(OutputError::Invalid("negative event time"));
        }
        let selected = match event.operation {
            TrackEventKind::Pin { rope, particle }
            | TrackEventKind::Unpin { rope, particle }
            | TrackEventKind::Attach { rope, particle, .. }
            | TrackEventKind::Detach { rope, particle, .. } => Some((rope, particle)),
            TrackEventKind::CaptureRejected { .. } => None,
        };
        if let Some((rope, particle)) = selected {
            let r = self
                .data
                .ropes
                .iter()
                .find(|r| r.identity == rope)
                .ok_or(OutputError::Invalid("unknown event rope"))?;
            if particle as usize >= r.reference_arc_lengths_m.len() {
                return Err(OutputError::Invalid("invalid event sample"));
            }
        }
        Ok(())
    }
    fn validate_frame(&self, frame: &TrackFrame) -> Result<(), OutputError> {
        frame.capture.validate()?;
        if frame.ropes.len() != self.data.ropes.len() {
            return Err(OutputError::Invalid("frame rope count changed"));
        }
        for (snapshot, reference) in frame.ropes.iter().zip(&self.data.ropes) {
            let count = reference.reference_arc_lengths_m.len();
            if snapshot.identity != reference.identity
                || snapshot.name != reference.definition.name
                || snapshot.capture != frame.capture
                || snapshot.positions_m.len() != count
                || snapshot.velocities_m_s.len() != count
                || snapshot.pinned.len() != count
                || snapshot.segments != reference.segments
                || snapshot.radius_m.value() <= 0.0
                || !snapshot
                    .positions_m
                    .iter()
                    .chain(&snapshot.velocities_m_s)
                    .flatten()
                    .copied()
                    .all(f64::is_finite)
            {
                return Err(OutputError::Invalid("inconsistent or non-finite snapshot"));
            }
            let samples = sample_rope(&reference.definition)
                .map_err(|_| OutputError::Invalid("invalid rope definition"))?;
            if snapshot.named_locations != *samples.named_locations()
                || snapshot.diagnostics.geometry.segments.len() != reference.segments.len()
                || snapshot.diagnostics.geometry.curvature.len() != count
                || snapshot.diagnostics.impulse_metadata.capture != frame.capture
                || snapshot.diagnostics.impulse_metadata.last_completed_step
                    != frame.capture.last_completed_step()
                || snapshot
                    .diagnostics
                    .attachments
                    .iter()
                    .any(|a| a.particle as usize >= count)
                || snapshot.capabilities != RopeCapabilities::default()
                || snapshot.diagnostics.unprovided != UnprovidedObservations::default()
                || snapshot.diagnostics.impulse_metadata.unit != "N s"
            {
                return Err(OutputError::Invalid("inconsistent snapshot diagnostics"));
            }
            let expected_edges = samples
                .structural_edges()
                .iter()
                .map(|&v| (v, EdgeFamily::Structural))
                .chain(
                    samples
                        .bend_edges()
                        .iter()
                        .map(|&v| (v, EdgeFamily::Bending)),
                );
            if snapshot.diagnostics.edge_impulses.len()
                != samples.structural_edges().len() + samples.bend_edges().len()
                || snapshot
                    .diagnostics
                    .edge_impulses
                    .iter()
                    .zip(expected_edges)
                    .enumerate()
                    .any(|(i, (e, (vertices, family)))| {
                        e.native_index as usize != i || e.vertices != vertices || e.family != family
                    })
                || snapshot
                    .diagnostics
                    .geometry
                    .segments
                    .iter()
                    .zip(&reference.segments)
                    .any(|(e, &v)| e.vertices != v)
                || snapshot
                    .diagnostics
                    .geometry
                    .curvature
                    .iter()
                    .enumerate()
                    .any(|(i, c)| {
                        c.particle_index as usize != i
                            || c.reference_arc_length_m != reference.reference_arc_lengths_m[i]
                    })
            {
                return Err(OutputError::Invalid(
                    "invalid diagnostic sample/edge mapping",
                ));
            }
            if frame.capture.phase != CapturePhase::AfterStep
                && (snapshot
                    .diagnostics
                    .edge_impulses
                    .iter()
                    .any(|e| !matches!(e.impulse_ns, Observation::Unavailable { .. }))
                    || snapshot
                        .diagnostics
                        .attachments
                        .iter()
                        .any(|a| !matches!(a.native_impulse_ns, Observation::Unavailable { .. })))
            {
                return Err(OutputError::Invalid(
                    "pre-step impulses cannot claim a completed substep observation",
                ));
            }
        }
        for (i, body) in frame.bodies.iter().enumerate() {
            validate_rotation(body.rotation_xyzw)?;
            if frame.bodies[..i]
                .iter()
                .any(|old| old.identity == body.identity)
            {
                return Err(OutputError::Invalid("duplicate display body"));
            }
        }
        if self
            .data
            .display_objects
            .iter()
            .filter_map(|o| o.body)
            .any(|id| !frame.bodies.iter().any(|b| b.identity == id))
        {
            return Err(OutputError::Invalid("missing recorded display body pose"));
        }
        Ok(())
    }
    fn validate(&self) -> Result<(), OutputError> {
        if self.data.schema_version != TRACK_SCHEMA_VERSION
            || self.data.purpose != "playback_only_not_solver_checkpoint"
            || self.data.scene.trim().is_empty()
            || self.data.coordinate_system.trim().is_empty()
            || !["f32", "f64"].contains(&self.data.precision.as_str())
            || self.data.rapier_version != "0.36.0"
            || self.data.world_settings.outer_dt_s.value() <= 0.0
            || self.data.units != recording_units()
        {
            return Err(OutputError::Invalid(
                "unsupported or invalid track metadata",
            ));
        }
        for (i, r) in self.data.ropes.iter().enumerate() {
            let samples = sample_rope(&r.definition)
                .map_err(|_| OutputError::Invalid("invalid recorded rope definition"))?;
            if self.data.ropes[..i]
                .iter()
                .any(|old| old.identity == r.identity)
                || r.reference_arc_lengths_m
                    .iter()
                    .map(|x| x.value())
                    .ne(samples.arc_lengths_m().iter().copied())
                || r.particle_masses_kg
                    .iter()
                    .map(|x| x.value())
                    .ne(samples.particle_masses_kg().iter().copied())
                || r.segments != samples.wire_segments()
            {
                return Err(OutputError::Invalid("invalid recorded sampling"));
            }
        }
        for object in &self.data.display_objects {
            validate_object(object)?;
        }
        for event in &self.data.events {
            self.validate_event(event)?;
        }
        for events in self.data.events.windows(2) {
            if events[0].time_s.value() > events[1].time_s.value()
                || events[0].step > events[1].step
            {
                return Err(OutputError::Invalid("unordered events"));
            }
        }
        for frame in &self.data.frames {
            self.validate_frame(frame)?;
        }
        for frames in self.data.frames.windows(2) {
            validate_frame_order(&frames[0], &frames[1])?;
        }
        Ok(())
    }
}
fn recording_units() -> BTreeMap<String, String> {
    BTreeMap::from([
        ("length".into(), "m".into()),
        ("time".into(), "s".into()),
        ("mass".into(), "kg".into()),
        ("velocity".into(), "m/s".into()),
        ("curvature".into(), "1/m".into()),
        ("impulse".into(), "N s".into()),
    ])
}
fn validate_rotation(q: [FiniteScalar; 4]) -> Result<(), OutputError> {
    if (q.iter().map(|x| x.value() * x.value()).sum::<f64>() - 1.0).abs() > 1e-5 {
        return Err(OutputError::Invalid("display rotation is not unit length"));
    }
    Ok(())
}
fn validate_object(object: &DisplayObject) -> Result<(), OutputError> {
    let positive = match object.shape {
        DisplayShape::Sphere { radius_m } => radius_m.value() > 0.0,
        DisplayShape::Cuboid { half_extents_m } => half_extents_m.iter().all(|x| x.value() > 0.0),
    };
    if object.name.trim().is_empty() || !positive {
        return Err(OutputError::Invalid("invalid display object"));
    }
    validate_rotation(object.rotation_xyzw)
}
fn validate_frame_order(old: &TrackFrame, new: &TrackFrame) -> Result<(), OutputError> {
    fn order(c: &CaptureStamp) -> (u128, u8) {
        match c.phase {
            CapturePhase::Initial => (0, 0),
            CapturePhase::BeforeStep => (u128::from(c.step.unwrap()) * 2, 1),
            CapturePhase::AfterStep => (u128::from(c.step.unwrap()) * 2 + 1, 1),
        }
    }
    if new.capture.time_s.value() < old.capture.time_s.value()
        || order(&new.capture) <= order(&old.capture)
    {
        return Err(OutputError::Invalid(
            "frame time/step went backwards or repeated",
        ));
    }
    Ok(())
}
