//! Graph snapshots, with curvature evaluated separately inside each span.
use crate::{harness::*, rapier::prelude::*, *};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

impl From<HarnessHandle> for RecordedId {
    fn from(h: HarnessHandle) -> Self {
        Self {
            set: h.set_id().value(),
            slot: h.slot(),
            generation: h.generation(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HarnessSnapshot {
    pub identity: RecordedId,
    pub name: String,
    pub capture: CaptureStamp,
    #[serde(serialize_with = "crate::output::serialize_points")]
    pub positions_m: Vec<Point3>,
    #[serde(serialize_with = "crate::output::serialize_points")]
    pub velocities_m_s: Vec<Point3>,
    pub segments: Vec<[u32; 2]>,
    pub radius_m: FiniteScalar,
    pub pinned: Vec<bool>,
    pub junction_particles: BTreeMap<String, u32>,
    pub span_particles: BTreeMap<String, Vec<u32>>,
    /// Particle indices and reference arcs in these diagnostics are span-local.
    pub span_geometry: BTreeMap<String, GeometryDiagnostics>,
    /// No cross-span/junction bending constraint or unique junction curvature.
    pub junction_curvature: Measurement,
    pub attachments: Vec<AttachmentObservation>,
    pub edge_impulses: Vec<EdgeImpulseObservation>,
    pub impulse_metadata: NativeImpulseMetadata,
    pub unprovided: UnprovidedObservations,
    pub capabilities: RopeCapabilities,
}
impl HarnessView<'_> {
    pub fn positions_m(&self) -> impl ExactSizeIterator<Item = Point3> + '_ {
        self.soft_body.particle_positions().map(crate::output::xyz)
    }
    pub fn velocities_m_s(&self) -> impl ExactSizeIterator<Item = Point3> + '_ {
        self.soft_body.particle_velocities().map(crate::output::xyz)
    }
    pub fn segments(&self) -> &[[u32; 2]] {
        self.samples.wire_segments()
    }
    #[allow(clippy::unnecessary_cast)]
    pub fn radius_m(&self) -> f64 {
        self.soft_body.particle_radius() as f64
    }
    /// Checked capture context, not proof of stepping or a solver checkpoint.
    #[allow(clippy::unnecessary_cast)]
    pub fn snapshot(&self, capture: CaptureStamp) -> Result<HarnessSnapshot, OutputError> {
        capture.validate()?;
        let phase = match capture.phase {
            CapturePhase::Initial => self.next_step == 0,
            CapturePhase::BeforeStep => capture.step == self.prepared && self.prepared.is_some(),
            CapturePhase::AfterStep => {
                self.prepared.is_none()
                    && capture.step.and_then(|n| n.checked_add(1)) == Some(self.next_step)
            }
        };
        if !phase || capture.outer_dt_s.value() as Real != self.world.integration_parameters.dt {
            return Err(OutputError::Invalid(
                "harness capture does not match registry phase/step/dt",
            ));
        }
        let positions: Vec<_> = self.positions_m().collect();
        let span_geometry = self
            .samples
            .spans()
            .iter()
            .map(|(name, s)| {
                let points: Vec<_> = s
                    .particle_indices()
                    .iter()
                    .map(|&i| positions[i as usize])
                    .collect();
                Ok((name.clone(), diagnose_geometry(s.reference(), &points)?))
            })
            .collect::<Result<BTreeMap<_, _>, DiagnosticError>>()?;
        let observed = capture.phase == CapturePhase::AfterStep;
        let attachments = self
            .soft_body
            .particle_attachments()
            .iter()
            .map(|a| {
                let anchor = self.world.bodies[a.body]
                    .position()
                    .transform_point(a.local_anchor);
                Ok(AttachmentObservation {
                    particle: a.particle,
                    body: a.body.into(),
                    local_anchor_m: crate::output::finite_point(a.local_anchor)?,
                    world_anchor_m: crate::output::finite_point(anchor)?,
                    position_error_m: Measurement::from_number(
                        (anchor - self.soft_body.particle_position(a.particle as usize)).length()
                            as f64,
                    ),
                    native_impulse_ns: if observed {
                        Observation::Available(crate::output::finite_point(a.impulse())?)
                    } else {
                        Observation::Unavailable {
                            reason: "native impulses provided only after an inspected caller step"
                                .into(),
                        }
                    },
                })
            })
            .collect::<Result<Vec<_>, DiagnosticError>>()?;
        let edge_impulses = self
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
                    Measurement::from_number(e.impulse() as f64)
                } else {
                    Observation::Unavailable {
                        reason: "native impulses provided only after an inspected caller step"
                            .into(),
                    }
                },
            })
            .collect();
        Ok(HarnessSnapshot {
            identity: self.handle.into(),
            name: self.specification.name.clone(),
            capture: capture.clone(),
            positions_m: positions,
            velocities_m_s: self.velocities_m_s().collect(),
            segments: self.segments().to_vec(),
            radius_m: FiniteScalar::new(self.radius_m())?,
            pinned: self.soft_body.particles().iter().map(|p| p.is_pinned()).collect(),
            junction_particles: self.samples.junction_particles().clone(),
            span_particles: self.samples.spans().iter()
                .map(|(n, s)| (n.clone(), s.particle_indices().to_vec())).collect(),
            span_geometry,
            junction_curvature: Observation::Unsupported {
                reason: "junction has several tangents; no cross-span bending or orientation clamp".into(),
            },
            attachments,
            edge_impulses,
            impulse_metadata: NativeImpulseMetadata {
                unit: "N s".into(),
                interval: "native last internal substep, not outer-step sum".into(),
                capture: capture.clone(),
                last_completed_step: capture.last_completed_step(),
                internal_substep_dt_s: Observation::Unavailable {
                    reason: "not independently exposed for this coupled world".into(),
                },
                edge_direction: "0.36 native signed distance-edge scalar; current geometry is not the saved substep gradient".into(),
                attachment_direction: "0.36 world-space vector: + on rigid body, - on particle".into(),
            },
            unprovided: Default::default(),
            capabilities: Default::default(),
        })
    }
}
