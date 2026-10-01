//! Particle-index commands for the step-validation layer.
use crate::{AttachmentHandle, Point3, RopeHandle, rapier::prelude::RigidBodyHandle};

/// Commands are validated as a complete ordered batch before any native mutation.
///
/// A particle can have one release followed by one acquisition in a batch.
/// Duplicate acquisitions, duplicate releases and acquisition-before-release are
/// rejected. Pin/attach are mutually exclusive, with at most one attachment.
/// A target may follow a current-position pin, but cannot precede a release in
/// the same batch. Only one explicit position/target per particle is accepted.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RopeCommand {
    /// Pin the particle at its current position using native semantics.
    Pin {
        rope: RopeHandle,
        particle: u32,
    },
    /// Explicitly set the world position, then pin and zero the velocity.
    PinAt {
        rope: RopeHandle,
        particle: u32,
        position_m: Point3,
    },
    /// Queue a pinned particle's next-step world target, without teleporting.
    MovePin {
        rope: RopeHandle,
        particle: u32,
        target_m: Point3,
    },
    Unpin {
        rope: RopeHandle,
        particle: u32,
    },
    /// Native current-position attachment to an existing ordinary rigid body.
    Attach {
        rope: RopeHandle,
        particle: u32,
        body: RigidBodyHandle,
    },
    /// Detach the single managed attachment. Unexpected external attachments fail validation.
    Detach {
        attachment: AttachmentHandle,
    },
}

#[derive(Clone, Debug)]
pub struct CreatedAttachment {
    pub command_index: usize,
    pub handle: AttachmentHandle,
    pub rope: RopeHandle,
    pub particle: u32,
    pub body: RigidBodyHandle,
    /// Captured from the particle's current position; metres in the body's frame.
    pub local_anchor_m: Point3,
}

#[derive(Clone, Debug)]
pub struct PreparedStep {
    pub step: u64,
    /// Effective native timestep represented as f64.
    pub dt_s: f64,
    pub applied_commands: usize,
    pub created_attachments: Vec<CreatedAttachment>,
}
