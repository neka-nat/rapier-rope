//! Material-location operations resolved against the registered reference samples.
//!
//! An attachment is a point constraint and can represent an explicit gripper
//! grasp or connector attachment. It does not constrain cross-section orientation
//! and does not establish a grasp through collision or friction.
//!
//! ```
//! use rapier_rope::{rapier::prelude::*, *};
//! let mut world = PhysicsWorld::new();
//! world.integration_parameters.dt = 1.0 / 240.0;
//! let id = WorldId(1);
//! let mut ropes = RopeSet::new(id)?;
//! let spec = RopeSpec::new("cable", vec![[0.0, 1.5, 0.0], [1.0, 1.5, 0.0]],
//!     NativeRopeMaterial::new(0.1, SpringSettings::new(500.0, 1.0),
//!         SpringSettings::new(20.0, 0.8)),
//!     SamplingSettings::new(1.0 / 32.0), CollisionSettings::new(0.005));
//! let rope = ropes.insert(id, &mut world, &spec)?;
//! let body = world.insert_body(RigidBodyBuilder::kinematic_position_based()
//!     .translation(Vector::new(0.5, 1.5, 0.0)));
//! let prepared = ropes.prepare_attachments(id, &mut world, 0, 1.0 / 240.0,
//!     &[AttachmentCommand::Attach { rope, body,
//!         location: RopeLocation::ArcLength { arc_length_m: 0.5, tolerance_m: 0.001 } }])?;
//! assert_eq!(prepared.locations[0].location.particle_index, 16);
//! let grasp = prepared.prepared.created_attachments[0].handle;
//! world.step();
//! ropes.inspect(id, &world, 0)?;
//! ropes.prepare_attachments(id, &mut world, 1, 1.0 / 240.0,
//!     &[AttachmentCommand::Detach { attachment: grasp }])?;
//! world.step();
//! ropes.inspect(id, &world, 1)?;
//! # Ok::<(), RopeSetError>(())
//! ```
use crate::{
    AttachmentHandle, Point3, PreparedStep, RegistryPhase, ResolvedLocation, RopeCommand,
    RopeHandle, RopeLocation, RopeSet, RopeSetError, RopeSetErrorKind, WorldId,
    rapier::prelude::{PhysicsWorld, RigidBodyHandle},
};

#[derive(Clone, Debug, PartialEq)]
pub enum AttachmentCommand {
    /// Explicitly set a world point, then pin; native pinning zeros the velocity.
    Pin {
        rope: RopeHandle,
        location: RopeLocation,
        position_m: Point3,
    },
    /// Queue a pinned sample's next-step position. No immediate position or velocity change.
    MovePin {
        rope: RopeHandle,
        location: RopeLocation,
        target_m: Point3,
    },
    /// Release a pin, preserving the current particle velocity before the next step.
    Unpin {
        rope: RopeHandle,
        location: RopeLocation,
    },
    /// Attach/grasp at the current particle position. The body must already exist
    /// in this world. No teleport and no velocity reset; local anchor is returned.
    Attach {
        rope: RopeHandle,
        location: RopeLocation,
        body: RigidBodyHandle,
    },
    /// Release the single managed attachment, preserving current particle velocity.
    Detach { attachment: AttachmentHandle },
}

#[derive(Clone, Debug)]
pub struct LocatedSample {
    pub command_index: usize,
    pub rope: RopeHandle,
    /// Reference arc length, not the stretched rope's current arc length.
    pub location: ResolvedLocation,
}

#[derive(Clone, Debug)]
pub struct AttachmentPreparation {
    /// Created attachments include command index, particle and native local anchor.
    pub prepared: PreparedStep,
    /// One selection per location-bearing command, in input order. Detach has no
    /// new material selection and is identified by its generational handle.
    pub locations: Vec<LocatedSample>,
}

impl RopeSet {
    /// Resolve all material locations and validate the entire ordered batch before
    /// mutation. Uses the same prepare/step/inspect protocol as [`Self::prepare`].
    /// No silent nearest-sample fallback outside the caller's tolerance.
    pub fn prepare_attachments(
        &mut self,
        id: WorldId,
        world: &mut PhysicsWorld,
        step: u64,
        h: f64,
        commands: &[AttachmentCommand],
    ) -> Result<AttachmentPreparation, RopeSetError> {
        let mut locations = Vec::new();
        let mut native_commands = Vec::with_capacity(commands.len());
        for (command_index, command) in commands.iter().enumerate() {
            let (rope, location) = match command {
                AttachmentCommand::Pin { rope, location, .. }
                | AttachmentCommand::MovePin { rope, location, .. }
                | AttachmentCommand::Unpin { rope, location }
                | AttachmentCommand::Attach { rope, location, .. } => (*rope, location),
                AttachmentCommand::Detach { attachment } => {
                    native_commands.push(RopeCommand::Detach {
                        attachment: *attachment,
                    });
                    continue;
                }
            };
            let view = self.get(id, world, rope).map_err(|mut e| {
                e.phase = RegistryPhase::Prepare;
                e
            })?;
            let selection = view.samples.resolve_location(location).map_err(|e| {
                RopeSetError::new(
                    RegistryPhase::Prepare,
                    Some(rope),
                    RopeSetErrorKind::Location(Box::new(e)),
                )
            })?;
            let particle = selection.particle_index;
            native_commands.push(match *command {
                AttachmentCommand::Pin { position_m, .. } => RopeCommand::PinAt {
                    rope,
                    particle,
                    position_m,
                },
                AttachmentCommand::MovePin { target_m, .. } => RopeCommand::MovePin {
                    rope,
                    particle,
                    target_m,
                },
                AttachmentCommand::Unpin { .. } => RopeCommand::Unpin { rope, particle },
                AttachmentCommand::Attach { body, .. } => RopeCommand::Attach {
                    rope,
                    particle,
                    body,
                },
                AttachmentCommand::Detach { .. } => unreachable!(),
            });
            locations.push(LocatedSample {
                command_index,
                rope,
                location: selection,
            });
        }
        let prepared = self.prepare(id, world, step, h, &native_commands)?;
        Ok(AttachmentPreparation {
            prepared,
            locations,
        })
    }
}
