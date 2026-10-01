//! Reference-space rope definitions, native construction and world-bound registration.
//!
//! Enable exactly one precision feature. The caller owns the world and stepping.
//!
//! ```
//! use rapier_rope::{build_rope, CollisionSettings, NativeRopeMaterial,
//!     RopeSpec, SamplingSettings, SpringSettings};
//! let spec = RopeSpec::new("cable", vec![[0.0; 3], [1.0, 0.0, 0.0]],
//!     NativeRopeMaterial::new(0.1, SpringSettings::new(500.0, 1.0),
//!         SpringSettings::new(20.0, 0.8)),
//!     SamplingSettings::new(1.0 / 32.0), CollisionSettings::new(0.005));
//! let (native, samples, specification) = build_rope(&spec)?.into_parts();
//! let mut world = rapier_rope::rapier::prelude::PhysicsWorld::new();
//! let handle = world.insert_soft_body(native);
//! assert_eq!(samples.reference_length_m(), 1.0);
//! # Ok::<(), rapier_rope::RopeError>(())
//! ```

#[cfg(all(feature = "f32", feature = "f64"))]
compile_error!("rapier-rope: enable exactly one of `f32` or `f64`, not both");
#[cfg(not(any(feature = "f32", feature = "f64")))]
compile_error!("rapier-rope: enable exactly one of `f32` or `f64`");

#[cfg(feature = "f32")]
pub use rapier_f32 as rapier;
#[cfg(all(feature = "f64", not(feature = "f32")))]
pub use rapier_f64 as rapier;

#[cfg(any(feature = "f32", feature = "f64"))]
pub mod attachment;
#[cfg(any(feature = "f32", feature = "f64"))]
pub mod builder;
#[cfg(any(feature = "f32", feature = "f64"))]
pub mod command;
pub mod diagnostics;
pub mod error;
#[cfg(any(feature = "f32", feature = "f64"))]
pub mod harness;
#[cfg(any(feature = "f32", feature = "f64"))]
pub mod harness_output;
pub mod id;
pub mod model;
#[cfg(any(feature = "f32", feature = "f64"))]
pub mod output;
pub mod sampling;
#[cfg(any(feature = "f32", feature = "f64"))]
pub mod set;

#[cfg(any(feature = "f32", feature = "f64"))]
pub use attachment::{AttachmentCommand, AttachmentPreparation, LocatedSample};
#[cfg(any(feature = "f32", feature = "f64"))]
pub use builder::{RopeBuild, build_rope};
#[cfg(any(feature = "f32", feature = "f64"))]
pub use command::{CreatedAttachment, PreparedStep, RopeCommand};
pub use diagnostics::*;
pub use error::{RopeError, RopeErrorKind};
#[cfg(any(feature = "f32", feature = "f64"))]
pub use harness::*;
#[cfg(any(feature = "f32", feature = "f64"))]
pub use harness_output::HarnessSnapshot;
pub use id::{AttachmentHandle, HarnessHandle, RopeHandle, RopeSetId, WorldId};
pub use model::*;
#[cfg(any(feature = "f32", feature = "f64"))]
pub use output::*;
pub use sampling::{ResolvedLocation, SampledRope, sample_rope};
#[cfg(any(feature = "f32", feature = "f64"))]
pub use set::{
    AttachmentView, RegistryPhase, RemovedRope, RopeSet, RopeSetError, RopeSetErrorKind,
    RopeStepObservation, RopeView, StepReport,
};
