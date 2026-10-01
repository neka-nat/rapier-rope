//! Engine-independent rope definitions. Lengths are metres, masses kilograms.
use serde::{Deserialize, Serialize};

/// A Cartesian point in metres, stored in definition precision (`f64`).
pub type Point3 = [f64; 3];

/// Native spring parameters, not calibrated `EA`, `EI`, or `GJ`.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct SpringSettings {
    /// Positive frequency in Hz. Sampling does not rescale this value.
    pub natural_frequency_hz: f64,
    /// Finite, nonnegative damping ratio.
    pub damping_ratio: f64,
}

impl SpringSettings {
    pub const fn new(natural_frequency_hz: f64, damping_ratio: f64) -> Self {
        Self {
            natural_frequency_hz,
            damping_ratio,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxialResponse {
    TensionOnly,
    TensionAndCompression,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct NativeRopeMaterial {
    pub linear_density_kg_m: f64,
    pub axial: SpringSettings,
    pub bending: SpringSettings,
    pub axial_response: AxialResponse,
    /// Native linear velocity damping, in inverse seconds.
    pub linear_damping: f64,
}

impl NativeRopeMaterial {
    pub fn new(linear_density_kg_m: f64, axial: SpringSettings, bending: SpringSettings) -> Self {
        Self {
            linear_density_kg_m,
            axial,
            bending,
            axial_response: AxialResponse::TensionOnly,
            linear_damping: 1.0,
        }
    }
}

/// AND-mode collision groups. All 32 membership/filter bits are supported.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub struct CollisionGroups {
    pub memberships: u32,
    pub filter: u32,
}

impl Default for CollisionGroups {
    fn default() -> Self {
        Self {
            memberships: u32::MAX,
            filter: u32::MAX,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct CollisionSettings {
    /// Common contact radius, independent of sample spacing.
    pub radius_m: f64,
    /// One friction coefficient for the entire rope collider.
    pub friction: f64,
    pub groups: CollisionGroups,
    pub self_contacts: bool,
}

impl CollisionSettings {
    pub fn new(radius_m: f64) -> Self {
        Self {
            radius_m,
            friction: 0.5,
            groups: CollisionGroups::default(),
            self_contacts: false,
        }
    }
}

/// Body-local native settings. The caller owns timestep, gravity and world iterations.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
pub struct NativeDynamicsSettings {
    pub additional_solver_iterations: usize,
    pub additional_pgs_iterations: usize,
    pub can_sleep: bool,
}

impl Default for NativeDynamicsSettings {
    fn default() -> Self {
        Self {
            additional_solver_iterations: 0,
            additional_pgs_iterations: 3,
            can_sleep: false,
        }
    }
}

/// Rotation then translation. No scaling or initial deformation is supported.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct RigidPlacement {
    pub translation_m: Point3,
    /// Unit quaternion in `[x, y, z, w]` order; norm error must be <= 1e-10.
    pub rotation_xyzw: [f64; 4],
}

impl Default for RigidPlacement {
    fn default() -> Self {
        Self {
            translation_m: [0.0; 3],
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct SamplingSettings {
    /// Maximum reference arc length of a generated structural segment.
    pub max_segment_length_m: f64,
    /// Requested samples may snap within this distance. Original vertices never merge.
    pub merge_tolerance_m: f64,
    /// Absolute reference arc lengths, including any clamp locations needed later.
    pub required_samples_m: Vec<f64>,
    /// Allocation budget, checked before generating samples.
    pub max_particles: usize,
    /// Allowed relative change of structural/bend distance after native conversion.
    pub max_native_edge_relative_error: f64,
}

impl SamplingSettings {
    pub fn new(max_segment_length_m: f64) -> Self {
        Self {
            max_segment_length_m,
            merge_tolerance_m: 1e-9,
            required_samples_m: Vec::new(),
            max_particles: 65_536,
            max_native_edge_relative_error: 1e-4,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct NamedLocation {
    pub name: String,
    /// Absolute arc length along the reference polyline, in metres.
    pub arc_length_m: f64,
}

impl NamedLocation {
    pub fn new(name: impl Into<String>, arc_length_m: f64) -> Self {
        Self {
            name: name.into(),
            arc_length_m,
        }
    }
}

/// A material position, not a position in the deformed world geometry.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum RopeLocation {
    Start,
    End,
    Named(String),
    /// Resolves to an existing sample only when within the explicit tolerance.
    ArcLength {
        arc_length_m: f64,
        tolerance_m: f64,
    },
}

/// Requests rejected by this point-particle native rope implementation.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum RopeCapability {
    AxialTorsion,
    OrientationClamp,
    CalibratedElasticModuli,
}

/// A reference shape and native rope settings; no world or Rapier handles are stored.
///
/// All definition values use `f64` in both precision modes. [`crate::build_rope`]
/// checks conversion to the chosen native precision before returning a builder.
/// The reference shape establishes native rest distances. Gravity settling, pinning
/// and world stepping belong to the caller.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct RopeSpec {
    pub name: String,
    pub reference_centerline_m: Vec<Point3>,
    pub placement: RigidPlacement,
    pub material: NativeRopeMaterial,
    pub sampling: SamplingSettings,
    pub collision: CollisionSettings,
    pub dynamics: NativeDynamicsSettings,
    pub named_locations: Vec<NamedLocation>,
    pub required_capabilities: Vec<RopeCapability>,
}

impl RopeSpec {
    pub fn new(
        name: impl Into<String>,
        reference_centerline_m: Vec<Point3>,
        material: NativeRopeMaterial,
        sampling: SamplingSettings,
        collision: CollisionSettings,
    ) -> Self {
        Self {
            name: name.into(),
            reference_centerline_m,
            placement: RigidPlacement::default(),
            material,
            sampling,
            collision,
            dynamics: NativeDynamicsSettings::default(),
            named_locations: Vec::new(),
            required_capabilities: Vec::new(),
        }
    }
}
