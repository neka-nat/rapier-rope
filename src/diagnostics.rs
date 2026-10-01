//! Geometric estimates and explicitly qualified native observations.
use crate::{Point3, SampledRope};
use serde::{Deserialize, Serialize};
use std::fmt;

/// Finite number: unlike ordinary serde floats, serializing NaN cannot silently
/// replace an observation by JSON null. All undefined values use explicit states.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "f64", into = "f64")]
pub struct FiniteScalar(f64);
impl FiniteScalar {
    pub fn new(value: f64) -> Result<Self, DiagnosticError> {
        if value.is_finite() {
            Ok(Self(value))
        } else {
            Err(DiagnosticError::NonFiniteNumber)
        }
    }
    pub fn value(self) -> f64 {
        self.0
    }
}
impl TryFrom<f64> for FiniteScalar {
    type Error = DiagnosticError;
    fn try_from(value: f64) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}
impl From<FiniteScalar> for f64 {
    fn from(value: FiniteScalar) -> Self {
        value.0
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "status", content = "data", rename_all = "snake_case")]
pub enum Observation<T> {
    Available(T),
    Undefined { reason: String },
    NonFinite { reason: String },
    Unsupported { reason: String },
    Unavailable { reason: String },
}
pub type Measurement = Observation<FiniteScalar>;
impl Measurement {
    pub fn from_number(value: f64) -> Self {
        match FiniteScalar::new(value) {
            Ok(value) => Self::Available(value),
            Err(_) => Self::NonFinite {
                reason: if value.is_nan() {
                    "nan"
                } else if value.is_sign_positive() {
                    "positive_infinity"
                } else {
                    "negative_infinity"
                }
                .into(),
            },
        }
    }
    pub fn value(&self) -> Option<f64> {
        match self {
            Self::Available(value) => Some(value.value()),
            _ => None,
        }
    }
}
fn undefined(reason: &str) -> Measurement {
    Observation::Undefined {
        reason: reason.into(),
    }
}
fn non_finite(reason: &str) -> Measurement {
    Observation::NonFinite {
        reason: reason.into(),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DiagnosticError {
    ParticleCount { expected: usize, actual: usize },
    NonFiniteNumber,
}
impl fmt::Display for DiagnosticError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "rope diagnostic: {self:?}")
    }
}
impl std::error::Error for DiagnosticError {}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SegmentDiagnostic {
    pub vertices: [u32; 2],
    pub reference_length_m: FiniteScalar,
    pub current_length_m: Measurement,
    /// Signed geometric strain (current/reference - 1), independent of native stress().
    pub axial_strain: Measurement,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CurvatureDiagnostic {
    pub particle_index: u32,
    pub reference_arc_length_m: FiniteScalar,
    pub incoming_length_m: Measurement,
    pub outgoing_length_m: Measurement,
    pub turning_angle_rad: Measurement,
    pub curvature_inv_m: Measurement,
    pub estimated_bend_radius_m: Measurement,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GeometryDiagnostics {
    pub positions_finite: bool,
    pub current_length_m: Measurement,
    pub max_tensile_strain: Measurement,
    pub segments: Vec<SegmentDiagnostic>,
    /// Evaluated at samples; endpoints have no two-sided curvature.
    pub curvature: Vec<CurvatureDiagnostic>,
    pub curvature_method: String,
}

fn finite(p: Point3) -> bool {
    p.into_iter().all(f64::is_finite)
}
fn subtract(a: Point3, b: Point3) -> Point3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn norm(p: Point3) -> f64 {
    p[0].hypot(p[1]).hypot(p[2])
}
fn length(a: Point3, b: Point3) -> Measurement {
    if finite(a) && finite(b) {
        Measurement::from_number(norm(subtract(b, a)))
    } else {
        non_finite("non_finite_position")
    }
}

/// Pure geometric diagnostics can describe invalid saved/input coordinates.
/// Native checked views still reject invalid world state before capture.
/// κ = atan2(|u×v|, u·v) / ((l_in+l_out)/2); radius = 1/κ.
pub fn diagnose_geometry(
    samples: &SampledRope,
    positions: &[Point3],
) -> Result<GeometryDiagnostics, DiagnosticError> {
    let count = samples.reference_positions_m().len();
    if positions.len() != count {
        return Err(DiagnosticError::ParticleCount {
            expected: count,
            actual: positions.len(),
        });
    }
    let mut total = 0.0;
    let mut maximum = 0.0_f64;
    let mut lengths_available = true;
    let mut strains_available = true;
    let segments = samples
        .structural_edges()
        .iter()
        .zip(samples.reference_edge_lengths_m())
        .map(|(&vertices, &reference)| {
            let [a, b] = vertices.map(|i| i as usize);
            let current = length(positions[a], positions[b]);
            let strain = if let Some(value) = current.value() {
                total += value;
                let result = Measurement::from_number(value / reference - 1.0);
                if let Some(value) = result.value() {
                    maximum = maximum.max(value);
                } else {
                    strains_available = false;
                }
                result
            } else {
                lengths_available = false;
                strains_available = false;
                non_finite("non_finite_segment_length")
            };
            SegmentDiagnostic {
                vertices,
                reference_length_m: FiniteScalar(reference),
                current_length_m: current,
                axial_strain: strain,
            }
        })
        .collect();
    let curvature = (0..count)
        .map(|i| {
            let incoming = if i > 0 {
                length(positions[i - 1], positions[i])
            } else {
                undefined("endpoint")
            };
            let outgoing = if i + 1 < count {
                length(positions[i], positions[i + 1])
            } else {
                undefined("endpoint")
            };
            let (angle, curvature, radius) = if !finite(positions[i]) {
                let x = non_finite("non_finite_position");
                (x.clone(), x.clone(), x)
            } else if i == 0 || i + 1 == count {
                let x = undefined("endpoint");
                (x.clone(), x.clone(), x)
            } else if let (Some(a), Some(b)) = (incoming.value(), outgoing.value()) {
                if a == 0.0 || b == 0.0 {
                    let x = undefined("collapsed_adjacent_segment");
                    (x.clone(), x.clone(), x)
                } else {
                    let u = subtract(positions[i], positions[i - 1]).map(|x| x / a);
                    let v = subtract(positions[i + 1], positions[i]).map(|x| x / b);
                    let cross = [
                        u[1] * v[2] - u[2] * v[1],
                        u[2] * v[0] - u[0] * v[2],
                        u[0] * v[1] - u[1] * v[0],
                    ];
                    let dot = u.iter().zip(v).map(|(x, y)| x * y).sum::<f64>();
                    let theta = norm(cross).atan2(dot);
                    let k = theta / (a * 0.5 + b * 0.5);
                    (
                        Measurement::from_number(theta),
                        Measurement::from_number(k),
                        if k == 0.0 {
                            undefined("straight_limit_infinite_radius")
                        } else {
                            Measurement::from_number(1.0 / k)
                        },
                    )
                }
            } else {
                let x = non_finite("non_finite_adjacent_segment");
                (x.clone(), x.clone(), x)
            };
            CurvatureDiagnostic {
                particle_index: i as u32,
                reference_arc_length_m: FiniteScalar(samples.arc_lengths_m()[i]),
                incoming_length_m: incoming,
                outgoing_length_m: outgoing,
                turning_angle_rad: angle,
                curvature_inv_m: curvature,
                estimated_bend_radius_m: radius,
            }
        })
        .collect();
    Ok(GeometryDiagnostics { positions_finite: positions.iter().all(|&p| finite(p)),
        current_length_m: if lengths_available { Measurement::from_number(total) } else { non_finite("non_finite_segment") },
        max_tensile_strain: if strains_available { Measurement::from_number(maximum) } else { non_finite("non_finite_strain") },
        segments, curvature, curvature_method: "turning_angle_at_sample / mean_current_adjacent_segment_length; endpoint undefined; estimated radius = 1/curvature".into() })
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum CapabilitySupport {
    Supported,
    Unsupported { reason: String },
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RopeCapabilities {
    pub geometry_diagnostics: CapabilitySupport,
    pub native_impulses: CapabilitySupport,
    pub axial_torsion: CapabilitySupport,
    pub orientation_clamp: CapabilitySupport,
    pub calibrated_elastic_moduli: CapabilitySupport,
}
impl Default for RopeCapabilities {
    fn default() -> Self {
        Self {
            geometry_diagnostics: CapabilitySupport::Supported,
            native_impulses: CapabilitySupport::Supported,
            axial_torsion: CapabilitySupport::Unsupported {
                reason: "native point-particle rope has no material frame or axial torsion".into(),
            },
            orientation_clamp: CapabilitySupport::Unsupported {
                reason: "attachments constrain a point only".into(),
            },
            calibrated_elastic_moduli: CapabilitySupport::Unsupported {
                reason: "native frequency/damping parameters are not calibrated EA/EI/GJ".into(),
            },
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct UnprovidedObservations {
    pub average_tension_n: Observation<Vec<FiniteScalar>>,
    pub material_stress_pa: Observation<Vec<FiniteScalar>>,
    pub axial_torsion_rad: Observation<Vec<FiniteScalar>>,
    pub solver_converged: Observation<bool>,
}
impl Default for UnprovidedObservations {
    fn default() -> Self {
        Self { average_tension_n: Observation::Unavailable { reason: "last internal substep duration and outer-step impulse sum are not independently known".into() },
            material_stress_pa: Observation::Unavailable { reason: "native stress() is not a calibrated material stress in Pa".into() },
            axial_torsion_rad: Observation::Unsupported { reason: "native point-particle rope has no material frame or axial torsion".into() },
            solver_converged: Observation::Unavailable { reason: "no independently verified native convergence observation".into() } }
    }
}
