use crate::model::RopeCapability;
use std::fmt;

/// Validation/resolution failures. A failed build does not mutate a world.
#[derive(Clone, Debug, PartialEq)]
pub enum RopeErrorKind {
    InvalidValue(&'static str),
    ZeroLengthSegment(usize),
    DuplicateName(String),
    LocationOutOfRange {
        requested_m: f64,
        length_m: f64,
    },
    UnknownLocation(String),
    OutsideTolerance {
        requested_m: f64,
        nearest_m: f64,
        tolerance_m: f64,
    },
    SamplingLimit {
        max_particles: usize,
    },
    PrecisionLoss(&'static str),
    UnsupportedCapability(RopeCapability),
}

#[derive(Clone, Debug, PartialEq)]
pub struct RopeError {
    pub rope_name: String,
    pub field: String,
    pub kind: RopeErrorKind,
}

impl RopeError {
    pub(crate) fn new(name: &str, field: impl Into<String>, kind: RopeErrorKind) -> Self {
        Self {
            rope_name: name.into(),
            field: field.into(),
            kind,
        }
    }
}

impl fmt::Display for RopeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "rope {:?}, {}: {:?}",
            self.rope_name, self.field, self.kind
        )
    }
}

impl std::error::Error for RopeError {}
