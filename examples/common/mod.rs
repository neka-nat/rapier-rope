//! Shared scene helpers for examples and regression tests.
pub mod config;
pub mod output;
pub mod probes;
pub mod scenes;

use rapier_rope::rapier::prelude::*;
use serde::Serialize;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

#[allow(clippy::unnecessary_cast)]
pub fn real(value: f64) -> Real {
    value as Real
}

pub fn scalar(value: Real) -> f64 {
    #[cfg(feature = "f32")]
    {
        f64::from(value)
    }
    #[cfg(feature = "f64")]
    {
        value
    }
}

pub fn vector(value: [f64; 3]) -> Vector {
    Vector::new(real(value[0]), real(value[1]), real(value[2]))
}

pub fn xyz(value: Vector) -> [f64; 3] {
    [scalar(value.x), scalar(value.y), scalar(value.z)]
}

pub fn precision() -> &'static str {
    #[cfg(feature = "f32")]
    {
        "f32"
    }
    #[cfg(feature = "f64")]
    {
        "f64"
    }
}

#[derive(Debug, Serialize)]
pub struct Check {
    pub name: String,
    pub value: f64,
    pub comparison: &'static str,
    pub limit: f64,
    pub passed: bool,
}

impl Check {
    pub fn at_most(name: &str, value: f64, limit: f64) -> Self {
        Self {
            name: name.into(),
            value,
            comparison: "<=",
            limit,
            passed: value.is_finite() && value <= limit,
        }
    }

    pub fn at_least(name: &str, value: f64, limit: f64) -> Self {
        Self {
            name: name.into(),
            value,
            comparison: ">=",
            limit,
            passed: value.is_finite() && value >= limit,
        }
    }

    pub fn condition(name: &str, passed: bool) -> Self {
        Self::at_least(name, f64::from(u8::from(passed)), 1.0)
    }
}
