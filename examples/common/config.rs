use super::{Result, precision};
use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Acceptance {
    pub relative_mass_error_f32: f64,
    pub relative_mass_error_f64: f64,
    pub radius_error_m: f64,
    pub max_tensile_strain: f64,
    pub max_attachment_error_m: f64,
    pub min_hanging_drop_m: f64,
    pub min_moving_anchor_travel_m: f64,
    pub min_obstacle_contact_steps: usize,
    pub max_obstacle_penetration_m: f64,
    pub max_settled_obstacle_gap_m: f64,
    pub min_payload_freefall_difference_m: f64,
    pub min_payload_rope_horizontal_motion_m: f64,
    pub min_payload_upward_attachment_impulse_ns: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub schema_version: u32,
    pub length_m: f64,
    pub linear_density_kg_m: f64,
    pub radius_m: f64,
    pub segments: usize,
    pub dt_s: f64,
    pub duration_s: f64,
    pub solver_iterations: usize,
    pub internal_pgs_iterations: usize,
    pub additional_solver_iterations: usize,
    pub additional_pgs_iterations: usize,
    pub edge_frequency_hz: f64,
    pub edge_damping_ratio: f64,
    pub bend_frequency_hz: f64,
    pub bend_damping_ratio: f64,
    pub linear_damping: f64,
    pub friction: f64,
    pub self_contacts: bool,
    pub gravity_m_s2: [f64; 3],
    pub record_every_steps: usize,
    pub hanging_start_m: [f64; 3],
    pub moving_body_m: [f64; 3],
    pub moving_translation_amplitude_m: f64,
    pub moving_rotation_amplitude_rad: f64,
    pub release_time_s: f64,
    pub obstacle_start_height_m: f64,
    pub obstacle_half_extents_m: [f64; 3],
    pub payload_anchor_m: [f64; 3],
    pub payload_mass_kg: f64,
    pub payload_radius_m: f64,
    pub payload_initial_velocity_m_s: [f64; 3],
    pub acceptance: Acceptance,
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let config: Self = serde_json::from_slice(&std::fs::read(path)?)?;
        config.validate()?;
        Ok(config)
    }

    pub fn validate(&self) -> Result<()> {
        // JSON cannot represent NaN/infinity. Still validate positivity and ranges
        // before invoking builder APIs which may index or panic on invalid inputs.
        if self.schema_version != 1 {
            return Err("unsupported config schema".into());
        }
        for (name, value) in [
            ("length_m", self.length_m),
            ("linear_density_kg_m", self.linear_density_kg_m),
            ("radius_m", self.radius_m),
            ("dt_s", self.dt_s),
            ("duration_s", self.duration_s),
            ("edge_frequency_hz", self.edge_frequency_hz),
            ("bend_frequency_hz", self.bend_frequency_hz),
            ("payload_mass_kg", self.payload_mass_kg),
            ("payload_radius_m", self.payload_radius_m),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(format!("{name} must be finite and positive").into());
            }
        }
        for value in [
            self.edge_damping_ratio,
            self.bend_damping_ratio,
            self.linear_damping,
            self.friction,
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err("damping and friction must be finite and nonnegative".into());
            }
        }
        if self.segments == 0
            || self.segments > 4096
            || self.solver_iterations == 0
            || self.internal_pgs_iterations == 0
            || self.record_every_steps == 0
        {
            return Err("invalid segment/iteration/record count".into());
        }
        let steps = self.duration_s / self.dt_s;
        if steps > 1_000_000.0 || (steps - steps.round()).abs() > 1e-6 {
            return Err("duration must be an integer number of steps, at most 1000000".into());
        }
        if !(0.0 < self.release_time_s && self.release_time_s < self.duration_s) {
            return Err("release_time_s must be inside the simulation interval".into());
        }
        for value in self.obstacle_half_extents_m {
            if !value.is_finite() || value <= 0.0 {
                return Err("obstacle half extents must be positive".into());
            }
        }
        Ok(())
    }

    pub fn mass_tolerance(&self) -> f64 {
        if precision() == "f32" {
            self.acceptance.relative_mass_error_f32
        } else {
            self.acceptance.relative_mass_error_f64
        }
    }
}
