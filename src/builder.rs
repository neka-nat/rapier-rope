//! Conversion to an unregistered native Rapier builder.
use crate::{
    error::{RopeError, RopeErrorKind},
    model::*,
    rapier::prelude::*,
    sampling::{SampledRope, distance, sample_rope},
};

/// Validated native construction, reference maps, and an owned input snapshot.
///
/// Mutating the native builder can invalidate the maps; they describe the returned
/// construction, not subsequent external topology changes.
#[derive(Clone, Debug)]
pub struct RopeBuild {
    specification: RopeSpec,
    sampled: SampledRope,
    native_builder: SoftBodyBuilder,
}

impl RopeBuild {
    pub fn specification(&self) -> &RopeSpec {
        &self.specification
    }
    pub fn sampled(&self) -> &SampledRope {
        &self.sampled
    }
    pub fn native_builder(&self) -> &SoftBodyBuilder {
        &self.native_builder
    }
    /// Consume for insertion into the caller's world, retaining reference data.
    pub fn into_parts(self) -> (SoftBodyBuilder, SampledRope, RopeSpec) {
        (self.native_builder, self.sampled, self.specification)
    }
}

#[allow(clippy::unnecessary_cast)]
pub(crate) fn convert(spec: &RopeSpec, field: &str, value: f64) -> Result<Real, RopeError> {
    let native = value as Real;
    if !native.is_finite() || (value != 0.0 && native == 0.0) {
        Err(RopeError::new(
            &spec.name,
            field,
            RopeErrorKind::PrecisionLoss("value overflows or underflows native precision"),
        ))
    } else {
        Ok(native)
    }
}

#[allow(clippy::unnecessary_cast)] // Selected native precision may already be f64.
fn scalar(value: Real) -> f64 {
    value as f64
}

fn spring(
    spec: &RopeSpec,
    field: &str,
    setting: SpringSettings,
) -> Result<SpringCoefficients<Real>, RopeError> {
    let result = SpringCoefficients::new(
        convert(spec, field, setting.natural_frequency_hz)?,
        convert(spec, field, setting.damping_ratio)?,
    );
    let omega_squared = result.angular_frequency() * result.angular_frequency();
    let damping_squared = result.damping_ratio * result.damping_ratio;
    if !omega_squared.is_finite()
        || omega_squared == 0.0
        || !(4.0 * damping_squared).is_finite()
        || (result.damping_ratio > 0.0 && damping_squared == 0.0)
    {
        return Err(RopeError::new(
            &spec.name,
            field,
            RopeErrorKind::PrecisionLoss(
                "native spring coefficient arithmetic overflows or underflows",
            ),
        ));
    }
    Ok(result)
}

fn placed(point: Point3, placement: RigidPlacement) -> Point3 {
    let q = placement.rotation_xyzw;
    let norm = q[0].hypot(q[1]).hypot(q[2]).hypot(q[3]);
    let [x, y, z, w] = q.map(|v| v / norm);
    let [a, b, c] = point;
    let t = [
        2.0 * (y * c - z * b),
        2.0 * (z * a - x * c),
        2.0 * (x * b - y * a),
    ];
    [
        a + w * t[0] + y * t[2] - z * t[1] + placement.translation_m[0],
        b + w * t[1] + z * t[0] - x * t[2] + placement.translation_m[1],
        c + w * t[2] + x * t[1] - y * t[0] + placement.translation_m[2],
    ]
}

/// Validate, resample and convert without inserting anything into a world.
///
/// Native rest lengths are derived from the placed particle positions. Collapsed,
/// nonfinite, or excessively distorted structural/bend distances are rejected.
/// A two-away bend of zero reference distance (a folded-back three-point shape)
/// is also rejected. No frequency retuning, plasticity or tearing is introduced.
pub fn build_rope(spec: &RopeSpec) -> Result<RopeBuild, RopeError> {
    let sampled = sample_rope(spec)?;
    let positions: Vec<_> = sampled
        .reference_positions_m()
        .iter()
        .enumerate()
        .map(|(i, &point)| {
            let p = placed(point, spec.placement);
            let field = format!("native.positions[{i}]");
            Ok(Vector::new(
                convert(spec, &field, p[0])?,
                convert(spec, &field, p[1])?,
                convert(spec, &field, p[2])?,
            ))
        })
        .collect::<Result<_, RopeError>>()?;
    for (i, &[a, b]) in sampled
        .structural_edges()
        .iter()
        .chain(sampled.bend_edges())
        .enumerate()
    {
        let expected = if i < sampled.structural_edges().len() {
            sampled.reference_edge_lengths_m()[i]
        } else {
            distance(
                sampled.reference_positions_m()[a as usize],
                sampled.reference_positions_m()[b as usize],
            )
        };
        let actual = scalar((positions[b as usize] - positions[a as usize]).length());
        if expected == 0.0
            || !actual.is_finite()
            || actual <= 0.0
            || (actual / expected - 1.0).abs() > spec.sampling.max_native_edge_relative_error
        {
            return Err(RopeError::new(
                &spec.name,
                format!("native.edges[{i}].rest_length"),
                RopeErrorKind::PrecisionLoss(
                    "edge collapses, overflows, or exceeds native rest-distance tolerance",
                ),
            ));
        }
    }
    let masses = sampled
        .particle_masses_kg()
        .iter()
        .enumerate()
        .map(|(i, &m)| {
            let field = format!("native.masses[{i}]");
            let value = convert(spec, &field, m)?;
            if !(1.0 / value).is_finite() {
                return Err(RopeError::new(
                    &spec.name,
                    field,
                    RopeErrorKind::PrecisionLoss("inverse particle mass overflows"),
                ));
            }
            Ok(value)
        })
        .collect::<Result<Vec<_>, RopeError>>()?;
    // Upstream computes the rest COM as sum(position * mass) / sum(mass).
    let total: Real = masses.iter().sum();
    let weighted = positions
        .iter()
        .zip(&masses)
        .fold(Vector::ZERO, |acc, (&p, &m)| acc + p * m);
    if !total.is_finite() || !weighted.is_finite() || !(weighted / total).is_finite() {
        return Err(RopeError::new(
            &spec.name,
            "native.rest_center_of_mass",
            RopeErrorKind::PrecisionLoss("mass-weighted position arithmetic overflows"),
        ));
    }
    let axial = spring(spec, "material.axial", spec.material.axial)?;
    let bending = spring(spec, "material.bending", spec.material.bending)?;
    let radius = convert(spec, "collision.radius_m", spec.collision.radius_m)?;
    let material = SoftBodyMaterial {
        edge_softness: axial,
        bend_softness: bending,
        plastic_yield: 0.0,
        edge_plastic_yield: 0.0,
        tear_strain: None,
        tear_force: None,
        ..SoftBodyMaterial::default()
    };
    let mut native = SoftBodyBuilder::new(positions)
        .masses(masses)
        .edges(sampled.structural_edges().to_vec())
        .bend_edges(sampled.bend_edges().to_vec())
        .wire(sampled.wire_segments().to_vec())
        .material(material)
        .particle_radius(radius)
        .self_contacts(spec.collision.self_contacts)
        .linear_damping(convert(
            spec,
            "material.linear_damping",
            spec.material.linear_damping,
        )?)
        .additional_solver_iterations(spec.dynamics.additional_solver_iterations)
        .additional_pgs_iterations(spec.dynamics.additional_pgs_iterations)
        .can_sleep(spec.dynamics.can_sleep)
        .surface_collider(
            ColliderBuilder::ball(radius)
                .friction(convert(
                    spec,
                    "collision.friction",
                    spec.collision.friction,
                )?)
                .restitution(0.0)
                .collision_groups(InteractionGroups::new(
                    Group::from_bits_retain(spec.collision.groups.memberships),
                    Group::from_bits_retain(spec.collision.groups.filter),
                    InteractionTestMode::And,
                )),
        );
    if spec.material.axial_response == AxialResponse::TensionOnly {
        native.tension_only_edges = (0..sampled.structural_edges().len() as u32).collect();
    }
    // Explicit native edge-index mapping: structural first, then two-away bends.
    native.edge_softness = (0..native.edges.len() as u32)
        .map(|i| (i, axial))
        .chain(
            (0..native.bend_edges.len() as u32).map(|i| (native.edges.len() as u32 + i, bending)),
        )
        .collect();
    Ok(RopeBuild {
        specification: spec.clone(),
        sampled,
        native_builder: native,
    })
}
