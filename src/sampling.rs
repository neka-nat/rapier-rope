//! Reference-space sampling, nominal mass and material-location maps.
use crate::{
    error::{RopeError, RopeErrorKind},
    model::*,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResolvedLocation {
    /// Sample index equals the native particle index.
    pub particle_index: u32,
    pub requested_arc_length_m: f64,
    pub actual_arc_length_m: f64,
    pub error_m: f64,
}

/// Immutable reference data retained after the native builder is consumed.
#[derive(Clone, Debug)]
pub struct SampledRope {
    name: String,
    reference_positions_m: Vec<Point3>,
    arc_lengths_m: Vec<f64>,
    structural_edges: Vec<[u32; 2]>,
    bend_edges: Vec<[u32; 2]>,
    reference_edge_lengths_m: Vec<f64>,
    particle_masses_kg: Vec<f64>,
    original_vertex_indices: Vec<u32>,
    required_samples: Vec<ResolvedLocation>,
    named_locations: BTreeMap<String, ResolvedLocation>,
    nominal_mass_kg: f64,
}

impl SampledRope {
    pub fn reference_positions_m(&self) -> &[Point3] {
        &self.reference_positions_m
    }
    pub fn arc_lengths_m(&self) -> &[f64] {
        &self.arc_lengths_m
    }
    pub fn structural_edges(&self) -> &[[u32; 2]] {
        &self.structural_edges
    }
    pub fn bend_edges(&self) -> &[[u32; 2]] {
        &self.bend_edges
    }
    /// Wire segments use exactly the structural connectivity.
    pub fn wire_segments(&self) -> &[[u32; 2]] {
        &self.structural_edges
    }
    pub fn reference_edge_lengths_m(&self) -> &[f64] {
        &self.reference_edge_lengths_m
    }
    /// Nominal masses include particles that the caller may later pin.
    pub fn particle_masses_kg(&self) -> &[f64] {
        &self.particle_masses_kg
    }
    pub fn original_vertex_indices(&self) -> &[u32] {
        &self.original_vertex_indices
    }
    /// Corresponds to `SamplingSettings::required_samples_m` in input order.
    pub fn required_samples(&self) -> &[ResolvedLocation] {
        &self.required_samples
    }
    pub fn named_locations(&self) -> &BTreeMap<String, ResolvedLocation> {
        &self.named_locations
    }
    pub fn reference_length_m(&self) -> f64 {
        *self.arc_lengths_m.last().unwrap()
    }
    pub fn nominal_mass_kg(&self) -> f64 {
        self.nominal_mass_kg
    }

    pub fn resolve_location(&self, location: &RopeLocation) -> Result<ResolvedLocation, RopeError> {
        match location {
            RopeLocation::Start => Ok(resolved(&self.arc_lengths_m, 0.0, 0.0)),
            RopeLocation::End => {
                let end = self.reference_length_m();
                Ok(resolved(&self.arc_lengths_m, end, end))
            }
            RopeLocation::Named(name) => self.named_locations.get(name).cloned().ok_or_else(|| {
                RopeError::new(
                    &self.name,
                    "location.name",
                    RopeErrorKind::UnknownLocation(name.clone()),
                )
            }),
            RopeLocation::ArcLength {
                arc_length_m,
                tolerance_m,
            } => {
                check_nonnegative(&self.name, "location.tolerance_m", *tolerance_m)?;
                check_location(
                    &self.name,
                    "location.arc_length_m",
                    *arc_length_m,
                    self.reference_length_m(),
                )?;
                let nearest = nearest(&self.arc_lengths_m, *arc_length_m);
                if (nearest - arc_length_m).abs() > *tolerance_m {
                    return Err(RopeError::new(
                        &self.name,
                        "location.arc_length_m",
                        RopeErrorKind::OutsideTolerance {
                            requested_m: *arc_length_m,
                            nearest_m: nearest,
                            tolerance_m: *tolerance_m,
                        },
                    ));
                }
                Ok(resolved(&self.arc_lengths_m, *arc_length_m, nearest))
            }
        }
    }
}

pub(crate) fn distance(a: Point3, b: Point3) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1]).hypot(b[2] - a[2])
}

fn invalid(name: &str, field: &str, reason: &'static str) -> RopeError {
    RopeError::new(name, field, RopeErrorKind::InvalidValue(reason))
}

pub(crate) fn check_positive(name: &str, field: &str, value: f64) -> Result<(), RopeError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(invalid(name, field, "must be finite and positive"))
    }
}

fn check_nonnegative(name: &str, field: &str, value: f64) -> Result<(), RopeError> {
    if value.is_finite() && value >= 0.0 {
        Ok(())
    } else {
        Err(invalid(name, field, "must be finite and nonnegative"))
    }
}

fn check_location(name: &str, field: &str, value: f64, length: f64) -> Result<(), RopeError> {
    if !value.is_finite() {
        return Err(invalid(name, field, "must be finite"));
    }
    if value < 0.0 || value > length {
        return Err(RopeError::new(
            name,
            field,
            RopeErrorKind::LocationOutOfRange {
                requested_m: value,
                length_m: length,
            },
        ));
    }
    Ok(())
}

fn validate(spec: &RopeSpec) -> Result<Vec<f64>, RopeError> {
    let name = &spec.name;
    if name.trim().is_empty() {
        return Err(invalid(name, "name", "must not be blank"));
    }
    if let Some(capability) = spec.required_capabilities.first() {
        return Err(RopeError::new(
            name,
            "required_capabilities",
            RopeErrorKind::UnsupportedCapability(*capability),
        ));
    }
    if spec.reference_centerline_m.len() < 2 {
        return Err(invalid(
            name,
            "reference_centerline_m",
            "requires at least two vertices",
        ));
    }
    for (i, point) in spec.reference_centerline_m.iter().enumerate() {
        if !point.iter().all(|v| v.is_finite()) {
            return Err(invalid(
                name,
                &format!("reference_centerline_m[{i}]"),
                "coordinates must be finite",
            ));
        }
    }
    check_positive(
        name,
        "material.linear_density_kg_m",
        spec.material.linear_density_kg_m,
    )?;
    for (field, spring) in [
        ("material.axial", spec.material.axial),
        ("material.bending", spec.material.bending),
    ] {
        check_positive(
            name,
            &format!("{field}.natural_frequency_hz"),
            spring.natural_frequency_hz,
        )?;
        check_nonnegative(
            name,
            &format!("{field}.damping_ratio"),
            spring.damping_ratio,
        )?;
    }
    check_nonnegative(
        name,
        "material.linear_damping",
        spec.material.linear_damping,
    )?;
    check_positive(name, "collision.radius_m", spec.collision.radius_m)?;
    check_nonnegative(name, "collision.friction", spec.collision.friction)?;
    check_positive(
        name,
        "sampling.max_segment_length_m",
        spec.sampling.max_segment_length_m,
    )?;
    check_nonnegative(
        name,
        "sampling.merge_tolerance_m",
        spec.sampling.merge_tolerance_m,
    )?;
    let tolerance = spec.sampling.max_native_edge_relative_error;
    if !tolerance.is_finite() || tolerance <= 0.0 || tolerance >= 1.0 {
        return Err(invalid(
            name,
            "sampling.max_native_edge_relative_error",
            "must be in (0, 1)",
        ));
    }
    // Native softness indices concatenate structural and bend edges.
    if !(2..=(u32::MAX as usize / 2)).contains(&spec.sampling.max_particles) {
        return Err(invalid(
            name,
            "sampling.max_particles",
            "must be in [2, u32::MAX / 2]",
        ));
    }
    if spec.reference_centerline_m.len() > spec.sampling.max_particles {
        return Err(limit(spec));
    }
    if !spec.placement.translation_m.iter().all(|v| v.is_finite()) {
        return Err(invalid(name, "placement.translation_m", "must be finite"));
    }
    let q = spec.placement.rotation_xyzw;
    let norm = q[0].hypot(q[1]).hypot(q[2]).hypot(q[3]);
    if !norm.is_finite() || (norm - 1.0).abs() > 1e-10 {
        return Err(invalid(
            name,
            "placement.rotation_xyzw",
            "must be a unit quaternion (norm tolerance 1e-10)",
        ));
    }
    let mut arcs = vec![0.0];
    for (i, pair) in spec.reference_centerline_m.windows(2).enumerate() {
        let length = distance(pair[0], pair[1]);
        if length == 0.0 {
            return Err(RopeError::new(
                name,
                format!("reference_centerline_m[{i}..{}]", i + 1),
                RopeErrorKind::ZeroLengthSegment(i),
            ));
        }
        let next = arcs.last().unwrap() + length;
        if !next.is_finite() || next <= *arcs.last().unwrap() {
            return Err(invalid(
                name,
                "reference_centerline_m",
                "arc length overflow or lost increment",
            ));
        }
        arcs.push(next);
    }
    let length = *arcs.last().unwrap();
    check_positive(
        name,
        "nominal_mass_kg",
        length * spec.material.linear_density_kg_m,
    )?;
    let mut names = BTreeSet::new();
    for (i, named) in spec.named_locations.iter().enumerate() {
        let field = format!("named_locations[{i}]");
        if named.name.trim().is_empty() {
            return Err(invalid(name, &format!("{field}.name"), "must not be blank"));
        }
        if !names.insert(&named.name) {
            return Err(RopeError::new(
                name,
                format!("{field}.name"),
                RopeErrorKind::DuplicateName(named.name.clone()),
            ));
        }
        check_location(
            name,
            &format!("{field}.arc_length_m"),
            named.arc_length_m,
            length,
        )?;
    }
    for (i, &s) in spec.sampling.required_samples_m.iter().enumerate() {
        check_location(
            name,
            &format!("sampling.required_samples_m[{i}]"),
            s,
            length,
        )?;
    }
    Ok(arcs)
}

fn limit(spec: &RopeSpec) -> RopeError {
    RopeError::new(
        &spec.name,
        "sampling.max_particles",
        RopeErrorKind::SamplingLimit {
            max_particles: spec.sampling.max_particles,
        },
    )
}

/// Nearest existing arc, with ties resolved toward the smaller arc length.
fn nearest(arcs: &[f64], requested: f64) -> f64 {
    let i = arcs.partition_point(|s| *s < requested);
    if i == 0 {
        arcs[0]
    } else if i == arcs.len() || requested - arcs[i - 1] <= arcs[i] - requested {
        arcs[i - 1]
    } else {
        arcs[i]
    }
}

fn resolved(arcs: &[f64], requested: f64, actual: f64) -> ResolvedLocation {
    let particle_index = arcs.binary_search_by(|s| s.total_cmp(&actual)).unwrap() as u32;
    ResolvedLocation {
        particle_index,
        requested_arc_length_m: requested,
        actual_arc_length_m: actual,
        error_m: (actual - requested).abs(),
    }
}

/// Resample each mandatory interval separately, preserving all original vertices.
///
/// Requests are sorted by arc length. Each may snap to the closest original vertex
/// or previously accepted request within `merge_tolerance_m` (ties choose the lower
/// arc). Snapping uses the accepted arc, so proximity chains cannot accumulate error.
/// The returned maps report both requested and actual material positions.
/// This function validates definitions in `f64`; native conversion is checked by
/// [`crate::build_rope`].
pub fn sample_rope(spec: &RopeSpec) -> Result<SampledRope, RopeError> {
    let original_arcs = validate(spec)?;
    let mut requests: Vec<_> = spec
        .sampling
        .required_samples_m
        .iter()
        .copied()
        .chain(spec.named_locations.iter().map(|v| v.arc_length_m))
        .enumerate()
        .map(|(i, s)| (s, i))
        .collect();
    requests.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let mut accepted = vec![0.0; requests.len()];
    let mut extra_arcs: Vec<f64> = Vec::new();
    for (s, i) in requests {
        let mut candidate = nearest(&original_arcs, s);
        if let Some(&previous) = extra_arcs.last()
            && ((s - previous).abs() < (s - candidate).abs()
                || ((s - previous).abs() == (s - candidate).abs() && previous < candidate))
        {
            candidate = previous;
        }
        if (candidate - s).abs() <= spec.sampling.merge_tolerance_m {
            accepted[i] = candidate;
        } else {
            if extra_arcs.len() + original_arcs.len() >= spec.sampling.max_particles {
                return Err(limit(spec));
            }
            extra_arcs.push(s);
            accepted[i] = s;
        }
    }
    let mut anchors = original_arcs.clone();
    anchors.extend(extra_arcs);
    anchors.sort_by(f64::total_cmp);
    let mut divisions = Vec::with_capacity(anchors.len() - 1);
    let mut count = 1usize;
    for pair in anchors.windows(2) {
        let ratio = (pair[1] - pair[0]) / spec.sampling.max_segment_length_m;
        // Prevent a roundoff-sized excess at an exact integer from adding a segment.
        let rounded = ratio.round();
        let ratio = if (ratio - rounded).abs() <= 8.0 * f64::EPSILON * ratio.abs().max(1.0) {
            rounded
        } else {
            ratio
        };
        if !ratio.is_finite() || ratio > spec.sampling.max_particles as f64 {
            return Err(limit(spec));
        }
        let n = (ratio.ceil() as usize).max(1);
        count = count.checked_add(n).ok_or_else(|| limit(spec))?;
        if count > spec.sampling.max_particles {
            return Err(limit(spec));
        }
        divisions.push(n);
    }
    let mut arcs = Vec::with_capacity(count);
    arcs.push(0.0);
    for (pair, n) in anchors.windows(2).zip(divisions) {
        for j in 1..n {
            arcs.push(pair[0] + (pair[1] - pair[0]) * (j as f64 / n as f64));
        }
        arcs.push(pair[1]);
    }
    if arcs.windows(2).any(|p| p[1] <= p[0]) {
        return Err(RopeError::new(
            &spec.name,
            "sampling",
            RopeErrorKind::PrecisionLoss("generated arc increments collapse in f64"),
        ));
    }
    let positions = arcs
        .iter()
        .map(|&s| {
            let index = original_arcs.partition_point(|a| *a < s);
            if index < original_arcs.len() && original_arcs[index] == s {
                return spec.reference_centerline_m[index];
            }
            let a = index - 1;
            let t = (s - original_arcs[a]) / (original_arcs[a + 1] - original_arcs[a]);
            std::array::from_fn(|axis| {
                spec.reference_centerline_m[a][axis]
                    + (spec.reference_centerline_m[a + 1][axis]
                        - spec.reference_centerline_m[a][axis])
                        * t
            })
        })
        .collect();
    let lengths: Vec<_> = arcs.windows(2).map(|p| p[1] - p[0]).collect();
    let mut masses = vec![0.0; count];
    for (i, &length) in lengths.iter().enumerate() {
        let half = length * spec.material.linear_density_kg_m * 0.5;
        masses[i] += half;
        masses[i + 1] += half;
    }
    for &mass in &masses {
        check_positive(&spec.name, "particle_masses_kg", mass)?;
    }
    let required_count = spec.sampling.required_samples_m.len();
    Ok(SampledRope {
        name: spec.name.clone(),
        reference_positions_m: positions,
        arc_lengths_m: arcs.clone(),
        structural_edges: (0..count as u32 - 1).map(|i| [i, i + 1]).collect(),
        bend_edges: (0..count as u32 - 2).map(|i| [i, i + 2]).collect(),
        reference_edge_lengths_m: lengths,
        particle_masses_kg: masses,
        original_vertex_indices: original_arcs
            .iter()
            .map(|&s| resolved(&arcs, s, s).particle_index)
            .collect(),
        required_samples: spec
            .sampling
            .required_samples_m
            .iter()
            .zip(&accepted)
            .map(|(&s, &a)| resolved(&arcs, s, a))
            .collect(),
        named_locations: spec
            .named_locations
            .iter()
            .zip(&accepted[required_count..])
            .map(|(v, &a)| (v.name.clone(), resolved(&arcs, v.arc_length_m, a)))
            .collect(),
        nominal_mass_kg: original_arcs.last().unwrap() * spec.material.linear_density_kg_m,
    })
}
