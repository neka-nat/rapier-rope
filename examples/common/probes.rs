use super::{Check, Result, config::Config, real, scalar, scenes, xyz};
use rapier_rope::rapier::prelude::*;
use serde::Serialize;

#[derive(Serialize)]
pub struct MassRow {
    pub segments: usize,
    pub mass_kg: f64,
    pub radius_m: f64,
    pub native_default_radius_m: f64,
}

#[derive(Serialize)]
pub struct ImpulseRow {
    pub solver_substeps: usize,
    pub actual_dt_s: f64,
    pub last_impulse_ns: f64,
    pub assumed_last_substep_s: f64,
    pub controlled_last_substep_force_n: f64,
    pub impulse_divided_by_outer_dt_n: f64,
    pub expected_supported_weight_n: f64,
    pub max_reported_extra_substeps: usize,
}

#[derive(Serialize)]
pub struct Probes {
    pub schema_version: u32,
    pub precision: &'static str,
    pub checks: Vec<Check>,
    pub mass_resolution: Vec<MassRow>,
    pub pin_driven_velocity_m_s: [f64; 3],
    pub rotated_body_local_anchor_m: [f64; 3],
    pub wire_midpoint_min_endpoint_distance_m: f64,
    pub wire_midpoint_active_contact_steps: usize,
    pub impulse_interval: Vec<ImpulseRow>,
    pub generic_force_output: &'static str,
}

impl Probes {
    pub fn passed(&self) -> bool {
        self.checks.iter().all(|check| check.passed)
    }
}

pub fn run(config: &Config) -> Result<Probes> {
    let mut checks = Vec::new();
    let mut mass_resolution = Vec::new();
    for segments in [32, 64, 128] {
        let raw = SoftBodyBuilder::rope(
            Vector::ZERO,
            Vector::X * real(config.length_m),
            segments + 1,
        );
        let native_default_radius_m = scalar(raw.particle_radius);
        let sb = SoftBody::from(scenes::configure(config, raw));
        let mass: f64 = sb.particles().iter().map(|p| scalar(p.mass())).sum();
        checks.push(Check::at_most(
            &format!("mass_relative_error_{segments}"),
            (mass / (config.length_m * config.linear_density_kg_m) - 1.0).abs(),
            config.mass_tolerance(),
        ));
        checks.push(Check::at_most(
            &format!("explicit_radius_error_{segments}_m"),
            (scalar(sb.particle_radius()) - config.radius_m).abs(),
            config.acceptance.radius_error_m,
        ));
        checks.push(Check::condition(
            &format!("structural_only_tension_{segments}"),
            sb.edges().iter().take(segments).all(|e| e.tension_only)
                && sb.edges().iter().skip(segments).all(|e| !e.tension_only),
        ));
        mass_resolution.push(MassRow {
            segments,
            mass_kg: mass,
            radius_m: scalar(sb.particle_radius()),
            native_default_radius_m,
        });
    }
    let all_tension =
        SoftBody::from(SoftBodyBuilder::rope(Vector::ZERO, Vector::X, 4).tension_only());
    checks.push(Check::condition(
        "native_tension_only_also_sets_bends",
        all_tension.edges().iter().all(|e| e.tension_only),
    ));
    let positions = vec![
        Vector::ZERO,
        Vector::new(0.2, 0.1, 0.0),
        Vector::new(0.7, 0.3, 0.1),
        Vector::new(1.0, 0.3, 0.1),
    ];
    let edges = [[0, 1], [1, 2], [2, 3]];
    let reference_length: f64 = edges
        .iter()
        .map(|&[a, b]| scalar((positions[a] - positions[b]).length()))
        .sum();
    let builder = SoftBodyBuilder::new(positions)
        .edges(edges.iter().map(|&[a, b]| [a as u32, b as u32]).collect())
        .bend_edges(vec![[0, 2], [1, 3]])
        .wire(vec![[0, 1], [1, 2], [2, 3]]);
    let material = scenes::configure(config, builder);
    checks.push(Check::condition(
        "plasticity_and_tearing_disabled",
        material.material.plastic_yield == 0.0
            && material.material.edge_plastic_yield == 0.0
            && material.material.tear_strain.is_none()
            && material.material.tear_force.is_none(),
    ));
    let polyline = SoftBody::from(material);
    let mass: f64 = polyline.particles().iter().map(|p| scalar(p.mass())).sum();
    checks.push(Check::at_most(
        "nonuniform_polyline_relative_mass_error",
        (mass / (reference_length * config.linear_density_kg_m) - 1.0).abs(),
        config.mass_tolerance(),
    ));
    checks.push(Check::condition(
        "nonuniform_polyline_topology",
        polyline.num_particles() == 4 && polyline.edges().len() == 5,
    ));

    let mut world = scenes::world(config);
    world.gravity = Vector::ZERO;
    let handle = world.insert_soft_body(scenes::rope(config, Vector::ZERO, Vector::X, 1));
    world.soft_bodies[handle].set_particle_velocity(1, Vector::new(1.0, 2.0, 3.0));
    world.soft_bodies[handle].set_particle_pinned(1, true);
    checks.push(Check::at_most(
        "pin_start_velocity_m_s",
        scalar(world.soft_bodies[handle].particle_velocity(1).length()),
        0.0,
    ));
    world.soft_bodies[handle].set_particle_pinned(0, true);
    let target = Vector::new(1.0, 0.1, 0.0);
    world.soft_bodies[handle].set_particle_kinematic_target(1, target);
    world.step();
    let pin_velocity = world.soft_bodies[handle].particle_velocity(1);
    checks.push(Check::at_most(
        "pinned_target_error_m",
        scalar((world.soft_bodies[handle].particle_position(1) - target).length()),
        1e-6,
    ));
    checks.push(Check::at_most(
        "pinned_target_velocity_error_m_s",
        scalar((pin_velocity - Vector::Y * real(0.1) / world.integration_parameters.dt).length()),
        1e-4,
    ));
    world.soft_bodies[handle].set_particle_pinned(1, false);
    checks.push(Check::at_most(
        "unpin_velocity_change_m_s",
        scalar((world.soft_bodies[handle].particle_velocity(1) - pin_velocity).length()),
        0.0,
    ));
    checks.push(Check::condition(
        "unpin_restores_nominal_mass",
        world.soft_bodies[handle].particles()[1].inv_mass() > 0.0,
    ));

    let body = world.insert_body(RigidBodyBuilder::kinematic_position_based().pose(Pose::new(
        Vector::new(2.0, 3.0, 1.0),
        Vector::Z * real(std::f64::consts::FRAC_PI_2),
    )));
    let before_position = world.soft_bodies[handle].particle_position(1);
    let before_velocity = world.soft_bodies[handle].particle_velocity(1);
    world.soft_bodies[handle].attach_particle(1, body, &world.bodies);
    let local_anchor = world.soft_bodies[handle].particle_attachments()[0].local_anchor;
    checks.push(Check::at_most(
        "rotated_local_anchor_reconstruction_error_m",
        scalar(
            (world.bodies[body].position().transform_point(local_anchor) - before_position)
                .length(),
        ),
        1e-6,
    ));
    checks.push(Check::at_most(
        "attach_position_change_m",
        scalar((world.soft_bodies[handle].particle_position(1) - before_position).length()),
        0.0,
    ));
    checks.push(Check::at_most(
        "attach_velocity_change_m_s",
        scalar((world.soft_bodies[handle].particle_velocity(1) - before_velocity).length()),
        0.0,
    ));
    world.soft_bodies[handle].attach_particle(1, body, &world.bodies);
    checks.push(Check::condition(
        "native_allows_duplicate_attach",
        world.soft_bodies[handle].particle_attachments().len() == 2,
    ));
    world.soft_bodies[handle].detach_particle(1);
    checks.push(Check::condition(
        "native_detach_removes_all",
        world.soft_bodies[handle].particle_attachments().is_empty(),
    ));
    checks.push(Check::at_most(
        "detach_velocity_change_m_s",
        scalar((world.soft_bodies[handle].particle_velocity(1) - before_velocity).length()),
        0.0,
    ));
    let root = world.soft_bodies[handle].root_body();
    world.remove_body(root);
    checks.push(Check::condition(
        "external_root_removal_deletes_soft_body",
        world.soft_bodies.get(handle).is_none(),
    ));
    checks.push(Check::condition(
        "external_root_removal_keeps_target_body",
        world.bodies.get(body).is_some(),
    ));

    // Both endpoints remain well outside the box. Only the wire interior can
    // touch it, so an active solver contact establishes segment collision.
    let mut contact_world = scenes::world(config);
    let half = Vector::new(0.05, 0.05, 0.1);
    let obstacle =
        contact_world.insert_collider(ColliderBuilder::cuboid(half.x, half.y, half.z), None);
    let wire = contact_world.insert_soft_body(scenes::rope(
        config,
        Vector::new(-0.5, 0.3, 0.0),
        Vector::new(0.5, 0.3, 0.0),
        1,
    ));
    let mut contact_steps = 0;
    let mut min_endpoint_distance = f64::MAX;
    for _ in 0..(0.5 / config.dt_s).ceil() as usize {
        contact_world.step();
        if contact_world.narrow_phase.contact_pairs().any(|pair| {
            (pair.collider1 == obstacle || pair.collider2 == obstacle)
                && pair.has_any_active_contact()
        }) {
            contact_steps += 1;
        }
        for point in contact_world.soft_bodies[wire].particle_positions() {
            let outside = (point.abs() - half).max(Vector::ZERO);
            min_endpoint_distance = min_endpoint_distance.min(scalar(outside.length()));
        }
    }
    checks.push(Check::condition(
        "wire_midpoint_finite",
        scenes::finite(&contact_world, wire),
    ));
    checks.push(Check::at_least(
        "wire_midpoint_active_contact_steps",
        contact_steps as f64,
        1.0,
    ));
    checks.push(Check::at_least(
        "wire_midpoint_endpoint_clearance_m",
        min_endpoint_distance,
        config.radius_m * 3.0,
    ));
    checks.push(Check::at_most(
        "wire_midpoint_final_surface_distance_m",
        scenes::obstacle_distance(&contact_world.soft_bodies[wire], &[[0, 1]], half)?.abs(),
        0.01,
    ));

    // Isolated vertical spring with no contact, no damping, no extra substeps.
    // This deliberately does not establish a general impulse -> force API.
    let mut impulse_interval = Vec::new();
    for substeps in [4, 8, 16] {
        let mut isolated = config.clone();
        isolated.solver_iterations = substeps;
        isolated.additional_solver_iterations = 0;
        isolated.additional_pgs_iterations = 0;
        isolated.linear_damping = 0.0;
        let mut test_world = scenes::world(&isolated);
        let handle = test_world.insert_soft_body(
            scenes::rope(&isolated, Vector::Y, Vector::ZERO, 1)
                .pinned_particles([0])
                .no_surface_collider(),
        );
        let mut max_extra = 0;
        for _ in 0..(2.0 / config.dt_s).round() as usize {
            test_world.step();
            max_extra = max_extra.max(
                test_world.bodies[test_world.soft_bodies[handle].root_body()]
                    .additional_solver_iterations(),
            );
        }
        let sb = &test_world.soft_bodies[handle];
        let dt = scalar(test_world.integration_parameters.dt);
        let impulse = scalar(sb.edges()[0].impulse());
        let force = impulse / (dt / substeps as f64);
        let weight = scalar(sb.particles()[1].mass()) * (-config.gravity_m_s2[1]);
        checks.push(Check::condition(
            &format!("impulse_probe_no_extra_substeps_{substeps}"),
            max_extra == 0,
        ));
        checks.push(Check::at_most(
            &format!("controlled_force_relative_error_{substeps}"),
            (force / weight - 1.0).abs(),
            0.05,
        ));
        impulse_interval.push(ImpulseRow {
            solver_substeps: substeps,
            actual_dt_s: dt,
            last_impulse_ns: impulse,
            assumed_last_substep_s: dt / substeps as f64,
            controlled_last_substep_force_n: force,
            impulse_divided_by_outer_dt_n: impulse / dt,
            expected_supported_weight_n: weight,
            max_reported_extra_substeps: max_extra,
        });
    }
    Ok(Probes {
        schema_version: 1,
        precision: super::precision(),
        checks,
        mass_resolution,
        pin_driven_velocity_m_s: xyz(pin_velocity),
        rotated_body_local_anchor_m: xyz(local_anchor),
        wire_midpoint_min_endpoint_distance_m: min_endpoint_distance,
        wire_midpoint_active_contact_steps: contact_steps,
        impulse_interval,
        generic_force_output: "unqualified: last internal substep impulse; contact/adaptive/component substeps need an explicit interval contract",
    })
}
