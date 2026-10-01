use super::{Check, Result, config::Config, real, scalar, vector, xyz};
use rapier_rope::rapier::{parry, prelude::*};
use serde::Serialize;
use std::time::Instant;

pub const CASES: [&str; 4] = ["hanging", "moving", "obstacle", "payload"];

pub fn world(config: &Config) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.gravity = vector(config.gravity_m_s2);
    world.integration_parameters.dt = real(config.dt_s);
    world.integration_parameters.num_solver_iterations = config.solver_iterations;
    world.integration_parameters.num_internal_pgs_iterations = config.internal_pgs_iterations;
    world
}

/// Native experiment construction, not a RopeSpec/resampling implementation.
pub fn rope(config: &Config, start: Vector, end: Vector, segments: usize) -> SoftBodyBuilder {
    configure(config, SoftBodyBuilder::rope(start, end, segments + 1))
}

pub fn configure(config: &Config, mut builder: SoftBodyBuilder) -> SoftBodyBuilder {
    let mut masses = vec![0.0; builder.positions.len()];
    for &[a, b] in &builder.edges {
        let half = (builder.positions[a as usize] - builder.positions[b as usize]).length()
            * real(config.linear_density_kg_m * 0.5);
        masses[a as usize] += half;
        masses[b as usize] += half;
    }
    builder.tension_only_edges = (0..builder.edges.len() as u32).collect();
    let material = SoftBodyMaterial {
        edge_softness: SpringCoefficients::new(
            real(config.edge_frequency_hz),
            real(config.edge_damping_ratio),
        ),
        bend_softness: SpringCoefficients::new(
            real(config.bend_frequency_hz),
            real(config.bend_damping_ratio),
        ),
        edge_plastic_yield: 0.0,
        plastic_yield: 0.0,
        tear_strain: None,
        tear_force: None,
        ..SoftBodyMaterial::default()
    };
    builder
        .material(material)
        .masses(masses)
        .particle_radius(real(config.radius_m))
        .linear_damping(real(config.linear_damping))
        .self_contacts(config.self_contacts)
        .additional_solver_iterations(config.additional_solver_iterations)
        .additional_pgs_iterations(config.additional_pgs_iterations)
        .can_sleep(false)
        .surface_collider(
            ColliderBuilder::ball(real(config.radius_m))
                .friction(real(config.friction))
                .restitution(0.0),
        )
}

#[derive(Serialize)]
pub struct Frame {
    pub step: usize,
    pub time_s: f64,
    pub positions_m: Vec<[f64; 3]>,
    pub velocities_m_s: Vec<[f64; 3]>,
    pub edge_impulses_ns: Vec<f64>,
    pub attachment_native_impulses_ns: Vec<[f64; 3]>,
    pub body_positions_m: Vec<[f64; 3]>,
}

#[derive(Serialize)]
pub struct Run {
    pub schema_version: u32,
    pub case: String,
    pub precision: &'static str,
    pub rapier_version: &'static str,
    pub coordinate_system: &'static str,
    pub attachment_impulse_convention: &'static str,
    pub effective_dt_s: f64,
    pub simulated_steps: usize,
    pub radius_m: f64,
    pub nominal_mass_kg: f64,
    pub structural_segments: Vec<[u32; 2]>,
    pub initial_positions_m: Vec<[f64; 3]>,
    pub particle_masses_kg: Vec<f64>,
    pub attachment_local_anchor_m: Option<[f64; 3]>,
    pub release_actual_time_s: Option<f64>,
    pub step_wall_time_s: f64,
    pub checks: Vec<Check>,
    pub frames: Vec<Frame>,
}

impl Run {
    pub fn passed(&self) -> bool {
        self.checks.iter().all(|check| check.passed)
    }
}

fn snapshot(
    world: &PhysicsWorld,
    handle: SoftBodyHandle,
    bodies: &[RigidBodyHandle],
    step: usize,
) -> Frame {
    let sb = &world.soft_bodies[handle];
    Frame {
        step,
        time_s: step as f64 * scalar(world.integration_parameters.dt),
        positions_m: sb.particle_positions().map(xyz).collect(),
        velocities_m_s: sb.particle_velocities().map(xyz).collect(),
        edge_impulses_ns: sb
            .edges()
            .iter()
            .map(|edge| scalar(edge.impulse()))
            .collect(),
        attachment_native_impulses_ns: sb
            .particle_attachments()
            .iter()
            .map(|a| xyz(a.impulse()))
            .collect(),
        body_positions_m: bodies
            .iter()
            .map(|handle| xyz(world.bodies[*handle].translation()))
            .collect(),
    }
}

pub fn finite(world: &PhysicsWorld, handle: SoftBodyHandle) -> bool {
    let sb = &world.soft_bodies[handle];
    world.quarantine().is_empty()
        && sb
            .particle_positions()
            .chain(sb.particle_velocities())
            .all(|v| v.is_finite())
        && sb.edges().iter().all(|edge| edge.impulse().is_finite())
        && sb
            .particle_attachments()
            .iter()
            .all(|a| a.impulse().is_finite())
        && world
            .bodies
            .iter()
            .all(|(_, body)| body.translation().is_finite() && body.linvel().is_finite())
}

pub fn obstacle_distance(sb: &SoftBody, segments: &[[u32; 2]], half: Vector) -> Result<f64> {
    let cuboid = parry::shape::Cuboid::new(half);
    let mut minimum = f64::MAX;
    for &[a, b] in segments {
        let capsule = parry::shape::Capsule::new(
            sb.particle_position(a as usize),
            sb.particle_position(b as usize),
            sb.particle_radius(),
        );
        if let Some(contact) = parry::query::contact(
            &Pose::IDENTITY,
            &capsule,
            &Pose::IDENTITY,
            &cuboid,
            real(100.0),
        )? {
            minimum = minimum.min(scalar(contact.dist));
        }
    }
    Ok(minimum)
}

pub fn run(case: &str, config: &Config) -> Result<Run> {
    config.validate()?;
    if !CASES.contains(&case) {
        return Err(format!("unknown case: {case}").into());
    }
    let mut world = world(config);
    let length = real(config.length_m);
    let (start, end) = match case {
        "payload" => {
            let start = vector(config.payload_anchor_m);
            (start, start - Vector::Y * length)
        }
        "obstacle" => {
            let start = Vector::new(-length * 0.5, real(config.obstacle_start_height_m), 0.0);
            (start, start + Vector::X * length)
        }
        _ => {
            let start = vector(config.hanging_start_m);
            (start, start + Vector::X * length)
        }
    };
    let mut builder = rope(config, start, end, config.segments);
    if case == "hanging" || case == "payload" {
        builder = builder.pinned_particles([0]);
    }
    let segments = builder.edges.clone();
    let handle = world.insert_soft_body(builder);
    let mut tracked_bodies = Vec::new();
    let mut attached = None;
    let mut obstacle = None;
    if case == "moving" {
        let body = world.insert_body(
            RigidBodyBuilder::kinematic_position_based().translation(vector(config.moving_body_m)),
        );
        world.soft_bodies[handle].attach_particle(0, body, &world.bodies);
        attached = Some((0, body));
        tracked_bodies.push(body);
    } else if case == "payload" {
        let center = end - Vector::Y * real(config.payload_radius_m * 1.2);
        for x_offset in [0.0, 2.0] {
            let (body, _) = world.insert(
                RigidBodyBuilder::dynamic()
                    .translation(center + Vector::X * real(x_offset))
                    .linvel(vector(config.payload_initial_velocity_m_s))
                    .can_sleep(false),
                ColliderBuilder::ball(real(config.payload_radius_m))
                    .mass(real(config.payload_mass_kg)),
            );
            tracked_bodies.push(body);
        }
        world.soft_bodies[handle].attach_particle(
            config.segments,
            tracked_bodies[0],
            &world.bodies,
        );
        attached = Some((config.segments, tracked_bodies[0]));
    } else if case == "obstacle" {
        let half = vector(config.obstacle_half_extents_m);
        let collider = world.insert_collider(
            ColliderBuilder::cuboid(half.x, half.y, half.z).friction(real(config.friction)),
            None,
        );
        obstacle = Some((collider, half));
    }
    let local_anchor = world.soft_bodies[handle]
        .particle_attachments()
        .first()
        .map(|a| a.local_anchor);
    let initial_anchor = attached.map(|(_, body)| {
        world.bodies[body]
            .position()
            .transform_point(local_anchor.unwrap())
    });
    let sb = &world.soft_bodies[handle];
    let mass: f64 = sb.particles().iter().map(|p| scalar(p.mass())).sum();
    let mut result = Run {
        schema_version: 1,
        case: case.into(),
        precision: super::precision(),
        rapier_version: "0.36.0",
        coordinate_system: "right-handed, Y-up, SI",
        attachment_impulse_convention: "0.36.0 native last-substep vector: + on rigid body, - on particle; no generic force conversion",
        effective_dt_s: scalar(world.integration_parameters.dt),
        simulated_steps: 0,
        radius_m: scalar(sb.particle_radius()),
        nominal_mass_kg: mass,
        structural_segments: segments.clone(),
        initial_positions_m: sb.particle_positions().map(xyz).collect(),
        particle_masses_kg: sb.particles().iter().map(|p| scalar(p.mass())).collect(),
        attachment_local_anchor_m: local_anchor.map(xyz),
        release_actual_time_s: None,
        step_wall_time_s: 0.0,
        checks: Vec::new(),
        frames: vec![snapshot(&world, handle, &tracked_bodies, 0)],
    };
    let steps = (config.duration_s / config.dt_s).round() as usize;
    let mut max_strain = 0.0_f64;
    let mut max_error = 0.0_f64;
    let mut max_anchor_travel = 0.0_f64;
    let mut max_rope_x = 0.0_f64;
    let mut max_upward_body_impulse = 0.0_f64;
    let mut contact_steps = 0;
    let mut min_obstacle_distance = 0.0_f64;
    let mut last_obstacle_distance = 0.0_f64;
    let mut release_velocity_error = None;
    let mut is_finite = true;
    for step in 0..steps {
        let t = step as f64 * result.effective_dt_s;
        if case == "moving" {
            let body = tracked_bodies[0];
            let phase = std::f64::consts::TAU * (t + result.effective_dt_s) / config.duration_s;
            world.bodies[body].set_next_kinematic_position(Pose::new(
                vector(config.moving_body_m)
                    + vector([
                        config.moving_translation_amplitude_m * phase.sin(),
                        0.5 * config.moving_translation_amplitude_m * phase.sin(),
                        0.0,
                    ]),
                Vector::Z * real(config.moving_rotation_amplitude_rad * phase.sin()),
            ));
            if t + 1e-9 >= config.release_time_s && attached.is_some() {
                let before = world.soft_bodies[handle].particle_velocity(0);
                world.soft_bodies[handle].detach_particle(0);
                release_velocity_error = Some(scalar(
                    (before - world.soft_bodies[handle].particle_velocity(0)).length(),
                ));
                result.release_actual_time_s = Some(t);
                attached = None;
            }
        }
        let timer = Instant::now();
        world.step();
        result.step_wall_time_s += timer.elapsed().as_secs_f64();
        result.simulated_steps = step + 1;
        if !finite(&world, handle) {
            is_finite = false;
            break;
        }
        let sb = &world.soft_bodies[handle];
        for edge in sb.edges().iter().take(config.segments) {
            let distance = (sb.particle_position(edge.vertices[0] as usize)
                - sb.particle_position(edge.vertices[1] as usize))
            .length();
            max_strain = max_strain.max(scalar(distance / edge.rest_length - 1.0));
        }
        if let Some((particle, body)) = attached {
            let anchor = world.bodies[body]
                .position()
                .transform_point(local_anchor.unwrap());
            max_error = max_error.max(scalar((anchor - sb.particle_position(particle)).length()));
            max_anchor_travel =
                max_anchor_travel.max(scalar((anchor - initial_anchor.unwrap()).length()));
        }
        if case == "payload" {
            max_rope_x = max_rope_x.max(scalar(
                (sb.particle_position(config.segments) - end).x.abs(),
            ));
            // In 0.36.0 SoftAttachmentConstraint::apply adds this native
            // impulse to the rigid body and subtracts it from the particle.
            max_upward_body_impulse =
                max_upward_body_impulse.max(scalar(sb.particle_attachments()[0].impulse().y));
        }
        if let Some((collider, half)) = obstacle {
            if world.narrow_phase.contact_pairs().any(|pair| {
                (pair.collider1 == collider || pair.collider2 == collider)
                    && pair.has_any_active_contact()
            }) {
                contact_steps += 1;
            }
            last_obstacle_distance = obstacle_distance(sb, &segments, half)?;
            min_obstacle_distance = min_obstacle_distance.min(last_obstacle_distance);
        }
        if (step + 1) % config.record_every_steps == 0
            || step + 1 == steps
            || result
                .release_actual_time_s
                .is_some_and(|time| (time - t).abs() < 1e-9)
        {
            result
                .frames
                .push(snapshot(&world, handle, &tracked_bodies, step + 1));
        }
    }
    let acceptance = &config.acceptance;
    result.checks.extend([
        Check::condition("finite_state_and_impulses", is_finite),
        Check::condition("completed_all_steps", result.simulated_steps == steps),
        Check::at_most(
            "relative_nominal_mass_error",
            (mass / (config.linear_density_kg_m * config.length_m) - 1.0).abs(),
            config.mass_tolerance(),
        ),
        Check::at_most(
            "radius_error_m",
            (result.radius_m - config.radius_m).abs(),
            acceptance.radius_error_m,
        ),
        Check::at_most(
            "max_structural_tensile_strain",
            max_strain,
            acceptance.max_tensile_strain,
        ),
    ]);
    match case {
        "hanging" => result.checks.push(Check::at_least(
            "free_end_drop_m",
            scalar(
                end.y
                    - world.soft_bodies[handle]
                        .particle_position(config.segments)
                        .y,
            ),
            acceptance.min_hanging_drop_m,
        )),
        "moving" => result.checks.extend([
            Check::at_most(
                "max_attachment_error_m",
                max_error,
                acceptance.max_attachment_error_m,
            ),
            Check::at_least(
                "anchor_travel_m",
                max_anchor_travel,
                acceptance.min_moving_anchor_travel_m,
            ),
            Check::condition("release_executed", release_velocity_error.is_some()),
            Check::at_most(
                "detach_velocity_change_m_s",
                release_velocity_error.unwrap_or(f64::MAX),
                0.0,
            ),
        ]),
        "obstacle" => result.checks.extend([
            Check::at_least(
                "active_contact_steps",
                contact_steps as f64,
                acceptance.min_obstacle_contact_steps as f64,
            ),
            Check::at_most(
                "max_capsule_box_penetration_m",
                -min_obstacle_distance,
                acceptance.max_obstacle_penetration_m,
            ),
            Check::at_most(
                "final_capsule_box_gap_m",
                last_obstacle_distance.max(0.0),
                acceptance.max_settled_obstacle_gap_m,
            ),
        ]),
        "payload" => result.checks.extend([
            Check::at_most(
                "max_attachment_error_m",
                max_error,
                acceptance.max_attachment_error_m,
            ),
            Check::at_least(
                "payload_height_above_freefall_m",
                scalar(
                    world.bodies[tracked_bodies[0]].translation().y
                        - world.bodies[tracked_bodies[1]].translation().y,
                ),
                acceptance.min_payload_freefall_difference_m,
            ),
            Check::at_least(
                "rope_tip_horizontal_motion_m",
                max_rope_x,
                acceptance.min_payload_rope_horizontal_motion_m,
            ),
            Check::at_least(
                "max_upward_payload_attachment_impulse_ns",
                max_upward_body_impulse,
                acceptance.min_payload_upward_attachment_impulse_ns,
            ),
        ]),
        _ => unreachable!(),
    }
    Ok(result)
}
