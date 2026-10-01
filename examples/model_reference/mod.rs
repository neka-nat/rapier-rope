//! Native comparison: two separate worlds, with identical caller-side operations.
use crate::common::{self, config::Config, real, scalar, vector, xyz};
use rapier_rope::{rapier::prelude::*, *};
use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct Comparison {
    pub case: String,
    pub precision: &'static str,
    pub matched_native_inputs: bool,
    pub steps: usize,
    pub max_initial_position_error_m: f64,
    pub max_initial_mass_error_kg: f64,
    pub max_position_error_m: f64,
    pub max_velocity_error_m_s: f64,
    pub max_body_position_error_m: f64,
    pub input_position_tolerance_m: f64,
    pub input_mass_tolerance_kg: f64,
    pub position_tolerance_m: f64,
    pub velocity_tolerance_m_s: f64,
    pub finite: bool,
    pub passed: bool,
}

fn definition(config: &Config, start: Vector, end: Vector) -> RopeSpec {
    let mut spec = RopeSpec::new(
        "baseline",
        vec![xyz(start), xyz(end)],
        NativeRopeMaterial::new(
            config.linear_density_kg_m,
            SpringSettings::new(config.edge_frequency_hz, config.edge_damping_ratio),
            SpringSettings::new(config.bend_frequency_hz, config.bend_damping_ratio),
        ),
        SamplingSettings::new(config.length_m / config.segments as f64),
        CollisionSettings::new(config.radius_m),
    );
    spec.material.linear_damping = config.linear_damping;
    spec.collision.friction = config.friction;
    spec.collision.self_contacts = config.self_contacts;
    spec.dynamics.additional_solver_iterations = config.additional_solver_iterations;
    spec.dynamics.additional_pgs_iterations = config.additional_pgs_iterations;
    spec
}

fn setup(
    case: &str,
    config: &Config,
    builder: SoftBodyBuilder,
    end: Vector,
) -> (PhysicsWorld, SoftBodyHandle, Vec<RigidBodyHandle>) {
    let mut world = common::scenes::world(config);
    let handle = world.insert_soft_body(builder);
    let mut bodies = Vec::new();
    match case {
        "moving" => {
            let body = world.insert_body(
                RigidBodyBuilder::kinematic_position_based()
                    .translation(vector(config.moving_body_m)),
            );
            world.soft_bodies[handle].attach_particle(0, body, &world.bodies);
            bodies.push(body);
        }
        "payload" => {
            let center = end - Vector::Y * real(config.payload_radius_m * 1.2);
            for offset in [0.0, 2.0] {
                let (body, _) = world.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(center + Vector::X * real(offset))
                        .linvel(vector(config.payload_initial_velocity_m_s))
                        .can_sleep(false),
                    ColliderBuilder::ball(real(config.payload_radius_m))
                        .mass(real(config.payload_mass_kg)),
                );
                bodies.push(body);
            }
            world.soft_bodies[handle].attach_particle(config.segments, bodies[0], &world.bodies);
        }
        "obstacle" => {
            let half = vector(config.obstacle_half_extents_m);
            world.insert_collider(
                ColliderBuilder::cuboid(half.x, half.y, half.z).friction(real(config.friction)),
                None,
            );
        }
        _ => {}
    }
    (world, handle, bodies)
}

pub fn compare(
    case: &str,
    config: &Config,
    matched_native_inputs: bool,
) -> common::Result<Comparison> {
    config.validate()?;
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
        "hanging" | "moving" | "polyline_placed" => {
            let start = vector(config.hanging_start_m);
            (start, start + Vector::X * length)
        }
        _ => return Err(format!("unknown comparison case: {case}").into()),
    };
    let mut spec = definition(config, start, end);
    let mut direct = if case == "polyline_placed" {
        // Independently specified nonuniform vertices, topology and rigid placement.
        spec.reference_centerline_m = vec![[0.0; 3], [0.3, 0.0, 0.0], [0.3, 0.4, 0.0]];
        spec.sampling.max_segment_length_m = 0.5;
        spec.placement = RigidPlacement {
            translation_m: [1.0, 2.0, 3.0],
            rotation_xyzw: [
                0.0,
                0.0,
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
            ],
        };
        common::scenes::configure(
            config,
            SoftBodyBuilder::new(vec![
                vector([1.0, 2.0, 3.0]),
                vector([1.0, 2.3, 3.0]),
                vector([0.6, 2.3, 3.0]),
            ])
            .edges(vec![[0, 1], [1, 2]])
            .bend_edges(vec![[0, 2]])
            .wire(vec![[0, 1], [1, 2]]),
        )
    } else {
        common::scenes::rope(config, start, end, config.segments)
    };
    let (mut wrapped, _, _) = build_rope(&spec)?.into_parts();
    if matches!(case, "hanging" | "payload" | "polyline_placed") {
        direct = direct.pinned_particles([0]);
        wrapped = wrapped.pinned_particles([0]);
    }
    assert_eq!(wrapped.positions.len(), direct.positions.len());
    assert_eq!(wrapped.edges, direct.edges);
    assert_eq!(wrapped.bend_edges, direct.bend_edges);
    assert_eq!(wrapped.wire, direct.wire);
    assert_eq!(wrapped.tension_only_edges, direct.tension_only_edges);
    assert_eq!(wrapped.particle_radius, direct.particle_radius);
    assert_eq!(
        wrapped.material.edge_softness.natural_frequency,
        direct.material.edge_softness.natural_frequency
    );
    assert_eq!(
        wrapped.material.edge_softness.damping_ratio,
        direct.material.edge_softness.damping_ratio
    );
    assert_eq!(
        wrapped.material.bend_softness.natural_frequency,
        direct.material.bend_softness.natural_frequency
    );
    assert_eq!(
        wrapped.material.bend_softness.damping_ratio,
        direct.material.bend_softness.damping_ratio
    );
    assert_eq!(
        wrapped.particle_settings.linear_damping,
        direct.particle_settings.linear_damping
    );
    assert_eq!(
        wrapped.particle_settings.additional_solver_iterations,
        direct.particle_settings.additional_solver_iterations
    );
    assert_eq!(
        wrapped.particle_settings.additional_pgs_iterations,
        direct.particle_settings.additional_pgs_iterations
    );
    if matched_native_inputs {
        // Isolate native mapping from f64-vs-native interpolation/mass roundoff.
        direct.positions.clone_from(&wrapped.positions);
        direct.masses.clone_from(&wrapped.masses);
    }
    // Motion/mass limits fixed before the first run. Initial-position limit revised
    // after a 1-ULP f32 failure: two epsilon units at the scene coordinate scale.
    let coordinate_scale = wrapped
        .positions
        .iter()
        .flat_map(|p| [scalar(p.x).abs(), scalar(p.y).abs(), scalar(p.z).abs()])
        .fold(1.0, f64::max);
    let (position_tol, velocity_tol, input_position_tol, input_mass_tol) = if cfg!(feature = "f32")
    {
        (
            1e-4,
            1e-3,
            2.0 * f64::from(f32::EPSILON) * coordinate_scale,
            1e-8,
        )
    } else {
        (1e-10, 1e-9, 1e-12, 1e-14)
    };
    let initial_position = wrapped
        .positions
        .iter()
        .zip(&direct.positions)
        .map(|(&a, &b)| scalar((a - b).length()))
        .fold(0.0, f64::max);
    let initial_mass = wrapped
        .masses
        .iter()
        .zip(&direct.masses)
        .map(|(&a, &b)| scalar((a - b).abs()))
        .fold(0.0, f64::max);
    let (mut a, ha, ba) = setup(case, config, direct, end);
    let (mut b, hb, bb) = setup(case, config, wrapped, end);
    let mut result = Comparison {
        case: case.into(),
        precision: common::precision(),
        matched_native_inputs,
        steps: 0,
        max_initial_position_error_m: initial_position,
        max_initial_mass_error_kg: initial_mass,
        max_position_error_m: initial_position,
        max_velocity_error_m_s: 0.0,
        max_body_position_error_m: 0.0,
        input_position_tolerance_m: input_position_tol,
        input_mass_tolerance_kg: input_mass_tol,
        position_tolerance_m: position_tol,
        velocity_tolerance_m_s: velocity_tol,
        finite: true,
        passed: false,
    };
    let dt = scalar(a.integration_parameters.dt);
    for step in 0..(config.duration_s / config.dt_s).round() as usize {
        let t = step as f64 * dt;
        if case == "moving" {
            let phase = std::f64::consts::TAU * (t + dt) / config.duration_s;
            let pose = Pose::new(
                vector(config.moving_body_m)
                    + vector([
                        config.moving_translation_amplitude_m * phase.sin(),
                        0.5 * config.moving_translation_amplitude_m * phase.sin(),
                        0.0,
                    ]),
                Vector::Z * real(config.moving_rotation_amplitude_rad * phase.sin()),
            );
            a.bodies[ba[0]].set_next_kinematic_position(pose);
            b.bodies[bb[0]].set_next_kinematic_position(pose);
            if t + 1e-9 >= config.release_time_s {
                a.soft_bodies[ha].detach_particle(0);
                b.soft_bodies[hb].detach_particle(0);
            }
        }
        a.step();
        b.step();
        result.steps = step + 1;
        if !common::scenes::finite(&a, ha) || !common::scenes::finite(&b, hb) {
            result.finite = false;
            break;
        }
        for (pa, pb) in a.soft_bodies[ha]
            .particle_positions()
            .zip(b.soft_bodies[hb].particle_positions())
        {
            result.max_position_error_m =
                result.max_position_error_m.max(scalar((pa - pb).length()));
        }
        for (va, vb) in a.soft_bodies[ha]
            .particle_velocities()
            .zip(b.soft_bodies[hb].particle_velocities())
        {
            result.max_velocity_error_m_s = result
                .max_velocity_error_m_s
                .max(scalar((va - vb).length()));
        }
        for (&ha, &hb) in ba.iter().zip(&bb) {
            result.max_body_position_error_m = result.max_body_position_error_m.max(scalar(
                (a.bodies[ha].translation() - b.bodies[hb].translation()).length(),
            ));
            result.max_velocity_error_m_s = result.max_velocity_error_m_s.max(scalar(
                (a.bodies[ha].linvel() - b.bodies[hb].linvel()).length(),
            ));
        }
    }
    result.passed = result.finite
        && initial_position <= input_position_tol
        && initial_mass <= input_mass_tol
        && result.max_position_error_m <= position_tol
        && result.max_body_position_error_m <= position_tol
        && result.max_velocity_error_m_s <= velocity_tol;
    if matched_native_inputs {
        result.passed &= result.max_position_error_m == 0.0
            && result.max_velocity_error_m_s == 0.0
            && result.max_body_position_error_m == 0.0;
    }
    Ok(result)
}
