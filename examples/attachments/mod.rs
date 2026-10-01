//! Headless evidence helpers; not a public diagnostics/output API.
use crate::common::{self, Check, Result, config::Config, real, scalar, vector, xyz};
use rapier_rope::{rapier::prelude::*, *};
use serde::Serialize;
use serde_json::json;
use std::path::Path;

const ID: WorldId = WorldId(3);

pub fn spec(config: &Config, name: &str, start: Point3, end: Point3) -> RopeSpec {
    let mut spec = RopeSpec::new(
        name,
        vec![start, end],
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

#[derive(Serialize)]
pub struct Frame {
    step: usize,
    time_s: f64,
    positions_m: Vec<Point3>,
    body_positions_m: Vec<Point3>,
}
#[derive(Serialize)]
pub struct Run {
    pub case: String,
    pub steps: usize,
    pub selected_particle: u32,
    pub requested_arc_length_m: f64,
    pub actual_arc_length_m: f64,
    pub local_anchor_m: Option<Point3>,
    pub nominal_rope_mass_kg: f64,
    pub payload_body_mass_kg: Option<f64>,
    pub release_time_s: Option<f64>,
    pub release_velocity_before_m_s: Option<Point3>,
    pub release_velocity_after_m_s: Option<Point3>,
    pub checks: Vec<Check>,
    pub frames: Vec<Frame>,
}
impl Run {
    pub fn passed(&self) -> bool {
        self.checks.iter().all(|c| c.passed)
    }
}
fn frame(
    world: &PhysicsWorld,
    native: SoftBodyHandle,
    bodies: &[RigidBodyHandle],
    step: usize,
) -> Frame {
    Frame {
        step,
        time_s: step as f64 * scalar(world.integration_parameters.dt),
        positions_m: world.soft_bodies[native]
            .particle_positions()
            .map(xyz)
            .collect(),
        body_positions_m: bodies
            .iter()
            .map(|h| xyz(world.bodies[*h].translation()))
            .collect(),
    }
}
fn strain(world: &PhysicsWorld, native: SoftBodyHandle, samples: &SampledRope) -> f64 {
    samples
        .structural_edges()
        .iter()
        .zip(samples.reference_edge_lengths_m())
        .map(|(&[a, b], &length)| {
            scalar(
                (world.soft_bodies[native].particle_position(a as usize)
                    - world.soft_bodies[native].particle_position(b as usize))
                .length(),
            ) / length
                - 1.0
        })
        .fold(0.0, f64::max)
}
fn mass(world: &PhysicsWorld, native: SoftBodyHandle) -> f64 {
    world.soft_bodies[native]
        .particles()
        .iter()
        .map(|p| scalar(p.mass()))
        .sum()
}
fn base_checks(
    config: &Config,
    run: &Run,
    finite: bool,
    max_strain: f64,
    max_error: f64,
) -> Vec<Check> {
    vec![
        Check::condition("finite_state_and_impulses", finite),
        Check::condition(
            "completed_all_steps",
            run.steps == (config.duration_s / config.dt_s).round() as usize,
        ),
        Check::at_most(
            "relative_nominal_mass_error",
            (run.nominal_rope_mass_kg / (config.linear_density_kg_m * config.length_m) - 1.0).abs(),
            config.mass_tolerance(),
        ),
        Check::at_most(
            "max_structural_tensile_strain",
            max_strain,
            config.acceptance.max_tensile_strain,
        ),
        Check::at_most(
            "max_constraint_error_m",
            max_error,
            config.acceptance.max_attachment_error_m,
        ),
    ]
}

pub fn moving(config: &Config, case: &str) -> Result<Run> {
    let pin = case == "moving_pin";
    let interior = case == "interior_grasp";
    if !pin && !interior && case != "endpoint_grasp" {
        return Err("unknown moving case".into());
    }
    let mut world = common::scenes::world(config);
    let mut ropes = RopeSet::new(ID)?;
    let start = vector(config.hanging_start_m);
    let end = start + Vector::X * real(config.length_m);
    let rope = ropes.insert(ID, &mut world, &spec(config, case, xyz(start), xyz(end)))?;
    let native = ropes.get(ID, &world, rope)?.native_handle;
    let samples = ropes.get(ID, &world, rope)?.samples.clone();
    let location = if interior {
        RopeLocation::ArcLength {
            arc_length_m: config.length_m * 0.5,
            tolerance_m: 0.0,
        }
    } else {
        RopeLocation::Start
    };
    let selected = samples.resolve_location(&location)?;
    let particle = selected.particle_index as usize;
    let initial = world.soft_bodies[native].particle_position(particle);
    let body = if pin {
        None
    } else {
        Some(world.insert_body(
            RigidBodyBuilder::kinematic_position_based().translation(vector(config.moving_body_m)),
        ))
    };
    let bodies = body.into_iter().collect::<Vec<_>>();
    let mut run = Run {
        case: case.into(),
        steps: 0,
        selected_particle: selected.particle_index,
        requested_arc_length_m: selected.requested_arc_length_m,
        actual_arc_length_m: selected.actual_arc_length_m,
        local_anchor_m: None,
        nominal_rope_mass_kg: mass(&world, native),
        payload_body_mass_kg: None,
        release_time_s: None,
        release_velocity_before_m_s: None,
        release_velocity_after_m_s: None,
        checks: vec![],
        frames: vec![frame(&world, native, &bodies, 0)],
    };
    let mut attachment = None;
    let mut local = Vector::ZERO;
    let mut released_height = None;
    let mut released = false;
    let mut max_error = 0.0_f64;
    let mut max_strain = 0.0_f64;
    let mut travel = 0.0_f64;
    let mut finite = true;
    let mut no_teleport = true;
    let mut pin_zero_velocity = true;
    let steps = (config.duration_s / config.dt_s).round() as usize;
    for step in 0..steps {
        let t = step as f64 * scalar(world.integration_parameters.dt);
        let phase = std::f64::consts::TAU * (t + scalar(world.integration_parameters.dt))
            / config.duration_s;
        let displacement = vector([
            config.moving_translation_amplitude_m * phase.sin(),
            config.moving_translation_amplitude_m * 0.5 * phase.sin(),
            0.0,
        ]);
        let target = initial + displacement;
        if let Some(body) = body {
            world.bodies[body].set_next_kinematic_position(Pose::new(
                vector(config.moving_body_m) + displacement,
                Vector::Z * real(config.moving_rotation_amplitude_rad * phase.sin()),
            ));
        }
        let releasing = !released && t + 1e-9 >= config.release_time_s;
        let before_position = world.soft_bodies[native].particle_position(particle);
        let before_velocity = world.soft_bodies[native].particle_velocity(particle);
        let commands = if step == 0 {
            vec![if pin {
                AttachmentCommand::Pin {
                    rope,
                    location: location.clone(),
                    position_m: xyz(initial),
                }
            } else {
                AttachmentCommand::Attach {
                    rope,
                    location: location.clone(),
                    body: body.unwrap(),
                }
            }]
        } else if releasing {
            vec![if pin {
                AttachmentCommand::Unpin {
                    rope,
                    location: location.clone(),
                }
            } else {
                AttachmentCommand::Detach {
                    attachment: attachment.ok_or("missing attachment")?,
                }
            }]
        } else if pin && !released {
            vec![AttachmentCommand::MovePin {
                rope,
                location: location.clone(),
                target_m: xyz(target),
            }]
        } else {
            vec![]
        };
        let prepared =
            ropes.prepare_attachments(ID, &mut world, step as u64, config.dt_s, &commands)?;
        if step == 0 {
            no_teleport = world.soft_bodies[native].particle_position(particle) == before_position;
            if pin {
                pin_zero_velocity =
                    world.soft_bodies[native].particle_velocity(particle) == Vector::ZERO;
            } else {
                let created = &prepared.prepared.created_attachments[0];
                attachment = Some(created.handle);
                run.local_anchor_m = Some(created.local_anchor_m);
                local = vector(created.local_anchor_m);
                no_teleport &=
                    world.soft_bodies[native].particle_velocity(particle) == before_velocity;
            }
        }
        if releasing {
            run.release_time_s = Some(t);
            run.release_velocity_before_m_s = Some(xyz(before_velocity));
            run.release_velocity_after_m_s =
                Some(xyz(world.soft_bodies[native].particle_velocity(particle)));
            released_height = Some(scalar(before_position.y));
            released = true;
        }
        world.step();
        ropes.inspect(ID, &world, step as u64)?;
        finite &= common::scenes::finite(&world, native);
        run.steps = step + 1;
        max_strain = max_strain.max(strain(&world, native, &samples));
        if !released {
            let anchor = if let Some(body) = body {
                world.bodies[body].position().transform_point(local)
            } else if step == 0 {
                initial
            } else {
                target
            };
            max_error = max_error.max(scalar(
                (anchor - world.soft_bodies[native].particle_position(particle)).length(),
            ));
            travel = travel.max(scalar((anchor - initial).length()));
        }
        if (step + 1) % config.record_every_steps == 0 || releasing || step + 1 == steps {
            run.frames.push(frame(&world, native, &bodies, step + 1));
        }
    }
    run.checks = base_checks(config, &run, finite, max_strain, max_error);
    run.checks.extend([
        Check::condition("initial_constraint_no_teleport", no_teleport),
        Check::at_least(
            "anchor_travel_m",
            travel,
            config.acceptance.min_moving_anchor_travel_m,
        ),
        Check::condition(
            "release_executed_at_one_second",
            run.release_time_s
                .is_some_and(|t| (t - config.release_time_s).abs() < 1e-6),
        ),
        Check::condition(
            "release_velocity_unchanged_before_step",
            run.release_velocity_before_m_s == run.release_velocity_after_m_s
                && run.release_velocity_before_m_s.is_some(),
        ),
        Check::at_least(
            "selected_particle_drop_after_release_m",
            released_height.ok_or("release missing")?
                - scalar(world.soft_bodies[native].particle_position(particle).y),
            config.acceptance.min_hanging_drop_m,
        ),
        Check::condition(
            "no_constraints_after_release",
            ropes.attachment_count() == 0
                && !world.soft_bodies[native].particles()[particle].is_pinned(),
        ),
    ]);
    if pin {
        run.checks.push(Check::condition(
            "pin_start_zero_velocity",
            pin_zero_velocity,
        ));
    }
    Ok(run)
}

pub fn payload(config: &Config) -> Result<Run> {
    let mut world = common::scenes::world(config);
    let mut ropes = RopeSet::new(ID)?;
    let start = vector(config.payload_anchor_m);
    let end = start - Vector::Y * real(config.length_m);
    let rope = ropes.insert(
        ID,
        &mut world,
        &spec(config, "payload", xyz(start), xyz(end)),
    )?;
    let native = ropes.get(ID, &world, rope)?.native_handle;
    let samples = ropes.get(ID, &world, rope)?.samples.clone();
    let center = end - Vector::Y * real(config.payload_radius_m * 1.2);
    let mut bodies = vec![];
    for offset in [0.0, 2.0] {
        let (body, _) = world.insert(
            RigidBodyBuilder::dynamic()
                .translation(center + Vector::X * real(offset))
                .linvel(vector(config.payload_initial_velocity_m_s))
                .can_sleep(false),
            ColliderBuilder::ball(real(config.payload_radius_m)).mass(real(config.payload_mass_kg)),
        );
        bodies.push(body);
    }
    let mut run = Run {
        case: "dynamic_payload".into(),
        steps: 0,
        selected_particle: config.segments as u32,
        requested_arc_length_m: config.length_m,
        actual_arc_length_m: config.length_m,
        local_anchor_m: None,
        nominal_rope_mass_kg: mass(&world, native),
        payload_body_mass_kg: Some(scalar(world.bodies[bodies[0]].mass())),
        release_time_s: None,
        release_velocity_before_m_s: None,
        release_velocity_after_m_s: None,
        checks: vec![],
        frames: vec![frame(&world, native, &bodies, 0)],
    };
    let mut local = Vector::ZERO;
    let mut max_error = 0.0_f64;
    let mut max_strain = 0.0_f64;
    let mut max_x = 0.0_f64;
    let mut upward_impulse = 0.0_f64;
    let mut finite = true;
    let steps = (config.duration_s / config.dt_s).round() as usize;
    for step in 0..steps {
        let commands = if step == 0 {
            vec![
                AttachmentCommand::Pin {
                    rope,
                    location: RopeLocation::Start,
                    position_m: xyz(start),
                },
                AttachmentCommand::Attach {
                    rope,
                    location: RopeLocation::End,
                    body: bodies[0],
                },
            ]
        } else {
            vec![]
        };
        let prepared =
            ropes.prepare_attachments(ID, &mut world, step as u64, config.dt_s, &commands)?;
        if step == 0 {
            let created = &prepared.prepared.created_attachments[0];
            run.local_anchor_m = Some(created.local_anchor_m);
            local = vector(created.local_anchor_m);
            run.selected_particle = prepared.locations[1].location.particle_index;
            run.actual_arc_length_m = prepared.locations[1].location.actual_arc_length_m;
        }
        world.step();
        ropes.inspect(ID, &world, step as u64)?;
        finite &= common::scenes::finite(&world, native);
        run.steps = step + 1;
        max_strain = max_strain.max(strain(&world, native, &samples));
        let tip = world.soft_bodies[native].particle_position(config.segments);
        let anchor = world.bodies[bodies[0]].position().transform_point(local);
        max_error = max_error.max(scalar((tip - anchor).length()));
        max_x = max_x.max(scalar((tip - end).x.abs()));
        upward_impulse = upward_impulse.max(scalar(
            world.soft_bodies[native].particle_attachments()[0]
                .impulse()
                .y,
        ));
        if (step + 1) % config.record_every_steps == 0 || step + 1 == steps {
            run.frames.push(frame(&world, native, &bodies, step + 1));
        }
    }
    // Independent direct-native construction with the same frozen settings.
    let direct = common::scenes::run("payload", config)?;
    let last = direct.frames.last().ok_or("missing direct-native frame")?;
    let final_difference = world.soft_bodies[native]
        .particle_positions()
        .zip(&last.positions_m)
        .map(|(p, q)| scalar((p - vector(*q)).length()))
        .chain(
            bodies
                .iter()
                .zip(&last.body_positions_m)
                .map(|(b, q)| scalar((world.bodies[*b].translation() - vector(*q)).length())),
        )
        .fold(0.0, f64::max);
    run.checks = base_checks(config, &run, finite, max_strain, max_error);
    run.checks.extend([
        Check::condition("direct_native_api_acceptance", direct.passed()),
        Check::at_most(
            "final_position_difference_from_direct_native_m",
            final_difference,
            if common::precision() == "f32" {
                1e-4
            } else {
                1e-10
            },
        ),
        Check::at_most(
            "rope_mass_change_after_payload_attachment_kg",
            (mass(&world, native) - run.nominal_rope_mass_kg).abs(),
            0.0,
        ),
        Check::at_most(
            "relative_payload_body_mass_error",
            (run.payload_body_mass_kg.unwrap() / config.payload_mass_kg - 1.0).abs(),
            config.mass_tolerance(),
        ),
        Check::at_least(
            "payload_height_above_freefall_m",
            scalar(
                world.bodies[bodies[0]].translation().y - world.bodies[bodies[1]].translation().y,
            ),
            config.acceptance.min_payload_freefall_difference_m,
        ),
        Check::at_least(
            "rope_tip_horizontal_motion_m",
            max_x,
            config.acceptance.min_payload_rope_horizontal_motion_m,
        ),
        Check::at_least(
            "max_upward_payload_native_last_substep_impulse_ns",
            upward_impulse,
            config.acceptance.min_payload_upward_attachment_impulse_ns,
        ),
    ]);
    Ok(run)
}

pub fn write_result(config: &Config, runs: Vec<Run>) -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() > 1 {
        return Err("usage: example [output.json]".into());
    }
    let passed = runs.iter().all(Run::passed);
    let result = json!({ "schema_version": 1, "precision": common::precision(), "rapier_version": "0.36.0", "configuration": config, "runs": runs, "passed": passed,
        "attachment_impulse_convention": "0.36.0 native last internal substep: + on rigid body, - on particle; not average force or tension" });
    let text = serde_json::to_string_pretty(&result)?;
    if let Some(path) = args.first() {
        let path = Path::new(path);
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, format!("{text}\n"))?;
    } else {
        println!("{text}");
    }
    if !passed {
        return Err("attachment example checks failed".into());
    }
    Ok(())
}
