//! Four baseline scenes plus a focused wire-midpoint contact record.
use crate::common::{self, Check, Result, config::Config, real, scalar, vector, xyz};
use rapier_rope::{rapier::prelude::*, *};
use std::path::Path;

pub const CASES: [&str; 5] = [
    "hanging",
    "moving",
    "obstacle",
    "payload",
    "obstacle_middle",
];
const ID: WorldId = WorldId(4);
fn f(value: f64) -> FiniteScalar {
    FiniteScalar::new(value).expect("finite experiment setting")
}
fn point(value: Point3) -> [FiniteScalar; 3] {
    value.map(f)
}
fn definition(config: &Config, case: &str, start: Vector, end: Vector) -> RopeSpec {
    let mut spec = RopeSpec::new(
        format!("{case} cable"),
        vec![xyz(start), xyz(end)],
        NativeRopeMaterial::new(
            config.linear_density_kg_m,
            SpringSettings::new(config.edge_frequency_hz, config.edge_damping_ratio),
            SpringSettings::new(config.bend_frequency_hz, config.bend_damping_ratio),
        ),
        SamplingSettings::new(if case == "obstacle_middle" {
            config.length_m
        } else {
            config.length_m / config.segments as f64
        }),
        CollisionSettings::new(config.radius_m),
    );
    spec.material.linear_damping = config.linear_damping;
    spec.collision.friction = config.friction;
    spec.collision.self_contacts = config.self_contacts;
    spec.dynamics.additional_solver_iterations = config.additional_solver_iterations;
    spec.dynamics.additional_pgs_iterations = config.additional_pgs_iterations;
    spec.named_locations
        .push(NamedLocation::new("end_connector", config.length_m));
    spec
}
fn snapshot(
    track: &mut RopeTrack,
    ropes: &RopeSet,
    world: &PhysicsWorld,
    rope: RopeHandle,
    bodies: &[RigidBodyHandle],
    stamp: CaptureStamp,
) -> Result<()> {
    track.push_frame(TrackFrame {
        capture: stamp.clone(),
        ropes: vec![ropes.centerline(ID, world, rope)?.snapshot(stamp)?],
        bodies: bodies
            .iter()
            .map(|&h| BodyPoseSnapshot::capture(world, h))
            .collect::<std::result::Result<Vec<_>, _>>()?,
    })?;
    Ok(())
}
pub fn run(case: &str, config: &Config) -> Result<(RopeTrack, Vec<Check>)> {
    if !CASES.contains(&case) {
        return Err("unknown track scene".into());
    }
    let mut world = common::scenes::world(config);
    let dt = scalar(world.integration_parameters.dt);
    let mut ropes = RopeSet::new(ID)?;
    let length = real(config.length_m);
    let (start, end) = if case == "payload" {
        let p = vector(config.payload_anchor_m);
        (p, p - Vector::Y * length)
    } else if case.starts_with("obstacle") {
        let p = Vector::new(-length * 0.5, real(config.obstacle_start_height_m), 0.0);
        (p, p + Vector::X * length)
    } else {
        let p = vector(config.hanging_start_m);
        (p, p + Vector::X * length)
    };
    let rope = ropes.insert(ID, &mut world, &definition(config, case, start, end))?;
    let native = ropes.get(ID, &world, rope)?.native_handle;
    let count = world.soft_bodies[native].num_particles();
    let tip = count - 1;
    let segments = ropes
        .get(ID, &world, rope)?
        .samples
        .wire_segments()
        .to_vec();
    let mut track = RopeTrack::new(case, "right-handed, Y-up, SI", &world)?;
    track.register_rope(&ropes.centerline(ID, &world, rope)?)?;
    let mut bodies = vec![];
    let mut obstacle = None;
    if case == "moving" {
        let body = world.insert_body(
            RigidBodyBuilder::kinematic_position_based().translation(vector(config.moving_body_m)),
        );
        bodies.push(body);
        track.add_display_object(DisplayObject {
            name: "gripper marker".into(),
            role: "marker".into(),
            body: Some(body.into()),
            shape: DisplayShape::Sphere { radius_m: f(0.015) },
            translation_m: point(config.moving_body_m),
            rotation_xyzw: [f(0.0), f(0.0), f(0.0), f(1.0)],
        })?;
    } else if case == "payload" {
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
            let pose = BodyPoseSnapshot::capture(&world, body)?;
            track.add_display_object(DisplayObject {
                name: if offset == 0.0 {
                    "payload"
                } else {
                    "freefall control"
                }
                .into(),
                role: if offset == 0.0 {
                    "collider"
                } else {
                    "freefall_control"
                }
                .into(),
                body: Some(body.into()),
                shape: DisplayShape::Sphere {
                    radius_m: f(config.payload_radius_m),
                },
                translation_m: pose.translation_m,
                rotation_xyzw: pose.rotation_xyzw,
            })?;
        }
    } else if case.starts_with("obstacle") {
        let half = vector(config.obstacle_half_extents_m);
        let collider = world.insert_collider(
            ColliderBuilder::cuboid(half.x, half.y, half.z).friction(real(config.friction)),
            None,
        );
        obstacle = Some((collider, half));
        track.add_display_object(DisplayObject {
            name: "obstacle collider".into(),
            role: "collider".into(),
            body: None,
            shape: DisplayShape::Cuboid {
                half_extents_m: point(config.obstacle_half_extents_m),
            },
            translation_m: point([0.0; 3]),
            rotation_xyzw: [f(0.0), f(0.0), f(0.0), f(1.0)],
        })?;
    }
    let mut attachment = None;
    let mut local = None;
    let mut released = false;
    let mut finite = true;
    let mut max_strain = 0.0_f64;
    let mut max_error = 0.0_f64;
    let mut travel = 0.0_f64;
    let mut max_x = 0.0_f64;
    let mut upward = 0.0_f64;
    let mut contacts = 0;
    let mut min_dist = 0.0_f64;
    let mut last_dist = 0.0_f64;
    let mut middle_only = 0;
    let mut release_velocity_equal = false;
    let steps = (config.duration_s / config.dt_s).round() as usize;
    for step in 0..steps {
        let time = step as f64 * dt;
        if case == "moving" {
            let phase = std::f64::consts::TAU * (time + dt) / config.duration_s;
            world.bodies[bodies[0]].set_next_kinematic_position(Pose::new(
                vector(config.moving_body_m)
                    + vector([
                        config.moving_translation_amplitude_m * phase.sin(),
                        0.5 * config.moving_translation_amplitude_m * phase.sin(),
                        0.0,
                    ]),
                Vector::Z * real(config.moving_rotation_amplitude_rad * phase.sin()),
            ));
        }
        let releasing = case == "moving" && !released && time + 1e-9 >= config.release_time_s;
        let before_velocity = world.soft_bodies[native].particle_velocity(0);
        let commands = if step == 0 {
            match case {
                "hanging" => vec![AttachmentCommand::Pin {
                    rope,
                    location: RopeLocation::Start,
                    position_m: xyz(start),
                }],
                "moving" => vec![AttachmentCommand::Attach {
                    rope,
                    location: RopeLocation::Start,
                    body: bodies[0],
                }],
                "payload" => vec![
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
                ],
                _ => vec![],
            }
        } else if releasing {
            vec![AttachmentCommand::Detach {
                attachment: attachment.ok_or("missing grasp")?,
            }]
        } else {
            vec![]
        };
        let prepared =
            ropes.prepare_attachments(ID, &mut world, step as u64, config.dt_s, &commands)?;
        if step == 0 {
            if case == "hanging" || case == "payload" {
                track.record_event(TrackEvent {
                    step: 0,
                    time_s: f(0.0),
                    operation: TrackEventKind::Pin {
                        rope: rope.into(),
                        particle: 0,
                    },
                })?;
            }
            if let Some(created) = prepared.prepared.created_attachments.first() {
                attachment = Some(created.handle);
                local = Some(vector(created.local_anchor_m));
                track.record_event(TrackEvent {
                    step: 0,
                    time_s: f(0.0),
                    operation: TrackEventKind::Attach {
                        rope: rope.into(),
                        particle: created.particle,
                        attachment: created.handle.into(),
                        body: created.body.into(),
                        local_anchor_m: point(created.local_anchor_m),
                    },
                })?;
            }
            snapshot(
                &mut track,
                &ropes,
                &world,
                rope,
                &bodies,
                CaptureStamp::new(CapturePhase::Initial, None, 0.0, dt)?,
            )?;
        }
        if releasing {
            released = true;
            release_velocity_equal =
                before_velocity == world.soft_bodies[native].particle_velocity(0);
            track.record_event(TrackEvent {
                step: step as u64,
                time_s: f(time),
                operation: TrackEventKind::Detach {
                    rope: rope.into(),
                    particle: 0,
                    attachment: attachment.unwrap().into(),
                    body: bodies[0].into(),
                },
            })?;
            snapshot(
                &mut track,
                &ropes,
                &world,
                rope,
                &bodies,
                CaptureStamp::new(CapturePhase::BeforeStep, Some(step as u64), time, dt)?,
            )?;
        }
        world.step();
        ropes.inspect(ID, &world, step as u64)?;
        finite &= common::scenes::finite(&world, native);
        let view = ropes.centerline(ID, &world, rope)?;
        let geometry = diagnose_geometry(
            ropes.get(ID, &world, rope)?.samples,
            &view.positions_m().collect::<Vec<_>>(),
        )?;
        max_strain = max_strain.max(
            geometry
                .max_tensile_strain
                .value()
                .ok_or("non-finite strain")?,
        );
        if let Some(local) = local
            && !released
        {
            let index = if case == "payload" { tip } else { 0 };
            let target = world.bodies[bodies[0]].position().transform_point(local);
            max_error = max_error.max(scalar(
                (target - world.soft_bodies[native].particle_position(index)).length(),
            ));
            travel = travel.max(scalar((target - start).length()));
        }
        if case == "payload" {
            max_x = max_x.max(scalar(
                (world.soft_bodies[native].particle_position(tip) - end)
                    .x
                    .abs(),
            ));
            upward = upward.max(scalar(
                world.soft_bodies[native].particle_attachments()[0]
                    .impulse()
                    .y,
            ));
        }
        if let Some((collider, half)) = obstacle {
            let active = world.narrow_phase.contact_pairs().any(|p| {
                (p.collider1 == collider || p.collider2 == collider) && p.has_any_active_contact()
            });
            if active {
                contacts += 1;
            }
            last_dist =
                common::scenes::obstacle_distance(&world.soft_bodies[native], &segments, half)?;
            min_dist = min_dist.min(last_dist);
            if case == "obstacle_middle"
                && active
                && last_dist < config.acceptance.max_settled_obstacle_gap_m
            {
                // Endpoints remain outside the cuboid by much more than the contact radius.
                let endpoint_gap = world.soft_bodies[native]
                    .particle_positions()
                    .map(|p| {
                        let outside = (p.abs() - half).max(Vector::ZERO);
                        scalar(outside.length()) - config.radius_m
                    })
                    .fold(f64::MAX, f64::min);
                if endpoint_gap > config.radius_m {
                    middle_only += 1;
                }
            }
        }
        if (step + 1) % config.record_every_steps == 0 || releasing || step + 1 == steps {
            snapshot(
                &mut track,
                &ropes,
                &world,
                rope,
                &bodies,
                CaptureStamp::new(
                    CapturePhase::AfterStep,
                    Some(step as u64),
                    (step + 1) as f64 * dt,
                    dt,
                )?,
            )?;
        }
    }
    let mass: f64 = world.soft_bodies[native]
        .particles()
        .iter()
        .map(|p| scalar(p.mass()))
        .sum();
    let mut checks = vec![
        Check::condition("finite_state_and_impulses", finite),
        Check::at_most(
            "max_tensile_strain",
            max_strain,
            config.acceptance.max_tensile_strain,
        ),
        Check::at_most(
            "relative_nominal_mass_error",
            (mass / (config.linear_density_kg_m * config.length_m) - 1.0).abs(),
            config.mass_tolerance(),
        ),
        Check::at_most(
            "radius_error_m",
            (scalar(world.soft_bodies[native].particle_radius()) - config.radius_m).abs(),
            config.acceptance.radius_error_m,
        ),
    ];
    match case {
        "hanging" => checks.push(Check::at_least(
            "free_tip_drop_m",
            scalar(end.y - world.soft_bodies[native].particle_position(tip).y),
            config.acceptance.min_hanging_drop_m,
        )),
        "moving" => checks.extend([
            Check::at_most(
                "max_attachment_error_m",
                max_error,
                config.acceptance.max_attachment_error_m,
            ),
            Check::at_least(
                "anchor_travel_m",
                travel,
                config.acceptance.min_moving_anchor_travel_m,
            ),
            Check::condition(
                "release_velocity_unchanged_before_step",
                release_velocity_equal,
            ),
        ]),
        "payload" => checks.extend([
            Check::at_most(
                "max_attachment_error_m",
                max_error,
                config.acceptance.max_attachment_error_m,
            ),
            Check::at_least(
                "payload_above_freefall_m",
                scalar(
                    world.bodies[bodies[0]].translation().y
                        - world.bodies[bodies[1]].translation().y,
                ),
                config.acceptance.min_payload_freefall_difference_m,
            ),
            Check::at_least(
                "rope_tip_horizontal_motion_m",
                max_x,
                config.acceptance.min_payload_rope_horizontal_motion_m,
            ),
            Check::at_least(
                "upward_native_last_substep_impulse_ns",
                upward,
                config.acceptance.min_payload_upward_attachment_impulse_ns,
            ),
        ]),
        _ => {
            checks.extend([
                Check::at_least(
                    "active_contact_steps",
                    contacts as f64,
                    config.acceptance.min_obstacle_contact_steps as f64,
                ),
                Check::at_most(
                    "max_capsule_box_penetration_m",
                    -min_dist,
                    config.acceptance.max_obstacle_penetration_m,
                ),
                Check::at_most(
                    "final_capsule_box_gap_m",
                    last_dist.max(0.0),
                    config.acceptance.max_settled_obstacle_gap_m,
                ),
            ]);
            if case == "obstacle_middle" {
                checks.push(Check::at_least(
                    "wire_midpoint_contact_with_endpoints_clear_steps",
                    middle_only as f64,
                    1.0,
                ));
            }
        }
    }
    let mut bytes = vec![];
    track.write_json(&mut bytes)?;
    let read = RopeTrack::read_json(bytes.as_slice())?;
    checks.push(Check::condition(
        "track_round_trip_first_last_radius_name_events",
        read.frames().len() == track.frames().len()
            && read.events().len() == track.events().len()
            && read.frames().first().unwrap().capture.time_s.value() == 0.0
            && read.frames().last().unwrap().capture.step == Some((steps - 1) as u64)
            && read.frames().last().unwrap().ropes[0].positions_m
                == track.frames().last().unwrap().ropes[0].positions_m
            && read.frames()[0].ropes[0].name == format!("{case} cable")
            && read.frames()[0].ropes[0].radius_m == track.frames()[0].ropes[0].radius_m,
    ));
    Ok((track, checks))
}
pub fn example(case: &str) -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() > 1 {
        return Err("usage: example [output.json]".into());
    }
    let config = Config::load(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/native/baseline.json"),
    )?;
    let (track, checks) = run(case, &config)?;
    if let Some(path) = args.first() {
        let path = Path::new(path);
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            std::fs::create_dir_all(parent)?;
        }
        track.write_json(std::fs::File::create(path)?)?;
    } else {
        track.write_json(std::io::stdout())?;
    }
    if !checks.iter().all(|c| c.passed) {
        return Err(format!("track scene failed: {checks:?}").into());
    }
    Ok(())
}
