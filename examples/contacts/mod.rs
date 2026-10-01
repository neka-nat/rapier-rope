//! Bounded contact scenes; experimental harness, not an alternative physics solver.
use crate::common::{self, Check, Result, config::Config, real, scalar, xyz};
use rapier_rope::{
    rapier::{parry, prelude::*},
    *,
};
use serde::Deserialize;
use serde_json::{Value, json};
use std::path::Path;

pub const ID: WorldId = WorldId(5);
pub const CONTACT_CASES: [&str; 3] = ["floor", "self_contact", "two_ropes"];
#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContactSettings {
    pub schema_version: u32,
    pub contact_steps: usize,
    pub contact_radius_m: f64,
    pub max_contact_penetration_m: f64,
    pub max_final_contact_gap_m: f64,
    pub min_contact_steps: usize,
    pub min_control_shape_difference_m: f64,
}
impl ContactSettings {
    pub fn load() -> Result<Self> {
        let root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let q: Self = serde_json::from_slice(&std::fs::read(
            root.join("tests/fixtures/native/contacts.json"),
        )?)?;
        if q.schema_version != 1
            || q.contact_steps == 0
            || q.contact_steps > 10000
            || q.min_contact_steps == 0
            || [
                q.contact_radius_m,
                q.max_contact_penetration_m,
                q.max_final_contact_gap_m,
                q.min_control_shape_difference_m,
            ]
            .iter()
            .any(|x| !x.is_finite() || *x <= 0.0)
        {
            return Err("invalid contact-test input".into());
        }
        Ok(q)
    }
}
pub fn spec(config: &Config, name: &str, points: Vec<Point3>) -> RopeSpec {
    let mut spec = RopeSpec::new(
        name,
        points,
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
    spec.dynamics.additional_solver_iterations = config.additional_solver_iterations;
    spec.dynamics.additional_pgs_iterations = config.additional_pgs_iterations;
    spec
}
pub fn capture(
    track: &mut RopeTrack,
    ropes: &RopeSet,
    world: &PhysicsWorld,
    handles: &[RopeHandle],
    stamp: CaptureStamp,
) -> Result<()> {
    track.push_frame(TrackFrame {
        capture: stamp.clone(),
        ropes: handles
            .iter()
            .map(|&h| ropes.centerline(ID, world, h)?.snapshot(stamp.clone()))
            .collect::<std::result::Result<_, OutputError>>()?,
        bodies: vec![],
    })?;
    Ok(())
}
fn distance_between(
    sb: &SoftBody,
    edges: &[[u32; 2]],
    other: &SoftBody,
    other_edges: &[[u32; 2]],
) -> f64 {
    let mut gap = f64::MAX;
    for &[a, b] in edges {
        let capsule = parry::shape::Capsule::new(
            sb.particle_position(a as usize),
            sb.particle_position(b as usize),
            sb.particle_radius(),
        );
        for &[c, d] in other_edges {
            let capsule2 = parry::shape::Capsule::new(
                other.particle_position(c as usize),
                other.particle_position(d as usize),
                other.particle_radius(),
            );
            // Signed capsule/capsule distance, including negative penetration.
            if let Some(contact) = parry::query::contact(
                &Pose::IDENTITY,
                &capsule,
                &Pose::IDENTITY,
                &capsule2,
                real(100.0),
            )
            .expect("capsule query supported")
            {
                gap = gap.min(scalar(contact.dist));
            }
        }
    }
    gap
}
pub fn contact(
    case: &str,
    config: &Config,
    q: &ContactSettings,
    enabled: bool,
) -> Result<(RopeTrack, Value)> {
    if !CONTACT_CASES.contains(&case) {
        return Err("unknown contact case".into());
    }
    let mut world = common::scenes::world(config);
    let mut ropes = RopeSet::new(ID)?;
    let mut a = match case {
        "self_contact" => spec(
            config,
            "folded cable",
            vec![
                [-0.5, 0.2, 0.0],
                [0.5, 0.2, 0.0],
                [0.5, 0.4, 0.003],
                [-0.5, 0.4, 0.003],
            ],
        ),
        _ => spec(
            config,
            "lower cable",
            vec![[-0.5, 0.2, 0.0], [0.5, 0.2, 0.0]],
        ),
    };
    a.collision.radius_m = q.contact_radius_m;
    a.collision.self_contacts = case == "self_contact" && enabled;
    if case == "two_ropes" && !enabled {
        a.collision.groups = CollisionGroups {
            memberships: 1,
            filter: 1,
        };
    }
    let lower = ropes.insert(ID, &mut world, &a)?;
    let mut handles = vec![lower];
    if case == "two_ropes" {
        let mut b = spec(
            config,
            "falling cable",
            vec![[0.0, 0.4, -0.5], [0.0, 0.4, 0.5]],
        );
        b.collision.radius_m = q.contact_radius_m;
        if !enabled {
            b.collision.groups = CollisionGroups {
                memberships: 2,
                filter: 2,
            };
        }
        handles.push(ropes.insert(ID, &mut world, &b)?);
    }
    let native: Vec<_> = handles
        .iter()
        .map(|&h| ropes.get(ID, &world, h).unwrap().native_handle)
        .collect();
    let samples = ropes.get(ID, &world, lower)?.samples;
    // Only the first straight metre is pinned. The distant strand of the same
    // rope remains free; the join is not part of the measured self-contact gap.
    let pinned: Vec<_> = if case == "floor" {
        vec![]
    } else {
        samples
            .arc_lengths_m()
            .iter()
            .enumerate()
            .filter(|(_, s)| **s <= 1.0 + 1e-9)
            .map(|(i, _)| (i, xyz(world.soft_bodies[native[0]].particle_position(i))))
            .collect()
    };
    let all_edges: Vec<Vec<[u32; 2]>> = handles
        .iter()
        .map(|&h| {
            ropes
                .get(ID, &world, h)
                .unwrap()
                .samples
                .wire_segments()
                .to_vec()
        })
        .collect();
    let lower_edges: Vec<_> = all_edges[0]
        .iter()
        .copied()
        .filter(|e| samples.arc_lengths_m()[e[1] as usize] <= 1.0 + 1e-9)
        .collect();
    let upper_edges: Vec<_> = if case == "self_contact" {
        let upper_start = 1.0 + (0.2_f64.powi(2) + 0.003_f64.powi(2)).sqrt();
        all_edges[0]
            .iter()
            .copied()
            .filter(|e| {
                samples.arc_lengths_m()[e[0] as usize] >= upper_start - 1e-9
            // Exclude the join-side 10cm, which shares the connecting material.
            && world.soft_bodies[native[0]].particle_position(e[0] as usize).x < real(0.4)
            })
            .collect()
    } else {
        vec![]
    };
    let floor = if case == "floor" {
        Some(
            world.insert_collider(
                ColliderBuilder::cuboid(2.0, 0.05, 2.0)
                    .translation(Vector::new(0.0, -0.05, 0.0))
                    .friction(real(config.friction)),
                None,
            ),
        )
    } else {
        None
    };
    let dt = scalar(world.integration_parameters.dt);
    let scene = if enabled {
        case.into()
    } else {
        format!("{case}_disabled_control")
    };
    let mut track = RopeTrack::new(&scene, "right-handed, Y-up, SI", &world)?;
    for &h in &handles {
        track.register_rope(&ropes.centerline(ID, &world, h)?)?;
    }
    if floor.is_some() {
        let f = |x| FiniteScalar::new(x).unwrap();
        track.add_display_object(DisplayObject {
            name: "floor collider".into(),
            role: "collider".into(),
            body: None,
            shape: DisplayShape::Cuboid {
                half_extents_m: [f(2.0), f(0.05), f(2.0)],
            },
            translation_m: [f(0.0), f(-0.05), f(0.0)],
            rotation_xyzw: [f(0.0), f(0.0), f(0.0), f(1.0)],
        })?;
    }
    let mut min_gap = f64::MAX;
    let mut final_gap = f64::MAX;
    let mut contact_steps = 0;
    let mut first_contact_step = None;
    let mut closest_gap_step = 0;
    let mut witnesses = 0;
    let mut finite = true;
    let mut max_strain = 0.0_f64;
    for step in 0..q.contact_steps {
        let commands: Vec<_> = if step == 0 {
            pinned
                .iter()
                .map(|&(particle, position_m)| RopeCommand::PinAt {
                    rope: lower,
                    particle: particle as u32,
                    position_m,
                })
                .collect()
        } else {
            vec![]
        };
        ropes.prepare(ID, &mut world, step as u64, config.dt_s, &commands)?;
        if step == 0 {
            for &(particle, _) in &pinned {
                track.record_event(TrackEvent {
                    step: 0,
                    time_s: FiniteScalar::new(0.0)?,
                    operation: TrackEventKind::Pin {
                        rope: lower.into(),
                        particle: particle as u32,
                    },
                })?;
            }
            capture(
                &mut track,
                &ropes,
                &world,
                &handles,
                CaptureStamp::new(CapturePhase::Initial, None, 0.0, dt)?,
            )?;
        }
        world.step();
        ropes.inspect(ID, &world, step as u64)?;
        for (i, &h) in handles.iter().enumerate() {
            finite &= common::scenes::finite(&world, native[i]);
            max_strain = max_strain.max(
                diagnose_geometry(
                    ropes.get(ID, &world, h)?.samples,
                    &ropes
                        .centerline(ID, &world, h)?
                        .positions_m()
                        .collect::<Vec<_>>(),
                )?
                .max_tensile_strain
                .value()
                .ok_or("invalid strain")?,
            );
        }
        let count: usize = native
            .iter()
            .map(|&h| {
                world.soft_bodies[h]
                    .edge_contact_segments(&world.soft_bodies)
                    .count()
            })
            .sum();
        witnesses += count;
        let active = if let Some(floor) = floor {
            world.narrow_phase.contact_pairs().any(|p| {
                (p.collider1 == floor || p.collider2 == floor) && p.has_any_active_contact()
            })
        } else if case == "two_ropes" {
            // Fully pinned soft surfaces may use rigid manifolds instead of the
            // owned soft-edge witness buffer. The scene contains only this pair.
            count > 0
                || world
                    .narrow_phase
                    .contact_pairs()
                    .any(|p| p.has_any_active_contact())
        } else {
            count > 0
        };
        if active {
            contact_steps += 1;
            first_contact_step.get_or_insert(step);
        }
        final_gap = match case {
            "floor" => world.soft_bodies[native[0]]
                .particle_positions()
                .map(|p| scalar(p.y) - q.contact_radius_m)
                .fold(f64::MAX, f64::min),
            "self_contact" => distance_between(
                &world.soft_bodies[native[0]],
                &lower_edges,
                &world.soft_bodies[native[0]],
                &upper_edges,
            ),
            _ => distance_between(
                &world.soft_bodies[native[0]],
                &all_edges[0],
                &world.soft_bodies[native[1]],
                &all_edges[1],
            ),
        };
        if final_gap < min_gap {
            min_gap = final_gap;
            closest_gap_step = step;
        }
        if (step + 1) % config.record_every_steps == 0 || step + 1 == q.contact_steps {
            capture(
                &mut track,
                &ropes,
                &world,
                &handles,
                CaptureStamp::new(
                    CapturePhase::AfterStep,
                    Some(step as u64),
                    (step + 1) as f64 * dt,
                    dt,
                )?,
            )?;
        }
    }
    let mut checks = vec![Check::condition("finite_state_and_impulses", finite)];
    if enabled {
        checks.extend([
            Check::at_least(
                "contact_record_steps",
                contact_steps as f64,
                q.min_contact_steps as f64,
            ),
            Check::at_most(
                "max_measured_capsule_penetration_m",
                (-min_gap).max(0.0),
                q.max_contact_penetration_m,
            ),
            Check::at_most(
                "max_tensile_strain",
                max_strain,
                config.acceptance.max_tensile_strain,
            ),
        ]);
        // The folded strand can slide off its support. It is a transient self-
        // contact experiment, whereas floor and crossed ropes are resting cases.
        if case != "self_contact" {
            checks.push(Check::at_most(
                "final_measured_capsule_gap_m",
                final_gap.max(0.0),
                q.max_final_contact_gap_m,
            ));
        }
    }
    let result = json!({"case":case,"enabled":enabled,"steps":q.contact_steps,"rope_count":handles.len(),
        "particles":native.iter().map(|&h| world.soft_bodies[h].num_particles()).collect::<Vec<_>>(),
        "contact_record_kind":if case=="self_contact" {"native last-step edge-contact witness; not an impulse test"}else{"active narrow-phase contact or native edge witness"},
        "contact_steps":contact_steps,"first_contact_step":first_contact_step,"closest_gap_step":closest_gap_step,"edge_contact_witnesses":witnesses,
        "min_capsule_gap_m":min_gap,"final_capsule_gap_m":final_gap,"max_tensile_strain":max_strain,
        "checks":checks,"passed":checks.iter().all(|c|c.passed)});
    let mut bytes = vec![];
    track.write_json(&mut bytes)?;
    let read = RopeTrack::read_json(bytes.as_slice())?;
    if serde_json::to_value(read.frames().last().unwrap())?
        != serde_json::to_value(track.frames().last().unwrap())?
    {
        return Err("contact track round-trip changed final frame".into());
    }
    Ok((track, result))
}
