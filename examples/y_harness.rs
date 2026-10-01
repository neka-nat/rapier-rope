//! Y harness with per-span material settings, a moving grasp and a dynamic plug.
use rapier_rope::{rapier::prelude::*, *};
use serde_json::{Value, json};
use std::{fs, path::PathBuf};
#[allow(clippy::unnecessary_cast)]
fn r(x: f64) -> Real {
    x as Real
}
#[allow(clippy::unnecessary_cast)]
fn scalar(x: Real) -> f64 {
    x as f64
}

pub fn spec() -> HarnessSpec {
    let nodes = vec![
        JunctionSpec::new("J", [0.0, 1.5, 0.0]),
        JunctionSpec::new("mount", [0.0, 2.0, 0.0]),
        JunctionSpec::new("left", [-0.5, 1.15, 0.0]),
        JunctionSpec::new("right", [0.5, 1.15, 0.0]),
    ];
    let spans = [
        ("stem", "mount", 0.1, 500.0),
        ("left", "left", 0.2, 600.0),
        ("right", "right", 0.15, 700.0),
    ]
    .into_iter()
    .map(|(name, end, density, hz)| {
        SpanSpec::new(
            name,
            "J",
            end,
            NativeRopeMaterial::new(
                density,
                SpringSettings::new(hz, 1.0),
                SpringSettings::new(20.0, 0.8),
            ),
            SamplingSettings::new(0.04),
        )
    })
    .collect();
    let mut c = CollisionSettings::new(0.005);
    c.self_contacts = true;
    HarnessSpec::new("Y harness", nodes, spans, c)
}
fn location(name: &str) -> HarnessLocation {
    HarnessLocation::Junction(name.into())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let out = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("target/y-harness.json"));
    let dt = 1.0 / 240.0;
    let id = WorldId(7);
    let spec = spec();
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = r(dt);
    world.integration_parameters.num_solver_iterations = 8;
    let mut set = HarnessSet::new(id)?;
    let h = set.insert(id, &mut world, &spec)?;
    let (gripper, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(-0.5, 1.15, 0.0)),
        ColliderBuilder::ball(r(0.015)).collision_groups(InteractionGroups::none()),
    );
    let (plug, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.5, 1.12, 0.02))
            .rotation(Vector::new(0.0, 0.0, 0.3))
            .can_sleep(false),
        ColliderBuilder::ball(r(0.025)).mass(r(0.015)),
    );
    let f = |x| FiniteScalar::new(x).unwrap();
    let objects = vec![
        DisplayObject {
            name: "gripper".into(),
            role: "marker".into(),
            body: Some(gripper.into()),
            shape: DisplayShape::Sphere { radius_m: f(0.015) },
            translation_m: [f(-0.5), f(1.15), f(0.0)],
            rotation_xyzw: [f(0.0), f(0.0), f(0.0), f(1.0)],
        },
        DisplayObject {
            name: "dynamic plug".into(),
            role: "connector".into(),
            body: Some(plug.into()),
            shape: DisplayShape::Sphere { radius_m: f(0.025) },
            translation_m: [f(0.5), f(1.12), f(0.02)],
            rotation_xyzw: [f(0.0), f(0.0), f(0.0), f(1.0)],
        },
    ];
    let view = set.get(id, &world, h)?;
    let span_maps:BTreeMap<String,Value>=view.samples.spans().iter().map(|(n,s)|(n.clone(),json!({"particles":s.particle_indices(),"reference_arc_lengths_m":s.reference().arc_lengths_m(),"structural_edge_range":s.structural_edge_range(),"bend_edge_range":s.bending_edge_range()}))).collect();
    let reference_masses = view.samples.particle_masses_kg().to_vec();
    let nominal_mass = view.samples.nominal_mass_kg();
    let count = reference_masses.len();
    let initial = view.positions_m().collect::<Vec<_>>();
    let j = view.samples.junction_particles()["J"] as usize;
    let mut frames = Vec::new();
    let mut events = Vec::new();
    let mut grasp = None;
    let mut max_strain = 0.0_f64;
    let mut max_anchor = 0.0_f64;
    let mut junction_motion = 0.0_f64;
    for step in 0..480_u64 {
        let commands = if step == 0 {
            vec![
                HarnessCommand::Pin {
                    harness: h,
                    location: location("mount"),
                    position_m: [0.0, 2.0, 0.0],
                },
                HarnessCommand::Attach {
                    harness: h,
                    location: location("left"),
                    body: gripper,
                },
                HarnessCommand::Attach {
                    harness: h,
                    location: location("right"),
                    body: plug,
                },
            ]
        } else if step == 240 {
            vec![HarnessCommand::Detach {
                attachment: grasp.unwrap(),
            }]
        } else {
            vec![]
        };
        let prepared = set.prepare(id, &mut world, step, dt, &commands)?;
        if step == 0 {
            grasp = Some(prepared.created_attachments[0].handle);
            for (kind, particle) in [
                ("pin", prepared.locations[0].location.particle_index),
                ("attach", prepared.created_attachments[0].particle),
                ("attach", prepared.created_attachments[1].particle),
            ] {
                events.push(
                    json!({"step":0,"time_s":0.0,"operation":{"kind":kind,"particle":particle}}),
                );
            }
        }
        if step == 240 {
            events.push(json!({"step":step,"time_s":1.0,"operation":{"kind":"detach"}}));
        }
        if step == 0 || step == 240 {
            let stamp =
                CaptureStamp::new(CapturePhase::BeforeStep, Some(step), step as f64 * dt, dt)?;
            frames.push(json!({"capture":stamp,"harnesses":[set.get(id,&world,h)?.snapshot(stamp.clone())?],"bodies":[BodyPoseSnapshot::capture(&world,gripper)?,BodyPoseSnapshot::capture(&world,plug)?]}));
        }
        let u = ((step + 1) as f64 * dt).min(1.0);
        world.bodies[gripper].set_next_kinematic_position(Pose::from_translation(Vector::new(
            r(-0.5 - 0.12 * u),
            r(1.15 + 0.12 * u),
            0.0,
        )));
        world.step();
        set.inspect(id, &world, step)?;
        let view = set.get(id, &world, h)?;
        let stamp = CaptureStamp::new(
            CapturePhase::AfterStep,
            Some(step),
            (step + 1) as f64 * dt,
            dt,
        )?;
        let snap = view.snapshot(stamp.clone())?;
        for g in snap.span_geometry.values() {
            max_strain = max_strain.max(g.max_tensile_strain.value().ok_or("invalid strain")?);
        }
        for a in &snap.attachments {
            max_anchor = max_anchor.max(a.position_error_m.value().ok_or("invalid anchor")?);
        }
        let p = snap.positions_m[j];
        let q = initial[j];
        junction_motion = junction_motion.max((p[0] - q[0]).hypot(p[1] - q[1]).hypot(p[2] - q[2]));
        if (step + 1) % 4 == 0 || step == 239 {
            frames.push(json!({"capture":stamp,"harnesses":[snap],"bodies":[BodyPoseSnapshot::capture(&world,gripper)?,BodyPoseSnapshot::capture(&world,plug)?]}));
        }
    }
    let plug_y = scalar(world.bodies[plug].translation().y);
    let freefall_y = 1.12 - 0.5 * 9.81 * 4.0;
    let result = json!({"passed":junction_motion>0.01 && max_anchor<0.02 && plug_y>freefall_y+0.5,"particles":count,"nominal_mass_kg":nominal_mass,"connector_mass_kg":scalar(world.bodies[plug].mass()),"connector_final_y_m":plug_y,"freefall_y_at_2s_m":freefall_y,"max_tensile_strain":max_strain,"max_anchor_error_m":max_anchor,"junction_motion_m":junction_motion,"frames":frames.len(),"final_time_s":2.0});
    let track = json!({"schema_version":1,"kind":"rapier_harness_example","purpose":"playback_only_not_solver_checkpoint","scene":"y_harness","coordinate_system":"right-handed Y-up SI","units":{"length":"m","time":"s","mass":"kg","impulse":"N s"},"precision":if cfg!(feature="f32"){"f32"}else{"f64"},"rapier_version":"0.36.0","package_version":env!("CARGO_PKG_VERSION"),"definition":spec,"reference_particle_masses_kg":reference_masses,"span_maps":span_maps,"world_settings":{"dt_s":scalar(world.integration_parameters.dt),"gravity_m_s2":[0.0,-9.81,0.0],"solver_iterations":8},"display_objects":objects,"events":events,"frames":frames});
    if let Some(parent) = out.parent()
        && !parent.as_os_str().is_empty()
    {
        fs::create_dir_all(parent)?;
    }
    fs::write(&out, serde_json::to_vec(&track)?)?;
    fs::write(
        out.with_extension("summary.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    println!("{}", serde_json::to_string(&result)?);
    if result["passed"] != true {
        return Err("Y replay acceptance conditions failed".into());
    }
    Ok(())
}
use std::collections::BTreeMap;
