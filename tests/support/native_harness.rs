//! feasibility check using only upstream native soft-body APIs.
use rapier_rope::rapier::prelude::*;
use serde_json::{Value, json};

#[allow(clippy::unnecessary_cast)]
fn r(v: f64) -> Real {
    v as Real
}
#[allow(clippy::unnecessary_cast)]
fn xyz(p: Vector) -> [f64; 3] {
    [p.x as f64, p.y as f64, p.z as f64]
}

fn builder(
    positions: Vec<Vector>,
    edges: Vec<[u32; 2]>,
    bends: Vec<[u32; 2]>,
    contacts: bool,
) -> SoftBodyBuilder {
    SoftBodyBuilder::new(positions)
        .particle_mass(r(0.003))
        .wire(edges.clone())
        .edges(edges)
        .bend_edges(bends)
        .material(SoftBodyMaterial {
            edge_softness: SpringCoefficients::new(r(500.0), r(1.0)),
            bend_softness: SpringCoefficients::new(r(20.0), r(0.8)),
            ..Default::default()
        })
        .particle_radius(r(0.01))
        .self_contacts(contacts)
        .additional_pgs_iterations(3)
        .can_sleep(false)
}

pub fn response() -> Value {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.dt = r(1.0 / 240.0);
    let points = vec![
        Vector::new(0.0, 1.0, 0.0),
        Vector::new(0.0, 0.75, 0.0),
        Vector::new(0.0, 0.5, 0.0),
        Vector::new(-0.25, 1.2, 0.0),
        Vector::new(-0.5, 1.4, 0.0),
        Vector::new(0.25, 1.2, 0.0),
        Vector::new(0.5, 1.4, 0.0),
    ];
    let edges = vec![[0, 1], [1, 2], [0, 3], [3, 4], [0, 5], [5, 6]];
    let h = world.insert_soft_body(builder(
        points.clone(),
        edges.clone(),
        vec![[0, 2], [0, 4], [0, 6]],
        true,
    ));
    world.soft_bodies[h].set_particle_pinned(2, true);
    world.soft_bodies[h].set_particle_pinned(4, true);
    let mut finite = true;
    for k in 0..240 {
        let u = r((k + 1) as f64 / 240.0);
        world.soft_bodies[h]
            .set_particle_kinematic_target(4, points[4] + Vector::new(-0.2 * u, 0.15 * u, 0.0));
        world.step();
        finite &= world.soft_bodies[h]
            .particle_positions()
            .chain(world.soft_bodies[h].particle_velocities())
            .all(|p| p.is_finite())
            && world.quarantine().is_empty();
    }
    let p = &world.soft_bodies[h];
    let junction_motion = (p.particle_position(0) - points[0]).length();
    let other_branch_motion = (p.particle_position(6) - points[6]).length();
    json!({"case":"native_y_response","particles":7,"structural_edges":edges,"junction_particle":0,
        "junction_motion_m":xyz(Vector::new(junction_motion,0.0,0.0))[0],"other_branch_motion_m":xyz(Vector::new(other_branch_motion,0.0,0.0))[0],
        "finite":finite,"passed":finite && junction_motion>r(0.01) && other_branch_motion>r(0.01)})
}

pub fn contact(enabled: bool) -> (Vec<[f64; 3]>, Value) {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = r(1.0 / 240.0);
    let mut points = vec![Vector::new(0.6, 0.6, 0.0)];
    let mut edges = Vec::new();
    let mut bends = Vec::new();
    let mut lower = Vec::new();
    for (leg, vertices) in [
        vec![Vector::new(0.8, 0.8, 0.0)],
        vec![Vector::new(0.5, 0.2, 0.0), Vector::new(-0.5, 0.2, 0.0)],
        vec![Vector::new(0.5, 0.4, 0.003), Vector::new(-0.5, 0.4, 0.003)],
    ]
    .into_iter()
    .enumerate()
    {
        let mut ids = vec![0];
        let mut start = points[0];
        for end in vertices {
            for k in 1..=10 {
                let id = points.len() as u32;
                points.push(start + (end - start) * r(k as f64 / 10.0));
                edges.push([*ids.last().unwrap(), id]);
                ids.push(id);
            }
            start = end;
        }
        bends.extend(ids.windows(3).map(|p| [p[0], p[2]]));
        if leg == 1 {
            lower = ids;
        }
    }
    let h = world.insert_soft_body(builder(points, edges, bends, enabled));
    for p in lower {
        world.soft_bodies[h].set_particle_pinned(p as usize, true);
    }
    let mut witness_steps = 0;
    let mut witnesses = 0;
    let mut finite = true;
    for _ in 0..480 {
        world.step();
        let count = world.soft_bodies[h]
            .edge_contact_segments(&world.soft_bodies)
            .count();
        witnesses += count;
        witness_steps += usize::from(count > 0);
        finite &= world.soft_bodies[h]
            .particle_positions()
            .chain(world.soft_bodies[h].particle_velocities())
            .all(|p| p.is_finite())
            && world.quarantine().is_empty();
    }
    (
        world.soft_bodies[h].particle_positions().map(xyz).collect(),
        json!({"case":"native_y_branch_contact","enabled":enabled,"finite":finite,"witness_steps":witness_steps,"edge_witnesses":witnesses}),
    )
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let response = response();
    let (on, contact_on) = contact(true);
    let (off, contact_off) = contact(false);
    let delta = on
        .iter()
        .zip(off)
        .map(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2]))
        .fold(0.0, f64::max);
    let passed = response["passed"] == true
        && contact_on["finite"] == true
        && contact_off["finite"] == true
        && contact_on["witness_steps"].as_u64().unwrap() > 0
        && contact_off["edge_witnesses"] == 0
        && delta > 0.005;
    let result = json!({"response":response,"contact_enabled":contact_on,"contact_disabled":contact_off,"control_shape_difference_m":delta,"passed":passed});
    println!("{}", serde_json::to_string_pretty(&result)?);
    if !passed {
        return Err("native Y harness feasibility checks failed".into());
    }
    Ok(())
}
