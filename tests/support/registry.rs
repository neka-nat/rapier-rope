use rapier_rope::{rapier::prelude::*, *};

pub const ID: WorldId = WorldId(17);

pub fn spec(name: &str) -> RopeSpec {
    RopeSpec::new(
        name,
        vec![[0.0, 1.5, 0.0], [1.0, 1.5, 0.0]],
        NativeRopeMaterial::new(
            0.1,
            SpringSettings::new(500.0, 1.0),
            SpringSettings::new(20.0, 0.8),
        ),
        SamplingSettings::new(1.0 / 8.0),
        CollisionSettings::new(0.005),
    )
}

pub fn setup() -> (RopeSet, PhysicsWorld, RopeHandle) {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = 1.0 / 240.0;
    let mut ropes = RopeSet::new(ID).unwrap();
    let rope = ropes.insert(ID, &mut world, &spec("rope")).unwrap();
    (ropes, world, rope)
}

pub fn step(
    ropes: &mut RopeSet,
    world: &mut PhysicsWorld,
    commands: &[RopeCommand],
) -> PreparedStep {
    let n = ropes.next_step();
    let prepared = ropes.prepare(ID, world, n, 1.0 / 240.0, commands).unwrap();
    world.step();
    ropes.inspect(ID, world, n).unwrap();
    prepared
}

pub fn attached(
    ropes: &mut RopeSet,
    world: &mut PhysicsWorld,
    rope: RopeHandle,
) -> (RigidBodyHandle, AttachmentHandle) {
    let body = world.insert_body(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 1.5, 0.0)),
    );
    let prepared = step(
        ropes,
        world,
        &[RopeCommand::Attach {
            rope,
            particle: 0,
            body,
        }],
    );
    (body, prepared.created_attachments[0].handle)
}
