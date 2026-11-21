// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A single link-origin controller moves owned primitive children through real physics.
use gz_physics::{BodyId, BodySpec, BodyState, PhysicsBackend, PhysicsError, RotationSpec, Shape};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, InputError, LoadError, LocalPose, Name, PhysicsEngine, VelocityControlCommandError,
    VelocityControlConfiguration, WorldEditError, WorldPose,
    initial_velocity_control_configurations, load_world_with_sdf_step, model_state,
    queue_link_world_velocity, queue_link_world_wrench, remove_model, reset_loaded_world,
    set_velocity_control_command, velocity_control_binding_matches,
    velocity_control_binding_matches_configuration,
};
use sdformat::NoResources;
use std::collections::BTreeMap;
use std::time::Duration;

const MODEL: &str = "1 2 10 1.5707963267948966 0 1.5707963267948966";
const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>3</ixx><iyy>3</iyy><izz>3</izz></inertia></inertial>";
const ANISOTROPIC: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial>";
const SURVIVOR: &str = "<model name='survivor'><pose>100 0 10 0 0 0</pose><link name='body'><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model>";
const GEOMETRY: [&str; 4] = [
    "<box><size>1 0.5 1.5</size></box>",
    "<cylinder><radius>0.5</radius><length>3</length></cylinder>",
    "<capsule><radius>0.5</radius><length>1.5</length></capsule>",
    "<sphere><radius>0.5</radius></sphere>",
];
fn collision(name: &str, pose: &str, geometry: &str) -> String {
    format!(
        "<collision name='{name}'><pose>{pose}</pose><density>7</density><geometry>{geometry}</geometry></collision>"
    )
}
fn child(index: usize) -> String {
    let rpy = [
        "1.5707963267948966 0 0",
        "0 1.5707963267948966 0",
        "-1.5707963267948966 0 0",
        "0 0 0.6",
    ][index % 4];
    collision(
        &format!("c{index}"),
        &format!("{} 0 0 {rpy}", index as f64 * 6. - 6.),
        GEOMETRY[index % 4],
    )
}
fn children(count: usize) -> String {
    (0..count).map(child).collect()
}
fn rig(collisions: &str, pose: &str, linear: &str, angular: &str) -> String {
    format!(
        "<model name='rig'><pose>{pose}</pose><plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'><initial_linear>{linear}</initial_linear><initial_angular>{angular}</initial_angular></plugin><link name='body'>{INERTIA}{collisions}<visual name='view'><pose>0 0.5 1 0 0 0.2</pose><geometry><box><size>0.2 0.2 0.2</size></box></geometry></visual><sensor name='imu' type='imu'><always_on>true</always_on><update_rate>1000</update_rate></sensor></link></model>"
    )
}
fn world(models: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}
fn fixture(count: usize, linear: &str, angular: &str) -> String {
    world(&(rig(&children(count), MODEL, linear, angular) + SURVIVOR))
}
fn load(xml: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        xml,
        "compound-velocity.sdf",
        "default",
        &NoResources,
        engine,
    )
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn handle(sim: &Simulation, name: &str) -> BodyId {
    sim.world()
        .get::<BodyHandle>(entity_by_id(sim.world(), id(sim, name)).unwrap())
        .unwrap()
        .id()
}
fn state(sim: &Simulation, name: &str) -> BodyState {
    sim.world()
        .resource::<PhysicsBackend>()
        .state(handle(sim, name))
        .unwrap()
}
fn pose(sim: &Simulation, name: &str) -> WorldPose {
    *sim.world()
        .get::<WorldPose>(entity_by_id(sim.world(), id(sim, name)).unwrap())
        .unwrap()
}
fn metadata(sim: &Simulation) -> BTreeMap<String, u64> {
    sim.world()
        .iter_entities()
        .filter_map(|e| Some((e.get::<Name>()?.0.clone(), e.get::<EntityId>()?.get())))
        .collect()
}
fn near(a: [f64; 3], b: [f64; 3]) {
    for (a, b) in a.into_iter().zip(b) {
        assert!(
            a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
            "{a} != {b}"
        );
    }
}
fn scalar(a: f64, b: f64) {
    assert!(
        a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
        "{a} != {b}"
    );
}
fn rotation(a: [f64; 4], b: [f64; 4]) {
    scalar(
        a.into_iter().zip(b).map(|(x, y)| x * y).sum::<f64>().abs(),
        1.,
    );
}
fn same_state(a: BodyState, b: BodyState) {
    near(a.position, b.position);
    near(a.velocity, b.velocity);
    near(a.angular_velocity, b.angular_velocity);
    rotation(a.orientation, b.orientation);
}
// Independent quaternion matrix and Hamilton product, not loader/controller helpers.
fn rotate([x, y, z, w]: [f64; 4], [a, b, c]: [f64; 3]) -> [f64; 3] {
    [
        (1. - 2. * (y * y + z * z)) * a + 2. * (x * y - z * w) * b + 2. * (x * z + y * w) * c,
        2. * (x * y + z * w) * a + (1. - 2. * (x * x + z * z)) * b + 2. * (y * z - x * w) * c,
        2. * (x * z - y * w) * a + 2. * (y * z + x * w) * b + (1. - 2. * (x * x + y * y)) * c,
    ]
}
fn product([x, y, z, w]: [f64; 4], [a, b, c, d]: [f64; 4]) -> [f64; 4] {
    [
        w * a + x * d + y * c - z * b,
        w * b - x * c + y * d + z * a,
        w * c + x * b - y * a + z * d,
        w * d - x * a - y * b - z * c,
    ]
}
fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] + b[i])
}
fn child_frame(index: usize) -> ([f64; 3], [f64; 4]) {
    let q = std::f64::consts::FRAC_1_SQRT_2;
    let orientation = match index % 4 {
        0 => [q, 0., 0., q],
        1 => [0., q, 0., q],
        2 => [-q, 0., 0., q],
        _ => [0., 0., 0.3f64.sin(), 0.3f64.cos()],
    };
    ([index as f64 * 6. - 6., 0., 0.], orientation)
}
fn query(
    sim: &Simulation,
    origin: [f64; 3],
    direction: [f64; 3],
    maximum: f64,
    exclude: Option<BodyId>,
) -> Result<Option<f64>, PhysicsError> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(origin, direction, maximum, exclude)
}
fn chord(sim: &Simulation) {
    let link = state(sim, "default::rig::body");
    let (p, q) = child_frame(1);
    let origin = add(
        link.position,
        rotate(link.orientation, add(p, rotate(q, [0.3, 4., 0.]))),
    );
    let direction = rotate(link.orientation, rotate(q, [0., -1., 0.]));
    scalar(
        query(sim, origin, direction, 5., None).unwrap().unwrap(),
        3.6,
    );
}

#[test]
fn compound_velocity_two_four_and_sixteen_members_keep_centered_six_axis_trajectories() {
    for count in [2, 4, 16] {
        let source = fixture(count, "1 2 3", "0.1 0.2 0.3");
        let centered = world(
            &(rig(
                &collision("center", "0 0 0 0 0 0", GEOMETRY[3]),
                MODEL,
                "1 2 3",
                "0.1 0.2 0.3",
            ) + SURVIVOR),
        );
        let mut compound = load(&source, PhysicsEngine::Rapier).unwrap();
        let mut control = load(&centered, PhysicsEngine::Rapier).unwrap();
        let owner = id(&compound, "default::rig");
        let other_owner = id(&control, "default::rig");
        assert!(velocity_control_binding_matches(&compound, owner));
        rotation(pose(&compound, "default::rig").rotation, [0.5; 4]);
        for (sim, owner) in [(&mut compound, owner), (&mut control, other_owner)] {
            set_velocity_control_command(sim, owner, [-0.5, 1., -1.5], [0.3, -0.2, 0.1]).unwrap();
            sim.step_exact(1).unwrap();
            // Rz90*Rx90 cycles X->Y, Y->Z, Z->X; child rotations must not enter the command.
            near(state(sim, "default::rig::body").velocity, [3., 1., 2.]);
            near(
                state(sim, "default::rig::body").angular_velocity,
                [0.3, 0.1, 0.2],
            );
            near(
                state(sim, "default::rig::body").position,
                [1.003, 2.001, 10.002],
            );
        }
        same_state(
            state(&compound, "default::rig::body"),
            state(&control, "default::rig::body"),
        );
        for _ in 0..20 {
            let before = state(&compound, "default::rig::body");
            let linear = rotate(before.orientation, [-0.5, 1., -1.5]);
            let angular = rotate(before.orientation, [0.3, -0.2, 0.1]);
            compound.step_exact(1).unwrap();
            control.step_exact(1).unwrap();
            near(state(&compound, "default::rig::body").velocity, linear);
            near(
                state(&compound, "default::rig::body").angular_velocity,
                angular,
            );
            near(
                state(&compound, "default::rig::body").position,
                add(before.position, linear.map(|x| x * 0.001)),
            );
            same_state(
                state(&compound, "default::rig::body"),
                state(&control, "default::rig::body"),
            );
        }
        let before = state(&compound, "default::rig::body");
        let target = rotate(before.orientation, [-0.5, 1., -1.5]);
        let link = id(&compound, "default::rig::body");
        let other_link = id(&control, "default::rig::body");
        queue_link_world_wrench(&mut compound, link, [2., -4., 6.], [0.; 3]).unwrap();
        queue_link_world_wrench(&mut control, other_link, [2., -4., 6.], [0.; 3]).unwrap();
        compound.step_exact(1).unwrap();
        control.step_exact(1).unwrap();
        near(
            state(&compound, "default::rig::body").velocity,
            add(target, [0.001, -0.002, 0.003]),
        );
        same_state(
            state(&compound, "default::rig::body"),
            state(&control, "default::rig::body"),
        );
    }
    // A principal-axis torque independently discriminates explicit anisotropic inertia.
    let source = fixture(4, "0 0 0", "0 0 0").replace(INERTIA, ANISOTROPIC);
    let mut sim = load(&source, PhysicsEngine::Rapier).unwrap();
    let link = id(&sim, "default::rig::body");
    queue_link_world_wrench(&mut sim, link, [2., 0., 0.], [0., 2., 0.]).unwrap();
    sim.step_exact(1).unwrap();
    near(state(&sim, "default::rig::body").velocity, [0.001, 0., 0.]);
    near(
        state(&sim, "default::rig::body").angular_velocity,
        [0., 0.001, 0.],
    );
}

#[test]
fn compound_velocity_advects_child_frames_strict_curved_rays_and_concave_queries() {
    let mut sim = load(&fixture(4, "1 2 3", "0.1 0.2 0.3"), PhysicsEngine::Rapier).unwrap();
    let original_ids = metadata(&sim);
    let owned = handle(&sim, "default::rig::body");
    for _ in 0..5 {
        let body = state(&sim, "default::rig::body");
        near(pose(&sim, "default::rig").translation, body.position);
        rotation(pose(&sim, "default::rig").rotation, body.orientation);
        for i in 0..4 {
            let (position, orientation) = child_frame(i);
            let name = format!("default::rig::body::c{i}");
            let expected = add(body.position, rotate(body.orientation, position));
            near(pose(&sim, &name).translation, expected);
            rotation(
                pose(&sim, &name).rotation,
                product(body.orientation, orientation),
            );
            let local = sim
                .world()
                .get::<LocalPose>(entity_by_id(sim.world(), id(&sim, &name)).unwrap())
                .unwrap()
                .0;
            near(local.translation, position);
            rotation(local.rotation, orientation);
            let (origin, direction, distance) = match i {
                0 => ([0., 0., 3.], [0., 0., -1.], 2.25),
                1 | 3 => ([0.3, 4., 0.], [0., -1., 0.], 3.6),
                _ => ([0., 0., 3.], [0., 0., -1.], 1.75),
            };
            let origin = add(
                expected,
                rotate(body.orientation, rotate(orientation, origin)),
            );
            let direction = rotate(body.orientation, rotate(orientation, direction));
            scalar(
                query(&sim, origin, direction, 5., None).unwrap().unwrap(),
                distance,
            );
            assert_eq!(
                query(&sim, origin, direction, 5., Some(owned)).unwrap(),
                None
            );
        }
        chord(&sim);
        near(pose(&sim, "default::rig::body").translation, body.position);
        rotation(pose(&sim, "default::rig::body").rotation, body.orientation);
        near(
            pose(&sim, "default::rig::body::imu").translation,
            body.position,
        );
        rotation(
            pose(&sim, "default::rig::body::imu").rotation,
            body.orientation,
        );
        near(
            pose(&sim, "default::rig::body::view").translation,
            add(body.position, rotate(body.orientation, [0., 0.5, 1.])),
        );
        rotation(
            pose(&sim, "default::rig::body::view").rotation,
            product(body.orientation, [0., 0., 0.1f64.sin(), 0.1f64.cos()]),
        );
        let gap = add(body.position, rotate(body.orientation, [3., 0., 3.]));
        let down = rotate(body.orientation, [0., 0., -1.]);
        assert_eq!(query(&sim, gap, down, 6., None).unwrap(), None);
        assert_eq!(metadata(&sim), original_ids);
        sim.step_exact(1).unwrap();
    }
    let geometry = collision("box", "2 0 0 0 0 0", "<box><size>1 1 1</size></box>")
        + &collision(
            "remote",
            "-3 0 0 0 0 0",
            "<sphere><radius>0.25</radius></sphere>",
        );
    let source = world(
        &(rig(&geometry, "0 0 0 0 0 0", "0 0 0", "0 0 0") + &SURVIVOR.replace("100 0 10", "2 0 2")),
    );
    let sim = load(&source, PhysicsEngine::Rapier).unwrap();
    let owner = handle(&sim, "default::rig::body");
    let other = handle(&sim, "default::survivor::body");
    scalar(
        query(&sim, [2., 0., 3.], [0., 0., -1.], 4., None)
            .unwrap()
            .unwrap(),
        0.75,
    );
    scalar(
        query(&sim, [2., 0., 3.], [0., 0., -1.], 4., Some(owner))
            .unwrap()
            .unwrap(),
        0.75,
    );
    assert_eq!(
        query(&sim, [2., 0., 3.], [0., 0., -1.], 2.499999, Some(other)).unwrap(),
        None
    );
    scalar(
        query(&sim, [2., 0., 3.], [0., 0., -1.], 2.5, Some(other))
            .unwrap()
            .unwrap(),
        2.5,
    );
    scalar(
        query(&sim, [2., 0., 0.], [1., 0., 0.], 0.125, Some(other))
            .unwrap()
            .unwrap(),
        0.,
    );
    // A prior inside-sphere hit must not hide another child's unusable cylinder arithmetic.
    let geometry = collision(
        "sphere",
        "0 0 0 0 0 0",
        "<sphere><radius>1</radius></sphere>",
    ) + &collision(
        "cylinder",
        "0 0 0 0 0 0",
        "<cylinder><radius>0.5</radius><length>3</length></cylinder>",
    );
    let sim = load(
        &world(&rig(&geometry, "0 0 0 0 0 0", "0 0 0", "0 0 0")),
        PhysicsEngine::Rapier,
    )
    .unwrap();
    assert_eq!(
        query(&sim, [1e-200, 0., 0.], [1., 0., 0.], 10., None),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(
        query(
            &sim,
            [1e-200, 0., 0.],
            [1., 0., 0.],
            10.,
            Some(handle(&sim, "default::rig::body"))
        )
        .unwrap(),
        None
    );
}

#[test]
fn compound_velocity_pause_latest_zero_and_invalid_commands_preserve_latch_timing() {
    let source = fixture(4, "1 0 0", "0 0 0").replace(MODEL, "1 2 10 0 0 0.7853981633974483");
    let mut sim = load(&source, PhysicsEngine::Rapier).unwrap();
    let owner = id(&sim, "default::rig");
    let link = id(&sim, "default::rig::body");
    let initial = state(&sim, "default::rig::body");
    sim.set_paused(true);
    set_velocity_control_command(&mut sim, owner, [5.; 3], [0.; 3]).unwrap();
    set_velocity_control_command(&mut sim, owner, [0., 2., 0.], [0.; 3]).unwrap();
    for (linear, angular) in [
        ([f64::NAN; 3], [0.; 3]),
        ([0.; 3], [f64::INFINITY; 3]),
        ([f64::MAX, f64::MAX, 0.], [0.; 3]),
    ] {
        assert!(matches!(
            set_velocity_control_command(&mut sim, owner, linear, angular),
            Err(VelocityControlCommandError::InvalidCommand)
        ));
        same_state(state(&sim, "default::rig::body"), initial);
    }
    assert!(matches!(
        queue_link_world_velocity(&mut sim, link, [8.; 3], [0.; 3]),
        Err(InputError::Unsupported(
            "VelocityControl owns link velocity"
        ))
    ));
    // The compound getter remains structurally outside world-edit's singleton domain.
    assert!(matches!(
        model_state(&sim, owner),
        Err(WorldEditError::UnsupportedModel)
    ));
    for _ in 0..3 {
        sim.tick().unwrap();
        same_state(state(&sim, "default::rig::body"), initial);
    }
    assert_eq!(sim.elapsed(), Duration::ZERO);
    sim.step_exact(1).unwrap();
    near(
        state(&sim, "default::rig::body").velocity,
        [
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
            0.,
        ],
    );
    sim.step_exact(1).unwrap();
    near(
        state(&sim, "default::rig::body").velocity,
        [-2f64.sqrt(), 2f64.sqrt(), 0.],
    );
    set_velocity_control_command(&mut sim, owner, [0.; 3], [0.; 3]).unwrap();
    sim.tick().unwrap();
    sim.step_exact(1).unwrap();
    near(
        state(&sim, "default::rig::body").velocity,
        [-2f64.sqrt(), 2f64.sqrt(), 0.],
    );
    sim.step_exact(1).unwrap();
    near(state(&sim, "default::rig::body").velocity, [0.; 3]);
    let stopped = state(&sim, "default::rig::body");
    sim.step_exact(3).unwrap();
    same_state(state(&sim, "default::rig::body"), stopped);
    assert!(velocity_control_binding_matches(&sim, owner));
}

#[test]
fn compound_velocity_ordinary_prepared_and_loaded_resets_preserve_owned_geometry_and_ids() {
    let mut sim = load(&fixture(4, "1 0 0", "0 0 0"), PhysicsEngine::Rapier).unwrap();
    let original_ids = metadata(&sim);
    let owner = id(&sim, "default::rig");
    let link = id(&sim, "default::rig::body");
    let survivor = id(&sim, "default::survivor::body");
    let owned = handle(&sim, "default::rig::body");
    let survivor_body = handle(&sim, "default::survivor::body");
    let config = initial_velocity_control_configurations(&sim)
        .unwrap()
        .next()
        .unwrap()
        .1
        .clone();
    let initial = sim.world().resource::<PhysicsBackend>().initial_state();
    set_velocity_control_command(&mut sim, owner, [7.; 3], [0.; 3]).unwrap();
    let extra = {
        let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
        let extra = backend
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.25 },
                    position: [200., 0., 10.],
                    velocity: [0.; 3],
                    mass: Some(1.),
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        assert!(backend.remove(owned));
        let prepared = backend.prepare_initial(&initial).unwrap();
        backend.apply_initial(prepared).unwrap();
        assert!(backend.state(extra).is_none());
        extra
    };
    assert_eq!(handle(&sim, "default::rig::body"), owned);
    assert_eq!(handle(&sim, "default::survivor::body"), survivor_body);
    assert!(velocity_control_binding_matches_configuration(
        &sim, owner, &config
    ));
    chord(&sim);
    // Bare backend reconstruction preserves geometry; Scene reset owns target/pending clearing.
    sim.reset().unwrap();
    sim.step_exact(2).unwrap();
    near(state(&sim, "default::rig::body").position, [1., 2.002, 10.]);
    set_velocity_control_command(&mut sim, owner, [8.; 3], [0.; 3]).unwrap();
    sim.step_exact(3).unwrap();
    set_velocity_control_command(&mut sim, owner, [9.; 3], [0.; 3]).unwrap();
    sim.reset().unwrap();
    assert_eq!(metadata(&sim), original_ids);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    chord(&sim);
    sim.step_exact(2).unwrap();
    near(state(&sim, "default::rig::body").velocity, [0., 1., 0.]);
    near(state(&sim, "default::rig::body").position, [1., 2.002, 10.]);
    set_velocity_control_command(&mut sim, owner, [4.; 3], [0.; 3]).unwrap();
    queue_link_world_velocity(&mut sim, survivor, [0., 3., 0.], [0.; 3]).unwrap();
    let epoch = sim.epoch();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.epoch(), epoch + 1);
    assert_eq!(metadata(&sim), original_ids);
    assert_eq!(handle(&sim, "default::rig::body"), owned);
    assert_eq!(handle(&sim, "default::survivor::body"), survivor_body);
    assert!(velocity_control_binding_matches_configuration(
        &sim, owner, &config
    ));
    sim.step_exact(2).unwrap();
    near(state(&sim, "default::rig::body").position, [1., 2.002, 10.]);
    near(state(&sim, "default::survivor::body").velocity, [0.; 3]);
    chord(&sim);
    let removed = remove_model(&mut sim, owner).unwrap();
    assert!(removed.contains(&owner) && removed.contains(&link));
    assert!(!velocity_control_binding_matches(&sim, owner));
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(owned)
            .is_none()
    );
    assert!(matches!(
        set_velocity_control_command(&mut sim, owner, [1.; 3], [0.; 3]),
        Err(VelocityControlCommandError::MissingController(_))
    ));
    let before = state(&sim, "default::survivor::body");
    queue_link_world_velocity(&mut sim, survivor, [0., 1., 0.], [0.; 3]).unwrap();
    sim.step_exact(3).unwrap();
    near(
        state(&sim, "default::survivor::body").position,
        add(before.position, [0., 0.003, 0.]),
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(metadata(&sim), original_ids);
    assert_eq!(handle(&sim, "default::rig::body"), owned);
    assert!(velocity_control_binding_matches_configuration(
        &sim, owner, &config
    ));
    chord(&sim);
    let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
    let next = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.25 },
                position: [200., 0., 10.],
                velocity: [0.; 3],
                mass: Some(1.),
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    assert!(next.raw() > extra.raw());
    assert!(backend.remove(next));
}

#[test]
fn compound_velocity_real_contacts_move_an_unowned_ball_where_centered_members_miss() {
    let ball = "<model name='ball'><pose>2 0 0.9 0 0 0</pose><link name='body'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model>";
    for (x, contact) in [(2., true), (0., false)] {
        let geometry = collision(
            "support",
            &format!("{x} 0 0.25 0 0 0"),
            "<box><size>1 1 0.5</size></box>",
        ) + &collision(
            "remote",
            "-3 0 0 0 0 0",
            "<sphere><radius>0.25</radius></sphere>",
        );
        let mut sim = load(
            &world(&(rig(&geometry, "0 0 0 0 0 0", "0 0 0", "0 0 0") + ball)),
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let owner = id(&sim, "default::rig");
        let other = handle(&sim, "default::ball::body");
        let before = state(&sim, "default::ball::body");
        let hit = query(&sim, [2., 0., 3.], [0., 0., -1.], 4., Some(other)).unwrap();
        if contact {
            scalar(hit.unwrap(), 2.5);
        } else {
            assert_eq!(hit, None);
        }
        // The retained 1.5 s observation window is selected before running either input.
        sim.step_exact(1500).unwrap();
        if contact {
            assert!(state(&sim, "default::ball::body").position[2] > before.position[2] + 1e-7);
        } else {
            same_state(state(&sim, "default::ball::body"), before);
        }
        let native = state(&sim, "default::rig::body");
        near(pose(&sim, "default::rig").translation, native.position);
        rotation(pose(&sim, "default::rig").rotation, native.orientation);
        assert!(velocity_control_binding_matches(&sim, owner));
    }
}

#[test]
fn compound_velocity_binding_drift_never_redirects_commands_or_releases_owned_velocity() {
    for fault in 0..3 {
        let mut sim = load(&fixture(4, "1 0 0", "0 0 0"), PhysicsEngine::Rapier).unwrap();
        let owner = id(&sim, "default::rig");
        let link = id(&sim, "default::rig::body");
        let other = id(&sim, "default::survivor::body");
        let before = state(&sim, "default::rig::body");
        let other_before = state(&sim, "default::survivor::body");
        let entity = entity_by_id(sim.world(), owner).unwrap();
        let mut config = sim
            .world()
            .get::<VelocityControlConfiguration>(entity)
            .unwrap()
            .clone();
        match fault {
            0 => {
                config.link = other;
                sim.world_mut().entity_mut(entity).insert(config);
            }
            1 => {
                sim.world_mut()
                    .entity_mut(entity)
                    .remove::<VelocityControlConfiguration>();
            }
            _ => {
                let entity = entity_by_id(sim.world(), link).unwrap();
                sim.world_mut()
                    .get_mut::<LocalPose>(entity)
                    .unwrap()
                    .0
                    .rotation = [0., 0., 0.05f64.sin(), 0.05f64.cos()];
            }
        }
        assert!(!velocity_control_binding_matches(&sim, owner));
        assert!(matches!(
            set_velocity_control_command(&mut sim, owner, [8.; 3], [0.; 3]),
            Err(VelocityControlCommandError::MissingController(_))
        ));
        assert!(matches!(
            queue_link_world_velocity(&mut sim, link, [9.; 3], [0.; 3]),
            Err(InputError::Unsupported(
                "VelocityControl owns link velocity"
            ))
        ));
        assert!(sim.step_exact(1).is_err());
        same_state(state(&sim, "default::rig::body"), before);
        same_state(state(&sim, "default::survivor::body"), other_before);
    }
}

#[test]
fn compound_velocity_closed_admission_checks_late_children_and_preserves_existing_worlds() {
    let valid = fixture(16, "1 0 0", "0 0 0");
    let mut existing = load(&valid, PhysicsEngine::Rapier).unwrap();
    let owner = id(&existing, "default::rig");
    let before = state(&existing, "default::rig::body");
    let before_ids = metadata(&existing);
    set_velocity_control_command(&mut existing, owner, [0., 2., 0.], [0.; 3]).unwrap();
    let last = child(15);
    assert_eq!(valid.matches(&last).count(), 1);
    let mut invalid=vec![
        (fixture(0,"1 0 0","0 0 0"),"empty"),
        (fixture(17,"1 0 0","0 0 0"),"seventeen"),
        (valid.replace(&last,&last.replace(GEOMETRY[3],"<mesh><uri>missing.gltf</uri></mesh>")),"last mesh"),
        (valid.replace(&last,&last.replace(GEOMETRY[3],"<plane><normal>0 0 1</normal><size>2 2</size></plane>")),"last plane"),
        (valid.replace(&last,&last.replace("<radius>0.5</radius>","<radius>1e308</radius>")),"last unrepresentable sphere"),
        (valid.replacen("<link name='body'>","<link name='body'><pose>0.1 0 0 0 0 0</pose>",1),"translated link"),
        (valid.replacen("<link name='body'>","<link name='body'><pose>0 0 0 0 0 0.1</pose>",1),"rotated link"),
        (valid.replace("<inertial>","<inertial><pose>0.1 0 0 0 0 0</pose>"),"offset COM"),
        (valid.replacen("<model name='rig'>","<model name='rig'><static>true</static>",1),"static owner"),
        (valid.replacen("</model>","<joint name='hinge' type='revolute'><parent>world</parent><child>body</child><axis><xyz>0 0 1</xyz></axis></joint></model>",1),"joint owner"),
        (valid.replacen("</model>","<model name='nested'/></model>",1),"nested owner"),
        (valid.replace(INERTIA,"<inertial auto='true'/>"),"upstream compound automatic inertia"),
    ];
    let second = rig(&children(2), "50 0 10 0 0 0", "0 0 0", "0 0 0").replacen(
        "name='rig'",
        "name='second'",
        1,
    );
    invalid.push((
        valid.replace("</world>", &format!("{second}</world>")),
        "second velocity owner",
    ));
    for (source, label) in invalid {
        assert_ne!(source, valid, "{label}");
        let result = load(&source, PhysicsEngine::Rapier);
        if matches!(label, "last mesh" | "last plane") {
            assert!(
                matches!(
                    result,
                    Err(LoadError::Unsupported {
                        feature: "VelocityControl centered primitive",
                        ..
                    })
                ),
                "{label}"
            );
        } else {
            assert!(result.is_err(), "{label}");
        }
        same_state(state(&existing, "default::rig::body"), before);
        assert_eq!(metadata(&existing), before_ids);
        assert!(velocity_control_binding_matches(&existing, owner));
    }
    assert!(load(&valid, PhysicsEngine::Tpe).is_err());
    let centered = world(&rig(
        &collision("only", "0 0 0 0 0 0", GEOMETRY[3]),
        MODEL,
        "0 0 0",
        "0 0 0",
    ));
    assert!(load(&centered, PhysicsEngine::Tpe).is_ok());
    let defaulted = fixture(2, "0 0 0", "0 0 0").replace(INERTIA, "");
    let defaults = load(&defaulted, PhysicsEngine::Rapier).unwrap();
    assert!(velocity_control_binding_matches(
        &defaults,
        id(&defaults, "default::rig")
    ));
    existing.step_exact(1).unwrap();
    near(
        state(&existing, "default::rig::body").velocity,
        [0., 1., 0.],
    );
    existing.step_exact(1).unwrap();
    near(
        state(&existing, "default::rig::body").velocity,
        [0., 0., 2.],
    );
    let reconstructed = load(&valid, PhysicsEngine::Rapier).unwrap();
    assert_eq!(metadata(&reconstructed), before_ids);
}
