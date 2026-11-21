// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Model-frame commands keep their link origin while posed geometry follows physics.
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, InputError, LoadError, LocalPose, Name, PhysicsEngine, Velocity,
    VelocityControlCommandError, VelocityControlConfiguration, WorldPose,
    initial_velocity_control_configurations, load_world_with_sdf_step, model_state,
    queue_link_world_velocity, queue_link_world_wrench, remove_model, reset_loaded_world,
    set_model_state, set_velocity_control_command, velocity_control_binding_matches,
    velocity_control_binding_matches_configuration,
};
use sdformat::NoResources;
use std::time::Duration;

const SHAPES: [&str; 4] = [
    "<box><size>0.5 1 1.5</size></box>",
    "<cylinder><radius>0.5</radius><length>1.5</length></cylinder>",
    "<capsule><radius>0.5</radius><length>1.5</length></capsule>",
    "<sphere><radius>0.5</radius></sphere>",
];
const LOCAL: &str = "2 -0.5 0.25 0 1.5707963267948966 0";
const IDENTITY_POSE: &str = "0 0 0 0 0 0";
const MODEL_POSE: &str = "0 0 10 0 0 1.5707963267948966";
const SURVIVOR: &str = "<model name='survivor'><pose>30 0 10 0 0 0</pose><link name='body'><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='shape'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model>";

fn fixture(shape: &str, local: &str, linear: &str, angular: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='rig'><pose>{MODEL_POSE}</pose><plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'><initial_linear>{linear}</initial_linear><initial_angular>{angular}</initial_angular></plugin><link name='body'><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='shape'><pose>{local}</pose><geometry>{shape}</geometry></collision><visual name='view'><pose>0 0 1 0 0 0</pose><geometry><box><size>0.2 0.2 0.2</size></box></geometry></visual><sensor name='imu' type='imu'><always_on>true</always_on><update_rate>1000</update_rate></sensor></link></model>{SURVIVOR}</world></sdf>"
    )
}

fn load(xml: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(xml, "posed-velocity.sdf", "default", &NoResources, engine)
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
fn pose(sim: &Simulation, name: &str) -> WorldPose {
    *sim.world()
        .get::<WorldPose>(entity_by_id(sim.world(), id(sim, name)).unwrap())
        .unwrap()
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
fn velocity(sim: &Simulation, name: &str) -> Velocity {
    *sim.world()
        .get::<Velocity>(entity_by_id(sim.world(), id(sim, name)).unwrap())
        .unwrap()
}
fn near(a: [f64; 3], b: [f64; 3]) {
    for (a, b) in a.into_iter().zip(b) {
        assert!(
            a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
            "{a} != {b}"
        );
    }
}
fn same_rotation(a: [f64; 4], b: [f64; 4]) {
    let dot: f64 = a.into_iter().zip(b).map(|(a, b)| a * b).sum();
    assert!(
        dot.is_finite() && (dot.abs() - 1.).abs() < 1e-11,
        "{a:?} != {b:?}"
    );
}
fn same_state(a: BodyState, b: BodyState) {
    near(a.position, b.position);
    near(a.velocity, b.velocity);
    near(a.angular_velocity, b.angular_velocity);
    same_rotation(a.orientation, b.orientation);
}
// Independent quaternion rotation matrix; no production frame helper or pose setter.
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
fn ray(
    sim: &Simulation,
    origin: [f64; 3],
    direction: [f64; 3],
    exclude: Option<BodyId>,
) -> Option<f64> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(origin, direction, 10., exclude)
        .unwrap()
}
fn distance(
    sim: &Simulation,
    origin: [f64; 3],
    direction: [f64; 3],
    expected: f64,
    exclude: Option<BodyId>,
) {
    let actual = ray(sim, origin, direction, exclude).unwrap();
    assert!(
        actual.is_finite() && (actual - expected).abs() < 1e-11,
        "{actual} != {expected}"
    );
}

#[test]
fn all_four_posed_primitives_keep_centered_control_six_axis_link_trajectories() {
    for shape in SHAPES {
        let mut posed = load(
            &fixture(shape, LOCAL, "1 2 3", "0.1 0.2 0.3"),
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let mut centered = load(
            &fixture(shape, IDENTITY_POSE, "1 2 3", "0.1 0.2 0.3"),
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let owner = id(&posed, "default::rig");
        let link = id(&posed, "default::rig::body");
        assert!(velocity_control_binding_matches(&posed, owner));
        near(velocity(&posed, "default::rig::body").linear, [0.; 3]);
        // First world target is fixed analytically by model Z90, not collider Y90.
        posed.step_exact(1).unwrap();
        centered.step_exact(1).unwrap();
        near(velocity(&posed, "default::rig::body").linear, [-2., 1., 3.]);
        near(
            velocity(&posed, "default::rig::body").angular,
            [-0.2, 0.1, 0.3],
        );
        near(
            pose(&posed, "default::rig").translation,
            [-0.002, 0.001, 10.003],
        );
        same_state(
            state(&posed, "default::rig::body"),
            state(&centered, "default::rig::body"),
        );
        for _ in 0..20 {
            let before = pose(&posed, "default::rig");
            let linear = rotate(before.rotation, [1., 2., 3.]);
            let angular = rotate(before.rotation, [0.1, 0.2, 0.3]);
            posed.step_exact(1).unwrap();
            centered.step_exact(1).unwrap();
            near(velocity(&posed, "default::rig::body").linear, linear);
            near(velocity(&posed, "default::rig::body").angular, angular);
            near(
                pose(&posed, "default::rig").translation,
                std::array::from_fn(|i| before.translation[i] + 0.001 * linear[i]),
            );
            same_state(
                state(&posed, "default::rig::body"),
                state(&centered, "default::rig::body"),
            );
        }
        // Existing world wrenches remain real under the velocity owner; they are
        // not swallowed by a pose shortcut or shifted to the collider center.
        let before = pose(&posed, "default::rig");
        let target = rotate(before.rotation, [1., 2., 3.]);
        queue_link_world_wrench(&mut posed, link, [2., 4., 6.], [0.; 3]).unwrap();
        let centered_link = id(&centered, "default::rig::body");
        queue_link_world_wrench(&mut centered, centered_link, [2., 4., 6.], [0.; 3]).unwrap();
        posed.step_exact(1).unwrap();
        centered.step_exact(1).unwrap();
        near(
            velocity(&posed, "default::rig::body").linear,
            std::array::from_fn(|i| target[i] + [0.001, 0.002, 0.003][i]),
        );
        same_state(
            state(&posed, "default::rig::body"),
            state(&centered, "default::rig::body"),
        );
        let target = rotate(pose(&posed, "default::rig").rotation, [1., 2., 3.]);
        posed.step_exact(1).unwrap();
        near(velocity(&posed, "default::rig::body").linear, target);
    }
}

#[test]
fn moving_posed_primitive_rays_and_collision_visual_imu_frames_follow_real_angular_steps() {
    let q = std::f64::consts::FRAC_1_SQRT_2;
    for (index, shape) in SHAPES.into_iter().enumerate() {
        let mut sim = load(
            &fixture(shape, LOCAL, "1 2 3", "0 0 0"),
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let owner = id(&sim, "default::rig");
        let body = handle(&sim, "default::rig::body");
        let shape_entity =
            entity_by_id(sim.world(), id(&sim, "default::rig::body::shape")).unwrap();
        let local = sim.world().get::<LocalPose>(shape_entity).unwrap().0;
        near(local.translation, [2., -0.5, 0.25]);
        same_rotation(local.rotation, [0., q, 0., q]);
        sim.step_exact(25).unwrap();
        // Z90 maps initial[1,2,3] to[-2,1,3]; the shape center is .5,2,.25 from the link.
        near(
            pose(&sim, "default::rig").translation,
            [-0.05, 0.025, 10.075],
        );
        near(
            pose(&sim, "default::rig::body::shape").translation,
            [0.45, 2.025, 10.325],
        );
        same_rotation(
            pose(&sim, "default::rig::body::shape").rotation,
            [-0.5, 0.5, 0.5, 0.5],
        );
        set_velocity_control_command(&mut sim, owner, [0.; 3], [0.25, -0.5, 0.75]).unwrap();
        sim.step_exact(1).unwrap(); // Old linear target, then latch the angular-only command.
        let turn_origin = pose(&sim, "default::rig").translation;
        let initial_rotation = pose(&sim, "default::rig").rotation;
        for _ in 0..20 {
            sim.step_exact(1).unwrap();
            let link = pose(&sim, "default::rig::body");
            near(link.translation, turn_origin);
            let offset = rotate(link.rotation, [2., -0.5, 0.25]);
            let center = std::array::from_fn(|i| link.translation[i] + offset[i]);
            near(pose(&sim, "default::rig::body::shape").translation, center);
            same_rotation(
                pose(&sim, "default::rig::body::shape").rotation,
                product(link.rotation, [0., q, 0., q]),
            );
            near(
                pose(&sim, "default::rig::body::view").translation,
                std::array::from_fn(|i| {
                    link.translation[i] + rotate(link.rotation, [0., 0., 1.])[i]
                }),
            );
            near(
                pose(&sim, "default::rig::body::imu").translation,
                link.translation,
            );
            // Principal-axis extents are analytic for each primitive; caller Y90
            // turns the local Z axis, not the model-frame velocity command.
            let extents = match index {
                0 => [0.25, 0.5, 0.75],
                1 => [0.5, 0.5, 0.75],
                2 => [0.5, 0.5, 1.25],
                _ => [0.5; 3],
            };
            for (axis, extent) in [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
                .into_iter()
                .zip(extents)
            {
                let direction = rotate(link.rotation, rotate([0., q, 0., q], axis));
                let origin = std::array::from_fn(|i| center[i] - (extent + 2.) * direction[i]);
                distance(&sim, origin, direction, 2., None);
                assert_eq!(ray(&sim, origin, direction, Some(body)), None);
            }
        }
        let dot: f64 = initial_rotation
            .into_iter()
            .zip(pose(&sim, "default::rig").rotation)
            .map(|(a, b)| a * b)
            .sum();
        assert!(
            dot.abs() < 0.99999,
            "angular command did not rotate the real body"
        );
    }
}

#[test]
fn posed_velocity_pause_latest_and_zero_preserve_previous_target_latch_timing() {
    for shape in SHAPES {
        let mut sim = load(
            &fixture(shape, LOCAL, "1 0 0", "0 0 0"),
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let owner = id(&sim, "default::rig");
        set_velocity_control_command(&mut sim, owner, [9.; 3], [0.; 3]).unwrap();
        set_velocity_control_command(&mut sim, owner, [0., 2., 0.], [0.; 3]).unwrap();
        for _ in 0..3 {
            sim.tick().unwrap();
            near(pose(&sim, "default::rig").translation, [0., 0., 10.]);
            near(velocity(&sim, "default::rig::body").linear, [0.; 3]);
        }
        assert_eq!(sim.elapsed(), Duration::ZERO);
        sim.step_exact(1).unwrap();
        near(velocity(&sim, "default::rig::body").linear, [0., 1., 0.]);
        near(pose(&sim, "default::rig").translation, [0., 0.001, 10.]);
        sim.step_exact(1).unwrap();
        near(velocity(&sim, "default::rig::body").linear, [-2., 0., 0.]);
        near(pose(&sim, "default::rig").translation, [-0.002, 0.001, 10.]);
        set_velocity_control_command(&mut sim, owner, [0.; 3], [0.; 3]).unwrap();
        sim.tick().unwrap();
        sim.step_exact(1).unwrap();
        near(velocity(&sim, "default::rig::body").linear, [-2., 0., 0.]);
        sim.step_exact(1).unwrap();
        near(velocity(&sim, "default::rig::body").linear, [0.; 3]);
        let stopped = pose(&sim, "default::rig").translation;
        sim.step_exact(3).unwrap();
        near(pose(&sim, "default::rig").translation, stopped);
    }
}

#[test]
fn posed_velocity_ordinary_loaded_resets_removal_and_reconstruction_preserve_owned_ids() {
    for shape in SHAPES {
        let mut sim = load(
            &fixture(shape, LOCAL, "1 0 0", "0 0 0"),
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let owner = id(&sim, "default::rig");
        let link = id(&sim, "default::rig::body");
        let survivor = id(&sim, "default::survivor::body");
        let original_body = handle(&sim, "default::rig::body");
        let survivor_body = handle(&sim, "default::survivor::body");
        let configuration = initial_velocity_control_configurations(&sim)
            .unwrap()
            .next()
            .unwrap()
            .1
            .clone();
        set_velocity_control_command(&mut sim, owner, [8.; 3], [0.; 3]).unwrap();
        sim.step_exact(3).unwrap();
        sim.reset().unwrap();
        near(pose(&sim, "default::rig").translation, [0., 0., 10.]);
        assert_eq!(handle(&sim, "default::rig::body"), original_body);
        sim.step_exact(1).unwrap();
        near(velocity(&sim, "default::rig::body").linear, [0., 1., 0.]);
        set_velocity_control_command(&mut sim, owner, [7.; 3], [0.; 3]).unwrap();
        queue_link_world_velocity(&mut sim, survivor, [0., 3., 0.], [0.; 3]).unwrap();
        let epoch = sim.epoch();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(sim.epoch(), epoch + 1);
        assert_eq!(id(&sim, "default::rig"), owner);
        assert_eq!(id(&sim, "default::rig::body"), link);
        assert_eq!(handle(&sim, "default::rig::body"), original_body);
        assert_eq!(handle(&sim, "default::survivor::body"), survivor_body);
        assert!(velocity_control_binding_matches_configuration(
            &sim,
            owner,
            &configuration
        ));
        sim.step_exact(1).unwrap();
        near(velocity(&sim, "default::rig::body").linear, [0., 1., 0.]);
        near(velocity(&sim, "default::survivor::body").linear, [0.; 3]);
        near(
            pose(&sim, "default::rig::body::shape").translation,
            [0.5, 2.001, 10.25],
        );
        queue_link_world_velocity(&mut sim, survivor, [0., 1., 0.], [0.; 3]).unwrap();
        sim.step_exact(1).unwrap();
        let before = pose(&sim, "default::survivor::body").translation;
        remove_model(&mut sim, owner).unwrap();
        assert!(!velocity_control_binding_matches(&sim, owner));
        assert!(matches!(
            set_velocity_control_command(&mut sim, owner, [1.; 3], [0.; 3]),
            Err(VelocityControlCommandError::MissingController(_))
        ));
        assert!(
            sim.world()
                .resource::<PhysicsBackend>()
                .state(original_body)
                .is_none()
        );
        sim.step_exact(3).unwrap();
        near(
            pose(&sim, "default::survivor::body").translation,
            [before[0], before[1] + 0.003, before[2]],
        );
        assert_eq!(handle(&sim, "default::survivor::body"), survivor_body);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(id(&sim, "default::rig"), owner);
        assert_eq!(handle(&sim, "default::rig::body"), original_body);
        assert!(velocity_control_binding_matches_configuration(
            &sim,
            owner,
            &configuration
        ));
        near(
            pose(&sim, "default::rig::body::shape").translation,
            [0.5, 2., 10.25],
        );
        sim.step_exact(1).unwrap();
        near(velocity(&sim, "default::rig::body").linear, [0., 1., 0.]);
    }
}

#[test]
fn controlled_posed_sphere_contacts_where_its_centered_peer_misses_without_pose_shortcuts() {
    let source =
        fixture(SHAPES[3], "2 0 0 0 0 0.7", "0 0 -1", "0 0 0").replace(MODEL_POSE, "0 0 1 0 0 0");
    let floor = "<model name='floor'><static>true</static><link name='body'><collision name='shape'><pose>2 0 0 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></collision></link></model>";
    let source = source.replace("</world>", &format!("{floor}</world>"));
    let centered = source.replace("2 0 0 0 0 0.7", IDENTITY_POSE);
    assert_ne!(source, centered);
    let mut posed = load(&source, PhysicsEngine::Rapier).unwrap();
    let mut control = load(&centered, PhysicsEngine::Rapier).unwrap();
    let owner = id(&posed, "default::rig");
    let floor = handle(&posed, "default::floor::body");
    distance(&posed, [2., 0., 3.], [0., 0., -1.], 1.5, Some(floor));
    assert_eq!(
        ray(
            &control,
            [2., 0., 3.],
            [0., 0., -1.],
            Some(handle(&control, "default::floor::body"))
        ),
        None
    );
    assert!(velocity_control_binding_matches(&posed, owner));
    posed.step_exact(10).unwrap();
    control.step_exact(10).unwrap();
    near(pose(&control, "default::rig").translation, [0., 0., 0.99]);
    near(
        velocity(&control, "default::rig::body").linear,
        [0., 0., -1.],
    );
    let actual = state(&posed, "default::rig::body");
    // The contact impulse acts at a displaced point: it may also rotate the link.
    // This checks real response rather than assuming a static centered balance.
    assert!(
        actual.position[2] > 0.990001,
        "missing offset contact: {actual:?}"
    );
    assert!(
        actual.velocity[2] > -0.999,
        "command was not resolved by contact: {actual:?}"
    );
    assert!(
        actual.angular_velocity[1] < -1e-7,
        "displaced contact torque missing: {actual:?}"
    );
    near(pose(&posed, "default::floor::body").translation, [0.; 3]);
}

#[test]
fn posed_velocity_invalid_commands_keep_pending_target_and_direct_motion_refusals() {
    for shape in SHAPES {
        let xml = fixture(shape, LOCAL, "1 0 0", "0 0 0")
            .replace(MODEL_POSE, "0 0 10 0 0 0.7853981633974483");
        // World edits admit a collision and visual, excluding sensor descendants.
        let imu = "<sensor name='imu' type='imu'><always_on>true</always_on><update_rate>1000</update_rate></sensor>";
        assert_eq!(xml.matches(imu).count(), 1);
        let sensorless = xml.replacen(imu, "", 1);
        assert_ne!(sensorless, xml);
        let mut sim = load(&sensorless, PhysicsEngine::Rapier).unwrap();
        let owner = id(&sim, "default::rig");
        let link = id(&sim, "default::rig::body");
        set_velocity_control_command(&mut sim, owner, [0., 2., 0.], [0.; 3]).unwrap();
        let before = state(&sim, "default::rig::body");
        for (linear, angular) in [
            ([f64::NAN; 3], [0.; 3]),
            ([0.; 3], [f64::INFINITY; 3]),
            ([f64::MAX, f64::MAX, 0.], [0.; 3]),
        ] {
            assert!(matches!(
                set_velocity_control_command(&mut sim, owner, linear, angular),
                Err(VelocityControlCommandError::InvalidCommand)
            ));
            same_state(state(&sim, "default::rig::body"), before);
        }
        assert!(matches!(
            queue_link_world_velocity(&mut sim, link, [9.; 3], [0.; 3]),
            Err(InputError::Unsupported(
                "VelocityControl owns link velocity"
            ))
        ));
        let original = model_state(&sim, owner).unwrap();
        assert!(matches!(
            set_model_state(&mut sim, owner, original),
            Err(gz_sim_scene::WorldEditError::UnsupportedModel)
        ));
        same_state(state(&sim, "default::rig::body"), before);
        assert!(velocity_control_binding_matches(&sim, owner));
        assert!(matches!(
            set_velocity_control_command(&mut sim, u64::MAX, [0.; 3], [0.; 3]),
            Err(VelocityControlCommandError::MissingController(_))
        ));
        assert!(matches!(
            set_velocity_control_command(&mut sim, link, [0.; 3], [0.; 3]),
            Err(VelocityControlCommandError::MissingController(_))
        ));
        sim.step_exact(1).unwrap();
        near(
            velocity(&sim, "default::rig::body").linear,
            [
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
                0.,
            ],
        );
        sim.step_exact(1).unwrap();
        near(
            velocity(&sim, "default::rig::body").linear,
            [-2f64.sqrt(), 2f64.sqrt(), 0.],
        );
    }
}

#[test]
fn posed_velocity_public_configuration_and_link_pose_drift_cannot_release_or_redirect_ownership() {
    for shape in SHAPES {
        for fault in 0..4 {
            let mut sim = load(
                &fixture(shape, LOCAL, "1 0 0", "0 0 0"),
                PhysicsEngine::Rapier,
            )
            .unwrap();
            let owner = id(&sim, "default::rig");
            let link = id(&sim, "default::rig::body");
            let other = id(&sim, "default::survivor::body");
            assert!(velocity_control_binding_matches(&sim, owner));
            let entity = entity_by_id(sim.world(), owner).unwrap();
            let mut config = sim
                .world()
                .get::<VelocityControlConfiguration>(entity)
                .unwrap()
                .clone();
            let before = state(&sim, "default::rig::body");
            match fault {
                0 => {
                    sim.world_mut()
                        .entity_mut(entity)
                        .remove::<VelocityControlConfiguration>();
                }
                1 => {
                    config.topic.push_str("/foreign");
                    sim.world_mut().entity_mut(entity).insert(config);
                }
                2 => {
                    config.link = other;
                    sim.world_mut().entity_mut(entity).insert(config);
                }
                _ => {
                    let e = entity_by_id(sim.world(), link).unwrap();
                    sim.world_mut()
                        .get_mut::<LocalPose>(e)
                        .unwrap()
                        .0
                        .translation[0] = 0.1;
                }
            }
            assert!(!velocity_control_binding_matches(&sim, owner));
            assert!(set_velocity_control_command(&mut sim, owner, [8.; 3], [0.; 3]).is_err());
            assert!(matches!(
                queue_link_world_velocity(&mut sim, link, [9.; 3], [0.; 3]),
                Err(InputError::Unsupported(_))
            ));
            assert!(sim.step_exact(1).is_err());
            same_state(state(&sim, "default::rig::body"), before);
            near(
                state(&sim, "default::survivor::body").position,
                [30., 0., 10.],
            );
        }
    }
}

#[test]
fn posed_velocity_admission_keeps_tpe_link_com_compound_and_controller_limits_closed() {
    for shape in SHAPES {
        let xml = fixture(shape, LOCAL, "1 0 0", "0 0 0");
        assert!(load(&xml, PhysicsEngine::Tpe).is_err());
        assert!(
            load(
                &fixture(shape, IDENTITY_POSE, "1 0 0", "0 0 0"),
                PhysicsEngine::Tpe
            )
            .is_ok()
        );
        for invalid in [
            xml.replace("<link name='body'>","<link name='body'><pose>0.1 0 0 0 0 0</pose>"),
            xml.replace("<inertial>","<inertial><pose>0.1 0 0 0 0 0</pose>"),
            xml.replacen("<model name='rig'>","<model name='rig'><static>true</static>",1),
            xml.replacen("</link>","<collision name='extra'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link>",1),
            xml.replacen("</link>","</link><link name='extra'/>",1),
            xml.replace("<initial_linear>1 0 0</initial_linear>","<initial_linear>nan 0 0</initial_linear>"),
        ] {
            assert_ne!(invalid,xml);assert!(load(&invalid,PhysicsEngine::Rapier).is_err());
        }
    }
}
