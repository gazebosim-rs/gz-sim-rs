// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{
    BodyId, BodySpec, JointPositionServo, JointVelocityMotor, MeshBodySpec, PhysicsBackend,
    PhysicsError, Shape,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_physics_rapier::RapierBackend;
use std::time::Duration;

const DT: Duration = Duration::from_millis(1);
const Y_AXIS: [f64; 3] = [0.0, 1.0, 0.0];

fn sphere(position: [f64; 3]) -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.2 },
        position,
        velocity: [0.0; 3],
        mass: Some(1.0),
    }
}

fn mixed_world(gravity: [f64; 3]) -> (PhysicsBackend, BodyId) {
    let mut world: PhysicsBackend = MultiFidelityBackend::new(gravity).unwrap().into();
    let moving = world
        .spawn_kinematic_link(
            BodySpec {
                velocity: Y_AXIS,
                ..sphere([10.0, 0.0, 0.0])
            },
            Default::default(),
            None,
        )
        .unwrap();
    (world, moving)
}

#[test]
fn reduced_hinge_keeps_generalized_effort_and_actuator_modes_in_a_mixed_world() {
    let (mut world, moving) = mixed_world([0.0; 3]);
    let child = world
        .spawn_link(sphere([0.0; 3]), Default::default(), None)
        .unwrap();
    let joint = world
        .add_reduced_revolute(None, child, [0.0; 3], Y_AXIS)
        .unwrap();
    assert_eq!(world.joint_child(joint), Some(child));
    assert_eq!(world.is_jointed(child), Some(true));
    assert_eq!(world.is_jointed(moving), Some(false));

    // The rotor's moment of inertia is 2/5 * 1 kg * (0.2 m)^2 = 0.016 kg m².
    let inertia = 0.016;
    world.validate_joint_effort(joint, inertia).unwrap();
    world.add_joint_effort(joint, inertia).unwrap();
    world.step(Duration::ZERO).unwrap();
    assert_eq!(world.joint_state(joint).unwrap().velocity, 0.0);
    for _ in 0..2 {
        world.step(DT).unwrap();
        assert!((world.joint_state(joint).unwrap().velocity - 0.001).abs() < 1e-12);
    }

    let motor = Some(JointVelocityMotor {
        target_velocity: 20.0,
        max_force: 0.08,
    });
    world.validate_joint_velocity_motor(joint, motor).unwrap();
    world.set_joint_velocity_motor(joint, motor).unwrap();
    world.step(DT).unwrap();
    let gain = world.joint_state(joint).unwrap().velocity - 0.001;
    assert!(
        gain > 0.0 && gain <= 0.08 * DT.as_secs_f64() / inertia + 1e-12,
        "{gain}"
    );

    let reverse = Some(JointVelocityMotor {
        target_velocity: -1.0,
        max_force: 100.0,
    });
    world.validate_joint_velocity_servo(joint, reverse).unwrap();
    world.set_joint_velocity_servo(joint, reverse).unwrap();
    for _ in 0..10 {
        world.step(DT).unwrap();
    }
    assert!((world.joint_state(joint).unwrap().velocity + 1.0).abs() < 1e-12);
    assert!((world.state(child).unwrap().angular_velocity[1] + 1.0).abs() < 1e-12);
    assert!((world.state(moving).unwrap().position[1] - 0.013).abs() < 1e-12);

    world.reset().unwrap();
    let target = Some(JointPositionServo {
        target_position: 1.0,
        max_velocity: 2.0,
        max_force: 0.08,
    });
    world
        .validate_joint_position_servo(joint, target, DT)
        .unwrap();
    world.set_joint_position_servo(joint, target, DT).unwrap();
    world.step(DT).unwrap();
    let state = world.joint_state(joint).unwrap();
    assert!(state.position > 0.0 && state.velocity > 0.0);
    assert!(state.velocity <= 0.08 * DT.as_secs_f64() / inertia + 1e-12);
    assert!(world.remove_joint(joint));
    assert_eq!(world.joint_child(joint), None);
    assert_eq!(world.joint_state(joint), None);
    assert_eq!(world.is_jointed(child), Some(false));
    assert_eq!(
        world.add_joint_effort(joint, 1.0),
        Err(PhysicsError::MissingJoint)
    );
}

#[test]
fn resource_downcasts_to_the_selected_backend_and_preserves_impulse_joint_refusals() {
    let (mut world, moving) = mixed_world([0.0; 3]);
    assert_eq!(world.engine(), "multifidelity");
    assert!(world.downcast_ref::<RapierBackend>().is_none());
    assert!(world.downcast_mut::<RapierBackend>().is_none());
    assert!(world.downcast_ref::<MultiFidelityBackend>().is_some());
    assert!(world.downcast_mut::<MultiFidelityBackend>().is_some());

    let fixed = world
        .spawn_link(
            BodySpec {
                mass: None,
                ..sphere([0.0, 0.0, -1.0])
            },
            Default::default(),
            None,
        )
        .unwrap();
    let child = world
        .spawn_link(sphere([0.0; 3]), Default::default(), None)
        .unwrap();
    let joint = world.add_revolute(fixed, child, [0.0; 3], Y_AXIS).unwrap();
    let initial = world.joint_state(joint).unwrap();
    assert_eq!(
        world.set_joint_position_servo(
            joint,
            Some(JointPositionServo {
                target_position: 1.0,
                max_velocity: 2.0,
                max_force: 1.0
            }),
            DT,
        ),
        Err(PhysicsError::Unsupported {
            engine: "multifidelity",
            capability: "continuous position servos on impulse joints",
        })
    );
    assert_eq!(world.joint_state(joint), Some(initial));
    assert!(world.remove(fixed));
    assert_eq!(world.joint_state(joint), None);
    assert_eq!(world.is_jointed(child), Some(false));
    world
        .apply_impulse(child, [2.0, 0.0, 0.0], [0.0; 3])
        .unwrap();
    world.step(DT).unwrap();
    assert!((world.state(child).unwrap().position[0] - 0.002).abs() < 1e-12);
    assert_eq!(world.state(moving).unwrap().velocity, Y_AXIS);
}

#[test]
fn triangle_mesh_contact_and_collision_groups_share_the_rigid_world() {
    for filter_contact in [false, true] {
        let (mut world, _) = mixed_world([0.0, 0.0, -9.81]);
        let triangle = MeshBodySpec {
            vertices: &[[-2.0, -2.0, 0.0], [2.0, -2.0, 0.0], [0.0, 2.0, 0.0]],
            triangles: &[[0, 1, 2]],
            collision_position: [0.0, 0.0, 0.25],
            collision_orientation: [0.0, 0.0, 0.0, 1.0],
            position: [0.0; 3],
            velocity: [0.0; 3],
            mass: None,
        };
        let invalid = MeshBodySpec {
            triangles: &[[0, 1, 3]],
            ..triangle
        };
        assert_eq!(
            world.spawn_mesh_link(invalid, Default::default(), None),
            Err(PhysicsError::Native {
                engine: "multifidelity",
                message: "mesh triangle index is outside its vertex buffer".into(),
            })
        );
        let floor = world
            .spawn_mesh_link(triangle, Default::default(), None)
            .unwrap();
        let ball = world
            .spawn_link(sphere([0.0, 0.0, 1.0]), Default::default(), None)
            .unwrap();
        let distance = world
            .cast_ray([0.0, 0.0, 2.0], [0.0, 0.0, -1.0], 3.0, Some(ball))
            .unwrap();
        assert_eq!(distance, Some(1.75));
        world.set_self_collision_group(floor, Some(42)).unwrap();
        if filter_contact {
            world.set_self_collision_group(ball, Some(42)).unwrap();
        }
        for _ in 0..600 {
            world.step(DT).unwrap();
        }
        let height = world.state(ball).unwrap().position[2];
        if filter_contact {
            assert!(height < 0.0, "{height}");
        } else {
            // The sphere rests with its center one radius above the offset triangle.
            assert!((height - 0.45).abs() < 0.01, "{height}");
        }
    }
}
