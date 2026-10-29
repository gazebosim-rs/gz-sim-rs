// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    BodyId, BodySpec, DynamicBackend, PhysicsError, RapierBackend, RotationSpec, Shape,
};
use rapier3d_f64::prelude::{Rotation, Vector};
use std::time::Duration;

const DT: Duration = Duration::from_millis(1);

fn body(backend: &mut RapierBackend, position: [f64; 3], mass: Option<f64>) -> BodyId {
    backend
        .spawn(BodySpec {
            shape: Shape::Sphere { radius: 0.1 },
            position,
            velocity: [0.0; 3],
            mass,
        })
        .unwrap()
}

#[test]
fn moving_rotated_parent_reports_relative_axis_velocity_and_complete_turns() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let mut endpoints = Vec::new();
    for (orientation, speed) in [
        ([0.0, 0.0, 0.5_f64.sqrt(), 0.5_f64.sqrt()], 2.0),
        ([0.0, 0.5_f64.sqrt(), 0.0, 0.5_f64.sqrt()], 3.0),
    ] {
        endpoints.push(
            backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.1 },
                        position: [0.0; 3],
                        velocity: [0.0; 3],
                        mass: Some(1.0),
                    },
                    RotationSpec {
                        orientation,
                        angular_velocity: [speed, 0.0, 0.0],
                    },
                    None,
                )
                .unwrap(),
        );
    }
    let joint = backend
        .add_revolute(endpoints[0], endpoints[1], [0.0; 3], [1.0, 0.0, 0.0])
        .unwrap();
    assert!(backend.joint_state(joint).unwrap().position.abs() < 1e-12);
    assert!((backend.joint_state(joint).unwrap().velocity - 1.0).abs() < 1e-12);
    for step in 1..=8000 {
        backend.step(DT).unwrap();
        let state = backend.joint_state(joint).unwrap();
        // The small-step Euler integration error is below ten microradians.
        assert!(
            (state.position - step as f64 * 0.001).abs() < 1e-5,
            "{step}: {state:?}"
        );
        assert!((state.velocity - 1.0).abs() < 1e-10);
    }
    assert!((backend.state(endpoints[0]).unwrap().angular_velocity[0] - 2.0).abs() < 1e-10);
    assert!((backend.state(endpoints[1]).unwrap().angular_velocity[0] - 3.0).abs() < 1e-10);
}

#[test]
fn fixed_parent_retains_its_pose_while_child_swings_about_the_joint() {
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let parent = body(&mut backend, [0.0; 3], None);
    let child = body(&mut backend, [1.0, 0.0, 0.0], Some(1.0));
    let fixed = backend.state(parent).unwrap();
    let joint = backend
        .add_revolute(parent, child, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    let initial = backend.initial_state();
    for _ in 0..1000 {
        backend.step(DT).unwrap();
        assert_eq!(backend.state(parent), Some(fixed));
        let state = backend.state(child).unwrap();
        let rotation = Rotation::from_xyzw(
            state.orientation[0],
            state.orientation[1],
            state.orientation[2],
            state.orientation[3],
        );
        let anchor = Vector::from_array(state.position) - rotation * Vector::X;
        assert!(anchor.length() < 1e-5, "{anchor:?}");
    }
    assert!(backend.joint_state(joint).unwrap().position.abs() > 0.1);
    assert!(backend.remove_joint(joint));
    assert_eq!(backend.is_jointed(parent), Some(false));
    assert_eq!(backend.is_jointed(child), Some(false));
    assert_eq!(backend.state(parent), Some(fixed));
    let prepared = backend.prepare_initial(&initial).unwrap();
    backend.apply_initial(prepared).unwrap();
    assert_eq!(backend.state(parent), Some(fixed));
    assert_eq!(backend.state(child).unwrap().position, [1.0, 0.0, 0.0]);
    assert!(backend.joint_state(joint).unwrap().position.abs() < 1e-12);
}

#[test]
fn endpoint_removal_and_prepared_reset_keep_joint_identity_and_ownership() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let parent = body(&mut backend, [-1.0, 0.0, 0.0], Some(1.0));
    let child = body(&mut backend, [1.0, 0.0, 0.0], Some(1.0));
    let joint = backend
        .add_revolute(parent, child, [0.0; 3], [0.0, 0.0, 1.0])
        .unwrap();
    let initial = backend.initial_state();
    for removed in [parent, child] {
        let prepared = backend.prepare_initial(&initial).unwrap();
        backend.apply_initial(prepared).unwrap();
        assert_eq!(backend.joint_child(joint), Some(child));
        assert_eq!(backend.is_jointed(parent), Some(true));
        assert_eq!(backend.is_jointed(child), Some(true));
        assert!(backend.remove(removed));
        assert!(backend.joint_state(joint).is_none());
        assert!(backend.joint_child(joint).is_none());
        let survivor = if removed == parent { child } else { parent };
        assert_eq!(backend.is_jointed(survivor), Some(false));
        backend.step(DT).unwrap();
        assert!(backend.state(survivor).is_some());
    }
    let prepared = backend.prepare_initial(&initial).unwrap();
    backend.apply_initial(prepared).unwrap();
    assert!(backend.remove_joint(joint));
    assert!(!backend.remove_joint(joint));
    assert_eq!(backend.is_jointed(parent), Some(false));
    assert_eq!(backend.is_jointed(child), Some(false));
    backend.reset().unwrap();
    let next = backend
        .add_revolute(parent, child, [0.0; 3], [0.0, 0.0, 1.0])
        .unwrap();
    assert_ne!(next, joint);
    backend.step(DT).unwrap();
    assert_eq!(
        backend.add_revolute(parent, child, [0.0; 3], [0.0, 0.0, 1.0]),
        Err(PhysicsError::JointConstructionClosed)
    );
}

#[test]
fn invalid_endpoints_and_frames_do_not_consume_joint_ids_or_change_bodies() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let parent = body(&mut backend, [0.0; 3], Some(1.0));
    let child = body(&mut backend, [1.0, 0.0, 0.0], Some(1.0));
    let fixed = body(&mut backend, [2.0, 0.0, 0.0], None);
    let before = [
        backend.state(parent),
        backend.state(child),
        backend.state(fixed),
    ];
    for (a, b, anchor, axis, error) in [
        (
            parent,
            parent,
            [0.0; 3],
            [1.0, 0.0, 0.0],
            PhysicsError::InvalidJoint,
        ),
        (
            BodyId::from_raw(999),
            child,
            [0.0; 3],
            [1.0, 0.0, 0.0],
            PhysicsError::MissingBody,
        ),
        (
            parent,
            BodyId::from_raw(999),
            [0.0; 3],
            [1.0, 0.0, 0.0],
            PhysicsError::MissingBody,
        ),
        (
            parent,
            fixed,
            [0.0; 3],
            [1.0, 0.0, 0.0],
            PhysicsError::InvalidJoint,
        ),
        (
            parent,
            child,
            [f64::NAN, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            PhysicsError::InvalidJoint,
        ),
        (
            parent,
            child,
            [0.0; 3],
            [0.0; 3],
            PhysicsError::InvalidJoint,
        ),
        (
            parent,
            child,
            [0.0; 3],
            [f64::INFINITY, 0.0, 0.0],
            PhysicsError::InvalidJoint,
        ),
    ] {
        assert_eq!(backend.add_revolute(a, b, anchor, axis), Err(error));
        assert_eq!(
            [
                backend.state(parent),
                backend.state(child),
                backend.state(fixed)
            ],
            before
        );
        assert_eq!(backend.is_jointed(parent), Some(false));
        assert_eq!(backend.is_jointed(child), Some(false));
    }
    let joint = backend
        .add_revolute(parent, child, [0.0; 3], [0.0, 0.0, 1.0])
        .unwrap();
    assert_eq!(joint, gz_physics_rapier::JointId::from_raw(0));
}

#[test]
fn mixed_world_and_body_constraints_restore_and_retire_only_connected_records() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let parent = body(&mut backend, [0.0; 3], Some(1.0));
    let child = body(&mut backend, [1.0, 0.0, 0.0], Some(1.0));
    let world = backend
        .add_world_revolute(parent, [0.0; 3], [0.0, 0.0, 1.0])
        .unwrap();
    let pair = backend
        .add_revolute(parent, child, [0.5, 0.0, 0.0], [0.0, 0.0, 1.0])
        .unwrap();
    let initial = backend.initial_state();
    assert!(backend.remove_joint(world));
    assert!(backend.joint_state(pair).is_some());
    backend.step(DT).unwrap();
    let restored = backend.prepare_initial(&initial).unwrap();
    backend.apply_initial(restored).unwrap();
    assert!(backend.joint_state(world).is_some());
    assert!(backend.joint_state(pair).is_some());
    assert!(backend.remove(child));
    assert!(backend.joint_state(pair).is_none());
    assert!(backend.joint_state(world).is_some());
    assert_eq!(backend.is_jointed(parent), Some(true));
    backend.step(DT).unwrap();
    let restored = backend.prepare_initial(&initial).unwrap();
    backend.apply_initial(restored).unwrap();
    assert!(backend.remove(parent));
    assert!(backend.joint_state(pair).is_none());
    assert!(backend.joint_state(world).is_none());
    assert_eq!(backend.is_jointed(child), Some(false));
    backend.step(DT).unwrap();
}

#[test]
fn overflowing_local_frames_are_rejected_before_constraints_are_created() {
    let half_angle = std::f64::consts::FRAC_PI_8;
    let rotated = [0.0, 0.0, half_angle.sin(), half_angle.cos()];
    for rotated_endpoint in 0..2 {
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let endpoints: Vec<_> = (0..2)
            .map(|index| {
                backend
                    .spawn_link(
                        BodySpec {
                            shape: Shape::Sphere { radius: 0.1 },
                            position: [index as f64, 0.0, 0.0],
                            velocity: [0.0; 3],
                            mass: Some(1.0),
                        },
                        RotationSpec {
                            orientation: if index == rotated_endpoint {
                                rotated
                            } else {
                                [0.0, 0.0, 0.0, 1.0]
                            },
                            ..RotationSpec::default()
                        },
                        None,
                    )
                    .unwrap()
            })
            .collect();
        let before = [backend.state(endpoints[0]), backend.state(endpoints[1])];
        assert_eq!(
            backend.add_revolute(
                endpoints[0],
                endpoints[1],
                [1.7e308, 1.7e308, 0.0],
                [0.0, 0.0, 1.0]
            ),
            Err(PhysicsError::InvalidJoint)
        );
        assert_eq!(
            [backend.state(endpoints[0]), backend.state(endpoints[1])],
            before
        );
        assert_eq!(backend.is_jointed(endpoints[0]), Some(false));
        assert_eq!(backend.is_jointed(endpoints[1]), Some(false));
        let first = backend
            .add_revolute(endpoints[0], endpoints[1], [0.0; 3], [0.0, 0.0, 1.0])
            .unwrap();
        assert_eq!(first, gz_physics_rapier::JointId::from_raw(0));
    }
}

#[test]
fn rotating_parent_axis_keeps_velocity_and_angle_in_the_same_relative_frame() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let mut endpoints = Vec::new();
    // Common spin about X makes the hinge's initially world-Z axis move.
    for speed in [2.0, 3.0] {
        endpoints.push(
            backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.1 },
                        position: [0.0; 3],
                        velocity: [0.0; 3],
                        mass: Some(1.0),
                    },
                    RotationSpec {
                        angular_velocity: [2.0, 0.0, speed],
                        ..RotationSpec::default()
                    },
                    None,
                )
                .unwrap(),
        );
    }
    let joint = backend
        .add_revolute(endpoints[0], endpoints[1], [0.0; 3], [0.0, 0.0, 1.0])
        .unwrap();
    let mut previous = backend.joint_state(joint).unwrap();
    let mut largest_axis_change = 0.0_f64;
    let mut largest_fixed_axis_error = 0.0_f64;
    for _ in 0..2000 {
        backend.step(DT).unwrap();
        let parent = backend.state(endpoints[0]).unwrap();
        let child = backend.state(endpoints[1]).unwrap();
        let rotation = Rotation::from_xyzw(
            parent.orientation[0],
            parent.orientation[1],
            parent.orientation[2],
            parent.orientation[3],
        );
        let axis = rotation * Vector::Z;
        let relative = Vector::from_array(child.angular_velocity)
            - Vector::from_array(parent.angular_velocity);
        let current = backend.joint_state(joint).unwrap();
        assert!((current.velocity - relative.dot(axis)).abs() < 1e-12);
        let angle_increment = 0.5 * DT.as_secs_f64() * (current.velocity + previous.velocity);
        assert!((current.position - previous.position - angle_increment).abs() < 1e-5);
        largest_axis_change = largest_axis_change.max((axis - Vector::Z).length());
        largest_fixed_axis_error =
            largest_fixed_axis_error.max((current.velocity - relative.z).abs());
        previous = current;
    }
    assert!(largest_axis_change > 1.0);
    assert!(largest_fixed_axis_error > 0.1);
}
