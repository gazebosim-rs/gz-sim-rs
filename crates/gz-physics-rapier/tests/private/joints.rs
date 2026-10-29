// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn spec() -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.1 },
        position: [0.0, 0.0, -1.0],
        velocity: [0.0; 3],
        mass: Some(1.0),
    }
}
#[test]
fn invalid_joint_construction_is_atomic_including_exhaustion_and_large_offsets() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let body = backend.spawn(spec()).unwrap();
    let before = backend.state(body).unwrap();
    for (anchor, axis) in [
        ([f64::INFINITY, 0.0, 0.0], [0.0, 1.0, 0.0]),
        ([0.0; 3], [0.0; 3]),
        ([0.0; 3], [f64::NAN, 0.0, 0.0]),
        ([0.0; 3], [1e308, 1e308, 0.0]),
        ([0.0; 3], [1e-300, 0.0, 0.0]),
    ] {
        assert_eq!(
            backend.add_world_revolute(body, anchor, axis),
            Err(PhysicsError::InvalidJoint)
        );
        assert!(backend.joints.records.is_empty());
        assert!(backend.joints.anchor.is_none());
        assert_eq!(backend.state(body).unwrap(), before);
    }
    let fixed = backend
        .spawn(BodySpec {
            position: [100.0, 0.0, 0.0],
            mass: None,
            ..spec()
        })
        .unwrap();
    assert_eq!(
        backend.add_world_revolute(fixed, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::InvalidJoint)
    );
    let removed = backend.spawn(spec()).unwrap();
    assert!(backend.remove(removed));
    assert_eq!(
        backend.add_world_revolute(removed, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::MissingBody)
    );
    // Spawn rejects unrepresentable AABBs first. Inject a distant solver pose to
    // exercise derived joint-frame overflow independently of collider validation.
    let handle = backend.bodies[&body].0;
    backend.world.bodies[handle].set_translation(Vector::new(-1e308, 0.0, 0.0), false);
    assert_eq!(
        backend.add_world_revolute(body, [1e308, 0.0, 0.0], [0.0, 1.0, 0.0]),
        Err(PhysicsError::InvalidJoint)
    );
    backend.world.bodies[handle].set_translation(Vector::from_array(before.position), false);
    backend.joints.next = u64::MAX;
    assert_eq!(
        backend.add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::JointIdOverflow)
    );
    assert!(backend.joints.anchor.is_none());
    assert!(backend.joints.records.is_empty());
    assert_eq!(backend.joints.next, u64::MAX);
}
#[test]
fn linked_sleep_policy_rebuilds_and_restores_only_when_the_last_constraint_retires() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let body = backend.spawn(spec()).unwrap();
    let handle = backend.bodies[&body].0;
    assert!(
        backend.world.bodies[handle]
            .activation()
            .normalized_linear_threshold
            >= 0.0
    );
    let first = backend
        .add_world_revolute(body, [0.0; 3], [0.0, 2.0, 0.0])
        .unwrap();
    let second = backend
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert!(
        backend.world.bodies[handle]
            .activation()
            .normalized_linear_threshold
            < 0.0
    );
    backend.reset().unwrap();
    let handle = backend.bodies[&body].0;
    assert!(
        backend.world.bodies[handle]
            .activation()
            .normalized_linear_threshold
            < 0.0
    );
    assert!(backend.remove_joint(first));
    assert!(
        backend.world.bodies[handle]
            .activation()
            .normalized_linear_threshold
            < 0.0
    );
    assert!(backend.remove_joint(second));
    assert!(
        backend.world.bodies[handle]
            .activation()
            .normalized_linear_threshold
            >= 0.0
    );
    assert!(backend.joints.anchor.is_none());
    assert_eq!(backend.world.bodies.len(), 1);
    let third = backend
        .add_world_revolute(body, [0.0; 3], [1.0, 0.0, 0.0])
        .unwrap();
    assert!(backend.remove(body));
    assert!(backend.joint_state(third).is_none());
    assert!(backend.joints.anchor.is_none());
    assert_eq!(backend.world.bodies.len(), 0);
}

#[test]
fn reduced_world_roots_are_independent_and_retire_without_changing_solver_work() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let a = backend.spawn(spec()).unwrap();
    let b = backend.spawn(spec()).unwrap();
    let first = backend
        .add_reduced_revolute(None, a, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    let second = backend
        .add_reduced_revolute(None, b, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert_eq!(backend.joints.world_roots.len(), 2);
    assert!(backend.joints.anchor.is_none());
    assert_eq!(backend.world.bodies.len(), 4);
    let left = backend
        .world
        .multibody_joints
        .rigid_body_link(backend.bodies[&a].0)
        .unwrap()
        .multibody;
    let right = backend
        .world
        .multibody_joints
        .rigid_body_link(backend.bodies[&b].0)
        .unwrap()
        .multibody;
    assert_ne!(left, right);
    assert_eq!(
        backend.world.integration_parameters.num_solver_iterations,
        4
    );
    for body in backend.world.bodies.iter().map(|(_, body)| body) {
        assert_eq!(body.additional_solver_iterations(), 0);
    }
    backend.reset().unwrap();
    assert_eq!(backend.joints.world_roots.len(), 2);
    assert_eq!(backend.world.bodies.len(), 4);
    assert!(backend.remove_joint(first));
    assert_eq!(backend.world.bodies.len(), 3);
    assert_eq!(backend.joints.world_roots.len(), 1);
    assert!(backend.joint_state(second).is_some());
    assert!(backend.remove(b));
    assert_eq!(backend.world.bodies.len(), 1);
    assert!(backend.joints.world_roots.is_empty());
    assert!(backend.joint_state(second).is_none());
}
