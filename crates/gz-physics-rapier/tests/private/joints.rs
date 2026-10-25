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
    backend.reset();
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
