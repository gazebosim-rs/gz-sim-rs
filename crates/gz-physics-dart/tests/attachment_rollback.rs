// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Moving-parent rollback and retirement preserve native Jacobian caches.

use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, PhysicsError, RotationSpec, Shape,
};
use gz_physics_dart::DartBackend;
use std::time::Duration;

fn spawn(
    backend: &mut DartBackend,
    position: [f64; 3],
    linear: [f64; 3],
    angular: [f64; 3],
) -> BodyId {
    backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position,
                velocity: linear,
                mass: Some(1.0),
            },
            RotationSpec {
                angular_velocity: angular,
                ..RotationSpec::default()
            },
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.004, 0.004, 0.004, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap()
}

fn near_state(a: BodyState, b: BodyState) {
    for (x, y) in a
        .position
        .into_iter()
        .chain(a.orientation)
        .chain(a.velocity)
        .chain(a.angular_velocity)
        .zip(
            b.position
                .into_iter()
                .chain(b.orientation)
                .chain(b.velocity)
                .chain(b.angular_velocity),
        )
    {
        assert!((x - y).abs() < 1e-12, "{a:?} != {b:?}");
    }
}

fn obstacle(backend: &mut DartBackend, position: [f64; 3]) -> BodyId {
    backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position,
                velocity: [0.0; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap()
}

#[test]
fn refused_attachment_preserves_native_motion_through_retry() {
    let mut failed = DartBackend::new([0.0; 3]).unwrap();
    let mut clean = DartBackend::new([0.0; 3]).unwrap();
    let mut sets = Vec::new();
    for backend in [&mut failed, &mut clean] {
        let parent = spawn(backend, [0.0; 3], [0.2, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let child = spawn(backend, [0.0, 0.0, -2.0], [-3.8, 0.0, 0.0], [0.0, 3.0, 0.0]);
        backend
            .add_world_wrench(parent, [0.3, 0.0, 0.0], [0.0, 0.2, 0.0])
            .unwrap();
        {
            backend
                .add_world_revolute(parent, [0.0; 3], [0.0, 1.0, 0.0])
                .unwrap();
        }
        sets.push([parent, child]);
    }
    let contact = obstacle(&mut failed, [0.15, 0.0, 0.0]);
    let before = sets[0].map(|id| failed.state(id));
    for _ in 0..2 {
        assert_eq!(
            failed.add_revolute(sets[0][0], sets[0][1], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]),
            Err(PhysicsError::InvalidJoint)
        );
        assert_eq!(sets[0].map(|id| failed.state(id)), before);
    }
    assert!(failed.remove(contact));
    for (backend, bodies) in [(&mut failed, sets[0]), (&mut clean, sets[1])] {
        let joint = backend
            .add_revolute(bodies[0], bodies[1], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0])
            .unwrap();
        assert_eq!(joint.raw(), 1);
        backend.add_joint_effort(joint, 0.7).unwrap();
    }
    for (a, b) in sets[0].into_iter().zip(sets[1]) {
        near_state(failed.state(a).unwrap(), clean.state(b).unwrap());
    }
    for _ in 0..3 {
        failed.step(Duration::from_millis(1)).unwrap();
        clean.step(Duration::from_millis(1)).unwrap();
        for (a, b) in sets[0].into_iter().zip(sets[1]) {
            near_state(failed.state(a).unwrap(), clean.state(b).unwrap());
        }
    }
}

fn impulse_after_retirement(remove_body: bool) {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let parent = spawn(&mut backend, [0.0, 0.0, -1.0], [0.0; 3], [0.0; 3]);
    let child = spawn(&mut backend, [0.0, 0.0, -3.0], [0.0; 3], [0.0; 3]);
    let root = backend
        .add_world_revolute(parent, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    let leaf = backend
        .add_revolute(parent, child, [0.0, 0.0, -2.0], [0.0, 1.0, 0.0])
        .unwrap();
    backend.apply_impulse(parent, [0.0; 3], [0.0; 3]).unwrap();
    if remove_body {
        assert!(backend.remove(child));
    } else {
        assert!(backend.remove_joint(leaf));
    }
    backend
        .apply_impulse(parent, [0.0; 3], [0.0, 0.1, 0.0])
        .unwrap();
    // Parent COM is one metre from its hinge: I_axis=0.004 + 1*1².
    let expected = 0.1 / 1.004;
    assert!((backend.joint_state(root).unwrap().velocity - expected).abs() < 1e-12);
    assert!((backend.state(parent).unwrap().angular_velocity[1] - expected).abs() < 1e-12);
    assert!((backend.state(parent).unwrap().velocity[0] + expected).abs() < 1e-12);
    backend.step(Duration::from_millis(1)).unwrap();
    assert!((backend.joint_state(root).unwrap().position - 0.001 * expected).abs() < 1e-12);
}

#[test]
fn leaf_joint_retirement_preserves_parent_impulse_response() {
    impulse_after_retirement(false);
}

#[test]
fn leaf_body_retirement_preserves_parent_impulse_response() {
    impulse_after_retirement(true);
}

#[test]
fn leaf_retirement_preserves_parent_velocity_projection() {
    for remove_body in [false, true] {
        let mut backend = DartBackend::new([0.0; 3]).unwrap();
        let parent = spawn(&mut backend, [0.0, 0.0, -1.0], [0.0; 3], [0.0; 3]);
        let child = spawn(&mut backend, [0.0, 0.0, -3.0], [0.0; 3], [0.0; 3]);
        let root = backend
            .add_world_revolute(parent, [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap();
        let leaf = backend
            .add_revolute(parent, child, [0.0, 0.0, -2.0], [0.0, 1.0, 0.0])
            .unwrap();
        if remove_body {
            assert!(backend.remove(child));
        } else {
            assert!(backend.remove_joint(leaf));
        }
        backend
            .set_world_velocity(parent, [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
            .unwrap();
        assert_eq!(backend.joint_state(root).unwrap().velocity, 1.0);
        assert_eq!(backend.state(parent).unwrap().velocity, [-1.0, 0.0, 0.0]);
        assert_eq!(
            backend.state(parent).unwrap().angular_velocity,
            [0.0, 1.0, 0.0]
        );
        backend.step(Duration::from_millis(1)).unwrap();
        assert!((backend.joint_state(root).unwrap().position - 0.001).abs() < 1e-12);
    }
}
