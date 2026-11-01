// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exact former refusal input, always exercised in fresh independent worlds.
use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, JointId, RotationSpec, Shape,
};
use gz_physics_dart::DartBackend;
use std::time::Duration;

fn stationary_pair() -> (DartBackend, [BodyId; 3], JointId) {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let spawn = |backend: &mut DartBackend, position| {
        backend
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.1 },
                    position,
                    velocity: [0.0; 3],
                    mass: Some(1.0),
                },
                RotationSpec::default(),
                Some(Inertia {
                    center: [0.0; 3],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                    tensor: [0.004, 0.004, 0.004, 0.0, 0.0, 0.0],
                }),
            )
            .unwrap()
    };
    let root = spawn(&mut backend, [0.0; 3]);
    let middle = spawn(&mut backend, [0.0, 0.0, -2.0]);
    let first = backend
        .add_revolute(root, middle, [0.0, 0.0, -1.0], [0.0, 1.0, 0.0])
        .unwrap();
    let extra = spawn(&mut backend, [10.0, 0.0, 0.0]);
    (backend, [root, middle, extra], first)
}

fn near(a: BodyState, b: BodyState) {
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

pub fn assert_exact_stationary_leaf_extension() {
    let (mut candidate, bodies, first) = stationary_pair();
    let (mut clean, controls, control_first) = stationary_pair();
    let before = bodies.map(|id| candidate.state(id));
    candidate
        .add_world_wrench(bodies[0], [0.2, -0.1, 0.3], [0.0, 0.4, 0.0])
        .unwrap();
    candidate.add_joint_effort(first, 0.7).unwrap();
    let second = candidate
        .add_revolute(bodies[1], bodies[2], [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    let control_second = clean
        .add_revolute(controls[1], controls[2], [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert_eq!(second.raw(), first.raw() + 1);
    // Preserve the exact original rejection test's complete snapshot strength.
    assert_eq!(bodies.map(|id| candidate.state(id)), before);
    assert_eq!(candidate.joint_child(first), Some(bodies[1]));
    assert_eq!(candidate.joint_child(second), Some(bodies[2]));
    clean
        .add_world_wrench(controls[0], [0.2, -0.1, 0.3], [0.0, 0.4, 0.0])
        .unwrap();
    clean.add_joint_effort(control_first, 0.7).unwrap();
    for backend in [&mut candidate, &mut clean] {
        backend.step(Duration::from_millis(1)).unwrap();
    }
    for _ in 0..3 {
        for (a, b) in bodies.into_iter().zip(controls) {
            near(candidate.state(a).unwrap(), clean.state(b).unwrap());
        }
        assert_eq!(
            candidate.joint_state(second),
            clean.joint_state(control_second)
        );
        candidate.step(Duration::from_millis(1)).unwrap();
        clean.step(Duration::from_millis(1)).unwrap();
    }
}
