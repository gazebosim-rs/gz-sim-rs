// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded world-root two-link articulation controls.

use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, JointId, PhysicsError, RotationSpec,
    Shape,
};
use gz_physics_dart::DartBackend;
use nalgebra::{Matrix2, Quaternion, UnitQuaternion, Vector2, Vector3};
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

fn chain(moving: bool) -> (DartBackend, [BodyId; 2], [JointId; 2]) {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let rate = if moving { 1.0 } else { 0.0 };
    let parent = spawn(
        &mut backend,
        [0.0, 0.0, -1.0],
        [-rate, 0.0, 0.0],
        [0.0, rate, 0.0],
    );
    let child = spawn(
        &mut backend,
        [0.0, 0.0, -3.0],
        [-5.0 * rate, 0.0, 0.0],
        [0.0, 3.0 * rate, 0.0],
    );
    let root = backend
        .add_world_revolute(parent, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    let leaf = backend
        .add_revolute(parent, child, [0.0, 0.0, -2.0], [0.0, 1.0, 0.0])
        .unwrap();
    (backend, [parent, child], [root, leaf])
}

fn rotation(state: BodyState) -> UnitQuaternion<f64> {
    let [x, y, z, w] = state.orientation;
    UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))
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

#[test]
fn moving_parent_and_child_keep_relative_rate_and_anchor_through_steps_and_resets() {
    let (mut backend, bodies, joints) = chain(true);
    let original = bodies.map(|id| backend.state(id).unwrap());
    let saved = backend.initial_state();
    assert_eq!(backend.joint_state(joints[0]).unwrap().velocity, 1.0);
    assert_eq!(backend.joint_state(joints[1]).unwrap().velocity, 2.0);
    assert_eq!(original[1].velocity, [-5.0, 0.0, 0.0]);
    assert_eq!(original[1].angular_velocity, [0.0, 3.0, 0.0]);
    for _ in 0..1000 {
        backend.step(Duration::from_millis(1)).unwrap();
        let [parent, child] = bodies.map(|id| backend.state(id).unwrap());
        let a = Vector3::from(parent.position) - rotation(parent) * Vector3::z();
        let b = Vector3::from(child.position) + rotation(child) * Vector3::z();
        assert!((a - b).norm() < 1e-10);
    }
    backend.reset().unwrap();
    for (id, expected) in bodies.into_iter().zip(original) {
        near_state(backend.state(id).unwrap(), expected);
    }
    assert!(backend.remove(bodies[0]));
    assert_eq!(backend.joint_state(joints[0]), None);
    assert_eq!(backend.joint_state(joints[1]), None);
    assert_eq!(backend.is_jointed(bodies[1]), Some(false));
    backend.step(Duration::from_millis(1)).unwrap();
    let prepared = backend.prepare_initial(&saved).unwrap();
    backend.apply_initial(prepared).unwrap();
    for (id, expected) in bodies.into_iter().zip(original) {
        near_state(backend.state(id).unwrap(), expected);
    }
    assert_eq!(backend.joint_child(joints[1]), Some(bodies[1]));
}

#[test]
fn projected_edits_preserve_ancestor_rates_and_refresh_descendant_caches() {
    let (mut backend, [parent, child], [root, leaf]) = chain(true);
    backend
        .set_world_velocity(child, [-7.0, 0.0, 0.0], [0.0, 5.0, 0.0])
        .unwrap();
    // Ancestor contributes -3 m/s and 1 rad/s at the child's COM.
    assert_eq!(backend.joint_state(root).unwrap().velocity, 1.0);
    assert_eq!(backend.joint_state(leaf).unwrap().velocity, 4.0);
    backend
        .set_world_velocity(parent, [-2.0, 0.0, 0.0], [0.0, 2.0, 0.0])
        .unwrap();
    assert_eq!(backend.joint_state(leaf).unwrap().velocity, 4.0);
    assert_eq!(backend.state(child).unwrap().velocity, [-10.0, 0.0, 0.0]);
    assert_eq!(
        backend.state(child).unwrap().angular_velocity,
        [0.0, 6.0, 0.0]
    );
    let states = [backend.state(parent), backend.state(child)];
    let rates = [backend.joint_state(root), backend.joint_state(leaf)];
    assert_eq!(
        backend.apply_impulse(child, [f64::MAX; 3], [f64::MAX; 3]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!([backend.state(parent), backend.state(child)], states);
    assert_eq!(
        [backend.joint_state(root), backend.joint_state(leaf)],
        rates
    );
}

#[test]
fn impulses_and_generalized_effort_use_coupled_mass_and_parent_reaction() {
    let mass = Matrix2::new(10.008, 3.004, 3.004, 1.004);
    for (target_child, generalized) in [
        (false, Vector2::new(1.0, 0.0)),
        (true, Vector2::new(1.0, 1.0)),
    ] {
        let (mut backend, bodies, joints) = chain(false);
        backend
            .apply_impulse(bodies[usize::from(target_child)], [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap();
        let expected = mass.lu().solve(&generalized).unwrap();
        for (id, expected) in joints.into_iter().zip(expected.iter()) {
            assert!((backend.joint_state(id).unwrap().velocity - expected).abs() < 1e-12);
        }
        assert!(
            (backend.state(bodies[0]).unwrap().angular_velocity[1] - expected[0]).abs() < 1e-12
        );
        assert!(
            (backend.state(bodies[1]).unwrap().angular_velocity[1] - expected.sum()).abs() < 1e-12
        );
    }
    let (mut backend, _, joints) = chain(false);
    backend.add_joint_effort(joints[1], 1.0).unwrap();
    backend.step(Duration::from_millis(1)).unwrap();
    let expected = mass.lu().solve(&Vector2::new(0.0, 0.001)).unwrap();
    for (id, expected) in joints.into_iter().zip(expected.iter()) {
        assert!((backend.joint_state(id).unwrap().velocity - expected).abs() < 1e-12);
    }
    assert!(backend.joint_state(joints[0]).unwrap().velocity < 0.0);
}

#[test]
fn retiring_either_joint_or_body_preserves_survivors_and_unrelated_ids() {
    for removed in 1..4 {
        let (mut backend, bodies, joints) = chain(true);
        let unrelated = spawn(&mut backend, [10.0, 0.0, 0.0], [0.0; 3], [0.0; 3]);
        backend.step(Duration::from_millis(1)).unwrap();
        let before = bodies.map(|id| backend.state(id).unwrap());
        let separate = backend.state(unrelated).unwrap();
        if removed < 2 {
            assert!(backend.remove_joint(joints[removed]));
            assert!(!backend.remove_joint(joints[removed]));
            for (id, expected) in bodies.into_iter().zip(before) {
                near_state(backend.state(id).unwrap(), expected);
            }
        } else {
            assert!(backend.remove(bodies[removed - 2]));
            assert!(!backend.remove(bodies[removed - 2]));
            near_state(
                backend.state(bodies[3 - removed]).unwrap(),
                before[3 - removed],
            );
        }
        near_state(backend.state(unrelated).unwrap(), separate);
        backend
            .apply_impulse(
                bodies[if removed == 2 { 1 } else { 0 }],
                [0.1, 0.0, 0.0],
                [0.0; 3],
            )
            .unwrap();
        backend.step(Duration::from_millis(1)).unwrap();
        backend.reset().unwrap();
        backend.step(Duration::from_millis(1)).unwrap();
        assert!(backend.state(unrelated).is_some());
    }
}

#[test]
fn unsupported_graphs_and_bad_joint_inputs_leave_all_endpoints_and_ids_unchanged() {
    let (mut backend, bodies, joints) = chain(true);
    let third = spawn(&mut backend, [3.0, 0.0, -3.0], [0.0; 3], [0.0; 3]);
    let initial = [
        backend.state(bodies[0]),
        backend.state(bodies[1]),
        backend.state(third),
    ];
    for (parent, child) in [
        (bodies[0], bodies[0]),
        (bodies[1], bodies[0]),
        (bodies[0], third),
        (bodies[1], third),
        (third, bodies[1]),
    ] {
        assert!(matches!(
            backend.add_revolute(parent, child, [0.0; 3], [0.0, 1.0, 0.0]),
            Err(PhysicsError::Unsupported { .. })
        ));
    }
    assert_eq!(
        [
            backend.state(bodies[0]),
            backend.state(bodies[1]),
            backend.state(third)
        ],
        initial
    );
    assert!(backend.remove_joint(joints[1]));
    for (anchor, axis) in [([f64::NAN; 3], [0.0, 1.0, 0.0]), ([0.0; 3], [0.0; 3])] {
        assert_eq!(
            backend.add_revolute(bodies[0], bodies[1], anchor, axis),
            Err(PhysicsError::InvalidJoint)
        );
    }
    let next = backend
        .add_revolute(bodies[0], bodies[1], [0.0, 0.0, -2.0], [0.0, 1.0, 0.0])
        .unwrap();
    assert_eq!(next.raw(), joints[1].raw() + 1);
    backend.step(Duration::from_millis(1)).unwrap();
    assert_eq!(
        backend.add_revolute(bodies[0], third, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::JointConstructionClosed)
    );
}

#[test]
fn contact_during_attachment_rolls_back_topology_without_consuming_joint_id() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let parent = spawn(&mut backend, [0.0, 0.0, -1.0], [0.0; 3], [0.0; 3]);
    let child = spawn(&mut backend, [0.0, 0.0, -3.0], [0.0; 3], [0.0; 3]);
    let root = backend
        .add_world_revolute(parent, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    let obstacle = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [0.0, 0.0, -1.15],
                velocity: [0.0; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    let before = [backend.state(parent), backend.state(child)];
    assert_eq!(
        backend.add_revolute(parent, child, [0.0, 0.0, -2.0], [0.0, 1.0, 0.0]),
        Err(PhysicsError::InvalidJoint)
    );
    assert_eq!([backend.state(parent), backend.state(child)], before);
    assert_eq!(backend.is_jointed(child), Some(false));
    assert!(backend.remove(obstacle));
    let leaf = backend
        .add_revolute(parent, child, [0.0, 0.0, -2.0], [0.0, 1.0, 0.0])
        .unwrap();
    assert_eq!(leaf.raw(), root.raw() + 1);
    backend.step(Duration::from_millis(1)).unwrap();
}

#[test]
fn approaching_contact_refuses_atomically_and_preserves_pending_effort() {
    let (mut failed, bodies, joints) = chain(true);
    let (mut clean, clean_bodies, clean_joints) = chain(true);
    let obstacle = failed
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [-0.205, 0.0, -3.0],
                velocity: [0.0; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    let mut refused = false;
    for _ in 0..10 {
        failed.add_joint_effort(joints[1], 0.7).unwrap();
        clean.add_joint_effort(clean_joints[1], 0.7).unwrap();
        let before = bodies.map(|id| failed.state(id));
        let rates = joints.map(|id| failed.joint_state(id));
        if let Err(error) = failed.step(Duration::from_millis(1)) {
            assert!(error.to_string().contains("contact"), "{error}");
            assert_eq!(bodies.map(|id| failed.state(id)), before);
            assert_eq!(joints.map(|id| failed.joint_state(id)), rates);
            assert!(failed.step(Duration::from_millis(1)).is_err());
            assert_eq!(bodies.map(|id| failed.state(id)), before);
            assert!(failed.remove(obstacle));
            refused = true;
            break;
        }
        clean.step(Duration::from_millis(1)).unwrap();
    }
    assert!(refused);
    for _ in 0..3 {
        failed.step(Duration::from_millis(1)).unwrap();
        clean.step(Duration::from_millis(1)).unwrap();
        for (left, right) in bodies.into_iter().zip(clean_bodies) {
            near_state(failed.state(left).unwrap(), clean.state(right).unwrap());
        }
    }
}

fn array<const N: usize>(value: &serde_json::Value) -> [f64; N] {
    std::array::from_fn(|i| value[i].as_f64().unwrap())
}

#[test]
fn pinned_gazebo_dart_aligned_and_rotated_moving_parent_force_trajectories_match() {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/moving_parent.json")).unwrap();
    for tilted in [false, true] {
        let rows: Vec<_> = fixture["observations"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|row| row["tilted"] == tilted)
            .collect();
        let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
        let mut bodies = Vec::new();
        for body in rows[0]["bodies"].as_array().unwrap() {
            bodies.push(
                backend
                    .spawn_link(
                        BodySpec {
                            shape: Shape::Sphere { radius: 0.1 },
                            position: array(&body["position"]),
                            velocity: array(&body["velocity"]),
                            mass: Some(1.0),
                        },
                        RotationSpec {
                            orientation: array(&body["orientation"]),
                            angular_velocity: array(&body["angular_velocity"]),
                        },
                        Some(Inertia {
                            center: array(&body["center"]),
                            orientation: [0.0, 0.0, 0.0, 1.0],
                            tensor: [0.004, 0.004, 0.004, 0.0, 0.0, 0.0],
                        }),
                    )
                    .unwrap(),
            );
        }
        let initial = &rows[0]["bodies"];
        let root = backend
            .add_world_revolute(
                bodies[0],
                array(&initial[0]["anchor"]),
                array(&initial[0]["axis"]),
            )
            .unwrap();
        let leaf = backend
            .add_revolute(
                bodies[0],
                bodies[1],
                array(&initial[1]["anchor"]),
                array(&initial[1]["axis"]),
            )
            .unwrap();
        let joints = [root, leaf];
        let mut tick = 0;
        for row in rows {
            while tick < row["tick"].as_u64().unwrap() {
                tick += 1;
                let effort = match tick {
                    1 => 1.5,
                    3 => -3.0,
                    5 => 2.0,
                    _ => 0.0,
                };
                backend.add_joint_effort(leaf, effort).unwrap();
                backend.step(Duration::from_millis(1)).unwrap();
            }
            for i in 0..2 {
                let expected = &row["bodies"][i];
                let state = backend.state(bodies[i]).unwrap();
                let joint = backend.joint_state(joints[i]).unwrap();
                let actual = state
                    .position
                    .into_iter()
                    .chain(state.orientation)
                    .chain(state.velocity)
                    .chain(state.angular_velocity)
                    .chain([joint.position, joint.velocity]);
                let reference = array::<3>(&expected["position"])
                    .into_iter()
                    .chain(array::<4>(&expected["orientation"]))
                    .chain(array::<3>(&expected["velocity"]))
                    .chain(array::<3>(&expected["angular_velocity"]))
                    .chain([
                        expected["q"].as_f64().unwrap(),
                        expected["dq"].as_f64().unwrap(),
                    ]);
                for (a, b) in actual.zip(reference) {
                    assert!(
                        (a - b).abs() < 1e-12,
                        "tilted={tilted} tick={tick} body={i}: {a} != {b}"
                    );
                }
            }
        }
    }
}

#[test]
fn missing_static_and_out_of_domain_joint_inputs_are_atomic() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let parent = spawn(&mut backend, [0.0; 3], [0.0; 3], [0.0, 0.0, 1e60]);
    let child = spawn(&mut backend, [0.0, 0.0, 0.3], [0.0; 3], [0.0, 0.0, -1e60]);
    backend
        .add_world_revolute(parent, [0.0; 3], [0.0, 0.0, 1.0])
        .unwrap();
    let before = [backend.state(parent), backend.state(child)];
    for (a, b) in [
        (BodyId::from_raw(u64::MAX), child),
        (parent, BodyId::from_raw(u64::MAX)),
    ] {
        assert_eq!(
            backend.add_revolute(a, b, [0.0; 3], [0.0, 0.0, 1.0]),
            Err(PhysicsError::MissingBody)
        );
    }
    // Each body is within the velocity limit, but its relative rate is not.
    assert_eq!(
        backend.add_revolute(parent, child, [0.0; 3], [0.0, 0.0, 1.0]),
        Err(PhysicsError::InvalidJoint)
    );
    assert_eq!([backend.state(parent), backend.state(child)], before);
    assert_eq!(
        backend.add_revolute(parent, child, [1e8, 0.0, 0.0], [1.0, 0.0, 0.0]),
        Err(PhysicsError::InvalidJoint)
    );
    let fixed = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [3.0; 3],
                velocity: [0.0; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    for (a, b) in [(fixed, child), (parent, fixed)] {
        assert!(matches!(
            backend.add_revolute(a, b, [0.0; 3], [0.0, 0.0, 1.0]),
            Err(PhysicsError::Unsupported { .. })
        ));
    }
    assert_eq!([backend.state(parent), backend.state(child)], before);
}

#[test]
fn body_joint_effort_is_one_shot_and_both_resets_discard_pending_commands() {
    let (mut backend, bodies, joints) = chain(false);
    let (mut zeroed, controls, control_joints) = chain(false);
    let initial = backend.initial_state();
    for (world, leaf) in [(&mut backend, joints[1]), (&mut zeroed, control_joints[1])] {
        world.add_joint_effort(leaf, 1.0).unwrap();
        world.step(Duration::ZERO).unwrap();
        world.step(Duration::from_millis(1)).unwrap();
    }
    for _ in 0..10 {
        zeroed.add_joint_effort(control_joints[1], 0.0).unwrap();
        backend.step(Duration::from_millis(1)).unwrap();
        zeroed.step(Duration::from_millis(1)).unwrap();
        for (a, b) in bodies.into_iter().zip(controls) {
            near_state(backend.state(a).unwrap(), zeroed.state(b).unwrap());
        }
    }
    for loaded in [false, true] {
        backend.add_joint_effort(joints[1], 1.0).unwrap();
        if loaded {
            let prepared = backend.prepare_initial(&initial).unwrap();
            backend.apply_initial(prepared).unwrap();
        } else {
            backend.reset().unwrap();
        }
        backend.step(Duration::from_millis(1)).unwrap();
        assert_eq!(backend.joint_state(joints[0]).unwrap().velocity, 0.0);
        assert_eq!(backend.joint_state(joints[1]).unwrap().velocity, 0.0);
    }
}

#[test]
fn predictive_default_limits_refuse_before_native_solve_and_keep_commands() {
    let (mut backend, bodies, joints) = chain(false);
    let (mut clean, controls, control_joints) = chain(false);
    backend
        .set_world_velocity(bodies[0], [-1e20, 0.0, 0.0], [0.0, 1e20, 0.0])
        .unwrap();
    backend.add_joint_effort(joints[1], 0.7).unwrap();
    backend
        .add_world_wrench(bodies[0], [0.0; 3], [0.0, 0.2, 0.0])
        .unwrap();
    let before = bodies.map(|id| backend.state(id));
    let rates = joints.map(|id| backend.joint_state(id));
    for _ in 0..2 {
        let error = backend.step(Duration::from_millis(1)).unwrap_err();
        assert!(error.to_string().contains("active joint limit"), "{error}");
        assert_eq!(bodies.map(|id| backend.state(id)), before);
        assert_eq!(joints.map(|id| backend.joint_state(id)), rates);
    }
    backend
        .set_world_velocity(bodies[0], [0.0; 3], [0.0; 3])
        .unwrap();
    clean.add_joint_effort(control_joints[1], 0.7).unwrap();
    clean
        .add_world_wrench(controls[0], [0.0; 3], [0.0, 0.2, 0.0])
        .unwrap();
    for _ in 0..3 {
        backend.step(Duration::from_millis(1)).unwrap();
        clean.step(Duration::from_millis(1)).unwrap();
        for (a, b) in bodies.into_iter().zip(controls) {
            near_state(backend.state(a).unwrap(), clean.state(b).unwrap());
        }
    }
}

#[test]
fn root_joint_removal_is_atomic_until_its_body_child_is_detached() {
    let (mut backend, bodies, joints) = chain(true);
    let (mut clean, controls, control_joints) = chain(true);
    let before = bodies.map(|id| backend.state(id));
    let rates = joints.map(|id| backend.joint_state(id));
    for (world, body, leaf) in [
        (&mut backend, bodies[0], joints[1]),
        (&mut clean, controls[0], control_joints[1]),
    ] {
        world.add_joint_effort(leaf, 0.7).unwrap();
        world
            .add_world_wrench(body, [0.0; 3], [0.0, 0.2, 0.0])
            .unwrap();
    }
    assert!(!backend.remove_joint(joints[0]));
    assert_eq!(bodies.map(|id| backend.state(id)), before);
    assert_eq!(joints.map(|id| backend.joint_state(id)), rates);
    assert_eq!(backend.joint_child(joints[0]), Some(bodies[0]));
    for _ in 0..3 {
        backend.step(Duration::from_millis(1)).unwrap();
        clean.step(Duration::from_millis(1)).unwrap();
        for (a, b) in bodies.into_iter().zip(controls) {
            near_state(backend.state(a).unwrap(), clean.state(b).unwrap());
        }
    }
    assert!(backend.remove_joint(joints[1]));
    assert!(backend.remove_joint(joints[0]));
    assert_eq!(backend.is_jointed(bodies[0]), Some(false));
    assert_eq!(backend.is_jointed(bodies[1]), Some(false));
    backend.step(Duration::from_millis(1)).unwrap();
}
