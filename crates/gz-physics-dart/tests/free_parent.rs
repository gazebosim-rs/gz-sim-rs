// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Contact-free two-body articulation with six free-root coordinates.

use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, JointId, PhysicsError, RotationSpec,
    Shape,
};
use gz_physics_dart::DartBackend;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
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

fn array<const N: usize>(value: &serde_json::Value) -> [f64; N] {
    std::array::from_fn(|i| value[i].as_f64().unwrap())
}

#[test]
fn pinned_gazebo_dart_free_root_twists_and_child_pulses_match() {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/free_parent.json")).unwrap();
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
        let leaf = backend
            .add_revolute(
                bodies[0],
                bodies[1],
                array(&initial[1]["anchor"]),
                array(&initial[1]["axis"]),
            )
            .unwrap();
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
                let joint = if i == 0 {
                    gz_physics::JointState {
                        position: 0.0,
                        velocity: 0.0,
                    }
                } else {
                    backend.joint_state(leaf).unwrap()
                };
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

fn pair(moving: bool, coincident: bool) -> (DartBackend, [BodyId; 2], JointId) {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let rate = if moving { 1.0 } else { 0.0 };
    let parent = spawn(&mut backend, [0.0; 3], [rate, 0.0, 0.0], [0.0, rate, 0.0]);
    let separation = if coincident { 0.0 } else { 2.0 };
    let child = spawn(
        &mut backend,
        [0.0, 0.0, -separation],
        [rate * (1.0 - 2.0 * separation), 0.0, 0.0],
        [0.0, 3.0 * rate, 0.0],
    );
    let leaf = backend
        .add_revolute(
            parent,
            child,
            [0.0, 0.0, -separation / 2.0],
            [0.0, 1.0, 0.0],
        )
        .unwrap();
    (backend, [parent, child], leaf)
}

#[test]
fn root_edits_refresh_child_and_child_projection_keeps_all_root_rates() {
    let (mut backend, [parent, child], leaf) = pair(true, false);
    assert_eq!(backend.joint_state(leaf).unwrap().velocity, 2.0);
    backend
        .set_world_velocity(parent, [2.0, 3.0, 4.0], [0.0, 2.0, 0.0])
        .unwrap();
    assert_eq!(backend.state(child).unwrap().velocity, [-4.0, 3.0, 4.0]);
    assert_eq!(
        backend.state(child).unwrap().angular_velocity,
        [0.0, 4.0, 0.0]
    );
    assert_eq!(backend.joint_state(leaf).unwrap().velocity, 2.0);
    let root = backend.state(parent).unwrap();
    backend
        .set_world_velocity(child, [-6.0, 3.0, 4.0], [0.0, 6.0, 0.0])
        .unwrap();
    assert_eq!(backend.state(parent).unwrap(), root);
    assert_eq!(backend.joint_state(leaf).unwrap().velocity, 4.0);
    let before = [backend.state(parent), backend.state(child)];
    for target in [parent, child] {
        assert_eq!(
            backend.set_world_velocity(target, [f64::MAX; 3], [0.0; 3]),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!(
            backend.apply_impulse(target, [f64::MAX; 3], [f64::MAX; 3]),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!([backend.state(parent), backend.state(child)], before);
        assert_eq!(backend.joint_state(leaf).unwrap().velocity, 4.0);
    }
    for _ in 0..100 {
        backend.step(Duration::from_millis(1)).unwrap();
        let a = backend.state(parent).unwrap();
        let b = backend.state(child).unwrap();
        assert!(
            (Vector3::from(a.position)
                - rotation(a) * Vector3::z()
                - Vector3::from(b.position)
                - rotation(b) * Vector3::z())
            .norm()
                < 1e-10
        );
    }
}

#[test]
fn coupled_impulses_and_internal_effort_conserve_linear_and_angular_momentum() {
    for target_child in [false, true] {
        let (mut backend, bodies, leaf) = pair(false, true);
        backend
            .apply_impulse(
                bodies[usize::from(target_child)],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            )
            .unwrap();
        for (index, body) in bodies.into_iter().enumerate() {
            let state = backend.state(body).unwrap();
            assert_eq!(state.velocity, [0.5, 0.0, 0.0]);
            let angular = if (index == 1) == target_child {
                250.0
            } else {
                0.0
            };
            assert!((state.angular_velocity[1] - angular).abs() < 1e-12);
        }
        assert!(
            (backend.joint_state(leaf).unwrap().velocity
                - if target_child { 250.0 } else { -250.0 })
            .abs()
                < 1e-12
        );
    }
    let (mut backend, bodies, leaf) = pair(false, true);
    backend.add_joint_effort(leaf, 1.0).unwrap();
    backend.step(Duration::ZERO).unwrap();
    backend.step(Duration::from_millis(1)).unwrap();
    assert!((backend.state(bodies[0]).unwrap().angular_velocity[1] + 0.25).abs() < 1e-12);
    assert!((backend.state(bodies[1]).unwrap().angular_velocity[1] - 0.25).abs() < 1e-12);
    assert!((backend.joint_state(leaf).unwrap().velocity - 0.5).abs() < 1e-12);
    backend.step(Duration::from_millis(1)).unwrap();
    assert!((backend.joint_state(leaf).unwrap().velocity - 0.5).abs() < 1e-12);
}

#[test]
fn resets_and_endpoint_retirement_preserve_ids_survivors_and_pending_scope() {
    for retirement in 0..3 {
        let (mut backend, bodies, leaf) = pair(true, false);
        let sibling = spawn(&mut backend, [20.0, 0.0, 0.0], [0.0; 3], [0.0; 3]);
        let saved = backend.initial_state();
        let initial = bodies.map(|id| backend.state(id).unwrap());
        backend.add_joint_effort(leaf, 0.7).unwrap();
        backend.step(Duration::from_millis(1)).unwrap();
        let before = bodies.map(|id| backend.state(id).unwrap());
        let unrelated = backend.state(sibling).unwrap();
        backend.add_joint_effort(leaf, 1.0).unwrap();
        if retirement == 0 {
            assert!(backend.remove_joint(leaf));
            for (id, state) in bodies.into_iter().zip(before) {
                near_state(backend.state(id).unwrap(), state);
            }
        } else {
            assert!(backend.remove(bodies[retirement - 1]));
            near_state(
                backend.state(bodies[2 - retirement]).unwrap(),
                before[2 - retirement],
            );
        }
        assert_eq!(backend.joint_state(leaf), None);
        assert!(!backend.remove_joint(leaf));
        near_state(backend.state(sibling).unwrap(), unrelated);
        backend.step(Duration::from_millis(1)).unwrap();
        let prepared = backend.prepare_initial(&saved).unwrap();
        backend.apply_initial(prepared).unwrap();
        for (id, state) in bodies.into_iter().zip(initial) {
            near_state(backend.state(id).unwrap(), state);
        }
        assert_eq!(backend.joint_child(leaf), Some(bodies[1]));
        assert_eq!(backend.joint_state(leaf).unwrap().velocity, 2.0);
        backend.add_joint_effort(leaf, 1.0).unwrap();
        backend.reset().unwrap();
        let (mut clean, controls, _) = pair(true, false);
        for _ in 0..3 {
            backend.step(Duration::from_millis(1)).unwrap();
            clean.step(Duration::from_millis(1)).unwrap();
            for (a, b) in bodies.into_iter().zip(controls) {
                near_state(backend.state(a).unwrap(), clean.state(b).unwrap());
            }
        }
    }
}

#[test]
fn unsupported_graphs_keep_endpoints_joint_ids_and_pending_inputs() {
    let (mut backend, bodies, leaf) = pair(false, false);
    let extra = spawn(&mut backend, [10.0, 0.0, 0.0], [0.0; 3], [0.0; 3]);
    let before = [
        backend.state(bodies[0]),
        backend.state(bodies[1]),
        backend.state(extra),
    ];
    for (parent, child) in [
        (bodies[0], extra),
        (bodies[1], extra),
        (extra, bodies[0]),
        (extra, bodies[1]),
        (bodies[1], bodies[0]),
        (bodies[0], bodies[0]),
    ] {
        assert!(matches!(
            backend.add_revolute(parent, child, [0.0; 3], [0.0, 1.0, 0.0]),
            Err(PhysicsError::Unsupported { .. })
        ));
    }
    assert!(matches!(
        backend.add_world_revolute(bodies[0], [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::Unsupported { .. })
    ));
    assert_eq!(
        [
            backend.state(bodies[0]),
            backend.state(bodies[1]),
            backend.state(extra)
        ],
        before
    );
    assert!(backend.remove_joint(leaf));
    let next = backend
        .add_revolute(bodies[0], bodies[1], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0])
        .unwrap();
    assert_eq!(next.raw(), leaf.raw() + 1);
    backend.step(Duration::from_millis(1)).unwrap();
    assert_eq!(
        backend.add_revolute(bodies[0], extra, [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::JointConstructionClosed)
    );
}

#[test]
fn original_free_pair_rejection_input_has_an_exact_positive_control_for_review() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let mut bodies = Vec::new();
    for x in [0.0, 2.0] {
        bodies.push(
            backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.5 },
                        position: [x, 0.0, 5.0],
                        velocity: [0.0; 3],
                        mass: Some(1.0),
                    },
                    RotationSpec::default(),
                    None,
                )
                .unwrap(),
        );
    }
    let before = bodies
        .iter()
        .map(|id| backend.state(*id))
        .collect::<Vec<_>>();
    let joint = backend
        .add_revolute(bodies[0], bodies[1], [0.0, 0.0, 5.0], [0.0, 1.0, 0.0])
        .unwrap();
    assert_eq!(
        bodies
            .iter()
            .map(|id| backend.state(*id))
            .collect::<Vec<_>>(),
        before
    );
    assert_eq!(backend.joint_child(joint), Some(bodies[1]));
    for body in &bodies {
        assert_eq!(backend.is_jointed(*body), Some(true));
    }
    backend.add_joint_effort(joint, 1.0).unwrap();
    backend.step(Duration::from_millis(1)).unwrap();
    assert!(backend.joint_state(joint).unwrap().velocity > 0.0);
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

fn check_static_contact_reference(
    backend: &mut DartBackend,
    bodies: [BodyId; 2],
    leaf: JointId,
    scenario: u64,
) {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/static_contacts.json")).unwrap();
    let mut tick = 0;
    let mut observations = 0;
    for row in fixture["observations"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|row| row["scenario"].as_u64().unwrap() == scenario)
    {
        while tick < row["tick"].as_u64().unwrap() {
            tick += 1;
            if scenario == 3 || tick == 1 {
                backend.add_joint_effort(leaf, 0.7).unwrap();
            }
            backend.step(Duration::from_millis(1)).unwrap();
        }
        for (index, id) in bodies.into_iter().enumerate() {
            let expected = &row["bodies"][index];
            near_state(
                backend.state(id).unwrap(),
                BodyState {
                    position: array(&expected["position"]),
                    orientation: array(&expected["orientation"]),
                    velocity: array(&expected["velocity"]),
                    angular_velocity: array(&expected["angular_velocity"]),
                },
            );
        }
        let state = backend.joint_state(leaf).unwrap();
        assert!((state.position - row["q"].as_f64().unwrap()).abs() < 1e-12);
        assert!((state.velocity - row["dq"].as_f64().unwrap()).abs() < 1e-12);
        observations += 1;
    }
    assert_eq!(observations, 17);
    assert_eq!(tick, 100);
}

#[test]
fn static_attachment_preserves_native_motion_and_pending_wrench() {
    attachment_rollback(false);
}

#[test]
fn published_world_root_attachment_rollback_preserves_native_motion() {
    attachment_rollback(true);
}

fn attachment_rollback(world_root: bool) {
    let mut failed = DartBackend::new([0.0; 3]).unwrap();
    let mut clean = DartBackend::new([0.0; 3]).unwrap();
    let mut sets = Vec::new();
    for backend in [&mut failed, &mut clean] {
        let parent = spawn(backend, [0.0; 3], [0.2, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let child = spawn(backend, [0.0, 0.0, -2.0], [-3.8, 0.0, 0.0], [0.0, 3.0, 0.0]);
        backend
            .add_world_wrench(parent, [0.3, 0.0, 0.0], [0.0, 0.2, 0.0])
            .unwrap();
        if world_root {
            backend
                .add_world_revolute(parent, [0.0; 3], [0.0, 1.0, 0.0])
                .unwrap();
        }
        sets.push([parent, child]);
    }
    let contact = obstacle(&mut failed, [0.15, 0.0, 0.0]);
    let before = sets[0].map(|id| failed.state(id));
    if !world_root {
        let joint = failed
            .add_revolute(sets[0][0], sets[0][1], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0])
            .unwrap();
        assert_eq!(joint.raw(), 0);
        assert_eq!(sets[0].map(|id| failed.state(id)), before);
        check_static_contact_reference(&mut failed, sets[0], joint, 0);
        let before = sets[0].map(|id| failed.state(id).unwrap());
        assert!(failed.remove_joint(joint));
        for (id, state) in sets[0].into_iter().zip(before) {
            near_state(failed.state(id).unwrap(), state);
        }
        assert!(failed.remove(contact));
        failed.step(Duration::from_millis(1)).unwrap();
        return;
    }
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
        assert_eq!(joint.raw(), u64::from(world_root));
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

#[test]
fn approaching_static_contact_matches_native_and_active_limits_keep_rollback() {
    for limit in [false, true] {
        let (mut failed, bodies, leaf) = pair(false, false);
        let (mut clean, controls, control_leaf) = pair(false, false);
        let contact = if limit {
            failed
                .set_world_velocity(bodies[1], [-1e20, 0.0, 0.0], [0.0, 1e20, 0.0])
                .unwrap();
            None
        } else {
            for (backend, root) in [(&mut failed, bodies[0]), (&mut clean, controls[0])] {
                backend
                    .set_world_velocity(root, [1.0, 0.0, 0.0], [0.0; 3])
                    .unwrap();
            }
            Some(obstacle(&mut failed, [0.205, 0.0, 0.0]))
        };
        if !limit {
            check_static_contact_reference(&mut failed, bodies, leaf, 3);
            assert!(failed.remove(contact.unwrap()));
            failed.step(Duration::from_millis(1)).unwrap();
            continue;
        }
        let mut refused = false;
        for _ in 0..20 {
            failed.add_joint_effort(leaf, 0.7).unwrap();
            clean.add_joint_effort(control_leaf, 0.7).unwrap();
            let before = bodies.map(|id| failed.state(id));
            let rate = failed.joint_state(leaf);
            if let Err(error) = failed.step(Duration::from_millis(1)) {
                assert!(
                    error.to_string().contains(if limit {
                        "active joint limit"
                    } else {
                        "contact"
                    }),
                    "{error}"
                );
                assert_eq!(bodies.map(|id| failed.state(id)), before);
                assert_eq!(failed.joint_state(leaf), rate);
                assert!(failed.step(Duration::from_millis(1)).is_err());
                assert_eq!(bodies.map(|id| failed.state(id)), before);
                if let Some(contact) = contact {
                    assert!(failed.remove(contact));
                } else {
                    failed
                        .set_world_velocity(bodies[1], [0.0; 3], [0.0; 3])
                        .unwrap();
                }
                refused = true;
                break;
            }
            clean.step(Duration::from_millis(1)).unwrap();
        }
        assert!(refused);
        for _ in 0..3 {
            failed.step(Duration::from_millis(1)).unwrap();
            clean.step(Duration::from_millis(1)).unwrap();
            for (a, b) in bodies.into_iter().zip(controls) {
                near_state(failed.state(a).unwrap(), clean.state(b).unwrap());
            }
        }
    }
}
