// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Prepared three-body controls. Native-first review precedes admission changes.
use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, JointId, PhysicsError, RotationSpec,
    Shape,
};
use gz_physics_dart::DartBackend;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use std::time::Duration;

#[path = "support/free_chain.rs"]
mod free_chain_support;
#[path = "support/three_body_static_positive.rs"]
mod three_body_static_positive;
#[path = "support/three_body_static_contacts.rs"]
mod three_body_static_support;

const CHILD: &str = "GZ_DART_FREE_CHAIN_CHILD";
const CASE: &str = "GZ_DART_FREE_CHAIN_CASE";

fn in_child(test: &str) -> bool {
    match std::env::var(CHILD) {
        Ok(selected) => {
            assert_eq!(selected, test);
            true
        }
        Err(std::env::VarError::NotPresent) => false,
        Err(error) => panic!("{error}"),
    }
}

fn run_child(test: &str, case: Option<usize>) -> String {
    use std::process::{Command, Stdio};
    use std::time::Instant;
    let mut command = Command::new(std::env::current_exe().unwrap());
    command
        .args(["--exact", test, "--nocapture"])
        .env(CHILD, test)
        .env_remove(CASE)
        .env_remove("LD_PRELOAD")
        .env_remove("LD_AUDIT")
        .env_remove("LD_LIBRARY_PATH")
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    if let Some(case) = case {
        command.env(CASE, case.to_string());
    }
    let mut child = command.spawn().unwrap();
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut timeout = false;
    while child.try_wait().unwrap().is_none() {
        if Instant::now() >= deadline {
            timeout = true;
            child.kill().unwrap();
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    let output = child.wait_with_output().unwrap();
    let stdout = String::from_utf8(output.stdout).unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        !timeout && output.status.success(),
        "{test} case {case:?}: {:?}\n{stdout}\n{stderr}",
        output.status
    );
    assert!(
        stdout.contains(&format!("test {test} ... ok"))
            && stdout.contains("test result: ok. 1 passed; 0 failed; 0 ignored;"),
        "child did not execute exactly one control: {stdout}"
    );
    stdout
}

fn body(backend: &mut DartBackend, position: [f64; 3], linear: [f64; 3], rate: f64) -> BodyId {
    backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position,
                velocity: linear,
                mass: Some(1.0),
            },
            RotationSpec {
                angular_velocity: [0.0, rate, 0.0],
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

fn pair(moving: bool) -> (DartBackend, [BodyId; 3], JointId) {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let root = body(
        &mut backend,
        [0.0; 3],
        if moving { [0.3, -0.2, 0.1] } else { [0.0; 3] },
        if moving { 1.0 } else { 0.0 },
    );
    let middle = body(
        &mut backend,
        [0.0, 0.0, -2.0],
        if moving { [-3.7, -0.2, 0.1] } else { [0.0; 3] },
        if moving { 3.0 } else { 0.0 },
    );
    let leaf = body(
        &mut backend,
        if moving {
            [0.0, 0.0, -4.0]
        } else {
            [10.0, 0.0, 0.0]
        },
        if moving { [-12.7, -0.2, 0.1] } else { [0.0; 3] },
        if moving { 6.0 } else { 0.0 },
    );
    let joint = backend
        .add_revolute(root, middle, [0.0, 0.0, -1.0], [0.0, 1.0, 0.0])
        .unwrap();
    (backend, [root, middle, leaf], joint)
}

fn finish(backend: &mut DartBackend, bodies: [BodyId; 3], moving: bool) -> JointId {
    backend
        .add_revolute(
            bodies[1],
            bodies[2],
            if moving { [0.0, 0.0, -3.0] } else { [0.0; 3] },
            [0.0, 1.0, 0.0],
        )
        .unwrap()
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

fn point(state: BodyState, local: [f64; 3]) -> Vector3<f64> {
    let [x, y, z, w] = state.orientation;
    Vector3::from(state.position)
        + UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z)) * Vector3::from(local)
}

fn array<const N: usize>(value: &serde_json::Value) -> [f64; N] {
    std::array::from_fn(|index| value[index].as_f64().unwrap())
}

#[test]
fn actual_native_eight_coordinate_observations_match_every_body_and_joint() {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/free_chain.json")).unwrap();
    assert_eq!(fixture["dart_version"], "6.13.2");
    assert_eq!(fixture["input"]["dt_ns"], 1_000_000);
    for moving in [false, true] {
        let rows: Vec<_> = fixture["observations"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|row| row["moving"] == moving)
            .collect();
        assert_eq!(rows.len(), 7);
        let mut backend = DartBackend::new(array(&fixture["input"]["gravity"])).unwrap();
        let mut bodies = Vec::new();
        for input in rows[0]["bodies"].as_array().unwrap() {
            bodies.push(
                backend
                    .spawn_link(
                        BodySpec {
                            shape: Shape::Sphere {
                                radius: fixture["input"]["shape"]["sphere_radius"]
                                    .as_f64()
                                    .unwrap(),
                            },
                            position: array(&input["position"]),
                            velocity: array(&input["velocity"]),
                            mass: Some(fixture["input"]["mass"].as_f64().unwrap()),
                        },
                        RotationSpec {
                            orientation: array(&input["orientation"]),
                            angular_velocity: array(&input["angular_velocity"]),
                        },
                        Some(Inertia {
                            center: array(&input["center"]),
                            orientation: array(&fixture["input"]["inertial_orientation"]),
                            tensor: array(&fixture["input"]["inertia_tensor"]),
                        }),
                    )
                    .unwrap(),
            );
        }
        let mut joints = Vec::new();
        for child in 1..3 {
            let input = &rows[0]["bodies"][child];
            joints.push(
                backend
                    .add_revolute(
                        bodies[child - 1],
                        bodies[child],
                        array(&input["anchor"]),
                        array(&input["axis"]),
                    )
                    .unwrap(),
            );
        }
        let mut tick = 0;
        for row in rows {
            while tick < row["tick"].as_u64().unwrap() {
                tick += 1;
                if let Some(effort) = fixture["input"]["leaf_efforts_by_tick"].get(tick.to_string())
                {
                    backend
                        .add_joint_effort(joints[1], effort.as_f64().unwrap())
                        .unwrap();
                }
                backend.step(Duration::from_millis(1)).unwrap();
            }
            for (i, id) in bodies.iter().enumerate() {
                let expected = &row["bodies"][i];
                near(
                    backend.state(*id).unwrap(),
                    BodyState {
                        position: array(&expected["position"]),
                        orientation: array(&expected["orientation"]),
                        velocity: array(&expected["velocity"]),
                        angular_velocity: array(&expected["angular_velocity"]),
                    },
                );
                if i != 0 {
                    let state = backend.joint_state(joints[i - 1]).unwrap();
                    assert!((state.position - expected["q"].as_f64().unwrap()).abs() < 1e-12);
                    assert!((state.velocity - expected["dq"].as_f64().unwrap()).abs() < 1e-12);
                }
            }
        }
    }
}

#[test]
fn prepared_exact_old_leaf_extension_preserves_endpoints_ids_and_queued_inputs() {
    free_chain_support::assert_exact_stationary_leaf_extension();
}

#[test]
fn moving_chain_keeps_both_anchors_and_restores_all_body_and_joint_identities() {
    let (mut backend, bodies, first) = pair(true);
    let second = finish(&mut backend, bodies, true);
    let initial = backend.initial_state();
    let before = bodies.map(|id| backend.state(id).unwrap());
    assert!((backend.joint_state(first).unwrap().velocity - 2.0).abs() < 1e-12);
    assert!((backend.joint_state(second).unwrap().velocity - 3.0).abs() < 1e-12);
    for _ in 0..1000 {
        backend.step(Duration::from_millis(1)).unwrap();
        let states = bodies.map(|id| backend.state(id).unwrap());
        assert!(
            (point(states[0], [0.0, 0.0, -1.0]) - point(states[1], [0.0, 0.0, 1.0])).norm() < 1e-10
        );
        assert!(
            (point(states[1], [0.0, 0.0, -1.0]) - point(states[2], [0.0, 0.0, 1.0])).norm() < 1e-10
        );
    }
    for loaded in [false, true] {
        backend.add_joint_effort(second, 0.7).unwrap();
        backend
            .add_world_wrench(bodies[0], [0.2, 0.0, 0.0], [0.0, 0.4, 0.0])
            .unwrap();
        if loaded {
            let prepared = backend.prepare_initial(&initial).unwrap();
            backend.apply_initial(prepared).unwrap();
        } else {
            backend.reset().unwrap();
        }
        for (id, expected) in bodies.into_iter().zip(before) {
            near(backend.state(id).unwrap(), expected);
        }
        assert_eq!(backend.joint_child(first), Some(bodies[1]));
        assert_eq!(backend.joint_child(second), Some(bodies[2]));
        assert!((backend.joint_state(first).unwrap().velocity - 2.0).abs() < 1e-12);
        assert!((backend.joint_state(second).unwrap().velocity - 3.0).abs() < 1e-12);
    }
}

#[test]
fn branches_cycles_fourth_body_and_world_conversion_remain_atomic_refusals() {
    let (mut backend, bodies, first) = pair(false);
    let second = finish(&mut backend, bodies, false);
    let extra = body(&mut backend, [20.0, 0.0, 0.0], [0.0; 3], 0.0);
    let all = [bodies[0], bodies[1], bodies[2], extra];
    let before = all.map(|id| backend.state(id));
    for (parent, child) in [
        (bodies[0], extra),
        (bodies[1], extra),
        (bodies[2], extra),
        (bodies[2], bodies[0]),
        (extra, bodies[0]),
        (extra, bodies[1]),
        (bodies[2], bodies[2]),
    ] {
        assert!(matches!(
            backend.add_revolute(parent, child, [0.0; 3], [0.0, 1.0, 0.0]),
            Err(PhysicsError::Unsupported { .. })
        ));
        assert_eq!(all.map(|id| backend.state(id)), before);
    }
    assert!(matches!(
        backend.add_world_revolute(bodies[0], [0.0; 3], [0.0, 1.0, 0.0]),
        Err(PhysicsError::Unsupported { .. })
    ));
    assert_eq!(all.map(|id| backend.state(id)), before);
    assert!(backend.remove_joint(second));
    let next = finish(&mut backend, bodies, false);
    assert_eq!(next.raw(), second.raw() + 1);
    assert_eq!(backend.joint_child(first), Some(bodies[1]));
}

#[test]
fn middle_and_root_retirement_preserve_survivors_and_prepared_reconstruction() {
    for removed in 0..3 {
        let (mut backend, bodies, first) = pair(true);
        let second = finish(&mut backend, bodies, true);
        let initial = backend.initial_state();
        let initial_states = bodies.map(|id| backend.state(id).unwrap());
        backend.step(Duration::from_millis(1)).unwrap();
        // Populate native dependent Jacobians before the subtree edit.
        backend
            .apply_impulse(bodies[2], [0.0; 3], [0.0; 3])
            .unwrap();
        let before = bodies.map(|id| backend.state(id).unwrap());
        assert!(backend.remove(bodies[removed]));
        assert!(!backend.remove(bodies[removed]));
        assert_eq!(backend.state(bodies[removed]), None);
        for i in 0..3 {
            if i != removed {
                near(backend.state(bodies[i]).unwrap(), before[i]);
            }
        }
        assert_eq!(backend.joint_state(first).is_some(), removed == 2);
        assert_eq!(backend.joint_state(second).is_some(), removed == 0);
        let survivor = bodies[if removed == 0 { 1 } else { 0 }];
        backend
            .apply_impulse(survivor, [0.1, 0.0, 0.0], [0.0; 3])
            .unwrap();
        backend.step(Duration::from_millis(1)).unwrap();
        backend.reset().unwrap();
        assert_eq!(backend.state(bodies[removed]), None);
        let prepared = backend.prepare_initial(&initial).unwrap();
        backend.apply_initial(prepared).unwrap();
        for (id, expected) in bodies.into_iter().zip(initial_states) {
            near(backend.state(id).unwrap(), expected);
        }
        assert_eq!(backend.joint_child(first), Some(bodies[1]));
        assert_eq!(backend.joint_child(second), Some(bodies[2]));
    }
}

#[test]
fn unrelated_three_body_components_keep_independent_rates_and_surviving_ids() {
    let (mut backend, first_bodies, first) = pair(false);
    let first_leaf = finish(&mut backend, first_bodies, false);
    let second_bodies = [
        body(&mut backend, [100.0, 0.0, 0.0], [0.0; 3], 0.0),
        body(&mut backend, [100.0, 0.0, -2.0], [0.0; 3], 0.0),
        body(&mut backend, [110.0, 0.0, 0.0], [0.0; 3], 0.0),
    ];
    let second = backend
        .add_revolute(
            second_bodies[0],
            second_bodies[1],
            [100.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
        )
        .unwrap();
    let second_leaf = backend
        .add_revolute(
            second_bodies[1],
            second_bodies[2],
            [100.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        )
        .unwrap();
    let initial = backend.initial_state();
    let before_first = first_bodies.map(|id| backend.state(id));
    let before_second = second_bodies.map(|id| backend.state(id));
    let second_rates = [
        backend.joint_state(second),
        backend.joint_state(second_leaf),
    ];
    backend
        .apply_impulse(first_bodies[2], [0.1, 0.0, 0.0], [0.0; 3])
        .unwrap();
    assert_ne!(first_bodies.map(|id| backend.state(id)), before_first);
    assert_eq!(second_bodies.map(|id| backend.state(id)), before_second);
    assert_eq!(
        [
            backend.joint_state(second),
            backend.joint_state(second_leaf)
        ],
        second_rates
    );
    assert!(backend.remove(first_bodies[1]));
    assert_eq!(backend.joint_state(first), None);
    assert_eq!(backend.joint_state(first_leaf), None);
    assert_eq!(second_bodies.map(|id| backend.state(id)), before_second);
    backend.step(Duration::from_millis(1)).unwrap();
    assert_eq!(second_bodies.map(|id| backend.state(id)), before_second);
    assert_eq!(
        [
            backend.joint_state(second),
            backend.joint_state(second_leaf)
        ],
        second_rates
    );
    let prepared = backend.prepare_initial(&initial).unwrap();
    backend.apply_initial(prepared).unwrap();
    assert_eq!(first_bodies.map(|id| backend.state(id)), before_first);
    assert_eq!(second_bodies.map(|id| backend.state(id)), before_second);
    for (joint, child) in [
        (first, first_bodies[1]),
        (first_leaf, first_bodies[2]),
        (second, second_bodies[1]),
        (second_leaf, second_bodies[2]),
    ] {
        assert_eq!(backend.joint_child(joint), Some(child));
    }
}

#[test]
fn static_third_attachment_preserves_warm_pair_and_pending_commands() {
    let test = "static_third_attachment_preserves_warm_pair_and_pending_commands";
    if !in_child(test) {
        run_child(test, None);
        return;
    }
    three_body_static_positive::assert_attachment_positive();
}

#[test]
fn approaching_three_body_static_contact_matches_native_for_1000_steps() {
    let test = "approaching_three_body_static_contact_matches_native_for_1000_steps";
    if !in_child(test) {
        run_child(test, None);
        return;
    }
    three_body_static_positive::assert_approaching_positive();
}

#[test]
fn predictive_multibody_limits_keep_all_states_and_pending_commands() {
    let test = "predictive_multibody_limits_keep_all_states_and_pending_commands";
    if !in_child(test) {
        run_child(test, None);
        return;
    }
    let (mut candidate, bodies, first) = pair(false);
    let (mut clean, controls, control_first) = pair(false);
    let second = finish(&mut candidate, bodies, false);
    let control_second = finish(&mut clean, controls, false);
    candidate
        .set_world_velocity(bodies[2], [0.0, 0.0, -2e21], [0.0, 2e20, 0.0])
        .unwrap();
    for (world, ids, joint) in [
        (&mut candidate, bodies, first),
        (&mut clean, controls, control_first),
    ] {
        world
            .add_world_wrench(ids[0], [0.2, 0.0, 0.0], [0.0, 0.1, 0.0])
            .unwrap();
        world.add_joint_effort(joint, 0.7).unwrap();
    }
    let before = bodies.map(|id| candidate.state(id));
    let rates = [candidate.joint_state(first), candidate.joint_state(second)];
    for _ in 0..2 {
        let error = candidate.step(Duration::from_millis(1)).unwrap_err();
        assert!(error.to_string().contains("active joint limit"), "{error}");
        assert_eq!(bodies.map(|id| candidate.state(id)), before);
        assert_eq!(
            [candidate.joint_state(first), candidate.joint_state(second)],
            rates
        );
    }
    candidate
        .set_world_velocity(bodies[2], [0.0; 3], [0.0; 3])
        .unwrap();
    for _ in 0..3 {
        candidate.step(Duration::from_millis(1)).unwrap();
        clean.step(Duration::from_millis(1)).unwrap();
        for (a, b) in bodies.into_iter().zip(controls) {
            near(candidate.state(a).unwrap(), clean.state(b).unwrap());
        }
        assert_eq!(
            candidate.joint_state(second),
            clean.joint_state(control_second)
        );
    }
}

#[test]
fn rejected_origin_and_leaf_rate_edits_restore_all_eight_rates_and_future_motion() {
    let test = "rejected_origin_and_leaf_rate_edits_restore_all_eight_rates_and_future_motion";
    if !in_child(test) {
        run_child(test, None);
        return;
    }
    let make = || {
        let mut backend = DartBackend::new([0.0; 3]).unwrap();
        let bodies = [0.0, -2.0, -4.0].map(|z| {
            backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.1 },
                        position: [0.0, 0.0, z],
                        velocity: [0.1, 0.2, 0.3],
                        mass: Some(1.0),
                    },
                    RotationSpec {
                        angular_velocity: [0.0, 0.2, 0.0],
                        ..Default::default()
                    },
                    Some(Inertia {
                        center: [0.0, 0.0, 0.1],
                        orientation: [0.0, 0.0, 0.0, 1.0],
                        tensor: [0.004, 0.004, 0.004, 0.0, 0.0, 0.0],
                    }),
                )
                .unwrap()
        });
        let first = backend
            .add_revolute(bodies[0], bodies[1], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0])
            .unwrap();
        let second = backend
            .add_revolute(bodies[1], bodies[2], [0.0, 0.0, -3.9], [1.0, 1.0, 1.0])
            .unwrap();
        (backend, bodies, [first, second])
    };
    let (mut candidate, bodies, joints) = make();
    let (mut clean, controls, control_joints) = make();
    candidate
        .apply_impulse(bodies[2], [0.0; 3], [0.0; 3])
        .unwrap();
    clean
        .apply_impulse(controls[2], [0.0; 3], [0.0; 3])
        .unwrap();
    let before = bodies.map(|id| candidate.state(id));
    let rates = joints.map(|id| candidate.joint_state(id));
    // Both public COM vectors fit the bound; conversion at the root exceeds it.
    assert_eq!(
        candidate.set_world_velocity(bodies[0], [-9.5e59, 0.0, 0.0], [0.0, 9.5e59, 0.0]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(bodies.map(|id| candidate.state(id)), before);
    assert_eq!(joints.map(|id| candidate.joint_state(id)), rates);
    // The leaf hinge sits at its COM. Projecting three bounded angular entries
    // along [1,1,1] makes the final (eighth) generalized rate exceed 1e60.
    assert_eq!(
        candidate.set_world_velocity(bodies[2], [0.0; 3], [9.5e59; 3]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(bodies.map(|id| candidate.state(id)), before);
    assert_eq!(joints.map(|id| candidate.joint_state(id)), rates);
    for _ in 0..3 {
        for (world, ids, joint) in [
            (&mut candidate, bodies, joints[1]),
            (&mut clean, controls, control_joints[1]),
        ] {
            world
                .add_world_wrench(ids[0], [0.1, 0.0, 0.0], [0.0, 0.2, 0.0])
                .unwrap();
            world.add_joint_effort(joint, 0.3).unwrap();
            world.step(Duration::from_millis(1)).unwrap();
        }
        for (a, b) in bodies.into_iter().zip(controls) {
            near(candidate.state(a).unwrap(), clean.state(b).unwrap());
        }
        for (a, b) in joints.into_iter().zip(control_joints) {
            assert_eq!(candidate.joint_state(a), clean.joint_state(b));
        }
    }
}

fn numeric_case(case: usize) -> &'static str {
    let radius = [1e-10_f64, 0.1, 10.0][case % 3];
    let masses = [
        [1.0, 1.0, 1.0],
        [2e-4, 1.0, 1e6],
        [1e6, 2e-4, 1.0],
        [1.0, 1e60, 1.0],
    ][case / 3 % 4];
    let tilted = case / 12 % 2 == 1;
    let anisotropic = case / 24 % 2 == 1;
    let orientation = UnitQuaternion::from_euler_angles(0.2, -0.3, 0.4);
    let q = orientation.quaternion();
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let mut bodies = Vec::new();
    for (index, mass) in masses.into_iter().enumerate() {
        let before: Vec<_> = bodies.iter().map(|id| backend.state(*id)).collect();
        let moment = (10.0 * mass * radius * radius).max(1e-60);
        let shape = match case % 4 {
            0 => Shape::Sphere { radius },
            1 => Shape::Box {
                half_extents: [radius; 3],
            },
            2 => Shape::Cylinder {
                radius,
                half_height: radius,
            },
            _ => Shape::Capsule {
                radius,
                half_length: radius,
            },
        };
        let result = backend.spawn_link(
            BodySpec {
                shape,
                position: [index as f64 * 4.0 * radius, 0.0, 0.0],
                velocity: [0.0; 3],
                mass: Some(mass),
            },
            RotationSpec {
                orientation: [q.i, q.j, q.k, q.w],
                ..Default::default()
            },
            Some(Inertia {
                center: [0.03 * radius, -0.02 * radius, 0.01 * radius],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [
                    moment,
                    moment * if anisotropic { 1e6 } else { 1.0 },
                    moment * if anisotropic { 1e6 } else { 1.0 },
                    0.0,
                    0.0,
                    0.0,
                ],
            }),
        );
        let Ok(body) = result else {
            assert_eq!(
                bodies
                    .iter()
                    .map(|id| backend.state(*id))
                    .collect::<Vec<_>>(),
                before
            );
            return "spawn refused";
        };
        bodies.push(body);
    }
    let mut joints = Vec::new();
    for child in 1..3 {
        let before: Vec<_> = bodies.iter().map(|id| backend.state(*id)).collect();
        let axis = if tilted && child == 2 {
            [0.3, 0.8, 0.5]
        } else {
            [0.0, 0.0, 1.0]
        };
        let result = backend.add_revolute(
            bodies[child - 1],
            bodies[child],
            [(child as f64 * 4.0 - 2.0) * radius, 0.0, 0.0],
            axis,
        );
        let Ok(joint) = result else {
            assert_eq!(
                bodies
                    .iter()
                    .map(|id| backend.state(*id))
                    .collect::<Vec<_>>(),
                before
            );
            return "attachment refused";
        };
        joints.push(joint);
    }
    let before: Vec<_> = bodies.iter().map(|id| backend.state(*id)).collect();
    if backend
        .apply_impulse(bodies[2], [radius * 1e-4; 3], [radius * radius * 1e-4; 3])
        .is_err()
    {
        assert_eq!(
            bodies
                .iter()
                .map(|id| backend.state(*id))
                .collect::<Vec<_>>(),
            before
        );
    }
    let mut advanced = 0;
    for round in 0..4 {
        backend
            .add_joint_effort(joints[1], radius * radius * masses[2].min(1.0) * 1e-4)
            .unwrap();
        let before: Vec<_> = bodies.iter().map(|id| backend.state(*id)).collect();
        let rates: Vec<_> = joints.iter().map(|id| backend.joint_state(*id)).collect();
        let dt = if round == 3 {
            Duration::MAX
        } else {
            Duration::from_millis(1)
        };
        if backend.step(dt).is_err() {
            assert_eq!(
                bodies
                    .iter()
                    .map(|id| backend.state(*id))
                    .collect::<Vec<_>>(),
                before
            );
            assert_eq!(
                joints
                    .iter()
                    .map(|id| backend.joint_state(*id))
                    .collect::<Vec<_>>(),
                rates
            );
            break;
        }
        advanced += 1;
        for id in &bodies {
            let state = backend.state(*id).unwrap();
            assert!(
                state
                    .position
                    .into_iter()
                    .chain(state.orientation)
                    .chain(state.velocity)
                    .chain(state.angular_velocity)
                    .all(f64::is_finite)
            );
        }
    }
    if advanced == 0 {
        "step refused"
    } else {
        "advanced"
    }
}

#[test]
fn isolated_numeric_boundary_inputs_do_not_abort_native_dart() {
    let test = "isolated_numeric_boundary_inputs_do_not_abort_native_dart";
    if in_child(test) {
        let case: usize = std::env::var(CASE).unwrap().parse().unwrap();
        assert!(case < 48);
        println!("outcome: {}", numeric_case(case));
        return;
    }
    let mut advanced = 0;
    let mut refused = 0;
    for case in 0..48 {
        let stdout = run_child(test, Some(case));
        if stdout.contains("outcome: advanced") {
            advanced += 1;
        } else {
            assert!(
                stdout.contains("outcome: spawn refused")
                    || stdout.contains("outcome: attachment refused")
                    || stdout.contains("outcome: step refused")
            );
            refused += 1;
        }
    }
    assert!(advanced > 0 && refused > 0);
    assert_eq!(advanced + refused, 48);
    println!("actual child outcomes: {advanced} advanced, {refused} explicitly refused");
}
