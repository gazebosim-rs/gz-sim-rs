// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Prepared static positives and live negatives; old refusal tests stay intact.
use gz_physics::{
    BodyId, BodySpec, DynamicBackend, Inertia, JointId, PhysicsError, RotationSpec, Shape,
};
use std::{
    process::{Command, Stdio},
    time::{Duration, Instant},
};
#[path = "support/three_body_static_contacts.rs"]
mod support;
use gz_physics_dart::DartBackend;
use support::DT;
const CHILD: &str = "GZ_DART_STATIC_CONTACT_CHILD";

fn child(test: &str) -> bool {
    match std::env::var(CHILD) {
        Ok(name) => {
            assert_eq!(name, test);
            true
        }
        Err(std::env::VarError::NotPresent) => {
            let mut process = Command::new(std::env::current_exe().unwrap())
                .args(["--exact", test, "--nocapture"])
                .env(CHILD, test)
                .env_remove("LD_PRELOAD")
                .env_remove("LD_AUDIT")
                .env_remove("LD_LIBRARY_PATH")
                .stdin(Stdio::null())
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
                .unwrap();
            let end = Instant::now() + Duration::from_secs(30);
            let mut timeout = false;
            while process.try_wait().unwrap().is_none() {
                if Instant::now() >= end {
                    timeout = true;
                    process.kill().unwrap();
                    break;
                }
                std::thread::sleep(Duration::from_millis(5));
            }
            let output = process.wait_with_output().unwrap();
            let text = String::from_utf8_lossy(&output.stdout);
            assert!(
                !timeout && output.status.success(),
                "{test}: {:?}\n{}\n{}",
                output.status,
                text,
                String::from_utf8_lossy(&output.stderr)
            );
            assert!(
                text.contains(&format!("test {test} ... ok"))
                    && text.contains("test result: ok. 1 passed; 0 failed; 0 ignored;"),
                "{text}"
            );
            false
        }
        Err(error) => panic!("{error}"),
    }
}
#[test]
fn same_input_joined_static_contact_steps_match_native() {
    if !child("same_input_joined_static_contact_steps_match_native") {
        return;
    }
    let (mut b, ids, first) = support::pair();
    let _floor = support::obstacle(&mut b, false, false);
    support::warm_commands(&mut b, ids, first);
    let second = support::finish(&mut b, ids);
    let data = support::fixture();
    let rows = data["cases"]["attach-live"]["rows"].as_array().unwrap();
    support::native_row(&b, ids, [first, second], &rows[0]);
    let mut matched = 1;
    for tick in 1..=100 {
        b.step(DT).unwrap();
        if let Some(row) = rows
            .iter()
            .find(|row| row["stage"] == format!("native-contact-step-{tick}"))
        {
            support::native_row(&b, ids, [first, second], row);
            matched += 1;
        }
    }
    assert_eq!(matched, rows.len());
}
#[test]
fn static_sphere_and_four_corner_box_trajectories_match_native() {
    if !child("static_sphere_and_four_corner_box_trajectories_match_native") {
        return;
    }
    let data = support::fixture();
    for sphere in [true, false] {
        let (mut b, ids, joints, _floor) = footprint(sphere);
        let name = if sphere {
            "footprint-sphere"
        } else {
            "footprint-box"
        };
        let rows = data["cases"][name]["rows"].as_array().unwrap();
        let mut matched = 0;
        for tick in 0..=100 {
            if tick > 0 {
                b.step(DT).unwrap();
            }
            if let Some(row) = rows
                .iter()
                .find(|row| row["stage"] == format!("footprint-contact-{tick}"))
            {
                support::native_row(&b, ids, joints, row);
                matched += 1;
            }
        }
        assert_eq!(matched, rows.len());
    }
}
#[test]
fn dynamic_obstacle_attachment_keeps_real_rollback_and_pending_commands() {
    if !child("dynamic_obstacle_attachment_keeps_real_rollback_and_pending_commands") {
        return;
    }
    let (mut candidate, ids, first) = support::pair();
    let (mut clean, controls, control_first) = support::pair();
    let floor = support::obstacle(&mut candidate, false, true);
    let clean_floor = support::obstacle(&mut clean, false, true);
    support::warm_commands(&mut candidate, ids, first);
    support::warm_commands(&mut clean, controls, control_first);
    let before = ids.map(|id| candidate.state(id));
    let rate = candidate.joint_state(first);
    for _ in 0..2 {
        assert_eq!(
            candidate.add_revolute(ids[1], ids[2], [0.; 3], [0., 1., 0.]),
            Err(PhysicsError::InvalidJoint)
        );
        assert_eq!(ids.map(|id| candidate.state(id)), before);
        assert_eq!(candidate.joint_state(first), rate);
    }
    assert!(candidate.remove(floor));
    assert!(clean.remove(clean_floor));
    let second = support::finish(&mut candidate, ids);
    let control_second = support::finish(&mut clean, controls);
    assert_eq!(second.raw(), first.raw() + 1);
    for _ in 0..3 {
        candidate.step(DT).unwrap();
        clean.step(DT).unwrap();
        support::compare(
            &candidate,
            ids,
            [first, second],
            &clean,
            controls,
            [control_first, control_second],
        );
    }
}
#[test]
fn approaching_dynamic_obstacle_keeps_repeat_refusal_and_pending_recovery() {
    if !child("approaching_dynamic_obstacle_keeps_repeat_refusal_and_pending_recovery") {
        return;
    }
    let (mut candidate, ids, joints, floor) = support::approaching(true);
    let (mut clean, controls, control_joints, clean_floor) = support::approaching(true);
    let mut refused = false;
    for _ in 0..1000 {
        support::approach_commands(&mut candidate, ids, joints[1]);
        support::approach_commands(&mut clean, controls, control_joints[1]);
        let before = ids.map(|id| candidate.state(id));
        let rates = joints.map(|id| candidate.joint_state(id));
        if let Err(error) = candidate.step(DT) {
            assert!(error.to_string().contains("a contact exceeded"), "{error}");
            assert_eq!(ids.map(|id| candidate.state(id)), before);
            assert_eq!(joints.map(|id| candidate.joint_state(id)), rates);
            assert!(candidate.step(DT).is_err());
            assert_eq!(ids.map(|id| candidate.state(id)), before);
            assert_eq!(joints.map(|id| candidate.joint_state(id)), rates);
            assert!(candidate.remove(floor));
            assert!(clean.remove(clean_floor));
            for _ in 0..3 {
                candidate.step(DT).unwrap();
                clean.step(DT).unwrap();
                support::compare(&candidate, ids, joints, &clean, controls, control_joints);
            }
            refused = true;
            break;
        }
        clean.step(DT).unwrap();
        support::compare(&candidate, ids, joints, &clean, controls, control_joints);
    }
    assert!(
        refused,
        "real dynamic contact never reached the refusal guard"
    );
}
#[test]
fn native_above_condition_static_contact_keeps_atomic_refusal_and_pending_recovery() {
    if !child("native_above_condition_static_contact_keeps_atomic_refusal_and_pending_recovery") {
        return;
    }
    let data = support::fixture();
    assert!(
        data["cases"]["above-condition"]["conditions"]
            .as_array()
            .unwrap()
            .iter()
            .all(|row| row["condition"].as_f64().unwrap() > 1e8)
    );
    let (mut candidate, ids, joints, floor) = conditioned_approach();
    let (mut clean, controls, control_joints, clean_floor) = conditioned_approach();
    let mut refused = false;
    for _ in 0..100 {
        for (b, bodies, j) in [
            (&mut candidate, ids, joints[1]),
            (&mut clean, controls, control_joints[1]),
        ] {
            b.add_world_wrench(bodies[0], [1e-5, 0., 0.], [0.; 3])
                .unwrap();
            b.add_joint_effort(j, 1e-5).unwrap();
        }
        let before = ids.map(|id| candidate.state(id));
        let rates = joints.map(|id| candidate.joint_state(id));
        if let Err(error) = candidate.step(DT) {
            assert!(error.to_string().contains("a contact exceeded"), "{error}");
            assert_eq!(ids.map(|id| candidate.state(id)), before);
            assert_eq!(joints.map(|id| candidate.joint_state(id)), rates);
            assert!(candidate.step(DT).is_err());
            assert_eq!(ids.map(|id| candidate.state(id)), before);
            assert_eq!(joints.map(|id| candidate.joint_state(id)), rates);
            assert!(candidate.remove(floor));
            assert!(clean.remove(clean_floor));
            for _ in 0..3 {
                candidate.step(DT).unwrap();
                clean.step(DT).unwrap();
                support::compare(&candidate, ids, joints, &clean, controls, control_joints);
            }
            refused = true;
            break;
        }
        clean.step(DT).unwrap();
        support::compare(&candidate, ids, joints, &clean, controls, control_joints);
    }
    assert!(refused, "above-condition contact was not refused");
}
#[test]
fn actual_case_zero_spawn_refusal_preserves_live_three_body_state_and_pending_commands() {
    if !child("actual_case_zero_spawn_refusal_preserves_live_three_body_state_and_pending_commands")
    {
        return;
    }
    let (mut candidate, ids, first) = support::pair();
    let second = support::finish(&mut candidate, ids);
    let (mut clean, controls, control_first) = support::pair();
    let control_second = support::finish(&mut clean, controls);
    support::warm_commands(&mut candidate, ids, first);
    support::warm_commands(&mut clean, controls, control_first);
    let before = ids.map(|id| candidate.state(id));
    assert!(before.iter().all(Option::is_some));
    let rates = [first, second].map(|id| candidate.joint_state(id));
    let data = support::fixture();
    let bad = &data["rejected_spawn"];
    let v3 = |key: &str| std::array::from_fn(|i| bad[key][i].as_f64().unwrap());
    let v4 = |key: &str| std::array::from_fn(|i| bad[key][i].as_f64().unwrap());
    assert_eq!(
        candidate.spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: v3("half_extents")
                },
                position: v3("position"),
                velocity: v3("velocity"),
                mass: Some(bad["mass"].as_f64().unwrap())
            },
            RotationSpec {
                orientation: v4("orientation"),
                angular_velocity: v3("angular_velocity")
            },
            Some(Inertia {
                center: v3("center"),
                orientation: v4("inertia_orientation"),
                tensor: std::array::from_fn(|i| bad["tensor"][i].as_f64().unwrap())
            })
        ),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(ids.map(|id| candidate.state(id)), before);
    assert_eq!([first, second].map(|id| candidate.joint_state(id)), rates);
    for _ in 0..3 {
        candidate.step(DT).unwrap();
        clean.step(DT).unwrap();
        support::compare(
            &candidate,
            ids,
            [first, second],
            &clean,
            controls,
            [control_first, control_second],
        );
    }
}

fn footprint(sphere: bool) -> (DartBackend, [BodyId; 3], [JointId; 2], BodyId) {
    let mut backend = DartBackend::new([0., 0., -9.8]).unwrap();
    let mut ids = Vec::new();
    for i in 0..3 {
        ids.push(
            backend
                .spawn_link(
                    BodySpec {
                        shape: if sphere {
                            Shape::Sphere { radius: 0.1 }
                        } else {
                            Shape::Box {
                                half_extents: [0.25, 0.25, 0.005],
                            }
                        },
                        position: [0., 0., if sphere { 0.095 } else { 0.0045 } + i as f64 * 0.5],
                        velocity: [0.; 3],
                        mass: Some(1.),
                    },
                    RotationSpec::default(),
                    Some(Inertia {
                        center: [0.; 3],
                        orientation: [0., 0., 0., 1.],
                        tensor: [0.004, 0.004, 0.004, 0., 0., 0.],
                    }),
                )
                .unwrap(),
        );
    }
    let ids: [BodyId; 3] = ids.try_into().unwrap();
    let first = backend
        .add_revolute(ids[0], ids[1], [0., 0., 0.25], [0., 1., 0.])
        .unwrap();
    let second = backend
        .add_revolute(ids[1], ids[2], [0., 0., 0.75], [0., 1., 0.])
        .unwrap();
    let floor = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [20., 20., 0.1],
                },
                position: [0., 0., -0.1],
                velocity: [0.; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    (backend, ids, [first, second], floor)
}
fn conditioned_approach() -> (DartBackend, [BodyId; 3], [JointId; 2], BodyId) {
    let mut backend = DartBackend::new([0.; 3]).unwrap();
    let mut ids = Vec::new();
    for i in 0..3 {
        let moment = 1e-4 * if i == 2 { 74282549781.30127 } else { 1. };
        ids.push(
            backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Box {
                            half_extents: if i == 0 {
                                [0.25, 0.25, 0.005]
                            } else {
                                [0.05; 3]
                            },
                        },
                        position: [0., 0., 0.0051 + 0.5 * i as f64],
                        velocity: [0., 0., -0.1],
                        mass: Some(if i == 0 { 120. } else { 0.6 }),
                    },
                    RotationSpec::default(),
                    Some(Inertia {
                        center: [0.; 3],
                        orientation: [0., 0., 0., 1.],
                        tensor: [moment, moment, moment, 0., 0., 0.],
                    }),
                )
                .unwrap(),
        );
    }
    let ids: [BodyId; 3] = ids.try_into().unwrap();
    let first = backend
        .add_revolute(ids[0], ids[1], [0., 0., 0.5051], [0., 1., 0.])
        .unwrap();
    let second = backend
        .add_revolute(ids[1], ids[2], [0., 0., 1.0051], [1., 0., 0.])
        .unwrap();
    let floor = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [50., 50., 0.05],
                },
                position: [0., 0., -0.05],
                velocity: [0.; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    (backend, ids, [first, second], floor)
}

#[test]
fn contact_mass_rebuilds_after_prepared_reset_and_pending_edits_match_native() {
    if !child("contact_mass_rebuilds_after_prepared_reset_and_pending_edits_match_native") {
        return;
    }
    let data = support::fixture();
    for sphere in [true, false] {
        let (mut backend, ids, joints, _floor) = footprint(sphere);
        let initial = backend.initial_state();
        let name = if sphere {
            "footprint-sphere"
        } else {
            "footprint-box"
        };
        let rows = data["cases"][name]["rows"].as_array().unwrap();
        for _epoch in 0..3 {
            let mut matched = 0;
            for tick in 0..=100 {
                if tick > 0 {
                    backend.step(DT).unwrap();
                }
                if let Some(row) = rows
                    .iter()
                    .find(|row| row["stage"] == format!("footprint-contact-{tick}"))
                {
                    support::native_row(&backend, ids, joints, row);
                    matched += 1;
                }
            }
            assert_eq!(matched, rows.len());
            backend
                .set_world_velocity(ids[0], [0.1, 0., 0.], [0., 0.2, 0.])
                .unwrap();
            backend
                .add_world_wrench(ids[0], [0.2, 0., 0.], [0., 0.1, 0.])
                .unwrap();
            backend.add_joint_effort(joints[0], 0.7).unwrap();
            let reset = backend.prepare_initial(&initial).unwrap();
            backend.apply_initial(reset).unwrap();
            assert!(ids.iter().all(|id| backend.state(*id).is_some()));
            assert!(joints.iter().all(|id| backend.joint_state(*id).is_some()));
        }
        // Rebuilding a partially removed articulation must not retain a factor
        // belonging to the former three-body native skeleton.
        assert!(backend.remove(ids[2]));
        backend.reset().unwrap();
        assert!(backend.state(ids[2]).is_none());
        assert!(backend.joint_state(joints[1]).is_none());
        backend.step(DT).unwrap();
        for id in [ids[0], ids[1]] {
            let state = backend.state(id).unwrap();
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
        assert!(backend.remove(ids[0]));
        assert!(backend.remove(ids[1]));
        backend.step(DT).unwrap();
    }
}

// Separate physical owners with different masses, footprints and world anchors.
// Comparing each to its isolated world detects accidental factor sharing between
// skeletons without treating cached/uncached helper equality as an oracle.
fn contact_owner(
    backend: &mut DartBackend,
    x: f64,
    mass: f64,
    sphere: bool,
) -> ([BodyId; 3], [JointId; 2]) {
    let ids = std::array::from_fn(|i| {
        backend
            .spawn_link(
                BodySpec {
                    shape: if sphere {
                        Shape::Sphere { radius: 0.1 }
                    } else {
                        Shape::Box {
                            half_extents: [0.25, 0.25, 0.005],
                        }
                    },
                    position: [x, 0., if sphere { 0.095 } else { 0.0045 } + i as f64 * 0.5],
                    velocity: [0.; 3],
                    mass: Some(mass),
                },
                RotationSpec::default(),
                Some(Inertia {
                    center: [0.; 3],
                    orientation: [0., 0., 0., 1.],
                    tensor: [0.004 * mass, 0.004 * mass, 0.004 * mass, 0., 0., 0.],
                }),
            )
            .unwrap()
    });
    let first = backend
        .add_revolute(ids[0], ids[1], [x, 0., 0.25], [0., 1., 0.])
        .unwrap();
    let second = backend
        .add_revolute(ids[1], ids[2], [x, 0., 0.75], [0., 1., 0.])
        .unwrap();
    (ids, [first, second])
}

fn contact_floor(backend: &mut DartBackend) {
    backend
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [20., 20., 0.1],
                },
                position: [0., 0., -0.1],
                velocity: [0.; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
}

#[test]
fn contact_mass_is_owned_by_each_articulation_and_expires_after_removal() {
    if !child("contact_mass_is_owned_by_each_articulation_and_expires_after_removal") {
        return;
    }
    let mut combined = DartBackend::new([0., 0., -9.8]).unwrap();
    let mut first = DartBackend::new([0., 0., -9.8]).unwrap();
    let mut second = DartBackend::new([0., 0., -9.8]).unwrap();
    let (a, aj) = contact_owner(&mut combined, 0., 1., false);
    let (b, bj) = contact_owner(&mut combined, 8., 3., true);
    let (ac, acj) = contact_owner(&mut first, 0., 1., false);
    let (bc, bcj) = contact_owner(&mut second, 8., 3., true);
    for world in [&mut combined, &mut first, &mut second] {
        contact_floor(world);
    }
    for _ in 0..100 {
        combined.step(DT).unwrap();
        first.step(DT).unwrap();
        second.step(DT).unwrap();
        for (ids, joints, controls, control_joints, isolated) in
            [(a, aj, ac, acj, &first), (b, bj, bc, bcj, &second)]
        {
            for (id, control) in ids.into_iter().zip(controls) {
                support::near(
                    combined.state(id).unwrap(),
                    isolated.state(control).unwrap(),
                );
            }
            for (id, control) in joints.into_iter().zip(control_joints) {
                let actual = combined.joint_state(id).unwrap();
                let expected = isolated.joint_state(control).unwrap();
                assert!((actual.position - expected.position).abs() < 1e-12);
                assert!((actual.velocity - expected.velocity).abs() < 1e-12);
            }
        }
    }
    for id in a {
        assert!(combined.remove(id));
    }
    combined.reset().unwrap();
    second.reset().unwrap();
    for _ in 0..3 {
        combined.step(DT).unwrap();
        second.step(DT).unwrap();
        support::compare(&combined, b, bj, &second, bc, bcj);
    }
}

#[test]
fn contact_mass_condition_refusal_is_recomputed_across_reset_and_floor_removal() {
    if !child("contact_mass_condition_refusal_is_recomputed_across_reset_and_floor_removal") {
        return;
    }
    let data = support::fixture();
    assert!(
        data["cases"]["above-condition"]["conditions"]
            .as_array()
            .unwrap()
            .iter()
            .all(|row| row["condition"].as_f64().unwrap() > 1e8)
    );
    let (mut backend, ids, joints, floor) = conditioned_approach();
    let initial = backend.initial_state();
    for _epoch in 0..2 {
        let mut refused = false;
        for _ in 0..100 {
            backend
                .add_world_wrench(ids[0], [1e-5, 0., 0.], [0.; 3])
                .unwrap();
            backend.add_joint_effort(joints[1], 1e-5).unwrap();
            let states = ids.map(|id| backend.state(id));
            let rates = joints.map(|id| backend.joint_state(id));
            if let Err(error) = backend.step(DT) {
                assert!(error.to_string().contains("a contact exceeded"), "{error}");
                for _ in 0..2 {
                    assert_eq!(ids.map(|id| backend.state(id)), states);
                    assert_eq!(joints.map(|id| backend.joint_state(id)), rates);
                    let repeated = backend.step(DT).unwrap_err();
                    assert!(
                        repeated.to_string().contains("a contact exceeded"),
                        "{repeated}"
                    );
                }
                assert_eq!(ids.map(|id| backend.state(id)), states);
                assert_eq!(joints.map(|id| backend.joint_state(id)), rates);
                assert!(backend.remove(floor));
                backend.step(DT).unwrap();
                refused = true;
                break;
            }
        }
        assert!(
            refused,
            "the native above-condition contact was not refused"
        );
        let reset = backend.prepare_initial(&initial).unwrap();
        backend.apply_initial(reset).unwrap();
        assert!(backend.state(floor).is_some());
    }
}
