// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded static-contact children; every unexpected error or abort is a failure.
use gz_physics::{BodySpec, DynamicBackend, Inertia, PhysicsError, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use nalgebra::UnitQuaternion;
use std::{
    process::{Command, Stdio},
    time::{Duration, Instant},
};

const TEST: &str = "static_three_body_numeric_contacts_remain_bounded_and_atomic";
const CASE: &str = "GZ_DART_STATIC_NUMERIC_CASE";
const DT: Duration = Duration::from_millis(1);

fn numeric_case(case: usize) -> &'static str {
    assert!(case < 96);
    let radius = [1e-10_f64, 0.1, 10.0][case / 4 % 3];
    let masses = [
        [1.0, 1.0, 1.0],
        [2e-4, 1.0, 1e6],
        [1e6, 2e-4, 1.0],
        [1.0, 1e60, 1.0],
    ][case / 12 % 4];
    let tilted = case >= 48;
    let anisotropic = (case / 24) % 2 == 1;
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
    let rotation = if tilted {
        UnitQuaternion::from_euler_angles(0.2, -0.3, 0.4)
    } else {
        UnitQuaternion::identity()
    };
    let q = rotation.quaternion();
    let mut backend = DartBackend::new([0.0, 0.0, -9.8]).unwrap();
    let floor = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [100.0 * radius, 100.0 * radius, radius],
                },
                position: [0.0, 0.0, -radius],
                velocity: [0.0; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    let mut ids = Vec::new();
    for (index, mass) in masses.into_iter().enumerate() {
        let before: Vec<_> = ids.iter().map(|id| backend.state(*id)).collect();
        let moment = (10.0 * mass * radius * radius).max(1e-60);
        // Every chosen shape has vertical support at least radius, even tilted.
        // Thus an accepted setup intersects the real static box by at least .01r.
        let result = backend.spawn_link(
            BodySpec {
                shape,
                position: [index as f64 * 4.0 * radius, 0.0, 0.99 * radius],
                velocity: [0.0; 3],
                mass: Some(mass),
            },
            RotationSpec {
                orientation: [q.i, q.j, q.k, q.w],
                ..Default::default()
            },
            Some(Inertia {
                center: if tilted {
                    [0.03 * radius, -0.02 * radius, 0.01 * radius]
                } else {
                    [0.0; 3]
                },
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
        match result {
            Ok(id) => ids.push(id),
            Err(PhysicsError::InvalidValue) => {
                assert_eq!(
                    ids.iter().map(|id| backend.state(*id)).collect::<Vec<_>>(),
                    before
                );
                return "spawn refused: InvalidValue";
            }
            Err(error) => panic!("case {case} unexpected spawn error: {error:?}"),
        }
    }
    let mut joints = Vec::new();
    for child in 1..3 {
        let before: Vec<_> = ids.iter().map(|id| backend.state(*id)).collect();
        let rates: Vec<_> = joints.iter().map(|id| backend.joint_state(*id)).collect();
        let axis = if tilted && child == 2 {
            [0.3, 0.8, 0.5]
        } else {
            [0.0, 0.0, 1.0]
        };
        let result = backend.add_revolute(
            ids[child - 1],
            ids[child],
            [(child as f64 * 4.0 - 2.0) * radius, 0.0, 0.99 * radius],
            axis,
        );
        match result {
            Ok(id) => joints.push(id),
            Err(PhysicsError::InvalidJoint) => {
                assert_eq!(
                    ids.iter().map(|id| backend.state(*id)).collect::<Vec<_>>(),
                    before
                );
                assert_eq!(
                    joints
                        .iter()
                        .map(|id| backend.joint_state(*id))
                        .collect::<Vec<_>>(),
                    rates
                );
                assert_eq!(
                    backend.add_revolute(
                        ids[child - 1],
                        ids[child],
                        [(child as f64 * 4.0 - 2.0) * radius, 0.0, 0.99 * radius],
                        axis
                    ),
                    Err(PhysicsError::InvalidJoint)
                );
                assert_eq!(
                    ids.iter().map(|id| backend.state(*id)).collect::<Vec<_>>(),
                    before
                );
                return "attachment refused: InvalidJoint";
            }
            Err(error) => panic!("case {case} unexpected attachment error: {error:?}"),
        }
    }
    for _ in 0..40 {
        backend
            .add_world_wrench(
                ids[0],
                [radius * radius * masses[0].min(1.0) * 0.001, 0.0, 0.0],
                [0.0; 3],
            )
            .unwrap();
        backend
            .add_joint_effort(joints[1], radius * radius * masses[2].min(1.0) * 0.001)
            .unwrap();
        let before: Vec<_> = ids.iter().map(|id| backend.state(*id)).collect();
        let rates: Vec<_> = joints.iter().map(|id| backend.joint_state(*id)).collect();
        if let Err(error) = backend.step(DT) {
            let active_limit = error.to_string().contains("active joint limit");
            assert!(
                matches!(&error, PhysicsError::Native { .. })
                    && (error.to_string().contains("a contact exceeded") || active_limit),
                "case {case} unexpected step error: {error:?}"
            );
            assert_eq!(
                ids.iter().map(|id| backend.state(*id)).collect::<Vec<_>>(),
                before
            );
            assert_eq!(
                joints
                    .iter()
                    .map(|id| backend.joint_state(*id))
                    .collect::<Vec<_>>(),
                rates
            );
            assert!(backend.step(DT).is_err());
            assert_eq!(
                ids.iter().map(|id| backend.state(*id)).collect::<Vec<_>>(),
                before
            );
            assert_eq!(
                joints
                    .iter()
                    .map(|id| backend.joint_state(*id))
                    .collect::<Vec<_>>(),
                rates
            );
            assert!(backend.remove(floor));
            if active_limit {
                for _ in 0..2 {
                    let error = backend.step(DT).unwrap_err();
                    assert!(
                        matches!(&error, PhysicsError::Native { .. })
                            && error.to_string().contains("active joint limit"),
                        "case {case} unexpected post-removal error: {error:?}"
                    );
                    assert_eq!(
                        ids.iter().map(|id| backend.state(*id)).collect::<Vec<_>>(),
                        before
                    );
                    assert_eq!(
                        joints
                            .iter()
                            .map(|id| backend.joint_state(*id))
                            .collect::<Vec<_>>(),
                        rates
                    );
                }
                // Clear the free-root rates before projecting each child's twist.
                for id in &ids {
                    backend.set_world_velocity(*id, [0.0; 3], [0.0; 3]).unwrap();
                }
                for joint in &joints {
                    assert_eq!(backend.joint_state(*joint).unwrap().velocity, 0.0);
                }
            }
            backend.step(DT).unwrap();
            return if active_limit {
                "active-limit refused atomically, explicit velocity edits recovered"
            } else {
                "contact refused atomically, removal recovered"
            };
        }
        for id in &ids {
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
    "advanced: 40 steps from real initial static contact"
}

#[test]
fn static_three_body_numeric_contacts_remain_bounded_and_atomic() {
    match std::env::var(CASE) {
        Ok(case) => {
            let case: usize = case.parse().unwrap();
            println!("STATIC_OUTCOME {}", numeric_case(case));
            return;
        }
        Err(std::env::VarError::NotPresent) => {}
        Err(error) => panic!("{error}"),
    }
    let output_directory = std::env::var_os("GZ_DART_STATIC_NUMERIC_OUTPUT")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| {
            std::env::temp_dir().join(format!("gz-dart-static-numeric-{}", std::process::id()))
        });
    std::fs::create_dir_all(&output_directory).unwrap();
    let mut observations = Vec::new();
    for case in 0..96 {
        let mut process = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", TEST, "--nocapture"])
            .env(CASE, case.to_string())
            .env_remove("LD_PRELOAD")
            .env_remove("LD_AUDIT")
            .env_remove("LD_LIBRARY_PATH")
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(30);
        let mut timeout = false;
        while process.try_wait().unwrap().is_none() {
            if Instant::now() >= deadline {
                timeout = true;
                process.kill().unwrap();
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        let output = process.wait_with_output().unwrap();
        std::fs::write(
            output_directory.join(format!("case-{case}.stdout")),
            &output.stdout,
        )
        .unwrap();
        std::fs::write(
            output_directory.join(format!("case-{case}.stderr")),
            &output.stderr,
        )
        .unwrap();
        let text = String::from_utf8_lossy(&output.stdout);
        let outcome = text
            .find("STATIC_OUTCOME ")
            .map(|start| text[start + 15..].lines().next().unwrap().to_owned());
        let ran_one = text.contains(&format!("test {TEST} ..."))
            && text.contains("test result: ok. 1 passed; 0 failed; 0 ignored;");
        observations.push(serde_json::json!({"case":case,"status":output.status.to_string(),"timeout":timeout,"success":output.status.success()&&ran_one,"outcome":outcome}));
        std::fs::write(
            output_directory.join("result.json"),
            serde_json::to_vec_pretty(&observations).unwrap(),
        )
        .unwrap();
    }
    assert_eq!(observations.len(), 96);
    assert!(
        observations.iter().all(|row| row["success"] == true
            && row["timeout"] == false
            && row["outcome"].is_string()),
        "See every retained child at {}",
        output_directory.display()
    );
    assert!(
        observations
            .iter()
            .any(|row| row["outcome"].as_str().unwrap().starts_with("advanced:"))
    );
    println!("{observations:?}");
}
