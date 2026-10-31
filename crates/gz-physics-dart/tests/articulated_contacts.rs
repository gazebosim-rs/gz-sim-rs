// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodySpec, DynamicBackend, Inertia, PlaneSpec, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use nalgebra::UnitQuaternion;
use std::{
    process::{Command, Stdio},
    time::{Duration, Instant},
};

#[test]
fn contact_boundary_child() {
    let Ok(case) = std::env::var("GZ_DART_CONTACT_CASE") else {
        return;
    };
    let case: usize = case.parse().unwrap();
    let radius = [1e-29_f64, 1e-10, 0.1, 10.0][case % 4];
    let (m1, m2) = [
        (2e-4, 2e-4),
        (1.0, 1.0),
        (2e-4, 1e6),
        (1e6, 2e-4),
        (1e60, 1e60),
        (1.0, 1e60),
    ][case / 4 % 6];
    let fixed = (case / 24).is_multiple_of(2);
    let offset = (case / 48) % 2;
    let anisotropic = (case / 96) % 2;
    let mut backend = DartBackend::new([0.0, 0.0, -9.8]).unwrap();
    backend
        .spawn_static_plane(PlaneSpec {
            position: [0.0; 3],
            orientation: [0.0, 0.0, 0.0, 1.0],
            size: [100.0; 2],
        })
        .unwrap();
    let q = UnitQuaternion::from_euler_angles(0.4, -0.7, 0.2);
    let q = q.quaternion();
    let mut bodies = Vec::new();
    for (i, mass) in [m1, m2].into_iter().enumerate() {
        let moment = (mass * radius * radius).max(1e-60) * 10.0;
        let result = backend.spawn_link(
            BodySpec {
                shape: match case / 192 {
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
                },
                position: [i as f64 * 3.0 * radius, 0.0, radius],
                velocity: [0.0; 3],
                mass: Some(mass),
            },
            RotationSpec {
                orientation: [q.i, q.j, q.k, q.w],
                ..Default::default()
            },
            Some(Inertia {
                center: [offset as f64 * 0.1 * radius; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [
                    moment,
                    moment * if anisotropic == 1 { 1e7 } else { 1.0 },
                    moment * if anisotropic == 1 { 1e7 } else { 1.0 },
                    0.0,
                    0.0,
                    0.0,
                ],
            }),
        );
        let Ok(body) = result else {
            println!("outcome: spawn refused");
            return;
        };
        bodies.push(body);
    }
    let anchor = [0.0, 0.0, radius];
    if fixed
        && backend
            .add_world_revolute(bodies[0], anchor, [0.3, 0.8, 0.5])
            .is_err()
    {
        println!("outcome: root refused");
        return;
    }
    let before = bodies
        .iter()
        .map(|id| backend.state(*id))
        .collect::<Vec<_>>();
    let Ok(joint) = backend.add_revolute(bodies[0], bodies[1], anchor, [0.0, 0.0, 1.0]) else {
        assert_eq!(
            bodies
                .iter()
                .map(|id| backend.state(*id))
                .collect::<Vec<_>>(),
            before
        );
        println!("outcome: attachment refused");
        return;
    };
    let mut completed = 0;
    for round in 0..40 {
        let before = bodies
            .iter()
            .map(|id| backend.state(*id))
            .collect::<Vec<_>>();
        let rate = backend.joint_state(joint);
        backend
            .add_joint_effort(joint, radius * radius * m2.min(1.0) * 0.001)
            .unwrap();
        let dt = if round == 39 {
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
            assert_eq!(backend.joint_state(joint), rate);
            assert!(backend.step(dt).is_err());
            assert_eq!(
                bodies
                    .iter()
                    .map(|id| backend.state(*id))
                    .collect::<Vec<_>>(),
                before
            );
            break;
        }
        completed += 1;
    }
    println!("outcome: {completed} steps");
}

#[test]
fn articulated_static_contacts_at_admission_boundaries_do_not_abort() {
    let mut outcomes = std::collections::BTreeMap::new();
    for case in 0..768 {
        let mut child = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "contact_boundary_child", "--nocapture"])
            .env("GZ_DART_CONTACT_CASE", case.to_string())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(30);
        loop {
            if child.try_wait().unwrap().is_some() {
                break;
            }
            if Instant::now() >= deadline {
                child.kill().unwrap();
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        let output = child.wait_with_output().unwrap();
        let stdout = String::from_utf8_lossy(&output.stdout);
        if let Some(outcome) = stdout.lines().find(|line| line.starts_with("outcome:")) {
            *outcomes.entry(outcome.to_owned()).or_insert(0) += 1;
        }
        assert!(
            output.status.success(),
            "case {case}: {:?}\n{}\n{}",
            output.status,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
    println!("{outcomes:?}");
}
