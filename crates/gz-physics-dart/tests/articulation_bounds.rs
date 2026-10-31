// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native assertion probes for admitted two-body articulation boundaries.
use gz_physics::{BodySpec, DynamicBackend, Inertia, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use std::{
    process::{Command, Stdio},
    time::{Duration, Instant},
};

#[test]
fn articulation_bound_child() {
    let Ok(case) = std::env::var("GZ_DART_ARTICULATION_CASE") else {
        return;
    };
    let case: usize = case.parse().unwrap();
    let radius = [1e-29_f64, 1e-10, 0.1][case % 3];
    let masses = [(1.0, 1.0), (1.0, 1e60), (1e60, 1.0), (1e-3, 1e5)][case / 3 % 4];
    let same_anchor = (case / 12).is_multiple_of(2);
    let parallel = (case / 24).is_multiple_of(2);
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let mut bodies = Vec::new();
    for (index, mass) in [masses.0, masses.1].into_iter().enumerate() {
        let moment = (mass * radius * radius).max(1e-60);
        bodies.push(
            backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius },
                        position: [index as f64 * 3.0 * radius, 0.0, 0.0],
                        velocity: [0.0; 3],
                        mass: Some(mass),
                    },
                    RotationSpec::default(),
                    Some(Inertia {
                        center: [0.0; 3],
                        orientation: [0.0, 0.0, 0.0, 1.0],
                        tensor: [moment, moment, moment, 0.0, 0.0, 0.0],
                    }),
                )
                .unwrap(),
        );
    }
    let root = backend
        .add_world_revolute(bodies[0], [0.0; 3], [0.0, 0.0, 1.0])
        .unwrap();
    let initial = bodies
        .iter()
        .map(|id| backend.state(*id))
        .collect::<Vec<_>>();
    let anchor = [if same_anchor { 0.0 } else { 3.0 * radius }, 0.0, 0.0];
    let axis = if parallel {
        [0.0, 0.0, 1.0]
    } else {
        [0.3, 0.8, 0.5]
    };
    let Ok(leaf) = backend.add_revolute(bodies[0], bodies[1], anchor, axis) else {
        assert_eq!(
            bodies
                .iter()
                .map(|id| backend.state(*id))
                .collect::<Vec<_>>(),
            initial
        );
        return;
    };
    for _ in 0..4 {
        let before = bodies
            .iter()
            .map(|id| backend.state(*id))
            .collect::<Vec<_>>();
        backend.add_joint_effort(leaf, 1.0).unwrap();
        if backend.step(Duration::from_millis(1)).is_err() {
            assert_eq!(
                bodies
                    .iter()
                    .map(|id| backend.state(*id))
                    .collect::<Vec<_>>(),
                before
            );
            break;
        }
    }
    let _ = backend.apply_impulse(bodies[1], [radius, 0.0, 0.0], [0.0, 0.0, radius * radius]);
    assert!(!backend.remove_joint(root));
    let _ = backend.step(Duration::from_millis(1));
    assert!(backend.remove(bodies[1]));
    let _ = backend.step(Duration::from_millis(1));
    assert!(backend.state(bodies[0]).is_some());
}

#[test]
fn mass_inertia_and_axis_boundaries_do_not_abort_native_dart() {
    for case in 0..48 {
        let mut child = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "articulation_bound_child", "--nocapture"])
            .env("GZ_DART_ARTICULATION_CASE", case.to_string())
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
        assert!(
            output.status.success(),
            "case {case}: {:?}\n{}\n{}",
            output.status,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
}
