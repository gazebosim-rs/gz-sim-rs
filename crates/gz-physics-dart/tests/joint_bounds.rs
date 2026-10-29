// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, Inertia, JointId, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use nalgebra::{UnitQuaternion, Vector3};
use std::{process::Command, time::Duration};

fn pair(case: usize) -> (DartBackend, [(BodyId, JointId); 2], f64) {
    let (radius_exponent, ratio, seed, effort_exponent) = [
        (-20.0, 1.0, 0.0, 0.0),
        (-20.0, 10.0, 1.0, 0.0),
        (-29.0, 1.0, 0.0, -10.0),
    ][case];
    let radius = 10f64.powf(radius_exponent);
    let effort = 10f64.powf(effort_exponent);
    let rotation =
        UnitQuaternion::from_euler_angles(0.3 + seed * 0.07, 0.6 - seed * 0.03, 0.9 + seed * 0.01);
    let q = rotation.quaternion();
    let orientation = if seed == 0.0 {
        [0.0, 0.0, 0.0, 1.0]
    } else {
        [q.i, q.j, q.k, q.w]
    };
    let axis = if seed == 0.0 {
        Vector3::y()
    } else {
        Vector3::new(0.3 + seed * 0.01, 0.7 - seed * 0.003, 0.2 + seed * 0.002).normalize()
    };
    let axis2 = Vector3::new(-0.4 + seed * 0.001, 0.2 + seed * 0.007, 0.9).normalize();
    let moment = 10f64.powf(2.0 * radius_exponent);
    let mut b = DartBackend::new([0.0; 3]).unwrap();
    let first = b
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [radius, 0.8 * radius, 0.6 * radius],
                },
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec {
                orientation,
                angular_velocity: (axis * 10.0).into(),
            },
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [moment, moment * ratio, moment * ratio, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let first_joint = b.add_world_revolute(first, [0.0; 3], axis.into()).unwrap();
    let position = [0.8 * radius, 0.0, -0.4 * radius];
    let second = b
        .spawn_link(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [radius; 3],
                },
                position,
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec {
                orientation,
                angular_velocity: (axis2 * 7.0).into(),
            },
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [moment * ratio, moment, moment * ratio, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let second_joint = b
        .add_world_revolute(second, position, axis2.into())
        .unwrap();
    (b, [(first, first_joint), (second, second_joint)], effort)
}

#[test]
fn joint_bound_child() {
    let Ok(case) = std::env::var("GZ_DART_JOINT_BOUND_CASE") else {
        return;
    };
    let (mut b, pair, effort) = pair(case.parse().unwrap());
    for _ in 0..20 {
        let before = pair.map(|(body, joint)| (b.state(body), b.joint_state(joint)));
        b.add_joint_effort(pair[0].1, effort).unwrap();
        if let Err(error) = b.step(Duration::from_millis(1)) {
            assert!(error.to_string().contains("active joint limit"), "{error}");
            assert_eq!(
                pair.map(|(body, joint)| (b.state(body), b.joint_state(joint))),
                before
            );
            return;
        }
    }
    panic!("the ill-conditioned mixed angular row was not refused");
}

#[test]
fn implicit_hinge_limits_refuse_ill_conditioned_rows_without_aborting() {
    for case in 0..3 {
        let output = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "joint_bound_child", "--nocapture"])
            .env("GZ_DART_JOINT_BOUND_CASE", case.to_string())
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "case {case}: {:?}\n{}\n{}",
            output.status,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
}

#[test]
fn refusal_preserves_other_bodies_and_pending_effort_and_world_wrench() {
    let (mut b, pair, effort) = pair(2);
    let safe = b
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.2 },
                position: [3.0, 0.0, 0.0],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    let safe_joint = b
        .add_world_revolute(safe, [3.0, 0.0, 0.0], [0.0, 1.0, 0.0])
        .unwrap();
    let before = [pair[0], pair[1], (safe, safe_joint)]
        .map(|(body, joint)| (b.state(body), b.joint_state(joint)));
    b.add_joint_effort(pair[0].1, effort).unwrap();
    b.add_joint_effort(safe_joint, 0.8).unwrap();
    b.add_world_wrench(safe, [0.0; 3], [0.0, 0.8, 0.0]).unwrap();
    for _ in 0..2 {
        let error = b.step(Duration::from_millis(1)).unwrap_err();
        assert!(error.to_string().contains("active joint limit"), "{error}");
        assert_eq!(
            [pair[0], pair[1], (safe, safe_joint)]
                .map(|(body, joint)| (b.state(body), b.joint_state(joint))),
            before
        );
    }
    assert!(b.remove(pair[0].0));
    assert!(b.remove(pair[1].0));
    b.step(Duration::from_millis(1)).unwrap();
    assert!((b.joint_state(safe_joint).unwrap().velocity - 0.1).abs() < 1e-12);
    b.step(Duration::from_millis(1)).unwrap();
    assert!((b.joint_state(safe_joint).unwrap().velocity - 0.1).abs() < 1e-12);
}

#[test]
fn active_limit_with_bounded_inverse_inertia_still_uses_native_solver() {
    for floor in [false, true] {
        let mut b = DartBackend::new([0.0; 3]).unwrap();
        if floor {
            b.spawn_link(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [5.0, 5.0, 0.5],
                    },
                    position: [0.0, 0.0, -0.7],
                    velocity: [0.0; 3],
                    mass: None,
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        }
        let body = b
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.2 },
                    position: [0.0; 3],
                    velocity: [0.0; 3],
                    mass: Some(1.0),
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        let joint = b
            .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
            .unwrap();
        for _ in 0..3 {
            b.add_joint_effort(joint, 1e20).unwrap();
            b.step(Duration::from_millis(1)).unwrap();
        }
        let sample = b.joint_state(joint).unwrap();
        assert!(sample.position.is_finite() && sample.velocity.is_finite());
        if !floor {
            assert!(sample.position > 1e16);
            assert!(sample.velocity.abs() < 1e12);
        }
    }
}

#[test]
fn isolated_scalar_limit_keeps_native_solver_even_with_tiny_inertia() {
    let mut b = DartBackend::new([0.0; 3]).unwrap();
    let body = b
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 1e-29 },
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [1e-58, 1e-58, 1e-58, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap();
    let joint = b
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    for _ in 0..10 {
        b.add_joint_effort(joint, 1e-10).unwrap();
        b.step(Duration::from_millis(1)).unwrap();
    }
    let sample = b.joint_state(joint).unwrap();
    assert!(sample.position.abs() > 1e16);
    assert!(sample.position.is_finite() && sample.velocity.is_finite());
}
