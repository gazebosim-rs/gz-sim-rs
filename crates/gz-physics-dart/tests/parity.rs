// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Same-engine parity with the gz-physics dartsim plugin built against the same
//! DART 6.13.2. tests/fixtures/reference.cc generates the fixture; docs/DART.md
//! records the command.

use gz_physics::{BodyId, BodySpec, DynamicBackend, Inertia, JointId, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde_json::Value;
use std::time::Duration;

fn ball() -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.5 },
        position: [0.0, 0.0, 5.0],
        velocity: [0.0; 3],
        mass: Some(1.0),
    }
}

fn rows() -> Vec<Value> {
    serde_json::from_str(include_str!("fixtures/reference.json")).unwrap()
}

fn array<const N: usize>(row: &Value, key: &str) -> [f64; N] {
    values(row, key).try_into().unwrap()
}

/// The plugin's tilted hinge, built from the world-frame inputs it recorded: a
/// rotated link with an offset COM in a rotated inertial frame, hinged about a
/// tilted axis and already turning at `rate`. The spawn twist adds an angular
/// component perpendicular to the axis, which the hinge cannot carry.
fn tilted_hinge(backend: &mut DartBackend) -> (BodyId, JointId) {
    let rows = rows();
    let inputs = rows
        .iter()
        .find(|row| row["scenario"] == "tilted_hinge_inputs")
        .unwrap();
    let start = rows
        .iter()
        .find(|row| row["scenario"] == "tilted_hinge" && row["step"] == 0)
        .unwrap();
    let position: [f64; 3] = array(start, "position");
    let orientation: [f64; 4] = array(start, "orientation");
    let center: [f64; 3] = array(inputs, "center");
    let anchor = Vector3::from(array::<3>(inputs, "anchor"));
    let axis = Vector3::from(array::<3>(inputs, "axis"));
    let rate = inputs["rate"].as_f64().unwrap();
    let [x, y, z, w] = orientation;
    let rotation = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
    let com = Vector3::from(position) + rotation * Vector3::from(center);
    let perpendicular = axis.cross(&Vector3::z()).normalize() * 0.4;
    let body = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position,
                velocity: (rate * axis.cross(&(com - anchor))).into(),
                mass: Some(inputs["mass"].as_f64().unwrap()),
            },
            RotationSpec {
                orientation,
                angular_velocity: (rate * axis + perpendicular).into(),
            },
            Some(Inertia {
                center,
                orientation: array(inputs, "inertial_orientation"),
                tensor: array(inputs, "tensor"),
            }),
        )
        .unwrap();
    let joint = backend
        .add_world_revolute(body, anchor.into(), axis.into())
        .unwrap();
    (body, joint)
}

fn scenario(name: &str) -> (DartBackend, BodyId, Option<JointId>) {
    let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    let rotation = RotationSpec::default();
    match name {
        "free_fall" => {
            let body = backend.spawn_link(ball(), rotation, None).unwrap();
            (backend, body, None)
        }
        "ground_contact" => {
            let ground = BodySpec {
                shape: Shape::Box {
                    half_extents: [10.0, 10.0, 0.5],
                },
                position: [0.0, 0.0, -0.5],
                velocity: [0.0; 3],
                mass: None,
            };
            backend.spawn_link(ground, rotation, None).unwrap();
            let body = backend.spawn_link(ball(), rotation, None).unwrap();
            (backend, body, None)
        }
        "tilted_hinge" => {
            let (body, joint) = tilted_hinge(&mut backend);
            (backend, body, Some(joint))
        }
        _ => {
            let theta = 0.02_f64;
            let body = backend
                .spawn_link(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.1 },
                        position: [-theta.sin(), 0.0, -theta.cos()],
                        velocity: [0.0; 3],
                        mass: Some(1.0),
                    },
                    RotationSpec {
                        orientation: [0.0, (theta / 2.0).sin(), 0.0, (theta / 2.0).cos()],
                        angular_velocity: [0.0; 3],
                    },
                    Some(Inertia {
                        center: [0.0; 3],
                        orientation: [0.0, 0.0, 0.0, 1.0],
                        tensor: [0.004, 0.004, 0.004, 0.0, 0.0, 0.0],
                    }),
                )
                .unwrap();
            let joint = backend
                .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
                .unwrap();
            (backend, body, Some(joint))
        }
    }
}

fn values(row: &Value, key: &str) -> Vec<f64> {
    row[key]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap())
        .collect()
}

/// Largest absolute difference per scenario across every recorded quantity.
fn compare(name: &str) -> f64 {
    let rows = rows();
    let (mut backend, body, joint) = scenario(name);
    let mut worst = 0.0_f64;
    let mut step = 0;
    let mut count = 0;
    for row in rows.iter().filter(|row| row["scenario"] == name) {
        let target = row["step"].as_u64().unwrap();
        while step < target {
            backend.step(Duration::from_millis(1)).unwrap();
            step += 1;
        }
        let state = backend.state(body).unwrap();
        let mut pairs: Vec<(f64, f64)> = [
            (state.position.to_vec(), values(row, "position")),
            (state.orientation.to_vec(), values(row, "orientation")),
            (state.velocity.to_vec(), values(row, "velocity")),
            (
                state.angular_velocity.to_vec(),
                values(row, "angular_velocity"),
            ),
        ]
        .into_iter()
        .flat_map(|(a, b)| a.into_iter().zip(b))
        .collect();
        if let Some(joint) = joint {
            let hinge = backend.joint_state(joint).unwrap();
            pairs.push((hinge.position, row["joint_position"].as_f64().unwrap()));
            pairs.push((hinge.velocity, row["joint_velocity"].as_f64().unwrap()));
        }
        for (actual, expected) in pairs {
            worst = worst.max((actual - expected).abs());
        }
        count += 1;
    }
    assert!(count > 100, "{name}: only {count} reference rows");
    worst
}

// Identical solver settings and inputs reproduce the plugin bit for bit.
#[test]
fn free_fall_matches_native_dartsim_plugin() {
    let worst = compare("free_fall");
    assert_eq!(worst, 0.0, "free fall differs by {worst:e}");
}

#[test]
fn ground_contact_matches_native_dartsim_plugin() {
    let worst = compare("ground_contact");
    assert_eq!(worst, 0.0, "ground contact differs by {worst:e}");
}

#[test]
fn world_revolute_matches_native_dartsim_plugin() {
    // The fixture's SDF places the joint 1 m along the child Z axis, which DART
    // composes with the child pose; this API takes the world anchor directly.
    // Those joint frames differ by rounding only (observed worst 1.1e-16 over
    // 4000 steps); 1e-14 leaves margin while still requiring identical dynamics.
    let worst = compare("world_revolute");
    assert!(worst < 1e-14, "pendulum differs by {worst:e}");
}

#[test]
fn tilted_offset_hinge_matches_native_dartsim_plugin() {
    // Inputs reach DART by different rounding paths: the inertial rotation via
    // nalgebra rather than gz-math, the world anchor/axis rather than the SDF
    // child-relative joint pose, and the initial rate by least-squares projection
    // rather than SetVelocity. Observed worst 7.8e-16 over 2000 steps; the same
    // 1e-14 bound as the pendulum still requires identical dynamics.
    let worst = compare("tilted_hinge");
    assert!(worst < 1e-14, "tilted hinge differs by {worst:e}");
}
