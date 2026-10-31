// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Original ApplyJointForce physics values, compared with the native plugin.
use gz_physics::{BodySpec, DynamicBackend, Inertia, PlaneSpec, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use std::time::Duration;

#[test]
fn original_ground_fixture_matches_native_body_and_joint_observations() {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/ground_contact.json")).unwrap();
    for force in [0.001, -0.001] {
        let mut backend = DartBackend::new([0.0, 0.0, -9.8]).unwrap();
        backend
            .spawn_static_plane(PlaneSpec {
                position: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                size: [100.0; 2],
            })
            .unwrap();
        let mut bodies = Vec::new();
        for (mass, z, half_extents, moments) in [
            (120.0, 0.005, [0.25, 0.25, 0.005], [2.501, 2.501, 5.0]),
            (0.6, 1.005, [0.125, 0.125, 0.025], [0.032, 0.032, 0.00012]),
        ] {
            bodies.push(
                backend
                    .spawn_link(
                        BodySpec {
                            shape: Shape::Box { half_extents },
                            position: [0.0, 0.0, z],
                            velocity: [0.0; 3],
                            mass: Some(mass),
                        },
                        RotationSpec::default(),
                        Some(Inertia {
                            center: [0.0; 3],
                            orientation: [0.0, 0.0, 0.0, 1.0],
                            tensor: [moments[0], moments[1], moments[2], 0.0, 0.0, 0.0],
                        }),
                    )
                    .unwrap(),
            );
        }
        let joint = backend
            .add_revolute(bodies[0], bodies[1], [0.0, 0.0, 0.505], [0.0, 0.0, 1.0])
            .unwrap();
        let mut tick = 0;
        for row in fixture["observations"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|r| r["force"].as_f64().unwrap() == force)
        {
            while tick < row["tick"].as_u64().unwrap() {
                tick += 1;
                backend
                    .add_joint_effort(joint, if tick <= 10 { 0.0 } else { force })
                    .unwrap();
                backend.step(Duration::from_millis(1)).unwrap();
            }
            for (i, body) in bodies.iter().enumerate() {
                let state = backend.state(*body).unwrap();
                let expected = &row["bodies"][i];
                for (field, actual) in [
                    ("position", state.position.to_vec()),
                    ("orientation", state.orientation.to_vec()),
                    ("velocity", state.velocity.to_vec()),
                    ("angular_velocity", state.angular_velocity.to_vec()),
                ] {
                    for (a, b) in actual.iter().zip(expected[field].as_array().unwrap()) {
                        let b = b.as_f64().unwrap();
                        assert!(
                            (a - b).abs() < 1e-12,
                            "force={force} tick={tick} body={i} {field}: {a} != {b}"
                        );
                    }
                }
            }
            let actual = backend.joint_state(joint).unwrap();
            for (field, a) in [("q", actual.position), ("dq", actual.velocity)] {
                let b = row[field].as_f64().unwrap();
                assert!(
                    (a - b).abs() < 1e-12,
                    "force={force} tick={tick} {field}: {a} != {b}"
                );
            }
        }
    }
}
