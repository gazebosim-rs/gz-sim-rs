// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Same-engine parity with the installed gz-physics MuJoCo plugin. The fixture
//! comes from tests/fixtures/reference/mujoco_reference.cc (see README.md there).
//! Both sides run MuJoCo 3.11.0 with the same options, body order and geometry,
//! so trajectories agree to rounding (worst observed 8.9e-16). The 1e-14 bound
//! allows native readback differences, such as quaternions rebuilt from matrices.
use gz_physics::{BodyId, DynamicBackend, JointId};
use gz_physics_mujoco::{BodySpec, Inertia, MujocoBackend, RotationSpec, Shape};
use serde_json::Value;
use std::time::Duration;

const TOLERANCE: f64 = 1e-14;

fn sphere(
    mujoco: &mut MujocoBackend,
    position: [f64; 3],
    rotation: RotationSpec,
    radius: f64,
    moment: f64,
) -> BodyId {
    mujoco
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius },
                position,
                velocity: [0.0; 3],
                mass: Some(1.0),
            },
            rotation,
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [moment, moment, moment, 0.0, 0.0, 0.0],
            }),
        )
        .unwrap()
}

fn compare(mujoco: &mut MujocoBackend, scenario: &str, body: BodyId, joint: Option<JointId>) {
    let rows: Vec<Value> =
        serde_json::from_str(include_str!("fixtures/mujoco_reference.json")).unwrap();
    let rows: Vec<_> = rows
        .iter()
        .filter(|row| row["scenario"] == scenario)
        .collect();
    assert!(!rows.is_empty());
    let mut step = 0;
    let mut worst = 0.0_f64;
    for row in rows {
        let target = row["iteration"].as_u64().unwrap();
        while step < target {
            mujoco.step(Duration::from_millis(1)).unwrap();
            step += 1;
        }
        let state = mujoco.state(body).unwrap();
        let vector = |key: &str| -> Vec<f64> {
            row[key]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap())
                .collect()
        };
        let mut pairs: Vec<(f64, f64)> = [
            (state.position.as_slice(), vector("position")),
            (state.orientation.as_slice(), vector("orientation")),
            (state.velocity.as_slice(), vector("velocity")),
            (
                state.angular_velocity.as_slice(),
                vector("angular_velocity"),
            ),
        ]
        .into_iter()
        .flat_map(|(actual, expected)| actual.iter().copied().zip(expected).collect::<Vec<_>>())
        .collect();
        if let Some(joint) = joint {
            let hinge = mujoco.joint_state(joint).unwrap();
            pairs.push((hinge.position, row["q"].as_f64().unwrap()));
            pairs.push((hinge.velocity, row["dq"].as_f64().unwrap()));
        }
        for (actual, expected) in pairs {
            worst = worst.max((actual - expected).abs());
            assert!(
                (actual - expected).abs() <= TOLERANCE,
                "{scenario} step {step}: {actual} versus {expected}"
            );
        }
    }
    eprintln!("{scenario}: worst absolute difference {worst:e}");
}

#[test]
fn free_fall_matches_the_native_plugin() {
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    let ball = sphere(
        &mut mujoco,
        [0.0, 0.0, 5.0],
        RotationSpec::default(),
        0.5,
        0.1,
    );
    compare(&mut mujoco, "free_fall", ball, None);
}

#[test]
fn sphere_contact_on_a_static_box_matches_the_native_plugin() {
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    mujoco
        .spawn(BodySpec {
            shape: Shape::Box {
                half_extents: [10.0, 10.0, 0.5],
            },
            position: [0.0, 0.0, -0.5],
            velocity: [0.0; 3],
            mass: None,
        })
        .unwrap();
    let ball = sphere(
        &mut mujoco,
        [0.0, 0.0, 1.0],
        RotationSpec::default(),
        0.5,
        0.1,
    );
    compare(&mut mujoco, "ground_contact", ball, None);
}

#[test]
fn world_revolute_pendulum_matches_the_native_plugin() {
    let theta = 0.02_f64;
    let mut mujoco = MujocoBackend::new([0.0, 0.0, -9.81]).unwrap();
    let body = sphere(
        &mut mujoco,
        [-theta.sin(), 0.0, -theta.cos()],
        RotationSpec {
            orientation: [0.0, (theta / 2.0).sin(), 0.0, (theta / 2.0).cos()],
            angular_velocity: [0.0; 3],
        },
        0.1,
        0.004,
    );
    let joint = mujoco
        .add_world_revolute(body, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    compare(&mut mujoco, "pendulum", body, Some(joint));
}
