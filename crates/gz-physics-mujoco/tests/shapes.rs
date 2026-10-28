// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cylinder and capsule cases ported from crates/gz-physics-rapier/tests/{cylinder,capsule}.rs.
use gz_physics::{DynamicBackend, PhysicsBackend, PhysicsError, install};
use gz_physics_mujoco::{BodySpec, MujocoBackend, RotationSpec, Shape};
use gz_sim_core::Simulation;
use std::time::Duration;

fn body(shape: Shape) -> BodySpec {
    BodySpec {
        shape,
        position: [0.0, 0.0, 5.0],
        velocity: [0.0; 3],
        mass: Some(2.0),
    }
}

fn cylinder() -> Shape {
    Shape::Cylinder {
        radius: 0.6,
        half_height: 1.4,
    }
}

fn capsule() -> Shape {
    Shape::Capsule {
        radius: 0.6,
        half_length: 1.4,
    }
}

fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = q;
    let t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ];
    [
        v[0] + w * t[0] + y * t[2] - z * t[1],
        v[1] + w * t[1] + z * t[0] - x * t[2],
        v[2] + w * t[2] + x * t[1] - y * t[0],
    ]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Applies the impulse that gives unit angular velocity about every world axis
/// for the given principal moments, before and after reset.
fn check_inertia(shape: Shape, radial: f64, axial: f64) {
    for rotated in [false, true] {
        let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let orientation = if rotated {
            [s, 0.0, 0.0, s]
        } else {
            [0.0, 0.0, 0.0, 1.0]
        };
        let id = mujoco
            .spawn_link(
                body(shape),
                RotationSpec {
                    orientation,
                    angular_velocity: [0.0; 3],
                },
                None,
            )
            .unwrap();
        let impulse = if rotated {
            [radial, axial, radial]
        } else {
            [radial, radial, axial]
        };
        for _ in 0..2 {
            mujoco.apply_impulse(id, [2.0, 0.0, 0.0], impulse).unwrap();
            let state = mujoco.state(id).unwrap();
            for (actual, expected) in state.velocity.into_iter().zip([1.0, 0.0, 0.0]) {
                assert!((actual - expected).abs() < 1e-12, "{state:?}");
            }
            for value in state.angular_velocity {
                assert!((value - 1.0).abs() < 1e-12, "{state:?}");
            }
            for (actual, expected) in state.orientation.into_iter().zip(orientation) {
                assert!((actual - expected).abs() < 1e-15);
            }
            mujoco.reset().unwrap();
            let state = mujoco.state(id).unwrap();
            assert_eq!(state.velocity, [0.0; 3]);
            assert_eq!(state.angular_velocity, [0.0; 3]);
        }
    }
}

#[test]
fn cylinder_inertia_uses_gazebo_z_axis_in_body_and_world_frames() {
    let axial = 0.5 * 2.0 * 0.6f64.powi(2);
    let radial = 2.0 * (3.0 * 0.6f64.powi(2) + 2.8f64.powi(2)) / 12.0;
    check_inertia(cylinder(), radial, axial);
}

#[test]
fn capsule_inertia_uses_gazebo_z_axis_in_body_and_world_frames() {
    let cylinder_mass = 2.0 * 2.8 / (2.8 + 4.0 * 0.6 / 3.0);
    let cap_mass = 2.0 - cylinder_mass;
    let axial = (0.5 * cylinder_mass + 0.4 * cap_mass) * 0.6f64.powi(2);
    // Two hemispheres shifted by half the cylindrical section length.
    let radial = cylinder_mass * (3.0 * 0.6f64.powi(2) + 2.8f64.powi(2)) / 12.0
        + cap_mass * (0.4 * 0.6f64.powi(2) + 3.0 * 2.8 * 0.6 / 8.0 + 1.4f64.powi(2));
    check_inertia(capsule(), radial, axial);
}

#[test]
fn box_inertia_uses_half_extents() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let id = mujoco
        .spawn(body(Shape::Box {
            half_extents: [0.1, 0.2, 0.3],
        }))
        .unwrap();
    let moments = [0.13, 0.10, 0.05].map(|v| 2.0 * v / 3.0);
    mujoco.apply_impulse(id, [0.0; 3], moments).unwrap();
    for value in mujoco.state(id).unwrap().angular_velocity {
        assert!((value - 1.0).abs() < 1e-12);
    }
}

/// The lowest support point of a resting cylinder or capsule stays on the floor.
fn rests_on_ground(shape: Shape, support: impl Fn([f64; 3]) -> [f64; 3]) {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
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
    let id = mujoco.spawn(body(shape)).unwrap();
    install(&mut sim, mujoco).unwrap();
    sim.step_exact(2000).unwrap();
    let state = sim.world().resource::<PhysicsBackend>().state(id).unwrap();
    let axis = rotate(state.orientation, [0.0, 0.0, 1.0]);
    let support = support(axis);
    // MuJoCo's soft contacts settle with a small penetration.
    assert!((state.position[2] + support[2]).abs() < 0.01, "{state:?}");
    let contact = cross(state.angular_velocity, support);
    assert!((state.velocity[2] + contact[2]).abs() < 0.01, "{state:?}");
}

#[test]
fn cylinder_ground_contact_matches_its_transformed_z_axis_support() {
    rests_on_ground(cylinder(), |axis| {
        let radial = [0.0, 1.0, 2.0].map(|i: f64| {
            let i = i as usize;
            [0.0, 0.0, 1.0][i] - axis[i] * axis[2]
        });
        let length = radial.into_iter().map(|v| v * v).sum::<f64>().sqrt();
        let unit = if length > 0.0 {
            radial.map(|v| v / length)
        } else {
            [0.0; 3]
        };
        [0, 1, 2].map(|i| -1.4 * axis[i] * axis[2].signum() - 0.6 * unit[i])
    });
}

#[test]
fn capsule_ground_contact_matches_its_transformed_z_axis_support() {
    rests_on_ground(capsule(), |axis| {
        [0, 1, 2].map(|i| -1.4 * axis[i] * axis[2].signum() - 0.6 * [0.0, 0.0, 1.0][i])
    });
}

#[test]
fn invalid_or_unrepresentable_shapes_leave_existing_state_unchanged() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    let id = mujoco.spawn(body(cylinder())).unwrap();
    let before = mujoco.state(id).unwrap();
    let cylinders = [
        0.0,
        -1.0,
        f64::NAN,
        f64::INFINITY,
        f64::MAX,
        f64::MIN_POSITIVE,
        f64::from_bits(1),
    ]
    .into_iter()
    .flat_map(|invalid| {
        [
            Shape::Cylinder {
                radius: invalid,
                half_height: 1.4,
            },
            Shape::Cylinder {
                radius: 0.6,
                half_height: invalid,
            },
        ]
    });
    let capsules = [0.0, -1.0, f64::NAN, f64::INFINITY, f64::MAX]
        .into_iter()
        .flat_map(|invalid| {
            [
                Shape::Capsule {
                    radius: invalid,
                    half_length: 1.4,
                },
                Shape::Capsule {
                    radius: 0.6,
                    half_length: invalid,
                },
            ]
        });
    for shape in cylinders.chain(capsules) {
        assert_eq!(
            mujoco.spawn(body(shape)),
            Err(PhysicsError::InvalidValue),
            "{shape:?}"
        );
        assert_eq!(mujoco.state(id), Some(before));
    }
}

#[test]
fn tiny_capsule_radius_rejects_underflow_but_short_segments_remain_valid() {
    let mut mujoco = MujocoBackend::new([0.0; 3]).unwrap();
    for tiny in [f64::MIN_POSITIVE, f64::from_bits(1)] {
        assert_eq!(
            mujoco.spawn(body(Shape::Capsule {
                radius: tiny,
                half_length: 1.4
            })),
            Err(PhysicsError::InvalidValue)
        );
        let id = mujoco
            .spawn(body(Shape::Capsule {
                radius: 0.6,
                half_length: tiny,
            }))
            .unwrap();
        assert_eq!(mujoco.state(id).unwrap().position, [0.0, 0.0, 5.0]);
    }
}
