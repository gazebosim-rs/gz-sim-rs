// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{BodySpec, PhysicsError, RapierBackend, RotationSpec, Shape, install};
use gz_physics_rapier::{DynamicBackend, PhysicsBackend};
use gz_sim_core::Simulation;
use rapier3d_f64::prelude::{Rotation, Vector};
use std::time::Duration;

fn cylinder() -> BodySpec {
    BodySpec {
        shape: Shape::Cylinder {
            radius: 0.6,
            half_height: 1.4,
        },
        position: [0.0, 0.0, 5.0],
        velocity: [0.0; 3],
        mass: Some(2.0),
    }
}

#[test]
fn cylinder_inertia_uses_gazebo_z_axis_in_body_and_world_frames() {
    for rotated in [false, true] {
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let orientation = if rotated {
            let s = std::f64::consts::FRAC_1_SQRT_2;
            [s, 0.0, 0.0, s]
        } else {
            [0.0, 0.0, 0.0, 1.0]
        };
        let id = backend
            .spawn_link(
                cylinder(),
                RotationSpec {
                    orientation,
                    angular_velocity: [0.0; 3],
                },
                None,
            )
            .unwrap();
        let axial = 0.5 * 2.0 * 0.6f64.powi(2);
        let radial = 2.0 * (3.0 * 0.6f64.powi(2) + 2.8f64.powi(2)) / 12.0;
        let impulse = if rotated {
            [radial, axial, radial]
        } else {
            [radial, radial, axial]
        };
        for _ in 0..2 {
            backend.apply_impulse(id, [2.0, 0.0, 0.0], impulse).unwrap();
            let state = backend.state(id).unwrap();
            assert_eq!(state.velocity, [1.0, 0.0, 0.0]);
            for value in state.angular_velocity {
                assert!((value - 1.0).abs() < 1e-12);
            }
            assert_eq!(state.orientation, orientation);
            backend.reset().unwrap();
            let state = backend.state(id).unwrap();
            assert_eq!(state.velocity, [0.0; 3]);
            assert_eq!(state.angular_velocity, [0.0; 3]);
        }
    }
}

#[test]
fn cylinder_ground_contact_matches_its_transformed_z_axis_support() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    backend
        .spawn(BodySpec {
            shape: Shape::Box {
                half_extents: [10.0, 10.0, 0.5],
            },
            position: [0.0, 0.0, -0.5],
            velocity: [0.0; 3],
            mass: None,
        })
        .unwrap();
    let id = backend.spawn(cylinder()).unwrap();
    install(&mut sim, backend).unwrap();
    sim.step_exact(2000).unwrap();
    let state = sim.world().resource::<PhysicsBackend>().state(id).unwrap();
    let q = state.orientation;
    let axis = Rotation::from_xyzw(q[0], q[1], q[2], q[3]) * Vector::Z;
    let radial = Vector::Z - axis * axis.z;
    let support = -1.4 * axis * axis.z.signum() - 0.6 * radial.normalize_or_zero();
    // The cylinder may rock; its lowest support point remains on the floor.
    assert!((state.position[2] + support.z).abs() < 0.01, "{state:?}");
    let contact_velocity =
        Vector::from(state.velocity) + Vector::from(state.angular_velocity).cross(support);
    assert!(contact_velocity.z.abs() < 0.01, "{state:?}");
}

#[test]
fn invalid_or_unrepresentable_cylinders_leave_existing_state_unchanged() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend.spawn(cylinder()).unwrap();
    let before = backend.state(id).unwrap();
    for invalid in [
        0.0,
        -1.0,
        f64::NAN,
        f64::INFINITY,
        f64::MAX,
        f64::MIN_POSITIVE,
        f64::from_bits(1),
    ] {
        for shape in [
            Shape::Cylinder {
                radius: invalid,
                half_height: 1.4,
            },
            Shape::Cylinder {
                radius: 0.6,
                half_height: invalid,
            },
        ] {
            assert_eq!(
                backend.spawn(BodySpec {
                    shape,
                    ..cylinder()
                }),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!(backend.state(id), Some(before));
        }
    }
}
