// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{BodySpec, PhysicsError, RapierBackend, RotationSpec, Shape, install};
use gz_sim_core::Simulation;
use rapier3d_f64::prelude::{Rotation, Vector};
use std::time::Duration;

fn capsule() -> BodySpec {
    BodySpec {
        shape: Shape::Capsule {
            radius: 0.6,
            half_length: 1.4,
        },
        position: [0.0, 0.0, 5.0],
        velocity: [0.0; 3],
        mass: Some(2.0),
    }
}

#[test]
fn capsule_inertia_uses_gazebo_z_axis_in_body_and_world_frames() {
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
                capsule(),
                RotationSpec {
                    orientation,
                    angular_velocity: [0.0; 3],
                },
                None,
            )
            .unwrap();
        let cylinder_mass = 2.0 * 2.8 / (2.8 + 4.0 * 0.6 / 3.0);
        let cap_mass = 2.0 - cylinder_mass;
        let axial = (0.5 * cylinder_mass + 0.4 * cap_mass) * 0.6f64.powi(2);
        // Two hemispheres shifted by half the cylindrical section length.
        let radial = cylinder_mass * (3.0 * 0.6f64.powi(2) + 2.8f64.powi(2)) / 12.0
            + cap_mass * (0.4 * 0.6f64.powi(2) + 3.0 * 2.8 * 0.6 / 8.0 + 1.4f64.powi(2));
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
            backend.reset();
            let state = backend.state(id).unwrap();
            assert_eq!(state.velocity, [0.0; 3]);
            assert_eq!(state.angular_velocity, [0.0; 3]);
        }
    }
}

#[test]
fn capsule_ground_contact_matches_its_transformed_z_axis_support() {
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
    let id = backend.spawn(capsule()).unwrap();
    install(&mut sim, backend).unwrap();
    sim.step_exact(2000).unwrap();
    let state = sim.world().resource::<RapierBackend>().state(id).unwrap();
    let q = state.orientation;
    let axis = Rotation::from_xyzw(q[0], q[1], q[2], q[3]) * Vector::Z;
    let support = -1.4 * axis * axis.z.signum() - 0.6 * Vector::Z;
    // The capsule may rock; its lowest support point remains on the floor.
    assert!((state.position[2] + support.z).abs() < 0.01, "{state:?}");
    let contact_velocity =
        Vector::from(state.velocity) + Vector::from(state.angular_velocity).cross(support);
    assert!(contact_velocity.z.abs() < 0.01, "{state:?}");
}

#[test]
fn invalid_or_unrepresentable_capsules_leave_existing_state_unchanged() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend.spawn(capsule()).unwrap();
    let before = backend.state(id).unwrap();
    for invalid in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::MAX] {
        for shape in [
            Shape::Capsule {
                radius: invalid,
                half_length: 1.4,
            },
            Shape::Capsule {
                radius: 0.6,
                half_length: invalid,
            },
        ] {
            assert_eq!(
                backend.spawn(BodySpec { shape, ..capsule() }),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!(backend.state(id), Some(before));
        }
    }
}

#[test]
fn tiny_capsule_radius_rejects_underflow_but_short_segments_remain_valid() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    for tiny in [f64::MIN_POSITIVE, f64::from_bits(1)] {
        assert_eq!(
            backend.spawn(BodySpec {
                shape: Shape::Capsule {
                    radius: tiny,
                    half_length: 1.4
                },
                ..capsule()
            }),
            Err(PhysicsError::InvalidValue)
        );
        let id = backend
            .spawn(BodySpec {
                shape: Shape::Capsule {
                    radius: 0.6,
                    half_length: tiny,
                },
                ..capsule()
            })
            .unwrap();
        assert_eq!(backend.state(id).unwrap().position, capsule().position);
    }
}
