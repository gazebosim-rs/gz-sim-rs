// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ports of gz-physics-rapier/tests/{capsule,cylinder}.rs plus box inertia.

use gz_physics::{BodySpec, BodyState, DynamicBackend, PhysicsError, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use nalgebra::{UnitQuaternion, Vector3};
use std::time::Duration;

fn body(shape: Shape) -> BodySpec {
    BodySpec {
        shape,
        position: [0.0, 0.0, 5.0],
        velocity: [0.0; 3],
        mass: Some(2.0),
    }
}

const CAPSULE: Shape = Shape::Capsule {
    radius: 0.6,
    half_length: 1.4,
};
const CYLINDER: Shape = Shape::Cylinder {
    radius: 0.6,
    half_height: 1.4,
};

/// Principal moments about body X, Y and Z for the shapes above with mass 2.
fn moments(shape: Shape) -> [f64; 3] {
    match shape {
        Shape::Capsule { .. } => {
            let cylinder_mass = 2.0 * 2.8 / (2.8 + 4.0 * 0.6 / 3.0);
            let cap_mass = 2.0 - cylinder_mass;
            let axial = (0.5 * cylinder_mass + 0.4 * cap_mass) * 0.6f64.powi(2);
            // Two hemispheres shifted by half the cylindrical section length.
            let radial = cylinder_mass * (3.0 * 0.6f64.powi(2) + 2.8f64.powi(2)) / 12.0
                + cap_mass * (0.4 * 0.6f64.powi(2) + 3.0 * 2.8 * 0.6 / 8.0 + 1.4f64.powi(2));
            [radial, radial, axial]
        }
        Shape::Cylinder { .. } => {
            let radial = 2.0 * (3.0 * 0.6f64.powi(2) + 2.8f64.powi(2)) / 12.0;
            [radial, radial, 0.5 * 2.0 * 0.6f64.powi(2)]
        }
        _ => {
            // Box with half extents 0.5, 1 and 1.5.
            let full = [1.0_f64, 2.0, 3.0];
            [
                2.0 * (full[1].powi(2) + full[2].powi(2)) / 12.0,
                2.0 * (full[0].powi(2) + full[2].powi(2)) / 12.0,
                2.0 * (full[0].powi(2) + full[1].powi(2)) / 12.0,
            ]
        }
    }
}

#[test]
fn shape_inertia_uses_gazebo_z_axis_in_body_and_world_frames() {
    let cuboid = Shape::Box {
        half_extents: [0.5, 1.0, 1.5],
    };
    for shape in [CAPSULE, CYLINDER, cuboid] {
        for rotated in [false, true] {
            let mut backend = DartBackend::new([0.0; 3]).unwrap();
            let orientation = if rotated {
                let s = std::f64::consts::FRAC_1_SQRT_2;
                [s, 0.0, 0.0, s]
            } else {
                [0.0, 0.0, 0.0, 1.0]
            };
            let id = backend
                .spawn_link(
                    body(shape),
                    RotationSpec {
                        orientation,
                        angular_velocity: [0.0; 3],
                    },
                    None,
                )
                .unwrap();
            let [x, y, z] = moments(shape);
            // A quarter turn about X exchanges the world Y and Z moments.
            let impulse = if rotated { [x, z, y] } else { [x, y, z] };
            for _ in 0..2 {
                backend.apply_impulse(id, [2.0, 0.0, 0.0], impulse).unwrap();
                let state = backend.state(id).unwrap();
                assert!((state.velocity[0] - 1.0).abs() < 1e-12);
                for value in state.angular_velocity {
                    assert!((value - 1.0).abs() < 1e-12, "{shape:?} {state:?}");
                }
                backend.reset().unwrap();
                let state = backend.state(id).unwrap();
                assert_eq!(state.velocity, [0.0; 3]);
                assert_eq!(state.angular_velocity, [0.0; 3]);
            }
        }
    }
}

/// Lowest point of a Z-axis shape (`half` along the axis, radius around it).
fn support(state: &BodyState, half: f64, radius: f64, capsule: bool) -> Vector3<f64> {
    let q = state.orientation;
    let rotation =
        UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q[3], q[0], q[1], q[2]));
    let axis = rotation * Vector3::z();
    let toward = if capsule {
        -Vector3::z()
    } else {
        -(Vector3::z() - axis * axis.z)
            .try_normalize(0.0)
            .unwrap_or_else(Vector3::zeros)
    };
    -half * axis * axis.z.signum() + radius * toward
}

#[test]
fn capsule_and_cylinder_ground_contact_match_their_z_axis_support() {
    for (shape, capsule) in [(CAPSULE, true), (CYLINDER, false)] {
        let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
        backend
            .spawn_link(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: [10.0, 10.0, 0.5],
                    },
                    position: [0.0, 0.0, -0.5],
                    velocity: [0.0; 3],
                    mass: None,
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        let id = backend
            .spawn_link(body(shape), RotationSpec::default(), None)
            .unwrap();
        for _ in 0..2000 {
            backend.step(Duration::from_millis(1)).unwrap();
        }
        let state = backend.state(id).unwrap();
        let lowest = support(&state, 1.4, 0.6, capsule);
        // The body may rock; its lowest support point remains on the floor.
        assert!((state.position[2] + lowest.z).abs() < 0.01, "{state:?}");
        let contact =
            Vector3::from(state.velocity) + Vector3::from(state.angular_velocity).cross(&lowest);
        assert!(contact.z.abs() < 0.01, "{state:?}");
    }
}

#[test]
fn invalid_or_unrepresentable_shapes_leave_existing_state_unchanged() {
    let mut backend = DartBackend::new([0.0; 3]).unwrap();
    let id = backend
        .spawn_link(body(CAPSULE), RotationSpec::default(), None)
        .unwrap();
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
                backend.spawn_link(body(shape), RotationSpec::default(), None),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!(backend.state(id), Some(before));
        }
    }
    for tiny in [f64::MIN_POSITIVE, f64::from_bits(1)] {
        for shape in [
            Shape::Capsule {
                radius: tiny,
                half_length: 1.4,
            },
            Shape::Cylinder {
                radius: tiny,
                half_height: 1.4,
            },
            Shape::Cylinder {
                radius: 0.6,
                half_height: tiny,
            },
        ] {
            assert_eq!(
                backend.spawn_link(body(shape), RotationSpec::default(), None),
                Err(PhysicsError::InvalidValue)
            );
        }
    }
    // DART admits no dimension below 1e-60 (docs/DART.md), where Rapier accepted
    // any positive capsule segment. The shortest admitted segment is valid
    // because the radius still spans every direction; a shorter one is not.
    let short = |half_length| {
        body(Shape::Capsule {
            radius: 0.6,
            half_length,
        })
    };
    let id = backend
        .spawn_link(short(1e-60), RotationSpec::default(), None)
        .unwrap();
    assert_eq!(backend.state(id).unwrap().position, [0.0, 0.0, 5.0]);
    for tiny in [0.99e-60, f64::MIN_POSITIVE] {
        assert_eq!(
            backend.spawn_link(short(tiny), RotationSpec::default(), None),
            Err(PhysicsError::InvalidValue)
        );
    }
}
