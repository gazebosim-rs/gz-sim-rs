// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodySpec, DynamicBackend, Inertia, PhysicsError, RotationSpec, Shape};
use gz_physics_rapier::RapierBackend;
use rapier3d_f64::math::{Matrix, Rotation, Vector};
fn input() -> (BodySpec, RotationSpec) {
    (
        BodySpec {
            shape: Shape::Box {
                half_extents: [0.1; 3],
            },
            position: [0.; 3],
            velocity: [0.; 3],
            mass: Some(2.),
        },
        RotationSpec {
            orientation: Rotation::from_scaled_axis(Vector::new(0.23, 0.87, 1.12)).to_array(),
            angular_velocity: [0.; 3],
        },
    )
}
#[test]
fn basis_impulses_recover_the_authored_inverse_for_rotated_repeated_and_near_repeated_moments() {
    let (spec, rot) = input();
    let q = Rotation::from_array(rot.orientation);
    for axial in [0.012, 0.072] {
        for perturb in [0., 1e-13, 1e-8] {
            for size in [1e-6, 1., 1e6] {
                for order in 0..3 {
                    let vals = [0.041, 0.041 + perturb, axial];
                    let d = Vector::new(vals[order], vals[(order + 1) % 3], vals[(order + 2) % 3])
                        * size;
                    for full in [false, true] {
                        let frame = Rotation::from_scaled_axis(Vector::new(-0.3, 0.08, 0.21));
                        let inner = if full {
                            Rotation::from_scaled_axis(Vector::new(0.82, -0.73, 1.31))
                        } else {
                            Rotation::IDENTITY
                        };
                        let r = Matrix::from_quat(inner);
                        let m = r * Matrix::from_diagonal(d) * r.transpose();
                        let inertia = Inertia {
                            center: [0.034, -0.025, 0.07],
                            orientation: frame.to_array(),
                            tensor: [
                                m.x_axis.x, m.y_axis.y, m.z_axis.z, m.y_axis.x, m.z_axis.x,
                                m.z_axis.y,
                            ],
                        };
                        for shape in [
                            spec.shape,
                            Shape::Sphere { radius: 0.12 },
                            Shape::Cylinder {
                                radius: 0.12,
                                half_height: 0.18,
                            },
                            Shape::Capsule {
                                radius: 0.1,
                                half_length: 0.12,
                            },
                        ] {
                            let spec = BodySpec { shape, ..spec };
                            for link in [false, true] {
                                let mut world = RapierBackend::new_multifidelity([0.; 3]).unwrap();
                                let id = if link {
                                    world.spawn_articulation_link(spec, rot, Some(inertia))
                                } else {
                                    world.spawn_link(spec, rot, Some(inertia))
                                }
                                .unwrap();
                                let orientation = q * frame * inner;
                                for axis in [Vector::X, Vector::Y, Vector::Z] {
                                    let before =
                                        Vector::from(world.state(id).unwrap().angular_velocity);
                                    world
                                        .apply_impulse(id, [0.; 3], (axis * size).to_array())
                                        .unwrap();
                                    let after =
                                        Vector::from(world.state(id).unwrap().angular_velocity);
                                    let want = orientation
                                        * (d.recip() * (orientation.inverse() * (axis * size)));
                                    assert!(
                                        (after - before).distance(want) < 2e-10,
                                        "order {order}, size {size}, member {link}, full {full}, shape {shape:?}"
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
#[test]
fn invalid_inertia_still_fails_before_spawning_a_body() {
    let (spec, rot) = input();
    for invalid in [
        [1., 1., 3.1, 0., 0., 0.],
        [1., 0., 1., 0., 0., 0.],
        [-0.2, 0.2, 0.2, 0., 0., 0.],
        [1., 1., 1., 2., 0., 0.],
        [1., 0.3, 0.4, f64::NAN, 0., 0.],
    ] {
        let mut world = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let f = Inertia {
            center: [0.; 3],
            orientation: [0., 0., 0., 1.],
            tensor: invalid,
        };
        assert!(matches!(
            world.spawn_link(spec, rot, Some(f)),
            Err(PhysicsError::InvalidValue)
        ));
        let id = world.spawn(spec).unwrap();
        assert_eq!(id.raw(), 0);
    }
}

#[test]
fn ordinary_locked_tree_integrates_a_known_torque_about_its_aggregate_com() {
    use std::time::Duration;
    let step = Duration::from_nanos(1);
    for axis in [Vector::X, Vector::Y, Vector::Z] {
        let mut world = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let base = Vector::new(0.14, -0.19, 0.05);
        let mut aggregate = Matrix::ZERO;
        let mut ids = vec![];
        for (i, sign) in [-1., 1.].into_iter().enumerate() {
            let q = Rotation::from_scaled_axis(Vector::new(0.35 + sign * 0.18, -0.28, 0.48));
            let axes = Rotation::from_scaled_axis(Vector::new(-0.3, 0.08 + sign * 0.07, 0.21));
            let r = Vector::new(0.11, 0.04, -0.15) * sign;
            let com = Vector::new(0.034, -0.025, 0.07);
            let center = base + r;
            let d = Vector::new(0.041, 0.041, if i == 0 { 0.072 } else { 0.012 });
            let rot = Matrix::from_quat(q * axes);
            let parallel = |e: Vector| e * r.length_squared() - r * r.dot(e);
            aggregate += rot * Matrix::from_diagonal(d) * rot.transpose()
                + Matrix::from_cols(
                    parallel(Vector::X),
                    parallel(Vector::Y),
                    parallel(Vector::Z),
                );
            let id = world
                .spawn_articulation_link(
                    BodySpec {
                        position: (center - q * com).to_array(),
                        velocity: [0.; 3],
                        mass: Some(1.),
                        shape: Shape::Box {
                            half_extents: [0.045; 3],
                        },
                    },
                    RotationSpec {
                        orientation: q.to_array(),
                        angular_velocity: [0.; 3],
                    },
                    Some(Inertia {
                        center: com.to_array(),
                        orientation: axes.to_array(),
                        tensor: [d.x, d.y, d.z, 0., 0., 0.],
                    }),
                )
                .unwrap();
            ids.push(id);
        }
        world
            .add_limited_revolute(
                Some(ids[0]),
                ids[1],
                base.to_array(),
                [1., 0., 0.],
                [0., 0.],
            )
            .unwrap();
        world.enable_exact_revolute_locks(ids[0]).unwrap();
        world
            .add_world_wrench(ids[1], [0.; 3], (axis / step.as_secs_f64()).to_array())
            .unwrap();
        world.step(step).unwrap();
        let expected = aggregate.inverse() * axis;
        let mut momentum = Vector::ZERO;
        for id in ids {
            let s = world.state(id).unwrap();
            momentum += Vector::from_array(s.velocity);
            let w = Vector::from_array(s.angular_velocity);
            assert!(w.distance(expected) < 5e-7, "{axis:?} {w:?} {expected:?}");
        }
        assert!(momentum.length() < 8e-7);
    }
}
