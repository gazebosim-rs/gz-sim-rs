// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Admission bounds against the packaged DART, which aborts through assertions
//! (NaN dynamics, an asymmetric or singular contact LCP), and ODE, which aborts
//! beyond its collision grid. Just-inside bodies rest on the floor or on each
//! other, rotated and anisotropic, and step the real engine; just-outside values
//! are refused. tests/fuzz.rs searches combinations of these extremes.

use gz_physics::{BodyId, BodySpec, DynamicBackend, Inertia, PhysicsError, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use std::time::Duration;

const LIMIT: f64 = 1e60;
const GRID: f64 = 5e8;
/// The conditioning limit K on 1/m, rho^2/lambda_min and a hinge's rho^2/I.
const K: f64 = 1e5;
/// Just outside a bound, in relative terms.
const OUT: f64 = 1.000001;
/// Just inside a bound, absorbing the rounding of the inertia rotation.
const IN: f64 = 1.000000001;

#[derive(Clone, Copy)]
struct Body {
    shape: Shape,
    mass: f64,
    /// Principal moments in the inertial frame, which `tilt` rotates.
    moments: [f64; 3],
    center: [f64; 3],
    position: [f64; 3],
    velocity: [f64; 3],
    angular: [f64; 3],
    tilt: f64,
}

const SPHERE: Shape = Shape::Sphere { radius: 0.5 };

const UNIT: Body = Body {
    shape: SPHERE,
    mass: 1.0,
    moments: [1.0; 3],
    center: [0.0; 3],
    position: [0.0, 0.0, 0.5],
    velocity: [0.0; 3],
    angular: [0.0; 3],
    tilt: 0.0,
};

fn quaternion(axis: [f64; 3], angle: f64) -> [f64; 4] {
    let length = axis.iter().map(|v| v * v).sum::<f64>().sqrt();
    let s = (angle / 2.0).sin();
    [
        axis[0] / length * s,
        axis[1] / length * s,
        axis[2] / length * s,
        (angle / 2.0).cos(),
    ]
}

fn spawn(backend: &mut DartBackend, body: Body) -> Result<BodyId, PhysicsError> {
    let [a, b, c] = body.moments;
    backend.spawn_link(
        BodySpec {
            shape: body.shape,
            position: body.position,
            velocity: body.velocity,
            mass: Some(body.mass),
        },
        RotationSpec {
            orientation: quaternion([1.0, 0.4, 0.2], body.tilt),
            angular_velocity: body.angular,
        },
        Some(Inertia {
            center: body.center,
            orientation: quaternion([0.3, 1.0, 0.5], 2.0 * body.tilt),
            tensor: [a, b, c, 0.0, 0.0, 0.0],
        }),
    )
}

fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = q;
    let u = [x, y, z];
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let t = cross(u, v).map(|c| 2.0 * c);
    let ut = cross(u, t);
    [0, 1, 2].map(|i| v[i] + w * t[i] + ut[i])
}

/// Half-width of a centered Z-axis shape along the unit vector `u`.
fn support(shape: Shape, u: [f64; 3]) -> f64 {
    let along = u[2].abs();
    match shape {
        Shape::Sphere { radius } => radius,
        Shape::Box { half_extents } => (0..3).map(|i| u[i].abs() * half_extents[i]).sum(),
        Shape::Cylinder {
            radius,
            half_height,
        } => along * half_height + radius * (1.0 - along * along).max(0.0).sqrt(),
        Shape::Capsule {
            radius,
            half_length,
        } => along * half_length + radius,
    }
}

/// An independent evaluation of the rule's sum_i (sum_{j!=i} e_j^2)/lambda_i
/// for `body`, whose principal axes are its inertial frame (rotated by 2*tilt).
fn spread(body: Body) -> f64 {
    let frame = quaternion([0.3, 1.0, 0.5], 2.0 * body.tilt);
    let e = [0, 1, 2].map(|j| {
        let mut axis = [0.0; 3];
        axis[j] = 1.0;
        let u = rotate(frame, axis);
        let offset: f64 = (0..3).map(|i| body.center[i] * u[i]).sum();
        (offset.abs() + support(body.shape, u)).powi(2)
    });
    let total: f64 = e.iter().sum();
    (0..3).map(|i| (total - e[i]) / body.moments[i]).sum()
}

/// A rotated, 20x anisotropic, offset body just inside the conditioning limit:
/// 1/m and the moment term each contribute half of K. Distinct moments make the
/// principal axes unique, so this evaluation and the backend's agree.
fn conditioned(shape: Shape, position: [f64; 3], tilt: f64) -> Body {
    let base = Body {
        shape,
        mass: 2.0 * IN / K,
        moments: [1.0, 20.0, 20.5],
        center: [0.05, -0.03, 0.02],
        position,
        tilt,
        ..UNIT
    };
    let scale = 2.0 * IN * spread(base) / K;
    Body {
        moments: base.moments.map(|m| m * scale),
        ..base
    }
}

/// A world with a static floor, so ODE collides on every step.
fn world(gravity: [f64; 3]) -> DartBackend {
    let mut backend = DartBackend::new(gravity).unwrap();
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
    backend
}

/// Steps `steps` times. The post-step guard may refuse a step, restoring the
/// state; the process must survive and published state must stay finite.
fn run(backend: &mut DartBackend, ids: &[BodyId], steps: usize) -> usize {
    let mut refused = 0;
    for _ in 0..steps {
        let before: Vec<_> = ids.iter().map(|&id| backend.state(id)).collect();
        match backend.step(Duration::from_millis(1)) {
            Ok(()) => {}
            Err(PhysicsError::Native { .. }) => {
                refused += 1;
                let after: Vec<_> = ids.iter().map(|&id| backend.state(id)).collect();
                assert_eq!(after, before);
            }
            Err(other) => panic!("unexpected {other:?}"),
        }
        for &id in ids {
            let state = backend.state(id).unwrap();
            assert!(state.position.into_iter().all(f64::is_finite));
            assert!(state.velocity.into_iter().all(f64::is_finite));
        }
    }
    refused
}

fn admitted(body: Body) -> usize {
    let mut backend = world([0.0, 0.0, -9.81]);
    let id = spawn(&mut backend, body).unwrap();
    run(&mut backend, &[id], 200)
}

fn refused(body: Body) {
    let mut backend = world([0.0, 0.0, -9.81]);
    let result = spawn(&mut backend, body);
    // Step regardless, so a missing guard reaches the engine with the value.
    if let Ok(id) = result {
        run(&mut backend, &[id], 200);
    }
    assert_eq!(result, Err(PhysicsError::InvalidValue));
}

#[test]
fn bodies_resting_on_the_floor_at_the_conditioning_limit_step() {
    let cube = Shape::Box {
        half_extents: [0.2; 3],
    };
    let capsule = Shape::Capsule {
        radius: 0.2,
        half_length: 0.5,
    };
    for (shape, height) in [(SPHERE, 0.5), (cube, 0.35), (capsule, 0.2)] {
        for tilt in [0.0, 0.8, 2.1] {
            let body = conditioned(shape, [0.0, 0.0, height], tilt);
            assert_eq!(admitted(body), 0);
            // Lighter, more anisotropic or more offset by one part in a million.
            refused(Body {
                mass: body.mass / OUT,
                ..body
            });
            let [lambda, b, c] = body.moments;
            refused(Body {
                moments: [lambda / OUT, b, c],
                ..body
            });
            refused(Body {
                center: body.center.map(|v| v * 1.01),
                ..body
            });
        }
    }
}

#[test]
fn stacks_and_piles_at_the_conditioning_limit_step() {
    // Three rotated, anisotropic, offset spheres stacked on the floor.
    let mut backend = world([0.0, 0.0, -9.81]);
    let ids: Vec<_> = (0..3)
        .map(|i| {
            let level = i as f64;
            let body = conditioned(SPHERE, [0.01 * level, 0.0, 0.5 + level], 0.4 + level);
            spawn(&mut backend, body).unwrap()
        })
        .collect();
    assert_eq!(run(&mut backend, &ids, 500), 0);
    // Six mixed shapes dropped onto each other: at K = 5e7 this pile aborted
    // the packaged DART with a zero LDLT pivot in its Dantzig LCP.
    let cube = Shape::Box {
        half_extents: [0.2; 3],
    };
    let capsule = Shape::Capsule {
        radius: 0.2,
        half_length: 0.5,
    };
    let mut backend = world([0.0, 0.0, -9.81]);
    let ids: Vec<_> = (0..6)
        .map(|i| {
            let shape = [SPHERE, cube, capsule][i % 3];
            let position = [
                0.35 * (i % 3) as f64 - 0.35,
                0.35 * (i / 3) as f64 - 0.35,
                0.6,
            ];
            spawn(
                &mut backend,
                conditioned(shape, position, 0.13 * i as f64 + 0.1),
            )
            .unwrap()
        })
        .collect();
    assert_eq!(run(&mut backend, &ids, 1000), 0);
    // A heavy body resting on a light one.
    let mut backend = world([0.0, 0.0, -9.81]);
    let light = spawn(&mut backend, conditioned(SPHERE, [0.0, 0.0, 0.5], 0.3)).unwrap();
    let heavy = spawn(
        &mut backend,
        Body {
            mass: 1e6,
            moments: [1e6; 3],
            position: [0.02, 0.0, 1.5],
            ..UNIT
        },
    )
    .unwrap();
    assert_eq!(run(&mut backend, &[light, heavy], 500), 0);
}

#[test]
fn rotated_extreme_anisotropy_is_capped_before_rotation() {
    let frame = {
        let q: [f64; 4] = [0.344, 0.328, -0.062, 0.878];
        let length = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        q.map(|v| v / length)
    };
    let mut backend = world([0.0, 0.0, -9.81]);
    let spawn_rotated = |backend: &mut DartBackend, mass: f64, tensor: [f64; 3]| {
        backend.spawn_link(
            BodySpec {
                shape: SPHERE,
                position: [0.0, 0.0, 0.5],
                velocity: [0.0; 3],
                mass: Some(mass),
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.6, 0.5, 0.6],
                orientation: frame,
                tensor: [tensor[0], tensor[1], tensor[2], 0.0, 0.0, 0.0],
            }),
        )
    };
    // Rotated, a 1e60:1 tensor's computed smallest moment is rounding noise, and
    // this body aborted DART resting on the floor; 1e10 kg with 1e40 moments
    // flew 17 km in 10 ms.
    for (mass, tensor) in [
        (1e30, [9.99999e59, 1.0, 9.99999e59]),
        (1e10, [1e40, 1.0, 1e40]),
        (10.0, [1e8 * OUT, 1.0, 1e8 * OUT]),
    ] {
        assert_eq!(
            spawn_rotated(&mut backend, mass, tensor),
            Err(PhysicsError::InvalidValue)
        );
    }
    let id = spawn_rotated(&mut backend, 1e3, [1e8 / OUT, 1.0, 1e8 / OUT]).unwrap();
    assert_eq!(run(&mut backend, &[id], 300), 0);
    let state = backend.state(id).unwrap();
    assert!(state.position[2].abs() < 1.0, "{state:?}");
}

#[test]
fn slender_everyday_bodies_are_admitted_and_rest_in_piles() {
    for (shape, mass) in [
        // A pencil, a cable link and a rod, with shape-derived inertia.
        (
            Shape::Cylinder {
                radius: 0.0035,
                half_height: 0.09,
            },
            0.006,
        ),
        (
            Shape::Capsule {
                radius: 0.002,
                half_length: 0.05,
            },
            0.002,
        ),
        (
            Shape::Capsule {
                radius: 0.005,
                half_length: 0.5,
            },
            0.01,
        ),
    ] {
        let mut backend = world([0.0, 0.0, -9.81]);
        let ids: Vec<_> = (0..5)
            .map(|j| {
                let tilt = 0.3 + 0.7 * j as f64;
                backend
                    .spawn_link(
                        BodySpec {
                            shape,
                            position: [0.01 * j as f64, 0.0, 0.05 + 0.1 * j as f64],
                            velocity: [0.0; 3],
                            mass: Some(mass),
                        },
                        RotationSpec {
                            orientation: quaternion([tilt.cos(), tilt.sin(), 0.3], 1.4),
                            angular_velocity: [j as f64, 0.0, 3.0],
                        },
                        None,
                    )
                    .unwrap()
            })
            .collect();
        assert_eq!(run(&mut backend, &ids, 2000), 0, "{shape:?}");
    }
}

#[test]
fn hinged_bodies_in_contact_respect_the_hinge_conditioning_limit() {
    let mut backend = world([0.0, 0.0, -9.81]);
    let body = conditioned(SPHERE, [0.0, 0.0, 0.5], 0.2);
    let hinged = spawn(&mut backend, body).unwrap();
    backend
        .add_world_revolute(hinged, [0.0, 0.0, 1.5], [0.0, 1.0, 0.0])
        .unwrap();
    let leaning = spawn(&mut backend, conditioned(SPHERE, [0.99, 0.0, 0.5], 0.7)).unwrap();
    assert_eq!(run(&mut backend, &[hinged, leaning], 500), 0);
    // A heavy body with a weak axis is free-admissible, but hinged through its
    // COM about that axis m*rho_a^2/I_axis = 1e3 * 0.25 / 1e-4 exceeds K; a
    // hinge 1 m away adds m*d^2 to I_axis and is admitted.
    let mut backend = world([0.0, 0.0, -9.81]);
    let heavy = Body {
        mass: 1e3,
        moments: [1e-4, 1e-4 * 1.5, 1e-4 * 1.5],
        position: [0.0, 0.0, 2.0],
        ..UNIT
    };
    let id = spawn(&mut backend, heavy).unwrap();
    assert_eq!(
        backend.add_world_revolute(id, [0.0, 0.0, 2.0], [1.0, 0.0, 0.0]),
        Err(PhysicsError::InvalidJoint)
    );
    backend
        .add_world_revolute(id, [0.0, 0.0, 3.0], [1.0, 0.0, 0.0])
        .unwrap();
    assert_eq!(run(&mut backend, &[id], 200), 0);
}

#[test]
fn hinge_rates_beyond_the_velocity_bound_are_refused() {
    // rate = (a.w + (a x r).v) / (1 + |a x r|^2) peaks at ~1.207 times the
    // admitted magnitude when |a x r| = sqrt(2) - 1.
    let t = 2f64.sqrt() - 1.0;
    let swinging = Body {
        position: [0.0, 0.0, 3.0],
        velocity: [0.0, 0.0, -LIMIT],
        angular: [0.0, LIMIT, 0.0],
        ..UNIT
    };
    let mut backend = world([0.0, 0.0, -9.81]);
    let id = spawn(&mut backend, swinging).unwrap();
    // a = Y, r = COM - anchor = (t, 0, 0), a x r = (0, 0, -t).
    assert_eq!(
        backend.add_world_revolute(id, [-t, 0.0, 3.0], [0.0, 1.0, 0.0]),
        Err(PhysicsError::InvalidJoint)
    );
    // Slower, the same hinge is admitted, but commanding that twist is refused.
    backend
        .set_world_velocity(id, [0.0, 0.0, -LIMIT / 2.0], [0.0, LIMIT / 2.0, 0.0])
        .unwrap();
    let joint = backend
        .add_world_revolute(id, [-t, 0.0, 3.0], [0.0, 1.0, 0.0])
        .unwrap();
    let rate = backend.joint_state(joint).unwrap().velocity;
    assert_eq!(
        backend.set_world_velocity(id, [0.0, 0.0, -LIMIT], [0.0, LIMIT, 0.0]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(backend.joint_state(joint).unwrap().velocity, rate);
    backend
        .set_world_velocity(id, [0.0; 3], [0.0, LIMIT, 0.0])
        .unwrap();
    run(&mut backend, &[id], 5);
}

#[test]
fn free_body_origin_velocities_stay_within_1e60() {
    // DART stores a free body's origin velocity: v_com - w x (R c).
    let mut backend = world([0.0, 0.0, -9.81]);
    let offset = Body {
        center: [1.0, 0.0, 0.0],
        position: [0.0, 0.0, 3.0],
        ..UNIT
    };
    let id = spawn(&mut backend, offset).unwrap();
    backend
        .set_world_velocity(id, [0.0; 3], [0.0, 0.0, LIMIT])
        .unwrap();
    let before = backend.state(id).unwrap();
    assert_eq!(
        backend.set_world_velocity(id, [0.0, -LIMIT, 0.0], [0.0, 0.0, LIMIT]),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(backend.state(id).unwrap(), before);
    run(&mut backend, &[id], 5);
    refused(Body {
        velocity: [LIMIT / 2.0, 0.0, 0.0],
        angular: [0.0, 0.0, LIMIT / 2.0 * OUT],
        ..offset
    });
}

#[test]
fn mass_moment_and_magnitude_upper_bounds_hold() {
    for body in [
        Body {
            mass: LIMIT,
            moments: [LIMIT; 3],
            center: [0.1, 0.0, 0.0],
            angular: [0.0, 0.0, 1e3],
            ..UNIT
        },
        Body {
            velocity: [0.0, LIMIT, 0.0],
            ..UNIT
        },
        Body {
            angular: [LIMIT, 0.0, 0.0],
            ..UNIT
        },
    ] {
        admitted(body);
    }
    for body in [
        Body {
            mass: LIMIT * OUT,
            ..UNIT
        },
        Body {
            moments: [LIMIT * OUT; 3],
            ..UNIT
        },
        Body {
            velocity: [0.0, LIMIT * OUT, 0.0],
            ..UNIT
        },
        Body {
            angular: [LIMIT * OUT, 0.0, 0.0],
            ..UNIT
        },
        Body {
            center: [LIMIT * OUT, 0.0, 0.0],
            moments: [LIMIT; 3],
            ..UNIT
        },
    ] {
        refused(body);
    }
}

#[test]
fn shape_dimensions_and_collision_bounds_stay_inside_odes_grid() {
    // The smallest admitted sphere (at the origin, where 1e-60 m is representable).
    admitted(Body {
        shape: Shape::Sphere { radius: 1e-60 },
        position: [0.0; 3],
        ..UNIT
    });
    // The largest sphere whose bounds stay on the grid, conditioned for its size.
    let big = Shape::Sphere { radius: GRID / 2.0 };
    admitted(Body {
        shape: big,
        moments: [1e17; 3],
        position: [0.0, 0.0, GRID / 2.0],
        ..UNIT
    });
    // ODE grids small bodies at 1/8 m; this one reaches exactly 5e8 m.
    admitted(Body {
        shape: Shape::Sphere { radius: 1e-3 },
        position: [GRID - 1e-3, 0.0, 0.0],
        ..UNIT
    });
    for (shape, position) in [
        (
            Shape::Sphere {
                radius: 1e-60 / OUT,
            },
            [0.0; 3],
        ),
        (Shape::Sphere { radius: GRID * OUT }, [0.0; 3]),
        (Shape::Sphere { radius: 1e-3 }, [GRID, 0.0, 0.0]),
    ] {
        refused(Body {
            shape,
            position,
            moments: [1e18; 3],
            ..UNIT
        });
    }
}

#[test]
fn gravity_magnitude_stays_within_1e60() {
    for gravity in [[0.0, 0.0, -LIMIT], [LIMIT / 2.0, 0.0, -LIMIT / 2.0]] {
        for mass in [2.0 / K, LIMIT] {
            let mut backend = world(gravity);
            let body = Body {
                mass,
                moments: [mass.max(1.0); 3],
                center: [0.1, 0.1, 0.0],
                ..UNIT
            };
            let id = spawn(&mut backend, body).unwrap();
            run(&mut backend, &[id], 5);
        }
    }
    for gravity in [[0.0, 0.0, -LIMIT * OUT], [0.0, 0.0, -1e308], [f64::NAN; 3]] {
        assert!(matches!(
            DartBackend::new(gravity),
            Err(PhysicsError::InvalidValue)
        ));
    }
}

#[test]
fn accumulated_force_and_torque_stay_within_1e60() {
    let light = conditioned(SPHERE, [0.0, 0.0, 0.5], 0.4);
    let mut backend = world([0.0, 0.0, -9.81]);
    let id = spawn(&mut backend, light).unwrap();
    backend
        .add_world_wrench(id, [LIMIT / 2.0, 0.0, 0.0], [0.0, 0.0, LIMIT / 2.0])
        .unwrap();
    backend
        .add_world_wrench(id, [LIMIT / 2.0, 0.0, 0.0], [0.0, 0.0, LIMIT / 2.0])
        .unwrap();
    for (force, torque) in [
        ([LIMIT * (OUT - 1.0), 0.0, 0.0], [0.0; 3]),
        ([0.0; 3], [0.0, 0.0, LIMIT * (OUT - 1.0)]),
    ] {
        assert_eq!(
            backend.add_world_wrench(id, force, torque),
            Err(PhysicsError::InvalidValue)
        );
    }
    // On the floor at 1e66 m/s^2: the velocity bound refuses the step.
    assert!(run(&mut backend, &[id], 5) > 0);
}

#[test]
fn impulse_results_stay_within_1e60() {
    let mut backend = world([0.0, 0.0, -9.81]);
    let id = spawn(&mut backend, UNIT).unwrap();
    backend
        .apply_impulse(id, [0.0; 3], [LIMIT, 0.0, 0.0])
        .unwrap();
    let before = backend.state(id).unwrap();
    for (linear, angular) in [
        ([0.0; 3], [LIMIT * (OUT - 1.0), 0.0, 0.0]),
        ([0.0; 3], [1e200, 0.0, 0.0]),
        ([0.0, LIMIT * OUT, 0.0], [0.0; 3]),
    ] {
        assert_eq!(
            backend.apply_impulse(id, linear, angular),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!(backend.state(id).unwrap(), before);
    }
    run(&mut backend, &[id], 5);
}

#[test]
fn hinge_anchors_stay_on_the_grid() {
    let mut backend = world([0.0, 0.0, -9.81]);
    let id = spawn(
        &mut backend,
        Body {
            position: [0.0, 0.0, 3.0],
            ..UNIT
        },
    )
    .unwrap();
    assert_eq!(
        backend.add_world_revolute(id, [GRID * OUT, 0.0, 0.0], [0.0, 1.0, 0.0]),
        Err(PhysicsError::InvalidJoint)
    );
    // The farthest admitted anchor: a 5e8 m lever arm turning at 1e60 rad/s.
    let joint = backend
        .add_world_revolute(id, [GRID, 0.0, 0.0], [0.0, 1.0, 0.0])
        .unwrap();
    backend
        .set_world_velocity(id, [0.0; 3], [0.0, LIMIT, 0.0])
        .unwrap();
    assert!(backend.joint_state(joint).unwrap().velocity.abs() <= LIMIT);
    run(&mut backend, &[id], 5);
}

#[test]
fn cases_that_aborted_the_packaged_dart_are_refused_before_the_engine() {
    let cube = Shape::Box {
        half_extents: [0.2; 3],
    };
    // Each of these aborted DART, mostly resting on the floor (isSymmetric).
    for body in [
        Body {
            shape: cube,
            mass: 1e-10,
            moments: [0.4 * 1e-10 * 0.04 * 2.0 / 3.0; 3],
            position: [0.0, 0.0, 0.2],
            ..UNIT
        },
        Body {
            moments: [1e-12, 1.0, 1.0],
            tilt: 0.9,
            ..UNIT
        },
        Body {
            mass: 1e-10,
            moments: [1e-10; 3],
            tilt: 0.9,
            ..UNIT
        },
        Body {
            center: [1e5, 0.0, 0.0],
            ..UNIT
        },
        Body {
            mass: 1e-3,
            moments: [1e-3, 1.0, 1.0],
            center: [8e59, 5e59, 2e59],
            position: [0.0, 0.0, 2.0],
            tilt: 1.3,
            ..UNIT
        },
        Body {
            mass: 1e-300,
            ..UNIT
        },
    ] {
        refused(body);
    }
    let mut backend = world([0.0, 0.0, -9.81]);
    let id = spawn(&mut backend, UNIT).unwrap();
    for (force, torque) in [([1e300, 0.0, 0.0], [0.0; 3]), ([0.0; 3], [0.0, 0.0, 1e308])] {
        assert_eq!(
            backend.add_world_wrench(id, force, torque),
            Err(PhysicsError::InvalidValue)
        );
    }
    assert!(matches!(
        DartBackend::new([0.0, 0.0, -1e308]),
        Err(PhysicsError::InvalidValue)
    ));
    run(&mut backend, &[id], 5);
}

#[test]
fn steps_leaving_the_collision_grid_are_refused_and_restored() {
    let mut backend = world([0.0; 3]);
    let id = spawn(
        &mut backend,
        Body {
            shape: Shape::Sphere { radius: 1e-3 },
            position: [GRID - 1e3, 0.0, 0.0],
            velocity: [4e5, 0.0, 0.0],
            ..UNIT
        },
    )
    .unwrap();
    // 400 m per 1 ms step: the third step would reach past 5e8 m.
    backend.step(Duration::from_millis(1)).unwrap();
    backend.step(Duration::from_millis(1)).unwrap();
    let before = backend.state(id).unwrap();
    assert!(matches!(
        backend.step(Duration::from_millis(1)),
        Err(PhysicsError::Native { .. })
    ));
    assert_eq!(backend.state(id).unwrap(), before);
    // Turning back is then admitted from the restored state.
    backend
        .set_world_velocity(id, [-4e5, 0.0, 0.0], [0.0; 3])
        .unwrap();
    backend.step(Duration::from_millis(1)).unwrap();
    assert!(backend.state(id).unwrap().position[0] < before.position[0]);
}

#[test]
fn contacts_far_from_a_light_body_are_refused_at_spawn_and_step() {
    let mut backend = world([0.0, 0.0, -9.81]);
    let big = Shape::Sphere { radius: 50.0 };
    let boulder = spawn(
        &mut backend,
        Body {
            shape: big,
            moments: [1e3; 3],
            position: [0.0, 0.0, 50.0],
            ..UNIT
        },
    )
    .unwrap();
    let before = backend.state(boulder).unwrap();
    // A 1 cm pebble spawned deep inside the boulder meets a contact point
    // metres from its own shape: |r|^2/lambda would reach ~1e8.
    let pebble = Body {
        shape: Shape::Sphere { radius: 0.01 },
        moments: [4e-5; 3],
        ..UNIT
    };
    assert_eq!(
        spawn(
            &mut backend,
            Body {
                position: [0.0, 0.0, 40.0],
                ..pebble
            }
        ),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(backend.state(boulder).unwrap(), before);
    // Beside the boulder it is admitted. Driven 10 m into it in one step, the
    // next step meets that deep contact before solving it and is refused.
    let id = spawn(
        &mut backend,
        Body {
            position: [0.0, 51.0, 50.0],
            ..pebble
        },
    )
    .unwrap();
    backend
        .set_world_velocity(id, [0.0, -1e3, 0.0], [0.0; 3])
        .unwrap();
    backend.step(Duration::from_millis(10)).unwrap();
    let before = backend.state(id).unwrap();
    assert!(before.position[1] < 45.0, "{before:?}");
    let error = backend.step(Duration::from_millis(10)).unwrap_err();
    assert!(error.to_string().contains("contact"), "{error}");
    assert_eq!(backend.state(id).unwrap(), before);
    run(&mut backend, &[boulder], 5);
}

#[test]
fn velocities_are_checked_before_the_contact_solve() {
    // Reduced from a reviewer's search seed that aborted DART's LCP
    // (isSymmetric) in an optimized build: a 1e-60 m, 1e-60 kg m^2 sphere
    // touching the floor and spinning fast, whose torque integrates to ~1e90
    // rad/s before the contact solve. The solve is skipped and the step refused.
    let tiny = 1.000001e-60;
    for spin in [4e40, 1e59] {
        let mut backend = world([0.0; 3]);
        let id = backend
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: tiny },
                    position: [0.0, 0.0, 0.9 * tiny],
                    velocity: [0.0; 3],
                    mass: Some(1e-3),
                },
                RotationSpec::default(),
                Some(Inertia {
                    center: [0.0; 3],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                    tensor: [tiny, tiny, tiny, 0.0, 0.0, 0.0],
                }),
            )
            .unwrap();
        backend
            .set_world_velocity(id, [0.0; 3], [-0.6 * spin, -0.1 * spin, -0.8 * spin])
            .unwrap();
        backend
            .add_world_wrench(
                id,
                [0.0; 3],
                [
                    6.843159977996546e29,
                    -6.216416891619866e29,
                    -3.8114724904070225e29,
                ],
            )
            .unwrap();
        let before = backend.state(id).unwrap();
        let error = backend.step(Duration::from_secs(1)).unwrap_err();
        assert!(error.to_string().contains("velocity"), "{error}");
        assert_eq!(backend.state(id).unwrap(), before);
    }
}

#[test]
fn a_contact_at_a_hinge_anchor_is_accepted() {
    // The floor contact point is the anchor: the hinge meets it with zero
    // inverse effective mass, which is valid, not a violation.
    let mut backend = world([0.0, 0.0, -9.81]);
    let id = spawn(&mut backend, UNIT).unwrap();
    backend
        .add_world_revolute(id, [0.0; 3], [0.0, 1.0, 0.0])
        .unwrap();
    assert_eq!(run(&mut backend, &[id], 1000), 0);
}
