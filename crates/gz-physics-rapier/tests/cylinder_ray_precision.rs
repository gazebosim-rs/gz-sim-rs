// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cylinder query precision uses unchanged physical colliders and strict oracles.
use gz_physics_rapier::{
    BodyId, BodySpec, DynamicBackend, PhysicsError, PosedCylinderBodySpec, RapierBackend,
    RotationSpec, Shape,
};
use std::{f64::consts::FRAC_1_SQRT_2, time::Duration};

fn close(actual: f64, expected: f64) {
    assert!(actual.is_finite() && expected.is_finite());
    assert!((actual - expected).abs() < 1e-11, "{actual} != {expected}");
}

fn hit(backend: &RapierBackend, origin: [f64; 3], direction: [f64; 3], expected: f64) {
    close(
        backend
            .cast_ray(origin, direction, 20., None)
            .unwrap()
            .unwrap(),
        expected,
    );
}

fn horizontal(posed: bool) -> (RapierBackend, BodyId) {
    let mut backend = RapierBackend::new([0.; 3]).unwrap();
    let id = if posed {
        // Independent quarter-turn composition: the collider center is
        // (.75,5.75,4), and the Gazebo cylinder axis becomes -X.
        backend
            .spawn_posed_cylinder_link(
                PosedCylinderBodySpec {
                    radius: 0.5,
                    half_height: 1.5,
                    collision_position: [0.25, -1.75, 0.],
                    collision_orientation: [0.5; 4],
                    position: [1., 4., 4.],
                    velocity: [0.; 3],
                    mass: None,
                },
                RotationSpec {
                    orientation: [0., 0., 1., 0.],
                    ..RotationSpec::default()
                },
                None,
            )
            .unwrap()
    } else {
        backend
            .spawn_link(
                BodySpec {
                    shape: Shape::Cylinder {
                        radius: 0.5,
                        half_height: 1.5,
                    },
                    position: [0.75, 5.75, 4.],
                    velocity: [0.; 3],
                    mass: None,
                },
                RotationSpec {
                    orientation: [0., FRAC_1_SQRT_2, 0., FRAC_1_SQRT_2],
                    ..RotationSpec::default()
                },
                None,
            )
            .unwrap()
    };
    (backend, id)
}

#[test]
fn cylinder_curved_chords_keep_strict_oracles_in_centered_and_composed_frames() {
    for posed in [false, true] {
        let (mut backend, id) = horizontal(posed);
        let before = backend.state(id);
        for _ in 0..2 {
            for (origin, direction, expected) in [
                ([5., 5.75, 4.], [-1., 0., 0.], 2.75),
                ([-4., 5.75, 4.], [1., 0., 0.], 3.25),
                ([0.75, 5.75, 8.], [0., 0., -1.], 3.5),
                ([0.75, 9., 4.], [0., -1., 0.], 2.75),
                // The curved 3-4-5 chord remains mandatory at 1e-11.
                ([0.75, 6.05, 8.], [0., 0., -1.], 3.6),
            ] {
                hit(&backend, origin, direction, expected);
                assert_eq!(backend.cast_ray(origin, direction, 20., Some(id)), Ok(None));
            }
            assert_eq!(backend.state(id), before);
            backend.step(Duration::from_millis(1)).unwrap();
            backend.reset().unwrap();
        }
    }
}

#[test]
fn cylinder_queries_keep_caps_tangency_inside_parallel_and_misses_distinct() {
    let mut backend = RapierBackend::new([0.; 3]).unwrap();
    let id = backend
        .spawn(BodySpec {
            shape: Shape::Cylinder {
                radius: 0.5,
                half_height: 1.5,
            },
            position: [0.; 3],
            velocity: [0.; 3],
            mass: None,
        })
        .unwrap();
    let before = backend.state(id);
    for (origin, direction, expected) in [
        ([0., 0., 3.], [0., 0., -1.], 1.5),
        ([0.4, 0., 3.], [0., 0., -1.], 1.5),
        ([-1.2, 0., 3.], [0.6, 0., -0.8], 1.875),
        ([2., 0.3, 3.], [-0.6, 0., -0.8], 8. / 3.),
        ([2., 0.3, 0.], [-1., 0., 0.], 1.6),
        ([2., 0.5, 0.], [-1., 0., 0.], 2.),
        ([0.5, 0., 3.], [0., 0., -1.], 1.5),
        ([0., 0., 0.], [0., 0., 1.], 0.),
        ([0.5, 0., 0.], [1., 0., 0.], 0.),
        ([0., 0., 1.5], [0., 0., 1.], 0.),
    ] {
        hit(&backend, origin, direction, expected);
    }
    for (origin, direction) in [
        ([0.501, 0., 3.], [0., 0., -1.]),
        ([2., 0.501, 0.], [-1., 0., 0.]),
        ([0., 0., 3.], [0., 0., 1.]),
        ([2., 0., 0.], [1., 0., 0.]),
        // This intersects the query capsule's spherical end, but is outside
        // the cylinder's finite slab and therefore must not become a hit.
        ([2., 0., 1.75], [-1., 0., 0.]),
    ] {
        assert_eq!(backend.cast_ray(origin, direction, 20., None), Ok(None));
    }
    assert_eq!(backend.state(id), before);
}

#[test]
fn cylinder_segment_bounds_nearest_exclusion_and_reset_preserve_owned_geometry() {
    let (mut backend, cylinder) = horizontal(true);
    let other = backend
        .spawn(BodySpec {
            shape: Shape::Sphere { radius: 0.25 },
            position: [0.75, 6.05, -2.],
            velocity: [0.; 3],
            mass: None,
        })
        .unwrap();
    let origin = [0.75, 6.05, 8.];
    let direction = [0., 0., -1.];
    let boundary = backend
        .cast_ray(origin, direction, 20., None)
        .unwrap()
        .unwrap();
    close(boundary, 3.6);
    assert_eq!(backend.cast_ray(origin, direction, 3., None), Ok(None));
    assert_eq!(
        backend.cast_ray(origin, direction, boundary - 1e-6, None),
        Ok(None)
    );
    close(
        backend
            .cast_ray(origin, direction, boundary, None)
            .unwrap()
            .unwrap(),
        boundary,
    );
    close(
        backend
            .cast_ray(origin, direction, boundary + 1e-6, None)
            .unwrap()
            .unwrap(),
        boundary,
    );
    close(
        backend
            .cast_ray(origin, direction, 20., Some(other))
            .unwrap()
            .unwrap(),
        3.6,
    );
    close(
        backend
            .cast_ray(origin, direction, 20., Some(cylinder))
            .unwrap()
            .unwrap(),
        9.75,
    );
    let initial = backend.initial_state();
    let cylinder_state = backend.state(cylinder);
    assert!(backend.remove(cylinder));
    close(
        backend
            .cast_ray(origin, direction, 20., None)
            .unwrap()
            .unwrap(),
        9.75,
    );
    let runtime = backend
        .spawn(BodySpec {
            shape: Shape::Sphere { radius: 0.25 },
            position: [20., 0., 0.],
            velocity: [0.; 3],
            mass: None,
        })
        .unwrap();
    let prepared = backend.prepare_initial(&initial).unwrap();
    backend.apply_initial(prepared).unwrap();
    assert_eq!(backend.state(runtime), None);
    assert_eq!(backend.state(cylinder), cylinder_state);
    hit(&backend, origin, direction, 3.6);
    backend.reset().unwrap();
    hit(&backend, origin, direction, 3.6);
    assert!(backend.remove(other));
    assert!(backend.remove(cylinder));
    assert_eq!(backend.cast_ray(origin, direction, 20., None), Ok(None));
    let next = backend
        .spawn(BodySpec {
            shape: Shape::Sphere { radius: 0.25 },
            position: [20., 0., 0.],
            velocity: [0.; 3],
            mass: None,
        })
        .unwrap();
    assert!(next > runtime);
}

#[test]
fn cylinder_queries_keep_binary_distant_chords_large_admitted_shapes_and_numeric_errors() {
    let mut backend = RapierBackend::new([0.; 3]).unwrap();
    let id = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Cylinder {
                    radius: 0.625,
                    half_height: 1.5,
                },
                position: [0.; 3],
                velocity: [0.; 3],
                mass: None,
            },
            RotationSpec {
                orientation: [0., FRAC_1_SQRT_2, 0., FRAC_1_SQRT_2],
                ..RotationSpec::default()
            },
            None,
        )
        .unwrap();
    let before = backend.state(id);
    // Binary-scaled 3-4-5 cross-section: .625²-.375²=.5². Both
    // the chord and distant time are exactly representable binary values.
    for z in [16., 1024., 4096.] {
        close(
            backend
                .cast_ray([0., 0.375, z], [0., 0., -1.], z + 1., None)
                .unwrap()
                .unwrap(),
            z - 0.5,
        );
    }
    let mut long = RapierBackend::new([0.; 3]).unwrap();
    long.spawn(BodySpec {
        shape: Shape::Cylinder {
            radius: 0.625,
            half_height: 4096.,
        },
        position: [0.; 3],
        velocity: [0.; 3],
        mass: None,
    })
    .unwrap();
    // The same binary .5 chord is reached at (1024-.5)/(4/5).
    // Z remains inside the long axis interval, so this is a curved side hit.
    close(
        long.cast_ray([1024., 0.375, 1024.], [-0.8, 0., -0.6], 2048., None)
            .unwrap()
            .unwrap(),
        1279.375,
    );
    for (origin, direction, maximum) in [
        ([f64::NAN, 0., 0.], [1., 0., 0.], 20.),
        ([0.; 3], [f64::INFINITY, 0., 0.], 20.),
        ([0.; 3], [2., 0., 0.], 20.),
        ([0.; 3], [1., 0., 0.], 0.),
        ([0.; 3], [1., 0., 0.], f64::INFINITY),
        ([f64::MAX, 0., 0.], [1., 0., 0.], f64::MAX),
    ] {
        assert_eq!(
            backend.cast_ray(origin, direction, maximum, None),
            Err(PhysicsError::InvalidValue)
        );
    }
    assert_eq!(backend.state(id), before);
    let scale = 1e60;
    let mut large = RapierBackend::new([0.; 3]).unwrap();
    let body = large
        .spawn(BodySpec {
            shape: Shape::Cylinder {
                radius: scale,
                half_height: scale,
            },
            position: [0.; 3],
            velocity: [0.; 3],
            mass: None,
        })
        .unwrap();
    let initial = large.state(body);
    let distance = large
        .cast_ray(
            [3. * scale, 0.6 * scale, 0.],
            [-1., 0., 0.],
            4. * scale,
            None,
        )
        .unwrap()
        .unwrap();
    close(distance / scale, 2.2);
    assert_eq!(large.state(body), initial);
    // These shapes retain finite admitted mass properties. Their extremely
    // thin axis/projection cannot support the analytic query arithmetic.
    for (radius, half_height) in [(1.0, 1e-200), (0.01, 1e-153)] {
        let mut thin = RapierBackend::new([0.; 3]).unwrap();
        let id = thin
            .spawn(BodySpec {
                shape: Shape::Cylinder {
                    radius,
                    half_height,
                },
                position: [0.; 3],
                velocity: [0.; 3],
                mass: None,
            })
            .unwrap();
        let before = thin.state(id);
        assert_eq!(
            thin.cast_ray([2. * radius, 0., 0.], [-1., 0., 0.], 4. * radius, None),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!(
            thin.cast_ray([2. * radius, 0., 0.], [-1., 0., 0.], 4. * radius, Some(id)),
            Ok(None)
        );
        assert_eq!(thin.state(id), before);
    }
}
