// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{
    BodyId, BodySpec, DynamicBackend, MeshBodySpec, PhysicsError, RapierBackend, RotationSpec,
    Shape,
};
use std::time::Duration;

const X: [f64; 3] = [1.0, 0.0, 0.0];
const Y: [f64; 3] = [0.0, 1.0, 0.0];
const Z: [f64; 3] = [0.0, 0.0, 1.0];
const IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

fn spec(shape: Shape, position: [f64; 3]) -> BodySpec {
    BodySpec {
        shape,
        position,
        velocity: [0.0; 3],
        mass: None,
    }
}

fn close(actual: Option<f64>, expected: f64) {
    let actual = actual.expect("expected a collision along the ray");
    assert!(actual.is_finite());
    assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
}

#[test]
fn all_admitted_primitives_use_world_frames_and_solid_inclusive_distances() {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    for (shape, orientation, origin, direction, expected) in [
        (Shape::Sphere { radius: 1.0 }, IDENTITY, [0.0; 3], X, 4.0),
        (
            Shape::Box {
                half_extents: [1.0, 0.5, 0.25],
            },
            [0.0, 0.0, s, s],
            [0.0; 3],
            X,
            4.5,
        ),
        (
            Shape::Cylinder {
                radius: 0.5,
                half_height: 2.0,
            },
            [0.0, s, 0.0, s],
            [0.0; 3],
            X,
            3.0,
        ),
        (
            Shape::Capsule {
                radius: 0.5,
                half_length: 1.0,
            },
            IDENTITY,
            [5.0, 0.0, -5.0],
            Z,
            3.5,
        ),
    ] {
        let mut backend = RapierBackend::new([0.0; 3]).unwrap();
        let body = backend
            .spawn_link(
                spec(shape, [5.0, 0.0, 0.0]),
                RotationSpec {
                    orientation,
                    ..RotationSpec::default()
                },
                None,
            )
            .unwrap();
        let before = backend.state(body);
        // No preparatory physics step or broad-phase update is performed.
        let first = backend.cast_ray(origin, direction, 20.0, None).unwrap();
        close(first, expected);
        let boundary = first.unwrap();
        close(
            backend.cast_ray(origin, direction, boundary, None).unwrap(),
            boundary,
        );
        assert_eq!(
            backend.cast_ray(origin, direction, expected - 0.01, None),
            Ok(None)
        );
        assert_eq!(
            backend.cast_ray([5.0, 0.0, 0.0], X, 10.0, None),
            Ok(Some(0.0))
        );
        assert_eq!(backend.cast_ray([0.0, 10.0, 0.0], X, 20.0, None), Ok(None));
        assert_eq!(backend.state(body), before);
    }
}

#[test]
fn closest_hit_excludes_only_the_named_body_and_ignores_contact_groups() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    assert_eq!(backend.cast_ray([0.0; 3], X, 10.0, None), Ok(None));
    let far = backend
        .spawn(spec(Shape::Sphere { radius: 0.5 }, [8.0, 0.0, 0.0]))
        .unwrap();
    let near = backend
        .spawn(spec(Shape::Sphere { radius: 0.5 }, [5.0, 0.0, 0.0]))
        .unwrap();
    let own = backend
        .spawn(spec(Shape::Sphere { radius: 1.0 }, [0.0; 3]))
        .unwrap();
    backend.set_self_collision_group(near, Some(7)).unwrap();
    backend.set_self_collision_group(own, Some(7)).unwrap();
    close(backend.cast_ray([0.0; 3], X, 10.0, None).unwrap(), 0.0);
    close(backend.cast_ray([0.0; 3], X, 10.0, Some(own)).unwrap(), 4.5);
    assert!(backend.remove(near));
    close(backend.cast_ray([0.0; 3], X, 10.0, Some(own)).unwrap(), 7.5);
    assert!(backend.remove(far));
    assert_eq!(backend.cast_ray([0.0; 3], X, 10.0, Some(own)), Ok(None));
    assert!(backend.remove(own));
    assert_eq!(
        backend.cast_ray([0.0; 3], X, 10.0, Some(own)),
        Err(PhysicsError::MissingBody)
    );
}

#[test]
fn invalid_and_successful_queries_preserve_ids_state_and_queued_work() {
    let mut queried = RapierBackend::new([0.0; 3]).unwrap();
    let mut control = RapierBackend::new([0.0; 3]).unwrap();
    let body = BodySpec {
        mass: Some(2.0),
        ..spec(Shape::Sphere { radius: 0.5 }, [5.0, 0.0, 0.0])
    };
    let id = queried.spawn(body).unwrap();
    assert_eq!(control.spawn(body).unwrap(), id);
    for backend in [&mut queried, &mut control] {
        backend
            .add_world_wrench(id, [2.0, 0.0, 0.0], [0.0, 0.0, 0.25])
            .unwrap();
    }
    let before = queried.state(id);
    for (origin, direction, max_distance) in [
        ([f64::NAN, 0.0, 0.0], X, 10.0),
        ([0.0, f64::INFINITY, 0.0], X, 10.0),
        ([0.0; 3], [f64::NAN, 0.0, 0.0], 10.0),
        ([0.0; 3], [0.0, f64::INFINITY, 0.0], 10.0),
        ([0.0; 3], [0.0; 3], 10.0),
        ([0.0; 3], [2.0, 0.0, 0.0], 10.0),
        ([0.0; 3], [1.0 + 1e-10, 0.0, 0.0], 10.0),
        ([0.0; 3], X, 0.0),
        ([0.0; 3], X, -1.0),
        ([0.0; 3], X, f64::NAN),
        ([0.0; 3], X, f64::INFINITY),
        ([f64::MAX, 0.0, 0.0], X, f64::MAX),
    ] {
        assert_eq!(
            queried.cast_ray(origin, direction, max_distance, None),
            Err(PhysicsError::InvalidValue)
        );
        assert_eq!(queried.state(id), before);
    }
    assert_eq!(
        queried.cast_ray([0.0; 3], X, 10.0, Some(BodyId::from_raw(u64::MAX))),
        Err(PhysicsError::MissingBody)
    );
    for _ in 0..3 {
        close(queried.cast_ray([0.0; 3], X, 10.0, None).unwrap(), 4.5);
        assert_eq!(queried.state(id), before);
    }
    assert_eq!(
        queried.cast_ray([0.0; 3], X, f64::MIN_POSITIVE, None),
        Ok(None)
    );
    let next = spec(Shape::Sphere { radius: 0.5 }, [50.0, 0.0, 0.0]);
    assert_eq!(queried.spawn(next).unwrap().raw(), 1);
    assert_eq!(control.spawn(next).unwrap().raw(), 1);
    for dt in [
        Duration::ZERO,
        Duration::from_millis(1),
        Duration::from_millis(1),
    ] {
        queried.step(dt).unwrap();
        control.step(dt).unwrap();
        assert_eq!(queried.state(id), control.state(id));
    }
    let velocity = queried.state(id).unwrap().velocity;
    assert!((velocity[0] - 0.001).abs() < 1e-12);
    assert!(queried.state(id).unwrap().angular_velocity[2] > 0.0);
}

#[test]
fn triangle_mesh_queries_compose_body_and_local_collision_frames_before_stepping() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let vertices = [
        [-2.0, -2.0, 0.0],
        [2.0, -2.0, 0.0],
        [2.0, 2.0, 0.0],
        [-2.0, 2.0, 0.0],
    ];
    let triangles = [[0, 1, 2], [0, 2, 3]];
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let id = backend
        .spawn_mesh_link(
            MeshBodySpec {
                vertices: &vertices,
                triangles: &triangles,
                collision_position: [0.0, 0.0, 1.0],
                collision_orientation: [0.0, 0.0, s, s],
                position: [5.0, 2.0, 3.0],
                velocity: [0.0; 3],
                mass: None,
            },
            RotationSpec {
                orientation: [0.0, -s, 0.0, s],
                ..RotationSpec::default()
            },
            None,
        )
        .unwrap();
    close(
        backend.cast_ray([0.0, 2.0, 3.0], X, 10.0, None).unwrap(),
        4.0,
    );
    assert_eq!(backend.cast_ray([0.0, 10.0, 3.0], X, 10.0, None), Ok(None));
    assert_eq!(
        backend.cast_ray([0.0, 2.0, 3.0], X, 10.0, Some(id)),
        Ok(None)
    );
    backend.reset().unwrap();
    close(
        backend.cast_ray([0.0, 2.0, 3.0], X, 10.0, None).unwrap(),
        4.0,
    );
    assert!(backend.remove(id));
    assert_eq!(backend.cast_ray([0.0, 2.0, 3.0], X, 10.0, None), Ok(None));
}

#[test]
fn owned_convex_geometry_motion_and_both_reset_paths_are_immediately_queryable() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = {
        let vertices = [
            [-0.5, -0.5, -0.5],
            [0.5, -0.5, -0.5],
            [0.5, 0.5, -0.5],
            [-0.5, 0.5, -0.5],
            [-0.5, -0.5, 0.5],
            [0.5, -0.5, 0.5],
            [0.5, 0.5, 0.5],
            [-0.5, 0.5, 0.5],
        ];
        let triangles = [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ];
        let s = std::f64::consts::FRAC_1_SQRT_2;
        backend
            .spawn_mesh_link(
                MeshBodySpec {
                    vertices: &vertices,
                    triangles: &triangles,
                    collision_position: [1.0, 0.0, 0.0],
                    collision_orientation: [0.0, s, 0.0, s],
                    position: [3.0, 4.0, 2.0],
                    velocity: [0.0, 1.0, 0.0],
                    mass: Some(2.0),
                },
                RotationSpec {
                    orientation: [0.0, 0.0, s, s],
                    ..RotationSpec::default()
                },
                None,
            )
            .unwrap()
    };
    // Borrowed source arrays are gone. Collider ownership, not an asset reload,
    // supplies every query and both forms of reset below.
    let captured = backend.initial_state();
    close(
        backend.cast_ray([3.0, 0.0, 2.0], Y, 10.0, None).unwrap(),
        4.5,
    );
    for _ in 0..100 {
        backend.step(Duration::from_millis(1)).unwrap();
    }
    let position = backend.state(id).unwrap().position;
    close(
        backend.cast_ray([3.0, 0.0, 2.0], Y, 10.0, None).unwrap(),
        position[1] + 0.5,
    );
    backend.reset().unwrap();
    close(
        backend.cast_ray([3.0, 0.0, 2.0], Y, 10.0, None).unwrap(),
        4.5,
    );
    assert!(backend.remove(id));
    let runtime = backend
        .spawn(spec(Shape::Sphere { radius: 0.25 }, [3.0, 2.0, 2.0]))
        .unwrap();
    close(
        backend.cast_ray([3.0, 0.0, 2.0], Y, 10.0, None).unwrap(),
        1.75,
    );
    let prepared = backend.prepare_initial(&captured).unwrap();
    backend.apply_initial(prepared).unwrap();
    assert!(backend.state(id).is_some());
    assert!(backend.state(runtime).is_none());
    close(
        backend.cast_ray([3.0, 0.0, 2.0], Y, 10.0, None).unwrap(),
        4.5,
    );
    assert!(backend.remove(id));
    backend.reset().unwrap();
    assert_eq!(backend.cast_ray([3.0, 0.0, 2.0], Y, 10.0, None), Ok(None));
}
