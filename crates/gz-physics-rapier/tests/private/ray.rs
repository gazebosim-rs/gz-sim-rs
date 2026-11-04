// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn casts_use_current_body_and_local_pose_without_refreshing_stale_collider_cache() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let id = backend
        .spawn(BodySpec {
            shape: Shape::Box {
                half_extents: [0.5, 1.0, 0.25],
            },
            position: [5.0, 0.0, 0.0],
            velocity: [0.0; 3],
            mass: None,
        })
        .unwrap();
    let handle = backend.bodies[&id].0;
    let collider = backend.world.bodies[handle].colliders()[0];
    let stale = *backend.world.colliders[collider].position();
    let s = std::f64::consts::FRAC_1_SQRT_2;
    backend.world.colliders[collider].set_position_wrt_parent(Pose::from_parts(
        Vector::X,
        Rotation::from_xyzw(0.0, 0.0, s, s),
    ));
    backend.world.bodies[handle].set_position(
        Pose::from_parts(
            Vector::new(0.0, 5.0, 0.0),
            Rotation::from_xyzw(0.0, 0.0, s, s),
        ),
        false,
    );
    assert_eq!(*backend.world.colliders[collider].position(), stale);
    let before = backend.state(id);
    let hit = backend
        .cast_ray([0.0; 3], [0.0, 1.0, 0.0], 10.0, None)
        .unwrap()
        .unwrap();
    assert!((hit - 5.0).abs() < 1e-12);
    assert_eq!(backend.state(id), before);
    assert_eq!(*backend.world.colliders[collider].position(), stale);
    assert!(
        backend
            .world
            .broad_phase
            .as_query_pipeline(
                backend.world.narrow_phase.query_dispatcher(),
                &backend.world.bodies,
                &backend.world.colliders,
                QueryFilter::default(),
            )
            .cast_ray(&Ray::new(Vector::ZERO, Vector::Y), 10.0, true)
            .is_none()
    );
}

#[test]
fn standalone_shapes_are_included_and_nonfinite_current_poses_are_refused() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let collider = backend.world.insert_collider(
        ColliderBuilder::ball(0.5).translation(Vector::new(3.0, 0.0, 0.0)),
        None,
    );
    assert_eq!(
        backend.cast_ray([0.0; 3], [1.0, 0.0, 0.0], 10.0, None),
        Ok(Some(2.5))
    );
    backend.world.colliders[collider].set_position(Pose::from_parts(
        Vector::new(f64::NAN, 0.0, 0.0),
        Rotation::IDENTITY,
    ));
    assert_eq!(
        backend.cast_ray([0.0; 3], [1.0, 0.0, 0.0], 10.0, None),
        Err(PhysicsError::InvalidValue)
    );
}
