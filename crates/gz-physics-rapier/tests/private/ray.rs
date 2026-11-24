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

#[test]
fn finite_cylinder_barrel_flat_caps_tangency_and_finite_ray_endpoints() {
    let hit = |o: [f64; 3], d: [f64; 3], limit: f64| {
        finite_cylinder_hit(
            &Ray::new(Vector::from(o), Vector::from(d)),
            0.5,
            0.25,
            limit,
        )
        .unwrap()
    };
    assert_eq!(hit([0.; 3], [1., 0., 0.], 2.), Some(0.));
    assert_eq!(hit([0.1, 1., 0.], [0., -1., 0.], 0.75), Some(0.75));
    assert_eq!(hit([0.1, 1., 0.], [0., -1., 0.], 0.749), None);
    assert_eq!(hit([0.51, 1., 0.], [0., -1., 0.], 2.), None);
    assert_eq!(hit([-2., 0.25, 0.5], [1., 0., 0.], 2.), Some(2.));
    assert_eq!(hit([-2., 0.2501, 0.], [1., 0., 0.], 4.), None);
    assert_eq!(hit([-2., 0., 0.501], [1., 0., 0.], 4.), None);
    assert_eq!(hit([-1e8, 0., 0.3], [1., 0., 0.], 1e8), Some(1e8 - 0.4));
    assert_eq!(hit([1., 0., 0.], [1., 0., 0.], 10.), None);
}

#[test]
fn pending_support_impulse_discard_preserves_clearance_queries_and_the_pending_pair() {
    use crate::contact_dispatcher::{DiscardPairs, NativeContacts};
    // Include the generation: these are live, independently allocated supports.
    let mut robot = RapierBackend::new([0.; 3]).unwrap();
    let bodies = [0., 1.].map(|x| {
        let id = robot
            .spawn(BodySpec {
                shape: Shape::Sphere { radius: 0.25 },
                mass: Some(1.),
                position: [x, 0., 1.],
                velocity: [0.; 3],
            })
            .unwrap();
        robot.bodies[&id].0
    });
    let pending = DiscardPairs::new(vec![(bodies[0], bodies[1])]);
    crate::query_parity::assert_native_queries(&NativeContacts(pending.clone()));
    assert_eq!(pending.remaining(), [(bodies[0], bodies[1])]);
}
