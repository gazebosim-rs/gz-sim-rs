// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn spec(x: f64) -> BodySpec {
    BodySpec {
        shape: Shape::Box {
            half_extents: [0.45, 0.35, 0.1],
        },
        position: [x, 0., 0.],
        velocity: [0.25, 0., 0.],
        mass: Some(1.),
    }
}

fn lane() -> (RapierBackend, Vec<BodyId>) {
    let mut backend = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
    let ids = (0..fleet_isolation::MIN_DECKS)
        .map(|i| {
            backend
                .spawn_velocity_boundary(spec(i as f64), Default::default(), None)
                .unwrap()
        })
        .collect();
    (backend, ids)
}

// Independent exhaustive cast from live public body poses and native collider-local
// offsets. It uses no fleet bounds, cached collider world pose, or candidate ordering.
fn compare(
    backend: &RapierBackend,
    origin: [f64; 3],
    dir: [f64; 3],
    limit: f64,
    exclude: Option<BodyId>,
) -> Option<f64> {
    let ray = Ray::new(origin.into(), dir.into());
    let mut expected: Option<f64> = None;
    for (id, (handle, ..)) in &backend.bodies {
        if Some(*id) == exclude {
            continue;
        }
        let state = backend.state(*id).unwrap();
        let pose = Pose::from_parts(
            state.position.into(),
            Rotation::from_array(state.orientation),
        );
        for collider in backend.world.bodies[*handle].colliders() {
            let collider = &backend.world.colliders[*collider];
            let p = pose * collider.position_wrt_parent().unwrap();
            if let Some(hit) = collider.shape().cast_ray(&p, &ray, limit, true) {
                expected = Some(expected.map_or(hit, |other| other.min(hit)));
            }
        }
    }
    let actual = backend.cast_ray(origin, dir, limit, exclude).unwrap();
    assert_eq!(actual, expected, "origin {origin:?}, dir {dir:?}");
    actual
}

#[test]
fn prescribed_ray_index_respects_edit_removal_late_spawn_and_reset() {
    let (mut backend, ids) = lane();
    let down = [0., 0., -1.];
    backend.step(Duration::from_secs(1)).unwrap();
    assert_eq!(
        compare(&backend, [0.25, 0., 1.], down, 0.9, None),
        Some(0.9)
    );
    assert!(backend.translation_rays.get().unwrap().is_some());
    assert_eq!(compare(&backend, [0.25, 0., 0.], down, 1., None), Some(0.));
    assert_eq!(
        compare(&backend, [0.25, 0., 1.], down, 1., Some(ids[0])),
        None
    );
    compare(&backend, [-2., 0.35, 0.1], [1., 0., 0.], 300., None);
    compare(&backend, [-2., 0.350001, 0.1], [1., 0., 0.], 300., None);
    compare(&backend, [1.1e6, 0., 1.], down, 1., None);
    compare(&backend, [0., 0., 1.], down, 1.1e6, None);
    let mut state = backend.state(ids[0]).unwrap();
    state.position = [10., 0., 0.5];
    backend.set_body_state(ids[0], state).unwrap();
    assert!(backend.translation_rays.get().is_none());
    assert_eq!(compare(&backend, [10., 0., 1.], down, 2., None), Some(0.4));
    assert!(backend.translation_rays.get().unwrap().is_none());
    assert!(backend.remove(ids[0]));
    backend
        .spawn_velocity_boundary(spec(-4.), Default::default(), None)
        .unwrap();
    compare(&backend, [-4., 0., 1.], down, 2., None);
    backend.reset().unwrap();
    compare(&backend, [-4., 0., 1.], down, 2., None);
    assert!(backend.translation_rays.get().unwrap().is_some());
    backend
        .set_world_velocity(ids[1], [-0.75, 0., 0.], [0.; 3])
        .unwrap();
    backend.step(Duration::from_millis(100)).unwrap();
    compare(&backend, [0.925, 0., 1.], down, 2., None);
    assert!(backend.translation_rays.get().unwrap().is_none());
}

#[test]
fn loaded_beams_follow_analytic_native_and_independently_moved_decks() {
    let (mut backend, ids) = lane();
    let payload = backend
        .spawn(BodySpec {
            position: [0., 0., 0.201],
            velocity: [0.; 3],
            shape: Shape::Box {
                half_extents: [0.1; 3],
            },
            ..spec(0.)
        })
        .unwrap();
    for _ in 0..80 {
        backend.step(Duration::from_millis(10)).unwrap();
    }
    assert!(backend.fleet_isolation_stats().unwrap().analytic_decks > 200);
    let down = [0., 0., -1.];
    assert!(compare(&backend, [0.2, 0., 1.], down, 2., None).unwrap() < 0.8);
    compare(&backend, [0.2, 0., 1.], down, 2., Some(payload));
    compare(&backend, [255.2, 0., 1.], down, 2., None);
    compare(&backend, [-1., 0.36, 0.], [1., 0., 0.], 300., None);
    let d = std::f64::consts::FRAC_1_SQRT_2;
    compare(&backend, [3., 0., 10.], [d, 0., -d], 25., None);
    compare(&backend, [1.1e6, 0., 10.], down, 25., None);
    let id = ids[200];
    backend
        .set_world_velocity(id, [0., 4., 0.], [0.; 3])
        .unwrap();
    for _ in 0..50 {
        backend.step(Duration::from_millis(10)).unwrap();
    }
    let pos = backend.state(id).unwrap().position;
    assert!(pos[1] > 1.9);
    assert_eq!(
        compare(&backend, [pos[0], pos[1], 1.], down, 2., None),
        Some(0.9)
    );
    compare(&backend, [pos[0], 0., 1.], down, 2., None);
    compare(&backend, [pos[0], pos[1], 1.], down, 2., Some(id));
}

#[test]
fn offset_and_standalone_geometry_do_not_use_body_centered_bounds() {
    let (mut backend, ids) = lane();
    let h = backend.bodies[&ids[0]].0;
    let c = backend.world.bodies[h].colliders()[0];
    let offset = Pose::translation(0., 3., 0.);
    backend.world.colliders[c].set_position_wrt_parent(offset);
    backend
        .bodies
        .get_mut(&ids[0])
        .unwrap()
        .1
        .collider
        .set_position(offset);
    compare(&backend, [0., 3., 1.], [0., 0., -1.], 1., None);
    assert!(backend.translation_rays.get().unwrap().is_none());
    let point = [0., 3., 0.];
    assert_eq!(
        backend.bodies_overlapping_aabb(point, point).unwrap(),
        [ids[0]]
    );
    backend.world.insert_collider(
        ColliderBuilder::ball(0.5).translation(Vector::new(0., 3.5, 0.)),
        Some(h),
    );
    // The two touching colliders belong to one body.
    assert_eq!(
        backend.bodies_overlapping_aabb(point, point).unwrap(),
        [ids[0]]
    );
    let only_second = [0., 4., 0.];
    assert_eq!(
        backend
            .bodies_overlapping_aabb(only_second, only_second)
            .unwrap(),
        [ids[0]]
    );
    backend
        .spawn(BodySpec {
            position: [0., 3., 0.198],
            velocity: [0.; 3],
            ..spec(0.)
        })
        .unwrap();
    backend.step(Duration::from_millis(10)).unwrap();
    assert!(backend.fleet_isolation_stats().is_none());
    compare(&backend, [0., 3., 1.], [0., 0., -1.], 1., None);
    let (mut backend, _) = lane();
    backend.world.insert_collider(
        ColliderBuilder::ball(0.5).translation(Vector::new(0., 3., 0.)),
        None,
    );
    assert_eq!(
        backend
            .cast_ray([0., 3., 1.], [0., 0., -1.], 1., None)
            .unwrap(),
        Some(0.5)
    );
    assert!(backend.translation_rays.get().unwrap().is_none());
    assert!(
        backend
            .bodies_overlapping_aabb(point, point)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn a_malformed_candidate_refuses_the_beam_even_after_a_valid_hit() {
    for index in [0, 1, 255] {
        let (mut backend, ids) = lane();
        let payload = backend
            .spawn(BodySpec {
                position: [0., 0., 0.2],
                velocity: [0.; 3],
                shape: Shape::Box {
                    half_extents: [0.1; 3],
                },
                ..spec(0.)
            })
            .unwrap();
        backend.step(Duration::from_millis(10)).unwrap();
        let bad = match index {
            0 => payload,
            1 => ids[0],
            _ => ids[index],
        };
        let x = backend.state(bad).unwrap().position[0];
        let handle = backend.bodies[&bad].0;
        backend.world.bodies[handle].set_translation(Vector::NAN, false);
        assert_eq!(
            backend.cast_ray([x, 0., 1.], [0., 0., -1.], 2., None),
            Err(PhysicsError::InvalidValue)
        );
    }
    let (mut backend, ids) = lane();
    let origin = [255., 0., 1.];
    compare(&backend, origin, [0., 0., -1.], 2., None);
    let handle = backend.bodies[&ids[255]].0;
    backend.world.bodies[handle].set_translation(Vector::NAN, false);
    assert_eq!(
        backend.cast_ray(origin, [0., 0., -1.], 2., None),
        Err(PhysicsError::InvalidValue)
    );
}

#[test]
fn a_rotated_prescribed_box_matches_the_native_shape_at_both_ends_of_its_motion() {
    let (mut backend, _) = lane();
    let angle = 0.635_f64;
    let id = backend
        .spawn_velocity_boundary(
            spec(-3.),
            RotationSpec {
                orientation: [0., 0., (angle * 0.5).sin(), (angle * 0.5).cos()],
                angular_velocity: [0.; 3],
            },
            None,
        )
        .unwrap();
    for _ in 0..2 {
        let x = backend.state(id).unwrap().position[0];
        for y in [0., 0.3, 0.45, 0.6] {
            compare(&backend, [x - 1., y, 0.], [1., 0., 0.], 2., None);
        }
        backend.step(Duration::from_millis(10)).unwrap();
    }
    assert!(backend.translation_rays.get().unwrap().is_some());
}

#[test]
fn native_beam_keeps_the_tip_of_a_rotating_box_and_checks_invalid_off_ray_poses() {
    let (mut world, _) = lane();
    let cargo = |x| BodySpec {
        shape: Shape::Box {
            half_extents: [0.8, 0.04, 0.04],
        },
        position: [x, 0., 3.],
        velocity: [0.; 3],
        mass: Some(1.),
    };
    let near = world.spawn(cargo(0.)).unwrap();
    let far = world.spawn(cargo(200.)).unwrap();
    let mut state = world.state(near).unwrap();
    state.orientation = Rotation::from_rotation_z(std::f64::consts::FRAC_PI_2).to_array();
    world.set_body_state(near, state).unwrap();
    world.step(Duration::from_millis(10)).unwrap();
    let center = world.state(near).unwrap().position;
    let origin = [center[0], center[1] + 0.79, center[2] + 1.];
    let down = [0., 0., -1.];
    assert!(compare(&world, origin, down, 1., None).is_some());
    let beam = SolidRay::new(origin, down, 1.).unwrap();
    let fleet_isolation::Dispatch::Active(group) = &world.fleet_dispatch else {
        panic!("loaded boxes must use selective native contact");
    };
    let mut candidates = Vec::new();
    group
        .visit_ray_bodies(&beam, &world.world, |h| {
            candidates.push(h);
            Ok(())
        })
        .unwrap()
        .unwrap();
    assert!(candidates.contains(&world.bodies[&near].0));
    let far_handle = world.bodies[&far].0;
    assert!(!candidates.contains(&far_handle));
    world.world.bodies[far_handle].set_translation(Vector::NAN, false);
    assert_eq!(
        world.cast_ray(origin, down, 1., None),
        Err(PhysicsError::InvalidValue)
    );
    assert!(
        world
            .cast_ray(origin, down, 1., Some(far))
            .unwrap()
            .is_some()
    );
}

#[test]
fn closed_gate_candidates_are_sorted_and_match_world_bounds_after_edits_and_reset() {
    let (mut world, ids) = lane();
    for (lo, hi) in [
        ([f64::NAN; 3], [0.; 3]),
        ([0.; 3], [f64::INFINITY; 3]),
        ([1., 0., 0.], [0.; 3]),
    ] {
        assert_eq!(
            world.bodies_overlapping_aabb(lo, hi),
            Err(PhysicsError::InvalidValue)
        );
    }
    let advance = Duration::from_millis(10);
    world.step(advance).unwrap();
    let query = |b: &RapierBackend, lo: [f64; 3], hi: [f64; 3]| {
        let mut expected = Vec::new();
        for (&id, (h, ..)) in &b.bodies {
            let state = b.state(id).unwrap();
            let pose = Pose::from_parts(
                state.position.into(),
                Rotation::from_array(state.orientation),
            );
            if b.world.bodies[*h].colliders().iter().any(|h| {
                let c = &b.world.colliders[*h];
                let bounds = c
                    .shape()
                    .compute_aabb(&(pose * c.position_wrt_parent().unwrap()));
                (0..3).all(|a| bounds.mins[a] <= hi[a] && bounds.maxs[a] >= lo[a])
            }) {
                expected.push(id);
            }
        }
        let hits = b.bodies_overlapping_aabb(lo, hi).unwrap();
        assert_eq!(hits, expected);
        hits
    };
    let x = world.state(ids[0]).unwrap().position[0] + 0.45;
    assert_eq!(query(&world, [x, 0., 0.], [x, 0., 0.]), [ids[0]]);
    let after = f64::from_bits(x.to_bits() + 1);
    assert!(query(&world, [after, 0., 0.], [after, 0., 0.]).is_empty());
    assert_eq!(query(&world, [-1.; 3], [2.6, 1., 1.]), ids[..4]);
    assert_eq!(query(&world, [-2.0e6; 3], [2.0e6; 3]), ids);
    let lost = ids[2];
    assert!(world.remove(lost));
    let replacement = world
        .spawn_velocity_boundary(spec(2.), Default::default(), None)
        .unwrap();
    world
        .set_world_velocity(replacement, [-1., 0., 0.], [0.; 3])
        .unwrap();
    world.step(advance).unwrap();
    assert!(query(&world, [-1.; 3], [3.6, 1., 1.]).contains(&replacement));
    assert!(world.translation_rays.get().unwrap().is_none());
    let malformed = world.bodies[&replacement].0;
    world.world.bodies[malformed].set_translation(Vector::NAN, false);
    assert_eq!(
        world.bodies_overlapping_aabb([-1.; 3], [3.6, 1., 1.]),
        Err(PhysicsError::InvalidValue)
    );
    world.reset().unwrap();
    query(&world, [-1.; 3], [3.6, 1., 1.]);
}

#[test]
fn a_rotated_box_reports_its_world_bounds_including_empty_corners() {
    let (mut world, _) = lane();
    let box_id = world
        .spawn_velocity_boundary(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.8, 0.05, 0.1],
                },
                ..spec(-3.)
            },
            RotationSpec {
                orientation: Rotation::from_rotation_z(std::f64::consts::FRAC_PI_4).to_array(),
                ..Default::default()
            },
            None,
        )
        .unwrap();
    let corner = [-2.45, -0.55, 0.];
    assert_eq!(
        world.bodies_overlapping_aabb(corner, corner).unwrap(),
        [box_id]
    );
    assert_eq!(
        world
            .cast_ray([corner[0], corner[1], 1.], [0., 0., -1.], 2., None)
            .unwrap(),
        None
    );
}
