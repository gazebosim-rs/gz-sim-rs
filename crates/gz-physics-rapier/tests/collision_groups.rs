// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_rapier::{BodyId, BodySpec, DynamicBackend, PhysicsError, RapierBackend, Shape};
use std::time::Duration;

fn pair() -> (RapierBackend, [BodyId; 2]) {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let bodies = [0.0, 0.75].map(|x| {
        backend
            .spawn(BodySpec {
                shape: Shape::Sphere { radius: 0.5 },
                position: [x, 0.0, 0.0],
                velocity: [0.0; 3],
                mass: Some(1.0),
            })
            .unwrap()
    });
    (backend, bodies)
}

fn advance(backend: &mut RapierBackend) {
    for _ in 0..100 {
        backend.step(Duration::from_millis(1)).unwrap();
    }
}

fn separation(backend: &RapierBackend, bodies: [BodyId; 2]) -> f64 {
    backend.state(bodies[1]).unwrap().position[0] - backend.state(bodies[0]).unwrap().position[0]
}

#[test]
fn same_group_disables_contacts_without_a_layer_count_or_id_limit() {
    for group in [0, 31, 32, 65536, u64::MAX] {
        let (mut backend, bodies) = pair();
        for body in bodies {
            backend.set_self_collision_group(body, Some(group)).unwrap();
        }
        advance(&mut backend);
        assert!((separation(&backend, bodies) - 0.75).abs() < 1e-12);
    }
}

#[test]
fn different_unassigned_and_cleared_groups_preserve_contacts() {
    let (mut reference, reference_bodies) = pair();
    advance(&mut reference);
    assert!(separation(&reference, reference_bodies) > 0.8);
    for groups in [
        [None, None],
        [Some(0), None],
        [None, Some(0)],
        [Some(0), Some(u64::MAX)],
    ] {
        let (mut backend, bodies) = pair();
        for (body, group) in bodies.into_iter().zip(groups) {
            backend.set_self_collision_group(body, Some(7)).unwrap();
            backend.set_self_collision_group(body, group).unwrap();
        }
        advance(&mut backend);
        for (body, expected) in bodies.into_iter().zip(reference_bodies) {
            let actual = backend.state(body).unwrap();
            let expected = reference.state(expected).unwrap();
            // Equivalent contact paths can differ by a final rounding bit.
            for (a, b) in actual.position.into_iter().zip(expected.position) {
                assert!(
                    (a - b).abs() <= 16.0 * f64::EPSILON,
                    "{groups:?}: {a} != {b}"
                );
            }
            assert_eq!(actual.velocity, expected.velocity);
            assert_eq!(actual.orientation, expected.orientation);
            assert_eq!(actual.angular_velocity, expected.angular_velocity);
        }
    }
}

#[test]
fn groups_survive_both_resets_and_removed_ids_stay_retired() {
    let (mut backend, bodies) = pair();
    for body in bodies {
        backend
            .set_self_collision_group(body, Some(u64::MAX))
            .unwrap();
    }
    let initial = backend.initial_state();
    advance(&mut backend);
    for body in bodies {
        let before = backend.state(body);
        assert!(matches!(
            backend.set_self_collision_group(body, None),
            Err(PhysicsError::Unsupported {
                capability: "runtime self-collision group edits",
                ..
            })
        ));
        assert_eq!(backend.state(body), before);
    }
    backend.reset().unwrap();
    advance(&mut backend);
    assert!((separation(&backend, bodies) - 0.75).abs() < 1e-12);
    let prepared = backend.prepare_initial(&initial).unwrap();
    backend.apply_initial(prepared).unwrap();
    backend.set_self_collision_group(bodies[0], None).unwrap();
    advance(&mut backend);
    let (mut reference, reference_bodies) = pair();
    advance(&mut reference);
    assert_eq!(
        separation(&backend, bodies),
        separation(&reference, reference_bodies)
    );
    let prepared = backend.prepare_initial(&initial).unwrap();
    backend.apply_initial(prepared).unwrap();
    advance(&mut backend);
    assert!((separation(&backend, bodies) - 0.75).abs() < 1e-12);
    assert!(backend.remove(bodies[0]));
    backend.reset().unwrap();
    assert_eq!(
        backend.set_self_collision_group(bodies[0], Some(0)),
        Err(PhysicsError::MissingBody)
    );
    assert!(backend.state(bodies[1]).is_some());
}

#[test]
fn prepared_groups_restore_earlier_edits_within_the_same_epoch() {
    let (mut backend, bodies) = pair();
    for body in bodies {
        backend.set_self_collision_group(body, Some(0)).unwrap();
    }
    let initial = backend.initial_state();
    let prepared = backend.prepare_initial(&initial).unwrap();
    backend
        .set_self_collision_group(bodies[0], Some(1))
        .unwrap();
    backend.apply_initial(prepared).unwrap();
    advance(&mut backend);
    assert!((separation(&backend, bodies) - 0.75).abs() < 1e-12);
}

#[test]
fn grouped_dynamic_bodies_still_contact_an_external_fixed_floor() {
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let floor = backend
        .spawn(BodySpec {
            shape: Shape::Box {
                half_extents: [5.0, 5.0, 0.1],
            },
            position: [0.0, 0.0, -0.1],
            velocity: [0.0; 3],
            mass: None,
        })
        .unwrap();
    let ball = backend
        .spawn(BodySpec {
            shape: Shape::Sphere { radius: 0.5 },
            position: [0.0, 0.0, 1.0],
            velocity: [0.0; 3],
            mass: Some(1.0),
        })
        .unwrap();
    backend
        .set_self_collision_group(ball, Some(u64::MAX))
        .unwrap();
    backend.set_self_collision_group(floor, Some(0)).unwrap();
    let fixed = backend.state(floor);
    for _ in 0..20 {
        advance(&mut backend);
    }
    let state = backend.state(ball).unwrap();
    assert!((state.position[2] - 0.5).abs() < 0.01);
    assert!(state.velocity[2].abs() < 0.01);
    assert_eq!(backend.state(floor), fixed);
}

#[test]
fn more_than_thirty_two_groups_coexist_without_cross_group_aliases() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let mut pairs = Vec::new();
    for cluster in 0..32 {
        let bodies = [(cluster, 0.0), (cluster + 32, 0.75)].map(|(group, offset)| {
            let body = backend
                .spawn(BodySpec {
                    shape: Shape::Sphere { radius: 0.5 },
                    position: [cluster as f64 * 3.0 + offset, 0.0, 0.0],
                    velocity: [0.0; 3],
                    mass: Some(1.0),
                })
                .unwrap();
            backend.set_self_collision_group(body, Some(group)).unwrap();
            body
        });
        pairs.push(bodies);
    }
    advance(&mut backend);
    for bodies in pairs {
        assert!(separation(&backend, bodies) > 0.8);
    }
}
