// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn sphere(position: [f64; 3]) -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.5 },
        position,
        velocity: [1.0, 0.0, 0.0],
        mass: Some(2.0),
    }
}

#[test]
fn prepared_reset_restores_removed_initial_bodies_and_retires_later_allocations() {
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let initial_body = backend
        .spawn_link(
            sphere([0.0, 0.0, 5.0]),
            RotationSpec {
                orientation: [0.0, 0.0, 0.5_f64.sin(), 0.5_f64.cos()],
                angular_velocity: [0.0, 0.0, 2.0],
            },
            Some(Inertia {
                center: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
                tensor: [0.2, 0.3, 0.4, 0.02, 0.0, 0.0],
            }),
        )
        .unwrap();
    let state = backend.state(initial_body).unwrap();
    let initial = backend.initial_state();
    backend.step(Duration::from_millis(10));
    assert_ne!(backend.state(initial_body), Some(state));
    assert!(backend.remove(initial_body));
    let runtime = backend.spawn(sphere([0.0, 0.0, 10.0])).unwrap();
    let runtime_state = backend.state(runtime);
    let prepared = backend.prepare_initial(&initial).unwrap();
    assert_eq!(backend.state(initial_body), None);
    assert_eq!(backend.state(runtime), runtime_state);
    // Allocations made after preparation must also remain retired after commit.
    let later = backend.spawn(sphere([0.0, 0.0, 20.0])).unwrap();
    backend.apply_initial(prepared).unwrap();
    assert_eq!(backend.state(initial_body), Some(state));
    assert_eq!(backend.state(runtime), None);
    assert_eq!(backend.state(later), None);
    let fresh = backend.spawn(sphere([0.0, 0.0, 30.0])).unwrap();
    assert!(fresh > later);
    backend.step(Duration::from_millis(1));
    let next = backend.state(initial_body).unwrap();
    assert!(next.position[0] > state.position[0]);
    assert!(next.position[2] < state.position[2]);
    assert_ne!(next.orientation, state.orientation);
    backend
        .apply_initial(backend.prepare_initial(&initial.clone()).unwrap())
        .unwrap();
    assert_eq!(backend.state(initial_body), Some(state));
    assert_eq!(backend.state(fresh), None);
    assert!(backend.spawn(sphere([0.0, 0.0, 40.0])).unwrap() > fresh);
}

#[test]
fn foreign_initial_and_prepared_states_leave_both_backends_unchanged() {
    let mut first = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    let original = first.spawn(sphere([0.0, 0.0, 5.0])).unwrap();
    let first_state = first.state(original);
    let initial = first.initial_state();
    let mut second = RapierBackend::new([0.0, 1.0, 0.0]).unwrap();
    let other = second.spawn(sphere([10.0, 0.0, 0.0])).unwrap();
    let second_state = second.state(other);
    assert!(matches!(
        second.prepare_initial(&initial),
        Err(PhysicsError::ForeignInitialState)
    ));
    let prepared = first.prepare_initial(&initial).unwrap();
    assert_eq!(
        second.apply_initial(prepared),
        Err(PhysicsError::ForeignInitialState)
    );
    assert_eq!(first.state(original), first_state);
    assert_eq!(second.state(other), second_state);
    assert_eq!(second.world.gravity.to_array(), [0.0, 1.0, 0.0]);
    drop(first);
    assert!(matches!(
        second.prepare_initial(&initial),
        Err(PhysicsError::ForeignInitialState)
    ));
    assert!(
        PhysicsError::ForeignInitialState
            .to_string()
            .contains("different backend")
    );
}

#[test]
fn empty_and_sparse_initial_states_preserve_body_ids_and_high_water_mark() {
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    let empty = backend.initial_state();
    let removed = backend.spawn(sphere([0.0; 3])).unwrap();
    let retained = backend.spawn(sphere([0.0, 2.0, 0.0])).unwrap();
    assert!(backend.remove(removed));
    let sparse = backend.initial_state();
    let retained_state = backend.state(retained);
    assert!(backend.remove(retained));
    backend
        .apply_initial(backend.prepare_initial(&sparse).unwrap())
        .unwrap();
    assert_eq!(backend.state(removed), None);
    assert_eq!(backend.state(retained), retained_state);
    backend
        .apply_initial(backend.prepare_initial(&empty).unwrap())
        .unwrap();
    assert!(backend.bodies.is_empty());
    assert_eq!(backend.state(retained), None);
    assert!(backend.spawn(sphere([0.0; 3])).unwrap() > retained);
}

#[test]
fn initial_gravity_and_solver_settings_are_restored_and_retained_reset_keeps_settings() {
    let mut backend = RapierBackend::new([0.0, 0.0, -9.81]).unwrap();
    backend.world.integration_parameters.dt = 0.002;
    backend.world.integration_parameters.num_solver_iterations = 7;
    backend.world.integration_parameters.length_unit = 2.0;
    let settings = backend.world.integration_parameters;
    let initial = backend.initial_state();
    backend.world.integration_parameters.dt = 0.1;
    backend.world.integration_parameters.num_solver_iterations = 2;
    backend.world.gravity = Vector::new(0.0, 0.0, -1.0);
    backend
        .apply_initial(backend.prepare_initial(&initial).unwrap())
        .unwrap();
    assert_eq!(backend.world.integration_parameters, settings);
    assert_eq!(backend.world.gravity.to_array(), [0.0, 0.0, -9.81]);
    backend.reset();
    assert_eq!(backend.world.integration_parameters, settings);
    assert_eq!(backend.world.gravity.to_array(), [0.0, 0.0, -9.81]);
}
