// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ownership and thread lifecycle of independent native worlds.

use gz_physics::{BodySpec, DynamicBackend, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use std::{sync::mpsc, thread, time::Duration};

fn contact_world() -> (DartBackend, gz_physics::BodyId) {
    let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
    let ground = BodySpec {
        shape: Shape::Box {
            half_extents: [10.0, 10.0, 0.5],
        },
        position: [0.0, 0.0, -0.5],
        velocity: [0.0; 3],
        mass: None,
    };
    backend
        .spawn_link(ground, RotationSpec::default(), None)
        .unwrap();
    let ball = BodySpec {
        shape: Shape::Sphere { radius: 0.5 },
        position: [0.0, 0.0, 0.6],
        velocity: [0.0; 3],
        mass: Some(1.0),
    };
    let id = backend
        .spawn_link(ball, RotationSpec::default(), None)
        .unwrap();
    (backend, id)
}

fn settle(backend: &mut DartBackend, id: gz_physics::BodyId) -> f64 {
    for _ in 0..200 {
        backend.step(Duration::from_millis(1)).unwrap();
    }
    backend.state(id).unwrap().position[2]
}

#[test]
fn concurrent_worlds_construct_step_and_drop_independently() {
    let (mut reference, id) = contact_world();
    let expected = settle(&mut reference, id);
    drop(reference);
    let workers: Vec<_> = (0..8)
        .map(|_| {
            thread::spawn(move || {
                for _ in 0..20 {
                    let (mut backend, id) = contact_world();
                    assert_eq!(settle(&mut backend, id), expected);
                }
            })
        })
        .collect();
    for worker in workers {
        worker.join().unwrap();
    }
}

#[test]
fn a_world_moves_between_threads_and_drops_on_another() {
    let (mut backend, id) = contact_world();
    let first = thread::spawn(move || {
        backend.step(Duration::from_millis(1)).unwrap();
        backend
    })
    .join()
    .unwrap();
    let (sender, receiver) = mpsc::channel();
    let second = thread::spawn(move || {
        let mut backend: DartBackend = receiver.recv().unwrap();
        settle(&mut backend, id)
    });
    sender.send(first).unwrap();
    let (mut reference, reference_id) = contact_world();
    reference.step(Duration::from_millis(1)).unwrap();
    assert_eq!(second.join().unwrap(), settle(&mut reference, reference_id));
}

#[test]
fn prebuilt_worlds_step_from_many_threads_with_identical_results() {
    let (mut reference, id) = contact_world();
    let expected = settle(&mut reference, id);
    for _ in 0..20 {
        let worlds: Vec<_> = (0..8).map(|_| contact_world()).collect();
        let workers: Vec<_> = worlds
            .into_iter()
            .map(|(mut backend, id)| thread::spawn(move || (settle(&mut backend, id), backend)))
            .collect();
        let done: Vec<_> = workers.into_iter().map(|w| w.join().unwrap()).collect();
        for (z, _) in &done {
            assert_eq!(*z, expected);
        }
    }
}
