// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{
    BodySpec, Shape,
    trajectory::{PathEnd, TranslationSpline, Waypoint},
};
use gz_physics_distributed::trajectory::{
    TrajectoryWorkers,
    material::{MaterialRecipe, Surface},
};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_physics_xpbd::{Particle, SoftBody, SolverConfig};
use std::{path::Path, time::Duration};

fn material(inverse_mass: f64, gravity: f64) -> SoftBody {
    let particles = (0..4)
        .map(|i| Particle {
            position: [i as f64 * 0.2, 1., 0.5],
            velocity: [0.; 3],
            inverse_mass,
        })
        .collect();
    SoftBody::cable(
        particles,
        0.0001,
        0.01,
        SolverConfig {
            gravity: [0., 0., gravity],
            self_contact: false,
            ..Default::default()
        },
    )
    .unwrap()
}

#[test]
fn incorrect_public_material_laws_are_rejected_before_a_solver_or_caller_advances() {
    let mut backend = MultiFidelityBackend::new([0., 0., 0.]).unwrap();
    let native = backend
        .spawn_kinematic_link(
            BodySpec {
                position: [0.; 3],
                velocity: [0.; 3],
                mass: Some(1.),
                shape: Shape::Sphere { radius: 0.1 },
            },
            Default::default(),
            None,
        )
        .unwrap();
    let path = TranslationSpline::new(
        &[
            Waypoint {
                time: Duration::ZERO,
                displacement: [0.; 3],
                velocity: [0.; 3],
            },
            Waypoint {
                time: Duration::from_secs(1),
                displacement: [1., 0., 0.],
                velocity: [0.; 3],
            },
        ],
        PathEnd::Hold,
    )
    .unwrap();
    backend.set_initial_translation_path(native, path).unwrap();
    backend.prepare_trajectory_owner().unwrap();
    let mut first = material(10., -1.);
    let mut second = material(20., -1.);
    let recipe = MaterialRecipe::new(
        vec![Surface::Sphere {
            body: native.raw(),
            radius: 0.1,
        }],
        [
            ("first".to_owned(), first.one_way_construction().unwrap()),
            ("second".to_owned(), second.one_way_construction().unwrap()),
        ],
    )
    .unwrap();
    let mut workers = TrajectoryWorkers::with_material(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        2,
        &backend,
        recipe,
    )
    .unwrap();
    let dt = Duration::from_millis(1);
    let origin = backend.checkpoint_trajectory().unwrap();
    let a = first.one_way_state().unwrap();
    let b = second.one_way_state().unwrap();
    assert!(
        workers
            .advance_with_material(&mut backend, &mut [&mut second, &mut first], 1, dt)
            .is_err()
    );
    let mut wrong_gravity = material(20., -2.);
    assert!(
        workers
            .advance_with_material(&mut backend, &mut [&mut first, &mut wrong_gravity], 1, dt)
            .is_err()
    );
    assert!(workers.last().is_none());
    assert_eq!(backend.checkpoint_trajectory().unwrap(), origin);
    assert_eq!(first.one_way_state().unwrap(), a);
    assert_eq!(second.one_way_state().unwrap(), b);
    for tick in 1..=4 {
        workers
            .advance_with_material(&mut backend, &mut [&mut first, &mut second], tick, dt)
            .unwrap();
    }
    assert_ne!(first.one_way_state().unwrap(), a);
}
