// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{
    BodySpec, DynamicBackend, Shape,
    trajectory::{PathEnd, TranslationSpline, Waypoint},
};
use gz_physics_xpbd::{Boundary, Particle, SolverConfig};
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
fn atomic_native_consensus_checks_every_material_before_publishing() {
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

    let initial = backend.trajectory_construction().unwrap();
    let original = backend.checkpoint_trajectory().unwrap();
    let image = |dt| {
        let mut b = MultiFidelityBackend::from_trajectory_construction(&initial).unwrap();
        b.install_trajectory(&original).unwrap();
        b.step(Duration::from_millis(dt)).unwrap();
        b.checkpoint_trajectory().unwrap()
    };
    let valid = image(2);
    let late = image(3);
    assert_ne!(valid, late);
    let mut a = material(10., -1.);
    let mut b = material(20., -1.);
    let solved = |mut body: SoftBody| {
        body.step(
            Duration::from_millis(2),
            &[Boundary::Plane {
                point: [0.; 3],
                normal: [0., 0., 1.],
                velocity: [0.; 3],
            }],
        )
        .unwrap();
        body.one_way_state().unwrap()
    };
    let want_a = solved(material(10., -1.));
    let want_b = solved(material(20., -1.));
    let before_a = a.one_way_state().unwrap();
    let before_b = b.one_way_state().unwrap();
    let rows = [[0usize], [1]];
    let ready = || {
        let report = TrajectoryWorkerStep {
            slot: 0,
            pid: 1,
            restarted: false,
            warm_deadline: None,
            warm_timeout: false,
            transferred_source: false,
            source_bytes: 0,
            successor_bytes: valid.len(),
            successor_payload_bytes: 0,
            wall: Duration::ZERO,
            profile: None,
        };
        vec![
            (1, report, valid.clone(), vec![want_b.clone()]),
            (0, report, valid.clone(), vec![want_a.clone()]),
        ]
    };
    for kind in 0..7 {
        let mut r = ready();
        match kind {
            0 => r[0].2 = late.clone(),
            1 => r[0].3.clear(),
            2 => r[0].3[0].particles[2].1[2] = f64::NAN,
            3 => r[0].3[0].reactions.clear(),
            4 => r[0].0 = 0,
            5 => {
                r.pop();
            }
            6 => {
                for v in &mut r {
                    v.2[50] ^= 8;
                }
            }
            _ => unreachable!(),
        }
        assert!(
            publish(
                &mut backend,
                &mut [&mut a, &mut b],
                1,
                rows.iter().map(<[_; 1]>::as_slice),
                r
            )
            .is_err()
        );
        assert_eq!(original, backend.checkpoint_trajectory().unwrap());
        assert_eq!(before_a, a.one_way_state().unwrap());
        assert_eq!(before_b, b.one_way_state().unwrap());
    }
    publish(
        &mut backend,
        &mut [&mut a, &mut b],
        1,
        rows.iter().map(<[_; 1]>::as_slice),
        ready(),
    )
    .unwrap();
    assert_eq!(valid, backend.checkpoint_trajectory().unwrap());
    assert_eq!(want_a, a.one_way_state().unwrap());
    assert_eq!(want_b, b.one_way_state().unwrap());
}
