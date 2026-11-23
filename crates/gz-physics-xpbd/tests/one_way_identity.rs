// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "one-way-checkpoint")]
use gz_physics_xpbd::{Particle, SoftBody, SolverConfig};
use std::time::Duration;

fn cable(mass: f64, gravity: f64, compliance: f64) -> SoftBody {
    let points = (0..8)
        .map(|i| Particle {
            position: [0.1 * i as f64, 0., 0.],
            velocity: [0., 0.1 * i as f64, 0.],
            inverse_mass: 1. / mass,
        })
        .collect();
    SoftBody::cable(
        points,
        compliance,
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
fn immutable_equations_survive_warm_transfer_and_separate_equal_length_materials() {
    let mut body = cable(0.1, -9.8, 1e-6);
    let id = body.one_way_identity().unwrap();
    let seed = body.one_way_construction().unwrap();
    let mut fresh = SoftBody::from_one_way_construction(&seed).unwrap();
    assert_eq!(fresh.one_way_identity().unwrap(), id);
    for _ in 0..47 {
        body.step(Duration::from_millis(1), &[]).unwrap();
    }
    assert_eq!(body.one_way_identity().unwrap(), id);
    fresh
        .install_one_way_state(body.one_way_state().unwrap())
        .unwrap();
    assert_eq!(fresh.one_way_identity().unwrap(), id);
    assert_ne!(
        fresh
            .clone()
            .with_angular_momentum_conservation()
            .one_way_identity()
            .unwrap(),
        id
    );
    for changed in [
        cable(0.2, -9.8, 1e-6),
        cable(0.1, -9.7, 1e-6),
        cable(0.1, -9.8, 2e-6),
    ] {
        assert_ne!(changed.one_way_identity().unwrap(), id);
    }
}
