// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_xpbd::{Particle, SoftBody, SolverConfig, XpbdError};
use std::time::Duration;

#[test]
fn gravity_changes_motion_and_invalid_fields_preserve_the_previous_state() {
    let p = Particle {
        position: [0.; 3],
        velocity: [0.; 3],
        inverse_mass: 2.,
    };
    let mut body = SoftBody::new(vec![p], vec![], SolverConfig::default()).unwrap();
    #[cfg(feature = "one-way-checkpoint")]
    let original = body.one_way_identity().unwrap();
    body.set_gravity([0., 1., 0.]).unwrap();
    assert_eq!(
        body.set_gravity([0., f64::NAN, 0.]),
        Err(XpbdError::InvalidConfig)
    );
    assert_eq!(body.config().gravity, [0., 1., 0.]);
    #[cfg(feature = "one-way-checkpoint")]
    assert_ne!(original, body.one_way_identity().unwrap());
    body.step(Duration::from_secs(1), &[]).unwrap();
    assert!((body.particles()[0].velocity[1] - 1.).abs() < 1e-12);
}
