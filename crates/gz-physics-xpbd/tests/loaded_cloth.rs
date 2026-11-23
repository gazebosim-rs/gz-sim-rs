// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_xpbd::{Particle, SoftBody, SolverConfig, XpbdError};
fn square() -> Vec<Particle> {
    [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]]
        .map(|position| Particle {
            position,
            velocity: [0.; 3],
            inverse_mass: 2.,
        })
        .to_vec()
}
#[test]
fn loaded_cloth_retains_authored_rest_lengths_and_pose_with_selected_row_laws() {
    let rest = square();
    let mut loaded = rest.clone();
    loaded[2].position[2] = 0.2;
    loaded[0].velocity = [1., 2., 3.];
    let s = SoftBody::cloth_from_rest(
        loaded.clone(),
        &rest,
        2,
        SolverConfig::default(),
        |a, b| (1 + a + b) as f64 * 0.001,
        |_| 0.02,
    )
    .unwrap();
    assert_eq!(s.particles(), &loaded);
    for edge in s.constraints() {
        assert_eq!(edge.compliance, (1 + edge.a + edge.b) as f64 * 0.001);
        if edge.a == 0 && edge.b == 2 {
            assert_eq!(edge.rest_length, 1.);
        }
    }
    assert_eq!(s.bending_constraints()[0].compliance, 0.02);
    let energy = s.mechanical_energy().unwrap();
    assert!(energy.distance > 0. && energy.bending > 0.);
    assert_eq!(s.last_self_contact_impulse().normal, 0.);
}
#[test]
fn loaded_cloth_rejects_unmatched_or_degenerate_input_and_invalid_laws() {
    let rest = square();
    let build = |loaded: Vec<Particle>, reference: &[Particle], edge, hinge| {
        SoftBody::cloth_from_rest(
            loaded,
            reference,
            2,
            SolverConfig::default(),
            |_, _| edge,
            |_| hinge,
        )
    };
    assert!(matches!(
        build(rest.clone(), &rest[..3], 1., 1.),
        Err(XpbdError::InvalidConstraint)
    ));
    let mut flat = rest.clone();
    flat[2].position = flat[3].position;
    assert!(matches!(
        build(flat, &rest, 1., 1.),
        Err(XpbdError::InvalidConstraint)
    ));
    assert!(matches!(
        build(rest.clone(), &rest, -0.1, 1.),
        Err(XpbdError::InvalidConstraint)
    ));
    assert!(matches!(
        build(rest.clone(), &rest, 1., f64::NAN),
        Err(XpbdError::InvalidConstraint)
    ));
    assert!(matches!(
        build(rest.clone(), &rest, 1., -0.1),
        Err(XpbdError::InvalidConstraint)
    ));
}
