// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::{Particle, SolverConfig};

#[test]
fn energy_uses_finite_particle_masses_and_the_rest_lengths_and_fold_angle() {
    let points =
        [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]].map(|position| Particle {
            position,
            velocity: [0.; 3],
            inverse_mass: 0.5,
        });
    let mut sheet =
        SoftBody::cloth(points.to_vec(), 2, 0.5, 1., 0.25, SolverConfig::default()).unwrap();
    assert_eq!(
        sheet.mechanical_energy().unwrap(),
        MaterialEnergy::default()
    );
    // Hinge about the common diagonal. Moving this one corner straight up
    // makes both of its unit edges sqrt(2) and tilts its face by atan(sqrt(2)).
    sheet.particles[1].position[2] = 1.;
    sheet.particles[1].velocity = [0., 2., 0.];
    sheet.particles[0].inverse_mass = 0.;
    sheet.particles[0].velocity = [8., 0., 0.];
    let energy = sheet.mechanical_energy().unwrap();
    assert!((energy.kinetic - 4.).abs() < 1e-12);
    assert!((energy.gravitational - 19.62).abs() < 1e-12);
    assert!((energy.distance - 2. * (2f64.sqrt() - 1.).powi(2)).abs() < 1e-12);
    assert!((energy.bending - 2. * 2f64.sqrt().atan().powi(2)).abs() < 1e-12);
    assert_eq!(
        energy.total(),
        energy.kinetic + energy.gravitational + energy.distance + energy.bending
    );
    sheet.particles[1].position = sheet.particles[0].position;
    assert!(matches!(
        sheet.mechanical_energy(),
        Err(XpbdError::Numerical)
    ));
}

#[test]
fn ideal_constraints_have_no_stored_spring_energy_and_overflow_is_reported() {
    let points =
        [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]].map(|position| Particle {
            position,
            velocity: [0.; 3],
            inverse_mass: 1.,
        });
    let mut sheet =
        SoftBody::cloth(points.to_vec(), 2, 0., 0., 0., SolverConfig::default()).unwrap();
    sheet.particles[1].position[0] = 2.;
    assert_eq!(
        sheet.mechanical_energy().unwrap(),
        MaterialEnergy::default()
    );
    sheet.particles[1].velocity[0] = f64::MAX;
    assert!(matches!(
        sheet.mechanical_energy(),
        Err(XpbdError::Numerical)
    ));
}
