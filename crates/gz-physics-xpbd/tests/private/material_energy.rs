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

#[test]
fn reported_elastic_forces_match_energy_gradients_on_each_live_node() {
    let rest = [[0., 0., 0.], [0.4, 0., 0.], [0., 0.3, 0.], [0.4, 0.3, 0.1]];
    let particles = rest.map(|position| Particle {
        position,
        velocity: [0.; 3],
        inverse_mass: 2.,
    });
    let mut sheet = SoftBody::cloth(
        particles.to_vec(),
        2,
        0.002,
        0.008,
        0.04,
        SolverConfig::default(),
    )
    .unwrap();
    sheet.particles[1].position = [0.44, 0.016, 0.07];
    sheet.particles[3].position = [0.395, 0.34, 0.11];
    sheet.particles[0].inverse_mass = 0.;
    let forces = sheet.elastic_node_forces().unwrap();
    for (i, f) in forces.iter().enumerate() {
        for axis in 0..3 {
            let old = sheet.particles[i].position[axis];
            let epsilon = 1e-6;
            sheet.particles[i].position[axis] = old - epsilon;
            let lo = sheet.mechanical_energy().unwrap();
            sheet.particles[i].position[axis] = old + epsilon;
            let hi = sheet.mechanical_energy().unwrap();
            sheet.particles[i].position[axis] = old;
            for (analytic, central) in [
                (
                    f.distance[axis],
                    (lo.distance - hi.distance) / (2. * epsilon),
                ),
                (f.bending[axis], (lo.bending - hi.bending) / (2. * epsilon)),
            ] {
                assert!(
                    (analytic - central).abs() < 1e-7,
                    "node {i}, axis {axis}: {analytic}, {central}"
                );
            }
        }
    }
    for contribution in [
        forces.iter().map(|f| f.distance).collect::<Vec<_>>(),
        forces.iter().map(|f| f.bending).collect(),
    ] {
        let sum: DVec3 = contribution.iter().sum();
        let torque: DVec3 = contribution
            .iter()
            .zip(sheet.particles())
            .map(|(f, p)| DVec3::from(p.position).cross(*f))
            .sum();
        assert!(sum.length() < 1e-13 && torque.length() < 1e-13);
    }
    sheet.particles[1].position = sheet.particles[0].position;
    assert_eq!(sheet.elastic_node_forces(), Err(XpbdError::Numerical));
}

#[test]
fn ideal_constraints_and_undefined_hinge_gradients_are_explicit() {
    let nodes = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]].map(|position| Particle {
        position,
        velocity: [0.; 3],
        inverse_mass: 1.,
    });
    let mut sheet =
        SoftBody::cloth(nodes.to_vec(), 2, 0., 0., 0., SolverConfig::default()).unwrap();
    sheet.particles[0].position[2] = 0.1;
    assert_eq!(
        sheet.elastic_node_forces().unwrap(),
        vec![MaterialElasticForce::default(); 4]
    );
    sheet.bending[0].compliance = 0.1;
    let p = sheet.bending[0]
        .vertices
        .map(|i| DVec3::from(sheet.particles[i].position));
    let (angle, _) = bending::geometry(p).unwrap();
    sheet.bending[0].rest_angle = angle - std::f64::consts::PI;
    assert_eq!(sheet.elastic_node_forces(), Err(XpbdError::Numerical));
    sheet
        .particles
        .iter_mut()
        .for_each(|p| p.position = [0.; 3]);
    assert_eq!(sheet.elastic_node_forces(), Err(XpbdError::Numerical));
    let mut stretched =
        SoftBody::cloth(nodes.to_vec(), 2, 0.001, 0.001, 0., SolverConfig::default()).unwrap();
    stretched.particles[1].position[1] = f64::MAX;
    assert_eq!(stretched.elastic_node_forces(), Err(XpbdError::Numerical));
}
