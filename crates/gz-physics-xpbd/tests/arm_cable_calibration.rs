// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_xpbd::{Particle, SoftBody, SolverConfig};
use std::time::Duration;

const SECTIONS: usize = 10;
const SPACING: f64 = 0.022;
const NODE_MASS: f64 = 0.002;
const GRAVITY: f64 = 9.81;
const STRETCH: f64 = 0.00001;
const CHORD: f64 = 0.003;

/// A horizontal cut carries the weight of all nodes below it. With section
/// extensions e[i], its edge contributes e[i]/STRETCH and each spanning chord
/// contributes (e[i] + e[neighbor])/CHORD. Eliminate the tridiagonal system.
fn equilibrium() -> [f64; SECTIONS] {
    let mut diagonal = [1. / STRETCH; SECTIONS];
    let mut load = [0.; SECTIONS];
    let off = 1. / CHORD;
    for i in 0..SECTIONS {
        diagonal[i] += off * (usize::from(i > 0) + usize::from(i + 1 < SECTIONS)) as f64;
        load[i] = (SECTIONS - i) as f64 * NODE_MASS * GRAVITY;
        if i > 0 {
            let multiplier = off / diagonal[i - 1];
            diagonal[i] -= multiplier * off;
            load[i] -= multiplier * load[i - 1];
        }
    }
    for i in (0..SECTIONS).rev() {
        if i + 1 < SECTIONS {
            load[i] -= off * load[i + 1];
        }
        load[i] /= diagonal[i];
    }
    load
}

#[test]
fn hanging_arm_cable_extension_converges_towards_independent_static_solution() {
    let sections = equilibrium();
    let reference = sections.iter().sum::<f64>();
    let rest = SPACING * SECTIONS as f64;
    let mut previous = f64::INFINITY;
    for (substeps, iterations, max_error) in
        [(1, 8, 45e-6), (2, 8, 9e-6), (4, 8, 0.8e-6), (4, 32, 0.1e-6)]
    {
        let points = (0..=SECTIONS)
            .map(|i| Particle {
                position: [0., 0., -(i as f64) * SPACING],
                velocity: [0.; 3],
                inverse_mass: if i == 0 { 0. } else { 1. / NODE_MASS },
            })
            .collect();
        let mut body = SoftBody::cable(
            points,
            STRETCH,
            CHORD,
            SolverConfig {
                substeps,
                iterations,
                gravity: [0., 0., -GRAVITY],
                particle_radius: 0.025,
                self_contact: false,
                friction: 0.7,
            },
        )
        .unwrap();
        for _ in 0..500 {
            body.step(Duration::from_millis(1), &[]).unwrap();
        }
        let measured = -body.particles()[SECTIONS].position[2] - rest;
        let error = (measured - reference).abs();
        assert_eq!(body.particles()[0].position, [0.; 3]);
        assert!(body.particles().iter().all(|p| p.velocity[2].abs() < 1e-9));
        eprintln!(
            "{substeps} substeps, {iterations} iterations: tip {measured:.12} m, reference {reference:.12} m, error {error:.12} m"
        );
        assert!(error < max_error && error < previous);
        previous = error;
    }
}
