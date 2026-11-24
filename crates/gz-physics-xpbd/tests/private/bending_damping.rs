// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::{Particle, SolverConfig};
use std::time::Duration;

fn square() -> SoftBody {
    let p = [[0., 0., 0.], [0.2, 0., 0.], [0., 0.2, 0.], [0.2, 0.2, 0.]].map(|position| Particle {
        position,
        velocity: [0.; 3],
        inverse_mass: 1.,
    });
    SoftBody::cloth(
        p.to_vec(),
        2,
        0.001,
        0.002,
        0.05,
        SolverConfig {
            gravity: [0.; 3],
            self_contact: false,
            substeps: 1,
            iterations: 8,
            ..Default::default()
        },
    )
    .unwrap()
}

#[test]
fn implicit_viscous_hinge_matches_a_single_angular_rate_and_conserves_momenta() {
    for drag in [1e-15, 0.08, f64::MAX] {
        let mut body = square().with_bending_damping(drag).unwrap();
        body.scratch.clone_from(&body.particles);
        for (i, p) in body.scratch.iter_mut().enumerate() {
            p.inverse_mass = 1. / (i + 1) as f64;
            p.velocity = [0.07, -0.1, [0.3, -0.2, 0.1, 0.4][i]];
        }
        let before = body.scratch.clone();
        let ids = body.bending[0].vertices;
        let coords = ids.map(|i| DVec3::from(before[i].position));
        let grad: [DVec3; 4] = std::array::from_fn(|i| {
            DVec3::from_array(std::array::from_fn(|axis| {
                let mut a = coords;
                let mut b = coords;
                let delta = 1e-7;
                a[i][axis] += delta;
                b[i][axis] -= delta;
                bending::angle_difference(
                    bending::geometry(a).unwrap().0,
                    bending::geometry(b).unwrap().0,
                ) / (2. * delta)
            }))
        });
        let inertia_inv: f64 = ids
            .into_iter()
            .zip(grad)
            .map(|(i, g)| before[i].inverse_mass * g.length_squared())
            .sum();
        let rate: f64 = ids
            .into_iter()
            .zip(grad)
            .map(|(i, g)| g.dot(before[i].velocity.into()))
            .sum();
        let h = 0.001;
        let j = -rate / (inertia_inv + (drag * h).recip());
        body.damp_bending(h).unwrap();
        let mut linear = DVec3::ZERO;
        let mut torque = DVec3::ZERO;
        let mut energy_change = 0.;
        for (id, g) in ids.into_iter().zip(grad) {
            let v = DVec3::from(body.scratch[id].velocity);
            let u = DVec3::from(before[id].velocity);
            let impulse = (v - u) / before[id].inverse_mass;
            assert!(impulse.distance(j * g) < 2e-10);
            linear += impulse;
            torque += DVec3::from(before[id].position).cross(impulse);
            energy_change +=
                (v.length_squared() - u.length_squared()) / (2. * before[id].inverse_mass);
        }
        assert!(linear.length() < 1e-13 && torque.length() < 1e-13);
        assert!(
            energy_change < 1e-14
                && (energy_change - (j * rate + 0.5 * j * j * inertia_inv)).abs() < 1e-10
        );
    }
}

#[test]
fn a_pinned_cloth_corner_matches_the_small_angle_damped_oscillator_on_refinement() {
    let mut initial = square();
    let ids = initial.bending[0].vertices;
    let tip = ids[2]; // Opposite a fixed diagonal of the square.
    for p in &mut initial.particles {
        p.inverse_mass = 0.;
    }
    initial.particles[tip].inverse_mass = 1.;
    initial.particles[tip].velocity[2] = 0.001;
    let lever = 0.2 / (2_f64.sqrt());
    let stiff = 1. / (0.05 * lever * lever);
    let damping = 0.08 / (lever * lever);
    let freq = (stiff - 0.25 * damping * damping).sqrt();
    let time = 0.04;
    let exact_z = 0.001 * (-0.5 * damping * time).exp() * (freq * time).sin() / freq;
    let mut errors = Vec::new();
    for count in [40, 80, 160, 320] {
        let mut b = initial.clone().with_bending_damping(0.08).unwrap();
        for _ in 0..count {
            b.step(Duration::from_nanos(40_000_000 / count), &[])
                .unwrap();
        }
        let z = b.particles()[tip].position[2];
        errors.push((exact_z - z).abs());
    }
    assert!(
        errors.windows(2).all(|p| p[1] < 0.65 * p[0]) && errors[3] < 1e-7,
        "{errors:?}"
    );
}

#[test]
fn a_stationary_or_rigidly_moving_sheet_has_no_relative_bend_to_damp() {
    let mut b = square().with_bending_damping(0.2).unwrap();
    for (i, p) in b.scratch.iter_mut().enumerate() {
        p.velocity = (DVec3::new(1., -2., 3.)
            + DVec3::new(0.3, -0.2, 0.6).cross(p.position.into()))
        .to_array();
        p.inverse_mass = if i == 0 { 0. } else { 1. };
    }
    let before = b.scratch.clone();
    b.damp_bending(0.5).unwrap();
    assert!(
        before
            .iter()
            .zip(&b.scratch)
            .all(|(a, b)| DVec3::from(a.velocity).distance(b.velocity.into()) < 1e-13)
    );
    for p in &mut b.scratch {
        p.inverse_mass = 0.;
    }
    b.damp_bending(0.5).unwrap();
    b.scratch[0].position = b.scratch[1].position;
    assert_eq!(b.damp_bending(0.1), Err(XpbdError::Numerical));
    for invalid in [-0.01, f64::NAN, f64::INFINITY] {
        assert!(square().with_bending_damping(invalid).is_err());
    }
    let particles = SoftBody::new(square().particles, vec![], SolverConfig::default()).unwrap();
    assert!(particles.with_bending_damping(0.2).is_err());
}

#[test]
fn two_prescribed_faces_report_the_viscous_moment_of_an_imposed_bend() {
    let seed = square();
    let mut b = SoftBody::cloth(
        seed.particles,
        2,
        0.001,
        0.002,
        0.05,
        SolverConfig {
            self_contact: true,
            ..SolverConfig::default()
        },
    )
    .unwrap()
    .with_self_contact_friction(0.3)
    .unwrap()
    .with_self_contact_loads()
    .unwrap()
    .with_bending_damping(0.08)
    .unwrap();
    let ids = b.bending[0].vertices;
    for p in &mut b.scratch {
        p.inverse_mass = 0.;
    }
    b.scratch[ids[2]].velocity[2] = 0.4;
    let before = b.scratch.clone();
    let (_, g) = bending::geometry(ids.map(|i| b.scratch[i].position.into())).unwrap();
    let h = 0.002;
    let j = -h * 0.08 * g[2].z * 0.4;
    b.begin_self_contact();
    b.damp_bending(h).unwrap();
    b.publish_self_contact();
    assert_eq!(b.scratch, before);
    for (i, direction) in ids.into_iter().zip(g) {
        assert!(
            b.last_shell_constraint_impulses()[i]
                .bend_damping
                .distance(j * direction)
                < 1e-14
        );
    }
}
