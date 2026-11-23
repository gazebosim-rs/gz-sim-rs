// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{Particle, SoftBody, SolverConfig};
use std::time::Duration;

fn config(n: u32) -> SolverConfig {
    SolverConfig {
        gravity: [0.; 3],
        substeps: n,
        iterations: 16,
        self_contact: false,
        friction: 0.,
        ..Default::default()
    }
}
fn particle(p: DVec3, v: DVec3, m: f64) -> Particle {
    Particle {
        position: p.to_array(),
        velocity: v.to_array(),
        inverse_mass: m.recip(),
    }
}
fn invariant(ps: &[Particle]) -> (DVec3, DVec3, f64) {
    let mut linear = DVec3::ZERO;
    let mut angular = DVec3::ZERO;
    let mut energy = 0.;
    for p in ps.iter().filter(|p| p.inverse_mass > 0.) {
        let v = DVec3::from(p.velocity);
        let mv = v / p.inverse_mass;
        linear += mv;
        angular += DVec3::from(p.position).cross(mv);
        energy += mv.dot(v) * 0.5;
    }
    (linear, angular, energy)
}
#[test]
fn spinning_rod_preserves_the_angular_momentum_and_energy_of_free_or_supported_rotation() {
    let axis = DVec3::new(1., -0.4, 0.7).normalize();
    let arm = axis.any_orthonormal_vector() * 0.7;
    let omega = axis * 6.;
    let origin = DVec3::new(0.3, 0.1, -0.2);
    for pinned in [false, true] {
        let fraction = if pinned { 0. } else { 0.65 };
        let x = [-fraction * arm, (1. - fraction) * arm];
        let mut ps = vec![
            particle(origin + x[0], omega.cross(x[0]), 0.035),
            particle(origin + x[1], omega.cross(x[1]), 0.065),
        ];
        if pinned {
            ps[0].inverse_mass = 0.;
        }
        let mut fine = Vec::new();
        for n in [1, 2, 4, 8] {
            let mut rod = SoftBody::cable(ps.clone(), 0., 0., config(n))
                .unwrap()
                .with_angular_momentum_conservation();
            let (p0, l0, e0) = invariant(&ps);
            // For the supported system, measure angular momentum about the fixed pivot.
            let l0 = l0 - origin.cross(p0);
            let mut dl = 0f64;
            let mut de = 0f64;
            let mut stretch = 0f64;
            for _ in 0..400 {
                rod.step(Duration::from_millis(10), &[]).unwrap();
                let v = rod.particles();
                let (p, l, e) = invariant(v);
                dl = dl.max((l - origin.cross(p)).distance(l0));
                de = de.max((e - e0).abs());
                stretch = stretch
                    .max((DVec3::from(v[0].position).distance(v[1].position.into()) - 0.7).abs());
                if pinned {
                    assert_eq!(v[0], ps[0]);
                } else {
                    assert!(p.distance(p0) < 1e-12);
                }
            }
            let oracle = origin + DQuat::from_axis_angle(axis, 24.) * x[1];
            let pos = DVec3::from(rod.particles()[1].position);
            let error = pos.distance(oracle);
            println!(
                "free rotor pinned={pinned} substeps={n}: dL={dl:e}, dE={de:e}, extension={stretch:e} m, trajectory error at 4 s={error:e} m"
            );
            assert!(dl < 1e-11 && de < 1e-10 && stretch < 1e-11 && error < 0.012);
            fine.push(error);
        }
        assert!(fine.windows(2).all(|e| e[1] / e[0] < 0.27));
    }
}

#[test]
fn a_deforming_unequal_mass_sheet_conserves_linear_and_angular_momentum_with_compliant_hinges() {
    let mut ps = Vec::new();
    for r in 0..3 {
        for c in 0..4 {
            let i = (4 * r + c) as f64;
            let x = DVec3::new(c as f64 * 0.12, r as f64 * 0.15, 0.03 * (0.72 * i).sin());
            let v = DVec3::new(
                0.7 * (1.22 * i).sin(),
                0.33 * (0.45 * i).cos(),
                1.2 * (0.87 * i).sin(),
            );
            ps.push(particle(x, v, 0.009 + 0.002 * (i % 4.)));
        }
    }
    let mut drift = Vec::new();
    for conserve in [false, true] {
        let body = SoftBody::cloth(ps.clone(), 4, 0.00015, 0.00018, 0.001, config(2)).unwrap();
        let mut body = if conserve {
            body.with_angular_momentum_conservation()
        } else {
            body
        };
        let (p0, l0, _) = invariant(&ps);
        let mut dl = 0f64;
        let mut dp = 0f64;
        for _ in 0..300 {
            body.step(Duration::from_millis(2), &[]).unwrap();
            let (p, l, _) = invariant(body.particles());
            dl = dl.max(l.distance(l0));
            dp = dp.max(p.distance(p0));
        }
        println!("freely deforming cloth conserve={conserve} dP={dp:e} dL={dl:e}");
        assert!(dp < 1e-12);
        drift.push(dl);
    }
    assert!(drift[0] > 1e-6 && drift[1] < 1e-12);
}

#[test]
fn large_transverse_travel_uses_a_finer_clock_and_refuses_an_unrepresentable_budget_atomically() {
    for omega in [90., 1e9] {
        let ps = vec![
            particle(-0.5 * DVec3::Y, -0.5 * omega * DVec3::Z, 1.),
            particle(0.5 * DVec3::Y, 0.5 * omega * DVec3::Z, 1.),
        ];
        let mut rod = SoftBody::cable(ps.clone(), 0., 0., config(1))
            .unwrap()
            .with_angular_momentum_conservation();
        let step = rod.step(Duration::from_millis(20), &[]);
        if omega > 1e8 {
            assert_eq!(step, Err(gz_physics_xpbd::XpbdError::UnresolvedStructure));
            assert_eq!(rod.particles(), ps);
            continue;
        }
        step.unwrap();
        let (_, l0, e0) = invariant(&ps);
        let (_, l, e) = invariant(rod.particles());
        let wanted = DQuat::from_rotation_x(omega * 0.02) * (DVec3::Y * 0.5);
        assert!(l.distance(l0) < 1e-9 && (e - e0).abs() < 1e-8);
        assert!(wanted.distance(rod.particles()[1].position.into()) < 0.015);
    }
}

#[test]
fn ideal_sheet_tracks_its_rotated_reference_while_preserving_spin_and_kinetic_energy() {
    let q = DQuat::from_euler(glam::EulerRot::XYZ, 0.71, -0.28, 0.9);
    let omega = q * (DVec3::Z * 8.);
    let origin = DVec3::new(0.8, -0.4, 0.62);
    let mut ps = vec![];
    for row in -1..=1 {
        for col in -1..=1 {
            let r = q * DVec3::new(0.18 * row as f64, 0.13 * col as f64, 0.);
            ps.push(particle(origin + r, omega.cross(r), 0.02));
        }
    }
    let mut body = SoftBody::cloth(ps.clone(), 3, 0., 0., 0., config(1))
        .unwrap()
        .with_angular_momentum_conservation();
    let (_, l0, e0) = invariant(&ps);
    for _ in 0..120 {
        body.step(Duration::from_millis(5), &[]).unwrap();
    }
    let (p, l, e) = invariant(body.particles());
    let rotation = DQuat::from_scaled_axis(omega * 0.6);
    let error = body
        .particles()
        .iter()
        .zip(&ps)
        .map(|(actual, initial)| {
            DVec3::from(actual.position)
                .distance(origin + rotation * (DVec3::from(initial.position) - origin))
        })
        .fold(0., f64::max);
    assert!(p.length() < 1e-11 && l.distance(l0) < 1e-11 && (e - e0).abs() < 1e-6 && error < 0.001);
}
