// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One measured public millisecond from the falling seven-column sheet.
use super::*;
use crate::{Particle, step_coupled_rigid};
use glam::DVec3;
use std::collections::BTreeMap;

#[path = "../fixtures/fold_slide_737.rs"]
mod data;

fn body() -> SoftBody {
    let nodes: Vec<_> = data::INCOMING
        .iter()
        .zip(data::REST)
        .map(|(p, position)| Particle {
            position,
            velocity: [0.; 3],
            inverse_mass: p.inverse_mass,
        })
        .collect();
    let distances = BTreeMap::from(data::DISTANCE_COMPLIANCE);
    let hinges = BTreeMap::from(data::HINGE_COMPLIANCE);
    let pair = |a: usize, b: usize| [a.min(b), a.max(b)];
    let mut body = SoftBody::cloth_with_material_laws(
        nodes,
        7,
        data::CONFIG,
        |a, b| distances[&pair(a, b)],
        |[a, b, _, _]| hinges[&pair(a, b)],
    )
    .unwrap()
    .with_self_contact_friction(0.3)
    .unwrap()
    .with_self_contact_loads()
    .unwrap()
    .with_pad_samples(data::JAW_SAMPLES.to_vec())
    .unwrap()
    .with_pad_loads();
    body.particles.copy_from_slice(&data::INCOMING);
    body
}

#[test]
fn saved_sheet_starts_clear_and_uses_the_measured_loads() {
    let mut sheet = body();
    assert!(
        sheet
            .surface
            .as_mut()
            .unwrap()
            .starts_clear(&sheet.particles, 2. * sheet.config.particle_radius)
            .unwrap()
    );
    assert_eq!(sheet.particles.len(), 28);
    assert_eq!(sheet.config.substeps, 32);
    sheet.surface.as_mut().unwrap().subdivide_on_budget = false;
    assert!(!splittable(&XpbdError::SurfaceBudget, &sheet));
    sheet.surface.as_mut().unwrap().subdivide_on_budget = true;
    assert!(splittable(&XpbdError::SurfaceBudget, &sheet));
    assert!(!splittable(&XpbdError::Numerical, &sheet));
}

#[test]
#[ignore = "time and compare three native solves from the recorded cut in release mode"]
fn matched_supported_shell_refines_the_737th_millisecond() {
    let dt = Duration::from_millis(1);
    for (base, order, expected_impulse) in [
        (32, RowOrder::Time, data::CANDIDATE_SUPPORT_NS),
        (32, RowOrder::TerminalSpeed, data::LEGACY_SUPPORT_NS),
        (512, RowOrder::CertifiedSpeed, data::FINE_SUPPORT_NS),
    ] {
        let mut sheet = body();
        let began = std::time::Instant::now();
        if order == RowOrder::Time {
            // The candidate must pass through its normal public entry point.
            step_coupled_rigid(&mut [&mut sheet], dt, &data::BOUNDARIES, &data::MOBILITY).unwrap();
        } else {
            let surface = sheet.surface.as_mut().unwrap();
            surface.retry_plane = true;
            surface.row_order = order;
            let mut retry = ShellRetry {
                remaining_splits: 1024 - base,
                deepest: 0,
            };
            solve_at_resolution(
                &mut [&mut sheet],
                dt,
                &data::BOUNDARIES,
                &data::MOBILITY,
                base.into(),
                Some(&mut retry),
            )
            .unwrap();
        }
        let elapsed = began.elapsed();
        let support = DVec3::from(sheet.last_boundary_impulses()[2].linear);
        let reference = DVec3::from(data::FINE_SUPPORT_NS);
        let mut max_position: f64 = 0.;
        let mut velocity_sum = 0.;
        let mut mass = 0.;
        for (p, (x, v)) in sheet.particles.iter().zip(data::FINE_MOTION) {
            max_position = max_position.max(DVec3::from(p.position).distance(DVec3::from(x)));
            velocity_sum +=
                DVec3::from(p.velocity).distance_squared(DVec3::from(v)) / p.inverse_mass;
            mass += p.inverse_mass.recip();
        }
        let speed = (velocity_sum / mass).sqrt();
        eprintln!(
            "grid {base} public={} elapsed={elapsed:?} maximum position error={max_position:e} m, mass-weighted rms velocity error={speed:e} m/s, support impulse error={:e} N s",
            order == RowOrder::Time,
            support.distance(reference)
        );
        assert!(support.distance(DVec3::from(expected_impulse)) < 1e-12);
        let (position_limit, speed_limit) = match order {
            RowOrder::Time => (15.5e-6, 0.0464),
            RowOrder::TerminalSpeed => (21.6e-6, 0.0601),
            RowOrder::CertifiedSpeed => (1e-12, 1e-9),
        };
        assert!(max_position < position_limit && speed < speed_limit);
    }
}
