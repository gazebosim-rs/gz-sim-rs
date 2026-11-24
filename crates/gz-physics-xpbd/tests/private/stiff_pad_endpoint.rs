// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::DVec3;

#[path = "../fixtures/stiff_pad_40926.rs"]
mod cut;

#[test]
fn a_loaded_folded_edge_can_finish_under_two_spring_pads_at_the_measured_cut() {
    let dt = std::time::Duration::from_nanos(15_625);
    let nodes = cut::INCOMING
        .into_iter()
        .zip(cut::REST)
        .map(|(p, position)| Particle {
            position,
            velocity: [0.; 3],
            ..p
        })
        .collect();
    let mut base = SoftBody::cloth(nodes, 3, 0.00001, 0.00002, 0.02, cut::CONFIG)
        .unwrap()
        .with_self_contact_friction(0.3)
        .unwrap()
        .with_pad_loads();
    base.particles.copy_from_slice(&cut::INCOMING);
    let momentum = |p: &[Particle]| {
        p.iter()
            .map(|p| DVec3::from(p.velocity) / p.inverse_mass)
            .sum::<DVec3>()
    };
    let initial = momentum(&base.particles);
    let gravity = dt.as_secs_f64()
        * DVec3::from(cut::CONFIG.gravity)
        * cut::INCOMING
            .iter()
            .map(|p| 1. / p.inverse_mass)
            .sum::<f64>();
    let mut solved = vec![];
    for substeps in [1, 2, 4, 8] {
        let mut body = base.clone();
        body.config.substeps = substeps;
        step_coupled_rigid(&mut [&mut body], dt, &cut::BOUNDARIES, &cut::MOBILITY).unwrap();
        let applied = body
            .last_boundary_impulses()
            .iter()
            .map(|j| DVec3::from(j.linear))
            .sum::<DVec3>();
        let residual = (momentum(body.particles()) - initial - gravity + applied).length();
        assert!(residual < 1e-13, "momentum residual {residual}");
        let contact = body.last_self_contact_impulse();
        assert!(contact.normal > 0. && contact.friction_work >= 0.);
        assert!(contact.tangent <= 0.3 * contact.normal + 1e-15);
        assert!(body.last_pad_loads().iter().any(|p| p.boundary == 0));
        assert!(body.last_pad_loads().iter().any(|p| p.boundary == 1));
        solved.push(body.particles().to_vec());
    }
    for coarse in &solved[..3] {
        for (p, reference) in coarse.iter().zip(solved.last().unwrap()) {
            assert!(DVec3::from(p.position).distance(reference.position.into()) < 1e-6);
        }
    }
}
