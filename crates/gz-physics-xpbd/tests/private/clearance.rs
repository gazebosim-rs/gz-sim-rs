// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::surface::Search;

fn loose_pair(z: f64) -> (Surface, Vec<Particle>) {
    let particles = [
        [0., 0., 0.],
        [1., 0., 0.],
        [0., 1., 0.],
        [0.2, 0.2, z],
        [0.3, 0.2, z],
        [0.2, 0.3, z],
    ]
    .map(|position| Particle {
        position,
        velocity: [0.; 3],
        inverse_mass: 1.,
    })
    .to_vec();
    (
        Surface::new(vec![[0, 1, 2], [3, 4, 5]], &particles).unwrap(),
        particles,
    )
}
#[test]
fn a_shared_translation_reuses_the_bound_and_a_crossing_still_hits() {
    let (mut s, mut p) = loose_pair(0.25);
    let gap = 0.02;
    let mut x: Vec<_> = p.iter().map(|n| DVec3::from(n.position)).collect();
    let cert = s.clearance.as_mut().unwrap();
    assert!(cert.clear(&x, &p, gap, 1e-9));
    assert!(!cert.may_rebuild);
    for (i, node) in p.iter_mut().enumerate() {
        node.position = (DVec3::from(node.position) + DVec3::new(10., -1., 0.5)).to_array();
        x[i] = DVec3::from(node.position) + DVec3::new(0.1, 0.3, 0.);
    }
    assert!(cert.clear(&x, &p, gap, 1e-9));
    for node in &mut x[3..] {
        node.z -= 0.3;
    }
    assert!(!cert.clear(&x, &p, gap, 1e-9));
    let mut ordinary = s.clone();
    ordinary.clearance = None;
    for mode in [
        Search::Ordinary,
        Search::Supported,
        Search::TerminalVelocity,
    ] {
        let find = |s: &mut Surface| {
            s.next_hit(&x, &p, gap, 1e-9, (&mut 0, &mut 0), mode)
                .unwrap()
                .map(|v| (v.time, v.stencil.ids, v.normal))
        };
        let hit = find(&mut ordinary);
        assert!(hit.is_some());
        assert_eq!(find(&mut s), hit);
    }
}

#[test]
fn incomplete_or_degenerate_certificates_cannot_leak_into_later_queries() {
    let (mut s, mut p) = loose_pair(0.25);
    let c = s.clearance.as_mut().unwrap();
    let x: Vec<_> = p.iter().map(|n| DVec3::from(n.position)).collect();
    assert!(!c.clear(&x, &p, 0.26, 1e-12));
    assert!(!c.clear(&x, &p, 0.001, 1e-12));
    for _ in 0..32 {
        c.begin_step();
    }
    assert!(c.clear(&x, &p, 0.001, 1e-12));
    p[3].position = p[0].position;
    let x: Vec<_> = p.iter().map(|n| DVec3::from(n.position)).collect();
    for _ in 0..32 {
        c.begin_step();
    }
    assert!(!c.clear(&x, &p, 0.001, 1e-12));
    let tri = Surface::new(vec![[0, 1, 2]], &p[..3]).unwrap();
    assert!(tri.clearance.is_none());
}

#[test]
fn a_clear_shell_skips_projection_but_a_new_crossing_is_verified() {
    let (mut shell, mut particles) = loose_pair(0.25);
    let mut x: Vec<_> = particles.iter().map(|p| DVec3::from(p.position)).collect();
    let before = x.clone();
    shell
        .project(&mut x, &mut particles, 0.02, 0.001, false)
        .unwrap();
    assert_eq!(x, before);
    shell.verify(&x, &particles, 0.02).unwrap();
    assert!(!shell.clearance.as_ref().unwrap().may_rebuild);
    for q in &mut x[3..] {
        q.z -= 0.3;
    }
    assert_eq!(
        shell.verify(&x, &particles, 0.02),
        Err(crate::XpbdError::UnresolvedSurface)
    );
}

#[test]
fn larger_shells_keep_the_spatial_sweep() {
    let particles: Vec<_> = (0..12)
        .flat_map(|i| {
            loose_pair(0.25).1[..3]
                .iter()
                .map(|p| {
                    let mut p = *p;
                    p.position[0] += 2. * f64::from(i);
                    p
                })
                .collect::<Vec<_>>()
        })
        .collect();
    let triangles = (0..12).map(|i| [3 * i, 3 * i + 1, 3 * i + 2]).collect();
    let surface = Surface::new(triangles, &particles).unwrap();
    assert!(surface.features.len() > 64);
    assert!(surface.clearance.is_none());
}
