// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/folded.rs"]
mod fixture;
#[allow(dead_code)]
#[path = "../examples/support/rotating_pad.rs"]
mod rotor;
use fixture::*;
use glam::DVec3;
use gz_physics_xpbd::MaterialFeature;
use gz_sim_scene::{deformable::articulated_impact::SupportApplication, reset_loaded_world};
use rotor::Rotor;
fn audit(a: &SupportApplication) -> (f64, f64) {
    let mut p = 0f64;
    let mut ang = 0f64;
    for l in &a.material {
        let j = DVec3::from(l.on_rigid);
        let mut net = j;
        let mut m = DVec3::from(l.site.point).cross(j);
        let n = DVec3::from(l.site.normal);
        let compression = -n.dot(j);
        assert!(compression >= -4e-16);
        if let Some(f) = l.friction {
            let tangent = DVec3::from(f.on_material);
            assert!(tangent.length() <= f.coefficient * compression + 2e-14);
        }
        for &(_, x, v) in &l.on_particles {
            net += DVec3::from(v);
            m += DVec3::from(x).cross(v.into());
        }
        p = p.max(net.length());
        ang = ang.max(m.length());
    }
    (p, ang)
}
#[test]
fn two_fold_faces_share_ridge_mass_and_match_the_instantaneous_normal_force_oracle() {
    let (expected, isolated) = cold_pressure();
    assert!(isolated > 1.4 * expected);
    for width in [250000u64, 62500, 15625] {
        let mut g = Rotor::new(&sdf(0., width, false)).unwrap();
        g.advance(1).unwrap();
        let r = &g.material().support_this_step()[0];
        assert_eq!(
            r.source_features,
            [
                (0, MaterialFeature::Triangle([0, 1, 3])),
                (0, MaterialFeature::Triangle([0, 2, 3]))
            ]
        );
        assert_eq!(r.region_clearances.len(), 2);
        let mut f = [0.; 2];
        let mut k = 0.;
        for a in r.beginning.iter().chain(&r.end) {
            assert_eq!(a.material.len(), 2);
            let q = audit(a);
            assert!(q.0 < 1e-13 && q.1 < 1e-13);
            k = a.native.before.mobility(0, 1).unwrap()[0][0];
            assert!(k < 0.);
            for (i, l) in a.material.iter().enumerate() {
                f[i] +=
                    -DVec3::from(l.on_rigid).dot(l.site.normal.into()) / r.duration.as_secs_f64();
            }
        }
        let relative = f[0] / expected - 1.;
        let theory_k = -(R * normal(1.).y).powi(2) / IZ;
        println!(
            "Cold folded sheet, {width} ns, measured pressure {f:?} N, exact shared-ridge {expected:.12} N, independent-row prediction {isolated:.12} N, relative first-interval error {relative:e}. Measured cross-face x mobility {k:e} m/s per N s vs {theory_k:e}."
        );
        assert!(
            (f[0] - f[1]).abs() < 1e-8 && relative.abs() < 0.004 && (k - theory_k).abs() < 2e-6
        );
        g.advance(499).unwrap();
        assert_eq!(g.material().articulated_impact_count(), 0);
        assert!(
            g.material()
                .support_this_step()
                .iter()
                .all(|c| c.source_features.len() == 2)
        );
        assert!(g.state().angular_velocity[2].abs() < 1e-10);
    }
}
fn angular(g: &Rotor) -> f64 {
    IZ * g.state().angular_velocity[2]
        + g.material()
            .particles()
            .iter()
            .map(|p| {
                DVec3::from(p.position)
                    .cross(DVec3::from(p.velocity) / p.inverse_mass)
                    .z
            })
            .sum::<f64>()
}
#[test]
fn distinct_same_sphere_tangential_rows_spin_a_real_native_rotor_then_release_individually() {
    let mut g = Rotor::new(&sdf(0.32, 250000, true)).unwrap();
    let start = angular(&g);
    let mut axial = 0.;
    let mut max_l = 0f64;
    let mut doubles = 0;
    let mut singles = 0;
    let mut last = 0.;
    let mut min_cross = 0f64;
    let mut worst = (0f64, 0f64);
    while g.sim.elapsed().as_secs_f64() < 0.8 {
        g.advance(1).unwrap();
        for r in g.material().support_this_step() {
            last = (r.start + r.duration).as_secs_f64();
            if r.source_features.len() == 2 {
                doubles += 1;
            } else {
                singles += 1;
            }
            assert!(r.region_clearances.iter().all(|d| *d >= -1.1e-12));
            for (i, &(sphere, feature)) in r.source_features.iter().enumerate() {
                assert_eq!(
                    r.tangent_travel(sphere, feature),
                    Some(r.tangential_travel[i].1)
                );
            }
            for a in r.beginning.iter().chain(&r.end) {
                let q = audit(a);
                worst.0 = worst.0.max(q.0);
                worst.1 = worst.1.max(q.1);
                if a.material.len() == 2 {
                    min_cross = min_cross.min(a.native.before.mobility(0, 1).unwrap()[0][0]);
                }
                for l in &a.material {
                    axial += DVec3::from(l.site.point).cross(l.on_rigid.into()).z;
                }
            }
        }
        max_l = max_l.max((angular(&g) - start).abs());
        assert!((IZ * g.state().angular_velocity[2] - axial).abs() < 2e-10);
    }
    println!(
        "Turning folded sheet: {doubles} double and {singles} single loaded cuts, last {:.9} s. Native omega {:.12} rad/s, total surface moment {axial:.12} N m s, max global axial momentum error {max_l:e}, individual impulse/moment residuals {worst:?}, most negative off-diagonal native response {min_cross:e}.",
        last,
        g.state().angular_velocity[2]
    );
    assert!(
        doubles > 800
            && singles > 0
            && last < 0.7
            && max_l < 2e-9
            && min_cross < -0.1
            && worst.0 < 1e-13
            && worst.1 < 1e-13
            && g.state().angular_velocity[2] > 0.3
    );
    let saved = (g.state(), g.material().particles().to_vec());
    reset_loaded_world(&mut g.sim, |_| ()).unwrap();
    g.advance(800).unwrap();
    assert_eq!(saved, (g.state(), g.material().particles().to_vec()));
}
