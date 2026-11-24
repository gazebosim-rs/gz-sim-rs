// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/inclined_sheet.rs"]
mod sheet;
use glam::DVec3;
use gz_sim_scene::{
    deformable::articulated_impact::ArticulatedSupportInterval, reset_loaded_world,
};
use sheet::*;
use std::{collections::BTreeSet, time::Duration};

#[derive(Default)]
struct Cut {
    rigid: DVec3,
    normal: [f64; 4],
    tangent: [DVec3; 4],
    max_linear: f64,
    max_angular: f64,
    positive_sliding_power: f64,
    head_virtual_slip: f64,
    end_slip: f64,
}
fn cut(r: &ArticulatedSupportInterval) -> Cut {
    let mut out = Cut::default();
    for (head, a) in r
        .beginning
        .iter()
        .map(|a| (true, a))
        .chain(r.end.iter().map(|a| (false, a)))
    {
        for l in &a.material {
            let i = l.site.sphere;
            let rigid = DVec3::from(l.on_rigid);
            let n = DVec3::from(l.site.normal);
            let f = l.friction.unwrap();
            let tangent = DVec3::from(f.on_material);
            let compression = -rigid.dot(n);
            assert!(
                compression >= -1e-17 && tangent.length() <= f.coefficient * compression + 2e-15
            );
            assert!(rigid.distance(-tangent - compression * n) < 2e-14);
            let mut closure = rigid;
            let mut az = DVec3::from(l.site.point).cross(rigid);
            for &(_, x, j) in &l.on_particles {
                let j = DVec3::from(j);
                closure += j;
                az += DVec3::from(x).cross(j);
            }
            out.max_linear = out.max_linear.max(closure.length());
            out.max_angular = out.max_angular.max(az.length());
            let slip = DVec3::from(f.final_slip);
            let tangent_speed = if head {
                let travel = r.tangential_travel.iter().find(|(s, _)| *s == i).unwrap().1;
                out.head_virtual_slip = out.head_virtual_slip.max(slip.length());
                DVec3::from(travel) / r.duration.as_secs_f64()
            } else {
                out.end_slip = out.end_slip.max(slip.length());
                slip
            };
            out.positive_sliding_power = out.positive_sliding_power.max(tangent_speed.dot(tangent));
            out.rigid += rigid;
            out.normal[i] += compression;
            out.tangent[i] += tangent;
        }
    }
    assert!(
        out.max_linear < 1e-13 && out.max_angular < 1e-13 && out.positive_sliding_power < 3e-12,
        "power {:e}",
        out.positive_sliding_power
    );
    out
}
fn angular(g: &Incline) -> f64 {
    let rigid = g
        .states()
        .into_iter()
        .map(|p| {
            DVec3::from(p.position)
                .cross(0.75 * DVec3::from(p.velocity))
                .z
                + 0.025 * p.angular_velocity[2]
        })
        .sum::<f64>();
    rigid
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
fn four_finite_pads_hold_the_loaded_skin_above_the_local_coefficient_threshold() {
    let mu = critical() + 0.00002;
    let mut old = 1.;
    for width in [500000, 250000, 62500] {
        let mut g = Incline::new(&sdf(mu, 0., width)).unwrap();
        let initial = g.material().particles().to_vec();
        let base = g.states();
        let mut micro = 0f64;
        let initial_ids = g.ids;
        let expected_n = 0.1 * G * normal().z;
        let relative = SKIN * SLOPE / HALF_SPAN;
        let mut worst_x = 0f64;
        let mut worst_j = 0f64;
        let mut worst_end = 0f64;
        for _ in 0..180 {
            g.step().unwrap();
            assert!(g.material().impacts_this_step().is_empty());
            let mut per = [(0., DVec3::ZERO); 4];
            for r in g.material().support_this_step() {
                let c = cut(r);
                micro = micro.max(c.head_virtual_slip);
                worst_end = worst_end.max(c.end_slip);
                for (i, (n, t)) in per.iter_mut().enumerate() {
                    *n += c.normal[i];
                    *t += c.tangent[i];
                }
            }
            for (i, (n, t)) in per.into_iter().enumerate() {
                let sign = if i == 0 || i == 3 { -1. } else { 1. };
                worst_j = worst_j
                    .max((n * 1000. - expected_n * (1. + sign * relative)).abs())
                    .max((t * 1000. + expected_n * SLOPE * downhill()).length());
            }
            for (p, b) in g.material().particles().iter().zip(&initial) {
                worst_x = worst_x.max(DVec3::from(p.position).distance(b.position.into()));
                assert!(DVec3::from(p.velocity).length() < 1e-6);
            }
            assert!(
                g.states()
                    .iter()
                    .zip(base)
                    .all(|(a, b)| DVec3::from(a.position).distance(b.position.into()) < 1e-8)
            );
        }
        println!(
            "Static four-pad threshold {:.12}, mu {mu:.9}, {width} ns width, maximum true nodal displacement {worst_x:e} m, pressure/traction discrepancy {worst_j:e} N, head post-kick slip {micro:e} and endpoint slip {worst_end:e} m/s.",
            critical()
        );
        assert!(worst_x < 5e-9 && worst_j < 5e-6 && worst_end < 1e-8 && micro < 0.7 * old);
        old = micro;
        let saved = g.material().particles().to_vec();
        let rigid = g.states();
        reset_loaded_world(&mut g.sim, |_| ()).unwrap();
        assert_eq!(initial_ids, g.ids);
        g.sim.step_exact(180).unwrap();
        assert_eq!(saved, g.material().particles());
        assert_eq!(rigid, g.states());
    }
}
#[test]
fn kinetic_cloth_crosses_its_internal_seam_and_matches_external_force_and_work() {
    for (mu, v0, end) in [(0.2, 0., 500), (0.45, 0.32, 500), (0.15, -0.42, 480)] {
        for width in [250000, 62500] {
            let mut g = Incline::new(&sdf(mu, v0, width)).unwrap();
            let (zero, v, m) = g.centroid();
            assert!((m - MASS).abs() < 1e-14);
            let mut previous = m * v;
            let mut distance = 0f64;
            let mut speed = 0f64;
            let mut budget = 0f64;
            let mut seam = BTreeSet::new();
            let mut needed = 0;
            let mut measured_switch = Duration::ZERO;
            let mut worst_force = 0f64;
            let mut friction_work = 0.;
            let mut prior_x = 0.;
            let mut last_velocity = v0;
            let mut worst_energy = 0f64;
            for _ in 0..end {
                g.step().unwrap();
                let t = g.sim.elapsed().as_secs_f64();
                let mut applied = DVec3::ZERO;
                let mut jt = DVec3::ZERO;
                for r in g.material().support_this_step() {
                    needed += 1;
                    let c = cut(r);
                    applied -= c.rigid;
                    jt += c.tangent.into_iter().sum::<DVec3>();
                    for a in r.beginning.iter().chain(&r.end) {
                        for l in &a.material {
                            if l.site.sphere == 2 {
                                seam.insert(l.site.feature);
                            }
                        }
                    }
                }
                let (x, v, m) = g.centroid();
                let actual = (x - zero).dot(downhill());
                let speed_now = v.dot(downhill());
                let (expect, v_expected) = planar_motion(t, mu, v0);
                distance = distance.max((actual - expect).abs());
                speed = speed.max((speed_now - v_expected).abs());
                let external = -G * m * 0.001 * DVec3::Z;
                budget = budget.max((m * v - previous - applied - external).length());
                previous = m * v;
                let before = last_velocity;
                if measured_switch.is_zero()
                    && (v0 > 0. && speed_now.abs() < 1e-7 || v0 < 0. && speed_now >= 0.)
                {
                    measured_switch = g.sim.elapsed();
                }
                last_velocity = speed_now;
                let force = jt.dot(downhill()) / 0.001;
                let mid = 0.5 * (before + speed_now);
                if mid.abs() > 0.003 {
                    let f = -mid.signum() * mu * m * G * normal().z;
                    worst_force = worst_force.max((force - f).abs());
                }
                friction_work += force * (actual - prior_x);
                prior_x = actual;
                let mechanical = 0.5 * m * (v.length_squared() - v0 * v0) + m * G * (x.z - zero.z);
                worst_energy = worst_energy.max((mechanical - friction_work).abs());
            }
            println!(
                "mu {mu}, v0 {v0}, {width} ns: maximum CM position/velocity discrepancy {distance:e} m/{speed:e} m/s, force {worst_force:e} N, momentum {budget:e} N s, mechanical work {worst_energy:e} J, first stop/reversal {measured_switch:?}, features {seam:?}, {needed} accepted intervals"
            );
            assert!(
                distance < 2e-4
                    && speed < 1e-3
                    && budget < 2e-12
                    && worst_force < 2e-5
                    && worst_energy < 1e-5
            );
            if v0 == 0. {
                assert!(seam.len() > 1);
            } else {
                let turn = -v0 / acceleration(mu, v0.signum());
                assert!((measured_switch.as_secs_f64() - turn).abs() < 0.0012);
            }
        }
    }
}
#[test]
fn off_center_sheet_drives_the_actual_finite_table_and_closes_angular_momentum() {
    let src = sdf(0.23, 0., 250000).replacen("mass=\"0.1325", "mass=\"0.2425", 1);
    let mut g = Incline::new(&src).unwrap();
    let initial = angular(&g);
    let mut spin = 0f64;
    let mut closure = 0f64;
    for _ in 0..360 {
        g.step().unwrap();
        for r in g.material().support_this_step() {
            cut(r);
        }
        closure = closure.max((angular(&g) - initial).abs());
        spin = spin.max(
            g.states()
                .iter()
                .map(|s| s.angular_velocity[2].abs())
                .fold(0., f64::max),
        );
    }
    println!(
        "Asymmetric sheet maximum native spin {spin:e} rad/s, free-axis angular momentum residual {closure:e} N m s."
    );
    assert!(spin > 1e-4 && closure < 3e-9);
}
