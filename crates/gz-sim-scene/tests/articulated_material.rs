// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/articulated_mobility.rs"]
mod fixture;
#[path = "support/gripper_cartesian.rs"]
mod independent;
use fixture::*;
use glam::DVec3;
use gz_physics_xpbd::Particle;
use gz_sim_scene::{
    deformable::{self, articulated_impact},
    reset_loaded_world,
};

const CLOTH: &str = include_str!("../../../examples/articulated_material_cloth_impact.sdf");
const CABLE: &str = include_str!("../../../examples/articulated_material_cable_impact.sdf");
const NORMAL: DVec3 = DVec3::new(0.28, 0., 0.96);
const TANGENT: DVec3 = DVec3::new(0.96, 0., -0.28);

fn sample(grip: &Gripper) -> &deformable::DeformableSample {
    deformable::bodies(&grip.sim).next().unwrap().1
}
fn pbudget(nodes: &[Particle]) -> (DVec3, DVec3, f64) {
    nodes
        .iter()
        .fold((DVec3::ZERO, DVec3::ZERO, 0.), |(p, l, e), n| {
            let v = DVec3::from(n.velocity);
            let m = v / n.inverse_mass;
            (
                p + m,
                l + DVec3::from(n.position).cross(m),
                e + 0.5 * v.dot(m),
            )
        })
}

#[test]
fn two_fingers_strike_real_face_and_segment_interiors_with_the_full_cartesian_response() {
    for (xml, cloth) in [(CLOTH, true), (CABLE, false)] {
        let mut g = Gripper::new(xml).unwrap();
        assert!(sample(&g).latest_articulated_impact().is_none());
        let mut free =
            Gripper::new(&xml.replace(r#"time_ns="100000000""#, r#"time_ns="101000000""#)).unwrap();
        free.sim.step_exact(100).unwrap();
        let before = sample(&free).particles();
        let state0 = free.states();
        g.sim
            .step_exact(100)
            .unwrap_or_else(|e| panic!("{e}: {:?}", g.sim.fault()));
        let receipt = sample(&g).latest_articulated_impact().unwrap();
        assert_eq!(receipt.time.as_nanos(), 100_000_000);
        assert_eq!(receipt.links, [g.ids[1].0, g.ids[2].0]);
        let loads = &receipt.material.loads;
        assert_eq!(loads.len(), 2);
        let pts = [
            [0.; 3],
            (NORMAL * 0.1).to_array(),
            (NORMAL * 0.1).to_array(),
        ];
        let operator = independent::cartesian(state0, CENTER, false, pts);
        let mut rigid = [[0.; 2]; 2];
        let ii = if cloth {
            vec![0.085, 0.11, 0.14, 0.095]
        } else {
            vec![0.18, 0.25]
        };
        let mut weights = vec![vec![0.; ii.len()]; 2];
        // Closed form at x midpoint and independent linear barycentric equations.
        for (row, y) in [0.29, -0.29].into_iter().enumerate() {
            if cloth {
                let b = (y + 0.49) / 0.98;
                weights[row] = if b > 0.5 {
                    vec![1. - b, 0., b - 0.5, 0.5]
                } else {
                    vec![0.5, 0.5 - b, 0., b]
                };
            } else {
                weights[row] = vec![0.5 - y, 0.5 + y];
            }
        }
        let mut k = [[0.; 2]; 2];
        for i in 0..2 {
            let load = &loads[i];
            assert_eq!(load.site.sphere, i);
            assert_eq!(
                load.site.feature.vertices().len(),
                if cloth { 3 } else { 2 }
            );
            assert!(DVec3::from(load.site.normal).distance(NORMAL) < 2e-11);
            assert!(DVec3::from(load.site.point).distance(pose(state0[i + 1], pts[i + 1])) < 2e-11);
            assert!(load.site.clearance.abs() < 2e-11);
            for n in 0..ii.len() {
                let expected = weights[i][n];
                let found = load
                    .site
                    .feature
                    .vertices()
                    .iter()
                    .position(|&id| id == n)
                    .map_or(0., |j| load.site.weights[j]);
                assert!((found - expected).abs() < 2e-11);
                let separation =
                    DVec3::from(before[n].position).distance(state0[i + 1].position.into()) - 0.112;
                assert!(
                    separation > 0.10,
                    "vertex-only proxy would hit: {separation:e}"
                );
            }
            for j in 0..2 {
                for a in 0..3 {
                    for b in 0..3 {
                        rigid[i][j] +=
                            NORMAL[a] * operator[(3 + 3 * i + a, 3 + 3 * j + b)] * NORMAL[b];
                    }
                }
                k[i][j] = rigid[i][j]
                    + (0..ii.len())
                        .map(|n| weights[i][n] * weights[j][n] / ii[n])
                        .sum::<f64>();
            }
        }
        let solve = |k: [[f64; 2]; 2]| {
            let rhs = 0.8 * (1. + 0.55);
            let det = k[0][0] * k[1][1] - k[1][0] * k[0][1];
            [
                rhs * (k[1][1] - k[0][1]) / det,
                rhs * (k[0][0] - k[1][0]) / det,
            ]
        };
        let expected = solve(k);
        let mut uncoupled = k;
        uncoupled[1][0] -= rigid[1][0];
        uncoupled[0][1] -= rigid[0][1];
        let naive = solve(uncoupled);
        let mut max_err = 0f64;
        for i in 0..2 {
            let j = DVec3::from(loads[i].on_rigid);
            max_err = max_err.max(j.distance(-expected[i] * NORMAL));
            assert!((loads[i].final_normal_velocity - 0.55 * 0.8).abs() < 2e-10);
            let point_ve = point_velocity(i + 1, g.states()[i + 1], pts[i + 1]);
            let soft_v = (0..ii.len())
                .map(|n| weights[i][n] * DVec3::from(sample(&g).particles()[n].velocity))
                .sum::<DVec3>();
            assert!((NORMAL.dot(soft_v - point_ve) - 0.55 * 0.8).abs() < 2e-10);
        }
        for (idx, n) in sample(&g).particles().iter().enumerate() {
            let j = weights[0][idx] * expected[0] + weights[1][idx] * expected[1];
            let dv = DVec3::from(n.velocity) - DVec3::from(before[idx].velocity);
            max_err = max_err.max((dv / n.inverse_mass).distance(j * NORMAL));
            assert!(dv.dot(TANGENT).abs() < 2e-11 && dv.y.abs() < 2e-11);
        }
        let a = budgets(state0);
        let b = budgets(g.states());
        let c = pbudget(before);
        let d = pbudget(sample(&g).particles());
        let dp = (b.0 - a.0 + d.0 - c.0).length();
        let dl = (b.1 - a.1 + d.1 - c.1).length();
        let de = b.2 - a.2 + d.2 - c.2 + 0.5 * (1. - 0.55) * 0.8 * (expected[0] + expected[1]);
        let cross_delta = (expected[0] - naive[0])
            .abs()
            .max((expected[1] - naive[1]).abs());
        println!(
            "{} cross-link normal mobility {:.9} kg^-1, analytic impulses {:?} N s, independent-link max error {cross_delta:.6} N s, coupled {max_err:e} N s, P {dp:e} N s, L {dl:e} N m s, E {de:e} J, loss {:.9} J",
            if cloth { "cloth" } else { "cable" },
            rigid[0][1],
            expected,
            receipt.material.dissipated_energy
        );
        assert!(
            max_err < 4e-11 && dp < 2e-12 && dl < 2e-12 && de.abs() < 4e-11 && cross_delta > 0.001
        );
        assert!(
            (receipt.material.kinetic_energy_change + receipt.material.dissipated_energy).abs()
                < 2e-12
        );
        let impact = receipt.clone();
        let body_states = g.states();
        let nodes = sample(&g).particles().to_vec();
        reset_loaded_world(&mut g.sim, |_| ()).unwrap();
        assert!(sample(&g).latest_articulated_impact().is_none());
        g.sim.step_exact(100).unwrap();
        let rerun = sample(&g).latest_articulated_impact().unwrap();
        assert_eq!(
            (rerun.epoch, &rerun.material, &rerun.native),
            (impact.epoch + 1, &impact.material, &impact.native)
        );
        assert_eq!(body_states, g.states());
        assert_eq!(nodes, sample(&g).particles());
        let links = rerun.links.clone();
        assert!(
            articulated_impact::resolve(&mut g.sim, "material", &links, 0.55)
                .unwrap()
                .is_none()
        );
    }
}

#[test]
fn authored_misses_fault_without_partial_impact_and_invalid_schedules_are_refused() {
    for (time, needle) in [("99000000", "no incoming"), ("101000000", "penetrates")] {
        let xml = CLOTH.replace("100000000", time);
        let mut g = Gripper::new(&xml).unwrap();
        assert!(g.sim.step_exact(101).is_err());
        let message = g.sim.fault().unwrap().to_string();
        assert!(message.to_lowercase().contains(needle), "{message}");
        assert!(sample(&g).latest_articulated_impact().is_none());
        assert!(
            g.states()
                .iter()
                .all(|s| s.velocity == [0.; 3] && s.angular_velocity == [0.; 3])
        );
    }
    for xml in [
        CLOTH.replace("100000000", "99900000"),
        CLOTH.replace("100000000", "0"),
        CLOTH.replace("0.55", "1.5"),
        CLOTH.replace("right_finger\"/>", "left_finger\"/>"),
        CLOTH.replace(
            "impact material=\"material\"",
            "impact material=\"missing\"",
        ),
    ] {
        assert!(Gripper::new(&xml).is_err());
    }
}

#[test]
fn grounded_impacts_match_world_hinge_cartesian_projection_and_contact_permutation() {
    for (xml, cloth) in [(CLOTH, true), (CABLE, false)] {
        let model = centered_xml(xml, true);
        let swapped = model
            .replace("left_finger\"/>", "spare\"/>")
            .replace("right_finger\"/>", "left_finger\"/>")
            .replace("spare\"/>", "right_finger\"/>");
        let mut g = Gripper::new(&model).unwrap();
        let mut reordered = Gripper::new(&swapped).unwrap();
        g.sim.step_exact(100).unwrap();
        reordered.sim.step_exact(100).unwrap();
        let r = sample(&g).latest_articulated_impact().unwrap();
        let b = &r.native.before;
        assert_eq!((b.dofs, b.links), (3, 4));
        let sites = [
            [0.; 3],
            (0.1 * NORMAL).to_array(),
            (0.1 * NORMAL).to_array(),
        ];
        let kc = independent::cartesian(g.states(), [[0.; 3]; 3], true, sites);
        let mut worst = 0f64;
        for t in 0..2 {
            for s in 0..2 {
                let got = b.mobility(t, s).unwrap();
                for i in 0..3 {
                    for j in 0..3 {
                        worst = worst.max((got[i][j] - kc[(3 + 3 * t + i, 3 + 3 * s + j)]).abs());
                    }
                }
            }
        }
        let incoming = -0.8 * NORMAL + 0.07 * TANGENT + DVec3::Y * 0.02;
        let mut total = DVec3::ZERO;
        let mut angular = DVec3::ZERO;
        let mut e = 0.;
        let mount = DVec3::new(-0.32, 0., 0.58);
        for (i, s) in g.states().into_iter().enumerate() {
            let v = DVec3::from(s.velocity);
            let w = DVec3::from(s.angular_velocity);
            let spin = angular_mass(i, s, w, false);
            total += MASS[i] * v;
            angular += (DVec3::from(s.position) - mount).cross(MASS[i] * v) + spin;
            e += 0.5 * (MASS[i] * v.length_squared() + w.dot(spin));
            let perm = reordered.states()[i];
            assert!(
                DVec3::from(perm.velocity).distance(v) < 2e-13
                    && DVec3::from(perm.angular_velocity).distance(w) < 2e-13
            );
        }
        for (n, other) in sample(&g)
            .particles()
            .iter()
            .zip(sample(&reordered).particles())
        {
            let v = DVec3::from(n.velocity);
            let dp = (v - incoming) / n.inverse_mass;
            total += dp;
            angular += (DVec3::from(n.position) - mount).cross(dp);
            e += 0.5 * (v.length_squared() - incoming.length_squared()) / n.inverse_mass;
            assert!(v.distance(other.velocity.into()) < 2e-12);
        }
        println!(
            "grounded {}, independent blocks {worst:e} kg^-1, imposed mount impulse {total:?} N s, free-axis angular residual {:e} N m s, energy residual {:e} J",
            if cloth { "cloth" } else { "cable" },
            angular.z,
            (e + r.material.dissipated_energy).abs()
        );
        assert!(
            worst < 1e-12
                && total.length() > 0.1
                && angular.z.abs() < 2e-10
                && (e + r.material.dissipated_energy).abs() < 2e-10
        );
    }
}

#[test]
fn same_sdf_cold_clone_rehydrates_bit_exact_across_impact_and_warm_ticks() {
    use std::time::Duration;
    for xml in [CLOTH, CABLE] {
        let mut g = Gripper::new(xml).unwrap();
        let cold = articulated_impact::initial_material(&g.sim, "material").unwrap();
        let mut direct = cold.clone();
        let dt = Duration::from_millis(1);
        for k in 0..=199 {
            if k > 0 {
                direct.step(dt, &[]).unwrap();
                g.sim.step_exact(1).unwrap();
            }
            if k == 100 {
                let r = sample(&g).latest_articulated_impact().unwrap();
                let spheres = g.states()[1..]
                    .iter()
                    .map(|s| gz_physics_xpbd::articulated_impact::ImpactSphere {
                        center: s.position,
                        radius: 0.1,
                    })
                    .collect::<Vec<_>>();
                let report = direct
                    .sphere_impact_at_barrier(&spheres)
                    .unwrap()
                    .prepare(
                        &r.native.before.velocities,
                        |i, j| r.native.before.mobility(i, j).unwrap(),
                        r.restitution,
                    )
                    .unwrap()
                    .commit();
                assert_eq!(report, r.material);
            }
            assert_eq!(sample(&g).particles(), direct.particles());
            if matches!(k, 0 | 32 | 99 | 100 | 101 | 117 | 189) {
                let image = direct.checkpoint_articulated_material().unwrap();
                let mut fresh = cold.clone();
                fresh.restore_articulated_material(&image).unwrap();
                assert_eq!(fresh.checkpoint_articulated_material().unwrap(), image);
                assert_eq!(fresh.particles(), direct.particles());
                let bytes = fresh.checkpoint_articulated_material().unwrap();
                let mut bad = image.clone();
                bad[20] ^= 1;
                let mut nonfinite = image.clone();
                let n = nonfinite.len();
                nonfinite[n - 8..].copy_from_slice(&f64::NAN.to_le_bytes());
                let mut extra = image.clone();
                extra.push(0);
                for packet in [
                    &bad[..],
                    &image[..8],
                    &image[..image.len() - 1],
                    &nonfinite[..],
                    &extra[..],
                ] {
                    assert!(fresh.restore_articulated_material(packet).is_err());
                    assert_eq!(fresh.checkpoint_articulated_material().unwrap(), bytes);
                }
                let other =
                    Gripper::new(&xml.replace("stretch=\"0.002\"", "stretch=\"0.0021\"")).unwrap();
                let mut different =
                    articulated_impact::initial_material(&other.sim, "material").unwrap();
                assert!(different.restore_articulated_material(&image).is_err());
                direct = fresh;
            }
        }
    }
}

#[cfg(feature = "distributed-physics")]
#[test]
fn distributed_cold_recipe_retains_the_ordinary_public_links_and_both_soft_constructions() {
    for xml in [CLOTH, CABLE] {
        let a = articulated_impact::distributed_material_contact_tree_spec(
            xml,
            "material.sdf",
            "default",
        )
        .unwrap();
        let g = Gripper::new(xml).unwrap();
        assert_eq!(a.step, g.sim.step_duration());
        assert_eq!(a.tree.model_id, g.id);
        assert_eq!(a.limits.len(), 2);
        assert_eq!(a.barriers.len(), 1);
        assert_eq!(a.barriers[0].links, [g.ids[1].0, g.ids[2].0]);
        assert_eq!(a.barriers[0].time.as_nanos(), 100_000_000);
        assert_eq!(a.material.particles(), sample(&g).particles());
        assert_eq!(
            a.material.checkpoint_articulated_material().unwrap(),
            articulated_impact::initial_material(&g.sim, &a.material_name)
                .unwrap()
                .checkpoint_articulated_material()
                .unwrap()
        );
    }
}
