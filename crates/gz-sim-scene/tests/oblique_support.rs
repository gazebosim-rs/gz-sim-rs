// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/folded.rs"]
mod fixture;
#[allow(dead_code)]
#[path = "../examples/support/articulated_mobility.rs"]
mod gripper;
use glam::DVec3;
use gz_sim_scene::deformable;
use parry3d_f64::{query::PointQuery, shape::Triangle};
use std::{collections::BTreeSet, time::Duration};

#[test]
fn rough_oblique_sheet_loses_support_at_fine_resolution_with_closed_force_and_moment_budgets() {
    let mut times = vec![];
    for width in [250000, 62500, 15625] {
        let mut g = gripper::Gripper::new(&fixture::oblique(0.9, width)).unwrap();
        let pivot = DVec3::new(-0.32, 0., 0.58);
        let mut momentum: Option<DVec3> = None;
        let mut initial_l = None;
        let mut first = None;
        let mut last = Duration::ZERO;
        let mut gaps = f64::INFINITY;
        let mut maxp = 0f64;
        let mut maxl = 0f64;
        let mut features = BTreeSet::new();
        let mut n = 0;
        for tick in 0..=1800 {
            if tick > 0 {
                g.sim
                    .step_exact(1)
                    .unwrap_or_else(|_| panic!("width {width}, tick {tick}, {:?}", g.sim.fault()));
            }
            let s = deformable::bodies(&g.sim).next().unwrap().1;
            let mut p = DVec3::ZERO;
            let mut l = 0.;
            let mut mass = 0.;
            let mut pos = vec![];
            for v in s.particles() {
                let x = DVec3::from(v.position);
                let j = DVec3::from(v.velocity) / v.inverse_mass;
                pos.push(x);
                p += j;
                l += (x - pivot).cross(j).z;
                mass += 1. / v.inverse_mass;
            }
            for (i, b) in g.states().into_iter().enumerate() {
                let x = DVec3::from(b.position);
                l += (x - pivot)
                    .cross(gripper::MASS[i] * DVec3::from(b.velocity))
                    .z
                    + gripper::angular_mass(i, b, b.angular_velocity.into(), false).z;
                if i > 0 {
                    for [i, j, k] in [[0, 1, 3], [0, 3, 2]] {
                        let closest = Triangle::new(pos[i], pos[j], pos[k])
                            .project_local_point(x, false)
                            .point;
                        gaps = gaps.min(x.distance(closest) - 0.112);
                    }
                }
            }
            let mut on_rigid = DVec3::ZERO;
            for e in s.impacts_this_step() {
                if first.is_none() {
                    first = Some(e.time);
                }
                for q in &e.material.loads {
                    on_rigid += DVec3::from(q.on_rigid);
                }
            }
            for r in s.support_this_step() {
                last = r.start + r.duration;
                n += 1;
                assert!(r.region_clearances.iter().all(|g| *g >= -1.1e-12));
                for a in r.beginning.iter().chain(&r.end) {
                    for q in &a.material {
                        features.insert((q.site.sphere, q.site.feature));
                        on_rigid += DVec3::from(q.on_rigid);
                    }
                }
            }
            if let Some(before) = momentum {
                maxp = maxp.max((p - before + on_rigid + mass * 9.81 * 0.001 * DVec3::Z).length());
            }
            if let Some(before) = initial_l {
                let delta: f64 = l - before;
                maxl = maxl.max(delta.abs());
            } else {
                initial_l = Some(l);
            }
            momentum = Some(p);
        }
        assert_eq!(first, Some(Duration::from_nanos(213456789)));
        assert!(features.len() >= 2 && gaps >= -1.05e-12 && maxp < 1e-11 && maxl < 4e-9);
        println!(
            "Oblique rough sheet, {width} ns support cap: {n} loaded intervals, {} impacts, latest {last:?}, per-public-step particle momentum residual {maxp:e} N s, total free-axis momentum residual {maxl:e} N m s, minimum independent sphere/face clearance {gaps:e} m.",
            deformable::bodies(&g.sim)
                .next()
                .unwrap()
                .1
                .articulated_impact_count()
        );
        times.push(last);
    }
    assert_eq!(times[0], Duration::from_millis(1800));
    assert!(times[1] > Duration::from_millis(1500) && times[1] < Duration::from_millis(1700));
    assert!(times[2] > Duration::from_millis(1400) && times[2] < Duration::from_millis(1500));
}
