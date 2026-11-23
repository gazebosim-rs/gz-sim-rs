// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/articulated_mobility.rs"]
mod gripper;
#[allow(dead_code)]
#[path = "../examples/support/supported_cable.rs"]
mod oracle;
use glam::DVec3;
use gz_sim_scene::{
    deformable::{self, articulated_impact::SupportApplication},
    reset_loaded_world,
};
use oracle::*;
use std::time::Duration;
fn soft(g: &gripper::Gripper) -> &deformable::DeformableSample {
    deformable::bodies(&g.sim).next().unwrap().1
}
fn audit(a: &SupportApplication) {
    for l in &a.material {
        let j = DVec3::from(l.on_rigid);
        assert!(j.dot(DVec3::from(l.site.normal)) <= 0.);
        let mut sum = j;
        let mut moment = DVec3::from(l.site.point).cross(j);
        for &(_, x, v) in &l.on_particles {
            sum += DVec3::from(v);
            moment += DVec3::from(x).cross(v.into());
        }
        assert!(sum.length() < 1e-13 && moment.length() < 1e-13);
    }
}
#[test]
fn asymmetric_static_weight_is_transmitted_at_each_finger_and_resting_initial_state_needs_no_collision()
 {
    for initially_resting in [false, true] {
        let mut src = REST.to_owned();
        if initially_resting {
            src = src.replace("0.712", "0.692");
        }
        let mut g = gripper::Gripper::new(&src).unwrap();
        let mut end = Duration::ZERO;
        let zero = if initially_resting { 0. } else { onset() };
        let mut accumulated = [DVec3::ZERO; 2];
        let mut event = None;
        let mut maxv = 0f64;
        while g.sim.elapsed() < Duration::from_millis(650) {
            g.sim
                .step_exact(1)
                .unwrap_or_else(|_| panic!("at {:?}: {:?}", g.sim.elapsed(), g.sim.fault()));
            let s = soft(&g);
            let mut this = [DVec3::ZERO; 2];
            for r in s.impacts_this_step() {
                assert!(event.is_none());
                event = Some(r.clone());
            }
            for r in s.normal_support_this_step() {
                assert!(r.start >= end);
                end = r.start + r.duration;
                for a in r.beginning.iter().chain(&r.end) {
                    audit(a);
                    for l in &a.material {
                        this[l.site.sphere] += DVec3::from(l.on_rigid);
                    }
                }
            }
            for (i, x) in this.into_iter().enumerate() {
                accumulated[i] += x;
                if g.sim.elapsed().as_secs_f64() > zero + 0.002 {
                    let mass = [0.26, 0.17][i];
                    assert!((x / 0.001 + mass * G * DVec3::Z).length() < 5e-7);
                }
            }
            if g.sim.elapsed().as_secs_f64() > zero + 0.002 {
                for p in s.particles() {
                    maxv = maxv.max(DVec3::from(p.velocity).length());
                    assert!((p.position[2] - 0.692).abs() < 2e-10);
                }
            }
        }
        assert_eq!(event.is_some(), !initially_resting);
        if let Some(r) = event {
            let d = r.detection.unwrap();
            assert!((d.public_start.as_secs_f64() + d.seconds_after_start - onset()).abs() < 2e-13);
        }
        for (i, j) in accumulated.iter().enumerate() {
            assert!((j.z + [0.26, 0.17][i] * G * (0.65 - zero)).abs() < 2e-8);
        }
        assert!(maxv < 3e-8);
        println!(
            "Initially resting {initially_resting}, static two-finger weight 2.550600 and 1.667700 N, total impulses {accumulated:?} N s, maximum equilibrium speed {maxv:e} m/s."
        );
        let particles = soft(&g).particles().to_vec();
        let rigid = g.states();
        reset_loaded_world(&mut g.sim, |_| ()).unwrap();
        assert!(soft(&g).normal_support_this_step().is_empty());
        g.sim.step_exact(650).unwrap();
        assert_eq!(particles, soft(&g).particles());
        assert_eq!(rigid, g.states());
    }
}
#[test]
fn finite_contact_force_and_release_follow_spherical_pendulum_energy_and_no_tension() {
    for width in [500000, 250000, 62500] {
        let mut g = gripper::Gripper::new(&RELEASE.replace("250000", &width.to_string())).unwrap();
        let (mut worst_x, mut worst_v, mut worst_n, mut worst_e) = (0f64, 0f64, 0f64, 0f64);
        let mut last = 0.;
        let mut native = 0f64;
        let mut impulses = 0;
        let t0 = onset();
        let td = detaches(SPEED);
        while g.sim.elapsed() < Duration::from_millis(500) {
            g.sim.step_exact(1).unwrap_or_else(|_| {
                panic!(
                    "width {width} ns at {:?}: {:?}",
                    g.sim.elapsed(),
                    g.sim.fault()
                )
            });
            let t = g.sim.elapsed().as_secs_f64();
            let s = soft(&g);
            impulses += s.impacts_this_step().len();
            let mut load = DVec3::ZERO;
            for r in s.normal_support_this_step() {
                last = (r.start + r.duration).as_secs_f64();
                for a in r.beginning.iter().chain(&r.end) {
                    audit(a);
                    for l in &a.material {
                        load -= DVec3::from(l.on_rigid);
                    }
                }
            }
            for (i, n) in s.particles().iter().enumerate() {
                let expected = exact(t, SPEED, i, 0.215);
                worst_x = worst_x.max(DVec3::from(n.position).distance(expected.position));
                worst_v = worst_v.max(DVec3::from(n.velocity).distance(expected.velocity));
                if t > t0 + 0.002 {
                    let energy = 0.215
                        * (0.5 * DVec3::from(n.velocity).length_squared() + G * n.position[2]);
                    let initial = 0.215 * (0.5 * SPEED * SPEED + G * (R + 0.58));
                    worst_e = worst_e.max((energy - initial).abs());
                }
            }
            for (i, r) in g.states().into_iter().enumerate() {
                native = native.max(
                    DVec3::from(r.velocity).length() + DVec3::from(r.angular_velocity).length(),
                );
                if i > 0 {
                    assert!(DVec3::from(r.position).distance(CENTRES[2 - i]) < 1e-8);
                }
            }
            if t > t0 + 0.002 && t < td - 0.002 {
                let impulse = 0.43
                    * (exact(t, SPEED, 0, 0.215).velocity
                        - exact(t - 0.001, SPEED, 0, 0.215).velocity)
                    + 0.43 * G * 0.001 * DVec3::Z;
                worst_n = worst_n.max((load - impulse).length() / 0.001);
            }
        }
        println!(
            "spherical release, {width} ns, true {td:.12} s vs measured end {last:.12} s, normal-force error {worst_n:e} N, full-trajectory position/velocity errors {worst_x:e} m/{worst_v:e} m/s, energy drift {worst_e:e} J per bead, native stationary residual {native:e}, impacts {impulses}"
        );
        assert_eq!(impulses, 1);
        assert!(
            (td - last).abs() < 3. * width as f64 * 1e-9
                && worst_x < 2e-4
                && worst_v < 2e-3
                && worst_n < 0.02
                && worst_e < 2e-4
                && native < 1e-7
        );
    }
}
#[test]
fn supercritical_horizontal_launch_separates_without_a_tensile_impulse() {
    let v = 1.24;
    let mut g = gripper::Gripper::new(&launched(v)).unwrap();
    let mut loads = 0;
    let mut worst = 0f64;
    while g.sim.elapsed() < Duration::from_millis(240) {
        g.sim
            .step_exact(1)
            .unwrap_or_else(|_| panic!("at {:?}: {:?}", g.sim.elapsed(), g.sim.fault()));
        loads += soft(&g).normal_support_this_step().len();
        for (i, p) in soft(&g).particles().iter().enumerate() {
            worst = worst.max(
                DVec3::from(p.position)
                    .distance(exact(g.sim.elapsed().as_secs_f64(), v, i, 0.215).position),
            );
        }
    }
    println!(
        "supercritical {v} m/s launch, {loads} loaded intervals, independent ballistic discrepancy {worst:e} m"
    );
    assert_eq!(loads, 0);
    assert_eq!(soft(&g).articulated_impact_count(), 1);
    assert!(worst < 1e-8);
}

#[test]
fn sheet_interior_barycentric_loads_balance_four_authored_masses() {
    let src = include_str!("../../../examples/articulated_supported_cloth.sdf");
    let mut g = gripper::Gripper::new(src).unwrap();
    g.sim.step_exact(250).unwrap();
    let s = soft(&g);
    let mut nodal = [DVec3::ZERO; 4];
    let mut per_tip = [DVec3::ZERO; 2];
    let mut kinds = std::collections::BTreeSet::new();
    for r in s.normal_support_this_step() {
        for a in r.beginning.iter().chain(&r.end) {
            audit(a);
            for l in &a.material {
                assert_eq!(l.site.feature.vertices().len(), 3);
                assert!(l.site.weights.iter().all(|w| *w > 0.15));
                kinds.insert((l.site.sphere, l.site.feature));
                per_tip[l.site.sphere] += DVec3::from(l.on_rigid);
                for &(i, _, j) in &l.on_particles {
                    nodal[i] += DVec3::from(j);
                }
            }
        }
    }
    assert_eq!(kinds.len(), 2);
    for (n, j) in s.particles().iter().zip(nodal) {
        assert!((n.inverse_mass * j / 0.001 - G * DVec3::Z).length() < 1e-7);
    }
    assert!((per_tip[0].z / 0.001 + 2.5506).abs() < 1e-7);
    assert!((per_tip[1].z / 0.001 + 1.6677).abs() < 1e-7);
    println!(
        "Held two actual triangle interiors {kinds:?} at 250 ms. Measured per-node vertical support forces {:?} N.",
        nodal.map(|j| j.z / 0.001)
    );
}
fn lz(g: &gripper::Gripper) -> f64 {
    let mount = DVec3::new(-0.32, 0., 0.58);
    let mut moment = 0.;
    for (i, s) in g.states().into_iter().enumerate() {
        let p = gripper::MASS[i] * DVec3::from(s.velocity);
        moment += (DVec3::from(s.position) - mount).cross(p).z
            + gripper::angular_mass(i, s, s.angular_velocity.into(), false).z;
    }
    for n in soft(g).particles() {
        moment += (DVec3::from(n.position) - mount)
            .cross(DVec3::from(n.velocity) / n.inverse_mass)
            .z;
    }
    moment
}
#[test]
fn asymmetric_transverse_slide_turns_native_articulation_and_preserves_free_axis_momentum() {
    let src = RELEASE
        .replacen("mass=\"0.215\"", "mass=\"0.17\"", 1)
        .replace("mass=\"0.215\"", "mass=\"0.26\"");
    let mut g = gripper::Gripper::new(&src).unwrap();
    let initial = lz(&g);
    let mut worst = 0f64;
    let mut spin = 0f64;
    let mut torque = 0.;
    let pivot = DVec3::new(-0.32, 0., 0.58);
    while g.sim.elapsed() < Duration::from_millis(340) {
        g.sim.step_exact(1).unwrap();
        for r in soft(&g).normal_support_this_step() {
            for a in r.beginning.iter().chain(&r.end) {
                audit(a);
                for l in &a.material {
                    torque += (DVec3::from(l.site.point) - pivot)
                        .cross(l.on_rigid.into())
                        .z;
                }
            }
        }
        let now = lz(&g);
        worst = worst.max((now - initial).abs());
        spin = spin.max(g.states()[0].angular_velocity[2].abs());
    }
    println!(
        "Unequal slider masses, peak actual native palm spin {spin:e} rad/s, summed native support moment {torque:e} N m s, worst total free-axis momentum discrepancy {worst:e} N m s."
    );
    assert!(spin > 1e-4 && torque.abs() > 1e-4 && worst < 2e-6);
}

#[test]
fn deforming_sheet_releases_and_reacquires_later_faces_without_penetration() {
    use parry3d_f64::{query::PointQuery, shape::Triangle};
    let s = include_str!("../../../examples/articulated_ballistic_cloth.sdf")
        .replace("friction=\"0.9\"", "friction=\"0\"")
        .replace(
            "detection=\"first_impact\" restitution=\"0.55\"",
            "detection=\"persistent_contact\" restitution=\"0\" max_contact_step_ns=\"250000\"",
        );
    let mut g = gripper::Gripper::new(&s).unwrap();
    let mut min = f64::INFINITY;
    let mut loaded = 0;
    let mut latest = Duration::ZERO;
    while g.sim.elapsed() < Duration::from_millis(600) {
        g.sim
            .step_exact(1)
            .unwrap_or_else(|_| panic!("at {:?}: {:?}", g.sim.elapsed(), g.sim.fault()));
        let sample = soft(&g);
        let p: Vec<_> = sample
            .particles()
            .iter()
            .map(|n| DVec3::from(n.position))
            .collect();
        for body in &g.states()[1..] {
            let c: DVec3 = body.position.into();
            for tri in [[0, 1, 3], [0, 3, 2]] {
                let [a, b, d] = tri.map(|id| p[id]);
                min = min.min(
                    c.distance(Triangle::new(a, b, d).project_local_point(c, false).point) - R,
                );
            }
        }
        for r in sample.normal_support_this_step() {
            loaded += 1;
            latest = r.start + r.duration;
            for a in r.beginning.iter().chain(&r.end) {
                audit(a);
            }
        }
    }
    assert!(min >= -1e-12 && loaded > 100 && soft(&g).articulated_impact_count() >= 5);
    assert!(latest > Duration::from_millis(320) && latest < Duration::from_millis(500));
    println!(
        "Oblique sheet, {} separate timed inelastic impacts and {loaded} finite loaded substeps followed by true release at {latest:?}, minimum independent public surface clearance {min:e} m through 600 ms.",
        soft(&g).articulated_impact_count()
    );
}
