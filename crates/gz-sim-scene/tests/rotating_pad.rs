// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/rotating_pad.rs"]
mod source;
use glam::DVec3;
use gz_sim_scene::reset_loaded_world;
use source::*;

const INERTIA: f64 = 0.032 + 1.2 * 0.45 * 0.45;
const PIVOT: DVec3 = DVec3::new(0., 0., 0.62);
fn arm(t: f64) -> DVec3 {
    let (s, c) = (4. * t).sin_cos();
    0.45 * DVec3::new(c, s, 0.)
}
fn compare_base(q: &Rotor, t: f64) {
    let s = q.state();
    assert!(
        DVec3::from(s.position).distance(PIVOT + arm(t)) < 1e-11,
        "{s:?} at {t}"
    );
    assert!(DVec3::from(s.angular_velocity).distance(4. * DVec3::Z) < 1e-11);
    assert!(DVec3::from(s.velocity).distance(4. * DVec3::Z.cross(arm(t))) < 1e-11);
}
#[test]
fn same_arc_inside_a_clear_endpoint_chord_has_exact_time_impulse_and_native_angular_reaction() {
    let first = (0.8 - 2. * (0.04_f64 / 0.9).asin()) / 4.;
    for step in ["0.4", "0.007", "0.001"] {
        let mut rotor =
            Rotor::new(&CABLE.replace("0.4</max_step_size>", &format!("{step}</max_step_size>")))
                .unwrap();
        compare_base(&rotor, 0.);
        let source = DVec3::from(rotor.material().particles()[0].position);
        let chord_clearance = 0.45 * (1. - 0.8_f64.cos()) - 0.04;
        assert!(chord_clearance > 0.09);
        let nstep = (first / rotor.sim.step_duration().as_secs_f64()).floor() as u64;
        rotor.advance(nstep).unwrap();
        compare_base(&rotor, rotor.sim.elapsed().as_secs_f64());
        rotor.advance(1).unwrap();
        let hit = rotor
            .material()
            .latest_articulated_impact()
            .unwrap()
            .clone();
        let d = hit.detection.unwrap();
        assert!((d.public_start.as_secs_f64() + d.seconds_after_start - first).abs() < 1e-11);
        assert_eq!(hit.time.as_nanos(), (first * 1e9).round() as u128);
        assert_eq!(hit.material.loads.len(), 1);
        let l = &hit.material.loads[0];
        assert_eq!(l.site.feature.vertices(), [0, 1]);
        let c = PIVOT + arm(hit.time.as_secs_f64());
        let nearest = DVec3::new(source.x, source.y, c.z);
        let normal = (nearest - c).normalize();
        let lever = (c - PIVOT).cross(normal).z;
        let approach = -4. * lever;
        let j = -1.55 * approach / (2.5 + lever * lever / INERTIA);
        let measured = DVec3::from(l.on_rigid);
        assert!(measured.distance(-j * normal) < 2e-9);
        let after = 4. - j * lever / INERTIA;
        assert!((rotor.state().angular_velocity[2] - after).abs() < 3e-9);
        assert!(
            (hit.native.kinetic_energy_change - 0.5 * INERTIA * (after * after - 16.)).abs() < 3e-9
        );
        assert!((l.final_normal_velocity + 0.55 * approach).abs() < 3e-9);
        let state = rotor.state();
        let nodes = rotor.material().particles().to_vec();
        reset_loaded_world(&mut rotor.sim, |_| ()).unwrap();
        rotor.advance(nstep + 1).unwrap();
        let again = rotor.material().latest_articulated_impact().unwrap();
        assert_eq!(again.material, hit.material);
        assert_eq!(again.native, hit.native);
        assert_eq!(again.epoch, hit.epoch + 1);
        assert_eq!(rotor.state(), state);
        assert_eq!(nodes, rotor.material().particles());
        println!(
            "{step} s public tick, true arc entry {first:.12} s, straight chord clearance {chord_clearance:.9} m, normal impulse {j:.9} N s; measured natural angular velocity 4 -> {after:.9} rad/s, axial momentum {jlever:.9} N m s",
            jlever = -j * lever
        );
    }
}
#[test]
fn moving_pad_and_falling_cloth_share_true_cartesian_point_mobility_and_skin_impulse() {
    let first = 0.2413376504;
    for step in ["0.001", "0.003", "0.014"] {
        let mut rotor =
            Rotor::new(&CLOTH.replace("0.003</max_step_size>", &format!("{step}</max_step_size>")))
                .unwrap();
        let seed = rotor.material().particles().to_vec();
        let nstep = (first / rotor.sim.step_duration().as_secs_f64()).floor() as u64;
        rotor.advance(nstep).unwrap();
        compare_base(&rotor, rotor.sim.elapsed().as_secs_f64());
        rotor.advance(1).unwrap();
        let hit = rotor.material().latest_articulated_impact().unwrap();
        let d = hit.detection.unwrap();
        assert!((d.public_start.as_secs_f64() + d.seconds_after_start - first).abs() < 1e-11);
        assert_eq!(hit.time.as_nanos(), 241337650);
        let t = hit.time.as_secs_f64();
        let l = &hit.material.loads[0];
        assert_eq!(l.site.feature.vertices().len(), 3);
        let point = DVec3::from(l.site.point);
        let axis = DVec3::Z.cross(point - PIVOT);
        let k = hit.native.before.mobility(0, 0).unwrap();
        for a in 0..3 {
            for b in 0..3 {
                assert!((k[a][b] - axis[a] * axis[b] / INERTIA).abs() < 1e-12);
            }
        }
        let impulse = DVec3::from(l.on_rigid);
        let after = 4. + axis.dot(impulse) / INERTIA;
        assert!((rotor.state().angular_velocity[2] - after).abs() < 1e-11);
        assert!(
            (hit.native.kinetic_energy_change - 0.5 * INERTIA * (after * after - 16.)).abs()
                < 1e-11
        );
        let incoming = DVec3::from(seed[0].velocity) - 9.81 * t * DVec3::Z - 4. * axis;
        let n = DVec3::from(l.site.normal);
        let tangent = l.friction.unwrap();
        assert!((incoming.dot(n) - l.initial_normal_velocity).abs() < 2e-11);
        assert!((l.final_normal_velocity + 0.55 * l.initial_normal_velocity).abs() < 2e-11);
        let jt = DVec3::from(tangent.on_material).length();
        let slip = DVec3::from(tangent.final_slip).length();
        assert!(jt > 0.02 && slip > 0.01);
        assert!((jt + 0.35 * impulse.dot(n)).abs() < 2e-11);
        println!(
            "{step} s public tick, true falling sheet meets pad at {first:.12} s, actual normal/tangent impulse {:.8}/{jt:.8} N s, residual skin slip {slip:.8} m/s, angular velocity after {after:.9} rad/s",
            -impulse.dot(n)
        );
    }
}

#[test]
fn cold_native_sampler_uses_authored_spin_and_invalid_pivot_constraints_fail_before_motion() {
    use gz_sim_scene::articulated_contact::{self, LinkContactSite};
    let mut valid = Rotor::new(CABLE).unwrap();
    let sites = [LinkContactSite {
        link: valid.link,
        local_point: [0., 0.035, 0.],
    }];
    let read = articulated_contact::sample(&mut valid.sim, &sites).unwrap();
    let v = 4. * DVec3::Z.cross(DVec3::from(read.world_points[0]) - PIVOT);
    assert!(v.distance(read.velocities[0].into()) < 1e-12);
    assert!(
        Rotor::new(&CABLE.replace("0 1.8 0</linear_velocity>", "0 0 0</linear_velocity>")).is_err()
    );
    for xml in [
        CABLE.replace("<upper>100</upper>", "<upper>1.1</upper>"),
        CABLE.replace("<gravity>0 0 0</gravity>", "<gravity>0 -9.81 0</gravity>"),
    ] {
        let mut rotor = Rotor::new(&xml).unwrap();
        let rigid = rotor.state();
        let soft = rotor.material().particles().to_vec();
        assert!(rotor.advance(1).is_err());
        assert_eq!(rotor.sim.elapsed(), std::time::Duration::ZERO);
        assert_eq!(rotor.state(), rigid);
        assert_eq!(rotor.material().particles(), soft);
    }
}
