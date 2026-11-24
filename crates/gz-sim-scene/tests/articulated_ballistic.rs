// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/articulated_mobility.rs"]
mod fixture;
use fixture::*;
use glam::DVec3;
use gz_sim_scene::{deformable, reset_loaded_world};
use std::time::Duration;

const CLOTH: &str = include_str!("../../../examples/articulated_ballistic_cloth.sdf");
const APEX: &str = include_str!("../../../examples/articulated_ballistic_apex.sdf");
const G: DVec3 = DVec3::new(0., 0., -9.81);
fn sample(g: &Gripper) -> &deformable::DeformableSample {
    deformable::bodies(&g.sim).next().unwrap().1
}
fn advance(g: &mut Gripper, n: u64) {
    g.sim
        .step_exact(n)
        .unwrap_or_else(|e| panic!("{e}, {:?}", g.sim.fault()));
}
#[test]
fn dropping_and_sliding_sheet_reaches_a_world_supported_tree_at_its_ballistic_time() {
    let v = DVec3::new(0.58, 0.19, 0.34);
    let n = DVec3::new(0.28, 0., 0.96);
    for (ns, dt) in [
        (500_000_u64, "0.0005"),
        (2_000_000, "0.002"),
        (7_000_000, "0.007"),
    ] {
        let xml = CLOTH.replace("0.001</max_step_size>", &format!("{dt}</max_step_size>"));
        let mut g = Gripper::new(&xml).unwrap();
        let start = sample(&g).particles().to_vec();
        let stationary = g.states();
        let center = DVec3::from(stationary[1].position);
        let gap = n.dot(DVec3::from(start[0].position) - center) - 0.112;
        let (b, a) = (n.dot(v), 0.5 * n.dot(G));
        let first = (-b - (b * b - 4. * a * gap).sqrt()) / (2. * a);
        let public_ticks = 213_456_789 / ns;
        advance(&mut g, public_ticks);
        let t = g.sim.elapsed().as_secs_f64();
        for (p, init) in sample(&g).particles().iter().zip(&start) {
            assert!(
                DVec3::from(p.position)
                    .distance(DVec3::from(init.position) + t * v + 0.5 * t * t * G)
                    < 5e-15
            );
            assert!(DVec3::from(p.velocity).distance(v + t * G) < 5e-15);
        }
        for (a, b) in g.states().iter().zip(stationary) {
            assert!(DVec3::from(a.position).distance(b.position.into()) < 2e-14);
            assert!(DVec3::from(a.velocity).length() < 2e-14);
        }
        assert!(sample(&g).latest_articulated_impact().is_none());
        advance(&mut g, 1);
        let hit = sample(&g).latest_articulated_impact().unwrap().clone();
        let d = hit.detection.unwrap();
        assert!((d.public_start.as_secs_f64() + d.seconds_after_start - first).abs() < 1e-12);
        assert_eq!(hit.time, Duration::from_nanos(213_456_789));
        let closing = n.dot(v + hit.time.as_secs_f64() * G);
        let mut net = DVec3::ZERO;
        assert_eq!(hit.material.loads.len(), 2);
        for load in &hit.material.loads {
            assert!((load.initial_normal_velocity - closing).abs() < 2e-12);
            assert!((load.final_normal_velocity + 0.55 * closing).abs() < 2e-12);
            let impulse = DVec3::from(load.on_rigid);
            net += impulse;
            assert!(
                (impulse
                    + load
                        .on_particles
                        .iter()
                        .map(|p| DVec3::from(p.1))
                        .sum::<DVec3>())
                .length()
                    < 5e-15
            );
        }
        let after = sample(&g).particles().to_vec();
        let native = g.states();
        reset_loaded_world(&mut g.sim, |_| ()).unwrap();
        advance(&mut g, public_ticks + 1);
        let again = sample(&g).latest_articulated_impact().unwrap();
        assert_eq!(again.epoch, hit.epoch + 1);
        assert_eq!(again.material, hit.material);
        assert_eq!(again.native, hit.native);
        assert_eq!(after, sample(&g).particles());
        assert_eq!(g.states(), native);
        println!(
            "public tick {ns} ns, analytic arrival {first:.12} s, actual source approach {closing:.9} m/s, net world impulse on supported gripper {net:?} N s, dissipation {:.9} J, clock travel {:e} m",
            hit.material.dissipated_energy, d.rounding_distance
        );
    }
}
#[test]
fn thrown_cable_contacts_at_an_interior_apex_missed_by_the_entire_endpoint_chord() {
    let mut g = Gripper::new(APEX).unwrap();
    let p = DVec3::from(sample(&g).particles()[0].position);
    let tip = g.states()[1].position;
    let below = |t: f64| p.z + 0.5 * t - 4.905 * t * t - tip[2];
    let gap = |t: f64| (p.x - tip[0]).hypot(below(t)) - 0.005;
    let vertical = (0.005f64.powi(2) - (p.x - tip[0]).powi(2)).sqrt();
    let first = (0.5 - (0.25 + 2. * 9.81 * (p.z - tip[2] + vertical)).sqrt()) / 9.81;
    let apex = 0.5 / 9.81;
    assert!(gap(0.)>0.01 && gap(0.1)>0.01 && gap(apex) < -0.0005);
    // Every linear interpolation between these endpoints stays below their maximum.
    let chord_gap = (p.x - tip[0]).hypot(below(0.).max(below(0.1))) - 0.005;
    assert!(chord_gap > 0.01);
    advance(&mut g, 1);
    let hit = sample(&g).latest_articulated_impact().unwrap();
    let d = hit.detection.unwrap();
    assert_eq!(hit.material.loads.len(), 2);
    assert!((d.seconds_after_start - first).abs() < 1e-12);
    assert_eq!(hit.time.as_nanos(), (first * 1e9).round() as u128);
    let normal = DVec3::new(0.6, 0., -0.8);
    let approach = -0.8 * (0.5 - 9.81 * hit.time.as_secs_f64());
    for load in &hit.material.loads {
        assert!(DVec3::from(load.site.normal).distance(normal) < 1e-8);
        assert!((load.initial_normal_velocity - approach).abs() < 2e-9);
        assert!((load.final_normal_velocity + 0.55 * approach).abs() < 2e-9);
    }
    println!(
        "100 ms interval, full straight-chord minimum gap {chord_gap:.9} m, hypothetical apex gap {:.9} m, analytic first impact {first:.12} s, normal speed {approach:.9} m/s, actual opposite impulse magnitudes {:?} N s",
        gap(apex),
        hit.material
            .loads
            .iter()
            .map(|v| DVec3::from(v.on_rigid).length())
            .collect::<Vec<_>>()
    );
}
#[test]
fn loaded_gravity_torque_and_free_falling_native_root_are_refused_before_either_body_advances() {
    let x = CLOTH.replace("0 0 1</xyz>", "0 1 0</xyz>");
    let mut floating = CLOTH.to_owned();
    let start = floating.find("<joint name=\"world_pivot\"").unwrap();
    let end = start + floating[start..].find("</joint>").unwrap() + 8;
    floating.replace_range(start..end, r#"<plugin filename="gz-sim-switchable-articulation-system" name="gz::sim::systems::SwitchableArticulation"/>"#);
    for xml in [x, floating] {
        let mut g = Gripper::new(&xml).unwrap();
        let native = g.states();
        let soft = sample(&g).particles().to_vec();
        assert!(g.sim.step_exact(1).is_err());
        assert_eq!(g.sim.elapsed(), Duration::ZERO);
        assert_eq!(g.states(), native);
        assert_eq!(soft, sample(&g).particles());
    }
}
