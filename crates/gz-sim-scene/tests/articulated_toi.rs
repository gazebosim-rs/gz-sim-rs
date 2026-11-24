// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/articulated_mobility.rs"]
mod fixture;
use fixture::*;
use glam::DVec3;
use gz_sim_scene::{deformable, queue_link_world_wrench, reset_loaded_world};
use std::time::Duration;
const CLOTH: &str = include_str!("../../../examples/articulated_first_impact.sdf");
const FAST: &str = include_str!("../../../examples/articulated_transient_cable.sdf");
const N: DVec3 = DVec3::new(0.28, 0., 0.96);
fn sample(g: &Gripper) -> &deformable::DeformableSample {
    deformable::bodies(&g.sim).next().unwrap().1
}
fn translated(xml: &str, v: DVec3) -> String {
    let marker = format!(
        r#"<plugin filename="gz-sim-initial-velocity-system" name="gz::sim::systems::InitialVelocity"><linear_velocity>{} {} {}</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>"#,
        v.x, v.y, v.z
    );
    let soft = v + DVec3::new(0.4, 0.12, -0.95);
    xml.replace(
        "<link name=\"palm\">",
        &format!("{marker}<link name=\"palm\">"),
    )
    .replace(
        "velocity=\"0.4 0.12 -0.95\"",
        &format!("velocity=\"{} {} {}\"", soft.x, soft.y, soft.z),
    )
}
#[test]
fn event_driven_native_and_cloth_split_at_the_off_grid_approach_and_reset_exactly() {
    for dt in ["0.0005", "0.001", "0.002", "0.004"] {
        let model = CLOTH.replace("0.001</max_step_size>", &format!("{dt}</max_step_size>"));
        let mut g = Gripper::new(&model).unwrap();
        let tick = g.sim.step_duration().as_nanos() as u64;
        let ticks = 100_375_123 / tick;
        g.sim.step_exact(ticks).unwrap_or_else(|e| {
            panic!(
                "{dt} {:?} {e} {:?} {:?}",
                g.sim.elapsed(),
                g.sim.fault(),
                sample(&g).particles()
            )
        });
        assert!(sample(&g).latest_articulated_impact().is_none());
        g.sim
            .step_exact(1)
            .unwrap_or_else(|e| panic!("{e}: {:?}", g.sim.fault()));
        let hit = sample(&g).latest_articulated_impact().unwrap().clone();
        let time = hit.detection.unwrap();
        let physical = time.public_start.as_secs_f64() + time.seconds_after_start;
        assert_eq!(time.public_start.as_nanos(), u128::from(tick * ticks));
        assert_eq!(hit.time.as_nanos(), 100_375_123);
        assert!((physical - 0.1003751234).abs() < 2e-11);
        assert_eq!(sample(&g).sim_time(), g.sim.elapsed());
        assert_eq!(hit.material.loads.len(), 2);
        for (load, &jn) in hit
            .material
            .loads
            .iter()
            .zip(&[0.18460178193416038, 0.15202041408363823])
        {
            assert!((load.site.clearance - 0.8 * 0.4e-9).abs() < 2e-11);
            assert!((-N.dot(load.on_rigid.into()) - jn).abs() < 6e-9);
            assert!(DVec3::from(load.friction.unwrap().final_slip).length() < 1e-12);
        }
        let elapsed = g.sim.elapsed();
        let baseline = g.states();
        let particles = sample(&g).particles().to_vec();
        reset_loaded_world(&mut g.sim, |_| ()).unwrap();
        g.sim.step_exact(ticks + 1).unwrap();
        let again = sample(&g).latest_articulated_impact().unwrap();
        assert_eq!(again.time, hit.time);
        assert_eq!(again.epoch, hit.epoch + 1);
        assert_eq!(again.material, hit.material);
        assert_eq!(again.native, hit.native);
        assert_eq!(again.detection, hit.detection);
        assert_eq!(g.states(), baseline);
        assert_eq!(sample(&g).particles(), particles);
        println!(
            "whole tick {tick} ns, first geometric {physical:.12} s, applied {} ns, published {} ns, spatial clock envelope {:e} m, both source faces survive epoch reset",
            hit.time.as_nanos(),
            elapsed.as_nanos(),
            time.rounding_distance
        );
    }
}
#[test]
fn double_clear_transient_between_ten_ms_endpoints_hits_both_native_fingertips() {
    let mut g = Gripper::new(FAST).unwrap();
    let initial = sample(&g).particles()[0];
    let signed = N.dot(DVec3::from(initial.position) - DVec3::new(0.32, initial.position[1], 0.58));
    let old = initial.velocity.into();
    let endpoint = signed + 0.01 * N.dot(old);
    assert!(signed > 0.005 && endpoint < -0.005);
    g.sim
        .step_exact(1)
        .unwrap_or_else(|e| panic!("{e}: {:?}", g.sim.fault()));
    let r = sample(&g).latest_articulated_impact().unwrap();
    assert_eq!(r.time, Duration::from_nanos(2_371_250));
    assert_eq!(r.material.loads.len(), 2);
    for load in &r.material.loads {
        assert!(
            (load.initial_normal_velocity + 1.8).abs() < 2e-10
                && (load.final_normal_velocity - 0.99).abs() < 2e-10
        );
    }
    println!(
        "10 ms interval, endpoint-only free-flight gaps {:.6} / {:.6} m, actual entry {} ns, two compressed normal impulses {:?} N s",
        signed - 0.005,
        -endpoint - 0.005,
        r.time.as_nanos(),
        r.material
            .loads
            .iter()
            .map(|l| -N.dot(l.on_rigid.into()))
            .collect::<Vec<_>>()
    );
}
#[test]
fn common_native_translation_preserves_relative_timing_and_forcing_is_refused_before_advancing() {
    let drift = DVec3::new(0.21, -0.12, 0.07);
    let mut free = Gripper::new(CLOTH).unwrap();
    let mut moving = Gripper::new(&translated(CLOTH, drift)).unwrap();
    free.sim
        .step_exact(101)
        .unwrap_or_else(|e| panic!("{:?} {e}: {:?}", free.sim.elapsed(), free.sim.fault()));
    moving
        .sim
        .step_exact(101)
        .unwrap_or_else(|e| panic!("{e}: {:?}", moving.sim.fault()));
    let a = sample(&free).latest_articulated_impact().unwrap();
    let b = sample(&moving).latest_articulated_impact().unwrap();
    assert_eq!(a.time, b.time);
    for (x, y) in a.material.loads.iter().zip(&b.material.loads) {
        assert!(DVec3::from(x.on_rigid).distance(y.on_rigid.into()) < 2e-10);
        assert!(
            DVec3::from(x.site.point)
                .distance(DVec3::from(y.site.point) - drift * b.time.as_secs_f64())
                < 2e-10
        );
    }
    let mut bad = Gripper::new(CLOTH).unwrap();
    let initial = bad.states();
    queue_link_world_wrench(&mut bad.sim, bad.ids[1].0, [0., 0., 0.2], [0.; 3]).unwrap();
    assert!(bad.sim.step_exact(1).is_err());
    assert_eq!(bad.sim.elapsed(), Duration::ZERO);
    assert_eq!(bad.states(), initial);
    assert!(sample(&bad).latest_articulated_impact().is_none());
}

#[cfg(feature="distributed-physics")]
#[test]
fn distributed_automatic_construction_preserves_the_true_ten_ms_clock_and_rejects_other_profiles() {
    use gz_sim_scene::deformable::articulated_impact::{distributed_automatic_material_contact_tree_spec as input,distributed_material_contact_tree_spec as scheduled};
    let ordinary=Gripper::new(FAST).unwrap();
    let r=input(FAST,"cable.sdf","default").unwrap();
    assert_eq!(r.step,Duration::from_millis(10));assert_eq!(r.material_name,"material");assert_eq!(r.restitution,0.55);
    assert_eq!(r.links,[ordinary.ids[1].0,ordinary.ids[2].0]);
    assert_eq!(r.material.particles(),sample(&ordinary).particles());assert_eq!(r.limits.len(),2);
    assert!(scheduled(FAST,"cable.sdf","default").is_err());
    let prescribed=FAST.replace("detection=\"first_impact\"","time_ns=\"10000000\"");
    assert!(input(&prescribed,"scheduled.sdf","default").is_err());
    assert!(input(&FAST.replace("0 0 0</gravity>","0 0 -9.81</gravity>"),"gravity.sdf","default").is_err());
}
