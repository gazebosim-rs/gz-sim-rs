// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/articulated_mobility.rs"]
mod gripper;
#[allow(dead_code)]
#[path = "../examples/support/rotating_pad.rs"]
mod rotor;
use glam::DVec3;
use gz_sim_scene::{
    deformable::{self, articulated_impact::ArticulatedMaterialImpact},
    reset_loaded_world,
};
use std::{sync::Arc, time::Duration};
const CLOTH: &str = include_str!("../../../examples/articulated_recurrent_cloth.sdf");
const TURN: &str = include_str!("../../../examples/rotating_recurrent_cloth.sdf");
fn sample(g: &gripper::Gripper) -> &deformable::DeformableSample {
    deformable::bodies(&g.sim).next().unwrap().1
}
fn last_time(event: &ArticulatedMaterialImpact) -> f64 {
    let d = event.detection.unwrap();
    d.public_start.as_secs_f64() + d.seconds_after_start
}
fn audit(event: &ArticulatedMaterialImpact) {
    assert!(event.material.dissipated_energy > 0.);
    for r in &event.material.loads {
        assert!(r.initial_normal_velocity < 0. && r.final_normal_velocity > 0.);
        assert!(
            (r.final_normal_velocity + event.restitution * r.initial_normal_velocity).abs() < 1e-10
        );
        let mut j = DVec3::from(r.on_rigid);
        let mut m = DVec3::from(r.site.point).cross(j);
        for (id, v) in &r.on_particles {
            let x = event
                .loaded_particles
                .iter()
                .find(|p| &p.0 == id)
                .unwrap()
                .1;
            j += DVec3::from(*v);
            m += DVec3::from(x).cross(DVec3::from(*v));
        }
        assert!(j.length() < 1e-12 && m.length() < 1e-12);
    }
}
fn run(g: &mut gripper::Gripper) -> Vec<Arc<ArticulatedMaterialImpact>> {
    let mut result = vec![];
    while g.sim.elapsed() < Duration::from_millis(320) {
        g.sim
            .step_exact(1)
            .unwrap_or_else(|e| panic!("{e} at {:?}: {:?}", g.sim.elapsed(), g.sim.fault()));
        result.extend(sample(g).impacts_this_step().iter().cloned());
    }
    assert_eq!(sample(g).articulated_impact_count() as usize, result.len());
    for r in &result {
        audit(r);
    }
    result
}
#[test]
fn returning_sheet_on_distinct_fingers_has_separate_nodal_moments_and_an_exact_epoch_reset() {
    let mut g = gripper::Gripper::new(CLOTH).unwrap();
    let a = run(&mut g);
    assert_eq!(a.len(), 3);
    assert_eq!(
        a.iter()
            .map(|r| r
                .material
                .loads
                .iter()
                .map(|l| l.site.sphere)
                .collect::<Vec<_>>())
            .collect::<Vec<_>>(),
        [vec![0, 1], vec![0], vec![1]]
    );
    for (r, t) in a.iter().zip([0.2134567893, 0.2372547880, 0.2662457289]) {
        assert!((last_time(r) - t).abs() < 2e-9);
    }
    let rigid = g.states();
    let nodes = sample(&g).particles().to_vec();
    reset_loaded_world(&mut g.sim, |_| ()).unwrap();
    assert_eq!(sample(&g).articulated_impact_count(), 0);
    assert!(sample(&g).impacts_this_step().is_empty());
    let b = run(&mut g);
    for (earlier, later) in a.iter().zip(b) {
        let mut prior = (**earlier).clone();
        prior.epoch = later.epoch;
        assert_eq!(prior, *later);
    }
    assert_eq!(nodes, sample(&g).particles());
    assert_eq!(rigid, g.states());
}
#[test]
fn two_impulses_can_be_published_in_chronological_order_inside_one_public_tick() {
    let s = CLOTH
        .replace("0.001</max_step_size>", "0.1</max_step_size>")
        .replace("4000", "125000");
    let mut g = gripper::Gripper::new(&s).unwrap();
    g.sim.step_exact(3).unwrap();
    let rows = sample(&g).impacts_this_step();
    assert_eq!(rows.len(), 2);
    assert_eq!(
        rows[0].detection.unwrap().public_start,
        Duration::from_millis(200)
    );
    assert_eq!(
        rows[1].detection.unwrap().public_start,
        Duration::from_millis(200)
    );
    assert!(last_time(&rows[1]) > last_time(&rows[0]) + 0.045);
    assert!(last_time(&rows[1]) < 0.265);
    audit(&rows[0]);
    audit(&rows[1]);
    assert_eq!(
        sample(&g).latest_articulated_impact().unwrap(),
        &**rows.last().unwrap()
    );
    g.sim.step_exact(1).unwrap();
    let next = sample(&g).impacts_this_step();
    assert!(next.iter().all(|r| r.time > Duration::from_millis(300)));
    assert_eq!(sample(&g).articulated_impact_count(), 2 + next.len() as u64);
}
#[test]
fn unresolved_nonseparating_impact_discards_native_and_material_work_for_the_whole_tick() {
    let s = CLOTH
        .replace("4000", "125000")
        .replace("restitution=\"0.55\"", "restitution=\"0\"");
    let mut g = gripper::Gripper::new(&s).unwrap();
    g.sim.step_exact(213).unwrap();
    let rigid = g.states();
    let soft = sample(&g).particles().to_vec();
    let t = g.sim.elapsed();
    assert!(g.sim.step_exact(1).is_err());
    assert_eq!(g.sim.elapsed(), t);
    assert_eq!(g.states(), rigid);
    assert_eq!(sample(&g).particles(), soft);
    assert!(sample(&g).latest_articulated_impact().is_none());
    assert_eq!(sample(&g).articulated_impact_count(), 0);
    println!(
        "Zero-restitution finite sheet refused after {:?}, retaining the complete pre-impulse public state. Source: {:?}",
        t,
        g.sim.fault()
    );
    assert!(gripper::Gripper::new(&CLOTH.replace("4000", "9")).is_err());
    assert!(gripper::Gripper::new(&CLOTH.replace("recurrent_contact", "first_impact")).is_err());
}
#[test]
fn second_rotating_pad_encounter_matches_its_accumulated_true_torque_about_the_supported_axis() {
    let mut g = rotor::Rotor::new(TURN).unwrap();
    let mut rows = vec![];
    while g.sim.elapsed() < Duration::from_millis(320) {
        g.advance(1).unwrap();
        rows.extend(g.material().impacts_this_step().iter().cloned());
    }
    assert_eq!(rows.len(), 2);
    let pivot = DVec3::new(0., 0., 0.62);
    let torque = rows
        .iter()
        .map(|r| {
            audit(r);
            let l = &r.material.loads[0];
            (DVec3::from(l.site.point) - pivot)
                .cross(l.on_rigid.into())
                .z
        })
        .sum::<f64>();
    assert!((g.state().angular_velocity[2] - 4. - torque / 0.275).abs() < 3e-10);
    assert!((last_time(&rows[1]) - 0.25603343013).abs() < 2e-9);
    let loss = rows[1].material.dissipated_energy;
    assert!((loss - 0.212171215).abs() < 2e-8);
}
