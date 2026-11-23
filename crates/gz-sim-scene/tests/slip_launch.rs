// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, primary_contact, reset_loaded_world};
const SCENE: &str = include_str!("../../../examples/multifidelity_skew_slip_launch.sdf");
fn load(xml: &str) -> gz_sim_core::Simulation {
    load_world_with_sdf_step(
        xml,
        "skew-slip-launch.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
#[test]
fn opening_a_new_native_step_less_than_one_nanosecond_after_launch_preserves_the_impulse() {
    let mut sim = load(SCENE);
    sim.step_exact(1).unwrap();
    let receipt = primary_contact::last_refinement(&sim)
        .unwrap()
        .interval
        .clone();
    assert_eq!(receipt.native_steps, 8);
    let x = &receipt.point_transfers;
    assert_eq!(x.len(), 2);
    let first = x[0].intervals[0].impact.unwrap();
    assert!((first.local_bracket[0] - 4.301373359826893e-6).abs() < 2e-14);
    assert!(x[0].intervals.last().unwrap().duration < 0.7e-9);
    let final_hit = x[1].intervals[0].impact.unwrap();
    let physical = DVec3::new(
        -0.019213782333591202,
        -0.026549285354606843,
        0.06554491542150198,
    );
    assert!(DVec3::from(final_hit.impulse_on_foot).distance(physical) < 2e-9);
    assert!(
        (x[1].start.as_secs_f64() + final_hit.local_bracket[0] - 1.8012664740444875e-5).abs()
            < 2e-13
    );
    let mut witnessed = 0;
    for tr in x {
        for p in &tr.intervals {
            if let Some(g) = p.slip_growth {
                assert!(g.acceleration > 0. && g.initial_component >= 0.);
                assert!(
                    DVec3::from(p.end_material_velocity).dot(g.direction)
                        >= g.initial_component + g.acceleration * p.duration - 2e-11
                );
                witnessed += 1;
            }
        }
    }
    assert!(witnessed >= 3);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        primary_contact::last_refinement(&sim).unwrap().interval,
        receipt
    );
}
#[test]
fn the_opt_in_and_competing_support_gate_zero_speed_following() {
    let ordinary = SCENE.replace(
        "<follow_skew_slip_launch>true</follow_skew_slip_launch>",
        "",
    );
    let (a, b) = SCENE.split_once("<model name=\"lift\">").unwrap();
    let duplicate = format!(
        "{a}<model name=\"second\">{}</model><model name=\"lift\">{b}",
        b.split_once("</model>").unwrap().0
    );
    for xml in [ordinary, duplicate] {
        let mut sim = load(&xml);
        sim.step_exact(1).unwrap();
        if let Some(r) = primary_contact::last_refinement(&sim) {
            assert!(
                r.interval
                    .sliding_skew_edges
                    .iter()
                    .all(|s| s.slip_growth.is_none())
            );
            assert!(
                r.interval
                    .point_transfers
                    .iter()
                    .flat_map(|p| &p.intervals)
                    .all(|s| s.slip_growth.is_none())
            );
        }
    }
}
