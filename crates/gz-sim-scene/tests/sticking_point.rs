// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, primary_contact, reset_loaded_world};
use std::time::Duration;
const WORLD: &str = include_str!("../../../examples/multifidelity_sticking_point.sdf");
fn load(s: &str) -> gz_sim_core::Simulation {
    load_world_with_sdf_step(
        s,
        "sticking-point.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
#[test]
fn spatial_skew_edges_stick_at_one_material_point_with_two_signed_reactions() {
    let mut sim = load(WORLD);
    sim.step_exact(1).unwrap();
    let first = primary_contact::last_refinement(&sim)
        .unwrap()
        .interval
        .clone();
    assert_eq!(first.native_steps, 8);
    assert_eq!(first.sticking_points.len(), 8);
    let h = first.finite_point_impacts[0].resultant;
    assert!(h.sticking);
    let physical = DVec3::new(0.0767799317809399, -0.17015031875258743, 0.5482775462082327);
    assert!(physical.distance(h.impulse_on_foot.into()) < 2e-12);
    let mut j = DVec3::ZERO;
    let mut normal = DVec3::ZERO;
    let mut amount = 0.;
    for (i, s) in first.sticking_points.iter().enumerate() {
        assert_eq!(s.start, Duration::from_nanos(500 * i as u64));
        assert_eq!(s.duration, Duration::from_nanos(500));
        assert_eq!(s.feature, gz_physics::SeparatingFeature::EdgePair(0, 1));
        assert!(s.minimum_normal_force > 0. && s.minimum_cone_margin > 0.);
        assert!(DVec3::from(s.point_start).distance(s.point_end.into()) < 2e-13);
        j += DVec3::from(s.impulse_on_foot);
        normal += DVec3::from(s.normal_impulse_on_foot);
        amount += s.normal_impulse;
    }
    assert!((j / 4e-6).distance(DVec3::new(2.121801, -4.499998, 14.732404)) < 0.003);
    let reaction = first
        .pairs
        .iter()
        .find(|p| (p.a, p.b) == (h.foot.min(h.support), h.foot.max(h.support)))
        .unwrap();
    let sign = if reaction.a == h.foot { 1. } else { -1. };
    let hj = DVec3::from(h.impulse_on_foot);
    let n = DVec3::from(h.normal);
    assert!((reaction.normal_impulse - amount - hj.dot(n)).abs() < 3e-13);
    assert!(
        (sign * DVec3::from(reaction.normal_impulse_on_a)).distance(normal + n * hj.dot(n)) < 3e-13
    );
    assert!(
        (sign
            * (DVec3::from(reaction.normal_impulse_on_a)
                + DVec3::from(reaction.tangent_impulse_on_a.unwrap())))
        .distance(j + hj)
            < 3e-13
    );
    for _ in 0..16 {
        sim.step_exact(1).unwrap();
        let r = primary_contact::last_refinement(&sim).unwrap();
        assert_eq!(r.sim_time, sim.elapsed());
        assert!(r.interval.finite_point_impacts.is_empty());
        // Once the impact interval finishes, the scheduler can carry the
        // same proved point through a whole public tick in one native call.
        assert_eq!(
            r.interval.sticking_points.len(),
            r.interval.native_steps as usize
        );
        assert_eq!(
            r.interval
                .sticking_points
                .iter()
                .map(|s| s.duration)
                .sum::<Duration>(),
            Duration::from_micros(4)
        );
        for s in &r.interval.sticking_points {
            assert!(DVec3::from(s.point_end).distance(h.point.into()) < 3e-12)
        }
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        primary_contact::last_refinement(&sim).unwrap().interval,
        first
    );
}
#[test]
fn competitor_and_disabled_following_keep_the_native_contact_live() {
    let ordinary = WORLD.replace("<follow_sticking_point>true</follow_sticking_point>", "");
    let (r, d) = WORLD.split_once("<model name=\"lift\">").unwrap();
    let duplicate = format!(
        "{r}<model name=\"second\">{}</model><model name=\"lift\">{d}",
        d.split_once("</model>").unwrap().0
    );
    for xml in [ordinary, duplicate] {
        let mut sim = load(&xml);
        sim.step_exact(1).unwrap();
        assert!(
            primary_contact::last_refinement(&sim)
                .is_none_or(|r| r.interval.sticking_points.is_empty())
        );
    }
}
