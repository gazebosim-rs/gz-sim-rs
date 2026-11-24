// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, primary_contact, reset_loaded_world};
use std::time::Duration;
const WORLD: &str = include_str!("../../../examples/multifidelity_sliding_skew_edge.sdf");
fn load(s: &str) -> gz_sim_core::Simulation {
    load_world_with_sdf_step(
        s,
        "sliding-skew-edge.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
#[test]
fn sliding_skew_edges_deliver_dissipative_traction_and_a_rotating_normal() {
    let mut sim = load(WORLD);
    sim.step_exact(1).unwrap();
    let interval = primary_contact::last_refinement(&sim)
        .unwrap()
        .interval
        .clone();
    assert_eq!(interval.native_steps, 8);
    assert_eq!(interval.sliding_skew_edges.len(), 8);
    assert!(interval.sliding_vertices.is_empty() && interval.sticking_points.is_empty());
    let hit = interval.finite_point_impacts[0].resultant;
    let expected = DVec3::new(0.41591444586208415, -0.32836158504201857, 1.059822922582959);
    assert!(expected.distance(hit.impulse_on_foot.into()) < 2e-12);
    let mut j = DVec3::ZERO;
    let mut normal_j = DVec3::ZERO;
    let mut normal = 0.;
    for (i, r) in interval.sliding_skew_edges.iter().enumerate() {
        assert_eq!(r.start, Duration::from_nanos(500 * i as u64));
        assert_eq!(r.duration, Duration::from_nanos(500));
        assert_eq!(r.feature, gz_physics::SeparatingFeature::EdgePair(0, 1));
        assert!(
            r.minimum_slip_speed > 0. && r.minimum_inward_acceleration > 0. && r.friction_work < 0.
        );
        let migration = DVec3::from(r.point_end) - DVec3::from(r.point_start);
        assert!(migration.y > 1e-9 && migration.x.abs() < 1e-12 && migration.z.abs() < 1e-12);
        j += DVec3::from(r.impulse_on_foot);
        normal_j += DVec3::from(r.normal_impulse_on_foot);
        normal += r.normal_impulse;
    }
    // The reference comes from integrating the same initial states in the deck frame.
    assert!((j / 4e-6).distance(DVec3::new(3.6776588, -2.9034863, 9.3713195)) < 0.004);
    assert!(normal_j.x < -1e-11);
    let pair = interval
        .pairs
        .iter()
        .find(|q| (q.a, q.b) == (hit.foot.min(hit.support), hit.foot.max(hit.support)))
        .unwrap();
    let sign = if pair.a == hit.foot { 1. } else { -1. };
    assert!((normal + DVec3::from(hit.normal).dot(expected) - pair.normal_impulse).abs() < 2e-12);
    assert!(
        (normal_j + DVec3::from(hit.normal) * DVec3::from(hit.normal).dot(expected))
            .distance(sign * DVec3::from(pair.normal_impulse_on_a))
            < 2e-12
    );
    assert!(
        (expected + j).distance(
            sign * (DVec3::from(pair.normal_impulse_on_a)
                + DVec3::from(pair.tangent_impulse_on_a.unwrap()))
        ) < 2e-12
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        primary_contact::last_refinement(&sim).unwrap().interval,
        interval
    );
}
#[test]
fn a_competing_pad_or_missing_registration_keeps_native_contact() {
    let ordinary = WORLD.replace(
        "<follow_sliding_skew_edge>true</follow_sliding_skew_edge>",
        "",
    );
    let (a, b) = WORLD.split_once("<model name=\"lift\">").unwrap();
    let duplicate = format!(
        "{a}<model name=\"second\">{}</model><model name=\"lift\">{b}",
        b.split_once("</model>").unwrap().0
    );
    for xml in [ordinary, duplicate] {
        let mut sim = load(&xml);
        sim.step_exact(1).unwrap();
        assert!(
            primary_contact::last_refinement(&sim)
                .is_none_or(|r| r.interval.sliding_skew_edges.is_empty())
        );
    }
}
