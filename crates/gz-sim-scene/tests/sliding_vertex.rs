// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, primary_contact, reset_loaded_world};
use std::time::Duration;
const WORLD: &str = include_str!("../../../examples/multifidelity_bound_vertex.sdf");
fn load(s: &str) -> gz_sim_core::Simulation {
    load_world_with_sdf_step(
        s,
        "vertex.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
#[test]
fn a_sliding_endpoint_carries_its_sustained_impulse_and_releases_the_feature_on_time() {
    let mut sim = load(WORLD);
    sim.step_exact(1).unwrap();
    let first = primary_contact::last_refinement(&sim)
        .unwrap()
        .interval
        .clone();
    assert_eq!(first.native_steps, 8);
    assert_eq!(first.sliding_vertices.len(), 8);
    let h = first.finite_point_impacts[0].resultant;
    let mut j = DVec3::ZERO;
    let mut elapsed = Duration::ZERO;
    for span in &first.sliding_vertices {
        assert_eq!(span.start, elapsed);
        elapsed += span.duration;
        j += DVec3::from(span.impulse_on_foot);
        assert!(
            span.minimum_inward_acceleration > 0.
                && span.minimum_slip_speed > 0.
                && span.friction_work < 0.
        );
    }
    assert_eq!(elapsed, Duration::from_micros(4));
    assert!((j.z - 4e-6 * 8.71218547).abs() < 1e-8);
    let pair = first
        .pairs
        .iter()
        .find(|p| (p.a, p.b) == (h.foot.min(h.support), h.foot.max(h.support)))
        .unwrap();
    assert!(pair.active_at_end);
    assert!((pair.normal_impulse - j.z - DVec3::from(h.impulse_on_foot).z).abs() < 2e-13);
    for _ in 0..3 {
        sim.step_exact(1).unwrap();
        let current = primary_contact::last_refinement(&sim).unwrap();
        assert_eq!(current.sim_time, sim.elapsed());
        let duration = current
            .interval
            .sliding_vertices
            .iter()
            .map(|s| s.duration)
            .sum::<Duration>();
        assert_eq!(duration, Duration::from_micros(4));
        assert!(current.interval.finite_point_impacts.is_empty());
    }
    // A different sole vertex arrives at 17.382975 microseconds. No
    // certified span on the original vertex may cross that physical event.
    sim.step_exact(1).unwrap();
    let delivered = sim
        .world()
        .resource::<gz_physics::PhysicsBackend>()
        .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
        .unwrap()
        .rigid_contact_observations();
    assert!(delivered.iter().any(
        |p| (p.a, p.b) == (h.foot.min(h.support), h.foot.max(h.support)) && p.normal_impulse > 0.
    ));
    if let Some(current) =
        primary_contact::last_refinement(&sim).filter(|r| r.sim_time == sim.elapsed())
    {
        for span in &current.interval.sliding_vertices {
            if span.point_start[1] < 0. {
                assert!(span.start + span.duration < Duration::from_nanos(1384))
            }
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
fn a_competing_deck_or_missing_registration_keeps_the_native_contacts() {
    let ordinary = WORLD.replace("<follow_sliding_vertex>true</follow_sliding_vertex>", "");
    let (rocker, deck) = WORLD.split_once("<model name=\"lift\">").unwrap();
    let duplicate = format!(
        "{rocker}<model name=\"other\">{}</model><model name=\"lift\">{deck}",
        deck.split_once("</model>").unwrap().0
    );
    for s in [ordinary, duplicate] {
        let mut world = load(&s);
        world.step_exact(1).unwrap();
        assert!(
            primary_contact::last_refinement(&world)
                .is_none_or(|r| r.interval.sliding_vertices.is_empty())
        );
    }
}
