// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, primary_contact::last_refinement,
    reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

const SDF: &str = include_str!("../../../examples/multifidelity_locked_sliding_corner.sdf");

fn state(s: &Simulation, name: &str) -> BodyState {
    #[allow(deprecated)]
    let id = s
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<BodyHandle>()
        .unwrap()
        .id();
    s.world().resource::<PhysicsBackend>().state(id).unwrap()
}

#[test]
fn locked_child_transmits_the_single_corner_impulse_at_first_touch() {
    let mut s = load_world_with_sdf_step(
        SDF,
        "locked.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let f = state(&s, "default::rocker::foot");
    let upper = state(&s, "default::rocker::upper");
    let deck = state(&s, "default::lift::pad");
    let q = DQuat::from_array(f.orientation);
    let r = [-0.18, 0.18]
        .into_iter()
        .flat_map(|x| [-0.1, 0.1].map(|y| q * DVec3::new(x, y, -0.13)))
        .min_by(|a, b| a.z.total_cmp(&b.z))
        .unwrap();
    let com = (DVec3::from(f.position) + DVec3::from(upper.position)) * 0.5;
    let deck_v = DVec3::from(deck.velocity);
    let v = (DVec3::from(f.velocity) + DVec3::from(upper.velocity)) * 0.5;
    let incoming = v - deck_v;
    let expected_time = (com.z + r.z - deck.position[2] - 0.06) / (-incoming.z);
    let mut results = Vec::new();
    for _ in 0..2 {
        for tick in 1..=25 {
            s.step_exact(1).unwrap();
            if let Some(record) = last_refinement(&s).filter(|a| a.iteration == tick) {
                for hit in &record.interval.corner_impacts {
                    assert_eq!(tick, 5);
                    let time = (s.elapsed() - s.step_duration() + hit.elapsed).as_secs_f64();
                    assert!((time - expected_time).abs() < 1e-9);
                    assert_eq!(hit.elapsed, Duration::from_nanos(187500));
                    let j = DVec3::from(hit.impulse_on_foot);
                    let post = incoming + j * 0.5 + (r.cross(j) / 0.05).cross(r);
                    assert!(incoming.distance(hit.before_relative_velocity.into()) < 1e-12);
                    assert!(post.distance(hit.after_relative_velocity.into()) < 1e-12);
                    assert!(j.z > 1. && post.z.abs() < 1e-12 && post.truncate().length() > 0.1);
                    assert!(
                        (j.truncate() + 0.5 * j.z * post.truncate().normalize()).length() < 1e-12
                    );
                    let energy = (v + j * 0.5).length_squared() - v.length_squared()
                        + 0.025 * (r.cross(j) / 0.05).length_squared();
                    assert!((energy - hit.kinetic_energy_change).abs() < 1e-12);
                    assert!((j.dot(deck_v) - energy - hit.dissipated_energy).abs() < 1e-12);
                    results.push(*hit);
                }
            }
        }
        reset_loaded_world(&mut s, |_| ()).unwrap();
        assert!(last_refinement(&s).is_none());
    }
    assert_eq!(results.len(), 2);
    assert_eq!(results[0], results[1]);
}
