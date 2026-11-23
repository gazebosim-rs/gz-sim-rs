// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, primary_contact, reset_loaded_world};
use std::time::Duration;
const WORLD: &str = include_str!("../../../examples/multifidelity_bound_vertex.sdf");
fn source() -> String {
    WORLD
        .replace("follow_sliding_vertex", "resolve_point_transfers")
        .replace("<native_steps>8", "<native_steps>2")
        .replace("<max_step_size>0.000004", "<max_step_size>0.000042")
}
fn world(text: &str) -> gz_sim_core::Simulation {
    load_world_with_sdf_step(
        text,
        "internal-transfer.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
#[test]
fn delivers_a_substep_internal_second_vertex_and_returns_to_native_at_an_unproved_switch() {
    let mut sim = world(&source());
    sim.step_exact(1).unwrap();
    let first = primary_contact::last_refinement(&sim)
        .unwrap()
        .interval
        .clone();
    assert_eq!(first.native_steps, 2);
    assert_eq!(first.finite_point_impacts.len(), 1);
    assert_eq!(first.point_transfers.len(), 1);
    let r = &first.point_transfers[0];
    assert_eq!(r.start, Duration::ZERO);
    assert_eq!(r.duration, Duration::from_micros(21));
    let [a, b] = r.intervals.as_slice() else {
        panic!("expected two sliding vertices")
    };
    assert!(!a.sticking && !b.sticking && a.offset == 0. && b.impact.is_none());
    let h = a.impact.unwrap();
    assert!((h.local_bracket[0] - 17.382975e-6).abs() < 2e-12);
    assert!((a.duration + b.duration - 0.000021).abs() < 2e-17);
    assert!(h.local_bracket[1] - h.local_bracket[0] < 2e-17);
    assert!(h.opposite_opening_speed > 0. && h.lost_energy > 0.);
    let j = DVec3::from(h.impulse_on_foot);
    assert!(j.distance(DVec3::new(-0.149154, -0.071664, 0.330953)) < 1e-6);
    assert!(
        (DVec3::from(a.impulse_on_foot) + j + DVec3::from(b.impulse_on_foot))
            .distance(r.impulse_on_foot.into())
            < 2e-13
    );
    let body = first.finite_point_impacts[0].resultant;
    let native_pair = first
        .pairs
        .iter()
        .find(|p| (p.a, p.b) == (body.foot.min(body.support), body.foot.max(body.support)))
        .unwrap();
    // The following 21 µs crosses a slip-from-rest transition and stays native.
    assert!(
        native_pair.normal_impulse
            > r.normal_impulse + DVec3::from(body.impulse_on_foot).dot(body.normal.into())
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        primary_contact::last_refinement(&sim).unwrap().interval,
        first
    );
}
#[test]
fn an_additional_touching_support_prevents_a_private_internal_transfer() {
    let s = source();
    let (r, d) = s.split_once("<model name=\"lift\">").unwrap();
    let text = format!(
        "{r}<model name=\"neighbor\">{}</model><model name=\"lift\">{d}",
        d.split_once("</model>").unwrap().0
    );
    let mut sim = world(&text);
    sim.step_exact(1).unwrap();
    assert!(
        primary_contact::last_refinement(&sim)
            .is_none_or(|r| r.interval.point_transfers.is_empty())
    );
}
