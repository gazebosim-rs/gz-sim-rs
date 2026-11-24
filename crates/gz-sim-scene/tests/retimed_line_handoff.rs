// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, primary_contact};
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

const WORLD: &str = include_str!("../../../examples/multifidelity_resumed_edge_slip.sdf");

fn world(xml: &str) -> gz_sim_core::Simulation {
    load_world_with_sdf_step(
        xml,
        "line-handoff.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

#[test]
fn a_retimed_point_hands_the_two_site_roll_to_native_on_the_original_clock() {
    let mut ordinary = world(WORLD);
    let mut sim = world(&WORLD.replace(
        "</pair>",
        "<retime_finite_transfer>true</retime_finite_transfer></pair>",
    ));
    for _ in 0..19 {
        ordinary.step_exact(1).unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(
            primary_contact::last_refinement(&sim),
            primary_contact::last_refinement(&ordinary)
        );
    }
    let calls = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&calls);
    primary_contact::set_refined_observer(&mut sim, move |_, native| {
        sink.lock().unwrap().push((
            native.elapsed,
            native.duration,
            native.point_span.map(|s| s.reaction().1),
            native.finite_point_impacts.len(),
            native
                .contact_patches()
                .iter()
                .flat_map(|p| &p.points)
                .map(|p| p.normal_impulse)
                .sum::<f64>(),
        ));
    })
    .unwrap();
    sim.step_exact(1).unwrap();
    let r = primary_contact::last_refinement(&sim).unwrap();
    let samples = calls.lock().unwrap();
    ordinary.step_exact(1).unwrap();
    let previous = &primary_contact::last_refinement(&ordinary)
        .unwrap()
        .interval;
    assert_eq!(previous.native_steps, 8);
    let previous_covered: Duration = previous
        .sliding_vertices
        .iter()
        .map(|s| s.duration)
        .chain(previous.point_transfers.iter().map(|s| s.duration))
        .sum();
    assert_eq!(previous_covered, Duration::from_micros(250));
    assert_eq!(r.sim_time, Duration::from_millis(10));
    assert_eq!(samples.len(), 8);
    assert_eq!(r.interval.native_steps, 8);
    // The independently integrated return is at 255801.884 ns in this tick.
    assert_eq!(samples[0].0, Duration::from_nanos(255801));
    assert_eq!(samples.last().unwrap().0, Duration::from_micros(500));
    assert!(samples.iter().map(|s| s.1).sum::<Duration>() == Duration::from_micros(500));
    assert!(samples[0].2.unwrap() > 0.);
    assert_eq!(samples[0].4, 0.);
    assert!(samples[1..].iter().any(|s| s.4 > 0.));
    let recorded_normal = samples
        .iter()
        .map(|s| s.2.unwrap_or_default() + s.4)
        .sum::<f64>();
    assert!((recorded_normal - r.requested_pairs[0].normal_impulse).abs() < 1e-12);
    assert!(samples[1..].iter().all(|s| s.2.is_none() && s.3 == 0));
}
