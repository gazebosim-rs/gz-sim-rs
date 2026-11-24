// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cost of changing a single translating deck, adding one, and entering native contact.
use gz_physics::{BodyId, BodySpec, DynamicBackend, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::{
    error::Error,
    time::{Duration, Instant},
};
const TICK: Duration = Duration::from_millis(10);
fn deck(x: f64, z: f64, v: f64) -> BodySpec {
    BodySpec {
        shape: Shape::Box {
            half_extents: [0.5, 0.5, 0.1],
        },
        position: [x, 0., z],
        velocity: [v, 0., 0.],
        mass: Some(1.),
    }
}
fn read(backend: &MultiFidelityBackend, ids: &[BodyId]) {
    let mut sum = 0.;
    backend.read_states(&mut |states| {
        for &id in ids {
            sum += states.state(id).unwrap().position[0];
        }
    });
    std::hint::black_box(sum);
}
fn main() -> Result<(), Box<dyn Error>> {
    let count: usize = std::env::args()
        .nth(1)
        .ok_or("usage: fleet_changes NUMBER_OF_DECKS")?
        .parse()?;
    if count < 2 {
        return Err("at least two decks required".into());
    }
    let mut backend = MultiFidelityBackend::new([0., 0., -9.81])?;
    let mut ids = Vec::with_capacity(count + 20);
    for i in 0..count {
        ids.push(backend.spawn_kinematic_link(
            deck(4. * i as f64, 0., 0.25),
            Default::default(),
            None,
        )?);
    }
    read(&backend, &ids);
    let (mut edits, mut inserts, mut readbacks) = (Vec::new(), Vec::new(), Vec::new());
    let mut x = 0.;
    let mut v = 0.25;
    for i in 0..20 {
        backend.step(TICK)?;
        x += v * TICK.as_secs_f64();
        v = -v;
        let now = Instant::now();
        backend.set_world_velocity(ids[0], [v, 0., 0.], [0.; 3])?;
        edits.push(now.elapsed());
        backend.step(TICK)?;
        x += v * TICK.as_secs_f64();
        let late = 4. * (count + i) as f64;
        let now = Instant::now();
        let id = backend.spawn_kinematic_link(deck(late, 0., -0.125), Default::default(), None)?;
        inserts.push(now.elapsed());
        ids.push(id);
        let now = Instant::now();
        read(&backend, &ids);
        readbacks.push(now.elapsed());
        assert!((backend.state(ids[0]).unwrap().position[0] - x).abs() < 1e-12);
        assert_eq!(backend.state(id).unwrap().position[0], late);
    }
    backend.step(TICK)?;
    x += v * TICK.as_secs_f64();
    let before = backend.state(ids[count - 1]).unwrap();
    assert!((before.position[0] - 4. * (count - 1) as f64 - 0.1025).abs() < 1e-8);
    let ray = [before.position[0], 0., 1.];
    assert!((backend.cast_ray(ray, [0., 0., -1.], 2., None)?.unwrap() - 0.9).abs() < 1e-12);
    let mut load = deck(x, 0.201, 0.);
    load.shape = Shape::Box {
        half_extents: [0.1; 3],
    };
    let now = Instant::now();
    let payload = backend.spawn_link(load, Default::default(), None)?;
    let materialize = now.elapsed();
    assert_eq!(backend.state(ids[count - 1]).unwrap(), before);
    for _ in 0..4 {
        backend.step(TICK)?;
    }
    assert_eq!(backend.active_contact_pairs(), 1);
    assert!((backend.state(payload).unwrap().position[2] - 0.2).abs() < 0.01);
    for (name, mut data) in [
        ("edit", edits),
        ("insertion", inserts),
        ("first_full_read", readbacks),
    ] {
        data.sort();
        println!(
            "decks={count} operation={name} n={} p50_us={:.3} p95_us={:.3}",
            data.len(),
            data[9].as_secs_f64() * 1e6,
            data[18].as_secs_f64() * 1e6
        );
    }
    println!(
        "full_native_transition_us={:.3} contacts={}",
        materialize.as_secs_f64() * 1e6,
        backend.active_contact_pairs()
    );
    Ok(())
}
