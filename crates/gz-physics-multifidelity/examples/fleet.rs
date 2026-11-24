// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

//! Release benchmark of the CPU backend through the world's Bevy physics schedule.
//! Arguments: number of moving decks, grid spacing in metres, payload stride
//! (zero disables payloads), and optional "observe" for all body poses per tick.
//! Timing excludes construction and the final correctness check.
use bevy_ecs::prelude::*;
use gz_physics::{BodyId, BodySpec, BodyState, DynamicBackend, PhysicsBackend, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{Phase, Simulation};
use std::{
    error::Error,
    time::{Duration, Instant},
};

const DT: Duration = Duration::from_millis(10);
const WARMUP: usize = 20;
const SAMPLES: usize = 100;
const SPEED: f64 = 0.25;

#[derive(Component)]
struct FleetBody(BodyId);

#[derive(Component)]
struct ObservedState(BodyState);

fn commit_states(
    physics: Res<PhysicsBackend>,
    mut bodies: Query<(&FleetBody, &mut ObservedState)>,
) {
    physics.read_states(&mut |states| {
        for (body, mut state) in &mut bodies {
            state.0 = states.state(body.0).expect("registered fleet body");
        }
    });
}

fn initial_xy(index: usize, columns: usize, spacing: f64) -> [f64; 2] {
    [
        (index % columns) as f64 * spacing,
        (index / columns) as f64 * spacing,
    ]
}

fn body(position: [f64; 3], half_extents: [f64; 3], speed: f64) -> BodySpec {
    BodySpec {
        shape: Shape::Box { half_extents },
        position,
        velocity: [speed, 0.0, 0.0],
        mass: Some(1.0),
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let count: usize = args.next().ok_or("missing deck count")?.parse()?;
    let spacing: f64 = args.next().ok_or("missing grid spacing")?.parse()?;
    let stride: usize = args.next().ok_or("missing payload stride")?.parse()?;
    let observe = match args.next().as_deref() {
        Some("observe") => true,
        None => false,
        _ => return Err("expected optional observe argument".into()),
    };
    if count == 0 || !spacing.is_finite() || spacing <= 1.0 || args.next().is_some() {
        return Err("expected positive deck count, grid spacing > 1 m, and payload stride".into());
    }

    let mut backend = MultiFidelityBackend::new([0.0, 0.0, -9.81])?;
    let mut decks = Vec::with_capacity(count);
    let mut loads: Vec<(usize, BodyId)> = Vec::new();
    let columns = count.isqrt() + usize::from(count.isqrt().pow(2) != count);
    for i in 0..count {
        let [x, y] = initial_xy(i, columns, spacing);
        decks.push(backend.spawn_kinematic_link(
            body([x, y, 0.0], [0.5, 0.5, 0.1], SPEED),
            Default::default(),
            None,
        )?);
        if stride != 0 && i % stride == 0 {
            let load = body([x, y, 0.201], [0.1; 3], 0.0);
            loads.push((i, backend.spawn_link(load, Default::default(), None)?));
        }
    }
    let mut sim = Simulation::new(DT)?;
    for &id in decks.iter().chain(loads.iter().map(|(_, id)| id)) {
        if observe {
            sim.world_mut().spawn((
                FleetBody(id),
                ObservedState(backend.state(id).ok_or("lost body")?),
            ));
        } else {
            sim.world_mut().spawn(FleetBody(id));
        }
    }
    gz_physics::install(&mut sim, backend)?;
    if observe {
        sim.add_systems(Phase::PhysicsCommit, commit_states);
    }
    sim.step_exact(WARMUP as u64)?;
    let mut latency = Vec::with_capacity(SAMPLES);
    let start = Instant::now();
    for _ in 0..SAMPLES {
        let tick = Instant::now();
        sim.step_exact(1)?;
        latency.push(tick.elapsed());
    }
    let elapsed = start.elapsed();
    latency.sort_unstable();

    let backend = sim.world().resource::<PhysicsBackend>();
    let active_contacts = backend
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .active_contact_pairs();
    assert_eq!(
        active_contacts,
        loads.len(),
        "each payload must contact exactly its own deck"
    );
    let expected_time = DT * (WARMUP + SAMPLES) as u32;
    assert_eq!(sim.elapsed(), expected_time);
    let readback_start = Instant::now();
    if observe {
        for entity in sim.world().iter_entities() {
            if let Some(body) = entity.get::<FleetBody>() {
                assert_eq!(
                    entity.get::<ObservedState>().unwrap().0,
                    backend.state(body.0).unwrap()
                );
            }
        }
    }
    let dx = SPEED * expected_time.as_secs_f64();
    let mut max_error: f64 = 0.0;
    for (i, id) in decks.into_iter().enumerate() {
        let [x, y] = initial_xy(i, columns, spacing);
        let state = backend.state(id).ok_or("lost deck")?;
        assert_eq!(state.velocity, [SPEED, 0.0, 0.0]);
        assert_eq!(state.orientation, [0.0, 0.0, 0.0, 1.0]);
        for (actual, expected) in state.position.into_iter().zip([x + dx, y, 0.0]) {
            let error = (actual - expected).abs();
            assert!(error < 1e-8, "deck {i}: {actual} vs {expected}");
            max_error = max_error.max(error);
        }
    }
    for &(i, id) in &loads {
        let [x, _] = initial_xy(i, columns, spacing);
        let state = backend.state(id).ok_or("lost payload")?;
        assert!((state.position[0] - x - dx).abs() < 0.05);
        assert!((state.position[2] - 0.2).abs() < 0.01);
        assert!((state.velocity[0] - SPEED).abs() < 0.01);
    }
    println!(
        "decks={count} active_pairs={active_contacts} observe_each_tick={observe} final_readback_ms={:.3} spacing_m={spacing} supported_payloads={} dt_ms={} warmup_steps={WARMUP} measured_steps={SAMPLES} wall_s={:.6} real_time_factor={:.4} p50_ms={:.4} p99_ms={:.4} max_pose_error_m={max_error:.3e}",
        readback_start.elapsed().as_secs_f64() * 1000.0,
        loads.len(),
        DT.as_millis(),
        elapsed.as_secs_f64(),
        DT.as_secs_f64() * SAMPLES as f64 / elapsed.as_secs_f64(),
        latency[SAMPLES / 2 - 1].as_secs_f64() * 1000.0,
        latency[SAMPLES * 99 / 100 - 1].as_secs_f64() * 1000.0
    );
    Ok(())
}
