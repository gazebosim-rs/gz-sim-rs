// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure the normal SDF stepping path with four continuously ground-supported sphere stacks.
use clap::Parser;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{
    SceneBackend,
    physics::{Body, ContactImpulse, GroundSupport},
};
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured};
use sdformat::NoResources;
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{error::Error, path::PathBuf, time::Instant};

const WARMUP: usize = 110;
const TICKS: usize = 100;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, default_value_t = 1)]
    round: usize,
    /// Descriptive label for this build's bounds, recorded with the timing row.
    #[arg(long)]
    bound: String,
}
#[derive(Serialize)]
struct Timing {
    round: usize,
    bound: String,
    warmup_ticks: usize,
    ticks: usize,
    bodies: usize,
    planned_body_solves: usize,
    min_components: usize,
    max_components: usize,
    min_populated_workers: usize,
    max_populated_workers: usize,
    positive_pair_receipts: usize,
    positive_floor_receipts: usize,
    integrated_pair_impulse_ns: f64,
    integrated_floor_impulse_ns: f64,
    physics_sha256: String,
    elapsed_ms: f64,
    tick_p50_us: f64,
    tick_p95_us: f64,
    tick_p99_us: f64,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let mut sim = load_world_configured(
        include_str!("../../../examples/distributed_diagonal_stacks.sdf"),
        "distributed_diagonal_stacks.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::DistributedIslands(4),
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(&args.worker),
        },
    )?;
    sim.step_exact(WARMUP as u64)?;
    let mut sample = Timing {
        round: args.round,
        bound: args.bound,
        warmup_ticks: WARMUP,
        ticks: TICKS,
        bodies: 8,
        planned_body_solves: 0,
        min_components: usize::MAX,
        max_components: 0,
        min_populated_workers: usize::MAX,
        max_populated_workers: 0,
        positive_pair_receipts: 0,
        positive_floor_receipts: 0,
        integrated_pair_impulse_ns: 0.,
        integrated_floor_impulse_ns: 0.,
        physics_sha256: String::new(),
        elapsed_ms: 0.,
        tick_p50_us: 0.,
        tick_p95_us: 0.,
        tick_p99_us: 0.,
    };
    let mut times = Vec::with_capacity(TICKS);
    let mut physics = Sha256::new();
    for tick in WARMUP + 1..=WARMUP + TICKS {
        let start = Instant::now();
        sim.step_exact(1)?;
        times.push(start.elapsed().as_secs_f64() * 1e6);
        let scene = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<SceneBackend>()
            .unwrap();
        let frame = scene.pool_frame().ok_or("missing four-worker frame")?;
        let fine = scene
            .last_island_refined_bodies()
            .ok_or("missing island receipt")?;
        let mut bodies: Vec<&Body> = Vec::new();
        let mut pairs: Vec<&ContactImpulse> = Vec::new();
        let mut floors: Vec<&GroundSupport> = Vec::new();
        let mut components = 0;
        let mut populated = 0;
        for worker in &frame.workers {
            bodies.extend(&worker.snapshot.bodies);
            pairs.extend(&worker.snapshot.impulses);
            floors.extend(&worker.snapshot.supports);
            components += worker.components.len();
            populated += usize::from(!worker.snapshot.bodies.is_empty());
        }
        bodies.sort_by_key(|b| b.id);
        pairs.sort_by_key(|p| (p.a, p.b));
        floors.sort_by_key(|p| p.body);
        if frame.tick != tick as u64
            || bodies.len() != 8
            || !bodies.iter().map(|b| b.id).eq(fine.iter().copied())
            || pairs.len() != 4
            || floors.len() != 4
            || !pairs
                .iter()
                .all(|p| p.normal_impulse.is_finite() && p.normal_impulse > 0.)
            || !floors
                .iter()
                .all(|p| p.normal_impulse.is_finite() && p.normal_impulse > 0.)
        {
            return Err(format!("incomplete loaded barrier at {tick}: fine={fine:?}, pairs={pairs:?}, supports={floors:?}").into());
        }
        // A positive load must connect both members of each column and reach its floor.
        for chunk in bodies.chunks_exact(2) {
            let [a, b] = chunk else { unreachable!() };
            let base = if a.position[2] < b.position[2] { a } else { b };
            if a.position[..2] != b.position[..2]
                || !pairs.iter().any(|p| (p.a, p.b) == (a.id, b.id))
                || !floors.iter().any(|p| p.body == base.id)
            {
                return Err(format!("column {} lost support at {tick}", base.id).into());
            }
        }
        sample.min_components = sample.min_components.min(components);
        sample.max_components = sample.max_components.max(components);
        sample.min_populated_workers = sample.min_populated_workers.min(populated);
        sample.max_populated_workers = sample.max_populated_workers.max(populated);
        sample.planned_body_solves += bodies.len() + 9 * fine.len();
        sample.positive_pair_receipts += pairs.len();
        sample.positive_floor_receipts += floors.len();
        sample.integrated_pair_impulse_ns += pairs.iter().map(|p| p.normal_impulse).sum::<f64>();
        sample.integrated_floor_impulse_ns += floors.iter().map(|p| p.normal_impulse).sum::<f64>();
        // Hash the canonical body poses, velocities and contact receipts at every
        // measured barrier; ownership, components and timing are deliberately excluded.
        physics.update(bincode::serialize(&(frame.tick, bodies, pairs, floors))?);
    }
    sample.physics_sha256 = format!("{:x}", physics.finalize());
    sample.elapsed_ms = times.iter().sum::<f64>() * 0.001;
    times.sort_unstable_by(f64::total_cmp);
    let p = |q: usize| times[(times.len() * q).div_ceil(100) - 1];
    (sample.tick_p50_us, sample.tick_p95_us, sample.tick_p99_us) = (p(50), p(95), p(99));
    println!("{}", serde_json::to_string(&sample)?);
    Ok(())
}
