// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Matched end-to-end ordinary-SDF comparison of two through eight local physics processes.
#[path = "support/grid.rs"]
mod grid;
use clap::Parser;
use gz_physics::PhysicsBackend;
use gz_physics_distributed::{
    SceneBackend,
    physics::{Body, ContactImpulse, GroundSupport},
    protocol::Observation,
};
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured};
use sdformat::NoResources;
use sha2::{Digest, Sha256};
use std::{error::Error, path::PathBuf, time::Instant};
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, default_value_t = 256)]
    bodies: usize,
    #[arg(long, default_value_t = 300)]
    ticks: usize,
    #[arg(long, default_value_t = 5)]
    trials: usize,
}
fn observe(sim: &gz_sim_core::Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<SceneBackend>()
        .unwrap()
}
fn state_digest(observations: &[Observation]) -> [u8; 32] {
    let mut bodies: Vec<&Body> = Vec::new();
    let mut contacts: Vec<&ContactImpulse> = Vec::new();
    let mut supports: Vec<&GroundSupport> = Vec::new();
    for w in observations {
        bodies.extend(&w.snapshot.bodies);
        contacts.extend(&w.snapshot.impulses);
        supports.extend(&w.snapshot.supports);
    }
    bodies.sort_by_key(|b| b.id);
    contacts.sort_by_key(|p| (p.a, p.b));
    supports.sort_by_key(|p| p.body);
    let mut out = Sha256::new();
    // This benchmark contains boxes only. Every construction value, state component,
    // pair impulse and floor impulse is included without depending on worker layout.
    for b in bodies {
        out.update(b.id.to_le_bytes());
        out.update([b.tier as u8, b.primitive as u8]);
        for v in b
            .position
            .into_iter()
            .chain(b.orientation)
            .chain(b.linear)
            .chain(b.angular)
            .chain(b.size)
            .chain([b.mass, b.friction])
        {
            out.update(v.to_le_bytes());
        }
    }
    out.update((contacts.len() as u64).to_le_bytes());
    for p in contacts {
        out.update(p.a.to_le_bytes());
        out.update(p.b.to_le_bytes());
        out.update(p.normal_impulse.to_le_bytes());
    }
    out.update((supports.len() as u64).to_le_bytes());
    for p in supports {
        out.update(p.body.to_le_bytes());
        out.update(p.normal_impulse.to_le_bytes());
    }
    out.finalize().into()
}
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    if a.trials == 0
        || a.ticks == 0
        || a.bodies == 0
        || a.bodies > 256
        || !a.bodies.is_multiple_of(2)
    {
        return Err(
            "choose a positive even population up to 256 and positive trial/tick counts".into(),
        );
    }
    let xml = grid::sdf(a.bodies);
    let widths = [2, 3, 4, 6, 8];
    let mut reference = Vec::new();
    println!(
        "bodies,trial,width,ticks,load_ms,p50_us,p95_us,p99_us,realtime_factor,request_bytes,reply_bytes"
    );
    for trial in 0..a.trials {
        for i in 0..widths.len() {
            let width = widths[(i + trial) % widths.len()];
            let engine = if width == 2 {
                PhysicsEngine::Distributed
            } else {
                PhysicsEngine::DistributedPool(width)
            };
            let load = Instant::now();
            let mut sim = load_world_configured(
                &xml,
                "grid.sdf",
                "default",
                &NoResources,
                WorldLoadOptions {
                    engine,
                    step: None,
                    noise_seed: [0; 32],
                    distributed_worker: Some(&a.worker),
                },
            )?;
            let load_ms = load.elapsed().as_secs_f64() * 1e3;
            let before = observe(&sim).local_worker_traffic().unwrap();
            let mut times = Vec::with_capacity(a.ticks);
            let mut sum = 0.;
            for tick in 0..a.ticks {
                let now = Instant::now();
                sim.step_exact(1)?;
                let elapsed = now.elapsed().as_secs_f64();
                sum += elapsed;
                times.push(elapsed * 1e6);
                let b = observe(&sim);
                let rows = b
                    .frame()
                    .map(|f| f.workers.as_slice())
                    .or_else(|| b.pool_frame().map(|f| f.workers.as_slice()))
                    .unwrap();
                let digest = state_digest(rows);
                if trial == 0 && width == 2 {
                    reference.push(digest);
                } else {
                    assert_eq!(digest, reference[tick], "width {width}, tick {}", tick + 1);
                }
            }
            times.sort_by(f64::total_cmp);
            let quantile = |p: usize| times[(p * times.len()).div_ceil(100) - 1];
            let after = observe(&sim).local_worker_traffic().unwrap();
            let mut request = 0;
            let mut reply = 0;
            for (a, b) in after.iter().zip(before) {
                request += a.request_bytes - b.request_bytes;
                reply += a.reply_bytes - b.reply_bytes;
            }
            println!(
                "{},{trial},{width},{},{load_ms:.3},{:.2},{:.2},{:.2},{:.4},{request},{reply}",
                a.bodies,
                a.ticks,
                quantile(50),
                quantile(95),
                quantile(99),
                a.ticks as f64 * 0.005 / sum
            );
        }
    }
    Ok(())
}
