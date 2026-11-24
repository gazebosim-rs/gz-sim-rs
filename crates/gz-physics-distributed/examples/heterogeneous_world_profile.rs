// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the pool with independent serial native worlds at the same public barriers.
#[path = "support/heterogeneous_worlds.rs"]
mod scene;
use clap::Parser;
use gz_physics_distributed::{
    physics::{GroundPlane, Island, SimulationStep, Snapshot},
    pool::WorkerPool,
};
use scene::{GRAVITY, Layout, TICKS, WARMUP};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{error::Error, path::PathBuf, time::Instant};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[arg(long, value_enum)]
    layout: Layout,
    /// Run one independent serial ECS/Rapier control per expected native world.
    #[arg(long, conflicts_with = "worker")]
    local: bool,
    #[arg(long, required_unless_present = "local")]
    worker: Option<PathBuf>,
}

#[derive(Serialize)]
struct Timing {
    layout: Layout,
    worlds: usize,
    bodies: usize,
    ticks: usize,
    body_substeps: usize,
    positive_pair_receipts: usize,
    positive_floor_receipts: usize,
    elapsed_ms: f64,
    tick_p50_us: f64,
    tick_p95_us: f64,
    physics_sha256: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    world_mean_us: Option<Vec<f64>>,
}

struct Witness {
    ids: Vec<u64>,
    pairs: Vec<(u64, u64)>,
    floors: Vec<u64>,
    digest: Sha256,
}
impl Witness {
    fn new(groups: &[Vec<u64>]) -> Self {
        Self {
            ids: groups.iter().flatten().copied().collect(),
            pairs: groups
                .iter()
                .flat_map(|g| g[1..].iter().map(|&b| (g[0], b)))
                .collect(),
            floors: groups.iter().map(|g| g[0]).collect(),
            digest: Sha256::new(),
        }
    }
    fn check<'a>(&mut self, tick: usize, frames: impl Iterator<Item = &'a Snapshot>) -> Result<()> {
        let mut bodies = Vec::new();
        let mut pairs = Vec::new();
        let mut floors = Vec::new();
        for frame in frames {
            if frame.tick != tick as u64 {
                return Err("incomplete barrier".into());
            }
            bodies.extend(&frame.bodies);
            pairs.extend(&frame.impulses);
            floors.extend(&frame.supports);
        }
        bodies.sort_by_key(|b| b.id);
        pairs.sort_by_key(|p| (p.a, p.b));
        floors.sort_by_key(|p| p.body);
        if !bodies.iter().map(|b| b.id).eq(self.ids.iter().copied())
            || !pairs
                .iter()
                .map(|p| (p.a, p.b))
                .eq(self.pairs.iter().copied())
            || !floors
                .iter()
                .map(|p| p.body)
                .eq(self.floors.iter().copied())
            || !pairs
                .iter()
                .map(|p| p.normal_impulse)
                .chain(floors.iter().map(|p| p.normal_impulse))
                .all(|j| j.is_finite() && j > 0.)
        {
            return Err(format!("unexpected contacts at tick {tick}: {} pair and {} floor contacts, expected {} and {}",pairs.len(),floors.len(),self.pairs.len(),self.floors.len()).into());
        }
        self.digest
            .update(bincode::serialize(&(tick as u64, bodies, pairs, floors))?);
        Ok(())
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let groups = scene::seeds(args.layout);
    let ids: Vec<Vec<_>> = groups
        .iter()
        .map(|g| g.iter().map(|b| b.id).collect())
        .collect();
    let mut witness = Witness::new(&ids);
    let ground = Some(GroundPlane::new(0.)?);
    let mut ticks = Vec::with_capacity(TICKS);
    let mut world_mean_us = None;
    if args.local {
        let mut worlds = groups
            .into_iter()
            .map(|bodies| {
                Island::restore(&Snapshot {
                    step: SimulationStep::Ms10,
                    ground,
                    bodies,
                    ..Default::default()
                })
            })
            .collect::<std::result::Result<Vec<_>, _>>()?;
        let mut world_us = vec![0.; worlds.len()];
        for tick in 1..=WARMUP + TICKS {
            let start = Instant::now();
            for (elapsed, world) in world_us.iter_mut().zip(&mut worlds) {
                let at = Instant::now();
                world.step_refined_in_field(GRAVITY, &[], 10)?;
                if tick > WARMUP {
                    *elapsed += at.elapsed().as_secs_f64() * 1e6;
                }
            }
            if tick > WARMUP {
                ticks.push(start.elapsed().as_secs_f64() * 1e6);
                let states: Vec<_> = worlds.iter().map(Island::snapshot).collect();
                witness.check(tick, states.iter())?;
            }
        }
        world_mean_us = Some(world_us.into_iter().map(|us| us / TICKS as f64).collect());
    } else {
        let seed = groups.into_iter().flatten().collect();
        let mut pool = WorkerPool::spawn(
            args.worker.as_deref().ok_or("worker path missing")?,
            0x77656967,
            vec![seed, vec![], vec![], vec![]],
            SimulationStep::Ms10,
            ground,
            1024,
        )?;
        for tick in 1..=WARMUP + TICKS {
            let start = Instant::now();
            pool.advance_islands_in_field(GRAVITY, &[])?;
            if tick > WARMUP {
                ticks.push(start.elapsed().as_secs_f64() * 1e6);
                let workers = &pool.frame().workers;
                if !workers[0].components.iter().map(|c| &c.ids).eq(&ids)
                    || workers[1..].iter().any(|w| !w.components.is_empty())
                    || pool.last_island_refined_bodies() != Some(&witness.ids)
                {
                    return Err(
                        format!("unexpected ownership or refined IDs at tick {tick}").into(),
                    );
                }
                witness.check(tick, workers.iter().map(|w| &w.snapshot))?;
            }
        }
    }
    let elapsed_ms = ticks.iter().sum::<f64>() / 1000.;
    ticks.sort_by(f64::total_cmp);
    let timing = Timing {
        layout: args.layout,
        worlds: ids.len(),
        bodies: witness.ids.len(),
        ticks: TICKS,
        body_substeps: witness.ids.len() * 10 * TICKS,
        positive_pair_receipts: witness.pairs.len() * TICKS,
        positive_floor_receipts: witness.floors.len() * TICKS,
        elapsed_ms,
        tick_p50_us: ticks[TICKS / 2 - 1],
        tick_p95_us: ticks[TICKS * 95 / 100 - 1],
        physics_sha256: format!("{:x}", witness.digest.finalize()),
        world_mean_us,
    };
    println!("{}", serde_json::to_string(&timing)?);
    Ok(())
}
