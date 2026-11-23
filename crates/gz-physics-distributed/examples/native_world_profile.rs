// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Separate per-world stepping cost from the effect of distributing fixed worlds over four peers.
#[path = "support/world_profile_scene.rs"]
mod world_profile_scene;
use clap::{Parser, ValueEnum};
use gz_physics_distributed::{
    physics::{Body, GroundPlane, Island, SimulationStep, Snapshot},
    pool::WorkerPool,
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{error::Error, path::PathBuf, time::Instant};
use world_profile_scene::sphere;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WARMUP: usize = 110;
const TICKS: usize = 200;

#[derive(Clone, Copy, Serialize, ValueEnum)]
#[serde(rename_all = "snake_case")]
enum Scene {
    Fleet,
    Stacks,
}
impl Scene {
    fn seeds(self, stack_tiles: u8) -> Vec<Vec<Body>> {
        match self {
            Self::Fleet => (0..24)
                .map(|i| vec![sphere(i + 1, [12. * i as f64, 0., 3.], 0.2, [0.25, 0., 0.])])
                .collect(),
            Self::Stacks => (0..stack_tiles)
                .flat_map(|tile| {
                    let mut groups = world_profile_scene::stacks();
                    for body in groups.iter_mut().flatten() {
                        body.id += 8 * u64::from(tile);
                        body.position[0] += 32. * f64::from(tile);
                    }
                    groups
                })
                .collect(),
        }
    }
    fn field(self) -> [f64; 3] {
        match self {
            Self::Fleet => [0.; 3],
            Self::Stacks => [0., 0., -9.81],
        }
    }
    fn ground(self) -> Result<Option<GroundPlane>> {
        match self {
            Self::Fleet => Ok(None),
            Self::Stacks => Ok(Some(GroundPlane::new(0.)?)),
        }
    }
    fn substeps(self) -> u8 {
        match self {
            Self::Fleet => 1,
            Self::Stacks => 10,
        }
    }
}
#[derive(Clone, Copy, Serialize, ValueEnum)]
#[serde(rename_all = "snake_case")]
enum Mode {
    Local,
    Pool,
}
#[derive(Parser)]
struct Args {
    #[arg(long)]
    scene: Scene,
    #[arg(long)]
    mode: Mode,
    /// Number of copies of the four-stack fixture, spaced 32 metres apart.
    #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u8).range(1..=8))]
    stack_tiles: u8,
    /// Local native world count, or populated owners within the fixed four-worker pool.
    #[arg(
        long,
        required_unless_present = "owner_counts",
        conflicts_with = "owner_counts"
    )]
    count: Option<usize>,
    /// Assign consecutive worlds to the named number of slots on each populated owner.
    #[arg(long, value_delimiter = ',')]
    owner_counts: Vec<u8>,
    #[arg(long)]
    worker: Option<PathBuf>,
    #[arg(long, default_value_t = 1)]
    round: usize,
}
#[derive(Serialize)]
struct Timing {
    round: usize,
    scene: Scene,
    mode: Mode,
    bodies: usize,
    worlds: usize,
    populated_workers: usize,
    pool_width: usize,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    owner_counts: Vec<u8>,
    body_substeps_per_tick: usize,
    ticks: usize,
    elapsed_ms: f64,
    tick_p50_us: f64,
    tick_p95_us: f64,
    physics_sha256: String,
}

fn hash_frame<'a>(
    states: impl Iterator<Item = &'a Snapshot>,
    tick: u64,
    expected_bodies: usize,
    loads: usize,
    hash: &mut Sha256,
) -> Result<()> {
    let mut bodies = Vec::new();
    let mut pairs = Vec::new();
    let mut supports = Vec::new();
    for s in states {
        if s.tick != tick {
            return Err("world clock mismatch".into());
        }
        bodies.extend(&s.bodies);
        pairs.extend(&s.impulses);
        supports.extend(&s.supports);
    }
    bodies.sort_by_key(|b| b.id);
    pairs.sort_by_key(|p| (p.a, p.b));
    supports.sort_by_key(|p| p.body);
    if bodies.len() != expected_bodies
        || pairs.len() != loads
        || supports.len() != loads
        || pairs
            .iter()
            .any(|p| !p.normal_impulse.is_finite() || p.normal_impulse <= 0.)
        || supports
            .iter()
            .any(|p| !p.normal_impulse.is_finite() || p.normal_impulse <= 0.)
    {
        return Err("expected contact load is missing".into());
    }
    hash.update(bincode::serialize(&(tick, bodies, pairs, supports))?);
    Ok(())
}
fn main() -> Result<()> {
    let a = Args::parse();
    if matches!(a.scene, Scene::Fleet) && a.stack_tiles != 1 {
        return Err("multiple tiles are only available for the stacks fixture".into());
    }
    let groups = a.scene.seeds(a.stack_tiles);
    let expected_loads = if matches!(a.scene, Scene::Stacks) {
        groups.len()
    } else {
        0
    };
    let max_count = groups.len();
    let count = a.count.unwrap_or(a.owner_counts.len());
    if count == 0 || count > max_count || (matches!(a.mode, Mode::Pool) && count > 4) {
        return Err("count outside the permitted range".into());
    }
    let mut placements = vec![vec![]; count];
    if a.owner_counts.is_empty() {
        for (i, group) in groups.into_iter().enumerate() {
            placements[i % count].extend(group);
        }
    } else {
        if !matches!(a.mode, Mode::Pool)
            || a.owner_counts.contains(&0)
            || a.owner_counts
                .iter()
                .map(|&n| usize::from(n))
                .sum::<usize>()
                != max_count
        {
            return Err("owner counts must cover all worlds in a pool".into());
        }
        let mut groups = groups.into_iter();
        for (shard, &n) in placements.iter_mut().zip(&a.owner_counts) {
            shard.extend(groups.by_ref().take(usize::from(n)).flatten());
        }
    }
    let num_bodies = placements.iter().map(Vec::len).sum::<usize>();
    let gravity = a.scene.field();
    let ground = a.scene.ground()?;
    let substeps = a.scene.substeps();
    let mut hash = Sha256::new();
    let mut times = Vec::with_capacity(TICKS);
    let (worlds, workers, width) = match a.mode {
        Mode::Local => {
            let mut worlds: Vec<_> = placements
                .into_iter()
                .map(|bodies| {
                    Island::restore(&Snapshot {
                        bodies,
                        step: SimulationStep::Ms10,
                        ground,
                        ..Snapshot::default()
                    })
                })
                .collect::<std::result::Result<_, _>>()?;
            for tick in 1..=WARMUP + TICKS {
                let start = Instant::now();
                for w in &mut worlds {
                    w.step_refined_in_field(gravity, &[], substeps)?;
                }
                if tick > WARMUP {
                    times.push(start.elapsed().as_secs_f64() * 1e6);
                    let snapshots: Vec<_> = worlds.iter().map(Island::snapshot).collect();
                    hash_frame(
                        snapshots.iter(),
                        tick as u64,
                        num_bodies,
                        expected_loads,
                        &mut hash,
                    )?;
                }
            }
            (worlds.len(), 0, 0)
        }
        Mode::Pool => {
            placements.resize(4, vec![]);
            let mut pool = WorkerPool::spawn(
                a.worker
                    .as_deref()
                    .ok_or("--worker required for the pool")?,
                0x776f726c64,
                placements,
                SimulationStep::Ms10,
                ground,
                1024,
            )?;
            for tick in 1..=WARMUP + TICKS {
                let start = Instant::now();
                pool.advance_islands_in_field(gravity, &[])?;
                if tick > WARMUP {
                    times.push(start.elapsed().as_secs_f64() * 1e6);
                    let workers = &pool.frame().workers;
                    if workers
                        .iter()
                        .filter(|w| !w.snapshot.bodies.is_empty())
                        .count()
                        != count
                        || workers.iter().map(|w| w.components.len()).sum::<usize>() != max_count
                        || pool.last_island_refined_bodies().unwrap().len()
                            != if substeps == 10 { num_bodies } else { 0 }
                    {
                        return Err("solver layout or schedule changed during sampling".into());
                    }
                    if !a.owner_counts.is_empty()
                        && workers
                            .iter()
                            .take(count)
                            .zip(&a.owner_counts)
                            .any(|(w, &n)| w.components.len() != usize::from(n))
                    {
                        return Err("native worlds changed owners during sampling".into());
                    }
                    hash_frame(
                        workers.iter().map(|w| &w.snapshot),
                        tick as u64,
                        num_bodies,
                        expected_loads,
                        &mut hash,
                    )?;
                }
            }
            (max_count, count, 4)
        }
    };
    let elapsed_ms = times.iter().sum::<f64>() * 0.001;
    times.sort_by(f64::total_cmp);
    let timing = Timing {
        round: a.round,
        scene: a.scene,
        mode: a.mode,
        bodies: num_bodies,
        worlds,
        populated_workers: workers,
        pool_width: width,
        owner_counts: a.owner_counts,
        body_substeps_per_tick: num_bodies * usize::from(substeps),
        ticks: TICKS,
        elapsed_ms,
        tick_p50_us: times[(TICKS / 2) - 1],
        tick_p95_us: times[(TICKS * 95 / 100) - 1],
        physics_sha256: format!("{:x}", hash.finalize()),
    };
    println!("{}", serde_json::to_string(&timing)?);
    Ok(())
}
