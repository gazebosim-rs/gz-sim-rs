// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure settled and changing loads on independent native worlds across fixed owners.
#[path = "support/changing_worlds.rs"]
mod scene;

use clap::Parser;
use gz_physics_distributed::{
    physics::{Body, GroundPlane, Island, SimulationStep, Snapshot},
    pool::WorkerPool,
};
use scene::{END, GRAVITY, Layout, SETTLED};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[arg(long, value_enum)]
    layout: Layout,
    /// Advance eight independent native Bevy worlds in serial.
    #[arg(long, conflicts_with = "worker")]
    local: bool,
    /// Place both pads on one owner (default) or assign each physical row to its own worker.
    #[arg(long, value_enum, conflicts_with = "local")]
    placement: Option<scene::Placement>,
    /// Cap granted solver helpers across all owners; zero leaves one lane per active owner.
    #[arg(long, conflicts_with = "local")]
    helper_limit: Option<usize>,
    #[arg(long, required_unless_present = "local")]
    worker: Option<PathBuf>,
    /// Optional JSONL output containing one timing and load sample per measured tick.
    #[arg(long)]
    rows: Option<PathBuf>,
}

#[derive(Serialize)]
struct Row {
    tick: u64,
    elapsed_us: f64,
    pad_pairs: [usize; 2],
    pad_receipts: [usize; 2],
    pad_supports: [f64; 2],
    #[serde(skip_serializing_if = "Option::is_none")]
    helper_mask: Option<u8>,
}

struct Trial {
    pad_ids: [u64; 2],
    ids: Vec<u64>,
    digest: Sha256,
    pair_receipts: usize,
    floor_receipts: usize,
    rows: Vec<Row>,
}
impl Trial {
    fn new(groups: &[Vec<Body>]) -> Self {
        let pads: Vec<_> = groups
            .iter()
            .filter(|g| g.len() == 25)
            .map(|g| g[0].id)
            .collect();
        Self {
            pad_ids: [pads[0], pads[1]],
            ids: groups.iter().flatten().map(|b| b.id).collect(),
            digest: Sha256::new(),
            pair_receipts: 0,
            floor_receipts: 0,
            rows: Vec::new(),
        }
    }
    fn sample<'a>(
        &mut self,
        tick: u64,
        elapsed_us: f64,
        helper_mask: Option<u8>,
        snapshots: impl Iterator<Item = &'a Snapshot>,
    ) -> Result<()> {
        let mut bodies = Vec::new();
        let mut pairs = Vec::new();
        let mut floors = Vec::new();
        for s in snapshots {
            if s.tick != tick {
                return Err(format!("wrong barrier at {tick}").into());
            }
            bodies.extend(&s.bodies);
            pairs.extend(&s.impulses);
            floors.extend(&s.supports);
        }
        bodies.sort_by_key(|b| b.id);
        pairs.sort_by_key(|p| (p.a, p.b));
        floors.sort_by_key(|p| p.body);
        if bodies.iter().map(|b| b.id).ne(self.ids.iter().copied()) {
            return Err(format!("wrong bodies at {tick}").into());
        }
        // Include warmup and all serialized floating point state, so a later
        // return to the same resting pose cannot hide an earlier divergence.
        self.digest
            .update(bincode::serialize(&(tick, &bodies, &pairs, &floors))?);
        self.pair_receipts += pairs.len();
        self.floor_receipts += floors.len();
        if tick > SETTLED {
            let pad_pairs = self.pad_ids.map(|id| {
                pairs
                    .iter()
                    .filter(|p| p.a == id && p.normal_impulse > 0.)
                    .count()
            });
            let pad_receipts = self
                .pad_ids
                .map(|id| pairs.iter().filter(|p| p.a == id).count());
            let pad_supports = self.pad_ids.map(|id| {
                floors
                    .iter()
                    .filter(|p| p.body == id)
                    .map(|p| p.normal_impulse)
                    .sum::<f64>()
            });
            if floors.len() != 8
                || pairs.len() != 6 + pad_receipts.iter().sum::<usize>()
                || !pairs
                    .iter()
                    .map(|p| p.normal_impulse)
                    .chain(floors.iter().map(|p| p.normal_impulse))
                    .all(|v| v.is_finite() && v >= 0.)
                || pad_supports.iter().any(|v| *v <= 0.)
            {
                return Err(format!(
                    "unexpected contact receipt at {tick}, pairs={}, floors={}",
                    pairs.len(),
                    floors.len()
                )
                .into());
            }
            self.rows.push(Row {
                tick,
                elapsed_us,
                pad_pairs,
                pad_receipts,
                pad_supports,
                helper_mask,
            });
        }
        Ok(())
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let groups = scene::seeds(args.layout);
    let components: Vec<Vec<_>> = groups
        .iter()
        .map(|g| g.iter().map(|b| b.id).collect())
        .collect();
    let mut trial = Trial::new(&groups);
    let ground = Some(GroundPlane::new(0.)?);
    let placement = args.placement.unwrap_or(scene::Placement::Single);
    let mut ownership = None;
    if args.local {
        let mut solvers = groups
            .iter()
            .map(|g| {
                Island::restore(&Snapshot {
                    step: SimulationStep::Ms10,
                    ground,
                    bodies: g.clone(),
                    ..Default::default()
                })
            })
            .collect::<std::result::Result<Vec<_>, _>>()?;
        for tick in 1..=END {
            let field = scene::wrenches(tick, &groups);
            let at = Instant::now();
            for (world, ids) in solvers.iter_mut().zip(&components) {
                let inputs: Vec<_> = field
                    .iter()
                    .filter(|w| ids.binary_search(&w.id).is_ok())
                    .cloned()
                    .collect();
                world.step_refined_in_field(GRAVITY, &inputs, 10)?;
            }
            let elapsed = at.elapsed().as_secs_f64() * 1e6;
            let snapshots: Vec<_> = solvers.iter().map(Island::snapshot).collect();
            trial.sample(tick, elapsed, None, snapshots.iter())?;
        }
    } else {
        let mut expected = vec![vec![]; 4];
        for (owner, members) in placement.owners(&groups).into_iter().zip(&components) {
            expected[owner].push(members);
        }
        let mut pool = WorkerPool::spawn(
            args.worker.as_deref().ok_or("missing worker")?,
            0x7368696674,
            placement.worker_seeds(&groups),
            SimulationStep::Ms10,
            ground,
            1024,
        )?;
        pool.set_local_helper_limit(args.helper_limit);
        for tick in 1..=END {
            let field = scene::wrenches(tick, &groups);
            let at = Instant::now();
            pool.advance_islands_in_field(GRAVITY, &field)?;
            let elapsed = at.elapsed().as_secs_f64() * 1e6;
            let workers = &pool.frame().workers;
            if workers.len() != expected.len()
                || workers
                    .iter()
                    .zip(&expected)
                    .any(|(w, want)| w.components.iter().map(|c| &c.ids).ne(want.iter().copied()))
                || pool.last_island_refined_bodies() != Some(&trial.ids)
            {
                return Err(format!("unexpected placement or refinement at tick {tick}").into());
            }
            let grant = pool
                .last_local_helper_mask()
                .ok_or("missing committed helper grant")?;
            trial.sample(
                tick,
                elapsed,
                Some(grant),
                workers.iter().map(|w| &w.snapshot),
            )?;
        }
        ownership = Some(
            pool.frame()
                .workers
                .iter()
                .enumerate()
                .map(|(owner, w)| {
                    serde_json::json!({"worker":owner, "bodies":w.snapshot.bodies.len(),
                "world_keys":w.components.iter().map(|c| c.ids[0]).collect::<Vec<_>>()})
                })
                .collect::<Vec<_>>(),
        );
    }
    // Each commanded launch must unload a previously fully loaded slab. Its
    // partner must subsequently carry the heavier load before the next launch.
    for (i, event) in scene::LAUNCHES.iter().enumerate() {
        let pad = i % 2;
        let before = trial.rows.iter().find(|r| r.tick == event - 1).unwrap();
        let mut interval = trial
            .rows
            .iter()
            .filter(|r| r.tick >= *event && r.tick < event + 40);
        if before.pad_pairs[pad] != 24
            || !interval.any(|r| r.pad_pairs[pad] == 0 && r.pad_pairs[1 - pad] == 24)
        {
            return Err(format!("missing physical load handoff following tick {event}").into());
        }
        // Ballistic flight lasts about 0.6 s. Every release, including the
        // final one, must end in renewed positive support on all 24 cargo.
        if !trial
            .rows
            .iter()
            .any(|r| r.tick > event + 45 && r.tick <= event + 70 && r.pad_pairs[pad] == 24)
        {
            return Err(format!("missing cargo landing after tick {event}").into());
        }
    }
    if let Some(path) = args.rows {
        let mut output = BufWriter::new(File::create_new(path)?);
        for row in &trial.rows {
            serde_json::to_writer(&mut output, row)?;
            writeln!(output)?;
        }
        output.flush()?;
    }
    let mut times: Vec<_> = trial.rows.iter().map(|r| r.elapsed_us).collect();
    times.sort_by(f64::total_cmp);
    let percentile = |percent: usize| times[(times.len() * percent).div_ceil(100) - 1];
    let mut grants = std::collections::BTreeMap::<u8, usize>::new();
    for mask in trial.rows.iter().filter_map(|row| row.helper_mask) {
        *grants.entry(mask).or_default() += 1;
    }
    let summary = serde_json::json!({
        "placement": if args.local {None} else {Some(placement)},
        "helper_limit":args.helper_limit,
        "ownership":ownership, "measured_helper_masks":grants,
        "pair_receipts":trial.pair_receipts, "floor_receipts":trial.floor_receipts,
        "layout": args.layout, "ticks":times.len(), "bodies":trial.ids.len(), "worlds":components.len(),
        "physics_sha256": format!("{:x}",trial.digest.finalize()),
        "elapsed_ms":times.iter().sum::<f64>()*1e-3,
        "tick_p50_us":percentile(50), "tick_p95_us":percentile(95), "tick_max_us":percentile(100),
    });
    println!("{}", serde_json::to_string(&summary)?);
    Ok(())
}
