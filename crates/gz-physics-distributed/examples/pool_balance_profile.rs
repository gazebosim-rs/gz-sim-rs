// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the cost and payback of optional measured whole-island placement.
#[path = "support/changing_oracle.rs"]
mod control;
#[path = "support/changing_worlds.rs"]
mod fixture;

use clap::Parser;
use control::Oracle;
use gz_physics_distributed::{
    physics::{GroundPlane, SimulationStep},
    pool::{PoolBalancePolicy, PoolLoadMode, WorkerPool},
};
use serde::Serialize;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    num::NonZeroU16,
    path::PathBuf,
    time::{Duration, Instant},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, value_enum)]
    layout: fixture::Layout,
    #[arg(long, value_enum, default_value = "single")]
    placement: fixture::Placement,
    #[arg(long)]
    helper_limit: Option<usize>,
    #[arg(long, default_value_t = 5)]
    handoff_ms: u64,
    #[arg(long, default_value = "64")]
    horizon: NonZeroU16,
    #[arg(long,default_value_t=16,value_parser=clap::value_parser!(u16).range(1..))]
    rounds: u16,
    #[arg(long)]
    packed: bool,
    /// Compare undisturbed, serial and scheduled measurements without executing a handoff.
    #[arg(long)]
    probe_study: bool,
    out: PathBuf,
}
#[derive(Clone, Copy, Serialize)]
#[serde(rename_all = "snake_case")]
enum Mode {
    Fixed,
    Sample,
    Scheduled,
    Rebalance,
}
#[derive(Serialize)]
struct Decision {
    tick: u64,
    members: Vec<u64>,
    source: u8,
    destination: u8,
    estimated_payback_ticks: u64,
    before_lower_ns: u64,
    after_serial_peak_ns: u64,
    handoff_us: Option<f64>,
}
#[derive(Serialize)]
struct Trial {
    mode: Mode,
    elapsed_us: f64,
    public_p95_us: f64,
    landing_us: Vec<f64>,
    decisions: Vec<Decision>,
    owners: Vec<u8>,
    measurement_us: f64,
    sampled_total_us: f64,
    sampled_p95_us: f64,
    scheduled_worker_masks: Vec<u8>,
    trajectory_sha256: String,
}
fn quantile(values: &mut [f64], pct: usize) -> f64 {
    values.sort_by(f64::total_cmp);
    values[(values.len() * pct).div_ceil(100) - 1]
}
fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    (v[v.len() / 2] + v[(v.len() - 1) / 2]) / 2.
}
fn trial(args: &Args, oracle: &Oracle, mode: Mode) -> Result<Trial> {
    let mut pool = WorkerPool::spawn(
        &args.worker,
        0x6c6f6164,
        args.placement.worker_seeds(&oracle.groups),
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        96,
    )?;
    pool.set_local_helper_limit(args.helper_limit);
    pool.set_packed_handoffs(args.packed);
    let mut owners: BTreeMap<_, _> = oracle
        .groups
        .iter()
        .zip(args.placement.owners(&oracle.groups))
        .flat_map(|(g, owner)| g.iter().map(move |b| (b.id, (owner as u8, 0u64))))
        .collect();
    let pads: Vec<_> = oracle
        .groups
        .iter()
        .filter(|g| g.len() == 25)
        .map(|g| g[0].id)
        .collect();
    let mut loaded = [0usize; 2];
    let span = NonZeroU16::new(8).unwrap();
    let mut window = None;
    let mut decisions = vec![];
    let mut times = vec![];
    let mut landing_us = vec![];
    let mut measurement_us = 0.;
    let mut sampled = vec![];
    let mut masks = vec![];
    for tick in 1..=fixture::END {
        let inputs = fixture::wrenches(tick, &oracle.groups);
        let start = Instant::now();
        if !matches!(mode, Mode::Fixed) && [71, 146, 246].contains(&tick) {
            let at = Instant::now();
            let probe_mode = if matches!(mode, Mode::Scheduled) {
                PoolLoadMode::Scheduled
            } else {
                PoolLoadMode::Serial
            };
            window = Some(pool.start_load_sample_with_mode(span, probe_mode)?);
            measurement_us += at.elapsed().as_secs_f64() * 1e6;
        }
        pool.advance_islands_in_field(fixture::GRAVITY, &inputs)?;
        if !matches!(mode, Mode::Fixed) && [78, 153, 253].contains(&tick) {
            let at = Instant::now();
            let sample = pool.finish_load_sample(window.take().ok_or("missing load window")?)?;
            masks.push(sample.scheduled_worker_mask());
            let decision = pool.plan_rebalance(
                &sample,
                PoolBalancePolicy {
                    horizon: args.horizon,
                    handoff_cost: Duration::from_millis(args.handoff_ms),
                },
                fixture::GRAVITY,
                &fixture::wrenches(tick + 1, &oracle.groups),
            )?;
            measurement_us += at.elapsed().as_secs_f64() * 1e6;
            if let Some(d) = decision {
                let moved = if matches!(mode, Mode::Rebalance) {
                    let at = Instant::now();
                    pool.transfer_component(d.members[0], d.destination)?;
                    let cost = at.elapsed().as_secs_f64() * 1e6;
                    for id in &d.members {
                        let o = owners.get_mut(id).ok_or("unknown move member")?;
                        o.0 = d.destination;
                        o.1 += 1;
                    }
                    Some(cost)
                } else {
                    None
                };
                decisions.push(Decision {
                    tick,
                    members: d.members,
                    source: d.source,
                    destination: d.destination,
                    before_lower_ns: d.before_lower_ns,
                    after_serial_peak_ns: d.after_serial_peak_ns,
                    estimated_payback_ticks: d.estimated_payback_ticks,
                    handoff_us: moved,
                });
            }
        }
        let us = start.elapsed().as_secs_f64() * 1e6;
        if (71..=78).contains(&tick) || (146..=153).contains(&tick) || (246..=253).contains(&tick) {
            sampled.push(us);
        }
        let frame = pool.frame();
        oracle.verify(frame, &owners)?;
        let contacts = std::array::from_fn(|i| {
            frame
                .workers
                .iter()
                .flat_map(|w| &w.snapshot.impulses)
                .filter(|p| p.a == pads[i] && p.normal_impulse > 0.)
                .count()
        });
        if tick > fixture::SETTLED {
            times.push(us);
            if contacts
                .iter()
                .zip(loaded)
                .any(|(&n, old)| old == 0 && n == 24)
            {
                landing_us.push(us);
            }
        }
        loaded = contacts;
    }
    if landing_us.len() != 6 {
        return Err("missing measured touchdown".into());
    }
    Ok(Trial {
        mode,
        elapsed_us: times.iter().sum(),
        public_p95_us: quantile(&mut times, 95),
        landing_us,
        decisions,
        owners: owners.values().map(|o| o.0).collect(),
        measurement_us,
        sampled_total_us: sampled.iter().sum(),
        sampled_p95_us: quantile(&mut sampled, 95),
        scheduled_worker_masks: masks,
        trajectory_sha256: oracle.digest.clone(),
    })
}
fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.out)?;
    let oracle = Oracle::new(args.layout)?;
    let mut log = BufWriter::new(File::create_new(args.out.join("trials.jsonl"))?);
    let order = [
        [0, 1, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
        [0, 2, 1],
        [1, 0, 2],
    ];
    let mut totals: [Vec<f64>; 3] = Default::default();
    let mut p95: [Vec<f64>; 3] = Default::default();
    let mut landings: [Vec<f64>; 3] = Default::default();
    let modes = if args.probe_study {
        [Mode::Fixed, Mode::Sample, Mode::Scheduled]
    } else {
        [Mode::Fixed, Mode::Sample, Mode::Rebalance]
    };
    let mut sampled_totals: [Vec<f64>; 3] = Default::default();
    let mut sampled_tails: [Vec<f64>; 3] = Default::default();
    let mut moves = 0;
    for round in 0..args.rounds {
        for i in order[usize::from(round) % order.len()] {
            let t = trial(&args, &oracle, modes[i])?;
            sampled_totals[i].push(t.sampled_total_us);
            sampled_tails[i].push(t.sampled_p95_us);
            totals[i].push(t.elapsed_us);
            p95[i].push(t.public_p95_us);
            landings[i].extend_from_slice(&t.landing_us);
            moves += t
                .decisions
                .iter()
                .filter(|d| d.handoff_us.is_some())
                .count();
            writeln!(log, "{}", serde_json::json!({"round":round+1,"trial":t}))?;
            log.flush()?;
        }
    }
    let change = |values: &[Vec<f64>; 3]| {
        [1, 2].map(|i| {
            median(
                &mut values[0]
                    .iter()
                    .zip(&values[i])
                    .map(|(a, b)| 100. * (b / a - 1.))
                    .collect::<Vec<_>>(),
            )
        })
    };
    let report = serde_json::json!({"rounds":args.rounds,"layout":args.layout,"placement":args.placement,"helper_limit":args.helper_limit,"packed":args.packed,
      "horizon":args.horizon,"handoff_budget_ms":args.handoff_ms,"actual_moves":moves,"per_arm_order":modes,
      "paired_change_order":[modes[1],modes[2]],"sampled_window_total_us":sampled_totals.each_mut().map(|v|median(v)),"sampled_window_p95_us":sampled_tails.each_mut().map(|v|median(v)),"paired_total_change_pct":change(&totals),"paired_public_p95_change_pct":change(&p95),
      "pooled_touchdown_p95_us":landings.each_mut().map(|v|quantile(v,95)),
      "native_digest":oracle.digest,"native_ticks_per_trial":oracle.ticks.len(),"pair_receipts_per_trial":oracle.pairs,"ground_receipts_per_trial":oracle.floors});
    fs::write(
        args.out.join("result.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    Ok(())
}
