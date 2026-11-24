// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Audit actual solver placement against committed receipts and a retrospective timing oracle.
use clap::Parser;
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::PathBuf,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    /// One run directory with load.jsonl and one numeric JSONL file per worker.
    directories: Vec<PathBuf>,
    /// Write a JSON summary and the per-transition observations.
    #[arg(long)]
    out: PathBuf,
    /// Verify the experimental worker that weights only positive normal impulses.
    #[arg(long)]
    positive_only: bool,
}
#[derive(Deserialize)]
struct World {
    ids: Vec<u64>,
    fine: bool,
    helper: bool,
    prior_pairs: usize,
    prior_supports: usize,
    pairs: usize,
    supports: usize,
    solve_ns: u64,
}
#[derive(Deserialize)]
struct Barrier {
    tick: u64,
    worlds: Vec<World>,
}
#[derive(Deserialize)]
struct Load {
    tick: u64,
    pad_pairs: [usize; 2],
    pad_receipts: [usize; 2],
}

fn json_lines<T: DeserializeOwned>(file: PathBuf) -> Result<Vec<T>> {
    BufReader::new(File::open(file)?)
        .lines()
        .map(|line| Ok(serde_json::from_str(&line?)?))
        .collect()
}

// Enumerate all proper nonempty intervals. The fixture verifies eight fine worlds on the worker.
fn best_peak(weights: &[u64]) -> u64 {
    let total: u64 = weights.iter().sum();
    let mut best = total;
    for (start, _) in weights.iter().enumerate() {
        let mut block = 0;
        for &w in &weights[start..] {
            block += w;
            if block < total {
                best = best.min(block.max(total - block));
            }
        }
    }
    best
}

fn observed_peak(weights: &[u64], worlds: &[World]) -> u64 {
    let mut lanes = [0; 2];
    for (weight, world) in weights.iter().zip(worlds) {
        lanes[usize::from(world.helper)] += weight;
    }
    lanes[0].max(lanes[1])
}

#[derive(Serialize, PartialEq)]
struct Transition {
    tick: u64,
    pad: usize,
    positive_from: usize,
    positive_to: usize,
    receipt_lag_ticks: Option<u64>,
    planner_input_lag_ticks: Option<u64>,
    optimal_split_lag_ticks: Option<u64>,
}
#[derive(Default)]
struct Samples {
    current: Vec<f64>,
    count: Vec<f64>,
}

impl Samples {
    fn record(&mut self, cur: f64, count: f64) {
        self.current.push(cur);
        self.count.push(count);
    }
    fn summary(&self) -> Value {
        let summarize = |v: &[f64]| {
            let mut values = v.to_owned();
            values.sort_by(f64::total_cmp);
            let p = |k: usize| values[(k * values.len()).div_ceil(100) - 1];
            json!({
                "samples": values.len(),
                "exact": values.iter().filter(|x| **x == 0.).count(),
                "within_5pct": values.iter().filter(|x| **x <= 5.).count(),
                "median_excess_pct": p(50),
                "p95_excess_pct": p(95),
                "max_excess_pct": p(100),
            })
        };
        json!({"observed":summarize(&self.current),"count_projection":summarize(&self.count)})
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.directories.is_empty() {
        return Err("supply at least one worker trace directory".into());
    }
    let mut all = Samples::default();
    let mut contacts = Samples::default();
    let mut transitions: Option<Vec<Transition>> = None;
    let mut changed_receipts = 0;
    let mut actual_replans = 0;
    let mut matched_new_receipts = 0;
    let mut source_digest = None;
    let mut loaded_times = [Vec::new(), Vec::new()];
    let mut airborne_times = [Vec::new(), Vec::new()];
    let mut loaded_slowest = [0, 0];
    let mut states = Vec::new();
    for dir in &args.directories {
        let physics: Value = serde_json::from_reader(File::open(dir.join("physics.json"))?)?;
        let sha = physics["physics_sha256"]
            .as_str()
            .ok_or("missing physics digest")?;
        if source_digest.get_or_insert(sha.to_owned()) != sha {
            return Err("physical digests differ".into());
        }
        let public: Vec<Load> = json_lines(dir.join("load.jsonl"))?;
        let mut owners = Vec::new();
        for entry in fs::read_dir(dir)? {
            let path = entry?.path();
            if path.extension().is_some_and(|ext| ext == "jsonl")
                && path
                    .file_stem()
                    .and_then(|stem| stem.to_str())
                    .and_then(|name| name.parse::<u32>().ok())
                    .is_some()
            {
                owners.push(json_lines::<Barrier>(path)?);
            }
        }
        let end = public.last().ok_or("empty timing samples")?.tick as usize;
        if owners.len() != 4
            || owners.iter().any(|frames| {
                frames.len() != end
                    || frames
                        .iter()
                        .enumerate()
                        .any(|(i, f)| f.tick as usize != i + 1)
            })
        {
            return Err("incomplete worker trace".into());
        }
        let mut busy = owners
            .iter()
            .filter(|rows| rows.iter().any(|f| !f.worlds.is_empty()));
        let frames = busy.next().ok_or("missing populated owner")?;
        if busy.next().is_some() {
            return Err("timing fixture requires one populated owner".into());
        }
        let pads: Vec<_> = frames[0]
            .worlds
            .iter()
            .enumerate()
            .filter(|(_, w)| w.ids.len() == 25)
            .map(|(i, _)| i)
            .collect();
        if pads.len() != 2 {
            return Err("expected two cargo worlds".into());
        }
        let mut events = Vec::new();
        let mut timeline = Vec::new();
        let mut helpers = Vec::new();
        for (i, sample) in public.iter().enumerate() {
            let tick = sample.tick;
            if tick < 2 || tick as usize > frames.len() {
                return Err("timing sample outside the trace".into());
            }
            let w = &frames[tick as usize - 1].worlds;
            let previous = &frames[tick as usize - 2].worlds;
            if w.len() != 8
                || previous.len() != 8
                || public[0].tick + i as u64 != tick
                || w.iter().zip(previous).any(|(w, p)| {
                    w.ids != p.ids
                        || !w.fine
                        || w.solve_ns == 0
                        || w.prior_pairs != p.pairs
                        || w.prior_supports != p.supports
                })
                || pads
                    .iter()
                    .zip(sample.pad_receipts)
                    .any(|(&j, p)| w[j].pairs != p)
            {
                return Err(format!("invalid solve/receipt boundary at {tick}").into());
            }
            let input: Vec<_> = w
                .iter()
                .map(|v| (1 + v.ids.len() + v.prior_pairs + v.prior_supports) as u64)
                .collect();
            let output: Vec<_> = w
                .iter()
                .map(|v| (1 + v.ids.len() + v.pairs + v.supports) as u64)
                .collect();
            let actual: Vec<_> = w.iter().map(|v| v.solve_ns).collect();
            for p in 0..2 {
                if sample.pad_pairs[p] == 24 && sample.pad_pairs[1 - p] == 0 {
                    let ns = actual[pads[p]];
                    loaded_slowest[p] += usize::from(actual.iter().all(|other| *other <= ns));
                    loaded_times[p].push(ns);
                    airborne_times[p].push(actual[pads[1 - p]]);
                }
            }
            let mut forecast = input.clone();
            let mut forecast_pad_pairs = [w[pads[0]].prior_pairs, w[pads[1]].prior_pairs];
            if args.positive_only && i > 0 {
                // Only cargo contacts change here. Every small pair and floor
                // support stays positively loaded throughout the measurement.
                forecast_pad_pairs = public[i - 1].pad_pairs;
                for (&j, pairs) in pads.iter().zip(forecast_pad_pairs) {
                    forecast[j] = (1 + w[j].ids.len() + pairs + w[j].prior_supports) as u64;
                }
            }
            let indices: Vec<_> = w
                .iter()
                .enumerate()
                .filter(|(_, v)| v.helper)
                .map(|(j, _)| j)
                .collect();
            if indices.is_empty()
                || indices.len() == w.len()
                || indices.windows(2).any(|p| p[1] != p[0] + 1)
                || observed_peak(&forecast, w) != best_peak(&forecast)
            {
                return Err(format!("unexpected lane assignment at tick {tick}").into());
            }
            if i > 0 && helpers != indices {
                actual_replans += 1
            }
            helpers = indices;
            if input != output {
                changed_receipts += 1;
            }
            if observed_peak(&output, w) == best_peak(&output) {
                matched_new_receipts += 1;
            }
            let optimal = best_peak(&actual) as f64;
            let excess = |n: u64| (n as f64 / optimal - 1.) * 100.;
            let (left, right) = actual.split_at(4);
            let count = excess(left.iter().sum::<u64>().max(right.iter().sum()));
            let cur = excess(observed_peak(&actual, w));
            all.record(cur, count);
            let changed = i > 0 && public[i - 1].pad_pairs != sample.pad_pairs;
            if changed || input != output {
                contacts.record(cur, count);
            }
            if i == 0
                || changed
                || input != output
                || helpers.iter().copied().ne(previous
                    .iter()
                    .enumerate()
                    .filter(|(_, v)| v.helper)
                    .map(|(j, _)| j))
            {
                timeline.push(json!({
                    "tick": tick,
                    "positive_pairs": sample.pad_pairs,
                    "input_pair_receipts": pads.iter().map(|&p| w[p].prior_pairs).collect::<Vec<_>>(),
                    "forecast_pair_counts": forecast_pad_pairs,
                    "helper_base_ids": helpers.iter().map(|&j| w[j].ids[0]).collect::<Vec<_>>(),
                }));
            }
            if i == 0 {
                continue;
            }
            for (p, &j) in pads.iter().enumerate() {
                let from = public[i - 1].pad_pairs[p];
                let to = sample.pad_pairs[p];
                if from == to {
                    continue;
                }
                let plateau: Vec<_> = public[i..]
                    .iter()
                    .take_while(|s| s.pad_pairs[p] == to)
                    .collect();
                let first = |pred: &dyn Fn(&Load, &[World]) -> bool| {
                    plateau
                        .iter()
                        .find(|s| pred(s, &frames[s.tick as usize - 1].worlds))
                        .map(|s| s.tick - tick)
                };
                let receipt = first(&|_, w| w[j].pairs == to);
                let prior = first(&|s, w| {
                    if args.positive_only {
                        public[(s.tick - public[0].tick - 1) as usize].pad_pairs[p] == to
                    } else {
                        w[j].prior_pairs == to
                    }
                });
                // Retrospective contact proxy substitutes each slab's actually
                // loaded pairs for its retained pair receipts on this tick.
                let optimal_split = first(&|s, w| {
                    let mut costs: Vec<_> = w
                        .iter()
                        .map(|v| (1 + v.ids.len() + v.pairs + v.supports) as u64)
                        .collect();
                    for (&k, pairs) in pads.iter().zip(s.pad_pairs) {
                        costs[k] = (1 + w[k].ids.len() + pairs + w[k].supports) as u64;
                    }
                    observed_peak(&costs, w) == best_peak(&costs)
                });
                events.push(Transition {
                    tick,
                    pad: p,
                    positive_from: from,
                    positive_to: to,
                    receipt_lag_ticks: receipt,
                    planner_input_lag_ticks: prior,
                    optimal_split_lag_ticks: optimal_split,
                });
            }
        }
        if let Some(existing) = &transitions {
            if existing != &events || timeline != states {
                return Err("repeated run changed physical transitions or assignments".into());
            }
        } else {
            transitions = Some(events);
            states = timeline;
        }
    }
    if contacts.current.is_empty() {
        return Err("no changing contact samples".into());
    }
    let mut heavy = Vec::new();
    for i in 0..2 {
        loaded_times[i].sort_unstable();
        airborne_times[i].sort_unstable();
        let n = loaded_times[i].len();
        if n == 0 {
            return Err("expected both cargo pads to take turns carrying load".into());
        }
        heavy.push(json!({
            "loaded_pad": i,
            "samples": n,
            "loaded_pad_slowest": loaded_slowest[i],
            "loaded_pad_median_us": loaded_times[i][(n-1)/2] as f64 / 1000.,
            "airborne_pad_median_us": airborne_times[i][(n-1)/2] as f64 / 1000.,
        }));
    }
    let report = json!({
        "runs": args.directories.len(),
        "positive_only": args.positive_only,
        "physics_sha256": source_digest,
        "barriers": all.current.len(),
        "heaviest_island": heavy,
        "changed_receipt_barriers": changed_receipts,
        "helper_changes": actual_replans,
        "post_receipt_optimal_barriers": matched_new_receipts,
        "time_projection": all.summary(),
        "change_tick_time_projection": contacts.summary(),
        "positive_pair_transitions": transitions,
        "lane_states": states,
    });
    fs::write(args.out, serde_json::to_string_pretty(&report)?)?;
    Ok(())
}
