// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Audit a completed or running seeded fault campaign and optionally compare matched schedules.
use clap::{Parser, ValueEnum};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::PathBuf,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    campaign: PathBuf,
    /// Continue reading flushed episodes from this separate directory (in seed order).
    #[arg(long)]
    continuation: Vec<PathBuf>,
    /// Require every episode to close the measured per-owner and total impulse budgets.
    #[arg(long)]
    momentum: bool,
    /// Require PID evidence for the confirm-policy fault at each loaded handoff.
    #[arg(long)]
    confirm_handoffs: bool,
    /// A completed campaign from the other coordinator build using the same seeds.
    #[arg(long)]
    reference: Option<PathBuf>,
    /// Require the final completion marker and check its counts against the event stream.
    #[arg(long)]
    complete: bool,
    /// Expected restart scope when a worker remained stopped through two real deadlines.
    #[arg(long, value_enum)]
    fencing: Option<Fence>,
}
#[derive(Clone, Copy, ValueEnum)]
enum Fence {
    FailedWorker,
    WholePool,
}
#[derive(Deserialize)]
struct Move {
    tick: u64,
    source: u8,
    destination: u8,
    members: usize,
    #[serde(default)]
    member_ids: Vec<u64>,
    withheld: String,
    recovery: Option<HandoffRecovery>,
}
impl Move {
    fn same_route(&self, other: &Self) -> bool {
        self.tick == other.tick
            && self.source == other.source
            && self.destination == other.destination
            && self.members == other.members
            && self.member_ids == other.member_ids
            && self.withheld == other.withheld
    }
}

#[derive(Deserialize)]
struct HandoffRecovery {
    policy: String,
    fault: String,
    endpoint: u8,
    before_pids: Vec<u32>,
    after_pids: Vec<u32>,
    elapsed_us: f64,
}
impl HandoffRecovery {
    fn confirmed(&self, movement: &Move, width: u8) -> Result<usize> {
        use std::collections::BTreeSet;
        if self.policy != "confirm"
            || ![movement.source, movement.destination].contains(&self.endpoint)
            || !self.elapsed_us.is_finite()
            || self.elapsed_us <= 0.0
        {
            return Err("invalid confirmation record".into());
        }
        for ids in [&self.before_pids, &self.after_pids] {
            if ids.len() != usize::from(width)
                || ids.contains(&0)
                || ids.iter().collect::<BTreeSet<_>>().len() != ids.len()
            {
                return Err("invalid participant PID set".into());
            }
        }
        let endpoint = usize::from(self.endpoint);
        let replaced = self.before_pids[endpoint] != self.after_pids[endpoint];
        let changes = self
            .before_pids
            .iter()
            .zip(&self.after_pids)
            .filter(|(a, b)| a != b)
            .count();
        let valid = match self.fault.as_str() {
            "lost" => !replaced,
            "killed" | "stopped" => replaced,
            "thaw" => true,
            _ => false,
        };
        if !valid || changes != usize::from(replaced) {
            return Err("wrong loaded-handoff recovery scope".into());
        }
        Ok(changes)
    }
}
#[derive(Deserialize)]
struct Deadline {
    phase: String,
    owner: u8,
    tick: u64,
    timeout_us: f64,
    retry_us: f64,
    replacements: usize,
    #[serde(default)]
    before_pids: Vec<u32>,
    #[serde(default)]
    after_pids: Vec<u32>,
}
#[derive(Deserialize)]
struct Episode {
    number: u64,
    width: u8,
    layout: String,
    packed_handoffs: bool,
    packed_observations: bool,
    helper_limit: Option<usize>,
    journal_limit: usize,
    #[serde(default)]
    initial_owners: Vec<u8>,
    moves: Vec<Move>,
    cache_hits: usize,
    replacements: usize,
    restored_at: u64,
    physics_sha256: String,
    deadline: Option<Deadline>,
    momentum: Option<Momentum>,
}
#[derive(Deserialize)]
struct Momentum {
    solves: u64,
    same_tick_checks: u64,
    moved_bodies: usize,
    peak_worker_residual_ns: f64,
    peak_global_residual_ns: f64,
}

#[derive(Deserialize, PartialEq)]
struct Oracle {
    trajectory_sha256: String,
    ticks: u64,
    pairs: u64,
    floors: u64,
}
#[derive(Default)]
struct Sample {
    first: Vec<f64>,
    retry: Vec<f64>,
    changed: u64,
}
#[derive(Default, Serialize)]
struct Counts {
    episodes: u64,
    ticks: u64,
    body_states: u64,
    pair_receipts: u64,
    ground_receipts: u64,
    handoffs: u64,
    cache_confirmations: u64,
    scheduled_worker_replacements: u64,
    full_pool_restores: u64,
    deadline_replacements: u64,
}
fn timing(v: &mut [f64]) -> serde_json::Value {
    v.sort_by(f64::total_cmp);
    serde_json::json!({"median_us":(v[v.len()/2]+v[(v.len()-1)/2])/2.,"p95_us":v[(v.len()*95).div_ceil(100)-1]})
}
fn match_trial(a: &Episode, b: &Episode) -> bool {
    a.number == b.number
        && a.width == b.width
        && a.layout == b.layout
        && a.packed_handoffs == b.packed_handoffs
        && a.packed_observations == b.packed_observations
        && a.helper_limit == b.helper_limit
        && a.journal_limit == b.journal_limit
        && a.initial_owners == b.initial_owners
        && a.moves.len() == b.moves.len()
        && a.moves.iter().zip(&b.moves).all(|(x, y)| x.same_route(y))
        && a.restored_at == b.restored_at
        && a.cache_hits == b.cache_hits
        && a.replacements == b.replacements
        && a.physics_sha256 == b.physics_sha256
        && a.deadline.as_ref().map(|d| (&d.phase, d.owner, d.tick))
            == b.deadline.as_ref().map(|d| (&d.phase, d.owner, d.tick))
}
fn main() -> Result<()> {
    let args = Args::parse();
    let controls: [Oracle; 2] =
        serde_json::from_reader(File::open(args.campaign.join("oracles.json"))?)?;
    let mut reference = args
        .reference
        .as_ref()
        .map(|p| File::open(p.join("episodes.jsonl")).map(|f| BufReader::new(f).lines()))
        .transpose()?;
    let mut counts = Counts::default();
    let mut first = None;
    let mut last = None;
    let mut by_width = BTreeMap::new();
    let mut by_schedule = BTreeMap::new();
    let mut by_phase = BTreeMap::new();
    let mut deadlines = BTreeMap::new();
    let mut handoff_recoveries: BTreeMap<String, (Vec<f64>, usize)> = BTreeMap::new();
    let mut segments = Vec::new();
    let mut momentum_episodes = 0_u64;
    let mut peak_momentum_residual = 0_f64;
    let mut moved_bodies = 0_usize;
    let paths = std::iter::once(&args.campaign).chain(&args.continuation);
    for (index, path) in paths.enumerate() {
        let segment_controls: [Oracle; 2] =
            serde_json::from_reader(File::open(path.join("oracles.json"))?)?;
        if segment_controls != controls {
            return Err("different native oracle in continuation".into());
        }
        let marker_path = path.join("result.json");
        let marker = if marker_path.try_exists()? {
            Some(serde_json::from_slice::<serde_json::Value>(&fs::read(
                marker_path,
            )?)?)
        } else {
            None
        };
        let final_segment = index == args.continuation.len();
        if args.complete && final_segment && marker.is_none() {
            return Err("final segment has no completion marker".into());
        }
        let start_episodes = counts.episodes;
        let start_ticks = counts.ticks;
        let mut segment_first = None;
        let mut discarded_trailing_bytes = 0_usize;
        let mut rows = BufReader::new(File::open(path.join("episodes.jsonl"))?);
        let mut buf = String::new();
        while rows.read_line(&mut buf)? > 0 {
            if !buf.ends_with('\n') {
                if marker.is_some() {
                    return Err("unterminated episode in a sealed segment".into());
                }
                discarded_trailing_bytes = buf.len();
                break;
            }
            let e: Episode = serde_json::from_str(&buf)?;
            buf.clear();
            let n = e.number;
            if last.is_some_and(|v| Some(n) != u64::checked_add(v, 1)) {
                return Err(format!("non-contiguous episode {n}").into());
            }
            segment_first.get_or_insert(n);
            first.get_or_insert(n);
            last = Some(n);
            let index = (n / 6 % 2) as usize;
            let o = &controls[index];
            if e.width != 3 + (n % 6) as u8
                || e.layout != ["adjacent", "separated"][index]
                || e.packed_handoffs != (n / 12 % 2 == 1)
                || e.packed_observations != (n / 24 % 2 == 1)
                || e.physics_sha256 != o.trajectory_sha256
                || e.moves.len() != 2
                || e.replacements != 1
                || e.restored_at > o.ticks
            {
                return Err(format!("invalid episode metadata at {n}").into());
            }
            if let Some(rows) = &mut reference {
                let other: Episode =
                    serde_json::from_str(&rows.next().ok_or("reference ended early")??)?;
                if !match_trial(&e, &other) {
                    return Err(format!("unmatched coordinator schedules at {n}").into());
                }
            }
            if let Some(m) = &e.momentum {
                let residual = m.peak_global_residual_ns.max(m.peak_worker_residual_ns);
                if m.solves != o.ticks
                    || m.same_tick_checks != 4
                    || m.moved_bodies != e.moves.iter().map(|v| v.members).sum::<usize>()
                    || !(0.0..=1e-9).contains(&m.peak_global_residual_ns)
                    || !(0.0..=1e-9).contains(&m.peak_worker_residual_ns)
                {
                    return Err(
                        format!("invalid external/transported impulse budget at {n}").into(),
                    );
                }
                momentum_episodes += 1;
                peak_momentum_residual = peak_momentum_residual.max(residual);
                moved_bodies += m.moved_bodies;
            } else if args.momentum {
                return Err(format!("missing momentum evidence at {n}").into());
            }
            let mut owners = e.initial_owners;
            for m in &e.moves {
                if m.source >= e.width
                    || m.destination >= e.width
                    || m.source == m.destination
                    || m.members == 0
                    || m.tick >= o.ticks
                {
                    return Err("invalid handoff".into());
                }
                if args.confirm_handoffs {
                    let witness = m
                        .recovery
                        .as_ref()
                        .ok_or("missing loaded-handoff PID evidence")?;
                    let replaced = witness.confirmed(m, e.width)?;
                    let (samples, replacements) =
                        handoff_recoveries.entry(witness.fault.clone()).or_default();
                    samples.push(witness.elapsed_us);
                    *replacements += replaced;
                }
                *by_phase.entry(m.withheld.clone()).or_insert(0_u64) += 1;
                if !owners.is_empty() {
                    if owners.len() != 62 || m.member_ids.len() != m.members {
                        return Err("incomplete owner trace".into());
                    }
                    for &id in &m.member_ids {
                        let key = usize::try_from(id.checked_sub(1).ok_or("invalid zero ID")?)?;
                        let owner = owners.get_mut(key).ok_or("invalid member")?;
                        if *owner != m.source {
                            return Err(format!("wrong previous owner at {n}/{}", m.tick).into());
                        }
                        *owner = m.destination;
                    }
                }
            }
            if let Some(d) = &e.deadline {
                let changed: Vec<_> = d
                    .before_pids
                    .iter()
                    .zip(&d.after_pids)
                    .enumerate()
                    .filter_map(|(i, (a, b))| (a != b).then_some(i as u8))
                    .collect();
                let has_pids = !d.before_pids.is_empty();
                if !(d.timeout_us >= 800_000. && d.retry_us > 0.)
                    || (has_pids
                        && (d.before_pids.len() != usize::from(e.width)
                            || d.after_pids.len() != d.before_pids.len()
                            || changed.len() != d.replacements))
                {
                    return Err(format!("invalid deadline witness in {n}").into());
                }
                let valid = match d.phase.as_str() {
                    "step_thaw" => d.replacements == 0,
                    "step_fence" => {
                        (match args.fencing {
                            Some(Fence::FailedWorker) => d.replacements == 1,
                            Some(Fence::WholePool) => d.replacements == e.width as usize,
                            None => d.replacements > 0,
                        }) && (!has_pids || changed.contains(&d.owner))
                    }
                    "prepare_thaw" => e.moves.iter().any(|m| {
                        m.tick == d.tick
                            && m.destination == d.owner
                            && m.withheld == "deadline_prepare"
                            && d.replacements == 2
                            && (!has_pids
                                || changed.contains(&m.source) && changed.contains(&m.destination))
                    }),
                    _ => false,
                };
                if !valid {
                    return Err(format!("unexpected {} recovery at {n}", d.phase).into());
                }
                let sample = deadlines
                    .entry(d.phase.clone())
                    .or_insert_with(Sample::default);
                sample.first.push(d.timeout_us);
                sample.retry.push(d.retry_us);
                sample.changed += d.replacements as u64;
                counts.deadline_replacements += d.replacements as u64;
            }
            *by_width.entry(e.width).or_insert(0_u64) += 1;
            *by_schedule
                .entry(format!(
                    "{}-{}-handoff{}-obs{}",
                    e.width,
                    e.layout,
                    u8::from(e.packed_handoffs),
                    u8::from(e.packed_observations)
                ))
                .or_insert(0_u64) += 1;
            counts.episodes += 1;
            counts.ticks += o.ticks;
            counts.body_states += o.ticks * 62;
            counts.pair_receipts += o.pairs;
            counts.ground_receipts += o.floors;
            counts.handoffs += e.moves.len() as u64;
            counts.cache_confirmations += e.cache_hits as u64;
            counts.scheduled_worker_replacements += e.replacements as u64;
            counts.full_pool_restores += 1;
        }
        let episodes = counts.episodes - start_episodes;
        let ticks = counts.ticks - start_ticks;
        if !final_segment && segment_first.is_none() {
            return Err("empty history segment".into());
        }
        if marker.as_ref().is_some_and(|m| {
            m["episodes"].as_u64() != Some(episodes)
                || m["ticks"].as_u64() != Some(ticks)
                || m["start"].as_u64() != segment_first
                || m["last"].as_u64() != last
        }) {
            return Err("completion marker disagrees with episode stream".into());
        }
        segments.push(serde_json::json!({"path":path,"first":segment_first,"last":segment_first.and(last),
            "episodes":episodes,"ticks":ticks,"sealed":marker.is_some(),"discarded_trailing_bytes":discarded_trailing_bytes}));
    }
    if args.complete && reference.as_mut().is_some_and(|r| r.next().is_some()) {
        return Err("extra reference episodes".into());
    }
    let handoff_times=handoff_recoveries.into_iter().map(|(fault,(mut samples,replacements))| {
        (fault,serde_json::json!({"events":samples.len(),"elapsed":timing(&mut samples),"replaced_workers":replacements}))
    }).collect::<BTreeMap<_,_>>();
    let measurements=deadlines.into_iter().map(|(phase,mut s)|(phase,serde_json::json!({"samples":s.first.len(),"first_deadline":timing(&mut s.first),"retry":timing(&mut s.retry),"replaced_workers":s.changed}))).collect::<BTreeMap<_,_>>();
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"first":first,"last":last,"counts":counts,"segments":segments,"confirmed_handoff_faults":handoff_times,"momentum_episodes":momentum_episodes,"moved_bodies_with_checked_momentum":moved_bodies,"peak_momentum_residual_ns":(momentum_episodes>0).then_some(peak_momentum_residual),"by_width":by_width,"schedule_combinations":by_schedule.len(),"by_schedule":by_schedule,"withheld_handoffs":by_phase,"deadlines":measurements})
        )?
    );
    Ok(())
}
