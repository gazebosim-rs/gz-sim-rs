// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare matched fixed, sampling-only and automatic runs, preserving each run's six landings.
#[path = "support/periodic_compare.rs"]
mod cross;
use clap::Parser;
use serde::Deserialize;
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    /// Treat consecutive directory arguments as matched reference/candidate block pairs.
    #[arg(long)]
    paired: bool,
    /// Independent completed profile output directories.
    #[arg(required = true)]
    sets: Vec<PathBuf>,
}
#[derive(Deserialize)]
struct Row {
    round: usize,
    data: Trial,
}
#[derive(Deserialize)]
struct Trial {
    mode: String,
    full_us: f64,
    steady_p95_us: f64,
    landing_us: [f64; 6],
    moves: Vec<Move>,
    #[serde(default)]
    closing_without_move_us: Vec<f64>,
    #[serde(default)]
    opening_only_us: Vec<f64>,
    windows_started: u64,
    #[serde(default)]
    windows_ineligible: u64,
    #[serde(default)]
    windows_deferred: u64,
    #[serde(default)]
    windows_woken: u64,
    #[serde(default)]
    sample_starts: Option<Vec<u64>>,
    #[serde(default)]
    contact_transitions: Option<Vec<u64>>,
    windows_completed: u64,
    windows_interrupted: u64,
    unavailable_probes: u64,
}
#[derive(Deserialize)]
struct Move {
    tick: u64,
    estimated_payback_ticks: u64,
}
fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    (v[v.len() / 2] + v[(v.len() - 1) / 2]) / 2.
}
fn p95(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[(v.len() * 95).div_ceil(100) - 1]
}
fn compare(pairs: &[(&Trial, &Trial)], metric: fn(&Trial) -> f64) -> Value {
    let values = |candidate| {
        pairs
            .iter()
            .map(|&(a, b)| metric(if candidate { b } else { a }))
            .collect::<Vec<_>>()
    };
    let delta = pairs
        .iter()
        .map(|&(a, b)| 100. * (metric(b) / metric(a) - 1.))
        .collect::<Vec<_>>();
    json!({"median_us":[median(values(false)),median(values(true))],
           "candidate_faster":delta.iter().filter(|&&e|e<0.).count(),"median_paired_change_pct":median(delta)})
}
fn reaction<'a>(trials: impl Iterator<Item = &'a Trial>) -> Result<Value> {
    let mut delays = Vec::new();
    let mut censored = 0;
    for t in trials {
        let (Some(starts), Some(transitions)) = (&t.sample_starts, &t.contact_transitions) else {
            return Ok(Value::Null);
        };
        if starts.len() as u64 != t.windows_started
            || !starts.windows(2).all(|p| p[0] < p[1])
            || !transitions.windows(2).all(|p| p[0] < p[1])
            || transitions.len() != 12
        {
            return Err("inconsistent sampling or contact timeline".into());
        }
        for &tick in transitions {
            if let Some(next) = starts.get(starts.partition_point(|&start| start < tick)) {
                delays.push((next - tick) as f64);
            } else {
                censored += 1;
            }
        }
    }
    Ok(
        json!({"fresh_observations":delays.len(),"right_censored":censored,"median_ticks":(!delays.is_empty()).then(||median(delays.clone())),
        "p95_ticks":(!delays.is_empty()).then(||p95(delays.clone())),"max_ticks":delays.into_iter().max_by(f64::total_cmp)}),
    )
}
type Round = BTreeMap<String, Trial>;
fn read(path: &std::path::Path) -> Result<(Value, Vec<Round>)> {
    let proof: Value = serde_json::from_slice(&fs::read(path.join("proof.json"))?)?;
    let n = proof["rounds"].as_u64().ok_or("missing round count")? as usize;
    if n == 0
        || proof["native_steps_per_trial"] != 350
        || proof["native_sha256"]
            .as_str()
            .is_none_or(|s| s.len() != 64)
    {
        return Err("missing native trajectory".into());
    }
    let mut rounds: Vec<BTreeMap<String, Trial>> = (0..n).map(|_| BTreeMap::new()).collect();
    for line in fs::read_to_string(path.join("trials.jsonl"))?.lines() {
        let Row { round, data } = serde_json::from_str(line)?;
        if ![data.full_us, data.steady_p95_us]
            .into_iter()
            .chain(data.landing_us)
            .all(|v| v.is_finite() && v > 0.)
            || data.windows_completed + data.windows_interrupted + data.unavailable_probes
                > data.windows_started
            || data.moves.iter().any(|m| {
                m.tick > 350
                    || m.estimated_payback_ticks
                        > proof["policy"]["horizon_ticks"].as_u64().unwrap_or(0)
            })
            || (data.mode != "automatic" && !data.moves.is_empty())
            || rounds
                .get_mut(round)
                .ok_or("invalid round")?
                .insert(data.mode.clone(), data)
                .is_some()
        {
            return Err("invalid or duplicate trial".into());
        }
    }
    if rounds.iter().any(|r| {
        r.len() != 3
            || ["fixed", "observe", "automatic"]
                .iter()
                .any(|&name| !r.contains_key(name))
    }) {
        return Err("incomplete fixed/observe/automatic triplet".into());
    }
    Ok((proof, rounds))
}
fn report(path: PathBuf) -> Result<(String, Value)> {
    let (proof, rounds) = read(&path)?;
    let n = rounds.len();
    let mut differences = BTreeMap::new();
    for (before, after) in [
        ("fixed", "observe"),
        ("fixed", "automatic"),
        ("observe", "automatic"),
    ] {
        let pairs: Vec<_> = rounds.iter().map(|r| (&r[before], &r[after])).collect();
        let landing = |arm: &str| p95(rounds.iter().flat_map(|r| r[arm].landing_us).collect());
        differences.insert(format!("{before}_to_{after}"),json!({
            "all_350_ticks":compare(&pairs,|t| t.full_us),"steady_tick_p95":compare(&pairs,|t|t.steady_p95_us),
            "pooled_landing_p95_us":[landing(before),landing(after)]}));
    }
    let auto = rounds.iter().map(|r| &r["automatic"]);
    let mut move_ticks: BTreeMap<u64, usize> = BTreeMap::new();
    for m in auto.clone().flat_map(|t| &t.moves) {
        *move_ticks.entry(m.tick).or_default() += 1;
    }
    Ok((
        path.file_name()
            .ok_or("missing directory name")?
            .to_string_lossy()
            .into_owned(),
        json!({
        "proof":proof,"matched_triplets":n,"comparisons":differences,"automatic_move_ticks":move_ticks,
        "next_sample_after_contact_change":reaction(auto.clone())?,
            "automatic_windows_woken":auto.clone().map(|t|t.windows_woken).sum::<u64>(),
            "automatic_windows_deferred":auto.clone().map(|t|t.windows_deferred).sum::<u64>(),
        "automatic_windows_ineligible":auto.clone().map(|t|t.windows_ineligible).sum::<u64>(),
        "automatic_runs_with_moves":auto.clone().filter(|t|!t.moves.is_empty()).count(),"automatic_windows_completed":auto.map(|t|t.windows_completed).sum::<u64>()}),
    ))
}
fn main() -> Result<()> {
    let args = Args::parse();
    if args.paired {
        println!(
            "{}",
            serde_json::to_string_pretty(&cross::compare_sets(&args.sets)?)?
        );
        return Ok(());
    }
    let mut sets = BTreeMap::new();
    for p in args.sets {
        let (name, summary) = report(p)?;
        if sets.insert(name, summary).is_some() {
            return Err("duplicate set name".into());
        }
    }
    println!("{}", serde_json::to_string_pretty(&sets)?);
    Ok(())
}
