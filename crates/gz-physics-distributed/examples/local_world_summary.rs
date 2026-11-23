// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Summarize matched coordinator/worker release trials from local_world_trials.
#[path = "support/local_world_control.rs"]
mod local_world_control;
use clap::Parser;
use local_world_control::physics_and_layout;
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::PathBuf,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const ARMS: [&str; 4] = ["old_old", "old_new", "new_old", "new_new"];

#[derive(Parser)]
struct Args {
    input: PathBuf,
}

#[derive(Deserialize)]
struct Trial {
    workload: String,
    round: u8,
    #[serde(flatten)]
    data: Design,
}

#[derive(Deserialize)]
#[serde(untagged)]
enum Design {
    Crossed {
        arms: BTreeMap<String, Value>,
        order: [String; 4],
    },
    Paired {
        baseline: Value,
        current: Value,
        baseline_first: bool,
    },
}

fn median(values: &[f64]) -> f64 {
    (values[values.len() / 2] + values[(values.len() - 1) / 2]) / 2.0
}

// Narrowest central order-statistic interval obtained by trimming both tails
// while retaining at least 95% binomial coverage of the population median.
fn median_interval(values: &[f64]) -> Option<[f64; 2]> {
    let n = values.len();
    let mut probability = 0.5_f64.powi(n as i32);
    let mut tail = probability;
    if tail > 0.025 {
        return None;
    }
    let mut rank = 0;
    while rank + 1 < n / 2 {
        probability *= (n - rank) as f64 / (rank + 1) as f64;
        if tail + probability > 0.025 {
            break;
        }
        tail += probability;
        rank += 1;
    }
    Some([values[rank], values[n - 1 - rank]])
}

fn contrast(rows: &[Vec<Value>], baseline: usize, candidate: usize, key: &str) -> Value {
    let mut deltas = rows
        .iter()
        .map(|row| {
            let old = row[baseline][key].as_f64().expect("validated time");
            let new = row[candidate][key].as_f64().expect("validated time");
            100.0 * (new / old - 1.0)
        })
        .collect::<Vec<_>>();
    deltas.sort_by(f64::total_cmp);
    json!({"paired_median_pct":median(&deltas),"median_interval95_pct":median_interval(&deltas),
           "improved":deltas.partition_point(|d| *d < 0.)})
}

fn summarize(mut trials: Vec<Trial>) -> Result<Value> {
    trials.sort_by_key(|row| row.round);
    let mut rows = Vec::new();
    let mut orders = BTreeMap::new();
    let mut expected_control = None;
    let crossed = matches!(
        trials.first().ok_or("empty group")?.data,
        Design::Crossed { .. }
    );
    for (index, trial) in trials.into_iter().enumerate() {
        if usize::from(trial.round) != index + 1 {
            return Err("missing or duplicate measured round".into());
        }
        let (metrics, order) = match trial.data {
            Design::Paired {
                baseline,
                current,
                baseline_first,
            } if !crossed => (
                vec![baseline, current],
                if baseline_first {
                    vec![0, 1]
                } else {
                    vec![1, 0]
                },
            ),
            Design::Crossed { mut arms, order } if crossed => {
                let positions = order
                    .iter()
                    .map(|name| ARMS.iter().position(|s| s == name).ok_or("unknown arm"))
                    .collect::<std::result::Result<Vec<_>, _>>()?;
                let mut unique = positions.clone();
                unique.sort_unstable();
                if unique != [0, 1, 2, 3] || arms.len() != 4 {
                    return Err("incomplete four-arm block".into());
                }
                let metrics = ARMS
                    .iter()
                    .map(|name| arms.remove(*name).ok_or("missing arm"))
                    .collect::<std::result::Result<Vec<_>, _>>()?;
                (metrics, positions)
            }
            _ => return Err("mixed trial designs within one workload".into()),
        };
        for values in &metrics {
            let control = physics_and_layout(values)?;
            if expected_control.get_or_insert_with(|| control.clone()) != &control {
                return Err(format!("physics or workload changed at round {}", trial.round).into());
            }
        }
        *orders.entry(order).or_insert(0usize) += 1;
        rows.push(metrics);
    }
    let end = if crossed { 3 } else { 1 };
    let mut contrasts = BTreeMap::new();
    let mut compare = |name, old, new| {
        contrasts.insert(
            name,
            json!({"elapsed":contrast(&rows,old,new,"elapsed_ms"),
                                     "tick_p95":contrast(&rows,old,new,"tick_p95_us")}),
        );
    };
    compare("combined", 0, end);
    if crossed {
        compare("coordinator_with_old_worker", 0, 2);
        compare("coordinator_with_new_worker", 1, 3);
        compare("worker_with_old_coordinator", 0, 1);
        compare("worker_with_new_coordinator", 2, 3);
    }
    let orders = orders
        .into_iter()
        .map(|(order, blocks)| json!({"order":order,"blocks":blocks}))
        .collect::<Vec<_>>();
    Ok(
        json!({"rounds":rows.len(),"orders":orders,"contrasts":contrasts,"physics":expected_control}),
    )
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut groups: BTreeMap<String, Vec<Trial>> = BTreeMap::new();
    for line in BufReader::new(File::open(args.input)?).lines() {
        let trial: Trial = serde_json::from_str(&line?)?;
        groups
            .entry(trial.workload.clone())
            .or_default()
            .push(trial);
    }
    if groups.is_empty() {
        return Err("empty trial file".into());
    }
    let mut report = BTreeMap::new();
    for (name, rows) in groups {
        report.insert(
            name.clone(),
            summarize(rows).map_err(|e| format!("{name}: {e}"))?,
        );
    }
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_central_interval_uses_order_statistics_and_requires_enough_pairs() {
        let values: Vec<_> = (1..=32).map(f64::from).collect();
        assert_eq!(median_interval(&values[..5]), None);
        assert_eq!(median_interval(&values[..6]), Some([1., 6.]));
        assert_eq!(median_interval(&values), Some([10., 23.]));
    }
}
