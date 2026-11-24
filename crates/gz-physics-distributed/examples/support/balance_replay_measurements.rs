// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct the measured run's public ownership barriers and independently check trial summaries.
use super::{Change, Comparison, Gauge, Owners, median, read, sha};
use crate::Result;
use serde::Deserialize;
use serde_json::Value;
use std::{
    collections::BTreeMap,
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};
#[derive(Deserialize)]
struct Decision {
    tick: u64,
    source: u8,
    destination: u8,
    members: Vec<u64>,
    handoff_us: Option<f64>,
    estimated_payback_ticks: u64,
}
#[derive(Deserialize)]
struct Trial {
    mode: String,
    elapsed_us: f64,
    public_p95_us: f64,
    landing_us: Vec<f64>,
    decisions: Vec<Decision>,
    owners: Vec<u8>,
    trajectory_sha256: String,
}
#[derive(Deserialize)]
struct Row {
    round: usize,
    trial: Trial,
}
struct Cell {
    summary: Value,
    gauges: Vec<Gauge>,
    first_round: BTreeMap<String, Trial>,
}
fn load(
    root: &Path,
    cell: &str,
    layout: &str,
    expected_helper: Option<u8>,
    evidence: &mut BTreeMap<String, String>,
) -> Result<Cell> {
    let path = root.join(cell).join("result.json");
    evidence.insert(format!("{cell}/result.json"), sha(&path)?);
    let summary = read(&path)?;
    let expected = summary["native_digest"]
        .as_str()
        .ok_or("missing oracle digest")?;
    let n = summary["rounds"].as_u64().ok_or("missing rounds")? as usize;
    if n == 0
        || summary["layout"] != layout
        || summary["placement"] != "single"
        || summary["helper_limit"] != serde_json::json!(expected_helper)
        || summary["horizon"] != 64
    {
        return Err("unexpected placement campaign".into());
    }
    let path = root.join(cell).join("trials.jsonl");
    evidence.insert(format!("{cell}/trials.jsonl"), sha(&path)?);
    let modes = ["fixed", "sample", "rebalance"];
    let mut times: [[Vec<f64>; 3]; 3] = Default::default();
    let mut first_round = BTreeMap::new();
    let mut seen = vec![[false; 3]; n];
    for line in BufReader::new(File::open(path)?).lines() {
        let row: Row = serde_json::from_str(&line?)?;
        let trial = row.trial;
        let i = modes
            .iter()
            .position(|&m| m == trial.mode)
            .ok_or("invalid placement mode")?;
        if row.round == 0
            || row.round > n
            || seen[row.round - 1][i]
            || trial.owners.len() != 62
            || trial.trajectory_sha256 != expected
            || trial.landing_us.len() != 6
        {
            return Err("incomplete paired placement trial".into());
        }
        seen[row.round - 1][i] = true;
        times[0][i].push(trial.elapsed_us);
        times[1][i].push(trial.public_p95_us);
        times[2][i].extend(&trial.landing_us);
        if row.round == 1 {
            first_round.insert(trial.mode.clone(), trial);
        }
    }
    if !seen.iter().flatten().all(|v| *v) {
        return Err("incomplete triplets".into());
    }
    let mut gauges = vec![];
    let names = [
        "charged_elapsed_median",
        "public_p95_median",
        "pooled_touchdown_p95",
    ];
    for (i, arm_times) in times.iter_mut().enumerate() {
        let values = arm_times.each_mut().map(|v| {
            if i != 2 {
                median(v)
            } else {
                v.sort_by(f64::total_cmp);
                v[(v.len() * 95).div_ceil(100) - 1]
            }
        });
        if i == 2
            && values.iter().enumerate().any(|(j, t)| {
                summary["pooled_touchdown_p95_us"][j]
                    .as_f64()
                    .is_none_or(|s| (s - t).abs() > 1e-8)
            })
        {
            return Err("touchdown p95 differs from raw samples".into());
        }
        let config = if expected_helper.is_some() {
            "no_helpers"
        } else {
            "automatic_helpers"
        };
        let samples = if i == 2 { 6 * n } else { n };
        gauges.push(Gauge::new(
            &format!("{config}_{}", names[i]),
            "us",
            [values[0], values[2]],
            samples as u64,
        )?);
        if i == 0 && expected_helper.is_some() {
            let mut gauge = Gauge::new(
                "no_helpers_charged_elapsed_median",
                "us",
                [values[0], values[1]],
                n as u64,
            )?;
            gauge.labels = Some(["fixed_sampling_control", "sample_only"]);
            gauges.push(gauge);
        }
    }
    Ok(Cell {
        summary,
        gauges,
        first_round,
    })
}
fn map(layout: &str, trial: &Trial) -> Result<Owners> {
    let mut owners = vec![0u8; 62];
    let mut changes = vec![Change {
        tick: 0,
        owners: owners.clone(),
    }];
    for d in &trial.decisions {
        if d.members.is_empty()
            || d.source == d.destination
            || d.destination >= 4
            || d.members
                .iter()
                .any(|id| *id == 0 || *id > 62 || owners[*id as usize - 1] != d.source)
        {
            return Err("invalid charged ownership plan".into());
        }
        if let Some(cost) = d.handoff_us {
            if !cost.is_finite()
                || cost <= 0.
                || trial.mode != "rebalance"
                || d.tick <= changes.last().unwrap().tick
            {
                return Err("invalid successful handoff".into());
            }
            for id in &d.members {
                owners[*id as usize - 1] = d.destination
            }
            changes.push(Change {
                tick: d.tick,
                owners: owners.clone(),
            });
        }
    }
    if owners != trial.owners {
        return Err("applied ownership decisions differ from final verified pool".into());
    }
    Ok(Owners {
        first_body_id: 1,
        last_tick: 350,
        layout: layout.to_owned(),
        placement: trial.mode.clone(),
        trajectory_sha256: trial.trajectory_sha256.clone(),
        owner_changes: changes,
    })
}
pub fn balance(root: &Path, layout: &str, shared: bool) -> Result<(Comparison, [Owners; 2])> {
    let cell = |helpers: &str| {
        if shared {
            format!("shared-{layout}-single-{helpers}")
        } else {
            format!("core-{layout}-single-{helpers}-h64")
        }
    };
    let mut evidence = BTreeMap::new();
    let main = load(root, &cell("0"), layout, Some(0), &mut evidence)?;
    let auto = load(root, &cell("auto"), layout, None, &mut evidence)?;
    if main.summary["native_digest"] != auto.summary["native_digest"] {
        return Err("helper campaign diverged".into());
    }
    let fixed = &main.first_round["fixed"];
    let policy = &main.first_round["rebalance"];
    let sample = &main.first_round["sample"];
    let maps = [map(layout, fixed)?, map(layout, policy)?];
    map(layout, sample)?;
    let mut ev = vec![];
    for tick in [78, 153, 253] {
        let describe = |trial: &Trial, charged: bool| -> Result<String> {
            if let Some(d) = trial.decisions.iter().find(|d| d.tick == tick) {
                if d.handoff_us.is_some() != charged {
                    return Err("proposal application disagrees with mode".into());
                }
                let ids = if d.members.windows(2).all(|w| w[1] == w[0] + 1) {
                    format!("{}-{}", d.members[0], d.members.last().unwrap())
                } else {
                    d.members
                        .iter()
                        .map(u64::to_string)
                        .collect::<Vec<_>>()
                        .join("_")
                };
                Ok(format!(
                    "{ids}_w{}_to_w{}_payback_{}t",
                    d.source, d.destination, d.estimated_payback_ticks
                ))
            } else {
                Ok("none".to_owned())
            }
        };
        ev.push((
            tick,
            format!(
                "applied_{}_sample_only_proposed_{}",
                describe(policy, true)?,
                describe(sample, false)?
            ),
        ));
    }
    let mut gauges = main.gauges;
    gauges.extend(
        auto.gauges
            .into_iter()
            .filter(|g| g.name.contains("elapsed") || g.name.contains("touchdown")),
    );
    Ok((
        Comparison {
            arms: ["fixed", "rebalance"],
            gauges,
            events: ev,
            expected_physics: Some(fixed.trajectory_sha256.clone()),
            evidence,
        },
        maps,
    ))
}
