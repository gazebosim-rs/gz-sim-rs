// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify paired sampling measurements against the actual frozen ownership and native digest.
use super::{Change, Comparison, Gauge, Owners, median, read, sha};
use crate::Result;
use serde::Deserialize;
use std::{
    collections::BTreeMap,
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};
#[derive(Deserialize)]
struct Row {
    round: usize,
    trial: Trial,
}
#[derive(Deserialize)]
struct Trial {
    mode: String,
    trajectory_sha256: String,
    owners: Vec<u8>,
    scheduled_worker_masks: Vec<u8>,
    sampled_p95_us: f64,
    sampled_total_us: f64,
    public_p95_us: f64,
    elapsed_us: f64,
    decisions: Vec<Proposal>,
}
#[derive(Deserialize)]
struct Proposal {
    handoff_us: Option<f64>,
}
pub fn scheduled(root: &Path, layout: &str) -> Result<(Comparison, [Owners; 2])> {
    let key = format!("core-{layout}-split-auto");
    let path = root.join(&key);
    let result = read(&path.join("result.json"))?;
    let n = result["rounds"].as_u64().ok_or("missing trial count")? as usize;
    let digest = result["native_digest"]
        .as_str()
        .ok_or("missing native digest")?;
    if n == 0
        || result["layout"] != layout
        || result["placement"] != "split"
        || !result["helper_limit"].is_null()
        || result["actual_moves"] != 0
        || result["native_ticks_per_trial"] != 350
    {
        return Err("unexpected scheduled sampling experiment".into());
    }
    let mut evidence = BTreeMap::new();
    for file in ["result.json", "trials.jsonl"] {
        evidence.insert(format!("{key}/{file}"), sha(&path.join(file))?);
    }
    let mut times: [[Vec<f64>; 3]; 4] = Default::default();
    let mut seen = vec![[false; 3]; n];
    let mut owners = None;
    let mut masks = None;
    for line in BufReader::new(File::open(path.join("trials.jsonl"))?).lines() {
        let row: Row = serde_json::from_str(&line?)?;
        let t = row.trial;
        let arm = ["fixed", "sample", "scheduled"]
            .iter()
            .position(|&s| s == t.mode)
            .ok_or("unexpected arm")?;
        if row.round == 0
            || row.round > n
            || seen[row.round - 1][arm]
            || t.trajectory_sha256 != digest
            || t.owners.len() != 62
            || !t.owners.contains(&0)
            || !t.owners.contains(&1)
            || t.owners.iter().any(|&w| w > 1)
            || owners.as_ref().is_some_and(|m| m != &t.owners)
            || t.decisions.iter().any(|d| d.handoff_us.is_some())
        {
            return Err("trial changed physics, ownership or pairing".into());
        }
        match arm {
            0 if !t.scheduled_worker_masks.is_empty() => {
                return Err("fixed arm opened a window".into());
            }
            1 if t.scheduled_worker_masks != [0; 3] => {
                return Err("serial arm granted scheduled mode".into());
            }
            2 => {
                if t.scheduled_worker_masks.len() != 3
                    || masks
                        .as_ref()
                        .is_some_and(|m| m != &t.scheduled_worker_masks)
                {
                    return Err("scheduled peers varied across trial windows".into());
                }
                masks = Some(t.scheduled_worker_masks);
            }
            _ => {}
        }
        seen[row.round - 1][arm] = true;
        owners = Some(t.owners);
        for (i, v) in [
            t.sampled_p95_us,
            t.sampled_total_us,
            t.public_p95_us,
            t.elapsed_us,
        ]
        .into_iter()
        .enumerate()
        {
            if !v.is_finite() || v <= 0. {
                return Err("invalid sampling time".into());
            }
            times[i][arm].push(v);
        }
    }
    if seen.iter().flatten().any(|&s| !s) {
        return Err("incomplete triplets".into());
    }
    let medians = times.each_mut().map(|v| v.each_mut().map(|r| median(r)));
    for (i, k) in ["sampled_window_p95_us", "sampled_window_total_us"]
        .into_iter()
        .enumerate()
    {
        for (j, &m) in medians[i].iter().enumerate() {
            if result[k][j].as_f64().is_none_or(|v| (v - m).abs() > 1e-7) {
                return Err("summary differs from raw sample".into());
            }
        }
    }
    let mut gauges = vec![];
    for (row, (source, name, other)) in [
        (0, "versus_fixed_sampled_24_ticks_p95_median", 0),
        (0, "sampled_24_ticks_p95_median", 1),
        (1, "sampled_24_ticks_total_median", 1),
        (2, "whole_run_public_p95_median", 1),
        (3, "whole_run_charged_elapsed_median", 1),
    ]
    .into_iter()
    .enumerate()
    {
        let mut g = Gauge::new(
            name,
            "us",
            [medians[source][other], medians[source][2]],
            n as u64,
        )?;
        g.labels = Some([["fixed", "serial_sampling"][other], "scheduled_sampling"]);
        g.window = Some([0, 351, row as u64]);
        gauges.push(g);
    }
    let masks = masks.ok_or("missing scheduled samples")?;
    let mut events = vec![(
        0,
        "two_active_owners_fixed_placement_four_processes".to_owned(),
    )];
    for (i, at) in [71, 146, 246].into_iter().enumerate() {
        events.push((
            at,
            format!(
                "window_{}_start_scheduled_mode_ack_mask_{}_serial_mask_0",
                i + 1,
                masks[i]
            ),
        ));
        events.push((
            at + 7,
            format!("window_{}_complete_8_ticks_ownership_unchanged", i + 1),
        ));
    }
    let map = Owners {
        first_body_id: 1,
        last_tick: 350,
        layout: layout.to_owned(),
        placement: "split".to_owned(),
        trajectory_sha256: digest.to_owned(),
        owner_changes: vec![Change {
            tick: 0,
            owners: owners.unwrap(),
        }],
    };
    Ok((
        Comparison {
            arms: ["serial_sampling", "scheduled_sampling"],
            gauges,
            events,
            expected_physics: Some(digest.to_owned()),
            evidence,
        },
        [map.clone(), map],
    ))
}
