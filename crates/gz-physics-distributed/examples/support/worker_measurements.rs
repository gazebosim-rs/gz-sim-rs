// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measurements and validated source identities displayed by the worker comparison replay.
use crate::{Result, read};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};

#[derive(Serialize)]
pub struct Gauge {
    /// Inclusive beginning, exclusive end and vertical row for a time-specific comparison.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub window: Option<[u64; 3]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub labels: Option<[&'static str; 2]>,
    pub name: String,
    pub unit: String,
    pub values: [f64; 2],
    pub samples: u64,
}
impl Gauge {
    fn new(name: &str, unit: &str, values: [f64; 2], samples: u64) -> Result<Self> {
        if samples == 0 || values.iter().any(|v| !v.is_finite() || *v < 0.) || values == [0., 0.] {
            return Err("invalid measured range".into());
        }
        Ok(Self {
            window: None,
            labels: None,
            name: name.to_owned(),
            unit: unit.to_owned(),
            values,
            samples,
        })
    }
}
#[derive(Serialize)]
pub struct Comparison {
    pub arms: [&'static str; 2],
    pub gauges: Vec<Gauge>,
    pub events: Vec<(u64, String)>,
    pub expected_physics: Option<String>,
    pub evidence: BTreeMap<String, String>,
}
fn num(row: &Value, k: &str) -> Result<f64> {
    row[k].as_f64().ok_or_else(|| format!("missing {k}").into())
}
fn sha(path: &Path) -> Result<String> {
    Ok(format!("{:x}", Sha256::digest(std::fs::read(path)?)))
}
fn events(items: &[(u64, &str)]) -> Vec<(u64, String)> {
    items.iter().map(|(t, s)| (*t, (*s).to_owned())).collect()
}

pub fn wire(root: &Path, layout: &str, trace: &Path) -> Result<Comparison> {
    let source = root.join("summary.json");
    let summary = read(&source)?;
    let dir = trace.parent().ok_or("missing qualification directory")?;
    let mut evidence = BTreeMap::from([("summary.json".to_owned(), sha(&source)?)]);
    let mut migrated = vec![];
    for (packed, name) in [(false, "json"), (true, "packed")] {
        let p = dir.join(format!("{name}.json"));
        let report = read(&p)?;
        evidence.insert(format!("{name}.json"), sha(&p)?);
        let ids: &Vec<Value> = report["migrant_ids"].as_array().ok_or("missing IDs")?;
        let peers = if packed { 4 } else { 0 };
        if ids.len() != 31
            || report["ticks"] != 350
            || report["wire"]["packed"] != packed
            || report["wire"]["negotiated_peers"] != serde_json::json!([peers, peers, peers])
            || report["wire"]["dropped_prepare"].as_u64().unwrap_or(0) == 0
        {
            return Err("unqualified encoding".into());
        }
        let movements = report["wire"]["transfers"]
            .as_array()
            .ok_or("missing movements")?;
        if movements.len() != 4 {
            return Err("expected four transfers".into());
        }
        let groups: Vec<String> = movements
            .iter()
            .map(|v| -> Result<_> {
                let ids = v["ids"].as_array().ok_or("missing members")?;
                Ok(format!(
                    "{}-{}",
                    ids.first().ok_or("empty group")?,
                    ids.last().unwrap()
                ))
            })
            .collect::<Result<_>>()?;
        if packed && migrated != groups {
            return Err("migrated groups differ by encoding".into());
        }
        migrated = groups;
    }
    let row = &summary["layouts"][layout]["single_to_split"];
    let samples = row["pairs"].as_u64().ok_or("missing pairs")?;
    let gauges = [
        ("four_moves_median_bytes", "bytes", "median_bytes"),
        ("four_moves_median", "us", "median_us"),
        ("four_moves_p95", "us", "p95_us"),
    ]
    .map(|(name, unit, col)| {
        Gauge::new(
            name,
            unit,
            [
                num(row, &format!("json_{col}"))?,
                num(row, &format!("packed_{col}"))?,
            ],
            samples,
        )
    })
    .into_iter()
    .collect::<Result<_>>()?;
    let mut ev = events(&[
        (75, ""),
        (120, "launch_step_ack_lost_and_retried"),
        (179, "cargo_owner_replaced_before_touchdown"),
        (185, "four_process_checkpoint_resumed"),
    ]);
    ev[0].1 = format!("prepare_ack_retry_and_handoff_ids_{}", migrated.join("_"));
    Ok(Comparison {
        arms: ["json", "packed"],
        gauges,
        events: ev,
        expected_physics: None,
        evidence,
    })
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    (v[v.len() / 2] + v[(v.len() - 1) / 2]) / 2.
}
#[derive(Deserialize)]
struct RetryCycle {
    cycle: usize,
    trajectory_sha256: String,
    counts: Value,
    retry_us: f64,
    retry_restarted_workers: usize,
    regular_p95_us: f64,
}
#[derive(Deserialize)]
struct RetryPair {
    pair: usize,
    layout: String,
    reference: Vec<RetryCycle>,
    candidate: Vec<RetryCycle>,
}
/// Read only complete pairs from both fixed CPU campaigns; hash the exact prefixes used.
pub fn retry(root: &Path, layout: &str, pairs_per_lane: usize) -> Result<Comparison> {
    let mut retry = [vec![], vec![]];
    let mut processes = [vec![], vec![]];
    let mut regular = [vec![], vec![]];
    let mut expected = None;
    let mut evidence = BTreeMap::new();
    for lane in ["pairs-4-7", "pairs-8-11"] {
        let file = File::open(root.join(lane).join("pairs.jsonl"))?;
        let mut count = 0;
        let mut hash = Sha256::new();
        for (i, line) in BufReader::new(file)
            .lines()
            .take(pairs_per_lane)
            .enumerate()
        {
            let line = line?;
            hash.update(line.as_bytes());
            hash.update(b"\n");
            let row: RetryPair = serde_json::from_str(&line)?;
            if row.pair != i + 1 {
                return Err("unordered pair file".into());
            }
            count += 1;
            if row.layout != layout {
                continue;
            }
            if row.reference.is_empty() || row.reference.len() != row.candidate.len() {
                return Err("unpaired cycles".into());
            }
            for (index, (a, b)) in row.reference.iter().zip(&row.candidate).enumerate() {
                if a.cycle != index + 1
                    || b.cycle != a.cycle
                    || a.trajectory_sha256 != b.trajectory_sha256
                    || a.counts != b.counts
                {
                    return Err("non-equivalent paired cycles".into());
                }
                if index == 0 {
                    if expected
                        .as_ref()
                        .is_some_and(|digest| digest != &a.trajectory_sha256)
                    {
                        return Err("initial trajectory differs between trials".into());
                    }
                    expected = Some(a.trajectory_sha256.clone());
                }
                for (arm, c) in [a, b].into_iter().enumerate() {
                    retry[arm].push(c.retry_us);
                    processes[arm].push(c.retry_restarted_workers as f64);
                    regular[arm].push(c.regular_p95_us);
                }
            }
        }
        if count != pairs_per_lane {
            return Err(format!("{lane} has only {count} completed pairs").into());
        }
        evidence.insert(
            format!("{lane}/pairs.jsonl:first_{count}_lines"),
            format!("{:x}", hash.finalize()),
        );
    }
    let samples = retry[0].len() as u64;
    if samples == 0 {
        return Err("no measured cycles".into());
    }
    let gauges = vec![
        Gauge::new(
            "lost_step_ack_retry_median",
            "us",
            retry.map(|mut v| median(&mut v)),
            samples,
        )?,
        Gauge::new(
            "retry_restarted_processes_median",
            "workers",
            processes.map(|mut v| median(&mut v)),
            samples,
        )?,
        Gauge::new(
            "regular_tick_p95_median",
            "us",
            regular.map(|mut v| median(&mut v)),
            samples,
        )?,
    ];
    Ok(Comparison {
        arms: ["prior", "revised"],
        gauges,
        events: events(&[
            (145, "loaded_slab_migrated"),
            (160, "lost_step_ack_retried"),
            (219, "cargo_owner_replaced_before_touchdown"),
            (225, "four_process_checkpoint_resumed"),
        ]),
        expected_physics: expected,
        evidence,
    })
}

#[derive(Clone, Deserialize, Serialize)]
pub struct Change {
    pub tick: u64,
    pub owners: Vec<u8>,
}
#[derive(Clone, Deserialize, Serialize)]
pub struct Owners {
    pub first_body_id: u64,
    pub last_tick: u64,
    pub layout: String,
    #[serde(default)]
    pub placement: String,
    pub trajectory_sha256: String,
    pub owner_changes: Vec<Change>,
}
impl Owners {
    fn read(path: &Path, layout: &str, placement: &str, workers: u8) -> Result<Self> {
        let map: Owners = serde_json::from_reader(File::open(path)?)?;
        if map.first_body_id != 1
            || map.last_tick != 350
            || map.layout != layout
            || map.placement != placement
            || map.owner_changes.first().is_none_or(|x| x.tick != 0)
            || map.owner_changes.windows(2).any(|w| w[0].tick >= w[1].tick)
            || map.owner_changes.iter().any(|r| {
                r.tick > 350 || r.owners.len() != 62 || r.owners.iter().any(|&w| w >= workers)
            })
        {
            return Err("invalid cold ownership sidecar".into());
        }
        Ok(map)
    }
    pub fn at(&self, tick: u64) -> &[u8] {
        &self.owner_changes[self.owner_changes.partition_point(|r| r.tick <= tick) - 1].owners
    }
}
pub fn cold(root: &Path, layout: &str) -> Result<(Comparison, [Owners; 2])> {
    let arms = ["spatial", "contact"];
    let mut evidence = BTreeMap::new();
    let mut owners = vec![];
    for arm in arms {
        let name = format!("owners-{layout}-{arm}.json");
        let path = root.join(&name);
        let map = Owners::read(&path, layout, arm, 4)?;
        evidence.insert(name, sha(&path)?);
        owners.push(map);
    }
    let owners: [Owners; 2] = owners.try_into().map_err(|_| "missing placement")?;
    let expected = owners[0].trajectory_sha256.clone();
    if owners[1].trajectory_sha256 != expected {
        return Err("cold trajectory mismatch".into());
    }
    let mut times: [[Vec<f64>; 2]; 4] = Default::default();
    for lane in ["cpu0", "cpu8"] {
        let name = format!("{lane}-{layout}.jsonl");
        let path = root.join(&name);
        evidence.insert(name, sha(&path)?);
        for (round, line) in BufReader::new(File::open(path)?).lines().enumerate() {
            let pair: Value = serde_json::from_str(&line?)?;
            if pair["round"] != round + 1
                || pair["spatial"]["ecs_poses_sha256"] != pair["contact"]["ecs_poses_sha256"]
            {
                return Err("invalid paired scene timing".into());
            }
            for (arm, key) in arms.iter().enumerate() {
                let row = &pair[key];
                if row["physics_sha256"] != expected
                    || row["layout"] != layout
                    || row["workers"] != 4
                {
                    return Err("cold benchmark disagrees with ownership trace".into());
                }
                let moves = owners[arm]
                    .at(0)
                    .iter()
                    .zip(owners[arm].at(1))
                    .filter(|(a, b)| a != b)
                    .count();
                if row["first_step_moved_bodies"] != moves {
                    return Err("first-step ownership count mismatch".into());
                }
                let first = num(row, "first_step_us")?;
                for (samples, value) in times.iter_mut().zip([
                    first,
                    first + num(row, "load_us")?,
                    num(row, "measured_p95_us")?,
                    moves as f64,
                ]) {
                    samples[arm].push(value)
                }
            }
        }
    }
    let mut gauges = vec![];
    for ((name, unit), mut data) in [
        ("first_step_median", "us"),
        ("load_and_first_step_median", "us"),
        ("regular_tick_p95_median", "us"),
        ("first_step_moved", "bodies"),
    ]
    .into_iter()
    .zip(times)
    {
        let samples = data[0].len();
        if samples == 0 {
            return Err("empty cold timing file".into());
        }
        let med = data.each_mut().map(|v| median(v));
        gauges.push(Gauge::new(name, unit, med, samples as u64)?);
    }
    Ok((
        Comparison {
            arms,
            gauges,
            events: events(&[(1, "first_step_contact_islands_placed")]),
            expected_physics: Some(expected),
            evidence,
        },
        owners,
    ))
}

/// Display the packed transport in both arms and verify every counted RPC against the raw trials.
pub fn batch(root: &Path, layout: &str, trace: &Path) -> Result<Comparison> {
    let source = root.join("summary.json");
    let summary = read(&source)?;
    let row = &summary["packed"][layout];
    let expected = row["physics_sha256"]
        .as_str()
        .ok_or("missing paired digest")?
        .to_owned();
    let pairs = row["pairs"].as_u64().ok_or("missing pairs")?;
    let arms = ["separate", "atomic"];
    let mut evidence = BTreeMap::from([("summary.json".to_owned(), sha(&source)?)]);
    let mut rpcs: [Option<u64>; 2] = [None, None];
    let mut samples = 0;
    for key in ["packed-0-3.jsonl", "packed-8-11.jsonl"] {
        let path = root.join(key);
        evidence.insert(key.to_owned(), sha(&path)?);
        for line in BufReader::new(File::open(path)?).lines() {
            let v: Value = serde_json::from_str(&line?)?;
            if v["layout"] != layout {
                continue;
            }
            samples += 1;
            if v["packed"] != true {
                return Err("expected a packed paired trial".into());
            }
            for (i, name) in arms.iter().enumerate() {
                let a = &v[name];
                let count = a["requests"].as_u64().ok_or("missing requests")?;
                if a["physics_sha256"] != expected
                    || a["replies"] != count
                    || rpcs[i].is_some_and(|n| n != count)
                {
                    return Err("inconsistent handoff traffic".into());
                }
                rpcs[i] = Some(count);
            }
        }
    }
    if samples != pairs || pairs == 0 {
        return Err("missing raw paired samples".into());
    }
    let dir = trace.parent().ok_or("missing qualification parent")?;
    let mut moves = [0.; 2];
    let mut fault = None;
    for (i, name) in arms.iter().enumerate() {
        let path = dir.join(format!("{name}.json"));
        evidence.insert(format!("{name}.json"), sha(&path)?);
        let v = read(&path)?;
        let w = &v["wire"];
        if w["atomic"] != (i == 1)
            || w["packed"] != true
            || w["negotiated_peers"] != serde_json::json!([4, 4, 4])
            || v["migrant_ids"]
                .as_array()
                .is_none_or(|ids| ids.len() != 31)
        {
            return Err("unqualified batch transfer".into());
        }
        let id = w["dropped_prepare"]
            .as_u64()
            .ok_or("missing fault location")?;
        if fault.is_some_and(|f| f != id) {
            return Err("fault on different groups".into());
        }
        fault = Some(id);
        moves[i] = num(w, "already_moved_at_failure")?;
    }
    let mut gauges = vec![];
    for (name, unit, column) in [
        ("four_moves_median", "us", "median_us"),
        ("four_moves_p95", "us", "p95_us"),
        ("four_moves_payload", "bytes", "bytes"),
    ] {
        gauges.push(Gauge::new(
            name,
            unit,
            [
                num(row, &format!("separate_{column}"))?,
                num(row, &format!("atomic_{column}"))?,
            ],
            pairs,
        )?);
    }
    gauges.push(Gauge::new(
        "four_moves_rpc_count",
        "calls",
        [rpcs[0].unwrap() as f64, rpcs[1].unwrap() as f64],
        pairs,
    )?);
    gauges.push(Gauge::new(
        "groups_published_at_third_prepare_loss",
        "groups",
        moves,
        1,
    )?);
    let mut ev = events(&[
        (75, ""),
        (120, "launch_step_ack_lost_and_retried"),
        (179, "cargo_owner_replaced_before_touchdown"),
        (185, "four_process_checkpoint_resumed"),
    ]);
    ev[0].1 = format!(
        "four_groups_31_ids_prepare_loss_root_{}_then_retry",
        fault.unwrap()
    );
    Ok(Comparison {
        arms,
        gauges,
        events: ev,
        expected_physics: Some(expected),
        evidence,
    })
}

/// Public step timing includes the original dispatch, recovery and the ECS publication.
pub fn scene_retry(root: &Path, layout: &str) -> Result<(Comparison, [Owners; 2])> {
    let arms = ["prior", "revised"];
    let mut evidence = BTreeMap::new();
    let mut maps = vec![];
    let mut ecs = None;
    for arm in arms {
        let name = format!("owners-{layout}-{arm}.json");
        let path = root.join(&name);
        let map = Owners::read(&path, layout, "contact", 2)?;
        evidence.insert(name, sha(&path)?);
        let name = format!("clip-{layout}-{arm}.json");
        let path = root.join(&name);
        let row = read(&path)?;
        evidence.insert(name, sha(&path)?);
        if row["workers"] != 2
            || row["physics_sha256"] != map.trajectory_sha256
            || row["fault"]["tick"] != 160
            || row["total_moved_bodies"] != 0
            || row["layout"] != layout
        {
            return Err("invalid fault-injected public ECS clip".into());
        }
        let pose = row["ecs_poses_sha256"]
            .as_str()
            .ok_or("missing ECS digest")?
            .to_owned();
        if ecs.as_ref().is_some_and(|s| s != &pose) {
            return Err("clip ECS poses differ".into());
        }
        ecs = Some(pose);
        maps.push(map);
    }
    let maps: [Owners; 2] = maps.try_into().map_err(|_| "missing clip")?;
    let expected = maps[0].trajectory_sha256.clone();
    if maps[1].trajectory_sha256 != expected || (0..351).any(|t| maps[0].at(t) != maps[1].at(t)) {
        return Err("clip ownership or physics differs".into());
    }
    let mut gauges = vec![];
    for width in 2..=8 {
        let dir = format!("{layout}-{width}");
        let name = format!("{dir}/result.json");
        let path = root.join(&name);
        let summary = read(&path)?;
        evidence.insert(name, sha(&path)?);
        let name = format!("{dir}/pairs.jsonl");
        let path = root.join(&name);
        evidence.insert(name, sha(&path)?);
        let mut costs: [Vec<f64>; 2] = Default::default();
        let mut changes = [0u64; 2];
        for line in BufReader::new(File::open(path)?).lines() {
            let pair: Value = serde_json::from_str(&line?)?;
            for (i, key) in ["reference", "candidate"].into_iter().enumerate() {
                let row = &pair[key];
                if row["physics_sha256"] != expected
                    || row["ecs_poses_sha256"].as_str() != ecs.as_deref()
                    || row["workers"] != width
                    || row["layout"] != layout
                    || row["fault"]["tick"] != pair["reference"]["fault"]["tick"]
                {
                    return Err("non-equivalent public ECS pair".into());
                }
                costs[i].push(num(&row["fault"], "public_step_us")?);
                changes[i] += row["fault"]["replaced_workers"]
                    .as_u64()
                    .ok_or("missing PID count")?;
            }
        }
        let samples = costs[0].len() as u64;
        if samples == 0
            || summary["matched_pairs"] != samples
            || summary["replaced_workers"] != serde_json::json!(changes)
        {
            return Err("missing public ECS fault sample".into());
        }
        let med = costs.map(|mut v| median(&mut v));
        if (med[0] - num(&summary, "reference_public_step_median_us")?).abs() > 1e-8
            || (med[1] - num(&summary, "candidate_public_step_median_us")?).abs() > 1e-8
        {
            return Err("public timing summary differs from raw samples".into());
        }
        gauges.push(Gauge::new(
            &format!("{width}_workers_full_fault_step_median"),
            "us",
            med,
            samples,
        )?);
        if width == 2 {
            gauges.push(Gauge::new(
                "two_workers_mean_pid_changes_per_fault",
                "workers",
                changes.map(|n| n as f64 / samples as f64),
                samples,
            )?);
        }
    }
    Ok((
        Comparison {
            arms,
            gauges,
            events: events(&[(160, "lost_reply_recovered_inside_public_step")]),
            expected_physics: Some(expected),
            evidence,
        },
        maps,
    ))
}

#[path = "balance_replay_measurements.rs"]
mod balance_data;
pub use balance_data::balance;

#[path = "transport_replay_measurements.rs"]
mod transport_data;
pub use transport_data::{digest, sparse};

#[path = "scheduled_replay_measurements.rs"]
mod scheduled_data;
pub use scheduled_data::scheduled;

#[path = "checkpoint_replay_measurements.rs"]
mod checkpoint_data;
pub use checkpoint_data::checkpoint;
