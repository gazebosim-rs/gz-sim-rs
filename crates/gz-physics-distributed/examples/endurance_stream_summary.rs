// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recheck every flushed public reset or matched endurance pair across independent runs.
use clap::{Parser, ValueEnum};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Clone, Copy, ValueEnum)]
enum Mode {
    PublicReset,
    Paired,
}
#[derive(Parser)]
struct Args {
    #[arg(value_enum)]
    mode: Mode,
    /// Run directories in observation order. Each starts a fresh simulation process.
    #[arg(num_args=1..)]
    directories: Vec<PathBuf>,
    /// Require a verified completion marker on the last segment.
    #[arg(long)]
    complete: bool,
}
#[derive(Debug, Default, Serialize)]
struct Counts {
    rows: u64,
    ticks: u64,
    body_states: u64,
    kills: u64,
    lost_acks: u64,
    handoffs: u64,
    disk_restores: u64,
    retry_replacements: u64,
    pair_receipts: u64,
    floor_receipts: u64,
}
fn number(v: &Value, k: &str) -> Result<u64> {
    v.get(k)
        .and_then(Value::as_u64)
        .ok_or_else(|| format!("missing integer {k}").into())
}
fn digest<'a>(v: &'a Value, k: &str) -> Result<&'a str> {
    v.get(k)
        .and_then(Value::as_str)
        .filter(|s| s.len() == 64 && s.bytes().all(|b| b.is_ascii_hexdigit()))
        .ok_or_else(|| format!("invalid digest {k}").into())
}
#[derive(Deserialize)]
struct Reset {
    epoch: u64,
    pid_killed: bool,
    trajectory: Value,
}
fn reset(row: Reset, index: u64, control: &Value, counts: &mut Counts) -> Result<()> {
    if row.epoch != index || row.pid_killed != index.is_multiple_of(2) {
        return Err("incomplete public reset sequence".into());
    }
    let t = &row.trajectory;
    for field in ["physics_sha256", "public_ids_sha256", "ecs_poses_sha256"] {
        if digest(t, field)? != digest(control, field)? {
            return Err(format!("changed {field} at reset {index}").into());
        }
    }
    for field in [
        "workers",
        "layout",
        "initial_counts",
        "total_moved_bodies",
        "pair_receipts",
        "floor_receipts",
    ] {
        if t.get(field).is_none() || t[field] != control[field] {
            return Err(format!("changed {field} at reset {index}").into());
        }
    }
    let fault = &t["fault"];
    if number(fault, "replaced_workers")? != u64::from(row.pid_killed)
        || number(fault, "tick")? != [160, 140, 240, 220][((index - 1) % 4) as usize]
    {
        return Err(format!("incorrect supervised barrier at reset {index}").into());
    }
    let bodies = control["initial_counts"]
        .as_array()
        .ok_or("no owner counts")?
        .iter()
        .map(|v| v.as_u64().ok_or("invalid owner count"))
        .collect::<std::result::Result<Vec<_>, _>>()?
        .into_iter()
        .sum::<u64>();
    if bodies != 62 {
        return Err("public fixture changed population".into());
    }
    counts.rows += 1;
    counts.ticks += 350;
    counts.body_states += 350 * bodies;
    counts.kills += u64::from(row.pid_killed);
    counts.lost_acks += u64::from(!row.pid_killed);
    counts.pair_receipts += number(t, "pair_receipts")?;
    counts.floor_receipts += number(t, "floor_receipts")?;
    Ok(())
}
#[derive(Deserialize)]
struct Pair {
    pair: u64,
    layout: String,
    reference_first: bool,
    reference: Vec<Value>,
    candidate: Vec<Value>,
}
fn pair(
    row: Pair,
    index: u64,
    groups: &mut BTreeMap<String, Counts>,
    layouts: &mut BTreeMap<String, [u64; 4]>,
) -> Result<()> {
    let odd = !index.is_multiple_of(2);
    if row.pair != index
        || row.layout != if odd { "adjacent" } else { "separated" }
        || row.reference_first != matches!(index % 4, 0 | 1)
        || row.reference.is_empty()
        || row.candidate.len() != row.reference.len()
    {
        return Err("unmatched ordered trial".into());
    }
    let mut restarts = [0; 2];
    for (i, (r, c)) in row.reference.iter().zip(&row.candidate).enumerate() {
        if number(r, "cycle")? != i as u64 + 1
            || r["cycle"] != c["cycle"]
            || r["counts"] != c["counts"]
            || digest(r, "trajectory_sha256")? != digest(c, "trajectory_sha256")?
            || number(&r["counts"], "ticks")? != 350 * (i as u64 + 1)
        {
            return Err(format!("mismatched physics at pair {index}, cycle {}", i + 1).into());
        }
        for (arm, record) in [r, c].into_iter().enumerate() {
            let replaced = number(record, "retry_restarted_workers")?;
            if replaced != [4, 0][arm] {
                return Err("incorrect retry replacement scope".into());
            }
            restarts[arm] += replaced;
        }
    }
    let counters = &row.reference.last().unwrap()["counts"];
    let ticks = number(counters, "ticks")?;
    let cycles = row.reference.len() as u64;
    let layout = layouts.entry(row.layout.clone()).or_default();
    layout[0] += cycles;
    layout[1] += ticks;
    layout[2] += restarts[0];
    layout[3] += restarts[1];
    for (i, arm) in ["reference", "candidate"].into_iter().enumerate() {
        let s = groups.entry(arm.to_owned()).or_default();
        s.rows += 1;
        s.ticks += ticks;
        s.body_states += number(counters, "body_states")?;
        s.kills += number(counters, "replacements")?;
        s.lost_acks += number(counters, "lost_acks")?;
        s.disk_restores += number(counters, "restores")?;
        s.handoffs += number(counters, "migrations")?;
        s.retry_replacements += restarts[i];
        s.pair_receipts += number(counters, "pair_receipts")?;
        s.floor_receipts += number(counters, "ground_receipts")?;
    }
    Ok(())
}
fn marker(path: &Path) -> Result<Option<Value>> {
    match fs::read(path) {
        Ok(bytes) => Ok(Some(serde_json::from_slice(&bytes)?)),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(e.into()),
    }
}
fn main() -> Result<()> {
    let a = Args::parse();
    let mut groups = BTreeMap::new();
    let mut segments = Vec::new();
    let mut first_control: Option<Value> = None;
    for (i, dir) in a.directories.iter().enumerate() {
        let mark = marker(&dir.join("result.json"))?;
        if i + 1 == a.directories.len() && a.complete && mark.is_none() {
            return Err("final segment is unsealed".into());
        }
        let (file, control) = match a.mode {
            Mode::Paired => ("pairs.jsonl", Value::Null),
            Mode::PublicReset => (
                "epochs.jsonl",
                serde_json::from_slice(&fs::read(dir.join("control.json"))?)?,
            ),
        };
        if let Some(old) = &first_control {
            for k in [
                "layout",
                "workers",
                "ecs_poses_sha256",
                "public_ids_sha256",
                "physics_sha256",
                "initial_counts",
                "pair_receipts",
                "floor_receipts",
            ] {
                if old[k] != control[k] {
                    return Err("different public reset controls between segments".into());
                }
            }
        } else {
            first_control = Some(control.clone());
        }
        let mut r = BufReader::new(File::open(dir.join(file))?);
        let mut line = String::new();
        let mut index = 0;
        let mut elapsed = 0.;
        let mut layouts = BTreeMap::new();
        let mut trailing_bytes = 0;
        while r.read_line(&mut line)? > 0 {
            if !line.ends_with('\n') {
                if mark.is_some() {
                    return Err("partial row in a sealed segment".into());
                }
                trailing_bytes = line.len();
                break;
            }
            let value: Value = serde_json::from_str(&line)?;
            line.clear();
            index += 1;
            let end = value["elapsed_s"].as_f64().ok_or("missing elapsed time")?;
            if end <= elapsed || !end.is_finite() {
                return Err("non-increasing measurement clock".into());
            }
            elapsed = end;
            match a.mode {
                Mode::Paired => pair(
                    serde_json::from_value(value)?,
                    index,
                    &mut groups,
                    &mut layouts,
                )?,
                Mode::PublicReset => reset(
                    serde_json::from_value(value)?,
                    index,
                    &control,
                    groups.entry("public".to_owned()).or_default(),
                )?,
            }
        }
        if index == 0 {
            return Err("empty endurance segment".into());
        }
        if let Some(m) = &mark {
            match a.mode {
                Mode::PublicReset => {
                    if number(m, "epochs")? != index
                        || number(m, "ticks")? != index * 350
                        || number(m, "kills")? != index / 2
                        || number(m, "lost_acks")? != index - index / 2
                        || number(m, "public_resets")? != index
                        || digest(m, "native_sha256")? != digest(&control, "physics_sha256")?
                        || digest(m, "ecs_sha256")? != digest(&control, "ecs_poses_sha256")?
                        || digest(m, "public_ids_sha256")? != digest(&control, "public_ids_sha256")?
                    {
                        return Err("wrong public reset completion marker".into());
                    }
                }
                Mode::Paired => {
                    for (layout, [cycles, ticks, r, c]) in &layouts {
                        let v = &m[layout];
                        if number(v, "matched_cycles")? != *cycles
                            || number(v, "ticks_per_arm")? != *ticks
                            || v["restarted_workers"] != json!([r, c])
                        {
                            return Err("wrong paired completion marker".into());
                        }
                    }
                }
            }
        }
        segments.push(json!({"directory":dir,"sealed":mark.is_some(),"rows":index,"last_flushed_elapsed_s":elapsed,
            "discarded_trailing_bytes":trailing_bytes}));
    }
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({"segments":segments,"arms":groups}))?
    );
    Ok(())
}
