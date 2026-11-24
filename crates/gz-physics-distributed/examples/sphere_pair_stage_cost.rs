// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Join per-process timing events on stdin with the stock benchmark's accepted intervals.
use clap::Parser;
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs::File,
    io::{self, BufRead},
    path::PathBuf,
};

#[derive(Parser)]
struct Args {
    /// Individual intervals produced by two_sphere_step_cost --samples.
    samples: PathBuf,
    /// Require a single committed cut and zero pending cuts for each interval.
    #[arg(long)]
    atomic: bool,
}
#[derive(Deserialize)]
struct Event {
    role: String,
    command: String,
    tick: u64,
    slot: Option<u8>,
    elapsed_ns: u64,
    checkpoint_ns: u64,
    native_step_ns: u64,
    receipt_encode_ns: u64,
    request_encode_ns: u64,
    receipt_decode_ns: u64,
    archive_encode_ns: u64,
    file_sync_ns: u64,
    dir_sync_ns: u64,
}
struct Interval {
    owners: [u8; 3],
    times: BTreeMap<String, u64>,
    rpc: BTreeMap<(String, u8), usize>,
    worker: BTreeMap<(String, u8), usize>,
    publications: BTreeMap<String, usize>,
}
impl Interval {
    fn add(&mut self, name: &str, ns: u64) {
        *self.times.entry(name.to_owned()).or_default() += ns;
    }
    fn event(&mut self, e: Event) -> Result<(), Box<dyn Error>> {
        if e.role == "disk" {
            *self.publications.entry(e.command).or_default() += 1;
            self.add("publish", e.elapsed_ns);
            self.add("archive_encode_and_hash", e.archive_encode_ns);
            self.add("file_sync", e.file_sync_ns);
            self.add("directory_sync", e.dir_sync_ns);
            self.add(
                "other_publish",
                e.elapsed_ns
                    .checked_sub(e.archive_encode_ns + e.file_sync_ns + e.dir_sync_ns)
                    .ok_or("overlapping disk spans")?,
            );
            return Ok(());
        }
        let slot = e.slot.ok_or("missing worker slot")?;
        let key = (e.command.clone(), slot);
        match e.role.as_str() {
            "rpc" | "submit" => {
                if e.role == "rpc" {
                    *self.rpc.entry(key).or_default() += 1;
                }
                self.add("rpc", e.elapsed_ns);
                let actor = if slot == self.owners[0] {
                    "hinge"
                } else if slot == self.owners[1] {
                    "pair"
                } else {
                    return Err("unexpected worker in a steady pair interval".into());
                };
                self.add(&format!("rpc_{actor}_{}", e.command), e.elapsed_ns);
                self.add("controller_request_encode_and_hash", e.request_encode_ns);
                self.add("controller_receipt_decode", e.receipt_decode_ns);
            }
            "worker" => {
                *self.worker.entry(key).or_default() += 1;
                self.add("worker_including_reply_write", e.elapsed_ns);
                self.add("native_step", e.native_step_ns);
                self.add("native_checkpoint", e.checkpoint_ns);
                self.add("worker_receipt_encode", e.receipt_encode_ns);
            }
            _ => return Err("unknown timing role".into()),
        }
        Ok(())
    }
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let expected_publications = if args.atomic { 1 } else { 2 };
    let samples: Vec<(u64, [u8; 3], [u8; 3], u64)> =
        serde_json::from_reader(File::open(args.samples)?)?;
    let mut intervals = BTreeMap::new();
    for (tick, before, after, elapsed) in samples {
        if before == after && before[1] == before[2] && before[0] != before[1] {
            intervals.insert(
                tick,
                Interval {
                    owners: before,
                    times: BTreeMap::from([("wall".into(), elapsed)]),
                    rpc: BTreeMap::new(),
                    worker: BTreeMap::new(),
                    publications: BTreeMap::new(),
                },
            );
        }
    }
    if intervals.is_empty() {
        return Err("no steady independent-pair ticks".into());
    }
    for line in io::stdin().lock().lines() {
        let event: Event = serde_json::from_str(&line?)?;
        // RPCs and pending cuts carry the input tick. The advancing cut
        // publishes after incrementing the clock and carries the output tick.
        let tick = if event.role == "disk" && event.command == "committed" {
            event.tick
        } else {
            event.tick.checked_add(1).ok_or("trace clock overflow")?
        };
        if let Some(interval) = intervals.get_mut(&tick) {
            interval.event(event)?;
        }
    }
    let mut calls = BTreeMap::new();
    for (&tick, row) in &intervals {
        if row.rpc.is_empty()
            || row.rpc != row.worker
            || row.publications.get("pending").copied().unwrap_or(0) != usize::from(!args.atomic)
            || row.publications.get("committed") != Some(&1)
            || row.publications.len() != expected_publications
        {
            return Err(
                format!("incomplete call/receipt or publication evidence at tick {tick}").into(),
            );
        }
        for ((cmd, _), n) in &row.rpc {
            *calls.entry(cmd).or_insert(0) += n;
        }
    }
    let mut by_wall: Vec<_> = intervals.values().collect();
    by_wall.sort_by_key(|row| std::cmp::Reverse(row.times["wall"]));
    let tail_rows = by_wall.len().div_ceil(20);
    let mut stats = BTreeMap::<_, Value>::new();
    let keys: BTreeSet<_> = by_wall.iter().flat_map(|row| row.times.keys()).collect();
    for key in keys {
        let mut ns = by_wall
            .iter()
            .map(|row| row.times.get(key).copied().unwrap_or(0))
            .collect::<Vec<_>>();
        let tail_mean = ns[..tail_rows].iter().sum::<u64>() / tail_rows as u64;
        ns.sort_unstable();
        let percentile = |p: usize| ns[(ns.len() * p).div_ceil(100) - 1];
        stats.insert(
            key,
            json!({"p50":percentile(50),"p95":percentile(95),"slowest_5pct_wall_mean":tail_mean}),
        );
    }
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "stable_ticks":intervals.keys().collect::<Vec<_>>(),"total_worker_calls":calls,
            "total_publications":intervals.len()*expected_publications,"tail_intervals":tail_rows,"nanoseconds":stats
        }))?
    );
    Ok(())
}
