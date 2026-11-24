// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure durable save, native compaction, validated open and process restoration separately.
#[path = "support/changing_oracle.rs"]
mod control;
#[allow(dead_code)]
#[path = "support/changing_worlds.rs"]
mod fixture;
use clap::{Parser, Subcommand};
use control::Oracle;
use gz_physics_distributed::{
    physics::{GroundPlane, SimulationStep},
    pool::{PoolNativeRecovery, WorkerPool},
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    process::Command,
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    action: Action,
}
#[derive(Subcommand)]
enum Action {
    Run {
        #[arg(long)]
        worker: PathBuf,
        #[arg(long, value_enum)]
        layout: fixture::Layout,
        #[arg(long,value_parser=clap::value_parser!(u8).range(3..=8))]
        width: u8,
        #[arg(long)]
        scratch: PathBuf,
    },
    Compare {
        #[arg(long)]
        prior: PathBuf,
        #[arg(long)]
        revised: PathBuf,
        #[arg(long)]
        prior_worker: PathBuf,
        #[arg(long)]
        revised_worker: PathBuf,
        #[arg(long, value_enum)]
        layout: fixture::Layout,
        #[arg(long,value_parser=clap::value_parser!(u8).range(3..=8))]
        width: u8,
        #[arg(long,default_value_t=24,value_parser=clap::value_parser!(u16).range(1..))]
        rounds: u16,
        out: PathBuf,
    },
}
#[derive(Serialize, Deserialize)]
struct Sample {
    save_us: f64,
    compact_us: f64,
    open_us: f64,
    resume_us: f64,
    request_bytes: u64,
    reply_bytes: u64,
    requests: u64,
    archive_bytes: usize,
    archive_sha256: String,
    pair_receipts: u64,
    floor_receipts: u64,
    trajectory_sha256: String,
}
fn us(at: Instant) -> f64 {
    at.elapsed().as_secs_f64() * 1e6
}
fn run(worker: &Path, layout: fixture::Layout, width: u8, scratch: &Path) -> Result<Sample> {
    let oracle = Oracle::new(layout)?;
    let mut seeds = vec![vec![]; usize::from(width)];
    let mut owners = BTreeMap::new();
    for (index, g) in oracle.groups.iter().enumerate() {
        let who = index % usize::from(width);
        seeds[who].extend_from_slice(g);
        owners.extend(g.iter().map(|b| (b.id, (who as u8, 0))));
    }
    let mut pool = WorkerPool::spawn(
        worker,
        0x62617365,
        seeds,
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        256,
    )?;
    let dir = tempfile::tempdir_in(scratch)?;
    let archive = dir.path().join("warm.gzpool");
    let group = oracle
        .groups
        .iter()
        .find(|g| g.len() == 25)
        .ok_or("missing cargo pad")?;
    for t in 1..=75 {
        pool.advance_islands_in_field(fixture::GRAVITY, &fixture::wrenches(t, &oracle.groups))?;
        if t == 70 {
            let dest = (owners[&group[0].id].0 + 1) % width;
            pool.transfer_component(group[0].id, dest)?;
            for b in group {
                owners.insert(b.id, (dest, 1));
            }
        }
        oracle.verify(pool.frame(), &owners)?;
    }
    let traffic = |p: &WorkerPool| {
        p.traffic().iter().fold([0u64; 3], |[n, a, b], c| {
            [n + c.requests, a + c.request_bytes, b + c.reply_bytes]
        })
    };
    let before = traffic(&pool);
    let at = Instant::now();
    pool.save_native_recovery(&archive)?;
    let save_us = us(at);
    let after = traffic(&pool);
    let data = fs::read(&archive)?;
    let archive_sha256 = format!("{:x}", Sha256::digest(&data));
    let at = Instant::now();
    pool.compact()?;
    let compact_us = us(at);
    let frame = pool.frame().clone();
    drop(pool);
    let at = Instant::now();
    let plan = PoolNativeRecovery::open(&archive)?;
    let open_us = us(at);
    let at = Instant::now();
    let mut restored = plan.resume(worker)?;
    let resume_us = us(at);
    if restored.frame() != &frame {
        return Err("restored checkpoint changed the native frame".into());
    }
    for t in 76..=fixture::END {
        restored
            .advance_islands_in_field(fixture::GRAVITY, &fixture::wrenches(t, &oracle.groups))?;
        oracle.verify(restored.frame(), &owners)?;
    }
    Ok(Sample {
        save_us,
        compact_us,
        open_us,
        resume_us,
        request_bytes: after[1] - before[1],
        reply_bytes: after[2] - before[2],
        requests: after[0] - before[0],
        archive_bytes: data.len(),
        archive_sha256,
        trajectory_sha256: oracle.digest,
        pair_receipts: oracle.pairs,
        floor_receipts: oracle.floors,
    })
}
fn main() -> Result<()> {
    match Args::parse().action {
        Action::Run {
            worker,
            layout,
            width,
            scratch,
        } => {
            println!(
                "{}",
                serde_json::to_string(&run(&worker, layout, width, &scratch)?)?
            );
        }
        Action::Compare {
            prior,
            revised,
            prior_worker,
            revised_worker,
            layout,
            width,
            rounds,
            out,
        } => {
            fs::create_dir(&out)?;
            let mut pairs = BufWriter::new(File::create_new(out.join("pairs.jsonl"))?);
            let mut times: [[Vec<f64>; 2]; 4] = Default::default();
            let mut archived = None;
            let layout_arg = match layout {
                fixture::Layout::Adjacent => "adjacent",
                fixture::Layout::Separated => "separated",
            };
            for r in 0..rounds {
                let mut trials = [None, None];
                for i in [usize::from(r % 2), 1 - usize::from(r % 2)] {
                    let output = Command::new([&prior, &revised][i])
                        .arg("run")
                        .arg("--worker")
                        .arg([&prior_worker, &revised_worker][i])
                        .args([
                            "--layout",
                            layout_arg,
                            "--width",
                            &width.to_string(),
                            "--scratch",
                        ])
                        .arg(&out)
                        .output()?;
                    if !output.status.success() {
                        return Err(format!(
                            "arm {i}: {}",
                            String::from_utf8_lossy(&output.stderr)
                        )
                        .into());
                    }
                    let t: Sample = serde_json::from_slice(&output.stdout)?;
                    let proof = (
                        t.archive_bytes,
                        t.archive_sha256.clone(),
                        t.trajectory_sha256.clone(),
                        t.pair_receipts,
                        t.floor_receipts,
                    );
                    if archived.get_or_insert_with(|| proof.clone()) != &proof {
                        return Err("archive or native trajectory changed".into());
                    }
                    for (j, v) in [t.save_us, t.compact_us, t.open_us, t.resume_us]
                        .into_iter()
                        .enumerate()
                    {
                        times[j][i].push(v);
                    }
                    trials[i] = Some(t);
                }
                writeln!(pairs, "{}", serde_json::json!({"round":r,"trial":trials}))?;
                pairs.flush()?;
            }
            let mut timing = BTreeMap::new();
            let median = |x: &mut [f64]| {
                x.sort_by(f64::total_cmp);
                (x[x.len() / 2] + x[(x.len() - 1) / 2]) / 2.
            };
            for (name, mut arms) in ["save", "compact", "open", "resume"].into_iter().zip(times) {
                let mut changes = arms[0]
                    .iter()
                    .zip(&arms[1])
                    .map(|(a, b)| 100. * (b / a - 1.))
                    .collect::<Vec<_>>();
                let med = arms.each_mut().map(|v| median(v));
                let p95 = arms.each_ref().map(|v| v[(v.len() * 95).div_ceil(100) - 1]);
                timing.insert(name,serde_json::json!({"median_us":med,"p95_us":p95,"median_paired_change_pct":median(&mut changes)}));
            }
            let (bytes, archive, trajectory, pairs, floors) = archived.ok_or("no trials")?;
            fs::write(
                out.join("summary.json"),
                serde_json::to_vec_pretty(
                    &serde_json::json!({"layout":layout,"width":width,"pairs":rounds,"archive_bytes":bytes,"archive_sha256":archive,"trajectory_sha256":trajectory,"pair_receipts":pairs,"floor_receipts":floors,"timings":timing}),
                )?,
            )?;
        }
    }
    Ok(())
}
