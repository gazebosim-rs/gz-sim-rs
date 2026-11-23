// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure exactly the bytes and time spent confirming partial step barriers.
#[path = "support/changing_oracle.rs"]
mod control;
#[allow(dead_code)]
#[path = "support/changing_worlds.rs"]
mod fixture;
use clap::{Parser, Subcommand};
use control::Oracle;
use gz_physics_distributed::{
    coordinator::Error as PoolError,
    physics::{GroundPlane, SimulationStep},
    pool::WorkerPool,
};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    command: Action,
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
        /// Kill each unconfirmed worker after the initial reply loss.
        #[arg(long)]
        kill_missing: bool,
        #[arg(long)]
        owner_trace: Option<PathBuf>,
    },
    Compare {
        #[arg(long)]
        worker: PathBuf,
        #[arg(long)]
        reference: PathBuf,
        #[arg(long)]
        candidate: PathBuf,
        #[arg(long, value_enum)]
        layout: fixture::Layout,
        #[arg(long,value_parser=clap::value_parser!(u8).range(3..=8))]
        width: u8,
        /// Kill each unconfirmed worker after the initial reply loss.
        #[arg(long)]
        kill_missing: bool,
        #[arg(long,default_value_t=16,value_parser=clap::value_parser!(u32).range(1..))]
        rounds: u32,
        out: PathBuf,
    },
}
#[derive(Serialize, Deserialize)]
struct Confirmation {
    tick: u64,
    missing: Vec<u8>,
    retry_us: f64,
    #[serde(default)]
    replacements: usize,
    requests: u64,
    request_bytes: u64,
    reply_bytes: u64,
}
#[derive(Serialize, Deserialize)]
struct Trial {
    layout: fixture::Layout,
    width: u8,
    confirmations: Vec<Confirmation>,
    ordinary_us: f64,
    digest: String,
    pairs: u64,
    floors: u64,
}
fn traffic(p: &WorkerPool) -> [u64; 3] {
    p.traffic().iter().fold([0; 3], |[n, tx, rx], t| {
        [n + t.requests, tx + t.request_bytes, rx + t.reply_bytes]
    })
}
fn run(
    worker: &Path,
    layout: fixture::Layout,
    width: u8,
    kill_missing: bool,
    trace: Option<&Path>,
) -> Result<Trial> {
    let oracle = Oracle::new(layout)?;
    let mut seeds = vec![vec![]; width as usize];
    let mut owners = BTreeMap::new();
    for (i, g) in oracle.groups.iter().enumerate() {
        let owner = i % usize::from(width);
        seeds[owner].extend_from_slice(g);
        owners.extend(g.iter().map(|b| (b.id, (owner as u8, 0))));
    }
    let second = oracle
        .groups
        .iter()
        .position(|g| g.len() == 25 && g[0].id != oracle.groups[0][0].id)
        .ok_or("missing second cargo pad")? as u8
        % width;
    let mut p = WorkerPool::spawn(
        worker,
        30376,
        seeds,
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        256,
    )?;
    let mut owner_changes = Vec::new();
    let map = |p: &WorkerPool| {
        owners
            .keys()
            .map(|&id| p.frame().body(id).unwrap().0)
            .collect::<Vec<_>>()
    };
    let mut last = map(&p);
    if trace.is_some() {
        owner_changes.push(serde_json::json!({"tick":0,"owners":last}));
    }
    let mut confirmations = vec![];
    let mut ordinary_us = 0.;
    for t in 1..=fixture::END {
        let field = fixture::wrenches(t, &oracle.groups);
        let missing = match t {
            80 => vec![0],
            140 => vec![width - 1],
            160 => vec![second],
            220 => vec![0, 1],
            260 => (0..width - 1).collect(),
            300 => (0..width).collect(),
            _ => vec![],
        };
        if missing.is_empty() {
            let start = Instant::now();
            p.advance_islands_in_field(fixture::GRAVITY, &field)?;
            if t > fixture::SETTLED {
                ordinary_us += start.elapsed().as_secs_f64() * 1e6;
            }
        } else {
            let pids = |p: &WorkerPool| (0..width).map(|w| p.worker_pid(w)).collect::<Vec<_>>();
            let before_pids = pids(&p);
            let frame = p.frame().clone();
            for &w in &missing {
                p.drop_next_acknowledgement(w)?;
            }
            if !matches!(
                p.advance_islands_in_field(fixture::GRAVITY, &field),
                Err(PoolError::Transport(_))
            ) || *p.frame() != frame
            {
                return Err("missing replies changed committed barrier".into());
            }
            if kill_missing {
                for &w in &missing {
                    p.terminate_worker(w)?;
                }
            }
            let before = traffic(&p);
            let start = Instant::now();
            p.advance_islands_in_field(fixture::GRAVITY, &field)?;
            let retry_us = start.elapsed().as_secs_f64() * 1e6;
            let after = traffic(&p);
            let now = pids(&p);
            let replacements = now.iter().zip(&before_pids).filter(|(a, b)| a != b).count();
            if (!kill_missing && replacements != 0)
                || (kill_missing
                    && missing
                        .iter()
                        .any(|&w| before_pids[w as usize] == now[w as usize]))
            {
                return Err("unexpected process replacement".into());
            }
            confirmations.push(Confirmation {
                replacements,
                tick: t,
                missing,
                retry_us,
                requests: after[0] - before[0],
                request_bytes: after[1] - before[1],
                reply_bytes: after[2] - before[2],
            });
        }
        oracle.verify(p.frame(), &owners)?;
        if trace.is_some() {
            let current = map(&p);
            if current != last {
                owner_changes.push(serde_json::json!({"tick":t,"owners":current}));
                last = current;
            }
        }
    }
    if let Some(path) = trace {
        serde_json::to_writer_pretty(
            File::create_new(path)?,
            &serde_json::json!({"layout":layout,"width":width,
            "first_body_id":1,"last_tick":fixture::END,"trajectory_sha256":oracle.digest,"owner_changes":owner_changes}),
        )?;
    }
    Ok(Trial {
        layout,
        width,
        confirmations,
        ordinary_us,
        digest: oracle.digest,
        pairs: oracle.pairs,
        floors: oracle.floors,
    })
}
fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    (v[v.len() / 2] + v[(v.len() - 1) / 2]) / 2.
}
fn main() -> Result<()> {
    match Args::parse().command {
        Action::Run {
            worker,
            layout,
            width,
            kill_missing,
            owner_trace,
        } => println!(
            "{}",
            serde_json::to_string(&run(
                &worker,
                layout,
                width,
                kill_missing,
                owner_trace.as_deref()
            )?)?
        ),
        Action::Compare {
            worker,
            reference,
            candidate,
            layout,
            width,
            rounds,
            kill_missing,
            out,
        } => {
            fs::create_dir(&out)?;
            let mut log = BufWriter::new(File::create_new(out.join("pairs.jsonl"))?);
            let read = |bin: &Path| -> Result<Trial> {
                let mut command = std::process::Command::new(bin);
                command.arg("run");
                if kill_missing {
                    command.arg("--kill-missing");
                }
                let output = command
                    .arg("--worker")
                    .arg(&worker)
                    .args([
                        "--width",
                        &width.to_string(),
                        "--layout",
                        match layout {
                            fixture::Layout::Adjacent => "adjacent",
                            fixture::Layout::Separated => "separated",
                        },
                    ])
                    .output()?;
                if !output.status.success() {
                    return Err(format!(
                        "{}: {}",
                        bin.display(),
                        String::from_utf8_lossy(&output.stderr)
                    )
                    .into());
                }
                Ok(serde_json::from_slice(&output.stdout)?)
            };
            let mut all = Vec::new();
            for i in 0..rounds {
                let reference_first = i % 2 == 0;
                let (a, b) = if reference_first {
                    (&reference, &candidate)
                } else {
                    (&candidate, &reference)
                };
                let a = read(a)?;
                let b = read(b)?;
                let (r, c) = if reference_first { (a, b) } else { (b, a) };
                if r.confirmations.len() != 6
                    || c.confirmations.len() != 6
                    || r.digest != c.digest
                    || (r.pairs, r.floors) != (c.pairs, c.floors)
                    || r.confirmations
                        .iter()
                        .zip(&c.confirmations)
                        .any(|(x, y)| (x.tick, &x.missing) != (y.tick, &y.missing))
                {
                    return Err("unmatched retry trial".into());
                }
                writeln!(
                    log,
                    "{}",
                    serde_json::json!({"reference_first":reference_first,"reference":r,"candidate":c})
                )?;
                log.flush()?;
                all.push((r, c));
            }
            let mut by_tick = vec![];
            for i in 0..6 {
                let (r, c) = &all[0];
                let (x, y) = (&r.confirmations[i], &c.confirmations[i]);
                if !kill_missing
                    && all.iter().any(|(a, b)| {
                        a.confirmations[i].request_bytes != x.request_bytes
                            || b.confirmations[i].request_bytes != y.request_bytes
                            || a.confirmations[i].reply_bytes != x.reply_bytes
                            || b.confirmations[i].reply_bytes != y.reply_bytes
                    })
                {
                    return Err("retry wire accounting varies across identical trials".into());
                }
                by_tick.push(serde_json::json!({"tick":x.tick,"missing":x.missing,"replacements":[x.replacements,y.replacements],"requests":[x.requests,y.requests],"request_bytes":[x.request_bytes,y.request_bytes],
                "reply_bytes":[x.reply_bytes,y.reply_bytes],"median_retry_us":[median(all.iter().map(|(a,_)|a.confirmations[i].retry_us).collect()),median(all.iter().map(|(_,b)|b.confirmations[i].retry_us).collect())]}));
            }
            fs::write(
                out.join("result.json"),
                serde_json::to_vec_pretty(
                    &serde_json::json!({"rounds":rounds,"layout":layout,"width":width,"kill_missing":kill_missing,"by_tick":by_tick,
            "median_paired_ordinary_time_change_pct":median(all.iter().map(|(a,b)|100.*(b.ordinary_us/a.ordinary_us-1.)).collect())}),
                )?,
            )?;
        }
    }
    Ok(())
}
