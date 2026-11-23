// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reproducible many-process fault qualification against eight independent native controls.
#[path = "support/changing_oracle.rs"]
mod control;
#[path = "support/deadline_fault.rs"]
mod deadline;
#[path = "support/changing_worlds.rs"]
mod fixture;
#[path = "support/handoff_fault.rs"]
mod migration;
#[path = "support/momentum_audit.rs"]
mod momentum;
use clap::Parser;
use control::Oracle;
use gz_physics_distributed::{
    coordinator::Error as PoolError,
    physics::{GroundPlane, SimulationStep},
    pool::{PoolNativeRecovery, WorkerPool},
};
use serde::Serialize;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::{Duration, Instant},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// Episode number selects the worker count, layout, encodings and repeatable fault schedule.
    #[arg(long, default_value_t = 0)]
    start: u64,
    #[arg(long,default_value_t=48,value_parser=clap::value_parser!(u32).range(1..))]
    episodes: u32,
    #[arg(long,conflicts_with="episodes",value_parser=clap::value_parser!(u32).range(1..))]
    seconds: Option<u32>,
    /// Suspend a real worker through the transport deadline in each episode (Linux).
    #[arg(long)]
    real_deadlines: bool,
    /// Record the complete loaded transfer, starting before its first attempt.
    #[arg(long, value_enum, conflicts_with = "real_deadlines")]
    handoff_study: Option<migration::Policy>,
    /// Suspend source or receiver on alternating moves in the handoff study.
    #[arg(long, requires = "handoff_study")]
    stopped_handoffs: bool,
    #[arg(long,requires="handoff_study",value_parser=clap::value_parser!(u16).range(1..=1000))]
    handoff_grace_ms: Option<u16>,
    /// Delay after the first one-second migration deadline before the peer thaws.
    #[arg(long,requires="stopped_handoffs",value_parser=clap::value_parser!(u16).range(1..=1200))]
    thaw_delay_ms: Option<u16>,
    /// Independently check external impulse and the momentum transported over owner changes.
    #[arg(long)]
    momentum_audit: bool,
    /// Keep per-worker physical terms for the selected complete episode numbers.
    #[arg(long, requires = "momentum_audit", value_delimiter = ',')]
    momentum_trace: Vec<u64>,
    out: PathBuf,
}
#[derive(Serialize)]
struct Move {
    tick: u64,
    source: u8,
    destination: u8,
    members: usize,
    withheld: &'static str,
    member_ids: Vec<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    recovery: Option<migration::Observation>,
}
#[derive(Serialize)]
struct Episode {
    number: u64,
    width: u8,
    layout: fixture::Layout,
    packed_handoffs: bool,
    packed_observations: bool,
    helper_limit: Option<usize>,
    journal_limit: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    deadline: Option<deadline::Observation>,
    #[serde(skip)]
    study: Option<migration::Policy>,
    #[serde(skip)]
    stopped_handoffs: bool,
    #[serde(skip)]
    grace_ms: u16,
    #[serde(skip)]
    thaw_ms: u16,
    initial_owners: Vec<u8>,
    moves: Vec<Move>,
    cache_hits: usize,
    replacements: usize,
    restored_at: u64,
    physics_sha256: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    momentum: Option<momentum::Summary>,
}
fn pids(p: &WorkerPool, width: u8) -> Vec<u32> {
    (0..width).map(|i| p.worker_pid(i).unwrap()).collect()
}
fn config(p: &mut WorkerPool, e: &Episode) -> Result<()> {
    p.set_local_helper_limit(e.helper_limit);
    p.set_packed_observations(e.packed_observations);
    if p.set_packed_handoffs(e.packed_handoffs)
        != if e.packed_handoffs {
            usize::from(e.width)
        } else {
            0
        }
    {
        return Err("worker handoff negotiation mismatch".into());
    }
    Ok(())
}
fn handoff(
    p: &mut WorkerPool,
    owners: &mut BTreeMap<u64, (u8, u64)>,
    e: &mut Episode,
    rng: &mut fastrand::Rng,
    pause_receiver: bool,
) -> Result<()> {
    let tick = p.frame().tick;
    let ids: Vec<_> = owners.keys().copied().collect();
    let member = ids[rng.usize(..ids.len())];
    let source = owners[&member].0;
    let destination = (source + rng.u8(1..e.width)) % e.width;
    let group = p.frame().workers[source as usize]
        .components
        .iter()
        .find(|g| g.ids.binary_search(&member).is_ok())
        .ok_or("missing component")?
        .ids
        .clone();
    let (endpoint, skip, withheld) = [
        (source, 0, "export"),
        (destination, 0, "prepare"),
        (source, 1, "release"),
        (destination, 1, "activate"),
        (source, 2, "observe"),
    ][rng.usize(..5)];
    let suspended = e.stopped_handoffs;
    let receiver = (e.number % 6 + e.moves.len() as u64) % 2 == 1;
    let (endpoint, skip, withheld) = if suspended {
        if receiver {
            (destination, 0, "prepare")
        } else {
            (source, 0, "export")
        }
    } else {
        (endpoint, skip, withheld)
    };
    let recovery = if let Some(policy) = e.study {
        let case = migration::Transfer {
            member,
            destination,
            endpoint,
            skip,
            width: e.width,
            grace_ms: e.grace_ms,
            thaw_ms: e.thaw_ms,
        };
        Some(migration::run(
            p,
            case,
            policy,
            migration::Fault::for_move(e.number, e.moves.len(), suspended),
        )?)
    } else {
        let committed = p.frame().clone();
        if pause_receiver {
            p.compact()?;
        }
        let history = p.retained_entries();
        let before = pids(p, e.width);
        let stopped = if pause_receiver {
            Some(deadline::Stopped::worker(p, destination)?)
        } else {
            p.drop_acknowledgement_after(endpoint, skip)?;
            None
        };
        let at = Instant::now();
        let result = p.transfer_component(member, destination);
        let timeout_us = if pause_receiver {
            deadline::check_timeout(result, at)?
        } else {
            if !matches!(result, Err(PoolError::Transport(_))) {
                return Err("expected a withheld handoff reply".into());
            }
            0.
        };
        if p.frame() != &committed {
            return Err(format!("partial {withheld} handoff published at {tick}").into());
        }
        drop(stopped);
        // A reserve-triggered compaction can retire the old journal while keeping the same state.
        if p.retained_entries() != history && p.retained_entries().iter().any(|&n| n != 0) {
            return Err("uncommitted handoff entered the published journals".into());
        }
        let at = Instant::now();
        p.transfer_component(member, destination)?;
        if pause_receiver {
            let retry_us = at.elapsed().as_secs_f64() * 1e6;
            let after = pids(p, e.width);
            let replacements = after
                .iter()
                .zip(before.iter().copied())
                .filter(|(a, b)| **a != *b)
                .count();
            if replacements != 2 {
                return Err(format!("partial prepare fenced {replacements} workers").into());
            }
            e.deadline = Some(deadline::Observation {
                before_pids: before,
                after_pids: after,
                phase: "prepare_thaw",
                owner: destination,
                tick,
                timeout_us,
                retry_us,
                replacements,
            });
        }
        None
    };
    for id in &group {
        let o = owners.get_mut(id).ok_or("unexpected migrating ID")?;
        o.0 = destination;
        o.1 += 1;
    }
    e.moves.push(Move {
        tick,
        source,
        destination,
        members: group.len(),
        member_ids: group,
        recovery,
        withheld: if pause_receiver {
            "deadline_prepare"
        } else {
            withheld
        },
    });
    Ok(())
}
fn episode(
    number: u64,
    layout: fixture::Layout,
    oracle: &Oracle,
    worker: &Path,
    scratch: &Path,
    args: &Args,
) -> Result<Episode> {
    let real_deadlines = args.real_deadlines;
    let width = 3 + (number % 6) as u8;
    let mut rng = fastrand::Rng::with_seed(number ^ 0x7740_0f85_89a4_be71);
    let mut e = Episode {
        number,
        width,
        layout,
        packed_handoffs: number / 12 % 2 == 1,
        packed_observations: number / 24 % 2 == 1,
        helper_limit: [None, Some(0), Some(1), Some(2)][rng.usize(..4)],
        journal_limit: [16, 32, 128][rng.usize(..3)],
        deadline: None,
        study: args.handoff_study,
        stopped_handoffs: args.stopped_handoffs,
        grace_ms: args.handoff_grace_ms.unwrap_or(150),
        thaw_ms: args.thaw_delay_ms.unwrap_or(100),
        initial_owners: vec![],
        moves: vec![],
        cache_hits: 0,
        replacements: 0,
        restored_at: rng.u64(250..=285),
        physics_sha256: oracle.digest.clone(),
        momentum: None,
    };
    let mut seeds = vec![vec![]; usize::from(width)];
    let mut owners = BTreeMap::new();
    for group in &oracle.groups {
        let owner = rng.u8(..width);
        seeds[owner as usize].extend_from_slice(group);
        owners.extend(group.iter().map(|b| (b.id, (owner, 0))));
    }
    let mut p = WorkerPool::spawn(
        worker,
        number,
        seeds,
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        e.journal_limit,
    )?;
    e.initial_owners = owners
        .keys()
        .map(|&id| p.frame().body(id).expect("seeded body").0)
        .collect();
    config(&mut p, &e)?;
    let trace = args
        .momentum_trace
        .contains(&number)
        .then(|| args.out.join(format!("momentum-{number}.jsonl")));
    let mut audit = args
        .momentum_audit
        .then(|| momentum::Audit::new(p.frame(), trace.as_deref()))
        .transpose()?;
    let moves = [rng.u64(fixture::SETTLED + 4..=139), rng.u64(183..=215)];
    let loses = [rng.u64(23..70), 160, rng.u64(287..=324)];
    let kill = rng.u64(220..=244);
    for t in 1..=fixture::END {
        let loads = fixture::wrenches(t, &oracle.groups);
        let fault_mode = (number / 6 + number % 6) % 3;
        if real_deadlines && t == 160 && fault_mode < 2 {
            let obs = deadline::step(&mut p, width, rng.u8(..width), &loads, fault_mode == 1)?;
            e.cache_hits += usize::from(obs.replacements == 0);
            e.deadline = Some(obs);
        } else {
            let before = loses.contains(&t).then(|| pids(&p, width));
            if before.is_some() {
                let committed = p.frame().clone();
                let target = rng.u8(..width);
                p.drop_next_acknowledgement(target)?;
                if t == 160 {
                    p.drop_next_acknowledgement((target + 1) % width)?;
                }
                if !matches!(
                    p.advance_islands_in_field(fixture::GRAVITY, &loads),
                    Err(PoolError::Transport(_))
                ) || p.frame() != &committed
                {
                    return Err("ambiguous step changed published state".into());
                }
            }
            p.advance_islands_in_field(fixture::GRAVITY, &loads)?;
            if let Some(prior) = before {
                if pids(&p, width) != prior {
                    return Err(format!("live reply cache required fencing at {t}").into());
                }
                e.cache_hits += 1;
            }
        }
        if let Some(audit) = &mut audit {
            audit.advance(p.frame(), fixture::GRAVITY, &loads, &[])?;
        }
        if moves.contains(&t) {
            handoff(
                &mut p,
                &mut owners,
                &mut e,
                &mut rng,
                real_deadlines && t == moves[0] && fault_mode == 2,
            )?;
        }
        if t == kill {
            let victim = rng.u8(..width);
            let pid = p.worker_pid(victim);
            let committed = p.frame().clone();
            p.terminate_worker(victim)?;
            p.recover(victim)?;
            if p.worker_pid(victim) == pid || p.frame() != &committed {
                return Err("inconsistent worker replacement".into());
            }
            e.replacements += 1;
        }
        if t == e.restored_at {
            let tempfile = scratch.join("checkpoint.gzpool");
            let committed = p.frame().clone();
            p.save_native_recovery(&tempfile)?;
            drop(p);
            p = PoolNativeRecovery::open(&tempfile)?.resume(worker)?;
            fs::remove_file(tempfile)?;
            if p.frame() != &committed {
                return Err("disk checkpoint changed state".into());
            }
            config(&mut p, &e)?;
        }
        if let Some(audit) = &mut audit
            && (moves.contains(&t) || t == kill || t == e.restored_at)
        {
            audit.same_tick(p.frame())?;
        }
        oracle.verify(p.frame(), &owners)?;
    }
    e.momentum = audit.map(momentum::Audit::finish).transpose()?;
    Ok(e)
}
fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.out)?;
    let controls = [
        Oracle::new(fixture::Layout::Adjacent)?,
        Oracle::new(fixture::Layout::Separated)?,
    ];
    let oracle_info=controls.each_ref().map(|c|serde_json::json!({"trajectory_sha256":c.digest,"ticks":c.ticks.len(),"pairs":c.pairs,"floors":c.floors}));
    fs::write(
        args.out.join("oracles.json"),
        serde_json::to_vec_pretty(&oracle_info)?,
    )?;
    let mut log = BufWriter::new(File::create_new(args.out.join("episodes.jsonl"))?);
    let scratch = tempfile::tempdir_in(&args.out)?;
    let begin = Instant::now();
    for n in 0..u32::MAX {
        let number = args
            .start
            .checked_add(u64::from(n))
            .ok_or("episode number overflow")?;
        let i = (number / 6 % 2) as usize;
        let layout = [fixture::Layout::Adjacent, fixture::Layout::Separated][i];
        let e = episode(
            number,
            layout,
            &controls[i],
            &args.worker,
            scratch.path(),
            &args,
        )
        .map_err(|e| format!("episode {number}: {e}"))?;
        serde_json::to_writer(&mut log, &e)?;
        writeln!(log)?;
        log.flush()?;
        if args.seconds.map_or(n + 1 >= args.episodes, |s| {
            begin.elapsed() >= Duration::from_secs(u64::from(s))
        }) {
            fs::write(
                args.out.join("result.json"),
                serde_json::to_vec_pretty(
                    &serde_json::json!({"episodes":n+1,"ticks":u64::from(n+1)*fixture::END,"elapsed_s":begin.elapsed().as_secs_f64(),"start":args.start,"last":number}),
                )?,
            )?;
            break;
        }
    }
    Ok(())
}
