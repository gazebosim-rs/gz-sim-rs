// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Long-lived coordinator with recurrent impacts, native process loss and ledger recovery.
#[path = "support/authored_hinge.rs"]
mod authored_hinge;
#[path = "support/published_pair.rs"]
mod published_pair;
#[path = "two_sphere_hinge/reference.rs"]
mod reference;
#[path = "support/two_sphere_run.rs"]
#[allow(dead_code)]
mod run;
use authored_hinge::AuthoredHinge;
use clap::Parser;
use gz_physics_distributed::{
    moving::WorkerTransport,
    multi_sphere::{MultiSphereObservation, MultiSpherePool, MultiSphereSpec, read_published_cut},
};
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    io::Write,
    path::{Path, PathBuf},
    time::{Instant, SystemTime, UNIX_EPOCH},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WORLD: &str = include_str!("../../../examples/distributed_sphere_pair_island.sdf");
const TICKS: u64 = 1220;

#[derive(Parser)]
pub(crate) struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    ledger_root: PathBuf,
    /// Worker count. Defaults to four on creation or the saved width on recovery.
    #[arg(long,value_parser=clap::value_parser!(u8).range(3..=8))]
    workers: Option<u8>,
    /// Copy and finish this convoy checkpoint before starting the requested full epochs.
    #[arg(long)]
    from_cut: Option<PathBuf>,
    #[arg(long)]
    zenoh: bool,
    /// Number of complete fault-injection epochs, excluding any inherited partial epoch.
    #[arg(long,value_parser=clap::value_parser!(u64).range(1..))]
    epochs: u64,
    /// Finish the current epoch when the UTC Unix time reaches this deadline.
    #[arg(long)]
    until: Option<u64>,
    output: PathBuf,
}

fn drive(reverse: bool) -> f64 {
    if reverse { -0.01 } else { 0.01 }
}
fn oracle(spec: &MultiSphereSpec, reverse: bool) -> Result<Vec<MultiSphereObservation>> {
    let mut sim = AuthoredHinge::load(WORLD, "convoy.sdf")?;
    let mut states = vec![reference::observe(&sim, spec, 0)?];
    for tick in 1..=TICKS {
        sim.queue(spec.hinge.joint_id, drive(reverse))?;
        sim.advance()?;
        states.push(reference::observe(&sim, spec, tick)?);
    }
    Ok(states)
}
#[derive(Default, Serialize)]
struct Resources {
    rss_kib: u64,
    descriptors: u64,
    threads: u64,
}
impl Resources {
    fn process(pid: u32) -> Result<Self> {
        let proc = PathBuf::from(format!("/proc/{pid}"));
        let status = fs::read_to_string(proc.join("status"))?;
        let rss_kib = status
            .lines()
            .find_map(|l| l.strip_prefix("VmRSS:"))
            .and_then(|v| v.split_whitespace().next())
            .ok_or("missing VmRSS")?
            .parse()?;
        Ok(Self {
            rss_kib,
            descriptors: fs::read_dir(proc.join("fd"))?.count() as u64,
            threads: fs::read_dir(proc.join("task"))?.count() as u64,
        })
    }
    fn include(&mut self, r: Self) {
        self.rss_kib += r.rss_kib;
        self.descriptors += r.descriptors;
        self.threads += r.threads;
    }
}
fn now() -> Result<u64> {
    Ok(SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs())
}

fn publish_report(dir: &Path, name: &str, value: &serde_json::Value) -> Result<()> {
    let mut file = tempfile::NamedTempFile::new_in(dir)?;
    serde_json::to_writer_pretty(file.as_file_mut(), value)?;
    file.as_file_mut().write_all(b"\n")?;
    file.persist_noclobber(dir.join(name))?;
    Ok(())
}

#[allow(dead_code)] // This example is also exercised by the integration test.
fn main() -> Result<()> {
    exercise(Args::parse())
}

pub(crate) fn exercise(a: Args) -> Result<()> {
    let spec = distributed_multi_sphere_spec(WORLD, "convoy.sdf", "default")?;
    let states = [oracle(&spec, false)?, oracle(&spec, true)?];
    fs::create_dir(&a.output)?;
    let mut log = fs::File::create_new(a.output.join("epochs.jsonl"))?;
    let ledger = tempfile::tempdir_in(&a.ledger_root)?;
    let j = spec.hinge.joint_id;
    let mut transport = if a.zenoh {
        WorkerTransport::LoopbackZenoh
    } else {
        WorkerTransport::Pipe
    };
    let archive = ledger.path().join("barrier.gzmultisphere");
    let (mut pool, source_digest) = if let Some(source) = &a.from_cut {
        let (cut, bytes) = published_pair::snapshot(source)?;
        if cut.spec != spec || a.workers.is_some_and(|width| width != cut.width) {
            return Err("source convoy recipe or worker width differs".into());
        }
        fs::write(&archive, &bytes)?;
        let pool = MultiSpherePool::open_scene_with_transport(
            &a.worker,
            ledger.path(),
            a.workers,
            spec,
            transport,
        )?;
        (pool, Some(format!("{:x}", Sha256::digest(&bytes))))
    } else {
        (
            MultiSpherePool::create_with_transport(
                &a.worker,
                ledger.path(),
                a.workers.unwrap_or(4),
                spec,
                transport,
            )?,
            None,
        )
    };
    let width = pool.width();
    let source_epoch = pool.epoch();
    let source_tick = pool.observation().hinge.tick;
    let source_effort = pool.pending_joint_effort();
    let first_full_epoch = source_epoch
        .checked_add(u64::from(source_digest.is_some()))
        .ok_or("source epoch overflow")?;
    let end_epoch = first_full_epoch
        .checked_add(a.epochs)
        .ok_or("end epoch overflow")?;
    end_epoch
        .checked_mul(TICKS)
        .ok_or("campaign clock overflow")?;
    if source_tick > TICKS
        || source_effort.is_some_and(|e| {
            !matches!(source_tick, 149 | 1213) || e != drive(source_epoch % 2 != 0) * 0.5
        })
    {
        return Err("checkpoint clock or pending input differs from the convoy drive".into());
    }
    let mut complete_epochs = 0_u64;
    let mut last_clock = (source_epoch, source_tick);
    let start = Instant::now();
    let start_unix = now()?;
    let (mut killed, mut reopened, mut transferred, mut steps, mut withheld) =
        (0u64, 0u64, 0u64, 0u64, 0u64);
    let mut attempted = ("start", 0, 0);
    let exercise = (|| -> Result<()> {
        for epoch in source_epoch..end_epoch {
            if epoch >= first_full_epoch
                && a.until.is_some_and(|end| {
                    SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .is_ok_and(|t| t.as_secs() >= end)
                })
            {
                break;
            }
            let inherited = epoch < first_full_epoch;
            let begin = if inherited { source_tick } else { 0 };
            if epoch != source_epoch {
                attempted = ("reset", epoch, 0);
                pool.reset()?;
            }
            let reverse = epoch % 2 != 0;
            let reference = &states[usize::from(reverse)];
            let mut moved = false;
            let mut digest = Sha256::new();
            let mut peak_impulse: f64 = 0.;
            let mut loaded = 0_u64;
            for tick in begin..=TICKS {
                if tick > begin {
                    if !inherited && tick == 149 + (epoch % 5) {
                        attempted = ("kill", epoch, tick);
                        let owner = pool.owners()[(epoch % 3) as usize];
                        run::kill(pool.worker_pid(owner).ok_or("missing native process")?)?;
                        killed += 1;
                    }
                    if !inherited && tick == 151 + (epoch % 7) {
                        attempted = ("withhold", epoch, tick);
                        pool.drop_next_acknowledgement(pool.owners()[0])?;
                        withheld += 1;
                    }
                    let held = tick == 150 || tick == 1214;
                    if held && pool.pending_joint_effort().is_none() {
                        attempted = ("queue effort", epoch, tick);
                        pool.queue_effort(j, drive(reverse) * 0.5)?;
                    }
                    attempted = ("advance", epoch, tick);
                    pool.advance_with_effort_at(
                        epoch,
                        tick - 1,
                        j,
                        if held {
                            drive(reverse) * 0.5
                        } else {
                            drive(reverse)
                        },
                    )?;
                    steps += 1;
                }
                attempted = ("compare", epoch, tick);
                let expected = &reference[tick as usize];
                let obs = pool.observation();
                let bytes = bincode::serialize(obs)?;
                let pending = if inherited && tick == begin {
                    source_effort
                } else {
                    None
                };
                if bytes != bincode::serialize(expected)?
                    || pool.pending_joint_effort() != pending
                    || pool.epoch() != epoch
                {
                    return Err(format!("wrong state, input or epoch at {epoch}/{tick}").into());
                }
                digest.update(bytes);
                run::validate_owners(obs, pool.owners())?;
                let impulse = reference::momentum_residual(
                    &spec,
                    &reference[tick.saturating_sub(1) as usize],
                    obs,
                );
                if !impulse.is_finite() || impulse > 1e-8 {
                    return Err(format!("impulse residual {impulse} at {epoch}/{tick}").into());
                }
                peak_impulse = peak_impulse.max(impulse);
                let [h, x, y] = pool.owners();
                let pair_loaded = obs.sphere_reaction.normal_impulse > 0.;
                loaded += u64::from(pair_loaded);
                if !inherited && pair_loaded && h != x && x == y && !moved {
                    attempted = ("transfer pair", epoch, tick);
                    pool.transfer_pair(h)?;
                    moved = true;
                    transferred += 1;
                }
                if !inherited && epoch % 9 == 4 && tick == 151 + (epoch % 3) {
                    transport = match transport {
                        WorkerTransport::Pipe => WorkerTransport::LoopbackZenoh,
                        WorkerTransport::LoopbackZenoh => WorkerTransport::Pipe,
                    };
                    // Verify identically re-encoded native receipts and fresh worker identities on both transports.
                    attempted = ("reopen", epoch, tick);
                    let new =
                        run::reopen_with_transport(pool, &a.worker, ledger.path(), transport)?;
                    pool = new.0;
                    reopened += 1;
                }
            }
            last_clock = (epoch, TICKS);
            if inherited {
                // An archive records physical state and pending input. It cannot tell
                // whether a previous controller had already injected a process fault.
                attempted = ("record continuation", epoch, TICKS);
                publish_report(
                    &a.output,
                    "continuation.json",
                    &serde_json::json!({
                        "source_sha256": source_digest,
                        "source_epoch": epoch,
                        "source_tick": begin,
                        "source_pending_effort": source_effort,
                        "final_tick": TICKS,
                        "verified_advances": TICKS - begin,
                        "workers": width,
                        "transport": format!("{transport:?}"),
                        "physical_digest": format!("{:x}", digest.finalize()),
                        "max_sphere_impulse_residual_ns": peak_impulse
                    }),
                )?;
                continue;
            }
            if loaded == 0 || !moved {
                return Err("epoch missed its loaded pair transfer".into());
            }
            complete_epochs += 1;
            attempted = ("record epoch", epoch, TICKS);
            let mut workers = Resources::default();
            for pid in run::pids(&pool)? {
                workers.include(Resources::process(pid)?);
            }
            serde_json::to_writer(
                &mut log,
                &serde_json::json!({"epoch":epoch,"reverse":reverse,"elapsed_seconds":start.elapsed().as_secs_f64(),"unix":now()?,"workers":width,"transport":format!("{transport:?}"),"cumulative_steps":steps,"cumulative_kills":killed,"cumulative_reopens":reopened,"cumulative_loaded_transfers":transferred,"cumulative_withheld":withheld,"pair_impulse_intervals":loaded,"max_sphere_impulse_residual_ns":peak_impulse,"controller":Resources::process(std::process::id())?,"worker_totals":workers,"physical_digest":format!("{:x}",digest.finalize())}),
            )?;
            log.write_all(b"\n")?;
            log.flush()?;
        }
        Ok(())
    })();
    if let Err(error) = exercise {
        fs::copy(
            ledger.path().join("barrier.gzmultisphere"),
            a.output.join("failed.gzmultisphere"),
        )?;
        fs::write(a.output.join("failure.txt"), error.to_string())?;
        let saved = read_published_cut(fs::File::open(a.output.join("failed.gzmultisphere"))?)?;
        let (operation, epoch, target_tick) = attempted;
        fs::write(
            a.output.join("failure.json"),
            serde_json::to_vec_pretty(&serde_json::json!({
                "operation":operation, "target_epoch":epoch, "target_tick":target_tick,
                "transport":format!("{transport:?}"), "committed_epoch":saved.epoch,
                "committed_tick":saved.observation.hinge.tick, "owners":saved.owners,
                "pending_effort":saved.pending_effort, "error":error.to_string()
            }))?,
        )?;
        return Err(error);
    }
    publish_report(
        &a.output,
        "summary.json",
        &serde_json::json!({
            "start_unix": start_unix,
            "end_unix": now()?,
            "elapsed_seconds": start.elapsed().as_secs_f64(),
            "steps": steps,
            "epochs": complete_epochs,
            "source_epoch": source_epoch,
            "source_tick": source_tick,
            "source_sha256": source_digest,
            "final_epoch": last_clock.0,
            "final_tick": last_clock.1,
            "schedule_tick": last_clock.0 * TICKS + last_clock.1,
            "worker_kills": killed,
            "reopens": reopened,
            "withheld_receipts": withheld,
            "loaded_transfers": transferred
        }),
    )
}
