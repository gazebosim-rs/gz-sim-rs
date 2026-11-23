// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Common process-loss, checkpoint and archive checks for the two-sphere examples.
use super::{MultiSphereObservation, MultiSpherePool, Result};
use clap::Parser;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};

const ARCHIVE: &str = "barrier.gzmultisphere";
#[derive(Parser)]
pub struct Args {
    #[arg(long)]
    pub(super) worker: PathBuf,
    #[arg(long, default_value_t=4, value_parser=clap::value_parser!(u8).range(3..=8))]
    pub(super) workers: u8,
    /// Use -0.010 N·m on each interval.
    #[arg(long)]
    pub(super) reverse: bool,
    /// Directory in which to synchronize the live ledger. Defaults to OUTPUT/ledger.
    #[arg(long)]
    pub(super) ledger_directory: Option<PathBuf>,
    /// Write just the summary and failure diagnostics.
    #[arg(long)]
    pub(super) summary_only: bool,
    /// New output directory.
    pub(super) output: PathBuf,
}

#[cfg(target_os = "linux")]
pub(super) fn kill(pid: u32) -> Result<()> {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    let pid = Pid::from_raw(pid.try_into()?).ok_or("invalid worker PID")?;
    Ok(pidfd_send_signal(
        &pidfd_open(pid, PidfdFlags::empty())?,
        Signal::KILL,
    )?)
}
#[cfg(not(target_os = "linux"))]
pub(super) fn kill(_: u32) -> Result<()> {
    Err("worker loss qualification requires Linux pidfds".into())
}

pub(super) fn pids(world: &MultiSpherePool) -> Result<Vec<u32>> {
    (0..world.width())
        .map(|slot| {
            world
                .worker_pid(slot)
                .ok_or_else(|| "missing live worker PID".into())
        })
        .collect()
}
pub(super) fn save(output: &Path, value: &Value) -> Result<()> {
    let mut file = BufWriter::new(File::create(output)?);
    serde_json::to_writer_pretty(&mut file, value)?;
    file.write_all(b"\n")?;
    Ok(file.flush()?)
}
pub(super) fn cut(args: &Args, ledger: &Path, tick: u64) -> Result<()> {
    if !args.summary_only {
        let dir = args.output.join("cuts").join(format!("tick-{tick}"));
        fs::create_dir_all(&dir)?;
        fs::copy(ledger.join(ARCHIVE), dir.join(ARCHIVE))?;
    }
    Ok(())
}
pub(super) fn reopen(
    world: MultiSpherePool,
    worker: &Path,
    ledger: &Path,
) -> Result<(MultiSpherePool, Value)> {
    reopen_with_transport(
        world,
        worker,
        ledger,
        gz_physics_distributed::moving::WorkerTransport::Pipe,
    )
}
pub(super) fn reopen_with_transport(
    world: MultiSpherePool,
    worker: &Path,
    ledger: &Path,
    transport: gz_physics_distributed::moving::WorkerTransport,
) -> Result<(MultiSpherePool, Value)> {
    let previous = pids(&world)?;
    let owners = world.owners();
    let obs = *world.observation();
    let pending = world.pending_joint_effort();
    let bytes = fs::read(ledger.join(ARCHIVE))?;
    let image = world.image().map(<[u8]>::to_vec);
    let pair = world.pair_image().map(<[u8]>::to_vec);
    drop(world);
    let world = MultiSpherePool::resume_with_transport(worker, ledger, transport)?;
    let current = pids(&world)?;
    if current.iter().any(|pid| previous.contains(pid))
        || world.owners() != owners
        || world.pending_joint_effort() != pending
        || bincode::serialize(world.observation())? != bincode::serialize(&obs)?
        || world.image() != image.as_deref()
        || world.pair_image() != pair.as_deref()
        || fs::read(ledger.join(ARCHIVE))? != bytes
    {
        return Err("fresh pool, durable receipt, native image, state or pending input differs after reopen".into());
    }
    Ok((
        world,
        json!({"before":previous,"after":current,"owners":owners,"pending_torque_nm":pending,
        "pair_bytes":pair.as_ref().map(Vec::len),"native_bytes":image.as_ref().map(Vec::len),"archive_bytes":bytes.len(),"archive_sha256":format!("{:x}",Sha256::digest(bytes))}),
    ))
}
pub(super) fn validate_owners(state: &MultiSphereObservation, owners: [u8; 3]) -> Result<()> {
    for (i, edges) in state.reactions.iter().enumerate() {
        if owners[0] != owners[i + 1]
            && edges.iter().any(|reaction| *reaction != Default::default())
        {
            return Err(format!("hinge contact with independently owned sphere {i}").into());
        }
    }
    if owners[1] != owners[2] && state.sphere_reaction != Default::default() {
        return Err("sphere–sphere contact on separate owners".into());
    }
    Ok(())
}
