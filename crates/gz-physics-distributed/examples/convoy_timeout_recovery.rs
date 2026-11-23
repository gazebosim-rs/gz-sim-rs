// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify bounded recovery against ordinary Bevy physics and a fault-free native pool.
#[path = "support/authored_hinge.rs"]
mod authored_hinge;
#[path = "support/published_pair.rs"]
mod published_pair;
#[path = "two_sphere_hinge/reference.rs"]
#[allow(dead_code)]
mod reference;
use authored_hinge::AuthoredHinge;
use clap::Parser;
use gz_physics_distributed::{
    articulation::ArticulationError,
    moving::WorkerTransport,
    multi_sphere::{MultiSphereObservation, MultiSpherePool, MultiSphereSpec},
};
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::Instant,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WORLD: &str = include_str!("../../../examples/distributed_sphere_pair_island.sdf");
const ARCHIVE: &str = "barrier.gzmultisphere";

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// Optional convoy soak checkpoint. Otherwise create an ordinary three-slot fixture at 148 ms.
    #[arg(long)]
    cut: Option<PathBuf>,
    #[arg(long)]
    zenoh: bool,
    /// Stop the independent sphere-pair owner in place of the hinge owner.
    #[arg(long)]
    stall_pair: bool,
    /// Pause this many replacement processes at bootstrap after stopping the selected owner.
    #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u8).range(0..=2))]
    bootstrap_stalls: u8,
    /// New directory for results and recoverable viewer snapshots.
    output: PathBuf,
}

#[cfg(target_os = "linux")]
fn stop(pid: u32) -> Result<()> {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    let pid = Pid::from_raw(pid.try_into()?).ok_or("invalid PID")?;
    Ok(pidfd_send_signal(
        &pidfd_open(pid, PidfdFlags::empty())?,
        Signal::STOP,
    )?)
}
#[cfg(not(target_os = "linux"))]
fn stop(_: u32) -> Result<()> {
    Err("worker suspension qualification requires Linux".into())
}

// Interpose only before exec. Every responding process is the supplied physics_worker.
#[cfg(unix)]
fn launch(slot: &std::ffi::OsStr, lease: &Path) -> Result<()> {
    use std::os::unix::process::CommandExt;
    let session = lease.parent().ok_or("missing session")?;
    for attempt in 1..=2 {
        let marker = session.join(format!(
            "stall-{}-{attempt}",
            slot.to_str().ok_or("invalid slot")?
        ));
        match fs::rename(&marker, marker.with_extension("fired")) {
            Ok(()) => {
                stop(std::process::id())?;
                break;
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => (),
            Err(e) => return Err(e.into()),
        }
    }
    Err(
        std::process::Command::new(fs::read_link(session.join("worker"))?)
            .arg("--managed-multisphere-v2")
            .arg(slot)
            .arg(lease)
            .exec()
            .into(),
    )
}

fn snapshot(dir: &Path, name: &str, bytes: &[u8]) -> Result<()> {
    let path = dir.join(name);
    fs::create_dir(&path)?;
    Ok(fs::write(path.join(ARCHIVE), bytes)?)
}

#[cfg(unix)]
fn exercise(a: Args) -> Result<()> {
    fs::create_dir(&a.output)?;
    let scratch = tempfile::tempdir_in(&a.output)?;
    let session = scratch.path().join("subject");
    let control = scratch.path().join("control");
    for path in [&session, &control] {
        fs::create_dir(path)?;
    }
    let worker = a.worker.canonicalize()?;
    std::os::unix::fs::symlink(&worker, session.join("worker"))?;
    let transport = if a.zenoh {
        WorkerTransport::LoopbackZenoh
    } else {
        WorkerTransport::Pipe
    };
    let spec = distributed_multi_sphere_spec(WORLD, "convoy.sdf", "default")?;
    let joint = spec.hinge.joint_id;
    if let Some(cut) = &a.cut {
        let (_, bytes) = published_pair::snapshot(cut)?;
        fs::write(control.join(ARCHIVE), bytes)?;
    } else {
        let mut pool = MultiSpherePool::create(&worker, &control, 3, spec)?;
        for tick in 0..148 {
            pool.advance_with_effort_at(0, tick, joint, 0.01)?;
        }
    }
    let seed = fs::read(control.join(ARCHIVE))?;
    fs::write(session.join(ARCHIVE), &seed)?;
    let mut expected = MultiSpherePool::resume_with_transport(&worker, &control, transport)?;
    let mut pool =
        MultiSpherePool::resume_with_transport(&std::env::current_exe()?, &session, transport)?;
    let epoch = pool.epoch();
    let first = pool.observation().hinge.tick;
    let drive = if epoch % 2 == 0 { 0.01 } else { -0.01 };
    let source_pending = pool.pending_joint_effort();
    if pool.spec() != &spec || first >= 1220 || source_pending.is_some_and(|e| e != drive / 2.) {
        return Err("expected an unfinished convoy epoch with the authored drive".into());
    }
    let mut ordinary = AuthoredHinge::load(WORLD, "convoy.sdf")?;
    for _ in 0..first {
        ordinary.queue(joint, drive)?;
        ordinary.advance()?;
    }
    let oracle = reference::observe(&ordinary, &spec, first)?;
    assert_eq!(
        bincode::serialize(pool.observation())?,
        bincode::serialize(&oracle)?
    );
    if source_pending.is_none() {
        for p in [&mut pool, &mut expected] {
            p.queue_effort(joint, drive / 2.)?;
        }
    }
    let barrier = fs::read(control.join(ARCHIVE))?;
    assert_eq!(fs::read(session.join(ARCHIVE))?, barrier);
    let [hinge, x, y] = pool.owners();
    let slot = if a.stall_pair {
        if x != y || x == hinge || pool.pair_image().is_none() {
            return Err("pair stall requires a separately owned native sphere pair".into());
        }
        x
    } else {
        hinge
    };
    let old_pid = pool.worker_pid(slot).ok_or("missing active owner")?;
    for attempt in 1..=a.bootstrap_stalls {
        fs::write(session.join(format!("stall-{slot}-{attempt}")), [])?;
    }
    stop(old_pid)?;
    let start = Instant::now();
    let result = pool.advance_with_effort_at(epoch, first, joint, drive / 2.);
    let fault_seconds = start.elapsed().as_secs_f64();
    let mut failure = None;
    if a.bootstrap_stalls == 2 {
        let Err(ref e @ ArticulationError::MultiSphereRecovery { ref operation, .. }) = result
        else {
            return Err(format!("expected bounded recovery exhaustion, got {result:?}").into());
        };
        let detail = e.to_string();
        let location = format!("epoch {epoch} tick {first} slot {slot} ");
        let operation = operation.to_string();
        assert!(operation.contains(&location));
        if !a.stall_pair {
            assert!(operation.contains(&format!("{location}torque")));
        }
        assert_eq!(detail.matches("recovery bootstrap").count(), 2);
        failure = Some(detail);
        assert!(
            pool.advance_with_effort_at(epoch, first, joint, drive / 2.)
                .is_err()
        );
        assert_eq!(fs::read(session.join(ARCHIVE))?, barrier);
        assert!((0..pool.width()).all(|i| pool.worker_pid(i).is_none()));
        drop(pool);
        pool = MultiSpherePool::resume_with_transport(&worker, &session, transport)?;
        assert_eq!(pool.pending_joint_effort(), Some(drive / 2.));
        pool.advance_with_effort_at(epoch, first, joint, drive / 2.)?;
    } else {
        result?;
    }
    for attempt in 1..=a.bootstrap_stalls {
        assert!(
            session
                .join(format!("stall-{slot}-{attempt}.fired"))
                .is_file()
        );
    }
    let new_pid = pool.worker_pid(slot).ok_or("missing restored owner")?;
    assert_ne!(new_pid, old_pid);
    assert!(matches!(
        pool.advance_with_effort_at(epoch, first, joint, drive / 2.),
        Err(ArticulationError::StaleInterval { .. })
    ));
    let mut digest = Sha256::new();
    let mut transfers = 0;
    for tick in first + 1..=1220 {
        ordinary.queue(joint, drive)?;
        ordinary.advance()?;
        let input = if tick == first + 1 { drive / 2. } else { drive };
        expected.advance_with_effort_at(epoch, tick - 1, joint, input)?;
        if tick > first + 1 {
            pool.advance_with_effort_at(epoch, tick - 1, joint, input)?;
        }
        let obs = reference::observe(&ordinary, &spec, tick)?;
        let state = bincode::serialize(&obs)?;
        assert_eq!(
            bincode::serialize(pool.observation())?,
            state,
            "ordinary state at tick {tick}"
        );
        assert_eq!(pool.pending_joint_effort(), None);
        digest.update(state);
        let [hinge, x, y] = pool.owners();
        if transfers == 0 && obs.sphere_reaction.normal_impulse > 0. && hinge != x && x == y {
            pool.transfer_pair(hinge)?;
            expected.transfer_pair(hinge)?;
            transfers += 1;
        }
        let archive = fs::read(session.join(ARCHIVE))?;
        assert_eq!(
            archive,
            fs::read(control.join(ARCHIVE))?,
            "native barrier at tick {tick}"
        );
        if tick == first + 1 {
            snapshot(&a.output, "first-recovered", &archive)?;
        }
        if tick == 150 && obs.sphere_reaction.normal_impulse > 0. {
            snapshot(&a.output, "loaded-contact", &archive)?;
        }
    }
    if first == 148 {
        assert_eq!(transfers, 1, "expected a loaded sphere-pair migration");
        assert!(a.output.join("loaded-contact").join(ARCHIVE).is_file());
    }
    let report = json!({"width":pool.width(), "transport":format!("{transport:?}"),
        "epoch":epoch,"source_tick":first,"source_pending_effort":source_pending,"source_sha256":format!("{:x}", Sha256::digest(seed)),
        "bootstrap_stalls":a.bootstrap_stalls,"stalled_owner":slot,"old_pid":old_pid,"restored_pid":new_pid,
        "fault_seconds":fault_seconds,"exhausted":failure,"first_recovered_tick":first + 1,
        "verified_advanced_ticks":1220-first,"last_tick":1220,"loaded_pair_transfers":transfers,
        "observation_sha256":format!("{:x}",digest.finalize())});
    fs::write(
        a.output.join("result.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    println!("{report}");
    Ok(())
}
fn main() -> Result<()> {
    #[cfg(unix)]
    {
        let args: Vec<_> = std::env::args_os().collect();
        if let [_, flag, slot, lease] = args.as_slice()
            && flag == "--managed-multisphere-v2"
        {
            return launch(slot, Path::new(lease));
        }
        exercise(Args::parse())
    }
    #[cfg(not(unix))]
    {
        Err("worker suspension qualification requires Linux".into())
    }
}
