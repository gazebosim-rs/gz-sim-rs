// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise guarded interval replay with real worker leases and native snapshots.
#![cfg(target_os = "linux")]

use gz_physics_distributed::{
    articulation::ArticulationError,
    moving::WorkerTransport,
    multi_sphere::{
        MultiSpherePool, MultiSphereSpec, RecoveryMonitor, RecoveryPhase, RecoveryStatus,
        inspect_published_cut, inspect_published_path,
    },
};
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use rustix::{
    event::{PollFd, PollFlags, Timespec, poll},
    fd::OwnedFd,
    process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal},
};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{self, Read, Write},
    path::Path,
    thread,
    time::{Duration, Instant},
};

type Result<T> = std::result::Result<T, Box<dyn Error + Send + Sync>>;
const WORKER: &str = env!("CARGO_BIN_EXE_physics_worker");
const SDF: &str = include_str!("../../../examples/distributed_sphere_pair_island.sdf");
const ARCHIVE: &str = "barrier.gzmultisphere";

fn spec() -> Result<MultiSphereSpec> {
    Ok(distributed_multi_sphere_spec(SDF, "convoy.sdf", "default")?)
}

fn process(pool: &MultiSpherePool, slot: u8) -> rustix::io::Result<OwnedFd> {
    let pid = Pid::from_raw(pool.worker_pid(slot).unwrap() as i32).unwrap();
    pidfd_open(pid, PidfdFlags::empty())
}

fn withhold_slot(dir: &Path, pool: &MultiSpherePool, slot: u8) -> Result<File> {
    pidfd_send_signal(process(pool, slot)?, Signal::KILL)?;
    let lease = File::open(dir.join(format!("worker-{slot}.lock")))?;
    lease.lock()?;
    Ok(lease)
}

fn await_status(monitor: &RecoveryMonitor, expected: RecoveryStatus) {
    let deadline = Instant::now() + Duration::from_secs(30);
    while monitor.snapshot() != Some(expected) {
        assert!(
            Instant::now() < deadline,
            "expected {expected:?}, got {:?}",
            monitor.snapshot()
        );
        thread::sleep(Duration::from_millis(2));
    }
}

#[test]
fn reopen_preserves_writer_lease_and_replays_the_loaded_interval_once() -> Result<()> {
    recover_loaded_interval(Some(-0.005))
}

#[test]
fn advancing_with_only_a_queued_input_preserves_the_native_successor() -> Result<()> {
    recover_loaded_interval(None)
}

fn recover_loaded_interval(contribution: Option<f64>) -> Result<()> {
    let dir = tempfile::tempdir()?;
    let session = dir.path().join("subject");
    let reference = dir.path().join("reference");
    let worker = Path::new(WORKER);
    let spec = spec()?;
    let joint = spec.hinge.joint_id;
    let mut pool = MultiSpherePool::create(worker, &session, 3, spec)?;
    pool.set_reopen_attempts(2)?;
    let monitor = pool.subscribe_recovery();
    assert_eq!(monitor.snapshot(), None);
    for tick in 0..150 {
        pool.advance_with_effort_at(0, tick, joint, -0.01)?;
    }
    let [hinge, left, right] = pool.owners();
    assert_eq!(left, right);
    assert_ne!(left, hinge);
    assert!(pool.observation().sphere_reaction.normal_impulse > 0.);
    pool.transfer_pair(hinge)?;
    pool.queue_effort(joint, -0.005)?;
    let archive = session.join(ARCHIVE);
    let saved = fs::read(&archive)?;
    fs::create_dir(&reference)?;
    fs::write(reference.join(ARCHIVE), &saved)?;
    let mut control = MultiSpherePool::resume(worker, &reference)?;
    if let Some(torque) = contribution {
        control.queue_effort(joint, torque)?;
    }
    control.advance()?;

    let [hinge, pair, _] = pool.owners();
    let old_workers: Vec<_> = (0..pool.width())
        .map(|i| pool.worker_pid(i).unwrap())
        .collect();
    let old_hinge = process(&pool, hinge)?;
    let held = withhold_slot(&session, &pool, pair)?;
    thread::scope(|scope| -> Result<()> {
        let release = scope.spawn(|| -> Result<()> {
            let status = RecoveryStatus {
                epoch: 0,
                tick: 150,
                phase: RecoveryPhase::Restoring,
                attempt: 0,
                budget: 2,
            };
            await_status(&monitor, status);
            // Both normal restoration attempts retain this sibling. Its exit
            // signals that the interrupted transaction has fenced the full pool.
            let mut fds = [PollFd::new(&old_hinge, PollFlags::IN)];
            let timeout = Timespec::try_from(Duration::from_secs(30))?;
            assert_eq!(
                poll(&mut fds, Some(&timeout))?,
                1,
                "pool did not fence its workers"
            );
            assert!(fds[0].revents().contains(PollFlags::IN));
            assert_eq!(fs::read(&archive)?, saved);
            assert!(matches!(
                MultiSpherePool::resume(worker, &session),
                Err(ArticulationError::Io(e)) if e.kind() == io::ErrorKind::WouldBlock
            ));
            await_status(
                &monitor,
                RecoveryStatus {
                    phase: RecoveryPhase::Reopening,
                    attempt: 1,
                    ..status
                },
            );
            drop(held);
            Ok(())
        });
        let result = match contribution {
            Some(torque) => pool.advance_with_effort_at(0, 150, joint, torque),
            None => pool.advance(),
        };
        release.join().expect("lease observer panicked")?;
        result?;
        Ok(())
    })?;
    assert_eq!(pool.reopen_attempts_used(), 1);
    assert_eq!(monitor.snapshot(), None);
    for (slot, old) in old_workers.into_iter().enumerate() {
        assert_ne!(pool.worker_pid(slot as u8).unwrap(), old);
    }
    assert_eq!(pool.pending_joint_effort(), None);
    assert_eq!(pool.observation(), control.observation());
    assert_eq!(fs::read(&archive)?, fs::read(reference.join(ARCHIVE))?);
    assert!(matches!(
        pool.advance_with_effort_at(0, 150, joint, -0.005),
        Err(ArticulationError::StaleInterval { tick: 151, .. })
    ));
    assert_eq!(pool.reopen_attempts_used(), 1);
    Ok(())
}

#[test]
fn ordinary_repair_is_visible_before_any_full_pool_starts() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let mut pool = MultiSpherePool::create(Path::new(WORKER), dir.path(), 3, spec()?)?;
    let monitor = pool.subscribe_recovery();
    let held = withhold_slot(dir.path(), &pool, pool.owners()[0])?;
    thread::scope(|scope| -> Result<()> {
        let watcher = scope.spawn(move || {
            await_status(
                &monitor,
                RecoveryStatus {
                    epoch: 0,
                    tick: 0,
                    phase: RecoveryPhase::Restoring,
                    attempt: 0,
                    budget: 0,
                },
            );
            drop(held);
        });
        let result = pool.advance();
        watcher.join().expect("lease observer panicked");
        Ok(result?)
    })?;
    assert_eq!(pool.observation().hinge.tick, 1);
    assert_eq!(pool.subscribe_recovery().snapshot(), None);
    assert_eq!(pool.reopen_attempts_used(), 0);
    Ok(())
}

#[test]
fn held_slot_exhausts_just_the_requested_budget_and_keeps_the_cut() -> Result<()> {
    let worker = Path::new(WORKER);
    let spec = spec()?;
    for budget in [0u8, 2] {
        let dir = tempfile::tempdir()?;
        let mut pool = MultiSpherePool::create(worker, dir.path(), 3, spec)?;
        pool.set_reopen_attempts(8)?;
        pool.set_reopen_attempts(budget)?;
        assert!(pool.set_reopen_attempts(9).is_err());
        assert_eq!(pool.reopen_attempt_budget(), budget);
        let first = pool.subscribe_recovery();
        pool.queue_effort(spec.hinge.joint_id, 0.005)?;
        let archive = dir.path().join(ARCHIVE);
        let before = fs::read(&archive)?;
        let held = withhold_slot(dir.path(), &pool, pool.owners()[0])?;
        assert!(
            pool.advance_with_effort_at(0, 0, spec.hinge.joint_id, 0.005)
                .is_err()
        );
        assert_eq!(pool.reopen_attempts_used(), u64::from(budget));
        let status = Some(RecoveryStatus {
            epoch: 0,
            tick: 0,
            phase: RecoveryPhase::Fenced,
            attempt: budget,
            budget,
        });
        assert_eq!(first.snapshot(), status);
        assert_eq!(pool.subscribe_recovery().snapshot(), status);
        let receipt = first.fence_receipt().expect("verified transport fence");
        assert_eq!(pool.operator_fence_receipt(), Some(&receipt));
        assert_eq!(
            pool.subscribe_recovery().fence_receipt(),
            Some(receipt.clone())
        );
        assert_eq!(receipt.sha256(), format!("{:x}", Sha256::digest(&before)));
        assert_eq!((receipt.epoch(), receipt.tick()), (0, 0));
        assert!((0..pool.width()).all(|i| pool.worker_pid(i).is_none()));
        assert_eq!(pool.pending_joint_effort(), Some(0.005));
        assert_eq!(fs::read(&archive)?, before);
        drop(held);
        assert!(
            pool.advance_with_effort_at(0, 0, spec.hinge.joint_id, 0.005)
                .is_err()
        );
        assert_eq!(pool.reopen_attempts_used(), u64::from(budget));
        assert_eq!(fs::read(&archive)?, before);
        drop(pool);
        let mut reopened = MultiSpherePool::resume_scene_guarded(
            worker,
            dir.path(),
            Some(3),
            spec,
            WorkerTransport::Pipe,
            &receipt,
        )?;
        reopened.advance_with_effort_at(0, 0, spec.hinge.joint_id, 0.005)?;
        assert_eq!(reopened.pending_joint_effort(), None);
        assert_eq!(reopened.observation().hinge.tick, 1);
    }
    Ok(())
}

#[test]
fn stale_input_and_publication_failures_cannot_start_a_reopen() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let spec = spec()?;
    let joint = spec.hinge.joint_id;
    let mut pool = MultiSpherePool::create(Path::new(WORKER), dir.path(), 3, spec)?;
    pool.set_reopen_attempts(3)?;
    let monitor = pool.subscribe_recovery();
    pool.queue_effort(joint, 0.005)?;
    let archive = dir.path().join(ARCHIVE);
    let saved = fs::read(&archive)?;
    assert!(matches!(
        pool.advance_with_effort_at(0, 1, joint, 0.),
        Err(ArticulationError::StaleInterval { .. })
    ));
    assert!(pool.advance_with_effort_at(0, 0, joint, f64::NAN).is_err());
    assert!(pool.advance_with_effort_at(0, 0, joint + 1, 0.).is_err());
    assert_eq!(fs::read(&archive)?, saved);
    assert_eq!(monitor.snapshot(), None);
    let staging = dir.path().join("barrier.pending.gzhinge");
    fs::create_dir(&staging)?;
    assert!(matches!(
        pool.advance_with_effort_at(0, 0, joint, 0.005),
        Err(ArticulationError::Publication(_))
    ));
    assert_eq!(
        monitor.snapshot(),
        Some(RecoveryStatus {
            epoch: 0,
            tick: 0,
            phase: RecoveryPhase::Fenced,
            attempt: 0,
            budget: 3
        })
    );
    assert_eq!(monitor.fence_receipt(), None);
    assert_eq!(pool.operator_fence_receipt(), None);
    fs::remove_dir(staging)?;
    assert!(pool.advance_with_effort_at(0, 0, joint, 0.005).is_err());
    assert_eq!(pool.reopen_attempts_used(), 0);
    assert_eq!(pool.pending_joint_effort(), Some(0.005));
    assert_eq!(fs::read(archive)?, saved);
    Ok(())
}

#[test]
fn another_valid_archive_at_the_same_clock_cannot_authorize_replay() -> Result<()> {
    let worker = Path::new(WORKER);
    let spec = spec()?;
    let joint = spec.hinge.joint_id;
    let subject = tempfile::tempdir()?;
    let reference = tempfile::tempdir()?;
    let mut pool = MultiSpherePool::create(worker, subject.path(), 3, spec)?;
    let mut other = MultiSpherePool::create(worker, reference.path(), 3, spec)?;
    pool.set_reopen_attempts(2)?;
    pool.queue_effort(joint, -0.005)?;
    other.queue_effort(joint, 0.005)?;
    let alternate = fs::read(reference.path().join(ARCHIVE))?;
    let archive = subject.path().join(ARCHIVE);
    assert_ne!(fs::read(&archive)?, alternate);
    // Simulate a stale checkpoint copy made outside the coordinator. The
    // checksum, recipe and clock alone cannot identify its pending input.
    fs::write(&archive, &alternate)?;
    let held = withhold_slot(subject.path(), &pool, pool.owners()[0])?;
    assert!(pool.advance_with_effort_at(0, 0, joint, 0.005).is_err());
    assert_eq!(pool.reopen_attempts_used(), 0);
    // A reader attaching after terminal failure can inspect the original cut.
    assert_eq!(
        pool.subscribe_recovery().snapshot(),
        Some(RecoveryStatus {
            epoch: 0,
            tick: 0,
            phase: RecoveryPhase::Fenced,
            attempt: 0,
            budget: 2
        })
    );
    assert!((0..pool.width()).all(|slot| pool.worker_pid(slot).is_none()));
    assert_eq!(pool.pending_joint_effort(), Some(-0.005));
    assert_eq!(fs::read(&archive)?, alternate);
    assert_eq!(pool.operator_fence_receipt(), None);
    drop(held);
    drop(pool);
    assert_eq!(
        MultiSpherePool::resume(worker, subject.path())?.pending_joint_effort(),
        Some(0.005)
    );
    Ok(())
}

#[test]
fn guarded_operator_reopen_preserves_loaded_contact_and_pending_input() -> Result<()> {
    let worker = Path::new(WORKER);
    let spec = spec()?;
    let joint = spec.hinge.joint_id;
    let subject = tempfile::tempdir()?;
    let dir = subject.path();
    let path = dir.join(ARCHIVE);
    let mut pool = MultiSpherePool::create(worker, dir, 3, spec)?;
    for tick in 0..150 {
        pool.advance_with_effort_at(0, tick, joint, -0.01)?;
    }
    assert!(pool.observation().sphere_reaction.normal_impulse > 0.);
    pool.transfer_pair(pool.owners()[0])?;
    let previous = File::open(&path)?;
    let other = fs::read(&path)?;
    pool.queue_effort(joint, -0.005)?;
    let saved = fs::read(&path)?;
    let inspected = inspect_published_path(&path)?;
    assert_eq!(inspected.cut.observation, *pool.observation());
    assert_eq!(inspected.cut.pending_effort, Some(-0.005));
    assert_eq!(inspected.sha256, format!("{:x}", Sha256::digest(&saved)));
    // Holding one pre-publication file descriptor keeps its digest and fields
    // together, including a different pending input at exactly the same tick.
    let older = inspect_published_cut(previous)?;
    assert_eq!(
        older.cut.observation.hinge.tick,
        inspected.cut.observation.hinge.tick
    );
    assert_eq!(older.cut.pending_effort, None);
    assert_eq!(older.sha256, format!("{:x}", Sha256::digest(&other)));

    let monitor = pool.subscribe_recovery();
    let held = withhold_slot(dir, &pool, pool.owners()[1])?;
    assert!(pool.advance_with_effort_at(0, 150, joint, -0.005).is_err());
    let fence = monitor.fence_receipt().ok_or("expected operator fence")?;
    assert_eq!(fence.sha256(), inspected.sha256);
    let guarded = |dir: &Path, width, spec| {
        MultiSpherePool::resume_scene_guarded(
            worker,
            dir,
            width,
            spec,
            WorkerTransport::Pipe,
            &fence,
        )
    };
    assert!(
        matches!(guarded(dir, Some(3), spec), Err(ArticulationError::Io(e)) if e.kind() == io::ErrorKind::WouldBlock)
    );
    drop(pool);
    assert_eq!(monitor.fence_receipt(), Some(fence.clone()));
    assert!(
        guarded(dir, None, spec).is_err(),
        "owner's lease is still held"
    );
    assert_eq!(fs::read(&path)?, saved);
    drop(held);

    let empty = tempfile::tempdir()?;
    assert!(guarded(empty.path(), None, spec).is_err());
    assert_eq!(fs::read_dir(empty.path())?.count(), 0);
    let missing = empty.path().join("missing");
    assert!(guarded(&missing, None, spec).is_err());
    assert!(!missing.exists());
    let staging = dir.join("barrier.pending.gzhinge");
    fs::write(&staging, b"partial candidate for inspection")?;
    let claim = dir.join("controller.lock");
    let copy = tempfile::tempdir()?;
    fs::copy(&path, copy.path().join(ARCHIVE))?;
    let copy_claim = copy.path().join("controller.lock");
    fs::copy(&claim, &copy_claim)?;
    assert!(matches!(
        guarded(copy.path(), None, spec),
        Err(ArticulationError::Invalid(_))
    ));
    fs::remove_file(&copy_claim)?;
    fs::hard_link(&claim, &copy_claim)?;
    assert!(matches!(
        guarded(copy.path(), None, spec),
        Err(ArticulationError::Invalid(_))
    ));

    let original_claim = dir.join("original-controller.lock");
    fs::rename(&claim, &original_claim)?;
    fs::copy(&original_claim, &claim)?;
    assert!(matches!(
        guarded(dir, None, spec),
        Err(ArticulationError::Invalid(_))
    ));
    fs::remove_file(&claim)?;
    // Preload the entire marker-read limit so even a regression cannot hang
    // the suite. Refusing the FIFO must leave all its bytes unread.
    rustix::fs::mkfifoat(
        rustix::fs::CWD,
        &claim,
        rustix::fs::Mode::RUSR | rustix::fs::Mode::WUSR,
    )?;
    let mut fifo = File::options().read(true).write(true).open(&claim)?;
    let sent = [b'X'; 64];
    fifo.write_all(&sent)?;
    assert!(
        matches!(guarded(dir, None, spec), Err(ArticulationError::Io(e)) if e.kind() == io::ErrorKind::InvalidInput)
    );
    let mut remaining = [0; 64];
    use std::os::unix::fs::OpenOptionsExt;
    File::options()
        .read(true)
        .custom_flags(rustix::fs::OFlags::NONBLOCK.bits() as i32)
        .open(&claim)?
        .read_exact(&mut remaining)?;
    assert_eq!(remaining, sent);
    drop(fifo);
    fs::remove_file(&claim)?;
    fs::rename(original_claim, &claim)?;

    let original_archive = dir.join("original-archive");
    fs::rename(&path, &original_archive)?;
    rustix::fs::mkfifoat(
        rustix::fs::CWD,
        &path,
        rustix::fs::Mode::RUSR | rustix::fs::Mode::WUSR,
    )?;
    assert!(
        matches!(guarded(dir, None, spec), Err(ArticulationError::Io(e)) if e.kind() == io::ErrorKind::InvalidInput)
    );
    assert!(
        matches!(inspect_published_path(&path), Err(ArticulationError::Io(e)) if e.kind() == io::ErrorKind::InvalidInput)
    );
    fs::remove_file(&path)?;
    fs::rename(original_archive, &path)?;
    assert!(guarded(dir, Some(4), spec).is_err());
    let mut wrong_recipe = spec;
    wrong_recipe.hinge.joint_id += 100;
    assert!(guarded(dir, Some(3), wrong_recipe).is_err());
    fs::write(&path, &other)?;
    assert!(guarded(dir, None, spec).is_err());
    assert_eq!(fs::read(&path)?, other);
    fs::write(&path, &saved[..saved.len() - 1])?;
    assert!(guarded(dir, None, spec).is_err());
    assert!(inspect_published_path(&path).is_err());
    assert_eq!(fs::read(&staging)?, b"partial candidate for inspection");
    fs::write(&path, &saved)?;

    let reference = tempfile::tempdir()?;
    let reference_path = reference.path().join(ARCHIVE);
    fs::write(&reference_path, &saved)?;
    let mut control = MultiSpherePool::resume(worker, reference.path())?;
    let mut resumed = guarded(dir, Some(3), spec)?;
    assert!(!staging.exists());
    assert_eq!(resumed.pending_joint_effort(), Some(-0.005));
    assert_eq!(resumed.observation(), control.observation());
    assert_eq!(fs::read(&path)?, saved);
    control.advance_with_effort_at(0, 150, joint, -0.005)?;
    resumed.advance_with_effort_at(0, 150, joint, -0.005)?;
    assert_eq!(resumed.observation(), control.observation());
    assert_eq!(fs::read(&path)?, fs::read(&reference_path)?);
    assert_eq!(resumed.pending_joint_effort(), None);
    drop(resumed);
    assert!(guarded(dir, None, spec).is_err());
    Ok(())
}

#[test]
fn transport_fence_on_a_nonadvancing_operation_never_issues_a_step_receipt() -> Result<()> {
    let worker = Path::new(WORKER);
    let spec = spec()?;
    let subject = tempfile::tempdir()?;
    let mut pool = MultiSpherePool::create(worker, subject.path(), 3, spec)?;
    let monitor = pool.subscribe_recovery();
    let lease = withhold_slot(subject.path(), &pool, pool.owners()[0])?;
    assert!(pool.queue_effort(spec.hinge.joint_id, 0.01).is_err());
    assert_eq!(monitor.snapshot().unwrap().phase, RecoveryPhase::Fenced);
    assert_eq!(monitor.fence_receipt(), None);
    assert!(pool.advance().is_err());
    assert_eq!(pool.operator_fence_receipt(), None);
    drop(lease);
    Ok(())
}

#[test]
#[ignore = "requires an unprivileged process subject to Linux directory permission checks"]
fn directory_sync_open_denial_after_rename_leaves_successor_inspect_only() -> Result<()> {
    use std::os::unix::fs::PermissionsExt;

    let worker = Path::new(WORKER);
    let spec = spec()?;
    let joint = spec.hinge.joint_id;
    let subject = tempfile::tempdir()?;
    let reference = tempfile::tempdir()?;
    let mut pool = MultiSpherePool::create(worker, subject.path(), 3, spec)?;
    let mut control = MultiSpherePool::create(worker, reference.path(), 3, spec)?;
    let monitor = pool.subscribe_recovery();
    control.advance_with_effort_at(0, 0, joint, -0.01)?;
    let permissions = fs::metadata(subject.path())?.permissions();
    // Write and search permit the temp file and atomic rename. Directory-open
    // for fsync then fails; the pathname already refers to the new successor.
    fs::set_permissions(subject.path(), fs::Permissions::from_mode(0o300))?;
    let can_read = File::open(subject.path()).is_ok();
    let result = pool.advance_with_effort_at(0, 0, joint, -0.01);
    fs::set_permissions(subject.path(), permissions)?;
    assert!(!can_read, "run this qualification as an unprivileged user");
    assert!(
        matches!(result, Err(ArticulationError::Publication(e)) if e.kind() == io::ErrorKind::PermissionDenied)
    );
    assert_eq!(pool.observation().hinge.tick, 0);
    assert_eq!(monitor.snapshot().unwrap().tick, 0);
    assert_eq!(monitor.fence_receipt(), None);
    assert_eq!(pool.operator_fence_receipt(), None);
    let committed = inspect_published_path(&subject.path().join(ARCHIVE))?;
    assert_eq!(committed.cut.observation.hinge.tick, 1);
    assert_eq!(
        committed.sha256,
        inspect_published_path(&reference.path().join(ARCHIVE))?.sha256
    );
    Ok(())
}
