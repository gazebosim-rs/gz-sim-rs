// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify the public atomic interval against a separately durable torque and advance.
#![cfg(target_os = "linux")]

use gz_physics_distributed::multi_sphere::{MultiSpherePool, read_published_cut};
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
use std::{error::Error, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/distributed_sphere_pair_island.sdf");
const ARCHIVE: &str = "barrier.gzmultisphere";

fn session() -> std::io::Result<tempfile::TempDir> {
    tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())
}

#[test]
fn atomic_interval_matches_both_receipts_across_worker_loss_and_regroup() -> Result<()> {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spec = distributed_multi_sphere_spec(SDF, "convoy.sdf", "default")?;
    let joint = spec.hinge.joint_id;
    let (a, b) = (session()?, session()?);
    let mut atomic = MultiSpherePool::create(worker, a.path(), 4, spec)?;
    let mut previous = MultiSpherePool::create(worker, b.path(), 4, spec)?;
    let mut transitions = Vec::new();
    for tick in 0..1220 {
        let owners = atomic.owners();
        let [hinge, sphere, tail] = owners;
        if tick == 150 {
            assert!(atomic.observation().sphere_reaction.normal_impulse > 0.);
            assert!(
                atomic
                    .observation()
                    .sphere_reaction
                    .warmstart_normal_impulse
                    > 0.
            );
            assert_eq!(sphere, tail);
            assert_ne!(hinge, sphere);
            for p in [&mut previous, &mut atomic] {
                p.transfer_pair(hinge)?;
            }
        }
        // Keep an independently acknowledged contribution across native retries.
        // Split the same 0.010 N·m exactly into two binary-scaled halves.
        let pending = matches!(tick, 0 | 130 | 150 | 152 | 1213);
        let effort = if pending {
            atomic.queue_effort(joint, 0.005)?;
            previous.queue_effort(joint, 0.005)?;
            0.005
        } else {
            0.01
        };
        let failure = match tick {
            // B fails after the live hinge and A have already advanced.
            0 => Some(tail),
            // The independently owned pair fails on its preview after the torque receipt.
            130 => Some(sphere),
            // The resident coupled hinge fails on its torque request.
            152 => Some(hinge),
            _ => None,
        };
        let killed = if let Some(slot) = failure {
            let pid = atomic.worker_pid(slot).unwrap();
            let fd = pidfd_open(Pid::from_raw(pid as i32).unwrap(), PidfdFlags::empty())?;
            pidfd_send_signal(fd, Signal::KILL)?;
            Some((slot, pid))
        } else {
            None
        };
        if tick == 129 {
            // Retransmit both the privately staged pair preview and the torque receipt.
            atomic.drop_next_acknowledgement(hinge)?;
            atomic.drop_next_acknowledgement(sphere)?;
        }
        previous.queue_effort(joint, effort)?;
        previous.advance()?;
        atomic.advance_with_effort(joint, effort)?;
        if let Some((slot, pid)) = killed {
            assert_ne!(atomic.worker_pid(slot), Some(pid));
        }
        if owners != atomic.owners() {
            transitions.push(tick + 1);
        }
        assert_eq!(atomic.observation(), previous.observation(), "tick {tick}");
        assert_eq!(atomic.owners(), previous.owners());
        assert_eq!(atomic.image(), previous.image());
        assert_eq!(atomic.pair_image(), previous.pair_image());
        assert_eq!(atomic.pending_joint_effort(), None);
        assert_eq!(
            fs::read(a.path().join(ARCHIVE))?,
            fs::read(b.path().join(ARCHIVE))?,
            "tick {tick}"
        );
        if matches!(tick, 129 | 149 | 150 | 1213) {
            // The inspector uses neither the controller lock nor a new worker.
            // A file opened before a later publication remains that exact cut.
            let file = fs::File::open(a.path().join(ARCHIVE))?;
            let previous_observation = *atomic.observation();
            let owners = atomic.owners();
            atomic.queue_effort(joint, 0.)?;
            previous.queue_effort(joint, 0.)?;
            let snapshot = read_published_cut(file)?;
            assert_eq!(snapshot.observation, previous_observation);
            assert_eq!(snapshot.owners, owners);
            assert_eq!(snapshot.spec, spec);
            assert_eq!(snapshot.pending_effort, None);
            assert_eq!(
                read_published_cut(fs::File::open(a.path().join(ARCHIVE))?)?.pending_effort,
                Some(0.)
            );
            let mut corrupt = fs::read(a.path().join(ARCHIVE))?;
            corrupt[8] ^= 1;
            assert!(read_published_cut(corrupt.as_slice()).is_err());
            drop(atomic);
            atomic = MultiSpherePool::resume(worker, a.path())?;
            assert_eq!(atomic.observation(), previous.observation());
        }
    }
    assert_eq!(transitions, [125, 151, 1214]);
    Ok(())
}

#[test]
fn rejection_and_cancellation_preserve_independently_queued_input() -> Result<()> {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spec = distributed_multi_sphere_spec(SDF, "convoy.sdf", "default")?;
    let dir = session()?;
    let path = dir.path().join(ARCHIVE);
    let mut p = MultiSpherePool::create(worker, dir.path(), 3, spec)?;
    let joint = spec.hinge.joint_id;
    p.queue_effort(joint, f64::MAX)?;
    let before = fs::read(&path)?;
    for (id, value) in [
        (joint + 1, 1.),
        (joint, f64::NAN),
        (joint, f64::INFINITY),
        (joint, f64::MAX),
    ] {
        assert!(p.advance_with_effort(id, value).is_err());
        assert_eq!(p.pending_joint_effort(), Some(f64::MAX));
        assert_eq!(p.observation().hinge.tick, 0);
        assert_eq!(fs::read(&path)?, before);
    }
    let control = session()?;
    fs::write(control.path().join(ARCHIVE), before)?;
    let mut expected = MultiSpherePool::resume(worker, control.path())?;
    expected.queue_effort(joint, -f64::MAX)?;
    expected.advance()?;
    p.advance_with_effort(joint, -f64::MAX)?;
    assert_eq!(p.pending_joint_effort(), None);
    assert_eq!(p.observation(), expected.observation());
    assert_eq!(fs::read(path)?, fs::read(control.path().join(ARCHIVE))?);
    Ok(())
}

#[test]
fn failed_atomic_publication_recovers_the_prior_cut_and_retries_once() -> Result<()> {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spec = distributed_multi_sphere_spec(SDF, "convoy.sdf", "default")?;
    let (attempt, control) = (session()?, session()?);
    let joint = spec.hinge.joint_id;
    let mut p = MultiSpherePool::create(worker, attempt.path(), 3, spec)?;
    p.queue_effort(joint, -0.004)?;
    let archive = attempt.path().join(ARCHIVE);
    let original = fs::read(&archive)?;
    fs::write(control.path().join(ARCHIVE), &original)?;
    let mut expected = MultiSpherePool::resume(worker, control.path())?;
    expected.queue_effort(joint, 0.014)?;
    expected.advance()?;

    // The real staging file cannot be opened. All worker steps have completed
    // privately but their receipt cannot be published or acknowledged.
    let staging = attempt.path().join("barrier.pending.gzhinge");
    fs::create_dir(&staging)?;
    assert!(matches!(
        p.advance_with_effort(joint, 0.014),
        Err(gz_physics_distributed::articulation::ArticulationError::Publication(_))
    ));
    assert_eq!(fs::read(&archive)?, original);
    assert!(matches!(
        p.advance_with_effort_at(12, 34, joint, 0.014),
        Err(
            gz_physics_distributed::articulation::ArticulationError::Invalid(
                "reopen the fenced session"
            )
        )
    ));
    drop(p);
    fs::remove_dir(&staging)?;
    let mut recovered = MultiSpherePool::resume(worker, attempt.path())?;
    assert_eq!(recovered.observation().hinge.tick, 0);
    assert_eq!(recovered.pending_joint_effort(), Some(-0.004));
    recovered.advance_with_effort(joint, 0.014)?;
    assert_eq!(recovered.observation(), expected.observation());
    assert_eq!(fs::read(archive)?, fs::read(control.path().join(ARCHIVE))?);
    Ok(())
}

#[test]
fn an_unresponsive_owner_is_fenced_before_replaying_the_uncommitted_step() -> Result<()> {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spec = distributed_multi_sphere_spec(SDF, "convoy.sdf", "default")?;
    let a = session()?;
    let b = session()?;
    let mut lost = MultiSpherePool::create(worker, a.path(), 3, spec)?;
    let mut ordinary = MultiSpherePool::create(worker, b.path(), 3, spec)?;
    let slot = lost.owners()[2];
    let pid = lost.worker_pid(slot).unwrap();
    let fd = pidfd_open(Pid::from_raw(pid as i32).unwrap(), PidfdFlags::empty())?;
    pidfd_send_signal(fd, Signal::STOP)?;
    lost.advance_with_effort(spec.hinge.joint_id, 0.01)?;
    ordinary.advance_with_effort(spec.hinge.joint_id, 0.01)?;
    assert_ne!(lost.worker_pid(slot), Some(pid));
    assert_eq!(lost.observation(), ordinary.observation());
    assert_eq!(
        fs::read(a.path().join(ARCHIVE))?,
        fs::read(b.path().join(ARCHIVE))?
    );
    drop(lost);
    let mut resumed = MultiSpherePool::resume(worker, a.path())?;
    for _ in 0..30 {
        resumed.advance()?;
        ordinary.advance()?;
        assert_eq!(resumed.observation(), ordinary.observation());
    }
    Ok(())
}

#[test]
fn loss_during_parallel_dispatch_retains_every_other_process_and_drains_its_reply() -> Result<()> {
    use std::time::{Duration, Instant};
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spec = distributed_multi_sphere_spec(SDF, "separate.sdf", "default")?;
    let a = session()?;
    let b = session()?;
    let mut live = MultiSpherePool::create(worker, a.path(), 4, spec)?;
    let mut control = MultiSpherePool::create(worker, b.path(), 4, spec)?;
    let owners = live.owners();
    assert_ne!(owners[0], owners[1]);
    assert_ne!(owners[0], owners[2]);
    assert_ne!(owners[1], owners[2]);
    for slot in [owners[2], owners[1], owners[0]] {
        let before = (0..live.width())
            .map(|i| live.worker_pid(i).unwrap())
            .collect::<Vec<_>>();
        let id = before[slot as usize];
        let fd = pidfd_open(Pid::from_raw(id as i32).unwrap(), PidfdFlags::empty())?;
        pidfd_send_signal(&fd, Signal::KILL)?;
        let until = Instant::now() + Duration::from_secs(5);
        loop {
            let status = fs::read_to_string(format!("/proc/{id}/stat"))?;
            if status
                .rsplit_once(") ")
                .is_some_and(|(_, tail)| tail.starts_with("Z "))
            {
                break;
            }
            assert!(
                Instant::now() < until,
                "killed child failed to exit before dispatch"
            );
            std::thread::yield_now();
        }
        live.advance_with_effort(spec.hinge.joint_id, -0.005)?;
        control.advance_with_effort(spec.hinge.joint_id, -0.005)?;
        for (i, &pid) in before.iter().enumerate() {
            assert_eq!(live.worker_pid(i as u8) == Some(pid), i != slot as usize);
        }
        assert_eq!(
            fs::read(a.path().join(ARCHIVE))?,
            fs::read(b.path().join(ARCHIVE))?
        );
    }
    Ok(())
}

#[test]
fn unavailable_replacement_lease_preserves_pending_interval_and_both_failures() -> Result<()> {
    use gz_physics_distributed::articulation::ArticulationError;
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spec = distributed_multi_sphere_spec(SDF, "convoy.sdf", "default")?;
    let joint = spec.hinge.joint_id;
    let (session, reference) = (session()?, session()?);
    let mut pool = MultiSpherePool::create(worker, session.path(), 3, spec)?;
    pool.queue_effort(joint, 0.005)?;
    let committed = fs::read(session.path().join(ARCHIVE))?;
    fs::write(reference.path().join(ARCHIVE), &committed)?;
    let slot = pool.owners()[0];
    let pid = Pid::from_raw(pool.worker_pid(slot).unwrap() as i32).unwrap();
    pidfd_send_signal(pidfd_open(pid, PidfdFlags::empty())?, Signal::KILL)?;
    // Both stock replacements encounter the same held native lease. Releasing
    // it after the operation returns makes the test independent of scheduling.
    let lease = fs::File::open(session.path().join(format!("worker-{slot}.lock")))?;
    lease.lock()?;
    let err = pool.advance_with_effort_at(0, 0, joint, 0.005).unwrap_err();
    let ArticulationError::MultiSphereRecovery {
        operation,
        recovery,
    } = err
    else {
        panic!("missing fault history: {err}");
    };
    assert!(
        operation
            .to_string()
            .contains(&format!("slot {slot} torque"))
    );
    let ArticulationError::MultiSphereRecovery {
        operation: first,
        recovery: second,
    } = *recovery
    else {
        panic!("missing second restoration: {recovery}");
    };
    for error in [first, second] {
        assert!(
            error
                .to_string()
                .contains(&format!("slot {slot} recovery bootstrap"))
        );
    }
    assert!(pool.advance_with_effort_at(0, 0, joint, 0.005).is_err());
    assert!((0..pool.width()).all(|i| pool.worker_pid(i).is_none()));
    assert_eq!(fs::read(session.path().join(ARCHIVE))?, committed);
    drop(lease);
    drop(pool);
    for path in [&session, &reference] {
        let mut p = MultiSpherePool::resume(worker, path.path())?;
        assert_eq!(p.pending_joint_effort(), Some(0.005));
        p.advance_with_effort_at(0, 0, joint, 0.005)?;
        assert!(matches!(
            p.advance_with_effort_at(0, 0, joint, 0.005),
            Err(ArticulationError::StaleInterval { .. })
        ));
    }
    assert_eq!(
        fs::read(session.path().join(ARCHIVE))?,
        fs::read(reference.path().join(ARCHIVE))?
    );
    Ok(())
}
