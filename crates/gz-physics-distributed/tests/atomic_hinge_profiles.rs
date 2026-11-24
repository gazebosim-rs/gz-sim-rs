// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare one-publication intervals with the original separately durable operations.
#![cfg(target_os = "linux")]

use gz_physics_distributed::{articulation::ArticulationPool, moving::MovingPool};
use gz_sim_scene::distributed_hinge::{distributed_hinge_spec, distributed_moving_hinge_spec};
use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
use std::{error::Error, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WORKER: &str = env!("CARGO_BIN_EXE_physics_worker");
const TREE: &str = include_str!("../../../examples/distributed_grounded_hinge.sdf");
const MOVING: &str = include_str!("../../../examples/distributed_hinge_impact.sdf");

fn session() -> std::io::Result<tempfile::TempDir> {
    tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())
}

fn kill(pid: u32) -> Result<()> {
    let fd = pidfd_open(Pid::from_raw(pid as i32).unwrap(), PidfdFlags::empty())?;
    pidfd_send_signal(fd, Signal::KILL)?;
    Ok(())
}

#[test]
fn floating_and_grounded_atomic_tree_reopen_actual_native_pending_effort() -> Result<()> {
    let worker = Path::new(WORKER);
    let authored = distributed_hinge_spec(TREE, "grounded.sdf", "default")?;
    for grounded in [false, true] {
        let mut spec = authored;
        if !grounded {
            spec.ground = None;
        }
        let joint = spec.joint_id;
        let (a, b) = (session()?, session()?);
        let mut atomic = ArticulationPool::create(worker, a.path(), 2, spec)?;
        let mut previous = ArticulationPool::create(worker, b.path(), 2, spec)?;
        let mut warm_ground = false;
        for tick in 0..420 {
            // A durable signed zero is still an admitted command; it matters
            // when a live actuator decides whether to supply its startup drive.
            let held = if tick == 0 { 0.0 } else { -0.004 };
            let driven = if tick % 100 < 50 { 0.014 } else { -0.006 };
            for p in [&mut previous, &mut atomic] {
                p.queue_effort(joint, held)?;
                assert_eq!(p.pending_joint_effort(), Some(held));
            }
            if matches!(tick, 0 | 90 | 200) {
                drop(atomic);
                atomic = ArticulationPool::resume(worker, a.path())?;
                assert_eq!(atomic.pending_joint_effort(), Some(held));
            }
            if tick == 200 {
                for p in [&mut previous, &mut atomic] {
                    p.transfer(1)?;
                    assert_eq!(p.pending_joint_effort(), Some(held));
                }
                let owner = atomic.owner();
                kill(atomic.worker_pid(owner).unwrap())?;
            } else if tick == 170 {
                atomic.drop_next_acknowledgement(atomic.owner())?;
            }
            previous.queue_effort(joint, driven)?;
            previous.advance()?;
            atomic.advance_with_effort(joint, driven)?;
            assert_eq!(atomic.pending_joint_effort(), None);
            assert_eq!(atomic.observation(), previous.observation());
            assert_eq!(atomic.image(), previous.image());
            assert_eq!(
                fs::read(a.path().join("barrier.gzhinge"))?,
                fs::read(b.path().join("barrier.gzhinge"))?
            );
            warm_ground |= atomic
                .observation()
                .ground_reactions
                .is_some_and(|reactions| {
                    reactions
                        .iter()
                        .any(|r| r.active && r.warmstart_normal_impulse > 0.)
                });
        }
        assert_eq!(warm_ground, grounded);
        atomic.queue_effort(joint, 0.375)?;
        atomic.reset()?;
        assert_eq!(atomic.pending_joint_effort(), None);
        assert_eq!(atomic.observation().tick, 0);
        drop(atomic);
        assert_eq!(
            ArticulationPool::resume(worker, a.path())?.pending_joint_effort(),
            None
        );
    }
    Ok(())
}

#[test]
fn moving_sphere_atomic_contact_survives_join_load_transfer_and_release() -> Result<()> {
    let worker = Path::new(WORKER);
    let spec = distributed_moving_hinge_spec(MOVING, "moving.sdf", "default")?;
    let joint = spec.hinge.joint_id;
    let (a, b) = (session()?, session()?);
    let mut atomic = MovingPool::create(worker, a.path(), 3, spec)?;
    let mut previous = MovingPool::create(worker, b.path(), 3, spec)?;
    let mut transferred = false;
    let mut released = false;
    for tick in 0..2200 {
        let owners = atomic.owners();
        let held = if tick == 0 { 0.0 } else { -0.01 };
        for p in [&mut previous, &mut atomic] {
            p.queue_effort(joint, held)?;
        }
        if tick == 0 {
            kill(atomic.worker_pid(owners[1]).unwrap())?;
        } else if !transferred
            && atomic
                .observation()
                .reactions
                .iter()
                .any(|r| r.active && r.warmstart_normal_impulse > 0.)
        {
            for p in [&mut previous, &mut atomic] {
                p.transfer(2)?;
            }
            drop(atomic);
            atomic = MovingPool::resume(worker, a.path())?;
            assert_eq!(atomic.pending_joint_effort(), Some(held));
            atomic.drop_next_acknowledgement(atomic.owners()[0])?;
            transferred = true;
        }
        previous.queue_effort(joint, 0.02)?;
        previous.advance()?;
        atomic.advance_with_effort(joint, 0.02)?;
        assert_eq!(atomic.pending_joint_effort(), None);
        assert_eq!(atomic.observation(), previous.observation());
        assert_eq!(atomic.owners(), previous.owners());
        assert_eq!(
            fs::read(a.path().join("barrier.gzmoving"))?,
            fs::read(b.path().join("barrier.gzmoving"))?
        );
        if transferred && atomic.owners()[0] != atomic.owners()[1] {
            released = true;
            break;
        }
    }
    assert!(transferred && released);
    Ok(())
}
