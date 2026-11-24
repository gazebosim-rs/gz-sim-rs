// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Contending independent coordinators can admit only one command for an epoch/tick.
use gz_physics_distributed::{
    articulation::{ArticulationError, ArticulationPool},
    moving::MovingPool,
    multi_sphere::MultiSpherePool,
};
use gz_sim_scene::distributed_hinge::{
    SessionBackend, distributed_hinge_spec, distributed_moving_hinge_spec,
    distributed_multi_sphere_spec,
};
use std::{
    error::Error,
    fs, io,
    path::Path,
    sync::Barrier,
    thread,
    time::{Duration, Instant},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
type PoolResult<T> = std::result::Result<T, ArticulationError>;
type Open<P> = fn(&Path, &Path) -> PoolResult<P>;
type Conditional<P> = fn(&mut P, u64, u64, u64, f64) -> PoolResult<()>;
const EXE: &str = env!("CARGO_BIN_EXE_physics_worker");

fn exercise<P: SessionBackend>(
    mut original: P,
    directory: &Path,
    archive: &str,
    open: Open<P>,
    step: Conditional<P>,
) -> Result<()> {
    let worker = Path::new(EXE);
    let file = directory.join(archive);
    let joint = original.hinge_sample().joint_id;
    step(&mut original, 0, 0, joint, 0.01)?;
    original.reset()?;
    original.queue_effort(joint, -0.003)?;
    assert_eq!(original.pending_joint_effort(), Some(-0.003));
    let seed = fs::read(&file)?;

    // A reset can recreate tick zero. Its incremented epoch must invalidate
    // old controls even if their tick now matches again.
    for (e, t) in [(0, 0), (1, 1), (2, 0)] {
        let result = step(&mut original, e, t, joint, 0.022);
        assert!(
            matches!(result, Err(ArticulationError::StaleInterval {expected_epoch, expected_tick, epoch:1, tick:0}) if (expected_epoch,expected_tick)==(e,t))
        );
        assert_eq!(original.pending_joint_effort(), Some(-0.003));
        assert_eq!(seed, fs::read(&file)?);
    }

    let start = Barrier::new(3);
    let lock_checked = Barrier::new(3);
    let go = Barrier::new(3);
    let competitors = thread::scope(|scope| {
        let mut contenders = Vec::new();
        for torque in [0.022, -0.015] {
            let (start, lock_checked, go) = (&start, &lock_checked, &go);
            contenders.push(scope.spawn(move || {
                start.wait();
                let locked=open(worker,directory);
                let contended=matches!(locked, Err(ArticulationError::Io(ref e)) if e.kind()==io::ErrorKind::WouldBlock);
                drop(locked);
                lock_checked.wait();
                go.wait();
                assert!(contended,"independent open bypassed active coordinator");
                let deadline=Instant::now()+Duration::from_secs(10);
                let mut pool=loop {
                    match open(worker,directory) {
                        Ok(p)=>break p,
                        Err(ArticulationError::Io(e)) if e.kind()==io::ErrorKind::WouldBlock && Instant::now()<deadline=>thread::sleep(Duration::from_millis(1)),
                        Err(e)=>return Err(e),
                    }
                };
                let result=step(&mut pool,1,0,joint,torque);
                let applied=match result {
                    Ok(())=>true,
                    Err(ArticulationError::StaleInterval {expected_epoch:1,expected_tick:0,epoch:1,tick:1})=>false,
                    Err(e)=>return Err(e),
                };
                Ok((torque,applied))
            }));
        }
        start.wait();
        lock_checked.wait();
        drop(original);
        go.wait();
        contenders
            .into_iter()
            .map(|h| h.join().unwrap())
            .collect::<PoolResult<Vec<_>>>()
    })?;
    let winners: Vec<_> = competitors.iter().filter(|(_, applied)| *applied).collect();
    assert_eq!(winners.len(), 1);
    let reference = tempfile::tempdir()?;
    let reference_file = reference.path().join(archive);
    fs::write(&reference_file, seed)?;
    let mut control = open(worker, reference.path())?;
    control.queue_effort(joint, winners[0].0)?;
    control.advance()?;
    assert_eq!(fs::read(&file)?, fs::read(&reference_file)?);
    let mut p = open(worker, directory)?;
    assert_eq!(p.pending_joint_effort(), None);
    step(&mut p, 1, 1, joint, 0.001)?;
    control.queue_effort(joint, 0.001)?;
    control.advance()?;
    assert_eq!(fs::read(file)?, fs::read(reference_file)?);
    Ok(())
}

#[test]
fn held_input_advances_once_across_competing_leases_and_epoch_reset() -> Result<()> {
    let worker = Path::new(EXE);
    let tree = distributed_hinge_spec(
        include_str!("../../../examples/distributed_grounded_hinge.sdf"),
        "tree.sdf",
        "default",
    )?;
    for grounded in [true, false] {
        let dir = tempfile::tempdir()?;
        let mut s = tree;
        if !grounded {
            s.ground = None;
        }
        exercise(
            ArticulationPool::create(worker, dir.path(), 2, s)?,
            dir.path(),
            "barrier.gzhinge",
            ArticulationPool::resume,
            ArticulationPool::advance_with_effort_at,
        )?;
    }
    let moving = distributed_moving_hinge_spec(
        include_str!("../../../examples/distributed_hinge_impact.sdf"),
        "moving.sdf",
        "default",
    )?;
    let dir = tempfile::tempdir()?;
    exercise(
        MovingPool::create(worker, dir.path(), 2, moving)?,
        dir.path(),
        "barrier.gzmoving",
        MovingPool::resume,
        MovingPool::advance_with_effort_at,
    )?;
    let pair = distributed_multi_sphere_spec(
        include_str!("../../../examples/distributed_sphere_pair_island.sdf"),
        "pair.sdf",
        "default",
    )?;
    let dir = tempfile::tempdir()?;
    exercise(
        MultiSpherePool::create(worker, dir.path(), 4, pair)?,
        dir.path(),
        "barrier.gzmultisphere",
        MultiSpherePool::resume,
        MultiSpherePool::advance_with_effort_at,
    )?;
    Ok(())
}

#[test]
fn migration_keeps_the_clock_eligible_and_stale_inputs_leave_a_dead_owner_untouched() -> Result<()>
{
    let worker = Path::new(EXE);
    let spec = distributed_hinge_spec(
        include_str!("../../../examples/distributed_grounded_hinge.sdf"),
        "tree.sdf",
        "default",
    )?;
    let dir = tempfile::tempdir()?;
    let reference = tempfile::tempdir()?;
    let archive = "barrier.gzhinge";
    let joint = spec.joint_id;
    let mut p = ArticulationPool::create(worker, dir.path(), 2, spec)?;
    p.advance_with_effort_at(0, 0, joint, 0.021)?;
    p.queue_effort(joint, -0.006)?;
    let destination = 1 - p.owner();
    let generation = p.generation();
    p.transfer(destination)?;
    assert_eq!(p.generation(), generation + 1);
    p.check_interval(0, 1)?;
    let before = fs::read(dir.path().join(archive))?;
    fs::write(reference.path().join(archive), &before)?;
    let mut control = ArticulationPool::resume(worker, reference.path())?;
    control.queue_effort(joint, 0.017)?;
    control.advance()?;

    let dead = p.worker_pid(destination).unwrap();
    assert!(
        std::process::Command::new("kill")
            .arg("-KILL")
            .arg(dead.to_string())
            .status()?
            .success()
    );
    assert!(matches!(
        p.advance_with_effort_at(0, 0, joint, 0.045),
        Err(ArticulationError::StaleInterval { .. })
    ));
    assert_eq!(p.worker_pid(destination), Some(dead));
    assert_eq!(fs::read(dir.path().join(archive))?, before);
    p.advance_with_effort_at(0, 1, joint, 0.017)?;
    assert_ne!(p.worker_pid(destination), Some(dead));
    assert_eq!(
        fs::read(dir.path().join(archive))?,
        fs::read(reference.path().join(archive))?
    );
    Ok(())
}
