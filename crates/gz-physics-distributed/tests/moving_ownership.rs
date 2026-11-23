// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::moving::MovingPool;
use gz_sim_scene::distributed_hinge::distributed_moving_hinge_spec;
use std::{error::Error, path::Path};
const SOURCE: &str = "distributed_hinge_impact.sdf";
const SDF: &str = include_str!("../../../examples/distributed_hinge_impact.sdf");

#[test]
fn fast_pass_joins_before_first_step_and_replayed_commands_step_once() -> Result<(), Box<dyn Error>>
{
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let directory = tempfile::tempdir()?;
    let mut spec = distributed_moving_hinge_spec(SDF, SOURCE, "default")?;
    spec.sphere.position = [3., 0., 0.5];
    spec.sphere.velocity = [-6000., 0., 0.];
    let mut pool = MovingPool::create(worker, directory.path(), 2, spec)?;
    let cold = *pool.observation();
    assert_eq!(pool.owners(), [0, 1]);
    pool.drop_next_acknowledgement(0)?;
    pool.queue_effort(spec.hinge.joint_id, 1.)?;
    drop(pool);
    let mut pool = MovingPool::resume(worker, directory.path())?;
    pool.advance()?;
    assert_eq!(pool.owners(), [0, 0]);
    assert_eq!(pool.worker_steps(0), Some(1));
    assert_eq!(pool.worker_steps(1), Some(0));
    assert!(pool.observation().hinge.qdot.abs() > 1e-6);
    pool.drop_next_acknowledgement(0)?;
    pool.advance()?;
    assert_eq!(pool.worker_steps(0), Some(2));
    assert_eq!(pool.observation().hinge.tick, 2);
    pool.reset()?;
    assert_eq!(pool.owners(), [0, 1]);
    assert_eq!(pool.observation(), &cold);
    assert_eq!(pool.epoch(), 1);
    drop(pool);
    let resumed = MovingPool::resume(worker, directory.path())?;
    assert_eq!(resumed.epoch(), 1);
    assert_eq!(resumed.observation(), &cold);
    Ok(())
}
