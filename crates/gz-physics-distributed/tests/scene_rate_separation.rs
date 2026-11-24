// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::{Coordinator, NativeRecoveryPlan, RecoveryPlan},
    physics::{Body, Primitive, SimulationStep, fixture},
    pool::{PoolNativeRecovery, WorkerPool},
    transport::WorkerProcess,
};
use std::{error::Error, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const ZERO: [f64; 3] = [0.; 3];
fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
fn glancing() -> Vec<Body> {
    let a = Body {
        id: 1,
        primitive: Primitive::Sphere,
        size: [0.2; 3],
        position: ZERO,
        linear: ZERO,
        angular: ZERO,
        friction: 0.,
        mass: 1.,
        ..fixture()[0].promoted()
    };
    let b = Body {
        id: 2,
        position: [-0.48, 0.12, 0.],
        linear: [80., 0., 0.],
        ..a.clone()
    };
    vec![a, b]
}

#[test]
fn pool_preserves_scene_rate_through_duplicate_receipt_separation_and_native_restart() -> Result<()>
{
    let seeds = vec![glancing(), vec![], vec![]];
    let mut pool = WorkerPool::spawn(
        worker(),
        3001,
        seeds.clone(),
        SimulationStep::Ms10,
        None,
        64,
    )?;
    let mut regular = WorkerPool::spawn(worker(), 3002, seeds, SimulationStep::Ms10, None, 64)?;
    let pid = pool.worker_pid(0);
    pool.drop_next_acknowledgement(0)?;
    assert!(pool.advance_adaptive_with_separation(ZERO, &[]).is_err());
    assert_eq!(pool.frame().tick, 0);
    assert_eq!(pool.last_substeps(), None);
    pool.advance_adaptive_with_separation(ZERO, &[])?;
    regular.advance_adaptive_in_field(ZERO, &[])?;
    assert_eq!(
        pool.frame().workers[0].snapshot,
        regular.frame().workers[0].snapshot
    );
    assert_eq!(pool.worker_pid(0), pid);
    assert_eq!(pool.last_substeps(), Some(10));
    assert_eq!(pool.last_island_refined_bodies(), None);
    let dir = tempfile::tempdir()?;
    let path = dir.path().join("fine");
    pool.save_native_recovery(&path)?;
    drop(pool);
    let mut pool = PoolNativeRecovery::open(&path)?.resume(worker())?;
    assert_eq!(pool.last_substeps(), Some(10));
    assert_eq!(pool.last_island_refined_bodies(), None);
    for _ in 0..8 {
        pool.advance_adaptive_with_separation(ZERO, &[])?;
        regular.advance_adaptive_in_field(ZERO, &[])?;
        assert_eq!(
            pool.frame().workers[0].snapshot,
            regular.frame().workers[0].snapshot
        );
        assert_eq!(pool.last_substeps(), regular.last_substeps());
        assert_eq!(pool.last_island_refined_bodies(), None);
    }
    assert_eq!(regular.frame().workers[0].components.len(), 1);
    assert_eq!(pool.frame().workers[0].components.len(), 2);
    assert_eq!(pool.last_substeps(), Some(1));
    let path = dir.path().join("coarse");
    pool.save_native_recovery(&path)?;
    let mut resumed = PoolNativeRecovery::open(path)?.resume(worker())?;
    assert_eq!(resumed.last_substeps(), Some(1));
    assert_eq!(resumed.last_island_refined_bodies(), None);
    assert_eq!(
        pool.advance_adaptive_with_separation(ZERO, &[])?,
        resumed.advance_adaptive_with_separation(ZERO, &[])?
    );
    Ok(())
}

#[test]
fn pair_replays_both_uniform_rates_from_native_and_action_archives() -> Result<()> {
    let peers = [
        WorkerProcess::spawn(worker(), 3003, 0, &glancing())?,
        WorkerProcess::spawn(worker(), 3003, 1, &[])?,
    ];
    let mut pair = Coordinator::new_recorded(3003, peers, 64)?;
    let dir = tempfile::tempdir()?;
    for tick in 1..=9 {
        pair.advance_adaptive_with_separation(ZERO, &[])?;
        if tick == 1 || tick == 9 {
            let rate = if tick == 1 { 10 } else { 1 };
            assert_eq!(pair.last_substeps(), Some(rate));
            assert_eq!(pair.last_island_refined_bodies(), None);
            let native_path = dir.path().join(format!("native-{tick}"));
            let journal = dir.path().join(format!("actions-{tick}"));
            pair.save_native_recovery(&native_path)?;
            pair.save_recovery(&journal)?;
            let mut from_native = NativeRecoveryPlan::open(native_path)?.resume(worker())?;
            let mut replay = RecoveryPlan::open(journal)?
                .resume(|run, slot, bodies| WorkerProcess::spawn(worker(), run, slot, bodies))?;
            for resumed in [&from_native, &replay] {
                assert_eq!(resumed.frame(), pair.frame());
                assert_eq!(resumed.last_substeps(), Some(rate));
                assert_eq!(resumed.last_island_refined_bodies(), None);
            }
            assert_eq!(
                from_native.advance_adaptive_with_separation(ZERO, &[])?,
                replay.advance_adaptive_with_separation(ZERO, &[])?
            );
        }
    }
    assert_eq!(pair.frame().workers[0].components.len(), 2);
    Ok(())
}

#[test]
#[ignore = "requires the archived native and action fixtures"]
fn resumes_preexisting_pair_and_pool_archives_before_switching_policy() -> Result<()> {
    let artifacts = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../artifacts");
    let pair = artifacts.join("multifidelity-native-recovery/files");
    let mut native = NativeRecoveryPlan::open(pair.join("tick-150.gznative"))?.resume(worker())?;
    let mut recorded =
        RecoveryPlan::open(pair.join("tick-150.gzrec"))?.resume_with_ground(|r, w, b, s, g| {
            WorkerProcess::spawn_with_ground(worker(), r, w, b, s, g)
        })?;
    assert_eq!(native.frame(), recorded.frame());
    assert_eq!(native.last_substeps(), recorded.last_substeps());
    assert_eq!(
        native.last_island_refined_bodies(),
        recorded.last_island_refined_bodies()
    );
    assert_eq!(
        native.advance_adaptive_with_separation(ZERO, &[])?,
        recorded.advance_adaptive_with_separation(ZERO, &[])?
    );
    assert!(native.last_island_refined_bodies().is_none());
    for source in [
        "multifidelity-distributed-refinement/encounter/native-barrier.gzpool",
        "multifidelity-island-refinement/encounter/native-barrier.gzpool",
    ] {
        let path = artifacts.join(source);
        let mut original = PoolNativeRecovery::open(&path)?.resume(worker())?;
        let mut split = PoolNativeRecovery::open(path)?.resume(worker())?;
        assert_eq!(original.frame(), split.frame());
        assert_eq!(original.last_substeps(), split.last_substeps());
        assert_eq!(
            original.last_island_refined_bodies(),
            split.last_island_refined_bodies()
        );
        let field = [0., 0., -9.81];
        original.advance_adaptive_in_field(field, &[])?;
        split.advance_adaptive_with_separation(field, &[])?;
        assert_eq!(original.last_substeps(), split.last_substeps());
        assert!(split.last_island_refined_bodies().is_none());
        for (a, b) in original.frame().workers.iter().zip(&split.frame().workers) {
            assert_eq!(a.snapshot, b.snapshot);
        }
    }
    Ok(())
}
