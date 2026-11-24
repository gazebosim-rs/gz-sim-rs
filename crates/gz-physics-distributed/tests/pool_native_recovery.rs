// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/pool_crash.rs"]
mod demo;
use gz_physics_distributed::{
    physics::{SimulationStep, fixture},
    pool::{NativeContactSnapshot, PoolNativeRecovery, WorkerPool},
};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};
fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}

#[test]
#[ignore = "private child for the coordinator SIGKILL qualification"]
fn pool_coordinator() {
    let dir = PathBuf::from(std::env::var_os("GZ_POOL_CRASH_DIR").unwrap());
    let profile = match std::env::var("GZ_POOL_CRASH_MODE").unwrap().as_str() {
        "adaptive" => demo::Profile::Adaptive,
        "curved-ground" => demo::Profile::CurvedGround,
        "islands" => demo::Profile::Islands,
        "diagonal-lanes" => demo::Profile::DiagonalLanes,
        "coarse" => demo::Profile::Coarse,
        _ => unreachable!(),
    };
    match std::env::var("GZ_POOL_CRASH_ROLE").unwrap().as_str() {
        "original" => demo::original(worker(), &dir, profile).unwrap(),
        "replacement" => demo::replacement(worker(), &dir, profile).unwrap(),
        _ => unreachable!(),
    }
}
#[test]
fn real_four_worker_coordinator_death_preserves_loaded_contacts_and_global_input_order()
-> demo::Result<()> {
    check_crash("coarse")
}

#[test]
fn real_coordinator_death_inside_refined_barrier_recovers_the_last_complete_tick()
-> demo::Result<()> {
    check_crash("adaptive")
}

#[test]
fn restored_sphere_pairs_retain_ground_support_after_a_coordinator_crash() -> demo::Result<()> {
    check_crash("curved-ground")
}

#[test]
fn mixed_island_rates_and_pending_inputs_survive_coordinator_sigkill() -> demo::Result<()> {
    check_crash("islands")
}

#[test]
fn separate_diagonal_contact_islands_survive_coordinator_sigkill() -> demo::Result<()> {
    check_crash("diagonal-lanes")
}

fn check_crash(mode: &str) -> demo::Result<()> {
    let dir = tempfile::tempdir()?;
    let role = |name: &str| -> demo::Result<Command> {
        let mut c = Command::new(std::env::current_exe()?);
        c.args(["--exact", "--ignored", "pool_coordinator", "--nocapture"])
            .env("GZ_POOL_CRASH_ROLE", name)
            .env("GZ_POOL_CRASH_MODE", mode)
            .env("GZ_POOL_CRASH_DIR", dir.path());
        Ok(c)
    };
    demo::supervise(role("original")?, role("replacement")?, dir.path())?;
    let native = dir.path().join("pool.native");
    let cut = NativeContactSnapshot::open(&native)?;
    let recovery = PoolNativeRecovery::open(&native)?;
    assert_eq!(cut.frame, *recovery.frame());
    assert!(cut.population.is_none());
    let finite = recovery.finite_contact_patches()?;
    let ground = recovery.ground_contact_patches()?;
    assert_eq!(cut.patches.len(), cut.frame.workers.len());
    for (owner, patches) in cut.patches.iter().enumerate() {
        assert_eq!(patches.finite, finite[owner]);
        assert_eq!(patches.ground, ground[owner]);
    }
    let result: Value = serde_json::from_slice(&fs::read(dir.path().join("result.json"))?)?;
    assert_eq!(result["resumed_exact"], 90);
    assert_eq!(result["frames"], 116);
    if mode == "diagonal-lanes" {
        assert_eq!(result["verified_separate_pair_barriers"], 91);
    }
    Ok(())
}

#[test]
fn independently_authentic_worker_images_must_agree_on_the_same_ownership_barrier()
-> demo::Result<()> {
    let dir = tempfile::tempdir()?;
    let temp = |name: &str| dir.path().join(name);
    let publish = |value: &Value| -> demo::Result<()> {
        let body = serde_json::to_vec(value)?;
        let mut bytes = b"GZPOOL01".to_vec();
        bytes.extend_from_slice(&Sha256::digest(&body));
        bytes.extend(body);
        fs::write(temp("mixed"), bytes)?;
        Ok(())
    };
    for width in [3, 8] {
        let initial = temp(&format!("initial-{width}"));
        let moved = temp(&format!("moved-{width}"));
        let mut seeds = vec![Vec::new(); width];
        seeds[0] = vec![fixture()[1].clone()];
        let mut p = WorkerPool::spawn(worker(), 17703, seeds, SimulationStep::Ms2, None, 15)?;
        p.save_native_recovery(&initial)?;
        p.transfer_component(2, 1)?;
        p.queue_impulse(2, [0.03, 0., 0.])?;
        p.save_native_recovery(&moved)?;
        let old: Value = serde_json::from_slice(&fs::read(initial)?[40..])?;
        let data: Value = serde_json::from_slice(&fs::read(&moved)?[40..])?;
        for (slot, from) in [(0, &old), (1, &old)] {
            let mut mixed = data.clone();
            mixed["images"][slot] = from["images"][slot].clone();
            publish(&mixed)?;
            assert!(PoolNativeRecovery::open(temp("mixed")).is_err());
        assert!(NativeContactSnapshot::open(temp("mixed")).is_err());
        }
        // Shape and native seals are individually valid, but a located body with a
        // nonzero generation must have a matching release fence on another worker.
        let mut missing = data.clone();
        missing["images"][0] = old["images"][2].clone();
        publish(&missing)?;
        assert!(PoolNativeRecovery::open(temp("mixed")).is_err());
        for edit in ["width", "queue", "limit", "run", "corrupt"] {
            let mut damaged = data.clone();
            match edit {
                "width" => {
                    damaged["images"].as_array_mut().unwrap().pop();
                }
                "queue" => damaged["queued"][0]["id"] = 999.into(),
                "limit" => damaged["journal_limit"] = 0.into(),
                "run" => damaged["images"][1]["run"] = 999.into(),
                "corrupt" => damaged["images"][0]["digest"][0] = 999.into(),
                _ => unreachable!(),
            }
            publish(&damaged)?;
            assert!(PoolNativeRecovery::open(temp("mixed")).is_err());
        }
        let mut corrupt = fs::read(&moved)?;
        *corrupt.last_mut().unwrap() ^= 1;
        fs::write(temp("mixed"), corrupt)?;
        assert!(PoolNativeRecovery::open(temp("mixed")).is_err());
        let plan = PoolNativeRecovery::open(&moved)?;
        assert_eq!(plan.frame(), p.frame());
        let expected = p.advance()?.clone();
        drop(p);
        let mut q = plan.resume(worker())?;
        assert_eq!(q.advance()?, &expected);
    }
    Ok(())
}

#[test]
fn valid_images_from_different_migration_histories_cannot_repeat_a_release_generation()
-> demo::Result<()> {
    let dir = tempfile::tempdir()?;
    let snapshot = |via: u8, name: &str| -> demo::Result<Value> {
        let mut seeds = vec![Vec::new(); 5];
        seeds[0] = vec![fixture()[1].clone()];
        let mut p = WorkerPool::spawn(worker(), 93178, seeds, SimulationStep::Ms5, None, 18)?;
        for owner in [via, 3, 2] {
            p.transfer_component(2, owner)?;
        }
        let path = dir.path().join(name);
        p.save_native_recovery(&path)?;
        Ok(serde_json::from_slice(&fs::read(path)?[40..])?)
    };
    let mut first = snapshot(1, "through-one")?;
    let second = snapshot(4, "through-four")?;
    // Shards 1 and 4 now each carry a genuine second release of body 2.
    // Both native seals, the active third generation and the latest serial look valid.
    first["images"][4] = second["images"][4].clone();
    let mut bytes = b"GZPOOL01".to_vec();
    let body = serde_json::to_vec(&first)?;
    bytes.extend_from_slice(&Sha256::digest(&body));
    bytes.extend(body);
    let mixed = dir.path().join("fork");
    fs::write(&mixed, bytes)?;
    let error = PoolNativeRecovery::open(mixed)
        .err()
        .expect("forked custody was accepted");
    assert!(
        error
            .to_string()
            .contains("duplicate pool release generation"),
        "{error}"
    );
    Ok(())
}

#[test]
fn concurrent_image_reads_fence_both_failed_owners_before_retrying_the_committed_archive()
-> demo::Result<()> {
    let dir = tempfile::tempdir()?;
    let seed = dir.path().join("before.gzpool");
    let retry = dir.path().join("retry.gzpool");
    let mut pool = WorkerPool::spawn(
        worker(),
        8112,
        vec![vec![fixture()[1].clone()], vec![], vec![], vec![]],
        SimulationStep::Ms10,
        None,
        32,
    )?;
    pool.advance()?;
    pool.transfer_component(2, 1)?;
    pool.queue_impulse(2, [0.2, -0., 0.])?;
    pool.save_native_recovery(&seed)?;
    let frame = pool.frame().clone();
    let journal = pool.retained_entries();
    let pids: Vec<_> = (0..4).map(|w| pool.worker_pid(w).unwrap()).collect();
    pool.terminate_worker(0)?;
    pool.terminate_worker(1)?;
    assert!(pool.save_native_recovery(&retry).is_err());
    assert!(!retry.exists());
    assert_eq!(pool.frame(), &frame);
    assert_eq!(pool.retained_entries(), journal);
    pool.save_native_recovery(&retry)?;
    assert_eq!(fs::read(&seed)?, fs::read(&retry)?);
    for (w, prior) in pids.into_iter().enumerate() {
        assert_eq!(pool.worker_pid(w as u8) != Some(prior), w < 2);
    }
    let mut old = PoolNativeRecovery::open(seed)?.resume(worker())?;
    for _ in 0..8 {
        assert_eq!(pool.advance()?, old.advance()?);
    }
    Ok(())
}
