// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reopen authored input on both signs and at an already completed epoch boundary.
#![cfg(target_os = "linux")]
#[path = "../examples/convoy_lifecycle_soak.rs"]
mod soak;

use clap::Parser;
use gz_physics_distributed::multi_sphere::MultiSpherePool;
use gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec;
use serde_json::Value;
use std::{error::Error, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/distributed_sphere_pair_island.sdf");
const ARCHIVE: &str = "barrier.gzmultisphere";

fn verify(
    source: &Path,
    ledger_root: &Path,
    epoch: u64,
    tick: u64,
    half: Option<f64>,
    complete: u64,
) -> Result<()> {
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let out = ledger_root.join(format!("resume-{epoch}-{tick}"));
    let seed = fs::read(source)?;
    let mut args = vec![
        "convoy_lifecycle_soak".as_ref(),
        "--worker".as_ref(),
        worker.as_os_str(),
        "--ledger-root".as_ref(),
        ledger_root.as_os_str(),
        "--epochs".as_ref(),
        "2".as_ref(),
        "--from-cut".as_ref(),
        source.as_os_str(),
    ];
    if complete == 0 {
        args.push(std::ffi::OsStr::new("--until"));
        args.push(std::ffi::OsStr::new("1"));
    }
    args.push(out.as_os_str());
    soak::exercise(soak::Args::try_parse_from(args)?)?;
    let read =
        |name: &str| -> Result<Value> { Ok(serde_json::from_slice(&fs::read(out.join(name))?)?) };
    let continuation = read("continuation.json")?;
    assert_eq!(
        continuation["source_pending_effort"],
        serde_json::json!(half)
    );
    assert_eq!(continuation["verified_advances"], 1220 - tick);
    let summary = read("summary.json")?;
    assert_eq!(summary["source_tick"], tick);
    assert_eq!(summary["source_epoch"], epoch);
    assert_eq!(summary["epochs"], complete);
    assert_eq!(summary["steps"], (1220 - tick) + complete * 1220);
    assert_eq!(summary["schedule_tick"], (epoch + complete + 1) * 1220);
    assert_eq!(summary["loaded_transfers"], complete);
    assert_eq!(fs::read(source)?, seed, "continuation changed the source");
    Ok(())
}

#[test]
fn signed_staged_inputs_and_finished_epoch_resume_at_the_exact_next_interval() -> Result<()> {
    let dir = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let ledger = dir.path().join("seed");
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let spec = distributed_multi_sphere_spec(SDF, "convoy.sdf", "default")?;
    let joint = spec.hinge.joint_id;
    let mut pool = MultiSpherePool::create(worker, &ledger, 3, spec)?;
    let source = ledger.join(ARCHIVE);
    for (epoch, target, effort) in [(0, 149, 0.01), (1, 1213, -0.01)] {
        if epoch > 0 {
            pool.reset()?;
        }
        for tick in 0..target {
            pool.advance_with_effort_at(epoch, tick, joint, effort)?;
        }
        pool.queue_effort(joint, effort / 2.)?;
        verify(&source, dir.path(), epoch, target, Some(effort / 2.), 2)?;
        pool.advance_with_effort_at(epoch, target, joint, effort / 2.)?;
        for tick in (target + 1)..1220 {
            pool.advance_with_effort_at(epoch, tick, joint, effort)?;
        }
    }
    verify(&source, dir.path(), 1, 1220, None, 0)?;
    Ok(())
}
