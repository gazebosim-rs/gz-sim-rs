// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/durable_run.rs"]
mod run;
use std::{path::Path, process::Command};

#[test]
#[ignore = "spawned by the coordinator crash/recovery test"]
fn coordinator_process() {
    let mode = std::env::var("GZ_RECOVERY_ROLE").unwrap();
    let dir = std::env::var_os("GZ_RECOVERY_OUTPUT").unwrap();
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    match mode.as_str() {
        "original" => run::original(worker, Path::new(&dir)).unwrap(),
        "replacement" => run::replacement(worker, Path::new(&dir)).unwrap(),
        _ => unreachable!(),
    }
}
#[test]
fn killed_coordinator_fences_workers_and_resumes_last_published_contact() {
    let dir = tempfile::tempdir().unwrap();
    let executable = std::env::current_exe().unwrap();
    let child = |role: &str| {
        let mut cmd = Command::new(&executable);
        cmd.args(["--ignored", "--exact", "coordinator_process", "--nocapture"])
            .env("GZ_RECOVERY_ROLE", role)
            .env("GZ_RECOVERY_OUTPUT", dir.path());
        cmd
    };
    run::supervise(child("original"), child("replacement"), dir.path()).unwrap();
    let v: serde_json::Value =
        serde_json::from_reader(std::fs::File::open(dir.path().join("verification.json")).unwrap())
            .unwrap();
    assert!(v["post_recovery_ticks"].as_u64().unwrap() > 50);
    assert!(v["replay_frames"].as_u64().unwrap() > 40);
}
