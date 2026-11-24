// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(target_os = "linux")]
#[path = "../examples/sphere_pair_island.rs"]
mod experiment;

use clap::Parser;
use serde_json::Value;
use std::{error::Error, fs};

#[test]
fn independent_warm_sphere_pair_migrates_recovers_and_joins_hinge() -> Result<(), Box<dyn Error>> {
    let output = tempfile::tempdir()?;
    let ledger = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let dir = output.path().join("record");
    experiment::qualify(experiment::Qualification::try_parse_from([
        "sphere_pair_island",
        "--worker",
        env!("CARGO_BIN_EXE_physics_worker"),
        "--workers",
        "3",
        "--summary-only",
        "--ticks",
        "1220",
        "--ledger-directory",
        ledger.path().to_str().unwrap(),
        dir.to_str().unwrap(),
    ])?)?;
    let result: Value = serde_json::from_slice(&fs::read(dir.join("summary.json"))?)?;
    let events = result["events"].as_array().unwrap();
    let action = |kind: &str| events.iter().find(|v| v["kind"] == kind).unwrap();
    assert_eq!(result["states_compared"], 1221);
    assert!(
        action("hinge_released")["reaction"]["active"]
            .as_bool()
            .unwrap()
    );
    assert_eq!(
        action("pair_handoff")["receipt"]["hops"]
            .as_array()
            .unwrap()
            .len(),
        2
    );
    assert!(
        action("warm_merge")["prior_reaction"]["warmstart_normal_impulse"]
            .as_f64()
            .unwrap()
            > 0.
    );
    assert_eq!(
        events.iter().filter(|e| e["kind"] == "worker_kill").count(),
        2
    );
    assert_eq!(result["restarts"].as_array().unwrap().len(), 4);
    Ok(())
}
