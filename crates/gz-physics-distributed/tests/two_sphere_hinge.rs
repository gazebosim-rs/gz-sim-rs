// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(target_os = "linux")]
#[path = "../examples/two_sphere_hinge.rs"]
mod experiment;

use clap::Parser;
use serde_json::Value;
use std::{error::Error, fs};

#[test]
fn two_spheres_share_a_loaded_hinge_split_and_later_collide() -> Result<(), Box<dyn Error>> {
    let root = tempfile::tempdir()?;
    // This is a deterministic trajectory check. Process-level durability is
    // exercised by the same operations on either filesystem.
    let ledger = tempfile::tempdir_in("/dev/shm").or_else(|_| tempfile::tempdir())?;
    let output = root.path().join("result");
    experiment::qualify(experiment::Args::try_parse_from([
        "two_sphere_hinge",
        "--worker",
        env!("CARGO_BIN_EXE_physics_worker"),
        "--workers",
        "3",
        "--ledger-directory",
        ledger.path().to_str().unwrap(),
        "--summary-only",
        output.to_str().unwrap(),
    ])?)?;
    let report: Value = serde_json::from_slice(&fs::read(output.join("summary.json"))?)?;
    assert_eq!(report["states_compared"], 1801);
    assert_eq!(report["queued_commands"], 1800);
    assert_eq!(report["restarts"].as_array().unwrap().len(), 3);
    let loaded = report["events"]
        .as_array()
        .unwrap()
        .iter()
        .find(|e| e["kind"] == "loaded_handoff")
        .unwrap();
    for impulse in loaded["warmstart_impulses_ns"].as_array().unwrap() {
        assert!(impulse.as_f64().unwrap() > 0.);
    }
    let collision = report["events"]
        .as_array()
        .unwrap()
        .iter()
        .find(|e| e["kind"] == "sphere_collision")
        .unwrap();
    let owners: [u8; 3] = serde_json::from_value(collision["owners"].clone())?;
    assert_eq!(owners[1], owners[2]);
    assert_ne!(owners[0], owners[1]);
    assert_eq!(
        report["physical_digest"],
        "93047933c1f21911a9e9823be018dea7fb426ad480674533800df0beaf9da853"
    );
    Ok(())
}
