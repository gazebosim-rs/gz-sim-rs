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
    assert_eq!(report["first_hinge_impacts"], serde_json::json!([205, 218]));
    assert_eq!(
        report["last_active_hinge_contacts"],
        serde_json::json!([338, 392])
    );
    assert_eq!(report["double_impact_tick"], 226);
    assert_eq!(report["partial_release_tick"], 466);
    assert_eq!(report["independent_again_tick"], 564);
    assert_eq!(report["sphere_impact_tick"], 1292);
    assert!(report["max_sphere_impulse_residual_ns"].as_f64().unwrap() < 1e-12);
    // Pinned with the authored principal frame preserved for massless offset colliders.
    assert_eq!(
        report["physical_digest"],
        "0e3a13c6066bd28a763ef8855cb51356c688be53f8ac5aa971baf3f0e5db3e6c"
    );
    Ok(())
}
