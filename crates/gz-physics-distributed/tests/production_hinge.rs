// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/production_hinge.rs"]
mod experiment;
use clap::Parser;
use serde_json::Value;
use std::{error::Error, fs};
#[test]
fn ordinary_sdf_matches_signed_efforts_across_live_handoff_process_death_and_fresh_restart()
-> Result<(), Box<dyn Error>> {
    let root = tempfile::tempdir()?;
    let dir = root.path().join("hinge");
    let args = experiment::Args::try_parse_from([
        "production_hinge",
        "--worker",
        env!("CARGO_BIN_EXE_physics_worker"),
        "--workers",
        "3",
        "--reverse",
        dir.to_str().unwrap(),
    ])?;
    experiment::qualify(args)?;
    let result: Value = serde_json::from_slice(&fs::read(dir.join("summary.json"))?)?;
    assert_eq!(result["frames_checked"], 385);
    assert_eq!(result["exact_native_images_after_advance"], 384);
    assert_eq!(result["held_effort_controls"].as_array().unwrap().len(), 6);
    let events = result["events"].as_array().unwrap();
    for name in ["transfer", "disk_resume"] {
        assert_eq!(events.iter().filter(|e| e["kind"] == name).count(), 2);
    }
    #[cfg(target_os = "linux")]
    assert_eq!(
        events
            .iter()
            .filter(|e| e["kind"] == "owner_replaced")
            .count(),
        2
    );
    Ok(())
}
