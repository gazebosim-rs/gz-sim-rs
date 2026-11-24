// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(target_os = "linux")]
#[path = "../examples/grounded_hinge.rs"]
mod experiment;
use clap::Parser;
use serde_json::Value;
use std::{error::Error, fs};

#[test]
fn articulated_floor_reactions_match_through_loaded_faults() -> Result<(), Box<dyn Error>> {
    let root = tempfile::tempdir()?;
    let path = root.path().join("run");
    let args = experiment::Args::try_parse_from([
        "grounded_hinge",
        "--worker",
        env!("CARGO_BIN_EXE_physics_worker"),
        "--workers",
        "3",
        "--reverse",
        path.to_str().unwrap(),
    ])?;
    experiment::qualify(args)?;
    let r: Value = serde_json::from_slice(&fs::read(path.join("summary.json"))?)?;
    assert_eq!(r["states_compared"], 641);
    assert_eq!(r["advancing_images_exact"], 640);
    assert_eq!(r["counterfactuals"].as_array().unwrap().len(), 6);
    let events = r["events"].as_array().unwrap();
    for name in ["transfer", "kill", "replaced", "resume"] {
        assert_eq!(events.iter().filter(|e| e["kind"] == name).count(), 2);
    }
    Ok(())
}
