// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(target_os = "linux")]
#[path = "../examples/moving_hinge.rs"]
mod experiment;
use clap::Parser;
use serde_json::Value;
use std::{error::Error, fs};

#[test]
fn moving_sphere_strikes_distributed_hinge_and_releases_after_owner_loss()
-> Result<(), Box<dyn Error>> {
    let root = tempfile::tempdir()?;
    let run = root.path().join("run");
    experiment::qualify(experiment::Args::try_parse_from([
        "moving_hinge",
        "--worker",
        env!("CARGO_BIN_EXE_physics_worker"),
        "--workers",
        "3",
        "--reverse",
        run.to_str().unwrap(),
    ])?)?;
    let r: Value = serde_json::from_slice(&fs::read(run.join("summary.json"))?)?;
    let events = r["events"].as_array().unwrap();
    for (kind, n) in [
        ("join", 1),
        ("split", 1),
        ("independent_owner_kill", 1),
        ("loaded_handoff", 1),
        ("replaced", 2),
        ("released_resume", 1),
    ] {
        assert_eq!(events.iter().filter(|e| e["kind"] == kind).count(), n);
    }
    assert!(
        r["first_split_after_contact"].as_u64().unwrap() > r["first_loaded_tick"].as_u64().unwrap()
    );
    assert!(r["joined_steps_verified"].as_u64().unwrap() > 0);
    assert!(r["independent_steps_verified"].as_u64().unwrap() > 10);
    assert!(r["omitted_torque_delta_rad_s"].as_f64().unwrap() > 1e-8);
    Ok(())
}

#[test]
fn continuously_driven_hinge_releases_and_reopens_with_the_next_command_pending()
-> Result<(), Box<dyn Error>> {
    let root = tempfile::tempdir()?;
    let run = root.path().join("run");
    experiment::qualify(experiment::Args::try_parse_from([
        "moving_hinge",
        "--worker",
        env!("CARGO_BIN_EXE_physics_worker"),
        "--workers",
        "3",
        "--continuous=0.01",
        "--reverse",
        run.to_str().unwrap(),
    ])?)?;
    let summary: Value = serde_json::from_slice(&fs::read(run.join("summary.json"))?)?;
    assert_eq!(summary["queued_commands"], 1600);
    assert_eq!(summary["exact_states"], 1601);
    let last_contact = summary["last_active_tick"].as_u64().unwrap();
    let released = summary["first_split_after_contact"].as_u64().unwrap();
    assert!((1..250).contains(&(released - last_contact)));
    let events = summary["events"].as_array().unwrap();
    let handoff = events
        .iter()
        .find(|e| e["kind"] == "loaded_handoff")
        .unwrap();
    let restart = events
        .iter()
        .find(|e| e["kind"] == "released_resume")
        .unwrap();
    assert_eq!(handoff["torque_nm"], -0.01);
    assert_eq!(restart["queued_effort_nm"], -0.01);
    assert_eq!(restart["at"], released);
    Ok(())
}
