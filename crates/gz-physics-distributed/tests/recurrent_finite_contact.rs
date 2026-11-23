// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/recurrent_finite_contact.rs"]
mod recurrent;
use std::{error::Error, fs, path::Path};

#[test]
#[ignore = "continuous 20,100-tick real-worker qualification and two full disk restores"]
fn recovered_loaded_pairs_continue_through_a_second_durable_checkpoint()
-> Result<(), Box<dyn Error>> {
    let root = tempfile::tempdir()?;
    let out = root.path().join("qualification");
    recurrent::qualify(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        200,
        false,
        false,
        &out,
    )?;
    let report: serde_json::Value = serde_json::from_slice(&fs::read(out.join("summary.json"))?)?;
    assert_eq!(report["counts"]["ticks"], 20100);
    assert_eq!(report["counts"]["landings"], 200);
    assert_eq!(report["counts"]["migrations"], 50);
    assert_eq!(report["counts"]["lost_acks"], 50);
    assert_eq!(report["counts"]["replacements"], 10);
    assert_eq!(report["counts"]["restores"], 2);
    assert_eq!(report["rolling_qualification"]["checked"], 191);
    assert!(out.join("last-checkpoint.gzpool").is_file());
    Ok(())
}
