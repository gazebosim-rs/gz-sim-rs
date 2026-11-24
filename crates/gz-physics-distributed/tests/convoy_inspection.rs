// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/inspection_series.rs"]
mod fixture;
use serde_json::Value;
use std::{error::Error, fs, path::Path, process::Command};

#[test]
fn comparison_locates_placement_then_pending_input_then_different_physics()
-> Result<(), Box<dyn Error + Send + Sync>> {
    let tmp = tempfile::tempdir()?;
    let data = tmp.path().join("series");
    fixture::generate(Path::new(env!("CARGO_BIN_EXE_physics_worker")), &data)?;
    let inspect = env!("CARGO_BIN_EXE_convoy_inspect");
    let manifest = data.join("series.json");
    let run = || {
        Command::new(inspect)
            .arg("--series")
            .arg(&manifest)
            .output()
    };
    let result = run()?;
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let series: Value = serde_json::from_slice(&result.stdout)?;
    assert_eq!(series["rows_checked"], 7);
    assert_eq!(series["first_archive_difference"]["row"], 1);
    assert_eq!(
        series["first_archive_difference"]["comparison"]["classification"],
        "placement_change"
    );
    assert_eq!(series["first_logical_difference"]["row"], 3);
    assert_eq!(
        series["first_logical_difference"]["comparison"]["classification"],
        "pending_input_change"
    );
    assert_eq!(series["first_physical_difference"]["row"], 4);
    let difference = &series["first_physical_difference"]["comparison"];
    assert_eq!(difference["classification"], "physical_divergence");
    assert_eq!(difference["left"]["tick"], 151);
    assert!(
        difference["physical_errors"]["max_linear_velocity_m_s"]
            .as_f64()
            .unwrap()
            > 0.
    );
    let ref_file = data.join("reference-151.gzmultisphere");
    let same = Command::new(inspect)
        .arg("--compare")
        .arg(&ref_file)
        .arg(&ref_file)
        .output()?;
    assert!(same.status.success());
    assert_eq!(
        serde_json::from_slice::<Value>(&same.stdout)?["classification"],
        "identical_archive"
    );
    let mut pairs: Vec<Value> = serde_json::from_slice(&fs::read(&manifest)?)?;
    pairs.swap(5, 6);
    fs::write(&manifest, serde_json::to_vec(&pairs)?)?;
    let backwards = run()?;
    assert!(!backwards.status.success());
    assert!(
        String::from_utf8_lossy(&backwards.stderr).contains("backwards checkpoint clock at row 7")
    );
    Ok(())
}
