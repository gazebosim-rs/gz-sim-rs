// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/rough_mobility.rs"]
mod mobility;
use std::{error::Error, fs, path::Path};

#[test]
fn measured_tangent_closes_a_warm_sliding_to_rolling_contact_in_two_reference_frames()
-> Result<(), Box<dyn Error>> {
    let temp = tempfile::tempdir()?;
    let mut hashes = vec![];
    for packed in [false, true] {
        let out = temp.path().join(if packed { "packed" } else { "json" });
        mobility::qualify(
            Path::new(env!("CARGO_BIN_EXE_physics_worker")),
            2,
            packed,
            false,
            &out,
        )?;
        let mut row = vec![];
        for frame in ["stationary", "moving"] {
            let summary: serde_json::Value =
                serde_json::from_slice(&fs::read(out.join(frame).join("summary.json"))?)?;
            assert_eq!(summary["ticks"], 600);
            assert_eq!(summary["events"].as_array().unwrap().len(), 10);
            assert_eq!(summary["observed"]["point_model_different_body_frames"], 0);
            assert_eq!(
                summary["observed"]["point_model_different_normal_frames"],
                0
            );
            assert_eq!(summary["observed"]["saturated_observations"], 143);
            row.push(summary["trajectory_sha256"].clone());
        }
        hashes.push(row);
    }
    assert_eq!(hashes[0], hashes[1]);
    Ok(())
}
