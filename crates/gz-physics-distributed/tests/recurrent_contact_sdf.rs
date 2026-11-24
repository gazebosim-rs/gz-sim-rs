// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::{error::Error, fs};
#[path = "../examples/recurrent_contact_sdf.rs"]
mod scene;
#[test]
fn sdf_link_wrenches_reset_and_retry_the_real_finite_landing_on_both_placements()
-> Result<(), Box<dyn Error>> {
    use clap::Parser;
    let root = tempfile::tempdir()?;
    let mut hashes = vec![];
    for canonical in [false, true] {
        let path = root.path().join(if canonical {
            "canonical.json"
        } else {
            "split.json"
        });
        let mut args = vec![
            "recurrent_contact_sdf",
            "--worker",
            env!("CARGO_BIN_EXE_physics_worker"),
            "--cycles",
            "10",
        ];
        if canonical {
            args.push("--canonical-reference");
        }
        args.push(path.to_str().unwrap());
        scene::run(scene::Args::try_parse_from(args)?)?;
        let stats: serde_json::Value = serde_json::from_slice(&fs::read(&path)?)?;
        assert_eq!(stats["ticks"], 1100);
        assert_eq!(stats["rolling_qualification"]["checked"], 1);
        assert_eq!(stats["public_pose_velocity_checks"], 4400);
        assert_eq!(stats["landing_retry"][0]["tick"], 146);
        hashes.push((
            stats["trajectory_sha256"].clone(),
            stats["input_sha256"].clone(),
            stats["public_ecs_sha256"].clone(),
        ));
    }
    assert_eq!(hashes[0], hashes[1]);
    Ok(())
}
