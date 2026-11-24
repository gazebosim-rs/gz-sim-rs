// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn invalid_hinge_coefficients_fail_during_world_admission() {
    for value in ["-0.01", "sticky"] {
        let xml = WORLD.replace(
            "bend_damping=\"0.08\"",
            &format!("bend_damping=\"{value}\""),
        );
        assert!(Recording::new(&xml, "invalid.sdf", "default").is_err());
    }
}

#[test]
fn measurement_requires_each_named_sample() {
    let xml = WORLD.replace("elastic", "replacement");
    let r = Recording::new(&xml, "renamed.sdf", "default").unwrap();
    assert_eq!(
        positions(&r.sim).unwrap_err().to_string(),
        "missing measured cloth"
    );
}

#[test]
fn preserves_io_errors_for_a_partial_material_recording() -> Result<()> {
    let temp = tempfile::tempdir()?;
    for filename in [
        "replay",
        "particle-frames.json",
        "damping.json",
        "world.sdf",
    ] {
        let dir = temp.path().join(filename.replace('.', "_"));
        fs::create_dir_all(&dir)?;
        let obstacle = dir.join(filename);
        if filename == "replay" {
            fs::write(&obstacle, "existing file")?;
        } else {
            fs::create_dir(&obstacle)?;
        }
        assert!(run(&dir).is_err(), "overwrote {filename}");
    }
    Ok(())
}
