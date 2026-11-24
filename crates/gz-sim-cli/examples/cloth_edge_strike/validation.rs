// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn malformed_scene_or_missing_material_display_is_rejected() -> Result<()> {
    let dir = tempfile::tempdir()?;
    assert!(export(dir.path(), "<sdf").is_err());
    assert!(export(dir.path(), &SDF.replace("columns=\"2\"", "columns=\"9\"")).is_err());
    let renamed_node = SDF
        .replace("name=\"p0\"", "name=\"node0\"")
        .replace("ridge::p0", "ridge::node0");
    assert!(
        export(dir.path(), &renamed_node)
            .unwrap_err()
            .to_string()
            .contains("missing physical node color")
    );
    let renamed_cloth = SDF
        .replace("<cloth name=\"hit\"", "<cloth name=\"spare\"")
        .replace("material=\"hit\"", "material=\"spare\"");
    assert!(
        export(dir.path(), &renamed_cloth)
            .unwrap_err()
            .to_string()
            .contains("missing display model")
    );
    assert!(!dir.path().join("replay").exists());
    Ok(())
}

#[test]
fn output_failures_are_reported_to_the_caller() -> Result<()> {
    for name in ["replay", "trace.json"] {
        let dir = tempfile::tempdir()?;
        fs::create_dir(dir.path().join(name))?;
        fs::write(dir.path().join(name).join("existing"), b"keep")?;
        assert!(export(dir.path(), SDF).is_err());
        assert_eq!(fs::read(dir.path().join(name).join("existing"))?, b"keep");
    }
    Ok(())
}
