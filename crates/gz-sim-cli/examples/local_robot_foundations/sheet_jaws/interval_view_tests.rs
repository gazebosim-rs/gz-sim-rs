// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn grasp() -> Result<String> {
    super::super::super::timed_scene(1., 0.001, Case::default(), 1, true, 0.)
}

#[test]
fn signed_height_reference_and_native_pressure_points_share_the_authored_origin() -> Result<()> {
    let mut r = Recording::new(&world(&grasp()?), "markers.sdf", "default")?;
    let force = [[0., 1.], [2., 0.1], [0.25, 3.], [0.4, 1.6]];
    capture(&mut r, &force, None)?;
    let scene: Value = serde_json::from_str(&r.frames[0].0)?;
    let visuals = scene["visuals"].as_array().unwrap();
    let z = |name: &str| {
        visuals.iter().find(|v| v["name"] == name).unwrap()["translation"][2]
            .as_f64()
            .unwrap()
    };
    let base = z("default::source::body::squeeze_zero");
    for (level, sides) in force.iter().enumerate() {
        for (side, &n) in sides.iter().enumerate() {
            let got = z(&format!("{MARKER}{level}_{side}"));
            assert!((((got - base) / 0.06).sinh() - n).abs() < 1e-12);
        }
    }
    assert!(
        (z("default::source::body::squeeze_one_newton") - z(&format!("{MARKER}0_1"))).abs() < 1e-14
    );
    Ok(())
}

#[test]
fn damaged_instrumentation_cannot_publish_partial_or_nonfinite_gauges() -> Result<()> {
    let xml = world(&grasp()?);
    for label in [
        "missing_marker",
        "pressure_wrong",
        "pressure_x_0",
        "pressure_0_x",
        "pressure_4_0",
        "pressure_0_2",
    ] {
        let mut r = Recording::new(
            &xml.replace("pressure_0_0", label),
            "changed.sdf",
            "default",
        )?;
        assert!(capture(&mut r, &[[1.; 2]; 4], None).is_err());
        assert!(r.frames.is_empty());
    }
    let mut r = Recording::new(&xml, "nominal.sdf", "default")?;
    for invalid in [f64::NAN, -1.] {
        assert!(capture(&mut r, &[[invalid; 2]; 4], None).is_err());
        assert!(r.frames.is_empty());
    }
    Ok(())
}

#[test]
fn missing_frames_wrong_physical_origins_and_unwritable_observations_refuse() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let xml = world(&grasp()?);
    for frames in [vec![], vec![json!({})], vec![json!({"time_ns":10})]] {
        let mut r = Recording::new(&xml, "origin.sdf", "default")?;
        capture(&mut r, &[[0.; 2]; 4], None)?;
        assert!(save(r, dir.path(), &frames).is_err());
    }
    let clean = tempfile::tempdir()?;
    fs::create_dir(clean.path().join("particle-frames.json"))?;
    let mut r = Recording::new(&xml, "output.sdf", "default")?;
    capture(&mut r, &[[0.; 2]; 4], None)?;
    assert!(save(r, clean.path(), &[json!({"time_ns":0})]).is_err());
    Ok(())
}

#[test]
fn the_contact_indicator_follows_the_selected_physical_particle_and_clears() -> Result<()> {
    let mut r = Recording::new(&world(&grasp()?), "witness.sdf", "default")?;
    let point = [0.27, -0.03, 0.131];
    for target in [None, Some(point), None] {
        capture(&mut r, &[[0.; 2]; 4], target)?;
    }
    let readings = r
        .frames
        .iter()
        .map(|(s, _)| -> Result<Value> {
            let scene: Value = serde_json::from_str(s)?;
            Ok(scene["visuals"]
                .as_array()
                .unwrap()
                .iter()
                .find(|v| v["name"] == "default::source::body::table_contact")
                .unwrap()["translation"]
                .clone())
        })
        .collect::<Result<Vec<_>>>()?;
    assert_eq!(readings[1], json!(point));
    assert_eq!(readings[0], readings[2]);
    assert!(readings[0][2].as_f64().unwrap() < arm_sheet::BENCH);
    Ok(())
}
