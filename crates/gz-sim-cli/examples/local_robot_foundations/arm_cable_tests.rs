// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn cable_lifts_on_both_pads_and_the_open_gripper_misses_on_each_clock() -> Result<()> {
    let out = tempfile::tempdir()?;
    run(out.path())?;
    let report: Value = serde_json::from_slice(&fs::read(out.path().join("comparison.json"))?)?;
    assert!(report["order_change_in_lift_m"].as_f64().unwrap() < 1e-7);
    assert!(
        report["max_order_change_in_jaw_support_n"]
            .as_f64()
            .unwrap()
            < 0.0162
    );
    let trials = report["trials"].as_array().unwrap();
    assert_eq!(trials.len(), 6);
    for (pair, grid) in trials.chunks_exact(2).zip([1, 2, 4]) {
        for trial in pair {
            assert_eq!(trial["substeps"], grid);
        }
        let [held, open] = pair else { unreachable!() };
        assert_eq!(held["closed"], true);
        assert_eq!(open["closed"], false);
        assert!(held["mean_hold_lift_m"].as_f64().unwrap() > 0.10);
        assert!(open["mean_hold_lift_m"].as_f64().unwrap().abs() < 1e-6);
        let up = held["mean_hold_jaw_up_n"].as_array().unwrap();
        assert!(up.iter().all(|n| n.as_f64().unwrap() > 0.));
        // Compute the gravitational force from the authored mass independently.
        let expected = 11. * 0.002 * 9.81;
        assert!(
            (up.iter().map(|n| n.as_f64().unwrap()).sum::<f64>() - expected).abs() < expected / 10.
        );
    }
    for label in ["held", "open"] {
        let dir = out.path().join(label);
        assert!(dir.join("replay/manifest.json").is_file());
        let frames: Vec<Value> =
            serde_json::from_slice(&fs::read(dir.join("particle-frames.json"))?)?;
        assert_eq!(frames.len(), 137);
        for (i, frame) in frames.iter().enumerate() {
            assert_eq!(frame["time_ns"], i as u64 * 25_000_000);
            assert_eq!(frame["positions"].as_array().unwrap().len(), COUNT);
        }
        let meta: Value = serde_json::from_slice(&fs::read(dir.join("frames.json"))?)?;
        assert_eq!(meta["particle_visuals"][10], sphere_name(10));
    }
    Ok(())
}

#[test]
fn a_table_supported_cable_is_rejected_as_a_grasp() -> Result<()> {
    let mut missed = cycle(1, false, false, None)?;
    missed.qualify()?;
    missed.closed = true;
    assert!(missed.qualify().is_err());
    Ok(())
}

#[test]
fn invalid_clock_missing_material_and_failed_archive_write_report_errors() -> Result<()> {
    assert!(cycle(0, false, false, None).is_err());
    let robot = Recording::new(arm::SCENE, "arm.sdf", "default")?;
    assert!(sample(&robot.sim).is_err());
    assert!(half_size(&robot.sdf, "arm", "missing-pad").is_err());
    let out = tempfile::tempdir()?;
    fs::create_dir(out.path().join("particle-frames.json"))?;
    assert!(cycle(1, false, false, Some(out.path())).is_err());
    Ok(())
}

#[test]
fn grid_and_jaw_order_disagreements_are_reported_independently() {
    let mut rows: [Metrics; 6] = std::array::from_fn(|_| Metrics::default());
    let mut reversed = Metrics::default();
    assert!(comparison(&rows[..2], &reversed).is_err());
    rows[0].mean_hold_tip_sag_m[0] = 0.006;
    assert!(comparison(&rows, &reversed).is_err());
    rows[0].mean_hold_tip_sag_m[0] = 0.;
    reversed.mean_hold_jaw_up_n[1] = 0.01;
    assert!(comparison(&rows, &reversed).is_err());
}

#[test]
fn a_hard_squeeze_and_an_unmeasured_bench_cannot_be_reported_as_a_lift() -> Result<()> {
    let xml = scene(arm::SCENE, 1, false)?;
    let hard = xml.replace("normal_compliance=\"0.001\"", "normal_compliance=\"0\"");
    let error = cycle_with_scene(&hard, 1, true, false, None).unwrap_err();
    assert!(error.to_string().contains("UnresolvedSweep"));
    let no_bench = xml.replace("<plane point=\"0 0 0.12\" normal=\"0 0 1\"/>", "");
    let error = cycle_with_scene(&no_bench, 1, false, false, None).unwrap_err();
    assert!(error.to_string().contains("missing jaw or bench reaction"));
    Ok(())
}

#[test]
fn a_changed_robot_fixture_cannot_silently_omit_the_support_or_material() {
    for tag in [
        "<model name=\"parcel\">",
        "<model name=\"source\">",
        "</link>",
        "</world>",
    ] {
        let damaged = arm::SCENE.replace(tag, "");
        assert!(scene(&damaged, 2, false).is_err(), "missing {tag}");
    }
}

#[test]
fn unwritable_archive_metadata_and_final_report_propagate_io_errors() -> Result<()> {
    let out = tempfile::tempdir()?;
    fs::create_dir(out.path().join("frames.json"))?;
    assert!(cycle(1, false, false, Some(out.path())).is_err());
    fs::create_dir(out.path().join("comparison.json"))?;
    assert!(run(out.path()).is_err());
    Ok(())
}
