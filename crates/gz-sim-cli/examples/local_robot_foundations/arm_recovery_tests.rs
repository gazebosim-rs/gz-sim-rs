// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn the_same_tactile_guard_transfers_to_a_rotating_arm() -> Result<()> {
    let output = tempfile::tempdir()?;
    run(output.path())?;
    let runs: Vec<Value> = serde_json::from_slice(&fs::read(output.path().join("matrix.json"))?)?;
    assert_eq!(runs.len(), 30);
    let sample = |mass: f64, mu: f64, feedback: bool| {
        runs.iter()
            .find(|d| d["mass_kg"] == mass && d["friction"] == mu && d["feedback"] == feedback)
            .unwrap()
    };
    let nominal = sample(0.2, 0.7, true);
    assert_eq!(nominal["outcome"], "placed");
    assert!(nominal["alarm_tick"].is_null());
    let fixed = sample(0.45, 0.35, false);
    let recovered = sample(0.45, 0.35, true);
    assert_eq!(fixed["outcome"], "miss");
    assert_eq!(recovered["outcome"], "placed");
    assert_eq!(recovered["alarm_tick"], fixed["alarm_tick"]);
    assert_eq!(recovered["opposed_transfer_ticks"], 1100);
    assert!(recovered["placement_error_m"].as_f64().unwrap() < 0.008);
    let abort = sample(0.85, 0.35, true);
    assert_eq!(abort["outcome"], "set_down");
    assert_eq!(abort["settled_source_ticks"], 200);
    assert_eq!(abort["settled_jaw_ticks"], 0);
    assert!(
        abort["release_tick"].as_u64().unwrap() >= abort["set_down_tick"].as_u64().unwrap() + 1520
    );
    for row in runs.iter().filter(|row| row["feedback"] == true) {
        assert_eq!(row["floor_impulse_ns"], 0.);
        assert_eq!(row["obstacle_impulse_ns"], 0.);
        assert!(row["max_jaw_command_n"].as_f64().unwrap() <= 9.);
    }
    for case in ["fixed", "recovery", "overload"] {
        assert!(
            output
                .path()
                .join(case)
                .join("replay/manifest.json")
                .is_file()
        );
    }
    Ok(())
}

#[test]
fn a_missed_grasp_missing_alarm_or_excess_effort_fails_qualification() -> Result<()> {
    let mut witness = trial(0.45, 0.35, false, None)?;
    assert!(witness.qualify("recovery").is_err());
    assert!(witness.qualify("invalid").is_err());
    witness.qualify("fixed")?;
    let alarm = witness.alarm_tick.take();
    assert!(witness.qualify("fixed").is_err());
    witness.alarm_tick = alarm;
    witness.max_jaw_command_n = 9.01;
    assert!(witness.qualify("fixed").is_err());
    Ok(())
}

#[test]
fn a_partial_archive_cannot_pass_qualification() -> Result<()> {
    let output = tempfile::tempdir()?;
    let fixed = output.path().join("fixed");
    fs::write(&fixed, "unwritable output directory")?;
    assert!(run(output.path()).is_err());
    assert!(!output.path().join("matrix.json").exists());
    fs::remove_file(&fixed)?;
    fs::create_dir_all(fixed.join("trial.json"))?;
    assert!(trial(0.45, 0.35, false, Some(&fixed)).is_err());
    Ok(())
}
