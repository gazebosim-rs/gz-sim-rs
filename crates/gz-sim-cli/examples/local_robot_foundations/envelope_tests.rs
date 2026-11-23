// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn reset_checker_identifies_truncation_malformed_frames_and_changed_poses() {
    let first = (r#"{"epoch":5,"p":[1,0,0]}"#.to_string(), String::new());
    let next = (r#"{"epoch":6,"p":[1,0,0]}"#.to_string(), String::new());
    let good = [next.clone()];
    assert!(verify_reset_frames(std::slice::from_ref(&first), &good).is_ok());
    assert!(verify_reset_frames(std::slice::from_ref(&first), &[]).is_err());
    assert!(
        verify_reset_frames(std::slice::from_ref(&first), std::slice::from_ref(&first)).is_err()
    );
    assert!(
        verify_reset_frames(
            std::slice::from_ref(&first),
            &[(r#"{"epoch":6,"p":[2,0,0]}"#.to_string(), String::new())]
        )
        .is_err()
    );
    let malformed = [("truncated".to_string(), String::new())];
    assert!(verify_reset_frames(&malformed, &good).is_err());
    assert!(verify_reset_frames(std::slice::from_ref(&first), &malformed).is_err());
    assert!(verify_reset_frames(&[("{}".to_string(), String::new())], &good).is_err());
}

#[test]
fn required_instrument_and_actuator_bindings_are_checked_before_stepping() -> Result<()> {
    let load = Load {
        mass_kg: 0.2,
        com_x_m: 0.,
        friction: 0.7,
        travel_s: 0.95,
    };
    for (needle, replacement) in [
        ("name=\"rail\"", "name=\"unexpected_axis\""),
        ("<model name=\"parcel\">", "<model name=\"moved_parcel\">"),
        ("<model name=\"source\">", "<model name=\"other_stand\">"),
        ("palm", "wrist"),
    ] {
        let xml = load.xml().replace(needle, replacement);
        let mut r = Recording::new(&xml, "binding.sdf", "default")?;
        assert!(cycle(&mut r, load, true, false).is_err());
        assert!(r.costs_ns.is_empty());
    }
    Ok(())
}

#[test]
fn tactile_recovery_and_overload_setdown_use_the_same_material_and_motion() -> Result<()> {
    for (mass_kg, expected) in [
        (0.2, "carry"),
        (0.45, "recovered_carry"),
        (0.85, "safe_set_down"),
    ] {
        let load = Load {
            mass_kg,
            com_x_m: 0.,
            friction: 0.35,
            travel_s: 0.95,
        };
        let (physical, _) = cycle(&mut recording(load)?, load, true, false)?;
        assert_eq!(physical.outcome, expected);
        assert_eq!(physical.floor_contact_ticks, 0);
        assert_eq!(physical.final_station_loaded_ticks, 200);
        assert_eq!(physical.final_jaw_loaded_ticks, 0);
        if expected == "carry" {
            assert!(physical.first_tactile_alarm_tick.is_none());
            assert!(physical.independent_slip_tick.is_none());
        } else {
            let alarm = physical.first_tactile_alarm_tick.unwrap();
            assert!(physical.independent_slip_tick.unwrap().abs_diff(alarm) < 25);
            let (fixed, _) = cycle(&mut recording(load)?, load, false, false)?;
            assert_eq!(fixed.outcome, "miss");
            assert_eq!(fixed.first_tactile_alarm_tick, Some(alarm));
            assert!(fixed.tighten_tick.is_none());
        }
        if let Some(abort) = physical.set_down_tick {
            assert!(physical.release_tick.unwrap() >= abort + 1120 + 20);
        }
    }
    Ok(())
}

#[test]
fn paired_campaign_writes_all_load_cases_and_replay_witnesses() -> Result<()> {
    let root = tempfile::tempdir()?;
    let output = root.path().join("results");
    write_new(&output, run)?;
    let matrix: Vec<Value> = serde_json::from_slice(&fs::read(output.join("campaign.json"))?)?;
    assert_eq!(matrix.len(), 24);
    for row in &matrix {
        assert_eq!(row["guarded"]["floor_contact_ticks"], 0);
        assert_eq!(row["guarded"]["closed_loaded_ticks_of_50"], 50);
        assert_eq!(row["guarded"]["final_jaw_loaded_ticks"], 0);
    }
    for tag in ["success", "recovery", "overload"] {
        let witness = output.join(tag);
        let metrics: Value = serde_json::from_slice(&fs::read(witness.join("recording.json"))?)?;
        assert_eq!(metrics["frames"], 101);
        assert_eq!(metrics["steps"], 5000);
        assert!(witness.join("replay/manifest.json").is_file());
    }
    // An existing file in place of a study's output tree must fail visibly.
    assert!(run(&output.join("campaign.json")).is_err());
    Ok(())
}

#[test]
fn supported_parcel_and_stand_receive_equal_and_opposite_vertical_reactions() -> Result<()> {
    let mut r = Recording::new(crate::arm::SCENE, "arm.sdf", "default")?;
    r.sim.step_exact(100)?;
    let handle = |name: &str| body(&r.sim, entity_id(&r.sim, &format!("default::{name}"))?);
    let (parcel, stand) = (handle("parcel::box")?, handle("source::body")?);
    let jaws = [handle("arm::left")?, handle("arm::right")?];
    let (target, floor) = (handle("target::body")?, handle("floor::body")?);
    let upward = tactile(&r.sim, parcel, jaws, [stand, target, floor])?.station_up_ns[0];
    let downward = tactile(&r.sim, stand, jaws, [parcel, target, floor])?.station_up_ns[0];
    let weight_impulse = 0.2 * 9.81 * r.sim.step_duration().as_secs_f64();
    assert!((upward - weight_impulse).abs() < 1e-6);
    assert_eq!(downward, -upward);
    Ok(())
}
