// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn a_midair_tug_requires_matched_states_real_slip_and_the_correct_loaded_stand() -> Result<()> {
    let mut runs = Comparison {
        stand: "source",
        quiet: run_case(1860, 0., true, None)?,
        fixed: run_case(1860, FORCE_N, false, None)?,
        guarded: run_case(1860, FORCE_N, true, None)?,
    };
    runs.check()?;
    let incoming = runs.guarded.swing.incoming.take();
    assert!(runs.check().is_err());
    runs.guarded.swing.incoming = incoming;
    let alarm = runs.guarded.trial.alarm_tick.take();
    assert!(runs.check().is_err());
    runs.guarded.trial.alarm_tick = alarm;
    let park = runs.guarded.trial.set_down_tick.take();
    assert!(runs.check().is_err());
    runs.guarded.trial.set_down_tick = park;
    let yaw = runs.guarded.swing.set_down_yaw_rad.take();
    assert!(runs.check().is_err());
    runs.guarded.swing.set_down_yaw_rad = yaw;
    let release = runs.guarded.trial.release_tick.take();
    assert!(runs.check().is_err());
    runs.guarded.trial.release_tick = release;
    runs.guarded.swing.min_support_n_before_release = Some(0.1);
    assert!(runs.check().is_err());
    runs.guarded.swing.min_support_n_before_release = Some(2.);
    let impact = runs.fixed.swing.parcel_obstacle_impulse_ns;
    runs.fixed.swing.parcel_obstacle_impulse_ns = 0.;
    assert!(runs.check().is_err());
    runs.fixed.swing.parcel_obstacle_impulse_ns = impact;
    runs.fixed.swing.incoming.as_mut().unwrap().bodies[0].velocity[1] += 0.001;
    assert!(runs.check().is_err());
    Ok(())
}

#[test]
fn both_disturbances_export_the_measured_events_and_matched_controls() -> Result<()> {
    let dir = tempfile::tempdir()?;
    run(dir.path())?;
    let rows: Value = serde_json::from_slice(&fs::read(dir.path().join("comparison.json"))?)?;
    assert_eq!(rows.as_array().unwrap().len(), 2);
    for (index, stand) in ["source", "target"].into_iter().enumerate() {
        let captures: Value =
            serde_json::from_slice(&fs::read(dir.path().join(stand).join("frames.json"))?)?;
        let events = &captures["captures"];
        let mut previous = 0;
        for event in [
            "before_tug",
            "tug",
            "alarm",
            "set_down",
            "release",
            "settled",
        ] {
            let frame = events[event].as_u64().unwrap();
            assert!(frame > previous);
            previous = frame;
        }
        assert_eq!(events, &rows[index]["guarded"]["swing"]["captures"]);
    }
    assert!(dir.path().join("fixed/replay/manifest.json").is_file());
    Ok(())
}
