// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn clock_and_receiver_domains_are_checked_before_writing_a_recording() {
    let d = tempfile::tempdir().unwrap();
    for (n, public, mode) in [
        (0, false, Mode::Timed),
        (7, false, Mode::Supported),
        (500, true, Mode::Supported),
        (16, true, Mode::Missing),
    ] {
        assert!(run(d.path(), mode, n, public, false).is_err());
        assert_eq!(d.path().read_dir().unwrap().count(), 0);
    }
}

#[test]
fn the_ordinary_timed_sequence_cannot_qualify_as_a_supported_placement() -> Result<()> {
    let xml = sheet_jaws::setdown_world(16, false, true)?;
    let mut r = Recording::new(&xml, "setdown.sdf", "default")?;
    let ticks = episode(&mut r, Mode::Timed, &mut None)?;
    let m = ObservationWindow::new(Mode::Timed, &ticks, 0.001)?;
    assert!(m.airborne_after_opening_s > 0.009);
    assert!(ObservationWindow::new(Mode::Supported, &ticks, 0.001).is_err());
    assert!(ObservationWindow::new(Mode::Missing, &ticks, 0.001).is_err());
    Ok(())
}

#[test]
fn a_solver_fault_prevents_a_release_receipt() -> Result<()> {
    let xml = sheet_jaws::setdown_world(16, false, true)?;
    let mut r = Recording::new(&xml, "setdown.sdf", "default")?;
    r.sim
        .world()
        .resource::<gz_sim_core::StepFault>()
        .report(std::io::Error::other("fault"));
    assert!(episode(&mut r, Mode::Supported, &mut None).is_err());
    Ok(())
}

#[test]
fn rejects_missing_support_readings_links_and_actuator_state() {
    let xml = sheet_jaws::setdown_world(16, false, true).unwrap();
    let without_table = xml.replace("<plane point=\"0 0 0.12\" normal=\"0 0 1\"/>", "");
    let mut r = Recording::new(&without_table, "absent-reading.sdf", "default").unwrap();
    let err = episode(&mut r, Mode::Supported, &mut None).unwrap_err();
    assert!(
        err.to_string()
            .contains("missing native jaw or table reading")
    );
    for name in ["default::source::body", "default::arm::close_left"] {
        let mut r = Recording::new(&xml, "bad-binding.sdf", "default").unwrap();
        let e = entity_by_id(r.sim.world(), entity_id(&r.sim, name).unwrap()).unwrap();
        r.sim.world_mut().entity_mut(e).remove::<Name>();
        assert!(episode(&mut r, Mode::Supported, &mut None).is_err());
    }
    let mut r = Recording::new(&xml, "bad-drive.sdf", "default").unwrap();
    let c = Controller::with_jaw_damping(&r.sim, 1.5).unwrap();
    let e = entity_by_id(
        r.sim.world(),
        entity_id(&r.sim, "default::arm::close_right").unwrap(),
    )
    .unwrap();
    r.sim
        .world_mut()
        .entity_mut(e)
        .remove::<gz_sim_scene::JointSample>();
    assert!(drive(&c, &mut r.sim, 2.8, Mode::Supported, Default::default()).is_err());
}

#[test]
fn the_original_sheet_recording_keeps_its_camera_and_committed_coordinates() {
    let xml = sheet_jaws::setdown_world(16, false, true).unwrap();
    let mut r = Recording::new(&xml, "sampling.sdf", "default").unwrap();
    let mut frames = Vec::new();
    arm_sheet::capture(&mut r, &mut frames).unwrap();
    r.advance().unwrap();
    arm_sheet::capture(&mut r, &mut frames).unwrap();
    let d = tempfile::tempdir().unwrap();
    arm_sheet::save_archive(r, frames, d.path(), json!({"first":0,"committed":1})).unwrap();
    let metadata: Value =
        serde_json::from_slice(&fs::read(d.path().join("frames.json")).unwrap()).unwrap();
    let particles: Value =
        serde_json::from_slice(&fs::read(d.path().join("particle-frames.json")).unwrap()).unwrap();
    assert_eq!(metadata["camera"], "right");
    assert_eq!(particles[1]["time_ns"], 1_000_000);
    assert_eq!(particles[1]["positions"].as_array().unwrap().len(), 12);
}

#[test]
fn the_same_receiver_can_drop_before_or_after_the_fingers_unload() -> Result<()> {
    let d = tempfile::tempdir()?;
    let mut cases = Vec::new();
    for mode in [Mode::Finite, Mode::DropEarly, Mode::DropLate] {
        let path = d.path().join(format!("{mode:?}"));
        fs::create_dir(&path)?;
        run(&path, mode, 16, false, false)?;
        let trace: Vec<Value> = serde_json::from_slice(&fs::read(path.join("ticks.json"))?)?;
        cases.push(trace);
    }
    let [control, early, late]: [_; 3] = cases.try_into().unwrap();
    for trial in [&early, &late] {
        let start = trial
            .iter()
            .position(|r| r["deck_descent_m"].as_f64().unwrap() > 1e-6)
            .unwrap();
        assert_eq!(trial[..start], control[..start]);
        assert_eq!(trial[start]["table_n"], 0.0);
        assert_eq!(trial[start + 1]["opening_ms"], trial[start]["opening_ms"]);
    }
    assert!(early.last().unwrap()["search_exhausted"].as_bool().unwrap());
    assert!(early.last().unwrap()["jaws_up_n"].as_f64().unwrap() > 0.08);
    assert!(late.last().unwrap()["table_n"].as_f64().unwrap() > 0.08);
    assert_eq!(late.last().unwrap()["jaws_up_n"], 0.0);
    Ok(())
}
