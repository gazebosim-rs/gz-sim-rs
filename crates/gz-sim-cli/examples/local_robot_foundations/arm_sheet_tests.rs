// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn lift_and_release_both_panels_and_replay_the_open_control() -> Result<()> {
    let out = tempfile::tempdir()?;
    for state in ["held", "open"] {
        let path = out.path().join(state);
        let mut m = run_case(
            Case {
                substeps: 1,
                closed: state == "held",
                ..Default::default()
            },
            Some(&path),
        )?;
        qualify_loaded_fold(&m)?;
        let frames: Vec<Value> =
            serde_json::from_slice(&fs::read(path.join("particle-frames.json"))?)?;
        let metadata: Value = serde_json::from_slice(&fs::read(path.join("frames.json"))?)?;
        assert_eq!(frames.len(), END / STRIDE + 1);
        assert_eq!(
            metadata["particle_visuals"].as_array().unwrap().len(),
            COUNT
        );
        assert_eq!(frames.last().unwrap()["time_ns"], END * 1_000_000);
        assert!(
            frames
                .iter()
                .all(|f| f["positions"].as_array().unwrap().len() == COUNT)
        );
        assert!(path.join("replay/manifest.json").exists());
        if m.case.closed {
            let pressure = m.mean_hold_self_contact_n;
            m.mean_hold_self_contact_n = 0.;
            assert!(m.qualify().is_ok() && qualify_loaded_fold(&m).is_err());
            m.mean_hold_self_contact_n = pressure;
            m.mean_hold_panel_up_by_jaw_n[0][0] = 0.;
            assert!(m.qualify().is_ok() && qualify_loaded_fold(&m).is_err());
        }
    }
    Ok(())
}

#[test]
fn altered_arm_fixture_and_material_settings_are_checked() -> Result<()> {
    let c = Case::default();
    for tag in [
        "<model name=\"parcel\">",
        "<model name=\"source\">",
        "</world>",
    ] {
        assert!(scene(&arm::SCENE.replace(tag, ""), c, NOMINAL_FLARE).is_err());
    }
    let incomplete_bench = arm::SCENE.split_once("<model name=\"source\">").unwrap().0;
    assert!(scene(&format!("{incomplete_bench}<model name=\"source\">"), c, 0.).is_err());
    let invalid = Case { substeps: 0, ..c };
    assert!(run_case(invalid, None).is_err());
    let src = scene(arm::SCENE, c, NOMINAL_FLARE)?;
    assert!(
        simulate(
            &src.replace("material=\"sheet\"", "material=\"absent\""),
            c,
            NOMINAL_FLARE,
            None
        )
        .is_err()
    );
    let empty = Recording::new(arm::SCENE, "arm.sdf", "default")?;
    assert!(cloth(&empty.sim).is_err());
    let mut empty = empty;
    assert!(capture(&mut empty, &mut vec![]).is_err());
    Ok(())
}

#[test]
fn missing_boundary_and_unwritable_observations_fail_the_study() -> Result<()> {
    let c = Case {
        closed: false,
        ..Default::default()
    };
    let xml = scene(arm::SCENE, c, NOMINAL_FLARE)?;
    let without_bench = xml.replace(
        &format!("<plane point=\"0 0 {BENCH}\" normal=\"0 0 1\"/>"),
        "",
    );
    assert!(simulate(&without_bench, c, NOMINAL_FLARE, None).is_err());
    let out = tempfile::tempdir()?;
    fs::create_dir(out.path().join("particle-frames.json"))?;
    assert!(run_case(c, Some(out.path())).is_err());
    Ok(())
}

#[test]
fn gripper_cannot_claim_a_sheet_carried_by_the_bench() -> Result<()> {
    let mut m = run_case(
        Case {
            closed: false,
            ..Default::default()
        },
        None,
    )?;
    m.qualify()?;
    m.case.closed = true;
    assert!(qualify_loaded_fold(&m).is_err());
    m.case.closed = false;
    m.max_pad_attribution_residual_ns = 1.;
    assert!(m.qualify().is_err());
    Ok(())
}

#[test]
fn unusable_fold_surfaces_report_the_failed_tick_and_solver_diagnostic() -> Result<()> {
    let c = Case::default();
    let xml = scene(arm::SCENE, c, NOMINAL_FLARE)?.replace(
        &format!("radius=\"{RADIUS}\" self_contact"),
        "radius=\"0.03\" self_contact",
    );
    let error = simulate(&xml, c, NOMINAL_FLARE, None).unwrap_err();
    let fault = error.downcast::<SheetStepFault>()?;
    assert_eq!(fault.tick, 1);
    assert!(fault.to_string().contains(&fault.diagnostic));
    assert!(!fault.diagnostic.is_empty());
    // Readback must never expose the previous material as a fresh measurement
    // after a contact refusal, including after a rejected reset.
    let mut r = Recording::new(&xml, "arm_sheet.sdf", "default")?;
    let energy = |sim: &Simulation| gz_sim_scene::deformable::material_energy(sim, "sheet");
    assert!(energy(&r.sim).unwrap()?.total().is_finite());
    assert!(r.advance().is_err());
    assert!(r.sim.is_faulted());
    assert!(energy(&r.sim).is_none());
    assert!(r.sim.reset().is_err());
    assert!(energy(&r.sim).is_none());
    Ok(())
}
