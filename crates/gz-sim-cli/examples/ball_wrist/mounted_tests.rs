// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::{StepFault, entity_by_id};

#[test]
fn a_missing_sensor_identity_or_pose_is_reported_at_its_source() -> Result<(), Box<dyn Error>> {
    for name in [
        "default::wrist",
        "default::wrist::bearing",
        "default::wrist::tool",
    ] {
        let mut sim = load(HANGING)?;
        let entity = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
            .id();
        sim.world_mut().entity_mut(entity).remove::<Name>();
        assert!(episode(&mut sim, true, |_| Ok(())).is_err());
    }
    for model in [true, false] {
        let mut sim = load(HANGING)?;
        let wrist = Wrist::bind(&sim)?;
        let id = if model { wrist.model } else { wrist.bearing };
        let e = entity_by_id(sim.world(), id).unwrap();
        sim.world_mut().entity_mut(e).remove::<EntityId>();
        assert!(Wrist::bind(&sim).is_err());
    }
    let mut sim = load(HANGING)?;
    sim.step_exact(1)?;
    let wrist = Wrist::bind(&sim)?;
    sim.world_mut()
        .entity_mut(wrist.joint)
        .remove::<BallJointSample>();
    assert!(wrist.state(&sim).is_err());
    sim.world_mut().entity_mut(wrist.tool).remove::<WorldPose>();
    assert!(wrist.state(&sim).is_err());
    Ok(())
}

#[test]
fn an_unwritable_or_interrupted_recording_does_not_qualify() -> Result<(), Box<dyn Error>> {
    for at in [0, 2] {
        let mut count = 0;
        let mut sim = load(HANGING)?;
        let interrupted = episode(&mut sim, true, |_| {
            count += 1;
            if count == at + 1 {
                return Err("recorder disconnected".into());
            }
            Ok(())
        });
        assert_eq!(
            interrupted.err().unwrap().to_string(),
            "recorder disconnected"
        );
    }
    let mut sim = load(HANGING)?;
    sim.world()
        .resource::<StepFault>()
        .report(std::io::Error::other("physics fault"));
    assert!(episode(&mut sim, false, |_| Ok(())).is_err());
    let taken = tempfile::NamedTempFile::new()?;
    assert!(record(taken.path(), HANGING).is_err());
    Ok(())
}

#[test]
fn refuse_an_unbalanced_robot_an_unknown_model_and_an_invalid_world() {
    let d = tempfile::tempdir().unwrap();
    for xml in [
        "<bad>",
        &HANGING.replace("name=\"wrist\"", "name=\"other\""),
        &HANGING.replace("<gravity>0 0 -9.81", "<gravity>0 -0.1 -9.81"),
        &HANGING.replace("type=\"ball\"", "type=\"gearbox\""),
    ] {
        let path = d.path().join("invalid");
        assert!(record(&path, xml).is_err());
        assert!(!path.exists());
    }
}

#[test]
fn the_report_uses_the_short_rotation_even_across_a_quaternion_sign_change() {
    let q = glam::DQuat::from_rotation_y(0.2);
    assert_eq!(separation(q), separation(-q));
    assert!(separation(q.conjugate() * (-q)) < 1e-15);
}

#[test]
fn an_already_parked_control_cannot_pass_as_a_continuously_solved_wrist()
-> Result<(), Box<dyn Error>> {
    let mut sim = load(HANGING)?;
    sim.step_exact(10)?;
    let wrist = Wrist::bind(&sim)?;
    set_model_fidelity(&mut sim, wrist.model, ArticulationTier::Prescribed)?;
    let error = episode(&mut sim, false, |_| Ok(()))
        .err()
        .unwrap()
        .to_string();
    assert!(error.contains("unexpected wrist tier at tick 1"), "{error}");
    Ok(())
}

#[test]
fn a_linear_encoder_cannot_stand_in_for_the_spherical_bearing_sample() -> Result<(), Box<dyn Error>>
{
    let xml = HANGING
        .replace(
            "name=\"bearing\" type=\"ball\"",
            "name=\"bearing\" type=\"prismatic\"",
        )
        .replace(
            "0.31 -0.42 0.63</pose></joint>",
            "0.31 -0.42 0.63</pose><axis><xyz>1 0 0</xyz></axis></joint>",
        );
    let mut sim = load(&xml)?;
    let error = episode(&mut sim, false, |_| Ok(()))
        .err()
        .unwrap()
        .to_string();
    assert!(error.contains("missing bearing sample"), "{error}");
    Ok(())
}
