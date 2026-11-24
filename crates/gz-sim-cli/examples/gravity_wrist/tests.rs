// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[test]
fn feedforward_holds_the_loaded_wrist_against_a_free_swing_and_repeats_after_reset() -> Result<()> {
    let tmp = tempfile::tempdir()?;
    record(tmp.path())?;
    assert!(tmp.path().join("replay/frames.jsonl").is_file());
    Ok(())
}

#[test]
fn a_fault_or_interrupted_recorder_stops_before_publishing_a_successful_trial() -> Result<()> {
    let mut sim = load_world_with_sdf_step(
        XML,
        "gravity_wrist.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let msg = episode(&mut sim, |tick, _| {
        if tick == 20 {
            Err("capture interrupted".into())
        } else {
            Ok(())
        }
    })
    .unwrap_err();
    assert_eq!(msg.to_string(), "capture interrupted");
    sim.world()
        .resource::<gz_sim_core::StepFault>()
        .report(std::io::Error::other("lost solver"));
    assert!(episode(&mut sim, |_, _| Ok(())).is_err());
    let tmp = tempfile::tempdir()?;
    let taken = tmp.path().join("occupied");
    fs::write(&taken, b"owner")?;
    assert!(record(&taken).is_err());
    assert_eq!(fs::read(&taken)?, b"owner");
    Ok(())
}

#[test]
fn removing_the_bearing_or_payload_clamp_invalidates_either_physical_comparison_arm() {
    for model in ["held", "free"] {
        for joint in ["bearing", "clamp"] {
            let mut sim = load_world_with_sdf_step(
                XML,
                "gravity_wrist.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity,
            )
            .unwrap();
            let entity = named(&sim, &format!("default::{model}::{joint}"));
            let handle = sim
                .world()
                .get::<gz_sim_scene::JointHandle>(entity)
                .unwrap()
                .id();
            assert!(
                sim.world_mut()
                    .resource_mut::<gz_physics::PhysicsBackend>()
                    .remove_joint(handle)
            );
            let mut captured = 0;
            assert!(
                episode(&mut sim, |_, _| {
                    captured += 1;
                    Ok(())
                })
                .is_err()
            );
            assert_eq!(captured, 0);
        }
    }
}
