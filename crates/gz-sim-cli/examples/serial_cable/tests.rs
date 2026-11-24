// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn reset_with_a_gripped_cable_clears_two_pending_jaw_reactions_and_repeats_pickup() -> Result<()> {
    let sdf = sdformat::load_str(SDF, "cable.sdf", &sdformat::NoResources)?;
    let mut sim = load()?;
    let grasp = |sim: &mut Simulation| -> Result<Sample> {
        let controller = Robot::load(sim, &sdf)?;
        let mut prior = DVec3::ZERO;
        for tick in 1..=5399 {
            controller.drive(sim, tick, false, None, 2.70)?;
            sim.step_exact(1)?;
            if tick == 5399 {
                prior = sample(sim, &controller, DVec3::ZERO)?.momentum.into();
            }
        }
        controller.drive(sim, 5400, false, None, 2.70)?;
        sim.step_exact(1)?;
        sample(sim, &controller, prior)
    };
    let loaded = grasp(&mut sim)?;
    assert!(loaded.squeeze.iter().all(|p| *p > 2.));
    assert!(loaded.source == 0. && loaded.shelf == 0. && loaded.bar == 0.);
    gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(grasp(&mut sim)?, loaded);
    Ok(())
}

#[test]
fn open_jaws_fail_the_usable_transfer_checks_and_the_full_replay_identifies_each_particle()
-> Result<()> {
    let sdf = sdformat::load_str(SDF, "cable.sdf", &sdformat::NoResources)?;
    let folder = tempfile::tempdir()?;
    let dir = folder.path().join("open");
    let open = run(&mut load()?, &sdf, true, 2.70, Some(&dir))?;
    let report = measure::Report::from_run(&open);
    report.qualify("open")?;
    assert!(report.qualify("clear").is_err());
    assert!(report.qualify("low").is_err());
    assert!(report.qualify("unknown").is_err());
    let positions: Vec<serde_json::Value> =
        serde_json::from_slice(&std::fs::read(dir.join("particle-frames.json"))?)?;
    assert_eq!(positions.len(), 201);
    assert!(
        positions
            .iter()
            .all(|f| f["positions"].as_array().unwrap().len() == 11)
    );
    Ok(())
}

#[test]
fn missing_nonbox_or_compound_clearance_geometry_refuses_the_measurement() -> Result<()> {
    let sim = load()?;
    let mut sdf = sdformat::load_str(SDF, "cable.sdf", &sdformat::NoResources)?;
    assert!(joint(&sim, "absent").is_err());
    assert!(BoxShape::find(&sim, &sdf, "absent").is_err());
    let link = &mut sdf.worlds[0].models[0].links[0];
    link.collisions[0].geometry = sdformat::Geometry::Sphere { radius: 0.1 };
    assert!(BoxShape::find(&sim, &sdf, "stand::post").is_err());
    sdf.worlds[0].models[0].links[0].collisions.clear();
    assert!(BoxShape::find(&sim, &sdf, "stand::post").is_err());
    Ok(())
}

#[test]
fn an_incomplete_workcell_cannot_be_reported_as_a_complete_transfer() -> Result<()> {
    let sdf = sdformat::load_str(SDF, "cable.sdf", &sdformat::NoResources)?;
    // A load receipt binds each physical collision and actuator independently.
    for missing in [
        "gripper::palm::palm",
        "gripper::left::left",
        "gripper::right::right",
        "receiver::shelf::top",
        "obstacle::bar::bar",
        "gripper::travel",
        "gripper::spindle",
        "gripper::wrist",
        "gripper::close_left",
        "gripper::close_right",
        "gripper::left",
        "gripper::right",
    ] {
        let mut sim = load()?;
        let name = format!("default::{missing}");
        let e = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>().is_some_and(|n| n.0 == name)
                    && e.get::<gz_sim_scene::EntityKind>()
                        != Some(&gz_sim_scene::EntityKind::Visual)
            })
            .unwrap()
            .id();
        sim.world_mut().entity_mut(e).remove::<Name>();
        assert!(Robot::load(&sim, &sdf).is_err(), "{missing}");
        assert!(run(&mut sim, &sdf, false, 2.70, None).is_err());
    }
    Ok(())
}

#[test]
fn sample_refuses_missing_physical_feedback_instead_of_publishing_zero_load() -> Result<()> {
    let sdf = sdformat::load_str(SDF, "cable.sdf", &sdformat::NoResources)?;
    for name in [
        "obstacle::bar::bar",
        "receiver::shelf::top",
        "gripper::carriage::truck",
        "gripper::left::left",
        "gripper::left",
    ] {
        let mut sim = load()?;
        let robot = Robot::load(&sim, &sdf)?;
        sim.step_exact(1)?;
        let e = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == format!("default::{name}"))
                    && e.get::<gz_sim_scene::EntityKind>()
                        != Some(&gz_sim_scene::EntityKind::Visual)
            })
            .unwrap()
            .id();
        sim.world_mut().entity_mut(e).remove::<WorldPose>();
        assert!(sample(&sim, &robot, DVec3::ZERO).is_err());
    }
    let mut sim = load()?;
    let robot = Robot::load(&sim, &sdf)?;
    // The initial published cable has geometry, but no measured boundary impulses.
    assert!(sample(&sim, &robot, DVec3::ZERO).is_err());
    sim.step_exact(1)?;
    sim.world_mut()
        .entity_mut(robot.spindle.entity)
        .remove::<JointSample>();
    assert!(sample(&sim, &robot, DVec3::ZERO).is_err());
    assert!(run(&mut sim, &sdf, false, 2.70, None).is_err());
    let mut sim = load()?;
    let robot = Robot::load(&sim, &sdf)?;
    sim.step_exact(1)?;
    sim.world_mut()
        .entity_mut(robot.wrist.entity)
        .remove::<gz_sim_scene::UniversalJointSample>();
    assert!(sample(&sim, &robot, DVec3::ZERO).is_err());
    Ok(())
}

#[test]
fn removed_gripper_or_material_refuses_commands_and_recording() -> Result<()> {
    let sdf = sdformat::load_str(SDF, "cable.sdf", &sdformat::NoResources)?;
    let mut sim = load()?;
    let robot = Robot::load(&sim, &sdf)?;
    let gripper = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::gripper"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    gz_sim_scene::remove_model(&mut sim, gripper)?;
    for tick in [1, 2, 900, 2400, 5600, 9600, 11375] {
        assert!(
            robot
                .drive(&mut sim, tick, false, Some(11000), 2.70)
                .is_err(),
            "{tick}"
        );
    }
    let mut no_material_sdf = SDF.to_owned();
    let a = no_material_sdf
        .find("    <plugin name=\"gz::sim::systems::Deformables\"")
        .unwrap();
    no_material_sdf.truncate(a);
    no_material_sdf.push_str("</world></sdf>");
    let mut no_material = load_world_with_sdf_step(
        &no_material_sdf,
        "cable.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let robot = Robot::load(&no_material, &sdf)?;
    assert!(sample(&no_material, &robot, DVec3::ZERO).is_err());
    assert!(run(&mut no_material, &sdf, true, 2.70, None).is_err());
    assert!(run(&mut no_material, &sdf, true, 2.70, Some(Path::new("."))).is_err());
    Ok(())
}
