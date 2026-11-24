// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn world(xml: &str) -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        xml,
        "snag.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
#[test]
fn real_snag_on_unexpected_bar_resets_during_a_loaded_retreat_and_repeats() -> Result<()> {
    let xml = Scene::Snag.xml();
    let sdf = sdformat::load_str(&xml, "snag.sdf", &sdformat::NoResources)?;
    let mut sim = world(&xml)?;
    let robot = Robot::load(&sim, &sdf)?;
    let mut controller = Controller::new();
    let mut momentum = DVec3::ZERO;
    let mut last = None;
    for tick in 1..=7500 {
        controller.drive(&robot, &mut sim, tick)?;
        sim.step_exact(1)?;
        let s = sample(&sim, &robot, momentum)?;
        momentum = s.momentum.into();
        let y = sim
            .world()
            .get::<JointSample>(robot.rail.entity)
            .unwrap()
            .state
            .position;
        controller.observe(tick, y, &s)?;
        if controller.stage == Stage::Backoff {
            assert!(s.bar > controller::FORCE_TRIGGER);
            assert!(s.squeeze.into_iter().all(|n| n > 0.1));
            last = Some((tick, s));
            break;
        }
    }
    let (tick, at_snag) = last.ok_or("no physical snag")?;
    gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
    let (mut rows, events) = simulate(&mut sim, &sdf, None)?;
    assert_eq!(rows[tick - 1].sample, at_snag);
    qualification::measure(Scene::Snag, &rows, &events)?;
    assert!(qualification::measure(Scene::Clear, &rows, &events).is_err());
    let opening = events
        .iter()
        .find(|e| e.stage == Stage::Opening)
        .unwrap()
        .tick;
    rows[opening - 4].sample.shelf = 0.;
    assert!(qualification::measure(Scene::Snag, &rows, &events).is_err());
    assert!(qualification::measure(Scene::Snag, &rows, &[]).is_err());
    Ok(())
}

#[test]
fn an_open_grasp_or_subthreshold_load_cannot_trigger_recovery() -> Result<()> {
    let mut ctl = Controller::new();
    let mut s = Sample::default();
    assert!(ctl.observe(5400, 0., &s).is_err());
    s.squeeze = [2., 2.];
    ctl.observe(5400, 0., &s)?;
    s.bar = controller::FORCE_TRIGGER;
    ctl.observe(5401, 0., &s)?;
    assert_eq!(ctl.stage, Stage::Transport);
    s.bar += 1e-6;
    ctl.observe(5402, 0.14, &s)?;
    assert_eq!(ctl.stage, Stage::Backoff);
    ctl.observe(6900, 0.04, &s)?;
    assert_eq!(ctl.stage, Stage::Backoff);
    s.bar = 0.;
    ctl.observe(6901, 0.04, &s)?;
    assert_eq!(ctl.stage, Stage::Raising);
    ctl.observe(8400, 0.04, &s)?;
    assert_eq!(ctl.stage, Stage::Raising);
    Ok(())
}
#[test]
fn removing_the_robot_refuses_every_late_motion_command() -> Result<()> {
    let xml = Scene::Snag.xml();
    let mut sim = world(&xml)?;
    let sdf = sdformat::load_str(&xml, "snag.sdf", &sdformat::NoResources)?;
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
    let mut ctl = Controller::new();
    for stage in [
        Stage::Pickup,
        Stage::Transport,
        Stage::Raising,
        Stage::Backoff,
        Stage::Returning,
        Stage::Placing,
        Stage::Opening,
        Stage::Withdrawn,
    ] {
        ctl.stage = stage;
        assert!(ctl.drive(&robot, &mut sim, 1).is_err());
    }
    assert!(simulate(&mut sim, &sdf, None).is_err());
    Ok(())
}

#[test]
fn a_missing_receiver_withholds_release_and_cannot_publish_a_successful_recording() -> Result<()> {
    let xml = Scene::Clear
        .xml()
        .replace("0 0.46 0.574", "0.85 0.46 0.574");
    let sdf = sdformat::load_str(&xml, "missed-shelf.sdf", &sdformat::NoResources)?;
    let mut sim = world(&xml)?;
    let error = simulate(&mut sim, &sdf, None)
        .err()
        .ok_or("false placement")?;
    assert!(
        error.to_string().contains("incomplete recovery at Placing"),
        "{error}"
    );
    let (_, cable) = deformable::bodies(&sim).next().unwrap();
    assert_eq!(cable.last_boundary_impulses()[3].linear, [0.; 3]);
    assert!(
        cable.last_boundary_impulses()[0]
            .linear
            .iter()
            .any(|&j| j != 0.)
    );
    Ok(())
}

#[test]
fn missing_material_and_existing_outputs_return_errors() -> Result<()> {
    let folder = tempfile::tempdir()?;
    assert!(record(folder.path()).is_err());
    let xml = Scene::Snag.xml();
    let (robot, _) = xml
        .split_once("    <plugin name=\"gz::sim::systems::Deformables\"")
        .unwrap();
    let xml = format!("{robot}</world></sdf>");
    let sdf = sdformat::load_str(&xml, "missing.sdf", &sdformat::NoResources)?;
    for output in [None, Some(folder.path())] {
        assert!(simulate(&mut world(&xml)?, &sdf, output).is_err());
    }
    Ok(())
}
