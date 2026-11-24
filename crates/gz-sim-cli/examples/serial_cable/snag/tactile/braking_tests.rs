// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn loss_of_supported_weight_requires_uninterrupted_forward_travel_and_bilateral_load() {
    let mut guard = SupportLoss::default();
    let mut jaw = JawReadings {
        squeeze_n: [1.; 2],
        rail_speed_m_s: 0.2,
        force_on_payload_n: [0., 0., 0.7 * WEIGHT],
        ..Default::default()
    };
    let mut signal = Signal {
        drag_n: 0.008,
        ..Default::default()
    };
    for _ in 0..3 {
        assert!(guard.observe(Stage::Transport, &jaw, &signal) < UNLOAD_TRAVEL_M);
    }
    assert!(guard.observe(Stage::Transport, &jaw, &signal) > UNLOAD_TRAVEL_M);
    jaw.force_on_payload_n[2] = WEIGHT;
    assert_eq!(guard.observe(Stage::Transport, &jaw, &signal), 0.);
    jaw.force_on_payload_n[2] = 0.7 * WEIGHT;
    jaw.rail_speed_m_s = -0.3;
    assert_eq!(guard.observe(Stage::Transport, &jaw, &signal), 0.);
    jaw.rail_speed_m_s = 0.45;
    assert!(guard.observe(Stage::Transport, &jaw, &signal) < UNLOAD_TRAVEL_M);
    jaw.squeeze_n[0] = 0.;
    assert_eq!(guard.observe(Stage::Transport, &jaw, &signal), 0.);
    jaw.squeeze_n[0] = 1.;
    assert_eq!(guard.observe(Stage::Backoff, &jaw, &signal), 0.);
    signal.drag_n = 0.;
    assert_eq!(guard.observe(Stage::Transport, &jaw, &signal), 0.);
}

#[test]
fn resetting_while_braking_clears_the_real_rail_effort_and_repeats_the_safe_recovery() -> Result<()>
{
    let xml = Scene::TooHigh.xml();
    let sdf = sdformat::load_str(&xml, "brake-test.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "brake-test.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let (robot, mut ctl) = (Robot::load(&sim, &sdf)?, Controller::new());
    ctl.active_braking = true;
    let (mut guard, mut sensor) = (SupportLoss::default(), Observer::default());
    let mut trip = None;
    for tick in 1..=7500 {
        ctl.drive(&robot, &mut sim, tick)?;
        sim.step_exact(1)?;
        let jaws = JawReadings::read(&sim, &robot)?;
        let (mut feedback, signal) = sensor.update(&jaws);
        feedback.snag |= guard.observe(ctl.stage, &jaws, &signal) >= UNLOAD_TRAVEL_M;
        ctl.apply(tick, jaws.rail_y_m, &feedback)?;
        if ctl.stage == Stage::Backoff {
            ctl.drive(&robot, &mut sim, tick + 1)?;
            let cmd = sim
                .world()
                .get::<JointForceCommand>(robot.rail.entity)
                .unwrap()
                .torque();
            assert_eq!(cmd, -45.);
            sim.set_paused(true);
            sim.tick()?;
            assert_eq!(JawReadings::read(&sim, &robot)?, jaws);
            trip = Some((tick, jaws));
            break;
        }
    }
    let (tick, jaws) = trip.ok_or("no protective brake command")?;
    gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
    assert!(
        sim.world()
            .get::<JointForceCommand>(robot.rail.entity)
            .is_none_or(|v| v.torque() == 0.)
    );
    let mut rows = monitored_run(&mut sim, &sdf, Fixture::Limit, None, Some(true))?;
    assert_eq!(rows[tick - 1].jaws, jaws);
    assert_eq!(rows[tick].rail_command_n, Some(-45.));
    qualification::measure("tall", &rows)?;
    rows[tick].rigid_pad_bar_impulse_ns = None;
    assert!(qualification::measure("tall", &rows).is_err());
    rows[tick].rigid_pad_bar_impulse_ns = Some([0.01, 0.]);
    assert!(qualification::measure("tall", &rows).is_err());
    for r in &mut rows {
        r.truth.bar = 0.;
    }
    assert!(qualification::measure("tall", &rows).is_err());
    Ok(())
}

#[test]
fn missing_collision_truth_cannot_certify_recovery() -> Result<()> {
    let xml = Scene::TooHigh.xml();
    let sdf = sdformat::load_str(&xml, "brake-test.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "brake-test.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let robot = Robot::load(&sim, &sdf)?;
    let bar = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::obstacle::bar")
        })
        .unwrap()
        .id();
    sim.world_mut().entity_mut(bar).remove::<BodyHandle>();
    assert!(PadContact::new(&sim, &robot).is_err());
    let temp = tempfile::tempdir()?;
    assert!(record(temp.path()).is_err());
    Ok(())
}

#[test]
fn retired_or_uninstrumented_links_cannot_masquerade_as_zero_rigid_impulse() -> Result<()> {
    let xml = Scene::TooHigh.xml();
    let sdf = sdformat::load_str(&xml, "brake-test.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "brake-test.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let robot = Robot::load(&sim, &sdf)?;
    let pairs = PadContact::new(&sim, &robot)?;
    let barrier = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::obstacle"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    gz_sim_scene::remove_model(&mut sim, barrier)?;
    assert!(pairs.read(&sim).is_err());
    assert!(PadContact::new(&sim, &robot).is_err());
    let rigid = load_world_with_sdf_step(
        "<sdf version='1.10'><world name='default'/></sdf>",
        "empty.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )?;
    assert!(pairs.read(&rigid).is_err());
    for i in 0..2 {
        gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
        let robot = Robot::load(&sim, &sdf)?;
        sim.world_mut()
            .entity_mut(robot.pads[i].entity)
            .remove::<BodyHandle>();
        assert!(PadContact::new(&sim, &robot).is_err());
        assert!(monitored_run(&mut sim, &sdf, Fixture::Limit, None, Some(true)).is_err());
    }
    Ok(())
}
