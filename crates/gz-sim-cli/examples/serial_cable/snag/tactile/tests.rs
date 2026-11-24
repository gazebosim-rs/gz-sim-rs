// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn encoder_acceleration_compensates_payload_inertia_and_one_short_dropout_recovers() {
    let mut r = JawReadings {
        squeeze_n: [1.2, 1.3],
        force_on_payload_n: [0., 0., WEIGHT],
        ..Default::default()
    };
    let mut observer = Observer::default();
    for _ in 0..80 {
        r.rail_speed_m_s += 3. * DT;
        r.force_on_payload_n[1] = (WEIGHT / 9.81) * 3.;
        let (feed, s) = observer.update(&r);
        assert!(feed.carried && !feed.snag && s.drag_n.abs() < 1e-12);
    }
    r.force_on_payload_n[1] = 0.12;
    for _ in 0..80 {
        observer.update(&r);
    }
    assert!(observer.update(&r).0.snag);
    r.squeeze_n[0] = 0.;
    for _ in 0..7 {
        assert!(!observer.update(&r).0.lost);
    }
    r.squeeze_n[0] = 1.;
    assert_eq!(observer.update(&r).1.pad_loss_ticks, 0);
    r.squeeze_n[1] = 0.;
    for _ in 0..7 {
        assert!(!observer.update(&r).0.lost);
    }
    assert!(observer.update(&r).0.lost);
}

fn world(xml: &str) -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        xml,
        "tactile.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}

#[test]
fn unobserved_obstacle_load_cannot_change_the_decision_but_invalidates_its_receipt() -> Result<()> {
    let xml = Fixture::Snag.xml();
    let sdf = sdformat::load_str(&xml, "tactile.sdf", &sdformat::NoResources)?;
    let mut rows = run(&mut world(&xml)?, &sdf, Fixture::Snag, None)?;
    let reference = rows
        .iter()
        .map(|r| r.truth.particle_state_sha256.clone())
        .collect::<Vec<_>>();
    qualification::measure(Fixture::Snag, &rows, &reference)?;
    let mut bad_prefix = reference.clone();
    bad_prefix[0].clear();
    assert!(qualification::measure(Fixture::Snag, &rows, &bad_prefix).is_err());
    assert!(qualification::measure(Fixture::Limit, &rows, &reference).is_err());

    let mut controller = Controller::new();
    let mut observer = Observer::default();
    for r in &mut rows {
        r.truth.bar = 0.;
        let (feed, signal) = observer.update(&r.jaws);
        assert_eq!(r.stage, controller.stage);
        assert_eq!(r.signal.drag_n, signal.drag_n);
        controller.apply(r.tick, r.jaws.rail_y_m, &feed)?;
    }
    assert!(controller.done(rows.len()));
    assert!(qualification::measure(Fixture::Snag, &rows, &reference).is_err());
    Ok(())
}

#[test]
fn absent_sensing_or_setdown_with_no_upward_support_cannot_complete_the_route() -> Result<()> {
    let xml = Fixture::Clear
        .xml()
        .replace("0 0.46 0.574", "0.85 0.46 0.574");
    let sdf = sdformat::load_str(&xml, "tactile.sdf", &sdformat::NoResources)?;
    let mut sim = world(&xml)?;
    let robot = Robot::load(&sim, &sdf)?;
    assert!(JawReadings::read(&sim, &robot).is_err());
    let error = run(&mut sim, &sdf, Fixture::Clear, None)
        .err()
        .ok_or("false placement")?;
    assert!(
        error.to_string().contains("did not complete in Placing"),
        "{error}"
    );
    let temp = tempfile::tempdir()?;
    assert!(record(temp.path()).is_err());
    Ok(())
}

#[test]
fn incomplete_encoders_or_pose_refuse_observation_and_a_faulted_tick_stops_the_recorder()
-> Result<()> {
    let xml = Fixture::Snag.xml();
    let sdf = sdformat::load_str(&xml, "tactile.sdf", &sdformat::NoResources)?;
    for missing in ["pad", "rail", "spindle"] {
        let mut sim = world(&xml)?;
        let robot = Robot::load(&sim, &sdf)?;
        sim.step_exact(1)?;
        match missing {
            "pad" => {
                sim.world_mut()
                    .entity_mut(robot.pads[0].entity)
                    .remove::<WorldPose>();
            }
            "rail" => {
                sim.world_mut()
                    .entity_mut(robot.rail.entity)
                    .remove::<JointSample>();
            }
            _ => {
                sim.world_mut()
                    .entity_mut(robot.spindle.entity)
                    .remove::<JointSample>();
            }
        }
        assert!(JawReadings::read(&sim, &robot).is_err());
    }
    let mut sim = world(&xml)?;
    sim.world()
        .resource::<gz_sim_core::StepFault>()
        .report(std::io::Error::other("injected fault"));
    assert!(run(&mut sim, &sdf, Fixture::Snag, None).is_err());
    Ok(())
}

#[test]
fn a_bar_reaching_the_rigid_jaws_fails_the_cable_only_recovery_envelope() -> Result<()> {
    let xml = Scene::TooHigh.xml();
    let sdf = sdformat::load_str(&xml, "tactile.sdf", &sdformat::NoResources)?;
    let rows = run(&mut world(&xml)?, &sdf, Fixture::Limit, None)?;
    assert!(rows.iter().any(|r| r.truth.robot_bar_gap < 1e-9));
    let matched = rows
        .iter()
        .map(|r| r.truth.particle_state_sha256.clone())
        .collect::<Vec<_>>();
    assert!(qualification::measure(Fixture::Limit, &rows, &matched).is_err());
    Ok(())
}
