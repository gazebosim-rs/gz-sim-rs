// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

const SHELF: &str = r#"<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity>
<physics name="clock" type="ode"><max_step_size>0.001</max_step_size></physics>
<model name="receiver"><static>true</static><link name="shelf"><collision name="support"><geometry><box><size>1 1 0.1</size></box></geometry></collision></link></model>
<model name="parcel"><link name="box"><pose>0 0 0.11 0 0 0</pose><inertial><mass>1</mass><inertia><ixx>0.001667</ixx><iyy>0.001667</iyy><izz>0.001667</izz></inertia></inertial><collision name="body"><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link></model>
</world></sdf>"#;

fn supported() -> Result<Simulation> {
    let mut sim = load_world_with_sdf_step(
        SHELF,
        "support.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    assert!(Monitor::new(&sim, false).is_err());
    sim.step_exact(500)?;
    Ok(sim)
}

#[test]
fn paused_removal_requests_wake_before_next_parcel_motion() -> Result<()> {
    for external in [false, true] {
        let mut sim = supported()?;
        let parcel = part(&sim, "parcel::box")?;
        let mut monitor = Monitor::new(&sim, !external)?;
        assert!(!monitor.before_idle_step(&mut sim, 250)?);
        let pose = parcel.pose(&sim)?;
        let clock = sim.elapsed();
        sim.set_paused(external);
        if external {
            gz_sim_scene::remove_model(&mut sim, monitor.receiver.id)?;
        }
        assert!(monitor.before_idle_step(&mut sim, 251)?);
        assert_eq!(sim.is_paused(), external);
        assert_eq!(sim.elapsed(), clock);
        let after = parcel.pose(&sim)?;
        assert_eq!(after.translation, pose.translation);
        assert_eq!(after.rotation, pose.rotation);
        let loss = monitor.loss.as_ref().unwrap();
        assert!(loss.retired && (loss.last_upward_impulse_ns - 0.00981).abs() < 1e-6);
        assert_eq!(loss.at_ns, clock.as_nanos());
        assert!(!monitor.before_idle_step(&mut sim, 252)?);
        sim.step_exact(1)?;
        assert!(parcel.pose(&sim)?.translation[2] < pose.translation[2]);
    }
    Ok(())
}

#[test]
fn released_contact_wakes_even_if_the_shelf_still_exists() -> Result<()> {
    let mut sim = supported()?;
    let parcel = part(&sim, "parcel::box")?;
    let mut monitor = Monitor::new(&sim, false)?;
    gz_sim_scene::queue_link_world_wrench(&mut sim, parcel.id, [0., 0., 1000.], [0.; 3])?;
    sim.step_exact(2)?;
    assert!(monitor.before_idle_step(&mut sim, 3)?);
    assert!(!monitor.loss.unwrap().retired);
    Ok(())
}

#[test]
fn missing_or_unbound_workcell_links_refuse_standby() -> Result<()> {
    for (name, component) in [
        ("receiver::shelf", false),
        ("receiver", false),
        ("parcel::box", false),
        ("parcel::box", true),
        ("receiver::shelf", true),
    ] {
        let mut sim = supported()?;
        let entity = part(&sim, name)?.entity;
        let mut link = sim.world_mut().entity_mut(entity);
        if component {
            link.remove::<BodyHandle>();
        } else {
            link.remove::<Name>();
        }
        assert!(Monitor::new(&sim, false).is_err(), "{name}");
    }
    let ordinary = load_world_with_sdf_step(
        SHELF,
        "support.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )?;
    assert!(Monitor::new(&ordinary, false).is_err());
    Ok(())
}

#[test]
fn failed_retirement_restores_pause_state_without_reporting_loss() -> Result<()> {
    for paused in [false, true] {
        let mut sim = supported()?;
        let mut monitor = Monitor::new(&sim, true)?;
        monitor.receiver.id = u64::MAX;
        sim.set_paused(paused);
        let time = sim.elapsed();
        assert!(monitor.before_idle_step(&mut sim, 251).is_err());
        assert!(monitor.loss.is_none());
        assert_eq!(sim.is_paused(), paused);
        assert_eq!(sim.elapsed(), time);
    }
    Ok(())
}

#[test]
fn removal_of_the_parcel_invalidates_its_load_reading() -> Result<()> {
    let mut sim = supported()?;
    let mut monitor = Monitor::new(&sim, false)?;
    let parcel = part(&sim, "parcel")?;
    gz_sim_scene::remove_model(&mut sim, parcel.id)?;
    assert!(monitor.before_idle_step(&mut sim, 1).is_err());
    assert!(monitor.loss.is_none());
    Ok(())
}
