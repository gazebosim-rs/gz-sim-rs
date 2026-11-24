// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use super::*;
#[test]
fn a_pull_must_fit_in_the_public_carry_clock() {
    for bad in [
        "",
        "3700",
        "3700,1",
        "3699,1,0",
        "7701,1,4",
        "7700,2,4",
        "3700,0,4",
        "3700,1,-2",
        "3700,1,NaN",
        "3700,1,inf",
        "3700,1,2,0",
        "3700,none,0",
        "3700,2,absent",
    ] {
        assert!(bad.parse::<Pull>().is_err(), "{bad}");
    }
    assert!("3700,4001,0".parse::<Pull>().is_ok());
    assert!("7700,1,3.5".parse::<Pull>().is_ok());
}

#[test]
fn lost_public_observations_and_invalid_force_targets_stop_the_experiment() -> Result<()> {
    let xml = Trial::GuardedCarry.xml();
    let mut sim = load_world_with_sdf_step(
        xml,
        "pull.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let command = "6200,65,8".parse::<Pull>()?;
    assert!(command.install(&mut sim, Trial::ClearObstacle).is_err());
    command.install(&mut sim, Trial::GuardedCarry)?;
    sim.step_exact(1)?;
    let mut m = Measurements::new(command);
    let parcel = part(&sim, "parcel::box")?;
    let rail = part(&sim, "gripper::travel")?;
    let mut stale = parcel;
    stale.id = u64::MAX;
    assert!(m.before_step(&mut sim, 6202, stale, rail).is_err());
    sim.world_mut()
        .entity_mut(rail.entity)
        .remove::<JointSample>();
    assert!(m.before_step(&mut sim, 6200, parcel, rail).is_err());
    assert!(m.after_alarm(&sim, parcel, rail, true, false).is_err());
    assert!(m.after_alarm(&sim, parcel, rail, false, true).is_err());
    sim.world_mut()
        .entity_mut(parcel.entity)
        .remove::<WorldPose>();
    assert!(ContactState::read(&sim, parcel, rail).is_err());
    Ok(())
}
