// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn mismatched_targets_and_missing_shelf_readings_fail_qualification() -> Result<()> {
    let mut adaptive = Rig::new()?;
    let mut control = Rig::new()?;
    let mut a = adaptive.state()?;
    let b = control.state()?;
    a.targets[0] += 0.1;
    assert!(Errors::default().measure(&a, &b).is_err());

    // Supply the actual overhead rail as the monitored support. It never carries
    // the delivered parcel, so a complete episode must fail its minimum-load gate.
    let overhead =
        gz_sim_core::entity_by_id(adaptive.sim.world(), id(&adaptive.sim, "overhead::rail"))
            .unwrap();
    adaptive.shelf = adaptive
        .sim
        .world()
        .get::<BodyHandle>(overhead)
        .unwrap()
        .id();
    assert!(
        episode(&mut adaptive, &mut control, 2)
            .unwrap_err()
            .to_string()
            .contains("standby bounds exceeded")
    );
    Ok(())
}

#[test]
fn missing_owners_and_observations_stop_an_episode() -> Result<()> {
    let mut adaptive = Rig::new()?;
    let mut control = Rig::new()?;
    adaptive.drives[0] = u64::MAX;
    assert!(episode(&mut adaptive, &mut control, 1).is_err());

    let mut adaptive = Rig::new()?;
    control.model = u64::MAX;
    assert!(episode(&mut adaptive, &mut control, 1).is_err());

    let mut control = Rig::new()?;
    control.shelf = BodyId::from_raw(u64::MAX);
    assert!(control.state().is_err());
    let mut adaptive = Rig::new()?;
    assert!(episode(&mut adaptive, &mut control, 1).is_err());

    let mut adaptive = Rig::new()?;
    adaptive.travel = u64::MAX;
    let mut control = Rig::new()?;
    assert!(episode(&mut adaptive, &mut control, 1).is_err());
    Ok(())
}
