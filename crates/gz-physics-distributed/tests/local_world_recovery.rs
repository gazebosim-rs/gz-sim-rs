// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/world_profile_scene.rs"]
mod fixture;
#[path = "../examples/support/local_world_recovery.rs"]
mod trial;
use std::path::Path;

#[test]
fn simultaneous_loaded_worlds_retain_each_receipt_through_transfer_and_restart() -> trial::Result<()>
{
    let dir = tempfile::tempdir()?;
    let report = trial::qualify(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        &dir.path().join("native.gzpool"),
        fixture::stacks(),
        0,
        |_| Ok(()),
    )?;
    assert_eq!(report.exact_body_states, trial::END * 8);
    assert_eq!(report.fully_loaded_ticks, trial::END - trial::SETTLED + 1);
    Ok(())
}

#[path = "../examples/support/heterogeneous_worlds.rs"]
mod unequal;

#[test]
fn slab_and_twenty_four_loaded_spheres_migrate_and_restore_together() -> trial::Result<()> {
    let dir = tempfile::tempdir()?;
    let groups = unequal::seeds(unequal::Layout::First);
    let heavy_base = groups[0][0].id;
    let weight_impulse =
        groups[0].iter().map(|b| b.mass).sum::<f64>() * -unequal::GRAVITY[2] * 0.01;
    let bodies = groups.iter().map(Vec::len).sum::<usize>() as u64;
    let report = trial::qualify(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        &dir.path().join("native.gzpool"),
        groups,
        0,
        |f| {
            if f.tick >= trial::SETTLED {
                let impulse = f
                    .floors
                    .iter()
                    .find(|p| p.body == heavy_base)
                    .ok_or("missing slab load")?
                    .normal_impulse;
                assert!(
                    (impulse - weight_impulse).abs() < 0.001,
                    "tick {}, slab load {}",
                    f.tick,
                    impulse
                );
            }
            Ok(())
        },
    )?;
    assert!(report.compared_ticks >= unequal::TICKS as u64);
    assert_eq!(report.exact_body_states, trial::END * bodies);
    assert_eq!(
        report.fully_loaded_ticks,
        trial::END - unequal::WARMUP as u64 + 1
    );
    Ok(())
}
