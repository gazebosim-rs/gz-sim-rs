// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/dense_separation.rs"]
mod dense;
use std::path::Path;

#[test]
fn diagonally_departing_fleet_releases_the_loaded_pair_and_returns_after_recovery()
-> dense::Result<()> {
    let out = tempfile::tempdir()?;
    let r = dense::qualify(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        out.path(),
        true,
    )?;
    // Under the original speed-limit cubes the 24 departing bodies stay joined
    // through tick 76 and rejoin at 158. At 0.6s the closest fleet member is
    // already more than 10 m from the pair, clear of even two 4 m travel balls.
    assert!(
        r.first_pair_only_component_tick
            .is_some_and(|tick| tick <= 60)
    );
    assert!(r.first_fleet_all_coarse_tick.is_some_and(|tick| tick <= 61));
    assert!(
        r.first_rejoined_component_tick
            .is_some_and(|tick| tick > 158)
    );
    assert!(r.planned_body_solves < 42_520);
    assert_eq!(r.exact_public_frames, dense::END);
    assert_eq!(r.native_body_comparisons, 26 * dense::END);
    assert_eq!(r.lost_ack_retried_tick, r.migration_tick + 1);
    assert_eq!(r.worker_replacement_barrier, r.lost_ack_retried_tick);
    assert_eq!(r.archive_restore_barrier, r.worker_replacement_barrier + 1);
    assert_eq!(r.resumed_ticks, dense::END - r.archive_restore_barrier);
    let (first, matching) = dense::verify_previous_checkpoint(
        Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        out.path(),
    )?;
    assert_eq!(
        (first, matching),
        (r.archive_restore_barrier, r.resumed_ticks)
    );
    Ok(())
}
