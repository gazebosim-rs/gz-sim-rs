// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/mixed_islands.rs"]
mod mixed;
#[test]
fn separate_local_solve_rates_survive_loaded_transfer_and_unacknowledged_steps() {
    let tmp = tempfile::tempdir().unwrap();
    let report = mixed::qualify(
        std::path::Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        tmp.path(),
        false,
    )
    .unwrap();
    assert_eq!(report.exact_public_frames, mixed::END);
    assert!(report.island_body_solves < report.comparable_scene_body_solves);
    assert_eq!(
        report.queued_impulses_applied_at,
        report.restarted_from_file_at.map(|t| t + 1)
    );
}
