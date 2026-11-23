// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/curved_ground.rs"]
mod curved_ground;
#[test]
fn curved_pair_and_cylinder_support_survive_warm_migration_lost_reply_and_restart() {
    let tmp = tempfile::tempdir().unwrap();
    let r = curved_ground::run(
        std::path::Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        tmp.path(),
        false,
    )
    .unwrap();
    assert_eq!(r.exact_public_frames, 150);
    assert_eq!(
        r.one_shot_applied_at,
        r.unacknowledged_step_before_disk_restore
    );
    assert!(r.peak_pair_load_ns > 0.1 && r.peak_cylinder_support_ns > 0.1);
}
