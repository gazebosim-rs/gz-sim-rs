// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/replay.rs"]
mod replay;
#[path = "../examples/support/split_run.rs"]
mod split_run;
#[test]
fn fission_under_load_preserves_native_trajectory_through_handoffs_and_replacement() {
    split_run::run(
        std::path::Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        None,
    )
    .unwrap();
}
