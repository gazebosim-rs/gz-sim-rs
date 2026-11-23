// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/distributed_encounter.rs"]
mod distributed_encounter;
#[path = "../examples/support/seeded.rs"]
mod seeded;
#[path = "../examples/support/train_fixture.rs"]
mod train_fixture;

#[test]
fn rotating_boxes_refine_move_recover_and_publish_the_whole_interval() {
    let result = distributed_encounter::run(
        std::path::Path::new(env!("CARGO_BIN_EXE_physics_worker")),
        20261161424,
        false,
        true,
        None,
    )
    .unwrap();
    assert_eq!(result.exact_public_frames, 335);
    assert!(result.exact_frames_after_restart > 50);
}
