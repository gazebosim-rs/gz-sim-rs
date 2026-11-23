// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/friction_incline.rs"]
mod incline;
#[test]
fn ordinary_sdf_sliding_and_static_reactions_match_native_and_analytical_controls() {
    incline::run(std::path::Path::new(env!("CARGO_BIN_EXE_physics_worker"))).unwrap();
}
