// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Move a solving contact pair between populated workers. Both bystanders retain ownership.
//! Build `cargo build -p gz-physics-distributed --bin physics_worker`, then run
//! `cargo run -p gz-physics-distributed --example selective_handoff`.
#[path = "support/handoff_options.rs"]
mod options;
#[path = "support/selective_qualification.rs"]
mod qualification;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    options::execute(|worker, capture, loss| qualification::run(worker, capture, loss, 2))
}
