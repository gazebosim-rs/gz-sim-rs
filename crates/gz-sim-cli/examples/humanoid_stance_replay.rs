// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Convert committed scene samples into a validated native replay.
#[path = "support/pose_replay.rs"]
mod pose_replay;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    pose_replay::run()
}
