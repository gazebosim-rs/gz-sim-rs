// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Control a local Gazebo world through its viewer WebSocket.
#[path = "gz-sim-convoy-control.rs"]
mod operator;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    operator::main()
}
