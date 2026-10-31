// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Consumer-generated Gazebo protobuf values. No application runtime or helpers.

pub mod msgs {
    include!(concat!(env!("OUT_DIR"), "/gz.msgs.rs"));
}

pub use prost;
