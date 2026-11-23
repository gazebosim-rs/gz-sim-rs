// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Runtime and recorded-scene export shared by the CLI and offline examples.
#[cfg(feature = "distributed-physics")]
mod grasp_placement;
pub mod live;
#[cfg(feature = "distributed-physics")]
pub use grasp_placement::{parse_grasp_fleet_placement, parse_grasp_placement};
mod operator_checkpoint;
pub use operator_checkpoint::OperatorCheckpoint;
mod web;

use std::error::Error;
pub use web::parked::ParkedFence;
pub use web::primitive_frame;
pub use web::recording::{EXPORT_ASSET_PREFIX, export_archive, export_archive_at};

/// Reports the failed system's own diagnostic, such as a physics engine error,
/// instead of the generic terminal step error that wraps it.
pub fn with_fault(
    simulation: &gz_sim_core::Simulation,
    error: impl Into<Box<dyn Error>>,
) -> Box<dyn Error> {
    match simulation.fault() {
        Some(fault) => fault.to_string().into(),
        None => error.into(),
    }
}
