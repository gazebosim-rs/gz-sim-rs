// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local lockstep simulation of centered rigid primitives across Gazebo workers.
//!
//! [`coordinator::Coordinator`] owns two independent native worker processes.
//! [`pool::WorkerPool`] owns three to eight. Each publishes one complete frame per
//! committed physical step. Private Bevy/Rapier worlds preserve native contact
//! caches when a connected solver changes owner. Predictive sweeps include
//! rotation, shared terrain, gravity and pending input before admitting a step.
//!
//! Pool checkpoints preserve native solver state and ordered impulses in a
//! bounded, immutable file. A replacement coordinator can resume a trusted
//! same-build archive after the former coordinator is fenced.
mod adaptive;
pub mod articulation;
pub mod moving;
pub mod multi_sphere;
#[cfg(all(target_os = "linux", feature = "barrier-trace"))]
mod barrier_trace;
pub use adaptive::{adaptive_substeps, adaptive_substeps_with_ground};
pub mod coordinator;
pub mod floating_tree;
pub mod grounded_robot;
mod paged_native;
pub mod physics;
pub mod pool;
pub mod protocol;
pub mod transport;
pub mod trajectory;

mod scene;
pub use scene::SceneBackend;

// Opt-in instrumentation uses the same monotonic timestamps as physical dispatch.
#[inline]
pub(crate) fn measured<T>(
    begin: &'static str,
    end: &'static str,
    operation: impl FnOnce() -> T,
) -> T {
    #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
    crate::barrier_trace::mark(begin, None);
    let result = operation();
    #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
    crate::barrier_trace::mark(end, None);
    #[cfg(not(all(target_os = "linux", feature = "barrier-trace")))]
    let _ = (begin, end);
    result
}
