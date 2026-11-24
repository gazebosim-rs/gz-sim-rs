// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Optional process-pool placement at an ordinary scene barrier.
use super::*;
use crate::pool::{PoolBalanceDecision, PoolBalancePolicy, PoolLoadMode, PoolLoadWindow};
use std::num::NonZeroU16;

impl SceneBackend {
    /// Enable periodically measured, whole-island placement inside the advancing
    /// transaction. A reset or fresh process keeps the policy and starts a new window.
    pub fn set_periodic_placement(
        &mut self,
        config: Option<crate::pool::PeriodicPlacement>,
    ) -> Result<(), PhysicsError> {
        if self.pool.is_some() || self.live.is_some() {
            return Err(unsupported(
                "configure periodic placement before initializing the SDF scene",
            ));
        }
        if let Some(schedule) = config {
            if self.width < 3 || self.placement == InitialPlacement::CanonicalReference {
                return Err(unsupported(
                    "periodic placement requires an ordinary three- to eight-worker pool",
                ));
            }
            schedule.validate().map_err(native)?;
        }
        self.periodic_placement = config;
        Ok(())
    }
    pub fn placement_status(&self) -> Option<&crate::pool::PlacementStatus> {
        self.pool.as_ref().and_then(WorkerPool::placement_status)
    }

    /// Budget for worker-local native helpers, including after a world reset.
    /// The two-worker coordinator uses a serial native batch in each process.
    pub fn set_local_helper_limit(&mut self, limit: Option<usize>) {
        self.local_helper_limit = limit;
        if let Some(pool) = &mut self.pool {
            pool.set_local_helper_limit(limit);
        }
    }
    /// The helper mask granted at the last published process-pool barrier.
    pub fn last_local_helper_mask(&self) -> Option<u8> {
        self.pool
            .as_ref()
            .and_then(WorkerPool::last_local_helper_mask)
    }
    /// Open a measured load window in an initialized three- through eight-worker scene.
    /// The scene continues stepping normally while the caller holds the token.
    pub fn start_pool_load_sample(
        &mut self,
        span: NonZeroU16,
    ) -> Result<PoolLoadWindow, PhysicsError> {
        self.start_pool_load_sample_with_mode(span, PoolLoadMode::Serial)
    }
    /// Select isolated serial cost or timings under the normal worker schedule.
    pub fn start_pool_load_sample_with_mode(
        &mut self,
        span: NonZeroU16,
        mode: PoolLoadMode,
    ) -> Result<PoolLoadWindow, PhysicsError> {
        self.pool
            .as_mut()
            .ok_or_else(|| unsupported("load sampling requires an initialized process pool"))?
            .start_load_sample_with_mode(span, mode)
            .map_err(native)
    }
    /// At the end of the window, charge the supplied transfer budget against
    /// estimated native savings. Apply at most one whole-island move with
    /// the scene's queued inputs and current gravity. The simulation tick stays fixed.
    pub fn rebalance_sampled_island(
        &mut self,
        window: PoolLoadWindow,
        policy: PoolBalancePolicy,
    ) -> Result<Option<PoolBalanceDecision>, PhysicsError> {
        let pool = self.pool.as_mut().ok_or_else(|| {
            unsupported("measured placement requires an initialized process pool")
        })?;
        let sample = pool.finish_load_sample(window).map_err(native)?;
        let decision = pool
            .plan_rebalance(&sample, policy, self.gravity, &self.wrenches)
            .map_err(native)?;
        if let Some(d) = &decision {
            supervised_pool_edit(pool, |p| {
                p.transfer_component(d.members[0], d.destination)
                    .map(|_| ())
            })?;
        }
        Ok(decision)
    }
}
