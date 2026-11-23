// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One-call placement replay through all ordinary Bevy phases.
use super::*;
use crate::pool::PlacementRecord;
use gz_physics::PhysicsBackend;
use gz_sim_core::Simulation;

impl SceneBackend {
    /// Optional placement committed by the last island step of this coordinator.
    pub fn last_island_placement(&self) -> Option<&PlacementRecord> {
        self.pool
            .as_ref()
            .and_then(WorkerPool::last_island_placement)
    }

    /// Advance one ordinary SDF/Bevy tick using the supplied optional placement.
    /// Admission against the current field occurs before any ECS system runs. The
    /// physics phase checks the actual field again after any pre-step systems.
    /// SDF callers use `gz_sim_scene::step_with_placement` to fence deferred
    /// public edits before entering this lower-level scheduling bridge.
    /// A custom system that changes those conditions can fault the simulation
    /// under its ordinary schedule contract. The placement is scoped to this
    /// call and cannot survive an early exit or become an input to a later tick.
    pub fn step_simulation_with_placement(
        sim: &mut Simulation,
        record: &PlacementRecord,
    ) -> Result<(), PhysicsError> {
        sim.validate_steps(1).map_err(native)?;
        let clock = sim.committed_update_info().map_err(native)?;
        let scope;
        {
            let mut backend = sim
                .world_mut()
                .get_resource_mut::<PhysicsBackend>()
                .ok_or_else(|| {
                    unsupported("placement replay requires an installed physics backend")
                })?;
            let scene = backend
                .downcast_mut::<Self>()
                .ok_or_else(|| unsupported("placement replay requires a distributed scene"))?;
            let pool = scene.pool.as_ref().ok_or_else(|| {
                unsupported("placement replay requires an initialized process pool")
            })?;
            if scene.refinement != Refinement::Island || pool.frame().tick != clock.iteration {
                return Err(unsupported(
                    "placement replay requires a synchronized island-refining scene",
                ));
            }
            pool.validate_island_placement(scene.gravity, &scene.wrenches, record)
                .map_err(native)?;
            scope = Arc::new(record.clone());
            scene.scoped_placement = Some(Arc::downgrade(&scope));
        }
        let result = sim.step_exact(1);
        let mut backend = sim
            .world_mut()
            .get_resource_mut::<PhysicsBackend>()
            .ok_or_else(|| unsupported("the scheduled physics backend was removed"))?;
        let scene = backend
            .downcast_mut::<Self>()
            .ok_or_else(|| unsupported("the scheduled physics backend was replaced"))?;
        scene.scoped_placement = None;
        result.map_err(native)?;
        if scene.last_island_placement() != Some(record) {
            return Err(native(
                "the schedule did not commit the requested placement",
            ));
        }
        Ok(())
    }
}
