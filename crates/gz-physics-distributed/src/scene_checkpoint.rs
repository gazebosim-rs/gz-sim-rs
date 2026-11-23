// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Join an application-owned scene manifest and the authoritative native barrier.
use super::*;
use crate::{
    coordinator::RecoveryError,
    pool::{PoolNativeRecovery, ScenePopulation},
};

impl SceneBackend {
    fn check_loaded_frame(&self, frame: &PoolFrame, tick: u64) -> Result<(), RecoveryError> {
        if frame.tick != tick
            || frame.workers.len() != usize::from(self.width)
            || frame.step() != self.step
            || frame.ground() != self.ground.map(|g| g.surface)
            || frame
                .workers
                .iter()
                .map(|w| w.snapshot.bodies.len())
                .sum::<usize>()
                != self.initial.len()
            || self.initial.keys().any(|id| id.raw() >= self.next)
            || self.initial.values().any(|seed| {
                frame
                    .body(seed.id)
                    .is_none_or(|(_, body)| !seed.same_definition(body))
            })
        {
            return Err(RecoveryError::Format(
                "native pool differs from loaded scene identities or clock",
            ));
        }
        Ok(())
    }

    /// Next monotonically issued handle, including all prior loaded resets.
    pub fn next_body_handle(&self) -> u64 {
        self.next
    }

    /// Stage an already committed clone's original definition in a fresh world.
    /// Its authoritative moving state and contacts arrive from validated worker images.
    pub fn stage_recovered_clone(
        &mut self,
        source: BodyId,
        id: BodyId,
        position: [f64; 3],
        orientation: [f64; 4],
    ) -> Result<(), PhysicsError> {
        if self.width < 3 || self.pool.is_some() || self.live.is_some() || id.raw() < self.next {
            return Err(unsupported(
                "recovered clone requires fresh ordered SDF staging",
            ));
        }
        let next = id.raw().checked_add(1).ok_or(PhysicsError::IdOverflow)?;
        let original = self.loading.get(&source).ok_or(PhysicsError::MissingBody)?;
        if original.tier == Tier::Prescribed {
            return Err(unsupported("passive clone template"));
        }
        let seed = Body {
            id: id.raw(),
            position,
            orientation,
            linear: [0.; 3],
            angular: [0.; 3],
            ..original.clone()
        };
        seed.bounds()?;
        self.loading.insert(id, seed.clone());
        self.initial.insert(id, seed);
        self.next = next;
        Ok(())
    }

    /// Skip previously issued, now absent handles while staging a recovered population.
    pub fn advance_staged_body_floor(&mut self, next: u64) -> Result<(), RecoveryError> {
        if self.width < 3 || self.pool.is_some() || self.live.is_some() || next < self.next {
            return Err(RecoveryError::Format("invalid staged native allocator"));
        }
        self.next = next;
        Ok(())
    }

    /// Ordered world-frame contributions already admitted for the next public interval.
    /// A paused tick leaves these inputs pending; an advancing commit consumes them.
    pub fn held_scene_wrenches(&self) -> &[BodyWrench] {
        &self.wrenches
    }

    /// Seal a passive loaded population and its caller-owned scene manifest into a new file.
    /// Worker-acknowledged impulses are represented in the pool image. A pending force/torque
    /// remains the scene caller's next-step input, so this version refuses the barrier.
    pub fn save_loaded_pool_checkpoint(
        &mut self,
        path: &Path,
        tick: u64,
        manifest: &[u8],
    ) -> Result<(), RecoveryError> {
        self.save_population_checkpoint(path, tick, manifest, ScenePopulation::Loaded)
    }

    pub fn save_population_checkpoint(
        &mut self,
        path: &Path,
        tick: u64,
        manifest: &[u8],
        population: ScenePopulation,
    ) -> Result<(), RecoveryError> {
        self.save_population_checkpoint_with_wrenches(path, tick, manifest, population, &[])
    }

    /// Seal caller-owned metadata that records these exact ordered one-tick inputs.
    /// The comparison prevents silently dropping or replacing an admitted contribution.
    pub fn save_population_checkpoint_with_wrenches(
        &mut self,
        path: &Path,
        tick: u64,
        manifest: &[u8],
        population: ScenePopulation,
        represented: &[BodyWrench],
    ) -> Result<(), RecoveryError> {
        if self
            .scoped_placement
            .as_ref()
            .and_then(std::sync::Weak::upgrade)
            .is_some()
        {
            return Err(RecoveryError::Format(
                "placement replay is running in the current ECS step",
            ));
        }
        let pool = self
            .pool
            .as_ref()
            .ok_or(RecoveryError::Format("initialized pool required"))?;
        if (population == ScenePopulation::Loaded
            && (pool.has_population_edits() || self.next != self.initial.len() as u64))
            || self.wrenches != represented
            || !self.before_start.is_empty()
        {
            return Err(RecoveryError::Format(
                "edited population or uncommitted scene input",
            ));
        }
        self.check_loaded_frame(pool.frame(), tick)?;
        if !represented.is_empty() {
            pool.validate_field(self.gravity, represented)?;
        }
        self.pool
            .as_mut()
            .expect("checked pool")
            .save_scene_recovery(path, manifest, population)
    }

    /// Bind a verified native barrier to the staged SDF backend before the application
    /// publishes it. Its loaded reset blueprint and shared ground belong to this backend.
    /// Returns whether every native body is still at its exact original clock-zero state.
    /// The caller must fence the former coordinator before resuming its workers.
    pub fn restore_loaded_pool_checkpoint(
        &mut self,
        plan: PoolNativeRecovery,
        tick: u64,
    ) -> Result<bool, RecoveryError> {
        self.restore_population_checkpoint(plan, tick, ScenePopulation::Loaded)
    }

    pub fn restore_population_checkpoint(
        &mut self,
        plan: PoolNativeRecovery,
        tick: u64,
        population: ScenePopulation,
    ) -> Result<bool, RecoveryError> {
        self.restore_population_checkpoint_with_wrenches(plan, tick, population, Vec::new())
    }

    /// Rebind the authoritative native barrier and its enclosing scene's next-tick
    /// field. All prospective routing is validated before this backend becomes live.
    pub fn restore_population_checkpoint_with_wrenches(
        &mut self,
        plan: PoolNativeRecovery,
        tick: u64,
        population: ScenePopulation,
        held: Vec<BodyWrench>,
    ) -> Result<bool, RecoveryError> {
        if self.pool.is_some()
            || self.live.is_some()
            || self.loading.len() != self.initial.len()
            || (population == ScenePopulation::Loaded
                && (plan.population_high_water().is_some()
                    || self.next != self.initial.len() as u64))
            || plan
                .population_high_water()
                .is_some_and(|high| high >= self.next)
            || !self.wrenches.is_empty()
            || !self.before_start.is_empty()
        {
            return Err(RecoveryError::Format("fresh unedited SDF staging required"));
        }
        self.check_loaded_frame(plan.frame(), tick)?;
        if plan.native_solver_profile() != self.native_profile {
            return Err(RecoveryError::Format(
                "native solver profile differs from loaded scene",
            ));
        }
        let untouched_start = tick == 0
            && self.loading.values().all(|seed| {
                plan.frame()
                    .body(seed.id)
                    .is_some_and(|(_, body)| body == seed)
            });
        if !held.is_empty() {
            plan.validate_pending_scene_field(self.gravity, &held)?;
        }
        let mut resumed = plan.resume(&self.executable)?;
        if !held.is_empty() {
            resumed.validate_field(self.gravity, &held)?;
        }
        resumed.set_packed_observations(self.packed_observations);
        resumed.enable_scene_supervision();
        resumed.set_packed_handoffs(self.packed_handoffs);
        resumed.set_local_helper_limit(self.local_helper_limit);
        resumed.set_periodic_placement(self.periodic_placement)?;
        self.wrenches = held;
        self.pool = Some(resumed);
        self.loading.clear();
        Ok(untouched_start)
    }
}
