// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native ownership of a complete prescribed-curve graph on the ordinary Bevy simulation clock.
pub mod checkpoint;
use super::{LoadError, MultiFidelityBackend, PhysicsBackend, PhysicsReset, Simulation};
use crate::deformable::Deformables;
use bevy_ecs::prelude::*;
pub use gz_physics_distributed::moving::WorkerTransport;
pub use gz_physics_distributed::trajectory::arm_launch_owner;
use gz_physics_distributed::{
    articulation::ArticulationError,
    trajectory::{TrajectoryWorkerStep, TrajectoryWorkers, partitions::MaterialPartitions},
};
enum Owner {
    Whole(TrajectoryWorkers),
    Split(MaterialPartitions),
}
impl Owner {
    fn owner(&self) -> usize {
        match self {
            Self::Whole(w) => w.owner(),
            Self::Split(w) => w.owner(),
        }
    }
    fn owners(&self) -> Vec<usize> {
        match self {
            Self::Whole(w) => vec![w.owner()],
            Self::Split(w) => w.owners(),
        }
    }
    fn workers(&self) -> usize {
        match self {
            Self::Whole(w) => w.workers(),
            Self::Split(w) => w.workers(),
        }
    }
    fn group_pid(&self, group: usize, slot: usize) -> Option<u32> {
        match self {
            Self::Whole(w) if group == 0 => w.pid(slot),
            Self::Whole(_) => None,
            Self::Split(w) => w.pid(group, slot),
        }
    }
    fn set_warm_budget(
        &mut self,
        floor: Option<std::time::Duration>,
    ) -> Result<(), ArticulationError> {
        match self {
            Self::Whole(w) => w.set_warm_budget(floor),
            Self::Split(w) => w.set_warm_budget(floor),
        }
    }
    fn set_packed_replies(&mut self, enabled: bool) {
        match self {
            Self::Whole(w) => w.set_packed_replies(enabled),
            Self::Split(w) => w.set_packed_replies(enabled),
        }
    }
    fn transports(&self) -> Vec<WorkerTransport> {
        match self {
            Self::Whole(w) => vec![w.transport()],
            Self::Split(w) => w.transports(),
        }
    }
    fn set_transport(
        &mut self,
        group: usize,
        route: WorkerTransport,
    ) -> Result<(), ArticulationError> {
        match self {
            Self::Split(w) => w.set_transport(group, route),
            Self::Whole(w) if group == 0 => {
                w.set_transport(route);
                Ok(())
            }
            Self::Whole(_) => Err(ArticulationError::Invalid("unknown trajectory group")),
        }
    }
    fn last(&self) -> Option<TrajectoryWorkerStep> {
        match self {
            Self::Whole(w) => w.last(),
            Self::Split(w) => w.last(),
        }
    }
    fn last_shards(&self) -> Vec<TrajectoryWorkerStep> {
        match self {
            Self::Whole(w) => w.last().into_iter().collect(),
            Self::Split(w) => w.last_shards().to_vec(),
        }
    }
    fn set_profiling(&mut self, on: bool) {
        match self {
            Self::Whole(w) => w.set_profiling(on),
            Self::Split(w) => w.set_profiling(on),
        }
    }
    fn reset(&mut self) {
        match self {
            Self::Whole(w) => w.reset(),
            Self::Split(w) => w.reset(),
        }
    }
    fn select(&mut self, slot: usize) -> Result<(), ArticulationError> {
        match self {
            Self::Whole(w) => w.select(slot),
            Self::Split(w) => w.select(slot),
        }
    }
    fn select_group(&mut self, group: usize, slot: usize) -> Result<(), ArticulationError> {
        match self {
            Self::Whole(w) if group == 0 => w.select(slot),
            Self::Whole(_) => Err(ArticulationError::Invalid("no such material partition")),
            Self::Split(w) => w.select_group(group, slot),
        }
    }
}
use gz_sim_core::{Phase, StepFault, UpdateInfo};
use std::{path::Path, sync::Mutex};
#[derive(Resource)]
pub struct TrajectoryProcessPool(Mutex<Owner>);
impl TrajectoryProcessPool {
    pub fn owner(&self) -> usize {
        self.0.lock().unwrap().owner()
    }
    /// Enable timed worker receipts for later steps in this loaded world.
    pub fn set_profiling(&mut self, enabled: bool) {
        self.0.get_mut().unwrap().set_profiling(enabled);
    }
    /// RPC selection per group, in stable placement order.
    pub fn transports(&self) -> Vec<WorkerTransport> {
        self.0.lock().unwrap().transports()
    }
    /// Current process slot for each material group.
    pub fn material_owners(&self) -> Vec<usize> {
        self.0.lock().unwrap().owners()
    }
    pub fn group_pid(&self, group: usize, slot: usize) -> Option<u32> {
        self.0.lock().unwrap().group_pid(group, slot)
    }
    pub fn last_material_steps(&self) -> Vec<TrajectoryWorkerStep> {
        self.0.lock().unwrap().last_shards()
    }
    pub fn workers(&self) -> usize {
        self.0.lock().unwrap().workers()
    }
    pub fn pid(&self, slot: usize) -> Option<u32> {
        self.0.lock().unwrap().group_pid(0, slot)
    }
    pub fn last(&self) -> Option<TrajectoryWorkerStep> {
        self.0.lock().unwrap().last()
    }
}
/// Move the next advancing interval to this owner, after checking the caller's last publication.
pub fn select(sim: &mut Simulation, epoch: u64, tick: u64, slot: usize) -> Result<(), String> {
    let cut = sim.committed_update_info().map_err(|e| e.to_string())?;
    if cut.epoch != epoch || cut.iteration != tick {
        return Err("stale trajectory owner change".into());
    }
    let mut res = sim
        .world_mut()
        .get_resource_mut::<TrajectoryProcessPool>()
        .ok_or("no trajectory process pool")?;
    res.0
        .get_mut()
        .unwrap()
        .select(slot)
        .map_err(|e| e.to_string())
}
/// Move just one material partition after validating the complete scene publication.
pub fn select_material_group(
    sim: &mut Simulation,
    epoch: u64,
    tick: u64,
    group: usize,
    slot: usize,
) -> Result<(), String> {
    let cut = sim.committed_update_info().map_err(|e| e.to_string())?;
    if (cut.epoch, cut.iteration) != (epoch, tick) {
        return Err("stale material owner change".into());
    }
    sim.world_mut()
        .get_resource_mut::<TrajectoryProcessPool>()
        .ok_or("no trajectory process pool")?
        .0
        .get_mut()
        .unwrap()
        .select_group(group, slot)
        .map_err(|e| e.to_string())
}
/// Move a group between private-pipe and loopback Zenoh RPC at its last complete SDF publication.
pub fn select_transport(
    sim: &mut Simulation,
    epoch: u64,
    tick: u64,
    group: usize,
    route: WorkerTransport,
) -> Result<(), String> {
    let i = sim.committed_update_info().map_err(|e| e.to_string())?;
    if sim.is_faulted() || (i.epoch, i.iteration) != (epoch, tick) {
        return Err("invalid committed cut for trajectory transport change".into());
    }
    sim.world_mut()
        .get_resource_mut::<TrajectoryProcessPool>()
        .ok_or("no trajectory process pool")?
        .0
        .get_mut()
        .unwrap()
        .set_transport(group, route)
        .map_err(|e| e.to_string())
}
pub(super) fn install(
    sim: &mut Simulation,
    mut backend: PhysicsBackend,
    worker: &Path,
    name: &str,
) -> Result<(), LoadError> {
    let native = backend
        .downcast_mut::<MultiFidelityBackend>()
        .expect("multifidelity engine");
    native
        .prepare_trajectory_owner()
        .map_err(|e| LoadError::TranslationPath {
            entity: name.into(),
            reason: e.to_string(),
        })?;
    let workers = if let Some(mut soft) = sim.world_mut().get_resource_mut::<Deformables>() {
        let recipe = soft.managed_recipe().map_err(|e| LoadError::Deformable {
            world: name.into(),
            reason: e.to_string(),
        })?;
        let w = if soft.worker_processes > 1 {
            MaterialPartitions::with_transport(
                worker,
                2,
                native,
                recipe,
                soft.worker_processes,
                soft.worker_threads,
                soft.worker_transport,
            )
            .map(Owner::Split)
        } else {
            TrajectoryWorkers::with_material_transport(
                worker,
                2,
                native,
                recipe,
                soft.worker_threads,
                soft.worker_transport,
            )
            .map(Owner::Whole)
        };
        let w = w.and_then(|mut owner| {
            owner.set_warm_budget(soft.warm_budget)?;
            owner.set_packed_replies(soft.packed_replies);
            Ok(owner)
        });
        if w.is_ok() {
            soft.enroll();
        }
        w
    } else {
        TrajectoryWorkers::new(worker, 2, native).map(Owner::Whole)
    }
    .map_err(|e| LoadError::TranslationPath {
        entity: name.into(),
        reason: e.to_string(),
    })?;
    sim.world_mut()
        .insert_resource(TrajectoryProcessPool(Mutex::new(workers)));
    gz_physics::install_with_step(sim, backend, advance)?;
    sim.add_systems(Phase::Reset, reset.after(PhysicsReset));
    Ok(())
}
fn reset(
    mut res: ResMut<TrajectoryProcessPool>,
    mut backend: ResMut<PhysicsBackend>,
    fault: Res<StepFault>,
) {
    res.0.get_mut().unwrap().reset();
    if let Err(e) = backend
        .downcast_mut::<MultiFidelityBackend>()
        .expect("native curve solver")
        .prepare_trajectory_owner()
    {
        fault.report(e);
    }
}
fn advance(
    time: Res<UpdateInfo>,
    fault: Res<StepFault>,
    mut backend: ResMut<PhysicsBackend>,
    mut res: ResMut<TrajectoryProcessPool>,
    soft: Option<ResMut<Deformables>>,
) {
    if time.dt.is_zero() {
        return;
    }
    let native = backend
        .downcast_mut::<MultiFidelityBackend>()
        .expect("installed native curve solver");
    let w = res.0.get_mut().unwrap();
    let result = match (w, soft) {
        (Owner::Whole(w), Some(mut s)) => {
            w.advance_with_material(native, &mut s.managed_bodies(), time.iteration, time.dt)
        }
        (Owner::Whole(w), None) => w.advance(native, time.iteration, time.dt),
        (Owner::Split(w), Some(mut s)) => {
            w.advance(native, &mut s.managed_bodies(), time.iteration, time.dt)
        }
        (Owner::Split(_), None) => Err(ArticulationError::Invalid(
            "material partition lost its public soft rows",
        )),
    };
    if let Err(e) = result {
        fault.report(e);
    }
}
