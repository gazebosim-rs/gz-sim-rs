// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Mixed GPU physics for explicitly independent SDF environments.
use super::*;
use gz_physics_gpu::{
    BodySlot, MixedBatch, MixedBatchOwner,
    simulation::{GpuReset, install_mixed},
};
pub use gz_physics_gpu::{
    Environment, EnvironmentState, ExecutionPlacement, GpuError, NativeContact,
    NativeHandoffRefusal, PairSolver, PlacementChange,
    simulation::{BatchBoundary, batch_boundary},
};
pub mod automatic;
mod config;
mod resident;
pub(super) use config::{Plan, is_plugin};
pub use gz_physics_gpu::{
    SpherePairBlockCheckpoint,
    pair_process::{PairProcessMetrics, PairProcessPool, set_width as set_resident_worker_count},
};
pub use resident::resident_pair_observation;
#[derive(Clone, Copy)]
struct Row {
    link: u64,
    index: usize,
    id: u64,
    body: BodySlot,
}
#[derive(Resource)]
pub struct GpuSceneBindings {
    owner: MixedBatchOwner,
    rows: Vec<Row>,
    planes: Vec<(u64, WorldPose)>,
}
impl GpuSceneBindings {
    /// Resolve a stable Gazebo link ID to its logical GPU environment and body role.
    pub fn resolve(&self, link: u64) -> Option<(u64, BodySlot)> {
        self.rows
            .binary_search_by_key(&link, |r| r.link)
            .ok()
            .map(|i| (self.rows[i].id, self.rows[i].body))
    }
}
/// Enabled by the SDF observation request. Holds the last due sensor-style sample.
/// ECS link poses and twists still commit on every 1 ms physics step.
#[derive(Resource)]
pub struct GpuSceneObservations {
    every: u64,
    epoch: u64,
    tick: u64,
    sim_time: Duration,
    environments: Vec<Environment>,
}
impl GpuSceneObservations {
    pub fn every(&self) -> u64 {
        self.every
    }
    pub fn epoch(&self) -> u64 {
        self.epoch
    }
    pub fn tick(&self) -> u64 {
        self.tick
    }
    pub fn sim_time(&self) -> Duration {
        self.sim_time
    }
    pub fn environments(&self) -> &[Environment] {
        &self.environments
    }
}
/// Owning allocation of the single physical GPU population installed in this world.
pub(crate) fn physics_owner(world: &bevy_ecs::world::World) -> Option<MixedBatchOwner> {
    match (
        world.get_resource::<MixedBatch>(),
        world.get_resource::<gz_physics_gpu::SpherePairBlockBatch>(),
    ) {
        (Some(m), None) => Some(m.owner()),
        (None, Some(b)) => Some(b.owner()),
        _ => None,
    }
}
/// Committed execution owners of the loaded physical population (CPU, GPU).
pub fn execution_counts(sim: &Simulation) -> Option<(usize, usize)> {
    if let Some(batch) = checked_mixed(sim) {
        let cpu = batch.cpu_environments();
        return Some((cpu, batch.environments().len() - cpu));
    }
    resident::batch(sim).map(|batch| (0, batch.lanes().len()))
}
fn checked_mixed(sim: &Simulation) -> Option<&MixedBatch> {
    let binding = sim.world().get_resource::<GpuSceneBindings>()?;
    let batch = sim.world().get_resource::<MixedBatch>()?;
    (batch.owner() == binding.owner).then_some(batch)
}
/// Resolve the current execution owner for one logical environment in this loaded world.
pub fn environment_placement(
    sim: &Simulation,
    id: u64,
) -> Result<ExecutionPlacement, gz_physics_gpu::GpuError> {
    if let Some(batch) = checked_mixed(sim) {
        return batch.execution_placement(id);
    }
    resident_pair_observation(sim, id).map(|_| ExecutionPlacement::Gpu)
}
/// Change the execution owner for stable environment IDs in this loaded scene.
pub fn set_environment_placement(
    sim: &mut Simulation,
    at: BatchBoundary,
    ids: &[u64],
    target: ExecutionPlacement,
) -> Result<PlacementChange, gz_physics_gpu::GpuError> {
    if checked_mixed(sim).is_none() {
        return Err(gz_physics_gpu::GpuError::StaleBoundary);
    }
    let changed = gz_physics_gpu::simulation::set_environment_placement(sim, at, ids, target)?;
    if !ids.is_empty()
        && sim
            .world()
            .contains_resource::<automatic::AutomaticPlacement>()
    {
        let owner = execution_counts(sim).and_then(|(cpu, gpu)| match (cpu, gpu) {
            (0, _) => Some(ExecutionPlacement::Gpu),
            (_, 0) => Some(ExecutionPlacement::Cpu),
            _ => None,
        });
        sim.world_mut()
            .resource_mut::<automatic::AutomaticPlacement>()
            .manual(at, owner);
    }
    Ok(changed)
}
/// Inspect the physical solver of a marked pair by its stable environment identity.
pub fn pair_solver(sim: &Simulation, id: u64) -> Result<PairSolver, gz_physics_gpu::GpuError> {
    checked_mixed(sim)
        .ok_or(GpuError::StaleBoundary)?
        .pair_solver(id)
}
/// Native impulses and retained contact history from the most recent native tick.
pub fn native_contact(
    sim: &Simulation,
    id: u64,
) -> Result<Option<NativeContact>, gz_physics_gpu::GpuError> {
    checked_mixed(sim)
        .ok_or(GpuError::StaleBoundary)?
        .native_contact(id)
}
/// Move complete free sphere pairs or sphere/plane contacts between Rapier and the specialized GPU solver.
pub fn set_pair_solver(
    sim: &mut Simulation,
    at: BatchBoundary,
    ids: &[u64],
    target: PairSolver,
) -> Result<PlacementChange, gz_physics_gpu::GpuError> {
    if sim.is_faulted() || at != batch_boundary(sim) || checked_mixed(sim).is_none() {
        return Err(gz_physics_gpu::GpuError::StaleBoundary);
    }
    sim.world_mut()
        .resource_mut::<MixedBatch>()
        .set_pair_solver(at.tick, ids, target)
}
/// Queue a world-frame impulse at a loaded sphere's center for the next advancing tick.
pub fn queue_link_impulse(
    sim: &mut Simulation,
    at: BatchBoundary,
    link: u64,
    impulse: [f64; 3],
) -> Result<(), gz_physics_gpu::GpuError> {
    let bindings = sim
        .world()
        .get_resource::<GpuSceneBindings>()
        .ok_or(gz_physics_gpu::GpuError::UnknownSceneLink(link))?;
    let entity = gz_sim_core::entity_by_id(sim.world(), link)
        .ok_or(gz_physics_gpu::GpuError::UnknownSceneLink(link))?;
    if sim.world().get::<EntityKind>(entity) != Some(&EntityKind::Link)
        || sim.world().get::<WorldPose>(entity).is_none()
        || sim.world().get::<Velocity>(entity).is_none()
    {
        return Err(gz_physics_gpu::GpuError::UnknownSceneLink(link));
    }
    let (id, body) = bindings
        .resolve(link)
        .ok_or(gz_physics_gpu::GpuError::UnknownSceneLink(link))?;
    if !sim
        .world()
        .get_resource::<MixedBatch>()
        .is_some_and(|b| b.owner() == bindings.owner)
    {
        return Err(gz_physics_gpu::GpuError::StaleBoundary);
    }
    gz_physics_gpu::simulation::queue_environment_impulses(
        sim,
        at,
        &[gz_physics_gpu::CentralImpulse { id, body, impulse }],
    )
}
pub(super) fn install(
    sim: &mut Simulation,
    plan: Plan,
    links: &BTreeMap<String, u64>,
    worker: Option<&std::path::Path>,
) -> Result<(), LoadError> {
    if let Some((ticks, rotation)) = plan.block {
        return resident::install(sim, plan, links, ticks, rotation, worker);
    }
    let batch = MixedBatch::from_fixed_scene(&plan.adapter, plan.initial)?
        .with_initial_cpu(&plan.initial_cpu)?
        .with_guarded_native_pairs(&plan.native_pairs)?;
    let batch = match plan.native_workers {
        Some((workers, false)) => batch.with_native_staging(workers)?,
        Some((workers, true)) => batch.with_overlapped_native_staging(workers)?,
        None => batch,
    };
    bind(
        sim,
        batch.environments(),
        plan.links,
        &plan.planes,
        links,
        batch.owner(),
        plan.every,
    );
    if plan.every.is_some() {
        sim.add_systems(Phase::PhysicsCommit, sample);
        sim.add_systems(Phase::Reset, sample.after(GpuReset));
    }
    install_mixed(sim, batch)?;
    sim.add_systems(Phase::PhysicsPrepare, validate_bindings);
    sim.add_systems(Phase::PhysicsCommit, sync.before(sync_models));
    sim.add_systems(Phase::Reset, sync.after(GpuReset).before(sync_models));
    if let Some(config) = plan.automatic {
        automatic::install_auto_placement(sim, config).map_err(|e| LoadError::GpuScene {
            entity: "GpuBatch".into(),
            reason: e.to_string(),
        })?;
    }
    Ok(())
}
fn bind(
    sim: &mut Simulation,
    initial: &[Environment],
    associations: Vec<(String, u64, BodySlot)>,
    plane_names: &[String],
    links: &BTreeMap<String, u64>,
    owner: MixedBatchOwner,
    every: Option<u64>,
) {
    let mut rows = Vec::new();
    let mut initial_velocities = BTreeMap::new();
    for (link, id, body) in associations {
        let index = initial
            .binary_search_by_key(&id, |env| env.id)
            .expect("checked logical identity");
        let link = *links.get(&link).expect("validated SDF link constructed");
        let entity = gz_sim_core::entity_by_id(sim.world(), link).expect("loaded stable ID");
        let (pose, velocity) = state(&initial[index].state, body);
        initial_velocities.insert(link, DVec3::from_array(velocity.linear));
        sim.world_mut().entity_mut(entity).insert((pose, velocity));
        rows.push(Row {
            link,
            index,
            id,
            body,
        });
    }
    // Existing IMU producers differentiate world link velocity. Seed their first
    // interval and reset history from the authored GPU center velocity.
    let world = sim.world_mut();
    let mut sensors = world.query::<&mut Imu>();
    for mut sensor in sensors.iter_mut(world) {
        if let Some(initial) = initial_velocities.get(&sensor.link) {
            sensor.initial_velocity = *initial;
            sensor.previous_velocity = *initial;
        }
    }
    rows.sort_unstable_by_key(|r| r.link);
    let planes = plane_names
        .iter()
        .map(|name| {
            let id = links[name];
            let e = gz_sim_core::entity_by_id(sim.world(), id).expect("loaded support plane");
            (
                id,
                *sim.world()
                    .get::<WorldPose>(e)
                    .expect("loaded support pose"),
            )
        })
        .collect();
    sim.world_mut().insert_resource(GpuSceneBindings {
        owner,
        rows,
        planes,
    });
    if let Some(every) = every {
        sim.world_mut().insert_resource(GpuSceneObservations {
            every,
            epoch: 0,
            tick: 0,
            sim_time: Duration::ZERO,
            environments: initial.to_vec(),
        });
    }
}
fn sample(batch: Res<MixedBatch>, info: Res<UpdateInfo>, mut output: ResMut<GpuSceneObservations>) {
    if (info.dt.is_zero() && info.epoch == output.epoch)
        || !(info.iteration.is_multiple_of(output.every))
    {
        return;
    }
    output.environments.copy_from_slice(batch.environments());
    output.epoch = info.epoch;
    output.tick = info.iteration;
    output.sim_time = info.sim_time;
}
fn validate_bindings(
    bindings: Option<Res<GpuSceneBindings>>,
    batch: Option<Res<MixedBatch>>,
    block: Option<Res<gz_physics_gpu::SpherePairBlockBatch>>,
    identities: EntityLookup,
    links: Query<(&WorldPose, &Velocity, &EntityKind)>,
    fault: Res<StepFault>,
) {
    let mut lookup = identities.reader();
    let owner = match (batch, block) {
        (Some(m), None) => Some(m.owner()),
        (None, Some(b)) => Some(b.owner()),
        _ => None,
    };
    let healthy = bindings.zip(owner).is_some_and(|(bindings, owner)| {
        bindings.owner == owner
            && bindings.rows.iter().all(|r| {
                lookup(r.link)
                    .and_then(|e| links.get(e).ok())
                    .is_some_and(|(_, _, kind)| *kind == EntityKind::Link)
            })
            && bindings.planes.iter().all(|(id, pose)| {
                lookup(*id)
                    .and_then(|e| links.get(e).ok())
                    .is_some_and(|(p, _, kind)| {
                        *kind == EntityKind::Link
                            && p.translation == pose.translation
                            && p.rotation == pose.rotation
                    })
            })
    });
    if !healthy {
        fault.report(std::io::Error::other(
            "GPU scene link or physics owner changed before dispatch",
        ));
    }
}
fn state(s: &EnvironmentState, body: BodySlot) -> (WorldPose, Velocity) {
    let (p, q, v, w) = match s {
        EnvironmentState::SpherePlane(s) => {
            (s.position, s.orientation, s.velocity, s.angular_velocity)
        }
        EnvironmentState::SpherePair(s) => {
            let b = if body == BodySlot::A { s.a } else { s.b };
            (b.position, b.orientation, b.velocity, b.angular_velocity)
        }
        EnvironmentState::SupportedSpherePair(s) => {
            let b = if body == BodySlot::A {
                s.pair.a
            } else {
                s.pair.b
            };
            (b.position, b.orientation, b.velocity, b.angular_velocity)
        }
    };
    (
        WorldPose {
            translation: p,
            rotation: q,
        },
        Velocity {
            linear: v,
            angular: w,
        },
    )
}
fn sync(
    batch: Res<MixedBatch>,
    bindings: Res<GpuSceneBindings>,
    ids: EntityLookup,
    mut links: Query<(&mut WorldPose, &mut Velocity)>,
) {
    let mut lookup = ids.reader();
    for row in &bindings.rows {
        let entity = lookup(row.link).expect("prepared link or restored initial topology");
        let (mut p, mut v) = links.get_mut(entity).expect("prepared link pose and twist");
        (*p, *v) = state(&batch.environments()[row.index].state, row.body);
    }
}
