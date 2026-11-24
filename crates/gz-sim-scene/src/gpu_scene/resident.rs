// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! SDF-authored finite-sphere environments with one Bevy publication per device block.
use super::*;
use gz_physics_gpu::{SpherePairBlockBatch as Batch, SpherePairBlockObservation, SphereRotation};

#[derive(Resource)]
struct EnvironmentIds(Vec<u64>);

pub(super) fn batch(sim: &Simulation) -> Option<&Batch> {
    let binding = sim.world().get_resource::<GpuSceneBindings>()?;
    let batch = sim.world().get_resource::<Batch>()?;
    (batch.owner() == binding.owner).then_some(batch)
}
/// Last committed reduction for this SDF environment, including contacts on interior device ticks.
/// Its first and last impact times are seconds relative to the start of the public interval.
pub fn resident_pair_observation(
    sim: &Simulation,
    id: u64,
) -> Result<&SpherePairBlockObservation, GpuError> {
    let batch = batch(sim).ok_or(GpuError::StaleBoundary)?;
    let ids = sim
        .world()
        .get_resource::<EnvironmentIds>()
        .ok_or(GpuError::StaleBoundary)?;
    let index = ids
        .0
        .binary_search(&id)
        .map_err(|_| GpuError::UnknownEnvironment(id))?;
    batch.lanes().get(index).ok_or(GpuError::StaleBoundary)
}
pub(super) fn install(
    sim: &mut Simulation,
    mut plan: Plan,
    links: &BTreeMap<String, u64>,
    ticks: u32,
    rotation: SphereRotation,
    worker: Option<&std::path::Path>,
) -> Result<(), LoadError> {
    plan.initial.sort_unstable_by_key(|e| e.id);
    let lanes = plan
        .initial
        .iter()
        .map(|e| match e.state {
            EnvironmentState::SpherePair(pair) => pair,
            _ => unreachable!("validated sphere pair profile"),
        })
        .collect();
    let batch = Batch::with_rotation(&plan.adapter, lanes, rotation)?;
    bind(
        sim,
        &plan.initial,
        plan.links,
        &[],
        links,
        batch.owner(),
        plan.every,
    );
    sim.world_mut()
        .insert_resource(EnvironmentIds(plan.initial.iter().map(|e| e.id).collect()));
    if let Some(width) = plan.pair_workers {
        let path = match worker {
            Some(exe) => exe.to_path_buf(),
            None => std::env::current_exe()
                .map_err(|e| GpuError::Device(e.to_string()))?
                .with_file_name(format!("gz_pair_worker{}", std::env::consts::EXE_SUFFIX)),
        };
        let pool = gz_physics_gpu::pair_process::PairProcessPool::new(&path, &plan.adapter, width)?;
        gz_physics_gpu::pair_process::install(sim, batch, ticks, pool)?;
    } else {
        gz_physics_gpu::simulation::install_sphere_pair_block(sim, batch, ticks)?;
    }
    sim.add_systems(Phase::PhysicsPrepare, validate_bindings);
    sim.add_systems(Phase::PhysicsCommit, sync.before(sync_models));
    sim.add_systems(Phase::Reset, sync.after(GpuReset).before(sync_models));
    if plan.every.is_some() {
        sim.add_systems(Phase::PhysicsCommit, sample);
        sim.add_systems(Phase::Reset, sample.after(GpuReset));
    }
    Ok(())
}
fn sync(
    batch: Res<Batch>,
    bindings: Res<GpuSceneBindings>,
    ids: EntityLookup,
    mut links: Query<(&mut WorldPose, &mut Velocity)>,
) {
    let mut lookup = ids.reader();
    for row in &bindings.rows {
        let entity = lookup(row.link).expect("prepared loaded sphere link");
        let (mut pose, mut velocity) = links.get_mut(entity).expect("prepared link state");
        let pair = &batch.lanes()[row.index].state;
        let b = if row.body == BodySlot::A {
            &pair.a
        } else {
            &pair.b
        };
        pose.translation = b.position;
        pose.rotation = b.orientation;
        velocity.linear = b.velocity;
        velocity.angular = b.angular_velocity;
    }
}
fn sample(batch: Res<Batch>, info: Res<UpdateInfo>, mut output: ResMut<GpuSceneObservations>) {
    if (info.dt.is_zero() && info.epoch == output.epoch)
        || !info.iteration.is_multiple_of(output.every)
    {
        return;
    }
    for (e, obs) in output.environments.iter_mut().zip(batch.lanes()) {
        e.state = EnvironmentState::SpherePair(obs.state);
    }
    output.epoch = info.epoch;
    output.tick = info.iteration;
    output.sim_time = info.sim_time;
}
