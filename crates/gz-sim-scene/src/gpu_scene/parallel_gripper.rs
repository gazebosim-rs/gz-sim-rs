// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Authored three-slider GPU gripper, its public link twists, joint samples and contact loads.
use super::*;
use gz_physics_gpu::{
    ParallelGripperBatch, ParallelGripperLane, simulation::install_parallel_gripper,
};
use std::collections::BTreeMap;
mod configuration;
pub(crate) use configuration::Plan;

#[derive(Resource)]
struct Binding {
    owner: MixedBatchOwner,
    floor: (u64, WorldPose),
    rows: Vec<Row>,
    commands: BTreeMap<u64, Vec<(usize, [f64; 3])>>,
}
struct Row {
    id: u64,
    links: [u64; 4],
    joints: [u64; 3],
    initial: [Transform; 4],
    origin: [f64; 3],
}
pub(crate) fn registered(world: &World) -> bool {
    world.contains_resource::<Binding>()
}
pub(crate) fn valid(world: &World) -> bool {
    let Some(b) = world.get_resource::<Binding>() else {
        return false;
    };
    if physics_owner(world).as_ref() != Some(&b.owner) {
        return false;
    }
    let node =
        |id: u64| gz_sim_core::entity_by_id(world, id).and_then(|e| world.get_entity(e).ok());
    node(b.floor.0).is_some_and(|e| {
        e.get::<EntityKind>() == Some(&EntityKind::Link)
            && e.get::<WorldPose>().is_some_and(|p| {
                p.translation == b.floor.1.translation && p.rotation == b.floor.1.rotation
            })
    }) && b.rows.iter().all(|r| {
        r.links.iter().all(|&id| {
            node(id).is_some_and(|e| {
                e.get::<EntityKind>() == Some(&EntityKind::Link)
                    && e.contains::<WorldPose>()
                    && e.contains::<Velocity>()
            })
        }) && r.joints.iter().all(|&id| {
            node(id).is_some_and(|e| {
                e.get::<EntityKind>() == Some(&EntityKind::Joint)
                    && e.contains::<JointDefinition>()
                    && e.contains::<JointSample>()
            })
        })
    }) && world
        .iter_entities()
        .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::Joint))
        .count()
        == 3 * b.rows.len()
}
pub(super) fn count(sim: &Simulation) -> Option<usize> {
    valid(sim.world()).then(|| sim.world().resource::<Binding>().rows.len())
}
/// Read the last committed jaw, sphere and table reactions in SI units.
pub fn observe(sim: &Simulation, id: u64) -> Result<&ParallelGripperLane, GpuError> {
    let world = sim.world();
    if !valid(world) {
        return Err(GpuError::StaleBoundary);
    }
    let index = index(world, id)?;
    Ok(&world.resource::<ParallelGripperBatch>().lanes()[index])
}
fn index(world: &World, id: u64) -> Result<usize, GpuError> {
    world
        .resource::<Binding>()
        .rows
        .binary_search_by_key(&id, |r| r.id)
        .map_err(|_| GpuError::UnknownEnvironment(id))
}
/// All committed physical lanes in increasing stable environment ID order.
pub fn observations(
    sim: &Simulation,
) -> Result<impl ExactSizeIterator<Item = (u64, &ParallelGripperLane)> + '_, GpuError> {
    let w = sim.world();
    if !valid(w) {
        return Err(GpuError::StaleBoundary);
    }
    Ok(w.resource::<Binding>()
        .rows
        .iter()
        .map(|r| r.id)
        .zip(w.resource::<ParallelGripperBatch>().lanes()))
}
/// Command the lift, left and right prismatic displacements from the authored pose.
pub fn set_targets(sim: &mut Simulation, id: u64, relative: [f64; 3]) -> Result<(), GpuError> {
    if sim.is_faulted() || !valid(sim.world()) {
        return Err(GpuError::StaleBoundary);
    }
    let index = index(sim.world(), id)?;
    let origin = sim.world().resource::<Binding>().rows[index].origin;
    sim.world_mut()
        .resource_mut::<ParallelGripperBatch>()
        .set_targets(index, std::array::from_fn(|i| origin[i] + relative[i]))
}
/// Set every gripper's three displacement commands in the same ascending ID order as
/// [observations]. The full action vector is validated before any target changes.
pub fn set_population_targets(sim: &mut Simulation, actions: &[[f64; 3]]) -> Result<(), GpuError> {
    if sim.is_faulted() || !valid(sim.world()) {
        return Err(GpuError::StaleBoundary);
    }
    sim.world_mut()
        .resource_mut::<ParallelGripperBatch>()
        .set_displacements(actions)
}
pub(super) fn install(
    sim: &mut Simulation,
    p: Plan,
    links: &BTreeMap<String, u64>,
) -> Result<(), LoadError> {
    let batch = ParallelGripperBatch::new(&p.adapter, p.rows.iter().map(|row| row.lane).collect())?;
    let owner = batch.owner();
    let mut rows = Vec::with_capacity(p.rows.len());
    let mut commands = BTreeMap::<u64, Vec<(usize, [f64; 3])>>::new();
    for (index, p) in p.rows.into_iter().enumerate() {
        let ids = p.links.map(|name| links[&name]);
        let body = gz_sim_core::entity_by_id(sim.world(), ids[0]).expect("loaded carriage");
        let parent = sim.world().get::<ChildOf>(body).unwrap().parent();
        let model = sim.world().get::<Name>(parent).unwrap().0.clone();
        let mut joints = [0; 3];
        for (i, j) in p.joints.into_iter().enumerate() {
            let e = spawn_node(
                sim,
                Some(parent),
                &format!("{model}::{}", j.name),
                EntityKind::Joint,
                p.local[i],
                p.poses[i],
            )?;
            sim.world_mut().entity_mut(e).insert((
                JointDefinition(Box::new(j)),
                JointSample {
                    state: gz_physics::JointState {
                        position: 0.,
                        velocity: 0.,
                    },
                    time: Duration::ZERO,
                    epoch: 0,
                },
            ));
            joints[i] = sim.world().get::<EntityId>(e).unwrap().get();
        }
        for (tick, target) in p.commands {
            commands.entry(tick).or_default().push((index, target));
        }
        rows.push(Row {
            id: p.id,
            links: ids,
            joints,
            initial: p.poses,
            origin: p.lane.position,
        });
    }
    let floor = links[&p.floor];
    let e = gz_sim_core::entity_by_id(sim.world(), floor).unwrap();
    let floor = (floor, *sim.world().get::<WorldPose>(e).unwrap());
    install_parallel_gripper(sim, batch)?;
    sim.world_mut().insert_resource(Binding {
        owner,
        rows,
        commands,
        floor,
    });
    sim.add_systems(Phase::PhysicsPrepare, prepare);
    sim.add_systems(Phase::PhysicsCommit, sync.before(sync_models));
    sim.add_systems(Phase::Reset, sync.after(GpuReset).before(sync_models));
    Ok(())
}
fn prepare(world: &mut World) {
    if !valid(world) {
        world
            .resource::<gz_sim_core::StepFault>()
            .report(std::io::Error::other(
                "GPU gripper population, table, link or joint changed",
            ));
        return;
    }
    let info = world.resource::<UpdateInfo>();
    if !info.paused
        && let Some(commands) = world
            .resource::<Binding>()
            .commands
            .get(&info.iteration)
            .cloned()
    {
        let mut batch = world.resource_mut::<ParallelGripperBatch>();
        for (index, target) in commands {
            if let Err(e) = batch.set_targets(index, target) {
                world.resource::<gz_sim_core::StepFault>().report(e);
                return;
            }
        }
    }
}
fn sync(
    info: Res<UpdateInfo>,
    binding: Res<Binding>,
    batch: Res<ParallelGripperBatch>,
    ids: EntityLookup,
    mut links: Query<(&mut WorldPose, &mut Velocity), Without<JointSample>>,
    mut joints: Query<(&mut WorldPose, &mut JointSample)>,
) {
    let mut lookup = ids.reader();
    for (binding, l) in binding.rows.iter().zip(batch.lanes()) {
        for i in 0..4 {
            let (mut pose, mut twist) = links.get_mut(lookup(binding.links[i]).unwrap()).unwrap();
            let initial = binding.initial[i];
            if i == 3 {
                *pose = WorldPose {
                    translation: [l.ball[0], initial.translation[1], l.ball[1]],
                    rotation: (DQuat::from_rotation_y(-l.angle)
                        * DQuat::from_array(initial.rotation))
                    .to_array(),
                };
                *twist = Velocity {
                    linear: [l.velocity[0], 0., l.velocity[1]],
                    angular: [0., -l.spin, 0.],
                };
            } else {
                let x = if i == 0 {
                    initial.translation[0]
                } else {
                    l.position[i]
                };
                *pose = WorldPose {
                    translation: [
                        x,
                        initial.translation[1],
                        initial.translation[2] + l.position[0] - binding.origin[0],
                    ],
                    rotation: initial.rotation,
                };
                *twist = Velocity {
                    linear: [if i == 0 { 0. } else { l.speed[i] }, 0., l.speed[0]],
                    angular: [0.; 3],
                };
                let (mut jp, mut sample) =
                    joints.get_mut(lookup(binding.joints[i]).unwrap()).unwrap();
                *jp = *pose;
                *sample = JointSample {
                    state: gz_physics::JointState {
                        position: l.position[i] - binding.origin[i],
                        velocity: l.speed[i],
                    },
                    time: info.sim_time,
                    epoch: info.epoch,
                };
            }
        }
    }
}
