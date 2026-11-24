// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Public joint drives and measurements of an SDF-admitted GPU arm.
use super::*;
mod population;
use bevy_ecs::component::Mutable;
use gz_physics_gpu::{
    FreeSphere, PlanarObjectBatch, PlanarObjectLane, simulation::install_planar_object,
};
use gz_physics_gpu::{PlanarToolBatch, PlanarToolLane, simulation::install_planar_tool};
use population::Population;
use std::collections::BTreeMap;
struct Motor {
    configuration: joint_controller::Parameters,
    target: f64,
}
struct Binding {
    id: u64,
    links: [u64; 2],
    object: Option<(u64, [f64; 4])>,
    joints: [u64; 2],
    base: [f64; 3],
    origin: [f64; 2],
    commands: BTreeMap<u64, [f64; 2]>,
    motors: [Option<Motor>; 2],
}
#[derive(Resource)]
pub struct ToolBindings {
    owner: MixedBatchOwner,
    rows: Vec<Binding>,
    plane: (u64, WorldPose),
}
fn checked_world(world: &World) -> Option<&dyn Population> {
    let b = population::get(world)?;
    let binding = world.get_resource::<ToolBindings>()?;
    (b.identity() == (binding.owner.clone(), binding.rows.len())).then_some(b)
}
pub(super) fn count(sim: &Simulation) -> Option<usize> {
    Some(checked_world(sim.world())?.identity().1)
}
pub(crate) fn joint_bindings_valid(world: &World) -> Option<bool> {
    let bindings = world.get_resource::<ToolBindings>()?;
    let healthy = checked_world(world).is_some()
        && bindings.rows.iter().all(|r| {
            r.joints.iter().all(|id| {
                gz_sim_core::entity_by_id(world, *id).is_some_and(|e| {
                    world.get::<EntityKind>(e) == Some(&EntityKind::Joint)
                        && world.get::<JointSample>(e).is_some()
                        && world.get::<JointDefinition>(e).is_some()
                })
            })
        })
        && world
            .iter_entities()
            .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::Joint))
            .count()
            == 2 * bindings.rows.len();
    Some(healthy)
}
fn index(sim: &Simulation, id: u64) -> Result<usize, GpuError> {
    checked_world(sim.world()).ok_or(GpuError::StaleBoundary)?;
    sim.world()
        .resource::<ToolBindings>()
        .rows
        .binary_search_by_key(&id, |r| r.id)
        .map_err(|_| GpuError::UnknownEnvironment(id))
}
fn motor_index(bindings: &ToolBindings, id: u64) -> Option<(usize, usize)> {
    bindings.rows.iter().enumerate().find_map(|(row, r)| {
        let axis = r.joints.iter().position(|&j| j == id)?;
        r.motors[axis].as_ref().map(|_| (row, axis))
    })
}
/// Resolve a controller's physical GPU axis to its stable loaded child link.
pub fn controlled_child(sim: &Simulation, id: u64) -> Option<u64> {
    checked_world(sim.world())?;
    let bindings = sim.world().resource::<ToolBindings>();
    let (row, axis) = motor_index(bindings, id)?;
    Some(bindings.rows[row].links[axis])
}
pub(crate) fn binding_matches(
    sim: &Simulation,
    id: u64,
    cfg: &JointControllerConfiguration,
) -> bool {
    let bindings = sim.world().resource::<ToolBindings>();
    checked_world(sim.world()).is_some()
        && motor_index(bindings, id).is_some_and(|(row, axis)| {
            &bindings.rows[row].motors[axis]
                .as_ref()
                .unwrap()
                .configuration
                .publication
                == cfg
        })
}
pub(crate) fn set_joint_command(
    sim: &mut Simulation,
    id: u64,
    entity: Entity,
    target: f64,
) -> Result<(), JointControllerCommandError> {
    let bindings = sim.world().resource::<ToolBindings>();
    let (row, axis) =
        motor_index(bindings, id).ok_or(JointControllerCommandError::MissingController(id))?;
    let cfg = &bindings.rows[row].motors[axis]
        .as_ref()
        .unwrap()
        .configuration;
    if sim.is_faulted()
        || sim.world().get::<JointControllerConfiguration>(entity) != Some(&cfg.publication)
    {
        return Err(JointControllerCommandError::InvalidPhysics);
    }
    let coordinate = sim
        .world()
        .get::<JointSample>(entity)
        .ok_or(JointControllerCommandError::InvalidPhysics)?
        .state
        .position;
    joint_controller::target_velocity(
        cfg.publication.mode,
        target,
        coordinate,
        cfg.max_velocity,
        sim.step_duration(),
    )
    .map_err(|_| JointControllerCommandError::InvalidCommand)?;
    sim.world_mut().resource_mut::<ToolBindings>().rows[row].motors[axis]
        .as_mut()
        .unwrap()
        .target = target;
    Ok(())
}
/// Observe the most recent committed arm state and signed N s contact impulses.
pub fn observe(sim: &Simulation, id: u64) -> Result<&PlanarToolLane, GpuError> {
    let i = index(sim, id)?;
    Ok(checked_world(sim.world())
        .ok_or(GpuError::StaleBoundary)?
        .tool(i))
}
/// Observe the free object's committed XZ state and its distinct tool and floor impulses.
pub fn observe_object(sim: &Simulation, id: u64) -> Result<Option<&FreeSphere>, GpuError> {
    let i = index(sim, id)?;
    Ok(checked_world(sim.world())
        .ok_or(GpuError::StaleBoundary)?
        .object(i))
}
/// Command both relative joint displacements in radians at this boundary.
/// A target authored for the next tick in the SDF takes precedence at that tick.
pub fn set_targets(sim: &mut Simulation, id: u64, radians: [f64; 2]) -> Result<(), GpuError> {
    let i = index(sim, id)?;
    if sim.is_faulted() || !joints::bindings_valid(sim.world()) {
        return Err(GpuError::StaleBoundary);
    }
    let row = &sim.world().resource::<ToolBindings>().rows[i];
    if row.motors.iter().any(Option::is_some) {
        return Err(GpuError::InvalidArmTarget(i));
    }
    let origin = row.origin;
    let target = [radians[0] + origin[0], radians[1] + origin[1]];
    if let Some(mut b) = sim.world_mut().get_resource_mut::<PlanarObjectBatch>() {
        b.set_target_angles(i, target)
    } else {
        sim.world_mut()
            .resource_mut::<PlanarToolBatch>()
            .set_target_angles(i, target)
    }
}
pub(super) fn install(
    sim: &mut Simulation,
    plan: planar_tool_config::Plan,
    links: &BTreeMap<String, u64>,
) -> Result<(), LoadError> {
    let owner = if let [a] = plan.arms.as_slice()
        && let Some((_, object)) = &a.object
    {
        let b = PlanarObjectBatch::new(
            &plan.adapter,
            vec![PlanarObjectLane {
                tool: a.lane,
                object: *object,
            }],
        )?;
        let owner = b.owner();
        install_planar_object(sim, b)?;
        schedule::<PlanarObjectBatch>(sim);
        owner
    } else {
        let b = PlanarToolBatch::new(&plan.adapter, plan.arms.iter().map(|a| a.lane).collect())?;
        let owner = b.owner();
        install_planar_tool(sim, b)?;
        schedule::<PlanarToolBatch>(sim);
        owner
    };
    let mut rows = vec![];
    for a in plan.arms {
        let object = a.object.map(|(name, _)| {
            let id = links[&name];
            let node = gz_sim_core::entity_by_id(sim.world(), id).expect("loaded object");
            (id, sim.world().get::<WorldPose>(node).unwrap().rotation)
        });
        let links = a.links.map(|name| links[&name]);
        let e = gz_sim_core::entity_by_id(sim.world(), links[0]).expect("loaded link");
        let parent = sim
            .world()
            .get::<ChildOf>(e)
            .expect("loaded link model")
            .parent();
        let mut ids = [0; 2];
        for (i, j) in a.joints.into_iter().enumerate() {
            let model = &sim.world().get::<Name>(parent).expect("model name").0;
            let name = format!("{model}::{}", j.name);
            let (local, pose) = a.joint_poses[i];
            let e = spawn_node(sim, Some(parent), &name, EntityKind::Joint, local, pose)?;
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
            if let Some(c) = &a.controllers[i] {
                sim.world_mut().entity_mut(e).insert(c.publication.clone());
            }
            ids[i] = sim.world().get::<EntityId>(e).unwrap().get();
        }
        rows.push(Binding {
            id: a.id,
            object,
            links,
            joints: ids,
            base: a.base,
            origin: a.origin,
            commands: a.commands,
            motors: a.controllers.map(|c| {
                c.map(|configuration| Motor {
                    target: configuration.initial_target,
                    configuration,
                })
            }),
        });
    }
    let plane = links[&plan.plane];
    let e = gz_sim_core::entity_by_id(sim.world(), plane).expect("loaded floor");
    let pose = *sim.world().get::<WorldPose>(e).unwrap();
    sim.world_mut().insert_resource(ToolBindings {
        owner,
        rows,
        plane: (plane, pose),
    });
    sim.add_systems(Phase::Reset, reset_targets);
    Ok(())
}
fn schedule<B: Population + Resource<Mutability = Mutable>>(sim: &mut Simulation) {
    sim.add_systems(Phase::PhysicsPrepare, prepare::<B>);
    sim.add_systems(Phase::PhysicsCommit, sync::<B>.before(sync_models));
    sim.add_systems(Phase::Reset, sync::<B>.after(GpuReset).before(sync_models));
}
type BoundNodes<'w, 's> = Query<
    'w,
    's,
    (
        &'static EntityKind,
        &'static WorldPose,
        Has<Velocity>,
        Has<JointSample>,
        Option<&'static JointControllerConfiguration>,
    ),
>;
fn prepare<B: Population + Resource<Mutability = Mutable>>(
    info: Res<UpdateInfo>,
    binding: Option<Res<ToolBindings>>,
    batch: Option<ResMut<B>>,
    ids: EntityLookup,
    nodes: BoundNodes,
    fault: Res<gz_sim_core::StepFault>,
) {
    let (Some(binding), Some(mut batch)) = (binding, batch) else {
        fault.report(std::io::Error::other("GPU tool population is missing"));
        return;
    };
    let mut lookup = ids.reader();
    let valid = batch.identity() == (binding.owner.clone(), binding.rows.len())
        && lookup(binding.plane.0)
            .and_then(|e| nodes.get(e).ok())
            .is_some_and(|(k, p, _, _, _)| {
                *k == EntityKind::Link
                    && p.translation == binding.plane.1.translation
                    && p.rotation == binding.plane.1.rotation
            })
        && binding.rows.iter().all(|r| {
            r.links
                .iter()
                .copied()
                .chain(r.object.map(|o| o.0))
                .all(|id| {
                    lookup(id)
                        .and_then(|e| nodes.get(e).ok())
                        .is_some_and(|(k, _, v, _, _)| *k == EntityKind::Link && v)
                })
                && r.joints.iter().enumerate().all(|(axis, id)| {
                    lookup(*id)
                        .and_then(|e| nodes.get(e).ok())
                        .is_some_and(|(k, _, _, j, c)| {
                            *k == EntityKind::Joint
                                && j
                                && c == r.motors[axis]
                                    .as_ref()
                                    .map(|m| &m.configuration.publication)
                        })
                })
        });
    if !valid {
        fault.report(std::io::Error::other(
            "GPU tool scene identity or floor pose changed",
        ));
        return;
    }
    if !info.paused {
        for (i, r) in binding.rows.iter().enumerate() {
            let angles = batch.tool(i).joint_angles();
            for (axis, motor) in r.motors.iter().enumerate() {
                if let Some(m) = motor {
                    let cfg = &m.configuration;
                    match joint_controller::target_velocity(
                        cfg.publication.mode,
                        m.target,
                        angles[axis] - r.origin[axis],
                        cfg.max_velocity,
                        info.dt,
                    ) {
                        Ok(speed) => {
                            if let Err(e) = batch.motor(i, axis, speed) {
                                fault.report(e);
                                return;
                            }
                        }
                        Err(e) => {
                            fault.report(e);
                            return;
                        }
                    }
                }
            }
            if let Some(target) = r.commands.get(&info.iteration)
                && let Err(e) = batch.target(i, *target)
            {
                fault.report(e);
                return;
            }
        }
    }
}
fn reset_targets(mut bindings: ResMut<ToolBindings>) {
    for m in bindings
        .rows
        .iter_mut()
        .flat_map(|r| r.motors.iter_mut().flatten())
    {
        m.target = m.configuration.initial_target;
    }
}
#[allow(clippy::type_complexity)]
fn sync<B: Population + Resource<Mutability = Mutable>>(
    info: Res<UpdateInfo>,
    batch: Res<B>,
    bindings: Res<ToolBindings>,
    ids: EntityLookup,
    mut links: Query<(&mut WorldPose, &mut Velocity), Without<JointSample>>,
    mut joints: Query<(&mut WorldPose, &mut JointSample)>,
) {
    let mut lookup = ids.reader();
    for (row, r) in bindings.rows.iter().enumerate() {
        let l = batch.tool(row);
        if let (Some((id, rotation)), Some(ball)) = (r.object, batch.object(row)) {
            let (mut pose, mut velocity) = links.get_mut(lookup(id).unwrap()).unwrap();
            let [x, z] = ball.position;
            let [vx, vz] = ball.velocity;
            *pose = WorldPose {
                translation: [r.base[0] + x, r.base[1], r.base[2] + z],
                rotation,
            };
            *velocity = Velocity {
                linear: [vx, 0., vz],
                angular: [0.; 3],
            };
        }
        let continuous = l.joint_angles();
        let [q1, q2] = l.arm.angle.map(|p| p[1].atan2(p[0]));
        let [elbow, _] = l.arm.points();
        let [w1, w2] = l.arm.speed;
        for (i, angle) in continuous.into_iter().enumerate() {
            let (mut pose, mut vel) = links.get_mut(lookup(r.links[i]).unwrap()).unwrap();
            let [x, z] = if i == 0 { [0.; 2] } else { elbow };
            *pose = WorldPose {
                translation: [r.base[0] + x, r.base[1], r.base[2] + z],
                rotation: DQuat::from_rotation_y(if i == 0 { -q1 } else { -q1 - q2 }).to_array(),
            };
            *vel = Velocity {
                linear: [-w1 * z, 0., w1 * x],
                angular: [0., if i == 0 { -w1 } else { -w1 - w2 }, 0.],
            };
            let (mut jp, mut sample) = joints.get_mut(lookup(r.joints[i]).unwrap()).unwrap();
            *jp = *pose;
            *sample = JointSample {
                state: gz_physics::JointState {
                    position: angle - r.origin[i],
                    velocity: l.arm.speed[i],
                },
                time: info.sim_time,
                epoch: info.epoch,
            };
        }
    }
}
