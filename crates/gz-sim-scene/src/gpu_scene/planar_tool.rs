// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Public joint drives and measurements of an SDF-admitted GPU arm.
use super::*;
use gz_physics_gpu::{PlanarToolBatch, PlanarToolLane, simulation::install_planar_tool};
use std::collections::BTreeMap;
struct Motor {
    configuration: joint_controller::Parameters,
    target: f64,
}
struct Binding {
    id: u64,
    links: [u64; 2],
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
pub(super) fn checked(sim: &Simulation) -> Option<&PlanarToolBatch> {
    let b = sim.world().get_resource::<PlanarToolBatch>()?;
    let binding = sim.world().get_resource::<ToolBindings>()?;
    (b.owner() == binding.owner && b.lanes().len() == binding.rows.len()).then_some(b)
}
pub(crate) fn joint_bindings_valid(world: &World) -> Option<bool> {
    let bindings = world.get_resource::<ToolBindings>()?;
    let healthy = world
        .get_resource::<PlanarToolBatch>()
        .is_some_and(|b| b.owner() == bindings.owner && b.lanes().len() == bindings.rows.len())
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
    checked(sim).ok_or(GpuError::StaleBoundary)?;
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
    checked(sim)?;
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
    checked(sim).is_some()
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
    Ok(&sim.world().resource::<PlanarToolBatch>().lanes()[i])
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
    let mut batch = sim.world_mut().resource_mut::<PlanarToolBatch>();
    batch.set_target_angles(i, [radians[0] + origin[0], radians[1] + origin[1]])
}
pub(super) fn install(
    sim: &mut Simulation,
    plan: planar_tool_config::Plan,
    links: &BTreeMap<String, u64>,
) -> Result<(), LoadError> {
    let batch = PlanarToolBatch::new(&plan.adapter, plan.arms.iter().map(|a| a.lane).collect())?;
    let owner = batch.owner();
    let mut rows = vec![];
    for a in plan.arms {
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
    install_planar_tool(sim, batch)?;
    sim.add_systems(Phase::PhysicsPrepare, prepare);
    sim.add_systems(Phase::Reset, reset_targets);
    sim.add_systems(Phase::PhysicsCommit, sync.before(sync_models));
    sim.add_systems(Phase::Reset, sync.after(GpuReset).before(sync_models));
    Ok(())
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
fn prepare(
    info: Res<UpdateInfo>,
    binding: Option<Res<ToolBindings>>,
    batch: Option<ResMut<PlanarToolBatch>>,
    ids: EntityLookup,
    nodes: BoundNodes,
    fault: Res<gz_sim_core::StepFault>,
) {
    let (Some(binding), Some(mut batch)) = (binding, batch) else {
        fault.report(std::io::Error::other("GPU tool population is missing"));
        return;
    };
    let mut lookup = ids.reader();
    let valid = batch.owner() == binding.owner
        && batch.lanes().len() == binding.rows.len()
        && lookup(binding.plane.0)
            .and_then(|e| nodes.get(e).ok())
            .is_some_and(|(k, p, _, _, _)| {
                *k == EntityKind::Link
                    && p.translation == binding.plane.1.translation
                    && p.rotation == binding.plane.1.rotation
            })
        && binding.rows.iter().all(|r| {
            r.links.iter().all(|id| {
                lookup(*id)
                    .and_then(|e| nodes.get(e).ok())
                    .is_some_and(|(k, _, v, _, _)| *k == EntityKind::Link && v)
            }) && r.joints.iter().enumerate().all(|(axis, id)| {
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
            let angles = batch.lanes()[i].joint_angles();
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
                            if let Err(e) = batch.set_motor_speed(i, axis, speed) {
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
                && let Err(e) = batch.set_target_angles(i, *target)
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
fn sync(
    info: Res<UpdateInfo>,
    batch: Res<PlanarToolBatch>,
    bindings: Res<ToolBindings>,
    ids: EntityLookup,
    mut links: Query<(&mut WorldPose, &mut Velocity), Without<JointSample>>,
    mut joints: Query<(&mut WorldPose, &mut JointSample)>,
) {
    let mut lookup = ids.reader();
    for (l, r) in batch.lanes().iter().zip(&bindings.rows) {
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
