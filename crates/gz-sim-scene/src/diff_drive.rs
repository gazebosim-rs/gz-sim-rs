// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! At most eight original loaded wheel pairs; native motors own the physical response.
use super::*;
use gz_physics::{JointId, JointVelocityMotor, PhysicsError};
mod config;
mod model;

const MAX_LOADED_DRIVES: usize = 8;

#[derive(Component, Clone, Debug, PartialEq, Eq)]
#[component(immutable)]
pub struct DiffDriveConfiguration {
    pub model_name: String,
    pub topic: String,
    pub odom_topic: String,
    pub frame_id: String,
    pub child_frame_id: String,
}

#[derive(Component, Debug)]
pub struct DiffDriveSample {
    pub time: Duration,
    pub epoch: u64,
    /// Encoder-integrated local odometry x/y/yaw, not body ground truth.
    pub pose: [f64; 3],
    pub twist: [f64; 2],
}

#[derive(Component, Clone)]
pub(super) struct Drive {
    owner: u64,
    publication: DiffDriveConfiguration,
    joints: [JointId; 2],
    force: [f64; 2],
    speed: [f64; 2],
    linear: [f64; 2],
    angular: [f64; 2],
    acceleration: [[f64; 2]; 2],
    prepared_velocity: Option<[f64; 2]>,
    desired: [f64; 2],
    geometry: model::Geometry,
    odometry: Option<model::Odometry>,
    schedule: SampleSchedule,
}

pub(super) struct Plan(config::Parameters);
pub(super) fn is_plugin(plugin: &sdformat::Plugin) -> bool {
    config::is_plugin(plugin)
}
pub(super) fn parse(model: &Model, local: bool) -> Result<Option<Plan>, LoadError> {
    config::parse_for_engine(model, local).map(|value| value.map(Plan))
}
impl Plan {
    pub(super) fn names(&self) -> impl Iterator<Item = &str> {
        [self.0.left.as_str(), self.0.right.as_str()].into_iter()
    }
    pub(super) fn validate(
        &self,
        model: &Model,
        sim: &Simulation,
        engine: Option<&str>,
        placement: (bool, bool),
        pose: Transform,
    ) -> Result<(), LoadError> {
        require(
            matches!(engine, Some("rapier" | "multifidelity")) && placement.0 && !placement.1,
            &model.scoped_name,
            "DiffDrive requires a top-level dynamic Rapier model",
        )?;
        require(
            sim.world()
                .iter_entities()
                .filter(|entity| entity.contains::<Drive>())
                .take(MAX_LOADED_DRIVES + 1)
                .count()
                < MAX_LOADED_DRIVES,
            &model.scoped_name,
            "multiple differential-drive systems",
        )?;
        require(
            pose.rotation[0].abs() <= POSE_TOLERANCE && pose.rotation[1].abs() <= POSE_TOLERANCE,
            &model.scoped_name,
            "non-planar DiffDrive initial frame",
        )?;
        model::Geometry::new(self.0.radius, self.0.separation).map_err(|_| {
            LoadError::Unsupported {
                entity: model.scoped_name.clone(),
                feature: "unrepresentable DiffDrive geometry",
            }
        })?;
        Ok(())
    }
}

// Local Gazebo velocity commands are constraint targets. The original Rapier-only
// drive uses its existing acceleration-based motors. Both respect each wheel's effort cap.
fn validate_motor(
    backend: &PhysicsBackend,
    joint: JointId,
    motor: Option<JointVelocityMotor>,
) -> Result<(), PhysicsError> {
    if backend.engine() == "multifidelity" {
        backend.validate_joint_velocity_servo(joint, motor)
    } else {
        backend.validate_joint_velocity_motor(joint, motor)
    }
}

fn set_motor(
    backend: &mut PhysicsBackend,
    joint: JointId,
    motor: Option<JointVelocityMotor>,
) -> Result<(), PhysicsError> {
    if backend.engine() == "multifidelity" {
        backend.set_joint_velocity_servo(joint, motor)
    } else {
        backend.set_joint_velocity_motor(joint, motor)
    }
}

pub(super) fn build(
    sim: &mut Simulation,
    model: &Model,
    entity: Entity,
    backend: &PhysicsBackend,
    plan: &Plan,
) -> Result<(), LoadError> {
    let parameters = &plan.0;
    let mut ids = Vec::new();
    for name in [&parameters.left, &parameters.right] {
        let scoped = format!("{}::{name}", model.scoped_name);
        let handle = sim
            .world()
            .iter_entities()
            .find(|node| node.get::<Name>().is_some_and(|name| name.0 == scoped))
            .and_then(|node| node.get::<JointHandle>())
            .ok_or(LoadError::Unsupported {
                entity: scoped,
                feature: "missing DiffDrive wheel binding",
            })?;
        ids.push(handle.id());
    }
    let joints = [ids[0], ids[1]];
    for (joint, force) in joints.into_iter().zip(parameters.wheel_force) {
        validate_motor(
            backend,
            joint,
            Some(JointVelocityMotor {
                target_velocity: 0.0,
                max_force: force,
            }),
        )?;
    }
    let mut schedule =
        SampleSchedule::new(parameters.rate).map_err(|_| LoadError::Unsupported {
            entity: model.scoped_name.clone(),
            feature: "unrepresentable odometry rate",
        })?;
    schedule
        .take_sample(Duration::ZERO, false)
        .map_err(|_| LoadError::Unsupported {
            entity: model.scoped_name.clone(),
            feature: "unrepresentable odometry cadence",
        })?;
    let geometry =
        model::Geometry::new(parameters.radius, parameters.separation).map_err(|_| {
            LoadError::Unsupported {
                entity: model.scoped_name.clone(),
                feature: "unrepresentable DiffDrive geometry",
            }
        })?;
    let angles = [
        backend
            .joint_state(joints[0])
            .ok_or(LoadError::Unsupported {
                entity: model.scoped_name.clone(),
                feature: "missing drive wheel state",
            })?
            .position,
        backend
            .joint_state(joints[1])
            .ok_or(LoadError::Unsupported {
                entity: model.scoped_name.clone(),
                feature: "missing drive wheel state",
            })?
            .position,
    ];
    let odometry = model::Odometry::new(geometry, angles, Duration::ZERO).map_err(|_| {
        LoadError::Unsupported {
            entity: model.scoped_name.clone(),
            feature: "invalid initial drive encoders",
        }
    })?;
    let owner = sim
        .world()
        .get::<EntityId>(entity)
        .expect("loaded model identity")
        .get();
    sim.world_mut().entity_mut(entity).insert((
        parameters.publication.clone(),
        Drive {
            owner,
            publication: parameters.publication.clone(),
            joints,
            force: parameters.wheel_force,
            speed: parameters.wheel_speed,
            linear: parameters.linear,
            angular: parameters.angular,
            acceleration: parameters.acceleration,
            prepared_velocity: (backend.engine() == "multifidelity").then_some([0.0; 2]),
            desired: [0.0; 2],
            geometry,
            odometry: Some(odometry),
            schedule,
        },
    ));
    Ok(())
}

#[derive(Debug, thiserror::Error)]
pub enum DiffDriveCommandError {
    #[error("model {0} does not exist or does not own a loaded differential drive")]
    MissingDrive(u64),
    #[error("drive physics bindings are missing or belong to another backend")]
    ForeignPhysics,
    #[error("drive command is not finite and representable")]
    InvalidCommand,
    #[error(transparent)]
    Physics(#[from] PhysicsError),
}

/// Replace the last valid command without stepping. A paused driver may receive
/// commands; its native motor targets change only on the next advancing step.
pub fn set_diff_drive_command(
    sim: &mut Simulation,
    model: u64,
    linear: f64,
    angular: f64,
) -> Result<(), DiffDriveCommandError> {
    if !linear.is_finite() || !angular.is_finite() {
        return Err(DiffDriveCommandError::InvalidCommand);
    }
    let world = sim.world();
    if !initial::matches_physics_owner(world) || !joints::bindings_valid(world) {
        return Err(DiffDriveCommandError::ForeignPhysics);
    }
    let entity = gz_sim_core::entity_by_id(world, model)
        .ok_or(DiffDriveCommandError::MissingDrive(model))?;
    let drive = world
        .get::<Drive>(entity)
        .ok_or(DiffDriveCommandError::MissingDrive(model))?;
    if world.get::<EntityKind>(entity) != Some(&EntityKind::Model)
        || world.get::<DiffDriveConfiguration>(entity) != Some(&drive.publication)
    {
        return Err(DiffDriveCommandError::MissingDrive(model));
    }
    let desired = [
        linear.clamp(drive.linear[0], drive.linear[1]),
        angular.clamp(drive.angular[0], drive.angular[1]),
    ];
    let targets = drive
        .geometry
        .bounded_targets(desired[0], desired[1], drive.speed)
        .map_err(|_| DiffDriveCommandError::InvalidCommand)?;
    let backend = world
        .get_resource::<PhysicsBackend>()
        .ok_or(DiffDriveCommandError::ForeignPhysics)?;
    for ((joint, target), force) in drive.joints.into_iter().zip(targets).zip(drive.force) {
        validate_motor(
            backend,
            joint,
            Some(JointVelocityMotor {
                target_velocity: target,
                max_force: force,
            }),
        )?;
    }
    if !binding_matches(world, model) {
        return Err(DiffDriveCommandError::MissingDrive(model));
    }
    sim.world_mut()
        .get_mut::<Drive>(entity)
        .expect("preflighted loaded drive")
        .desired = desired;
    Ok(())
}

#[derive(Resource)]
pub(super) struct Installed;
pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>()
        || !sim
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<Drive>())
    {
        return;
    }
    sim.add_systems(Phase::PreStep, apply);
    sim.add_systems(Phase::PostStep, observe);
    sim.add_systems(Phase::Reset, reset.after(sync_attached_poses));
    sim.world_mut().insert_resource(Installed);
}

pub(super) fn retire_if_unused(
    sim: &mut Simulation,
) -> Result<(), bevy_ecs::schedule::ScheduleError> {
    if !sim.world().contains_resource::<Installed>()
        || sim
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<Drive>())
    {
        return Ok(());
    }
    sim.initialize_schedules()?;
    sim.remove_systems_in_set(Phase::PreStep, apply)?;
    sim.remove_systems_in_set(Phase::PostStep, observe)?;
    sim.remove_systems_in_set(Phase::Reset, reset)?;
    sim.world_mut().remove_resource::<Installed>();
    Ok(())
}

#[derive(Debug, thiserror::Error)]
#[error("differential drive {entity:?}: {message}")]
struct DriveFault {
    entity: Entity,
    message: String,
}

fn axis_matches(left: &Option<sdformat::Axis>, right: &Option<sdformat::Axis>) -> bool {
    match (left, right) {
        (None, None) => true,
        (Some(left), Some(right)) => {
            left.xyz.map(f64::to_bits) == right.xyz.map(f64::to_bits)
                && left.expressed_in == right.expressed_in
                && [
                    left.lower,
                    left.upper,
                    left.effort,
                    left.velocity,
                    left.stiffness,
                    left.dissipation,
                ]
                .map(f64::to_bits)
                    == [
                        right.lower,
                        right.upper,
                        right.effort,
                        right.velocity,
                        right.stiffness,
                        right.dissipation,
                    ]
                    .map(f64::to_bits)
                && left.dynamics == right.dynamics
                && left.mimic == right.mimic
        }
        _ => false,
    }
}

fn definition_matches(left: &JointDefinition, right: &JointDefinition) -> bool {
    let left = &left.0;
    let right = &right.0;
    left.name == right.name
        && left.kind == right.kind
        && left.parent == right.parent
        && left.child == right.child
        && left.pose == right.pose
        && axis_matches(&left.axis, &right.axis)
        && axis_matches(&left.axis2, &right.axis2)
        && left.screw_thread_pitch.to_bits() == right.screw_thread_pitch.to_bits()
        && left.physics_xml == right.physics_xml
        && left.source == right.source
        && left.sensors.is_empty()
        && right.sensors.is_empty()
}

/// Check only this active original owner. No mutation, IO or unrelated-joint scan.
pub fn diff_drive_binding_matches(sim: &Simulation, id: u64) -> bool {
    binding_matches(sim.world(), id)
}

/// Restoration-only structural witness against immutable original metadata.
/// Does not admit changed public metadata for commands, steps or live streams.
pub fn diff_drive_binding_matches_configuration(
    sim: &Simulation,
    id: u64,
    trusted: &DiffDriveConfiguration,
) -> bool {
    binding_matches_configuration(sim.world(), id, trusted)
}

fn binding_matches(world: &World, id: u64) -> bool {
    let Some(entity) = gz_sim_core::entity_by_id(world, id) else {
        return false;
    };
    let Some(configuration) = world.get::<DiffDriveConfiguration>(entity) else {
        return false;
    };
    binding_matches_configuration(world, id, configuration)
}

fn binding_matches_configuration(world: &World, id: u64, trusted: &DiffDriveConfiguration) -> bool {
    if !initial::matches_physics_owner(world) {
        return false;
    }
    let Some(initial) = world.get_resource::<initial::InitialScene>() else {
        return false;
    };
    let Some(original) = initial.diff_drive_identities().find(|row| row.id == id) else {
        return false;
    };
    let Some(blueprint) = initial.diff_drive_blueprint(id) else {
        return false;
    };
    let Some(entity) = gz_sim_core::entity_by_id(world, id) else {
        return false;
    };
    let Some(drive) = world.get::<Drive>(entity) else {
        return false;
    };
    let Some(parent_id) = original.parent else {
        return false;
    };
    let Some(parent) = gz_sim_core::entity_by_id(world, parent_id) else {
        return false;
    };
    let Some(backend) = world.get_resource::<PhysicsBackend>() else {
        return false;
    };
    if id == 0
        || parent_id == 0
        || original.kind != EntityKind::Model
        || world.get::<EntityId>(entity).map(EntityId::get) != Some(id)
        || world.get::<EntityKind>(entity) != Some(&original.kind)
        || !world
            .get::<Name>(entity)
            .is_some_and(|name| name.0 == original.name)
        || world.get::<ChildOf>(entity).map(ChildOf::parent) != Some(parent)
        || world.get::<EntityKind>(parent) != Some(&EntityKind::World)
        || !world.get::<Name>(parent).is_some_and(|name| {
            original.name == format!("{}::{}", name.0, original.configuration.model_name)
        })
        || trusted != original.configuration
        || drive.owner != id
        || blueprint.owner != id
        || drive.publication != *original.configuration
        || drive.publication != blueprint.publication
        || drive.joints != blueprint.joints
        || drive.joints[0] == drive.joints[1]
        || drive.force != blueprint.force
        || drive.speed != blueprint.speed
        || drive.linear != blueprint.linear
        || drive.angular != blueprint.angular
        || drive.acceleration != blueprint.acceleration
        || drive.geometry != blueprint.geometry
    {
        return false;
    }
    let mut children = std::collections::BTreeSet::new();
    for joint in drive.joints {
        let Some(original_joint) = initial.diff_drive_wheel_identity(id, joint) else {
            return false;
        };
        let Some(node) = gz_sim_core::entity_by_id(world, original_joint.id) else {
            return false;
        };
        let Some(handle) = world.get::<JointHandle>(node) else {
            return false;
        };
        let Some(definition) = world.get::<JointDefinition>(node) else {
            return false;
        };
        let Some(child) = gz_sim_core::entity_by_id(world, handle.child()) else {
            return false;
        };
        let Some(parent_link) = world.iter_entities().find(|link| {
            link.get::<ChildOf>().map(ChildOf::parent) == Some(entity)
                && link.get::<Name>().is_some_and(|name| {
                    name.0 == format!("{}::{}", original.name, definition.0.parent)
                })
        }) else {
            return false;
        };
        if original_joint.id == 0
            || original_joint.kind != EntityKind::Joint
            || original_joint.parent != Some(id)
            || world.get::<EntityId>(node).map(EntityId::get) != Some(original_joint.id)
            || world.get::<EntityKind>(node) != Some(&original_joint.kind)
            || !world.get::<Name>(node).is_some_and(|name| {
                name.0 == original_joint.name
                    && name.0 == format!("{}::{}", original.name, definition.0.name)
            })
            || world.get::<ChildOf>(node).map(ChildOf::parent) != Some(entity)
            || !world.entity(node).contains::<joints::JointNode>()
            || world.get::<JointSample>(node).is_none()
            || handle.id() != original_joint.handle.id()
            || handle.child() != original_joint.handle.child()
            || joints::input_binding(world, node) != Some(joint)
            || !definition_matches(definition, original_joint.definition)
            || !children.insert(handle.child())
            || world.get::<EntityKind>(child) != Some(&EntityKind::Link)
            || world.get::<ChildOf>(child).map(ChildOf::parent) != Some(entity)
            || !world
                .get::<Name>(child)
                .is_some_and(|name| name.0 == format!("{}::{}", original.name, definition.0.child))
            || !world
                .get::<BodyHandle>(child)
                .is_some_and(|body| backend.joint_child(joint) == Some(body.id()))
            || parent_link.get::<EntityKind>() != Some(&EntityKind::Link)
            || parent_link.get::<BodyHandle>().is_none()
            || backend.joint_state(joint).is_none()
        {
            return false;
        }
    }
    true
}

fn ownership_fault(entity: Entity) -> DriveFault {
    DriveFault {
        entity,
        message: "invalid loaded drive ownership".into(),
    }
}

fn active_drives(world: &World) -> Result<Vec<Entity>, DriveFault> {
    let rows: Vec<_> = world
        .iter_entities()
        .filter(|row| row.contains::<Drive>() || row.contains::<DiffDriveConfiguration>())
        .take(MAX_LOADED_DRIVES + 1)
        .map(|row| row.id())
        .collect();
    let Some(initial) = world.get_resource::<initial::InitialScene>() else {
        return if let Some(first) = rows.first() {
            Err(ownership_fault(*first))
        } else {
            Ok(Vec::new())
        };
    };
    let originals: Vec<_> = initial
        .diff_drive_identities()
        .take(MAX_LOADED_DRIVES + 1)
        .collect();
    let mut expected = std::collections::BTreeSet::new();
    // An original entity still alive without BOTH components is corruption;
    // a truly removed model has no current entity and is a retired owner.
    for original in &originals {
        if let Some(entity) = gz_sim_core::entity_by_id(world, original.id) {
            if !binding_matches(world, original.id) {
                return Err(ownership_fault(entity));
            }
            expected.insert(entity);
        }
    }
    if let Some(first) = rows.first()
        && (originals.len() > MAX_LOADED_DRIVES
            || rows.len() > MAX_LOADED_DRIVES
            || !initial::matches_physics_owner(world)
            || !joints::bindings_valid(world))
    {
        return Err(ownership_fault(*first));
    }
    let mut ids = std::collections::BTreeSet::new();
    let mut joints = std::collections::BTreeSet::new();
    for entity in &rows {
        let Some(drive) = world.get::<Drive>(*entity) else {
            return Err(ownership_fault(*entity));
        };
        if !expected.remove(entity)
            || !ids.insert(drive.owner)
            || !drive.joints.into_iter().all(|joint| joints.insert(joint))
        {
            return Err(ownership_fault(*entity));
        }
    }
    if let Some(entity) = expected.first() {
        return Err(ownership_fault(*entity));
    }
    Ok(rows)
}

struct MotorPlan {
    prepared_velocity: Option<[f64; 2]>,
    entity: Entity,
    joints: [JointId; 2],
    motors: [JointVelocityMotor; 2],
}

fn apply(world: &mut World) {
    if world.resource::<UpdateInfo>().dt.is_zero()
        || world.resource::<StepFault>().error().is_some()
    {
        return;
    }
    let result = (|| -> Result<(), DriveFault> {
        let mut planned = Vec::new();
        for entity in active_drives(world)? {
            let drive = world
                .get::<Drive>(entity)
                .expect("preflighted original drive");
            let dt = world.resource::<UpdateInfo>().dt.as_secs_f64();
            // Gazebo prepares the speed in PostUpdate for the following physics tick.
            let prepared_velocity = drive.prepared_velocity.map(|velocity| {
                std::array::from_fn(|i| {
                    let previous = velocity[i];
                    let [min, max] = drive.acceleration[i];
                    drive.desired[i].clamp(previous + min * dt, previous + max * dt)
                })
            });
            let applied = drive.prepared_velocity.unwrap_or(drive.desired);
            let targets = drive
                .geometry
                .bounded_targets(applied[0], applied[1], drive.speed)
                .map_err(|error| DriveFault {
                    entity,
                    message: error.to_string(),
                })?;
            let motors = [
                JointVelocityMotor {
                    target_velocity: targets[0],
                    max_force: drive.force[0],
                },
                JointVelocityMotor {
                    target_velocity: targets[1],
                    max_force: drive.force[1],
                },
            ];
            for (joint, motor) in drive.joints.into_iter().zip(motors) {
                validate_motor(world.resource::<PhysicsBackend>(), joint, Some(motor)).map_err(
                    |error| DriveFault {
                        entity,
                        message: error.to_string(),
                    },
                )?;
            }
            planned.push(MotorPlan {
                prepared_velocity,
                entity,
                joints: drive.joints,
                motors,
            });
        }
        for plan in planned {
            let mut backend = world.resource_mut::<PhysicsBackend>();
            for (joint, motor) in plan.joints.into_iter().zip(plan.motors) {
                set_motor(&mut backend, joint, Some(motor)).map_err(|error| DriveFault {
                    entity: plan.entity,
                    message: error.to_string(),
                })?;
            }
            world
                .get_mut::<Drive>(plan.entity)
                .expect("preflighted drive")
                .prepared_velocity = plan.prepared_velocity;
        }
        Ok(())
    })();
    if let Err(error) = result {
        world.resource::<StepFault>().report(error);
    }
}

fn observe(world: &mut World) {
    let time = *world.resource::<UpdateInfo>();
    if time.dt.is_zero() || world.resource::<StepFault>().error().is_some() {
        return;
    }
    let result = (|| -> Result<(), DriveFault> {
        let mut prepared = Vec::new();
        for row in world
            .iter_entities()
            .filter(|row| row.contains::<Drive>())
            .take(MAX_LOADED_DRIVES + 1)
        {
            let entity = row.id();
            let mut drive = row.get::<Drive>().expect("selected drive").clone();
            let prepare = (|| -> Result<Option<model::Reading>, String> {
                let mut angles = [0.0; 2];
                for (angle, joint) in angles.iter_mut().zip(drive.joints) {
                    *angle = world
                        .resource::<PhysicsBackend>()
                        .joint_state(joint)
                        .ok_or("missing drive wheel state")?
                        .position;
                }
                let Some(odometry) = &mut drive.odometry else {
                    return Err("missing initial encoder reference".into());
                };
                let reading = odometry
                    .observe(angles, time.sim_time)
                    .map_err(|error| error.to_string())?;
                if !drive
                    .schedule
                    .take_sample(time.sim_time, false)
                    .map_err(|error| error.to_string())?
                {
                    return Ok(None);
                }
                Ok(Some(reading))
            })();
            let reading = prepare.map_err(|message| DriveFault { entity, message })?;
            prepared.push((entity, drive, reading));
        }
        active_drives(world)?;
        for (entity, drive, reading) in prepared {
            *world
                .get_mut::<Drive>(entity)
                .expect("preflighted original drive") = drive;
            if let Some(reading) = reading {
                world.entity_mut(entity).insert(DiffDriveSample {
                    time: time.sim_time,
                    epoch: time.epoch,
                    pose: reading.pose,
                    twist: reading.twist,
                });
            }
        }
        Ok(())
    })();
    if let Err(error) = result {
        world.resource::<StepFault>().report(error);
    }
}

fn reset(world: &mut World) {
    let rows: Vec<_> = world
        .iter_entities()
        .filter(|row| row.contains::<Drive>())
        .take(MAX_LOADED_DRIVES + 1)
        .map(|row| row.id())
        .collect();
    // Preserve the M8 failure cleanup even if later validation refuses reset.
    for entity in &rows {
        {
            let mut drive = world.get_mut::<Drive>(*entity).expect("selected drive");
            drive.desired = [0.0; 2];
            drive.prepared_velocity = drive.prepared_velocity.map(|_| [0.0; 2]);
            drive.schedule.reset();
        }
        world.entity_mut(*entity).remove::<DiffDriveSample>();
    }
    let result = (|| -> Result<(), DriveFault> {
        let mut prepared = Vec::new();
        for entity in rows {
            let mut drive = world.get::<Drive>(entity).expect("selected drive").clone();
            let prepare = (|| -> Result<(), String> {
                drive
                    .schedule
                    .take_sample(Duration::ZERO, false)
                    .map_err(|error| error.to_string())?;
                let mut angles = [0.0; 2];
                for (angle, joint) in angles.iter_mut().zip(drive.joints) {
                    *angle = world
                        .resource::<PhysicsBackend>()
                        .joint_state(joint)
                        .ok_or("missing reset wheel state")?
                        .position;
                }
                drive
                    .odometry
                    .as_mut()
                    .ok_or("missing initial encoder reference")?
                    .reset(angles, Duration::ZERO)
                    .map_err(|error| error.to_string())?;
                for joint in drive.joints {
                    validate_motor(world.resource::<PhysicsBackend>(), joint, None)
                        .map_err(|error| error.to_string())?;
                }
                Ok(())
            })();
            prepare.map_err(|message| DriveFault { entity, message })?;
            prepared.push((entity, drive));
        }
        active_drives(world)?;
        {
            let mut backend = world.resource_mut::<PhysicsBackend>();
            for (entity, drive) in &prepared {
                for joint in drive.joints {
                    set_motor(&mut backend, joint, None).map_err(|error| DriveFault {
                        entity: *entity,
                        message: error.to_string(),
                    })?;
                }
            }
        }
        for (entity, drive) in prepared {
            *world.get_mut::<Drive>(entity).expect("preflighted drive") = drive;
        }
        Ok(())
    })();
    if let Err(error) = result {
        world.resource::<StepFault>().report(error);
    }
}

#[cfg(test)]
#[path = "../tests/private/diff_drive.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/multiple_diff_drives.rs"]
mod multiple_diff_drives_tests;
