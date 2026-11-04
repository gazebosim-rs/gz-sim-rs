// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One configured differential wheel pair; native motors own the physical response.
use super::*;
use gz_physics::{JointId, JointVelocityMotor};
mod config;
mod model;

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
    desired: [f64; 2],
    geometry: model::Geometry,
    odometry: Option<model::Odometry>,
    schedule: SampleSchedule,
}

pub(super) struct Plan(config::Parameters);
pub(super) fn is_plugin(plugin: &sdformat::Plugin) -> bool {
    config::is_plugin(plugin)
}
pub(super) fn parse(model: &Model) -> Result<Option<Plan>, LoadError> {
    config::parse(model).map(|value| value.map(Plan))
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
            engine == Some("rapier") && placement.0 && !placement.1,
            &model.scoped_name,
            "DiffDrive requires a top-level dynamic Rapier model",
        )?;
        require(
            !sim.world()
                .iter_entities()
                .any(|entity| entity.contains::<Drive>()),
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
        backend.validate_joint_velocity_motor(
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
        backend.validate_joint_velocity_motor(
            joint,
            Some(JointVelocityMotor {
                target_velocity: target,
                max_force: force,
            }),
        )?;
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

fn apply(world: &mut World) {
    if world.resource::<UpdateInfo>().dt.is_zero()
        || world.resource::<StepFault>().error().is_some()
    {
        return;
    }
    let result = (|| -> Result<(), DriveFault> {
        let (id, joints, motors) = {
            let mut drivers = world
                .iter_entities()
                .filter(|entity| entity.contains::<Drive>());
            let Some(entity) = drivers.next() else {
                return Ok(());
            };
            let id = entity.id();
            let invalid = || DriveFault {
                entity: id,
                message: "invalid loaded drive ownership".into(),
            };
            if drivers.next().is_some()
                || !initial::matches_physics_owner(world)
                || !joints::bindings_valid(world)
            {
                return Err(invalid());
            }
            let drive = entity.get::<Drive>().expect("selected configured drive");
            if entity.get::<EntityId>().map(EntityId::get) != Some(drive.owner)
                || gz_sim_core::entity_by_id(world, drive.owner) != Some(id)
                || entity.get::<EntityKind>() != Some(&EntityKind::Model)
                || entity.get::<DiffDriveConfiguration>() != Some(&drive.publication)
            {
                return Err(invalid());
            }
            let targets = drive
                .geometry
                .bounded_targets(drive.desired[0], drive.desired[1], drive.speed)
                .map_err(|error| DriveFault {
                    entity: id,
                    message: error.to_string(),
                })?;
            let joints = drive.joints;
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
            (id, joints, motors)
        };
        let mut backend = world.resource_mut::<PhysicsBackend>();
        for (joint, motor) in joints.into_iter().zip(motors) {
            backend
                .validate_joint_velocity_motor(joint, Some(motor))
                .map_err(|error| DriveFault {
                    entity: id,
                    message: error.to_string(),
                })?;
        }
        for (joint, motor) in joints.into_iter().zip(motors) {
            backend
                .set_joint_velocity_motor(joint, Some(motor))
                .map_err(|error| DriveFault {
                    entity: id,
                    message: error.to_string(),
                })?;
        }
        Ok(())
    })();
    if let Err(error) = result {
        world.resource::<StepFault>().report(error);
    }
}

fn observe(
    time: Res<UpdateInfo>,
    backend: Res<PhysicsBackend>,
    fault: Res<StepFault>,
    mut drivers: Query<(Entity, &mut Drive)>,
    mut commands: Commands,
) {
    if time.dt.is_zero() {
        return;
    }
    for (entity, mut drive) in &mut drivers {
        let result = (|| -> Result<Option<model::Reading>, String> {
            let mut angles = [0.0; 2];
            for (angle, joint) in angles.iter_mut().zip(drive.joints) {
                *angle = backend
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
        match result {
            Ok(Some(reading)) => {
                commands.entity(entity).insert(DiffDriveSample {
                    time: time.sim_time,
                    epoch: time.epoch,
                    pose: reading.pose,
                    twist: reading.twist,
                });
            }
            Ok(None) => {}
            Err(message) => {
                fault.report(DriveFault { entity, message });
                return;
            }
        }
    }
}

fn reset(
    mut drivers: Query<(Entity, &mut Drive)>,
    mut backend: ResMut<PhysicsBackend>,
    fault: Res<StepFault>,
    mut commands: Commands,
) {
    for (entity, mut drive) in &mut drivers {
        drive.desired = [0.0; 2];
        drive.schedule.reset();
        let result = (|| -> Result<(), String> {
            drive
                .schedule
                .take_sample(Duration::ZERO, false)
                .map_err(|error| error.to_string())?;
            let mut angles = [0.0; 2];
            for (angle, joint) in angles.iter_mut().zip(drive.joints) {
                *angle = backend
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
            for id in drive.joints {
                backend
                    .validate_joint_velocity_motor(id, None)
                    .map_err(|error| error.to_string())?;
            }
            for id in drive.joints {
                backend
                    .set_joint_velocity_motor(id, None)
                    .map_err(|error| error.to_string())?;
            }
            Ok(())
        })();
        commands.entity(entity).remove::<DiffDriveSample>();
        if let Err(message) = result {
            fault.report(DriveFault { entity, message });
            return;
        }
    }
}

#[cfg(test)]
#[path = "../tests/private/diff_drive.rs"]
mod tests;
