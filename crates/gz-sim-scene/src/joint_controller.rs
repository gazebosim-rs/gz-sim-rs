// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Demand-activated position/velocity targets on the maintained Rapier motors.
use super::*;
mod config;
mod model;

/// Position uses a bounded, damped native reduced-coordinate servo.
/// SDF command identities are reused; native ABS/PID command-order parity is not claimed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointControllerMode {
    Position,
    Velocity,
}

/// Original loaded command identity; changing it does not reconfigure a motor.
#[derive(Component, Clone, Debug, PartialEq, Eq)]
#[component(immutable)]
pub struct JointControllerConfiguration {
    pub model_name: String,
    pub joint_name: String,
    pub topic: String,
    pub mode: JointControllerMode,
}

#[derive(Debug, thiserror::Error)]
pub enum JointControllerCommandError {
    #[error("joint {0} does not own an admitted loaded controller")]
    MissingController(u64),
    #[error("controller ownership or physical state is invalid")]
    InvalidPhysics,
    #[error("joint command is not finite and representable")]
    InvalidCommand,
    #[error(transparent)]
    Physics(#[from] PhysicsError),
}

#[derive(Component, Clone)]
pub(super) struct Controller {
    owner: u64,
    publication: JointControllerConfiguration,
    joint: gz_physics::JointId,
    max_velocity: f64,
    max_force: f64,
    initial_target: f64,
    target: f64,
}

#[derive(Clone, Copy)]
enum MotorRequest {
    Position(gz_physics::JointPositionServo),
    Velocity(gz_physics::JointVelocityMotor),
}
impl MotorRequest {
    fn validate(
        self,
        backend: &PhysicsBackend,
        joint: gz_physics::JointId,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        match self {
            Self::Position(servo) => backend.validate_joint_position_servo(joint, Some(servo), dt),
            Self::Velocity(motor) => backend.validate_joint_velocity_servo(joint, Some(motor)),
        }
    }
    fn set(
        self,
        backend: &mut PhysicsBackend,
        joint: gz_physics::JointId,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        match self {
            Self::Position(servo) => backend.set_joint_position_servo(joint, Some(servo), dt),
            Self::Velocity(motor) => backend.set_joint_velocity_servo(joint, Some(motor)),
        }
    }
}
fn request(
    mode: JointControllerMode,
    target: f64,
    coordinate: f64,
    max_velocity: f64,
    max_force: f64,
    dt: Duration,
) -> Result<MotorRequest, model::InvalidControllerInput> {
    // Retain the existing finite continuous-error and per-step admission guards.
    // Position actuation uses the native damped servo, not this ABS velocity bound.
    let velocity = model::target_velocity(mode, target, coordinate, max_velocity, dt)?;
    Ok(match mode {
        JointControllerMode::Position => MotorRequest::Position(gz_physics::JointPositionServo {
            target_position: target,
            max_velocity,
            max_force,
        }),
        JointControllerMode::Velocity => MotorRequest::Velocity(gz_physics::JointVelocityMotor {
            target_velocity: velocity,
            max_force,
        }),
    })
}

pub(super) struct Plan(Vec<config::Parameters>);
pub(super) fn is_plugin(plugin: &sdformat::Plugin) -> bool {
    config::is_plugin(plugin)
}
pub(super) fn parse(model: &Model) -> Result<Option<Plan>, LoadError> {
    config::parse(model).map(|parameters| parameters.map(Plan))
}
impl Plan {
    pub(super) fn names(&self) -> impl Iterator<Item = &str> {
        self.0.iter().map(|parameters| parameters.joint.as_str())
    }
    pub(super) fn validate(
        &self,
        model: &Model,
        sim: &Simulation,
        engine: Option<&str>,
        placement: (bool, bool),
    ) -> Result<(), LoadError> {
        require(
            engine == Some("rapier") && placement.0 && !placement.1,
            &model.scoped_name,
            "joint controllers require a top-level dynamic Rapier model",
        )?;
        require(
            !sim.world()
                .iter_entities()
                .any(|entity| entity.contains::<Controller>()),
            &model.scoped_name,
            "multiple joint-controller chains",
        )
    }
}

pub(super) fn build(
    sim: &mut Simulation,
    model: &Model,
    model_entity: Entity,
    backend: &PhysicsBackend,
    plan: &Plan,
) -> Result<(), LoadError> {
    let mut prepared = Vec::with_capacity(plan.0.len());
    for parameters in &plan.0 {
        let name = format!("{}::{}", model.scoped_name, parameters.joint);
        let node = sim
            .world()
            .iter_entities()
            .find(|entity| entity.get::<Name>().is_some_and(|n| n.0 == name))
            .ok_or(LoadError::Unsupported {
                entity: name.clone(),
                feature: "missing controller joint",
            })?;
        require(
            node.get::<ChildOf>().map(ChildOf::parent) == Some(model_entity)
                && node.get::<EntityKind>() == Some(&EntityKind::Joint),
            &name,
            "controller joint ownership",
        )?;
        let handle = node.get::<JointHandle>().ok_or(LoadError::Unsupported {
            entity: name.clone(),
            feature: "missing controller joint binding",
        })?;
        let state = backend
            .joint_state(handle.id())
            .ok_or(LoadError::Unsupported {
                entity: name.clone(),
                feature: "missing controller joint state",
            })?;
        let motor = request(
            parameters.publication.mode,
            parameters.initial_target,
            state.position,
            parameters.max_velocity,
            parameters.max_force,
            sim.step_duration(),
        )
        .map_err(|_| LoadError::Unsupported {
            entity: name.clone(),
            feature: "unrepresentable controller initialization",
        })?;
        motor.validate(backend, handle.id(), sim.step_duration())?;
        prepared.push((
            node.id(),
            Controller {
                owner: node.get::<EntityId>().expect("loaded joint identity").get(),
                publication: parameters.publication.clone(),
                joint: handle.id(),
                max_velocity: parameters.max_velocity,
                max_force: parameters.max_force,
                initial_target: parameters.initial_target,
                target: parameters.initial_target,
            },
        ));
    }
    for (entity, controller) in prepared {
        sim.world_mut()
            .entity_mut(entity)
            .insert((controller.publication.clone(), controller));
    }
    Ok(())
}

fn command_error(error: model::InvalidControllerInput) -> JointControllerCommandError {
    match error {
        model::InvalidControllerInput::Command => JointControllerCommandError::InvalidCommand,
        model::InvalidControllerInput::Time
        | model::InvalidControllerInput::State
        | model::InvalidControllerInput::Bounds => JointControllerCommandError::InvalidPhysics,
    }
}

/// Replace a pending SI target without stepping. Velocity commands saturate at
/// the configured target speed; positions remain continuous, without wrapping.
pub fn set_joint_controller_command(
    sim: &mut Simulation,
    id: u64,
    target: f64,
) -> Result<(), JointControllerCommandError> {
    if !target.is_finite() {
        return Err(JointControllerCommandError::InvalidCommand);
    }
    let world = sim.world();
    if !initial::matches_physics_owner(world) || !joints::bindings_valid(world) {
        return Err(JointControllerCommandError::InvalidPhysics);
    }
    let entity = gz_sim_core::entity_by_id(world, id)
        .ok_or(JointControllerCommandError::MissingController(id))?;
    let controller = world
        .get::<Controller>(entity)
        .ok_or(JointControllerCommandError::MissingController(id))?;
    if controller.owner != id
        || world.get::<EntityKind>(entity) != Some(&EntityKind::Joint)
        || world.get::<JointControllerConfiguration>(entity) != Some(&controller.publication)
        || world
            .get::<JointHandle>(entity)
            .is_none_or(|handle| handle.id() != controller.joint)
    {
        return Err(JointControllerCommandError::InvalidPhysics);
    }
    let backend = world.resource::<PhysicsBackend>();
    let coordinate = backend
        .joint_state(controller.joint)
        .ok_or(JointControllerCommandError::InvalidPhysics)?
        .position;
    let motor = request(
        controller.publication.mode,
        target,
        coordinate,
        controller.max_velocity,
        controller.max_force,
        sim.step_duration(),
    )
    .map_err(command_error)?;
    motor.validate(backend, controller.joint, sim.step_duration())?;
    let desired = match motor {
        MotorRequest::Velocity(motor) => motor.target_velocity,
        MotorRequest::Position(_) => target,
    };
    sim.world_mut()
        .get_mut::<Controller>(entity)
        .expect("preflighted controller")
        .target = desired;
    Ok(())
}

#[derive(Resource)]
pub(super) struct Installed;
pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>()
        || !sim
            .world()
            .iter_entities()
            .any(|entity| entity.contains::<Controller>())
    {
        return;
    }
    sim.add_systems(Phase::PreStep, apply);
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
            .any(|entity| entity.contains::<Controller>())
    {
        return Ok(());
    }
    sim.initialize_schedules()?;
    sim.remove_systems_in_set(Phase::PreStep, apply)?;
    sim.remove_systems_in_set(Phase::Reset, reset)?;
    sim.world_mut().remove_resource::<Installed>();
    Ok(())
}

#[derive(Debug, thiserror::Error)]
#[error("joint controller {entity:?}: {message}")]
struct ControllerFault {
    entity: Entity,
    message: String,
}
fn apply(world: &mut World) {
    let time = *world.resource::<UpdateInfo>();
    if time.dt.is_zero() || world.resource::<StepFault>().error().is_some() {
        return;
    }
    let result = (|| -> Result<(), ControllerFault> {
        let prepared = {
            let mut entries = [None; 2];
            for (index, entity) in world
                .iter_entities()
                .filter(|entity| entity.contains::<Controller>())
                .enumerate()
            {
                let fail = |message: String| ControllerFault {
                    entity: entity.id(),
                    message,
                };
                let controller = entity.get::<Controller>().expect("selected controller");
                if index >= entries.len()
                    || !initial::matches_physics_owner(world)
                    || !joints::bindings_valid(world)
                    || entity.get::<EntityId>().map(EntityId::get) != Some(controller.owner)
                    || gz_sim_core::entity_by_id(world, controller.owner) != Some(entity.id())
                    || entity.get::<EntityKind>() != Some(&EntityKind::Joint)
                    || entity.get::<JointControllerConfiguration>() != Some(&controller.publication)
                    || entity
                        .get::<JointHandle>()
                        .is_none_or(|handle| handle.id() != controller.joint)
                {
                    return Err(fail("invalid controller ownership".into()));
                }
                let state = world
                    .resource::<PhysicsBackend>()
                    .joint_state(controller.joint)
                    .ok_or_else(|| fail("missing controller joint state".into()))?;
                let motor = request(
                    controller.publication.mode,
                    controller.target,
                    state.position,
                    controller.max_velocity,
                    controller.max_force,
                    time.dt,
                )
                .map_err(|error| fail(error.to_string()))?;
                entries[index] = Some((entity.id(), controller.joint, motor));
            }
            entries
        };
        if prepared.iter().all(Option::is_none) {
            return Ok(());
        }
        let mut backend = world.resource_mut::<PhysicsBackend>();
        for (entity, joint, motor) in prepared.into_iter().flatten() {
            motor
                .validate(&backend, joint, time.dt)
                .map_err(|error| ControllerFault {
                    entity,
                    message: error.to_string(),
                })?;
        }
        for (entity, joint, motor) in prepared.into_iter().flatten() {
            motor
                .set(&mut backend, joint, time.dt)
                .map_err(|error| ControllerFault {
                    entity,
                    message: error.to_string(),
                })?;
        }
        Ok(())
    })();
    if let Err(error) = result {
        world.resource::<StepFault>().report(error);
    }
}
fn reset(
    mut controllers: Query<(Entity, &mut Controller)>,
    mut backend: ResMut<PhysicsBackend>,
    fault: Res<StepFault>,
) {
    let result = (|| -> Result<(), ControllerFault> {
        let mut entries = [None; 2];
        for (index, (entity, controller)) in controllers.iter().enumerate() {
            if index >= entries.len() {
                return Err(ControllerFault {
                    entity,
                    message: "too many loaded controllers".into(),
                });
            }
            backend
                .validate_joint_position_servo(controller.joint, None, Duration::ZERO)
                .map_err(|error| ControllerFault {
                    entity,
                    message: error.to_string(),
                })?;
            entries[index] = Some((entity, controller.joint));
        }
        for (entity, joint) in entries.into_iter().flatten() {
            backend
                .set_joint_position_servo(joint, None, Duration::ZERO)
                .map_err(|error| ControllerFault {
                    entity,
                    message: error.to_string(),
                })?;
        }
        for (_, mut controller) in &mut controllers {
            controller.target = controller.initial_target;
        }
        Ok(())
    })();
    if let Err(error) = result {
        fault.report(error);
    }
}

#[cfg(test)]
#[path = "../tests/private/joint_controllers.rs"]
mod tests;
