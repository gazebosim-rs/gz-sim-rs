// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Demand-activated position/velocity targets on the maintained Rapier motors.
use super::*;
use std::collections::{BTreeMap, BTreeSet};
mod config;
mod model;
#[cfg(feature = "gpu-batch")]
pub(crate) use config::Parameters;
#[cfg(feature = "gpu-batch")]
pub(crate) fn planar_parameters(model: &Model) -> Result<Vec<Parameters>, LoadError> {
    Ok(config::parse_fields(model, true)?.unwrap_or_default())
}
#[cfg(feature = "gpu-batch")]
pub(crate) use model::target_velocity;

pub(crate) const MAX_CONTROLLER_OWNERS: usize = 8;
const MAX_CONTROLLER_INPUTS: usize = MAX_CONTROLLER_OWNERS * 2;

/// Position targets use a damped servo for reduced revolutes. Local screw and
/// wrist plugins use the SDF absolute-position velocity command. Bounds are in SI units.
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
    actuation: Actuation,
    owner: u64,
    publication: JointControllerConfiguration,
    joint: gz_physics::JointId,
    max_velocity: f64,
    max_force: f64,
    initial_target: f64,
    target: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Actuation {
    Reduced,
    Screw,
    Universal(usize),
}

impl Actuation {
    fn coordinate(self, backend: &PhysicsBackend, joint: gz_physics::JointId) -> Option<f64> {
        if let Self::Universal(axis) = self {
            return Some(
                backend
                    .downcast_ref::<MultiFidelityBackend>()?
                    .universal_joint_state(joint)?
                    .position[axis],
            );
        }
        backend.joint_state(joint).map(|state| state.position)
    }

    fn clear(self) -> MotorRequest {
        match self {
            Self::Universal(axis) => MotorRequest::Universal(axis, None),
            _ => MotorRequest::Clear,
        }
    }
}

#[derive(Clone, Copy)]
enum MotorRequest {
    Universal(usize, Option<gz_physics::JointVelocityMotor>),
    Clear,
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
            Self::Universal(axis, motor) => backend
                .downcast_ref::<MultiFidelityBackend>()
                .ok_or(PhysicsError::InvalidJoint)?
                .validate_universal_motor(joint, axis, motor),
            Self::Clear => backend.validate_joint_position_servo(joint, None, dt),
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
            Self::Universal(axis, motor) => backend
                .downcast_mut::<MultiFidelityBackend>()
                .ok_or(PhysicsError::InvalidJoint)?
                .set_universal_motor(joint, axis, motor),
            Self::Clear => backend.set_joint_position_servo(joint, None, dt),
            Self::Position(servo) => backend.set_joint_position_servo(joint, Some(servo), dt),
            Self::Velocity(motor) => backend.set_joint_velocity_servo(joint, Some(motor)),
        }
    }
}
impl Actuation {
    fn request(
        self,
        mode: JointControllerMode,
        target: f64,
        coordinate: f64,
        max_velocity: f64,
        max_force: f64,
        dt: Duration,
    ) -> Result<MotorRequest, model::InvalidControllerInput> {
        let velocity = model::target_velocity(mode, target, coordinate, max_velocity, dt)?;
        let motor = gz_physics::JointVelocityMotor {
            target_velocity: velocity,
            max_force,
        };
        Ok(match (self, mode) {
            (Self::Universal(axis), _) => MotorRequest::Universal(axis, Some(motor)),
            (Self::Reduced, JointControllerMode::Position) => {
                MotorRequest::Position(gz_physics::JointPositionServo {
                    target_position: target,
                    max_velocity,
                    max_force,
                })
            }
            _ => MotorRequest::Velocity(motor),
        })
    }
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
            engine
                == Some(if self.0[0].actuation == Actuation::Reduced {
                    "rapier"
                } else {
                    "multifidelity"
                })
                && placement.0
                && !placement.1,
            &model.scoped_name,
            "joint controllers require a top-level dynamic model on their supported engine",
        )?;
        let mut owners = BTreeMap::new();
        let mut slots = 0;
        for entity in sim
            .world()
            .iter_entities()
            .filter(|entity| entity.contains::<Controller>())
            .take(MAX_CONTROLLER_INPUTS + 1)
        {
            let parent = entity
                .get::<ChildOf>()
                .ok_or_else(|| LoadError::Unsupported {
                    entity: model.scoped_name.clone(),
                    feature: "joint controller ownership",
                })?;
            *owners.entry(parent.parent()).or_insert(0usize) += 1;
            slots += 1;
        }
        require(
            owners.len() < MAX_CONTROLLER_OWNERS
                && owners.values().all(|count| *count <= 2)
                && slots + self.0.len() <= MAX_CONTROLLER_INPUTS,
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
        let coordinate = parameters
            .actuation
            .coordinate(backend, handle.id())
            .ok_or(LoadError::Unsupported {
                entity: name.clone(),
                feature: "missing controller joint state",
            })?;
        let motor = parameters
            .actuation
            .request(
                parameters.publication.mode,
                parameters.initial_target,
                coordinate,
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
                actuation: parameters.actuation,
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

/// Current held SI position or velocity target on a locally owned CPU controller.
pub fn held_joint_controller_target(sim: &Simulation, id: u64) -> Option<f64> {
    Some(
        sim.world()
            .get::<Controller>(gz_sim_core::entity_by_id(sim.world(), id)?)?
            .target,
    )
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
    #[cfg(feature = "gpu-batch")]
    if world.contains_resource::<gpu_scene::planar_tool::ToolBindings>() {
        return gpu_scene::planar_tool::set_joint_command(sim, id, entity, target);
    }
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
    validate_loaded_controllers(world).map_err(|_| JointControllerCommandError::InvalidPhysics)?;
    let backend = world.resource::<PhysicsBackend>();
    let coordinate = controller
        .actuation
        .coordinate(backend, controller.joint)
        .ok_or(JointControllerCommandError::InvalidPhysics)?;
    let motor = controller
        .actuation
        .request(
            controller.publication.mode,
            target,
            coordinate,
            controller.max_velocity,
            controller.max_force,
            position_response(world, id, sim.step_duration()),
        )
        .map_err(command_error)?;
    motor.validate(backend, controller.joint, sim.step_duration())?;
    let desired = if controller.publication.mode == JointControllerMode::Velocity {
        target.clamp(-controller.max_velocity, controller.max_velocity)
    } else {
        target
    };
    sim.world_mut()
        .get_mut::<Controller>(entity)
        .expect("preflighted controller")
        .target = desired;
    Ok(())
}

#[derive(Resource, Default)]
struct PositionResponses(BTreeMap<u64, Duration>);

fn position_response(world: &World, id: u64, native: Duration) -> Duration {
    world
        .get_resource::<PositionResponses>()
        .and_then(|policy| policy.0.get(&id))
        .copied()
        .unwrap_or(native)
}

/// Set the physical period used to turn position error into target velocity on
/// local screws and universal axes. Native updates and actuator limits retain
/// their authored timing. The period must span at least one physics step.
/// Configuration survives loaded-world reset. Passing `None` restores the
/// native one-step response. All named joints are validated before mutation.
pub fn set_joint_position_response_period(
    sim: &mut Simulation,
    ids: &[u64],
    period: Option<Duration>,
) -> Result<(), JointControllerCommandError> {
    if period.is_some_and(|p| p < sim.step_duration()) {
        return Err(JointControllerCommandError::InvalidCommand);
    }
    for &id in ids {
        if !joint_controller_binding_matches(sim, id) {
            return Err(JointControllerCommandError::MissingController(id));
        }
        let controller = gz_sim_core::entity_by_id(sim.world(), id)
            .and_then(|e| sim.world().get::<Controller>(e))
            .ok_or(JointControllerCommandError::MissingController(id))?;
        if controller.publication.mode != JointControllerMode::Position
            || !matches!(
                controller.actuation,
                Actuation::Screw | Actuation::Universal(_)
            )
        {
            return Err(JointControllerCommandError::InvalidCommand);
        }
        // The native position law requires its maximum travel per response to
        // remain representable, even for a very long requested period.
        if period.is_some_and(|dt| !(controller.max_velocity * dt.as_secs_f64()).is_finite()) {
            return Err(JointControllerCommandError::InvalidCommand);
        }
    }
    if let Some(value) = period {
        let mut policy = sim.world_mut().get_resource_or_init::<PositionResponses>();
        policy.0.extend(ids.iter().map(|id| (*id, value)));
    } else if let Some(mut policy) = sim.world_mut().get_resource_mut::<PositionResponses>() {
        for id in ids {
            policy.0.remove(id);
        }
    }
    Ok(())
}

/// Read-only witness for a loaded controller's private owner and native binding.
/// Callers also validate public identity, parent/child and the complete input set.
/// Missing private storage, foreign physics ownership or binding drift returns false.
pub fn joint_controller_binding_matches(sim: &Simulation, id: u64) -> bool {
    let Some(entity) = gz_sim_core::entity_by_id(sim.world(), id) else {
        return false;
    };
    let Some(configuration) = sim.world().get::<JointControllerConfiguration>(entity) else {
        return false;
    };
    joint_controller_binding_matches_configuration(sim, id, configuration)
}

/// Check the private/native binding against a trusted saved configuration.
/// This deliberately does not compare the current public configuration: reset
/// reconstruction classifies a well-formed metadata drift separately. Callers
/// must still require the full public witness before installing any input.
pub fn joint_controller_binding_matches_configuration(
    sim: &Simulation,
    id: u64,
    configuration: &JointControllerConfiguration,
) -> bool {
    let world = sim.world();
    if !initial::matches_physics_owner(world) {
        return false;
    }
    let Some(entity) = gz_sim_core::entity_by_id(world, id) else {
        return false;
    };
    #[cfg(feature = "gpu-batch")]
    if world.contains_resource::<gpu_scene::planar_tool::ToolBindings>() {
        return gpu_scene::planar_tool::binding_matches(sim, id, configuration);
    }
    let Some(controller) = world.get::<Controller>(entity) else {
        return false;
    };
    controller.owner == id
        && world
            .get::<JointHandle>(entity)
            .is_some_and(|handle| handle.id() == controller.joint)
        && &controller.publication == configuration
        && world
            .get_resource::<PhysicsBackend>()
            .and_then(|backend| controller.actuation.coordinate(backend, controller.joint))
            .is_some()
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
// The immutable blueprint reserves removed originals; live rows alone cannot
// authorize a new owner or a replacement joint with otherwise plausible fields.
fn admitted_slots(world: &World) -> Option<usize> {
    let initial = world.get_resource::<initial::InitialScene>()?;
    let mut ids = BTreeSet::new();
    let mut names = BTreeSet::new();
    let mut owners = BTreeMap::new();
    for original in initial.joint_controller_identities() {
        let parent = original.parent?;
        if original.id == 0
            || parent == 0
            || original.kind != EntityKind::Joint
            || !ids.insert(original.id)
            || !names.insert(original.name)
        {
            return None;
        }
        *owners.entry(parent).or_insert(0usize) += 1;
        if ids.len() > MAX_CONTROLLER_INPUTS
            || owners.len() > MAX_CONTROLLER_OWNERS
            || owners.values().any(|count| *count > 2)
        {
            return None;
        }
    }
    Some(ids.len())
}

fn validate_loaded_controllers(world: &World) -> Result<(), ControllerFault> {
    let controllers: Vec<_> = world
        .iter_entities()
        .filter(|entity| entity.contains::<Controller>())
        .take(MAX_CONTROLLER_INPUTS + 1)
        .collect();
    let Some(first) = controllers.first() else {
        return Ok(());
    };
    let invalid = |entity| ControllerFault {
        entity,
        message: "invalid controller ownership".into(),
    };
    let slots = admitted_slots(world).ok_or_else(|| invalid(first.id()))?;
    if controllers.len() > slots
        || !initial::matches_physics_owner(world)
        || !joints::bindings_valid(world)
    {
        return Err(invalid(first.id()));
    }
    let initial = world.resource::<initial::InitialScene>();
    let originals: Vec<_> = initial.joint_controller_identities().collect();
    let mut ids = BTreeSet::new();
    let mut handles = BTreeSet::new();
    let mut children = BTreeSet::new();
    for entity in controllers {
        let controller = entity.get::<Controller>().expect("selected controller");
        let original = originals
            .iter()
            .find(|row| row.id == controller.owner)
            .ok_or_else(|| invalid(entity.id()))?;
        let parent_id = original.parent.ok_or_else(|| invalid(entity.id()))?;
        let parent =
            gz_sim_core::entity_by_id(world, parent_id).ok_or_else(|| invalid(entity.id()))?;
        let parent_name = world
            .get::<Name>(parent)
            .ok_or_else(|| invalid(entity.id()))?;
        let root = world
            .get::<ChildOf>(parent)
            .map(ChildOf::parent)
            .ok_or_else(|| invalid(entity.id()))?;
        let root_name = world
            .get::<Name>(root)
            .ok_or_else(|| invalid(entity.id()))?;
        let handle = entity
            .get::<JointHandle>()
            .ok_or_else(|| invalid(entity.id()))?;
        if entity.get::<EntityId>().map(EntityId::get) != Some(original.id)
            || gz_sim_core::entity_by_id(world, original.id) != Some(entity.id())
            || entity.get::<EntityKind>() != Some(&original.kind)
            || entity.get::<Name>().map(|name| name.0.as_str()) != Some(original.name)
            || entity.get::<ChildOf>().map(ChildOf::parent) != Some(parent)
            || world.get::<EntityKind>(parent) != Some(&EntityKind::Model)
            || world.get::<EntityKind>(root) != Some(&EntityKind::World)
            || parent_name.0.rsplit_once("::")
                != Some((
                    root_name.0.as_str(),
                    original.configuration.model_name.as_str(),
                ))
            || original.name.rsplit_once("::")
                != Some((
                    parent_name.0.as_str(),
                    original.configuration.joint_name.as_str(),
                ))
            || entity.get::<JointControllerConfiguration>() != Some(original.configuration)
            || controller.publication != *original.configuration
            || handle.id() != controller.joint
            || !ids.insert(original.id)
            || !handles.insert(handle.id())
            || !children.insert(handle.child())
        {
            return Err(invalid(entity.id()));
        }
    }
    Ok(())
}

fn apply(world: &mut World) {
    let time = *world.resource::<UpdateInfo>();
    if time.dt.is_zero() || world.resource::<StepFault>().error().is_some() {
        return;
    }
    let result = (|| -> Result<(), ControllerFault> {
        validate_loaded_controllers(world)?;
        let prepared = {
            let mut entries = [None; MAX_CONTROLLER_INPUTS];
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
                let coordinate = controller
                    .actuation
                    .coordinate(world.resource::<PhysicsBackend>(), controller.joint)
                    .ok_or_else(|| fail("missing controller joint state".into()))?;
                let motor = controller
                    .actuation
                    .request(
                        controller.publication.mode,
                        controller.target,
                        coordinate,
                        controller.max_velocity,
                        controller.max_force,
                        position_response(world, controller.owner, time.dt),
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
fn reset(world: &mut World) {
    let result = (|| -> Result<(), ControllerFault> {
        // Preserve the original query order and overflow owner while keeping
        // every native clear and target restore after the complete preflight.
        let current: Vec<_> = {
            let mut query = world.query::<(Entity, &mut Controller)>();
            query
                .iter_mut(world)
                .take(MAX_CONTROLLER_INPUTS + 1)
                .map(|(entity, controller)| {
                    (entity, controller.joint, controller.actuation.clear())
                })
                .collect()
        };
        if current.is_empty() {
            return Ok(());
        }
        let slots = admitted_slots(world).unwrap_or_default();
        // Retain each earlier row's native failure before reaching a later
        // overflow, exactly as the original query-order preflight did.
        for (index, (entity, joint, clear)) in current.iter().enumerate() {
            if index >= slots {
                return Err(ControllerFault {
                    entity: *entity,
                    message: "too many loaded controllers".into(),
                });
            }
            clear
                .validate(world.resource::<PhysicsBackend>(), *joint, Duration::ZERO)
                .map_err(|error| ControllerFault {
                    entity: *entity,
                    message: error.to_string(),
                })?;
        }
        validate_loaded_controllers(world)?;
        for (entity, joint, clear) in current {
            clear
                .set(
                    &mut world.resource_mut::<PhysicsBackend>(),
                    joint,
                    Duration::ZERO,
                )
                .map_err(|error| ControllerFault {
                    entity,
                    message: error.to_string(),
                })?;
        }
        let mut query = world.query::<&mut Controller>();
        for mut controller in query.iter_mut(world) {
            controller.target = controller.initial_target;
        }
        Ok(())
    })();
    if let Err(error) = result {
        world.resource::<StepFault>().report(error);
    }
}

#[cfg(test)]
#[path = "../tests/private/joint_controllers.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/multiple_joint_controllers.rs"]
mod multiple_joint_controllers_tests;

#[cfg(test)]
#[path = "../tests/private/multiple_joint_controllers_native.rs"]
mod multiple_joint_controllers_native_tests;
