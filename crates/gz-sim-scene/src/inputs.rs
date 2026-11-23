// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{BodyHandle, EntityKind, LinkCenterOfMass, LocalPose, WorldPose, tpe};
use bevy_ecs::prelude::*;
use gz_physics::{BodyId, JointId, PhysicsBackend, PhysicsError};
use gz_physics_tpe::{ModelBounds, Motion};
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo, entity_by_id};
use std::time::Duration;

type Pair = ([f64; 3], [f64; 3]);
#[cfg(feature = "distributed-physics")]
pub(super) type SavedInput = (u64, Option<Pair>, Option<Pair>);

#[derive(Resource)]
struct InputsInstalled;

#[cfg(feature = "distributed-physics")]
pub(super) fn has_pending_input(world: &World) -> bool {
    world.component_id::<PendingInput>().is_some_and(|id| {
        world
            .archetypes()
            .iter()
            .any(|a| a.contains(id) && !a.is_empty())
    })
}

#[cfg(feature = "distributed-physics")]
pub(super) fn saved_inputs(world: &World) -> Vec<SavedInput> {
    world
        .iter_entities()
        .filter_map(|e| {
            let p = e.get::<PendingInput>()?;
            Some((
                e.get::<gz_sim_core::EntityId>()?.get(),
                p.velocity,
                p.wrench,
            ))
        })
        .collect()
}
#[derive(Component, Clone, Copy, Default)]
struct PendingInput {
    velocity: Option<Pair>,
    wrench: Option<Pair>,
}

#[derive(Debug, thiserror::Error)]
pub enum InputError {
    #[error("input contains a nonfinite value or accumulated wrench overflow")]
    InvalidValue,
    #[error("public entity {0} does not exist")]
    MissingEntity(u64),
    #[error("entity {0} is not a supported top-level single-link model's link")]
    NotFreeLink(u64),
    #[error("entity {0} is not a loaded joint")]
    NotJoint(u64),
    #[error("scene physics bindings are incomplete or inconsistent")]
    InvalidPhysics,
    #[error("unsupported physics input: {0}")]
    Unsupported(&'static str),
    #[error(transparent)]
    Clock(#[from] gz_sim_core::StepError),
    #[error(transparent)]
    Physics(#[from] gz_physics::PhysicsError),
    #[error(transparent)]
    Tpe(#[from] gz_physics_tpe::MotionError),
}

enum Binding {
    Dynamic(BodyId),
    Kinematic(BodyId),
    Tpe(Entity),
}

impl Binding {
    fn require_wrench(&self) -> Result<(), InputError> {
        match self {
            Self::Dynamic(_) => Ok(()),
            Self::Kinematic(_) => Err(InputError::Unsupported(
                "prescribed kinematic external wrench",
            )),
            Self::Tpe(_) => Err(InputError::Unsupported("TPE external wrench")),
        }
    }

    fn velocity_at_com(
        &self,
        world: &World,
        entity: Entity,
        (linear, angular): Pair,
        dt: Duration,
    ) -> Result<[f64; 3], InputError> {
        let (Self::Dynamic(body) | Self::Kinematic(body)) = self else {
            return Ok(linear);
        };
        let backend = world.resource::<PhysicsBackend>();
        let linear = if let Some(center) = world.get::<LinkCenterOfMass>(entity) {
            let state = backend.state(*body).ok_or(InputError::InvalidPhysics)?;
            (glam::DVec3::from_array(linear) + center.velocity_shift(state.orientation, angular))
                .to_array()
        } else {
            linear
        };
        if matches!(self, Self::Kinematic(_)) {
            backend
                .validate_kinematic_velocity(linear, angular, dt)
                .map_err(input_error)?;
        }
        Ok(linear)
    }
}

fn binding(world: &World, entity: Entity, id: u64) -> Result<Binding, InputError> {
    if !world.contains_resource::<InputsInstalled>()
        || !crate::initial::matches_physics_owner(world)
    {
        return Err(InputError::InvalidPhysics);
    }
    let model = world.get::<ChildOf>(entity).map(ChildOf::parent);
    let model = model
        .filter(|m| world.get::<EntityKind>(*m) == Some(&EntityKind::Model))
        .ok_or(InputError::NotFreeLink(id))?;
    let parent = world.get::<ChildOf>(model).map(ChildOf::parent);
    if world.get::<EntityKind>(entity) != Some(&EntityKind::Link)
        || parent.and_then(|p| world.get::<EntityKind>(p)) != Some(&EntityKind::World)
        || world
            .get::<Children>(model)
            .into_iter()
            .flat_map(|c| c.iter())
            .filter(|e| world.get::<EntityKind>(*e) == Some(&EntityKind::Link))
            .count()
            != 1
    {
        return Err(InputError::NotFreeLink(id));
    }
    let backend = world.get_resource::<PhysicsBackend>();
    if backend.is_some() == world.contains_resource::<tpe::TpeWorld>() {
        return Err(InputError::InvalidPhysics);
    }
    if let Some(backend) = backend {
        let id = world
            .get::<BodyHandle>(entity)
            .ok_or(InputError::InvalidPhysics)?
            .id();
        let binding = match backend.is_dynamic(id) {
            Some(true) => Binding::Dynamic(id),
            Some(false) if backend.is_kinematic(id) == Some(true) => Binding::Kinematic(id),
            Some(false) => return Err(InputError::Unsupported("fixed-body input")),
            None => return Err(InputError::InvalidPhysics),
        };
        if backend.is_jointed(id) == Some(true) {
            return Err(InputError::Unsupported("jointed-body input"));
        }
        return backend
            .state(id)
            .map(|_| binding)
            .ok_or(InputError::InvalidPhysics);
    }
    let bounds = world
        .get::<ModelBounds>(model)
        .ok_or(InputError::InvalidPhysics)?;
    world
        .get::<LocalPose>(entity)
        .ok_or(InputError::InvalidPhysics)?;
    world
        .get::<WorldPose>(model)
        .ok_or(InputError::InvalidPhysics)?;
    if bounds.is_static() {
        return Err(InputError::Unsupported("fixed-body input"));
    }
    if world.get::<Motion>(model).is_none() || world.get::<tpe::TpeLink>(entity).is_none() {
        return Err(InputError::InvalidPhysics);
    }
    Ok(Binding::Tpe(model))
}

fn target(world: &World, id: u64, values: Pair) -> Result<(Entity, Binding), InputError> {
    let entity = target_entity(world, id, values)?;
    Ok((entity, binding(world, entity, id)?))
}

fn target_entity(world: &World, id: u64, values: Pair) -> Result<Entity, InputError> {
    if !values.0.into_iter().chain(values.1).all(f64::is_finite) {
        return Err(InputError::InvalidValue);
    }
    entity_by_id(world, id).ok_or(InputError::MissingEntity(id))
}

/// Queues world-frame velocities for a free single-link body at the next advancing step.
/// Dynamic backends interpret linear velocity at the link origin; TPE keeps
/// its model-motion velocity. Replaces a prior request; paused ticks retain it.
/// Reset/removal cancel it. Backend COM conversion happens at application.
/// Fixed bodies reject input. A multi-worker body with retained native contact
/// history rejects a twist before it replaces any queued input. Cold topology
/// may change while paused and is checked at application. TPE accepts rotated
/// model/link frames in world axes.
pub fn queue_link_world_velocity(
    sim: &mut Simulation,
    id: u64,
    linear: [f64; 3],
    angular: [f64; 3],
) -> Result<(), InputError> {
    sim.validate_steps(1)?;
    let values = (linear, angular);
    let (entity, binding) = target(sim.world(), id, values)?;
    #[cfg(feature = "distributed-physics")]
    if let Binding::Dynamic(body) | Binding::Kinematic(body) = &binding
        && let Some(scene) = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<gz_physics_distributed::SceneBackend>()
    {
        scene
            .validate_deferred_velocity_target(*body)
            .map_err(input_error)?;
    }
    if matches!(binding, Binding::Kinematic(_)) {
        if crate::damped_sole::translating_support(sim.world(), id) && angular != [0.; 3] {
            return Err(InputError::Unsupported("coated sole support rotation"));
        }
        binding.velocity_at_com(sim.world(), entity, values, sim.step_duration())?;
    }
    let mut pending = sim
        .world()
        .get::<PendingInput>(entity)
        .copied()
        .unwrap_or_default();
    pending.velocity = Some(values);
    sim.world_mut().entity_mut(entity).insert(pending);
    Ok(())
}

/// Queue a world-frame force at a point expressed in link-local coordinates.
///
/// The lever arm is measured from the authored center of mass at the committed
/// pose. The resulting wrench follows the same one-advancing-step lifetime as
/// [`queue_link_world_wrench`], including when queued while paused.
pub fn queue_link_force_at_point(
    sim: &mut Simulation,
    id: u64,
    point_in_link: [f64; 3],
    world_force: [f64; 3],
) -> Result<(), InputError> {
    let world = sim.world();
    let entity = target_entity(world, id, (point_in_link, world_force))?;
    let pose = world
        .get::<WorldPose>(entity)
        .ok_or(InputError::InvalidPhysics)?;
    let center = world
        .get::<LinkCenterOfMass>(entity)
        .map_or([0.; 3], |offset| offset.0);
    let local_arm = glam::DVec3::from(point_in_link) - glam::DVec3::from(center);
    let arm = glam::DQuat::from_array(pose.rotation) * local_arm;
    let torque = arm.cross(glam::DVec3::from(world_force));
    queue_link_world_wrench(sim, id, world_force, torque.to_array())
}

/// Accumulates world-frame force at the center of mass and world-frame torque.
/// On a switchable articulation, its solver holds the wrench for one dynamic
/// step. A prescribed assembly carries it through promotion; reset cancels it.
/// Consumed by one advancing step. TPE has no wrench capability. Invalid admission
/// preserves pending inputs. Runtime failures fault the run, without rollback.
pub fn queue_link_world_wrench(
    sim: &mut Simulation,
    id: u64,
    force: [f64; 3],
    torque: [f64; 3],
) -> Result<(), InputError> {
    sim.validate_steps(1)?;
    let entity = target_entity(sim.world(), id, (force, torque))?;
    let body = crate::articulation::wrench_body(sim.world(), entity, id)
        .map_err(|_| InputError::InvalidPhysics)?
        .or_else(|| {
            let world = sim.world();
            let model = world.get::<ChildOf>(entity)?.parent();
            // Ordinary primitive inputs need no scan of the native joint registry.
            if !world.get::<Children>(model)?.iter().any(|child| {
                world.get::<crate::JointDefinition>(child).is_some_and(|j| {
                    matches!(
                        j.0.kind,
                        sdformat::JointKind::Fixed
                            | sdformat::JointKind::Prismatic
                            | sdformat::JointKind::Screw
                            | sdformat::JointKind::Universal
                            | sdformat::JointKind::Revolute2
                            | sdformat::JointKind::Ball
                    )
                })
            }) {
                return None;
            }
            let body = world.get::<BodyHandle>(entity)?.id();
            let native = world
                .get_resource::<PhysicsBackend>()?
                .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()?;
            ((native.is_welded_link(body) || native.is_robot_link(body))
                && crate::initial::matches_physics_owner(world)
                && crate::initial::owns_loaded_body(world, id, body))
            .then_some(body)
        });
    if let Some(body) = body {
        if !sim.world().contains_resource::<InputsInstalled>() {
            return Err(InputError::InvalidPhysics);
        }
        return sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_world_wrench(body, force, torque)
            .map_err(input_error);
    }
    let target = binding(sim.world(), entity, id)?;
    target.require_wrench()?;
    #[cfg(feature = "distributed-physics")]
    if let Binding::Dynamic(body) = target
        && let Some(mut backend) = sim.world_mut().get_resource_mut::<PhysicsBackend>()
        && (backend
            .downcast_ref::<gz_physics_distributed::SceneBackend>()
            .is_some()
            || backend.engine() == "distributed-hinge")
    {
        // Let the worker-backed implementation check target/capability at admission.
        // The coordinator accounts for this one-tick wrench during routing.
        // Native force and torque act throughout all internal substeps.
        return backend
            .add_world_wrench(body, force, torque)
            .map_err(input_error);
    }
    let mut pending = sim
        .world()
        .get::<PendingInput>(entity)
        .copied()
        .unwrap_or_default();
    accumulate_wrench(&mut pending, force, torque)?;
    sim.world_mut().entity_mut(entity).insert(pending);
    Ok(())
}

fn accumulate_wrench(
    pending: &mut PendingInput,
    force: [f64; 3],
    torque: [f64; 3],
) -> Result<(), InputError> {
    let (old_force, old_torque) = pending.wrench.unwrap_or(([0.0; 3], [0.0; 3]));
    let force = std::array::from_fn(|i| old_force[i] + force[i]);
    let torque = std::array::from_fn(|i| old_torque[i] + torque[i]);
    if !force.into_iter().chain(torque).all(f64::is_finite) {
        return Err(InputError::InvalidValue);
    }
    pending.wrench = Some((force, torque));
    Ok(())
}

fn wrench_shadows(
    world: &World,
    entries: &[(u64, [f64; 3], [f64; 3])],
) -> Result<std::collections::BTreeMap<Entity, PendingInput>, InputError> {
    let mut shadows = std::collections::BTreeMap::new();
    for &(id, force, torque) in entries {
        let (entity, binding) = target(world, id, (force, torque))?;
        binding.require_wrench()?;
        let pending = shadows.entry(entity).or_insert_with(|| {
            world
                .get::<PendingInput>(entity)
                .copied()
                .unwrap_or_default()
        });
        // Keep every addition in caller order, starting at the existing pending
        // input. Pre-aggregation would change both rounding and overflow refusal.
        accumulate_wrench(pending, force, torque)?;
    }
    Ok(shadows)
}

/// Checks the complete ordered batch without changing pending input components.
pub(super) fn validate_world_wrench_batch(
    world: &World,
    entries: &[(u64, [f64; 3], [f64; 3])],
) -> Result<(), InputError> {
    wrench_shadows(world, entries).map(|_| ())
}

/// Commits only after every target and every intermediate sum is valid.
/// This preserves unrelated pending velocity commands and singleton API inputs.
pub(super) fn queue_world_wrench_batch(
    world: &mut World,
    entries: &[(u64, [f64; 3], [f64; 3])],
) -> Result<(), InputError> {
    let shadows = wrench_shadows(world, entries)?;
    for (entity, pending) in shadows {
        world.entity_mut(entity).insert(pending);
    }
    Ok(())
}

/// Latest persistent generalized torque in Nm, applied once per advancing step.
/// Set through `set_joint_force`; reset clears it and retirement removes it.
#[derive(Component, Clone, Copy, Debug, Default)]
pub struct JointForceCommand(f64);

impl JointForceCommand {
    pub fn torque(&self) -> f64 {
        self.0
    }
}

fn joint_target(sim: &Simulation, id: u64, torque: f64) -> Result<(Entity, JointId), InputError> {
    sim.validate_steps(1)?;
    if !torque.is_finite() {
        return Err(InputError::InvalidValue);
    }
    let world = sim.world();
    let entity = entity_by_id(world, id).ok_or(InputError::MissingEntity(id))?;
    if world.get::<EntityKind>(entity) != Some(&EntityKind::Joint) {
        return Err(InputError::NotJoint(id));
    }
    if !world.contains_resource::<InputsInstalled>() || world.contains_resource::<tpe::TpeWorld>() {
        return Err(InputError::InvalidPhysics);
    }
    let joint = crate::joints::input_binding(world, entity).ok_or(InputError::InvalidPhysics)?;
    Ok((entity, joint))
}

fn input_error(error: PhysicsError) -> InputError {
    match error {
        PhysicsError::InvalidValue => InputError::InvalidValue,
        error => InputError::Physics(error),
    }
}

/// Replaces the persistent force on a loaded joint, without changing one-step inputs.
/// Paused ticks retain the latest value; zero stops future forcing. Individual
/// admission errors preserve the prior value. Combined inputs are checked at stepping.
pub fn set_joint_force(sim: &mut Simulation, id: u64, torque: f64) -> Result<(), InputError> {
    JointForceCommand::set_batch(sim, [(id, torque)])
}

impl JointForceCommand {
    /// Set persistent generalized efforts on a group of loaded joints together.
    /// Each input is (public joint entity ID, torque in N·m or slider force in N).
    /// Validation completes before any command changes. For repeated joint IDs
    /// the last supplied value takes effect.
    pub fn set_batch<const N: usize>(
        sim: &mut Simulation,
        commands: [(u64, f64); N],
    ) -> Result<(), InputError> {
        let mut entities = [Entity::PLACEHOLDER; N];
        for (slot, (id, effort)) in entities.iter_mut().zip(commands) {
            let (entity, joint) = joint_target(sim, id, effort)?;
            let backend = sim.world().resource::<PhysicsBackend>();
            #[cfg(feature = "distributed-physics")]
            if backend.engine() == "distributed-hinge" {
                return Err(input_error(PhysicsError::Unsupported {
                    engine: backend.engine(),
                    capability: "persistent joint effort",
                }));
            }
            backend
                .validate_joint_effort(joint, effort)
                .map_err(input_error)?;
            *slot = entity;
        }
        for (entity, (_, effort)) in entities.into_iter().zip(commands) {
            sim.world_mut().entity_mut(entity).insert(Self(effort));
        }
        if N != 0 {
            crate::joint_force_lifecycle::install(sim);
        }
        Ok(())
    }
}

/// Accumulates a one-step torque (N·m) on a hinge or force (N) on a slider by public ID.
/// Paused ticks retain commands; successful steps consume them. Reset and retirement
/// discard them. Validation errors preserve admitted commands; runtime faults stop the run.
pub fn queue_joint_effort(sim: &mut Simulation, id: u64, torque: f64) -> Result<(), InputError> {
    let (_, joint) = joint_target(sim, id, torque)?;
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .add_joint_effort(joint, torque)
        .map_err(input_error)
}

/// Accumulates one-step torques in N·m on the two moving universal axes.
pub fn queue_universal_joint_effort(
    sim: &mut Simulation,
    id: u64,
    efforts: [f64; 2],
) -> Result<(), InputError> {
    let (_, joint) = joint_target(sim, id, efforts[0])?;
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<gz_physics_multifidelity::MultiFidelityBackend>()
        .ok_or(InputError::InvalidPhysics)?
        .add_universal_effort(joint, efforts)
        .map_err(input_error)
}

pub(super) fn apply_joint_forces(
    time: Res<UpdateInfo>,
    mut backend: ResMut<PhysicsBackend>,
    commands: Query<(Option<&crate::JointHandle>, &JointForceCommand)>,
    fault: Res<StepFault>,
) {
    if time.dt.is_zero() {
        return;
    }
    for (handle, command) in &commands {
        let Some(handle) = handle else {
            fault.report(InputError::InvalidPhysics);
            return;
        };
        if command.0 == 0.0 {
            continue;
        }
        if let Err(error) = backend.add_joint_effort(handle.id(), command.0) {
            fault.report(error);
            return;
        }
    }
}

pub(super) fn install(sim: &mut Simulation) {
    sim.world_mut().insert_resource(InputsInstalled);
    sim.add_systems(Phase::PhysicsPrepare, apply);
    sim.add_systems(Phase::Reset, clear);
}

fn execute(world: &mut World, entity: Entity, input: PendingInput) -> Result<(), InputError> {
    let id = world
        .get::<gz_sim_core::EntityId>(entity)
        .ok_or(InputError::InvalidPhysics)?
        .get();
    let binding = binding(world, entity, id)?;
    if input.wrench.is_some() {
        binding.require_wrench()?;
    }
    let velocity = input
        .velocity
        .map(|values| {
            binding
                .velocity_at_com(world, entity, values, world.resource::<UpdateInfo>().dt)
                .map(|linear| (linear, values.1))
        })
        .transpose()?;
    match binding {
        Binding::Dynamic(body) | Binding::Kinematic(body) => {
            let mut backend = world.resource_mut::<PhysicsBackend>();
            if let Some((linear, angular)) = velocity {
                backend.set_world_velocity(body, linear, angular)?;
            }
            if let Some((force, torque)) = input.wrench {
                backend.add_world_wrench(body, force, torque)?;
            }
        }
        Binding::Tpe(model) => {
            if let Some((linear, angular)) = velocity {
                world
                    .get_mut::<Motion>(model)
                    .ok_or(InputError::InvalidPhysics)?
                    .set_world_velocity(linear, angular)?;
            }
        }
    }
    if let Some((linear, _)) = input.velocity {
        // A velocity replacement is configuration, not an integrated force impulse.
        let children: Vec<_> = world
            .get::<Children>(entity)
            .map(|children| children.iter().collect())
            .unwrap_or_default();
        for child in children {
            if let Some(mut sensor) = world.get_mut::<crate::Imu>(child)
                && sensor.link == id
            {
                sensor.previous_velocity = glam::DVec3::from_array(linear);
            }
        }
    }
    Ok(())
}

pub(super) fn apply(world: &mut World) {
    if world.resource::<UpdateInfo>().dt.is_zero() {
        return;
    }
    let inputs: Vec<_> = world
        .query::<(Entity, &PendingInput)>()
        .iter(world)
        .map(|(e, p)| (e, *p))
        .collect();
    for (entity, input) in inputs {
        if let Err(error) = execute(world, entity, input) {
            world.resource::<StepFault>().report(error);
            return;
        }
        world.entity_mut(entity).remove::<PendingInput>();
    }
}

fn clear(
    mut commands: Commands,
    inputs: Query<Entity, With<PendingInput>>,
    mut forces: Query<&mut JointForceCommand>,
) {
    for entity in &inputs {
        commands.entity(entity).remove::<PendingInput>();
    }
    for mut force in &mut forces {
        force.0 = 0.0;
    }
}

#[cfg(test)]
#[path = "../tests/private/inputs.rs"]
mod tests;
