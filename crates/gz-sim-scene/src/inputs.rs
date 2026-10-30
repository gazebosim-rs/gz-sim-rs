// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{BodyHandle, EntityKind, LocalPose, WorldPose, tpe};
use bevy_ecs::prelude::*;
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_tpe::{ModelBounds, Motion};
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo, entity_by_id};

type Pair = ([f64; 3], [f64; 3]);

#[derive(Resource)]
struct InputsInstalled;

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
    Tpe(Entity),
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
    let rapier = world.get_resource::<PhysicsBackend>();
    if rapier.is_some() == world.contains_resource::<tpe::TpeWorld>() {
        return Err(InputError::InvalidPhysics);
    }
    if let Some(backend) = rapier {
        let id = world
            .get::<BodyHandle>(entity)
            .ok_or(InputError::InvalidPhysics)?
            .id();
        if backend.is_dynamic(id) == Some(false) {
            return Err(InputError::Unsupported("fixed-body input"));
        }
        if backend.is_jointed(id) == Some(true) {
            return Err(InputError::Unsupported("jointed-body input"));
        }
        return backend
            .state(id)
            .map(|_| Binding::Dynamic(id))
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
    if !values.0.into_iter().chain(values.1).all(f64::is_finite) {
        return Err(InputError::InvalidValue);
    }
    let entity = entity_by_id(world, id).ok_or(InputError::MissingEntity(id))?;
    Ok((entity, binding(world, entity, id)?))
}

/// Queues world-frame velocities for a free single-link body at the next advancing step.
/// Replaces a prior velocity request; paused ticks retain it. Reset/removal cancel it.
/// Fixed bodies reject input. TPE accepts rotated model/link frames in world axes.
pub fn queue_link_world_velocity(
    sim: &mut Simulation,
    id: u64,
    linear: [f64; 3],
    angular: [f64; 3],
) -> Result<(), InputError> {
    sim.validate_steps(1)?;
    let values = (linear, angular);
    let (entity, _) = target(sim.world(), id, values)?;
    let mut pending = sim
        .world()
        .get::<PendingInput>(entity)
        .copied()
        .unwrap_or_default();
    pending.velocity = Some(values);
    sim.world_mut().entity_mut(entity).insert(pending);
    Ok(())
}

/// Accumulates world-frame force at the center of mass and world-frame torque.
/// Consumed by one advancing step. TPE has no wrench capability. Invalid admission
/// preserves pending inputs. Runtime failures fault the run, without rollback.
pub fn queue_link_world_wrench(
    sim: &mut Simulation,
    id: u64,
    force: [f64; 3],
    torque: [f64; 3],
) -> Result<(), InputError> {
    sim.validate_steps(1)?;
    let (entity, binding) = target(sim.world(), id, (force, torque))?;
    if matches!(binding, Binding::Tpe(_)) {
        return Err(InputError::Unsupported("TPE external wrench"));
    }
    let mut pending = sim
        .world()
        .get::<PendingInput>(entity)
        .copied()
        .unwrap_or_default();
    let (old_force, old_torque) = pending.wrench.unwrap_or(([0.0; 3], [0.0; 3]));
    let force = std::array::from_fn(|i| old_force[i] + force[i]);
    let torque = std::array::from_fn(|i| old_torque[i] + torque[i]);
    if !force.into_iter().chain(torque).all(f64::is_finite) {
        return Err(InputError::InvalidValue);
    }
    pending.wrench = Some((force, torque));
    sim.world_mut().entity_mut(entity).insert(pending);
    Ok(())
}

/// Accumulates one-step generalized effort for a loaded revolute's public entity ID.
/// Paused ticks retain commands; successful steps consume them. Reset and retirement
/// discard them. Validation errors preserve admitted commands; runtime faults stop the run.
pub fn queue_joint_effort(sim: &mut Simulation, id: u64, torque: f64) -> Result<(), InputError> {
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
    match sim
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .add_joint_effort(joint, torque)
    {
        Err(gz_physics::PhysicsError::InvalidValue) => Err(InputError::InvalidValue),
        result => result.map_err(InputError::Physics),
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
    match binding(world, entity, id)? {
        Binding::Dynamic(body) => {
            let mut backend = world.resource_mut::<PhysicsBackend>();
            if let Some((linear, angular)) = input.velocity {
                backend.set_world_velocity(body, linear, angular)?;
            }
            if let Some((force, torque)) = input.wrench {
                backend.add_world_wrench(body, force, torque)?;
            }
        }
        Binding::Tpe(model) => {
            if input.wrench.is_some() {
                return Err(InputError::Unsupported("TPE external wrench"));
            }
            if let Some((linear, angular)) = input.velocity {
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

fn apply(world: &mut World) {
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

fn clear(mut commands: Commands, inputs: Query<Entity, With<PendingInput>>) {
    for entity in &inputs {
        commands.entity(entity).remove::<PendingInput>();
    }
}

#[cfg(test)]
#[path = "../tests/private/inputs.rs"]
mod tests;
