// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{BodyHandle, EntityKind};
use bevy_ecs::hierarchy::{ChildOf, Children};
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use std::collections::BTreeSet;

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum RemoveError {
    #[error("public entity {0} does not exist")]
    MissingEntity(u64),
    #[error("entity {0} is not a model directly owned by the world")]
    NotWorldModel(u64),
    #[error("scene physics bindings are incomplete or inconsistent")]
    InvalidPhysics,
    #[error("scene schedule initialization failed: {0}")]
    Schedule(String),
    #[error("model was removed but system schedule retirement failed: {0}")]
    CommittedSchedule(String),
}

/// Removes a top-level model, its descendants and their physics bodies.
/// Call at a lifecycle boundary, outside an executing simulation schedule.
/// All checks precede model/physics mutation. Retiring the last IMU or joint initializes
/// pending schedules without running callbacks; arbitrary Local initializers
/// follow the core initialization contract. Returned public IDs retire here.
/// Schedule-built observers run before final validation or after body commit.
/// CommittedSchedule reports teardown failure after removal and faults the run.
/// This does not restore removed models during the existing core time reset.
pub fn remove_model(sim: &mut Simulation, id: u64) -> Result<Vec<u64>, RemoveError> {
    remove_model_checked(sim, id, false)
}

#[derive(Debug, thiserror::Error)]
pub(super) enum SystemRetirementError {
    #[error(transparent)]
    Schedule(#[from] bevy_ecs::schedule::ScheduleError),
    #[error("a schedule observer added consumers while their callbacks were being retired")]
    AddedConsumer,
}

pub(super) fn retire_unused_systems(sim: &mut Simulation) -> Result<(), SystemRetirementError> {
    loop {
        let before = (
            sim.world()
                .contains_resource::<crate::imu_lifecycle::Installed>(),
            sim.world()
                .contains_resource::<crate::joint_lifecycle::Installed>(),
            sim.world()
                .contains_resource::<crate::joint_force_lifecycle::Installed>(),
        );
        crate::imu_lifecycle::retire_if_unused(sim)?;
        crate::joint_lifecycle::retire_if_unused(sim)?;
        crate::joint_force_lifecycle::retire_if_unused(sim)?;
        let after = (
            sim.world()
                .contains_resource::<crate::imu_lifecycle::Installed>(),
            sim.world()
                .contains_resource::<crate::joint_lifecycle::Installed>(),
            sim.world()
                .contains_resource::<crate::joint_force_lifecycle::Installed>(),
        );
        if before == after {
            if sim.world().iter_entities().any(|entity| {
                !after.0 && entity.contains::<crate::Imu>()
                    || !after.1 && entity.contains::<crate::JointHandle>()
                    || !after.2 && entity.contains::<crate::JointForceCommand>()
            }) {
                return Err(SystemRetirementError::AddedConsumer);
            }
            return Ok(());
        }
        // A schedule build may remove another family's last consumer.
        // Markers only retire here, so at most three rounds can change them.
    }
}

fn remove_model_checked(
    sim: &mut Simulation,
    id: u64,
    schedules_ready: bool,
) -> Result<Vec<u64>, RemoveError> {
    let world = sim.world();
    let root = entity_by_id(world, id).ok_or(RemoveError::MissingEntity(id))?;
    let parent = world.get::<ChildOf>(root).map(ChildOf::parent);
    if world.get::<EntityKind>(root) != Some(&EntityKind::Model)
        || parent.and_then(|entity| world.get::<EntityKind>(entity)) != Some(&EntityKind::World)
    {
        return Err(RemoveError::NotWorldModel(id));
    }
    if !crate::joints::bindings_valid(world) {
        return Err(RemoveError::InvalidPhysics);
    }
    let backend = world.get_resource::<PhysicsBackend>();
    if backend.is_some() == world.contains_resource::<crate::tpe::TpeWorld>() {
        return Err(RemoveError::InvalidPhysics);
    }
    let mut pending = vec![root];
    let mut bodies = BTreeSet::new();
    let mut subtree = BTreeSet::new();
    let mut ids = Vec::new();
    while let Some(entity) = pending.pop() {
        if backend.is_none() {
            let valid = match world.get::<EntityKind>(entity) {
                Some(EntityKind::Model) => world.get::<gz_physics_tpe::Motion>(entity).is_some(),
                Some(EntityKind::Link) => world.get::<crate::tpe::TpeLink>(entity).is_some(),
                _ => true,
            };
            if !valid {
                return Err(RemoveError::InvalidPhysics);
            }
        }
        subtree.insert(entity);
        if let Some(id) = world.get::<EntityId>(entity) {
            ids.push(id.get());
        }
        if let Some(body) = world.get::<BodyHandle>(entity)
            && (backend.is_none_or(|backend| backend.state(body.id()).is_none())
                || !bodies.insert(body.id()))
        {
            return Err(RemoveError::InvalidPhysics);
        }
        if let Some(children) = world.get::<Children>(entity) {
            pending.extend(children.iter());
        }
    }
    if world.iter_entities().any(|entity| {
        !subtree.contains(&entity.id())
            && entity
                .get::<BodyHandle>()
                .is_some_and(|body| bodies.contains(&body.id()))
    }) {
        return Err(RemoveError::InvalidPhysics);
    }
    let retire_imu = world.contains_resource::<crate::imu_lifecycle::Installed>()
        && !world
            .iter_entities()
            .any(|entity| entity.contains::<crate::Imu>() && !subtree.contains(&entity.id()));
    let retire_joint = world.contains_resource::<crate::joint_lifecycle::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::JointHandle>() && !subtree.contains(&entity.id())
        });
    let retire_force = world.contains_resource::<crate::joint_force_lifecycle::Installed>()
        && !world.iter_entities().any(|entity| {
            entity.contains::<crate::JointForceCommand>() && !subtree.contains(&entity.id())
        });
    if (retire_imu || retire_joint || retire_force) && !schedules_ready {
        sim.initialize_schedules()
            .map_err(|error| RemoveError::Schedule(error.to_string()))?;
        // Builds synchronously notify World observers. Their mutations must
        // precede the final hierarchy, binding and native-body validation.
        return remove_model_checked(sim, id, true);
    }
    let body_order =
        crate::joints::removal_order(sim.world(), &bodies).ok_or(RemoveError::InvalidPhysics)?;
    if let Some(mut backend) = sim.world_mut().get_resource_mut::<PhysicsBackend>() {
        for body in body_order {
            assert!(
                backend.remove(body),
                "preflighted bodies remain owned until commit"
            );
        }
    }
    // ChildOf's linked-spawn relationship also removes descendants and ID hooks.
    sim.world_mut().entity_mut(root).despawn();
    ids.sort_unstable();
    // No schedule build may intervene between final validation and body commit.
    // Teardown observers now see the committed removal, not stale body handles.
    if let Err(error) = retire_unused_systems(sim) {
        let message = error.to_string();
        sim.world()
            .resource::<gz_sim_core::StepFault>()
            .report(error);
        return Err(RemoveError::CommittedSchedule(message));
    }
    Ok(ids)
}
