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
}

/// Removes a top-level model, its descendants and their physics bodies.
/// Call at a lifecycle boundary, outside an executing simulation schedule.
/// All checks precede mutation. The returned public IDs are retired in this world.
/// This does not restore removed models during the existing core time reset.
pub fn remove_model(sim: &mut Simulation, id: u64) -> Result<Vec<u64>, RemoveError> {
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
    if let Some(mut backend) = sim.world_mut().get_resource_mut::<PhysicsBackend>() {
        for body in bodies {
            assert!(
                backend.remove(body),
                "preflighted bodies remain owned until commit"
            );
        }
    }
    // ChildOf's linked-spawn relationship also removes descendants and ID hooks.
    sim.world_mut().entity_mut(root).despawn();
    ids.sort_unstable();
    Ok(ids)
}
