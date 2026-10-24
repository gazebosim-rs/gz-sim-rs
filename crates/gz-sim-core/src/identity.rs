// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::{lifecycle::HookContext, prelude::*, world::DeferredWorld};
use std::collections::BTreeMap;

/// A world-local public identity, independent of Bevy's entity generation.
/// Assign with `assign_id`; do not transfer this component between entities.
#[derive(Component, Debug)]
#[component(immutable, on_discard = forget_id)]
pub struct EntityId(u64);

impl EntityId {
    pub fn get(&self) -> u64 {
        self.0
    }
}

#[derive(Resource, Default)]
struct IdIndex {
    last: u64,
    entities: BTreeMap<u64, Entity>,
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum IdentityError {
    #[error("entity does not exist")]
    MissingEntity,
    #[error("public entity IDs are exhausted")]
    Exhausted,
    #[error("entity identity was moved outside its owning world or entity")]
    MovedIdentity,
}

/// Assigns one nonzero ID for the entity's lifetime. IDs are never recycled.
/// Repeated assignment returns its existing ID. Failure leaves the entity unchanged.
pub fn assign_id(world: &mut World, entity: Entity) -> Result<u64, IdentityError> {
    let current = world
        .get_entity(entity)
        .map_err(|_| IdentityError::MissingEntity)?;
    if let Some(id) = current.get::<EntityId>() {
        return if entity_by_id(world, id.0) == Some(entity) {
            Ok(id.0)
        } else {
            Err(IdentityError::MovedIdentity)
        };
    }
    world.init_resource::<IdIndex>();
    let mut index = world.resource_mut::<IdIndex>();
    let id = index.last.checked_add(1).ok_or(IdentityError::Exhausted)?;
    index.last = id;
    index.entities.insert(id, entity);
    world.entity_mut(entity).insert(EntityId(id));
    Ok(id)
}

/// Resolves only a live entity that still owns the requested identity.
pub fn entity_by_id(world: &World, id: u64) -> Option<Entity> {
    let entity = *world.get_resource::<IdIndex>()?.entities.get(&id)?;
    (world.get::<EntityId>(entity)?.0 == id).then_some(entity)
}

fn forget_id(mut world: DeferredWorld, context: HookContext) {
    let id = world
        .get::<EntityId>(context.entity)
        .expect("removal hook runs before the identity is removed")
        .0;
    if let Some(mut index) = world.get_resource_mut::<IdIndex>()
        && index.entities.get(&id) == Some(&context.entity)
    {
        index.entities.remove(&id);
    }
}

#[cfg(test)]
#[path = "../tests/private/identity.rs"]
mod tests;
