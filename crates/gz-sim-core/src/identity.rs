// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::{
    lifecycle::HookContext,
    prelude::*,
    system::SystemParam,
    world::{DeferredWorld, WorldId},
};
use std::{collections::BTreeMap, sync::Arc};

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
    owner: Arc<()>,
    last: u64,
    entities: BTreeMap<u64, Entity>,
}

/// Read-only public-ID resolution for scheduled systems, checking the live owner.
#[derive(SystemParam)]
pub struct EntityLookup<'w, 's> {
    index: Option<Res<'w, IdIndex>>,
    identities: Query<'w, 's, &'static EntityId>,
}

impl EntityLookup<'_, '_> {
    pub fn get(&self, id: u64) -> Option<Entity> {
        let entity = *self.index.as_ref()?.entities.get(&id)?;
        (self.identities.get(entity).ok()?.get() == id).then_some(entity)
    }
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum IdentityError {
    #[error("entity does not exist")]
    MissingEntity,
    #[error("public entity IDs are exhausted")]
    Exhausted,
    #[error("entity identity was moved outside its owning world or entity")]
    MovedIdentity,
    #[error("entity has no public identity")]
    MissingIdentity,
    #[error("identity reservation belongs to a different world")]
    ForeignReservation,
    #[error("identity allocator state no longer contains this reservation")]
    InvalidReservation,
    #[error("public entity {0} still has an owner")]
    IdentityInUse(u64),
    #[error("target already has a different or invalid identity")]
    AlreadyIdentified,
}

/// Permission to restore an initial logical object's ID in its original world.
/// Pair restored objects with the simulation epoch when retaining runtime handles.
#[derive(Clone, Debug)]
pub struct ReservedId {
    world: WorldId,
    allocator: Arc<()>,
    id: u64,
}

impl ReservedId {
    pub fn get(&self) -> u64 {
        self.id
    }

    /// Checks origin and allocator continuity; the ID may still have a live owner.
    pub fn validate(&self, world: &World) -> Result<(), IdentityError> {
        if world.id() != self.world {
            return Err(IdentityError::ForeignReservation);
        }
        if !world.get_resource::<IdIndex>().is_some_and(|index| {
            Arc::ptr_eq(&index.owner, &self.allocator) && index.last >= self.id
        }) {
            return Err(IdentityError::InvalidReservation);
        }
        Ok(())
    }
}

/// Captures an existing identity for explicit restoration during a scene reset.
/// Ordinary assignment never allocates this ID to a different logical object.
pub fn reserve_id(world: &World, entity: Entity) -> Result<ReservedId, IdentityError> {
    let current = world
        .get_entity(entity)
        .map_err(|_| IdentityError::MissingEntity)?;
    let id = current
        .get::<EntityId>()
        .ok_or(IdentityError::MissingIdentity)?
        .get();
    if entity_by_id(world, id) != Some(entity) {
        return Err(IdentityError::MovedIdentity);
    }
    Ok(ReservedId {
        world: world.id(),
        allocator: world.resource::<IdIndex>().owner.clone(),
        id,
    })
}

/// Restores a reserved logical identity after its prior owner is gone.
/// Repeated restoration to the same owner is harmless. All errors precede mutation.
pub fn restore_id(
    world: &mut World,
    entity: Entity,
    reserved: &ReservedId,
) -> Result<u64, IdentityError> {
    reserved.validate(world)?;
    let current = world
        .get_entity(entity)
        .map_err(|_| IdentityError::MissingEntity)?;
    if let Some(id) = current.get::<EntityId>() {
        return if id.get() == reserved.id && entity_by_id(world, reserved.id) == Some(entity) {
            Ok(reserved.id)
        } else {
            Err(IdentityError::AlreadyIdentified)
        };
    }
    let mut index = world.resource_mut::<IdIndex>();
    if index.entities.contains_key(&reserved.id) {
        return Err(IdentityError::IdentityInUse(reserved.id));
    }
    index.entities.insert(reserved.id, entity);
    world.entity_mut(entity).insert(EntityId(reserved.id));
    Ok(reserved.id)
}

/// Assigns one nonzero ID for the entity's lifetime within an intact allocator.
/// Clearing required world resources tears down the identity namespace.
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
