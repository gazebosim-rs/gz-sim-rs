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
    pages: BTreeMap<u64, Box<IdPage>>,
}

// Public IDs are allocated in order but whole ranges may retire. Keep a dense
// array within each occupied page and release the page when its last ID retires.
const PAGE_SIZE: usize = 64;

struct IdPage {
    entries: [Option<Entity>; PAGE_SIZE],
    live: usize,
}

impl Default for IdPage {
    fn default() -> Self {
        Self {
            entries: [None; PAGE_SIZE],
            live: 0,
        }
    }
}

fn location(id: u64) -> (u64, usize) {
    (id / PAGE_SIZE as u64, (id % PAGE_SIZE as u64) as usize)
}

impl IdIndex {
    fn entity(&self, id: u64) -> Option<Entity> {
        let (key, slot) = location(id);
        self.pages.get(&key)?.entries[slot]
    }

    fn insert(&mut self, id: u64, entity: Entity) {
        let (key, slot) = location(id);
        let page = self.pages.entry(key).or_default();
        if page.entries[slot].replace(entity).is_none() {
            page.live += 1;
        }
    }

    fn forget(&mut self, id: u64, entity: Entity) {
        let (key, slot) = location(id);
        let std::collections::btree_map::Entry::Occupied(mut entry) = self.pages.entry(key) else {
            return;
        };
        let page = entry.get_mut();
        if page.entries[slot] == Some(entity) {
            page.entries[slot] = None;
            page.live -= 1;
            if page.live == 0 {
                entry.remove();
            }
        }
    }
}

/// Read-only public-ID resolution for scheduled systems, checking the live owner.
#[derive(SystemParam)]
pub struct EntityLookup<'w, 's> {
    index: Option<Res<'w, IdIndex>>,
    identities: Query<'w, 's, &'static EntityId>,
}

impl EntityLookup<'_, '_> {
    pub fn get(&self, id: u64) -> Option<Entity> {
        self.live_owner(id, self.index.as_ref()?.entity(id)?)
    }

    /// A scoped resolver for many public IDs. Nearby lookups share a page
    /// search, including when IDs repeat or arrive out of order. Each access
    /// still checks the current ECS component against its authoritative owner.
    pub fn reader(&self) -> impl FnMut(u64) -> Option<Entity> + '_ {
        let index = self.index.as_deref();
        let mut cached: Option<(u64, &IdPage)> = None;
        move |id| {
            let (key, slot) = location(id);
            if cached.is_none_or(|(previous, _)| previous != key) {
                cached = Some((key, index?.pages.get(&key)?));
            }
            self.live_owner(id, cached?.1.entries[slot]?)
        }
    }

    fn live_owner(&self, id: u64, entity: Entity) -> Option<Entity> {
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
    if index.entity(reserved.id).is_some() {
        return Err(IdentityError::IdentityInUse(reserved.id));
    }
    index.insert(reserved.id, entity);
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
    index.insert(id, entity);
    world.entity_mut(entity).insert(EntityId(id));
    Ok(id)
}

/// Read-only allocation preflight for an exclusive-world lifecycle commit.
/// This is not a reservation: assignment must follow without another allocator
/// mutation. An uninitialized namespace projects ID one without creating state.
pub fn next_entity_id(world: &World) -> Result<u64, IdentityError> {
    world
        .get_resource::<IdIndex>()
        .map_or(0, |index| index.last)
        .checked_add(1)
        .ok_or(IdentityError::Exhausted)
}

/// Preserve a verified previously issued range while rebuilding a world in ascending
/// public-ID order. Skipped IDs have no owner and allocate no pages. The floor can
/// only move forward and never changes existing identities or reset reservations.
pub fn advance_entity_id_floor(world: &mut World, next: u64) -> Result<(), IdentityError> {
    if next < next_entity_id(world)? {
        return Err(IdentityError::InvalidReservation);
    }
    world.init_resource::<IdIndex>();
    world.resource_mut::<IdIndex>().last = next - 1;
    Ok(())
}

/// Resolves only a live entity that still owns the requested identity.
pub fn entity_by_id(world: &World, id: u64) -> Option<Entity> {
    let entity = world.get_resource::<IdIndex>()?.entity(id)?;
    (world.get::<EntityId>(entity)?.0 == id).then_some(entity)
}

fn forget_id(mut world: DeferredWorld, context: HookContext) {
    let id = world
        .get::<EntityId>(context.entity)
        .expect("removal hook runs before the identity is removed")
        .0;
    if let Some(mut index) = world.get_resource_mut::<IdIndex>() {
        index.forget(id, context.entity);
    }
}

#[cfg(test)]
#[path = "../tests/private/identity.rs"]
mod tests;
