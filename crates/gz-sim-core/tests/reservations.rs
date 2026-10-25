// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::*;
use gz_sim_core::{EntityId, IdentityError, assign_id, entity_by_id, reserve_id, restore_id};

#[test]
fn restoration_revives_only_the_reserved_logical_id_and_keeps_allocation_monotonic() {
    let mut world = World::new();
    let original = world.spawn_empty().id();
    let id = assign_id(&mut world, original).unwrap();
    let reserved = reserve_id(&world, original).unwrap();
    assert_eq!(reserved.get(), id);
    assert_eq!(restore_id(&mut world, original, &reserved).unwrap(), id);
    world.despawn(original);
    assert!(entity_by_id(&world, id).is_none());
    let runtime = world.spawn_empty().id();
    let runtime_id = assign_id(&mut world, runtime).unwrap();
    assert!(runtime_id > id);
    let restored = world.spawn_empty().id();
    assert_ne!(restored, original);
    assert_eq!(restore_id(&mut world, restored, &reserved).unwrap(), id);
    assert_eq!(assign_id(&mut world, restored).unwrap(), id);
    assert_eq!(entity_by_id(&world, id), Some(restored));
    world.despawn(restored);
    let again = world.spawn_empty().id();
    assert_eq!(
        restore_id(&mut world, again, &reserved.clone()).unwrap(),
        id
    );
    let fresh = world.spawn_empty().id();
    assert!(assign_id(&mut world, fresh).unwrap() > runtime_id);
    assert_eq!(entity_by_id(&world, runtime_id), Some(runtime));
}

#[test]
fn invalid_targets_and_occupied_identities_fail_without_rebinding() {
    let mut world = World::new();
    let original = world.spawn_empty().id();
    assert!(matches!(
        reserve_id(&world, original),
        Err(IdentityError::MissingIdentity)
    ));
    let id = assign_id(&mut world, original).unwrap();
    let reserved = reserve_id(&world, original).unwrap();
    let target = world.spawn_empty().id();
    assert_eq!(
        restore_id(&mut world, target, &reserved),
        Err(IdentityError::IdentityInUse(id))
    );
    assert!(world.get::<EntityId>(target).is_none());
    let target_id = assign_id(&mut world, target).unwrap();
    assert_eq!(
        restore_id(&mut world, target, &reserved),
        Err(IdentityError::AlreadyIdentified)
    );
    assert_eq!(entity_by_id(&world, id), Some(original));
    assert_eq!(entity_by_id(&world, target_id), Some(target));
    world.despawn(target);
    assert!(matches!(
        reserve_id(&world, target),
        Err(IdentityError::MissingEntity)
    ));
    assert_eq!(
        restore_id(&mut world, target, &reserved),
        Err(IdentityError::MissingEntity)
    );
}

#[test]
fn foreign_worlds_and_moved_identity_components_cannot_claim_a_reservation() {
    let mut world = World::new();
    let original = world.spawn_empty().id();
    let id = assign_id(&mut world, original).unwrap();
    let reserved = reserve_id(&world, original).unwrap();
    let mut other = World::new();
    let target = other.spawn_empty().id();
    assert_eq!(
        reserved.validate(&other),
        Err(IdentityError::ForeignReservation)
    );
    assert_eq!(
        restore_id(&mut other, target, &reserved),
        Err(IdentityError::ForeignReservation)
    );
    assert!(other.get::<EntityId>(target).is_none());
    let moved = world.entity_mut(original).take::<EntityId>().unwrap();
    other.entity_mut(target).insert(moved);
    assert!(matches!(
        reserve_id(&other, target),
        Err(IdentityError::MovedIdentity)
    ));
    assert_eq!(restore_id(&mut world, original, &reserved).unwrap(), id);
    assert_eq!(entity_by_id(&world, id), Some(original));
    assert!(entity_by_id(&other, id).is_none());
    other.despawn(target);
    assert_eq!(entity_by_id(&world, id), Some(original));
}

#[test]
fn clearing_allocator_resources_invalidates_old_reservations() {
    let mut world = World::new();
    let entity = world.spawn_empty().id();
    assign_id(&mut world, entity).unwrap();
    let reserved = reserve_id(&world, entity).unwrap();
    world.despawn(entity);
    world.clear_resources();
    let target = world.spawn_empty().id();
    assert_eq!(
        reserved.validate(&world),
        Err(IdentityError::InvalidReservation)
    );
    assert_eq!(
        restore_id(&mut world, target, &reserved),
        Err(IdentityError::InvalidReservation)
    );
    assert!(world.get::<EntityId>(target).is_none());
    assign_id(&mut world, target).unwrap();
    assert_eq!(
        reserved.validate(&world),
        Err(IdentityError::InvalidReservation)
    );
}
