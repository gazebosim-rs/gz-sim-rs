// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::Simulation;
use std::time::Duration;

#[test]
fn identities_survive_reset_and_bevy_recycling_never_reuses_public_ids() {
    let mut simulation = Simulation::new(Duration::from_millis(1)).unwrap();
    assert_eq!(entity_by_id(simulation.world(), 0), None);
    let first = simulation.world_mut().spawn_empty().id();
    let id = assign_id(simulation.world_mut(), first).unwrap();
    assert_eq!(id, 1);
    assert_eq!(assign_id(simulation.world_mut(), first), Ok(id));
    assert_eq!(simulation.world().get::<EntityId>(first).unwrap().get(), id);
    simulation.step_exact(2).unwrap();
    simulation.reset().unwrap();
    assert_eq!(entity_by_id(simulation.world(), id), Some(first));
    simulation.world_mut().despawn(first);
    assert_eq!(entity_by_id(simulation.world(), id), None);
    assert!(simulation.world().resource::<IdIndex>().entities.is_empty());
    let next = simulation.world_mut().spawn_empty().id();
    assert_ne!(first, next);
    assert_eq!(assign_id(simulation.world_mut(), next), Ok(2));
    assert_eq!(
        assign_id(simulation.world_mut(), first),
        Err(IdentityError::MissingEntity)
    );
    assert_eq!(entity_by_id(simulation.world(), id), None);
    simulation.world_mut().entity_mut(next).remove::<EntityId>();
    assert_eq!(entity_by_id(simulation.world(), 2), None);
    assert_eq!(assign_id(simulation.world_mut(), next), Ok(3));
    let mut other = World::new();
    let unrelated = other.spawn_empty().id();
    assert_eq!(assign_id(&mut other, unrelated), Ok(1));
    assert_eq!(entity_by_id(&other, 2), None);
}

#[test]
fn moving_identity_components_cannot_redirect_lookups_or_reuse_ids() {
    let mut first = World::new();
    let a = first.spawn_empty().id();
    let b = first.spawn_empty().id();
    assign_id(&mut first, a).unwrap();
    let moved = first.entity_mut(a).take::<EntityId>().unwrap();
    first.entity_mut(b).insert(moved);
    assert_eq!(entity_by_id(&first, 1), None);
    assert_eq!(assign_id(&mut first, b), Err(IdentityError::MovedIdentity));
    first.despawn(b);
    assert_eq!(assign_id(&mut first, a), Ok(2));

    // A different world's equal numeric ID must retain its own mapping.
    let mut other = World::new();
    let original = other.spawn_empty().id();
    assign_id(&mut other, original).unwrap();
    let second = other.spawn_empty().id();
    assert_eq!(assign_id(&mut other, second), Ok(2));
    let moved = first.entity_mut(a).take::<EntityId>().unwrap();
    let foreign = other.spawn(moved).id();
    assert_eq!(
        assign_id(&mut other, foreign),
        Err(IdentityError::MovedIdentity)
    );
    assert_eq!(entity_by_id(&other, 1), Some(original));
    other.despawn(foreign);
    assert_eq!(entity_by_id(&other, 1), Some(original));
    assert_eq!(entity_by_id(&other, 2), Some(second));

    let detached = other.entity_mut(original).take::<EntityId>().unwrap();
    let mut empty = World::new();
    let orphan = empty.spawn(detached).id();
    assert_eq!(
        assign_id(&mut empty, orphan),
        Err(IdentityError::MovedIdentity)
    );
    empty.despawn(orphan);
}

#[test]
fn exhaustion_is_checked_before_assignment_and_removed_ids_stay_retired() {
    let mut world = World::new();
    world.insert_resource(IdIndex {
        last: u64::MAX - 1,
        ..Default::default()
    });
    let last = world.spawn_empty().id();
    assert_eq!(assign_id(&mut world, last), Ok(u64::MAX));
    world.despawn(last);
    let rejected = world.spawn_empty().id();
    assert_eq!(
        assign_id(&mut world, rejected),
        Err(IdentityError::Exhausted)
    );
    assert!(world.get::<EntityId>(rejected).is_none());
    assert!(world.resource::<IdIndex>().entities.is_empty());
}
