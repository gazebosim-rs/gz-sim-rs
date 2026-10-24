// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use gz_physics_rapier::{BodyId, RapierBackend};
use gz_sim_core::{EntityId, Simulation, assign_id, entity_by_id};
use gz_sim_scene::{BodyHandle, ImuSample, Name, RemoveError, load_world, remove_model};
use sdformat::NoResources;
use std::time::Duration;

fn load(xml: &str) -> Simulation {
    load_world(
        xml,
        "remove.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
    )
    .unwrap()
}
fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world().get::<EntityId>(named(sim, name)).unwrap().get()
}
fn bodies(sim: &Simulation) -> Vec<BodyId> {
    sim.world()
        .iter_entities()
        .filter_map(|e| e.get::<BodyHandle>().map(BodyHandle::id))
        .collect()
}

#[test]
fn removal_retires_model_descendants_bodies_and_samples_without_ghosts() {
    let mut sim = load(include_str!("fixtures/fall.sdf"));
    sim.step_exact(2).unwrap();
    let ball = id(&sim, "default::ball");
    let ground = named(&sim, "default::ground");
    let ground_id = id(&sim, "default::ground");
    let body = sim
        .world()
        .get::<BodyHandle>(named(&sim, "default::ball::body"))
        .unwrap()
        .id();
    let named_ids: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0.starts_with("default::ball"))
        })
        .map(|e| e.get::<EntityId>().unwrap().get())
        .collect();
    let removed = remove_model(&mut sim, ball).unwrap();
    assert_eq!(removed.len(), 4);
    assert!(named_ids.iter().all(|id| removed.contains(id)));
    for id in &removed {
        assert_eq!(entity_by_id(sim.world(), *id), None);
    }
    assert_eq!(entity_by_id(sim.world(), ground_id), Some(ground));
    assert_eq!(sim.world().resource::<RapierBackend>().state(body), None);
    sim.step_exact(2).unwrap();
    assert_eq!(bodies(&sim).len(), 1);
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<ImuSample>())
    );
    assert_eq!(
        remove_model(&mut sim, ball),
        Err(RemoveError::MissingEntity(ball))
    );
    // The existing core reset restores retained domain state, not the initial scene.
    sim.reset().unwrap();
    assert_eq!(entity_by_id(sim.world(), ball), None);
    assert_eq!(sim.world().resource::<RapierBackend>().state(body), None);
    let next = sim.world_mut().spawn_empty().id();
    assert!(assign_id(sim.world_mut(), next).unwrap() > *removed.last().unwrap());
}

#[test]
fn nested_hierarchy_is_removed_recursively_but_is_not_a_direct_remove_target() {
    let mut sim = load(include_str!("fixtures/nested.sdf"));
    let nested = id(&sim, "default::outer::inner");
    assert_eq!(
        remove_model(&mut sim, nested),
        Err(RemoveError::NotWorldModel(nested))
    );
    let body_ids = bodies(&sim);
    let outer = id(&sim, "default::outer");
    let removed = remove_model(&mut sim, outer).unwrap();
    assert_eq!(removed.len(), 10);
    for body in body_ids {
        assert_eq!(sim.world().resource::<RapierBackend>().state(body), None);
    }
    sim.step_exact(1).unwrap();
    assert!(bodies(&sim).is_empty());
    let mut static_empty = load(
        "<sdf version='1.12'><world name='default'><model name='empty'><static>true</static></model></world></sdf>",
    );
    let empty = id(&static_empty, "default::empty");
    assert_eq!(remove_model(&mut static_empty, empty).unwrap(), vec![empty]);
}

#[test]
fn invalid_targets_and_inconsistent_physics_fail_before_scene_mutation() {
    let mut sim = load(include_str!("fixtures/fall.sdf"));
    for name in ["default", "default::ball::body", "default::ball::body::imu"] {
        let id = id(&sim, name);
        assert_eq!(
            remove_model(&mut sim, id),
            Err(RemoveError::NotWorldModel(id))
        );
        assert!(entity_by_id(sim.world(), id).is_some());
    }
    assert_eq!(
        remove_model(&mut sim, u64::MAX),
        Err(RemoveError::MissingEntity(u64::MAX))
    );
    let ball = id(&sim, "default::ball");
    let body = sim
        .world()
        .get::<BodyHandle>(named(&sim, "default::ball::body"))
        .unwrap()
        .id();
    let ground_body = sim
        .world()
        .get::<BodyHandle>(named(&sim, "default::ground::base"))
        .unwrap()
        .id();
    sim.world_mut().resource_mut::<RapierBackend>().remove(body);
    assert_eq!(
        remove_model(&mut sim, ball),
        Err(RemoveError::InvalidPhysics)
    );
    assert!(entity_by_id(sim.world(), ball).is_some());
    assert!(
        sim.world()
            .resource::<RapierBackend>()
            .state(ground_body)
            .is_some()
    );
    sim.world_mut().remove_resource::<RapierBackend>();
    assert_eq!(
        remove_model(&mut sim, ball),
        Err(RemoveError::InvalidPhysics)
    );
    assert!(entity_by_id(sim.world(), ball).is_some());
}

#[test]
fn duplicate_body_bindings_are_rejected_before_any_body_is_removed() {
    use bevy_ecs::hierarchy::ChildOf;
    let mut sim = load(include_str!("fixtures/fall.sdf"));
    let mut other = load(include_str!("fixtures/fall.sdf"));
    let foreign_body = named(&other, "default::ball::body");
    let handle = other
        .world_mut()
        .entity_mut(foreign_body)
        .take::<BodyHandle>()
        .unwrap();
    let ball = id(&sim, "default::ball");
    let parent = entity_by_id(sim.world(), ball).unwrap();
    sim.world_mut().spawn((handle, ChildOf(parent)));
    assert_eq!(
        remove_model(&mut sim, ball),
        Err(RemoveError::InvalidPhysics)
    );
    assert!(entity_by_id(sim.world(), ball).is_some());
    assert!(bodies(&sim).into_iter().all(|body| {
        sim.world()
            .resource::<RapierBackend>()
            .state(body)
            .is_some()
    }));
}

#[test]
fn retained_body_alias_is_rejected_without_removing_either_model() {
    use bevy_ecs::hierarchy::ChildOf;
    let mut sim = load(include_str!("fixtures/fall.sdf"));
    let mut other = load(include_str!("fixtures/fall.sdf"));
    let foreign_ground = named(&other, "default::ground::base");
    let handle = other
        .world_mut()
        .entity_mut(foreign_ground)
        .take::<BodyHandle>()
        .unwrap();
    let ball = id(&sim, "default::ball");
    let parent = entity_by_id(sim.world(), ball).unwrap();
    sim.world_mut().spawn((handle, ChildOf(parent)));
    let before: Vec<_> = bodies(&sim)
        .into_iter()
        .map(|body| (body, sim.world().resource::<RapierBackend>().state(body)))
        .collect();
    let count = sim.world().entities().len();
    assert_eq!(
        remove_model(&mut sim, ball),
        Err(RemoveError::InvalidPhysics)
    );
    assert_eq!(sim.world().entities().len(), count);
    assert!(entity_by_id(sim.world(), ball).is_some());
    for (body, state) in before {
        assert!(state.is_some());
        assert_eq!(sim.world().resource::<RapierBackend>().state(body), state);
    }
}
