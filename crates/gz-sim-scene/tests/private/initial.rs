// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Original ResetTo assertions adapted from EntityComponentManager_TEST.cc.
/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
use super::*;
use gz_sim_core::assign_id;

fn load() -> Simulation {
    load_world(
        include_str!("../fixtures/fall.sdf"),
        "reset.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
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

#[test]
fn removed_initial_models_restore_ids_physics_and_sensors_without_running_normal_systems() {
    #[derive(Resource, Default)]
    struct Counts {
        ordinary: u64,
        resets: u64,
    }
    let mut sim = load();
    sim.world_mut().insert_resource(Counts::default());
    sim.add_systems(Phase::PreStep, |mut counts: ResMut<Counts>| {
        counts.ordinary += 1
    });
    sim.add_systems(Phase::Reset, |mut counts: ResMut<Counts>| {
        counts.resets += 1
    });
    let ball = id(&sim, "default::ball");
    let old_body_entity = named(&sim, "default::ball::body");
    let body_id = id(&sim, "default::ball::body");
    let physics = sim.world().get::<BodyHandle>(old_body_entity).unwrap().id();
    sim.step_exact(3).unwrap();
    remove_model(&mut sim, ball).unwrap();
    assert!(entity_by_id(sim.world(), ball).is_none());
    let value = reset_loaded_world(&mut sim, |sim| {
        let report = sim.world().resource::<SceneResetInfo>();
        assert_eq!(report.previous_time, Duration::from_millis(3));
        assert_eq!(report.epoch, 1);
        assert!(report.restored_ids.contains(&ball));
        assert!(report.removed_ids.is_empty());
        assert_eq!(sim.world().resource::<Counts>().ordinary, 3);
        assert_eq!(sim.world().resource::<Counts>().resets, 1);
        assert_eq!(
            sim.world()
                .get::<WorldPose>(named(sim, "default::ball"))
                .unwrap()
                .translation[2],
            5.0
        );
        assert!(
            !sim.world()
                .iter_entities()
                .any(|e| e.contains::<ImuSample>())
        );
        42
    })
    .unwrap();
    assert_eq!(value, 42);
    let restored = entity_by_id(sim.world(), body_id).unwrap();
    assert_ne!(restored, old_body_entity);
    assert_eq!(
        sim.world().get::<BodyHandle>(restored).unwrap().id(),
        physics
    );
    assert_eq!(
        sim.world()
            .resource::<RapierBackend>()
            .state(physics)
            .unwrap()
            .position[2],
        5.0
    );
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert!(sim.is_paused());
    sim.step_exact(1).unwrap();
    let sample = sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<ImuSample>())
        .unwrap();
    assert_eq!(sample.epoch, 1);
    assert_eq!(sample.time, Duration::from_millis(1));
    assert!(
        sim.world()
            .resource::<RapierBackend>()
            .state(physics)
            .unwrap()
            .position[2]
            < 5.0
    );
    sim.set_paused(false);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(!sim.is_paused());
    assert_eq!(sim.world().resource::<SceneResetInfo>().epoch, 2);
    assert_eq!(sim.world().resource::<Counts>().resets, 2);
}

#[test]
fn runtime_additions_remain_observable_through_reset_then_retire_without_resource_loss() {
    #[derive(Resource)]
    struct Sentinel(u64);
    let mut sim = load();
    sim.world_mut().insert_resource(Sentinel(123));
    let parent = named(&sim, "default::ball");
    let parent_id = id(&sim, "default::ball");
    let first = sim
        .world_mut()
        .spawn((Name("extra".into()), ChildOf(parent)))
        .id();
    let first_id = assign_id(sim.world_mut(), first).unwrap();
    let second = sim
        .world_mut()
        .spawn((Name("child".into()), ChildOf(first)))
        .id();
    let second_id = assign_id(sim.world_mut(), second).unwrap();
    reset_loaded_world(&mut sim, |sim| {
        let report = sim.world().resource::<SceneResetInfo>();
        assert_eq!(report.removed_ids, [first_id, second_id]);
        assert!(sim.world().get::<PendingResetRemoval>(first).is_some());
        assert!(sim.world().get::<PendingResetRemoval>(second).is_some());
        assert_eq!(sim.world().get::<Name>(first).unwrap().0, "extra");
        assert_eq!(
            sim.world().get::<ChildOf>(first).unwrap().parent(),
            entity_by_id(sim.world(), parent_id).unwrap()
        );
        assert_eq!(sim.world().get::<ChildOf>(second).unwrap().parent(), first);
        assert_eq!(sim.world().resource::<Sentinel>().0, 123);
    })
    .unwrap();
    assert!(entity_by_id(sim.world(), first_id).is_none());
    assert!(entity_by_id(sim.world(), second_id).is_none());
    assert_eq!(sim.world().resource::<Sentinel>().0, 123);
    let fresh = sim.world_mut().spawn_empty().id();
    assert!(assign_id(sim.world_mut(), fresh).unwrap() > second_id);
}

#[test]
fn invalid_initial_backend_identity_and_resource_ownership_fail_before_mutation() {
    let mut bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        reset_loaded_world(&mut bare, |_| ()),
        Err(SceneResetError::MissingInitialState)
    ));
    let mut sim = load();
    sim.step_exact(2).unwrap();
    let ball = named(&sim, "default::ball");
    let count = sim.world().entities().len();
    let original = sim.world_mut().remove_resource::<RapierBackend>().unwrap();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::MissingPhysics)
    ));
    sim.world_mut()
        .insert_resource(RapierBackend::new([0.0; 3]).unwrap());
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::Physics(PhysicsError::ForeignInitialState))
    ));
    sim.world_mut().insert_resource(original);
    #[derive(Resource)]
    struct Linked;
    sim.world_mut().insert_resource(Linked);
    let component = sim.world().component_id::<Linked>().unwrap();
    let resource = sim.world().resource_entities().get(component).unwrap();
    sim.world_mut().entity_mut(resource).insert(ChildOf(ball));
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::InfrastructureOwnership)
    ));
    sim.world_mut().entity_mut(resource).remove::<ChildOf>();
    sim.world_mut().remove_resource::<Linked>();
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    assert!(sim.world().get_entity(ball).is_ok());
    assert!(sim.world().entities().len() >= count);
    let mut other = load();
    let initial = sim.world_mut().remove_resource::<InitialScene>().unwrap();
    other.world_mut().insert_resource(initial);
    assert!(matches!(
        reset_loaded_world(&mut other, |_| ()),
        Err(SceneResetError::Identity(IdentityError::ForeignReservation))
    ));
}

#[test]
fn unsupported_runtime_bindings_and_moved_identities_leave_scene_and_clock_unchanged() {
    let mut sim = load();
    sim.step_exact(2).unwrap();
    let ball = id(&sim, "default::ball");
    let body = sim
        .world()
        .get::<BodyHandle>(named(&sim, "default::ball::body"))
        .unwrap()
        .id();
    let before = sim.world().resource::<RapierBackend>().state(body);
    let extra = sim.world_mut().spawn(BodyHandle(body)).id();
    assign_id(sim.world_mut(), extra).unwrap();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::RuntimePhysics)
    ));
    assert_eq!(sim.world().resource::<RapierBackend>().state(body), before);
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    assert!(entity_by_id(sim.world(), ball).is_some());
    sim.world_mut().entity_mut(extra).despawn();
    let mut other = load();
    let foreign = named(&other, "default::ball");
    let identity = other
        .world_mut()
        .entity_mut(foreign)
        .take::<EntityId>()
        .unwrap();
    let extra = sim.world_mut().spawn(identity).id();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::Identity(IdentityError::MovedIdentity))
    ));
    assert_eq!(sim.world().resource::<RapierBackend>().state(body), before);
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    sim.world_mut().entity_mut(extra).despawn();
    let unnamed_id = sim.world_mut().spawn(Name("unidentified".into())).id();
    reset_loaded_world(&mut sim, |sim| {
        assert!(sim.world().get::<PendingResetRemoval>(unnamed_id).is_some());
        assert!(
            sim.world()
                .resource::<SceneResetInfo>()
                .removed_ids
                .is_empty()
        );
    })
    .unwrap();
    assert!(sim.world().get_entity(unnamed_id).is_err());
}

fn two_named_entities() -> (Simulation, u64, u64) {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    gz_physics_rapier::install(&mut sim, RapierBackend::new([0.0; 3]).unwrap()).unwrap();
    sim.world_mut().insert_resource(Gravity([0.0; 3]));
    let pose = Transform {
        translation: [1.0, 2.0, 3.0],
        rotation: DQuat::from_euler(glam::EulerRot::ZYX, 0.3, 0.2, 0.1).to_array(),
    };
    let first = spawn_node(&mut sim, None, "entity1", EntityKind::Model, pose, pose).unwrap();
    let second = spawn_node(
        &mut sim,
        None,
        "entity2",
        EntityKind::Model,
        Transform::default(),
        Transform::default(),
    )
    .unwrap();
    let first = sim.world().get::<EntityId>(first).unwrap().get();
    let second = sim.world().get::<EntityId>(second).unwrap().get();
    capture(&mut sim);
    (sim, first, second)
}

#[test]
fn native_reset_to_deleted_entity_retains_the_two_new_name_observations() {
    for _ in 1..10 {
        let (mut sim, first, second) = two_named_entities();
        let mut names = sim.world_mut().query_filtered::<Entity, Added<Name>>();
        assert_eq!(names.iter(sim.world()).count(), 2);
        let removed = entity_by_id(sim.world(), first).unwrap();
        sim.world_mut().entity_mut(removed).despawn();
        sim.world_mut().clear_trackers();
        assert_eq!(names.iter(sim.world()).count(), 0);
        reset_loaded_world(&mut sim, |sim| {
            let new: Vec<_> = names.iter(sim.world()).collect();
            assert_eq!(new.len(), 2);
            assert!(new.contains(&entity_by_id(sim.world(), first).unwrap()));
            assert!(new.contains(&entity_by_id(sim.world(), second).unwrap()));
            assert_eq!(
                sim.world()
                    .get::<WorldPose>(entity_by_id(sim.world(), first).unwrap())
                    .unwrap()
                    .translation,
                [1.0, 2.0, 3.0]
            );
        })
        .unwrap();
    }
}

#[test]
fn native_reset_to_added_entity_retains_the_one_removed_name_observation() {
    for _ in 1..10 {
        let (mut sim, first, second) = two_named_entities();
        let extra = sim.world_mut().spawn(Name("entity3".into())).id();
        let third = assign_id(sim.world_mut(), extra).unwrap();
        let mut names = sim.world_mut().query_filtered::<Entity, Added<Name>>();
        sim.world_mut().clear_trackers();
        assert_eq!(names.iter(sim.world()).count(), 0);
        reset_loaded_world(&mut sim, |sim| {
            assert!(names.iter(sim.world()).count() >= 2);
            let removed: Vec<_> = sim
                .world()
                .iter_entities()
                .filter(|e| e.contains::<PendingResetRemoval>() && e.contains::<Name>())
                .collect();
            assert_eq!(removed.len(), 1);
            assert_eq!(removed[0].id(), extra);
            assert_eq!(removed[0].get::<Name>().unwrap().0, "entity3");
            assert_eq!(
                sim.world().resource::<SceneResetInfo>().removed_ids,
                [third]
            );
        })
        .unwrap();
        assert!(entity_by_id(sim.world(), first).is_some());
        assert!(entity_by_id(sim.world(), second).is_some());
        assert!(entity_by_id(sim.world(), third).is_none());
    }
}

#[test]
fn nested_canonical_and_attached_frames_rebuild_with_original_parent_identities() {
    let mut sim = load_world(
        include_str!("../fixtures/nested.sdf"),
        "nested.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let expected: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|e| e.contains::<EntityKind>())
        .map(|e| {
            (
                e.get::<EntityId>().unwrap().get(),
                e.get::<ChildOf>()
                    .map(|p| sim.world().get::<EntityId>(p.parent()).unwrap().get()),
                *e.get::<WorldPose>().unwrap(),
                *e.get::<LocalPose>().unwrap(),
            )
        })
        .collect();
    sim.step_exact(4).unwrap();
    let outer = id(&sim, "default::outer");
    remove_model(&mut sim, outer).unwrap();
    reset_loaded_world(&mut sim, |sim| {
        for (id, parent, world_pose, local_pose) in &expected {
            let entity = entity_by_id(sim.world(), *id).unwrap();
            assert_eq!(
                sim.world().get::<ChildOf>(entity).map(|p| sim
                    .world()
                    .get::<EntityId>(p.parent())
                    .unwrap()
                    .get()),
                *parent
            );
            let world = sim.world().get::<WorldPose>(entity).unwrap();
            let local = sim.world().get::<LocalPose>(entity).unwrap();
            for (actual, expected) in world
                .translation
                .into_iter()
                .chain(world.rotation)
                .chain(local.0.translation)
                .chain(local.0.rotation)
                .zip(
                    world_pose
                        .translation
                        .into_iter()
                        .chain(world_pose.rotation)
                        .chain(local_pose.0.translation)
                        .chain(local_pose.0.rotation),
                )
            {
                assert!((actual - expected).abs() < 1e-12);
            }
        }
    })
    .unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|e| e.contains::<EntityKind>())
            .count(),
        expected.len()
    );
}

#[test]
fn identity_moved_to_a_resource_is_rejected_before_any_reset_mutation() {
    #[derive(Resource)]
    struct Holder;
    let mut sim = load();
    sim.step_exact(2).unwrap();
    let original = named(&sim, "default");
    let moved = sim
        .world_mut()
        .entity_mut(original)
        .take::<EntityId>()
        .unwrap();
    let public_id = moved.get();
    sim.world_mut().insert_resource(Holder);
    let component = sim.world().component_id::<Holder>().unwrap();
    let resource = sim.world().resource_entities().get(component).unwrap();
    sim.world_mut().entity_mut(resource).insert(moved);
    let body = sim
        .world()
        .get::<BodyHandle>(named(&sim, "default::ball::body"))
        .unwrap()
        .id();
    let state = sim.world().resource::<RapierBackend>().state(body);
    let count = sim.world().entities().len();
    let info = *sim.world().resource::<UpdateInfo>();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::Identity(IdentityError::MovedIdentity))
    ));
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    assert_eq!(*sim.world().resource::<UpdateInfo>(), info);
    assert_eq!(sim.world().entities().len(), count);
    assert_eq!(sim.world().resource::<RapierBackend>().state(body), state);
    assert!(sim.world().get::<EntityId>(original).is_none());
    assert_eq!(
        sim.world().get::<EntityId>(resource).unwrap().get(),
        public_id
    );
    assert!(entity_by_id(sim.world(), public_id).is_none());
    assert!(!sim.world().contains_resource::<SceneResetInfo>());
}

#[test]
fn registered_systems_and_global_observers_survive_scene_reconstruction() {
    #[derive(Event)]
    struct Ping;
    #[derive(Resource, Default)]
    struct Count(u64);
    let mut sim = load();
    let system = sim.world_mut().register_system(|| 42_u32);
    assert_eq!(sim.world_mut().run_system(system).unwrap(), 42);
    sim.world_mut().init_resource::<Count>();
    let observer = sim
        .world_mut()
        .add_observer(|_: On<Ping>, mut count: ResMut<Count>| count.0 += 1)
        .id();
    sim.world_mut().trigger(Ping);
    assert_eq!(sim.world().resource::<Count>().0, 1);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.world_mut().run_system(system).unwrap(), 42);
    assert!(sim.world().get_entity(observer).is_ok());
    sim.world_mut().trigger(Ping);
    assert_eq!(sim.world().resource::<Count>().0, 2);
    let parent = named(&sim, "default::ball");
    sim.world_mut().entity_mut(observer).insert(ChildOf(parent));
    let info = *sim.world().resource::<UpdateInfo>();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::InfrastructureOwnership)
    ));
    assert_eq!(*sim.world().resource::<UpdateInfo>(), info);
    assert_eq!(sim.world_mut().run_system(system).unwrap(), 42);
}

#[test]
fn mixed_initial_node_and_observer_ownership_fails_before_reconstruction() {
    #[derive(Event)]
    struct Ping;
    let mut sim = load();
    sim.step_exact(2).unwrap();
    let world = named(&sim, "default");
    let world_id = id(&sim, "default");
    sim.world_mut()
        .entity_mut(world)
        .insert(Observer::new(|_: On<Ping>| {}));
    let count = sim.world().entities().len();
    let info = *sim.world().resource::<UpdateInfo>();
    let body = sim
        .world()
        .get::<BodyHandle>(named(&sim, "default::ball::body"))
        .unwrap()
        .id();
    let state = sim.world().resource::<RapierBackend>().state(body);
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::InfrastructureOwnership)
    ));
    assert_eq!(sim.world().entities().len(), count);
    assert_eq!(*sim.world().resource::<UpdateInfo>(), info);
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    assert_eq!(entity_by_id(sim.world(), world_id), Some(world));
    assert!(sim.world().get::<Observer>(world).is_some());
    assert_eq!(sim.world().resource::<RapierBackend>().state(body), state);
}
