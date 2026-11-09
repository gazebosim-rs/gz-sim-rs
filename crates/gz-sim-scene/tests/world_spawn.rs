// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Startup-prepared primitive clone controls; no qualification until M10 is complete.
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, entity_by_id, next_entity_id};
use gz_sim_scene::{
    BodyHandle, ModelState, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step, model_state,
    prepare_model_template, remove_model, reset_loaded_world, set_model_state,
    spawn_model_template,
};
use sdformat::NoResources;

const XML: &str = "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><model name='box'><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='c'><geometry><box><size>1 2 3</size></box></geometry></collision><visual name='v'><geometry><box><size>1 2 3</size></box></geometry></visual></link></model><model name='sphere'><static>true</static><link name='body'><collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><visual name='v'><geometry><sphere><radius>0.5</radius></sphere></geometry></visual></link></model></world></sdf>";

#[test]
fn runtime_clones_keep_existing_native_state_and_ids_and_loaded_reset_retires_additions() {
    let parsed = sdformat::load_str(XML, "spawn.sdf", &NoResources).unwrap();
    let mut sim =
        load_world_with_sdf_step(XML, "spawn.sdf", "w", &NoResources, PhysicsEngine::Rapier)
            .unwrap();
    let templates: Vec<_> = parsed.worlds[0]
        .models
        .iter()
        .map(|model| prepare_model_template(&sim, &parsed, model).unwrap())
        .collect();
    let original: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| {
            entity.get::<BodyHandle>().map(|handle| {
                (
                    entity.get::<EntityId>().unwrap().get(),
                    entity.id(),
                    handle.id(),
                )
            })
        })
        .collect();
    sim.step_exact(3).unwrap();
    let before: Vec<_> = original
        .iter()
        .map(|(_, _, body)| {
            sim.world()
                .resource::<PhysicsBackend>()
                .state(*body)
                .unwrap()
        })
        .collect();
    let clock = (sim.epoch(), sim.elapsed());
    let mut spawned = Vec::new();
    for (index, template) in templates.iter().enumerate() {
        let first = next_entity_id(sim.world()).unwrap();
        let pose = WorldPose {
            translation: [10.0 + index as f64 * 5.0, 0.0, 2.0],
            rotation: [0.0, 0.0, 1.0, 0.0],
        };
        let id = spawn_model_template(&mut sim, template, pose).unwrap();
        assert_eq!(id, first);
        assert_eq!(model_state(&sim, id).unwrap().translation, pose.translation);
        for offset in 0..4 {
            assert!(entity_by_id(sim.world(), id + offset).is_some());
        }
        assert_eq!(next_entity_id(sim.world()).unwrap(), first + 4);
        spawned.push(id);
    }
    assert_eq!((sim.epoch(), sim.elapsed()), clock);
    for ((id, entity, body), before) in original.iter().zip(before) {
        assert_eq!(entity_by_id(sim.world(), *id), Some(*entity));
        assert_eq!(
            sim.world()
                .resource::<PhysicsBackend>()
                .state(*body)
                .unwrap(),
            before
        );
    }
    let mut state: ModelState = model_state(&sim, spawned[0]).unwrap();
    state.translation = [20.0, 1.0, 2.0];
    state.linear_velocity = [0.5, 0.0, 0.0];
    set_model_state(&mut sim, spawned[0], state).unwrap();
    assert_eq!(model_state(&sim, spawned[0]).unwrap(), state);
    sim.reset().unwrap();
    assert_eq!(
        model_state(&sim, spawned[0]).unwrap().translation,
        [10.0, 0.0, 2.0]
    );
    assert_eq!(
        model_state(&sim, spawned[0]).unwrap().linear_velocity,
        [0.0; 3]
    );
    assert_eq!(
        model_state(&sim, spawned[1]).unwrap().translation,
        [15.0, 0.0, 2.0]
    );
    let retired = remove_model(&mut sim, spawned[0]).unwrap();
    assert_eq!(retired, (spawned[0]..spawned[0] + 4).collect::<Vec<_>>());
    assert!(model_state(&sim, spawned[0]).is_err());
    let high_water = next_entity_id(sim.world()).unwrap();
    reset_loaded_world(&mut sim, |restored| {
        for offset in 0..4 {
            let entity = entity_by_id(restored.world(), spawned[1] + offset).unwrap();
            assert!(
                restored
                    .world()
                    .get::<gz_sim_scene::PendingResetRemoval>(entity)
                    .is_some()
            );
            assert!(restored.world().get::<BodyHandle>(entity).is_none());
        }
    })
    .unwrap();
    assert!(model_state(&sim, spawned[1]).is_err());
    assert_eq!(next_entity_id(sim.world()).unwrap(), high_water);
    for (id, _, _) in original {
        assert!(entity_by_id(sim.world(), id).is_some());
    }
}

#[test]
fn invalid_spawn_preserves_allocator_native_bodies_and_scene() {
    let parsed = sdformat::load_str(XML, "spawn.sdf", &NoResources).unwrap();
    let mut sim =
        load_world_with_sdf_step(XML, "spawn.sdf", "w", &NoResources, PhysicsEngine::Rapier)
            .unwrap();
    let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
    let first = next_entity_id(sim.world()).unwrap();
    let names: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| {
            entity
                .get::<Name>()
                .map(|name| (entity.id(), name.0.clone()))
        })
        .collect();
    for pose in [
        WorldPose {
            translation: [f64::NAN, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        },
        WorldPose {
            translation: [0.0; 3],
            rotation: [0.0; 4],
        },
        WorldPose {
            translation: [f64::MAX, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        },
    ] {
        assert!(spawn_model_template(&mut sim, &template, pose).is_err());
        assert_eq!(next_entity_id(sim.world()).unwrap(), first);
        for (entity, name) in &names {
            assert_eq!(&sim.world().get::<Name>(*entity).unwrap().0, name);
        }
        assert_eq!(
            sim.world()
                .iter_entities()
                .filter(|entity| entity.contains::<BodyHandle>())
                .count(),
            2
        );
    }
    let other =
        load_world_with_sdf_step(XML, "other.sdf", "w", &NoResources, PhysicsEngine::Tpe).unwrap();
    assert!(prepare_model_template(&other, &parsed, &parsed.worlds[0].models[0]).is_err());
    let mut foreign =
        load_world_with_sdf_step(XML, "foreign.sdf", "w", &NoResources, PhysicsEngine::Rapier)
            .unwrap();
    assert!(
        spawn_model_template(
            &mut foreign,
            &template,
            WorldPose {
                translation: [5.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0]
            }
        )
        .is_err()
    );
    assert_eq!(next_entity_id(foreign.world()).unwrap(), first);
}

#[test]
fn incomplete_runtime_physics_refuses_loaded_reset_before_any_reconstruction() {
    let parsed = sdformat::load_str(XML, "spawn.sdf", &NoResources).unwrap();
    let mut sim =
        load_world_with_sdf_step(XML, "spawn.sdf", "w", &NoResources, PhysicsEngine::Rapier)
            .unwrap();
    let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
    let model = spawn_model_template(
        &mut sim,
        &template,
        WorldPose {
            translation: [5.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
        },
    )
    .unwrap();
    let link = entity_by_id(sim.world(), model + 1).unwrap();
    let native = sim.world().get::<BodyHandle>(link).unwrap().id();
    let before = sim
        .world()
        .resource::<PhysicsBackend>()
        .state(native)
        .unwrap();
    let ids: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<EntityId>().map(|id| (id.get(), entity.id())))
        .collect();
    let clock = (sim.epoch(), sim.elapsed());
    sim.world_mut().entity_mut(link).remove::<BodyHandle>();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(gz_sim_scene::SceneResetError::RuntimePhysics)
    ));
    assert_eq!((sim.epoch(), sim.elapsed()), clock);
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(native)
            .unwrap(),
        before
    );
    for (id, entity) in ids {
        assert_eq!(entity_by_id(sim.world(), id), Some(entity));
    }
}

#[test]
fn rotated_runtime_model_remains_editable_and_resettable_after_real_stepping() {
    let parsed = sdformat::load_str(XML, "spawn.sdf", &NoResources).unwrap();
    let mut sim =
        load_world_with_sdf_step(XML, "spawn.sdf", "w", &NoResources, PhysicsEngine::Rapier)
            .unwrap();
    let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
    let q = std::f64::consts::FRAC_1_SQRT_2;
    let pose = WorldPose {
        translation: [12.0, 3.0, 5.0],
        rotation: [0.0, 0.0, q, q],
    };
    let model = spawn_model_template(&mut sim, &template, pose).unwrap();
    sim.step_exact(3).unwrap();
    let mut state = model_state(&sim, model).unwrap();
    for (actual, expected) in state.translation.into_iter().zip(pose.translation) {
        assert!((actual - expected).abs() < 1e-12);
    }
    state.translation[0] += 1.0;
    let actual = set_model_state(&mut sim, model, state).unwrap();
    assert_eq!(actual.translation, state.translation);
    sim.step_exact(2).unwrap();
    assert!(model_state(&sim, model).is_ok());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(entity_by_id(sim.world(), model).is_none());
}

fn logical_camera_spawn_xml() -> String {
    XML.replace(
        "<model name='sphere'><static>true</static><link name='body'>",
        "<model name='sphere'><static>true</static><link name='body'><sensor name='logical' type='logical_camera'><update_rate>0</update_rate><logical_camera><near>0.1</near><far>1000</far><horizontal_fov>1.5</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor>",
    )
}

#[test]
fn logical_camera_runtime_model_bound_refuses_before_mutation_and_recovers_after_removal() {
    let xml = logical_camera_spawn_xml();
    let parsed = sdformat::load_str(&xml, "logical-spawn.sdf", &NoResources).unwrap();
    let mut sim = load_world_with_sdf_step(
        &xml,
        "logical-spawn.sdf",
        "w",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
    let pose = WorldPose {
        translation: [600.0, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };
    let mut last = 0;
    // Two loaded models plus 126 additions reach the admitted final bound.
    for index in 0..126 {
        let spaced = WorldPose {
            translation: [10.0 + 4.0 * index as f64, 0.0, 0.0],
            ..pose
        };
        last = spawn_model_template(&mut sim, &template, spaced).unwrap();
    }
    let ids: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(|id| (id.get(), e.id())))
        .collect();
    let states: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|e| {
            e.get::<BodyHandle>().map(|body| {
                (
                    body.id(),
                    sim.world()
                        .resource::<PhysicsBackend>()
                        .state(body.id())
                        .unwrap(),
                )
            })
        })
        .collect();
    let first = next_entity_id(sim.world()).unwrap();
    let clock = (sim.epoch(), sim.elapsed());
    assert!(matches!(
        spawn_model_template(&mut sim, &template, pose),
        Err(gz_sim_scene::WorldEditError::UnsupportedModel)
    ));
    assert_eq!(next_entity_id(sim.world()).unwrap(), first);
    assert_eq!((sim.epoch(), sim.elapsed()), clock);
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|e| e.contains::<EntityId>())
            .count(),
        ids.len()
    );
    for (id, entity) in ids {
        assert_eq!(entity_by_id(sim.world(), id), Some(entity));
    }
    for (body, state) in states {
        assert_eq!(
            sim.world().resource::<PhysicsBackend>().state(body),
            Some(state)
        );
    }
    remove_model(&mut sim, last).unwrap();
    assert_eq!(
        spawn_model_template(&mut sim, &template, pose).unwrap(),
        first
    );
    // A due acquisition still admits all 128 model origins after the edit.
    sim.step_exact(1).unwrap();
    let sample = sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<gz_sim_scene::LogicalCameraSample>())
        .unwrap();
    assert!(sample.reading.is_ok());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(
                |e| e.get::<gz_sim_scene::EntityKind>() == Some(&gz_sim_scene::EntityKind::Model)
            )
            .count(),
        2
    );
}

#[test]
fn logical_camera_spawn_rejects_corrupt_model_names_and_hierarchy_atomically() {
    let xml = logical_camera_spawn_xml();
    let parsed = sdformat::load_str(&xml, "logical-spawn.sdf", &NoResources).unwrap();
    let mut sim = load_world_with_sdf_step(
        &xml,
        "logical-spawn.sdf",
        "w",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
    let models: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|e| e.get::<gz_sim_scene::EntityKind>() == Some(&gz_sim_scene::EntityKind::Model))
        .map(|e| (e.id(), e.get::<Name>().unwrap().0.clone()))
        .collect();
    let first = next_entity_id(sim.world()).unwrap();
    let pose = WorldPose {
        translation: [5.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };
    let bodies = sim
        .world()
        .iter_entities()
        .filter(|e| e.contains::<BodyHandle>())
        .count();
    for name in [
        models[1].1.clone(),
        "w::nested::name".into(),
        "w::bad name".into(),
        "elsewhere::model".into(),
    ] {
        sim.world_mut().get_mut::<Name>(models[0].0).unwrap().0 = name;
        assert!(matches!(
            spawn_model_template(&mut sim, &template, pose),
            Err(gz_sim_scene::WorldEditError::InvalidScene)
        ));
        assert_eq!(next_entity_id(sim.world()).unwrap(), first);
        assert_eq!(
            sim.world()
                .iter_entities()
                .filter(|e| e.contains::<BodyHandle>())
                .count(),
            bodies
        );
    }
    sim.world_mut().get_mut::<Name>(models[0].0).unwrap().0 = models[0].1.clone();
    sim.world_mut()
        .entity_mut(models[0].0)
        .insert(bevy_ecs::hierarchy::ChildOf(models[1].0));
    assert!(matches!(
        spawn_model_template(&mut sim, &template, pose),
        Err(gz_sim_scene::WorldEditError::InvalidScene)
    ));
    assert_eq!(next_entity_id(sim.world()).unwrap(), first);
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|e| e.contains::<BodyHandle>())
            .count(),
        bodies
    );
}

#[test]
fn runtime_spawning_without_logical_camera_retains_its_existing_model_domain() {
    let parsed = sdformat::load_str(XML, "ordinary-spawn.sdf", &NoResources).unwrap();
    let mut sim = load_world_with_sdf_step(
        XML,
        "ordinary-spawn.sdf",
        "w",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
    let pose = WorldPose {
        translation: [10.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };
    for _ in 0..127 {
        spawn_model_template(&mut sim, &template, pose).unwrap();
    }
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(
                |e| e.get::<gz_sim_scene::EntityKind>() == Some(&gz_sim_scene::EntityKind::Model)
            )
            .count(),
        129
    );
}
