// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_assets::LocalResources;
use gz_sim_core::EntityId;
use gz_sim_scene::{Name, PhysicsEngine, WorldPose, load_world_with_engine, reset_loaded_world};
use std::{fs, time::Duration};
use tempfile::TempDir;

#[test]
fn configured_local_include_constructs_both_engines_and_preserves_loaded_reset() {
    let temp = TempDir::new().unwrap();
    let directory = temp.path().join("models/robot");
    fs::create_dir_all(&directory).unwrap();
    fs::write(
        directory.join("model.config"),
        "<model><sdf version='1.12'>body.sdf</sdf></model>",
    )
    .unwrap();
    fs::write(directory.join("body.sdf"),"<sdf version='1.12'><model name='included'><link name='body'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model></sdf>").unwrap();
    let xml = "<sdf version='1.12'><world name='default'><include><uri>model://robot</uri><name>robot</name><pose>1 2 3 0 0 0</pose></include></world></sdf>";
    let model_xml = fs::read_to_string(directory.join("body.sdf")).unwrap();
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        fs::write(directory.join("body.sdf"), &model_xml).unwrap();
        let resolver =
            LocalResources::new(temp.path().into(), vec![temp.path().join("models")], 4096)
                .unwrap();
        let mut sim = load_world_with_engine(
            xml,
            "world.sdf",
            "default",
            Duration::from_millis(1),
            &resolver,
            engine,
        )
        .unwrap();
        let link = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::robot::body")
            })
            .unwrap()
            .id();
        assert_eq!(
            sim.world().get::<WorldPose>(link).unwrap().translation,
            [1.0, 2.0, 3.0]
        );
        assert!(resolver.take_warnings().unwrap().is_empty());
        drop(resolver);
        fs::remove_file(directory.join("body.sdf")).unwrap();
        sim.step_exact(2).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        let restored = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::robot::body")
            })
            .unwrap()
            .id();
        assert_eq!(
            sim.world().get::<WorldPose>(restored).unwrap().translation,
            [1.0, 2.0, 3.0]
        );
    }
}

#[test]
fn offline_fuel_include_survives_cache_removal_and_loaded_reset_for_both_engines() {
    let temp = TempDir::new().unwrap();
    let cache = temp.path().join("cache");
    let directory = cache.join("fuel.gazebosim.org/alice/models/robot/2");
    let model = "<sdf version='1.12'><model name='included'><link name='body'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model></sdf>";
    let xml = "<sdf version='1.12'><world name='default'><include><uri>https://fuel.gazebosim.org/1.0/alice/models/robot/2</uri><name>robot</name><pose>1 2 3 0 0 0</pose></include></world></sdf>";
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        fs::create_dir_all(&directory).unwrap();
        fs::write(
            directory.join("model.config"),
            "<model><sdf version='1.12'>model.sdf</sdf></model>",
        )
        .unwrap();
        fs::write(directory.join("model.sdf"), model).unwrap();
        let client = gz_fuel_tools::FuelClient::new(gz_fuel_tools::ClientConfig {
            cache_location: cache.clone(),
            ..Default::default()
        });
        let resolver = LocalResources::new(temp.path().into(), vec![], 4096)
            .unwrap()
            .with_cached_fuel(client);
        let mut sim = load_world_with_engine(
            xml,
            "world.sdf",
            "default",
            Duration::from_millis(1),
            &resolver,
            engine,
        )
        .unwrap();
        let link = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::robot::body")
            })
            .unwrap()
            .id();
        assert_eq!(
            sim.world().get::<WorldPose>(link).unwrap().translation,
            [1.0, 2.0, 3.0]
        );
        assert!(resolver.take_warnings().unwrap().is_empty());
        let logical_id = sim.world().get::<EntityId>(link).unwrap().get();
        drop(resolver);
        fs::remove_dir_all(&cache).unwrap();
        sim.step_exact(2).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        let restored = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::robot::body")
            })
            .unwrap()
            .id();
        assert_eq!(
            sim.world().get::<EntityId>(restored).unwrap().get(),
            logical_id
        );
        assert_eq!(
            sim.world().get::<WorldPose>(restored).unwrap().translation,
            [1.0, 2.0, 3.0]
        );
    }
}
