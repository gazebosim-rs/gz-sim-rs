// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_assets::LocalResources;
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
