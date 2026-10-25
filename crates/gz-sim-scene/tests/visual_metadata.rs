// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::EntityId;
use gz_sim_scene::{Name, VisualProperties, load_world, remove_model, reset_loaded_world};
use sdformat::NoResources;
use std::time::Duration;

#[test]
fn visual_metadata_remains_owned_through_model_removal_and_loaded_reset() {
    let xml=include_str!("fixtures/fall.sdf").replace("<link name=\"body\">","<link name=\"body\"><visual name='view'><cast_shadows>false</cast_shadows><transparency>0.25</transparency><visibility_flags>42</visibility_flags><laser_retro>0</laser_retro><geometry><sphere><radius>0.5</radius></sphere></geometry></visual>");
    let mut sim = load_world(
        &xml,
        "metadata.sdf",
        "default",
        Duration::from_millis(1),
        &NoResources,
    )
    .unwrap();
    let visual = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::ball::body::view")
        })
        .unwrap()
        .id();
    let initial = sim
        .world()
        .get::<VisualProperties>(visual)
        .unwrap()
        .0
        .clone();
    assert!(!initial.cast_shadows);
    assert_eq!(initial.transparency, 0.25_f32);
    assert_eq!(initial.visibility_flags, 42);
    assert_eq!(initial.laser_retro, Some(0.0));
    let model = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::ball"))
        .unwrap()
        .id();
    let model_id = sim.world().get::<EntityId>(model).unwrap().get();
    remove_model(&mut sim, model_id).unwrap();
    assert!(sim.world().get_entity(visual).is_err());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::ball::body::view")
        })
        .unwrap()
        .id();
    assert_ne!(restored, visual);
    assert_eq!(
        sim.world().get::<VisualProperties>(restored).unwrap().0,
        initial
    );
}
