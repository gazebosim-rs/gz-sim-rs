// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::EntityId;
use gz_sim_scene::{
    AltimeterPublication, AltimeterSample, Name, PhysicsEngine, load_world_with_sdf_step,
    remove_model, reset_loaded_world,
};

#[test]
fn robot_altimeter_and_both_stl_formats_coexist_through_removal_and_loaded_reset() {
    let source = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/m13_m14_robot.sdf")
        .canonicalize()
        .expect("combined example");
    let xml = std::fs::read_to_string(&source).expect("example XML");
    let uri = format!("file://{}", source.display());
    let mut sim = load_world_with_sdf_step(
        &xml,
        &uri,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .expect("STL collision, visual metadata and altimeter together");
    let (sensor_id, metadata) = sim
        .world()
        .iter_entities()
        .find_map(|entity| {
            entity.get::<AltimeterPublication>().map(|metadata| {
                (
                    entity.get::<EntityId>().expect("sensor identity").get(),
                    metadata.clone(),
                )
            })
        })
        .expect("one loaded altimeter");
    assert_eq!(metadata.vertical_reference, 0.4);
    assert_eq!(
        metadata.topic,
        "/world/default/model/robot/link/base_link/sensor/altimeter/altimeter"
    );
    let landmark = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::stl_landmark")
        })
        .expect("binary collision landmark")
        .get::<EntityId>()
        .expect("model identity")
        .get();
    sim.step_exact(3).expect("real physics and ideal sampling");
    let sample = sim
        .world()
        .iter_entities()
        .find_map(|entity| entity.get::<AltimeterSample>())
        .expect("altimeter acquired alongside STL geometry");
    assert!(
        sample
            .reading
            .as_ref()
            .expect("finite measurement")
            .vertical_position
            .is_finite()
    );
    remove_model(&mut sim, landmark).expect("remove STL landmark independently");
    assert!(
        sim.world()
            .iter_entities()
            .any(|entity| entity.get::<EntityId>().map(EntityId::get) == Some(sensor_id))
    );
    reset_loaded_world(&mut sim, |_| ()).expect("reset both independent increments");
    assert!(
        sim.world()
            .iter_entities()
            .any(|entity| entity.get::<EntityId>().map(EntityId::get) == Some(landmark))
    );
    let restored = sim
        .world()
        .iter_entities()
        .find(|entity| entity.get::<EntityId>().map(EntityId::get) == Some(sensor_id))
        .expect("same sensor identity");
    assert_eq!(restored.get::<AltimeterPublication>(), Some(&metadata));
    assert!(restored.get::<AltimeterSample>().is_none());
}
