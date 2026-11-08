// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_scene::{
    AirPressurePublication, AirPressureSample, AltimeterPublication, Name, PhysicsEngine,
    WorldPose, initial_air_pressure_publications, load_world_with_sdf_step, prepare_model_template,
    remove_model, reset_loaded_world,
};

#[test]
fn pressure_altimeter_and_obj_robot_coexist_through_removal_and_loaded_reset() {
    let source = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/m15_m16_robot.sdf")
        .canonicalize()
        .unwrap();
    let xml = std::fs::read_to_string(&source).unwrap();
    let uri = format!("file://{}", source.display());
    let mut sim = load_world_with_sdf_step(
        &xml,
        &uri,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let (sensor_id, metadata, initial_pose) = sim
        .world()
        .iter_entities()
        .find_map(|entity| {
            entity.get::<AirPressurePublication>().map(|metadata| {
                (
                    entity.get::<EntityId>().unwrap().get(),
                    metadata.clone(),
                    *entity.get::<WorldPose>().unwrap(),
                )
            })
        })
        .unwrap();
    assert_eq!(initial_pose.translation, [0.1, 0.0, 0.4]);
    assert_eq!(metadata.entity_name, "robot::base_link::air_pressure");
    assert_eq!(
        metadata.topic,
        "/world/default/model/robot/link/base_link/sensor/air_pressure/air_pressure"
    );
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|e| e.contains::<AltimeterPublication>())
            .count(),
        1
    );
    let landmark = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::obj_landmark")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let robot = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::robot")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let parsed = sdformat::load_str(&xml, &uri, &sdformat::NoResources).unwrap();
    assert!(
        prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[1]).is_err(),
        "runtime sensor/robot spawning remains unsupported"
    );
    let initial_counts = sim.scheduled_system_counts();
    sim.step_exact(3).unwrap();
    let sample = sim
        .world()
        .get::<AirPressureSample>(entity_by_id(sim.world(), sensor_id).unwrap())
        .unwrap();
    let reading = sample.reading.as_ref().unwrap();
    assert!(reading.pressure > 101000.0 && reading.pressure < 101325.0);
    assert_eq!(reading.variance, 0.0);
    remove_model(&mut sim, landmark).unwrap();
    assert!(entity_by_id(sim.world(), sensor_id).is_some());
    remove_model(&mut sim, robot).unwrap();
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<AirPressurePublication>())
    );
    assert_eq!(
        initial_air_pressure_publications(&sim)
            .unwrap()
            .collect::<Vec<_>>(),
        vec![&metadata]
    );
    sim.step_exact(2).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.scheduled_system_counts(), initial_counts);
    assert!(entity_by_id(sim.world(), landmark).is_some());
    let restored = sim
        .world()
        .entity(entity_by_id(sim.world(), sensor_id).unwrap());
    assert_eq!(restored.get::<AirPressurePublication>(), Some(&metadata));
    let restored_pose = restored.get::<WorldPose>().unwrap();
    assert_eq!(restored_pose.translation, initial_pose.translation);
    assert_eq!(restored_pose.rotation, initial_pose.rotation);
    assert!(restored.get::<AirPressureSample>().is_none());
    sim.step_exact(1).unwrap();
    let sample = sim
        .world()
        .get::<AirPressureSample>(entity_by_id(sim.world(), sensor_id).unwrap())
        .unwrap();
    assert_eq!(sample.epoch, 1);
    assert_eq!(sample.time, std::time::Duration::from_millis(1));
    assert!(sample.reading.is_ok());
}
