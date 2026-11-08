// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    AirPressureSample, JointStatePublication, JointStateSample, MagnetometerPublication,
    MagnetometerSample, Name, PhysicsEngine, SceneResetError, initial_joint_state_publications,
    initial_magnetometer_publications, load_world_with_sdf_step, remove_model, reset_loaded_world,
    set_diff_drive_command,
};
use std::time::Duration;

fn named(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

#[test]
fn magnetic_and_joint_publication_coexist_with_the_accepted_robot_and_restore_loaded_ids() {
    let source = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/m17_m18_robot.sdf")
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
    let robot = named(&sim, "default::robot");
    let sensor = named(&sim, "default::robot::base_link::magnetometer");
    let robot_entity = entity_by_id(sim.world(), robot).unwrap();
    let sensor_entity = entity_by_id(sim.world(), sensor).unwrap();
    let magnetic = sim
        .world()
        .get::<MagnetometerPublication>(sensor_entity)
        .unwrap()
        .clone();
    let joints = sim
        .world()
        .get::<JointStatePublication>(robot_entity)
        .unwrap()
        .clone();
    assert_eq!(magnetic.topic, "/m17_m18/magnetometer");
    assert_eq!(magnetic.entity_name, "robot::base_link::magnetometer");
    assert_eq!(joints.topic, "/m17_m18/joint_state");
    assert_eq!(joints.joints.len(), 2);
    let initial_counts = sim.scheduled_system_counts();
    set_diff_drive_command(&mut sim, robot, 0.4, 0.8).unwrap();
    sim.step_exact(20).unwrap();
    let joint_sample = sim.world().get::<JointStateSample>(robot_entity).unwrap();
    assert_eq!(
        (joint_sample.time, joint_sample.epoch, joint_sample.revision),
        (Duration::from_millis(20), 0, 2)
    );
    assert!(
        joint_sample
            .joints
            .iter()
            .any(|joint| joint.velocity.abs() > 0.0)
    );
    let field = sim
        .world()
        .get::<MagnetometerSample>(sensor_entity)
        .unwrap();
    assert_eq!(field.epoch, 0);
    assert!(field.time > Duration::ZERO && field.time <= Duration::from_millis(20));
    assert!(
        field
            .reading
            .as_ref()
            .unwrap()
            .field_tesla
            .iter()
            .all(|value| value.is_finite())
    );
    assert!(sim.world().iter_entities().any(|entity| {
        entity
            .get::<AirPressureSample>()
            .is_some_and(|sample| sample.reading.is_ok())
    }));

    let landmark = named(&sim, "default::obj_landmark");
    remove_model(&mut sim, landmark).unwrap();
    assert!(
        sim.world()
            .get::<JointStatePublication>(robot_entity)
            .is_some()
    );
    remove_model(&mut sim, robot).unwrap();
    assert!(entity_by_id(sim.world(), sensor).is_none());
    assert!(!sim.world().iter_entities().any(|entity| {
        entity.contains::<MagnetometerSample>() || entity.contains::<JointStateSample>()
    }));
    let retired_counts = sim.scheduled_system_counts();
    assert!(retired_counts.0 < initial_counts.0 && retired_counts.1 < initial_counts.1);
    assert_eq!(
        initial_magnetometer_publications(&sim)
            .unwrap()
            .collect::<Vec<_>>(),
        vec![&magnetic]
    );
    assert_eq!(
        initial_joint_state_publications(&sim)
            .unwrap()
            .collect::<Vec<_>>(),
        vec![(robot, &joints)]
    );
    sim.step_exact(3).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.scheduled_system_counts(), initial_counts);
    let restored_robot = entity_by_id(sim.world(), robot).unwrap();
    let restored_sensor = entity_by_id(sim.world(), sensor).unwrap();
    assert_eq!(
        sim.world().get::<JointStatePublication>(restored_robot),
        Some(&joints)
    );
    assert_eq!(
        sim.world().get::<MagnetometerPublication>(restored_sensor),
        Some(&magnetic)
    );
    assert!(
        sim.world()
            .get::<JointStateSample>(restored_robot)
            .is_none()
    );
    assert!(
        sim.world()
            .get::<MagnetometerSample>(restored_sensor)
            .is_none()
    );
    sim.step_exact(20).unwrap();
    assert_eq!(
        sim.world()
            .get::<JointStateSample>(restored_robot)
            .unwrap()
            .epoch,
        1
    );
    assert_eq!(
        sim.world()
            .get::<MagnetometerSample>(restored_sensor)
            .unwrap()
            .epoch,
        1
    );
}

#[test]
fn initial_stream_getters_require_a_loader_blueprint() {
    let empty = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        initial_joint_state_publications(&empty),
        Err(SceneResetError::MissingInitialState)
    ));
    assert!(matches!(
        initial_magnetometer_publications(&empty),
        Err(SceneResetError::MissingInitialState)
    ));
}
