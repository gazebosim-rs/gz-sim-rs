// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn sensor_move_preserves_parsed_metadata_and_presence() {
    let xml = include_str!("../fixtures/camera-binding.sdf");
    let mut sdf = sdformat::load_str(xml, "camera-binding.sdf", &sdformat::NoResources).unwrap();
    let mut world = World::new();
    let entity = world.spawn_empty().id();
    let mut entities = [entity].into_iter();
    retain_camera_sensors(&mut sdf.worlds[0].models, &mut entities, &mut world);
    assert!(entities.next().is_none());
    let retained = &world.get::<CameraSensor>(entity).unwrap().0;
    assert_eq!(retained.topic, "/explicit-camera");
    assert_eq!(retained.pose.transform.translation, [1.0, 0.0, 0.0]);
    assert!(!retained.camera().unwrap().lens.has_intrinsics);
    assert!(!retained.camera().unwrap().lens.has_projection);
    assert!(
        sdf.worlds[0].models[0].links[0].sensors[0]
            .camera()
            .is_none()
    );
}

#[test]
fn sensor_move_skips_imu_and_retains_camera_order() {
    let xml = include_str!("../fixtures/camera-binding.sdf").replace(
        "</sensor>",
        "</sensor><sensor name='imu' type='imu'><imu/></sensor><sensor name='second' type='camera'><topic>/second</topic><camera><image><width>64</width><height>64</height></image><clip><near>0.1</near><far>10</far></clip></camera></sensor>",
    );
    let mut sdf = sdformat::load_str(&xml, "mixed.sdf", &sdformat::NoResources).unwrap();
    let mut world = World::new();
    let first = world.spawn_empty().id();
    let second = world.spawn_empty().id();
    let mut entities = [first, second].into_iter();
    retain_camera_sensors(&mut sdf.worlds[0].models, &mut entities, &mut world);
    assert!(entities.next().is_none());
    assert_eq!(
        world.get::<CameraSensor>(first).unwrap().0.topic,
        "/explicit-camera"
    );
    assert_eq!(
        world.get::<CameraSensor>(second).unwrap().0.topic,
        "/second"
    );
    let sensors = &sdf.worlds[0].models[0].links[0].sensors;
    assert!(sensors[0].camera().is_none());
    assert!(sensors[1].imu().is_some());
    assert!(sensors[2].camera().is_none());
}

#[test]
fn sensor_move_descends_into_nested_models() {
    let xml = include_str!("../fixtures/camera-binding.sdf").replace(
        "</model>",
        "<model name='child'><link name='nested'><sensor name='inside' type='camera'><topic>/nested</topic><camera><image><width>64</width><height>64</height></image><clip><near>0.1</near><far>10</far></clip></camera></sensor></link></model></model>",
    );
    let mut sdf = sdformat::load_str(&xml, "nested.sdf", &sdformat::NoResources).unwrap();
    let mut world = World::new();
    let outer = world.spawn_empty().id();
    let nested = world.spawn_empty().id();
    let mut entities = [outer, nested].into_iter();
    retain_camera_sensors(&mut sdf.worlds[0].models, &mut entities, &mut world);
    assert!(entities.next().is_none());
    assert_eq!(
        world.get::<CameraSensor>(outer).unwrap().0.topic,
        "/explicit-camera"
    );
    assert_eq!(
        world.get::<CameraSensor>(nested).unwrap().0.topic,
        "/nested"
    );
    assert!(
        sdf.worlds[0].models[0].models[0].links[0].sensors[0]
            .camera()
            .is_none()
    );
}
