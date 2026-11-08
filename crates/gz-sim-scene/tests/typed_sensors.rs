// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "support/lidar_admission.rs"]
mod lidar_admission;
use gz_sim_scene::{LoadError, PhysicsEngine, load_world_with_engine};
use sdformat::{NoResources, SensorConfig, load_str};
use std::{collections::HashSet, time::Duration};

fn scene(sensor: &str) -> String {
    let base = include_str!("fixtures/fall.sdf");
    let start = base.find("<sensor name=\"imu\"").unwrap();
    let end = start + base[start..].find("</sensor>").unwrap() + "</sensor>".len();
    format!("{}{}{}", &base[..start], sensor, &base[end..])
}

#[test]
fn parsed_sensor_capabilities_admit_named_increments_and_preserve_other_refusals() {
    let cases = [
        ("contact", "<contact><collision>shape</collision></contact>"),
        ("force_torque", "<force_torque/>"),
        ("magnetometer", "<magnetometer/>"),
        ("altimeter", "<altimeter/>"),
        ("air_pressure", "<air_pressure/>"),
        ("navsat", "<navsat/>"),
        ("air_speed", "<air_speed/>"),
        (
            "lidar",
            "<lidar><scan><horizontal><samples>16</samples><resolution>1</resolution><min_angle>-1</min_angle><max_angle>1</max_angle></horizontal></scan><range><min>0.1</min><max>10</max><resolution>0.01</resolution></range></lidar>",
        ),
        (
            "camera",
            "<camera><image><width>8</width><height>8</height><format>R8G8B8</format></image><clip><near>0.1</near><far>10</far></clip></camera>",
        ),
    ];
    let mut variants = HashSet::new();
    let mut admitted_cpu_lidars = 0;
    let mut admitted_altimeters = 0;
    for (kind, configuration) in cases {
        let xml = scene(&format!(
            "<sensor name='unsupported' type='{kind}'><always_on>true</always_on>{configuration}</sensor>"
        ));
        let parsed = load_str(&xml, "typed-sensors.sdf", &NoResources).unwrap();
        let sensor = &parsed.worlds[0].models[1].links[0].sensors[0];
        assert_eq!(sensor.kind, kind);
        assert!(sensor.effective_type().is_some());
        assert!(sensor.imu().is_none());
        assert!(!matches!(sensor.config, SensorConfig::Imu(_)));
        variants.insert(std::mem::discriminant(&sensor.config));
        for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
            let result = load_world_with_engine(
                &xml,
                "typed-sensors.sdf",
                "default",
                Duration::from_millis(1),
                &NoResources,
                engine,
            );
            if kind == "lidar" && matches!(engine, PhysicsEngine::Rapier) {
                lidar_admission::assert_cpu_lidar(
                    result.unwrap(),
                    &xml,
                    "typed-sensors.sdf",
                    "default::ball::body::unsupported",
                    16,
                    [0.0, 0.0, 5.0],
                );
                admitted_cpu_lidars += 1;
                continue;
            }
            if kind == "altimeter" && matches!(engine, PhysicsEngine::Rapier) {
                let sim = result.unwrap();
                let publications = sim
                    .world()
                    .iter_entities()
                    .filter_map(|entity| entity.get::<gz_sim_scene::AltimeterPublication>())
                    .collect::<Vec<_>>();
                assert_eq!(publications.len(), 1);
                assert_eq!(publications[0].entity_name, "ball::body::unsupported");
                assert_eq!(publications[0].vertical_reference, 5.0);
                assert_eq!(
                    publications[0].topic,
                    "/world/default/model/ball/link/body/sensor/unsupported/altimeter"
                );
                admitted_altimeters += 1;
                continue;
            }
            match result {
                Err(LoadError::Unsupported { entity, feature }) => {
                    assert_eq!(entity, "default::ball::body::unsupported");
                    assert_eq!(
                        feature,
                        if kind == "altimeter" {
                            "Rapier ideal altimeter"
                        } else {
                            "non-IMU sensors"
                        }
                    );
                }
                _ => {
                    panic!("{kind} on {engine:?} must fail at the typed sensor capability boundary")
                }
            }
        }
    }
    assert_eq!(variants.len(), 9);
    assert_eq!(admitted_cpu_lidars, 1);
    assert_eq!(admitted_altimeters, 1);
}

#[test]
fn parsed_joint_sensors_are_not_silently_dropped_or_lowered_as_measurements() {
    let xml = include_str!("fixtures/pendulum.sdf").replace("</joint>", "<sensor name='wrench' type='force_torque'><force_torque><frame>sensor</frame><measure_direction>child_to_parent</measure_direction></force_torque></sensor></joint>");
    let parsed = load_str(&xml, "joint-sensor.sdf", &NoResources).unwrap();
    let joint = &parsed.worlds[0].models[0].joints[0];
    assert_eq!(joint.sensors.len(), 1);
    assert!(joint.sensors[0].force_torque().is_some());
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        match load_world_with_engine(
            &xml,
            "joint-sensor.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
            engine,
        ) {
            Err(LoadError::Unsupported { entity, feature }) => {
                assert_eq!(entity, "default::robot::hinge");
                assert_eq!(feature, "joint sensors");
            }
            _ => panic!("joint sensor must be rejected before {engine:?} lowering"),
        }
    }
}

#[test]
fn static_attachment_sink_is_preserved_without_changing_simulator_static_policy() {
    let xml = "<sdf version='1.12'><world name='default'><model name='anchor'><static>true</static><pose>1 2 3 0 0 0.4</pose><link name='base'><collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link><frame name='mount' attached_to='__model__'><pose>1 0 0 0 0 0</pose></frame></model></world></sdf>";
    let parsed = load_str(xml, "static-frame.sdf", &NoResources).unwrap();
    assert_eq!(
        parsed.attached_body("default::anchor::mount").unwrap(),
        "default::anchor::__model__"
    );
    let expected = parsed.pose_of("default::anchor::base", "default").unwrap();
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim = load_world_with_engine(
            xml,
            "static-frame.sdf",
            "default",
            Duration::from_millis(1),
            &NoResources,
            engine,
        )
        .unwrap();
        sim.step_exact(10).unwrap();
        let state = |sim: &gz_sim_core::Simulation| {
            let entity = sim
                .world()
                .iter_entities()
                .find(|e| {
                    e.get::<gz_sim_scene::Name>()
                        .is_some_and(|name| name.0 == "default::anchor::base")
                })
                .unwrap();
            (
                entity.get::<gz_sim_core::EntityId>().unwrap().get(),
                *entity.get::<gz_sim_scene::WorldPose>().unwrap(),
            )
        };
        let before = state(&sim);
        for (actual, target) in before
            .1
            .translation
            .into_iter()
            .zip(expected.translation)
            .chain(before.1.rotation.into_iter().zip(expected.rotation))
        {
            assert!((actual - target).abs() < 1e-12);
        }
        gz_sim_scene::reset_loaded_world(&mut sim, |_| ()).unwrap();
        let after = state(&sim);
        assert_eq!(after.0, before.0);
        assert_eq!(after.1.translation, before.1.translation);
        assert_eq!(after.1.rotation, before.1.rotation);
    }
}
