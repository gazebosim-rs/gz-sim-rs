// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::logical_camera::{
    LogicalCameraConfig, LogicalCameraModel, LogicalCameraPose, logical_camera,
};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    LogicalCameraSample, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step,
    queue_link_world_velocity, remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;
const XML: &str = include_str!("fixtures/m30_logical_camera.sdf");
fn named(sim: &Simulation, name: &str) -> (bevy_ecs::entity::Entity, u64) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (e.id(), e.get::<EntityId>().unwrap().get())
}
fn p(p: WorldPose) -> LogicalCameraPose {
    LogicalCameraPose {
        translation: p.translation,
        rotation: p.rotation,
    }
}
#[test]
fn moving_camera_and_target_origin_relative_rotation_and_held_due_frames() {
    let mut sim = load_world_with_sdf_step(
        XML,
        "moving",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let e = named(&sim, "default::rig::body::logical_camera").0;
    let body_id = named_id(&sim, "default::rig::body");
    queue_link_world_velocity(&mut sim, body_id, [0.5, 0.0, 0.0], [0.0, 0.0, 0.8]).unwrap();
    let target_id = named(&sim, "default::target::body").1;
    queue_link_world_velocity(&mut sim, target_id, [0.0, 0.3, 0.0], [0.0, 0.5, 0.0]).unwrap();
    let mut held = None;
    let mut due = Vec::new();
    for tick in 1..=30 {
        sim.step_exact(1).unwrap();
        let s = sim.world().get::<LogicalCameraSample>(e).unwrap();
        let pair = (s.time, s.reading.clone().unwrap());
        if held.as_ref().is_none_or(|(time, _)| *time != s.time) {
            let camera = p(*sim.world().get::<WorldPose>(e).unwrap());
            let objects: Vec<_> = ["rig", "target", "behind"]
                .into_iter()
                .map(|name| LogicalCameraModel {
                    name: name.into(),
                    pose: p(*sim
                        .world()
                        .get::<WorldPose>(named(&sim, &format!("default::{name}")).0)
                        .unwrap()),
                })
                .collect();
            let expected = logical_camera(
                LogicalCameraConfig {
                    near: 0.5,
                    far: 10.0,
                    horizontal_fov: std::f64::consts::FRAC_PI_2,
                    aspect_ratio: 1.0,
                },
                camera,
                &objects,
            )
            .unwrap();
            assert_eq!(pair.1, expected);
            assert_eq!(s.time, Duration::from_millis(tick));
            due.push(pair.clone());
        } else {
            assert_eq!(held.as_ref(), Some(&pair));
        }
        held = Some(pair);
    }
    assert_eq!(
        due.iter().map(|v| v.0.as_millis()).collect::<Vec<_>>(),
        [1, 10, 20, 30]
    );
    assert_ne!(due[0].1.pose.rotation, due[3].1.pose.rotation);
    assert_ne!(due[0].1.models, due[3].1.models);
    assert!(
        due.iter()
            .all(|v| v.1.models.iter().any(|m| m.name == "rig"))
    );
    let identity = sim.world().get::<EntityId>(e).unwrap().get();
    let initial = due[0].1.clone();
    let rig = named(&sim, "default::rig").1;
    remove_model(&mut sim, rig).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let fresh = entity_by_id(sim.world(), identity).unwrap();
    assert_ne!(fresh, e);
    sim.step_exact(1).unwrap();
    let reset = sim.world().get::<LogicalCameraSample>(fresh).unwrap();
    assert_eq!((reset.epoch, reset.time), (1, Duration::from_millis(1)));
    assert_ne!(reset.reading.as_ref().unwrap().pose, initial.pose);
    assert_eq!(
        reset.reading.as_ref().unwrap().pose.translation,
        [-1.0, 0.0, 3.0]
    );
}
fn named_id(sim: &Simulation, name: &str) -> u64 {
    named(sim, name).1
}
#[test]
fn model_geometry_size_does_not_change_origin_visibility() {
    let acquire = |xml: &str| {
        let mut sim = load_world_with_sdf_step(
            xml,
            "geometry",
            "default",
            &NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        sim.step_exact(1).unwrap();
        sim.world()
            .get::<LogicalCameraSample>(named(&sim, "default::rig::body::logical_camera").0)
            .unwrap()
            .reading
            .clone()
            .unwrap()
    };
    // Keep the admitted one collider per link. Static geometry prevents the
    // large overlapping boxes from changing the model origins through dynamics.
    let xml = XML
        .replace(
            "<model name='rig'>",
            "<model name='rig'><static>true</static>",
        )
        .replace(
            "<model name='target'>",
            "<model name='target'><static>true</static>",
        );
    let small = acquire(&xml);
    let huge = acquire(&xml.replace("<size>0.1 0.1 0.1</size>", "<size>100 100 100</size>"));
    assert_eq!(small, huge);
}
