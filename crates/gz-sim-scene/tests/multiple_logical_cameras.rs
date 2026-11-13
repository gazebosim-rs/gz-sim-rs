// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use glam::{DQuat, DVec3};
use gz_sensors::logical_camera::{
    LogicalCameraModel, LogicalCameraPose, LogicalCameraReading, logical_camera,
};
use gz_sim_core::{EntityId, Phase, Simulation, StepError, StepFault, entity_by_id};
use gz_sim_scene::{
    EntityKind, LoadError, LogicalCameraPublication, LogicalCameraSample, Name, PhysicsEngine,
    SceneResetError, WorldPose, clear_logical_camera_interest, load_world_with_sdf_step,
    queue_link_world_wrench, remove_model, reset_loaded_world, set_logical_camera_interest,
};
use sdformat::NoResources;
use std::time::Duration;

fn sensor_xml(index: usize, rate: u32, yaw: &str) -> String {
    format!(
        "<sensor name='camera{index}' type='logical_camera'><pose>-1 0 0 0 0 {yaw}</pose><update_rate>{rate}</update_rate><topic>/m53/camera{index}</topic><frame_id>rig{index}/body/camera{index}</frame_id><logical_camera><near>0.5</near><far>10</far><horizontal_fov>1.5707963267948966</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor>"
    )
}
fn model(index: usize, sensors: &str) -> String {
    format!(
        "<model name='rig{index}'><pose>0 {} 3 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='box'><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision>{sensors}</link></model>",
        index * 3
    )
}
fn world(count: usize) -> String {
    let models: String = (0..count)
        .map(|i| {
            model(
                i,
                &sensor_xml(
                    i,
                    [100, 50, 25][i % 3],
                    if i.is_multiple_of(2) { "0" } else { "0.3" },
                ),
            )
        })
        .collect();
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='step' type='ode'><max_step_size>0.001</max_step_size></physics>{models}<model name='target'><pose>4 0 3 0 0 0.4</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='box'><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link></model></world></sdf>"
    )
}
fn try_load(xml: &str, name: &str) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        xml,
        "multiple-logical-cameras",
        name,
        &NoResources,
        PhysicsEngine::Rapier,
    )
}
fn load(xml: &str) -> Simulation {
    try_load(xml, "default").unwrap()
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (e.id(), e.get::<EntityId>().unwrap().get())
}
fn camera(sim: &Simulation, i: usize) -> Entity {
    named(sim, &format!("default::rig{i}::body::camera{i}")).0
}
fn sample(sim: &Simulation, e: Entity) -> (Duration, u64, LogicalCameraReading) {
    let s = sim.world().get::<LogicalCameraSample>(e).unwrap();
    (s.time, s.epoch, s.reading.as_ref().unwrap().clone())
}
fn pose(p: WorldPose) -> LogicalCameraPose {
    LogicalCameraPose {
        translation: p.translation,
        rotation: p.rotation,
    }
}
fn geometry(sim: &Simulation, e: Entity) {
    let current = sample(sim, e);
    assert_eq!(current.0, sim.elapsed());
    let camera = pose(*sim.world().get::<WorldPose>(e).unwrap());
    let objects: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|v| v.get::<EntityKind>() == Some(&EntityKind::Model))
        .map(|v| LogicalCameraModel {
            name: v
                .get::<Name>()
                .unwrap()
                .0
                .strip_prefix("default::")
                .unwrap()
                .into(),
            pose: pose(*v.get::<WorldPose>().unwrap()),
        })
        .collect();
    let expected = logical_camera(
        sim.world()
            .get::<LogicalCameraPublication>(e)
            .unwrap()
            .config,
        camera,
        &objects,
    )
    .unwrap();
    assert_eq!(current.2, expected);
    // Independent inverse-quaternion full-pose check, separate from membership.
    let inverse = DQuat::from_array(camera.rotation).inverse();
    for detected in &current.2.models {
        let original = &objects
            .iter()
            .find(|m| m.name == detected.name)
            .unwrap()
            .pose;
        let translation = inverse
            * (DVec3::from_array(original.translation) - DVec3::from_array(camera.translation));
        let rotation = inverse * DQuat::from_array(original.rotation);
        for (a, b) in detected
            .pose
            .translation
            .into_iter()
            .zip(translation.to_array())
            .chain(detected.pose.rotation.into_iter().zip(rotation.to_array()))
        {
            assert!((a - b).abs() < 1e-12);
        }
    }
}

#[test]
fn distinct_owners_real_motion_fresh_frames_and_loaded_reset_preserve_full_geometry() {
    let xml = world(2);
    let mut sim = load(&xml);
    let handles = [camera(&sim, 0), camera(&sim, 1)];
    let ids = handles.map(|e| sim.world().get::<EntityId>(e).unwrap().get());
    let metadata = handles.map(|e| {
        sim.world()
            .get::<LogicalCameraPublication>(e)
            .unwrap()
            .clone()
    });
    let body = named(&sim, "default::rig0::body");
    let initial = *sim.world().get::<WorldPose>(body.0).unwrap();
    queue_link_world_wrench(&mut sim, body.1, [8.0, -4.0, 2.0], [2.0, -3.0, 4.0]).unwrap();
    let target = named(&sim, "default::target::body").1;
    queue_link_world_wrench(&mut sim, target, [-2.0, 3.0, 1.0], [1.0, 2.0, -1.0]).unwrap();
    let mut previous = [None, None];
    let mut stamps = [Vec::new(), Vec::new()];
    for _ in 0..41 {
        sim.step_exact(1).unwrap();
        for (index, (old, times)) in previous.iter_mut().zip(&mut stamps).enumerate() {
            let value = sample(&sim, handles[index]);
            if value.0 == sim.elapsed() {
                geometry(&sim, handles[index]);
                times.push(value.0);
            } else {
                assert_eq!(Some(&value), old.as_ref());
            }
            *old = Some(value);
        }
    }
    assert_eq!(stamps[0], [1, 10, 20, 30, 40].map(Duration::from_millis));
    assert_eq!(stamps[1], [1, 20, 40].map(Duration::from_millis));
    let moved = *sim.world().get::<WorldPose>(body.0).unwrap();
    assert_ne!(initial.translation, moved.translation);
    assert_ne!(initial.rotation, moved.rotation);
    assert_ne!(
        sample(&sim, handles[0]).2.pose,
        sample(&sim, handles[1]).2.pose
    );
    assert_ne!(
        sample(&sim, handles[0]).2.models,
        sample(&sim, handles[1]).2.models
    );
    for (i, e) in handles.into_iter().enumerate() {
        assert!(
            sample(&sim, e)
                .2
                .models
                .iter()
                .any(|m| m.name == format!("rig{i}"))
        );
    }
    reset_loaded_world(&mut sim, |sim| {
        for id in ids {
            assert!(
                sim.world()
                    .get::<LogicalCameraSample>(entity_by_id(sim.world(), id).unwrap())
                    .is_none()
            );
        }
    })
    .unwrap();
    let mut fresh = load(&xml);
    sim.step_exact(1).unwrap();
    fresh.step_exact(1).unwrap();
    for (i, id) in ids.into_iter().enumerate() {
        let e = entity_by_id(sim.world(), id).unwrap();
        assert_ne!(e, handles[i]);
        assert_eq!(
            sim.world().get::<LogicalCameraPublication>(e),
            Some(&metadata[i])
        );
        let reset = sample(&sim, e);
        assert_eq!((reset.0, reset.1), (Duration::from_millis(1), 1));
        assert_eq!(reset.2, sample(&fresh, camera(&fresh, i)).2);
        geometry(&sim, e);
    }
}

#[test]
fn eight_rates_are_independent_and_ninth_or_late_invalid_configuration_refuses() {
    let mut sim = load(&world(8));
    let mut stamps: [Vec<Duration>; 8] = std::array::from_fn(|_| Vec::new());
    let mut previous = std::array::from_fn::<_, 8, _>(|_| None);
    for _ in 0..41 {
        sim.step_exact(1).unwrap();
        for (i, (old, times)) in previous.iter_mut().zip(&mut stamps).enumerate() {
            let e = camera(&sim, i);
            let value = sample(&sim, e);
            if value.0 == sim.elapsed() {
                geometry(&sim, e);
                times.push(value.0);
            } else {
                assert_eq!(Some(&value), old.as_ref());
            }
            *old = Some(value);
        }
    }
    for (i, times) in stamps.iter().enumerate() {
        let expected: &[u64] = match i % 3 {
            0 => &[1, 10, 20, 30, 40],
            1 => &[1, 20, 40],
            _ => &[1, 40],
        };
        assert_eq!(
            *times,
            expected
                .iter()
                .copied()
                .map(Duration::from_millis)
                .collect::<Vec<_>>()
        );
    }
    assert!(matches!(
        try_load(&world(9), "default"),
        Err(LoadError::Unsupported {
            feature: "one loaded logical_camera",
            ..
        })
    ));
    let valid = world(8);
    let marker = sensor_xml(7, 50, "0.3");
    assert!(valid.contains(&marker));
    let invalid = valid.replace(&marker, &marker.replace("<far>10</far>", "<far>0.1</far>"));
    assert!(matches!(
        try_load(&invalid, "default"),
        Err(LoadError::Unsupported {
            feature: "bounded logical_camera configuration",
            ..
        })
    ));
    let bad_metadata = valid.replace(
        &marker,
        &marker.replace("<update_rate>", "<visualize>true</visualize><update_rate>"),
    );
    assert!(matches!(
        try_load(&bad_metadata, "default"),
        Err(LoadError::Unsupported {
            feature: "logical_camera metadata",
            ..
        })
    ));
}

#[test]
fn demand_pause_equal_time_and_reconnection_do_not_advance_other_owner_deadlines() {
    let mut sim = load(&world(2));
    let a = camera(&sim, 0);
    let b = camera(&sim, 1);
    set_logical_camera_interest(&mut sim, [a, a]);
    sim.step_exact(21).unwrap();
    assert_eq!(sample(&sim, a).0, Duration::from_millis(20));
    assert!(sim.world().get::<LogicalCameraSample>(b).is_none());
    let held = sample(&sim, a);
    sim.tick().unwrap();
    assert_eq!(sample(&sim, a), held);
    assert!(sim.world().get::<LogicalCameraSample>(b).is_none());
    set_logical_camera_interest(&mut sim, [a, b]);
    sim.step_exact(1).unwrap();
    assert_eq!(sample(&sim, a), held);
    assert_eq!(sample(&sim, b).0, Duration::from_millis(22));
    geometry(&sim, b);
    set_logical_camera_interest(&mut sim, [b]);
    assert!(sim.world().get::<LogicalCameraSample>(a).is_none());
    sim.step_exact(18).unwrap();
    assert_eq!(sample(&sim, b).0, Duration::from_millis(40));
    set_logical_camera_interest(&mut sim, []);
    assert!(sim.world().get::<LogicalCameraSample>(b).is_none());
    sim.step_exact(7).unwrap();
    clear_logical_camera_interest(&mut sim);
    sim.step_exact(1).unwrap();
    assert_eq!(sample(&sim, a).0, Duration::from_millis(48));
    geometry(&sim, a);
    assert!(sim.world().get::<LogicalCameraSample>(b).is_none());
    let mut held_a = sample(&sim, a);
    for tick in 49..=59 {
        sim.step_exact(1).unwrap();
        assert!(sim.world().get::<LogicalCameraSample>(b).is_none());
        if tick == 50 {
            assert_eq!(sample(&sim, a).0, Duration::from_millis(50));
            geometry(&sim, a);
            held_a = sample(&sim, a);
        } else {
            assert_eq!(sample(&sim, a), held_a);
        }
    }
    assert_eq!(held_a.0, Duration::from_millis(50));
    sim.step_exact(1).unwrap();
    for e in [a, b] {
        assert_eq!(sample(&sim, e).0, Duration::from_millis(60));
        geometry(&sim, e);
    }
}

#[test]
fn removing_camera_and_target_preserves_peer_then_last_retirement_and_loaded_restore() {
    let mut sim = load(&world(2));
    let original_counts = sim.scheduled_system_counts();
    let handles = [camera(&sim, 0), camera(&sim, 1)];
    let ids = handles.map(|e| sim.world().get::<EntityId>(e).unwrap().get());
    sim.step_exact(1).unwrap();
    let rig = named(&sim, "default::rig0").1;
    remove_model(&mut sim, rig).unwrap();
    assert!(entity_by_id(sim.world(), ids[0]).is_none());
    assert_eq!(sim.scheduled_system_counts(), original_counts);
    let target = named(&sim, "default::target").1;
    remove_model(&mut sim, target).unwrap();
    sim.step_exact(19).unwrap();
    let peer = sample(&sim, handles[1]);
    assert_eq!(peer.0, Duration::from_millis(20));
    assert_eq!(
        peer.2
            .models
            .iter()
            .map(|m| m.name.as_str())
            .collect::<Vec<_>>(),
        ["rig1"]
    );
    geometry(&sim, handles[1]);
    let rig = named(&sim, "default::rig1").1;
    remove_model(&mut sim, rig).unwrap();
    let retired = sim.scheduled_system_counts();
    assert_eq!(original_counts, (retired.0 + 1, retired.1 + 1));
    sim.reset().unwrap();
    for id in ids {
        assert!(entity_by_id(sim.world(), id).is_none());
    }
    assert_eq!(sim.scheduled_system_counts(), retired);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.scheduled_system_counts(), original_counts);
    sim.step_exact(1).unwrap();
    for (i, id) in ids.into_iter().enumerate() {
        let e = entity_by_id(sim.world(), id).unwrap();
        assert_ne!(e, handles[i]);
        assert_eq!(
            (sample(&sim, e).0, sample(&sim, e).1),
            (Duration::from_millis(1), 2)
        );
        geometry(&sim, e);
    }
}

#[test]
fn absence_single_compatibility_world_prefix_and_empty_images_keep_existing_domain() {
    let mut absent = load(&world(0));
    let empty_counts = absent.scheduled_system_counts();
    set_logical_camera_interest(&mut absent, []);
    absent.step_exact(2).unwrap();
    absent.reset().unwrap();
    assert_eq!(absent.scheduled_system_counts(), empty_counts);
    let mut single = load(&world(1));
    assert_eq!(
        single.scheduled_system_counts(),
        (empty_counts.0 + 1, empty_counts.1 + 1)
    );
    single.step_exact(1).unwrap();
    geometry(&single, camera(&single, 0));
    let xml = world(1).replace("world name='default'", "world name='outer_inner'");
    // XML names still reject scope separators before any Scene is built.
    let invalid = xml.replace("world name='outer_inner'", "world name='outer::inner'");
    match try_load(&invalid, "outer::inner") {
        Err(LoadError::Sdf(diagnostic)) => {
            assert_eq!(diagnostic.code, sdformat::ErrorCode::InvalidValue);
            assert_eq!(diagnostic.message, "reserved or invalid name");
            assert_eq!(diagnostic.source.uri, "multiple-logical-cameras");
        }
        Err(error) => panic!("expected scoped WORLD-name parser refusal, got {error}"),
        Ok(_) => panic!("scoped XML WORLD name must remain refused"),
    }
    let mut scoped = try_load(&xml, "outer_inner").unwrap();
    scoped.step_exact(1).unwrap();
    let e = named(&scoped, "outer_inner::rig0::body::camera0").0;
    assert_eq!(
        scoped
            .world()
            .get::<LogicalCameraPublication>(e)
            .unwrap()
            .entity_name,
        "rig0::body::camera0"
    );
    assert_eq!(sample(&scoped, e).2, sample(&single, camera(&single, 0)).2);
    let backwards = world(1).replace(
        &sensor_xml(0, 100, "0"),
        &sensor_xml(0, 100, "3.141592653589793"),
    );
    let mut backward = load(&backwards);
    backward.step_exact(1).unwrap();
    assert!(sample(&backward, camera(&backward, 0)).2.models.is_empty());
    geometry(&backward, camera(&backward, 0));
}

fn fault_clock(fault: bevy_ecs::prelude::Res<StepFault>) {
    fault.report(StepError::Overflow);
}
#[test]
fn metadata_drift_invalidates_only_its_frame_and_terminal_reset_is_nonmutating() {
    let mut sim = load(&world(2));
    sim.step_exact(1).unwrap();
    let a = camera(&sim, 0);
    let b = camera(&sim, 1);
    let metadata = [a, b].map(|e| {
        sim.world()
            .get::<LogicalCameraPublication>(e)
            .unwrap()
            .clone()
    });
    sim.world_mut()
        .get_mut::<LogicalCameraPublication>(a)
        .unwrap()
        .frame_id = "drift".into();
    sim.step_exact(19).unwrap();
    assert!(
        sim.world()
            .get::<LogicalCameraSample>(a)
            .unwrap()
            .reading
            .is_err()
    );
    assert_eq!(sample(&sim, b).0, Duration::from_millis(20));
    geometry(&sim, b);
    *sim.world_mut()
        .get_mut::<LogicalCameraPublication>(a)
        .unwrap() = metadata[0].clone();
    sim.step_exact(10).unwrap();
    geometry(&sim, a);
    let values = [sample(&sim, a), sample(&sim, b)];
    let bodies = [0, 1].map(|i| {
        *sim.world()
            .get::<WorldPose>(named(&sim, &format!("default::rig{i}::body")).0)
            .unwrap()
    });
    sim.add_systems(Phase::PostStep, fault_clock);
    assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
    assert_eq!([sample(&sim, a), sample(&sim, b)], values);
    let clock = (sim.elapsed(), sim.epoch());
    let mut called = false;
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| {
            called = true;
        }),
        Err(SceneResetError::Clock(StepError::Faulted))
    ));
    assert!(!called);
    assert_eq!((sim.elapsed(), sim.epoch()), clock);
    assert_eq!([sample(&sim, a), sample(&sim, b)], values);
    assert_eq!(
        sim.world().get::<LogicalCameraPublication>(a),
        Some(&metadata[0])
    );
    assert_eq!(
        sim.world().get::<LogicalCameraPublication>(b),
        Some(&metadata[1])
    );
    for (i, body) in bodies.iter().enumerate() {
        let current = sim
            .world()
            .get::<WorldPose>(named(&sim, &format!("default::rig{i}::body")).0)
            .unwrap();
        assert_eq!(current.translation, body.translation);
        assert_eq!(current.rotation, body.rotation);
    }
}
