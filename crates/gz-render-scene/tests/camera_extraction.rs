// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use gz_render_protocol::Request;
use gz_render_scene::{SnapshotError, extract_camera};
use gz_sim_core::assign_id;
use gz_sim_scene::{CameraSensor, EntityKind, PhysicsEngine, WorldPose, load_world_with_sdf_step};
use sdformat::{Camera, NoResources, Sensor, SensorConfig};
use std::sync::Arc;

#[test]
fn explicit_extraction_uses_live_camera_pose_and_rejects_unqualified_lens() {
    let xml = "<sdf version='1.12'><world name='default'><model name='m'><static>true</static><link name='l'><collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model></world></sdf>";
    let mut sim =
        load_world_with_sdf_step(xml, "camera", "default", &NoResources, PhysicsEngine::Tpe)
            .unwrap();
    let mut camera = Camera {
        image_width: 64,
        image_height: 64,
        horizontal_fov: std::f64::consts::FRAC_PI_2,
        ..Default::default()
    };
    camera.lens.has_intrinsics = false;
    camera.lens.has_projection = false;
    let sensor = Sensor {
        kind: "camera".to_owned(),
        config: SensorConfig::Camera(Box::new(camera)),
        ..Default::default()
    };
    let entity = sim
        .world_mut()
        .spawn((
            EntityKind::Sensor,
            CameraSensor(Arc::new(sensor)),
            WorldPose {
                translation: [1_000_000_000.0, 3.0, 4.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        ))
        .id();
    let id = assign_id(sim.world_mut(), entity).unwrap();
    let systems = sim.scheduled_system_counts();
    let request = extract_camera(&sim, id, 7, [1_000_000_000.0, 0.0, 0.0]).unwrap();
    assert!(request.wire_valid());
    let Request::CalibratedSnapshot {
        request_id,
        camera,
        stamp,
        ..
    } = request
    else {
        panic!("calibrated request")
    };
    assert_eq!(request_id, 7);
    assert_eq!(stamp.epoch, 0);
    assert!((camera.fx - 32.0).abs() < 1e-12);
    assert_eq!(camera.translation, [1_000_000_000.0, 3.0, 4.0]);
    assert_eq!(systems, sim.scheduled_system_counts());
    let sensor = &mut sim.world_mut().get_mut::<CameraSensor>(entity).unwrap().0;
    let SensorConfig::Camera(camera) = &mut Arc::make_mut(sensor).config else {
        panic!("camera")
    };
    camera.lens.has_projection = true;
    assert_eq!(
        extract_camera(&sim, id, 8, [0.0; 3]),
        Err(SnapshotError::InvalidCamera(id))
    );
    assert_eq!(
        extract_camera(&sim, id + 100, 8, [0.0; 3]),
        Err(SnapshotError::MissingCamera(id + 100))
    );
}

#[test]
fn loaded_sensor_extraction_tracks_committed_time_and_reset_epoch() {
    use gz_sim_core::EntityId;
    use gz_sim_scene::{Name, reset_loaded_world};
    let source = include_str!("../../gz-sim-scene/tests/fixtures/camera-binding.sdf");
    let mut sim = load_world_with_sdf_step(
        source,
        "camera-binding.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find_map(|entity| {
            (entity.get::<Name>()?.0 == "default::robot::body::camera")
                .then(|| entity.get::<EntityId>().unwrap().get())
        })
        .unwrap();
    sim.step_exact(1).unwrap();
    let request = extract_camera(&sim, id, 1, [1_000_000_000.0, 0.0, 0.0]).unwrap();
    let Request::CalibratedSnapshot { stamp, camera, .. } = request else {
        panic!("calibrated request")
    };
    assert_eq!(stamp.epoch, 0);
    assert_eq!(stamp.seconds, sim.elapsed().as_secs());
    assert_eq!(stamp.nanoseconds, sim.elapsed().subsec_nanos());
    assert_eq!(camera.translation, [1_000_000_000.0, 3.0, 0.0]);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let Request::CalibratedSnapshot { stamp, .. } =
        extract_camera(&sim, id, 2, [1_000_000_000.0, 0.0, 0.0]).unwrap()
    else {
        panic!("calibrated request")
    };
    assert_eq!(stamp.epoch, 1);
    assert_eq!((stamp.seconds, stamp.nanoseconds), (0, 0));
}

#[test]
fn rejects_missing_or_corrupt_live_camera_state_and_unrepresentable_clips() {
    use gz_sim_core::EntityId;
    use gz_sim_scene::Name;
    let source = include_str!("../../gz-sim-scene/tests/fixtures/camera-binding.sdf");
    let mut sim = load_world_with_sdf_step(
        source,
        "camera-binding.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::robot::body::camera")
        })
        .unwrap()
        .id();
    let id = sim.world().get::<EntityId>(entity).unwrap().get();
    let origin = [1_000_000_000.0, 0.0, 0.0];
    sim.world_mut().entity_mut(entity).insert(EntityKind::Link);
    assert_eq!(
        extract_camera(&sim, id, 1, origin),
        Err(SnapshotError::MissingCamera(id))
    );
    sim.world_mut()
        .entity_mut(entity)
        .insert(EntityKind::Sensor);
    let sensor = sim.world().get::<CameraSensor>(entity).unwrap().clone();
    sim.world_mut().entity_mut(entity).remove::<CameraSensor>();
    assert_eq!(
        extract_camera(&sim, id, 1, origin),
        Err(SnapshotError::MissingCamera(id))
    );
    sim.world_mut().entity_mut(entity).insert(sensor);
    let pose = *sim.world().get::<WorldPose>(entity).unwrap();
    sim.world_mut().entity_mut(entity).remove::<WorldPose>();
    assert_eq!(
        extract_camera(&sim, id, 1, origin),
        Err(SnapshotError::MissingCamera(id))
    );
    sim.world_mut().entity_mut(entity).insert(WorldPose {
        rotation: [0.0; 4],
        ..pose
    });
    assert_eq!(
        extract_camera(&sim, id, 1, origin),
        Err(SnapshotError::InvalidCamera(id))
    );
    sim.world_mut().entity_mut(entity).insert(pose);
    let sensor = &mut sim.world_mut().get_mut::<CameraSensor>(entity).unwrap().0;
    let SensorConfig::Camera(camera) = &mut Arc::make_mut(sensor).config else {
        panic!("camera")
    };
    camera.near_clip = 1e-300;
    camera.far_clip = 2e-300;
    assert!(gz_sim_scene::supports_camera_sensor(sensor));
    assert_eq!(
        extract_camera(&sim, id, 1, origin),
        Err(SnapshotError::InvalidCamera(id))
    );
}
