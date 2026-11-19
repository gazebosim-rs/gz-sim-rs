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
    camera.lens.projection.tx = 1.0;
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

#[test]
fn loaded_depth_extracts_a_typed_snapshot_without_scheduling_or_relabeling_rgb() {
    let xml = include_str!("../../gz-camera-output/tests/fixtures/m6-depth.sdf");
    let mut sim = load_world_with_sdf_step(
        xml,
        "m6-depth.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<CameraSensor>())
        .unwrap()
        .id();
    let id = sim
        .world()
        .get::<gz_sim_core::EntityId>(entity)
        .unwrap()
        .get();
    let counts = sim.scheduled_system_counts();
    let request = extract_camera(&sim, id, 7, [0.0; 3]).unwrap();
    assert!(request.wire_valid());
    let Request::CalibratedDepthSnapshot {
        request_id,
        stamp,
        camera,
        visuals,
        ..
    } = request
    else {
        panic!("loaded depth must retain its capture kind");
    };
    assert_eq!(request_id, 7);
    assert_eq!(stamp.epoch, 0);
    assert_eq!(camera.translation, [3.0, -2.0, 1.0]);
    assert_eq!((camera.width, camera.height), (64, 64));
    assert!((camera.fx - 32.0).abs() < 1e-12);
    assert_eq!(visuals.len(), 1);
    assert_eq!(sim.scheduled_system_counts(), counts);
    let sensor = &mut sim.world_mut().get_mut::<CameraSensor>(entity).unwrap().0;
    Arc::make_mut(sensor).kind = "camera".into();
    assert!(matches!(
        extract_camera(&sim, id, 8, [0.0; 3]).unwrap(),
        Request::CalibratedSnapshot { .. }
    ));
    assert_eq!(sim.scheduled_system_counts(), counts);
}

const CYLINDER_SCENE: &str = include_str!("../../../examples/m64_rgb_cylinder.sdf");
fn cylinder_scene() -> (gz_sim_core::Simulation, u64) {
    cylinder_source(CYLINDER_SCENE)
}
fn cylinder_source(source: &str) -> (gz_sim_core::Simulation, u64) {
    let sim = load_world_with_sdf_step(
        source,
        "m64-cylinder.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find_map(|entity| {
            entity
                .contains::<CameraSensor>()
                .then(|| entity.get::<gz_sim_core::EntityId>().unwrap().get())
        })
        .unwrap();
    (sim, id)
}
fn cylinder_visual(sim: &gz_sim_core::Simulation) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<gz_sim_scene::Name>()
                .is_some_and(|name| name.0 == "default::cylinder::body::red_cap")
        })
        .unwrap()
        .id()
}

#[test]
fn loaded_rgb_cylinder_geometry_preserves_boxes_material_and_origin_contract() {
    use gz_render_protocol::Geometry;
    use gz_sim_core::EntityId;
    use gz_sim_scene::{Name, VisualGeometry};
    let (mut sim, id) = cylinder_scene();
    let systems = sim.scheduled_system_counts();
    let visual = cylinder_visual(&sim);
    let visual_id = sim.world().get::<EntityId>(visual).unwrap().get();
    let before: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|row| {
            let pose = row.get::<WorldPose>()?;
            Some((
                row.get::<EntityId>()?.get(),
                pose.translation,
                pose.rotation,
            ))
        })
        .collect();
    let Request::CalibratedShapeSnapshot {
        request_id,
        stamp,
        origin,
        camera,
        visuals,
    } = extract_camera(&sim, id, 9, [0.0; 3]).unwrap()
    else {
        panic!("required RGB shape snapshot");
    };
    assert_eq!(request_id, 9);
    assert_eq!((stamp.epoch, stamp.seconds, stamp.nanoseconds), (0, 0, 0));
    assert_eq!(origin, [0.0; 3]);
    assert_eq!((camera.width, camera.height), (64, 64));
    assert_eq!(visuals.len(), 3);
    assert!(visuals.windows(2).all(|pair| pair[0].id < pair[1].id));
    let shape = visuals.iter().find(|row| row.id == visual_id).unwrap();
    assert_eq!(
        shape.geometry,
        Geometry::Cylinder {
            radius: 1.0,
            length: 2.0
        }
    );
    assert_eq!(shape.translation, [4.0, 0.0, 0.0]);
    assert_eq!(shape.color, [1.0, 0.0, 0.0]);
    let half = 0.5_f64.sqrt();
    assert!((shape.rotation[1] - half).abs() < 1e-12 && (shape.rotation[3] - half).abs() < 1e-12);
    assert_eq!(shape.rotation[0], 0.0);
    assert_eq!(shape.rotation[2], 0.0);
    let names: std::collections::BTreeMap<_, _> = sim
        .world()
        .iter_entities()
        .filter_map(|row| Some((row.get::<EntityId>()?.get(), row.get::<Name>()?.0.clone())))
        .collect();
    for row in visuals.iter().filter(|row| row.id != visual_id) {
        let expected = if names[&row.id] == "default::backdrop::body::blue" {
            [0.2, 6.0, 6.0]
        } else {
            assert_eq!(names[&row.id], "default::occluder::body::green");
            [0.4; 3]
        };
        assert_eq!(row.geometry, Geometry::Box { size: expected });
    }
    assert_eq!(
        before,
        sim.world()
            .iter_entities()
            .filter_map(|row| {
                let pose = row.get::<WorldPose>()?;
                Some((
                    row.get::<EntityId>()?.get(),
                    pose.translation,
                    pose.rotation,
                ))
            })
            .collect::<Vec<_>>()
    );
    assert_eq!(sim.scheduled_system_counts(), systems);
    // Full world poses remain f64; only the derivative renderer subtracts origin.
    let source = CYLINDER_SCENE
        .replace(
            "<pose>0 0 0 0 0 0</pose>",
            "<pose>1000000000 0 0 0 0 0</pose>",
        )
        .replace(
            "<pose>4 0 0 0 0 0</pose>",
            "<pose>1000000004 0 0 0 0 0</pose>",
        )
        .replace(
            "<pose>7 0 0 0 0 0</pose>",
            "<pose>1000000007 0 0 0 0 0</pose>",
        )
        .replace(
            "<pose>2 0 0 0 0 0</pose>",
            "<pose>1000000002 0 0 0 0 0</pose>",
        );
    let (shifted, shifted_id) = cylinder_source(&source);
    let Request::CalibratedShapeSnapshot {
        camera,
        origin,
        visuals,
        ..
    } = extract_camera(&shifted, shifted_id, 10, [1_000_000_000.0, 0.0, 0.0]).unwrap()
    else {
        panic!("shape");
    };
    assert_eq!(origin, [1_000_000_000.0, 0.0, 0.0]);
    assert_eq!(camera.translation[0], 1_000_000_000.0);
    let shifted_visual_id = shifted
        .world()
        .get::<EntityId>(cylinder_visual(&shifted))
        .unwrap()
        .get();
    assert_eq!(
        visuals
            .iter()
            .find(|row| row.id == shifted_visual_id)
            .unwrap()
            .translation,
        [1_000_000_004.0, 0.0, 0.0]
    );
    // An all-box world returns the exact pre-extension request form and fields.
    sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
        Arc::new(sdformat::Geometry::Box { size: [2.0; 3] });
    let Request::CalibratedSnapshot { visuals, .. } =
        extract_camera(&sim, id, 11, [1_000_000_000.0, 0.0, 0.0]).unwrap()
    else {
        panic!("legacy box form");
    };
    assert_eq!(
        visuals.iter().find(|row| row.id == visual_id).unwrap().size,
        [2.0; 3]
    );
}

#[test]
fn cylinder_rgb_refuses_invalid_dimensions_unsupported_geometry_and_material_without_mutation() {
    use gz_sim_scene::{VisualGeometry, VisualMaterial};
    let (mut sim, id) = cylinder_scene();
    let visual = cylinder_visual(&sim);
    let visual_id = sim
        .world()
        .get::<gz_sim_core::EntityId>(visual)
        .unwrap()
        .get();
    let counts = sim.scheduled_system_counts();
    let pose = *sim.world().get::<WorldPose>(visual).unwrap();
    for (radius, length) in [
        (0.0, 2.0),
        (-1.0, 2.0),
        (1.0, 0.0),
        (1.0, -1.0),
        (f64::NAN, 2.0),
        (1.0, f64::INFINITY),
        (f64::MAX, 2.0),
        (f64::MIN_POSITIVE, 2.0),
        (1.0, f64::MIN_POSITIVE),
        (f64::from(f32::MAX), 2.0),
    ] {
        sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
            Arc::new(sdformat::Geometry::Cylinder { radius, length });
        assert_eq!(
            extract_camera(&sim, id, 1, [0.0; 3]),
            Err(SnapshotError::InvalidVisual(visual_id))
        );
        assert_eq!(
            sim.world().get::<WorldPose>(visual).unwrap().translation,
            pose.translation
        );
        assert_eq!(
            sim.world().get::<WorldPose>(visual).unwrap().rotation,
            pose.rotation
        );
        assert_eq!(sim.scheduled_system_counts(), counts);
    }
    sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
        Arc::new(sdformat::Geometry::Sphere { radius: 1.0 });
    assert!(matches!(
        extract_camera(&sim, id, 1, [0.0; 3]),
        Ok(gz_render_protocol::Request::CalibratedShapeSnapshot { visuals, .. })
            if visuals.iter().any(|row| row.id == visual_id
                && row.geometry == gz_render_protocol::Geometry::Sphere { radius: 1.0 })
    ));
    sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
        Arc::new(sdformat::Geometry::Plane {
            normal: [0.0, 0.0, 1.0],
            size: [1.0; 2],
        });
    assert_eq!(
        extract_camera(&sim, id, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedGeometry(visual_id))
    );
    sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
        Arc::new(sdformat::Geometry::Cylinder {
            radius: 1.0,
            length: 2.0,
        });
    sim.world_mut()
        .get_mut::<VisualMaterial>(visual)
        .unwrap()
        .0
        .diffuse[3] = 0.5;
    assert_eq!(
        extract_camera(&sim, id, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedMaterial(visual_id))
    );
    assert_eq!(
        sim.world().get::<WorldPose>(visual).unwrap().translation,
        pose.translation
    );
    assert_eq!(
        sim.world().get::<WorldPose>(visual).unwrap().rotation,
        pose.rotation
    );
    assert_eq!(sim.scheduled_system_counts(), counts);
}

#[test]
fn cylinder_geometry_stays_refused_by_generic_and_depth_box_only_boundaries() {
    let (mut sim, id) = cylinder_scene();
    let visual = cylinder_visual(&sim);
    let visual_id = sim
        .world()
        .get::<gz_sim_core::EntityId>(visual)
        .unwrap()
        .get();
    assert_eq!(
        gz_render_scene::extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedGeometry(visual_id))
    );
    let entity = sim
        .world()
        .iter_entities()
        .find(|row| row.contains::<CameraSensor>())
        .unwrap()
        .id();
    Arc::make_mut(&mut sim.world_mut().get_mut::<CameraSensor>(entity).unwrap().0).kind =
        "depth_camera".into();
    let Request::CalibratedDepthShapeSnapshot { visuals, .. } =
        extract_camera(&sim, id, 1, [0.0; 3]).unwrap()
    else {
        panic!("curved depth admission")
    };
    assert!(visuals.iter().any(|row| row.id == visual_id
        && row.geometry
            == gz_render_protocol::Geometry::Cylinder {
                radius: 1.0,
                length: 2.0
            }));

    assert!(sim.elapsed().is_zero());
    assert_eq!(sim.epoch(), 0);
}

const SPHERE_SCENE: &str = include_str!("../../../examples/m66_rgb_sphere.sdf");
fn sphere_source(source: &str) -> (gz_sim_core::Simulation, u64) {
    let sim = load_world_with_sdf_step(
        source,
        "m66-sphere.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find_map(|row| {
            row.contains::<CameraSensor>()
                .then(|| row.get::<gz_sim_core::EntityId>().unwrap().get())
        })
        .unwrap();
    (sim, id)
}
fn sphere_visual(sim: &gz_sim_core::Simulation) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|row| {
            row.get::<gz_sim_scene::Name>()
                .is_some_and(|name| name.0 == "default::sphere::body::red_ball")
        })
        .unwrap()
        .id()
}

#[test]
fn sphere_rgb_extracts_mixed_shapes_and_preserves_local_pose_origin_and_legacy_boxes() {
    use gz_render_protocol::Geometry;
    use gz_sim_core::EntityId;
    use gz_sim_scene::{Name, VisualGeometry};
    let posed = SPHERE_SCENE.replace(
        "<visual name=\"red_ball\">",
        "<visual name=\"red_ball\"><pose>0.25 0.5 -0.25 0 0 1.5707963267948966</pose>",
    );
    assert_ne!(posed, SPHERE_SCENE);
    let (mut sim, id) = sphere_source(&posed);
    let visual = sphere_visual(&sim);
    let visual_id = sim.world().get::<EntityId>(visual).unwrap().get();
    let occluder = sim
        .world()
        .iter_entities()
        .find(|row| {
            row.get::<Name>()
                .is_some_and(|name| name.0 == "default::occluder::body::green")
        })
        .unwrap()
        .id();
    sim.world_mut()
        .get_mut::<VisualGeometry>(occluder)
        .unwrap()
        .0 = Arc::new(sdformat::Geometry::Cylinder {
        radius: 0.2,
        length: 0.4,
    });
    let before = sim.scheduled_system_counts();
    let elapsed = sim.elapsed();
    let epoch = sim.epoch();
    let pose = *sim.world().get::<WorldPose>(visual).unwrap();
    let request = extract_camera(&sim, id, 7, [0.0; 3]).unwrap();
    assert!(request.wire_valid());
    let Request::CalibratedShapeSnapshot {
        visuals, camera, ..
    } = request
    else {
        panic!("RGB shape snapshot")
    };
    assert_eq!((camera.width, camera.height), (64, 64));
    assert_eq!(visuals.len(), 3);
    assert!(visuals.windows(2).all(|pair| pair[0].id < pair[1].id));
    assert!(
        visuals
            .iter()
            .any(|row| matches!(row.geometry, Geometry::Box { .. }))
    );
    assert!(
        visuals
            .iter()
            .any(|row| matches!(row.geometry, Geometry::Cylinder { .. }))
    );
    let sphere = visuals.iter().find(|row| row.id == visual_id).unwrap();
    assert_eq!(sphere.geometry, Geometry::Sphere { radius: 1.0 });
    assert_eq!(sphere.translation, [4.25, 0.5, -0.25]);
    assert_eq!(sphere.color, [1.0, 0.0, 0.0]);
    assert!(
        (sphere.rotation[2] - 0.5_f64.sqrt()).abs() < 1e-12
            && (sphere.rotation[3] - 0.5_f64.sqrt()).abs() < 1e-12
    );
    assert_eq!(sphere.rotation[0], 0.0);
    assert_eq!(sphere.rotation[1], 0.0);
    assert_eq!(
        sim.world().get::<WorldPose>(visual).unwrap().translation,
        pose.translation
    );
    assert_eq!(
        sim.world().get::<WorldPose>(visual).unwrap().rotation,
        pose.rotation
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
        (elapsed, epoch, before)
    );
    let source = SPHERE_SCENE
        .replace(
            "<pose>0 0 0 0 0 0</pose>",
            "<pose>1000000000 0 0 0 0 0</pose>",
        )
        .replace(
            "<pose>4 0 0 0 0 0</pose>",
            "<pose>1000000004 0 0 0 0 0</pose>",
        )
        .replace(
            "<pose>7 0 0 0 0 0</pose>",
            "<pose>1000000007 0 0 0 0 0</pose>",
        )
        .replace(
            "<pose>2 0 0 0 0 0</pose>",
            "<pose>1000000002 0 0 0 0 0</pose>",
        );
    let (mut shifted, shifted_id) = sphere_source(&source);
    let Request::CalibratedShapeSnapshot {
        origin,
        camera,
        visuals,
        ..
    } = extract_camera(&shifted, shifted_id, 8, [1e9, 0.0, 0.0]).unwrap()
    else {
        panic!("shifted sphere")
    };
    assert_eq!(origin, [1e9, 0.0, 0.0]);
    assert_eq!(camera.translation, [1e9, 0.0, 0.0]);
    assert!(
        visuals
            .iter()
            .any(|row| row.geometry == Geometry::Sphere { radius: 1.0 }
                && row.translation == [1e9 + 4.0, 0.0, 0.0])
    );
    let sphere = sphere_visual(&shifted);
    shifted
        .world_mut()
        .get_mut::<VisualGeometry>(sphere)
        .unwrap()
        .0 = Arc::new(sdformat::Geometry::Box { size: [2.0; 3] });
    let legacy = extract_camera(&shifted, shifted_id, 9, [1e9, 0.0, 0.0]).unwrap();
    assert!(matches!(legacy, Request::CalibratedSnapshot { .. }));
    // A shape round trip restores the exact legacy request form and bytes.
    shifted
        .world_mut()
        .get_mut::<VisualGeometry>(sphere)
        .unwrap()
        .0 = Arc::new(sdformat::Geometry::Sphere { radius: 1.0 });
    assert!(matches!(
        extract_camera(&shifted, shifted_id, 10, [1e9, 0.0, 0.0]).unwrap(),
        Request::CalibratedShapeSnapshot { .. }
    ));
    shifted
        .world_mut()
        .get_mut::<VisualGeometry>(sphere)
        .unwrap()
        .0 = Arc::new(sdformat::Geometry::Box { size: [2.0; 3] });
    let restored = extract_camera(&shifted, shifted_id, 9, [1e9, 0.0, 0.0]).unwrap();
    let mut old_wire = Vec::new();
    let mut restored_wire = Vec::new();
    gz_render_protocol::write_request(&mut old_wire, &legacy).unwrap();
    gz_render_protocol::write_request(&mut restored_wire, &restored).unwrap();
    assert_eq!(old_wire, restored_wire);
}

#[test]
fn sphere_rgb_refuses_invalid_radius_material_and_depth_without_mutation() {
    use gz_sim_scene::{VisualGeometry, VisualMaterial};
    let (mut sim, id) = sphere_source(SPHERE_SCENE);
    let visual = sphere_visual(&sim);
    let visual_id = sim
        .world()
        .get::<gz_sim_core::EntityId>(visual)
        .unwrap()
        .get();
    let systems = sim.scheduled_system_counts();
    let pose = *sim.world().get::<WorldPose>(visual).unwrap();
    // The consumed SDF parser normalizes some bad raw radii. These faults target
    // the canonical component/API boundary, without changing that parser policy.
    for radius in [
        0.0,
        -1.0,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::MAX,
        f64::MIN_POSITIVE,
        f64::from(f32::MAX),
    ] {
        sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
            Arc::new(sdformat::Geometry::Sphere { radius });
        assert_eq!(
            extract_camera(&sim, id, 1, [0.0; 3]),
            Err(SnapshotError::InvalidVisual(visual_id))
        );
        assert_eq!(
            sim.world().get::<WorldPose>(visual).unwrap().translation,
            pose.translation
        );
        assert_eq!(
            sim.world().get::<WorldPose>(visual).unwrap().rotation,
            pose.rotation
        );
        assert_eq!(sim.scheduled_system_counts(), systems);
        assert_eq!((sim.elapsed(), sim.epoch()), (std::time::Duration::ZERO, 0));
    }
    sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
        Arc::new(sdformat::Geometry::Sphere { radius: 1.0 });
    assert_eq!(
        gz_render_scene::extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedGeometry(visual_id))
    );
    let sensor = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
    Arc::make_mut(&mut sim.world_mut().get_mut::<CameraSensor>(sensor).unwrap().0).kind =
        "depth_camera".into();
    let Request::CalibratedDepthShapeSnapshot { visuals, .. } =
        extract_camera(&sim, id, 1, [0.0; 3]).unwrap()
    else {
        panic!("curved depth admission")
    };
    assert!(visuals.iter().any(|row| row.id == visual_id
        && row.geometry == gz_render_protocol::Geometry::Sphere { radius: 1.0 }));

    Arc::make_mut(&mut sim.world_mut().get_mut::<CameraSensor>(sensor).unwrap().0).kind =
        "camera".into();
    sim.world_mut()
        .get_mut::<VisualMaterial>(visual)
        .unwrap()
        .0
        .diffuse[3] = 0.5;
    assert_eq!(
        extract_camera(&sim, id, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedMaterial(visual_id))
    );
    assert_eq!(sim.scheduled_system_counts(), systems);
    assert_eq!((sim.elapsed(), sim.epoch()), (std::time::Duration::ZERO, 0));
}

#[test]
fn sphere_rgb_shapes_keep_shared_identity_and_visual_bound() {
    use gz_sim_scene::{VisualGeometry, VisualMaterial, VisualProperties};
    let (mut sim, id) = sphere_source(SPHERE_SCENE);
    let visual = sphere_visual(&sim);
    let material = sim.world().get::<VisualMaterial>(visual).unwrap().clone();
    let properties = sim.world().get::<VisualProperties>(visual).unwrap().clone();
    let pose = *sim.world().get::<WorldPose>(visual).unwrap();
    for n in 0..125 {
        let geometry = match n % 3 {
            0 => sdformat::Geometry::Sphere { radius: 0.5 },
            1 => sdformat::Geometry::Cylinder {
                radius: 0.5,
                length: 1.0,
            },
            _ => sdformat::Geometry::Box { size: [1.0; 3] },
        };
        let entity = sim
            .world_mut()
            .spawn((
                EntityKind::Visual,
                VisualGeometry(Arc::new(geometry)),
                material.clone(),
                properties.clone(),
                pose,
            ))
            .id();
        assign_id(sim.world_mut(), entity).unwrap();
    }
    let Request::CalibratedShapeSnapshot { visuals, .. } =
        extract_camera(&sim, id, 1, [0.0; 3]).unwrap()
    else {
        panic!("mixed128")
    };
    assert_eq!(visuals.len(), 128);
    assert!(visuals.windows(2).all(|p| p[0].id < p[1].id));
    let extra = sim
        .world_mut()
        .spawn((
            EntityKind::Visual,
            VisualGeometry(Arc::new(sdformat::Geometry::Sphere { radius: 1.0 })),
            material,
            properties,
            pose,
        ))
        .id();
    assign_id(sim.world_mut(), extra).unwrap();
    assert_eq!(
        extract_camera(&sim, id, 2, [0.0; 3]),
        Err(SnapshotError::TooManyVisuals)
    );
    sim.world_mut().despawn(extra);
    sim.world_mut()
        .entity_mut(visual)
        .remove::<gz_sim_core::EntityId>();
    assert_eq!(
        extract_camera(&sim, id, 3, [0.0; 3]),
        Err(SnapshotError::InvalidIdentity)
    );
}

const ELLIPSOID_SCENE: &str = include_str!("../../../examples/m67_rgb_ellipsoid.sdf");
fn ellipsoid_source(source: &str) -> (gz_sim_core::Simulation, u64) {
    let sim = load_world_with_sdf_step(
        source,
        "m67-ellipsoid.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find_map(|row| {
            row.contains::<CameraSensor>()
                .then(|| row.get::<gz_sim_core::EntityId>().unwrap().get())
        })
        .unwrap();
    (sim, id)
}
fn ellipsoid_visual(sim: &gz_sim_core::Simulation) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|row| {
            row.get::<gz_sim_scene::Name>()
                .is_some_and(|name| name.0 == "default::ellipsoid::body::red_ellipsoid")
        })
        .unwrap()
        .id()
}

#[test]
fn ellipsoid_rgb_preserves_local_semiaxes_pose_origin_and_exact_legacy_box_request() {
    use gz_render_protocol::Geometry;
    use gz_sim_core::EntityId;
    use gz_sim_scene::{Name, VisualGeometry, VisualMaterial, VisualProperties};
    let posed = ELLIPSOID_SCENE.replace(
        "<visual name=\"red_ellipsoid\">",
        "<visual name=\"red_ellipsoid\"><pose>0.25 0.5 -0.25 1.5707963267948966 0 0</pose>",
    );
    assert_ne!(posed, ELLIPSOID_SCENE);
    let (mut sim, id) = ellipsoid_source(&posed);
    let visual = ellipsoid_visual(&sim);
    let visual_id = sim.world().get::<EntityId>(visual).unwrap().get();
    let pose = *sim.world().get::<WorldPose>(visual).unwrap();
    let material = sim.world().get::<VisualMaterial>(visual).unwrap().clone();
    let properties = sim.world().get::<VisualProperties>(visual).unwrap().clone();
    let occluder = sim
        .world()
        .iter_entities()
        .find(|row| {
            row.get::<Name>()
                .is_some_and(|name| name.0 == "default::occluder::body::green")
        })
        .unwrap()
        .id();
    sim.world_mut()
        .get_mut::<VisualGeometry>(occluder)
        .unwrap()
        .0 = Arc::new(sdformat::Geometry::Cylinder {
        radius: 0.2,
        length: 0.4,
    });
    let extra = sim
        .world_mut()
        .spawn((
            EntityKind::Visual,
            VisualGeometry(Arc::new(sdformat::Geometry::Sphere { radius: 0.2 })),
            material,
            properties,
            pose,
        ))
        .id();
    assign_id(sim.world_mut(), extra).unwrap();
    let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
    let request = extract_camera(&sim, id, 7, [0.0; 3]).unwrap();
    assert!(request.wire_valid());
    let Request::CalibratedShapeSnapshot {
        visuals, camera, ..
    } = request
    else {
        panic!("ellipsoid RGB shape snapshot")
    };
    assert_eq!((camera.width, camera.height), (64, 64));
    assert_eq!(visuals.len(), 4);
    assert!(visuals.windows(2).all(|rows| rows[0].id < rows[1].id));
    assert!(
        visuals
            .iter()
            .any(|row| matches!(row.geometry, Geometry::Box { .. }))
    );
    assert!(
        visuals
            .iter()
            .any(|row| matches!(row.geometry, Geometry::Cylinder { .. }))
    );
    assert!(
        visuals
            .iter()
            .any(|row| matches!(row.geometry, Geometry::Sphere { .. }))
    );
    let ellipsoid = visuals.iter().find(|row| row.id == visual_id).unwrap();
    assert_eq!(
        ellipsoid.geometry,
        Geometry::Ellipsoid {
            radii: [0.5, 1.5, 0.75]
        }
    );
    assert_eq!(ellipsoid.translation, [4.25, 0.5, -0.25]);
    assert_eq!(ellipsoid.color, [1.0, 0.0, 0.0]);
    assert!((ellipsoid.rotation[0] - 0.5_f64.sqrt()).abs() < 1e-12);
    assert_eq!((ellipsoid.rotation[1], ellipsoid.rotation[2]), (0.0, 0.0));
    assert!((ellipsoid.rotation[3] - 0.5_f64.sqrt()).abs() < 1e-12);
    let after = sim.world().get::<WorldPose>(visual).unwrap();
    assert_eq!(
        (after.translation, after.rotation),
        (pose.translation, pose.rotation)
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
        before
    );

    let source = ELLIPSOID_SCENE
        .replace(
            "<pose>0 0 0 0 0 0</pose>",
            "<pose>1000000000 0 0 0 0 0</pose>",
        )
        .replace(
            "<pose>4 0 0 0 0 0</pose>",
            "<pose>1000000004 0 0 0 0 0</pose>",
        )
        .replace(
            "<pose>7 0 0 0 0 0</pose>",
            "<pose>1000000007 0 0 0 0 0</pose>",
        )
        .replace(
            "<pose>2 0 0 0 0 0</pose>",
            "<pose>1000000002 0 0 0 0 0</pose>",
        );
    let (mut shifted, shifted_id) = ellipsoid_source(&source);
    let Request::CalibratedShapeSnapshot {
        origin,
        camera,
        visuals,
        ..
    } = extract_camera(&shifted, shifted_id, 8, [1e9, 0.0, 0.0]).unwrap()
    else {
        panic!("shifted ellipsoid")
    };
    assert_eq!(origin, [1e9, 0.0, 0.0]);
    assert_eq!(camera.translation, [1e9, 0.0, 0.0]);
    assert!(visuals.iter().any(|row| row.geometry
        == Geometry::Ellipsoid {
            radii: [0.5, 1.5, 0.75]
        }
        && row.translation == [1e9 + 4.0, 0.0, 0.0]));
    let visual = ellipsoid_visual(&shifted);
    shifted
        .world_mut()
        .get_mut::<VisualGeometry>(visual)
        .unwrap()
        .0 = Arc::new(sdformat::Geometry::Box { size: [2.0; 3] });
    let legacy = extract_camera(&shifted, shifted_id, 9, [1e9, 0.0, 0.0]).unwrap();
    assert!(matches!(legacy, Request::CalibratedSnapshot { .. }));
    shifted
        .world_mut()
        .get_mut::<VisualGeometry>(visual)
        .unwrap()
        .0 = Arc::new(sdformat::Geometry::Ellipsoid {
        radii: [0.5, 1.5, 0.75],
    });
    assert!(matches!(
        extract_camera(&shifted, shifted_id, 10, [1e9, 0.0, 0.0]).unwrap(),
        Request::CalibratedShapeSnapshot { .. }
    ));
    shifted
        .world_mut()
        .get_mut::<VisualGeometry>(visual)
        .unwrap()
        .0 = Arc::new(sdformat::Geometry::Box { size: [2.0; 3] });
    let restored = extract_camera(&shifted, shifted_id, 9, [1e9, 0.0, 0.0]).unwrap();
    let (mut old_wire, mut new_wire) = (Vec::new(), Vec::new());
    gz_render_protocol::write_request(&mut old_wire, &legacy).unwrap();
    gz_render_protocol::write_request(&mut new_wire, &restored).unwrap();
    assert_eq!(old_wire, new_wire);
}

#[test]
fn ellipsoid_rgb_refuses_each_invalid_axis_material_and_depth_before_mutation() {
    use gz_sim_scene::{VisualGeometry, VisualMaterial};
    let (mut sim, id) = ellipsoid_source(ELLIPSOID_SCENE);
    let visual = ellipsoid_visual(&sim);
    let visual_id = sim
        .world()
        .get::<gz_sim_core::EntityId>(visual)
        .unwrap()
        .get();
    let pose = *sim.world().get::<WorldPose>(visual).unwrap();
    let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
    // These are canonical component faults; XML nonpositive triples normalize.
    for axis in 0..3 {
        for invalid in [
            0.0,
            -1.0,
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::MAX,
            f64::MIN_POSITIVE,
            f64::from(f32::MAX),
        ] {
            let mut radii = [0.5, 1.5, 0.75];
            radii[axis] = invalid;
            sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
                Arc::new(sdformat::Geometry::Ellipsoid { radii });
            assert_eq!(
                extract_camera(&sim, id, 1, [0.0; 3]),
                Err(SnapshotError::InvalidVisual(visual_id))
            );
            let after = sim.world().get::<WorldPose>(visual).unwrap();
            assert_eq!(
                (after.translation, after.rotation),
                (pose.translation, pose.rotation)
            );
            assert_eq!(
                (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
                before
            );
        }
    }
    sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
        Arc::new(sdformat::Geometry::Ellipsoid {
            radii: [0.5, 1.5, 0.75],
        });
    assert_eq!(
        gz_render_scene::extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedGeometry(visual_id))
    );
    let sensor = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
    Arc::make_mut(&mut sim.world_mut().get_mut::<CameraSensor>(sensor).unwrap().0).kind =
        "depth_camera".into();
    let Request::CalibratedDepthShapeSnapshot { visuals, .. } =
        extract_camera(&sim, id, 1, [0.0; 3]).unwrap()
    else {
        panic!("curved depth admission")
    };
    assert!(visuals.iter().any(|row| row.id == visual_id
        && row.geometry
            == gz_render_protocol::Geometry::Ellipsoid {
                radii: [0.5, 1.5, 0.75]
            }));

    Arc::make_mut(&mut sim.world_mut().get_mut::<CameraSensor>(sensor).unwrap().0).kind =
        "camera".into();
    sim.world_mut()
        .get_mut::<VisualMaterial>(visual)
        .unwrap()
        .0
        .diffuse[3] = 0.5;
    assert_eq!(
        extract_camera(&sim, id, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedMaterial(visual_id))
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
        before
    );
}

#[test]
fn ellipsoid_rgb_mixed_shapes_share_128_ids_and_reject_missing_identity() {
    use gz_sim_scene::{VisualGeometry, VisualMaterial, VisualProperties};
    let (mut sim, id) = ellipsoid_source(ELLIPSOID_SCENE);
    let visual = ellipsoid_visual(&sim);
    let material = sim.world().get::<VisualMaterial>(visual).unwrap().clone();
    let properties = sim.world().get::<VisualProperties>(visual).unwrap().clone();
    let pose = *sim.world().get::<WorldPose>(visual).unwrap();
    for n in 0..125 {
        let geometry = match n % 4 {
            0 => sdformat::Geometry::Ellipsoid {
                radii: [0.2, 0.3, 0.4],
            },
            1 => sdformat::Geometry::Sphere { radius: 0.2 },
            2 => sdformat::Geometry::Cylinder {
                radius: 0.2,
                length: 0.4,
            },
            _ => sdformat::Geometry::Box { size: [0.4; 3] },
        };
        let entity = sim
            .world_mut()
            .spawn((
                EntityKind::Visual,
                VisualGeometry(Arc::new(geometry)),
                material.clone(),
                properties.clone(),
                pose,
            ))
            .id();
        assign_id(sim.world_mut(), entity).unwrap();
    }
    let Request::CalibratedShapeSnapshot { visuals, .. } =
        extract_camera(&sim, id, 1, [0.0; 3]).unwrap()
    else {
        panic!("mixed128")
    };
    assert_eq!(visuals.len(), 128);
    assert!(visuals.windows(2).all(|rows| rows[0].id < rows[1].id));
    let extra = sim
        .world_mut()
        .spawn((
            EntityKind::Visual,
            VisualGeometry(Arc::new(sdformat::Geometry::Ellipsoid {
                radii: [0.2, 0.3, 0.4],
            })),
            material,
            properties,
            pose,
        ))
        .id();
    assign_id(sim.world_mut(), extra).unwrap();
    assert_eq!(
        extract_camera(&sim, id, 2, [0.0; 3]),
        Err(SnapshotError::TooManyVisuals)
    );
    sim.world_mut().despawn(extra);
    let (mut small, small_id) = ellipsoid_source(ELLIPSOID_SCENE);
    let actual = ellipsoid_visual(&small);
    small
        .world_mut()
        .entity_mut(actual)
        .remove::<gz_sim_core::EntityId>();
    assert_eq!(
        extract_camera(&small, small_id, 3, [0.0; 3]),
        Err(SnapshotError::InvalidIdentity)
    );
    assign_id(small.world_mut(), actual).unwrap();
    assert!(extract_camera(&small, small_id, 4, [0.0; 3]).is_ok());
}

#[test]
fn ellipsoid_source_parser_defaults_and_normalizes_whole_triple_before_rgb_admission() {
    use gz_sim_scene::VisualGeometry;
    for dimensions in [
        "",
        "<radii>0 1.5 0.75</radii>",
        "<radii>0.5 -1 0.75</radii>",
        "<radii>0.5 1.5 0</radii>",
    ] {
        let source = ELLIPSOID_SCENE.replace("<radii>0.5 1.5 0.75</radii>", dimensions);
        let (sim, id) = ellipsoid_source(&source);
        assert!(
            matches!(sim.world().get::<VisualGeometry>(ellipsoid_visual(&sim)).unwrap().0.as_ref(), sdformat::Geometry::Ellipsoid { radii } if *radii == [1.0; 3])
        );
        let Request::CalibratedShapeSnapshot { visuals, .. } =
            extract_camera(&sim, id, 1, [0.0; 3]).unwrap()
        else {
            panic!("canonical unit ellipsoid")
        };
        assert!(
            visuals
                .iter()
                .any(|row| row.geometry
                    == gz_render_protocol::Geometry::Ellipsoid { radii: [1.0; 3] })
        );
    }
    for invalid in ["1 2", "1 2 3 4", "NaN 1 1", "1 inf 1", "1 1 bad", ""] {
        let source = ELLIPSOID_SCENE.replace(
            "<radii>0.5 1.5 0.75</radii>",
            &format!("<radii>{invalid}</radii>"),
        );
        assert!(
            load_world_with_sdf_step(
                &source,
                "m67-invalid-radii.sdf",
                "default",
                &NoResources,
                PhysicsEngine::Tpe
            )
            .is_err()
        );
    }
}

const DEPTH_PRIMITIVES_SCENE: &str = include_str!("../../../examples/m68_depth_primitives.sdf");

#[test]
fn calibrated_depth_primitives_preserve_all_local_shapes_and_exact_box_wire_round_trip() {
    use gz_render_protocol::Geometry;
    use gz_sim_scene::VisualGeometry;
    let (mut sim, id) = ellipsoid_source(DEPTH_PRIMITIVES_SCENE);
    let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
    let request = extract_camera(&sim, id, 1, [0.0; 3]).unwrap();
    assert!(request.wire_valid());
    let Request::CalibratedDepthShapeSnapshot {
        stamp,
        visuals,
        camera,
        ..
    } = request
    else {
        panic!("mixed depth request")
    };
    assert_eq!((stamp.epoch, stamp.seconds, stamp.nanoseconds), (0, 0, 0));
    assert_eq!((camera.width, camera.height), (64, 64));
    assert_eq!(visuals.len(), 5);
    assert!(visuals.windows(2).all(|rows| rows[0].id < rows[1].id));
    assert_eq!(
        visuals
            .iter()
            .filter(|row| matches!(row.geometry, Geometry::Box { .. }))
            .count(),
        2
    );
    let cylinder = visuals
        .iter()
        .find(|row| {
            row.geometry
                == Geometry::Cylinder {
                    radius: 0.5,
                    length: 1.0,
                }
        })
        .unwrap();
    assert_eq!(cylinder.translation, [4.0, -1.5, 0.0]);
    assert!((cylinder.rotation[1] - 0.5_f64.sqrt()).abs() < 1e-12);
    assert!((cylinder.rotation[3] - 0.5_f64.sqrt()).abs() < 1e-12);
    assert!(
        visuals
            .iter()
            .any(|row| row.geometry == Geometry::Sphere { radius: 0.75 }
                && row.translation == [4.0, 0.0, 0.0])
    );
    assert!(visuals.iter().any(|row| row.geometry
        == Geometry::Ellipsoid {
            radii: [0.5, 0.6, 0.35]
        }
        && row.translation == [4.0, 1.5, 0.0]));
    // Generic extraction remains unchanged even though calibrated depth admits shapes.
    assert!(matches!(
        gz_render_scene::extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedGeometry(_))
    ));
    let entities: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|row| row.contains::<VisualGeometry>())
        .map(|row| row.id())
        .collect();
    let originals: Vec<_> = entities
        .iter()
        .map(|entity| sim.world().get::<VisualGeometry>(*entity).unwrap().clone())
        .collect();
    for entity in &entities {
        sim.world_mut()
            .get_mut::<VisualGeometry>(*entity)
            .unwrap()
            .0 = Arc::new(sdformat::Geometry::Box { size: [1.0; 3] });
    }
    let old = extract_camera(&sim, id, 2, [0.0; 3]).unwrap();
    assert!(matches!(old, Request::CalibratedDepthSnapshot { .. }));
    for (entity, original) in entities.iter().zip(&originals) {
        sim.world_mut()
            .get_mut::<VisualGeometry>(*entity)
            .unwrap()
            .0 = Arc::clone(&original.0);
    }
    assert!(matches!(
        extract_camera(&sim, id, 3, [0.0; 3]).unwrap(),
        Request::CalibratedDepthShapeSnapshot { .. }
    ));
    for entity in &entities {
        sim.world_mut()
            .get_mut::<VisualGeometry>(*entity)
            .unwrap()
            .0 = Arc::new(sdformat::Geometry::Box { size: [1.0; 3] });
    }
    let restored = extract_camera(&sim, id, 2, [0.0; 3]).unwrap();
    let (mut old_bytes, mut restored_bytes) = (Vec::new(), Vec::new());
    gz_render_protocol::write_request(&mut old_bytes, &old).unwrap();
    gz_render_protocol::write_request(&mut restored_bytes, &restored).unwrap();
    assert_eq!(old_bytes, restored_bytes);
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
        before
    );
}

#[test]
fn depth_primitives_keep_semantic_faults_unsupported_shapes_and_shared_128_identity_bound() {
    use gz_sim_scene::{VisualGeometry, VisualMaterial, VisualProperties};
    let (mut sim, id) = ellipsoid_source(DEPTH_PRIMITIVES_SCENE);
    let visual = ellipsoid_visual(&sim);
    let visual_id = sim
        .world()
        .get::<gz_sim_core::EntityId>(visual)
        .unwrap()
        .get();
    let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
    for axis in 0..3 {
        for value in [
            0.0,
            -1.0,
            f64::NAN,
            f64::INFINITY,
            f64::MAX,
            f64::MIN_POSITIVE,
            f64::from(f32::MAX),
        ] {
            let mut radii = [0.5, 0.6, 0.35];
            radii[axis] = value;
            sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
                Arc::new(sdformat::Geometry::Ellipsoid { radii });
            assert_eq!(
                extract_camera(&sim, id, 1, [0.0; 3]),
                Err(SnapshotError::InvalidVisual(visual_id))
            );
            assert_eq!(
                (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
                before
            );
        }
    }
    for geometry in [
        sdformat::Geometry::Sphere { radius: 0.0 },
        sdformat::Geometry::Cylinder {
            radius: 1.0,
            length: 0.0,
        },
    ] {
        sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 = Arc::new(geometry);
        assert_eq!(
            extract_camera(&sim, id, 1, [0.0; 3]),
            Err(SnapshotError::InvalidVisual(visual_id))
        );
    }
    sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
        Arc::new(sdformat::Geometry::Capsule {
            radius: 0.5,
            length: 1.0,
        });
    assert_eq!(
        extract_camera(&sim, id, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedGeometry(visual_id))
    );
    sim.world_mut().get_mut::<VisualGeometry>(visual).unwrap().0 =
        Arc::new(sdformat::Geometry::Ellipsoid {
            radii: [0.5, 0.6, 0.35],
        });
    let material = sim.world().get::<VisualMaterial>(visual).unwrap().clone();
    let properties = sim.world().get::<VisualProperties>(visual).unwrap().clone();
    let pose = *sim.world().get::<WorldPose>(visual).unwrap();
    for _ in 0..123 {
        let entity = sim
            .world_mut()
            .spawn((
                EntityKind::Visual,
                VisualGeometry(Arc::new(sdformat::Geometry::Sphere { radius: 0.1 })),
                material.clone(),
                properties.clone(),
                pose,
            ))
            .id();
        assign_id(sim.world_mut(), entity).unwrap();
    }
    let Request::CalibratedDepthShapeSnapshot { visuals, .. } =
        extract_camera(&sim, id, 2, [0.0; 3]).unwrap()
    else {
        panic!("128 depth visuals")
    };
    assert_eq!(visuals.len(), 128);
    assert!(visuals.windows(2).all(|rows| rows[0].id < rows[1].id));
    let extra = sim
        .world_mut()
        .spawn((
            EntityKind::Visual,
            VisualGeometry(Arc::new(sdformat::Geometry::Sphere { radius: 0.1 })),
            material,
            properties,
            pose,
        ))
        .id();
    assign_id(sim.world_mut(), extra).unwrap();
    assert_eq!(
        extract_camera(&sim, id, 3, [0.0; 3]),
        Err(SnapshotError::TooManyVisuals)
    );
    sim.world_mut().despawn(extra);
    sim.world_mut()
        .entity_mut(visual)
        .remove::<gz_sim_core::EntityId>();
    assert_eq!(
        extract_camera(&sim, id, 4, [0.0; 3]),
        Err(SnapshotError::InvalidIdentity)
    );
    assert_eq!(
        (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
        before
    );
}

fn intrinsics_source(kind: &str, shapes: bool) -> String {
    let source = ELLIPSOID_SCENE.replace("type=\"camera\"", &format!("type=\"{kind}\""));
    if shapes {
        source
    } else {
        source.replace(
            "<ellipsoid><radii>0.5 1.5 0.75</radii></ellipsoid>",
            "<box><size>1 3 1.5</size></box>",
        )
    }
}

fn request_camera(request: &Request) -> &gz_render_protocol::Camera {
    match request {
        Request::CalibratedSnapshot { camera, .. }
        | Request::CalibratedShapeSnapshot { camera, .. }
        | Request::CalibratedDepthSnapshot { camera, .. }
        | Request::CalibratedDepthShapeSnapshot { camera, .. } => camera,
        _ => panic!("calibrated camera request"),
    }
}

#[test]
fn explicit_intrinsics_select_rgb_depth_box_and_shape_request_calibration() {
    let lens = "<lens><intrinsics><fx>40</fx><fy>41</fy><cx>20</cx><cy>21</cy><s>0</s></intrinsics></lens>";
    for kind in ["camera", "depth_camera"] {
        for shapes in [false, true] {
            let source =
                intrinsics_source(kind, shapes).replace("</camera>", &format!("{lens}</camera>"));
            let (mut sim, id) = ellipsoid_source(&source);
            sim.step_exact(1).unwrap();
            let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
            let request = extract_camera(&sim, id, 7, [0.0; 3]).unwrap();
            let stamp = match (&request, kind, shapes) {
                (Request::CalibratedSnapshot { stamp, .. }, "camera", false)
                | (Request::CalibratedShapeSnapshot { stamp, .. }, "camera", true)
                | (Request::CalibratedDepthSnapshot { stamp, .. }, "depth_camera", false)
                | (Request::CalibratedDepthShapeSnapshot { stamp, .. }, "depth_camera", true) => {
                    stamp
                }
                _ => panic!("wrong calibrated request family"),
            };
            assert_eq!(
                (stamp.epoch, stamp.seconds, stamp.nanoseconds),
                (0, 0, 1_000_000)
            );
            assert_eq!(
                request_camera(&request),
                &gz_render_protocol::Camera {
                    width: 64,
                    height: 64,
                    fx: 40.0,
                    fy: 41.0,
                    cx: 20.0,
                    cy: 21.0,
                    skew: 0.0,
                    tx: 0.0,
                    ty: 0.0,
                    near: 0.1,
                    far: 10.0,
                    translation: [0.0; 3],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                }
            );
            assert!(request.wire_valid());
            let mut bytes = Vec::new();
            gz_render_protocol::write_request(&mut bytes, &request).unwrap();
            assert_eq!(
                gz_render_protocol::read_request(&mut bytes.as_slice()).unwrap(),
                Some(request)
            );
            assert_eq!(before.0, std::time::Duration::from_millis(1));
            assert_eq!(
                (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
                before
            );
        }
    }
}

#[test]
fn absent_intrinsics_ignore_dormant_lens_values_and_preserve_exact_request_bytes() {
    for kind in ["camera", "depth_camera"] {
        for shapes in [false, true] {
            let (mut sim, id) = ellipsoid_source(&intrinsics_source(kind, shapes));
            let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
            let original = extract_camera(&sim, id, 1, [0.0; 3]).unwrap();
            let selected = request_camera(&original);
            assert!((selected.fx - 32.0).abs() < 1e-12);
            assert_eq!(selected.fx, selected.fy);
            assert_eq!((selected.cx, selected.cy), (32.0, 32.0));
            let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
            {
                let mut sensor = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
                let SensorConfig::Camera(camera) = &mut Arc::make_mut(&mut sensor.0).config else {
                    panic!("camera metadata")
                };
                assert!(!camera.lens.has_intrinsics && !camera.lens.has_projection);
                camera.lens.intrinsics.fx = f64::NAN;
                camera.lens.intrinsics.fy = -1.0;
                camera.lens.intrinsics.cx = f64::INFINITY;
                camera.lens.intrinsics.cy = f64::NEG_INFINITY;
                camera.lens.intrinsics.skew = 1.0;
                camera.lens.projection.fx = 9.0;
                camera.lens.projection.tx = 5.0;
            }
            let dormant = extract_camera(&sim, id, 1, [0.0; 3]).unwrap();
            let (mut original_bytes, mut dormant_bytes) = (Vec::new(), Vec::new());
            gz_render_protocol::write_request(&mut original_bytes, &original).unwrap();
            gz_render_protocol::write_request(&mut dormant_bytes, &dormant).unwrap();
            assert_eq!(original_bytes, dormant_bytes);
            assert_eq!(original, dormant);
            assert_eq!(
                (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
                before
            );
        }
    }
}

#[test]
fn explicit_intrinsics_refuse_malformed_and_unrepresentable_calibration_without_mutation() {
    let invalid = [
        ([f64::NAN, 41.0, 20.0, 21.0, 0.0], false),
        ([f64::INFINITY, 41.0, 20.0, 21.0, 0.0], false),
        ([0.0, 41.0, 20.0, 21.0, 0.0], false),
        ([-1.0, 41.0, 20.0, 21.0, 0.0], false),
        ([40.0, f64::NAN, 20.0, 21.0, 0.0], false),
        ([40.0, f64::INFINITY, 20.0, 21.0, 0.0], false),
        ([40.0, 0.0, 20.0, 21.0, 0.0], false),
        ([40.0, -1.0, 20.0, 21.0, 0.0], false),
        ([40.0, 41.0, f64::NAN, 21.0, 0.0], false),
        ([40.0, 41.0, f64::INFINITY, 21.0, 0.0], false),
        ([40.0, 41.0, 20.0, f64::NAN, 0.0], false),
        ([40.0, 41.0, 20.0, f64::INFINITY, 0.0], false),
        ([40.0, 41.0, 20.0, 21.0, f64::NAN], false),
        ([40.0, 41.0, 20.0, 21.0, 0.1], false),
        ([f64::MAX, 41.0, 20.0, 21.0, 0.0], true),
        ([40.0, f64::MIN_POSITIVE, 20.0, 21.0, 0.0], true),
        ([40.0, 41.0, f64::MAX, 21.0, 0.0], true),
        ([40.0, 41.0, 20.0, f64::MAX, 0.0], true),
    ];
    for kind in ["camera", "depth_camera"] {
        for shapes in [false, true] {
            let (mut sim, id) = ellipsoid_source(&intrinsics_source(kind, shapes));
            let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
            let original = sim.world().get::<CameraSensor>(entity).unwrap().clone();
            let accepted = extract_camera(&sim, id, 1, [0.0; 3]).unwrap();
            let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
            for (values, metadata_supported) in invalid {
                let mut changed = original.clone();
                let SensorConfig::Camera(camera) = &mut Arc::make_mut(&mut changed.0).config else {
                    panic!("camera metadata")
                };
                camera.lens.has_intrinsics = true;
                let intrinsics = &mut camera.lens.intrinsics;
                let [fx, fy, cx, cy, skew] = values;
                intrinsics.fx = fx;
                intrinsics.fy = fy;
                intrinsics.cx = cx;
                intrinsics.cy = cy;
                intrinsics.skew = skew;
                assert_eq!(
                    gz_sim_scene::supports_camera_sensor(&changed.0),
                    metadata_supported
                );
                let rejected = Arc::clone(&changed.0);
                sim.world_mut().entity_mut(entity).insert(changed);
                assert_eq!(
                    extract_camera(&sim, id, 2, [0.0; 3]),
                    Err(SnapshotError::InvalidCamera(id))
                );
                assert_eq!(
                    (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
                    before
                );
                assert!(Arc::ptr_eq(
                    &sim.world().get::<CameraSensor>(entity).unwrap().0,
                    &rejected
                ));
                sim.world_mut().entity_mut(entity).insert(original.clone());
                assert_eq!(extract_camera(&sim, id, 1, [0.0; 3]).unwrap(), accepted);
            }
        }
    }
}

fn projection_source(kind: &str, shapes: bool, explicit_k: bool, explicit_p: bool) -> String {
    let k = if explicit_k {
        "<intrinsics><fx>40</fx><fy>41</fy><cx>20</cx><cy>21</cy><s>0</s></intrinsics>"
    } else {
        ""
    };
    let p = if explicit_p {
        "<projection><p_fx>52</p_fx><p_fy>36</p_fy><p_cx>11</p_cx><p_cy>45</p_cy><tx>0</tx><ty>0</ty></projection>"
    } else {
        ""
    };
    intrinsics_source(kind, shapes).replace("</camera>", &format!("<lens>{k}{p}</lens></camera>"))
}

#[test]
fn projection_selection_keeps_independent_k_and_p_in_all_four_request_families() {
    for kind in ["camera", "depth_camera"] {
        for shapes in [false, true] {
            for explicit_k in [false, true] {
                for explicit_p in [false, true] {
                    let (mut sim, id) =
                        ellipsoid_source(&projection_source(kind, shapes, explicit_k, explicit_p));
                    sim.step_exact(1).unwrap();
                    let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
                    let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
                    let sensor = &sim.world().get::<CameraSensor>(entity).unwrap().0;
                    let selected = gz_render_scene::camera_calibrations(sensor, id).unwrap();
                    let k = selected.intrinsics.config();
                    if explicit_k {
                        assert_eq!([k.fx, k.fy, k.cx, k.cy], [40.0, 41.0, 20.0, 21.0]);
                    } else {
                        assert!((k.fx - 32.0).abs() < 1e-12);
                        assert_eq!(k.fx, k.fy);
                        assert_eq!((k.cx, k.cy), (32.0, 32.0));
                    }
                    let p = selected.projection.config();
                    if explicit_p {
                        assert_eq!([p.fx, p.fy, p.cx, p.cy], [52.0, 36.0, 11.0, 45.0]);
                        assert_ne!(k, p);
                    } else {
                        assert_eq!(k, p);
                    }
                    assert_eq!((p.skew, p.tx, p.ty), (0.0, 0.0, 0.0));
                    let request = extract_camera(&sim, id, 9, [0.0; 3]).unwrap();
                    let stamp = match (&request, kind, shapes) {
                        (Request::CalibratedSnapshot { stamp, .. }, "camera", false)
                        | (Request::CalibratedShapeSnapshot { stamp, .. }, "camera", true)
                        | (Request::CalibratedDepthSnapshot { stamp, .. }, "depth_camera", false)
                        | (
                            Request::CalibratedDepthShapeSnapshot { stamp, .. },
                            "depth_camera",
                            true,
                        ) => stamp,
                        _ => panic!("wrong projection request family"),
                    };
                    assert_eq!(
                        (stamp.epoch, stamp.seconds, stamp.nanoseconds),
                        (0, 0, 1_000_000)
                    );
                    let actual = request_camera(&request);
                    assert_eq!(
                        [actual.fx, actual.fy, actual.cx, actual.cy],
                        [p.fx, p.fy, p.cx, p.cy]
                    );
                    assert_eq!((actual.skew, actual.tx, actual.ty), (0.0, 0.0, 0.0));
                    let mut bytes = Vec::new();
                    gz_render_protocol::write_request(&mut bytes, &request).unwrap();
                    assert_eq!(
                        gz_render_protocol::read_request(&mut bytes.as_slice()).unwrap(),
                        Some(request)
                    );
                    assert_eq!(
                        (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
                        before
                    );
                }
            }
        }
    }
}

#[test]
fn absent_projection_keeps_exact_requests_despite_invalid_dormant_projection_values() {
    for kind in ["camera", "depth_camera"] {
        for shapes in [false, true] {
            for explicit_k in [false, true] {
                let (mut sim, id) =
                    ellipsoid_source(&projection_source(kind, shapes, explicit_k, false));
                let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
                let original = extract_camera(&sim, id, 1, [0.0; 3]).unwrap();
                let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
                {
                    let mut sensor = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
                    let SensorConfig::Camera(camera) = &mut Arc::make_mut(&mut sensor.0).config
                    else {
                        panic!("camera metadata")
                    };
                    assert!(!camera.lens.has_projection);
                    let p = &mut camera.lens.projection;
                    p.fx = f64::NAN;
                    p.fy = -1.0;
                    p.cx = f64::INFINITY;
                    p.cy = f64::NEG_INFINITY;
                    p.tx = 1.0;
                    p.ty = f64::NAN;
                }
                let sensor = &sim.world().get::<CameraSensor>(entity).unwrap().0;
                let selected = gz_render_scene::camera_calibrations(sensor, id).unwrap();
                assert_eq!(selected.intrinsics.config(), selected.projection.config());
                let dormant = extract_camera(&sim, id, 1, [0.0; 3]).unwrap();
                let (mut old_bytes, mut new_bytes) = (Vec::new(), Vec::new());
                gz_render_protocol::write_request(&mut old_bytes, &original).unwrap();
                gz_render_protocol::write_request(&mut new_bytes, &dormant).unwrap();
                assert_eq!(old_bytes, new_bytes);
                assert_eq!(original, dormant);
                assert_eq!(
                    (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
                    before
                );
            }
        }
    }
}

#[test]
fn selected_k_and_p_each_reject_invalid_or_unrepresentable_values_without_scene_mutation() {
    let invalid = [
        [0.0, 36.0, 11.0, 45.0],
        [52.0, -1.0, 11.0, 45.0],
        [f64::NAN, 36.0, 11.0, 45.0],
        [52.0, f64::INFINITY, 11.0, 45.0],
        [52.0, 36.0, f64::NAN, 45.0],
        [52.0, 36.0, 11.0, f64::NEG_INFINITY],
        [f64::MAX, 36.0, 11.0, 45.0],
        [52.0, f64::MIN_POSITIVE, 11.0, 45.0],
        [52.0, 36.0, f64::MAX, 45.0],
        [52.0, 36.0, 11.0, f64::MAX],
    ];
    for kind in ["camera", "depth_camera"] {
        for shapes in [false, true] {
            let (mut sim, id) = ellipsoid_source(&projection_source(kind, shapes, true, true));
            let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
            let original = sim.world().get::<CameraSensor>(entity).unwrap().clone();
            let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
            let valid = extract_camera(&sim, id, 1, [0.0; 3]).unwrap();
            for change_projection in [false, true] {
                for [fx, fy, cx, cy] in invalid {
                    let mut changed = original.clone();
                    let SensorConfig::Camera(camera) = &mut Arc::make_mut(&mut changed.0).config
                    else {
                        panic!("camera metadata")
                    };
                    if change_projection {
                        let p = &mut camera.lens.projection;
                        p.fx = fx;
                        p.fy = fy;
                        p.cx = cx;
                        p.cy = cy;
                    } else {
                        let k = &mut camera.lens.intrinsics;
                        k.fx = fx;
                        k.fy = fy;
                        k.cx = cx;
                        k.cy = cy;
                    }
                    let retained = Arc::clone(&changed.0);
                    assert!(matches!(
                        gz_render_scene::camera_calibrations(&changed.0, id),
                        Err(SnapshotError::InvalidCamera(actual)) if actual == id
                    ));
                    sim.world_mut().entity_mut(entity).insert(changed);
                    assert_eq!(
                        extract_camera(&sim, id, 2, [0.0; 3]),
                        Err(SnapshotError::InvalidCamera(id))
                    );
                    assert!(Arc::ptr_eq(
                        &retained,
                        &sim.world().get::<CameraSensor>(entity).unwrap().0
                    ));
                    assert_eq!(
                        (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
                        before
                    );
                }
            }
            for [tx, ty] in [
                [1.0, 0.0],
                [0.0, -1.0],
                [f64::NAN, 0.0],
                [0.0, f64::INFINITY],
            ] {
                let mut changed = original.clone();
                let SensorConfig::Camera(camera) = &mut Arc::make_mut(&mut changed.0).config else {
                    panic!("camera metadata")
                };
                camera.lens.projection.tx = tx;
                camera.lens.projection.ty = ty;
                sim.world_mut().entity_mut(entity).insert(changed);
                assert_eq!(
                    extract_camera(&sim, id, 3, [0.0; 3]),
                    Err(SnapshotError::InvalidCamera(id))
                );
            }
            sim.world_mut().entity_mut(entity).insert(original);
            assert_eq!(extract_camera(&sim, id, 1, [0.0; 3]).unwrap(), valid);
            assert_eq!(
                (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
                before
            );
        }
    }
}
