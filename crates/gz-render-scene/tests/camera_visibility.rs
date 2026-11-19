// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use gz_render_protocol::{Geometry, Request};
use gz_render_scene::{SnapshotError, extract, extract_camera};
use gz_sim_core::{EntityId, Simulation, assign_id, entity_by_id};
use gz_sim_scene::{
    CameraSensor, EntityKind, PhysicsEngine, VisualGeometry, VisualMaterial, VisualProperties,
    WorldPose, load_world_with_sdf_step,
};
use std::sync::Arc;

fn scene(kind: &str) -> (Simulation, u64, Vec<Entity>) {
    let source = include_str!("../../../examples/m67_rgb_ellipsoid.sdf")
        .replace("type=\"camera\"", &format!("type=\"{kind}\""));
    let sim = load_world_with_sdf_step(
        &source,
        "camera-visibility.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let camera = sim
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<CameraSensor>())
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let mut visuals: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|entity| entity.get::<EntityKind>() == Some(&EntityKind::Visual))
        .map(|entity| entity.id())
        .collect();
    visuals.sort_by_key(|entity| sim.world().get::<EntityId>(*entity).unwrap().get());
    assert_eq!(visuals.len(), 3);
    (sim, camera, visuals)
}

fn mask(sim: &mut Simulation, camera: u64, value: u32) {
    let entity = entity_by_id(sim.world(), camera).unwrap();
    let mut metadata = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
    let sdformat::SensorConfig::Camera(camera) = &mut Arc::make_mut(&mut metadata.0).config else {
        panic!("camera metadata")
    };
    camera.visibility_mask = value;
}

fn selected(request: &Request) -> Vec<u64> {
    match request {
        Request::CalibratedSnapshot { visuals, .. }
        | Request::CalibratedDepthSnapshot { visuals, .. } => {
            visuals.iter().map(|row| row.id).collect()
        }
        Request::CalibratedShapeSnapshot { visuals, .. }
        | Request::CalibratedDepthShapeSnapshot { visuals, .. } => {
            visuals.iter().map(|row| row.id).collect()
        }
        _ => panic!("calibrated request"),
    }
}

#[test]
fn camera_masks_select_exact_sorted_ids_across_all_shapes_and_camera_families() {
    for kind in ["camera", "depth_camera"] {
        for geometry in [
            sdformat::Geometry::Box { size: [1.0; 3] },
            sdformat::Geometry::Cylinder {
                radius: 0.5,
                length: 1.0,
            },
            sdformat::Geometry::Sphere { radius: 0.5 },
            sdformat::Geometry::Ellipsoid {
                radii: [0.5, 0.75, 1.0],
            },
        ] {
            let (mut sim, camera, visuals) = scene(kind);
            let ids: Vec<_> = visuals
                .iter()
                .map(|entity| sim.world().get::<EntityId>(*entity).unwrap().get())
                .collect();
            for (entity, flags) in visuals.iter().zip([1, 2, 3]) {
                sim.world_mut()
                    .get_mut::<VisualGeometry>(*entity)
                    .unwrap()
                    .0 = Arc::new(geometry.clone());
                sim.world_mut()
                    .get_mut::<VisualProperties>(*entity)
                    .unwrap()
                    .0
                    .visibility_flags = flags;
            }
            let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
            for (visibility, expected) in [
                (0, vec![]),
                (1, vec![ids[0], ids[2]]),
                (2, vec![ids[1], ids[2]]),
                (4, vec![]),
                (3, ids.clone()),
                (0x0FFF_FFFF, ids.clone()),
                (u32::MAX, ids.clone()),
            ] {
                mask(&mut sim, camera, visibility);
                let request = extract_camera(&sim, camera, 1, [0.0; 3]).unwrap();
                assert_eq!(selected(&request), expected);
                match (&request, kind) {
                    (Request::CalibratedSnapshot { .. }, "camera")
                    | (Request::CalibratedDepthSnapshot { .. }, "depth_camera") => {
                        assert!(
                            expected.is_empty()
                                || matches!(&geometry, sdformat::Geometry::Box { .. })
                        );
                    }
                    (Request::CalibratedShapeSnapshot { visuals, .. }, "camera")
                    | (Request::CalibratedDepthShapeSnapshot { visuals, .. }, "depth_camera") => {
                        assert!(!expected.is_empty());
                        assert!(
                            visuals
                                .iter()
                                .all(|row| !matches!(row.geometry, Geometry::Box { .. }))
                        );
                    }
                    _ => panic!("wrong visibility request family"),
                }
                assert!(request.wire_valid());
                assert_eq!(
                    (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
                    before
                );
            }
            mask(&mut sim, camera, u32::MAX);
            sim.world_mut()
                .get_mut::<VisualProperties>(visuals[1])
                .unwrap()
                .0
                .visibility_flags = 0;
            assert_eq!(
                selected(&extract_camera(&sim, camera, 1, [0.0; 3]).unwrap()),
                [ids[0], ids[2]]
            );
            sim.world_mut()
                .get_mut::<VisualProperties>(visuals[0])
                .unwrap()
                .0
                .visibility_flags = 0x0800_0000;
            sim.world_mut()
                .get_mut::<VisualProperties>(visuals[2])
                .unwrap()
                .0
                .visibility_flags = 1;
            mask(&mut sim, camera, 0x0800_0000);
            assert_eq!(
                selected(&extract_camera(&sim, camera, 1, [0.0; 3]).unwrap()),
                [ids[0]]
            );
        }
    }
}

#[test]
fn default_visibility_and_all_user_bits_preserve_exact_calibrated_request_bytes() {
    for kind in ["camera", "depth_camera"] {
        let (mut sim, camera, visuals) = scene(kind);
        let original = extract_camera(&sim, camera, 1, [0.0; 3]).unwrap();
        let mut original_bytes = Vec::new();
        gz_render_protocol::write_request(&mut original_bytes, &original).unwrap();
        for camera_mask in [u32::MAX, 0x0FFF_FFFF] {
            mask(&mut sim, camera, camera_mask);
            for flags in [u32::MAX, 0x0FFF_FFFF] {
                for entity in &visuals {
                    sim.world_mut()
                        .get_mut::<VisualProperties>(*entity)
                        .unwrap()
                        .0
                        .visibility_flags = flags;
                }
                let unchanged = extract_camera(&sim, camera, 1, [0.0; 3]).unwrap();
                let mut bytes = Vec::new();
                gz_render_protocol::write_request(&mut bytes, &unchanged).unwrap();
                assert_eq!(bytes, original_bytes);
                assert_eq!(unchanged, original);
            }
        }
    }
}

#[test]
fn reserved_visibility_bits_fail_for_known_cameras_and_hidden_visuals() {
    for kind in ["camera", "depth_camera"] {
        for reserved in [0x1000_0000, 0x8000_0000, 0x7FFF_FFFF] {
            let (mut sim, camera, visuals) = scene(kind);
            mask(&mut sim, camera, reserved);
            assert_eq!(
                extract_camera(&sim, camera, 1, [0.0; 3]),
                Err(SnapshotError::InvalidCamera(camera))
            );
            mask(&mut sim, camera, 0);
            let id = sim.world().get::<EntityId>(visuals[0]).unwrap().get();
            sim.world_mut()
                .get_mut::<VisualProperties>(visuals[0])
                .unwrap()
                .0
                .visibility_flags = reserved;
            assert_eq!(
                extract_camera(&sim, camera, 1, [0.0; 3]),
                Err(SnapshotError::UnsupportedMaterial(id))
            );
        }
    }
}

#[test]
fn zero_masks_cannot_hide_invalid_visual_components_material_geometry_pose_or_lights() {
    for fault in 0..11 {
        let (mut sim, camera, visuals) = scene("camera");
        mask(&mut sim, camera, 0);
        for entity in &visuals {
            sim.world_mut()
                .get_mut::<VisualProperties>(*entity)
                .unwrap()
                .0
                .visibility_flags = 0;
        }
        let entity = visuals[0];
        let id = sim.world().get::<EntityId>(entity).unwrap().get();
        let expected = match fault {
            0 => {
                sim.world_mut().entity_mut(entity).remove::<EntityId>();
                SnapshotError::InvalidIdentity
            }
            1 => {
                sim.world_mut()
                    .entity_mut(entity)
                    .remove::<VisualGeometry>();
                SnapshotError::MissingVisualData(id)
            }
            2 => {
                sim.world_mut()
                    .entity_mut(entity)
                    .remove::<VisualProperties>();
                SnapshotError::MissingVisualData(id)
            }
            3 => {
                sim.world_mut()
                    .entity_mut(entity)
                    .remove::<VisualMaterial>();
                SnapshotError::MissingVisualData(id)
            }
            4 => {
                sim.world_mut().entity_mut(entity).remove::<WorldPose>();
                SnapshotError::MissingVisualData(id)
            }
            5 => {
                sim.world_mut().get_mut::<VisualGeometry>(entity).unwrap().0 =
                    Arc::new(sdformat::Geometry::Capsule {
                        radius: 0.5,
                        length: 1.0,
                    });
                SnapshotError::UnsupportedGeometry(id)
            }
            6 => {
                sim.world_mut().get_mut::<VisualGeometry>(entity).unwrap().0 =
                    Arc::new(sdformat::Geometry::Box {
                        size: [1.0, 0.0, 1.0],
                    });
                SnapshotError::InvalidVisual(id)
            }
            7 => {
                sim.world_mut()
                    .get_mut::<VisualMaterial>(entity)
                    .unwrap()
                    .0
                    .diffuse[3] = 0.5;
                SnapshotError::UnsupportedMaterial(id)
            }
            8 => {
                sim.world_mut()
                    .get_mut::<VisualProperties>(entity)
                    .unwrap()
                    .0
                    .visibility_flags = 0x1000_0000;
                SnapshotError::UnsupportedMaterial(id)
            }
            9 => {
                sim.world_mut()
                    .get_mut::<WorldPose>(entity)
                    .unwrap()
                    .rotation = [0.0; 4];
                SnapshotError::InvalidVisual(id)
            }
            10 => {
                sim.world_mut().spawn(EntityKind::Light);
                SnapshotError::UnsupportedLighting
            }
            _ => unreachable!(),
        };
        let before = (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts());
        assert_eq!(extract_camera(&sim, camera, 1, [0.0; 3]), Err(expected));
        assert_eq!(
            (sim.elapsed(), sim.epoch(), sim.scheduled_system_counts()),
            before
        );
    }
}

#[test]
fn global_visual_limit_is_enforced_before_zero_mask_filtering() {
    let (mut sim, camera, visuals) = scene("depth_camera");
    mask(&mut sim, camera, 0);
    let template = visuals[0];
    let geometry = sim.world().get::<VisualGeometry>(template).unwrap().clone();
    let material = sim.world().get::<VisualMaterial>(template).unwrap().clone();
    let mut properties = sim
        .world()
        .get::<VisualProperties>(template)
        .unwrap()
        .clone();
    properties.0.visibility_flags = 0;
    let pose = *sim.world().get::<WorldPose>(template).unwrap();
    for _ in visuals.len()..128 {
        let entity = sim
            .world_mut()
            .spawn((
                EntityKind::Visual,
                geometry.clone(),
                material.clone(),
                properties.clone(),
                pose,
            ))
            .id();
        assign_id(sim.world_mut(), entity).unwrap();
    }
    assert!(selected(&extract_camera(&sim, camera, 1, [0.0; 3]).unwrap()).is_empty());
    let extra = sim
        .world_mut()
        .spawn((EntityKind::Visual, geometry, material, properties, pose))
        .id();
    assign_id(sim.world_mut(), extra).unwrap();
    for camera_mask in [0, 1, u32::MAX] {
        mask(&mut sim, camera, camera_mask);
        assert_eq!(
            extract_camera(&sim, camera, 2, [0.0; 3]),
            Err(SnapshotError::TooManyVisuals)
        );
    }
}

#[test]
fn generic_and_unknown_camera_paths_retain_visibility_and_geometry_error_order() {
    let (mut sim, camera, visuals) = scene("camera");
    let first = visuals[0];
    let id = sim.world().get::<EntityId>(first).unwrap().get();
    for entity in &visuals {
        sim.world_mut()
            .get_mut::<VisualGeometry>(*entity)
            .unwrap()
            .0 = Arc::new(sdformat::Geometry::Box { size: [1.0; 3] });
    }
    let original = extract(&sim, 1, [0.0; 3]).unwrap();
    sim.world_mut()
        .get_mut::<VisualProperties>(first)
        .unwrap()
        .0
        .visibility_flags = 1;
    mask(&mut sim, camera, 0);
    assert!(selected(&extract_camera(&sim, camera, 1, [0.0; 3]).unwrap()).is_empty());
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedMaterial(id))
    );
    assert_eq!(
        extract_camera(&sim, u64::MAX, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedMaterial(id))
    );
    let sensor = entity_by_id(sim.world(), camera).unwrap();
    sim.world_mut().entity_mut(sensor).insert(EntityKind::Link);
    assert_eq!(
        extract_camera(&sim, camera, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedMaterial(id))
    );
    sim.world_mut()
        .get_mut::<VisualProperties>(first)
        .unwrap()
        .0
        .visibility_flags = u32::MAX;
    assert_eq!(extract(&sim, 1, [0.0; 3]).unwrap(), original);
    assert_eq!(
        extract_camera(&sim, camera, 1, [0.0; 3]),
        Err(SnapshotError::MissingCamera(camera))
    );
    sim.world_mut()
        .entity_mut(sensor)
        .insert(EntityKind::Sensor);
    sim.world_mut().get_mut::<VisualGeometry>(first).unwrap().0 =
        Arc::new(sdformat::Geometry::Sphere { radius: 0.5 });
    assert_eq!(
        extract(&sim, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedGeometry(id))
    );
    assert_eq!(
        extract_camera(&sim, u64::MAX, 1, [0.0; 3]),
        Err(SnapshotError::UnsupportedGeometry(id))
    );
}
