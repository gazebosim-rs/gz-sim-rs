// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::EntityId;
use gz_sim_scene::{
    CameraSensor, LoadError, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step,
    remove_model, reset_loaded_world, supports_camera_sensor,
};
use sdformat::{Camera, CameraPixelFormat, NoiseKind, Sensor, SensorConfig};
use std::sync::Arc;

fn camera() -> Sensor {
    let mut camera = Camera {
        image_width: 64,
        image_height: 64,
        ..Default::default()
    };
    camera.lens.has_intrinsics = false;
    camera.lens.has_projection = false;
    Sensor {
        kind: "camera".into(),
        config: SensorConfig::Camera(Box::new(camera)),
        ..Default::default()
    }
}

#[test]
fn projection_offsets_admit_finite_metadata_in_all_presence_combinations() {
    for kind in ["camera", "depth_camera"] {
        for has_k in [false, true] {
            for has_p in [false, true] {
                for [tx, ty] in [[-0.125, 0.25], [f64::MAX, -f64::MAX], [-0.0, 0.0]] {
                    let mut sensor = camera();
                    sensor.kind = kind.into();
                    let SensorConfig::Camera(config) = &mut sensor.config else {
                        unreachable!()
                    };
                    config.lens.has_intrinsics = has_k;
                    config.lens.has_projection = has_p;
                    config.lens.intrinsics.fx = 30.0;
                    config.lens.intrinsics.fy = 31.0;
                    config.lens.projection.fx = 52.0;
                    config.lens.projection.fy = 36.0;
                    config.lens.projection.tx = tx;
                    config.lens.projection.ty = ty;
                    assert!(supports_camera_sensor(&sensor));
                    for nonfinite in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                        for offset in [0, 1] {
                            let mut changed = sensor.clone();
                            let SensorConfig::Camera(config) = &mut changed.config else {
                                unreachable!()
                            };
                            if offset == 0 {
                                config.lens.projection.tx = nonfinite;
                            } else {
                                config.lens.projection.ty = nonfinite;
                            }
                            assert_eq!(supports_camera_sensor(&changed), !has_p);
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn loaded_projection_offsets_restore_original_metadata_and_identity_on_reset() {
    let source = include_str!("fixtures/camera-binding.sdf").replace(
        "</camera>",
        "<lens><projection><p_fx>52</p_fx><p_fy>36</p_fy><p_cx>11</p_cx><p_cy>45</p_cy><tx>-0.125</tx><ty>0.25</ty></projection></lens></camera>",
    );
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for kind in ["camera", "depth_camera"] {
            let source = source.replacen("type=\"camera\"", &format!("type=\"{kind}\""), 1);
            let mut sim = load_world_with_sdf_step(
                &source,
                "projection-offsets.sdf",
                "default",
                &sdformat::NoResources,
                engine,
            )
            .unwrap();
            let find = |sim: &gz_sim_core::Simulation| {
                sim.world()
                    .iter_entities()
                    .find(|entity| entity.contains::<CameraSensor>())
                    .unwrap()
                    .id()
            };
            let entity = find(&sim);
            let id = sim.world().get::<EntityId>(entity).unwrap().get();
            let pose = *sim.world().get::<WorldPose>(entity).unwrap();
            let original = sim.world().get::<CameraSensor>(entity).unwrap().0.clone();
            let projection = &original.camera().unwrap().lens.projection;
            assert_eq!([projection.tx, projection.ty], [-0.125, 0.25]);
            let systems = sim.scheduled_system_counts();
            {
                let mut current = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
                let SensorConfig::Camera(config) = &mut Arc::make_mut(&mut current.0).config else {
                    unreachable!()
                };
                config.lens.projection.tx = 3.0;
                config.lens.projection.ty = -4.0;
            }
            assert_eq!([projection.tx, projection.ty], [-0.125, 0.25]);
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            let restored = find(&sim);
            assert_eq!(sim.world().get::<EntityId>(restored).unwrap().get(), id);
            let restored_pose = sim.world().get::<WorldPose>(restored).unwrap();
            assert_eq!(restored_pose.translation, pose.translation);
            assert_eq!(restored_pose.rotation, pose.rotation);
            assert!(Arc::ptr_eq(
                &original,
                &sim.world().get::<CameraSensor>(restored).unwrap().0
            ));
            assert_eq!(sim.scheduled_system_counts(), systems);
            assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
        }
    }
}

#[test]
fn nonpositive_projection_with_or_without_intrinsics_stays_outside_loader_domain() {
    let source = include_str!("fixtures/camera-binding.sdf");
    let k = "<intrinsics><fx>40</fx><fy>41</fy><cx>20</cx><cy>21</cy><s>0</s></intrinsics>";
    let p = "<projection><p_fx>0</p_fx><p_fy>51</p_fy><p_cx>25</p_cx><p_cy>26</p_cy><tx>0.1</tx><ty>0</ty></projection>";
    let matching_p = "<projection><p_fx>40</p_fx><p_fy>0</p_fy><p_cx>20</p_cx><p_cy>21</p_cy><tx>0</tx><ty>-0.2</ty></projection>";
    for lens in [p.to_owned(), format!("{k}{p}"), format!("{k}{matching_p}")] {
        let source = source.replace("</camera>", &format!("<lens>{lens}</lens></camera>"));
        let parsed =
            sdformat::load_str(&source, "explicit-lens.sdf", &sdformat::NoResources).unwrap();
        let camera = parsed.worlds[0].models[0].links[0].sensors[0]
            .camera()
            .unwrap();
        assert!(camera.lens.has_intrinsics || camera.lens.has_projection);
        assert!(matches!(
            load_world_with_sdf_step(
                &source,
                "explicit-lens.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::Tpe
            ),
            Err(LoadError::Unsupported {
                feature: "non-IMU sensors",
                ..
            })
        ));
    }
}

#[test]
fn explicit_intrinsics_metadata_keeps_presence_and_the_existing_camera_domain() {
    for kind in ["camera", "depth_camera"] {
        let mut sensor = camera();
        sensor.kind = kind.into();
        let SensorConfig::Camera(config) = &mut sensor.config else {
            unreachable!()
        };
        config.lens.has_intrinsics = true;
        assert!(supports_camera_sensor(&sensor));
        let edits: [fn(&mut Camera); 15] = [
            |c| c.lens.intrinsics.fx = 0.0,
            |c| c.lens.intrinsics.fy = 0.0,
            |c| c.lens.intrinsics.fx = -1.0,
            |c| c.lens.intrinsics.fy = -1.0,
            |c| c.lens.intrinsics.fx = f64::NAN,
            |c| c.lens.intrinsics.fy = f64::INFINITY,
            |c| c.lens.intrinsics.cx = f64::NAN,
            |c| c.lens.intrinsics.cy = f64::NEG_INFINITY,
            |c| c.lens.intrinsics.skew = f64::INFINITY,
            |c| c.lens.intrinsics.skew = f64::NAN,
            |c| c.horizontal_fov = 0.0,
            |c| c.near_clip = 0.0,
            |c| c.image_width = 32,
            |c| c.distortion.k1 = 0.1,
            |c| {
                c.lens.has_projection = true;
                c.lens.projection.tx = f64::NEG_INFINITY;
            },
        ];
        for edit in edits {
            let mut invalid = sensor.clone();
            let SensorConfig::Camera(config) = &mut invalid.config else {
                unreachable!()
            };
            edit(config);
            assert!(!supports_camera_sensor(&invalid));
        }
        let SensorConfig::Camera(config) = &mut sensor.config else {
            unreachable!()
        };
        config.lens.intrinsics.fx = 40.0;
        config.lens.intrinsics.fy = 52.0;
        config.lens.intrinsics.cx = -20.0;
        config.lens.intrinsics.cy = 96.0;
        assert!(supports_camera_sensor(&sensor));
    }
}

#[test]
fn absent_intrinsics_ignore_dormant_lens_values_without_admitting_projection() {
    let mut sensor = camera();
    let SensorConfig::Camera(config) = &mut sensor.config else {
        unreachable!()
    };
    config.lens.intrinsics.fx = f64::NAN;
    config.lens.intrinsics.fy = -1.0;
    config.lens.intrinsics.cx = f64::INFINITY;
    config.lens.intrinsics.cy = f64::NEG_INFINITY;
    config.lens.intrinsics.skew = f64::NAN;
    assert!(supports_camera_sensor(&sensor));
    let SensorConfig::Camera(config) = &mut sensor.config else {
        unreachable!()
    };
    config.lens.has_projection = true;
    config.lens.projection.tx = f64::NAN;
    assert!(!supports_camera_sensor(&sensor));
}

#[test]
fn parsed_explicit_intrinsics_restore_original_loaded_metadata_and_identity() {
    let source = include_str!("fixtures/camera-binding.sdf").replace(
        "</camera>",
        "<lens><intrinsics><fx>40</fx><fy>52</fy><cx>20</cx><cy>44</cy><s>0</s></intrinsics></lens></camera>",
    );
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for kind in ["camera", "depth_camera"] {
            let source = source.replacen("type=\"camera\"", &format!("type=\"{kind}\""), 1);
            let mut sim = load_world_with_sdf_step(
                &source,
                "explicit-intrinsics.sdf",
                "default",
                &sdformat::NoResources,
                engine,
            )
            .unwrap();
            let find = |sim: &gz_sim_core::Simulation| {
                sim.world()
                    .iter_entities()
                    .find(|entity| entity.contains::<CameraSensor>())
                    .unwrap()
                    .id()
            };
            let entity = find(&sim);
            let id = sim.world().get::<EntityId>(entity).unwrap().get();
            let pose = *sim.world().get::<WorldPose>(entity).unwrap();
            let original = sim.world().get::<CameraSensor>(entity).unwrap().0.clone();
            let config = original.camera().unwrap();
            assert!(config.lens.has_intrinsics && !config.lens.has_projection);
            let k = &config.lens.intrinsics;
            assert_eq!(
                [k.fx, k.fy, k.cx, k.cy, k.skew],
                [40.0, 52.0, 20.0, 44.0, 0.0]
            );
            assert_eq!(original.kind, kind);
            let systems = sim.scheduled_system_counts();
            {
                let mut current = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
                let SensorConfig::Camera(config) = &mut Arc::make_mut(&mut current.0).config else {
                    unreachable!()
                };
                config.lens.intrinsics.cx = 5.0;
            }
            assert_eq!(original.camera().unwrap().lens.intrinsics.cx, 20.0);
            assert!(!Arc::ptr_eq(
                &original,
                &sim.world().get::<CameraSensor>(entity).unwrap().0
            ));
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            let restored = find(&sim);
            assert_eq!(sim.world().get::<EntityId>(restored).unwrap().get(), id);
            let restored_pose = sim.world().get::<WorldPose>(restored).unwrap();
            assert_eq!(restored_pose.translation, pose.translation);
            assert_eq!(restored_pose.rotation, pose.rotation);
            assert!(Arc::ptr_eq(
                &original,
                &sim.world().get::<CameraSensor>(restored).unwrap().0
            ));
            assert_eq!(sim.scheduled_system_counts(), systems);
            assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
        }
    }
}

#[test]
fn parsed_camera_plugins_and_extensions_are_rejected_before_construction() {
    let source = include_str!("fixtures/camera-binding.sdf");
    for (child, feature) in [
        ("<plugin name='p' filename='p.so'/>", "sensor plugins"),
        ("<alwaysOn>true</alwaysOn>", "sensor extensions"),
    ] {
        let source = source.replace("</sensor>", &format!("{child}</sensor>"));
        let parsed =
            sdformat::load_str(&source, "camera-extra.sdf", &sdformat::NoResources).unwrap();
        let sensor = &parsed.worlds[0].models[0].links[0].sensors[0];
        assert!(!supports_camera_sensor(sensor));
        if feature == "sensor plugins" {
            assert_eq!(sensor.plugins.len(), 1);
        } else {
            assert_eq!(sensor.extensions.len(), 1);
        }
        assert!(matches!(
            load_world_with_sdf_step(&source, "camera-extra.sdf", "default", &sdformat::NoResources, PhysicsEngine::Tpe),
            Err(LoadError::Unsupported { feature: actual, .. }) if actual == feature
        ));
    }
}

#[test]
fn camera_fov_and_clip_admission_checks_finite_ordered_bounds() {
    let valid = camera();
    for fov in [0.001, 1.0] {
        let mut sensor = valid.clone();
        let SensorConfig::Camera(camera) = &mut sensor.config else {
            unreachable!()
        };
        camera.horizontal_fov = fov;
        assert!(supports_camera_sensor(&sensor));
    }
    for fov in [
        0.0,
        0.000_999,
        std::f64::consts::PI,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NAN,
    ] {
        let mut sensor = valid.clone();
        let SensorConfig::Camera(camera) = &mut sensor.config else {
            unreachable!()
        };
        camera.horizontal_fov = fov;
        assert!(!supports_camera_sensor(&sensor));
    }
    for (near, far) in [
        (f64::NAN, 10.0),
        (f64::INFINITY, 10.0),
        (-0.1, 10.0),
        (0.1, f64::NAN),
        (0.1, f64::INFINITY),
        (0.1, 0.0),
        (0.1, 0.1),
    ] {
        let mut sensor = valid.clone();
        let SensorConfig::Camera(camera) = &mut sensor.config else {
            unreachable!()
        };
        camera.near_clip = near;
        camera.far_clip = far;
        assert!(!supports_camera_sensor(&sensor));
    }
    let source = include_str!("fixtures/camera-binding.sdf");
    let edge = source.replace(
        "1.5707963267948966</horizontal_fov>",
        "0.001</horizontal_fov>",
    );
    assert!(
        load_world_with_sdf_step(
            &edge,
            "edge.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Tpe
        )
        .is_ok()
    );
    let below = source.replace(
        "1.5707963267948966</horizontal_fov>",
        "0.000999</horizontal_fov>",
    );
    assert!(matches!(
        load_world_with_sdf_step(
            &below,
            "below.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Tpe
        ),
        Err(LoadError::Unsupported {
            feature: "non-IMU sensors",
            ..
        })
    ));
}

#[test]
fn prepared_sdf_fixture_exposes_unambiguous_hfov_camera_metadata() {
    let sdf = sdformat::load_str(
        include_str!("fixtures/camera-binding.sdf"),
        "camera-binding.sdf",
        &sdformat::NoResources,
    )
    .unwrap();
    let sensor = &sdf.worlds[0].models[0].links[0].sensors[0];
    assert!(supports_camera_sensor(sensor));
    let camera = sensor.camera().unwrap();
    assert!(!camera.lens.has_intrinsics);
    assert!(!camera.lens.has_projection);
    assert_eq!(sensor.topic, "/explicit-camera");
    assert_eq!(sensor.pose.transform.translation, [1.0, 0.0, 0.0]);
}

#[test]
fn ordinary_camera_retains_nondefault_wide_angle_lens_metadata() {
    let source = include_str!("fixtures/camera-binding.sdf").replace(
        "</camera>",
        "<lens><type>equidistant</type><scale_to_hfov>false</scale_to_hfov><env_texture_size>128</env_texture_size></lens></camera>",
    );
    let mut sim = load_world_with_sdf_step(
        &source,
        "lens-metadata.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let find = |sim: &gz_sim_core::Simulation| {
        sim.world()
            .iter_entities()
            .find_map(|entity| {
                (entity.get::<Name>()?.0 == "default::robot::body::camera").then(|| entity.id())
            })
            .unwrap()
    };
    let entity = find(&sim);
    let sensor = sim.world().get::<CameraSensor>(entity).unwrap().0.clone();
    let lens = &sensor.camera().unwrap().lens;
    assert_eq!(lens.kind, "equidistant");
    assert!(!lens.scale_to_hfov);
    assert_eq!(lens.environment_texture_size, 128);
    assert!(!lens.has_intrinsics && !lens.has_projection);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(Arc::ptr_eq(
        &sensor,
        &sim.world().get::<CameraSensor>(find(&sim)).unwrap().0
    ));
}

#[test]
fn loaded_camera_retains_pose_identity_and_metadata_through_reset_and_removal() {
    let source = include_str!("fixtures/camera-binding.sdf");
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let source = source.to_owned();
        let start = source.find("<sensor name=\"camera\"").unwrap();
        let end = start + source[start..].find("</sensor>").unwrap() + "</sensor>".len();
        let without_sensor = format!("{}{}", &source[..start], &source[end..]);
        let mut sim = load_world_with_sdf_step(
            &source,
            "camera-binding.sdf",
            "default",
            &sdformat::NoResources,
            engine,
        )
        .unwrap_or_else(|error| panic!("{engine:?}: {error:?}"));
        let baseline = load_world_with_sdf_step(
            &without_sensor,
            "no-camera.sdf",
            "default",
            &sdformat::NoResources,
            engine,
        )
        .unwrap();
        assert_eq!(
            sim.scheduled_system_counts(),
            baseline.scheduled_system_counts()
        );
        drop(baseline);
        let find = |sim: &gz_sim_core::Simulation, name: &str| {
            sim.world()
                .iter_entities()
                .find(|entity| entity.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap()
                .id()
        };
        let model = find(&sim, "default::robot");
        let model_id = sim.world().get::<EntityId>(model).unwrap().get();
        let entity = find(&sim, "default::robot::body::camera");
        let id = sim.world().get::<EntityId>(entity).unwrap().get();
        let pose = sim.world().get::<WorldPose>(entity).unwrap();
        let expected_x = 1_000_000_000.0;
        assert!((pose.translation[0] - expected_x).abs() < 1e-7);
        assert!((pose.translation[1] - 3.0).abs() < 1e-12);
        assert!((pose.rotation[2] - 0.5_f64.sqrt()).abs() < 1e-12);
        let original = sim.world().get::<CameraSensor>(entity).unwrap().0.clone();
        let weak = Arc::downgrade(&original);
        let parsed =
            sdformat::load_str(&source, "camera-binding.sdf", &sdformat::NoResources).unwrap();
        let parsed_sensor = &parsed.worlds[0].models[0].links[0].sensors[0];
        assert_eq!(original.source, parsed_sensor.source);
        assert_eq!(original.configuration_xml, parsed_sensor.configuration_xml);
        assert_eq!(original.config, parsed_sensor.config);
        assert_eq!(original.pose, parsed_sensor.pose);
        assert_eq!(original.frame_id, parsed_sensor.frame_id);
        assert_eq!(original.update_rate, parsed_sensor.update_rate);
        assert_eq!(original.topic, "/explicit-camera");
        assert!(!original.camera().unwrap().lens.has_intrinsics);
        assert!(!original.camera().unwrap().lens.has_projection);
        Arc::make_mut(&mut sim.world_mut().get_mut::<CameraSensor>(entity).unwrap().0).topic =
            "/edited".into();
        assert_eq!(original.topic, "/explicit-camera");
        sim.step_exact(1).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        let restored = find(&sim, "default::robot::body::camera");
        assert_eq!(sim.world().get::<EntityId>(restored).unwrap().get(), id);
        assert!(Arc::ptr_eq(
            &original,
            &sim.world().get::<CameraSensor>(restored).unwrap().0
        ));
        remove_model(&mut sim, model_id).unwrap();
        assert!(sim.world().get_entity(restored).is_err());
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        let restored = find(&sim, "default::robot::body::camera");
        assert_eq!(sim.world().get::<EntityId>(restored).unwrap().get(), id);
        assert!(Arc::ptr_eq(
            &original,
            &sim.world().get::<CameraSensor>(restored).unwrap().0
        ));
        drop(original);
        drop(sim);
        assert!(weak.upgrade().is_none());
    }
}

#[test]
fn camera_domain_rejects_unimplemented_features_and_invalid_geometry() {
    let valid = camera();
    assert!(supports_camera_sensor(&valid));
    let mut missing = valid.clone();
    missing.config = SensorConfig::Unconfigured;
    assert!(!supports_camera_sensor(&missing));
    for kind in ["depth_camera", "depth"] {
        let mut depth = valid.clone();
        depth.kind = kind.into();
        assert!(supports_camera_sensor(&depth));
    }
    let mut other_type = valid.clone();
    other_type.kind = "thermal_camera".into();
    assert!(!supports_camera_sensor(&other_type));
    other_type.kind = "wide_angle_camera".into();
    assert!(!supports_camera_sensor(&other_type));
    let edits: [fn(&mut Camera); 19] = [
        |c| c.image_width = 63,
        |c| c.image_height = 63,
        |c| c.pixel_format = CameraPixelFormat::LInt8,
        |c| c.anti_aliasing = 0,
        |c| c.triggered = true,
        |c| c.save_frames = true,
        |c| c.image_noise.kind = NoiseKind::Gaussian,
        |c| c.distortion.k1 = 0.1,
        |c| c.pose.transform.translation[0] = 1.0,
        |c| c.visibility_mask = 0x1000_0000,
        |c| c.has_depth_near_clip = true,
        |c| c.has_depth_far_clip = true,
        |c| c.has_segmentation_type = true,
        |c| c.has_bounding_box_type = true,
        |c| {
            c.lens.has_intrinsics = true;
            c.lens.intrinsics.fx = 0.0;
        },
        |c| {
            c.lens.has_projection = true;
            c.lens.projection.tx = f64::NEG_INFINITY;
        },
        |c| c.horizontal_fov = f64::NAN,
        |c| c.near_clip = 0.0,
        |c| c.far_clip = c.near_clip,
    ];
    for kind in ["camera", "depth_camera"] {
        for edit in edits {
            let mut sensor = valid.clone();
            sensor.kind = kind.into();
            let SensorConfig::Camera(camera) = &mut sensor.config else {
                unreachable!()
            };
            edit(camera);
            assert!(!supports_camera_sensor(&sensor));
        }
        let sensor_edits: [fn(&mut Sensor); 2] =
            [|s| s.visualize = true, |s| s.enable_metrics = true];
        for edit in sensor_edits {
            let mut sensor = valid.clone();
            sensor.kind = kind.into();
            edit(&mut sensor);
            assert!(!supports_camera_sensor(&sensor));
        }
    }
}

#[test]
fn explicit_projection_metadata_checks_each_selected_scalar_and_presence() {
    for kind in ["camera", "depth_camera"] {
        for explicit_k in [false, true] {
            let mut sensor = camera();
            sensor.kind = kind.into();
            let SensorConfig::Camera(config) = &mut sensor.config else {
                unreachable!()
            };
            config.lens.has_intrinsics = explicit_k;
            config.lens.has_projection = true;
            config.lens.projection.fx = 20.0;
            config.lens.projection.fy = 24.0;
            config.lens.projection.cx = -42.0;
            config.lens.projection.cy = 96.0;
            assert!(supports_camera_sensor(&sensor));
            let edits: [fn(&mut Camera); 14] = [
                |c| c.lens.projection.fx = 0.0,
                |c| c.lens.projection.fy = -1.0,
                |c| c.lens.projection.fx = f64::NAN,
                |c| c.lens.projection.fy = f64::INFINITY,
                |c| c.lens.projection.cx = f64::NAN,
                |c| c.lens.projection.cy = f64::NEG_INFINITY,
                |c| c.lens.projection.tx = f64::NEG_INFINITY,
                |c| c.lens.projection.ty = f64::NEG_INFINITY,
                |c| c.lens.projection.tx = f64::NAN,
                |c| c.lens.projection.ty = f64::INFINITY,
                |c| c.image_width = 32,
                |c| c.near_clip = 0.0,
                |c| c.distortion.k1 = 0.1,
                |c| {
                    c.lens.has_intrinsics = true;
                    c.lens.intrinsics.skew = f64::INFINITY;
                },
            ];
            for edit in edits {
                let mut invalid = sensor.clone();
                let SensorConfig::Camera(config) = &mut invalid.config else {
                    unreachable!()
                };
                edit(config);
                assert!(!supports_camera_sensor(&invalid));
            }
            let SensorConfig::Camera(config) = &mut sensor.config else {
                unreachable!()
            };
            config.lens.has_projection = false;
            config.lens.projection.fx = f64::NAN;
            config.lens.projection.fy = -1.0;
            config.lens.projection.cx = f64::INFINITY;
            config.lens.projection.cy = f64::NEG_INFINITY;
            config.lens.projection.tx = f64::NAN;
            config.lens.projection.ty = f64::INFINITY;
            assert!(supports_camera_sensor(&sensor));
        }
    }
}

#[test]
fn parsed_monocular_projection_keeps_all_four_presence_combinations() {
    let k = "<intrinsics><fx>52</fx><fy>44</fy><cx>34</cx><cy>29</cy><s>0</s></intrinsics>";
    let p = "<projection><p_fx>40</p_fx><p_fy>48</p_fy><p_cx>20</p_cx><p_cy>25</p_cy><tx>0</tx><ty>0</ty></projection>";
    for kind in ["camera", "depth_camera"] {
        for has_k in [false, true] {
            for has_p in [false, true] {
                let lens = format!(
                    "<lens>{}{}</lens></camera>",
                    if has_k { k } else { "" },
                    if has_p { p } else { "" }
                );
                let source = include_str!("fixtures/camera-binding.sdf")
                    .replace("</camera>", &lens)
                    .replacen("type=\"camera\"", &format!("type=\"{kind}\""), 1);
                for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
                    let sim = load_world_with_sdf_step(
                        &source,
                        "projection-presence.sdf",
                        "default",
                        &sdformat::NoResources,
                        engine,
                    )
                    .unwrap();
                    let sensor = sim
                        .world()
                        .iter_entities()
                        .find_map(|e| e.get::<CameraSensor>())
                        .unwrap();
                    let config = sensor.0.camera().unwrap();
                    assert_eq!(sensor.0.kind, kind);
                    assert_eq!(config.lens.has_intrinsics, has_k);
                    assert_eq!(config.lens.has_projection, has_p);
                    assert!(supports_camera_sensor(&sensor.0));
                    if has_p {
                        let p = &config.lens.projection;
                        assert_eq!(
                            [p.fx, p.fy, p.cx, p.cy, p.tx, p.ty],
                            [40.0, 48.0, 20.0, 25.0, 0.0, 0.0]
                        );
                    }
                    assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
                    let baseline = load_world_with_sdf_step(
                        include_str!("fixtures/camera-binding.sdf"),
                        "baseline.sdf",
                        "default",
                        &sdformat::NoResources,
                        engine,
                    )
                    .unwrap();
                    assert_eq!(
                        sim.scheduled_system_counts(),
                        baseline.scheduled_system_counts()
                    );
                }
            }
        }
    }
}

#[test]
fn loaded_projection_reset_restores_original_calibrations_identity_and_pose() {
    let source = include_str!("fixtures/camera-binding.sdf").replace("</camera>", "<lens><intrinsics><fx>52</fx><fy>44</fy><cx>34</cx><cy>29</cy><s>0</s></intrinsics><projection><p_fx>40</p_fx><p_fy>48</p_fy><p_cx>20</p_cx><p_cy>25</p_cy><tx>0</tx><ty>0</ty></projection></lens></camera>");
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim = load_world_with_sdf_step(
            &source,
            "projection-reset.sdf",
            "default",
            &sdformat::NoResources,
            engine,
        )
        .unwrap();
        let find = |sim: &gz_sim_core::Simulation| {
            sim.world()
                .iter_entities()
                .find(|e| e.contains::<CameraSensor>())
                .unwrap()
                .id()
        };
        let entity = find(&sim);
        let original = sim.world().get::<CameraSensor>(entity).unwrap().0.clone();
        let id = sim.world().get::<EntityId>(entity).unwrap().get();
        let pose = *sim.world().get::<WorldPose>(entity).unwrap();
        let systems = sim.scheduled_system_counts();
        {
            let mut current = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
            let SensorConfig::Camera(config) = &mut Arc::make_mut(&mut current.0).config else {
                unreachable!()
            };
            config.lens.intrinsics.fx = 9.0;
            config.lens.projection.cx = 8.0;
        }
        assert_eq!(original.camera().unwrap().lens.intrinsics.fx, 52.0);
        assert_eq!(original.camera().unwrap().lens.projection.cx, 20.0);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        let restored = find(&sim);
        assert_eq!(sim.world().get::<EntityId>(restored).unwrap().get(), id);
        assert!(Arc::ptr_eq(
            &original,
            &sim.world().get::<CameraSensor>(restored).unwrap().0
        ));
        let current_pose = sim.world().get::<WorldPose>(restored).unwrap();
        assert_eq!(current_pose.translation, pose.translation);
        assert_eq!(current_pose.rotation, pose.rotation);
        assert_eq!(sim.scheduled_system_counts(), systems);
        assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
    }
}

#[test]
fn signed_skew_metadata_admits_finite_values_and_ignores_absent_intrinsics() {
    for kind in ["camera", "depth_camera"] {
        for has_p in [false, true] {
            for skew in [-35.0, -0.0, 0.0, 12.0, f64::MAX] {
                let mut sensor = camera();
                sensor.kind = kind.into();
                let SensorConfig::Camera(config) = &mut sensor.config else {
                    unreachable!()
                };
                config.lens.has_intrinsics = true;
                config.lens.has_projection = has_p;
                config.lens.intrinsics.skew = skew;
                assert!(
                    supports_camera_sensor(&sensor),
                    "finite metadata skew{skew}"
                );
                for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                    let mut rejected = sensor.clone();
                    {
                        let SensorConfig::Camera(config) = &mut rejected.config else {
                            unreachable!()
                        };
                        config.lens.intrinsics.skew = invalid;
                    }
                    assert!(!supports_camera_sensor(&rejected));
                    {
                        let SensorConfig::Camera(config) = &mut rejected.config else {
                            unreachable!()
                        };
                        config.lens.has_intrinsics = false;
                    }
                    assert!(supports_camera_sensor(&rejected));
                }
                let SensorConfig::Camera(config) = &mut sensor.config else {
                    unreachable!()
                };
                config.lens.has_projection = true;
                config.lens.projection.ty = f64::NAN;
                assert!(!supports_camera_sensor(&sensor));
            }
        }
    }
}

#[test]
fn loaded_signed_skew_restores_original_metadata_and_world_pose_on_reset() {
    for kind in ["camera", "depth_camera"] {
        for skew in [-10.0, 12.0] {
            let lens = format!(
                "<lens><intrinsics><fx>52</fx><fy>44</fy><cx>34</cx><cy>29</cy><s>{skew}</s></intrinsics></lens></camera>"
            );
            let source = include_str!("fixtures/camera-binding.sdf")
                .replace("</camera>", &lens)
                .replacen("type=\"camera\"", &format!("type=\"{kind}\""), 1);
            for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
                let mut sim = load_world_with_sdf_step(
                    &source,
                    "signed-skew.sdf",
                    "default",
                    &sdformat::NoResources,
                    engine,
                )
                .unwrap();
                let find = |sim: &gz_sim_core::Simulation| {
                    sim.world()
                        .iter_entities()
                        .find(|e| e.contains::<CameraSensor>())
                        .unwrap()
                        .id()
                };
                let entity = find(&sim);
                let id = sim.world().get::<EntityId>(entity).unwrap().get();
                let original = sim.world().get::<CameraSensor>(entity).unwrap().0.clone();
                assert_eq!(original.kind, kind);
                assert_eq!(original.camera().unwrap().lens.intrinsics.skew, skew);
                let pose = *sim.world().get::<WorldPose>(entity).unwrap();
                let systems = sim.scheduled_system_counts();
                {
                    let mut sensor = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
                    let SensorConfig::Camera(config) = &mut Arc::make_mut(&mut sensor.0).config
                    else {
                        unreachable!()
                    };
                    config.lens.intrinsics.skew = -skew;
                }
                assert_eq!(original.camera().unwrap().lens.intrinsics.skew, skew);
                reset_loaded_world(&mut sim, |_| ()).unwrap();
                let restored = find(&sim);
                assert_eq!(sim.world().get::<EntityId>(restored).unwrap().get(), id);
                assert!(Arc::ptr_eq(
                    &original,
                    &sim.world().get::<CameraSensor>(restored).unwrap().0
                ));
                let current_pose = sim.world().get::<WorldPose>(restored).unwrap();
                assert_eq!(current_pose.translation, pose.translation);
                assert_eq!(current_pose.rotation, pose.rotation);
                assert_eq!(sim.scheduled_system_counts(), systems);
                assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
            }
        }
    }
}

#[test]
fn camera_visibility_metadata_admits_user_masks_and_refuses_reserved_bits() {
    for kind in ["camera", "depth_camera"] {
        for mask in [0, 1, 2, 3, 0x0800_0000, 0x0FFF_FFFF, u32::MAX] {
            let mut sensor = camera();
            sensor.kind = kind.into();
            {
                let SensorConfig::Camera(config) = &mut sensor.config else {
                    unreachable!()
                };
                config.visibility_mask = mask;
            }
            assert!(supports_camera_sensor(&sensor), "mask={mask:#x}");
            let SensorConfig::Camera(config) = &mut sensor.config else {
                unreachable!()
            };
            config.image_width = 65;
            assert!(!supports_camera_sensor(&sensor));
        }
        for mask in [0x1000_0000, 0x1000_0001, 0x8000_0000, 0xFFFF_FFFE] {
            let mut sensor = camera();
            sensor.kind = kind.into();
            let SensorConfig::Camera(config) = &mut sensor.config else {
                unreachable!()
            };
            config.visibility_mask = mask;
            assert!(!supports_camera_sensor(&sensor), "mask={mask:#x}");
        }
    }
}

#[test]
fn parsed_visibility_masks_and_visual_flags_restore_original_ownership_on_reset() {
    for kind in ["camera", "depth_camera"] {
        for mask in [0, 1, 3, 0x0FFF_FFFF, u32::MAX] {
            let source = include_str!("fixtures/camera-binding.sdf")
                .replacen("type=\"camera\"", &format!("type=\"{kind}\""), 1)
                .replace("</camera>", &format!("<visibility_mask>{mask}</visibility_mask></camera>"))
                .replace("</link>", "<visual name='masked'><visibility_flags>2</visibility_flags><geometry><box><size>1 1 1</size></box></geometry></visual></link>");
            let parsed =
                sdformat::load_str(&source, "visibility.sdf", &sdformat::NoResources).unwrap();
            assert_eq!(
                parsed.worlds[0].models[0].links[0].sensors[0]
                    .camera()
                    .unwrap()
                    .visibility_mask,
                mask
            );
            for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
                let mut sim = load_world_with_sdf_step(
                    &source,
                    "visibility.sdf",
                    "default",
                    &sdformat::NoResources,
                    engine,
                )
                .unwrap();
                let sensor = sim
                    .world()
                    .iter_entities()
                    .find(|e| e.contains::<CameraSensor>())
                    .unwrap()
                    .id();
                let visual = sim
                    .world()
                    .iter_entities()
                    .find(|e| e.contains::<gz_sim_scene::VisualProperties>())
                    .unwrap()
                    .id();
                let sensor_id = sim.world().get::<EntityId>(sensor).unwrap().get();
                let visual_id = sim.world().get::<EntityId>(visual).unwrap().get();
                let original = sim.world().get::<CameraSensor>(sensor).unwrap().0.clone();
                let flags = sim
                    .world()
                    .get::<gz_sim_scene::VisualProperties>(visual)
                    .unwrap()
                    .0
                    .clone();
                let systems = sim.scheduled_system_counts();
                {
                    let mut current = sim.world_mut().get_mut::<CameraSensor>(sensor).unwrap();
                    let SensorConfig::Camera(camera) = &mut Arc::make_mut(&mut current.0).config
                    else {
                        unreachable!()
                    };
                    camera.visibility_mask = if mask == 1 { 2 } else { 1 };
                }
                {
                    let mut current = sim
                        .world_mut()
                        .get_mut::<gz_sim_scene::VisualProperties>(visual)
                        .unwrap();
                    current.0.visibility_flags = 1;
                }
                assert_eq!(original.camera().unwrap().visibility_mask, mask);
                assert_eq!(flags.visibility_flags, 2);
                reset_loaded_world(&mut sim, |_| ()).unwrap();
                let sensor = gz_sim_core::entity_by_id(sim.world(), sensor_id).unwrap();
                let visual = gz_sim_core::entity_by_id(sim.world(), visual_id).unwrap();
                assert!(Arc::ptr_eq(
                    &original,
                    &sim.world().get::<CameraSensor>(sensor).unwrap().0
                ));
                assert_eq!(
                    flags,
                    sim.world()
                        .get::<gz_sim_scene::VisualProperties>(visual)
                        .unwrap()
                        .0
                );
                assert_eq!(
                    sim.world()
                        .get::<CameraSensor>(sensor)
                        .unwrap()
                        .0
                        .camera()
                        .unwrap()
                        .visibility_mask,
                    mask
                );
                assert_eq!(
                    sim.world()
                        .get::<gz_sim_scene::VisualProperties>(visual)
                        .unwrap()
                        .0
                        .visibility_flags,
                    2
                );
                assert_eq!(sim.scheduled_system_counts(), systems);
                assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
            }
        }
    }
}

#[test]
fn parsed_reserved_camera_visibility_masks_fail_before_world_construction() {
    for mask in [0x1000_0000_u32, 0x1000_0001, 0x8000_0000, 0xFFFF_FFFE] {
        let source = include_str!("fixtures/camera-binding.sdf").replace(
            "</camera>",
            &format!("<visibility_mask>{mask}</visibility_mask></camera>"),
        );
        let parsed =
            sdformat::load_str(&source, "reserved-mask.sdf", &sdformat::NoResources).unwrap();
        assert_eq!(
            parsed.worlds[0].models[0].links[0].sensors[0]
                .camera()
                .unwrap()
                .visibility_mask,
            mask
        );
        assert!(!supports_camera_sensor(
            &parsed.worlds[0].models[0].links[0].sensors[0]
        ));
        for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
            assert!(matches!(
                load_world_with_sdf_step(
                    &source,
                    "reserved-mask.sdf",
                    "default",
                    &sdformat::NoResources,
                    engine
                ),
                Err(LoadError::Unsupported {
                    feature: "non-IMU sensors",
                    ..
                })
            ));
        }
    }
}
