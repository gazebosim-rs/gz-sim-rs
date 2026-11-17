// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy::camera::Exposure;

#[test]
fn permanent_shader_failures_are_distinct_from_pending_imports() {
    assert!(
        permanent_shader_error(&ShaderCacheError::ShaderNotLoaded(
            Handle::<Shader>::default().id()
        ))
        .is_none()
    );
    assert!(permanent_shader_error(&ShaderCacheError::ShaderImportNotYetAvailable).is_none());
    assert!(
        permanent_shader_error(&ShaderCacheError::ProcessShaderError(
            "invalid shader".into()
        ))
        .unwrap()
        .contains("invalid shader")
    );
    assert!(
        permanent_shader_error(&ShaderCacheError::CreateShaderModule(
            "invalid module".into()
        ))
        .unwrap()
        .contains("invalid module")
    );
}

#[test]
fn invalid_shader_fails_the_renderer() {
    use bevy::render::render_resource::{RenderPipelineDescriptor, VertexState};
    let mut renderer = Renderer::new().unwrap();
    let shader = renderer
        .app
        .world_mut()
        .resource_mut::<Assets<Shader>>()
        .add(Shader::from_wesl("not valid WESL", "invalid.wesl"));
    let descriptor = RenderPipelineDescriptor {
        vertex: VertexState {
            shader,
            ..default()
        },
        ..default()
    };
    renderer
        .app
        .get_sub_app_mut(RenderApp)
        .unwrap()
        .world_mut()
        .resource_mut::<PipelineCache>()
        .queue_render_pipeline(descriptor);
    for _ in 0..200 {
        match renderer.update() {
            Err(RenderError::Shader(_)) => {
                assert!(matches!(renderer.update(), Err(RenderError::Shader(_))));
                return;
            }
            Err(error) => panic!("unexpected error: {error}"),
            Ok(_) => std::thread::sleep(Duration::from_millis(10)),
        }
    }
    panic!("invalid shader did not report an error");
}

fn assert_missing_resource_blocks_readiness<R: Resource>(renderer: &mut Renderer) {
    let resource = renderer
        .app
        .get_sub_app_mut(RenderApp)
        .unwrap()
        .world_mut()
        .remove_resource::<R>()
        .unwrap();
    assert!(!renderer.scene_assets_ready());
    renderer
        .app
        .get_sub_app_mut(RenderApp)
        .unwrap()
        .world_mut()
        .insert_resource(resource);
}

#[test]
fn readback_failure_and_terminal_device_error_are_reported() {
    let mut renderer = Renderer::new().unwrap();
    let fence = Arc::new(AtomicBool::new(false));
    renderer.in_flight = Some(fence.clone());
    assert_eq!(renderer.update().unwrap(), UpdateStatus::Waiting);
    fence.store(true, Ordering::Release);
    assert_eq!(renderer.update().unwrap(), UpdateStatus::Advanced);
    renderer
        .set_snapshot(&Snapshot {
            epoch: 0,
            time: Duration::ZERO,
            origin: DVec3::ZERO,
            visuals: vec![],
        })
        .unwrap();
    for _ in 0..200 {
        renderer.update().unwrap();
        if renderer.ready {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(renderer.ready);
    assert_missing_resource_blocks_readiness::<PipelineCache>(&mut renderer);
    assert_missing_resource_blocks_readiness::<RenderAssets<GpuImage>>(&mut renderer);
    assert_missing_resource_blocks_readiness::<RenderAssets<RenderMesh>>(&mut renderer);
    assert_missing_resource_blocks_readiness::<ErasedRenderAssets<PreparedMaterial>>(&mut renderer);
    assert_missing_resource_blocks_readiness::<RenderMaterialInstances>(&mut renderer);
    renderer
        .set_snapshot(&Snapshot {
            epoch: 0,
            time: Duration::ZERO,
            origin: DVec3::ZERO,
            visuals: vec![BoxVisual {
                id: 1,
                pose: WorldPose {
                    translation: DVec3::ZERO,
                    rotation: glam::DQuat::IDENTITY,
                },
                size: DVec3::ONE,
                color: [1.0, 0.0, 0.0],
            }],
        })
        .unwrap();
    for _ in 0..200 {
        renderer.update().unwrap();
        if renderer.ready {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(renderer.ready);
    let entity = renderer.visuals[&1].entity;
    renderer
        .app
        .world_mut()
        .entity_mut(entity)
        .remove::<ViewVisibility>();
    assert!(!renderer.scene_assets_ready());
    renderer
        .app
        .world_mut()
        .entity_mut(entity)
        .insert(ViewVisibility::default());
    let render = renderer.app.remove_sub_app(RenderApp).unwrap();
    assert!(!renderer.scene_assets_ready());
    renderer.app.insert_sub_app(RenderApp, render);
    renderer.pending = Some(Pending {
        epoch: 0,
        time: Duration::ZERO,
        valid: true,
        pixels: Arc::new(Mutex::new(Some(None))),
    });
    assert!(matches!(
        renderer.take_frame(),
        Err(RenderError::ReadbackFailed)
    ));
    assert!(renderer.take_frame().unwrap().is_none());
    let handler = renderer.app.world().resource::<RenderErrorHandler>().0;
    let error = bevy::render::error_handler::RenderError {
        ty: bevy::render::error_handler::ErrorType::DeviceLost,
        description: "device lost".into(),
        source: None,
    };
    let policy = handler(&error, renderer.app.world_mut(), &mut World::new());
    assert!(matches!(policy, RenderErrorPolicy::StopRendering));
    assert!(matches!(
        renderer.update(),
        Err(RenderError::RenderStopped(_))
    ));
    renderer.app.world_mut().resource_mut::<DeviceFailure>().0 = None;
    renderer
        .app
        .add_systems(Update, |mut failure: ResMut<DeviceFailure>| {
            failure.0 = Some(StoppedReason::Device("device lost".into()));
        });
    assert!(
        matches!(renderer.update(), Err(RenderError::RenderStopped(reason)) if reason == "device lost")
    );
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::RenderStopped(_))
    ));
    assert!(matches!(
        renderer.take_frame(),
        Err(RenderError::RenderStopped(_))
    ));
}

#[test]
fn concurrent_renderer_construction_and_drop() {
    let barrier = Arc::new(std::sync::Barrier::new(2));
    std::thread::scope(|scope| {
        let handles: Vec<_> = (0..2)
            .map(|_| {
                let barrier = barrier.clone();
                scope.spawn(move || {
                    barrier.wait();
                    let renderer = Renderer::new().unwrap();
                    assert!(!renderer.adapter_info().name.is_empty());
                    drop(renderer);
                })
            })
            .collect();
        for handle in handles {
            handle.join().unwrap();
        }
    });
}

#[test]
fn destroyed_device_is_reported_by_renderer() {
    if std::env::var_os("GZ_RENDER_DESTROY_DEVICE_TEST").is_some() {
        let mut renderer = Renderer::new().unwrap();
        renderer
            .set_snapshot(&Snapshot {
                epoch: 1,
                time: Duration::ZERO,
                origin: DVec3::ZERO,
                visuals: vec![],
            })
            .unwrap();
        renderer.update().unwrap();
        renderer
            .app
            .world()
            .resource::<RenderDevice>()
            .wgpu_device()
            .destroy();
        for _ in 0..200 {
            match renderer.update() {
                Err(RenderError::RenderStopped(reason)) => {
                    eprintln!("destroyed device reported: {reason}");
                    assert!(matches!(
                        renderer.update(),
                        Err(RenderError::RenderStopped(_))
                    ));
                    return;
                }
                Err(error) => panic!("unexpected device failure: {error}"),
                Ok(_) => std::thread::sleep(Duration::from_millis(10)),
            }
        }
        panic!("destroyed device was not reported");
    }
    let mut child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "gpu::tests::destroyed_device_is_reported_by_renderer",
            "--nocapture",
        ])
        .env("GZ_RENDER_DESTROY_DEVICE_TEST", "1")
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .unwrap();
    let deadline = std::time::Instant::now() + Duration::from_secs(30);
    let timed_out = loop {
        if child.try_wait().unwrap().is_some() {
            break false;
        }
        if std::time::Instant::now() >= deadline {
            child.kill().unwrap();
            break true;
        }
        std::thread::sleep(Duration::from_millis(5));
    };
    let output = child.wait_with_output().unwrap();
    assert!(!timed_out, "destroyed-device child timed out");
    assert!(
        output.status.success(),
        "destroyed-device child failed: {}\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(String::from_utf8_lossy(&output.stdout).contains("1 passed; 0 failed"));
    assert!(String::from_utf8_lossy(&output.stderr).contains("destroyed device reported:"));
}

#[test]
fn calibrated_projection_subview_and_corners_match_pixels() {
    use gz_render_frame::camera::CameraConfig;
    let calibration = Calibration::new(CameraConfig {
        width: 64,
        height: 64,
        fx: 32.0,
        fy: 32.0,
        cx: 16.0,
        cy: 40.0,
        skew: 0.0,
        tx: 0.0,
        ty: 0.0,
        near: 0.1,
        far: 10.0,
    })
    .unwrap();
    let mut projection = CalibratedProjection(calibration);
    projection.update(64.0, 64.0);
    assert_eq!(projection.far(), 10.0);
    let corners = projection.get_frustum_corners(-1.0, -2.0);
    assert_eq!(corners[0], bevy::math::Vec3A::new(1.5, -0.75, -1.0));
    assert_eq!(corners[1], bevy::math::Vec3A::new(1.5, 1.25, -1.0));
    assert_eq!(corners[2], bevy::math::Vec3A::new(-0.5, 1.25, -1.0));
    assert_eq!(corners[3], bevy::math::Vec3A::new(-0.5, -0.75, -1.0));
    assert_eq!(corners[4], bevy::math::Vec3A::new(3.0, -1.5, -2.0));
    assert_eq!(corners[5], bevy::math::Vec3A::new(3.0, 2.5, -2.0));
    assert_eq!(corners[6], bevy::math::Vec3A::new(-1.0, 2.5, -2.0));
    assert_eq!(corners[7], bevy::math::Vec3A::new(-1.0, -1.5, -2.0));
    let sub = SubCameraView {
        full_size: UVec2::new(64, 64),
        offset: Vec2::new(8.0, 8.0),
        size: UVec2::new(32, 32),
    };
    // Full-image pixel (24,24) is the centre of this 32×32 subview.
    let view = Vec4::new(0.25, 0.5, -1.0, 1.0);
    let clip = projection.get_clip_from_view_for_sub(&sub) * view;
    assert!((clip.x / clip.w).abs() < 1e-6);
    assert!((clip.y / clip.w).abs() < 1e-6);
}

fn directional_light() -> DirectionalWorldLight {
    DirectionalWorldLight {
        id: 2,
        rotation: glam::DQuat::IDENTITY,
        direction: DVec3::NEG_Z,
        color: [1.0, 0.5, 0.25],
        illuminance_lux: 1000.0,
    }
}

#[test]
fn directional_conversion_matches_local_pose_and_all_axes() {
    for rotation in [
        glam::DQuat::IDENTITY,
        glam::DQuat::from_rotation_x(0.7),
        glam::DQuat::from_rotation_y(-1.2),
        glam::DQuat::from_rotation_z(2.1),
    ] {
        for direction in [
            DVec3::X,
            DVec3::NEG_X,
            DVec3::Y,
            DVec3::NEG_Y,
            DVec3::Z,
            DVec3::NEG_Z,
        ] {
            for magnitude in [1e-300, 1.0, 1e300] {
                let light = DirectionalWorldLight {
                    rotation,
                    direction: direction * magnitude,
                    ..directional_light()
                };
                let (derived, transform) = derive_light(&light).unwrap();
                let world = rotation * direction;
                let expected = Vec3::new(world.x as f32, world.z as f32, -world.y as f32);
                assert!((transform.rotation * Vec3::NEG_Z - expected).length() < 1e-6);
                assert_eq!(transform.translation, Vec3::ZERO);
                assert_eq!(derived.illuminance, 1000.0);
                assert_eq!(derived.color, Color::linear_rgb(1.0, 0.5, 0.25));
                assert!(!derived.shadow_maps_enabled);
                assert!(!derived.contact_shadows_enabled);
            }
        }
    }
}

#[test]
fn directional_validation_rejects_nonfinite_zero_overflow_and_zero_id() {
    let base = directional_light();
    for invalid in [
        DirectionalWorldLight {
            id: 0,
            ..base.clone()
        },
        DirectionalWorldLight {
            direction: DVec3::ZERO,
            ..base.clone()
        },
        DirectionalWorldLight {
            direction: DVec3::splat(f64::NAN),
            ..base.clone()
        },
        DirectionalWorldLight {
            direction: DVec3::splat(f64::INFINITY),
            ..base.clone()
        },
        DirectionalWorldLight {
            color: [f64::NAN, 0.0, 0.0],
            ..base.clone()
        },
        DirectionalWorldLight {
            color: [-0.1, 0.0, 0.0],
            ..base.clone()
        },
        DirectionalWorldLight {
            color: [1.1, 0.0, 0.0],
            ..base.clone()
        },
        DirectionalWorldLight {
            illuminance_lux: -1.0,
            ..base.clone()
        },
        DirectionalWorldLight {
            illuminance_lux: f64::NAN,
            ..base.clone()
        },
        DirectionalWorldLight {
            illuminance_lux: f64::INFINITY,
            ..base.clone()
        },
        DirectionalWorldLight {
            illuminance_lux: f64::MAX,
            ..base.clone()
        },
        DirectionalWorldLight {
            illuminance_lux: f64::MIN_POSITIVE,
            ..base.clone()
        },
    ] {
        assert!(
            matches!(derive_light(&invalid), Err(RenderError::InvalidLight)),
            "{invalid:?}"
        );
    }
    for rotation in [
        glam::DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
        glam::DQuat::from_xyzw(f64::NAN, 0.0, 0.0, 1.0),
    ] {
        assert!(matches!(
            derive_light(&DirectionalWorldLight {
                rotation,
                ..base.clone()
            }),
            Err(RenderError::Pose(_))
        ));
    }
    for lux in [0.0, f64::from(f32::from_bits(1)), f64::from(f32::MAX)] {
        assert_eq!(
            derive_light(&DirectionalWorldLight {
                illuminance_lux: lux,
                ..base.clone()
            })
            .unwrap()
            .0
            .illuminance,
            lux as f32
        );
    }
}

#[test]
fn directional_install_is_atomic_and_retires_owned_entity() {
    let mut renderer = Renderer::new().unwrap();
    let snapshot = Snapshot {
        epoch: 1,
        time: Duration::ZERO,
        origin: DVec3::ZERO,
        visuals: vec![BoxVisual {
            id: 1,
            pose: WorldPose {
                translation: DVec3::ZERO,
                rotation: glam::DQuat::IDENTITY,
            },
            size: DVec3::ONE,
            color: [1.0, 0.0, 0.0],
        }],
    };
    let exposure = renderer
        .app
        .world()
        .get::<Exposure>(renderer.camera)
        .unwrap()
        .ev100;
    assert_eq!(exposure, 9.7);
    let mut light = directional_light();
    renderer
        .set_directional_snapshot(&snapshot, &light)
        .unwrap();
    let (_, entity) = renderer.light.unwrap();
    let material = renderer.visuals[&1].material.clone();
    let assert_lit = |renderer: &Renderer| {
        assert_eq!(renderer.light, Some((2, entity)));
        assert_eq!(
            renderer
                .app
                .world()
                .get::<DirectionalLight>(entity)
                .unwrap()
                .illuminance,
            1000.0
        );
        let installed = renderer
            .app
            .world()
            .get::<DirectionalLight>(entity)
            .unwrap();
        assert!(!installed.shadow_maps_enabled);
        assert!(!installed.contact_shadows_enabled);
        assert_eq!(
            renderer
                .app
                .world()
                .get::<Exposure>(renderer.camera)
                .unwrap()
                .ev100,
            exposure
        );
        assert_eq!(
            renderer
                .app
                .world()
                .get::<AmbientLight>(renderer.camera)
                .unwrap()
                .brightness,
            0.0
        );
        let material = renderer
            .app
            .world()
            .resource::<Assets<StandardMaterial>>()
            .get(&material)
            .unwrap();
        assert!(!material.unlit);
        assert_eq!(
            (material.reflectance, material.perceptual_roughness),
            (0.0, 1.0)
        );
    };
    assert_lit(&renderer);
    assert!(matches!(
        renderer.set_directional_snapshot(&snapshot, &light),
        Err(RenderError::SnapshotBusy)
    ));
    renderer.snapshot_dirty = false;
    renderer.pending = Some(Pending {
        epoch: 1,
        time: Duration::ZERO,
        valid: true,
        pixels: Arc::new(Mutex::new(None)),
    });
    light.id = 1;
    assert!(matches!(
        renderer.set_directional_snapshot(&snapshot, &light),
        Err(RenderError::InvalidLight)
    ));
    light.id = 3;
    let invalid_light = DirectionalWorldLight {
        direction: DVec3::ZERO,
        ..light.clone()
    };
    assert!(matches!(
        renderer.set_directional_snapshot(&snapshot, &invalid_light),
        Err(RenderError::InvalidLight)
    ));
    use gz_render_frame::camera::CameraConfig;
    let config = CameraConfig {
        width: 64,
        height: 64,
        fx: 32.0,
        fy: 32.0,
        cx: 32.0,
        cy: 32.0,
        skew: 0.0,
        tx: 0.0,
        ty: 0.0,
        near: 0.1,
        far: 10.0,
    };
    let camera_before = *renderer
        .app
        .world()
        .get::<Transform>(renderer.camera)
        .unwrap();
    assert!(matches!(
        renderer.set_calibrated_directional_snapshot(
            &snapshot,
            Calibration::new(CameraConfig {
                width: 65,
                ..config
            })
            .unwrap(),
            snapshot.visuals[0].pose,
            &light,
        ),
        Err(RenderError::InvalidCameraDimensions)
    ));
    assert!(matches!(
        renderer.set_calibrated_directional_snapshot(
            &snapshot,
            Calibration::new(config).unwrap(),
            WorldPose {
                rotation: glam::DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
                ..snapshot.visuals[0].pose
            },
            &light,
        ),
        Err(RenderError::Pose(_))
    ));
    assert_eq!(
        *renderer
            .app
            .world()
            .get::<Transform>(renderer.camera)
            .unwrap(),
        camera_before
    );
    assert_eq!(renderer.stamp, Some((snapshot.epoch, snapshot.time)));
    let mut invalid = snapshot.clone();
    invalid.visuals[0].size = DVec3::ZERO;
    assert!(matches!(
        renderer.set_directional_snapshot(&invalid, &light),
        Err(RenderError::InvalidVisual)
    ));
    assert_lit(&renderer);
    assert!(renderer.pending.as_ref().unwrap().valid);
    assert!(!renderer.snapshot_dirty);
    renderer
        .set_directional_snapshot(&snapshot, &light)
        .unwrap();
    let (_, replacement) = renderer.light.unwrap();
    assert_ne!(replacement, entity);
    assert!(renderer.app.world().get_entity(entity).is_err());
    assert!(!renderer.pending.as_ref().unwrap().valid);
    renderer.snapshot_dirty = false;
    renderer.set_snapshot(&snapshot).unwrap();
    assert!(renderer.light.is_none());
    assert_eq!(
        renderer
            .app
            .world()
            .get::<Exposure>(renderer.camera)
            .unwrap()
            .ev100,
        exposure
    );
    assert!(renderer.app.world().get_entity(replacement).is_err());
    assert!(
        renderer
            .app
            .world()
            .get::<AmbientLight>(renderer.camera)
            .is_none()
    );
    let material = renderer
        .app
        .world()
        .resource::<Assets<StandardMaterial>>()
        .get(&material)
        .unwrap();
    assert!(material.unlit);
    assert_eq!(
        (material.reflectance, material.perceptual_roughness),
        (
            StandardMaterial::default().reflectance,
            StandardMaterial::default().perceptual_roughness
        )
    );
}

fn depth_calibration() -> Calibration {
    Calibration::new(gz_render_frame::camera::CameraConfig {
        width: 64,
        height: 64,
        fx: 40.,
        fy: 48.,
        cx: 29.,
        cy: 35.,
        skew: 0.,
        tx: 0.,
        ty: 0.,
        near: 0.1,
        far: 100.,
    })
    .unwrap()
}

#[test]
fn depth_unprojects_actual_reverse_z_to_axial_metres_and_preserves_background() {
    let calibration = depth_calibration();
    let projection = calibration.render_projection();
    let inverse = projection.as_dmat4().inverse();
    let z = projection.project_point3(Vec3::new(0., 0., -3.)).z;
    let raw = z.to_le_bytes().repeat(64 * 64);
    let metres = depth::decode(&raw, inverse).unwrap();
    assert_eq!(metres.len(), 64 * 64);
    assert!(metres.iter().all(|value| (*value - 3.).abs() < 1e-4));
    let empty = depth::decode(&0_f32.to_le_bytes().repeat(64 * 64), inverse).unwrap();
    assert!(empty.iter().all(|value| *value == f32::INFINITY));
    let near = depth::decode(&1_f32.to_le_bytes().repeat(64 * 64), inverse).unwrap();
    assert!(near.iter().all(|value| (*value - 0.1).abs() < 1e-6));
    assert!(matches!(
        depth::decode(&raw[..raw.len() - 1], inverse),
        Err(RenderError::ReadbackFailed)
    ));
    for invalid in [f32::NAN, f32::INFINITY, -0.1, 1.1] {
        assert!(matches!(
            depth::decode(&invalid.to_le_bytes().repeat(64 * 64), inverse),
            Err(RenderError::ReadbackFailed)
        ));
    }
    assert!(matches!(
        depth::decode(&raw, glam::DMat4::NAN),
        Err(RenderError::ReadbackFailed)
    ));
}

#[test]
fn depth_near_admission_checks_oriented_boxes_before_touching_renderer_state() {
    let calibration = depth_calibration();
    let camera = WorldPose {
        translation: DVec3::splat(1e12),
        rotation: glam::DQuat::IDENTITY,
    };
    let mut snapshot = Snapshot {
        epoch: 1,
        time: Duration::ZERO,
        origin: camera.translation,
        visuals: vec![BoxVisual {
            id: 1,
            pose: WorldPose {
                translation: camera.translation + DVec3::new(3., 0., 0.),
                rotation: glam::DQuat::IDENTITY,
            },
            size: DVec3::splat(1.),
            color: [1., 0., 0.],
        }],
    };
    depth::validate_near(&snapshot, calibration, camera).unwrap();
    snapshot.visuals[0].pose.translation = camera.translation;
    assert!(matches!(
        depth::validate_near(&snapshot, calibration, camera),
        Err(RenderError::DepthNearClip)
    ));
    snapshot.visuals[0].pose.translation = camera.translation + DVec3::new(1., 0., 0.);
    snapshot.visuals[0].size = DVec3::new(0.1, 4., 0.1);
    depth::validate_near(&snapshot, calibration, camera).unwrap();
    snapshot.visuals[0].pose.rotation = glam::DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    assert!(matches!(
        depth::validate_near(&snapshot, calibration, camera),
        Err(RenderError::DepthNearClip)
    ));
    snapshot.visuals[0].size.x = f64::NAN;
    assert!(matches!(
        depth::validate_near(&snapshot, calibration, camera),
        Err(RenderError::InvalidVisual)
    ));
}

#[test]
fn depth_admission_and_missing_resources_preserve_single_slot_ownership() {
    let mut renderer = Renderer::new().unwrap();
    assert!(renderer.take_depth_frame().unwrap().is_none());
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NoSnapshot)
    ));
    let snapshot = Snapshot {
        epoch: 41,
        time: Duration::from_millis(17),
        origin: DVec3::ZERO,
        visuals: vec![],
    };
    let camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: glam::DQuat::IDENTITY,
    };
    renderer.set_snapshot(&snapshot).unwrap();
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NoDepthSnapshot)
    ));
    renderer.update().unwrap();
    renderer
        .set_calibrated_depth_snapshot(&snapshot, depth_calibration(), camera)
        .unwrap();
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NotReady)
    ));
    assert!(!renderer.readback_in_flight());
    for _ in 0..200 {
        renderer.update().unwrap();
        if renderer.ready {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(renderer.ready);
    let camera_entity = renderer
        .app
        .world_mut()
        .entity_mut(renderer.camera)
        .take::<bevy::render::sync_world::RenderEntity>()
        .unwrap();
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NotReady)
    ));
    assert!(!renderer.readback_in_flight());
    let render_entity = camera_entity.id();
    renderer
        .app
        .world_mut()
        .entity_mut(renderer.camera)
        .insert(camera_entity);
    let attachment = renderer
        .app
        .get_sub_app_mut(RenderApp)
        .unwrap()
        .world_mut()
        .entity_mut(render_entity)
        .take::<bevy::render::view::ViewDepthStencilTexture>()
        .unwrap();
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NotReady)
    ));
    assert!(!renderer.readback_in_flight());
    renderer
        .app
        .get_sub_app_mut(RenderApp)
        .unwrap()
        .world_mut()
        .entity_mut(render_entity)
        .insert(attachment);
    renderer.request_depth_readback().unwrap();
    let old_slot = renderer
        .depth_pending
        .as_ref()
        .unwrap()
        .pending
        .pixels
        .clone();
    let mut excessive = snapshot.clone();
    excessive.visuals = (0..=MAX_OBJECTS)
        .map(|index| BoxVisual {
            id: index as u64 + 1,
            pose: WorldPose {
                translation: DVec3::new(3., 0., 0.),
                rotation: glam::DQuat::IDENTITY,
            },
            size: DVec3::ONE,
            color: [1.; 3],
        })
        .collect();
    assert!(matches!(
        renderer.set_calibrated_depth_snapshot(&excessive, depth_calibration(), camera),
        Err(RenderError::InvalidIds)
    ));
    assert_eq!(renderer.stamp, Some((snapshot.epoch, snapshot.time)));
    assert!(renderer.depth_pending.as_ref().unwrap().pending.valid);
    assert!(Arc::ptr_eq(
        &old_slot,
        &renderer.depth_pending.as_ref().unwrap().pending.pixels
    ));
    renderer.app.world_mut().resource_mut::<DeviceFailure>().0 =
        Some(StoppedReason::Device("depth device lost".into()));
    assert!(
        matches!(renderer.take_depth_frame(), Err(RenderError::RenderStopped(reason)) if reason == "depth device lost")
    );
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::RenderStopped(_))
    ));
    assert!(matches!(
        renderer.set_calibrated_depth_snapshot(&snapshot, depth_calibration(), camera),
        Err(RenderError::RenderStopped(_))
    ));
    assert!(renderer.readback_in_flight());
}

#[test]
fn depth_completed_failures_retire_but_poison_keeps_the_slot_owned() {
    let mut renderer = Renderer::new().unwrap();
    let snapshot = Snapshot {
        epoch: 72,
        time: Duration::from_millis(9),
        origin: DVec3::ZERO,
        visuals: vec![],
    };
    let camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: glam::DQuat::IDENTITY,
    };
    renderer
        .set_calibrated_depth_snapshot(&snapshot, depth_calibration(), camera)
        .unwrap();
    for _ in 0..200 {
        renderer.update().unwrap();
        if renderer.ready {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(renderer.ready);
    for bytes in [None, Some(vec![0; 3])] {
        renderer.request_depth_readback().unwrap();
        let slot = renderer
            .depth_pending
            .as_ref()
            .unwrap()
            .pending
            .pixels
            .clone();
        // Wait for the real callback before injecting a corrupt completion; no
        // callback may race this fault or write into a retired test slot.
        for _ in 0..200 {
            renderer
                .app
                .world()
                .resource::<RenderDevice>()
                .poll(wgpu::PollType::Poll)
                .unwrap();
            if slot.lock().unwrap().is_some() {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(
            slot.lock().unwrap().is_some(),
            "actual depth copy did not complete"
        );
        *slot.lock().unwrap() = Some(bytes);
        assert!(matches!(
            renderer.take_depth_frame(),
            Err(RenderError::ReadbackFailed)
        ));
        assert!(!renderer.readback_in_flight());
        assert!(renderer.take_depth_frame().unwrap().is_none());
    }
    renderer.request_depth_readback().unwrap();
    let slot = renderer
        .depth_pending
        .as_ref()
        .unwrap()
        .pending
        .pixels
        .clone();
    std::thread::scope(|scope| {
        let slot = slot.clone();
        assert!(
            scope
                .spawn(move || {
                    let _guard = slot.lock().unwrap();
                    panic!("poison depth completion");
                })
                .join()
                .is_err()
        );
    });
    assert!(matches!(
        renderer.take_depth_frame(),
        Err(RenderError::ReadbackPoisoned)
    ));
    assert!(renderer.readback_in_flight());
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::Busy)
    ));
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    renderer.cancel_readback();
    assert!(renderer.readback_in_flight());
    assert!(!renderer.depth_pending.as_ref().unwrap().pending.valid);
    assert!(matches!(
        renderer.take_depth_frame(),
        Err(RenderError::ReadbackPoisoned)
    ));
}

#[test]
fn depth_rejects_finite_unprojection_that_is_not_forward() {
    let raw = 1_f32.to_le_bytes().repeat(64 * 64);
    assert!(matches!(
        depth::decode(&raw, glam::DMat4::IDENTITY),
        Err(RenderError::ReadbackFailed)
    ));
    let snapshot = Snapshot {
        epoch: 0,
        time: Duration::ZERO,
        origin: DVec3::ZERO,
        visuals: vec![],
    };
    let invalid = WorldPose {
        translation: DVec3::ZERO,
        rotation: glam::DQuat::from_xyzw(0., 0., 0., 0.),
    };
    assert!(matches!(
        depth::validate_near(&snapshot, depth_calibration(), invalid),
        Err(RenderError::Pose(_))
    ));
}

fn primitive_calibration() -> Calibration {
    Calibration::from_horizontal_fov(64, 64, std::f64::consts::FRAC_PI_2, 0.1, 100.0).unwrap()
}
fn primitive_cylinder() -> CylinderVisual {
    CylinderVisual {
        id: 1,
        pose: WorldPose {
            translation: DVec3::new(4.0, 0.0, 0.0),
            rotation: glam::DQuat::IDENTITY,
        },
        radius: 1.0,
        length: 2.0,
        color: [1.0, 0.0, 0.0],
    }
}

#[test]
fn cylinder_dimensions_and_mixed_ids_preflight_before_scene_mutation() {
    let mut renderer = Renderer::new().unwrap();
    let calibration = primitive_calibration();
    let camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: glam::DQuat::IDENTITY,
    };
    let baseline = Snapshot {
        epoch: 7,
        time: Duration::from_millis(1),
        origin: DVec3::ZERO,
        visuals: vec![BoxVisual {
            id: 2,
            pose: primitive_cylinder().pose,
            size: DVec3::ONE,
            color: [0.0, 1.0, 0.0],
        }],
    };
    renderer
        .set_calibrated_snapshot(&baseline, calibration, camera)
        .unwrap();
    // Private lifecycle-state fixture; this is not an image or GPU completion receipt.
    renderer.snapshot_dirty = false;
    renderer.pending = Some(Pending {
        epoch: 7,
        time: baseline.time,
        valid: true,
        pixels: Arc::new(Mutex::new(None)),
    });
    let before_meshes = renderer.app.world().resource::<Assets<Mesh>>().len();
    let before_materials = renderer
        .app
        .world()
        .resource::<Assets<StandardMaterial>>()
        .len();
    let before_camera = *renderer
        .app
        .world()
        .get::<Transform>(renderer.camera)
        .unwrap();
    let before_entity = renderer.visuals[&2].entity;
    let before_mesh = renderer.visuals[&2].mesh.id();
    let valid = PrimitiveSnapshot {
        epoch: 8,
        time: Duration::from_millis(2),
        origin: DVec3::ZERO,
        visuals: vec![PrimitiveVisual::Cylinder(primitive_cylinder())],
    };
    let assert_unchanged = |renderer: &Renderer| {
        assert!(renderer.cylinder_mesh.is_none());
        assert_eq!(
            renderer.app.world().resource::<Assets<Mesh>>().len(),
            before_meshes
        );
        assert_eq!(
            renderer
                .app
                .world()
                .resource::<Assets<StandardMaterial>>()
                .len(),
            before_materials
        );
        assert_eq!(renderer.stamp, Some((baseline.epoch, baseline.time)));
        assert_eq!(
            *renderer
                .app
                .world()
                .get::<Transform>(renderer.camera)
                .unwrap(),
            before_camera
        );
        assert_eq!(renderer.visuals[&2].entity, before_entity);
        assert_eq!(renderer.visuals[&2].mesh.id(), before_mesh);
        assert_eq!(renderer.visuals.len(), 1);
        assert!(renderer.pending.as_ref().unwrap().valid);
        assert!(!renderer.snapshot_dirty);
    };
    for value in [
        0.0,
        -1.0,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::MAX,
        1e-300,
    ] {
        for radius in [true, false] {
            let mut cylinder = primitive_cylinder();
            if radius {
                cylinder.radius = value;
            } else {
                cylinder.length = value;
            }
            let mut invalid = valid.clone();
            invalid.visuals[0] = PrimitiveVisual::Cylinder(cylinder);
            assert!(matches!(
                renderer.set_calibrated_primitive_snapshot(&invalid, calibration, camera),
                Err(RenderError::InvalidVisual)
            ));
            assert_unchanged(&renderer);
        }
    }
    let mut diameter_overflow = valid.clone();
    diameter_overflow.visuals[0] = PrimitiveVisual::Cylinder(CylinderVisual {
        radius: 2e38,
        ..primitive_cylinder()
    });
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&diameter_overflow, calibration, camera),
        Err(RenderError::InvalidVisual)
    ));
    assert_unchanged(&renderer);
    for fault in 0..5 {
        let mut invalid = valid.clone();
        if fault == 1 {
            invalid.visuals.push(PrimitiveVisual::Box(BoxVisual {
                id: 1,
                pose: camera,
                size: DVec3::ONE,
                color: [0.0; 3],
            }));
        } else {
            let PrimitiveVisual::Cylinder(c) = &mut invalid.visuals[0] else {
                unreachable!()
            };
            match fault {
                0 => c.id = 0,
                2 => c.color[0] = f32::NAN,
                3 => c.pose.rotation = glam::DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
                _ => c.pose.translation.x = f64::NAN,
            }
        }
        assert!(
            renderer
                .set_calibrated_primitive_snapshot(&invalid, calibration, camera)
                .is_err()
        );
        assert_unchanged(&renderer);
    }
    let wrong =
        Calibration::from_horizontal_fov(32, 64, std::f64::consts::FRAC_PI_2, 0.1, 100.0).unwrap();
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&valid, wrong, camera),
        Err(RenderError::InvalidCameraDimensions)
    ));
    assert_unchanged(&renderer);
    let mut full = valid.clone();
    full.visuals = (1_u64..=129)
        .map(|id| {
            if id.is_multiple_of(2) {
                PrimitiveVisual::Box(BoxVisual {
                    id,
                    pose: camera,
                    size: DVec3::ONE,
                    color: [0.0; 3],
                })
            } else {
                PrimitiveVisual::Cylinder(CylinderVisual {
                    id,
                    ..primitive_cylinder()
                })
            }
        })
        .collect();
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&full, calibration, camera),
        Err(RenderError::InvalidIds)
    ));
    assert_unchanged(&renderer);
    full.visuals.pop();
    renderer
        .set_calibrated_primitive_snapshot(&full, calibration, camera)
        .unwrap();
    assert_eq!(renderer.visuals.len(), 128);
    assert_eq!(
        renderer.app.world().resource::<Assets<Mesh>>().len(),
        before_meshes + 1
    );
    assert!(!renderer.pending.as_ref().unwrap().valid);
}

#[test]
fn unit_cylinder_axis_caps_and_shape_replacement_reuse_bounded_meshes() {
    let mut renderer = Renderer::new().unwrap();
    let original_mesh_count = renderer.app.world().resource::<Assets<Mesh>>().len();
    assert!(renderer.cylinder_mesh.is_none());
    let camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: glam::DQuat::IDENTITY,
    };
    let old_depth = Snapshot {
        epoch: 0,
        time: Duration::ZERO,
        origin: DVec3::ZERO,
        visuals: vec![BoxVisual {
            id: 1,
            pose: primitive_cylinder().pose,
            size: DVec3::ONE,
            color: [0.0, 1.0, 0.0],
        }],
    };
    renderer
        .set_calibrated_depth_snapshot(&old_depth, primitive_calibration(), camera)
        .unwrap();
    assert!(renderer.depth_calibration.is_some());
    renderer.snapshot_dirty = false;
    let mut cylinder = primitive_cylinder();
    cylinder.length = 4.0;
    cylinder.pose.rotation = glam::DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);
    let mut snapshot = PrimitiveSnapshot {
        epoch: 1,
        time: Duration::ZERO,
        origin: DVec3::ZERO,
        visuals: vec![PrimitiveVisual::Cylinder(cylinder)],
    };
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, primitive_calibration(), camera)
        .unwrap();
    let mesh = renderer.cylinder_mesh.as_ref().unwrap().clone();
    assert_ne!(mesh.id(), renderer.mesh.id());
    assert_eq!(renderer.visuals[&1].mesh.id(), mesh.id());
    let transform = renderer
        .app
        .world()
        .get::<Transform>(renderer.visuals[&1].entity)
        .unwrap();
    assert_eq!(transform.scale, Vec3::new(2.0, 4.0, 2.0));
    assert!((transform.rotation * Vec3::Y - Vec3::Z).length() < 1e-6);
    let assets = renderer.app.world().resource::<Assets<Mesh>>();
    let cylinder_mesh = assets.get(&mesh).unwrap();
    let Some(bevy::mesh::VertexAttributeValues::Float32x3(positions)) =
        cylinder_mesh.attribute(Mesh::ATTRIBUTE_POSITION)
    else {
        panic!("missing cylinder positions");
    };
    assert_eq!(positions.len(), 130);
    assert_eq!(cylinder_mesh.indices().unwrap().len(), 372);
    assert!(
        positions
            .iter()
            .all(|p| (p[0] * p[0] + p[2] * p[2] - 0.25).abs() < 1e-6 && p[1].abs() == 0.5)
    );
    let Some(bevy::mesh::VertexAttributeValues::Float32x3(normals)) =
        cylinder_mesh.attribute(Mesh::ATTRIBUTE_NORMAL)
    else {
        panic!("missing cylinder normals");
    };
    assert_eq!(
        normals.iter().filter(|n| **n == [0.0, 1.0, 0.0]).count(),
        32
    );
    assert_eq!(
        normals.iter().filter(|n| **n == [0.0, -1.0, 0.0]).count(),
        32
    );
    for index in 0_usize..4 {
        renderer.snapshot_dirty = false;
        snapshot.visuals[0] = if index.is_multiple_of(2) {
            PrimitiveVisual::Box(BoxVisual {
                id: 1,
                pose: primitive_cylinder().pose,
                size: DVec3::ONE,
                color: [0.0, 1.0, 0.0],
            })
        } else {
            PrimitiveVisual::Cylinder(primitive_cylinder())
        };
        renderer
            .set_calibrated_primitive_snapshot(&snapshot, primitive_calibration(), camera)
            .unwrap();
        let expected = if index.is_multiple_of(2) {
            renderer.mesh.id()
        } else {
            mesh.id()
        };
        assert_eq!(renderer.visuals[&1].mesh.id(), expected);
        assert_eq!(
            renderer
                .app
                .world()
                .get::<Mesh3d>(renderer.visuals[&1].entity)
                .unwrap()
                .0
                .id(),
            expected
        );
        assert_eq!(
            renderer.app.world().resource::<Assets<Mesh>>().len(),
            original_mesh_count + 1
        );
    }
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NoDepthSnapshot)
    ));
    for _ in 0..200 {
        renderer.update().unwrap();
        if renderer.ready {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(renderer.ready && renderer.scene_assets_ready());
    let box_id = renderer.mesh.id();
    let cylinder_id = mesh.id();
    let old_box = renderer
        .app
        .get_sub_app_mut(RenderApp)
        .unwrap()
        .world_mut()
        .resource_mut::<RenderAssets<RenderMesh>>()
        .remove(box_id);
    assert!(
        renderer.scene_assets_ready(),
        "unselected cuboid cannot gate cylinder readiness"
    );
    let old_cylinder = renderer
        .app
        .get_sub_app_mut(RenderApp)
        .unwrap()
        .world_mut()
        .resource_mut::<RenderAssets<RenderMesh>>()
        .remove(cylinder_id)
        .unwrap();
    assert!(
        !renderer.scene_assets_ready(),
        "selected cylinder is required for readiness"
    );
    let mut assets = renderer
        .app
        .get_sub_app_mut(RenderApp)
        .unwrap()
        .world_mut()
        .resource_mut::<RenderAssets<RenderMesh>>();
    if let Some(old_box) = old_box {
        assets.insert(box_id, old_box);
    }
    assets.insert(cylinder_id, old_cylinder);
}

fn primitive_sphere() -> SphereVisual {
    SphereVisual {
        id: 1,
        pose: primitive_cylinder().pose,
        radius: 1.0,
        color: [1.0, 0.0, 0.0],
    }
}

#[test]
fn sphere_preflight_preserves_mixed_scene_and_pending_capture_before_mesh_allocation() {
    let mut renderer = Renderer::new().unwrap();
    let camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: glam::DQuat::IDENTITY,
    };
    let baseline = Snapshot {
        epoch: 91,
        time: Duration::from_millis(7),
        origin: DVec3::ZERO,
        visuals: vec![BoxVisual {
            id: 2,
            pose: primitive_sphere().pose,
            size: DVec3::ONE,
            color: [0.0, 1.0, 0.0],
        }],
    };
    renderer
        .set_calibrated_snapshot(&baseline, primitive_calibration(), camera)
        .unwrap();
    // A private lifecycle fixture, not a claim of GPU completion or image pixels.
    renderer.snapshot_dirty = false;
    let slot = Arc::new(Mutex::new(None));
    renderer.pending = Some(Pending {
        epoch: baseline.epoch,
        time: baseline.time,
        valid: true,
        pixels: slot.clone(),
    });
    let meshes = renderer.app.world().resource::<Assets<Mesh>>().len();
    let materials = renderer
        .app
        .world()
        .resource::<Assets<StandardMaterial>>()
        .len();
    let entity = renderer.visuals[&2].entity;
    let material = renderer.visuals[&2].material.id();
    let transform = *renderer
        .app
        .world()
        .get::<Transform>(renderer.camera)
        .unwrap();
    let valid = PrimitiveSnapshot {
        epoch: 92,
        time: Duration::ZERO,
        origin: DVec3::ZERO,
        visuals: vec![PrimitiveVisual::Sphere(primitive_sphere())],
    };
    let unchanged = |r: &Renderer| {
        assert!(r.sphere_mesh.is_none() && r.cylinder_mesh.is_none());
        assert_eq!(r.app.world().resource::<Assets<Mesh>>().len(), meshes);
        assert_eq!(
            r.app.world().resource::<Assets<StandardMaterial>>().len(),
            materials
        );
        assert_eq!(r.visuals.len(), 1);
        assert_eq!(r.visuals[&2].entity, entity);
        assert_eq!(r.visuals[&2].material.id(), material);
        assert_eq!(r.visuals[&2].mesh.id(), r.mesh.id());
        assert_eq!(
            *r.app.world().get::<Transform>(r.camera).unwrap(),
            transform
        );
        assert_eq!(r.stamp, Some((baseline.epoch, baseline.time)));
        assert!(!r.snapshot_dirty);
        let pending = r.pending.as_ref().unwrap();
        assert!(pending.valid && Arc::ptr_eq(&pending.pixels, &slot));
    };
    for radius in [
        0.0,
        -1.0,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::MAX,
        1e-300,
        2e38,
    ] {
        let mut invalid = valid.clone();
        invalid.visuals[0] = PrimitiveVisual::Sphere(SphereVisual {
            radius,
            ..primitive_sphere()
        });
        assert!(matches!(
            renderer.set_calibrated_primitive_snapshot(&invalid, primitive_calibration(), camera),
            Err(RenderError::InvalidVisual)
        ));
        unchanged(&renderer);
    }
    for fault in 0..8 {
        let mut invalid = valid.clone();
        match fault {
            0 => {
                let PrimitiveVisual::Sphere(sphere) = &mut invalid.visuals[0] else {
                    unreachable!()
                };
                sphere.id = 0;
            }
            1 => invalid.visuals.push(PrimitiveVisual::Box(BoxVisual {
                id: 1,
                pose: camera,
                size: DVec3::ONE,
                color: [0.0; 3],
            })),
            2 => invalid
                .visuals
                .push(PrimitiveVisual::Cylinder(primitive_cylinder())),
            3..=6 => {
                let PrimitiveVisual::Sphere(sphere) = &mut invalid.visuals[0] else {
                    unreachable!()
                };
                match fault {
                    3 => sphere.color[0] = f32::NAN,
                    4 => sphere.color[1] = 1.1,
                    5 => sphere.pose.rotation = glam::DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
                    _ => sphere.pose.translation.x = f64::NAN,
                }
            }
            _ => invalid.origin.x = f64::INFINITY,
        }
        assert!(
            renderer
                .set_calibrated_primitive_snapshot(&invalid, primitive_calibration(), camera)
                .is_err()
        );
        unchanged(&renderer);
    }
    let wrong =
        Calibration::from_horizontal_fov(32, 64, std::f64::consts::FRAC_PI_2, 0.1, 20.0).unwrap();
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&valid, wrong, camera),
        Err(RenderError::InvalidCameraDimensions)
    ));
    unchanged(&renderer);
    let bad_camera = WorldPose {
        rotation: glam::DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
        ..camera
    };
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&valid, primitive_calibration(), bad_camera),
        Err(RenderError::Pose(_))
    ));
    unchanged(&renderer);
    let mut full = valid.clone();
    full.visuals = (1_u64..=129)
        .map(|id| match id % 3 {
            0 => PrimitiveVisual::Box(BoxVisual {
                id,
                pose: camera,
                size: DVec3::ONE,
                color: [0.0; 3],
            }),
            1 => PrimitiveVisual::Sphere(SphereVisual {
                id,
                ..primitive_sphere()
            }),
            _ => PrimitiveVisual::Cylinder(CylinderVisual {
                id,
                ..primitive_cylinder()
            }),
        })
        .collect();
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&full, primitive_calibration(), camera),
        Err(RenderError::InvalidIds)
    ));
    unchanged(&renderer);
    full.visuals.pop();
    renderer
        .set_calibrated_primitive_snapshot(&full, primitive_calibration(), camera)
        .unwrap();
    assert_eq!(renderer.visuals.len(), 128);
    assert_eq!(
        renderer.app.world().resource::<Assets<Mesh>>().len(),
        meshes + 2
    );
    assert!(renderer.sphere_mesh.is_some() && renderer.cylinder_mesh.is_some());
    assert!(!renderer.pending.as_ref().unwrap().valid);
}

#[test]
fn unit_uv_sphere_and_three_shape_replacements_keep_selected_mesh_and_asset_bounds() {
    let mut renderer = Renderer::new().unwrap();
    let original_meshes = renderer.app.world().resource::<Assets<Mesh>>().len();
    let original_materials = renderer
        .app
        .world()
        .resource::<Assets<StandardMaterial>>()
        .len();
    assert!(renderer.sphere_mesh.is_none() && renderer.cylinder_mesh.is_none());
    let camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: glam::DQuat::IDENTITY,
    };
    let mut snapshot = PrimitiveSnapshot {
        epoch: 93,
        time: Duration::ZERO,
        origin: DVec3::ZERO,
        visuals: vec![PrimitiveVisual::Sphere(primitive_sphere())],
    };
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, primitive_calibration(), camera)
        .unwrap();
    let mesh = renderer.sphere_mesh.as_ref().unwrap().clone();
    assert!(renderer.cylinder_mesh.is_none());
    let entity = renderer.visuals[&1].entity;
    let material = renderer.visuals[&1].material.id();
    assert_ne!(mesh.id(), renderer.mesh.id());
    {
        let assets = renderer.app.world().resource::<Assets<Mesh>>();
        let sphere = assets.get(&mesh).unwrap();
        let Some(bevy::mesh::VertexAttributeValues::Float32x3(positions)) =
            sphere.attribute(Mesh::ATTRIBUTE_POSITION)
        else {
            panic!("missing sphere positions")
        };
        let Some(bevy::mesh::VertexAttributeValues::Float32x3(normals)) =
            sphere.attribute(Mesh::ATTRIBUTE_NORMAL)
        else {
            panic!("missing sphere normals")
        };
        let Some(bevy::mesh::VertexAttributeValues::Float32x2(uvs)) =
            sphere.attribute(Mesh::ATTRIBUTE_UV_0)
        else {
            panic!("missing sphere UVs")
        };
        assert_eq!(positions.len(), 33 * 19);
        assert_eq!(normals.len(), positions.len());
        assert_eq!(uvs.len(), positions.len());
        assert_eq!(sphere.indices().unwrap().len(), 32 * 34 * 3);
        for (position, normal) in positions.iter().zip(normals) {
            let p = Vec3::from_array(*position);
            let n = Vec3::from_array(*normal);
            assert!(p.is_finite() && n.is_finite());
            assert!((p.length_squared() - 0.25).abs() < 1e-6);
            assert!((n.length_squared() - 1.0).abs() < 1e-6);
            assert!((n - p * 2.0).length() < 1e-6);
        }
        assert!(
            uvs.iter()
                .flatten()
                .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
        );
    }
    for (index, kind) in [0, 1, 2, 0, 2, 1, 2].into_iter().enumerate() {
        // Private settled-state fixture: real asynchronous replacement is tested in software.rs.
        renderer.snapshot_dirty = false;
        snapshot.visuals[0] = match kind {
            0 => PrimitiveVisual::Box(BoxVisual {
                id: 1,
                pose: primitive_sphere().pose,
                size: DVec3::ONE,
                color: [0.0, 1.0, 0.0],
            }),
            1 => PrimitiveVisual::Cylinder(primitive_cylinder()),
            _ => PrimitiveVisual::Sphere(SphereVisual {
                radius: 1.0 + index as f64 * 0.125,
                ..primitive_sphere()
            }),
        };
        renderer
            .set_calibrated_primitive_snapshot(&snapshot, primitive_calibration(), camera)
            .unwrap();
        let expected = match kind {
            0 => renderer.mesh.id(),
            1 => renderer.cylinder_mesh.as_ref().unwrap().id(),
            _ => mesh.id(),
        };
        assert_eq!(renderer.visuals[&1].entity, entity);
        assert_eq!(renderer.visuals[&1].material.id(), material);
        assert_eq!(renderer.visuals[&1].mesh.id(), expected);
        assert_eq!(
            renderer.app.world().get::<Mesh3d>(entity).unwrap().0.id(),
            expected
        );
        assert_eq!(
            renderer
                .app
                .world()
                .resource::<Assets<StandardMaterial>>()
                .len(),
            original_materials + 1
        );
        assert!(renderer.app.world().resource::<Assets<Mesh>>().len() <= original_meshes + 2);
        if kind == 2 {
            let transform = renderer.app.world().get::<Transform>(entity).unwrap();
            assert_eq!(
                transform.scale,
                Vec3::splat((2.0 + index as f64 * 0.25) as f32)
            );
            let expected = to_render_pose(primitive_sphere().pose, DVec3::ZERO).unwrap();
            assert_eq!(transform.translation, expected.translation);
            assert_eq!(transform.rotation, expected.rotation);
        }
    }
    assert_eq!(
        renderer.app.world().resource::<Assets<Mesh>>().len(),
        original_meshes + 2
    );
    for _ in 0..200 {
        renderer.update().unwrap();
        if renderer.ready {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(renderer.ready && renderer.scene_assets_ready());
    let box_id = renderer.mesh.id();
    let cylinder_id = renderer.cylinder_mesh.as_ref().unwrap().id();
    let sphere_id = mesh.id();
    let (old_box, old_cylinder) = {
        let mut assets = renderer
            .app
            .get_sub_app_mut(RenderApp)
            .unwrap()
            .world_mut()
            .resource_mut::<RenderAssets<RenderMesh>>();
        (assets.remove(box_id), assets.remove(cylinder_id))
    };
    assert!(
        renderer.scene_assets_ready(),
        "unselected box/cylinder cannot gate sphere readiness"
    );
    let old_sphere = renderer
        .app
        .get_sub_app_mut(RenderApp)
        .unwrap()
        .world_mut()
        .resource_mut::<RenderAssets<RenderMesh>>()
        .remove(sphere_id)
        .unwrap();
    assert!(
        !renderer.scene_assets_ready(),
        "selected sphere must be ready"
    );
    let mut assets = renderer
        .app
        .get_sub_app_mut(RenderApp)
        .unwrap()
        .world_mut()
        .resource_mut::<RenderAssets<RenderMesh>>();
    if let Some(mesh) = old_box {
        assets.insert(box_id, mesh);
    }
    if let Some(mesh) = old_cylinder {
        assets.insert(cylinder_id, mesh);
    }
    assets.insert(sphere_id, old_sphere);
}

#[test]
fn sphere_snapshot_busy_depth_and_cancelled_slot_keep_their_original_ownership() {
    let mut renderer = Renderer::new().unwrap();
    let original_meshes = renderer.app.world().resource::<Assets<Mesh>>().len();
    let original_materials = renderer
        .app
        .world()
        .resource::<Assets<StandardMaterial>>()
        .len();
    let camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: glam::DQuat::IDENTITY,
    };
    let old = Snapshot {
        epoch: 94,
        time: Duration::from_millis(8),
        origin: DVec3::ZERO,
        visuals: vec![BoxVisual {
            id: 1,
            pose: primitive_sphere().pose,
            size: DVec3::ONE,
            color: [1.0; 3],
        }],
    };
    renderer
        .set_calibrated_depth_snapshot(&old, primitive_calibration(), camera)
        .unwrap();
    let mut sphere = PrimitiveSnapshot {
        epoch: 95,
        time: Duration::ZERO,
        origin: DVec3::ZERO,
        visuals: vec![PrimitiveVisual::Sphere(primitive_sphere())],
    };
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&sphere, primitive_calibration(), camera),
        Err(RenderError::SnapshotBusy)
    ));
    assert!(renderer.sphere_mesh.is_none() && renderer.depth_calibration.is_some());
    assert_eq!(renderer.stamp, Some((old.epoch, old.time)));
    // A buffered callback-state fixture, never a fabricated software-rendered frame.
    renderer.snapshot_dirty = false;
    let pixels = Arc::new(Mutex::new(Some(Some(vec![
        0;
        (WIDTH * HEIGHT * 4) as usize
    ]))));
    renderer.pending = Some(Pending {
        epoch: old.epoch,
        time: old.time,
        valid: true,
        pixels: pixels.clone(),
    });
    renderer
        .set_calibrated_primitive_snapshot(&sphere, primitive_calibration(), camera)
        .unwrap();
    assert!(renderer.depth_calibration.is_none());
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    assert!(Arc::ptr_eq(
        &renderer.pending.as_ref().unwrap().pixels,
        &pixels
    ));
    assert!(!renderer.pending.as_ref().unwrap().valid);
    assert!(renderer.take_frame().unwrap().is_none());
    assert!(!renderer.readback_in_flight());
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NoDepthSnapshot)
    ));
    assert_eq!(renderer.stamp, Some((sphere.epoch, sphere.time)));
    renderer.snapshot_dirty = false;
    sphere.epoch += 1;
    sphere.visuals.clear();
    renderer
        .set_calibrated_primitive_snapshot(&sphere, primitive_calibration(), camera)
        .unwrap();
    assert!(renderer.visuals.is_empty());
    assert_eq!(
        renderer
            .app
            .world()
            .resource::<Assets<StandardMaterial>>()
            .len(),
        original_materials
    );
    assert_eq!(
        renderer.app.world().resource::<Assets<Mesh>>().len(),
        original_meshes + 1
    );
    assert_eq!(renderer.stamp, Some((sphere.epoch, Duration::ZERO)));
    renderer.app.world_mut().resource_mut::<DeviceFailure>().0 =
        Some(StoppedReason::Device("sphere device lost".into()));
    sphere
        .visuals
        .push(PrimitiveVisual::Sphere(primitive_sphere()));
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&sphere, primitive_calibration(), camera),
        Err(RenderError::RenderStopped(_))
    ));
    assert!(renderer.visuals.is_empty());
    assert_eq!(
        renderer.app.world().resource::<Assets<Mesh>>().len(),
        original_meshes + 1
    );
}

fn primitive_ellipsoid() -> EllipsoidVisual {
    EllipsoidVisual {
        id: 1,
        pose: primitive_sphere().pose,
        radii: DVec3::new(0.5, 1.5, 0.75),
        color: [1.0, 0.0, 0.0],
    }
}

#[test]
fn ellipsoid_each_axis_preflight_preserves_pending_scene_and_shared_bound() {
    let mut renderer = Renderer::new().unwrap();
    let camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: glam::DQuat::IDENTITY,
    };
    let old = Snapshot {
        epoch: 101,
        time: Duration::from_millis(8),
        origin: DVec3::ZERO,
        visuals: vec![BoxVisual {
            id: 2,
            pose: primitive_sphere().pose,
            size: DVec3::ONE,
            color: [0.0, 1.0, 0.0],
        }],
    };
    renderer
        .set_calibrated_snapshot(&old, primitive_calibration(), camera)
        .unwrap();
    // Lifecycle-state fixture only; the software test requires real GPU completion.
    renderer.snapshot_dirty = false;
    let pixels = Arc::new(Mutex::new(None));
    renderer.pending = Some(Pending {
        epoch: old.epoch,
        time: old.time,
        valid: true,
        pixels: pixels.clone(),
    });
    let entity = renderer.visuals[&2].entity;
    let material = renderer.visuals[&2].material.id();
    let meshes = renderer.app.world().resource::<Assets<Mesh>>().len();
    let materials = renderer
        .app
        .world()
        .resource::<Assets<StandardMaterial>>()
        .len();
    let camera_transform = *renderer
        .app
        .world()
        .get::<Transform>(renderer.camera)
        .unwrap();
    let unchanged = |r: &Renderer| {
        assert!(r.sphere_mesh.is_none() && r.cylinder_mesh.is_none());
        assert_eq!(r.app.world().resource::<Assets<Mesh>>().len(), meshes);
        assert_eq!(
            r.app.world().resource::<Assets<StandardMaterial>>().len(),
            materials
        );
        assert_eq!(r.visuals.len(), 1);
        assert_eq!(r.visuals[&2].entity, entity);
        assert_eq!(r.visuals[&2].material.id(), material);
        assert_eq!(r.visuals[&2].mesh.id(), r.mesh.id());
        assert_eq!(
            *r.app.world().get::<Transform>(r.camera).unwrap(),
            camera_transform
        );
        assert_eq!(r.stamp, Some((old.epoch, old.time)));
        assert!(!r.snapshot_dirty);
        let pending = r.pending.as_ref().unwrap();
        assert!(pending.valid && Arc::ptr_eq(&pending.pixels, &pixels));
    };
    let valid = PrimitiveSnapshot {
        epoch: 102,
        time: Duration::ZERO,
        origin: DVec3::ZERO,
        visuals: vec![PrimitiveVisual::Ellipsoid(primitive_ellipsoid())],
    };
    for axis in 0..3 {
        for radius in [
            0.0,
            -1.0,
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::MAX,
            1e-300,
            2e38,
        ] {
            let mut ellipsoid = primitive_ellipsoid();
            ellipsoid.radii[axis] = radius;
            let invalid = PrimitiveSnapshot {
                visuals: vec![PrimitiveVisual::Ellipsoid(ellipsoid)],
                ..valid.clone()
            };
            assert!(matches!(
                renderer.set_calibrated_primitive_snapshot(
                    &invalid,
                    primitive_calibration(),
                    camera
                ),
                Err(RenderError::InvalidVisual)
            ));
            unchanged(&renderer);
        }
    }
    for fault in 0..8 {
        let mut ellipsoid = primitive_ellipsoid();
        let mut invalid = valid.clone();
        let mut selected_camera = camera;
        let mut calibration = primitive_calibration();
        match fault {
            0 => ellipsoid.id = 0,
            1 => invalid
                .visuals
                .push(PrimitiveVisual::Sphere(primitive_sphere())),
            2 => ellipsoid.color[0] = f32::NAN,
            3 => ellipsoid.color[1] = 1.01,
            4 => ellipsoid.pose.rotation = glam::DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
            5 => ellipsoid.pose.translation.x = f64::NAN,
            6 => selected_camera.rotation = glam::DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
            _ => {
                calibration = gz_render_frame::camera::Calibration::from_horizontal_fov(
                    32,
                    64,
                    std::f64::consts::FRAC_PI_2,
                    0.1,
                    20.0,
                )
                .unwrap()
            }
        }
        invalid.visuals[0] = PrimitiveVisual::Ellipsoid(ellipsoid);
        assert!(
            renderer
                .set_calibrated_primitive_snapshot(&invalid, calibration, selected_camera)
                .is_err()
        );
        unchanged(&renderer);
    }
    let mut mixed = valid.clone();
    mixed.visuals = (1..=129)
        .map(|id| match id % 4 {
            0 => PrimitiveVisual::Box(BoxVisual {
                id,
                pose: primitive_sphere().pose,
                size: DVec3::ONE,
                color: [0.0; 3],
            }),
            1 => PrimitiveVisual::Sphere(SphereVisual {
                id,
                ..primitive_sphere()
            }),
            2 => PrimitiveVisual::Cylinder(CylinderVisual {
                id,
                ..primitive_cylinder()
            }),
            _ => PrimitiveVisual::Ellipsoid(EllipsoidVisual {
                id,
                ..primitive_ellipsoid()
            }),
        })
        .collect();
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&mixed, primitive_calibration(), camera),
        Err(RenderError::InvalidIds)
    ));
    unchanged(&renderer);
    mixed.visuals.pop();
    renderer
        .set_calibrated_primitive_snapshot(&mixed, primitive_calibration(), camera)
        .unwrap();
    assert_eq!(renderer.visuals.len(), 128);
    assert!(!renderer.pending.as_ref().unwrap().valid);
    assert_eq!(
        renderer.app.world().resource::<Assets<Mesh>>().len(),
        meshes + 2
    );
}

#[test]
fn ellipsoid_first_reuses_sphere_mesh_and_axis_scale_through_all_replacements() {
    let mut renderer = Renderer::new().unwrap();
    let initial_meshes = renderer.app.world().resource::<Assets<Mesh>>().len();
    let initial_materials = renderer
        .app
        .world()
        .resource::<Assets<StandardMaterial>>()
        .len();
    let camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: glam::DQuat::IDENTITY,
    };
    let mut snapshot = PrimitiveSnapshot {
        epoch: 103,
        time: Duration::ZERO,
        origin: DVec3::ZERO,
        visuals: vec![PrimitiveVisual::Ellipsoid(primitive_ellipsoid())],
    };
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, primitive_calibration(), camera)
        .unwrap();
    let sphere = renderer.sphere_mesh.as_ref().unwrap().clone();
    assert!(renderer.cylinder_mesh.is_none());
    assert_eq!(
        renderer.app.world().resource::<Assets<Mesh>>().len(),
        initial_meshes + 1
    );
    let entity = renderer.visuals[&1].entity;
    let material = renderer.visuals[&1].material.id();
    for (index, kind) in [0, 1, 2, 3, 0, 1, 0].into_iter().enumerate() {
        renderer.snapshot_dirty = false; // Settled-state fixture, not GPU completion.
        snapshot.time = Duration::from_millis(index as u64 + 1);
        let mut ellipsoid = primitive_ellipsoid();
        ellipsoid.pose.rotation = glam::DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);
        ellipsoid.radii += DVec3::splat(index as f64 * 0.125);
        snapshot.visuals[0] = match kind {
            0 => PrimitiveVisual::Ellipsoid(ellipsoid.clone()),
            1 => PrimitiveVisual::Sphere(primitive_sphere()),
            2 => PrimitiveVisual::Box(BoxVisual {
                id: 1,
                pose: ellipsoid.pose,
                size: DVec3::ONE,
                color: ellipsoid.color,
            }),
            _ => PrimitiveVisual::Cylinder(primitive_cylinder()),
        };
        renderer
            .set_calibrated_primitive_snapshot(&snapshot, primitive_calibration(), camera)
            .unwrap();
        let expected_mesh = match kind {
            0 | 1 => sphere.id(),
            2 => renderer.mesh.id(),
            _ => renderer.cylinder_mesh.as_ref().unwrap().id(),
        };
        assert_eq!(renderer.visuals[&1].entity, entity);
        assert_eq!(renderer.visuals[&1].material.id(), material);
        assert_eq!(renderer.visuals[&1].mesh.id(), expected_mesh);
        assert_eq!(
            renderer.app.world().get::<Mesh3d>(entity).unwrap().0.id(),
            expected_mesh
        );
        assert_eq!(renderer.sphere_mesh.as_ref().unwrap().id(), sphere.id());
        assert_eq!(
            renderer
                .app
                .world()
                .resource::<Assets<StandardMaterial>>()
                .len(),
            initial_materials + 1
        );
        assert!(renderer.app.world().resource::<Assets<Mesh>>().len() <= initial_meshes + 2);
        if kind == 0 {
            let transform = renderer.app.world().get::<Transform>(entity).unwrap();
            assert_eq!(
                transform.scale,
                Vec3::new(
                    (2.0 * ellipsoid.radii.x) as f32,
                    (2.0 * ellipsoid.radii.z) as f32,
                    (2.0 * ellipsoid.radii.y) as f32
                )
            );
            let pose = to_render_pose(ellipsoid.pose, DVec3::ZERO).unwrap();
            assert_eq!(transform.translation, pose.translation);
            assert_eq!(transform.rotation, pose.rotation);
        }
    }
    assert_eq!(
        renderer.app.world().resource::<Assets<Mesh>>().len(),
        initial_meshes + 2
    );
    for _ in 0..200 {
        renderer.update().unwrap();
        if renderer.ready {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(renderer.ready && renderer.scene_assets_ready());
    let old_sphere = renderer
        .app
        .get_sub_app_mut(RenderApp)
        .unwrap()
        .world_mut()
        .resource_mut::<RenderAssets<RenderMesh>>()
        .remove(sphere.id())
        .unwrap();
    assert!(
        !renderer.scene_assets_ready(),
        "ellipsoid readiness must require the actual shared sphere mesh"
    );
    renderer
        .app
        .get_sub_app_mut(RenderApp)
        .unwrap()
        .world_mut()
        .resource_mut::<RenderAssets<RenderMesh>>()
        .insert(sphere.id(), old_sphere);
    assert!(renderer.scene_assets_ready());
    renderer.snapshot_dirty = false;
    snapshot.visuals.clear();
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, primitive_calibration(), camera)
        .unwrap();
    assert!(renderer.visuals.is_empty());
    assert_eq!(
        renderer
            .app
            .world()
            .resource::<Assets<StandardMaterial>>()
            .len(),
        initial_materials
    );
    assert_eq!(
        renderer.app.world().resource::<Assets<Mesh>>().len(),
        initial_meshes + 2
    );
}

#[test]
fn ellipsoid_busy_depth_and_buffered_retirement_preserve_single_capture_ownership() {
    let mut renderer = Renderer::new().unwrap();
    let camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: glam::DQuat::IDENTITY,
    };
    let old = Snapshot {
        epoch: 104,
        time: Duration::from_millis(9),
        origin: DVec3::ZERO,
        visuals: vec![BoxVisual {
            id: 1,
            pose: primitive_sphere().pose,
            size: DVec3::ONE,
            color: [1.0; 3],
        }],
    };
    renderer
        .set_calibrated_depth_snapshot(&old, primitive_calibration(), camera)
        .unwrap();
    let mut snapshot = PrimitiveSnapshot {
        epoch: 105,
        time: Duration::ZERO,
        origin: DVec3::ZERO,
        visuals: vec![PrimitiveVisual::Ellipsoid(primitive_ellipsoid())],
    };
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&snapshot, primitive_calibration(), camera),
        Err(RenderError::SnapshotBusy)
    ));
    assert!(renderer.depth_calibration.is_some() && renderer.sphere_mesh.is_none());
    renderer.snapshot_dirty = false;
    // Buffered callback fixture only; no invented software pixels.
    let pixels = Arc::new(Mutex::new(Some(Some(vec![
        0;
        (WIDTH * HEIGHT * 4) as usize
    ]))));
    renderer.pending = Some(Pending {
        epoch: old.epoch,
        time: old.time,
        valid: true,
        pixels: pixels.clone(),
    });
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, primitive_calibration(), camera)
        .unwrap();
    assert!(renderer.depth_calibration.is_none());
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    assert!(Arc::ptr_eq(
        &renderer.pending.as_ref().unwrap().pixels,
        &pixels
    ));
    assert!(!renderer.pending.as_ref().unwrap().valid);
    assert!(renderer.take_frame().unwrap().is_none() && !renderer.readback_in_flight());
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NoDepthSnapshot)
    ));
    assert_eq!(renderer.stamp, Some((snapshot.epoch, Duration::ZERO)));
    renderer.snapshot_dirty = false;
    snapshot.epoch += 1;
    snapshot.visuals.clear();
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, primitive_calibration(), camera)
        .unwrap();
    assert!(renderer.visuals.is_empty());
    let meshes = renderer.app.world().resource::<Assets<Mesh>>().len();
    renderer.app.world_mut().resource_mut::<DeviceFailure>().0 =
        Some(StoppedReason::Device("ellipsoid device lost".into()));
    snapshot
        .visuals
        .push(PrimitiveVisual::Ellipsoid(primitive_ellipsoid()));
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&snapshot, primitive_calibration(), camera),
        Err(RenderError::RenderStopped(_))
    ));
    assert!(renderer.visuals.is_empty());
    assert_eq!(
        renderer.app.world().resource::<Assets<Mesh>>().len(),
        meshes
    );
}
