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
