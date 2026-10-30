// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

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
