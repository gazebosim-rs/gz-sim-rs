// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "gpu")]
use glam::{DQuat, DVec3};
use gz_render_frame::WorldPose;
use gz_renderer::{BoxVisual, RenderError, Renderer, Snapshot};
use std::time::Duration;

fn retire_cancelled(renderer: &mut Renderer) {
    for _ in 0..200 {
        renderer.update().unwrap();
        assert!(renderer.take_frame().unwrap().is_none());
        if !renderer.readback_in_flight() {
            return;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    panic!("cancelled readback did not retire");
}

fn image(renderer: &mut Renderer) -> gz_renderer::Frame {
    for _ in 0..300 {
        match renderer.request_readback() {
            Ok(()) => break,
            Err(RenderError::NotReady) => {
                renderer.update().unwrap();
                std::thread::sleep(Duration::from_millis(10));
            }
            Err(error) => panic!("request failed: {error}"),
        }
    }
    for _ in 0..200 {
        renderer.update().unwrap();
        if let Some(frame) = renderer.take_frame().unwrap() {
            return frame;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    panic!("readback did not complete");
}

fn red_extent(frame: &gz_renderer::Frame) -> (usize, usize) {
    let mut min_x = 64;
    let mut max_x = 0;
    let mut min_y = 64;
    let mut max_y = 0;
    for y in 0..64 {
        for x in 0..64 {
            let offset = (y * 64 + x) * 4;
            let pixel = &frame.rgba[offset..offset + 4];
            if pixel[0] > 200 && pixel[1] < 20 && pixel[2] < 20 {
                min_x = min_x.min(x);
                max_x = max_x.max(x);
                min_y = min_y.min(y);
                max_y = max_y.max(y);
            }
        }
    }
    assert!(min_x <= max_x && min_y <= max_y, "no red pixels");
    (max_x - min_x + 1, max_y - min_y + 1)
}

#[test]
fn software_box_snapshot_and_cancellation() {
    let mut renderer = Renderer::new().unwrap();
    println!("adapter: {:?}", renderer.adapter_info());
    if std::env::var_os("GZ_RENDER_REQUIRE_LLVMPIPE").is_some() {
        let info = renderer.adapter_info();
        assert_eq!(info.device_type, wgpu::DeviceType::Cpu);
        assert!(
            info.name.to_ascii_lowercase().contains("llvmpipe"),
            "{info:?}"
        );
    }
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::NoSnapshot)
    ));
    let mut snapshot = Snapshot {
        epoch: 7,
        time: Duration::from_millis(3),
        origin: DVec3::splat(1e12),
        visuals: vec![BoxVisual {
            id: 1,
            pose: WorldPose {
                translation: DVec3::splat(1e12),
                rotation: DQuat::IDENTITY,
            },
            size: DVec3::splat(2.0),
            color: [1.0, 0.0, 0.0],
        }],
    };
    renderer.set_snapshot(&snapshot).unwrap();
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::NotReady)
    ));
    assert!(matches!(
        renderer.set_snapshot(&snapshot),
        Err(RenderError::SnapshotBusy)
    ));
    let mut invalid = snapshot.clone();
    invalid.visuals = vec![snapshot.visuals[0].clone(); 129];
    assert!(matches!(
        renderer.set_snapshot(&invalid),
        Err(RenderError::InvalidIds)
    ));
    invalid = snapshot.clone();
    invalid.visuals.push(invalid.visuals[0].clone());
    assert!(matches!(
        renderer.set_snapshot(&invalid),
        Err(RenderError::InvalidIds)
    ));
    invalid.visuals.pop();
    invalid.visuals[0].id = 0;
    assert!(matches!(
        renderer.set_snapshot(&invalid),
        Err(RenderError::InvalidIds)
    ));
    invalid.visuals[0].id = 1;
    invalid.visuals[0].size = DVec3::ZERO;
    assert!(matches!(
        renderer.set_snapshot(&invalid),
        Err(RenderError::InvalidVisual)
    ));
    invalid.visuals[0].size = DVec3::ONE;
    invalid.visuals[0].color[0] = f32::NAN;
    assert!(matches!(
        renderer.set_snapshot(&invalid),
        Err(RenderError::InvalidVisual)
    ));
    invalid.visuals[0].color[0] = 1.0;
    invalid.visuals[0].pose.translation.x = f64::NAN;
    assert!(matches!(
        renderer.set_snapshot(&invalid),
        Err(RenderError::Pose(_))
    ));
    let first = image(&mut renderer);
    assert_eq!((first.epoch, first.time), (7, Duration::from_millis(3)));
    assert_eq!(first.rgba.len(), 64 * 64 * 4);
    let offset = (32 * 64 + 32) * 4;
    assert_eq!(&first.rgba[offset..offset + 4], &[255, 0, 0, 255]);
    snapshot.visuals[0].size = DVec3::new(1.0, 2.0, 3.0);
    renderer.set_snapshot(&snapshot).unwrap();
    let upright = image(&mut renderer);
    let (upright_width, upright_height) = red_extent(&upright);
    assert!(
        upright_height > upright_width * 2,
        "unexpected upright extent: {upright_width}x{upright_height}"
    );
    snapshot.visuals[0].pose.rotation = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    renderer.set_snapshot(&snapshot).unwrap();
    let turned = image(&mut renderer);
    let (turned_width, turned_height) = red_extent(&turned);
    assert!(
        turned_width > upright_width,
        "turn did not expose the longer local Y side"
    );
    assert!(
        turned_height > turned_width,
        "local Z should remain the taller screen dimension"
    );
    snapshot.visuals[0].pose.rotation = DQuat::IDENTITY;
    snapshot.visuals[0].size = DVec3::splat(2.0);
    snapshot.epoch = 8;
    snapshot.time = Duration::ZERO;
    snapshot.visuals[0].color = [0.0, 1.0, 0.0];
    renderer.set_snapshot(&snapshot).unwrap();
    let next = image(&mut renderer);
    assert_eq!((next.epoch, next.time), (8, Duration::ZERO));
    assert_eq!(&next.rgba[offset..offset + 4], &[0, 255, 0, 255]);
    snapshot.visuals[0].pose.translation.x += 10.0;
    renderer.set_snapshot(&snapshot).unwrap();
    let moved = image(&mut renderer);
    assert_ne!(&moved.rgba[offset..offset + 4], &[0, 255, 0, 255]);
    snapshot.visuals[0].pose.translation.x -= 10.0;
    renderer.set_snapshot(&snapshot).unwrap();
    let restored = image(&mut renderer);
    assert_eq!(&restored.rgba[offset..offset + 4], &[0, 255, 0, 255]);
    renderer.request_readback().unwrap();
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    renderer.cancel_readback();
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    retire_cancelled(&mut renderer);
    renderer.request_readback().unwrap();
    snapshot.epoch = 9;
    snapshot.visuals.clear();
    renderer.set_snapshot(&snapshot).unwrap();
    assert!(renderer.readback_in_flight());
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    retire_cancelled(&mut renderer);
    let empty = image(&mut renderer);
    assert_eq!(empty.epoch, 9);
    assert_ne!(&empty.rgba[offset..offset + 4], &[0, 255, 0, 255]);
    renderer.request_readback().unwrap();
    renderer.update().unwrap();
    drop(renderer);
    let replacement = Renderer::new().unwrap();
    assert!(!replacement.adapter_info().name.is_empty());
}

#[test]
fn calibrated_camera_projects_boxes_and_enforces_clip_planes() {
    use gz_render_frame::camera::{Calibration, CameraConfig};
    let mut renderer = Renderer::new().unwrap();
    println!("calibrated adapter: {:?}", renderer.adapter_info());
    if std::env::var_os("GZ_RENDER_REQUIRE_LLVMPIPE").is_some() {
        assert_eq!(renderer.adapter_info().device_type, wgpu::DeviceType::Cpu);
        assert!(
            renderer
                .adapter_info()
                .name
                .to_ascii_lowercase()
                .contains("llvmpipe")
        );
    }
    let origin = DVec3::splat(1e12);
    let camera_pose = WorldPose {
        translation: origin,
        rotation: DQuat::IDENTITY,
    };
    let mut snapshot = Snapshot {
        epoch: 2,
        time: Duration::from_millis(7),
        origin,
        visuals: vec![BoxVisual {
            id: 1,
            pose: WorldPose {
                translation: origin + DVec3::new(4.0, 0.0, 0.0),
                rotation: DQuat::IDENTITY,
            },
            size: DVec3::ONE,
            color: [1.0, 0.0, 0.0],
        }],
    };
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
    let center = (32 * 64 + 32) * 4;
    renderer
        .set_calibrated_snapshot(&snapshot, Calibration::new(config).unwrap(), camera_pose)
        .unwrap();
    assert!(matches!(
        renderer.set_calibrated_snapshot(&snapshot, Calibration::new(config).unwrap(), camera_pose),
        Err(RenderError::SnapshotBusy)
    ));
    let mut replacement = snapshot.clone();
    replacement.visuals[0].color = [0.0, 1.0, 0.0];
    let shifted_camera = CameraConfig {
        cx: 16.0,
        cy: 40.0,
        ..config
    };
    assert!(matches!(
        renderer.set_calibrated_snapshot(
            &replacement,
            Calibration::new(shifted_camera).unwrap(),
            camera_pose,
        ),
        Err(RenderError::SnapshotBusy)
    ));
    let initial = image(&mut renderer);
    assert_eq!((initial.epoch, initial.time), (2, Duration::from_millis(7)));
    assert_eq!(&initial.rgba[center..center + 4], &[255, 0, 0, 255]);
    let shifted_pixel = (40 * 64 + 16) * 4;
    assert_ne!(
        &initial.rgba[shifted_pixel..shifted_pixel + 4],
        &[0, 255, 0, 255]
    );
    assert_ne!(
        &initial.rgba[shifted_pixel..shifted_pixel + 4],
        &[255, 0, 0, 255]
    );
    let invalid_pose = WorldPose {
        rotation: DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
        ..camera_pose
    };
    assert!(matches!(
        renderer.set_calibrated_snapshot(
            &snapshot,
            Calibration::new(config).unwrap(),
            invalid_pose
        ),
        Err(RenderError::Pose(_))
    ));
    let unchanged = image(&mut renderer);
    assert_eq!(
        (unchanged.epoch, unchanged.time),
        (2, Duration::from_millis(7))
    );
    assert_eq!(&unchanged.rgba[center..center + 4], &[255, 0, 0, 255]);
    snapshot.time += Duration::from_millis(1);
    let shifted = CameraConfig {
        cx: 16.0,
        cy: 40.0,
        ..config
    };
    renderer
        .set_calibrated_snapshot(&snapshot, Calibration::new(shifted).unwrap(), camera_pose)
        .unwrap();
    let frame = image(&mut renderer);
    let shifted_pixel = (40 * 64 + 16) * 4;
    assert_eq!(
        &frame.rgba[shifted_pixel..shifted_pixel + 4],
        &[255, 0, 0, 255]
    );
    assert_ne!(&frame.rgba[center..center + 4], &[255, 0, 0, 255]);
    for clipped in [
        CameraConfig { far: 3.0, ..config },
        CameraConfig {
            near: 5.0,
            ..config
        },
    ] {
        snapshot.time += Duration::from_millis(1);
        renderer
            .set_calibrated_snapshot(&snapshot, Calibration::new(clipped).unwrap(), camera_pose)
            .unwrap();
        let frame = image(&mut renderer);
        assert_ne!(&frame.rgba[center..center + 4], &[255, 0, 0, 255]);
    }
    let wrong_size = Calibration::new(CameraConfig {
        width: 65,
        ..config
    })
    .unwrap();
    assert!(matches!(
        renderer.set_calibrated_snapshot(&snapshot, wrong_size, camera_pose),
        Err(RenderError::InvalidCameraDimensions)
    ));
    let unchanged = image(&mut renderer);
    assert_eq!(
        (unchanged.epoch, unchanged.time),
        (snapshot.epoch, snapshot.time)
    );
    assert_ne!(&unchanged.rgba[center..center + 4], &[255, 0, 0, 255]);
    snapshot.visuals[0].pose.translation = origin;
    renderer.set_snapshot(&snapshot).unwrap();
    let legacy = image(&mut renderer);
    assert_eq!(&legacy.rgba[center..center + 4], &[255, 0, 0, 255]);
}

#[test]
fn software_directional_face_response_and_unlit_restoration() {
    use gz_renderer::DirectionalWorldLight;
    let mut renderer = Renderer::new().unwrap();
    if std::env::var_os("GZ_RENDER_REQUIRE_LLVMPIPE").is_some() {
        assert_eq!(renderer.adapter_info().device_type, wgpu::DeviceType::Cpu);
        assert!(
            renderer
                .adapter_info()
                .name
                .to_ascii_lowercase()
                .contains("llvmpipe")
        );
    }
    let snapshot = Snapshot {
        epoch: 11,
        time: Duration::from_millis(9),
        origin: DVec3::splat(1e12),
        visuals: vec![BoxVisual {
            id: 1,
            pose: WorldPose {
                translation: DVec3::splat(1e12),
                rotation: DQuat::IDENTITY,
            },
            size: DVec3::splat(2.0),
            color: [1.0, 0.0, 0.0],
        }],
    };
    let mut light = DirectionalWorldLight {
        id: 2,
        rotation: DQuat::IDENTITY,
        direction: DVec3::Y,
        color: [1.0; 3],
        illuminance_lux: 1000.0,
    };
    let center = (32 * 64 + 32) * 4;
    renderer.set_snapshot(&snapshot).unwrap();
    let unlit = image(&mut renderer);
    assert_eq!(&unlit.rgba[center..center + 4], &[255, 0, 0, 255]);
    renderer
        .set_directional_snapshot(&snapshot, &light)
        .unwrap();
    let lit = image(&mut renderer);
    assert_eq!((lit.epoch, lit.time), (snapshot.epoch, snapshot.time));
    let red = lit.rgba[center];
    assert!(
        red > 20 && red < 250,
        "expected unsaturated directional response: {red}"
    );
    assert_eq!(&lit.rgba[center + 1..center + 4], &[0, 0, 255]);
    light.illuminance_lux = 200.0;
    renderer
        .set_directional_snapshot(&snapshot, &light)
        .unwrap();
    let dimmed = image(&mut renderer);
    assert!(dimmed.rgba[center] > 0 && dimmed.rgba[center] < red);
    light.rotation = DQuat::from_rotation_z(std::f64::consts::PI);
    renderer
        .set_directional_snapshot(&snapshot, &light)
        .unwrap();
    let away = image(&mut renderer);
    assert_eq!(&away.rgba[center..center + 4], &[0, 0, 0, 255]);
    light.rotation = DQuat::IDENTITY;
    light.illuminance_lux = 0.0;
    renderer
        .set_directional_snapshot(&snapshot, &light)
        .unwrap();
    let off = image(&mut renderer);
    assert_eq!(&off.rgba[center..center + 4], &[0, 0, 0, 255]);
    renderer.set_snapshot(&snapshot).unwrap();
    let restored = image(&mut renderer);
    assert_eq!(restored.rgba, unlit.rgba);
    use gz_render_frame::camera::{Calibration, CameraConfig};
    let calibration = Calibration::new(CameraConfig {
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
    })
    .unwrap();
    let camera = WorldPose {
        translation: snapshot.origin - DVec3::Y * 5.0,
        rotation: DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
    };
    light.illuminance_lux = 1000.0;
    renderer
        .set_calibrated_directional_snapshot(&snapshot, calibration, camera, &light)
        .unwrap();
    let calibrated_lit = image(&mut renderer);
    assert!(calibrated_lit.rgba[center] > 20 && calibrated_lit.rgba[center] < 250);
    renderer
        .set_calibrated_snapshot(&snapshot, calibration, camera)
        .unwrap();
    let calibrated_unlit = image(&mut renderer);
    assert_eq!(
        &calibrated_unlit.rgba[center..center + 4],
        &[255, 0, 0, 255]
    );
}

#[test]
fn pending_directional_captures_retire_before_replacement_and_unlit_successors() {
    fn submit_capture(renderer: &mut Renderer) {
        renderer.request_readback().unwrap();
        for _ in 0..200 {
            match renderer.update().unwrap() {
                gz_renderer::UpdateStatus::Advanced => {
                    assert!(renderer.readback_in_flight());
                    return;
                }
                gz_renderer::UpdateStatus::Waiting => {
                    std::thread::sleep(Duration::from_millis(10));
                }
            }
        }
        panic!("capture submission did not advance");
    }
    use gz_render_frame::camera::{Calibration, CameraConfig};
    use gz_renderer::DirectionalWorldLight;
    let mut renderer = Renderer::new().unwrap();
    if std::env::var_os("GZ_RENDER_REQUIRE_LLVMPIPE").is_some() {
        assert_eq!(renderer.adapter_info().device_type, wgpu::DeviceType::Cpu);
        assert!(
            renderer
                .adapter_info()
                .name
                .to_ascii_lowercase()
                .contains("llvmpipe")
        );
    }
    let calibration = Calibration::new(CameraConfig {
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
    })
    .unwrap();
    let origin = DVec3::splat(1e12);
    let camera = WorldPose {
        translation: origin - DVec3::Y * 5.0,
        rotation: DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
    };
    let mut snapshot = Snapshot {
        epoch: 20,
        time: Duration::from_millis(7),
        origin,
        visuals: vec![BoxVisual {
            id: 1,
            pose: WorldPose {
                translation: origin,
                rotation: DQuat::IDENTITY,
            },
            size: DVec3::splat(2.0),
            color: [0.0, 1.0, 0.0],
        }],
    };
    let mut light = DirectionalWorldLight {
        id: 2,
        rotation: DQuat::IDENTITY,
        direction: DVec3::Y,
        color: [1.0; 3],
        illuminance_lux: 1000.0,
    };
    renderer
        .set_calibrated_snapshot(&snapshot, calibration, camera)
        .unwrap();
    let unlit = image(&mut renderer);
    let center = (32 * 64 + 32) * 4;
    assert_eq!(&unlit.rgba[center..center + 4], &[0, 255, 0, 255]);
    snapshot.visuals[0].color = [1.0, 0.0, 0.0];
    renderer
        .set_calibrated_directional_snapshot(&snapshot, calibration, camera, &light)
        .unwrap();
    let first = image(&mut renderer);
    assert!(first.rgba[center] > 20 && first.rgba[center] < 250);
    assert_eq!((first.epoch, first.time), (20, Duration::from_millis(7)));
    // Submit a real screenshot to Bevy, retaining its delivery slot even if the
    // device completes before the next snapshot arrives.
    submit_capture(&mut renderer);
    snapshot.epoch = 21;
    snapshot.time = Duration::ZERO;
    snapshot.visuals[0].color = [0.0, 1.0, 0.0];
    light.id = 3;
    light.illuminance_lux = 200.0;
    renderer
        .set_calibrated_directional_snapshot(&snapshot, calibration, camera, &light)
        .unwrap();
    assert!(renderer.readback_in_flight());
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    retire_cancelled(&mut renderer);
    let replacement = image(&mut renderer);
    assert_eq!((replacement.epoch, replacement.time), (21, Duration::ZERO));
    assert_eq!(replacement.rgba[center], 0);
    assert!(replacement.rgba[center + 1] > 0 && replacement.rgba[center + 1] < first.rgba[center]);
    submit_capture(&mut renderer);
    snapshot.epoch = 22;
    snapshot.time = Duration::from_nanos(1);
    renderer
        .set_calibrated_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert!(renderer.readback_in_flight());
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    retire_cancelled(&mut renderer);
    let restored = image(&mut renderer);
    assert_eq!(
        (restored.epoch, restored.time),
        (22, Duration::from_nanos(1))
    );
    assert_eq!(restored.rgba, unlit.rgba);
}

fn request_metric_depth(renderer: &mut Renderer) {
    for _ in 0..300 {
        match renderer.request_depth_readback() {
            Ok(()) => return,
            Err(RenderError::NotReady) => {
                renderer.update().unwrap();
                std::thread::sleep(Duration::from_millis(10));
            }
            Err(error) => panic!("depth request failed: {error}"),
        }
    }
    panic!("depth scene never became ready");
}

fn metric_depth(renderer: &mut Renderer) -> gz_renderer::DepthFrame {
    request_metric_depth(renderer);
    for _ in 0..200 {
        renderer.update().unwrap();
        if let Some(frame) = renderer.take_depth_frame().unwrap() {
            return frame;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    panic!("metric depth readback did not complete");
}

#[test]
fn software_metric_depth_uses_actual_geometry_and_preserves_readback_lifecycle() {
    use gz_render_frame::camera::{Calibration, CameraConfig};
    let mut renderer = Renderer::new().unwrap();
    println!("metric depth adapter: {:?}", renderer.adapter_info());
    if std::env::var_os("GZ_RENDER_REQUIRE_LLVMPIPE").is_some() {
        assert_eq!(renderer.adapter_info().device_type, wgpu::DeviceType::Cpu);
        assert!(
            renderer
                .adapter_info()
                .name
                .to_ascii_lowercase()
                .contains("llvmpipe")
        );
    }
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NoSnapshot)
    ));
    let calibration = Calibration::new(CameraConfig {
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
    .unwrap();
    let origin = DVec3::splat(1e12);
    let camera = WorldPose {
        translation: origin,
        rotation: DQuat::from_rotation_z(0.3),
    };
    let visual = |id, forward| BoxVisual {
        id,
        pose: WorldPose {
            translation: origin + camera.rotation * DVec3::new(forward, 0., 0.),
            rotation: camera.rotation,
        },
        size: DVec3::ONE,
        color: [1., 0., 0.],
    };
    let mut scene = Snapshot {
        epoch: 4,
        time: Duration::from_millis(7),
        origin,
        visuals: vec![visual(1, 4.)],
    };
    renderer
        .set_calibrated_snapshot(&scene, calibration, camera)
        .unwrap();
    let original_rgb = image(&mut renderer);
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NoDepthSnapshot)
    ));
    renderer
        .set_calibrated_depth_snapshot(&scene, calibration, camera)
        .unwrap();
    let first = metric_depth(&mut renderer);
    assert_eq!((first.epoch, first.time), (scene.epoch, scene.time));
    assert_eq!(first.metres.len(), 64 * 64);
    // Match pinned gz-sensors' existing DEPTH_TOL=1e-4 for this BOX face.
    // Off-principal pixels have the same optical axial depth, not radial range.
    for (x, y) in [(29, 35), (26, 35), (31, 35), (29, 40)] {
        assert!((first.metres[y * 64 + x] - 3.5).abs() < 1e-4);
    }
    assert_eq!(first.metres[0], f32::INFINITY);
    assert_eq!(first.metres[63], f32::INFINITY);
    assert_eq!(first.metres[25 * 64 + 29], f32::INFINITY);

    // Invalid near geometry must leave a real, requested frame deliverable.
    request_metric_depth(&mut renderer);
    assert!(renderer.readback_in_flight());
    let wrong_size = Calibration::new(CameraConfig {
        width: 65,
        ..calibration.config()
    })
    .unwrap();
    assert!(matches!(
        renderer.set_calibrated_depth_snapshot(&scene, wrong_size, camera),
        Err(RenderError::InvalidCameraDimensions)
    ));
    let mut near = scene.clone();
    near.epoch += 1;
    near.visuals[0].pose.translation = origin;
    assert!(matches!(
        renderer.set_calibrated_depth_snapshot(&near, calibration, camera),
        Err(RenderError::DepthNearClip)
    ));
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    let mut delivered = false;
    for _ in 0..200 {
        renderer.update().unwrap();
        if let Some(frame) = renderer.take_depth_frame().unwrap() {
            assert_eq!((frame.epoch, frame.time), (scene.epoch, scene.time));
            assert!((frame.metres[35 * 64 + 29] - 3.5).abs() < 1e-4);
            delivered = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(
        delivered,
        "invalid snapshot cancelled the valid depth capture"
    );

    // A committed successor cancels delivery but does not free the owned slot.
    request_metric_depth(&mut renderer);
    scene.epoch += 1;
    scene.time += Duration::from_millis(1);
    scene.visuals.push(visual(2, 2.5));
    renderer
        .set_calibrated_depth_snapshot(&scene, calibration, camera)
        .unwrap();
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::Busy)
    ));
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    for _ in 0..200 {
        renderer.update().unwrap();
        assert!(renderer.take_depth_frame().unwrap().is_none());
        if !renderer.readback_in_flight() {
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(!renderer.readback_in_flight());
    let occluded = metric_depth(&mut renderer);
    assert_eq!((occluded.epoch, occluded.time), (scene.epoch, scene.time));
    assert!((occluded.metres[35 * 64 + 29] - 2.).abs() < 1e-4);

    // RGB also reserves the shared slot until its real cancelled delivery retires.
    renderer.request_readback().unwrap();
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::Busy)
    ));
    renderer.cancel_readback();
    retire_cancelled(&mut renderer);
    scene.visuals = vec![visual(1, 200.)];
    scene.time += Duration::from_millis(1);
    renderer
        .set_calibrated_depth_snapshot(&scene, calibration, camera)
        .unwrap();
    let far = metric_depth(&mut renderer);
    assert!(far.metres.iter().all(|value| *value == f32::INFINITY));
    scene.visuals.clear();
    scene.time += Duration::from_millis(1);
    renderer
        .set_calibrated_depth_snapshot(&scene, calibration, camera)
        .unwrap();
    let empty = metric_depth(&mut renderer);
    assert!(empty.metres.iter().all(|value| *value == f32::INFINITY));

    scene.visuals = vec![visual(1, 4.)];
    scene.time += Duration::from_millis(1);
    renderer
        .set_calibrated_snapshot(&scene, calibration, camera)
        .unwrap();
    let restored = image(&mut renderer);
    assert_eq!(original_rgb.rgba, restored.rgba);
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NoDepthSnapshot)
    ));
}

#[test]
fn software_cylinder_side_endcap_rotation_and_box_occlusion_are_real_pixels() {
    use gz_render_frame::camera::Calibration;
    use gz_renderer::{CylinderVisual, PrimitiveSnapshot, PrimitiveVisual};
    let mut renderer = Renderer::new().unwrap();
    if std::env::var_os("GZ_RENDER_REQUIRE_LLVMPIPE").is_some() {
        assert_eq!(renderer.adapter_info().device_type, wgpu::DeviceType::Cpu);
        assert!(
            renderer
                .adapter_info()
                .name
                .to_ascii_lowercase()
                .contains("llvmpipe")
        );
    }
    let origin = DVec3::splat(1e12);
    let camera = WorldPose {
        translation: origin,
        rotation: DQuat::IDENTITY,
    };
    let calibration =
        Calibration::from_horizontal_fov(64, 64, std::f64::consts::FRAC_PI_2, 0.1, 20.0).unwrap();
    let cube = BoxVisual {
        id: 1,
        pose: WorldPose {
            translation: origin + DVec3::new(4.0, 0.0, 0.0),
            rotation: DQuat::IDENTITY,
        },
        size: DVec3::splat(2.0),
        color: [1.0, 0.0, 0.0],
    };
    let legacy = Snapshot {
        epoch: 4,
        time: Duration::from_millis(1),
        origin,
        visuals: vec![cube.clone()],
    };
    renderer
        .set_calibrated_snapshot(&legacy, calibration, camera)
        .unwrap();
    let original_box = image(&mut renderer);
    let mut snapshot = PrimitiveSnapshot {
        epoch: 4,
        time: Duration::from_millis(2),
        origin,
        visuals: vec![PrimitiveVisual::Box(cube.clone())],
    };
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert_eq!(image(&mut renderer).rgba, original_box.rgba);
    let cylinder = CylinderVisual {
        id: 1,
        pose: cube.pose,
        radius: 1.0,
        length: 2.0,
        color: [1.0, 0.0, 0.0],
    };
    // Cancel a real in-flight old-box readback before installing cylinder geometry.
    renderer.request_readback().unwrap();
    snapshot.visuals[0] = PrimitiveVisual::Cylinder(cylinder.clone());
    snapshot.time = Duration::from_millis(3);
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    retire_cancelled(&mut renderer);
    let side = image(&mut renderer);
    assert_eq!((side.epoch, side.time), (snapshot.epoch, snapshot.time));
    let (width, height) = red_extent(&side);
    assert!(
        height > width + 2,
        "upright Z cylinder extent {width}x{height}"
    );
    let rolled = CylinderVisual {
        pose: WorldPose {
            rotation: DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
            ..cylinder.pose
        },
        ..cylinder.clone()
    };
    snapshot.visuals[0] = PrimitiveVisual::Cylinder(rolled);
    snapshot.time = Duration::from_millis(4);
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    let rolled = image(&mut renderer);
    let (width, height) = red_extent(&rolled);
    assert!(
        width > height + 2,
        "rolled cylinder extent {width}x{height}"
    );
    let endcap = CylinderVisual {
        pose: WorldPose {
            rotation: DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2),
            ..cylinder.pose
        },
        ..cylinder.clone()
    };
    snapshot.visuals[0] = PrimitiveVisual::Cylinder(endcap.clone());
    snapshot.time = Duration::from_millis(5);
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    let cap = image(&mut renderer);
    let pixel = |frame: &gz_renderer::Frame, x: usize, y: usize| {
        let offset = (y * 64 + x) * 4;
        <[u8; 4]>::try_from(&frame.rgba[offset..offset + 4]).unwrap()
    };
    // Strictly interior samples distinguish the fixed32-facet cap from an AABB.
    assert_eq!(pixel(&cap, 32, 32), [255, 0, 0, 255]);
    assert_eq!(pixel(&cap, 32, 24), [255, 0, 0, 255]);
    assert_eq!(pixel(&cap, 40, 32), [255, 0, 0, 255]);
    assert_eq!(pixel(&cap, 23, 23), [43, 44, 47, 255]);
    assert_eq!(pixel(&cap, 40, 40), [43, 44, 47, 255]);
    // Invalid geometry must not cancel an actual pending capture or alter its stamp/pixels.
    renderer.request_readback().unwrap();
    let mut invalid = snapshot.clone();
    invalid.visuals[0] = PrimitiveVisual::Cylinder(CylinderVisual {
        length: 0.0,
        ..endcap.clone()
    });
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&invalid, calibration, camera),
        Err(RenderError::InvalidVisual)
    ));
    let mut unchanged = None;
    for _ in 0..200 {
        renderer.update().unwrap();
        if let Some(frame) = renderer.take_frame().unwrap() {
            unchanged = Some(frame);
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    let unchanged = unchanged.expect("pending cap capture did not complete");
    assert_eq!(
        (unchanged.epoch, unchanged.time),
        (snapshot.epoch, snapshot.time)
    );
    assert_eq!(unchanged.rgba, cap.rgba);
    let near_box = BoxVisual {
        id: 2,
        pose: WorldPose {
            translation: origin + DVec3::new(2.0, 0.0, 0.0),
            rotation: DQuat::IDENTITY,
        },
        size: DVec3::splat(0.4),
        color: [0.0, 1.0, 0.0],
    };
    snapshot.visuals = vec![
        PrimitiveVisual::Cylinder(endcap.clone()),
        PrimitiveVisual::Box(near_box.clone()),
    ];
    snapshot.time = Duration::from_millis(6);
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    let occluded = image(&mut renderer);
    assert_eq!(pixel(&occluded, 32, 32), [0, 255, 0, 255]);
    assert_eq!(pixel(&occluded, 32, 24), [255, 0, 0, 255]);
    snapshot.visuals.reverse();
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert_eq!(image(&mut renderer).rgba, occluded.rgba);
    let far_box = BoxVisual {
        pose: WorldPose {
            translation: origin + DVec3::new(6.0, 0.0, 0.0),
            ..near_box.pose
        },
        ..near_box
    };
    snapshot.visuals = vec![
        PrimitiveVisual::Box(far_box),
        PrimitiveVisual::Cylinder(endcap),
    ];
    snapshot.time = Duration::from_millis(7);
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert_eq!(pixel(&image(&mut renderer), 32, 32), [255, 0, 0, 255]);
    snapshot.visuals = vec![PrimitiveVisual::Box(cube)];
    snapshot.epoch += 1;
    snapshot.time = Duration::ZERO;
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    let restored = image(&mut renderer);
    assert_eq!(
        (restored.epoch, restored.time),
        (snapshot.epoch, snapshot.time)
    );
    assert_eq!(
        restored.rgba, original_box.rgba,
        "same-ID cylinder-to-box must restore every original box pixel"
    );
    snapshot.visuals.clear();
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert_eq!(pixel(&image(&mut renderer), 32, 32), [43, 44, 47, 255]);
}

#[test]
fn software_spheres_preserve_silhouette_occlusion_origin_and_shape_round_trip() {
    use gz_render_frame::camera::Calibration;
    use gz_renderer::{CylinderVisual, PrimitiveSnapshot, PrimitiveVisual, SphereVisual};
    let mut renderer = Renderer::new().unwrap();
    if std::env::var_os("GZ_RENDER_REQUIRE_LLVMPIPE").is_some() {
        assert_eq!(renderer.adapter_info().device_type, wgpu::DeviceType::Cpu);
        assert!(
            renderer
                .adapter_info()
                .name
                .to_ascii_lowercase()
                .contains("llvmpipe")
        );
    }
    let origin = DVec3::splat(1e12);
    let camera = WorldPose {
        translation: origin,
        rotation: DQuat::IDENTITY,
    };
    let calibration =
        Calibration::from_horizontal_fov(64, 64, std::f64::consts::FRAC_PI_2, 0.1, 20.0).unwrap();
    let cube = BoxVisual {
        id: 1,
        pose: WorldPose {
            translation: origin + DVec3::new(4.0, 0.0, 0.0),
            rotation: DQuat::IDENTITY,
        },
        size: DVec3::splat(2.0),
        color: [1.0, 0.0, 0.0],
    };
    let backdrop = BoxVisual {
        id: 2,
        pose: WorldPose {
            translation: origin + DVec3::new(7.0, 0.0, 0.0),
            rotation: DQuat::IDENTITY,
        },
        size: DVec3::new(0.2, 6.0, 6.0),
        color: [0.0, 0.0, 1.0],
    };
    let near_box = BoxVisual {
        id: 3,
        pose: WorldPose {
            translation: origin + DVec3::new(2.0, 0.0, 0.0),
            rotation: DQuat::IDENTITY,
        },
        size: DVec3::splat(0.4),
        color: [0.0, 1.0, 0.0],
    };
    let mut legacy = Snapshot {
        epoch: 96,
        time: Duration::from_millis(1),
        origin,
        visuals: vec![cube.clone(), backdrop.clone(), near_box.clone()],
    };
    renderer
        .set_calibrated_snapshot(&legacy, calibration, camera)
        .unwrap();
    let original = image(&mut renderer);
    let pixel = |frame: &gz_renderer::Frame, x: usize, y: usize| {
        let offset = (y * 64 + x) * 4;
        <[u8; 4]>::try_from(&frame.rgba[offset..offset + 4]).unwrap()
    };
    assert_eq!(pixel(&original, 24, 24), [255, 0, 0, 255]);
    assert_eq!(pixel(&original, 39, 39), [255, 0, 0, 255]);
    let sphere = SphereVisual {
        id: 1,
        pose: cube.pose,
        radius: 1.0,
        color: cube.color,
    };
    let mut snapshot = PrimitiveSnapshot {
        epoch: 96,
        time: Duration::from_millis(2),
        origin,
        visuals: vec![
            PrimitiveVisual::Sphere(sphere.clone()),
            PrimitiveVisual::Box(backdrop.clone()),
            PrimitiveVisual::Box(near_box.clone()),
        ],
    };
    // A rejected sphere must preserve an actual asynchronous box frame and stamp.
    renderer.request_readback().unwrap();
    let mut invalid = snapshot.clone();
    invalid.visuals[0] = PrimitiveVisual::Sphere(SphereVisual {
        radius: 0.0,
        ..sphere.clone()
    });
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&invalid, calibration, camera),
        Err(RenderError::InvalidVisual)
    ));
    let mut old = None;
    for _ in 0..200 {
        renderer.update().unwrap();
        if let Some(frame) = renderer.take_frame().unwrap() {
            old = Some(frame);
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    let old = old.expect("valid old box capture was cancelled by invalid sphere");
    assert_eq!((old.epoch, old.time), (legacy.epoch, legacy.time));
    assert_eq!(old.rgba, original.rgba);
    renderer.request_readback().unwrap();
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    retire_cancelled(&mut renderer);
    let frame = image(&mut renderer);
    assert_eq!((frame.epoch, frame.time), (snapshot.epoch, snapshot.time));
    // Interiors/corners are away from MSAA edges. The fixed UV approximation is
    // a sphere silhouette, not a cuboid proxy or analytic/native render oracle.
    assert_eq!(pixel(&frame, 32, 32), [0, 255, 0, 255]);
    assert_eq!(pixel(&frame, 32, 26), [255, 0, 0, 255]);
    assert_eq!(pixel(&frame, 38, 32), [255, 0, 0, 255]);
    assert_eq!(pixel(&frame, 24, 24), [0, 0, 255, 255]);
    assert_eq!(pixel(&frame, 39, 39), [0, 0, 255, 255]);
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NoDepthSnapshot)
    ));
    snapshot.visuals.reverse();
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert_eq!(
        image(&mut renderer).rgba,
        frame.rgba,
        "input order cannot change opaque occlusion"
    );
    let mut local = snapshot.clone();
    local.origin = DVec3::ZERO;
    for visual in &mut local.visuals {
        match visual {
            PrimitiveVisual::Box(v) => v.pose.translation -= origin,
            PrimitiveVisual::Sphere(v) => v.pose.translation -= origin,
            PrimitiveVisual::Cylinder(_)
            | PrimitiveVisual::Ellipsoid(_)
            | PrimitiveVisual::Capsule(_) => unreachable!(),
        }
    }
    let local_camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
    };
    renderer
        .set_calibrated_primitive_snapshot(&local, calibration, local_camera)
        .unwrap();
    assert_eq!(
        image(&mut renderer).rgba,
        frame.rgba,
        "large origin must preserve the complete calibrated image"
    );
    let spin = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    for visual in &mut local.visuals {
        let pose = match visual {
            PrimitiveVisual::Box(v) => &mut v.pose,
            PrimitiveVisual::Sphere(v) => &mut v.pose,
            PrimitiveVisual::Cylinder(_)
            | PrimitiveVisual::Ellipsoid(_)
            | PrimitiveVisual::Capsule(_) => unreachable!(),
        };
        pose.translation = spin * pose.translation;
        pose.rotation = spin * pose.rotation;
    }
    renderer
        .set_calibrated_primitive_snapshot(
            &local,
            calibration,
            WorldPose {
                rotation: spin,
                ..local_camera
            },
        )
        .unwrap();
    assert_eq!(
        image(&mut renderer).rgba,
        frame.rgba,
        "rotating camera and every visual together must preserve the view"
    );
    snapshot.visuals = vec![
        PrimitiveVisual::Cylinder(CylinderVisual {
            id: 1,
            pose: WorldPose {
                rotation: DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2),
                ..cube.pose
            },
            radius: 1.0,
            length: 2.0,
            color: cube.color,
        }),
        PrimitiveVisual::Box(backdrop),
        PrimitiveVisual::Box(near_box),
    ];
    snapshot.time = Duration::from_millis(3);
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    let cylinder = image(&mut renderer);
    assert_eq!(pixel(&cylinder, 32, 32), [0, 255, 0, 255]);
    assert_eq!(pixel(&cylinder, 32, 26), [255, 0, 0, 255]);
    legacy.epoch += 1;
    legacy.time = Duration::ZERO;
    renderer
        .set_calibrated_snapshot(&legacy, calibration, camera)
        .unwrap();
    let restored = image(&mut renderer);
    assert_eq!((restored.epoch, restored.time), (legacy.epoch, legacy.time));
    assert_eq!(
        restored.rgba, original.rgba,
        "same-ID Box/Sphere/Cylinder/Box must restore every old box pixel"
    );
    renderer.request_readback().unwrap();
    snapshot.epoch = legacy.epoch + 1;
    snapshot.time = Duration::ZERO;
    snapshot.visuals.clear();
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    retire_cancelled(&mut renderer);
    let empty = image(&mut renderer);
    assert_eq!((empty.epoch, empty.time), (snapshot.epoch, Duration::ZERO));
    assert!(empty.rgba.chunks_exact(4).all(|p| p == [43, 44, 47, 255]));
}

#[test]
fn software_ellipsoids_preserve_axis_silhouette_occlusion_and_exact_old_frames() {
    use gz_render_frame::camera::Calibration;
    use gz_renderer::{
        CylinderVisual, EllipsoidVisual, PrimitiveSnapshot, PrimitiveVisual, SphereVisual,
    };
    let mut renderer = Renderer::new().unwrap();
    if std::env::var_os("GZ_RENDER_REQUIRE_LLVMPIPE").is_some() {
        assert_eq!(renderer.adapter_info().device_type, wgpu::DeviceType::Cpu);
        assert!(
            renderer
                .adapter_info()
                .name
                .to_ascii_lowercase()
                .contains("llvmpipe")
        );
    }
    let origin = DVec3::splat(1e12);
    let camera = WorldPose {
        translation: origin,
        rotation: DQuat::IDENTITY,
    };
    let calibration =
        Calibration::from_horizontal_fov(64, 64, std::f64::consts::FRAC_PI_2, 0.1, 20.0).unwrap();
    let ellipsoid = EllipsoidVisual {
        id: 1,
        pose: WorldPose {
            translation: origin + DVec3::new(4.0, 0.0, 0.0),
            rotation: DQuat::IDENTITY,
        },
        radii: DVec3::new(0.5, 1.5, 0.75),
        color: [1.0, 0.0, 0.0],
    };
    let cube = BoxVisual {
        id: 1,
        pose: ellipsoid.pose,
        size: 2.0 * ellipsoid.radii,
        color: ellipsoid.color,
    };
    let backdrop = BoxVisual {
        id: 2,
        pose: WorldPose {
            translation: origin + DVec3::new(7.0, 0.0, 0.0),
            rotation: DQuat::IDENTITY,
        },
        size: DVec3::new(0.2, 6.0, 6.0),
        color: [0.0, 0.0, 1.0],
    };
    let near_box = BoxVisual {
        id: 3,
        pose: WorldPose {
            translation: origin + DVec3::new(2.0, 0.0, 0.0),
            rotation: DQuat::IDENTITY,
        },
        size: DVec3::splat(0.4),
        color: [0.0, 1.0, 0.0],
    };
    let mut old_box = Snapshot {
        epoch: 106,
        time: Duration::from_millis(1),
        origin,
        visuals: vec![cube.clone(), backdrop.clone(), near_box.clone()],
    };
    renderer
        .set_calibrated_snapshot(&old_box, calibration, camera)
        .unwrap();
    let box_frame = image(&mut renderer);
    let pixel = |frame: &gz_renderer::Frame, x: usize, y: usize| {
        let offset = (y * 64 + x) * 4;
        <[u8; 4]>::try_from(&frame.rgba[offset..offset + 4]).unwrap()
    };
    assert_eq!(
        pixel(&box_frame, 43, 37),
        [255, 0, 0, 255],
        "the proxy AABB occupies this patch"
    );
    let mut snapshot = PrimitiveSnapshot {
        epoch: old_box.epoch,
        time: Duration::from_millis(2),
        origin,
        visuals: vec![
            PrimitiveVisual::Sphere(SphereVisual {
                id: 1,
                pose: ellipsoid.pose,
                radius: 1.5,
                color: ellipsoid.color,
            }),
            PrimitiveVisual::Box(backdrop.clone()),
            PrimitiveVisual::Box(near_box.clone()),
        ],
    };
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    let large_sphere = image(&mut renderer);
    assert_eq!(
        pixel(&large_sphere, 32, 41),
        [255, 0, 0, 255],
        "a largest-radius sphere is taller than the ellipsoid"
    );
    snapshot.visuals[0] = PrimitiveVisual::Sphere(SphereVisual {
        id: 1,
        pose: ellipsoid.pose,
        radius: 1.0,
        color: ellipsoid.color,
    });
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    let sphere_frame = image(&mut renderer);
    snapshot.visuals[0] = PrimitiveVisual::Ellipsoid(EllipsoidVisual {
        radii: DVec3::ONE,
        ..ellipsoid.clone()
    });
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert_eq!(
        image(&mut renderer).rgba,
        sphere_frame.rgba,
        "equal radii must preserve every accepted sphere pixel"
    );
    renderer.request_readback().unwrap();
    let mut invalid = snapshot.clone();
    invalid.visuals[0] = PrimitiveVisual::Ellipsoid(EllipsoidVisual {
        radii: DVec3::new(1.0, 0.0, 1.0),
        ..ellipsoid.clone()
    });
    assert!(matches!(
        renderer.set_calibrated_primitive_snapshot(&invalid, calibration, camera),
        Err(RenderError::InvalidVisual)
    ));
    let mut completed = None;
    for _ in 0..200 {
        renderer.update().unwrap();
        if let Some(frame) = renderer.take_frame().unwrap() {
            completed = Some(frame);
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    let completed =
        completed.expect("invalid semiaxis must not cancel the actual old sphere capture");
    assert_eq!(
        (completed.epoch, completed.time),
        (snapshot.epoch, snapshot.time)
    );
    assert_eq!(completed.rgba, sphere_frame.rgba);
    renderer.request_readback().unwrap();
    snapshot.visuals[0] = PrimitiveVisual::Ellipsoid(ellipsoid.clone());
    snapshot.time = Duration::from_millis(3);
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    retire_cancelled(&mut renderer);
    let baseline = image(&mut renderer);
    assert_eq!(
        (baseline.epoch, baseline.time),
        (snapshot.epoch, snapshot.time)
    );
    // Interior samples: these distinguish both a spherical proxy and the AABB,
    // while the nearer box retains actual opaque occlusion at the center.
    for (x, y) in [(42, 32), (32, 27)] {
        assert_eq!(pixel(&baseline, x, y), [255, 0, 0, 255]);
    }
    for (x, y) in [(43, 37), (32, 41)] {
        assert_eq!(pixel(&baseline, x, y), [0, 0, 255, 255]);
    }
    assert_eq!(pixel(&baseline, 32, 32), [0, 255, 0, 255]);
    let mut rolled = ellipsoid.clone();
    rolled.pose.rotation = DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);
    snapshot.visuals[0] = PrimitiveVisual::Ellipsoid(rolled);
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    let vertical = image(&mut renderer);
    for (x, y) in [(32, 42), (27, 32)] {
        assert_eq!(pixel(&vertical, x, y), [255, 0, 0, 255]);
    }
    for (x, y) in [(37, 43), (41, 32)] {
        assert_eq!(pixel(&vertical, x, y), [0, 0, 255, 255]);
    }
    assert_eq!(pixel(&vertical, 32, 32), [0, 255, 0, 255]);
    snapshot.visuals[0] = PrimitiveVisual::Ellipsoid(ellipsoid.clone());
    snapshot.visuals.reverse();
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert_eq!(
        image(&mut renderer).rgba,
        baseline.rgba,
        "input order cannot change opaque ellipsoid output"
    );
    let mut local = snapshot.clone();
    local.origin = DVec3::ZERO;
    for visual in &mut local.visuals {
        match visual {
            PrimitiveVisual::Box(v) => v.pose.translation -= origin,
            PrimitiveVisual::Ellipsoid(v) => v.pose.translation -= origin,
            PrimitiveVisual::Sphere(_)
            | PrimitiveVisual::Cylinder(_)
            | PrimitiveVisual::Capsule(_) => unreachable!(),
        }
    }
    let local_camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
    };
    renderer
        .set_calibrated_primitive_snapshot(&local, calibration, local_camera)
        .unwrap();
    assert_eq!(
        image(&mut renderer).rgba,
        baseline.rgba,
        "large origin must preserve the complete ellipsoid view"
    );
    let spin = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    for visual in &mut local.visuals {
        let pose = match visual {
            PrimitiveVisual::Box(v) => &mut v.pose,
            PrimitiveVisual::Ellipsoid(v) => &mut v.pose,
            PrimitiveVisual::Sphere(_)
            | PrimitiveVisual::Cylinder(_)
            | PrimitiveVisual::Capsule(_) => unreachable!(),
        };
        pose.translation = spin * pose.translation;
        pose.rotation = spin * pose.rotation;
    }
    renderer
        .set_calibrated_primitive_snapshot(
            &local,
            calibration,
            WorldPose {
                rotation: spin,
                ..local_camera
            },
        )
        .unwrap();
    assert_eq!(
        image(&mut renderer).rgba,
        baseline.rgba,
        "common camera/visual rigid rotation must preserve every pixel"
    );
    snapshot.visuals = vec![
        PrimitiveVisual::Cylinder(CylinderVisual {
            id: 1,
            pose: ellipsoid.pose,
            radius: 1.0,
            length: 2.0,
            color: ellipsoid.color,
        }),
        PrimitiveVisual::Box(backdrop.clone()),
        PrimitiveVisual::Box(near_box.clone()),
    ];
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert_eq!(pixel(&image(&mut renderer), 32, 32), [0, 255, 0, 255]);
    snapshot.visuals[0] = PrimitiveVisual::Sphere(SphereVisual {
        id: 1,
        pose: ellipsoid.pose,
        radius: 1.0,
        color: ellipsoid.color,
    });
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert_eq!(
        image(&mut renderer).rgba,
        sphere_frame.rgba,
        "ellipsoid/cylinder/sphere roundtrip must restore the old sphere frame"
    );
    old_box.epoch += 1;
    old_box.time = Duration::ZERO;
    renderer
        .set_calibrated_snapshot(&old_box, calibration, camera)
        .unwrap();
    let restored = image(&mut renderer);
    assert_eq!(
        (restored.epoch, restored.time),
        (old_box.epoch, old_box.time)
    );
    assert_eq!(
        restored.rgba, box_frame.rgba,
        "full legacy Box image must survive every shape replacement"
    );
    renderer.request_readback().unwrap();
    snapshot.epoch = old_box.epoch + 1;
    snapshot.time = Duration::ZERO;
    snapshot.visuals.clear();
    renderer
        .set_calibrated_primitive_snapshot(&snapshot, calibration, camera)
        .unwrap();
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    retire_cancelled(&mut renderer);
    let empty = image(&mut renderer);
    assert_eq!((empty.epoch, empty.time), (snapshot.epoch, Duration::ZERO));
    assert!(empty.rgba.chunks_exact(4).all(|p| p == [43, 44, 47, 255]));
}

// A bounded, test-only triangle oracle over independently constructed locked
// Bevy meshes. It never reads the renderer's entities, meshes or depth decoder.
// Gazebo camera rays have local X=1, so intersection t is AXIAL metres.
fn primitive_mesh_axial_reference(
    visuals: &[(&bevy::prelude::Mesh, WorldPose, DVec3)],
    config: gz_render_frame::camera::CameraConfig,
    camera: WorldPose,
    origin: DVec3,
    pixel: (usize, usize),
) -> f32 {
    use bevy::mesh::VertexAttributeValues;
    assert!(visuals.len() <= 6 && pixel.0 < 64 && pixel.1 < 64);
    assert_eq!((config.width, config.height), (64, 64));
    assert_eq!((config.skew, config.tx, config.ty), (0.0, 0.0, 0.0));
    // Derive the pinhole coefficients independently from declared intrinsics;
    // narrow each coefficient as the GPU does, without a production projection
    // or conversion helper concealing an incorrect camera transform.
    let width = f64::from(config.width);
    let height = f64::from(config.height);
    let focal_x = f64::from((2.0 * config.fx / width) as f32);
    let focal_y = f64::from((2.0 * config.fy / height) as f32);
    let centre_x = f64::from((1.0 - 2.0 * config.cx / width) as f32);
    let centre_y = f64::from((2.0 * config.cy / height - 1.0) as f32);
    let nx = 2.0 * (pixel.0 as f64 + 0.5) / width - 1.0;
    let ny = 1.0 - 2.0 * (pixel.1 as f64 + 0.5) / height;
    let ray =
        camera.rotation * DVec3::new(1.0, -(nx + centre_x) / focal_x, (ny + centre_y) / focal_y);
    let start = camera.translation - origin;
    let mut nearest = f64::INFINITY;
    for &(mesh, pose, dimensions) in visuals {
        let VertexAttributeValues::Float32x3(positions) = mesh
            .attribute(bevy::prelude::Mesh::ATTRIBUTE_POSITION)
            .unwrap()
        else {
            panic!("locked mesh positions changed")
        };
        let indices: Vec<_> = mesh.indices().unwrap().iter().collect();
        assert!(indices.len().is_multiple_of(3));
        // Independent Bevy(local X,Y,Z)->Gazebo(local X,-Z,Y) contract.
        // Match admitted f32 scale, without consulting an installed Transform.
        let scale = dimensions.as_vec3().as_dvec3();
        let vertex = |index: usize| {
            let [x, y, z] = positions[index].map(f64::from);
            (pose.translation - origin) + pose.rotation * (DVec3::new(x, -z, y) * scale)
        };
        for triangle in indices.chunks_exact(3) {
            let a = vertex(triangle[0]);
            let e1 = vertex(triangle[1]) - a;
            let e2 = vertex(triangle[2]) - a;
            let cross = ray.cross(e2);
            let det = e1.dot(cross);
            if det.abs() <= f64::EPSILON {
                continue;
            }
            let delta = start - a;
            let u = delta.dot(cross) / det;
            if !(0.0..=1.0).contains(&u) {
                continue;
            }
            let q = delta.cross(e1);
            let v = ray.dot(q) / det;
            if v < 0.0 || u + v > 1.0 {
                continue;
            }
            let t = e2.dot(q) / det;
            if t > config.near && t < config.far {
                nearest = nearest.min(t);
            }
        }
    }
    nearest as f32
}

fn retire_primitive_depth_cancelled(renderer: &mut Renderer) {
    for _ in 0..200 {
        renderer.update().unwrap();
        assert!(renderer.take_frame().unwrap().is_none());
        assert!(renderer.take_depth_frame().unwrap().is_none());
        if !renderer.readback_in_flight() {
            return;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    panic!("shared primitive-depth cancellation did not retire");
}

#[test]
fn software_primitive_depth_matches_finite_meshes_and_preserves_modes_and_old_frames() {
    use bevy::{
        mesh::primitives::{CylinderAnchor, CylinderMeshBuilder},
        prelude::{Cuboid, MeshBuilder, Meshable, Sphere},
    };
    use gz_render_frame::camera::{Calibration, CameraConfig};
    use gz_renderer::{
        CylinderVisual, EllipsoidVisual, PrimitiveSnapshot, PrimitiveVisual, SphereVisual,
    };
    let mut renderer = Renderer::new().unwrap();
    // Independently declared 64x64, pi/2 pinhole intrinsics; the oracle reads
    // this same input contract, never Calibration's derived projection.
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
    let calibration = Calibration::new(config).unwrap();
    let camera = WorldPose {
        translation: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
    };
    let pose = |x, y, z| WorldPose {
        translation: DVec3::new(x, y, z),
        rotation: DQuat::IDENTITY,
    };
    let box_at = |id, x, size| BoxVisual {
        id,
        pose: pose(x, 0., 0.),
        size,
        color: [0., 0., 1.],
    };
    let old = Snapshot {
        epoch: 501,
        time: Duration::ZERO,
        origin: DVec3::ZERO,
        visuals: vec![box_at(1, 4., DVec3::splat(1.5))],
    };
    renderer
        .set_calibrated_snapshot(&old, calibration, camera)
        .unwrap();
    let old_rgb = image(&mut renderer).rgba;
    renderer
        .set_calibrated_depth_snapshot(&old, calibration, camera)
        .unwrap();
    let old_depth = metric_depth(&mut renderer).metres;
    let cylinder_pose = WorldPose {
        rotation: DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2),
        ..pose(4., -1.5, 0.)
    };
    let sphere_pose = pose(4., 0., 0.);
    let ellipsoid_pose = pose(4., 1.5, 0.);
    let backdrop = box_at(4, 7., DVec3::new(0.2, 8., 8.));
    let occluder = box_at(5, 2., DVec3::splat(0.4));
    let mut scene = PrimitiveSnapshot {
        epoch: 502,
        time: Duration::from_millis(100),
        origin: DVec3::ZERO,
        visuals: vec![
            PrimitiveVisual::Cylinder(CylinderVisual {
                id: 1,
                pose: cylinder_pose,
                radius: 0.5,
                length: 1.,
                color: [1., 0., 0.],
            }),
            PrimitiveVisual::Sphere(SphereVisual {
                id: 2,
                pose: sphere_pose,
                radius: 0.75,
                color: [1., 0., 0.],
            }),
            PrimitiveVisual::Ellipsoid(EllipsoidVisual {
                id: 3,
                pose: ellipsoid_pose,
                radii: DVec3::new(0.5, 0.6, 0.35),
                color: [1., 0., 0.],
            }),
            PrimitiveVisual::Box(backdrop.clone()),
            PrimitiveVisual::Box(occluder.clone()),
        ],
    };
    // Independently create the pinned meshes, never clone installed GPU assets.
    let cube = Cuboid::new(1., 1., 1.).mesh().build();
    let cylinder = CylinderMeshBuilder {
        caps: true,
        ..CylinderMeshBuilder::new(0.5, 1., 32)
            .segments(1)
            .anchor(CylinderAnchor::MidPoint)
    }
    .build();
    let sphere = Sphere::new(0.5).mesh().uv(32, 18);
    let reference = [
        (&cylinder, cylinder_pose, DVec3::ONE),
        (&sphere, sphere_pose, DVec3::splat(1.5)),
        (&sphere, ellipsoid_pose, DVec3::new(1., 1.2, 0.7)),
        (&cube, backdrop.pose, backdrop.size),
        (&cube, occluder.pose, occluder.size),
    ];
    renderer
        .set_calibrated_primitive_depth_snapshot(&scene, calibration, camera)
        .unwrap();
    let frame = metric_depth(&mut renderer);
    assert_eq!((frame.epoch, frame.time), (scene.epoch, scene.time));
    for pixel in [
        (32, 32),
        (44, 32),
        (32, 27),
        (29, 30),
        (20, 32),
        (22, 31),
        (40, 35),
        (16, 34),
        (0, 0),
    ] {
        let expected =
            primitive_mesh_axial_reference(&reference, config, camera, DVec3::ZERO, pixel);
        let actual = frame.metres[pixel.1 * 64 + pixel.0];
        if expected == f32::INFINITY {
            assert_eq!(actual, f32::INFINITY, "background {pixel:?}");
        } else {
            assert!(
                (actual - expected).abs() <= 1e-4,
                "finite mesh {pixel:?}: {actual} != {expected}"
            );
        }
    }
    assert!((frame.metres[32 * 64 + 32] - 1.8).abs() <= 1e-4);
    assert!((frame.metres[32 * 64 + 44] - 3.5).abs() <= 1e-4);
    for (x, y) in [(40, 35), (16, 34)] {
        assert!((frame.metres[y * 64 + x] - 6.9).abs() <= 1e-4);
    }
    assert_eq!(frame.metres[0], f32::INFINITY);
    assert_ne!(
        frame.metres[27 * 64 + 32].to_bits(),
        frame.metres[30 * 64 + 29].to_bits(),
        "curved sphere must not become a flat box proxy"
    );
    // Common rigid camera/world motion and large origin preserve axial samples.
    let offset = DVec3::splat(1e9);
    let turn = DQuat::from_rotation_z(0.37);
    scene.origin = offset;
    for visual in &mut scene.visuals {
        let pose = match visual {
            PrimitiveVisual::Box(v) => &mut v.pose,
            PrimitiveVisual::Cylinder(v) => &mut v.pose,
            PrimitiveVisual::Sphere(v) => &mut v.pose,
            PrimitiveVisual::Ellipsoid(v) => &mut v.pose,
            PrimitiveVisual::Capsule(v) => &mut v.pose,
        };
        pose.translation = offset + turn * pose.translation;
        pose.rotation = turn * pose.rotation;
    }
    let shifted_camera = WorldPose {
        translation: offset,
        rotation: turn,
    };
    renderer
        .set_calibrated_primitive_depth_snapshot(&scene, calibration, shifted_camera)
        .unwrap();
    let shifted = metric_depth(&mut renderer);
    for (a, b) in frame.metres.iter().zip(&shifted.metres) {
        if a.is_infinite() {
            assert_eq!(*b, f32::INFINITY);
        } else {
            assert!((a - b).abs() <= 1e-4);
        }
    }
    // Same-ID, same fixed mesh and equal radii must give full bitwise depth equality.
    scene.origin = DVec3::ZERO;
    scene.visuals = vec![PrimitiveVisual::Sphere(SphereVisual {
        id: 1,
        pose: sphere_pose,
        radius: 0.75,
        color: [1., 0., 0.],
    })];
    renderer
        .set_calibrated_primitive_depth_snapshot(&scene, calibration, camera)
        .unwrap();
    let equal_sphere = metric_depth(&mut renderer);
    scene.visuals = vec![PrimitiveVisual::Ellipsoid(EllipsoidVisual {
        id: 1,
        pose: sphere_pose,
        radii: DVec3::splat(0.75),
        color: [1., 0., 0.],
    })];
    renderer
        .set_calibrated_primitive_depth_snapshot(&scene, calibration, camera)
        .unwrap();
    let equal_ellipsoid = metric_depth(&mut renderer);
    assert_eq!(
        equal_sphere
            .metres
            .iter()
            .map(|v| v.to_bits())
            .collect::<Vec<_>>(),
        equal_ellipsoid
            .metres
            .iter()
            .map(|v| v.to_bits())
            .collect::<Vec<_>>()
    );
    // Local ellipsoid orientation is part of the geometric contract, not an
    // enclosing-box depth surface. Compare only declared interior rays.
    let rolled_pose = WorldPose {
        rotation: DQuat::from_rotation_x(0.4),
        ..sphere_pose
    };
    scene.visuals = vec![PrimitiveVisual::Ellipsoid(EllipsoidVisual {
        id: 1,
        pose: rolled_pose,
        radii: DVec3::new(0.5, 0.8, 0.6),
        color: [1., 0., 0.],
    })];
    renderer
        .set_calibrated_primitive_depth_snapshot(&scene, calibration, camera)
        .unwrap();
    let rolled = metric_depth(&mut renderer);
    let rolled_reference = [(&sphere, rolled_pose, DVec3::new(1., 1.6, 1.2))];
    for pixel in [(32, 30), (30, 33), (34, 32)] {
        let expected =
            primitive_mesh_axial_reference(&rolled_reference, config, camera, DVec3::ZERO, pixel);
        assert!(expected.is_finite());
        assert!((rolled.metres[pixel.1 * 64 + pixel.0] - expected).abs() <= 1e-4);
    }
    // A valid mode switch cancels the pending depth frame but keeps its slot
    // until the actual callback retires. No stale frame may escape.
    request_metric_depth(&mut renderer);
    renderer
        .set_calibrated_snapshot(&old, calibration, camera)
        .unwrap();
    assert!(matches!(
        renderer.request_readback(),
        Err(RenderError::Busy)
    ));
    retire_primitive_depth_cancelled(&mut renderer);
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::NoDepthSnapshot)
    ));
    assert_eq!(image(&mut renderer).rgba, old_rgb);
    renderer.request_readback().unwrap();
    renderer
        .set_calibrated_primitive_depth_snapshot(&scene, calibration, camera)
        .unwrap();
    assert!(matches!(
        renderer.request_depth_readback(),
        Err(RenderError::Busy)
    ));
    retire_primitive_depth_cancelled(&mut renderer);
    metric_depth(&mut renderer);
    renderer
        .set_calibrated_depth_snapshot(&old, calibration, camera)
        .unwrap();
    assert_eq!(
        metric_depth(&mut renderer)
            .metres
            .iter()
            .map(|v| v.to_bits())
            .collect::<Vec<_>>(),
        old_depth.iter().map(|v| v.to_bits()).collect::<Vec<_>>()
    );
    scene.epoch += 1;
    scene.time = Duration::ZERO;
    scene.visuals.clear();
    renderer
        .set_calibrated_primitive_depth_snapshot(&scene, calibration, camera)
        .unwrap();
    let empty = metric_depth(&mut renderer);
    assert_eq!((empty.epoch, empty.time), (scene.epoch, Duration::ZERO));
    assert!(empty.metres.iter().all(|v| *v == f32::INFINITY));
    // Far clipping remains exact +INF, independently of a primitive's mesh.
    scene.visuals = vec![PrimitiveVisual::Sphere(SphereVisual {
        id: 1,
        pose: pose(20., 0., 0.),
        radius: 0.75,
        color: [1., 0., 0.],
    })];
    renderer
        .set_calibrated_primitive_depth_snapshot(&scene, calibration, camera)
        .unwrap();
    assert!(
        metric_depth(&mut renderer)
            .metres
            .iter()
            .all(|v| *v == f32::INFINITY)
    );
}
