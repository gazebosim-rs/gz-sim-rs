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
