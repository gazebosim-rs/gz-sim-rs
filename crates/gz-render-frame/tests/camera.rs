// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3, DVec4, Vec3};
use gz_render_frame::{
    WorldPose,
    camera::{Calibration, CameraConfig, CameraError, to_render_camera_pose},
};

fn config() -> CameraConfig {
    CameraConfig {
        width: 640,
        height: 480,
        fx: 400.0,
        fy: 450.0,
        cx: 270.0,
        cy: 190.0,
        skew: 0.0,
        tx: 0.0,
        ty: 0.0,
        near: 0.1,
        far: 100.0,
    }
}
fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
}

#[test]
fn pinned_native_asymmetric_projection_matches_after_opengl_to_reverse_depth() {
    // Pinned gz-sensors 8ddf4c CameraSensorUtil.cc, compiled directly with
    // canonical gz-math headers: width=640, height=480, fx=400, fy=450,
    // cx=270, cy=190, skew=0, near=.1, far=100. These are native OpenGL rows.
    let native = [
        [1.25, 0.0, 0.15625, 0.0],
        [0.0, 1.875, -0.20833333333333326, 0.0],
        [0.0, 0.0, -1.002002002002002, -0.20020020020020018],
        [0.0, 0.0, -1.0, 0.0],
    ];
    let calibration = Calibration::new(config()).unwrap();
    let matrix = calibration.projection().to_cols_array_2d();
    for row in 0..4 {
        for column in 0..4 {
            let expected = if row == 2 {
                (native[3][column] - native[2][column]) / 2.0
            } else {
                native[row][column]
            };
            close(matrix[column][row], expected);
        }
    }
    for (depth, expected) in [(0.1, 1.0), (100.0, 0.0)] {
        let clip = calibration.projection() * DVec4::new(0.0, 0.0, -depth, 1.0);
        close(clip.z / clip.w, expected);
    }
    assert!(calibration.render_projection().is_finite());
}

#[test]
fn optical_pixels_match_clip_matrix_for_off_axis_nonsquare_camera() {
    let calibration = Calibration::new(config()).unwrap();
    for optical in [
        DVec3::new(0.0, 0.0, 2.0),
        DVec3::new(0.4, -0.2, 2.0),
        DVec3::new(-1.0, 0.5, 4.0),
    ] {
        let pixel = calibration.project_optical(optical).unwrap();
        let clip = calibration.projection() * DVec4::new(optical.x, -optical.y, -optical.z, 1.0);
        close(pixel[0], (clip.x / clip.w + 1.0) * 320.0);
        close(pixel[1], (1.0 - clip.y / clip.w) * 240.0);
    }
    assert_eq!(
        calibration
            .project_optical(DVec3::new(0.4, -0.2, 2.0))
            .unwrap(),
        [350.0, 145.0]
    );
}

#[test]
fn upstream_fov_and_explicit_intrinsic_fixture_values_are_preserved() {
    // gz-sensors CameraIntrinsics expects 866.23 within 0.3 for 1.0472 rad.
    let fixture_fov = "1.0472".parse::<f64>().unwrap();
    let derived = Calibration::from_horizontal_fov(1000, 1000, fixture_fov, 0.1, 100.0)
        .unwrap()
        .config();
    assert!((derived.fx - 866.23).abs() < 0.3);
    assert!((derived.fy - 866.23).abs() < 0.3);
    assert_eq!((derived.cx, derived.cy), (500.0, 500.0));
    for (fx, fy, cx, cy) in [(866.23, 866.23, 500.0, 500.0), (900.0, 900.0, 501.0, 501.0)] {
        let actual = Calibration::new(CameraConfig {
            width: 1000,
            height: 1000,
            fx,
            fy,
            cx,
            cy,
            ..config()
        })
        .unwrap()
        .config();
        assert_eq!(
            (actual.fx, actual.fy, actual.cx, actual.cy),
            (fx, fy, cx, cy)
        );
    }
}

#[test]
fn camera_basis_maps_gazebo_forward_left_and_up_after_rebasing() {
    let origin = DVec3::new(1e12, -1e12, 5.0);
    let pose = WorldPose {
        translation: origin + DVec3::new(0.25, -0.5, 1.0),
        rotation: DQuat::IDENTITY,
    };
    let converted = to_render_camera_pose(pose, origin).unwrap();
    assert!((converted.translation - Vec3::new(0.25, 1.0, 0.5)).length() < 1e-6);
    assert!((converted.rotation * -Vec3::Z - Vec3::X).length() < 1e-6);
    assert!((converted.rotation * Vec3::X - Vec3::Z).length() < 1e-6);
    assert!((converted.rotation * Vec3::Y - Vec3::Y).length() < 1e-6);
    let yaw = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    let rotated = to_render_camera_pose(
        WorldPose {
            rotation: yaw,
            ..pose
        },
        origin,
    )
    .unwrap();
    assert!((rotated.rotation * -Vec3::Z + Vec3::Z).length() < 1e-6);
    let rotation = DQuat::from_euler(glam::EulerRot::XYZ, 0.4, -0.3, 0.7);
    let rotated = to_render_camera_pose(WorldPose { rotation, ..pose }, origin).unwrap();
    let map =
        |direction: DVec3| Vec3::new(direction.x as f32, direction.z as f32, -direction.y as f32);
    for (camera_axis, gazebo_axis) in [
        (-Vec3::Z, DVec3::X),
        (Vec3::X, -DVec3::Y),
        (Vec3::Y, DVec3::Z),
    ] {
        assert!((rotated.rotation * camera_axis - map(rotation * gazebo_axis)).length() < 1e-6);
    }
    assert!(
        to_render_camera_pose(
            WorldPose {
                rotation: DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
                ..pose
            },
            origin
        )
        .is_err()
    );
}

#[test]
fn invalid_camera_inputs_are_rejected_before_matrix_or_projection() {
    for bad in [
        CameraConfig {
            width: 0,
            ..config()
        },
        CameraConfig {
            width: 4097,
            ..config()
        },
        CameraConfig {
            width: 4096,
            height: 4096,
            ..config()
        },
    ] {
        assert_eq!(
            Calibration::new(bad).unwrap_err(),
            CameraError::InvalidDimensions
        );
    }
    for bad in [
        CameraConfig {
            fx: 0.0,
            ..config()
        },
        CameraConfig {
            fy: f64::NAN,
            ..config()
        },
        CameraConfig {
            cx: f64::INFINITY,
            ..config()
        },
    ] {
        assert_eq!(
            Calibration::new(bad).unwrap_err(),
            CameraError::InvalidIntrinsics
        );
    }
    for bad in [
        CameraConfig {
            skew: 1.0,
            ..config()
        },
        CameraConfig {
            tx: 300.0,
            ..config()
        },
        CameraConfig {
            ty: 200.0,
            ..config()
        },
        CameraConfig {
            skew: f64::NAN,
            ..config()
        },
    ] {
        assert_eq!(
            Calibration::new(bad).unwrap_err(),
            CameraError::UnsupportedProjection
        );
    }
    for bad in [
        CameraConfig {
            near: 0.0,
            ..config()
        },
        CameraConfig {
            far: 0.1,
            ..config()
        },
        CameraConfig {
            far: f64::INFINITY,
            ..config()
        },
    ] {
        assert_eq!(Calibration::new(bad).unwrap_err(), CameraError::InvalidClip);
    }
    for bad in [
        CameraConfig {
            fx: f64::MAX,
            ..config()
        },
        CameraConfig {
            cx: f64::MAX,
            ..config()
        },
        CameraConfig {
            fx: 1e-30,
            far: 1e30,
            ..config()
        },
        CameraConfig {
            far: 1e20,
            ..config()
        },
        CameraConfig {
            near: f64::MIN_POSITIVE,
            ..config()
        },
        CameraConfig {
            near: f64::from(f32::MIN_POSITIVE) / 2.0,
            far: 1e-37,
            ..config()
        },
        CameraConfig {
            far: f64::MAX,
            ..config()
        },
    ] {
        assert_eq!(
            Calibration::new(bad).unwrap_err(),
            CameraError::ProjectionOutOfRange
        );
    }
    for fov in [0.0, std::f64::consts::PI, f64::NAN] {
        assert_eq!(
            Calibration::from_horizontal_fov(640, 480, fov, 0.1, 100.0).unwrap_err(),
            CameraError::InvalidIntrinsics
        );
    }
    let camera = Calibration::new(config()).unwrap();
    for point in [
        DVec3::ZERO,
        DVec3::new(0.0, 0.0, -1.0),
        DVec3::new(f64::NAN, 0.0, 1.0),
    ] {
        assert_eq!(
            camera.project_optical(point),
            Err(CameraError::InvalidPoint)
        );
    }
    assert_eq!(
        camera.project_optical(DVec3::new(f64::MAX, 0.0, 0.1)),
        Err(CameraError::ProjectionOutOfRange)
    );
}
