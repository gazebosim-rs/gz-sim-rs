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
            skew: f64::INFINITY,
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

#[test]
fn retained_native_zero_and_positive_skew_matrices_match_reverse_depth_projection() {
    let mut capture = include_str!("fixtures/native_projection_capture.txt").lines();
    for skew in [0.0, 35.0] {
        assert_eq!(capture.next().unwrap(), format!("skew {skew}"));
        let native: Vec<Vec<f64>> = capture
            .by_ref()
            .take(4)
            .map(|line| {
                line.split_whitespace()
                    .map(|value| value.parse().unwrap())
                    .collect()
            })
            .collect();
        assert_eq!(native.len(), 4);
        assert!(native.iter().all(|row| row.len() == 4));
        let calibration = Calibration::new(CameraConfig { skew, ..config() }).unwrap();
        let matrix = calibration.projection().to_cols_array_2d();
        for (row, native_row) in native.iter().enumerate() {
            for (column, actual_column) in matrix.iter().enumerate() {
                let expected = if row == 2 {
                    (native[3][column] - native_row[column]) / 2.0
                } else {
                    native_row[column]
                };
                close(actual_column[row], expected);
            }
        }
        close(matrix[1][0], 2.0 * skew / 640.0);
    }
    assert!(capture.next().is_none());
}

#[test]
fn signed_skew_optical_pixels_and_inverse_rays_match_the_clip_matrix() {
    for (skew, expected_u) in [(35.0, 353.5), (-35.0, 346.5)] {
        let calibration = Calibration::new(CameraConfig { skew, ..config() }).unwrap();
        close(calibration.projection().y_axis.x, 2.0 * skew / 640.0);
        assert_eq!(
            calibration
                .project_optical(DVec3::new(0.4, -0.2, 2.0))
                .unwrap(),
            [expected_u, 145.0]
        );
        let inverse = calibration.projection().inverse();
        for point in [DVec3::new(0.4, -0.2, 2.0), DVec3::new(-0.3, 0.5, 4.0)] {
            let pixel = calibration.project_optical(point).unwrap();
            let clip = calibration.projection() * DVec4::new(point.x, -point.y, -point.z, 1.0);
            close(pixel[0], (clip.x / clip.w + 1.0) * 320.0);
            close(pixel[1], (1.0 - clip.y / clip.w) * 240.0);
            let ray_y = (pixel[1] - 190.0) / 450.0;
            let ray_x = ((pixel[0] - 270.0) + skew * ray_y) / 400.0;
            close(ray_x, point.x / point.z);
            close(ray_y, point.y / point.z);
            let view = inverse.project_point3(clip.truncate() / clip.w);
            assert!((view - DVec3::new(point.x, -point.y, -point.z)).length() < 1e-12);
        }
        assert_eq!(
            calibration.project_optical(DVec3::new(0.0, f64::MAX, 0.1)),
            Err(CameraError::ProjectionOutOfRange)
        );
    }
}

#[test]
fn shared_skew_corners_preserve_plane_order_and_project_to_image_edges() {
    for skew in [16.0, -16.0] {
        let calibration = Calibration::new(CameraConfig {
            width: 64,
            height: 64,
            fx: 32.0,
            fy: 16.0,
            cx: 32.0,
            cy: 32.0,
            skew,
            near: 1.0,
            far: 8.0,
            ..config()
        })
        .unwrap();
        let first = if skew > 0.0 {
            [
                [4.0, -4.0, -2.0],
                [0.0, 4.0, -2.0],
                [-4.0, 4.0, -2.0],
                [0.0, -4.0, -2.0],
            ]
        } else {
            [
                [0.0, -4.0, -2.0],
                [4.0, 4.0, -2.0],
                [0.0, 4.0, -2.0],
                [-4.0, -4.0, -2.0],
            ]
        };
        let corners = calibration.frustum_corners(-2.0, -4.0);
        let image_edges = [[64.0, 64.0], [64.0, 0.0], [0.0, 0.0], [0.0, 64.0]];
        for (index, (point, expected)) in corners[..4].iter().zip(first).enumerate() {
            assert_eq!(*point, DVec3::from_array(expected));
            assert_eq!(corners[index + 4], *point * 2.0);
            assert_eq!(
                calibration
                    .project_optical(DVec3::new(point.x, -point.y, -point.z))
                    .unwrap(),
                image_edges[index]
            );
            let clip = calibration.projection() * point.extend(1.0);
            close((clip.x / clip.w + 1.0) * 32.0, image_edges[index][0]);
            close((1.0 - clip.y / clip.w) * 32.0, image_edges[index][1]);
        }
        let positive_plane = calibration.frustum_corners(2.0, -4.0);
        for (positive, negative) in positive_plane[..4].iter().zip(&corners[..4]) {
            assert_eq!(
                (positive.x, positive.y, positive.z),
                (negative.x, negative.y, -negative.z)
            );
        }
    }
}

#[test]
fn skew_representability_rejects_non_normal_shear_coupled_corners_and_invalid_inverse() {
    for skew in [f64::MIN_POSITIVE, -f64::MIN_POSITIVE, f64::MAX, -f64::MAX] {
        assert_eq!(
            Calibration::new(CameraConfig { skew, ..config() }).unwrap_err(),
            CameraError::ProjectionOutOfRange
        );
    }
    let coupled = CameraConfig {
        width: 64,
        height: 64,
        fx: 1e-6,
        fy: 1e-6,
        cx: 32.0,
        cy: 32.0,
        near: 1e30,
        far: 2e30,
        ..config()
    };
    let plain = Calibration::new(coupled).unwrap();
    assert!(
        plain
            .frustum_corners(-coupled.near, -coupled.far)
            .iter()
            .all(|point| point.as_vec3().is_finite())
    );
    for skew in [1.0, -1.0] {
        assert_eq!(
            Calibration::new(CameraConfig { skew, ..coupled }).unwrap_err(),
            CameraError::ProjectionOutOfRange
        );
    }
    let inverse = CameraConfig {
        fx: 1e22,
        fy: 1e22,
        near: 1.0,
        far: 2.0,
        ..config()
    };
    assert!(Calibration::new(inverse).is_ok());
    assert_eq!(
        Calibration::new(CameraConfig {
            skew: 1.0,
            ..inverse
        })
        .unwrap_err(),
        CameraError::ProjectionOutOfRange
    );
    let rounded_plane = CameraConfig {
        width: 64,
        height: 64,
        fx: 2.0_f64.powi(-121),
        fy: 32.0,
        cx: 32.0,
        cy: 32.0,
        near: 0.5,
        far: 1.0 - 2.0_f64.powi(-26),
        skew: 96.0 - 2.0_f64.powi(-18) + 2.0_f64.powi(-20),
        ..config()
    };
    // The configured far corner fits, but the actual f32 plane rounds outward
    // to 1 and its sheared right-bottom corner exceeds the f32 overflow midpoint.
    let corner_x = |distance| (32.0 + rounded_plane.skew) * distance / rounded_plane.fx;
    assert!(corner_x(rounded_plane.far).is_finite());
    assert!((corner_x(rounded_plane.far) as f32).is_finite());
    assert_eq!(rounded_plane.far as f32, 1.0);
    assert!((corner_x(f64::from(rounded_plane.far as f32)) as f32).is_infinite());
    assert!(
        Calibration::new(CameraConfig {
            skew: 0.0,
            ..rounded_plane
        })
        .is_ok()
    );
    assert_eq!(
        Calibration::new(rounded_plane).unwrap_err(),
        CameraError::ProjectionOutOfRange
    );
}

#[test]
fn zero_skew_preserves_exact_dyadic_matrix_and_frustum_values() {
    let expected_matrix = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.5, 0.25, 1.0, -1.0, 0.0, 0.0, 2.0, 0.0,
    ];
    let expected_corners = [
        [1.5, -0.75, -1.0],
        [1.5, 1.25, -1.0],
        [-0.5, 1.25, -1.0],
        [-0.5, -0.75, -1.0],
        [3.0, -1.5, -2.0],
        [3.0, 2.5, -2.0],
        [-1.0, 2.5, -2.0],
        [-1.0, -1.5, -2.0],
    ];
    for skew in [0.0, -0.0] {
        let calibration = Calibration::new(CameraConfig {
            width: 64,
            height: 64,
            fx: 32.0,
            fy: 32.0,
            cx: 16.0,
            cy: 40.0,
            near: 1.0,
            far: 2.0,
            skew,
            ..config()
        })
        .unwrap();
        assert_eq!(calibration.projection().to_cols_array(), expected_matrix);
        assert_eq!(
            calibration
                .frustum_corners(-1.0, -2.0)
                .map(|point| point.to_array()),
            expected_corners
        );
        assert_eq!(
            calibration
                .project_optical(DVec3::new(0.25, -0.5, 1.0))
                .unwrap(),
            [24.0, 24.0]
        );
    }
}
