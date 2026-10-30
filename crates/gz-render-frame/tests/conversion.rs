// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3, Quat, Vec3};
use gz_render_frame::{ConversionError, WorldPose, to_render_pose};

fn pose(translation: DVec3, rotation: DQuat) -> WorldPose {
    WorldPose {
        translation,
        rotation,
    }
}

#[test]
fn rebases_before_narrowing_and_maps_axes() {
    let origin = DVec3::new(1e12, -1e12, 1e12);
    let p = pose(origin + DVec3::new(1.25, 2.5, 3.75), DQuat::IDENTITY);
    let render = to_render_pose(p, origin).unwrap();
    assert_eq!(render.translation, Vec3::new(1.25, 3.75, -2.5));
    assert_eq!(render.rotation, Quat::IDENTITY);
}

#[test]
fn rotation_changes_both_coordinate_bases() {
    for (axis, vector, expected) in [
        (DVec3::Z, Vec3::X, -Vec3::Z),
        (DVec3::X, -Vec3::Z, Vec3::Y),
        (DVec3::Y, Vec3::Y, Vec3::X),
    ] {
        let q = DQuat::from_axis_angle(axis, std::f64::consts::FRAC_PI_2);
        let result = to_render_pose(pose(DVec3::ZERO, q), DVec3::ZERO).unwrap();
        assert!((result.rotation * vector - expected).length() < 1e-6);
    }
}

#[test]
fn rejects_non_finite_non_unit_and_unrepresentable_inputs() {
    let identity = pose(DVec3::ZERO, DQuat::IDENTITY);
    assert_eq!(
        to_render_pose(identity, DVec3::splat(f64::NAN)),
        Err(ConversionError::NonFinite)
    );
    assert_eq!(
        to_render_pose(
            pose(DVec3::splat(f64::INFINITY), DQuat::IDENTITY),
            DVec3::ZERO
        ),
        Err(ConversionError::NonFinite)
    );
    assert_eq!(
        to_render_pose(
            pose(DVec3::ZERO, DQuat::from_xyzw(f64::NAN, 0.0, 0.0, 1.0)),
            DVec3::ZERO
        ),
        Err(ConversionError::NonFinite)
    );
    assert_eq!(
        to_render_pose(
            pose(DVec3::ZERO, DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0)),
            DVec3::ZERO
        ),
        Err(ConversionError::NonUnitRotation)
    );
    assert_eq!(
        to_render_pose(
            pose(DVec3::splat(f64::MAX), DQuat::IDENTITY),
            DVec3::splat(-f64::MAX)
        ),
        Err(ConversionError::PositionOutOfRange)
    );
    assert_eq!(
        to_render_pose(
            pose(DVec3::splat(f64::from(f32::MAX) * 2.0), DQuat::IDENTITY),
            DVec3::ZERO
        ),
        Err(ConversionError::PositionOutOfRange)
    );
}

#[test]
fn accepts_roundoff_and_rejects_quaternion_norm_errors() {
    let near_unit = DQuat::from_xyzw(0.0, 0.0, 0.0, 1.0 + 2e-13);
    let result = to_render_pose(pose(DVec3::ZERO, near_unit), DVec3::ZERO).unwrap();
    assert_eq!(result.rotation, Quat::IDENTITY);
    let non_unit = DQuat::from_xyzw(0.0, 0.0, 0.0, 1.0 + 1e-11);
    assert_eq!(
        to_render_pose(pose(DVec3::ZERO, non_unit), DVec3::ZERO),
        Err(ConversionError::NonUnitRotation)
    );
    let huge = DQuat::from_xyzw(f64::MAX, 0.0, 0.0, 1.0);
    assert_eq!(
        to_render_pose(pose(DVec3::ZERO, huge), DVec3::ZERO),
        Err(ConversionError::NonUnitRotation)
    );
}

#[test]
fn rotated_vectors_agree_for_general_orientation() {
    let rotation = DQuat::from_euler(glam::EulerRot::XYZ, 0.3, -1.2, 2.1);
    let local = DVec3::new(0.4, -0.6, 1.7);
    let rotated = rotation * local;
    let expected = Vec3::new(rotated.x as f32, rotated.z as f32, -rotated.y as f32);
    let render_local = Vec3::new(local.x as f32, local.z as f32, -local.y as f32);
    let render_pose = to_render_pose(pose(DVec3::ZERO, rotation), DVec3::ZERO).unwrap();
    assert!((render_pose.rotation * render_local - expected).length() < 1e-6);
}
