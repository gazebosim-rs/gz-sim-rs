// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_sensors::logical_camera::*;
include!("fixtures/logical_camera_native_nominal.rs");

fn config() -> LogicalCameraConfig {
    LogicalCameraConfig {
        near: 1.0,
        far: 8.0,
        horizontal_fov: std::f64::consts::FRAC_PI_2,
        aspect_ratio: 1.0,
    }
}
fn pose(p: [f64; 3]) -> LogicalCameraPose {
    LogicalCameraPose {
        translation: p,
        rotation: [0.0, 0.0, 0.0, 1.0],
    }
}
fn model(name: &str, p: [f64; 3]) -> LogicalCameraModel {
    LogicalCameraModel {
        name: name.into(),
        pose: pose(p),
    }
}
#[test]
fn nominal_points_keep_strict_adjacent_near_far_and_behind_controls() {
    // Preserve all original nominal points and their strict adjacent controls.
    // Only near/far are guaranteed exact-zero points. The typed native replay
    // below checks all original nominal classifications, including side edges.
    // A geometric tan*x edge is not a signed-distance promise.
    let h = (config().horizontal_fov * 0.5).tan() * 4.0;
    for (index, (point, outside, inside)) in [
        (
            [1.0, 0.0, 0.0],
            [1.0 - 1e-9, 0.0, 0.0],
            [1.0 + 1e-9, 0.0, 0.0],
        ),
        (
            [8.0, 0.0, 0.0],
            [8.0 + 1e-9, 0.0, 0.0],
            [8.0 - 1e-9, 0.0, 0.0],
        ),
        ([4.0, h, 0.0], [4.0, h + 1e-9, 0.0], [4.0, h - 1e-9, 0.0]),
        ([4.0, -h, 0.0], [4.0, -h - 1e-9, 0.0], [4.0, -h + 1e-9, 0.0]),
        ([4.0, 0.0, h], [4.0, 0.0, h + 1e-9], [4.0, 0.0, h - 1e-9]),
        ([4.0, 0.0, -h], [4.0, 0.0, -h - 1e-9], [4.0, 0.0, -h + 1e-9]),
    ]
    .into_iter()
    .enumerate()
    {
        let edge = logical_camera(config(), pose([0.0; 3]), &[model("edge", point)]).unwrap();
        if index < 2 {
            assert_eq!(edge.models.len(), 1, "inclusive near/far {point:?}");
        }
        // The separate actual native replay checks nominal side-edge membership.
        assert!(
            logical_camera(config(), pose([0.0; 3]), &[model("outside", outside)])
                .unwrap()
                .models
                .is_empty()
        );
        assert_eq!(
            logical_camera(config(), pose([0.0; 3]), &[model("inside", inside)])
                .unwrap()
                .models
                .len(),
            1
        );
    }
    assert!(
        logical_camera(
            config(),
            pose([0.0; 3]),
            &[model("behind", [-2.0, 0.0, 0.0])]
        )
        .unwrap()
        .models
        .is_empty()
    );
}
#[test]
fn all_native_face_centers_have_exact_zero_distance_and_strict_adjacent_sides() {
    // Native ComputePlanes corner and left-associative center arithmetic.
    let t = (config().horizontal_fov * 0.5).tan();
    let nw = 2.0 * t * config().near;
    let fw = 2.0 * t * config().far;
    let nc = DVec3::new(1.0, 0.0, 0.0);
    let fc = DVec3::new(8.0, 0.0, 0.0);
    let un = DVec3::Z * (nw * 0.5);
    let rn = (-DVec3::Y) * (nw * 0.5);
    let uf = DVec3::Z * (fw * 0.5);
    let rf = (-DVec3::Y) * (fw * 0.5);
    let ntl = nc + un - rn;
    let ntr = nc + un + rn;
    let nbl = nc - un - rn;
    let nbr = nc - un + rn;
    let ftl = fc + uf - rf;
    let ftr = fc + uf + rf;
    let fbl = fc - uf - rf;
    let fbr = fc - uf + rf;
    for (a, b, c, center) in [
        (ntl, ntr, nbl, nc),
        (ftr, ftl, fbl, fc),
        (ftl, ntl, nbl, (ftl + ntl + fbl + nbl) / 4.0),
        (ntr, ftr, fbr, (ftr + ntr + fbr + nbr) / 4.0),
        (ntl, ftl, ntr, (ftr + ntr + ftl + ntl) / 4.0),
        (nbl, nbr, fbr, (fbr + nbr + fbl + nbl) / 4.0),
    ] {
        let n = (b - a).cross(c - a);
        let n = n / n.length();
        let offset = center.dot(n);
        assert_eq!(n.dot(center) - offset, 0.0, "exact zero, no epsilon");
        let inside = center + n * 1e-9;
        let outside = center - n * 1e-9;
        assert!(n.dot(inside) - offset > 0.0);
        assert!(n.dot(outside) - offset < 0.0);
        for (point, visible) in [(center, true), (inside, true), (outside, false)] {
            let reading = logical_camera(
                config(),
                pose([0.0; 3]),
                &[model("target", point.to_array())],
            )
            .unwrap();
            assert_eq!(
                reading
                    .models
                    .iter()
                    .map(|m| m.name.as_str())
                    .collect::<Vec<_>>(),
                if visible { vec!["target"] } else { vec![] },
                "face point {point:?}"
            );
        }
    }
}
#[test]
fn rotated_offset_full_relative_pose_lexical_order_and_owner_inclusion() {
    let q = DQuat::from_rotation_z(0.7);
    let p = DVec3::new(2.0, -3.0, 4.0);
    let camera = LogicalCameraPose {
        translation: p.to_array(),
        rotation: q.to_array(),
    };
    let target_q = q * DQuat::from_rotation_x(0.4);
    let mut owner = model("owner", (p + q * DVec3::new(2.0, 0.0, 0.0)).to_array());
    owner.pose.rotation = target_q.to_array();
    let objects = [
        model("z", (p + q * DVec3::new(4.0, 0.0, 0.0)).to_array()),
        owner,
    ];
    let out = logical_camera(config(), camera, &objects).unwrap();
    assert_eq!(out.pose, camera);
    assert_eq!(
        out.models
            .iter()
            .map(|m| m.name.as_str())
            .collect::<Vec<_>>(),
        ["owner", "z"]
    );
    for (a, b) in out.models[0]
        .pose
        .translation
        .into_iter()
        .zip([2.0, 0.0, 0.0])
    {
        assert!((a - b).abs() < 1e-14);
    }
    for (a, b) in out.models[0]
        .pose
        .rotation
        .into_iter()
        .zip(DQuat::from_rotation_x(0.4).to_array())
    {
        assert!((a - b).abs() < 1e-14);
    }
    assert!(
        logical_camera(config(), camera, &[])
            .unwrap()
            .models
            .is_empty()
    );
}
#[test]
fn invalid_whole_frame_never_returns_partial_models() {
    for field in 0..4 {
        for v in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -1.0] {
            let mut c = config();
            match field {
                0 => c.near = v,
                1 => c.far = v,
                2 => c.horizontal_fov = v,
                _ => c.aspect_ratio = v,
            };
            assert!(c.validate().is_err());
        }
    }
    for c in [
        LogicalCameraConfig {
            far: 1.0,
            ..config()
        },
        LogicalCameraConfig {
            horizontal_fov: std::f64::consts::PI,
            ..config()
        },
        LogicalCameraConfig {
            near: f64::MIN_POSITIVE,
            far: f64::MIN_POSITIVE * 2.0,
            ..config()
        },
        LogicalCameraConfig {
            far: f64::MAX,
            ..config()
        },
        LogicalCameraConfig {
            aspect_ratio: f64::MIN_POSITIVE,
            ..config()
        },
    ] {
        assert!(c.validate().is_err());
    }
    for bad in [
        LogicalCameraPose {
            rotation: [0.0; 4],
            ..pose([2.0, 0.0, 0.0])
        },
        pose([f64::NAN, 0.0, 0.0]),
        LogicalCameraPose {
            rotation: [0.0, 0.0, 0.0, 2.0],
            ..pose([2.0, 0.0, 0.0])
        },
    ] {
        assert!(logical_camera(config(), bad, &[]).is_err());
        assert!(
            logical_camera(
                config(),
                pose([0.0; 3]),
                &[
                    model("good", [2.0, 0.0, 0.0]),
                    LogicalCameraModel {
                        name: "bad".into(),
                        pose: bad
                    }
                ]
            )
            .is_err()
        );
    }
    for name in ["", "a::b", "bad name", "bad\n"] {
        assert!(logical_camera(config(), pose([0.0; 3]), &[model(name, [2.0, 0.0, 0.0])]).is_err());
    }
    let duplicate = [
        model("same", [2.0, 0.0, 0.0]),
        model("same", [3.0, 0.0, 0.0]),
    ];
    assert!(logical_camera(config(), pose([0.0; 3]), &duplicate).is_err());
    let models: Vec<_> = (0..128)
        .map(|i| model(&format!("m{i:03}"), [2.0, 0.0, 0.0]))
        .collect();
    assert_eq!(
        logical_camera(config(), pose([0.0; 3]), &models)
            .unwrap()
            .models
            .len(),
        128
    );
    let mut models = models;
    models.push(model("overflow", [2.0, 0.0, 0.0]));
    assert!(logical_camera(config(), pose([0.0; 3]), &models).is_err());
}

#[test]
fn captured_native_nominal_points_match_actual_visible_names_without_epsilon() {
    for (name, point, expected) in NATIVE_NOMINAL {
        let input = model(name, point);
        let reading =
            logical_camera(config(), pose([0.0; 3]), std::slice::from_ref(&input)).unwrap();
        assert_eq!(
            reading
                .models
                .iter()
                .map(|m| m.name.as_str())
                .collect::<Vec<_>>(),
            expected,
            "native nominal input {name}: {point:?}"
        );
        if !expected.is_empty() {
            assert_eq!(
                reading.models,
                vec![input],
                "native full relative identity-camera pose"
            );
        }
    }
}
