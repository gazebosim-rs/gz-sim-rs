// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_protocol::{Camera, Request, Stamp, Visual};

pub fn snapshot(request_id: u64, stamp: Stamp, x: f64) -> Request {
    Request::CalibratedDepthSnapshot {
        request_id,
        stamp,
        origin: [0.0; 3],
        visuals: vec![Visual {
            id: 1,
            translation: [x, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            size: [1.0; 3],
            color: [0.25, 0.5, 0.75],
        }],
        camera: Box::new(Camera {
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
            far: 8.0,
            translation: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }),
    }
}

pub fn assert_box_depth(bytes: &[u8], near_face: f32) {
    assert_eq!(bytes.len(), 64 * 64 * 4);
    let depths = bytes
        .chunks_exact(4)
        .map(|word| f32::from_le_bytes(word.try_into().unwrap()))
        .collect::<Vec<_>>();
    assert!(depths.iter().all(|value| *value > 0.0));
    assert_eq!(depths[0], f32::INFINITY);
    // A planar box face has the same axial distance off the central ray.
    for (row, column) in [(32, 32), (31, 30), (30, 33)] {
        assert!((depths[row * 64 + column] - near_face).abs() < 1e-4);
    }
}

pub fn color_snapshot(request_id: u64, stamp: Stamp, x: f64) -> Request {
    let Request::CalibratedDepthSnapshot {
        request_id,
        stamp,
        origin,
        visuals,
        camera,
    } = snapshot(request_id, stamp, x)
    else {
        unreachable!()
    };
    Request::CalibratedSnapshot {
        request_id,
        stamp,
        origin,
        visuals,
        camera,
    }
}
