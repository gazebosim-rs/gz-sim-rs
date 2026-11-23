// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_client::{Client, TrustedWorker};
use gz_render_protocol::{
    Camera, FailureKind, Geometry, PROTOCOL_VERSION, Reply, Request, ShapeVisual, Stamp,
};
use std::time::{Duration, Instant};
fn wait(client: &Client) -> Reply {
    let end = Instant::now() + Duration::from_secs(30);
    loop {
        if let Some(r) = client.try_reply().unwrap() {
            return r;
        }
        assert!(Instant::now() < end, "worker reply deadline");
        std::thread::sleep(Duration::from_millis(2));
    }
}
fn camera() -> Box<Camera> {
    Box::new(Camera {
        width: 64,
        height: 64,
        fx: 32.,
        fy: 32.,
        cx: 32.,
        cy: 32.,
        skew: 0.,
        tx: 0.,
        ty: 0.,
        near: 0.1,
        far: 20.,
        translation: [0.; 3],
        rotation: [0., 0., 0., 1.],
    })
}
fn visual(radius: f64, length: f64) -> ShapeVisual {
    ShapeVisual {
        id: 1,
        translation: [4., 0., 0.],
        rotation: [0., 0., 0., 1.],
        geometry: Geometry::Capsule { radius, length },
        color: [1., 0., 0.],
    }
}
fn start() -> Client {
    let c = Client::start(
        TrustedWorker::from_operator_config(env!("CARGO_BIN_EXE_gz-render-worker").into()).unwrap(),
    )
    .unwrap();
    assert!(matches!(
        wait(&c),
        Reply::Ready {
            version: PROTOCOL_VERSION
        }
    ));
    c
}
#[test]
fn capsule_worker_preserves_actual_rgb_depth_stamps_and_retires_cleanly() {
    let mut client = start();
    let stamp = Stamp {
        epoch: 4,
        seconds: 1,
        nanoseconds: 7,
    };
    client
        .try_send(Request::CalibratedShapeSnapshot {
            request_id: 1,
            stamp,
            origin: [0.; 3],
            visuals: vec![visual(0.75, 2.)],
            camera: camera(),
        })
        .unwrap();
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 1 }
    ));
    client.try_send(Request::Capture { request_id: 2 }).unwrap();
    let Reply::Frame {
        stamp: actual,
        rgba,
        ..
    } = wait(&client)
    else {
        panic!("RGB frame")
    };
    assert_eq!(actual, stamp);
    assert_eq!(rgba.len(), 64 * 64 * 4);
    assert_eq!(
        &rgba[(32 * 64 + 32) * 4..(32 * 64 + 32) * 4 + 4],
        &[255, 0, 0, 255]
    );
    client
        .try_send(Request::CalibratedDepthShapeSnapshot {
            request_id: 3,
            stamp: Stamp {
                nanoseconds: 8,
                ..stamp
            },
            origin: [0.; 3],
            visuals: vec![visual(0.75, 2.)],
            camera: camera(),
        })
        .unwrap();
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 3 }
    ));
    client.try_send(Request::Capture { request_id: 4 }).unwrap();
    let Reply::DepthFrame {
        stamp: actual,
        metres_le,
        ..
    } = wait(&client)
    else {
        panic!("depth frame")
    };
    assert_eq!(actual.nanoseconds, 8);
    assert_eq!(metres_le.len(), 64 * 64 * 4);
    let at = (32 * 64 + 32) * 4;
    let depth = f32::from_le_bytes(metres_le[at..at + 4].try_into().unwrap());
    assert!(depth.is_finite() && (depth - 3.25).abs() < 0.04);
    let lifetime = client.diagnostic_lifetime();
    client.shutdown(Duration::from_secs(10)).unwrap();
    assert!(client.diagnostic_shutdown_complete());
    drop(client);
    assert!(lifetime.is_retired());
}
#[test]
fn capsule_worker_semantic_refusal_keeps_the_previous_capsule_snapshot() {
    let mut client = start();
    let stamp = Stamp {
        epoch: 1,
        seconds: 0,
        nanoseconds: 1,
    };
    client
        .try_send(Request::CalibratedShapeSnapshot {
            request_id: 1,
            stamp,
            origin: [0.; 3],
            visuals: vec![visual(0.75, 2.)],
            camera: camera(),
        })
        .unwrap();
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 1 }
    ));
    client
        .try_send(Request::CalibratedShapeSnapshot {
            request_id: 2,
            stamp: Stamp {
                nanoseconds: 2,
                ..stamp
            },
            origin: [0.; 3],
            visuals: vec![visual(-1., 2.)],
            camera: camera(),
        })
        .unwrap();
    assert!(matches!(
        wait(&client),
        Reply::Error {
            request_id: Some(2),
            kind: FailureKind::InvalidSnapshot,
            ..
        }
    ));
    client.try_send(Request::Capture { request_id: 3 }).unwrap();
    let Reply::Frame {
        stamp: actual,
        rgba,
        ..
    } = wait(&client)
    else {
        panic!("retained frame")
    };
    assert_eq!(actual, stamp);
    assert_eq!(
        &rgba[(32 * 64 + 32) * 4..(32 * 64 + 32) * 4 + 4],
        &[255, 0, 0, 255]
    );
    client.shutdown(Duration::from_secs(10)).unwrap();
}
