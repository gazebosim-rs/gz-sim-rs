// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_camera_output::{
    CameraOutput, CameraOutputConfig, CameraOutputError, CameraWriterScope, Progress,
};
use gz_render_protocol::FailureKind;
use gz_sim_core::EntityId;
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_sdf_step};
use std::{
    num::NonZeroUsize,
    sync::mpsc,
    thread,
    time::{Duration, Instant},
};

use zenoh::Wait;

fn session() -> zenoh::Session {
    let mut config = zenoh::Config::default();
    for (path, value) in [
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(path, value).unwrap();
    }
    zenoh::open(config).wait().unwrap()
}

fn image_observer(session: &zenoh::Session) -> (zenoh::pubsub::Subscriber<()>, mpsc::Receiver<()>) {
    let (published, image) = mpsc::sync_channel(1);
    let subscriber = session
        .declare_subscriber("@/camera-fault@/image")
        .allowed_origin(zenoh::sample::Locality::SessionLocal)
        .callback(move |_: zenoh::sample::Sample| {
            let _ = published.try_send(());
        })
        .wait()
        .unwrap();
    (subscriber, image)
}

#[test]
fn real_client_pipes_propagate_typed_worker_failures_and_retire() {
    let source = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../crates/gz-sim-scene/tests/fixtures/camera-binding.sdf"
    ));
    for mode in 1..=7 {
        let sim = load_world_with_sdf_step(
            source,
            "camera-binding.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Tpe,
        )
        .unwrap();
        let id = sim
            .world()
            .iter_entities()
            .find_map(|entity| {
                (entity.get::<Name>()?.0 == "default::robot::body::camera")
                    .then(|| entity.get::<EntityId>().unwrap().get())
            })
            .unwrap();
        let session = session();
        let scope = CameraWriterScope::new(NonZeroUsize::new(1).unwrap());
        let (_subscription, image) = image_observer(&session);
        let mut owner = CameraOutput::new(
            &sim,
            &session,
            &scope,
            id,
            CameraOutputConfig {
                origin: [1_000_000_000.0, f64::from(mode), 0.0],
                frame_id: "frame".into(),
                image_key: "@/camera-fault@/image".into(),
                info_key: "@/camera-fault@/info".into(),
                worker_path: env!("CARGO_BIN_EXE_camera_fault_fixture").into(),
                max_message_bytes: NonZeroUsize::new(16 * 1024 * 1024).unwrap(),
            },
        )
        .unwrap();
        assert_eq!(owner.request(&sim).unwrap(), Progress::Pending);
        let deadline = Instant::now() + Duration::from_secs(10);
        let error = loop {
            match owner.pump(&sim) {
                Ok(
                    Progress::Pending
                    | Progress::Backpressure
                    | Progress::SnapshotAccepted
                    | Progress::CaptureSubmitted,
                ) => thread::yield_now(),
                Err(error) => break error,
                other => panic!("unexpected progress in {mode}: {other:?}"),
            }
            assert!(Instant::now() < deadline, "peer timed out for {mode}");
        };
        match mode {
            1 | 3 => assert!(matches!(error, CameraOutputError::Busy)),
            2 => assert!(matches!(
                error,
                CameraOutputError::Worker {
                    kind: FailureKind::InvalidSnapshot,
                    request_id: Some(1),
                    ..
                }
            )),
            4 => assert!(matches!(
                error,
                CameraOutputError::Worker {
                    kind: FailureKind::Device,
                    request_id: Some(2),
                    ..
                }
            )),
            5 => assert!(matches!(
                error,
                CameraOutputError::Worker {
                    kind: FailureKind::Shader,
                    request_id: None,
                    ..
                }
            )),
            6 => assert!(matches!(error, CameraOutputError::Client(_))),
            7 => assert!(matches!(error, CameraOutputError::Protocol)),
            _ => unreachable!(),
        }
        assert!(matches!(
            owner.request(&sim),
            Err(CameraOutputError::RetireRequired)
        ));
        owner.retire();
        assert!(image.recv_timeout(Duration::from_millis(20)).is_err());
    }
}

#[test]
fn startup_failure_and_protocol_version_error_reach_the_output_owner() {
    let source = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../crates/gz-sim-scene/tests/fixtures/camera-binding.sdf"
    ));
    for (path, startup) in [
        (env!("CARGO_BIN_EXE_startup_failure_fixture"), true),
        (env!("CARGO_BIN_EXE_bad_version_fixture"), false),
    ] {
        let sim = load_world_with_sdf_step(
            source,
            "camera-binding.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Tpe,
        )
        .unwrap();
        let id = sim
            .world()
            .iter_entities()
            .find_map(|entity| {
                (entity.get::<Name>()?.0 == "default::robot::body::camera")
                    .then(|| entity.get::<EntityId>().unwrap().get())
            })
            .unwrap();
        let session = session();
        let scope = CameraWriterScope::new(NonZeroUsize::new(1).unwrap());
        let (_subscription, image) = image_observer(&session);
        let mut owner = CameraOutput::new(
            &sim,
            &session,
            &scope,
            id,
            CameraOutputConfig {
                origin: [1_000_000_000.0, 0.0, 0.0],
                frame_id: "frame".into(),
                image_key: "@/camera-fault@/image".into(),
                info_key: "@/camera-fault@/info".into(),
                worker_path: path.into(),
                max_message_bytes: NonZeroUsize::new(16 * 1024 * 1024).unwrap(),
            },
        )
        .unwrap();
        assert_eq!(owner.request(&sim).unwrap(), Progress::Pending);
        let deadline = Instant::now() + Duration::from_secs(10);
        let error = loop {
            match owner.pump(&sim) {
                Ok(Progress::Pending | Progress::Backpressure) => thread::yield_now(),
                Err(error) => break error,
                other => panic!("unexpected startup result: {other:?}"),
            }
            assert!(Instant::now() < deadline);
        };
        if startup {
            assert!(matches!(
                error,
                CameraOutputError::Worker {
                    kind: FailureKind::Startup,
                    request_id: None,
                    ..
                }
            ));
        } else {
            assert!(matches!(
                error,
                CameraOutputError::Client(gz_render_client::ClientError::ProtocolVersion)
            ));
        }
        assert!(matches!(
            owner.request(&sim),
            Err(CameraOutputError::RetireRequired)
        ));
        owner.retire();
        assert!(image.recv_timeout(Duration::from_millis(20)).is_err());
    }
}
