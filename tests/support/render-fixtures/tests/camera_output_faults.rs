// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_camera_output::{CameraOutput, CameraOutputError, Progress};
use gz_render_protocol::FailureKind;
use gz_sim_core::EntityId;
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_sdf_step};
use gz_transport::{Config, Node};
use socket2::{Domain, Protocol, Socket, Type};
use std::{
    net::{Ipv4Addr, SocketAddrV4},
    thread,
    time::{Duration, Instant},
};

fn node() -> (Node, [Socket; 2]) {
    let reserve = || {
        let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP)).unwrap();
        socket.set_reuse_address(true).unwrap();
        #[cfg(target_os = "linux")]
        socket.set_multicast_all_v4(false).unwrap();
        socket
            .bind(&SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0).into())
            .unwrap();
        socket
    };
    let held = [reserve(), reserve()];
    let ports = held
        .each_ref()
        .map(|s| s.local_addr().unwrap().as_socket_ipv4().unwrap().port());
    assert_ne!(ports[0], ports[1]);
    let config = Config {
        partition: format!("camera-fault-{}-{}", std::process::id(), ports[0]),
        message_port: ports[0],
        service_port: ports[1],
        ..Default::default()
    };
    (Node::new(config).unwrap(), held)
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
        let (node, _held) = node();
        let image = node.subscribe("/image", "gz.msgs.Image").unwrap();
        let mut owner = CameraOutput::new(
            &sim,
            &node,
            id,
            [1_000_000_000.0, f64::from(mode), 0.0],
            "frame".into(),
            ("/image", "/info"),
            env!("CARGO_BIN_EXE_camera_fault_fixture").into(),
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
        let (node, _held) = node();
        let image = node.subscribe("/image", "gz.msgs.Image").unwrap();
        let mut owner = CameraOutput::new(
            &sim,
            &node,
            id,
            [1_000_000_000.0, 0.0, 0.0],
            "frame".into(),
            ("/image", "/info"),
            path.into(),
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
