// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_msgs::prost::Message;
use gz_sim_core::EntityId;
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_sdf_step};
use socket2::{Domain, Protocol, Socket, Type};
use std::net::{Ipv4Addr, SocketAddrV4};

fn fixture() -> (Simulation, u64) {
    let source = include_str!("../../../gz-sim-scene/tests/fixtures/camera-binding.sdf");
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
    (sim, id)
}

fn node() -> (Node, [Socket; 2]) {
    node_with_max(16 * 1024 * 1024)
}

fn node_with_max(max_message_bytes: usize) -> (Node, [Socket; 2]) {
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
    let ports = held.each_ref().map(|socket| {
        socket
            .local_addr()
            .unwrap()
            .as_socket_ipv4()
            .unwrap()
            .port()
    });
    assert_ne!(ports[0], ports[1]);
    let config = gz_transport::Config {
        partition: format!("camera-pump-{}-{}", std::process::id(), ports[0]),
        message_port: ports[0],
        service_port: ports[1],
        max_message_bytes,
        ..Default::default()
    };
    (Node::new(config).unwrap(), held)
}

#[test]
fn no_demand_and_info_only_never_start_a_worker() {
    let (sim, id) = fixture();
    let systems = sim.scheduled_system_counts();
    let (node, _held) = node();
    let mut output = CameraOutput::new(
        &sim,
        &node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "frame".into(),
        ("/image", "/info"),
        "/nonexistent/worker".into(),
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::NoDemand);
    assert!(output.client.is_none());
    let subscription = node.subscribe("/info", "gz.msgs.CameraInfo").unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::InfoPublished);
    let packet = subscription.recv_timeout(Duration::from_secs(5)).unwrap();
    let info = gz_msgs::msgs::CameraInfo::decode(packet.data.as_slice()).unwrap();
    assert_eq!((info.width, info.height), (64, 64));
    assert_eq!(info.header.unwrap().stamp.unwrap().sec, 0);
    assert!(output.client.is_none());
    assert_eq!(systems, sim.scheduled_system_counts());
}

#[test]
fn invalid_camera_or_worker_configuration_precedes_advertisement() {
    let (sim, id) = fixture();
    let (node, _held) = node();
    assert!(matches!(
        CameraOutput::new(
            &sim,
            &node,
            id + 100,
            [0.0; 3],
            "frame".into(),
            ("/image", "/info"),
            "/nonexistent/worker".into()
        ),
        Err(CameraOutputError::Snapshot(_))
    ));
    assert!(matches!(
        CameraOutput::new(
            &sim,
            &node,
            id,
            [1_000_000_000.0, 0.0, 0.0],
            "".into(),
            ("/image", "/info"),
            "/nonexistent/worker".into()
        ),
        Err(CameraOutputError::InvalidFrameId)
    ));
    assert!(matches!(
        CameraOutput::new(
            &sim,
            &node,
            id,
            [1_000_000_000.0, 0.0, 0.0],
            "frame".into(),
            ("/image", "/info"),
            "relative-worker".into()
        ),
        Err(CameraOutputError::Client(ClientError::InvalidWorkerPath))
    ));
    // Both topics remain available after failed construction.
    let _output = CameraOutput::new(
        &sim,
        &node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "frame".into(),
        ("/image", "/info"),
        "/nonexistent/worker".into(),
    )
    .unwrap();
}

#[test]
fn same_entity_metadata_replacement_invalidates_inflight_acquisition() {
    let (mut sim, id) = fixture();
    let (node, _held) = node();
    let _image = node.subscribe("/image", "gz.msgs.Image").unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "frame".into(),
        ("/image", "/info"),
        "/bin/false".into(),
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    let entity = entity_by_id(sim.world(), id).unwrap();
    let mut sensor = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
    sensor.0 = std::sync::Arc::new((*sensor.0).clone());
    assert_eq!(output.pump(&sim).unwrap(), Progress::RetireRequired);
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    output.retire();
}

#[test]
fn actual_worker_exit_is_reported_and_requires_retirement() {
    let (sim, id) = fixture();
    let (node, _held) = node();
    let _image = node.subscribe("/image", "gz.msgs.Image").unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "frame".into(),
        ("/image", "/info"),
        "/bin/false".into(),
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    let deadline = std::time::Instant::now() + Duration::from_secs(10);
    loop {
        match output.pump(&sim) {
            Ok(Progress::Backpressure | Progress::Pending) => std::thread::yield_now(),
            Err(CameraOutputError::Client(_) | CameraOutputError::Protocol) => break,
            other => panic!("unexpected worker result: {other:?}"),
        }
        assert!(
            std::time::Instant::now() < deadline,
            "worker exit not observed"
        );
    }
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::RetireRequired)
    ));
    output.retire();
}

#[test]
fn idle_replacement_requires_old_owner_drop_before_reconstruction() {
    let (mut sim, id) = fixture();
    let (node, _held) = node();
    let make = |sim: &Simulation| {
        CameraOutput::new(
            sim,
            &node,
            id,
            [1_000_000_000.0, 0.0, 0.0],
            "frame".into(),
            ("/image", "/info"),
            "/bin/false".into(),
        )
    };
    let mut output = make(&sim).unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::NoDemand);
    let info = node.subscribe("/info", "gz.msgs.CameraInfo").unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::InfoPublished);
    info.recv_timeout(Duration::from_secs(5)).unwrap();
    let entity = entity_by_id(sim.world(), id).unwrap();
    let mut sensor = sim.world_mut().get_mut::<CameraSensor>(entity).unwrap();
    sensor.0 = std::sync::Arc::new((*sensor.0).clone());
    assert!(matches!(
        output.pump(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    output.retire();
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    assert!(matches!(make(&sim), Err(CameraOutputError::Transport(_))));
    drop(output);
    let _replacement = make(&sim).unwrap();
}

#[test]
fn same_name_and_restored_id_cannot_rebind_an_existing_idle_owner() {
    use gz_sim_scene::{EntityKind, Name, WorldPose};
    let (mut sim, id) = fixture();
    let (node, _held) = node();
    let mut output = CameraOutput::new(
        &sim,
        &node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "frame".into(),
        ("/image", "/info"),
        "/bin/false".into(),
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::NoDemand);
    let entity = entity_by_id(sim.world(), id).unwrap();
    let reservation = gz_sim_core::reserve_id(sim.world(), entity).unwrap();
    let name = sim.world().get::<Name>(entity).unwrap().0.clone();
    let pose = *sim.world().get::<WorldPose>(entity).unwrap();
    let sensor = sim.world().get::<CameraSensor>(entity).unwrap().clone();
    assert!(sim.world_mut().despawn(entity));
    let replacement = sim
        .world_mut()
        .spawn((Name(name), EntityKind::Sensor, pose, sensor))
        .id();
    assert_ne!(entity, replacement);
    assert_eq!(
        gz_sim_core::restore_id(sim.world_mut(), replacement, &reservation).unwrap(),
        id
    );
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    output.retire();
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    drop(output);
    let _replacement = CameraOutput::new(
        &sim,
        &node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "frame".into(),
        ("/image", "/info"),
        "/bin/false".into(),
    )
    .unwrap();
}

#[test]
fn explicit_retirement_joins_idle_worker_before_returning() {
    let (sim, id) = fixture();
    let (node, _held) = node();
    let _image = node.subscribe("/image", "gz.msgs.Image").unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "frame".into(),
        ("/image", "/info"),
        "/bin/cat".into(),
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    let lifetime = output.client.as_ref().unwrap().diagnostic_lifetime();
    assert!(!lifetime.is_retired());
    // A real child that never sends a protocol greeting cannot accept work.
    assert_eq!(output.pump(&sim).unwrap(), Progress::Backpressure);
    output.retire();
    assert!(lifetime.is_retired());
    assert!(output.client.is_none());
}

#[test]
fn idle_pump_and_demand_loss_require_explicit_worker_retirement() {
    let (sim, id) = fixture();
    let (node, _held) = node();
    let image = node.subscribe("/image", "gz.msgs.Image").unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "frame".into(),
        ("/image", "/info"),
        "/bin/cat".into(),
    )
    .unwrap();
    assert_eq!(output.pump(&sim).unwrap(), Progress::NoDemand);
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    let lifetime = output.client.as_ref().unwrap().diagnostic_lifetime();
    drop(image);
    assert_eq!(output.pump(&sim).unwrap(), Progress::RetireRequired);
    assert_eq!(output.pump(&sim).unwrap(), Progress::RetireRequired);
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::RetireRequired)
    ));
    output.retire();
    assert!(lifetime.is_retired());
    assert_eq!(output.pump(&sim).unwrap(), Progress::NoDemand);
}

#[test]
fn exhausted_request_ids_do_not_start_worker_or_publish_info() {
    let (sim, id) = fixture();
    let (node, _held) = node();
    let info = node.subscribe("/info", "gz.msgs.CameraInfo").unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "frame".into(),
        ("/image", "/info"),
        "/bin/cat".into(),
    )
    .unwrap();
    output.next_id = u64::MAX - 1;
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::IdExhausted)
    ));
    output.next_id = u64::MAX;
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::IdExhausted)
    ));
    assert!(output.client.is_none());
    assert!(info.recv_timeout(Duration::from_millis(20)).is_err());
}

#[test]
fn unrepresentable_acquisition_time_cannot_publish_or_start_worker() {
    use gz_sim_core::assign_id;
    use gz_sim_scene::{EntityKind, WorldPose};
    use sdformat::{Camera, Sensor, SensorConfig};
    let mut sim = Simulation::new(Duration::from_secs(i64::MAX as u64 + 1)).unwrap();
    let mut camera = Camera {
        image_width: 64,
        image_height: 64,
        horizontal_fov: std::f64::consts::FRAC_PI_2,
        ..Default::default()
    };
    camera.lens.has_intrinsics = false;
    camera.lens.has_projection = false;
    let sensor = Sensor {
        kind: "camera".into(),
        config: SensorConfig::Camera(Box::new(camera)),
        ..Default::default()
    };
    let entity = sim
        .world_mut()
        .spawn((
            EntityKind::Sensor,
            CameraSensor(std::sync::Arc::new(sensor)),
            WorldPose {
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        ))
        .id();
    let id = assign_id(sim.world_mut(), entity).unwrap();
    let (node, _held) = node();
    let info = node.subscribe("/info", "gz.msgs.CameraInfo").unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &node,
        id,
        [0.0; 3],
        "frame".into(),
        ("/image", "/info"),
        "/bin/cat".into(),
    )
    .unwrap();
    sim.step_exact(1).unwrap();
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::Message(CameraMessageError::TimeRange))
    ));
    assert!(output.client.is_none());
    assert!(info.recv_timeout(Duration::from_millis(20)).is_err());
}

#[test]
fn stopped_transport_is_reported_before_and_during_an_acquisition() {
    let (sim, id) = fixture();
    let (mut node, _held) = node();
    let image = node.subscribe("/image", "gz.msgs.Image").unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "frame".into(),
        ("/image", "/info"),
        "/bin/cat".into(),
    )
    .unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    node.cancel();
    assert!(matches!(
        output.demand(),
        Err(CameraOutputError::Transport(gz_transport::Error::Shutdown))
    ));
    assert!(matches!(
        output.pump(&sim),
        Err(CameraOutputError::Transport(gz_transport::Error::Shutdown))
    ));
    output.retire();
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::Transport(gz_transport::Error::Shutdown))
    ));
    drop(image);
    node.shutdown();
}

#[test]
fn duplicate_topics_and_oversized_info_fail_without_starting_a_worker() {
    let (sim, id) = fixture();
    let (node, _held) = node();
    assert!(matches!(
        CameraOutput::new(
            &sim,
            &node,
            id,
            [1_000_000_000.0, 0.0, 0.0],
            "frame".into(),
            ("/same", "/same"),
            "/bin/cat".into()
        ),
        Err(CameraOutputError::Transport(
            gz_transport::Error::AlreadyAdvertised
        ))
    ));
    let _replacement = node
        .advertise_typed::<gz_msgs::msgs::Image>("/same")
        .unwrap();
    let (node, _held) = node_with_max(16);
    let info = node.subscribe("/info", "gz.msgs.CameraInfo").unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "frame".into(),
        ("/image", "/info"),
        "/bin/cat".into(),
    )
    .unwrap();
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::Transport(
            gz_transport::Error::InvalidFrame
        ))
    ));
    assert!(output.client.is_none());
    assert!(info.recv_timeout(Duration::from_millis(20)).is_err());
}

#[test]
fn scene_mutation_after_construction_fails_before_publication_or_worker_start() {
    let (mut sim, id) = fixture();
    let (node, _held) = node();
    let info = node.subscribe("/info", "gz.msgs.CameraInfo").unwrap();
    let mut output = CameraOutput::new(
        &sim,
        &node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "frame".into(),
        ("/image", "/info"),
        "/bin/cat".into(),
    )
    .unwrap();
    let visual = sim.world_mut().spawn(gz_sim_scene::EntityKind::Visual).id();
    let visual_id = gz_sim_core::assign_id(sim.world_mut(), visual).unwrap();
    assert!(
        matches!(output.request(&sim), Err(CameraOutputError::Snapshot(SnapshotError::MissingVisualData(actual))) if actual == visual_id)
    );
    assert!(output.client.is_none());
    assert!(info.recv_timeout(Duration::from_millis(20)).is_err());
}

#[test]
fn isolated_supervisor_thread_exhaustion_is_reported_before_acquisition() {
    const CHILD: &str = "GZ_CAMERA_THREAD_LIMIT_CASE";
    if std::env::var_os(CHILD).is_some() {
        use rustix::process::{Resource, Rlimit, getrlimit, setrlimit};
        let (sim, id) = fixture();
        let (node, _held) = node();
        let image = node.subscribe("/image", "gz.msgs.Image").unwrap();
        let mut output = CameraOutput::new(
            &sim,
            &node,
            id,
            [1_000_000_000.0, 0.0, 0.0],
            "frame".into(),
            ("/image", "/info"),
            "/bin/cat".into(),
        )
        .unwrap();
        let limit = getrlimit(Resource::Nproc);
        setrlimit(
            Resource::Nproc,
            Rlimit {
                current: Some(1),
                maximum: limit.maximum,
            },
        )
        .unwrap();
        assert!(
            matches!(output.request(&sim), Err(CameraOutputError::Client(ClientError::Io(error))) if error.raw_os_error() == Some(11))
        );
        assert!(output.client.is_none());
        assert!(output.acquisition.is_none());
        assert!(image.recv_timeout(Duration::from_millis(20)).is_err());
        return;
    }
    let result = std::process::Command::new("timeout")
        .args(["--signal=KILL", "20s", "unshare", "-Ur"])
        .arg(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "tests::isolated_supervisor_thread_exhaustion_is_reported_before_acquisition",
            "--nocapture",
        ])
        .env(CHILD, "1")
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert!(String::from_utf8_lossy(&result.stdout).contains("1 passed; 0 failed"));
}
