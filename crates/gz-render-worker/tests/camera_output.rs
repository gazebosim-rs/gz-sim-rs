// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_camera_output::{CameraOutput, CameraOutputError, Progress};
use gz_msgs::prost::Message;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_sdf_step, reset_loaded_world};
use gz_transport::{Config, Node};
use socket2::{Domain, Protocol, Socket, Type};
use std::{
    net::{Ipv4Addr, SocketAddrV4},
    thread,
    time::{Duration, Instant},
};

fn simulation() -> (Simulation, u64) {
    let source = include_str!("../../gz-sim-scene/tests/fixtures/camera-binding.sdf").replace(
        "</link>",
        "<visual name='box'><pose>5 0 0 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry><material><diffuse>1 0 0 1</diffuse></material></visual></link>",
    );
    let sim = load_world_with_sdf_step(
        &source,
        "camera-output.sdf",
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

fn node() -> (Node, [Socket; 2], Config) {
    node_with_max(16 * 1024 * 1024)
}

fn node_with_max(max_message_bytes: usize) -> (Node, [Socket; 2], Config) {
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
    let config = Config {
        partition: format!("camera-output-{}-{}", std::process::id(), ports[0]),
        message_port: ports[0],
        service_port: ports[1],
        max_message_bytes,
        ..Default::default()
    };
    (Node::new(config.clone()).unwrap(), held, config)
}

fn owner(sim: &Simulation, id: u64, node: &Node) -> CameraOutput {
    CameraOutput::new(
        sim,
        node,
        id,
        [1_000_000_000.0, 0.0, 0.0],
        "camera_frame".into(),
        ("/image", "/info"),
        env!("CARGO_BIN_EXE_gz-render-worker").into(),
    )
    .unwrap()
}

fn await_buffered_frame(output: &mut CameraOutput, sim: &Simulation) {
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut accepted = false;
    let mut submitted = false;
    loop {
        match output.pump(sim).unwrap() {
            Progress::SnapshotAccepted => accepted = true,
            Progress::CaptureSubmitted => {
                assert!(accepted, "capture must follow accepted snapshot");
                submitted = true;
            }
            Progress::FrameReady => {
                assert!(submitted, "frame must follow submitted capture");
                return;
            }
            Progress::Pending | Progress::Backpressure => thread::yield_now(),
            other => panic!("unexpected progress: {other:?}"),
        }
        assert!(Instant::now() < deadline, "frame readback timed out");
    }
}

fn await_submitted_capture(output: &mut CameraOutput, sim: &Simulation) {
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut accepted = false;
    loop {
        match output.pump(sim).unwrap() {
            Progress::SnapshotAccepted => accepted = true,
            Progress::CaptureSubmitted => {
                assert!(accepted, "capture must follow accepted snapshot");
                return;
            }
            Progress::Pending | Progress::Backpressure => thread::yield_now(),
            other => panic!("unexpected progress: {other:?}"),
        }
        assert!(Instant::now() < deadline, "capture submission timed out");
    }
}

#[test]
fn committed_acquisition_stamp_survives_later_step_through_real_worker() {
    let (mut sim, id) = simulation();
    sim.step_exact(1).unwrap();
    let acquired = sim.elapsed();
    let (node, _held, _config) = node();
    let images = node.subscribe("/image", "gz.msgs.Image").unwrap();
    let infos = node.subscribe("/info", "gz.msgs.CameraInfo").unwrap();
    let mut output = owner(&sim, id, &node);
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    assert!(matches!(output.request(&sim), Err(CameraOutputError::Busy)));
    sim.step_exact(1).unwrap();
    assert!(sim.elapsed() > acquired);
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        match output.pump(&sim).unwrap() {
            Progress::ImagePublished => break,
            Progress::Pending
            | Progress::Backpressure
            | Progress::SnapshotAccepted
            | Progress::CaptureSubmitted
            | Progress::FrameReady => thread::yield_now(),
            other => panic!("unexpected progress: {other:?}"),
        }
        assert!(Instant::now() < deadline);
    }
    let image = gz_msgs::msgs::Image::decode(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .data
            .as_slice(),
    )
    .unwrap();
    let info = gz_msgs::msgs::CameraInfo::decode(
        infos
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .data
            .as_slice(),
    )
    .unwrap();
    let image_header = image.header.unwrap();
    let info_header = info.header.unwrap();
    assert_eq!(image_header.stamp, info_header.stamp);
    let stamp = image_header.stamp.unwrap();
    assert_eq!(
        (stamp.sec, stamp.nsec),
        (acquired.as_secs() as i64, acquired.subsec_nanos() as i32)
    );
    assert_eq!((image.width, image.height, image.step), (64, 64, 192));
    assert_eq!(
        &image.data[(32 * 64 + 32) * 3..(32 * 64 + 32) * 3 + 3],
        &[255, 0, 0]
    );
    assert_eq!(image_header.data[0].value, ["camera_frame"]);
    assert_eq!(image_header.data[1].value, ["0"]);
    drop(images);
    drop(infos);
    assert_eq!(output.request(&sim).unwrap(), Progress::RetireRequired);
    output.retire();
    let images = node.subscribe("/image", "gz.msgs.Image").unwrap();
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        match output.pump(&sim).unwrap() {
            Progress::ImagePublished => break,
            Progress::Pending
            | Progress::Backpressure
            | Progress::SnapshotAccepted
            | Progress::CaptureSubmitted
            | Progress::FrameReady => thread::yield_now(),
            other => panic!("unexpected restart progress: {other:?}"),
        }
        assert!(Instant::now() < deadline);
    }
    let second = gz_msgs::msgs::Image::decode(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .data
            .as_slice(),
    )
    .unwrap();
    assert_eq!(second.header.unwrap().data[1].value, ["1"]);
    let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
    let mut sensor = sim
        .world_mut()
        .get_mut::<gz_sim_scene::CameraSensor>(entity)
        .unwrap();
    sensor.0 = std::sync::Arc::new((*sensor.0).clone());
    assert_eq!(output.pump(&sim).unwrap(), Progress::RetireRequired);
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    output.retire();
}

#[test]
fn reset_invalidates_pending_acquisition_before_publication() {
    let (mut sim, id) = simulation();
    let (node, _held, _config) = node();
    let images = node.subscribe("/image", "gz.msgs.Image").unwrap();
    let mut output = owner(&sim, id, &node);
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    await_buffered_frame(&mut output, &sim);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(output.pump(&sim).unwrap(), Progress::RetireRequired);
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    output.retire();
    assert!(images.recv_timeout(Duration::from_millis(100)).is_err());
    assert!(matches!(
        CameraOutput::new(
            &sim,
            &node,
            id,
            [1_000_000_000.0, 0.0, 0.0],
            "camera_frame".into(),
            ("/image", "/info"),
            env!("CARGO_BIN_EXE_gz-render-worker").into()
        ),
        Err(CameraOutputError::Transport(_))
    ));
    drop(output);
    let _replacement = owner(&sim, id, &node);
}

#[test]
fn reset_after_capture_submission_discards_any_later_frame() {
    let (mut sim, id) = simulation();
    let (node, _held, _config) = node();
    let images = node.subscribe("/image", "gz.msgs.Image").unwrap();
    let mut output = owner(&sim, id, &node);
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    await_submitted_capture(&mut output, &sim);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(output.pump(&sim).unwrap(), Progress::RetireRequired);
    output.retire();
    assert!(images.recv_timeout(Duration::from_millis(100)).is_err());
}

struct NativePeer(Option<std::process::Child>);
impl Drop for NativePeer {
    fn drop(&mut self) {
        if let Some(mut child) = self.0.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

#[test]
fn native_subscriber_receives_actual_sdf_camera_frame_and_info() {
    use std::{
        path::PathBuf,
        process::{Command, Stdio},
    };
    let prefix = PathBuf::from(
        std::env::var_os("GZ_REFERENCE_PREFIX").expect("native reference prefix required"),
    );
    let pkg = Command::new("pkg-config")
        .args(["--cflags", "--libs", "gz-transport"])
        .env("PKG_CONFIG_PATH", prefix.join("lib/pkgconfig"))
        .output()
        .unwrap();
    assert!(
        pkg.status.success(),
        "{}",
        String::from_utf8_lossy(&pkg.stderr)
    );
    let directory = tempfile::tempdir().unwrap();
    let executable = directory.path().join("camera-subscriber");
    let flags = std::str::from_utf8(&pkg.stdout).unwrap();
    let compiled = Command::new("c++")
        .args(["-std=c++17", "-Wall", "-Wextra", "-Werror"])
        .arg(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/camera_subscriber.cc"
        ))
        .args(flags.split_whitespace())
        .arg(format!("-Wl,-rpath,{}", prefix.join("lib").display()))
        .arg("-o")
        .arg(&executable)
        .output()
        .unwrap();
    assert!(
        compiled.status.success(),
        "{}",
        String::from_utf8_lossy(&compiled.stderr)
    );
    let (mut sim, id) = simulation();
    sim.step_exact(1).unwrap();
    let acquired = sim.elapsed();
    let (node, _held, config) = node();
    let mut output = owner(&sim, id, &node);
    let child = Command::new(executable)
        .env("GZ_PARTITION", &config.partition)
        .env("GZ_IP", "127.0.0.1")
        .env("GZ_DISCOVERY_MSG_PORT", config.message_port.to_string())
        .env("GZ_DISCOVERY_SRV_PORT", config.service_port.to_string())
        .env("GZ_DISCOVERY_MULTICAST_IP", config.multicast.to_string())
        .env("GZ_TRANSPORT_IMPLEMENTATION", "zeromq")
        .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
        .env_remove("GZ_TRANSPORT_USERNAME")
        .env_remove("GZ_TRANSPORT_PASSWORD")
        .env_remove("GZ_RELAY")
        .env("LD_LIBRARY_PATH", prefix.join("lib"))
        .env_remove("LD_PRELOAD")
        .env_remove("LD_AUDIT")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut peer = NativePeer(Some(child));
    let deadline = Instant::now() + Duration::from_secs(20);
    while output.demand().unwrap() != (true, true) {
        assert!(
            peer.0.as_mut().unwrap().try_wait().unwrap().is_none(),
            "native subscriber exited early"
        );
        assert!(Instant::now() < deadline, "native discovery timeout");
        thread::sleep(Duration::from_millis(10));
    }
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    sim.step_exact(1).unwrap();
    loop {
        match output.pump(&sim).unwrap() {
            Progress::ImagePublished => break,
            Progress::Pending
            | Progress::Backpressure
            | Progress::SnapshotAccepted
            | Progress::CaptureSubmitted
            | Progress::FrameReady => thread::yield_now(),
            other => panic!("unexpected progress: {other:?}"),
        }
        assert!(Instant::now() < deadline, "renderer timeout");
    }
    while peer.0.as_mut().unwrap().try_wait().unwrap().is_none() {
        assert!(Instant::now() < deadline, "native subscriber timeout");
        thread::sleep(Duration::from_millis(10));
    }
    let result = peer.0.take().unwrap().wait_with_output().unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert_eq!(
        String::from_utf8(result.stdout).unwrap(),
        format!(
            "ready\n{} {}\n",
            acquired.as_secs(),
            acquired.subsec_nanos()
        )
    );
    output.retire();
}

#[test]
fn removed_camera_fault_and_lost_demand_retire_pending_worker() {
    for case in ["removed", "replacement", "fault", "demand"] {
        let (mut sim, id) = simulation();
        let (node, _held, _config) = node();
        let mut images = Some(node.subscribe("/image", "gz.msgs.Image").unwrap());
        let mut output = owner(&sim, id, &node);
        assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
        await_buffered_frame(&mut output, &sim);
        match case {
            "removed" => {
                let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
                assert!(sim.world_mut().despawn(entity));
            }
            "replacement" => {
                let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
                let mut sensor = sim
                    .world_mut()
                    .get_mut::<gz_sim_scene::CameraSensor>(entity)
                    .unwrap();
                sensor.0 = std::sync::Arc::new((*sensor.0).clone());
            }
            "fault" => sim
                .world()
                .resource::<gz_sim_core::StepFault>()
                .report(std::io::Error::other("fixture fault")),
            "demand" => drop(images.take()),
            _ => unreachable!(),
        }
        assert_eq!(
            output.pump(&sim).unwrap(),
            Progress::RetireRequired,
            "{case}"
        );
        match case {
            "demand" => assert!(matches!(
                output.request(&sim),
                Err(CameraOutputError::RetireRequired)
            )),
            _ => assert!(matches!(
                output.request(&sim),
                Err(CameraOutputError::OwnerInvalid)
            )),
        }
        output.retire();
        if let Some(images) = images {
            assert!(
                images.recv_timeout(Duration::from_millis(100)).is_err(),
                "{case}"
            );
        }
    }
}

#[test]
fn oversized_real_frame_is_reported_without_publication() {
    let (sim, id) = simulation();
    let (node, _held, _config) = node_with_max(1024);
    let images = node.subscribe("/image", "gz.msgs.Image").unwrap();
    let mut output = owner(&sim, id, &node);
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    await_buffered_frame(&mut output, &sim);
    assert!(matches!(
        output.pump(&sim),
        Err(CameraOutputError::Transport(
            gz_transport::Error::InvalidFrame
        ))
    ));
    assert!(images.recv_timeout(Duration::from_millis(100)).is_err());
    output.retire();
}
