// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_camera_output::{
    CameraOutput, CameraOutputConfig, CameraOutputError, CameraWriterScope, Progress,
};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_proto::prost::Message;
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_sdf_step, reset_loaded_world};
use std::{
    num::NonZeroUsize,
    thread,
    time::{Duration, Instant},
};
use zenoh::Wait;

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

fn session() -> (zenoh::Session, CameraWriterScope, NonZeroUsize) {
    session_with_max(16 * 1024 * 1024)
}

fn session_with_max(max_message_bytes: usize) -> (zenoh::Session, CameraWriterScope, NonZeroUsize) {
    session_at(max_message_bytes, "tcp/127.0.0.1:0", false)
}

fn session_at(
    max_message_bytes: usize,
    endpoint: &str,
    connect: bool,
) -> (zenoh::Session, CameraWriterScope, NonZeroUsize) {
    let mut config = zenoh::Config::default();
    let endpoints = format!("[\"{endpoint}\"]");
    for (path, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", if connect { "[]" } else { &endpoints }),
        ("connect/endpoints", if connect { &endpoints } else { "[]" }),
        ("connect/timeout_ms", "2000"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(path, value).unwrap();
    }
    (
        zenoh::open(config).wait().unwrap(),
        CameraWriterScope::new(NonZeroUsize::new(4).unwrap()),
        NonZeroUsize::new(max_message_bytes).unwrap(),
    )
}

fn owner(
    sim: &Simulation,
    id: u64,
    session: &zenoh::Session,
    scope: &CameraWriterScope,
    limit: NonZeroUsize,
) -> CameraOutput {
    CameraOutput::new(
        sim,
        session,
        scope,
        id,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "camera_frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: env!("CARGO_BIN_EXE_gz-render-worker").into(),
            max_message_bytes: limit,
        },
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
    let (session, scope, limit) = session();
    let images = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    let infos = session.declare_subscriber("@/camera@/info").wait().unwrap();
    let mut output = owner(&sim, id, &session, &scope, limit);
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
    let image = gz_sim_proto::msgs::Image::decode(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    let info = gz_sim_proto::msgs::CameraInfo::decode(
        infos
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
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
    let images = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
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
    let second = gz_sim_proto::msgs::Image::decode(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
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

fn scheduled_owner(
    sim: &Simulation,
    session: &zenoh::Session,
    scope: &CameraWriterScope,
    limit: NonZeroUsize,
) -> gz_camera_output::ScheduledCameraOutput {
    gz_camera_output::ScheduledCameraOutput::prepare(
        sim,
        session,
        scope,
        CameraOutputConfig {
            origin: [1_000_000_000.0, 0.0, 0.0],
            frame_id: "camera_frame".into(),
            image_key: "@/camera@/image".into(),
            info_key: "@/camera@/info".into(),
            worker_path: env!("CARGO_BIN_EXE_gz-render-worker").into(),
            max_message_bytes: limit,
        },
    )
    .unwrap()
    .unwrap()
}

fn scheduled_frame(
    output: &mut gz_camera_output::ScheduledCameraOutput,
    sim: &Simulation,
    publish: bool,
) {
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        match output.pump(sim).unwrap() {
            Progress::FrameReady if !publish => return,
            Progress::ImagePublished if publish => return,
            Progress::Pending
            | Progress::Backpressure
            | Progress::SnapshotAccepted
            | Progress::CaptureSubmitted
            | Progress::FrameReady => thread::yield_now(),
            other => panic!("unexpected scheduled progress: {other:?}"),
        }
        assert!(
            Instant::now() < deadline,
            "scheduled frame readback timed out"
        );
    }
}

#[test]
fn scheduled_camera_committed_cadence_preserves_bounded_busy_and_wire_stamps() {
    let (mut sim, id) = simulation();
    let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
    std::sync::Arc::make_mut(
        &mut sim
            .world_mut()
            .get_mut::<gz_sim_scene::CameraSensor>(entity)
            .unwrap()
            .0,
    )
    .update_rate = 100.0;
    let (session, scope, limit) = session();
    let images = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    let infos = session.declare_subscriber("@/camera@/info").wait().unwrap();
    let mut output = scheduled_owner(&sim, &session, &scope, limit);
    assert_eq!(output.request_due(&sim).unwrap(), None);
    sim.step_exact(1).unwrap();
    assert_eq!(output.request_due(&sim).unwrap(), Some(Progress::Pending));
    assert_eq!(output.request_due(&sim).unwrap(), None);
    sim.step_exact(9).unwrap();
    assert!(matches!(
        output.request_due(&sim),
        Err(CameraOutputError::Busy)
    ));
    // Finish the first captured frame after physics advanced. Its original
    // acquisition stamp survives; the busy call did not consume the 10ms slot.
    scheduled_frame(&mut output, &sim, true);
    for (millis, sequence) in [(1, 0), (10, 1)] {
        if sequence == 1 {
            assert_eq!(output.request_due(&sim).unwrap(), Some(Progress::Pending));
            scheduled_frame(&mut output, &sim, true);
        }
        let image = gz_sim_proto::msgs::Image::decode(
            images
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .unwrap()
                .payload()
                .to_bytes()
                .as_ref(),
        )
        .unwrap();
        let info = gz_sim_proto::msgs::CameraInfo::decode(
            infos
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .unwrap()
                .payload()
                .to_bytes()
                .as_ref(),
        )
        .unwrap();
        let header = image.header.as_ref().unwrap();
        assert_eq!(header.stamp, info.header.unwrap().stamp);
        let stamp = header.stamp.as_ref().unwrap();
        assert_eq!((stamp.sec, stamp.nsec), (0, millis * 1_000_000));
        assert_eq!(header.data[0].value, ["camera_frame"]);
        assert_eq!(header.data[1].key, "seq");
        assert_eq!(header.data[1].value, [sequence.to_string()]);
        assert_eq!((image.width, image.height, image.step), (64, 64, 192));
        assert_eq!(image.data.len(), 64 * 64 * 3);
        assert_eq!(
            &image.data[(32 * 64 + 32) * 3..(32 * 64 + 32) * 3 + 3],
            &[255, 0, 0]
        );
    }
    assert_eq!(output.request_due(&sim).unwrap(), None);
    assert!(images.try_recv().unwrap().is_none());
    assert!(infos.try_recv().unwrap().is_none());
    drop(images);
    sim.step_exact(10).unwrap();
    assert_eq!(
        output.request_due(&sim).unwrap(),
        Some(Progress::RetireRequired)
    );
    assert_eq!(output.pump(&sim).unwrap(), Progress::RetireRequired);
    assert!(infos.try_recv().unwrap().is_none());
    output.retire();
    // Worker retirement did not consume the 20ms slot. Info-only output at
    // that same committed time remains eligible and starts no new renderer.
    assert_eq!(
        output.request_due(&sim).unwrap(),
        Some(Progress::InfoPublished)
    );
    let info = gz_sim_proto::msgs::CameraInfo::decode(
        infos
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(info.header.unwrap().stamp.unwrap().nsec, 20_000_000);
    assert_eq!(output.request_due(&sim).unwrap(), None);
    let images = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    sim.step_exact(10).unwrap();
    assert_eq!(output.request_due(&sim).unwrap(), Some(Progress::Pending));
    scheduled_frame(&mut output, &sim, true);
    let image = gz_sim_proto::msgs::Image::decode(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    let header = image.header.unwrap();
    assert_eq!(header.stamp.unwrap().nsec, 30_000_000);
    assert_eq!(header.data[1].value, ["2"]);
    let info = gz_sim_proto::msgs::CameraInfo::decode(
        infos
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(info.header.unwrap().stamp.unwrap().nsec, 30_000_000);
    assert!(images.try_recv().unwrap().is_none());
    assert!(infos.try_recv().unwrap().is_none());
    output.set_active(false);
    assert!(matches!(
        output.request_due(&sim),
        Err(CameraOutputError::RetireRequired)
    ));
    output.retire();
    assert_eq!(output.request_due(&sim).unwrap(), None);
}

#[test]
fn scheduled_camera_never_publishes_buffered_frames_after_reset_or_removal() {
    let (mut sim, _) = simulation();
    let (session, scope, limit) = session();
    let images = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    let mut output = scheduled_owner(&sim, &session, &scope, limit);
    sim.step_exact(1).unwrap();
    assert_eq!(output.request_due(&sim).unwrap(), Some(Progress::Pending));
    scheduled_frame(&mut output, &sim, false);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.epoch(), 1);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    output.set_active(false);
    assert!(matches!(
        output.request_due(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    assert!(matches!(
        output.pump(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    output.retire();
    drop(output);
    assert!(images.try_recv().unwrap().is_none());
    let mut output = scheduled_owner(&sim, &session, &scope, limit);
    sim.step_exact(1).unwrap();
    assert_eq!(output.request_due(&sim).unwrap(), Some(Progress::Pending));
    scheduled_frame(&mut output, &sim, true);
    let image = gz_sim_proto::msgs::Image::decode(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    let header = image.header.unwrap();
    assert_eq!(header.stamp.unwrap().nsec, 1_000_000);
    assert_eq!(header.data[1].value, ["0"]);
    sim.step_exact(1).unwrap();
    assert_eq!(output.request_due(&sim).unwrap(), Some(Progress::Pending));
    scheduled_frame(&mut output, &sim, false);
    let model = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::robot")
        })
        .unwrap();
    let model_id = model.get::<EntityId>().unwrap().get();
    gz_sim_scene::remove_model(&mut sim, model_id).unwrap();
    assert!(matches!(
        output.pump(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    assert!(matches!(
        output.request_due(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    output.retire();
    drop(output);
    assert!(images.try_recv().unwrap().is_none());
    assert!(
        gz_camera_output::ScheduledCameraOutput::prepare(
            &sim,
            &session,
            &scope,
            CameraOutputConfig {
                origin: [1_000_000_000.0, 0.0, 0.0],
                frame_id: "camera_frame".into(),
                image_key: "@/camera@/image".into(),
                info_key: "@/camera@/info".into(),
                worker_path: env!("CARGO_BIN_EXE_gz-render-worker").into(),
                max_message_bytes: limit,
            },
        )
        .unwrap()
        .is_none()
    );
}

#[test]
fn scheduled_camera_partial_info_then_image_error_does_not_retry_same_stamp() {
    let (mut sim, _) = simulation();
    let (session, scope, limit) = session_with_max(1024);
    let images = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    let infos = session.declare_subscriber("@/camera@/info").wait().unwrap();
    let mut output = scheduled_owner(&sim, &session, &scope, limit);
    sim.step_exact(1).unwrap();
    assert_eq!(output.request_due(&sim).unwrap(), Some(Progress::Pending));
    let info = gz_sim_proto::msgs::CameraInfo::decode(
        infos
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(info.header.unwrap().stamp.unwrap().nsec, 1_000_000);
    scheduled_frame(&mut output, &sim, false);
    assert!(matches!(
        output.pump(&sim),
        Err(CameraOutputError::PublicationTooLarge { limit: 1024, .. })
    ));
    assert_eq!(output.request_due(&sim).unwrap(), None);
    assert_eq!(output.pump(&sim).unwrap(), Progress::NoDemand);
    assert!(infos.try_recv().unwrap().is_none());
    assert!(images.try_recv().unwrap().is_none());
    // The next advancing unlimited-rate frame admits a new timestamp, rather
    // than silently retrying the failed frame or reporting it as successful.
    sim.step_exact(1).unwrap();
    assert_eq!(output.request_due(&sim).unwrap(), Some(Progress::Pending));
    let info = gz_sim_proto::msgs::CameraInfo::decode(
        infos
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(info.header.unwrap().stamp.unwrap().nsec, 2_000_000);
    output.retire();
    assert!(images.try_recv().unwrap().is_none());
}

#[test]
fn reset_invalidates_pending_acquisition_before_publication() {
    let (mut sim, id) = simulation();
    let (session, scope, limit) = session();
    let images = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    let mut output = owner(&sim, id, &session, &scope, limit);
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    await_buffered_frame(&mut output, &sim);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(output.pump(&sim).unwrap(), Progress::RetireRequired);
    assert!(matches!(
        output.request(&sim),
        Err(CameraOutputError::OwnerInvalid)
    ));
    output.retire();
    assert!(
        images
            .recv_timeout(Duration::from_millis(100))
            .unwrap()
            .is_none()
    );
    assert!(matches!(
        CameraOutput::new(
            &sim,
            &session,
            &scope,
            id,
            CameraOutputConfig {
                origin: [1_000_000_000.0, 0.0, 0.0],
                frame_id: "camera_frame".into(),
                image_key: "@/camera@/image".into(),
                info_key: "@/camera@/info".into(),
                worker_path: env!("CARGO_BIN_EXE_gz-render-worker").into(),
                max_message_bytes: limit
            }
        ),
        Err(CameraOutputError::AlreadyPublishing)
    ));
    drop(output);
    let _replacement = owner(&sim, id, &session, &scope, limit);
}

#[test]
fn reset_after_capture_submission_discards_any_later_frame() {
    let (mut sim, id) = simulation();
    let (session, scope, limit) = session();
    let images = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    let mut output = owner(&sim, id, &session, &scope, limit);
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    await_submitted_capture(&mut output, &sim);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(output.pump(&sim).unwrap(), Progress::RetireRequired);
    output.retire();
    assert!(
        images
            .recv_timeout(Duration::from_millis(100))
            .unwrap()
            .is_none()
    );
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
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let built = Command::new("python3")
        .arg(root.join("scripts/native-peer.py"))
        .args([
            "--client",
            "camera",
            "--profile",
            "gazebo-zenoh-client-v1",
            "--json",
        ])
        .output()
        .unwrap();
    assert!(
        built.status.success(),
        "{}",
        String::from_utf8_lossy(&built.stderr)
    );
    let reference: serde_json::Value = serde_json::from_slice(&built.stdout).unwrap();
    assert_eq!(reference["profile"], "gazebo-zenoh-client-v1");
    let executable = PathBuf::from(reference["binary"].as_str().unwrap());
    let prefix = PathBuf::from(reference["prefix"].as_str().unwrap());
    let vendor = PathBuf::from(reference["vendor_prefix"].as_str().unwrap());
    let provenance = PathBuf::from(reference["provenance"].as_str().unwrap());
    for path in [&executable, &prefix, &vendor, &provenance] {
        assert!(
            path.is_absolute(),
            "selected native path is not absolute: {path:?}"
        );
    }
    assert!(executable.is_file() && provenance.is_file());
    assert!(prefix.is_dir() && vendor.is_dir());
    let runtime = std::env::join_paths([prefix.join("lib"), vendor.join("lib")]).unwrap();
    let linkage = Command::new("ldd")
        .arg(&executable)
        .env("LD_LIBRARY_PATH", &runtime)
        .env_remove("LD_PRELOAD")
        .env_remove("LD_AUDIT")
        .output()
        .unwrap();
    assert!(
        linkage.status.success(),
        "{}",
        String::from_utf8_lossy(&linkage.stderr)
    );
    let linkage = std::str::from_utf8(&linkage.stdout).unwrap();
    let selected = prefix.canonicalize().unwrap();
    for library in ["libgz-transport", "libgz-msgs"] {
        assert!(
            linkage
                .lines()
                .any(|line| line.trim_start().starts_with(library)),
            "{library} is linked: {linkage}"
        );
    }
    for line in linkage
        .lines()
        .filter(|line| line.trim_start().starts_with("libgz-"))
    {
        let path = line
            .split("=>")
            .nth(1)
            .unwrap()
            .split_whitespace()
            .next()
            .unwrap();
        assert!(
            PathBuf::from(path)
                .canonicalize()
                .unwrap()
                .starts_with(&selected),
            "{line}"
        );
    }
    let zenoh = linkage
        .lines()
        .find(|line| line.trim_start().starts_with("libzenohc.so "))
        .expect("selected native client must link Zenoh C");
    let loaded = PathBuf::from(
        zenoh
            .split("=>")
            .nth(1)
            .unwrap()
            .split_whitespace()
            .next()
            .unwrap(),
    );
    assert!(
        loaded
            .canonicalize()
            .unwrap()
            .starts_with(vendor.canonicalize().unwrap()),
        "{zenoh}"
    );
    let directory = tempfile::tempdir().unwrap();
    let (mut sim, id) = simulation();
    sim.step_exact(1).unwrap();
    let acquired = sim.elapsed();
    // Stable Zenoh 1.8 does not expose a TCP:0 locator. Reserve a unique
    // loopback address, then release it immediately before the library binds.
    // A collision is a visible setup error, never a retry or readiness claim.
    let reservation = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("tcp/{}", reservation.local_addr().unwrap());
    let native_config = directory.path().join("zenoh.json");
    std::fs::write(&native_config, format!(r#"{{"mode":"peer","listen":{{"endpoints":["{endpoint}"]}},"connect":{{"endpoints":[]}},"scouting":{{"multicast":{{"enabled":false}},"gossip":{{"enabled":false}}}},"transport":{{"shared_memory":{{"enabled":false}}}}}}"#)).unwrap();
    drop(reservation);
    let child = Command::new(executable)
        .env("GZ_PARTITION", "camera")
        .env("GZ_TRANSPORT_IMPLEMENTATION", "zenoh")
        .env("RUST_LOG", "error")
        .env("ZENOH_CONFIG", &native_config)
        .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
        .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
        .env_remove("GZ_TRANSPORT_USERNAME")
        .env_remove("GZ_TRANSPORT_PASSWORD")
        .env_remove("GZ_RELAY")
        .env("LD_LIBRARY_PATH", &runtime)
        .env_remove("LD_PRELOAD")
        .env_remove("LD_AUDIT")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut peer = NativePeer(Some(child));
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut ready = [0; 6];
    for byte in &mut ready {
        use rustix::event::{PollFd, PollFlags, Timespec, poll};
        use std::io::Read;
        let stdout = peer.0.as_mut().unwrap().stdout.as_mut().unwrap();
        let remaining = deadline
            .checked_duration_since(Instant::now())
            .expect("native READY timeout");
        let timeout = Timespec {
            tv_sec: remaining.as_secs().try_into().unwrap(),
            tv_nsec: remaining.subsec_nanos().into(),
        };
        assert_eq!(
            poll(&mut [PollFd::new(&*stdout, PollFlags::IN)], Some(&timeout)).unwrap(),
            1,
            "native READY timeout"
        );
        stdout.read_exact(std::slice::from_mut(byte)).unwrap();
    }
    assert_eq!(&ready, b"ready\n");
    let (session, scope, limit) = session_at(16 * 1024 * 1024, &endpoint, true);
    let mut output = owner(&sim, id, &session, &scope, limit);
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
        String::from_utf8([ready.as_slice(), result.stdout.as_slice()].concat()).unwrap(),
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
        let (session, scope, limit) = session();
        let mut images = Some(
            session
                .declare_subscriber("@/camera@/image")
                .wait()
                .unwrap(),
        );
        let mut output = owner(&sim, id, &session, &scope, limit);
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
                images
                    .recv_timeout(Duration::from_millis(100))
                    .unwrap()
                    .is_none(),
                "{case}"
            );
        }
    }
}

#[test]
fn oversized_real_frame_is_reported_without_publication() {
    let (sim, id) = simulation();
    let (session, scope, limit) = session_with_max(1024);
    let images = session
        .declare_subscriber("@/camera@/image")
        .wait()
        .unwrap();
    let mut output = owner(&sim, id, &session, &scope, limit);
    assert_eq!(output.request(&sim).unwrap(), Progress::Pending);
    await_buffered_frame(&mut output, &sim);
    assert!(matches!(
        output.pump(&sim),
        Err(CameraOutputError::PublicationTooLarge { .. })
    ));
    assert!(
        images
            .recv_timeout(Duration::from_millis(100))
            .unwrap()
            .is_none()
    );
    output.retire();
}
