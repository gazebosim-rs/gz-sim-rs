// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../../gz-camera-output/tests/support/isolated_camera.rs"]
mod isolation;
#[path = "../../../tests/support/zenoh_peer.rs"]
mod selected_native;

use gz_camera_output::{
    CameraOutputError, CameraServer, CameraServerError, CameraServerOptions, Progress,
};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_proto::{
    msgs::{Boolean, Entity, Image, WorldControl, WorldReset},
    prost::Message,
};
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_sdf_step};
use gz_sim_server::{Error as ServerError, ServerOptions};
use std::{
    net::TcpListener,
    num::NonZeroUsize,
    sync::atomic::{AtomicU64, Ordering},
    thread,
    time::{Duration, Instant},
};
use zenoh::Wait;

fn configuration(listen: &[String], connect: &[String]) -> zenoh::Config {
    let mut config = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\"".into()),
        ("listen/endpoints", serde_json::to_string(listen).unwrap()),
        ("connect/endpoints", serde_json::to_string(connect).unwrap()),
        ("connect/timeout_ms", "2000".into()),
        ("scouting/multicast/enabled", "false".into()),
        ("scouting/gossip/enabled", "false".into()),
    ] {
        config.insert_json5(key, &value).unwrap();
    }
    config
}

fn camera_source() -> String {
    include_str!("../../gz-sim-scene/tests/fixtures/camera-binding.sdf")
        .replace("<world name=\"default\">", "<world name=\"default\"><plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/>")
        .replace("</link>", "<visual name='box'><pose>5 0 0 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry><material><diffuse>1 0 0 1</diffuse></material></visual></link>")
}

fn simulation() -> Simulation {
    load_camera(&camera_source())
}

fn load_camera(source: &str) -> Simulation {
    let mut sim = load_world_with_sdf_step(
        source,
        "automatic-camera-worker.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    sim.set_paused(false);
    sim
}

struct Application {
    app: CameraServer,
    options: CameraServerOptions,
    camera_endpoint: String,
    control: zenoh::Session,
}
impl Application {
    fn new() -> Self {
        Self::with_scene(simulation(), &[])
    }

    fn with_scene(simulation: Simulation, remaps: &[(&str, &str)]) -> Self {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let camera = TcpListener::bind("127.0.0.1:0").unwrap();
        let server = TcpListener::bind("127.0.0.2:0").unwrap();
        let camera_endpoint = format!("tcp/{}", camera.local_addr().unwrap());
        let server_endpoint = format!("tcp/{}", server.local_addr().unwrap());
        let options = CameraServerOptions {
            worker_path: env!("CARGO_BIN_EXE_gz-render-worker").into(),
            origin: [1_000_000_000., 0., 0.],
            zenoh: configuration(std::slice::from_ref(&camera_endpoint), &[]),
        };
        let mut world = ServerOptions {
            partition: format!(
                "automatic-worker-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ),
            ..Default::default()
        };
        for (from, to) in remaps {
            world.add_topic_remap(from, to).unwrap();
        }
        drop(camera);
        drop(server);
        let app = CameraServer::new(
            simulation,
            "default",
            world,
            configuration(std::slice::from_ref(&server_endpoint), &[]),
            NonZeroUsize::new(4).unwrap(),
            &options,
        )
        .unwrap();
        let control = zenoh::open(configuration(&[], &[server_endpoint]))
            .wait()
            .unwrap();
        Self {
            app,
            options,
            camera_endpoint,
            control,
        }
    }
    fn image_session(&self) -> zenoh::Session {
        zenoh::open(configuration(
            &[],
            std::slice::from_ref(&self.camera_endpoint),
        ))
        .wait()
        .unwrap()
    }
    fn image_key(&self) -> String {
        self.app
            .server()
            .options()
            .topic_key("/explicit-camera")
            .unwrap()
    }
    fn begin_frame(&mut self) -> Duration {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let frame = self.app.update(&self.options).unwrap();
            match frame.requested {
                Some(Progress::Pending) => return frame.info.sim_time,
                Some(Progress::NoDemand) => {}
                other => panic!("unexpected admission: {other:?}"),
            }
            // Actual frames observe asynchronous remote matching. No request,
            // startup error or failed acquisition is repeated by this helper.
            assert!(Instant::now() < deadline, "image matching did not arrive");
            thread::yield_now();
        }
    }
    fn frame_ready(&mut self, snapshot_may_be_consumed: bool) {
        let deadline = Instant::now() + Duration::from_secs(20);
        // An update returning Busy may already have pumped SnapshotAccepted.
        let (mut snapshot, mut submitted) = (snapshot_may_be_consumed, false);
        loop {
            match self.app.pump().unwrap().unwrap() {
                Progress::SnapshotAccepted => snapshot = true,
                Progress::CaptureSubmitted => {
                    assert!(snapshot);
                    submitted = true;
                }
                Progress::FrameReady => {
                    assert!(submitted);
                    return;
                }
                Progress::Pending | Progress::Backpressure => thread::yield_now(),
                other => panic!("unexpected capture: {other:?}"),
            }
            assert!(
                Instant::now() < deadline,
                "automatic camera readback timed out"
            );
        }
    }
    fn control(&mut self, command: WorldControl) {
        let key = self
            .app
            .server()
            .options()
            .topic_key("/world/default/control")
            .unwrap();
        let replies = self
            .control
            .get(key)
            .payload(command.encode_to_vec())
            .timeout(Duration::from_secs(5))
            .wait()
            .unwrap();
        let before = self.app.server().counters().controls_accepted;
        let deadline = Instant::now() + Duration::from_secs(5);
        while self.app.server().counters().controls_accepted == before {
            self.app.server_mut().update().unwrap();
            assert!(Instant::now() < deadline);
            thread::yield_now();
        }
        let sample = replies
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap()
            .into_result()
            .unwrap();
        assert!(
            Boolean::decode(sample.payload().to_bytes().as_ref())
                .unwrap()
                .data
        );
        assert_eq!(self.app.server().counters().controls_accepted, before + 1);
    }
}

fn check_image(sample: zenoh::sample::Sample, stamp: Duration, sequence: u64) {
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.Image"
    );
    let image = Image::decode(sample.payload().to_bytes().as_ref()).unwrap();
    assert_eq!((image.width, image.height, image.step), (64, 64, 192));
    assert_eq!(image.data.len(), 64 * 64 * 3);
    assert_eq!(
        &image.data[(32 * 64 + 32) * 3..(32 * 64 + 32) * 3 + 3],
        &[255, 0, 0]
    );
    let header = image.header.unwrap();
    let time = header.stamp.unwrap();
    assert_eq!(
        (time.sec, time.nsec),
        (stamp.as_secs() as i64, stamp.subsec_nanos() as i32)
    );
    assert_eq!(header.data[0].value, ["robot::body::camera"]);
    assert_eq!(header.data[1].key, "seq");
    assert_eq!(header.data[1].value, [sequence.to_string()]);
}

#[test]
fn loaded_camera_admits_one_frame_preserves_busy_stamp_and_retires_on_interest_loss() {
    if isolation::child(
        "loaded_camera_admits_one_frame_preserves_busy_stamp_and_retires_on_interest_loss",
        90,
    ) {
        return;
    }
    let mut app = Application::new();
    assert_eq!(
        app.app.update(&app.options).unwrap().requested,
        Some(Progress::NoDemand)
    );
    let session = app.image_session();
    let images = session.declare_subscriber(app.image_key()).wait().unwrap();
    let acquired = app.begin_frame();
    assert!(matches!(
        app.app.update(&app.options),
        Err(CameraServerError::Camera(CameraOutputError::Busy))
    ));
    assert!(app.app.server().simulation().elapsed() > acquired);
    app.frame_ready(true);
    assert_eq!(app.app.pump().unwrap(), Some(Progress::ImagePublished));
    check_image(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap(),
        acquired,
        0,
    );
    assert!(images.try_recv().unwrap().is_none());
    drop(images);
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match app.app.pump().unwrap() {
            Some(Progress::RetireRequired) => break,
            Some(Progress::NoDemand) => thread::yield_now(),
            other => panic!("unexpected retirement: {other:?}"),
        }
        assert!(Instant::now() < deadline, "withdrawal not observed");
    }
    assert_eq!(
        app.app.update(&app.options).unwrap().requested,
        Some(Progress::NoDemand)
    );
    let images = session.declare_subscriber(app.image_key()).wait().unwrap();
    let acquired = app.begin_frame();
    app.frame_ready(false);
    assert_eq!(app.app.pump().unwrap(), Some(Progress::ImagePublished));
    check_image(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap(),
        acquired,
        1,
    );
    app.app.shutdown();
    assert!(!app.app.has_camera_output());
}

#[test]
fn buffered_frame_is_canceled_by_accepted_reset_and_exposed_server_shutdown() {
    if isolation::child(
        "buffered_frame_is_canceled_by_accepted_reset_and_exposed_server_shutdown",
        90,
    ) {
        return;
    }
    let mut app = Application::new();
    let session = app.image_session();
    let images = session.declare_subscriber(app.image_key()).wait().unwrap();
    app.begin_frame();
    app.frame_ready(false);
    app.control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    });
    assert_eq!(app.app.server().simulation().epoch(), 0);
    assert!(app.app.server().io_preparation_required());
    assert!(matches!(
        app.app.pump(),
        Err(CameraServerError::Server(
            ServerError::IoPreparationRequired
        ))
    ));
    assert!(!app.app.has_camera_output());
    assert!(images.try_recv().unwrap().is_none());
    app.app.prepare_io(&app.options).unwrap();
    let reset = app.app.update(&app.options).unwrap();
    assert_eq!(reset.info.epoch, 1);
    assert_eq!(reset.info.sim_time, Duration::ZERO);
    assert!(reset.requested.is_none());
    drop(images);
    session.close().wait().unwrap();
    let session = app.image_session();
    let images = session.declare_subscriber(app.image_key()).wait().unwrap();
    app.control(WorldControl {
        pause: false,
        ..Default::default()
    });
    let acquired = app.begin_frame();
    app.frame_ready(false);
    assert_eq!(app.app.pump().unwrap(), Some(Progress::ImagePublished));
    check_image(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap(),
        acquired,
        0,
    );
    app.begin_frame();
    app.frame_ready(false);
    let epoch = app.app.server().simulation().epoch();
    let elapsed = app.app.server().simulation().elapsed();
    app.app.server_mut().shutdown();
    assert!(matches!(
        app.app.pump(),
        Err(CameraServerError::Server(ServerError::WorldClosed))
    ));
    assert!(!app.app.has_camera_output());
    assert!(images.try_recv().unwrap().is_none());
    assert_eq!(app.app.server().simulation().epoch(), epoch);
    assert_eq!(app.app.server().simulation().elapsed(), elapsed);
}

#[test]
fn actual_model_removal_cancels_buffered_frame_and_releases_camera_family() {
    if isolation::child(
        "actual_model_removal_cancels_buffered_frame_and_releases_camera_family",
        90,
    ) {
        return;
    }
    let mut app = Application::new();
    let session = app.image_session();
    let images = session.declare_subscriber(app.image_key()).wait().unwrap();
    app.begin_frame();
    app.frame_ready(false);
    let model = app
        .app
        .server()
        .simulation()
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::robot"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let key = app
        .app
        .server()
        .options()
        .topic_key("/world/default/remove")
        .unwrap();
    let replies = app
        .control
        .get(key)
        .payload(
            Entity {
                id: model,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let before = app.app.server().counters().removals_completed;
    let deadline = Instant::now() + Duration::from_secs(5);
    while app.app.server().counters().removals_completed == before {
        app.app.server_mut().update().unwrap();
        assert!(Instant::now() < deadline, "model removal was not committed");
        thread::yield_now();
    }
    let reply = replies
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap()
        .into_result()
        .unwrap();
    assert!(
        Boolean::decode(reply.payload().to_bytes().as_ref())
            .unwrap()
            .data
    );
    assert_eq!(app.app.server().counters().removals_completed, before + 1);
    assert!(gz_sim_core::entity_by_id(app.app.server().simulation().world(), model).is_none());
    assert_eq!(app.app.pump().unwrap(), None);
    assert!(!app.app.has_camera_output());
    assert!(images.try_recv().unwrap().is_none());
    for _ in 0..2 {
        let frame = app.app.update(&app.options).unwrap();
        assert!(frame.pumped.is_none() && frame.requested.is_none());
    }
}

struct OwnedNative(std::process::Child);
impl Drop for OwnedNative {
    fn drop(&mut self) {
        if matches!(self.0.try_wait(), Ok(None)) {
            let _ = self.0.kill();
        }
        let _ = self.0.wait();
    }
}

#[test]
fn cpp_subscriber_receives_default_derived_topics_and_loaded_frame_from_camera_server() {
    if isolation::child(
        "cpp_subscriber_receives_default_derived_topics_and_loaded_frame_from_camera_server",
        90,
    ) {
        return;
    }
    use std::{
        io::Read,
        path::Path,
        process::{Command, Stdio},
    };
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let peer = selected_native::NativePeer::build(root, "camera");
    let source = camera_source().replace(
        "<topic>/explicit-camera</topic>",
        "<topic></topic><frame_id>camera_frame</frame_id>",
    );
    let image = "/world/default/model/robot/link/body/sensor/camera/image";
    let info = "/world/default/model/robot/link/body/sensor/camera/camera_info";
    // The unchanged native fixture uses /image and /info. Only the actual
    // loaded-world default derivation reaches these one-pass remap sources.
    let scene = load_camera(&source);
    let sensor = scene
        .world()
        .iter_entities()
        .find_map(|entity| entity.get::<gz_sim_scene::CameraSensor>())
        .unwrap();
    assert!(sensor.0.topic.is_empty());
    assert!(sensor.0.camera().unwrap().camera_info_topic.is_empty());
    assert_eq!(sensor.0.frame_id, "camera_frame");
    let mut app = Application::with_scene(scene, &[(image, "/image"), (info, "/info")]);
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("zenoh.json");
    std::fs::write(
        &path,
        serde_json::to_vec(&serde_json::json!({
            "mode":"peer", "listen":{"endpoints":[]},
            "connect":{"endpoints":[app.camera_endpoint]},
            "scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},
            "transport":{"shared_memory":{"enabled":false}}
        }))
        .unwrap(),
    )
    .unwrap();
    let mut command = Command::new(&peer.binary);
    peer.configure_runtime(&mut command);
    command
        .env("GZ_IP", "127.0.0.1")
        .env("GZ_PARTITION", &app.app.server().options().partition)
        .env("ZENOH_CONFIG", &path)
        .env("RUST_LOG", "error")
        .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
        .env_remove("GZ_TRANSPORT_USERNAME")
        .env_remove("GZ_TRANSPORT_PASSWORD")
        .env_remove("GZ_RELAY")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = OwnedNative(command.spawn().unwrap());
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut ready = [0; 6];
    for byte in &mut ready {
        use rustix::event::{PollFd, PollFlags, Timespec, poll};
        let stdout = child.0.stdout.as_mut().unwrap();
        let remaining = deadline
            .checked_duration_since(Instant::now())
            .expect("CPP READY timeout");
        let timeout = Timespec {
            tv_sec: remaining.as_secs().try_into().unwrap(),
            tv_nsec: remaining.subsec_nanos().into(),
        };
        assert_eq!(
            poll(&mut [PollFd::new(&*stdout, PollFlags::IN)], Some(&timeout)).unwrap(),
            1
        );
        stdout.read_exact(std::slice::from_mut(byte)).unwrap();
    }
    assert_eq!(&ready, b"ready\n");
    // Read the app's own matching snapshots before admitting its first frame.
    // Actual unchanged CPP receipts below are the delivery/decoding proof.
    while app.app.camera_demand().unwrap() != Some((true, true)) {
        assert!(
            child.0.try_wait().unwrap().is_none(),
            "CPP subscriber exited before matching"
        );
        assert!(
            Instant::now() < deadline,
            "CPP subscriptions not observed by camera owner"
        );
        thread::yield_now();
    }
    assert_eq!(app.app.server().simulation().elapsed(), Duration::ZERO);
    let frame = app.app.update(&app.options).unwrap();
    assert_eq!(frame.info.sim_time, Duration::from_millis(1));
    assert_eq!(frame.requested, Some(Progress::Pending));
    app.frame_ready(false);
    assert_eq!(app.app.pump().unwrap(), Some(Progress::ImagePublished));
    while child.0.try_wait().unwrap().is_none() {
        assert!(Instant::now() < deadline, "CPP camera receipt timed out");
        thread::yield_now();
    }
    let mut stdout = String::new();
    let mut stderr = String::new();
    child
        .0
        .stdout
        .take()
        .unwrap()
        .read_to_string(&mut stdout)
        .unwrap();
    child
        .0
        .stderr
        .take()
        .unwrap()
        .read_to_string(&mut stderr)
        .unwrap();
    let status = child.0.wait().unwrap();
    // Retain the actual unchanged native client's receipt/status, separately
    // from compiler provenance. No environment dump or reconstructed output.
    let capture = peer.binary.parent().unwrap();
    std::fs::write(capture.join("automatic-camera.stdout"), &stdout).unwrap();
    std::fs::write(capture.join("automatic-camera.stderr"), &stderr).unwrap();
    std::fs::write(capture.join("automatic-camera-receipt.json"), serde_json::to_vec_pretty(&serde_json::json!({
        "exit_code": status.code(), "default_image": image, "default_info": info,
        "actual_image_key": app.app.server().options().topic_key(image).unwrap(),
        "actual_info_key": app.app.server().options().topic_key(info).unwrap(),
        "gz_ip": "127.0.0.1", "backend": "zenoh", "frame": "camera_frame",
        "sim_ns": frame.info.sim_time.as_nanos().to_string(),
        "scope": "Actual unchanged C++ Image+CameraInfo receipt from the single CameraServer path; not native rendering/Manager parity"
    })).unwrap()).unwrap();
    assert!(status.success(), "{stdout} {stderr}");
    assert_eq!(stdout, "0 1000000\n");
    assert!(stderr.is_empty(), "{stderr}");
    app.app.shutdown();
    assert!(!app.app.has_camera_output());
}
