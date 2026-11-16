// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Compiled by gz-render-worker's integration target so the real worker binary
// comes from Cargo's exact CARGO_BIN_EXE binding, not a guessed installed path.
#[path = "../../gz-camera-output/tests/support/isolated_camera.rs"]
mod isolation;
#[path = "../../../tests/support/zenoh_peer.rs"]
mod selected_native;

use gz_camera_output::{
    CameraOutputError, CameraServer, CameraServerError, CameraServerOptions, Progress,
};
use gz_sim_proto::{
    msgs::{CameraInfo, Header, Image, PixelFormatType, WorldControl, WorldReset},
    prost::Message,
};
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use gz_sim_server::ServerOptions;
use std::{
    num::NonZeroUsize,
    thread,
    time::{Duration, Instant},
};
use zenoh::Wait;

const XML: &str = include_str!("../../../examples/m64_rgb_cylinder.sdf");

fn configuration(endpoint: Option<&str>) -> zenoh::Config {
    let mut config = zenoh::Config::default();
    let connect = endpoint.map_or_else(|| "[]".to_owned(), |value| format!("[\"{value}\"]"));
    for (path, value) in [
        ("mode", "\"peer\""),
        (
            "listen/endpoints",
            if endpoint.is_none() {
                "[\"tcp/127.0.0.1:0\"]"
            } else {
                "[]"
            },
        ),
        ("connect/endpoints", &connect),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(path, value).unwrap();
    }
    config
}

struct Rig {
    app: CameraServer,
    settings: CameraServerOptions,
    observer: zenoh::Session,
    image: String,
    info: String,
}

impl Rig {
    fn new() -> Self {
        Self::with_source(XML)
    }

    fn with_source(source: &str) -> Self {
        // Keep the actual listening Session alive: there is no dropped socket
        // reservation or port guess between the observer and its two peers.
        let observer = zenoh::open(configuration(None)).wait().unwrap();
        let locators: Vec<_> = observer
            .info()
            .locators()
            .wait()
            .into_iter()
            .map(|l| l.to_string())
            .collect();
        assert_eq!(locators.len(), 1);
        assert!(locators[0].starts_with("tcp/127.0.0.1:"));
        assert_ne!(locators[0], "tcp/127.0.0.1:0");
        let mut simulation = load_world_with_sdf_step(
            source,
            "m64-cylinder.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        simulation.set_paused(false);
        let options = ServerOptions {
            partition: format!("m64-cylinder-{}", std::process::id()),
            ..Default::default()
        };
        let image = options.topic_key("/m64/rgb/image").unwrap();
        let info = options.topic_key("/m64/rgb/camera_info").unwrap();
        let settings = CameraServerOptions {
            worker_path: env!("CARGO_BIN_EXE_gz-render-worker").into(),
            origin: [0.0; 3],
            zenoh: configuration(Some(&locators[0])),
        };
        let app = CameraServer::new(
            simulation,
            "default",
            options,
            configuration(Some(&locators[0])),
            NonZeroUsize::new(8).unwrap(),
            &settings,
        )
        .unwrap();
        Self {
            app,
            settings,
            observer,
            image,
            info,
        }
    }

    fn matching(&mut self) {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if self.app.camera_demand().unwrap() == Some((true, true)) {
                return;
            }
            assert!(
                Instant::now() < deadline,
                "real image/info subscribers did not match"
            );
            thread::yield_now();
        }
    }

    fn frame_ready(&mut self, snapshot_may_be_consumed: bool) {
        let deadline = Instant::now() + Duration::from_secs(20);
        // capture() may have consumed SnapshotAccepted before returning Busy.
        let mut snapshot = snapshot_may_be_consumed;
        let mut capture = false;
        loop {
            match self.app.pump().unwrap().unwrap() {
                Progress::SnapshotAccepted => snapshot = true,
                Progress::CaptureSubmitted => {
                    assert!(snapshot);
                    capture = true;
                }
                Progress::FrameReady => {
                    assert!(capture);
                    return;
                }
                Progress::Pending | Progress::Backpressure => thread::yield_now(),
                other => panic!("unexpected actual worker transition: {other:?}"),
            }
            assert!(Instant::now() < deadline, "actual RGB readback timed out");
        }
    }

    fn control(&mut self, pause: bool, reset: bool) {
        assert!(self.app.server_mut().apply_local_control(WorldControl {
            pause,
            reset: reset.then_some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        }));
    }
}

fn header(header: Header, stamp: Duration, sequence: Option<u64>) {
    let time = header.stamp.unwrap();
    assert_eq!(time.sec, i64::try_from(stamp.as_secs()).unwrap());
    assert_eq!(time.nsec, i32::try_from(stamp.subsec_nanos()).unwrap());
    assert_eq!(
        header
            .data
            .iter()
            .find(|entry| entry.key == "frame_id")
            .unwrap()
            .value,
        ["rig/body/camera"]
    );
    let actual = header.data.iter().find(|entry| entry.key == "seq");
    match sequence {
        Some(value) => assert_eq!(actual.unwrap().value, [value.to_string()]),
        None => assert!(actual.is_none()),
    }
}

fn image(sample: zenoh::sample::Sample, stamp: Duration, sequence: u64) -> Image {
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.Image"
    );
    let image = Image::decode(sample.payload().to_bytes().as_ref()).unwrap();
    header(image.header.clone().unwrap(), stamp, Some(sequence));
    assert_eq!((image.width, image.height, image.step), (64, 64, 192));
    assert_eq!(image.pixel_format_type, PixelFormatType::RgbInt8 as i32);
    assert_eq!(image.data.len(), 64 * 64 * 3);
    for (x, y, expected) in [
        (32, 32, [0, 255, 0]), // near opaque box occludes the cap
        (32, 24, [255, 0, 0]),
        (40, 32, [255, 0, 0]), // cap interior
        (23, 23, [0, 0, 255]),
        (40, 40, [0, 0, 255]), // inside AABB, outside cylinder
    ] {
        let offset = (y * 64 + x) * 3;
        assert_eq!(
            &image.data[offset..offset + 3],
            &expected,
            "pixel ({x},{y})"
        );
    }
    image
}

fn info(sample: zenoh::sample::Sample, stamp: Duration) {
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.CameraInfo"
    );
    let info = CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
    header(info.header.unwrap(), stamp, None);
    assert_eq!((info.width, info.height), (64, 64));
    let k = info.intrinsics.unwrap().k;
    assert_eq!(k.len(), 9);
    for (actual, expected) in k
        .into_iter()
        .zip([32.0, 0.0, 32.0, 0.0, 32.0, 32.0, 0.0, 0.0, 1.0])
    {
        assert!((actual - expected).abs() < 1e-12);
    }
    assert_eq!(info.distortion.unwrap().k, [0.0; 5]);
    assert_eq!(
        info.rectification_matrix,
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    );
    let projection = info.projection.unwrap().p;
    assert_eq!(projection.len(), 12);
    for (actual, expected) in projection.into_iter().zip([
        32.0, 0.0, 32.0, 0.0, 0.0, 32.0, 32.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    ]) {
        assert!((actual - expected).abs() < 1e-12);
    }
}

#[test]
fn canonical_rgb_cylinder_frame_preserves_dimensions_stride_frames_and_sequence() {
    if isolation::child(
        "canonical_rgb_cylinder_frame_preserves_dimensions_stride_frames_and_sequence",
        90,
    ) {
        return;
    }
    let mut rig = Rig::new();
    let images = rig
        .observer
        .declare_subscriber(rig.image.clone())
        .wait()
        .unwrap();
    let infos = rig
        .observer
        .declare_subscriber(rig.info.clone())
        .wait()
        .unwrap();
    rig.matching();
    let mut acquired = Vec::new();
    while rig.app.server().simulation().elapsed() < Duration::from_millis(200) {
        let tick = rig.app.tick(&rig.settings).unwrap();
        let (_, request) = rig.app.capture().unwrap();
        match request {
            Some(Progress::Pending) => {
                info(
                    infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
                    tick.sim_time,
                );
                rig.frame_ready(false);
                assert_eq!(rig.app.pump().unwrap(), Some(Progress::ImagePublished));
                image(
                    images
                        .recv_timeout(Duration::from_secs(5))
                        .unwrap()
                        .unwrap(),
                    tick.sim_time,
                    acquired.len() as u64,
                );
                acquired.push(tick.sim_time);
            }
            None => {}
            other => panic!("unexpected cylinder admission: {other:?}"),
        }
    }
    assert_eq!(
        acquired,
        [
            Duration::from_millis(1),
            Duration::from_millis(100),
            Duration::from_millis(200)
        ]
    );
    rig.control(true, false);
    for _ in 0..3 {
        let frame = rig.app.update(&rig.settings).unwrap();
        assert!(frame.info.paused && frame.info.dt.is_zero() && frame.requested.is_none());
        assert_eq!(frame.info.sim_time, Duration::from_millis(200));
    }
    assert!(images.try_recv().unwrap().is_none());
    assert!(infos.try_recv().unwrap().is_none());
    rig.app.shutdown();
    assert!(!rig.app.has_camera_output());
    // A second genuine world removes only the green VISUAL, retaining its box
    // collision/body. The exposed cap must now occupy the centre pixel.
    let source=XML.replace("<visual name=\"green\"><geometry><box><size>0.4 0.4 0.4</size></box></geometry><material><diffuse>0 1 0 1</diffuse></material></visual>","");
    assert_ne!(source, XML);
    let mut exposed = Rig::with_source(&source);
    let pixels = exposed
        .observer
        .declare_subscriber(exposed.image.clone())
        .wait()
        .unwrap();
    let metadata = exposed
        .observer
        .declare_subscriber(exposed.info.clone())
        .wait()
        .unwrap();
    exposed.matching();
    let frame = exposed.app.update(&exposed.settings).unwrap();
    assert_eq!(frame.requested, Some(Progress::Pending));
    info(
        metadata
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap(),
        frame.info.sim_time,
    );
    exposed.frame_ready(false);
    assert_eq!(exposed.app.pump().unwrap(), Some(Progress::ImagePublished));
    let sample = pixels
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap();
    let actual = Image::decode(sample.payload().to_bytes().as_ref()).unwrap();
    header(actual.header.unwrap(), frame.info.sim_time, Some(0));
    assert_eq!(
        &actual.data[(32 * 64 + 32) * 3..(32 * 64 + 32) * 3 + 3],
        &[255, 0, 0]
    );
    assert_eq!(
        &actual.data[(23 * 64 + 23) * 3..(23 * 64 + 23) * 3 + 3],
        &[0, 0, 255]
    );
    exposed.app.shutdown();
    assert!(!exposed.app.has_camera_output());
}

#[test]
fn rgb_cylinder_reset_cancels_buffered_shape_capture_without_replaying_old_stamp() {
    if isolation::child(
        "rgb_cylinder_reset_cancels_buffered_shape_capture_without_replaying_old_stamp",
        90,
    ) {
        return;
    }
    let mut rig = Rig::new();
    let images = rig
        .observer
        .declare_subscriber(rig.image.clone())
        .wait()
        .unwrap();
    let infos = rig
        .observer
        .declare_subscriber(rig.info.clone())
        .wait()
        .unwrap();
    rig.matching();
    let first = rig.app.update(&rig.settings).unwrap();
    assert_eq!(first.requested, Some(Progress::Pending));
    info(
        infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
        first.info.sim_time,
    );
    for _ in 1..99 {
        rig.app.server_mut().update().unwrap();
    }
    let tick = rig.app.tick(&rig.settings).unwrap();
    assert_eq!(tick.sim_time, Duration::from_millis(100));
    assert!(matches!(
        rig.app.capture(),
        Err(CameraServerError::Camera(CameraOutputError::Busy))
    ));
    assert_eq!(
        rig.app.server().simulation().elapsed(),
        Duration::from_millis(100)
    );
    rig.frame_ready(true);
    rig.control(true, true);
    assert!(matches!(
        rig.app.pump(),
        Err(CameraServerError::Server(
            gz_sim_server::Error::IoPreparationRequired
        ))
    ));
    assert!(!rig.app.has_camera_output());
    assert!(images.try_recv().unwrap().is_none());
    rig.app.prepare_io(&rig.settings).unwrap();
    let reset = rig.app.update(&rig.settings).unwrap();
    assert_eq!((reset.info.epoch, reset.info.sim_time), (1, Duration::ZERO));
    assert!(reset.info.paused && reset.requested.is_none());
    rig.matching();
    rig.control(false, false);
    let frame = rig.app.update(&rig.settings).unwrap();
    assert_eq!(
        (frame.info.epoch, frame.info.sim_time),
        (1, Duration::from_millis(1))
    );
    assert_eq!(frame.requested, Some(Progress::Pending));
    info(
        infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
        frame.info.sim_time,
    );
    rig.frame_ready(false);
    assert_eq!(rig.app.pump().unwrap(), Some(Progress::ImagePublished));
    image(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap(),
        frame.info.sim_time,
        0,
    );
    assert!(images.try_recv().unwrap().is_none());
    rig.app.shutdown();
    assert!(!rig.app.has_camera_output());
    // Existing CameraServer retirement discards the worker status: this test
    // claims output retirement, not an invented renderer exit/reap code.
}

struct NativeChild {
    child: std::process::Child,
    stdout: std::path::PathBuf,
    stderr: std::path::PathBuf,
}
impl Drop for NativeChild {
    fn drop(&mut self) {
        if matches!(self.child.try_wait(), Ok(None)) {
            let _ = self.child.kill();
        }
        let _ = self.child.wait();
        if thread::panicking() {
            eprintln!(
                "cylinder native stdout: {}",
                std::fs::read_to_string(&self.stdout).unwrap_or_default()
            );
            eprintln!(
                "cylinder native stderr: {}",
                std::fs::read_to_string(&self.stderr).unwrap_or_default()
            );
        }
    }
}
impl NativeChild {
    fn output(&self) -> String {
        assert!(std::fs::metadata(&self.stdout).unwrap().len() <= 65536);
        assert!(std::fs::metadata(&self.stderr).unwrap().len() <= 4096);
        std::fs::read_to_string(&self.stdout).unwrap()
    }
}
fn unhex(value: &str) -> Vec<u8> {
    assert!(value.len() <= 32768 && value.len().is_multiple_of(2));
    value
        .as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let nibble = |byte: u8| match byte {
                b'0'..=b'9' => byte - b'0',
                b'a'..=b'f' => byte - b'a' + 10,
                _ => panic!("noncanonical native hex"),
            };
            nibble(pair[0]) * 16 + nibble(pair[1])
        })
        .collect()
}

#[test]
fn cpp_cylinder_consumer_receives_real_software_pixels_and_full_canonical_messages() {
    if isolation::child(
        "cpp_cylinder_consumer_receives_real_software_pixels_and_full_canonical_messages",
        90,
    ) {
        return;
    }
    use std::{
        fs,
        path::Path,
        process::{Command, Stdio},
    };
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let peer = selected_native::NativePeer::build(root, "camera");
    let mut rig = Rig::new();
    let directory = tempfile::tempdir().unwrap();
    let network = directory.path().join("cylinder-zenoh.json");
    let locators: Vec<_> = rig
        .observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|value| value.to_string())
        .collect();
    fs::write(
        &network,
        serde_json::to_vec(&serde_json::json!({
            "mode":"client","listen":{"endpoints":[]},"connect":{"endpoints":locators},
            "scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},
            "transport":{"shared_memory":{"enabled":false}}
        }))
        .unwrap(),
    )
    .unwrap();
    let capture = peer.binary.parent().unwrap();
    let stdout = capture.join("m64-camera.stdout");
    let stderr = capture.join("m64-camera.stderr");
    let create = |path: &Path| {
        fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .unwrap()
    };
    let mut command = Command::new(&peer.binary);
    peer.configure_runtime(&mut command);
    command
        .arg("cylinder")
        .env("GZ_IP", "127.0.0.1")
        .env("GZ_PARTITION", &rig.app.server().options().partition)
        .env("ZENOH_CONFIG", &network)
        .env("RUST_LOG", "error")
        .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
        .env_remove("GZ_TRANSPORT_USERNAME")
        .env_remove("GZ_TRANSPORT_PASSWORD")
        .env_remove("GZ_RELAY")
        .stdin(Stdio::null())
        .stdout(Stdio::from(create(&stdout)))
        .stderr(Stdio::from(create(&stderr)));
    let mut native = NativeChild {
        child: command.spawn().unwrap(),
        stdout,
        stderr,
    };
    let deadline = Instant::now() + Duration::from_secs(20);
    while native.output() != "ready\n" {
        assert!(
            native.child.try_wait().unwrap().is_none(),
            "native exited before ready"
        );
        assert!(Instant::now() < deadline, "native READY deadline");
        thread::yield_now();
    }
    // Before adding local subscribers, both demanded routes can only come
    // from the actual CPP consumer.
    rig.matching();
    let images = rig
        .observer
        .declare_subscriber(rig.image.clone())
        .wait()
        .unwrap();
    let infos = rig
        .observer
        .declare_subscriber(rig.info.clone())
        .wait()
        .unwrap();
    // The native receipt below establishes actual upstream delivery.
    assert_eq!(rig.app.server().simulation().elapsed(), Duration::ZERO);
    let frame = rig.app.update(&rig.settings).unwrap();
    assert_eq!(frame.info.sim_time, Duration::from_millis(1));
    assert_eq!(frame.requested, Some(Progress::Pending));
    let metadata = infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    let expected_info = CameraInfo::decode(metadata.payload().to_bytes().as_ref()).unwrap();
    info(metadata, frame.info.sim_time);
    rig.frame_ready(false);
    assert_eq!(rig.app.pump().unwrap(), Some(Progress::ImagePublished));
    let expected_image = image(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap(),
        frame.info.sim_time,
        0,
    );
    let status = loop {
        native.output();
        if let Some(status) = native.child.try_wait().unwrap() {
            break status;
        }
        assert!(
            Instant::now() < deadline,
            "native canonical receipt deadline"
        );
        thread::yield_now();
    };
    let output = native.output();
    let errors = fs::read_to_string(&native.stderr).unwrap();
    fs::write(capture.join("m64-camera-receipt.json"),serde_json::to_vec_pretty(&serde_json::json!({
        "exit_code":status.code(),"child_reaped":true,"mode":"cylinder",
        "image_key":rig.image,"info_key":rig.info,"frame":"rig/body/camera","sim_ns":"1000000",
        "stdout":native.stdout,"stderr":native.stderr,
        "scope":"Actual upstream canonical receipt of software-worker Image and CameraInfo; not native rendering or Ogre parity; renderer exit code UNKNOWN"
    })).unwrap()).unwrap();
    assert!(
        status.success(),
        "native status {status}: {output} {errors}"
    );
    assert!(errors.is_empty(), "{errors}");
    let lines: Vec<_> = output.lines().collect();
    assert_eq!(lines.len(), 4);
    assert_eq!(lines[0], "ready");
    assert_eq!(lines[3], "PASS");
    assert_eq!(
        Image::decode(unhex(lines[1].strip_prefix("IMAGE ").unwrap()).as_slice()).unwrap(),
        expected_image
    );
    assert_eq!(
        CameraInfo::decode(unhex(lines[2].strip_prefix("INFO ").unwrap()).as_slice()).unwrap(),
        expected_info
    );
    assert!(Instant::now() < deadline);
    rig.app.shutdown();
    assert!(!rig.app.has_camera_output());
}
