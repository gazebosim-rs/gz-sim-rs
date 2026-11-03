// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
// Compiled by gz-render-worker's integration target so the real worker binary
// comes from Cargo's exact CARGO_BIN_EXE binding, not a guessed installed path.
#[path = "../support/isolated_camera.rs"]
mod isolation;

use gz_camera_output::{
    CameraOutputError, CameraServer, CameraServerError, CameraServerOptions, Progress,
};
use gz_sim_core::EntityId;
use gz_sim_proto::{
    msgs::{Boolean, CameraInfo, Entity, Header, Image, PixelFormatType, WorldControl, WorldReset},
    prost::Message,
};
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_sdf_step};
use gz_sim_server::ServerOptions;
use std::{
    num::NonZeroUsize,
    thread,
    time::{Duration, Instant},
};
use zenoh::Wait;

const XML: &str = include_str!("../fixtures/m6-depth.sdf");

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
        Self::with_xml(XML)
    }

    fn with_xml(xml: &str) -> Self {
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
            xml,
            "m6-depth.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        simulation.set_paused(false);
        let options = ServerOptions {
            partition: format!("m6-depth-{}", std::process::id()),
            ..Default::default()
        };
        let image = options.topic_key("/m6/depth/image").unwrap();
        let info = options.topic_key("/m6/depth/camera_info").unwrap();
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

    fn frame_ready(&mut self) {
        let deadline = Instant::now() + Duration::from_secs(20);
        let mut snapshot = false;
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
            assert!(Instant::now() < deadline, "actual depth readback timed out");
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
        ["rig/body/depth_camera"]
    );
    let actual = header.data.iter().find(|entry| entry.key == "seq");
    match sequence {
        Some(value) => assert_eq!(actual.unwrap().value, [value.to_string()]),
        None => assert!(actual.is_none()),
    }
}

fn depth_packet(sample: zenoh::sample::Sample, stamp: Duration, sequence: u64) -> Image {
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.Image"
    );
    let image = Image::decode(sample.payload().to_bytes().as_ref()).unwrap();
    header(image.header.clone().unwrap(), stamp, Some(sequence));
    assert_eq!((image.width, image.height, image.step), (64, 64, 256));
    assert_eq!(image.pixel_format_type, PixelFormatType::RFloat32 as i32);
    assert_eq!(image.pixel_format_type, 13);
    assert_eq!(image.data.len(), 64 * 64 * 4);
    for word in image.data.chunks_exact(4) {
        let metres = f32::from_le_bytes(word.try_into().unwrap());
        assert!(
            metres > 0.0 && !metres.is_nan(),
            "invalid axial depth: {metres}"
        );
    }
    image
}

fn metres(image: &Image, x: usize, y: usize) -> f32 {
    let offset = (y * 64 + x) * 4;
    f32::from_le_bytes(image.data[offset..offset + 4].try_into().unwrap())
}

fn image(sample: zenoh::sample::Sample, stamp: Duration, sequence: u64) -> Image {
    let image = depth_packet(sample, stamp, sequence);
    // Rig and box share a nonidentity world rotation. All selected rays hit the
    // same face: optical axial metres stay 4, rather than increasing radially.
    for (x, y) in [(32, 32), (28, 32), (36, 32), (32, 28), (32, 36)] {
        assert!((metres(&image, x, y) - 4.0).abs() < 1e-4);
    }
    for (x, y) in [(0, 0), (63, 0), (0, 63), (63, 63)] {
        assert_eq!(metres(&image, x, y).to_bits(), f32::INFINITY.to_bits());
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
fn loaded_depth_metres_calibration_and_ten_hz_acquisitions_reach_real_zenoh() {
    if isolation::child(
        "m6_depth::loaded_depth_metres_calibration_and_ten_hz_acquisitions_reach_real_zenoh",
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
        let committed = rig.app.tick(&rig.settings).unwrap();
        let elapsed = rig.app.server().simulation().elapsed();
        let (_, requested) = rig.app.capture().unwrap();
        assert_eq!(rig.app.server().simulation().elapsed(), elapsed);
        match requested {
            Some(Progress::Pending) => {
                acquired.push(committed.sim_time);
                info(
                    infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
                    committed.sim_time,
                );
                rig.frame_ready();
                assert_eq!(rig.app.pump().unwrap(), Some(Progress::ImagePublished));
                image(
                    images
                        .recv_timeout(Duration::from_secs(5))
                        .unwrap()
                        .unwrap(),
                    committed.sim_time,
                    u64::try_from(acquired.len() - 1).unwrap(),
                );
            }
            None => {}
            other => panic!("unexpected due admission: {other:?}"),
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
        let paused = rig.app.update(&rig.settings).unwrap();
        assert!(paused.info.paused && paused.info.dt.is_zero() && paused.requested.is_none());
        assert_eq!(paused.info.sim_time, Duration::from_millis(200));
    }
    assert!(images.try_recv().unwrap().is_none());
    assert!(infos.try_recv().unwrap().is_none());
    rig.app.shutdown();
    assert!(!rig.app.has_camera_output());
}

#[test]
fn busy_advances_physics_once_and_reset_discards_buffered_pixels_before_rebinding() {
    if isolation::child(
        "m6_depth::busy_advances_physics_once_and_reset_discards_buffered_pixels_before_rebinding",
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
    assert_eq!(first.info.sim_time, Duration::from_millis(1));
    // Deliberately withhold camera pumping while committing normal physics ticks.
    // At the next due time, one IPC transition cannot complete the acquisition.
    for _ in 1..99 {
        rig.app.server_mut().update().unwrap();
    }
    let committed = rig.app.tick(&rig.settings).unwrap();
    assert_eq!(committed.sim_time, Duration::from_millis(100));
    assert_eq!(committed.dt, Duration::from_millis(1));
    assert!(!committed.paused);
    assert!(matches!(
        rig.app.capture(),
        Err(CameraServerError::Camera(CameraOutputError::Busy))
    ));
    assert_eq!(
        rig.app.server().simulation().elapsed(),
        Duration::from_millis(100)
    );
    rig.control(true, false);
    let paused = rig.app.server_mut().update().unwrap();
    assert!(paused.paused && paused.dt.is_zero());
    info(
        infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
        first.info.sim_time,
    );
    rig.frame_ready();
    assert_eq!(rig.app.pump().unwrap(), Some(Progress::ImagePublished));
    let original = image(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap(),
        first.info.sim_time,
        0,
    );
    rig.control(false, false);
    let pending = rig.app.update(&rig.settings).unwrap();
    assert_eq!(pending.requested, Some(Progress::Pending));
    assert_eq!(pending.info.sim_time, Duration::from_millis(101));
    info(
        infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
        pending.info.sim_time,
    );
    rig.frame_ready();
    rig.control(true, true);
    assert!(rig.app.server().io_preparation_required());
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
    let resumed = rig.app.update(&rig.settings).unwrap();
    assert_eq!(resumed.requested, Some(Progress::Pending));
    assert_eq!(
        (resumed.info.epoch, resumed.info.sim_time),
        (1, Duration::from_millis(1))
    );
    info(
        infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
        resumed.info.sim_time,
    );
    rig.frame_ready();
    assert_eq!(rig.app.pump().unwrap(), Some(Progress::ImagePublished));
    let fresh = image(
        images
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap(),
        resumed.info.sim_time,
        0,
    );
    assert_eq!(fresh.data, original.data);
    assert!(images.try_recv().unwrap().is_none());
    rig.app.shutdown();
}

#[test]
fn removing_the_loaded_camera_model_cancels_readback_and_releases_output() {
    if isolation::child(
        "m6_depth::removing_the_loaded_camera_model_cancels_readback_and_releases_output",
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
    let frame = rig.app.update(&rig.settings).unwrap();
    assert_eq!(frame.requested, Some(Progress::Pending));
    info(
        infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
        frame.info.sim_time,
    );
    rig.frame_ready();
    let model = rig
        .app
        .server()
        .simulation()
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::rig"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let key = rig
        .app
        .server()
        .options()
        .topic_key("/world/default/remove")
        .unwrap();
    let replies = rig
        .observer
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
    let before = rig.app.server().counters().removals_completed;
    let deadline = Instant::now() + Duration::from_secs(5);
    while rig.app.server().counters().removals_completed == before {
        rig.app.server_mut().update().unwrap();
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
    assert_eq!(rig.app.server().counters().removals_completed, before + 1);
    assert!(gz_sim_core::entity_by_id(rig.app.server().simulation().world(), model).is_none());
    assert_eq!(rig.app.pump().unwrap(), None);
    assert!(!rig.app.has_camera_output());
    assert!(images.try_recv().unwrap().is_none());
    for _ in 0..3 {
        let frame = rig.app.update(&rig.settings).unwrap();
        assert!(frame.requested.is_none() && frame.pumped.is_none());
    }
    rig.app.shutdown();
}

#[test]
fn far_clipping_is_positive_infinity_and_near_intersections_are_explicitly_refused() {
    if isolation::child(
        "m6_depth::far_clipping_is_positive_infinity_and_near_intersections_are_explicitly_refused",
        90,
    ) {
        return;
    }
    let target_pose = "<pose>3 3 1 0 0 1.5707963267948966</pose>";
    assert_eq!(XML.matches(target_pose).count(), 1);
    {
        // Move the entire box beyond far=10 in the same rotated camera frame.
        let far_xml = XML.replace(target_pose, "<pose>3 10 1 0 0 1.5707963267948966</pose>");
        let mut rig = Rig::with_xml(&far_xml);
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
        let acquired = rig.app.update(&rig.settings).unwrap();
        assert_eq!(acquired.requested, Some(Progress::Pending));
        assert_eq!(acquired.info.sim_time, Duration::from_millis(1));
        info(
            infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
            acquired.info.sim_time,
        );
        rig.frame_ready();
        assert_eq!(rig.app.pump().unwrap(), Some(Progress::ImagePublished));
        let frame = depth_packet(
            images
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .unwrap(),
            acquired.info.sim_time,
            0,
        );
        assert!(
            frame
                .data
                .chunks_exact(4)
                .all(|word| word == f32::INFINITY.to_le_bytes())
        );
        assert_eq!(
            rig.app.server().simulation().elapsed(),
            acquired.info.sim_time
        );
        rig.app.shutdown();
        assert!(!rig.app.has_camera_output());
    }
    {
        // The front face now crosses the near plane. This is an admitted SDF
        // scene but is explicitly outside this renderer's metric-depth domain.
        let near_xml = XML.replace(target_pose, "<pose>3 -1 1 0 0 1.5707963267948966</pose>");
        let mut rig = Rig::with_xml(&near_xml);
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
        let acquired = rig.app.update(&rig.settings).unwrap();
        assert_eq!(acquired.requested, Some(Progress::Pending));
        assert_eq!(acquired.info.sim_time, Duration::from_millis(1));
        info(
            infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
            acquired.info.sim_time,
        );
        let deadline = Instant::now() + Duration::from_secs(20);
        loop {
            match rig.app.pump() {
                Ok(Some(Progress::Pending | Progress::Backpressure)) => thread::yield_now(),
                Err(CameraServerError::Camera(CameraOutputError::Worker {
                    kind,
                    request_id,
                    message,
                })) => {
                    assert_eq!(kind, gz_render_protocol::FailureKind::InvalidSnapshot);
                    assert_eq!(request_id, Some(1));
                    assert_eq!(
                        message,
                        "metric depth requires every box strictly beyond the calibrated near plane"
                    );
                    break;
                }
                other => panic!("near intersection was not explicitly refused: {other:?}"),
            }
            assert!(Instant::now() < deadline, "near rejection timed out");
        }
        assert_eq!(
            rig.app.server().simulation().elapsed(),
            acquired.info.sim_time
        );
        assert!(images.try_recv().unwrap().is_none());
        assert!(infos.try_recv().unwrap().is_none());
        rig.app.shutdown();
        assert!(!rig.app.has_camera_output());
    }
}

struct CliChild {
    child: std::process::Child,
    stdout: std::path::PathBuf,
    stderr: std::path::PathBuf,
}

impl Drop for CliChild {
    fn drop(&mut self) {
        if matches!(self.child.try_wait(), Ok(None)) {
            // This subprocess owns its process group, including its renderer.
            // A failed assertion must not leave either actor behind.
            let pid =
                rustix::process::Pid::from_raw(i32::try_from(self.child.id()).unwrap()).unwrap();
            let _ = rustix::process::kill_process_group(pid, rustix::process::Signal::KILL);
        }
        let _ = self.child.wait();
        if thread::panicking() {
            eprintln!(
                "CLI stdout: {}",
                std::fs::read_to_string(&self.stdout).unwrap_or_default()
            );
            eprintln!(
                "CLI stderr: {}",
                std::fs::read_to_string(&self.stderr).unwrap_or_default()
            );
        }
    }
}

fn cli_control(observer: &zenoh::Session, key: &str, pause: bool) {
    let replies = observer
        .get(key)
        .payload(
            WorldControl {
                pause,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
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
}

#[test]
fn actual_cli_camera_worker_publishes_depth_and_finishes_its_exact_iteration_budget() {
    use std::{
        fs,
        os::unix::process::CommandExt,
        path::Path,
        process::{Command, Stdio},
    };
    if isolation::child(
        "m6_depth::actual_cli_camera_worker_publishes_depth_and_finishes_its_exact_iteration_budget",
        90,
    ) {
        return;
    }
    // The normal renderer verifier resolves and hashes both executables from its
    // build-only Cargo JSON, then validates them again after this test run.
    // This test never invokes Cargo or guesses a target directory.
    let cli = fs::canonicalize(
        std::env::var_os("GZ_RENDER_CLI_PATH")
            .expect("renderer verifier must bind the CLI executable"),
    )
    .unwrap();
    let worker = fs::canonicalize(
        std::env::var_os("GZ_RENDER_WORKER_PATH")
            .expect("renderer verifier must bind the worker executable"),
    )
    .unwrap();
    assert_eq!(
        worker,
        fs::canonicalize(env!("CARGO_BIN_EXE_gz-render-worker")).unwrap()
    );
    let bindings: serde_json::Value = serde_json::from_str(
        &std::env::var("GZ_RENDER_EXECUTABLES_JSON")
            .expect("renderer verifier must supply inline Cargo executable bindings"),
    )
    .unwrap();
    for (name, path) in [("gz-sim-rs", &cli), ("gz-render-worker", &worker)] {
        let artifact = &bindings[name]["cargo_artifact"];
        assert_eq!(
            fs::canonicalize(artifact["executable"].as_str().unwrap()).unwrap(),
            *path
        );
        assert_eq!(artifact["target"]["name"], name);
    }
    let artifact = &bindings["gz-sim-rs"]["cargo_artifact"];
    let source = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../gz-sim-cli/src/main.rs")
        .canonicalize()
        .unwrap();
    assert_eq!(
        fs::canonicalize(artifact["target"]["src_path"].as_str().unwrap()).unwrap(),
        source
    );
    assert!(
        artifact["features"]
            .as_array()
            .unwrap()
            .iter()
            .any(|feature| feature == "rgb-camera")
    );

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
    let options = ServerOptions {
        partition: format!("m6-cli-depth-{}", std::process::id()),
        ..Default::default()
    };
    let images = observer
        .declare_subscriber(options.topic_key("/m6/depth/image").unwrap())
        .with(zenoh::handlers::FifoChannel::new(4))
        .wait()
        .unwrap();
    let infos = observer
        .declare_subscriber(options.topic_key("/m6/depth/camera_info").unwrap())
        .with(zenoh::handlers::FifoChannel::new(4))
        .wait()
        .unwrap();
    let control = options.topic_key("/world/default/control").unwrap();
    let directory = tempfile::TempDir::new().unwrap();
    let network = directory.path().join("zenoh.json");
    fs::write(
        &network,
        serde_json::to_vec(&serde_json::json!({
            "mode":"peer", "listen":{"endpoints":[]}, "connect":{"endpoints":locators},
            "scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}}
        }))
        .unwrap(),
    )
    .unwrap();
    let fixture = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../gz-camera-output/tests/fixtures/m6-depth.sdf")
        .canonicalize()
        .unwrap();
    let stdout = directory.path().join("stdout");
    let stderr = directory.path().join("stderr");
    let stdout_file = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&stdout)
        .unwrap();
    let stderr_file = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&stderr)
        .unwrap();
    // As in the existing namespace harness, GNU timeout remains a live group
    // watchdog even if the test itself is killed before Rust guards can drop.
    let child = Command::new("timeout")
        .args(["--signal=KILL", "60s"])
        .arg(cli)
        .arg(fixture)
        .args([
            "--physics-engine",
            "rapier",
            "--server",
            "--run",
            "--iterations",
            "10000",
            "--camera-worker",
        ])
        .arg(worker)
        .env("ZENOH_CONFIG", &network)
        .env("GZ_PARTITION", &options.partition)
        .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
        .env_remove("GZ_RELAY")
        .stdin(Stdio::null())
        .stdout(Stdio::from(stdout_file))
        .stderr(Stdio::from(stderr_file))
        .process_group(0)
        .spawn()
        .unwrap();
    let mut child = CliChild {
        child,
        stdout,
        stderr,
    };
    let metadata = infos.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    let packet = CameraInfo::decode(metadata.payload().to_bytes().as_ref()).unwrap();
    let time = packet.header.as_ref().unwrap().stamp.as_ref().unwrap();
    let acquired = Duration::new(
        u64::try_from(time.sec).unwrap(),
        u32::try_from(time.nsec).unwrap(),
    );
    assert!(acquired > Duration::ZERO && acquired < Duration::from_secs(10));
    info(metadata, acquired);
    // Pause after actual acquisition metadata, keeping the finite run alive
    // through initial GPU startup while the preceding image finishes normally.
    cli_control(&observer, &control, true);
    image(
        images
            .recv_timeout(Duration::from_secs(20))
            .unwrap()
            .unwrap(),
        acquired,
        0,
    );
    assert!(child.child.try_wait().unwrap().is_none());
    cli_control(&observer, &control, false);
    let deadline = Instant::now() + Duration::from_secs(30);
    let status = loop {
        if let Some(status) = child.child.try_wait().unwrap() {
            break status;
        }
        assert!(Instant::now() < deadline, "finite CLI run did not stop");
        thread::sleep(Duration::from_millis(5));
    };
    let stdout = fs::read_to_string(&child.stdout).unwrap();
    let stderr = fs::read_to_string(&child.stderr).unwrap();
    assert!(status.success(), "{status}: {stdout} {stderr}");
    assert_eq!(
        stdout.lines().next(),
        Some("simulation_ns=10000000000 iterations=10000")
    );
    assert!(!stderr.contains("Error:"), "{stderr}");
}
