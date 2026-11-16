// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two real software-renderer owners; no native rendering/Manager parity claim.
#[path = "../../gz-camera-output/tests/support/isolated_camera.rs"]
mod isolation;
#[path = "../../../tests/support/zenoh_peer.rs"]
mod selected_native;
use gz_camera_output::{
    CameraCapture, CameraOutputError, CameraServer, CameraServerError, CameraServerOptions,
    Progress,
};
use gz_render_client::ClientLifetime;
use gz_sim_core::{EntityId, Phase, entity_by_id};
use gz_sim_proto::{
    msgs::{CameraInfo, Header, Image, PixelFormatType, WorldControl, WorldReset},
    prost::Message,
};
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_sdf_step};
use gz_sim_server::ServerOptions;
use std::{
    io::{BufRead, BufReader, Read},
    num::NonZeroUsize,
    process::{Child, Command, Stdio},
    sync::{
        Arc, Mutex,
        atomic::{AtomicU8, Ordering},
        mpsc,
    },
    thread,
    time::{Duration, Instant},
};
use zenoh::Wait;
const XML: &str = include_str!("../../../examples/m63_two_rgb_cameras.sdf");
type Subscriber =
    zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>;
fn configuration(endpoint: Option<&str>) -> zenoh::Config {
    zenoh::Config::from_json5(
        &serde_json::json!({"mode":"peer",
        "listen":{"endpoints":if endpoint.is_none(){vec!["tcp/127.0.0.1:0"]}else{vec![]}},
        "connect":{"endpoints":endpoint.into_iter().collect::<Vec<_>>(),"timeout_ms":2000},
        "scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},
        "transport":{"shared_memory":{"enabled":false}}})
        .to_string(),
    )
    .unwrap()
}
fn topic(owner: usize, image: bool) -> String {
    format!(
        "/m63/{}/{}",
        if owner == 0 { "a" } else { "b" },
        if image { "image" } else { "camera_info" }
    )
}
struct Rig {
    app: CameraServer,
    settings: CameraServerOptions,
    observer: zenoh::Session,
    endpoint: String,
    ids: [u64; 2],
    remove: Arc<AtomicU8>,
}
impl Rig {
    fn new() -> Self {
        Self::with_source(XML)
    }
    fn with_source(source: &str) -> Self {
        let observer = zenoh::open(configuration(None)).wait().unwrap();
        let endpoints: Vec<_> = observer
            .info()
            .locators()
            .wait()
            .into_iter()
            .map(|e| e.to_string())
            .collect();
        assert_eq!(endpoints.len(), 1);
        assert_ne!(endpoints[0], "tcp/127.0.0.1:0");
        let mut sim = load_world_with_sdf_step(
            source,
            "m63-two-rgb.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        sim.set_paused(false);
        let ids = std::array::from_fn(|owner| {
            let name = format!(
                "default::rig_{}::body::camera",
                if owner == 0 { "a" } else { "b" }
            );
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap()
                .get::<EntityId>()
                .unwrap()
                .get()
        });
        assert_ne!(ids[0], ids[1]);
        let remove = Arc::new(AtomicU8::new(0));
        let event = Arc::clone(&remove);
        // This admitted lifecycle system removes the actual sensor entity only.
        // Its model and native body remain; this is not camera world-edit support.
        sim.add_systems(
            Phase::Finalize,
            move |world: &mut bevy_ecs::world::World| {
                let selected = event.swap(0, Ordering::AcqRel);
                if selected != 0 {
                    assert!(selected <= 2);
                    let entity = entity_by_id(world, ids[usize::from(selected - 1)]).unwrap();
                    assert!(world.despawn(entity));
                }
            },
        );
        let settings = CameraServerOptions {
            worker_path: env!("CARGO_BIN_EXE_gz-render-worker").into(),
            origin: [0.0; 3],
            zenoh: configuration(Some(&endpoints[0])),
        };
        let options = ServerOptions {
            partition: format!("m63-pair-{}", std::process::id()),
            ..Default::default()
        };
        let app = CameraServer::new_pair(
            sim,
            "default",
            options,
            configuration(Some(&endpoints[0])),
            NonZeroUsize::new(8).unwrap(),
            &settings,
        )
        .unwrap();
        Self {
            app,
            settings,
            observer,
            endpoint: endpoints[0].clone(),
            ids,
            remove,
        }
    }
    fn subscriber(&self, owner: usize, image: bool) -> Subscriber {
        self.observer
            .declare_subscriber(
                self.app
                    .server()
                    .options()
                    .topic_key(&topic(owner, image))
                    .unwrap(),
            )
            .wait()
            .unwrap()
    }
    fn matching(&mut self, expected: [(bool, bool); 2]) {
        let until = Instant::now() + Duration::from_secs(5);
        while self
            .ids
            .iter()
            .copied()
            .zip(expected)
            .any(|(id, value)| self.app.camera_demand_for(id).unwrap() != Some(value))
        {
            assert!(
                Instant::now() < until,
                "pair matching is advisory; actual packets are checked separately"
            );
            thread::yield_now();
        }
    }
    fn pause(&mut self, pause: bool) {
        assert!(self.app.server_mut().apply_local_control(WorldControl {
            pause,
            ..Default::default()
        }));
    }
    fn tick(&mut self) -> Duration {
        self.app.tick(&self.settings).unwrap().sim_time
    }
    fn advance_to(&mut self, millis: u64) {
        while self.app.server().simulation().elapsed() < Duration::from_millis(millis) {
            self.tick();
        }
        assert_eq!(
            self.app.server().simulation().elapsed(),
            Duration::from_millis(millis)
        );
    }
    fn capture(&mut self) -> Vec<CameraCapture> {
        let before = self.app.server().simulation().elapsed();
        let rows = self.app.capture_all().unwrap();
        assert_eq!(
            self.app.server().simulation().elapsed(),
            before,
            "capture cannot repeat physics"
        );
        assert!(rows.len() <= 2);
        assert_eq!(
            rows.iter()
                .map(|row| row.sensor_id)
                .collect::<std::collections::BTreeSet<_>>()
                .len(),
            rows.len()
        );
        rows
    }
    fn ready(&mut self, selected: &[usize]) {
        let until = Instant::now() + Duration::from_secs(20);
        let mut ready = [false; 2];
        while selected.iter().any(|owner| !ready[*owner]) {
            for &owner in selected {
                if ready[owner] {
                    continue;
                }
                match self.app.pump_camera(self.ids[owner]).unwrap().unwrap() {
                    Progress::FrameReady => ready[owner] = true,
                    Progress::Pending
                    | Progress::Backpressure
                    | Progress::SnapshotAccepted
                    | Progress::CaptureSubmitted => {}
                    other => panic!("unexpected actual pair worker phase: {other:?}"),
                }
            }
            assert!(
                Instant::now() < until,
                "unchanged twenty-second actual readback budget"
            );
            thread::yield_now();
        }
    }
    fn publish(
        &mut self,
        owner: usize,
        images: &Subscriber,
        stamp: Duration,
        sequence: u64,
    ) -> Image {
        assert_eq!(
            self.app.pump_camera(self.ids[owner]).unwrap(),
            Some(Progress::ImagePublished)
        );
        check_image(receive(images, b"gz.msgs.Image"), owner, stamp, sequence)
    }
    fn reset(&mut self) {
        assert!(self.app.server_mut().apply_local_control(WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        }));
        assert!(matches!(
            self.app.pump_camera(self.ids[0]),
            Err(CameraServerError::Server(
                gz_sim_server::Error::IoPreparationRequired
            ))
        ));
        assert!(!self.app.has_camera_output());
        self.app.prepare_io(&self.settings).unwrap();
        let committed = self.app.tick(&self.settings).unwrap();
        assert!(committed.sim_time.is_zero() && committed.dt.is_zero());
        for row in self.capture() {
            assert!(row.result.unwrap().requested.is_none());
        }
        assert!(self.app.server().simulation().elapsed().is_zero());
        for id in self.ids {
            assert!(entity_by_id(self.app.server().simulation().world(), id).is_some());
        }
    }
}
fn requested(rows: &[CameraCapture], id: u64) -> Option<Progress> {
    rows.iter()
        .find(|row| row.sensor_id == id)
        .unwrap()
        .result
        .as_ref()
        .unwrap()
        .requested
}
fn receive(subscriber: &Subscriber, attachment: &[u8]) -> Vec<u8> {
    let sample = subscriber
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap();
    assert_eq!(sample.attachment().unwrap().to_bytes().as_ref(), attachment);
    assert!(sample.payload().len() <= 16384);
    sample.payload().to_bytes().into_owned()
}
fn check_header(header: &Header, owner: usize, stamp: Duration, sequence: Option<u64>) {
    let actual = header.stamp.as_ref().unwrap();
    assert_eq!(
        (actual.sec, actual.nsec),
        (stamp.as_secs() as i64, stamp.subsec_nanos() as i32)
    );
    assert_eq!(header.data.len(), if sequence.is_some() { 2 } else { 1 });
    assert_eq!(header.data[0].key, "frame_id");
    assert_eq!(
        header.data[0].value,
        [format!(
            "rig_{}/body/camera",
            if owner == 0 { "a" } else { "b" }
        )]
    );
    if let Some(sequence) = sequence {
        assert_eq!(header.data[1].key, "seq");
        assert_eq!(header.data[1].value, [sequence.to_string()]);
    }
}
fn check_image(wire: Vec<u8>, owner: usize, stamp: Duration, sequence: u64) -> Image {
    let image = Image::decode(wire.as_slice()).unwrap();
    assert_eq!(image.encode_to_vec(), wire);
    check_header(image.header.as_ref().unwrap(), owner, stamp, Some(sequence));
    assert_eq!((image.width, image.height, image.step), (64, 64, 192));
    assert_eq!(image.pixel_format_type, PixelFormatType::RgbInt8 as i32);
    assert_eq!(image.data.len(), 64 * 64 * 3);
    let pixel = |x: usize, y: usize| &image.data[(y * 64 + x) * 3..(y * 64 + x) * 3 + 3];
    assert_eq!(
        pixel(32, 32),
        if owner == 0 {
            &[255, 0, 0]
        } else {
            &[0, 0, 255]
        }
    );
    assert_eq!(pixel(24, 24), &[0, 255, 0], "camera +Y left/+Z up");
    assert_eq!(
        pixel(40, 40),
        if owner == 0 {
            &[0, 0, 255]
        } else {
            &[255, 0, 0]
        }
    );
    image
}
fn check_info(wire: Vec<u8>, owner: usize, stamp: Duration) -> CameraInfo {
    let info = CameraInfo::decode(wire.as_slice()).unwrap();
    assert_eq!(info.encode_to_vec(), wire);
    check_header(info.header.as_ref().unwrap(), owner, stamp, None);
    assert_eq!((info.width, info.height), (64, 64));
    let k = &info.intrinsics.as_ref().unwrap().k;
    assert_eq!(k.len(), 9);
    for (actual, expected) in k
        .iter()
        .zip([32.0, 0.0, 32.0, 0.0, 32.0, 32.0, 0.0, 0.0, 1.0])
    {
        assert!((*actual - expected).abs() < 1e-12);
    }
    assert_eq!(
        info.distortion.as_ref().unwrap().model,
        gz_sim_proto::msgs::camera_info::distortion::DistortionModelType::PlumbBob as i32
    );
    assert_eq!(info.distortion.as_ref().unwrap().k, [0.0; 5]);
    assert_eq!(
        info.rectification_matrix,
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    );
    let projection = &info.projection.as_ref().unwrap().p;
    assert_eq!(projection.len(), 12);
    for (actual, expected) in projection.iter().zip([
        32.0, 0.0, 32.0, 0.0, 0.0, 32.0, 32.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    ]) {
        assert!((*actual - expected).abs() < 1e-12);
    }
    info
}
fn proc_identity(pid: u32) -> Option<(u64, char, u32)> {
    match std::fs::File::open(format!("/proc/{pid}/stat")) {
        Ok(file) => {
            let mut stat = String::new();
            file.take(8193).read_to_string(&mut stat).unwrap();
            assert!(stat.len() <= 8192);
            let (_, tail) = stat.rsplit_once(") ").unwrap();
            let fields: Vec<_> = tail.split_whitespace().collect();
            Some((
                fields[19].parse().unwrap(),
                fields[0].chars().next().unwrap(),
                fields[1].parse().unwrap(),
            ))
        }
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => None,
        Err(error) => panic!("actual owned worker identity read failed: {error}"),
    }
}
struct Worker {
    pid: u32,
    start: u64,
    lifetime: ClientLifetime,
}
fn worker(rig: &Rig, owner: usize) -> Worker {
    let (pid, lifetime) = rig.app.camera_worker_diagnostics(rig.ids[owner]).unwrap();
    let pid = pid.expect("actual worker was started for this owner");
    let (start, state, parent) = proc_identity(pid).unwrap();
    assert!(!matches!(state, 'Z' | 'X'));
    assert_eq!(parent, std::process::id());
    Worker {
        pid,
        start,
        lifetime,
    }
}
fn retired(worker: &Worker) {
    let until = Instant::now() + Duration::from_secs(5);
    loop {
        let current = proc_identity(worker.pid);
        let gone = current.is_none_or(|(start, _, _)| start != worker.start);
        if gone && worker.lifetime.is_retired() {
            return;
        }
        assert!(
            Instant::now() < until,
            "owned worker PID={} start={} current={current:?}; weak_retired={}; worker exit code remains UNKNOWN",
            worker.pid,
            worker.start,
            worker.lifetime.is_retired()
        );
        thread::yield_now();
    }
}

#[derive(Default)]
struct Captured {
    bytes: Vec<u8>,
    error: Option<String>,
}
struct Native {
    child: Child,
    lines: mpsc::Receiver<String>,
    stdout: Arc<Mutex<Captured>>,
    stderr: Arc<Mutex<Captured>>,
    readers: Vec<thread::JoinHandle<()>>,
    capture: std::path::PathBuf,
    _config: tempfile::TempDir,
}
impl Native {
    fn start(rig: &Rig) -> Self {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .parent()
            .unwrap();
        let peer = selected_native::NativePeer::build(root, "camera");
        let config = tempfile::tempdir().unwrap();
        let path = config.path().join("client.json");
        std::fs::write(&path, serde_json::json!({"mode":"client","listen":{"endpoints":[]},"connect":{"endpoints":[rig.endpoint],"timeout_ms":2000},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},"transport":{"shared_memory":{"enabled":false}}}).to_string()).unwrap();
        let mut command = Command::new(&peer.binary);
        peer.configure_runtime(&mut command);
        command
            .arg("pair")
            .env("GZ_IP", "127.0.0.1")
            .env("GZ_PARTITION", &rig.app.server().options().partition)
            .env("ZENOH_CONFIG", path)
            .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
            .env_remove("GZ_TRANSPORT_USERNAME")
            .env_remove("GZ_TRANSPORT_PASSWORD")
            .env_remove("GZ_RELAY")
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        let mut child = command.spawn().unwrap();
        let (tx, lines) = mpsc::sync_channel(8);
        let stdout = Arc::new(Mutex::new(Captured::default()));
        let stderr = Arc::new(Mutex::new(Captured::default()));
        let out = child.stdout.take().unwrap();
        let err = child.stderr.take().unwrap();
        let saved_out = Arc::clone(&stdout);
        let saved_err = Arc::clone(&stderr);
        let readers = vec![
            thread::spawn(move || {
                let mut reader = BufReader::new(out).take(1_048_577);
                loop {
                    let mut line = Vec::new();
                    match reader.read_until(b'\n', &mut line) {
                        Ok(0) => break,
                        Ok(_) => {
                            let mut saved = saved_out.lock().unwrap();
                            if saved.bytes.len() + line.len() > 1_048_576 || line.len() > 65536 {
                                saved.error = Some("bounded native stdout exceeded".into());
                                break;
                            }
                            saved.bytes.extend_from_slice(&line);
                            match String::from_utf8(line) {
                                Ok(line) => {
                                    if tx.try_send(line.trim_end_matches('\n').to_owned()).is_err()
                                    {
                                        saved.error = Some("native phase queue exceeded".into());
                                        break;
                                    }
                                }
                                Err(error) => {
                                    saved.error = Some(error.to_string());
                                    break;
                                }
                            }
                        }
                        Err(error) => {
                            saved_out.lock().unwrap().error = Some(error.to_string());
                            break;
                        }
                    }
                }
            }),
            thread::spawn(move || {
                let mut bytes = Vec::new();
                let result = err.take(1_048_577).read_to_end(&mut bytes);
                let mut saved = saved_err.lock().unwrap();
                if bytes.len() > 1_048_576 {
                    saved.error = Some("bounded native stderr exceeded".into());
                } else if let Err(error) = result {
                    saved.error = Some(error.to_string());
                }
                saved.bytes = bytes;
            }),
        ];
        Self {
            child,
            lines,
            stdout,
            stderr,
            readers,
            capture: peer.binary.parent().unwrap().to_owned(),
            _config: config,
        }
    }
    fn line(&self) -> String {
        for saved in [&self.stdout, &self.stderr] {
            assert!(saved.lock().unwrap().error.is_none());
        }
        self.lines
            .recv_timeout(Duration::from_secs(20))
            .expect("bounded actual native pair phase")
    }
    fn finish(&mut self, ids: [u64; 2]) {
        let until = Instant::now() + Duration::from_secs(5);
        let status = loop {
            if let Some(status) = self.child.try_wait().unwrap() {
                break status;
            }
            assert!(
                Instant::now() < until,
                "actual native pair wait did not complete"
            );
            thread::yield_now();
        };
        while self.readers.iter().any(|reader| !reader.is_finished()) {
            assert!(Instant::now() < until);
            thread::yield_now();
        }
        for reader in self.readers.drain(..) {
            reader.join().unwrap();
        }
        let out = self.stdout.lock().unwrap();
        let err = self.stderr.lock().unwrap();
        std::fs::write(self.capture.join("two-rgb-camera.stdout"), &out.bytes).unwrap();
        std::fs::write(self.capture.join("two-rgb-camera.stderr"), &err.bytes).unwrap();
        std::fs::write(self.capture.join("two-rgb-camera-receipt.json"), serde_json::to_vec_pretty(&serde_json::json!({"actual_exit_code":status.code(),"sensor_ids":ids,"mode":"pair","actual_four_typed_receipts":true,"worker_exit_code":"UNKNOWN","scope":"Actual C++ Image/CameraInfo pair receipt; not native rendering/Manager parity"})).unwrap()).unwrap();
        assert!(
            status.success(),
            "actual CPP wait={status:?} stdout={} stderr={}",
            String::from_utf8_lossy(&out.bytes),
            String::from_utf8_lossy(&err.bytes)
        );
        assert!(out.error.is_none() && err.error.is_none());
        assert!(
            err.bytes.is_empty(),
            "{}",
            String::from_utf8_lossy(&err.bytes)
        );
    }
}
impl Drop for Native {
    fn drop(&mut self) {
        if matches!(self.child.try_wait(), Ok(None)) {
            let _ = self.child.kill();
        }
        let until = Instant::now() + Duration::from_secs(5);
        loop {
            match self.child.try_wait() {
                Ok(Some(_)) => break,
                Ok(None) if Instant::now() < until => thread::yield_now(),
                other => {
                    eprintln!("owned CPP cleanup actual wait UNKNOWN: {other:?}");
                    break;
                }
            }
        }
        for saved in [&self.stdout, &self.stderr] {
            let saved = saved.lock().unwrap();
            if std::thread::panicking() {
                eprintln!(
                    "native pair partial bytes={} error={:?}: {}",
                    saved.bytes.len(),
                    saved.error,
                    String::from_utf8_lossy(&saved.bytes)
                );
            }
        }
    }
}
fn decode_hex(text: &str) -> Vec<u8> {
    assert!(text.len() <= 32768 && text.len().is_multiple_of(2));
    text.as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let digit = |c| match c {
                b'0'..=b'9' => c - b'0',
                b'a'..=b'f' => c - b'a' + 10,
                _ => panic!("actual native hex must be canonical"),
            };
            digit(pair[0]) * 16 + digit(pair[1])
        })
        .collect()
}

#[test]
fn two_real_rgb_views_keep_pixels_stamps_sequences_and_owned_shutdown() {
    if isolation::child(
        "two_real_rgb_views_keep_pixels_stamps_sequences_and_owned_shutdown",
        180,
    ) {
        return;
    }
    let mut rig = Rig::new();
    let images: [Subscriber; 2] = std::array::from_fn(|owner| rig.subscriber(owner, true));
    let infos: [Subscriber; 2] = std::array::from_fn(|owner| rig.subscriber(owner, false));
    let mut native = Native::start(&rig);
    assert_eq!(native.line(), "ready pair");
    rig.matching([(true, true); 2]);
    let first = rig.tick();
    assert_eq!(first, Duration::from_millis(1));
    let rows = rig.capture();
    assert_eq!(rows.len(), 2);
    for id in rig.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    let first_infos: [CameraInfo; 2] = std::array::from_fn(|owner| {
        check_info(receive(&infos[owner], b"gz.msgs.CameraInfo"), owner, first)
    });
    rig.ready(&[0, 1]);
    let workers = [worker(&rig, 0), worker(&rig, 1)];
    assert_ne!(workers[0].pid, workers[1].pid);
    let first_images: [Image; 2] =
        std::array::from_fn(|owner| rig.publish(owner, &images[owner], first, 0));
    assert_ne!(first_images[0].data, first_images[1].data);
    for (owner, (image, info)) in first_images.iter().zip(&first_infos).enumerate() {
        for (kind, expected) in [
            ("IMAGE", image.encode_to_vec()),
            ("INFO", info.encode_to_vec()),
        ] {
            let line = native.line();
            let fields: Vec<_> = line.split(' ').collect();
            assert_eq!(fields.len(), 3);
            assert_eq!(fields[..2], [kind, &owner.to_string()]);
            assert_eq!(decode_hex(fields[2]), expected);
        }
    }
    assert_eq!(native.line(), "PASS canonical pair cameras");
    native.finish(rig.ids);
    let mut acquisitions: [Vec<Duration>; 2] = std::array::from_fn(|_| vec![first]);
    while rig.app.server().simulation().elapsed() < Duration::from_millis(200) {
        let stamp = rig.tick();
        let rows = rig.capture();
        let mut due = Vec::new();
        for (owner, &id) in rig.ids.iter().enumerate() {
            match requested(&rows, id) {
                Some(Progress::Pending) => {
                    check_info(receive(&infos[owner], b"gz.msgs.CameraInfo"), owner, stamp);
                    due.push(owner);
                }
                None => {}
                other => panic!("unexpected pair cadence admission:{other:?}"),
            }
        }
        rig.ready(&due);
        for owner in due {
            rig.publish(
                owner,
                &images[owner],
                stamp,
                acquisitions[owner].len() as u64,
            );
            acquisitions[owner].push(stamp);
        }
    }
    assert_eq!(acquisitions[0], [1, 100, 200].map(Duration::from_millis));
    assert_eq!(
        acquisitions[1],
        [1, 50, 100, 150, 200].map(Duration::from_millis)
    );
    rig.pause(true);
    for _ in 0..16 {
        assert_eq!(rig.tick(), Duration::from_millis(200));
        let rows = rig.capture();
        for id in rig.ids {
            assert_eq!(requested(&rows, id), None);
        }
    }
    for subscriber in images.iter().chain(&infos) {
        assert!(subscriber.try_recv().unwrap().is_none());
    }
    rig.app.shutdown();
    assert!(!rig.app.has_camera_output());
    for witness in &workers {
        retired(witness);
    }
    rig.observer.close().wait().unwrap();
}

#[test]
fn pair_info_only_then_independent_image_demand_starts_only_selected_workers() {
    if isolation::child(
        "pair_info_only_then_independent_image_demand_starts_only_selected_workers",
        180,
    ) {
        return;
    }
    let mut rig = Rig::new();
    assert_eq!(rig.tick(), Duration::from_millis(1));
    for row in rig.capture() {
        assert_eq!(row.result.unwrap().requested, Some(Progress::NoDemand));
    }
    for id in rig.ids {
        assert!(rig.app.camera_worker_diagnostics(id).is_none());
    }
    let infos: [Subscriber; 2] = std::array::from_fn(|owner| rig.subscriber(owner, false));
    rig.matching([(false, true); 2]);
    let stamp = rig.tick();
    assert_eq!(stamp, Duration::from_millis(2));
    for row in rig.capture() {
        assert_eq!(row.result.unwrap().requested, Some(Progress::InfoPublished));
    }
    for (owner, subscriber) in infos.iter().enumerate() {
        check_info(receive(subscriber, b"gz.msgs.CameraInfo"), owner, stamp);
        assert!(rig.app.camera_worker_diagnostics(rig.ids[owner]).is_none());
    }
    let image_a = rig.subscriber(0, true);
    rig.matching([(true, true), (false, true)]);
    for millis in 3..=100 {
        let stamp = rig.tick();
        assert_eq!(stamp, Duration::from_millis(millis));
        let rows = rig.capture();
        for (owner, subscriber) in infos.iter().enumerate() {
            match requested(&rows, rig.ids[owner]) {
                Some(Progress::InfoPublished) => {
                    assert_eq!(owner, 1);
                    check_info(receive(subscriber, b"gz.msgs.CameraInfo"), owner, stamp);
                }
                Some(Progress::Pending) => {
                    assert_eq!((owner, millis), (0, 100));
                    check_info(receive(subscriber, b"gz.msgs.CameraInfo"), owner, stamp);
                }
                None => {}
                other => panic!("unexpected demand admission:{other:?}"),
            }
        }
    }
    rig.ready(&[0]);
    let a = worker(&rig, 0);
    rig.publish(0, &image_a, Duration::from_millis(100), 0);
    assert!(rig.app.camera_worker_diagnostics(rig.ids[1]).is_none());
    let image_b = rig.subscriber(1, true);
    rig.matching([(true, true); 2]);
    rig.advance_to(150);
    let rows = rig.capture();
    assert_eq!(requested(&rows, rig.ids[0]), None);
    assert_eq!(requested(&rows, rig.ids[1]), Some(Progress::Pending));
    check_info(
        receive(&infos[1], b"gz.msgs.CameraInfo"),
        1,
        Duration::from_millis(150),
    );
    rig.ready(&[1]);
    let b = worker(&rig, 1);
    rig.publish(1, &image_b, Duration::from_millis(150), 0);
    assert_ne!(a.pid, b.pid);
    drop(image_a);
    rig.matching([(false, true), (true, true)]);
    assert_eq!(
        rig.app.pump_camera(rig.ids[0]).unwrap(),
        Some(Progress::RetireRequired)
    );
    retired(&a);
    assert!(!b.lifetime.is_retired());
    assert_eq!(proc_identity(b.pid).unwrap().0, b.start);
    assert!(rig.app.camera_worker_diagnostics(rig.ids[0]).is_none());
    rig.advance_to(200);
    let rows = rig.capture();
    assert_eq!(requested(&rows, rig.ids[0]), Some(Progress::InfoPublished));
    assert_eq!(requested(&rows, rig.ids[1]), Some(Progress::Pending));
    for (owner, subscriber) in infos.iter().enumerate() {
        check_info(
            receive(subscriber, b"gz.msgs.CameraInfo"),
            owner,
            Duration::from_millis(200),
        );
    }
    rig.ready(&[1]);
    rig.publish(1, &image_b, Duration::from_millis(200), 1);
    assert!(rig.app.camera_worker_diagnostics(rig.ids[0]).is_none());
    rig.app.shutdown();
    retired(&b);
    assert!(!rig.app.has_camera_output());
    rig.observer.close().wait().unwrap();
}

#[test]
fn pair_busy_reset_and_sensor_retirement_preserve_the_other_real_camera() {
    if isolation::child(
        "pair_busy_reset_and_sensor_retirement_preserve_the_other_real_camera",
        180,
    ) {
        return;
    }
    let mut rig = Rig::new();
    let image_a = rig.subscriber(0, true);
    let infos: [Subscriber; 2] = std::array::from_fn(|owner| rig.subscriber(owner, false));
    rig.matching([(true, true), (false, true)]);
    let first = rig.tick();
    let rows = rig.capture();
    assert_eq!(requested(&rows, rig.ids[0]), Some(Progress::Pending));
    assert_eq!(requested(&rows, rig.ids[1]), Some(Progress::InfoPublished));
    for (owner, subscriber) in infos.iter().enumerate() {
        check_info(receive(subscriber, b"gz.msgs.CameraInfo"), owner, first);
    }
    let image_b = rig.subscriber(1, true);
    rig.matching([(true, true); 2]);
    // Withhold capture while physics commits. Owner A retains its original slot;
    // owner B's missed intervals coalesce rather than creating a backlog.
    rig.advance_to(100);
    let rows = rig.capture();
    let a = rows.iter().find(|row| row.sensor_id == rig.ids[0]).unwrap();
    assert!(matches!(
        &a.result,
        Err(CameraServerError::Camera(CameraOutputError::Busy))
    ));
    assert_eq!(requested(&rows, rig.ids[1]), Some(Progress::Pending));
    assert_eq!(
        rig.app.server().simulation().elapsed(),
        Duration::from_millis(100)
    );
    check_info(
        receive(&infos[1], b"gz.msgs.CameraInfo"),
        1,
        Duration::from_millis(100),
    );
    rig.ready(&[0, 1]);
    rig.publish(0, &image_a, first, 0);
    rig.publish(1, &image_b, Duration::from_millis(100), 0);
    let old = [worker(&rig, 0), worker(&rig, 1)];
    rig.advance_to(200);
    let rows = rig.capture();
    for id in rig.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, subscriber) in infos.iter().enumerate() {
        check_info(
            receive(subscriber, b"gz.msgs.CameraInfo"),
            owner,
            Duration::from_millis(200),
        );
    }
    rig.ready(&[0, 1]);
    rig.reset();
    for witness in &old {
        retired(witness);
    }
    assert_eq!(rig.app.server().simulation().epoch(), 1);
    assert!(
        image_a.try_recv().unwrap().is_none() && image_b.try_recv().unwrap().is_none(),
        "old buffered images cannot escape reset"
    );
    rig.matching([(true, true); 2]);
    rig.pause(false);
    assert_eq!(rig.tick(), Duration::from_millis(1));
    let rows = rig.capture();
    for id in rig.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, subscriber) in infos.iter().enumerate() {
        check_info(
            receive(subscriber, b"gz.msgs.CameraInfo"),
            owner,
            Duration::from_millis(1),
        );
    }
    rig.ready(&[0, 1]);
    rig.publish(0, &image_a, Duration::from_millis(1), 0);
    rig.publish(1, &image_b, Duration::from_millis(1), 0);
    let current = [worker(&rig, 0), worker(&rig, 1)];
    rig.advance_to(100);
    let rows = rig.capture();
    for id in rig.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, subscriber) in infos.iter().enumerate() {
        check_info(
            receive(subscriber, b"gz.msgs.CameraInfo"),
            owner,
            Duration::from_millis(100),
        );
    }
    rig.ready(&[0, 1]);
    rig.pause(true);
    let model = rig
        .app
        .server()
        .simulation()
        .world()
        .iter_entities()
        .find(|row| {
            row.get::<Name>()
                .is_some_and(|name| name.0 == "default::rig_a")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let before = rig.app.server().simulation().elapsed();
    assert!(matches!(
        rig.app.server_mut().remove_editable_model(model),
        Err(gz_sim_server::Error::CameraWorldEdit)
    ));
    assert_eq!(rig.app.server().simulation().elapsed(), before);
    rig.remove.store(1, Ordering::Release);
    assert_eq!(rig.tick(), before);
    assert!(entity_by_id(rig.app.server().simulation().world(), rig.ids[0]).is_none());
    rig.publish(1, &image_b, Duration::from_millis(100), 1);
    retired(&current[0]);
    assert!(!current[1].lifetime.is_retired());
    assert_eq!(proc_identity(current[1].pid).unwrap().0, current[1].start);
    assert!(
        image_a.try_recv().unwrap().is_none(),
        "removed owner cannot publish its held frame"
    );
    assert_eq!(rig.app.pump_camera(rig.ids[0]).unwrap(), None);
    rig.remove.store(2, Ordering::Release);
    assert_eq!(rig.tick(), before);
    assert_eq!(rig.app.pump_camera(rig.ids[1]).unwrap(), None);
    retired(&current[1]);
    assert!(!rig.app.has_camera_output());
    rig.reset();
    assert_eq!(rig.app.server().simulation().epoch(), 2);
    rig.matching([(true, true); 2]);
    rig.pause(false);
    assert_eq!(rig.tick(), Duration::from_millis(1));
    let rows = rig.capture();
    for id in rig.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, subscriber) in infos.iter().enumerate() {
        check_info(
            receive(subscriber, b"gz.msgs.CameraInfo"),
            owner,
            Duration::from_millis(1),
        );
    }
    rig.ready(&[0, 1]);
    rig.publish(0, &image_a, Duration::from_millis(1), 0);
    rig.publish(1, &image_b, Duration::from_millis(1), 0);
    let final_workers = [worker(&rig, 0), worker(&rig, 1)];
    rig.app.shutdown();
    for witness in &final_workers {
        retired(witness);
    }
    assert!(!rig.app.has_camera_output());
    rig.observer.close().wait().unwrap();
}

#[test]
fn two_real_cylinder_views_preserve_pair_cadence_reset_and_owned_retirement() {
    if isolation::child(
        "two_real_cylinder_views_preserve_pair_cadence_reset_and_owned_retirement",
        180,
    ) {
        return;
    }
    // Change only the centre VISUAL geometry and its local pose. Collision
    // boxes, marker visuals, camera models/poses/topics and 10/20Hz stay exact.
    let mut source = XML.to_owned();
    for name in ["red", "blue"] {
        let old =
            format!("<visual name=\"{name}\"><geometry><box><size>2 2 2</size></box></geometry>");
        let new = format!(
            "<visual name=\"{name}\"><pose>0 0 0 0 1.5707963267948966 0</pose><geometry><cylinder><radius>1</radius><length>2</length></cylinder></geometry>"
        );
        assert_eq!(source.matches(&old).count(), 1);
        source = source.replace(&old, &new);
    }
    let mut rig = Rig::with_source(&source);
    let images: [Subscriber; 2] = std::array::from_fn(|owner| rig.subscriber(owner, true));
    let infos: [Subscriber; 2] = std::array::from_fn(|owner| rig.subscriber(owner, false));
    let cap = |image: &Image, owner: usize| {
        let pixel = |x: usize, y: usize| &image.data[(y * 64 + x) * 3..(y * 64 + x) * 3 + 3];
        let color = if owner == 0 {
            &[255, 0, 0]
        } else {
            &[0, 0, 255]
        };
        assert_eq!(pixel(32, 24), color, "actual cap interior");
        assert_eq!(pixel(40, 32), color, "actual cap interior");
        // Off-diagonal corners avoid both retained marker quadrants. These
        // pixels are inside a proxy Box and outside the circular near cap.
        assert_eq!(
            pixel(40, 23),
            &[43, 44, 47],
            "cylinder must not become a Box proxy"
        );
        assert_eq!(
            pixel(23, 40),
            &[43, 44, 47],
            "cylinder must not become a Box proxy"
        );
    };
    rig.matching([(true, true); 2]);
    let first = rig.tick();
    assert_eq!(first, Duration::from_millis(1));
    let rows = rig.capture();
    assert_eq!(rows.len(), 2);
    for id in rig.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, subscriber) in infos.iter().enumerate() {
        check_info(receive(subscriber, b"gz.msgs.CameraInfo"), owner, first);
    }
    rig.ready(&[0, 1]);
    let original = [worker(&rig, 0), worker(&rig, 1)];
    assert_ne!(original[0].pid, original[1].pid);
    let frames: [Image; 2] =
        std::array::from_fn(|owner| rig.publish(owner, &images[owner], first, 0));
    for (owner, image) in frames.iter().enumerate() {
        cap(image, owner);
    }
    assert_ne!(frames[0].data, frames[1].data);
    rig.advance_to(50);
    let rows = rig.capture();
    assert_eq!(requested(&rows, rig.ids[0]), None);
    assert_eq!(requested(&rows, rig.ids[1]), Some(Progress::Pending));
    check_info(
        receive(&infos[1], b"gz.msgs.CameraInfo"),
        1,
        Duration::from_millis(50),
    );
    rig.ready(&[1]);
    cap(&rig.publish(1, &images[1], Duration::from_millis(50), 1), 1);
    rig.pause(true);
    for _ in 0..16 {
        assert_eq!(rig.tick(), Duration::from_millis(50));
        for row in rig.capture() {
            assert!(row.result.unwrap().requested.is_none());
        }
    }
    for subscriber in images.iter().chain(&infos) {
        assert!(subscriber.try_recv().unwrap().is_none());
    }
    rig.pause(false);
    rig.advance_to(100);
    let rows = rig.capture();
    for id in rig.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, subscriber) in infos.iter().enumerate() {
        check_info(
            receive(subscriber, b"gz.msgs.CameraInfo"),
            owner,
            Duration::from_millis(100),
        );
    }
    rig.ready(&[0, 1]);
    // Both real shape readbacks are held before publication; reset cancels both.
    rig.reset();
    assert_eq!(rig.app.server().simulation().epoch(), 1);
    for witness in &original {
        retired(witness);
    }
    for subscriber in &images {
        assert!(subscriber.try_recv().unwrap().is_none());
    }
    rig.matching([(true, true); 2]);
    rig.pause(false);
    assert_eq!(rig.tick(), Duration::from_millis(1));
    let rows = rig.capture();
    for id in rig.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, subscriber) in infos.iter().enumerate() {
        check_info(
            receive(subscriber, b"gz.msgs.CameraInfo"),
            owner,
            Duration::from_millis(1),
        );
    }
    rig.ready(&[0, 1]);
    let restored = [worker(&rig, 0), worker(&rig, 1)];
    assert_ne!(restored[0].pid, restored[1].pid);
    for (owner, subscriber) in images.iter().enumerate() {
        cap(
            &rig.publish(owner, subscriber, Duration::from_millis(1), 0),
            owner,
        );
    }
    rig.pause(true);
    rig.remove.store(1, Ordering::Release);
    assert_eq!(rig.tick(), Duration::from_millis(1));
    assert!(entity_by_id(rig.app.server().simulation().world(), rig.ids[0]).is_none());
    assert_eq!(rig.app.pump_camera(rig.ids[0]).unwrap(), None);
    retired(&restored[0]);
    assert!(!restored[1].lifetime.is_retired());
    assert_eq!(proc_identity(restored[1].pid).unwrap().0, restored[1].start);
    rig.pause(false);
    rig.advance_to(50);
    let rows = rig.capture();
    assert_eq!(rows.len(), 1);
    assert_eq!(requested(&rows, rig.ids[1]), Some(Progress::Pending));
    check_info(
        receive(&infos[1], b"gz.msgs.CameraInfo"),
        1,
        Duration::from_millis(50),
    );
    rig.ready(&[1]);
    cap(&rig.publish(1, &images[1], Duration::from_millis(50), 1), 1);
    assert!(images[0].try_recv().unwrap().is_none());
    rig.app.shutdown();
    retired(&restored[1]);
    assert!(!rig.app.has_camera_output());
    rig.observer.close().wait().unwrap();
    // Retired means matching PID/start identity actually absent and weak Client
    // lifetime retired. The application's discarded worker exit code is UNKNOWN.
}
