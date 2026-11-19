// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent RGB/depth software owners, canonical upstream wire consumers.
//! No registration, synchronized exposure, native rendering or throughput claim.
#[path = "../../gz-camera-output/tests/support/isolated_camera.rs"]
mod isolation;
#[path = "support/mixed_native.rs"]
mod native;
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
    io::Read,
    num::NonZeroUsize,
    sync::{
        Arc,
        atomic::{AtomicU8, Ordering},
    },
    thread,
    time::{Duration, Instant},
};
use zenoh::Wait;
const XML: &str = include_str!("../../../examples/m71_mixed_cameras.sdf");
type Subscriber =
    zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>;
fn config(endpoint: Option<&str>) -> zenoh::Config {
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
struct Rig {
    app: CameraServer,
    settings: CameraServerOptions,
    observer: zenoh::Session,
    endpoint: String,
    ids: [u64; 2],
    remove: Arc<AtomicU8>,
}
impl Rig {
    fn new(shared: bool) -> Self {
        let observer = zenoh::open(config(None)).wait().unwrap();
        let endpoints = observer
            .info()
            .locators()
            .wait()
            .into_iter()
            .map(|e| e.to_string())
            .collect::<Vec<_>>();
        assert_eq!(endpoints.len(), 1);
        assert_ne!(endpoints[0], "tcp/127.0.0.1:0");
        let mut sim = load_world_with_sdf_step(
            XML,
            "m71-mixed-cameras.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        sim.set_paused(false);
        let ids = std::array::from_fn(|owner| {
            let wanted = format!(
                "default::rig_{}::body::camera",
                if owner == 0 { "a" } else { "b" }
            );
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == wanted))
                .unwrap()
                .get::<EntityId>()
                .unwrap()
                .get()
        });
        assert_ne!(ids[0], ids[1]);
        let remove = Arc::new(AtomicU8::new(0));
        let event = Arc::clone(&remove);
        sim.add_systems(
            Phase::Finalize,
            move |world: &mut bevy_ecs::world::World| {
                let selected = event.swap(0, Ordering::AcqRel);
                if selected != 0 {
                    assert!(selected <= 2);
                    assert!(
                        world.despawn(entity_by_id(world, ids[usize::from(selected - 1)]).unwrap())
                    );
                }
            },
        );
        let settings = CameraServerOptions {
            worker_path: env!("CARGO_BIN_EXE_gz-render-worker").into(),
            origin: [0.0; 3],
            zenoh: config(Some(&endpoints[0])),
        };
        let options = ServerOptions {
            partition: format!("m71-mixed-{}", std::process::id()),
            ..Default::default()
        };
        let app = if shared {
            CameraServer::new_mixed_pair_with_server_session(
                sim,
                "default",
                options,
                config(Some(&endpoints[0])),
                NonZeroUsize::new(8).unwrap(),
                &settings,
            )
        } else {
            CameraServer::new_mixed_pair(
                sim,
                "default",
                options,
                config(Some(&endpoints[0])),
                NonZeroUsize::new(8).unwrap(),
                &settings,
            )
        }
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
    fn sub(&self, owner: usize, image: bool) -> Subscriber {
        let topic = format!(
            "/m71/{}/{}",
            if owner == 0 { "rgb" } else { "depth" },
            if image { "image" } else { "camera_info" }
        );
        self.observer
            .declare_subscriber(self.app.server().options().topic_key(&topic).unwrap())
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
                "matching is advisory; actual packets are checked independently"
            );
            thread::yield_now();
        }
    }
    fn tick(&mut self) -> Duration {
        self.app.tick(&self.settings).unwrap().sim_time
    }
    fn advance(&mut self, millis: u64) {
        while self.app.server().simulation().elapsed() < Duration::from_millis(millis) {
            self.tick();
        }
        assert_eq!(
            self.app.server().simulation().elapsed(),
            Duration::from_millis(millis)
        );
    }
    fn pause(&mut self, paused: bool) {
        assert!(self.app.server_mut().apply_local_control(WorldControl {
            pause: paused,
            ..Default::default()
        }));
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
                .map(|r| r.sensor_id)
                .collect::<std::collections::BTreeSet<_>>()
                .len(),
            rows.len()
        );
        rows
    }
    fn ready(&mut self, owners: &[usize]) {
        let until = Instant::now() + Duration::from_secs(20);
        let mut ready = [false; 2];
        while owners.iter().any(|i| !ready[*i]) {
            for &owner in owners {
                if ready[owner] {
                    continue;
                }
                match self.app.pump_camera(self.ids[owner]).unwrap().unwrap() {
                    Progress::FrameReady => ready[owner] = true,
                    Progress::Pending
                    | Progress::Backpressure
                    | Progress::SnapshotAccepted
                    | Progress::CaptureSubmitted => {}
                    other => panic!("unexpected mixed worker phase {other:?}"),
                }
            }
            assert!(
                Instant::now() < until,
                "unchanged twenty-second readback budget"
            );
            thread::yield_now();
        }
    }
    fn publish(&mut self, owner: usize, sub: &Subscriber, stamp: Duration, sequence: u64) -> Image {
        assert_eq!(
            self.app.pump_camera(self.ids[owner]).unwrap(),
            Some(Progress::ImagePublished)
        );
        image(receive(sub, b"gz.msgs.Image"), owner, stamp, sequence)
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
        let step = self.app.tick(&self.settings).unwrap();
        assert!(step.sim_time.is_zero() && step.dt.is_zero());
        for row in self.capture() {
            assert!(row.result.unwrap().requested.is_none());
        }
        for id in self.ids {
            assert!(entity_by_id(self.app.server().simulation().world(), id).is_some());
        }
    }
}
fn requested(rows: &[CameraCapture], id: u64) -> Option<Progress> {
    rows.iter()
        .find(|r| r.sensor_id == id)
        .unwrap()
        .result
        .as_ref()
        .unwrap()
        .requested
}
fn receive(sub: &Subscriber, attachment: &[u8]) -> Vec<u8> {
    let sample = sub.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    assert_eq!(sample.attachment().unwrap().to_bytes().as_ref(), attachment);
    assert!(
        sample.payload().len()
            <= if attachment == b"gz.msgs.Image" {
                32768
            } else {
                16384
            }
    );
    sample.payload().to_bytes().into_owned()
}
fn header(value: &Header, owner: usize, stamp: Duration, sequence: Option<u64>) {
    let time = value.stamp.as_ref().unwrap();
    assert_eq!(
        (time.sec, time.nsec),
        (stamp.as_secs() as i64, stamp.subsec_nanos() as i32)
    );
    assert_eq!(value.data.len(), if sequence.is_some() { 2 } else { 1 });
    assert_eq!(value.data[0].key, "frame_id");
    assert_eq!(
        value.data[0].value,
        [format!(
            "rig_{}/body/camera",
            if owner == 0 { "a" } else { "b" }
        )]
    );
    if let Some(n) = sequence {
        assert_eq!(value.data[1].key, "seq");
        assert_eq!(value.data[1].value, [n.to_string()]);
    }
}
fn info(wire: Vec<u8>, owner: usize, stamp: Duration) -> CameraInfo {
    let v = CameraInfo::decode(wire.as_slice()).unwrap();
    assert_eq!(v.encode_to_vec(), wire);
    header(v.header.as_ref().unwrap(), owner, stamp, None);
    assert_eq!((v.width, v.height), (64, 64));
    let k = &v.intrinsics.as_ref().unwrap().k;
    assert_eq!(k.len(), 9);
    for (a, b) in k
        .iter()
        .zip([32.0, 0.0, 32.0, 0.0, 32.0, 32.0, 0.0, 0.0, 1.0])
    {
        assert!((*a - b).abs() < 1e-12);
    }
    let d = v.distortion.as_ref().unwrap();
    assert_eq!(
        d.model,
        gz_sim_proto::msgs::camera_info::distortion::DistortionModelType::PlumbBob as i32
    );
    assert_eq!(d.k, [0.0; 5]);
    assert_eq!(
        v.rectification_matrix,
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    );
    let p = &v.projection.as_ref().unwrap().p;
    assert_eq!(p.len(), 12);
    for (a, b) in p.iter().zip([
        32.0, 0.0, 32.0, 0.0, 0.0, 32.0, 32.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    ]) {
        assert!((*a - b).abs() < 1e-12);
    }
    v
}
fn image(wire: Vec<u8>, owner: usize, stamp: Duration, sequence: u64) -> Image {
    assert!(wire.len() <= if owner == 0 { 16384 } else { 32768 });
    let v = Image::decode(wire.as_slice()).unwrap();
    assert_eq!(v.encode_to_vec(), wire);
    header(v.header.as_ref().unwrap(), owner, stamp, Some(sequence));
    assert_eq!((v.width, v.height), (64, 64));
    if owner == 0 {
        assert_eq!(
            (v.step, v.pixel_format_type, v.data.len()),
            (192, PixelFormatType::RgbInt8 as i32, 12288)
        );
        let pixel = |x: usize, y: usize| &v.data[(y * 64 + x) * 3..(y * 64 + x) * 3 + 3];
        for y in 30..=33 {
            for x in 30..=33 {
                assert_eq!(pixel(x, y), [255, 0, 0]);
            }
        }
        for (x, y) in [(0, 0), (24, 24), (40, 40), (63, 63)] {
            assert_eq!(pixel(x, y), [0, 0, 255]);
        }
    } else {
        assert_eq!(owner, 1);
        assert_eq!(
            (v.step, v.pixel_format_type, v.data.len()),
            (256, PixelFormatType::RFloat32 as i32, 16384)
        );
        for (i, word) in v.data.chunks_exact(4).enumerate() {
            // Independent literal depth-rig projection and box fronts, not extraction or received calibration.
            let front = 2.8_f64;
            let ray_y = (32.0 - (i % 64) as f64 - 0.5) / 32.0;
            let ray_z = (32.0 - (i / 64) as f64 - 0.5) / 32.0;
            let expected = if (ray_y * front).abs() <= 0.2 && (ray_z * front).abs() <= 0.2 {
                front
            } else {
                5.9
            };
            let actual = f32::from_le_bytes(word.try_into().unwrap());
            assert!(
                actual.is_finite() && actual > 0.0 && (f64::from(actual) - expected).abs() < 1e-4,
                "pixel{i} actual{actual} expected{expected}"
            );
        }
    }
    v
}
fn proc_identity(pid: u32) -> Option<(u64, char, u32)> {
    match std::fs::File::open(format!("/proc/{pid}/stat")) {
        Ok(file) => {
            let mut text = String::new();
            file.take(8193).read_to_string(&mut text).unwrap();
            assert!(text.len() <= 8192);
            let (_, tail) = text.rsplit_once(") ").unwrap();
            let f = tail.split_whitespace().collect::<Vec<_>>();
            Some((
                f[19].parse().unwrap(),
                f[0].chars().next().unwrap(),
                f[1].parse().unwrap(),
            ))
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
        Err(e) => panic!("owned worker identity read failed {e}"),
    }
}
struct Worker {
    pid: u32,
    start: u64,
    lifetime: ClientLifetime,
}
fn worker(r: &Rig, owner: usize) -> Worker {
    let (pid, lifetime) = r.app.camera_worker_diagnostics(r.ids[owner]).unwrap();
    let pid = pid.unwrap();
    let (start, state, parent) = proc_identity(pid).unwrap();
    assert!(!matches!(state, 'Z' | 'X'));
    assert_eq!(parent, std::process::id());
    Worker {
        pid,
        start,
        lifetime,
    }
}
fn retired(w: &Worker) {
    let until = Instant::now() + Duration::from_secs(5);
    while proc_identity(w.pid).is_some() || !w.lifetime.is_retired() {
        assert!(
            Instant::now() < until,
            "owned worker{} start{} not retired; returncode UNKNOWN",
            w.pid,
            w.start
        );
        thread::yield_now();
    }
}
fn hex(text: &str) -> Vec<u8> {
    assert!(text.len() <= 65536 && text.len().is_multiple_of(2));
    text.as_bytes()
        .chunks_exact(2)
        .map(|p| {
            let digit = |b| match b {
                b'0'..=b'9' => b - b'0',
                b'a'..=b'f' => b - b'a' + 10,
                _ => panic!("canonical lowercase hex"),
            };
            digit(p[0]) * 16 + digit(p[1])
        })
        .collect()
}

#[test]
fn mixed_real_workers_keep_canonical_wire_independent_cadence_and_paused_time() {
    if isolation::child(
        "mixed_real_workers_keep_canonical_wire_independent_cadence_and_paused_time",
        180,
    ) {
        return;
    }
    let mut r = Rig::new(false);
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let peer = selected_native::NativePeer::build(root, "camera");
    let mut cpp = [
        native::Native::start(&r.endpoint, &r.app.server().options().partition, &peer, 0),
        native::Native::start(&r.endpoint, &r.app.server().options().partition, &peer, 1),
    ];
    assert_ne!(cpp[0].pid(), cpp[1].pid());
    for (owner, consumer) in cpp.iter().enumerate() {
        assert_eq!(consumer.line(), format!("ready mixed pair {owner}"));
    }
    // ONLY the actual upstream peers create the first remote demand boundary.
    r.matching([(true, true); 2]);
    for id in r.ids {
        assert!(r.app.camera_worker_diagnostics(id).is_none());
    }
    let images: [Subscriber; 2] = std::array::from_fn(|i| r.sub(i, true));
    let infos: [Subscriber; 2] = std::array::from_fn(|i| r.sub(i, false));
    let first = r.tick();
    assert_eq!(first, Duration::from_millis(1));
    let rows = r.capture();
    for id in r.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    let first_infos: [CameraInfo; 2] =
        std::array::from_fn(|i| info(receive(&infos[i], b"gz.msgs.CameraInfo"), i, first));
    r.ready(&[0, 1]);
    let workers = [worker(&r, 0), worker(&r, 1)];
    assert_ne!(workers[0].pid, workers[1].pid);
    let first_images: [Image; 2] = std::array::from_fn(|i| r.publish(i, &images[i], first, 0));
    let mut stdout_bytes = [0; 2];
    for owner in 0..2 {
        for (kind, expected) in [
            ("IMAGE", first_images[owner].encode_to_vec()),
            ("INFO", first_infos[owner].encode_to_vec()),
        ] {
            let line = cpp[owner].line();
            let (actual, raw) = line.split_once(' ').unwrap();
            assert_eq!(actual, kind);
            assert_eq!(hex(raw), expected, "all actual canonical bytes");
        }
        assert_eq!(
            cpp[owner].line(),
            format!("PASS canonical mixed pair {owner}")
        );
        stdout_bytes[owner] = cpp[owner].finish();
        assert!(stdout_bytes[owner] <= 65536);
        assert_eq!(cpp[owner].actual_exit(), Some(0));
    }
    let receipt=serde_json::to_vec_pretty(&serde_json::json!({"actual_consumer_exit_codes":[cpp[0].actual_exit(),cpp[1].actual_exit()],
        "actual_reaped":[true,true],"consumer_PIDs":[cpp[0].pid(),cpp[1].pid()],"sensor_ids":r.ids,"stdout_bytes":stdout_bytes,
        "per_stdout_cap":65536,"per_stderr_cap":4096,"aggregate_cap":131072,"worker_exit_code":"UNKNOWN",
        "scope":"Two actual upstream canonical mixed-camera consumers; software RGB pixels and metric depth, not native rendering or Manager parity."})).unwrap();
    assert!(stdout_bytes.into_iter().sum::<usize>() + receipt.len() <= 131072);
    let mut saved = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(
            peer.binary
                .parent()
                .unwrap()
                .join("m71-two-mixed-consumers.json"),
        )
        .unwrap();
    std::io::Write::write_all(&mut saved, &receipt).unwrap();
    saved.sync_all().unwrap();
    let mut stamps: [Vec<Duration>; 2] = std::array::from_fn(|_| vec![first]);
    while r.app.server().simulation().elapsed() < Duration::from_millis(200) {
        let stamp = r.tick();
        let rows = r.capture();
        let mut due = Vec::new();
        for (owner, subscription) in infos.iter().enumerate() {
            match requested(&rows, r.ids[owner]) {
                Some(Progress::Pending) => {
                    info(receive(subscription, b"gz.msgs.CameraInfo"), owner, stamp);
                    due.push(owner);
                }
                None => {}
                other => panic!("mixed cadence admission{other:?}"),
            }
        }
        r.ready(&due);
        for owner in due {
            r.publish(owner, &images[owner], stamp, stamps[owner].len() as u64);
            stamps[owner].push(stamp);
        }
    }
    assert_eq!(stamps[0], [1, 100, 200].map(Duration::from_millis));
    assert_eq!(stamps[1], [1, 50, 100, 150, 200].map(Duration::from_millis));
    r.pause(true);
    for _ in 0..16 {
        assert_eq!(r.tick(), Duration::from_millis(200));
        for row in r.capture() {
            assert!(row.result.unwrap().requested.is_none());
        }
    }
    for sub in images.iter().chain(&infos) {
        assert!(sub.try_recv().unwrap().is_none());
    }
    r.app.shutdown();
    for w in &workers {
        retired(w);
    }
    assert!(!r.app.has_camera_output());
    r.observer.close().wait().unwrap();
}

#[test]
fn mixed_info_only_and_selected_image_demand_keep_other_worker_lazy() {
    if isolation::child(
        "mixed_info_only_and_selected_image_demand_keep_other_worker_lazy",
        180,
    ) {
        return;
    }
    for selected in 0..2 {
        let mut r = Rig::new(false);
        assert_eq!(r.tick(), Duration::from_millis(1));
        for row in r.capture() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::NoDemand));
        }
        for id in r.ids {
            assert!(r.app.camera_worker_diagnostics(id).is_none());
        }
        let infos: [Subscriber; 2] = std::array::from_fn(|i| r.sub(i, false));
        r.matching([(false, true); 2]);
        let stamp = r.tick();
        assert_eq!(stamp, Duration::from_millis(2));
        for row in r.capture() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::InfoPublished));
        }
        for (owner, subscription) in infos.iter().enumerate() {
            info(receive(subscription, b"gz.msgs.CameraInfo"), owner, stamp);
            assert!(r.app.camera_worker_diagnostics(r.ids[owner]).is_none());
        }
        let images = r.sub(selected, true);
        let mut demand = [(false, true); 2];
        demand[selected] = (true, true);
        r.matching(demand);
        let time = if selected == 0 { 100 } else { 50 };
        r.advance(time);
        let rows = r.capture();
        for (owner, subscription) in infos.iter().enumerate() {
            let expected = if owner == selected {
                Some(Progress::Pending)
            } else if time.is_multiple_of(if owner == 0 { 100 } else { 50 }) {
                Some(Progress::InfoPublished)
            } else {
                None
            };
            assert_eq!(requested(&rows, r.ids[owner]), expected);
            if expected.is_some() {
                info(
                    receive(subscription, b"gz.msgs.CameraInfo"),
                    owner,
                    Duration::from_millis(time),
                );
            }
        }
        r.ready(&[selected]);
        let w = worker(&r, selected);
        r.publish(selected, &images, Duration::from_millis(time), 0);
        assert!(
            r.app
                .camera_worker_diagnostics(r.ids[1 - selected])
                .is_none()
        );
        drop(images);
        r.matching([(false, true); 2]);
        assert_eq!(
            r.app.pump_camera(r.ids[selected]).unwrap(),
            Some(Progress::RetireRequired)
        );
        retired(&w);
        for id in r.ids {
            assert!(r.app.camera_worker_diagnostics(id).is_none());
        }
        assert!(r.app.has_camera_output());
        r.app.shutdown();
        r.observer.close().wait().unwrap();
    }
}

#[test]
fn mixed_busy_reset_removal_and_shared_session_preserve_the_other_owner() {
    if isolation::child(
        "mixed_busy_reset_removal_and_shared_session_preserve_the_other_owner",
        180,
    ) {
        return;
    }
    let mut r = Rig::new(true);
    let image_a = r.sub(0, true);
    let infos: [Subscriber; 2] = std::array::from_fn(|i| r.sub(i, false));
    r.matching([(true, true), (false, true)]);
    let first = r.tick();
    let rows = r.capture();
    assert_eq!(requested(&rows, r.ids[0]), Some(Progress::Pending));
    assert_eq!(requested(&rows, r.ids[1]), Some(Progress::InfoPublished));
    for (owner, subscription) in infos.iter().enumerate() {
        info(receive(subscription, b"gz.msgs.CameraInfo"), owner, first);
    }
    let image_b = r.sub(1, true);
    r.matching([(true, true); 2]);
    r.advance(100);
    let rows = r.capture();
    assert!(matches!(
        &rows
            .iter()
            .find(|v| v.sensor_id == r.ids[0])
            .unwrap()
            .result,
        Err(CameraServerError::Camera(CameraOutputError::Busy))
    ));
    assert_eq!(requested(&rows, r.ids[1]), Some(Progress::Pending));
    assert_eq!(
        r.app.server().simulation().elapsed(),
        Duration::from_millis(100)
    );
    info(
        receive(&infos[1], b"gz.msgs.CameraInfo"),
        1,
        Duration::from_millis(100),
    );
    r.ready(&[0, 1]);
    r.publish(0, &image_a, first, 0);
    r.publish(1, &image_b, Duration::from_millis(100), 0);
    let old = [worker(&r, 0), worker(&r, 1)];
    r.advance(200);
    let rows = r.capture();
    for id in r.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, subscription) in infos.iter().enumerate() {
        info(
            receive(subscription, b"gz.msgs.CameraInfo"),
            owner,
            Duration::from_millis(200),
        );
    }
    r.ready(&[0, 1]);
    r.reset();
    for w in &old {
        retired(w);
    }
    assert_eq!(r.app.server().simulation().epoch(), 1);
    assert!(
        image_a.try_recv().unwrap().is_none() && image_b.try_recv().unwrap().is_none(),
        "reset discards both old buffered images"
    );
    r.matching([(true, true); 2]);
    r.pause(false);
    assert_eq!(r.tick(), Duration::from_millis(1));
    let rows = r.capture();
    for id in r.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, subscription) in infos.iter().enumerate() {
        info(
            receive(subscription, b"gz.msgs.CameraInfo"),
            owner,
            Duration::from_millis(1),
        );
    }
    r.ready(&[0, 1]);
    r.publish(0, &image_a, Duration::from_millis(1), 0);
    r.publish(1, &image_b, Duration::from_millis(1), 0);
    let current = [worker(&r, 0), worker(&r, 1)];
    r.advance(100);
    let rows = r.capture();
    for id in r.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, subscription) in infos.iter().enumerate() {
        info(
            receive(subscription, b"gz.msgs.CameraInfo"),
            owner,
            Duration::from_millis(100),
        );
    }
    r.ready(&[0, 1]);
    r.pause(true);
    let before = r.app.server().simulation().elapsed();
    r.remove.store(1, Ordering::Release);
    assert_eq!(r.tick(), before);
    assert!(entity_by_id(r.app.server().simulation().world(), r.ids[0]).is_none());
    r.publish(1, &image_b, Duration::from_millis(100), 1);
    retired(&current[0]);
    assert!(!current[1].lifetime.is_retired());
    assert_eq!(proc_identity(current[1].pid).unwrap().0, current[1].start);
    assert!(image_a.try_recv().unwrap().is_none());
    assert_eq!(r.app.pump_camera(r.ids[0]).unwrap(), None);
    r.remove.store(2, Ordering::Release);
    assert_eq!(r.tick(), before);
    assert_eq!(r.app.pump_camera(r.ids[1]).unwrap(), None);
    retired(&current[1]);
    assert!(!r.app.has_camera_output());
    assert!(
        !r.app.server().zenoh_session().is_closed(),
        "retiring mixed outputs cannot close shared world IO"
    );
    r.reset();
    assert_eq!(r.app.server().simulation().epoch(), 2);
    r.matching([(true, true); 2]);
    r.pause(false);
    assert_eq!(r.tick(), Duration::from_millis(1));
    let rows = r.capture();
    for id in r.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, subscription) in infos.iter().enumerate() {
        info(
            receive(subscription, b"gz.msgs.CameraInfo"),
            owner,
            Duration::from_millis(1),
        );
    }
    r.ready(&[0, 1]);
    r.publish(0, &image_a, Duration::from_millis(1), 0);
    r.publish(1, &image_b, Duration::from_millis(1), 0);
    let last = [worker(&r, 0), worker(&r, 1)];
    r.app.shutdown();
    for w in &last {
        retired(w);
    }
    r.observer.close().wait().unwrap();
}

#[test]
fn mixed_real_worker_fault_does_not_skip_peer_or_repeat_committed_time() {
    use rustix::{
        event::{PollFd, PollFlags, Timespec, poll},
        process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal},
    };
    if isolation::child(
        "mixed_real_worker_fault_does_not_skip_peer_or_repeat_committed_time",
        180,
    ) {
        return;
    }
    let mut r = Rig::new(true);
    let image_a = r.sub(0, true);
    let infos: [Subscriber; 2] = std::array::from_fn(|i| r.sub(i, false));
    r.matching([(true, true), (false, true)]);
    assert_eq!(r.tick(), Duration::from_millis(1));
    let rows = r.capture();
    assert_eq!(requested(&rows, r.ids[0]), Some(Progress::Pending));
    assert_eq!(requested(&rows, r.ids[1]), Some(Progress::InfoPublished));
    for (owner, subscription) in infos.iter().enumerate() {
        info(
            receive(subscription, b"gz.msgs.CameraInfo"),
            owner,
            Duration::from_millis(1),
        );
    }
    // Client::start publishes its child PID asynchronously. Observe a positive
    // owned identity without pumping away the fault that capture_all must see.
    let until = Instant::now() + Duration::from_secs(5);
    let failed = loop {
        if let Some((Some(pid), lifetime)) = r.app.camera_worker_diagnostics(r.ids[0]) {
            let (start, state, parent) =
                proc_identity(pid).expect("published worker PID must be present");
            assert!(!matches!(state, 'Z' | 'X'));
            assert_eq!(parent, std::process::id());
            break Worker {
                pid,
                start,
                lifetime,
            };
        }
        assert!(
            Instant::now() < until,
            "positive owned worker PID was not published"
        );
        thread::yield_now();
    };
    let fd = pidfd_open(
        Pid::from_raw(failed.pid as i32).unwrap(),
        PidfdFlags::empty(),
    )
    .unwrap();
    assert_eq!(proc_identity(failed.pid).unwrap().0, failed.start);
    pidfd_send_signal(&fd, Signal::KILL).unwrap();
    assert_eq!(
        poll(
            &mut [PollFd::new(&fd, PollFlags::IN)],
            Some(&Timespec {
                tv_sec: 5,
                tv_nsec: 0
            })
        )
        .unwrap(),
        1
    );
    let image_b = r.sub(1, true);
    r.matching([(true, true); 2]);
    r.advance(100);
    let until = Instant::now() + Duration::from_secs(5);
    let mut peer_requested = false;
    let mut peer_published = false;
    loop {
        let rows = r.capture();
        assert_eq!(
            r.app.server().simulation().elapsed(),
            Duration::from_millis(100)
        );
        let peer = rows
            .iter()
            .find(|v| v.sensor_id == r.ids[1])
            .unwrap()
            .result
            .as_ref()
            .expect("failed RGB owner must not skip/fail its healthy depth peer");
        if peer.requested == Some(Progress::Pending) {
            assert!(
                !peer_requested,
                "no repeated sample at the same committed time"
            );
            peer_requested = true;
        } else {
            assert!(peer.requested.is_none());
        }
        if peer.pumped == Some(Progress::ImagePublished) {
            assert!(!peer_published);
            peer_published = true;
        }
        match &rows
            .iter()
            .find(|v| v.sensor_id == r.ids[0])
            .unwrap()
            .result
        {
            Err(CameraServerError::Camera(CameraOutputError::Client(_))) => {
                assert!(peer_requested);
                break;
            }
            Err(CameraServerError::Camera(CameraOutputError::Busy)) | Ok(_) => {}
            other => panic!("unexpected killed-owner capture result {other:?}"),
        }
        assert!(
            Instant::now() < until,
            "capture_all did not report the actual killed-worker Client error"
        );
        thread::yield_now();
    }
    info(
        receive(&infos[1], b"gz.msgs.CameraInfo"),
        1,
        Duration::from_millis(100),
    );
    let until = Instant::now() + Duration::from_secs(20);
    while !peer_published {
        match r.app.pump_camera(r.ids[1]).unwrap().unwrap() {
            Progress::ImagePublished => peer_published = true,
            Progress::FrameReady
            | Progress::Pending
            | Progress::Backpressure
            | Progress::SnapshotAccepted
            | Progress::CaptureSubmitted => {}
            other => panic!("healthy peer failed after the other worker faulted {other:?}"),
        }
        assert!(
            Instant::now() < until,
            "unchanged twenty-second actual readback budget"
        );
        thread::yield_now();
    }
    image(
        receive(&image_b, b"gz.msgs.Image"),
        1,
        Duration::from_millis(100),
        0,
    );
    let healthy = worker(&r, 1);
    assert_eq!(
        r.app.pump_camera(r.ids[0]).unwrap(),
        Some(Progress::RetireRequired)
    );
    retired(&failed);
    assert!(
        image_a.try_recv().unwrap().is_none(),
        "failed owner cannot release a stale frame"
    );
    assert_eq!(
        r.app.server().simulation().elapsed(),
        Duration::from_millis(100)
    );
    assert!(!r.app.server().zenoh_session().is_closed());
    r.app.shutdown();
    retired(&healthy);
    r.observer.close().wait().unwrap();
}
