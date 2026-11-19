// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Explicit SDF intrinsics through actual software workers and canonical consumers.
//! No registration, synchronized exposure, native rendering or throughput claim.
#[path = "../../gz-camera-output/tests/support/isolated_camera.rs"]
mod isolation;
#[path = "support/intrinsics_native.rs"]
mod native;
#[path = "../../../tests/support/zenoh_peer.rs"]
mod selected_native;
use gz_camera_output::{
    CameraCapture, CameraServer, CameraServerError, CameraServerOptions, Progress,
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
const XML: &str = include_str!("../../../examples/m72_camera_intrinsics.sdf");
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
        Self::with_xml(shared, XML, "m72-camera-intrinsics.sdf")
    }
    fn with_xml(shared: bool, source: &str, label: &str) -> Self {
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
            source,
            label,
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
                    if selected == 3 || selected == 4 {
                        let entity = entity_by_id(world, ids[0]).unwrap();
                        let mut sensor =
                            world.get_mut::<gz_sim_scene::CameraSensor>(entity).unwrap();
                        let sdformat::SensorConfig::Camera(camera) =
                            &mut Arc::make_mut(&mut sensor.0).config
                        else {
                            unreachable!()
                        };
                        if selected == 3 {
                            camera.lens.intrinsics.cx += 1.0;
                        } else {
                            camera.lens.projection.cx += 1.0;
                        }
                    } else {
                        assert!(selected <= 2);
                        assert!(
                            world.despawn(
                                entity_by_id(world, ids[usize::from(selected - 1)]).unwrap()
                            )
                        );
                    }
                }
            },
        );
        let settings = CameraServerOptions {
            worker_path: env!("CARGO_BIN_EXE_gz-render-worker").into(),
            origin: [0.0; 3],
            zenoh: config(Some(&endpoints[0])),
        };
        let options = ServerOptions {
            partition: format!("m72-intrinsics-{}", std::process::id()),
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
            "/m72/{}/{}",
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
const K: [[f64; 4]; 2] = [[40.0, 48.0, 20.0, 25.0], [20.0, 24.0, 42.0, 20.0]];
fn info(wire: Vec<u8>, owner: usize, stamp: Duration) -> CameraInfo {
    let v = CameraInfo::decode(wire.as_slice()).unwrap();
    assert_eq!(v.encode_to_vec(), wire);
    header(v.header.as_ref().unwrap(), owner, stamp, None);
    assert_eq!((v.width, v.height), (64, 64));
    let [fx, fy, cx, cy] = K[owner];
    assert_eq!(
        v.intrinsics.as_ref().unwrap().k,
        [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    );
    assert_eq!(
        v.projection.as_ref().unwrap().p,
        [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    );
    assert_eq!(
        v.rectification_matrix,
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    );
    let d = v.distortion.as_ref().unwrap();
    assert_eq!(d.k, [0.0; 5]);
    assert_eq!(d.model, 0);
    v
}
// Independent slab intersections against literal fixture bounds. Forward ray X=1
// makes t optical-axis depth, not Euclidean range or received-K-derived truth.
fn hit(owner: usize, x: usize, y: usize, fallback: bool) -> (f64, usize, bool) {
    let [fx, fy, cx, cy] = if fallback { [32.0; 4] } else { K[owner] };
    let origin = [0.0, if owner == 0 { -2.0 } else { 2.0 }, 1.0];
    let ray = [1.0, (cx - x as f64 - 0.5) / fx, (cy - y as f64 - 0.5) / fy];
    let centers = [[6.0, 0.0, 1.0], [2.0, -2.0, 1.0], [3.0, 2.0, 1.0]];
    let halves = [[0.1, 8.0, 8.0], [0.2; 3], [0.2; 3]];
    let mut best = (f64::INFINITY, 0, false);
    for (box_id, (center, half)) in centers.iter().zip(halves).enumerate() {
        let mut enter = 0.1_f64;
        let mut leave = 10.0_f64;
        for (((c, h), o), d) in center.iter().zip(half).zip(origin).zip(ray) {
            let lo = c - h - o;
            let hi = c + h - o;
            if d == 0.0 {
                if lo > 0.0 || hi < 0.0 {
                    leave = -1.0;
                }
            } else {
                let a = lo / d;
                let b = hi / d;
                enter = enter.max(a.min(b));
                leave = leave.min(a.max(b));
            }
        }
        if enter <= leave && enter < best.0 {
            let front = (enter - (center[0] - half[0])).abs() < 1e-10;
            let interior = front
                && (origin[1] + enter * ray[1] - center[1]).abs() < half[1] - 0.03
                && (origin[2] + enter * ray[2] - center[2]).abs() < half[2] - 0.03;
            best = (enter, box_id, interior);
        }
    }
    best
}
// A center ray alone does not classify a multisampled pixel. Project the
// literal box corners without using received calibration or renderer helpers.
fn projected_box_bounds(owner: usize, box_id: usize, front_only: bool) -> [f64; 4] {
    let [fx, fy, cx, cy] = K[owner];
    let centers = [[6.0, 0.0, 1.0], [2.0, -2.0, 1.0], [3.0, 2.0, 1.0]];
    let halves = [[0.1, 8.0, 8.0], [0.2; 3], [0.2; 3]];
    let center = centers[box_id];
    let half = halves[box_id];
    let front = center[0] - half[0];
    let xs = [
        front,
        if front_only {
            front
        } else {
            center[0] + half[0]
        },
    ];
    let origin_y = if owner == 0 { -2.0 } else { 2.0 };
    let mut bounds = [
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ];
    for px in xs {
        for py in [center[1] - half[1], center[1] + half[1]] {
            for pz in [center[2] - half[2], center[2] + half[2]] {
                let u = cx - fx * (py - origin_y) / px;
                let v = cy - fy * (pz - 1.0) / px;
                bounds[0] = bounds[0].min(u);
                bounds[1] = bounds[1].max(u);
                bounds[2] = bounds[2].min(v);
                bounds[3] = bounds[3].max(v);
            }
        }
    }
    bounds
}
fn homogeneous_rgb_footprint(owner: usize, x: usize, y: usize, box_id: usize, depth: f64) -> bool {
    let x0 = x as f64;
    let y0 = y as f64;
    let x1 = x0 + 1.0;
    let y1 = y0 + 1.0;
    let selected = projected_box_bounds(owner, box_id, true);
    // The whole square must be strictly inside the selected front face.
    if !(x0 > selected[0] && x1 < selected[1] && y0 > selected[2] && y1 < selected[3]) {
        return false;
    }
    for (other, front) in [5.9, 1.8, 2.8].into_iter().enumerate() {
        if other != box_id && front < depth {
            // Eight-corner bounds conservatively cover every nearer box's
            // silhouette. Even touching that bound excludes the flat-color
            // assertion, without guessing GPU sample locations or blending.
            let bounds = projected_box_bounds(owner, other, false);
            if x1 >= bounds[0] && x0 <= bounds[1] && y1 >= bounds[2] && y0 <= bounds[3] {
                return false;
            }
        }
    }
    true
}
fn image(wire: Vec<u8>, owner: usize, stamp: Duration, sequence: u64) -> Image {
    let v = Image::decode(wire.as_slice()).unwrap();
    assert_eq!(v.encode_to_vec(), wire);
    header(v.header.as_ref().unwrap(), owner, stamp, Some(sequence));
    assert_eq!((v.width, v.height), (64, 64));
    let mut red = 0;
    let mut blue = 0;
    let mut infinity = 0;
    let mut differs = 0;
    let mut checked = 0;
    let mut excluded = 0;
    if owner == 0 {
        assert_eq!(
            (v.step, v.pixel_format_type, v.data.len()),
            (192, PixelFormatType::RgbInt8 as i32, 12288)
        );
    } else {
        assert_eq!(
            (v.step, v.pixel_format_type, v.data.len()),
            (256, PixelFormatType::RFloat32 as i32, 16384)
        );
    }
    for y in 0..64 {
        for x in 0..64 {
            let (expected, box_id, interior) = hit(owner, x, y, false);
            let (old, oldbox, _) = hit(owner, x, y, true);
            if owner == 0 {
                if !interior || !homogeneous_rgb_footprint(owner, x, y, box_id, expected) {
                    excluded += 1;
                    continue;
                }
                checked += 1;
                let color = match box_id {
                    0 => [0, 0, 255],
                    1 => [255, 0, 0],
                    2 => [0, 255, 0],
                    _ => unreachable!(),
                };
                let actual = &v.data[(y * 64 + x) * 3..(y * 64 + x) * 3 + 3];
                assert_eq!(
                    actual, color,
                    "RGB owner{owner} pixel{x},{y} box{box_id} expected{color:?} actual{actual:?}; whole footprint classified homogeneous"
                );
                red += usize::from(box_id == 1);
                blue += usize::from(box_id == 0);
                differs += usize::from(box_id == 1 && (!old.is_finite() || oldbox != 1));
            } else if owner == 1 {
                checked += 1;
                let offset = (y * 64 + x) * 4;
                let actual = f32::from_le_bytes(v.data[offset..offset + 4].try_into().unwrap());
                if expected.is_infinite() {
                    assert_eq!(
                        actual,
                        f32::INFINITY,
                        "depth owner{owner} pixel{x},{y} expected+INF actual{actual}"
                    );
                    infinity += 1;
                } else {
                    assert!(
                        actual.is_finite()
                            && actual > 0.0
                            && (f64::from(actual) - expected).abs() < 1e-4,
                        "depth owner{owner} pixel{x},{y} expected{expected} actual{actual}"
                    );
                }
                differs += usize::from(
                    expected.is_infinite() != old.is_infinite()
                        || (expected.is_finite()
                            && old.is_finite()
                            && (expected - old).abs() > 0.1),
                );
            }
        }
    }
    if owner == 0 {
        assert_eq!(
            checked + excluded,
            4096,
            "RGB owner{owner} whole-pixel accounting checked{checked} excluded{excluded}"
        );
        assert!(
            excluded > 0,
            "RGB fixture must exercise excluded nonhomogeneous/boundary footprints"
        );
        assert!(
            red > 20 && blue > 100 && differs > 20,
            "RGB owner{owner} checked{checked} excluded{excluded} red{red} blue{blue} HFOV_disagreements{differs}; expected red>20 blue>100 disagreements>20"
        );
    } else {
        assert_eq!(
            (checked, excluded),
            (4096, 0),
            "depth owner{owner} every axial-depth word remains checked"
        );
        assert!(
            infinity > 100 && differs > 100,
            "depth owner{owner} checked{checked} +INF{infinity} HFOV_disagreements{differs}; expected +INF>100 disagreements>100"
        );
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
    let until = Instant::now() + Duration::from_secs(5);
    let mut observed = pid;
    while observed.is_none() {
        assert!(
            Instant::now() < until,
            "actual worker PID publication budget"
        );
        thread::yield_now();
        observed = r.app.camera_worker_diagnostics(r.ids[owner]).unwrap().0;
    }
    let pid = observed.unwrap();
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
fn explicit_k_real_workers_and_native_consumers_observe_pixels_depth_and_independent_cadence() {
    if isolation::child(
        "explicit_k_real_workers_and_native_consumers_observe_pixels_depth_and_independent_cadence",
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
    let mut cpp = std::array::from_fn::<_, 2, _>(|owner| {
        native::Native::start(
            &r.endpoint,
            &r.app.server().options().partition,
            &peer,
            owner,
        )
    });
    assert_ne!(cpp[0].pid(), cpp[1].pid());
    for (owner, consumer) in cpp.iter().enumerate() {
        assert_eq!(
            consumer.line(),
            format!("ready explicit intrinsics {owner}")
        );
    }
    r.matching([(true, true); 2]);
    for id in r.ids {
        assert!(r.app.camera_worker_diagnostics(id).is_none());
    }
    let images: [Subscriber; 2] = std::array::from_fn(|owner| r.sub(owner, true));
    let infos: [Subscriber; 2] = std::array::from_fn(|owner| r.sub(owner, false));
    let first = r.tick();
    assert_eq!(first, Duration::from_millis(1));
    let rows = r.capture();
    for id in r.ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    let first_infos: [CameraInfo; 2] = std::array::from_fn(|owner| {
        info(receive(&infos[owner], b"gz.msgs.CameraInfo"), owner, first)
    });
    r.ready(&[0, 1]);
    let workers = [worker(&r, 0), worker(&r, 1)];
    assert_ne!(workers[0].pid, workers[1].pid);
    let first_images: [Image; 2] =
        std::array::from_fn(|owner| r.publish(owner, &images[owner], first, 0));
    let mut totals = [0; 2];
    for owner in 0..2 {
        for (kind, expected) in [
            ("IMAGE", first_images[owner].encode_to_vec()),
            ("INFO", first_infos[owner].encode_to_vec()),
        ] {
            let line = cpp[owner].line();
            let (actual, raw) = line.split_once(' ').unwrap();
            assert_eq!(actual, kind);
            assert_eq!(hex(raw), expected, "all actual canonical consumer bytes");
        }
        assert_eq!(
            cpp[owner].line(),
            format!("PASS canonical explicit intrinsics {owner}")
        );
        totals[owner] = cpp[owner].finish();
        assert_eq!(cpp[owner].actual_exit(), Some(0));
    }
    let receipt=serde_json::to_vec_pretty(&serde_json::json!({"actual_consumer_exit_codes":[cpp[0].actual_exit(),cpp[1].actual_exit()],
        "consumer_PIDs":[cpp[0].pid(),cpp[1].pid()],"actual_reaped":[true,true],"stdout_bytes":totals,
        "per_stdout_cap":65536,"per_stderr_cap":4096,"aggregate_cap":131072,"K":K,"sensor_ids":r.ids,
        "worker_exit_codes":"UNKNOWN","scope":"Actual canonical explicit-K consumers and software workers; no native renderer pixel equivalence."})).unwrap();
    assert!(totals.into_iter().sum::<usize>() + receipt.len() <= 131072);
    let mut saved = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(
            peer.binary
                .parent()
                .unwrap()
                .join("m72-two-explicit-K-consumers.json"),
        )
        .unwrap();
    std::io::Write::write_all(&mut saved, &receipt).unwrap();
    saved.sync_all().unwrap();
    let mut stamps: [Vec<Duration>; 2] = std::array::from_fn(|_| vec![first]);
    while r.app.server().simulation().elapsed() < Duration::from_millis(200) {
        let stamp = r.tick();
        let rows = r.capture();
        let mut due = Vec::new();
        for (owner, sub) in infos.iter().enumerate() {
            match requested(&rows, r.ids[owner]) {
                Some(Progress::Pending) => {
                    info(receive(sub, b"gz.msgs.CameraInfo"), owner, stamp);
                    due.push(owner);
                }
                None => {}
                other => panic!("explicit-K cadence {other:?}"),
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
    for _ in 0..8 {
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
    r.observer.close().wait().unwrap();
}

#[test]
fn explicit_k_info_only_and_selected_depth_demand_never_start_an_rgb_worker() {
    if isolation::child(
        "explicit_k_info_only_and_selected_depth_demand_never_start_an_rgb_worker",
        180,
    ) {
        return;
    }
    let mut r = Rig::new(false);
    assert_eq!(r.tick(), Duration::from_millis(1));
    for row in r.capture() {
        assert_eq!(row.result.unwrap().requested, Some(Progress::NoDemand));
    }
    let infos: [Subscriber; 2] = std::array::from_fn(|owner| r.sub(owner, false));
    r.matching([(false, true); 2]);
    let stamp = r.tick();
    assert_eq!(stamp, Duration::from_millis(2));
    for row in r.capture() {
        assert_eq!(row.result.unwrap().requested, Some(Progress::InfoPublished));
    }
    for (owner, sub) in infos.iter().enumerate() {
        info(receive(sub, b"gz.msgs.CameraInfo"), owner, stamp);
        assert!(r.app.camera_worker_diagnostics(r.ids[owner]).is_none());
    }
    let depth = r.sub(1, true);
    r.matching([(false, true), (true, true)]);
    r.advance(50);
    let rows = r.capture();
    assert_eq!(requested(&rows, r.ids[0]), None);
    assert_eq!(requested(&rows, r.ids[1]), Some(Progress::Pending));
    info(
        receive(&infos[1], b"gz.msgs.CameraInfo"),
        1,
        Duration::from_millis(50),
    );
    r.ready(&[1]);
    let w = worker(&r, 1);
    r.publish(1, &depth, Duration::from_millis(50), 0);
    assert!(r.app.camera_worker_diagnostics(r.ids[0]).is_none());
    drop(depth);
    r.matching([(false, true); 2]);
    assert_eq!(
        r.app.pump_camera(r.ids[1]).unwrap(),
        Some(Progress::RetireRequired)
    );
    retired(&w);
    assert!(r.app.camera_worker_diagnostics(r.ids[0]).is_none());
    r.app.shutdown();
    r.observer.close().wait().unwrap();
}

#[test]
fn explicit_k_reset_discards_buffered_frames_and_removal_retires_only_its_owner() {
    if isolation::child(
        "explicit_k_reset_discards_buffered_frames_and_removal_retires_only_its_owner",
        180,
    ) {
        return;
    }
    let mut r = Rig::new(true);
    let ids = r.ids;
    let images: [Subscriber; 2] = std::array::from_fn(|owner| r.sub(owner, true));
    let infos: [Subscriber; 2] = std::array::from_fn(|owner| r.sub(owner, false));
    r.matching([(true, true); 2]);
    let first = r.tick();
    assert_eq!(first, Duration::from_millis(1));
    let rows = r.capture();
    for id in ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, sub) in infos.iter().enumerate() {
        info(receive(sub, b"gz.msgs.CameraInfo"), owner, first);
    }
    r.ready(&[0, 1]);
    let old = [worker(&r, 0), worker(&r, 1)];
    // Buffered but not yet published frames must not escape across the reset.
    r.reset();
    assert_eq!(r.ids, ids);
    for w in &old {
        retired(w);
    }
    for sub in &images {
        assert!(sub.try_recv().unwrap().is_none());
    }
    r.matching([(true, true); 2]);
    r.pause(false);
    assert_eq!(r.tick(), first);
    let rows = r.capture();
    for id in ids {
        assert_eq!(requested(&rows, id), Some(Progress::Pending));
    }
    for (owner, sub) in infos.iter().enumerate() {
        info(receive(sub, b"gz.msgs.CameraInfo"), owner, first);
    }
    r.ready(&[0, 1]);
    let current = [worker(&r, 0), worker(&r, 1)];
    // The depth peer still publishes its selected K at sequence0 after RGB removal.
    r.pause(true);
    r.remove.store(1, Ordering::Release);
    assert_eq!(r.tick(), first);
    assert_eq!(r.app.pump_camera(ids[0]).unwrap(), None);
    retired(&current[0]);
    assert!(images[0].try_recv().unwrap().is_none());
    r.publish(1, &images[1], first, 0);
    assert_eq!(proc_identity(current[1].pid).unwrap().0, current[1].start);
    assert!(!current[1].lifetime.is_retired());
    assert!(!r.app.server().zenoh_session().is_closed());
    r.remove.store(2, Ordering::Release);
    assert_eq!(r.tick(), first);
    assert_eq!(r.app.pump_camera(ids[1]).unwrap(), None);
    retired(&current[1]);
    assert!(!r.app.has_camera_output());
    assert!(!r.app.server().zenoh_session().is_closed());
    r.app.shutdown();
    r.observer.close().wait().unwrap();
}

#[test]
fn explicit_k_drift_retires_a_real_buffered_frame_before_any_stale_image() {
    if isolation::child(
        "explicit_k_drift_retires_a_real_buffered_frame_before_any_stale_image",
        180,
    ) {
        return;
    }
    let mut r = Rig::new(true);
    let rgb = r.sub(0, true);
    let infos: [Subscriber; 2] = std::array::from_fn(|owner| r.sub(owner, false));
    r.matching([(true, true), (false, true)]);
    let first = r.tick();
    assert_eq!(first, Duration::from_millis(1));
    let rows = r.capture();
    assert_eq!(requested(&rows, r.ids[0]), Some(Progress::Pending));
    assert_eq!(requested(&rows, r.ids[1]), Some(Progress::InfoPublished));
    for (owner, sub) in infos.iter().enumerate() {
        info(receive(sub, b"gz.msgs.CameraInfo"), owner, first);
    }
    r.ready(&[0]);
    let old = worker(&r, 0);
    assert!(rgb.try_recv().unwrap().is_none());
    r.pause(true);
    r.remove.store(3, Ordering::Release);
    assert_eq!(r.tick(), first);
    assert!(matches!(
        r.app.pump_camera(r.ids[0]),
        Err(CameraServerError::Camera(
            gz_camera_output::CameraOutputError::OwnerInvalid
        ))
    ));
    retired(&old);
    assert!(!r.app.has_camera_output());
    assert!(rgb.try_recv().unwrap().is_none());
    for sub in &infos {
        assert!(sub.try_recv().unwrap().is_none());
    }
    assert!(!r.app.server().zenoh_session().is_closed());
    r.reset();
    r.matching([(true, true), (false, true)]);
    r.pause(false);
    assert_eq!(r.tick(), first);
    let rows = r.capture();
    assert_eq!(requested(&rows, r.ids[0]), Some(Progress::Pending));
    assert_eq!(requested(&rows, r.ids[1]), Some(Progress::InfoPublished));
    for (owner, sub) in infos.iter().enumerate() {
        info(receive(sub, b"gz.msgs.CameraInfo"), owner, first);
    }
    r.ready(&[0]);
    let fresh = worker(&r, 0);
    r.publish(0, &rgb, first, 0);
    assert!(r.app.camera_worker_diagnostics(r.ids[1]).is_none());
    r.app.shutdown();
    retired(&fresh);
    r.observer.close().wait().unwrap();
}

// M73 adds distinct published K/P while reusing M72's unchanged custody,
// full-footprint RGB and all4096 depth oracles. P deliberately equals M72 K.
mod projection {
    use super::*;
    const XML: &str = include_str!("../../../examples/m73_camera_projection.sdf");
    const INTRINSICS: [[f64; 4]; 2] = [[52.0, 44.0, 34.0, 29.0], [36.0, 42.0, 26.0, 38.0]];
    fn rig() -> Rig {
        Rig::with_xml(true, XML, "m73-camera-projection.sdf")
    }
    fn sub(r: &Rig, owner: usize, image: bool) -> Subscriber {
        let topic = format!(
            "/m73/{}/{}",
            if owner == 0 { "rgb" } else { "depth" },
            if image { "image" } else { "camera_info" }
        );
        r.observer
            .declare_subscriber(r.app.server().options().topic_key(&topic).unwrap())
            .wait()
            .unwrap()
    }
    fn info(wire: Vec<u8>, owner: usize, stamp: Duration) -> CameraInfo {
        let v = CameraInfo::decode(wire.as_slice()).unwrap();
        assert_eq!(v.encode_to_vec(), wire);
        header(v.header.as_ref().unwrap(), owner, stamp, None);
        assert_eq!((v.width, v.height), (64, 64));
        let [fx, fy, cx, cy] = INTRINSICS[owner];
        assert_eq!(
            v.intrinsics.as_ref().unwrap().k,
            [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        );
        let [pfx, pfy, pcx, pcy] = K[owner];
        assert_eq!(
            v.projection.as_ref().unwrap().p,
            [pfx, 0.0, pcx, 0.0, 0.0, pfy, pcy, 0.0, 0.0, 0.0, 1.0, 0.0]
        );
        assert_ne!(fx, pfx);
        assert_ne!(cx, pcx);
        assert_eq!(
            v.rectification_matrix,
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        );
        let d = v.distortion.as_ref().unwrap();
        assert_eq!(d.model, 0);
        assert_eq!(d.k, [0.0; 5]);
        v
    }
    fn image(wire: Vec<u8>, owner: usize, stamp: Duration, sequence: u64) -> Image {
        let v = super::image(wire, owner, stamp, sequence);
        let [fx, fy, cx, cy] = INTRINSICS[owner];
        let (x, y, front) = if owner == 0 {
            (20, 25, 1.8)
        } else {
            (42, 20, 2.8)
        };
        let k_ray_y = (cx - x as f64 - 0.5) / fx;
        let k_ray_z = (cy - y as f64 - 0.5) / fy;
        assert!(
            (k_ray_y * front).abs() > 0.2 || (k_ray_z * front).abs() > 0.2,
            "literal K ray must miss the selected foreground"
        );
        if owner == 0 {
            assert_eq!(
                &v.data[(y * 64 + x) * 3..(y * 64 + x) * 3 + 3],
                [255, 0, 0],
                "RGB P-vs-K pixel{x},{y}"
            );
        } else {
            let offset = (y * 64 + x) * 4;
            let actual = f32::from_le_bytes(v.data[offset..offset + 4].try_into().unwrap());
            assert!(
                (f64::from(actual) - 2.8).abs() < 1e-4,
                "depth P-vs-K pixel{x},{y} expected2.8 actual{actual}"
            );
        }
        v
    }
    fn publish(
        r: &mut Rig,
        owner: usize,
        subscriber: &Subscriber,
        stamp: Duration,
        sequence: u64,
    ) -> Image {
        assert_eq!(
            r.app.pump_camera(r.ids[owner]).unwrap(),
            Some(Progress::ImagePublished)
        );
        image(
            receive(subscriber, b"gz.msgs.Image"),
            owner,
            stamp,
            sequence,
        )
    }
    #[test]
    fn explicit_projection_real_workers_and_canonical_consumers_follow_p_and_publish_distinct_k() {
        if isolation::child(
            "projection::explicit_projection_real_workers_and_canonical_consumers_follow_p_and_publish_distinct_k",
            180,
        ) {
            return;
        }
        let mut r = rig();
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .parent()
            .unwrap();
        let peer = selected_native::NativePeer::build(root, "camera");
        let mut cpp: [native::Native; 2] = std::array::from_fn(|owner| {
            native::Native::start_mode(
                &r.endpoint,
                &r.app.server().options().partition,
                &peer,
                owner,
                "explicit-projection",
            )
        });
        assert_ne!(cpp[0].pid(), cpp[1].pid());
        for (owner, consumer) in cpp.iter().enumerate() {
            assert_eq!(
                consumer.line(),
                format!("ready explicit projection {owner}")
            );
        }
        r.matching([(true, true); 2]);
        for id in r.ids {
            assert!(r.app.camera_worker_diagnostics(id).is_none());
        }
        let images: [Subscriber; 2] = std::array::from_fn(|owner| sub(&r, owner, true));
        let infos: [Subscriber; 2] = std::array::from_fn(|owner| sub(&r, owner, false));
        let first = r.tick();
        assert_eq!(first, Duration::from_millis(1));
        let rows = r.capture();
        for id in r.ids {
            assert_eq!(requested(&rows, id), Some(Progress::Pending));
        }
        let first_infos: [CameraInfo; 2] = std::array::from_fn(|owner| {
            info(receive(&infos[owner], b"gz.msgs.CameraInfo"), owner, first)
        });
        r.ready(&[0, 1]);
        let workers = [worker(&r, 0), worker(&r, 1)];
        assert_ne!(workers[0].pid, workers[1].pid);
        let first_images: [Image; 2] =
            std::array::from_fn(|owner| publish(&mut r, owner, &images[owner], first, 0));
        let mut totals = [0; 2];
        for owner in 0..2 {
            for (kind, expected) in [
                ("IMAGE", first_images[owner].encode_to_vec()),
                ("INFO", first_infos[owner].encode_to_vec()),
            ] {
                let line = cpp[owner].line();
                let (actual, raw) = line.split_once(' ').unwrap();
                assert_eq!(actual, kind);
                assert_eq!(
                    hex(raw),
                    expected,
                    "all actual canonical distinct-K/P bytes"
                );
            }
            assert_eq!(
                cpp[owner].line(),
                format!("PASS canonical explicit projection {owner}")
            );
            totals[owner] = cpp[owner].finish();
            assert_eq!(cpp[owner].actual_exit(), Some(0));
        }
        let receipt=serde_json::to_vec_pretty(&serde_json::json!({"actual_consumer_exit_codes":[cpp[0].actual_exit(),cpp[1].actual_exit()],
            "consumer_PIDs":[cpp[0].pid(),cpp[1].pid()],"actual_reaped":[true,true],"stdout_bytes":totals,
            "per_stdout_cap":65536,"per_stderr_cap":4096,"aggregate_cap":131072,"K":INTRINSICS,"P":K,"sensor_ids":r.ids,
            "worker_exit_codes":"UNKNOWN","scope":"Actual canonical distinct-K/P consumers and software workers; no native renderer pixel equivalence."})).unwrap();
        assert!(totals.into_iter().sum::<usize>() + receipt.len() <= 131072);
        let mut saved = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(
                peer.binary
                    .parent()
                    .unwrap()
                    .join("m73-two-distinct-K-P-consumers.json"),
            )
            .unwrap();
        std::io::Write::write_all(&mut saved, &receipt).unwrap();
        saved.sync_all().unwrap();
        let mut stamps: [Vec<Duration>; 2] = std::array::from_fn(|_| vec![first]);
        while r.app.server().simulation().elapsed() < Duration::from_millis(200) {
            let stamp = r.tick();
            let rows = r.capture();
            let mut due = Vec::new();
            for (owner, subscriber) in infos.iter().enumerate() {
                match requested(&rows, r.ids[owner]) {
                    Some(Progress::Pending) => {
                        info(receive(subscriber, b"gz.msgs.CameraInfo"), owner, stamp);
                        due.push(owner);
                    }
                    None => {}
                    other => panic!("projection cadence {other:?}"),
                }
            }
            r.ready(&due);
            for owner in due {
                publish(
                    &mut r,
                    owner,
                    &images[owner],
                    stamp,
                    stamps[owner].len() as u64,
                );
                stamps[owner].push(stamp);
            }
        }
        assert_eq!(stamps[0], [1, 100, 200].map(Duration::from_millis));
        assert_eq!(stamps[1], [1, 50, 100, 150, 200].map(Duration::from_millis));
        r.pause(true);
        for _ in 0..8 {
            assert_eq!(r.tick(), Duration::from_millis(200));
            for row in r.capture() {
                assert!(row.result.unwrap().requested.is_none());
            }
        }
        for subscriber in images.iter().chain(&infos) {
            assert!(subscriber.try_recv().unwrap().is_none());
        }
        r.app.shutdown();
        for w in &workers {
            retired(w);
        }
        r.observer.close().wait().unwrap();
    }
    #[test]
    fn explicit_projection_lazy_demand_drift_reset_and_removal_preserve_original_calibrations() {
        if isolation::child(
            "projection::explicit_projection_lazy_demand_drift_reset_and_removal_preserve_original_calibrations",
            180,
        ) {
            return;
        }
        let mut r = rig();
        let ids = r.ids;
        assert_eq!(r.tick(), Duration::from_millis(1));
        for row in r.capture() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::NoDemand));
        }
        let infos: [Subscriber; 2] = std::array::from_fn(|owner| sub(&r, owner, false));
        r.matching([(false, true); 2]);
        let stamp = r.tick();
        assert_eq!(stamp, Duration::from_millis(2));
        for row in r.capture() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::InfoPublished));
        }
        for (owner, subscriber) in infos.iter().enumerate() {
            info(receive(subscriber, b"gz.msgs.CameraInfo"), owner, stamp);
            assert!(r.app.camera_worker_diagnostics(ids[owner]).is_none());
        }
        let rgb = sub(&r, 0, true);
        r.matching([(true, true), (false, true)]);
        r.advance(100);
        let rows = r.capture();
        assert_eq!(requested(&rows, ids[0]), Some(Progress::Pending));
        assert_eq!(requested(&rows, ids[1]), Some(Progress::InfoPublished));
        for (owner, subscriber) in infos.iter().enumerate() {
            info(
                receive(subscriber, b"gz.msgs.CameraInfo"),
                owner,
                Duration::from_millis(100),
            );
        }
        r.ready(&[0]);
        let old = worker(&r, 0);
        assert!(r.app.camera_worker_diagnostics(ids[1]).is_none());
        assert!(rgb.try_recv().unwrap().is_none());
        r.pause(true);
        r.remove.store(4, Ordering::Release);
        assert_eq!(r.tick(), Duration::from_millis(100));
        assert!(matches!(
            r.app.pump_camera(ids[0]),
            Err(CameraServerError::Camera(
                gz_camera_output::CameraOutputError::OwnerInvalid
            ))
        ));
        retired(&old);
        assert!(!r.app.has_camera_output());
        assert!(rgb.try_recv().unwrap().is_none());
        assert!(!r.app.server().zenoh_session().is_closed());
        r.reset();
        assert_eq!(r.ids, ids);
        r.matching([(true, true), (false, true)]);
        r.pause(false);
        let first = r.tick();
        assert_eq!(first, Duration::from_millis(1));
        let rows = r.capture();
        assert_eq!(requested(&rows, ids[0]), Some(Progress::Pending));
        assert_eq!(requested(&rows, ids[1]), Some(Progress::InfoPublished));
        for (owner, subscriber) in infos.iter().enumerate() {
            info(receive(subscriber, b"gz.msgs.CameraInfo"), owner, first);
        }
        r.ready(&[0]);
        let fresh_rgb = worker(&r, 0);
        publish(&mut r, 0, &rgb, first, 0);
        assert!(r.app.camera_worker_diagnostics(ids[1]).is_none());
        let depth = sub(&r, 1, true);
        r.matching([(true, true); 2]);
        r.advance(50);
        let rows = r.capture();
        assert_eq!(requested(&rows, ids[0]), None);
        assert_eq!(requested(&rows, ids[1]), Some(Progress::Pending));
        info(
            receive(&infos[1], b"gz.msgs.CameraInfo"),
            1,
            Duration::from_millis(50),
        );
        r.ready(&[1]);
        let fresh_depth = worker(&r, 1);
        publish(&mut r, 1, &depth, Duration::from_millis(50), 0);
        r.pause(true);
        r.remove.store(1, Ordering::Release);
        assert_eq!(r.tick(), Duration::from_millis(50));
        assert_eq!(r.app.pump_camera(ids[0]).unwrap(), None);
        retired(&fresh_rgb);
        assert_eq!(proc_identity(fresh_depth.pid).unwrap().0, fresh_depth.start);
        assert!(!fresh_depth.lifetime.is_retired());
        assert!(!r.app.server().zenoh_session().is_closed());
        r.remove.store(2, Ordering::Release);
        assert_eq!(r.tick(), Duration::from_millis(50));
        assert_eq!(r.app.pump_camera(ids[1]).unwrap(), None);
        retired(&fresh_depth);
        assert!(!r.app.has_camera_output());
        assert!(!r.app.server().zenoh_session().is_closed());
        r.app.shutdown();
        r.observer.close().wait().unwrap();
    }
}
