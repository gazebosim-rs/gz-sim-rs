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
#[path = "../../gz-camera-output/tests/private/projection_offsets.rs"]
mod projection_offsets;
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
                    if (3..=6).contains(&selected) || matches!(selected, 8 | 9) {
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
                        } else if selected == 4 {
                            camera.lens.projection.cx += 1.0;
                        } else if selected == 5 {
                            camera.lens.intrinsics.skew += 1.0;
                        } else if selected == 6 {
                            camera.visibility_mask = 0;
                        } else if selected == 8 {
                            camera.lens.projection.tx += 1.0;
                        } else {
                            camera.lens.projection.ty += 1.0;
                        }
                    } else if selected == 7 {
                        let visual = world
                            .iter_entities()
                            .find(|e| {
                                e.get::<Name>()
                                    .is_some_and(|n| n.0 == "default::occluder_a::body::surface")
                            })
                            .unwrap()
                            .id();
                        world
                            .get_mut::<gz_sim_scene::VisualProperties>(visual)
                            .unwrap()
                            .0
                            .visibility_flags = 0;
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
    pub(super) fn image(wire: Vec<u8>, owner: usize, stamp: Duration, sequence: u64) -> Image {
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

// M74 shares all transport/process custody; only its independent pixel-ray
// oracle is new. M72/M73 image validators and test bodies remain unchanged.
mod skew {
    use super::*;
    const XML: &str = include_str!("../../../examples/m74_camera_skew.sdf");
    const CALIBRATION: [[f64; 4]; 2] = [[52.0, 44.0, 34.0, 29.0], [36.0, 42.0, 26.0, 38.0]];
    const SKEW: [f64; 2] = [36.0, -48.0];
    const CENTERS: [[f64; 3]; 3] = [[6.0, 0.0, 1.0], [2.0, -2.0, 1.0], [3.0, 2.0, 1.0]];
    const HALVES: [[f64; 3]; 3] = [[0.1, 8.0, 8.0], [0.2; 3], [0.2; 3]];
    fn source(explicit_p: bool) -> String {
        if !explicit_p {
            return XML.into();
        }
        XML.replace("<s>36</s></intrinsics>","<s>36</s></intrinsics><projection><p_fx>40</p_fx><p_fy>48</p_fy><p_cx>20</p_cx><p_cy>25</p_cy><tx>0</tx><ty>0</ty></projection>")
            .replace("<s>-48</s></intrinsics>","<s>-48</s></intrinsics><projection><p_fx>20</p_fx><p_fy>24</p_fy><p_cx>42</p_cx><p_cy>20</p_cy><tx>0</tx><ty>0</ty></projection>")
    }
    fn sub(r: &Rig, owner: usize, image: bool) -> Subscriber {
        let key = format!(
            "/m74/{}/{}",
            if owner == 0 { "rgb" } else { "depth" },
            if image { "image" } else { "camera_info" }
        );
        r.observer
            .declare_subscriber(r.app.server().options().topic_key(&key).unwrap())
            .wait()
            .unwrap()
    }
    fn info(wire: Vec<u8>, owner: usize, stamp: Duration, explicit_p: bool) -> CameraInfo {
        let v = CameraInfo::decode(wire.as_slice()).unwrap();
        assert_eq!(v.encode_to_vec(), wire);
        header(v.header.as_ref().unwrap(), owner, stamp, None);
        assert_eq!((v.width, v.height), (64, 64));
        let [fx, fy, cx, cy] = CALIBRATION[owner];
        let [pfx, pfy, pcx, pcy] = if explicit_p {
            K[owner]
        } else {
            CALIBRATION[owner]
        };
        // Pinned native PopulateInfo omits skew. This is a Source-derived
        // compatibility limitation, not an actual nonzero-skew producer capture.
        assert_eq!(
            v.intrinsics.as_ref().unwrap().k,
            [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        );
        assert_eq!(
            v.projection.as_ref().unwrap().p,
            [pfx, 0.0, pcx, 0.0, 0.0, pfy, pcy, 0.0, 0.0, 0.0, 1.0, 0.0]
        );
        assert_eq!(
            v.rectification_matrix,
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        );
        let d = v.distortion.as_ref().unwrap();
        assert_eq!(d.model, 0);
        assert_eq!(d.k, [0.0; 5]);
        v
    }
    fn ray(k: [f64; 4], s: f64, u: f64, v: f64) -> [f64; 3] {
        let [fx, fy, cx, cy] = k;
        let down = (v - cy) / fy;
        [1.0, -((u - cx) + s * down) / fx, -down]
    }
    fn hit(owner: usize, u: f64, v: f64, k: [f64; 4], s: f64) -> (f64, usize, bool) {
        let origin = [0.0, if owner == 0 { -2.0 } else { 2.0 }, 1.0];
        let direction = ray(k, s, u, v);
        let mut best = (f64::INFINITY, 0, false);
        for (box_id, (center, half)) in CENTERS.iter().zip(HALVES).enumerate() {
            let mut enter = 0.1_f64;
            let mut leave = 10.0_f64;
            for (((c, h), o), d) in center.iter().zip(half).zip(origin).zip(direction) {
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
                best = (
                    enter,
                    box_id,
                    front
                        && (origin[1] + enter * direction[1] - center[1]).abs() < half[1] - 0.03
                        && (origin[2] + enter * direction[2] - center[2]).abs() < half[2] - 0.03,
                );
            }
        }
        best
    }
    fn bounds(owner: usize, box_id: usize) -> [f64; 4] {
        let [fx, fy, cx, cy] = CALIBRATION[owner];
        let s = SKEW[owner];
        let c = CENTERS[box_id];
        let h = HALVES[box_id];
        let oy = if owner == 0 { -2.0 } else { 2.0 };
        let mut b = [
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ];
        for x in [c[0] - h[0], c[0] + h[0]] {
            for y in [c[1] - h[1], c[1] + h[1]] {
                for z in [c[2] - h[2], c[2] + h[2]] {
                    let u = cx - fx * (y - oy) / x + s * (z - 1.0) / x;
                    let v = cy - fy * (z - 1.0) / x;
                    b[0] = b[0].min(u);
                    b[1] = b[1].max(u);
                    b[2] = b[2].min(v);
                    b[3] = b[3].max(v);
                }
            }
        }
        b
    }
    fn homogeneous(owner: usize, x: usize, y: usize, box_id: usize, depth: f64) -> bool {
        let c = CENTERS[box_id];
        let h = HALVES[box_id];
        let oy = if owner == 0 { -2.0 } else { 2.0 };
        // A sheared projected face is a parallelogram. Four inverse-projected
        // corners inside its convex physical rectangle prove the WHOLE pixel;
        // an axis-aligned projected selected-face bbox would not prove this.
        for u in [x as f64, x as f64 + 1.0] {
            for v in [y as f64, y as f64 + 1.0] {
                let d = ray(CALIBRATION[owner], SKEW[owner], u, v);
                if (oy + depth * d[1] - c[1]).abs() >= h[1]
                    || (1.0 + depth * d[2] - c[2]).abs() >= h[2]
                {
                    return false;
                }
            }
        }
        for (other, front) in [5.9, 1.8, 2.8].into_iter().enumerate() {
            if other != box_id && front < depth {
                let b = bounds(owner, other);
                if x as f64 + 1.0 >= b[0]
                    && x as f64 <= b[1]
                    && y as f64 + 1.0 >= b[2]
                    && y as f64 <= b[3]
                {
                    return false;
                }
            }
        }
        true
    }
    fn image(
        wire: Vec<u8>,
        owner: usize,
        stamp: Duration,
        sequence: u64,
        explicit_p: bool,
    ) -> Image {
        if explicit_p {
            return super::projection::image(wire, owner, stamp, sequence);
        }
        let v = Image::decode(wire.as_slice()).unwrap();
        assert_eq!(v.encode_to_vec(), wire);
        header(v.header.as_ref().unwrap(), owner, stamp, Some(sequence));
        assert_eq!((v.width, v.height), (64, 64));
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
        let mut checked = 0;
        let mut excluded = 0;
        let mut red = 0;
        let mut blue = 0;
        let mut infinite = 0;
        let mut hfov_diff = 0;
        let mut zero_diff = 0;
        for y in 0..64 {
            for x in 0..64 {
                let current = hit(
                    owner,
                    x as f64 + 0.5,
                    y as f64 + 0.5,
                    CALIBRATION[owner],
                    SKEW[owner],
                );
                let zero = hit(
                    owner,
                    x as f64 + 0.5,
                    y as f64 + 0.5,
                    CALIBRATION[owner],
                    0.0,
                );
                let hfov = hit(owner, x as f64 + 0.5, y as f64 + 0.5, [32.0; 4], 0.0);
                if owner == 0 {
                    if !current.2 || !homogeneous(owner, x, y, current.1, current.0) {
                        excluded += 1;
                        continue;
                    }
                    checked += 1;
                    let color = match current.1 {
                        0 => [0, 0, 255],
                        1 => [255, 0, 0],
                        2 => [0, 255, 0],
                        _ => unreachable!(),
                    };
                    let actual = &v.data[(y * 64 + x) * 3..(y * 64 + x) * 3 + 3];
                    assert_eq!(
                        actual, color,
                        "signed RGB owner{owner} pixel{x},{y} expected{color:?} actual{actual:?}"
                    );
                    red += usize::from(current.1 == 1);
                    blue += usize::from(current.1 == 0);
                    hfov_diff +=
                        usize::from(current.1 == 1 && (!hfov.0.is_finite() || hfov.1 != 1));
                    zero_diff += usize::from(
                        current.1 != zero.1 || current.0.is_infinite() != zero.0.is_infinite(),
                    );
                } else {
                    checked += 1;
                    let offset = (y * 64 + x) * 4;
                    let actual = f32::from_le_bytes(v.data[offset..offset + 4].try_into().unwrap());
                    if current.0.is_infinite() {
                        assert_eq!(
                            actual,
                            f32::INFINITY,
                            "signed depth owner{owner} pixel{x},{y} expected+INF actual{actual}"
                        );
                        infinite += 1;
                    } else {
                        assert!(
                            actual.is_finite()
                                && actual > 0.0
                                && (f64::from(actual) - current.0).abs() < 1e-4,
                            "signed depth owner{owner} pixel{x},{y} expected{} actual{actual}",
                            current.0
                        );
                    }
                    hfov_diff += usize::from(
                        current.0.is_infinite() != hfov.0.is_infinite()
                            || (current.0.is_finite()
                                && hfov.0.is_finite()
                                && (current.0 - hfov.0).abs() > 0.1),
                    );
                    zero_diff += usize::from(
                        current.0.is_infinite() != zero.0.is_infinite()
                            || (current.0.is_finite()
                                && zero.0.is_finite()
                                && (current.0 - zero.0).abs() > 0.1),
                    );
                }
            }
        }
        if owner == 0 {
            assert_eq!(checked + excluded, 4096);
            assert!(excluded > 0);
            assert!(
                red > 20 && blue > 100 && hfov_diff > 20 && zero_diff > 0,
                "signed RGB checked{checked} excluded{excluded} red{red} blue{blue} HFOV{hfov_diff} zero{zero_diff}"
            );
            // At pixel40,25 all4corners lie strictly in the redfront and its
            // center keeps0.03m margin. s0 and s-36 miss it; no MSAA pattern guess.
            let actual = hit(0, 40.5, 25.5, CALIBRATION[0], 36.0);
            assert!(actual.1 == 1 && actual.2 && homogeneous(0, 40, 25, actual.1, actual.0));
            for u in [40.0, 41.0] {
                for v in [25.0, 26.0] {
                    let d = ray(CALIBRATION[0], 36.0, u, v);
                    assert!(
                        (1.8 * d[1]).abs() < 0.17 && (1.8 * d[2]).abs() < 0.17,
                        "all4positive36 corners retain0.03m margin"
                    );
                    let zero = ray(CALIBRATION[0], 0.0, u, v);
                    assert!(
                        (1.8 * zero[1]).abs() > 0.2,
                        "whole zero-skew footprint misses foreground"
                    );
                }
            }
            for s in [0.0, -36.0] {
                assert_ne!(hit(0, 40.5, 25.5, CALIBRATION[0], s).1, 1);
            }
            assert_eq!(
                &v.data[(25 * 64 + 40) * 3..(25 * 64 + 40) * 3 + 3],
                [255, 0, 0],
                "positive-skew sign discriminator40,25"
            );
        } else {
            assert_eq!((checked, excluded), (4096, 0));
            assert!(
                infinite > 100 && hfov_diff > 100 && zero_diff > 100,
                "signed depth +INF{infinite} HFOV{hfov_diff} zero{zero_diff}"
            );
            // Literal backdrop bounds guarantee80bottom-left pixels x0..9/
            // y56..63 plus32top-right pixels x60..63/y0..7 are +INF under-48.
            // Both zeroK and HFOV hit the backdrop there:112>100, not sampled limits.
            for (xs, ys) in [(0..10, 56..64), (60..64, 0..8)] {
                for y in ys {
                    for x in xs.clone() {
                        let offset = (y * 64 + x) * 4;
                        assert_eq!(
                            f32::from_le_bytes(v.data[offset..offset + 4].try_into().unwrap()),
                            f32::INFINITY,
                            "guaranteed background{x},{y}"
                        );
                    }
                }
            }
            assert_eq!(
                f32::from_le_bytes(v.data[(63 * 64) * 4..(63 * 64) * 4 + 4].try_into().unwrap()),
                f32::INFINITY
            );
            assert!(
                hit(1, 0.5, 63.5, CALIBRATION[1], 48.0).0.is_finite(),
                "opposite sign must differ at0,63"
            );
        }
        v
    }
    fn publish(
        r: &mut Rig,
        owner: usize,
        sub: &Subscriber,
        stamp: Duration,
        sequence: u64,
        p: bool,
    ) -> Image {
        assert_eq!(
            r.app.pump_camera(r.ids[owner]).unwrap(),
            Some(Progress::ImagePublished)
        );
        image(receive(sub, b"gz.msgs.Image"), owner, stamp, sequence, p)
    }
    fn original_metadata(r: &Rig, explicit_p: bool) {
        for (owner, id) in r.ids.iter().copied().enumerate() {
            let e = entity_by_id(r.app.server().simulation().world(), id).unwrap();
            let s = r
                .app
                .server()
                .simulation()
                .world()
                .get::<gz_sim_scene::CameraSensor>(e)
                .unwrap();
            let c = s.0.camera().unwrap();
            assert_eq!(c.lens.intrinsics.skew, SKEW[owner]);
            assert_eq!(c.lens.has_projection, explicit_p);
            assert_eq!(
                [
                    c.lens.intrinsics.fx,
                    c.lens.intrinsics.fy,
                    c.lens.intrinsics.cx,
                    c.lens.intrinsics.cy
                ],
                CALIBRATION[owner]
            );
            if explicit_p {
                assert_eq!(
                    [
                        c.lens.projection.fx,
                        c.lens.projection.fy,
                        c.lens.projection.cx,
                        c.lens.projection.cy
                    ],
                    K[owner]
                );
            }
        }
    }
    fn canonical_first(
        r: &mut Rig,
        explicit_p: bool,
    ) -> ([Subscriber; 2], [Subscriber; 2], [Worker; 2]) {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .parent()
            .unwrap();
        let peer = selected_native::NativePeer::build(root, "camera");
        let mode = if explicit_p {
            "signed-skew-projection"
        } else {
            "signed-skew"
        };
        let mut cpp: [native::Native; 2] = std::array::from_fn(|owner| {
            native::Native::start_mode(
                &r.endpoint,
                &r.app.server().options().partition,
                &peer,
                owner,
                mode,
            )
        });
        assert_ne!(cpp[0].pid(), cpp[1].pid());
        for (owner, consumer) in cpp.iter().enumerate() {
            assert_eq!(consumer.line(), format!("ready {mode} {owner}"));
        }
        r.matching([(true, true); 2]);
        for id in r.ids {
            assert!(r.app.camera_worker_diagnostics(id).is_none());
        }
        let images: [Subscriber; 2] = std::array::from_fn(|owner| sub(r, owner, true));
        let infos: [Subscriber; 2] = std::array::from_fn(|owner| sub(r, owner, false));
        let first = r.tick();
        assert_eq!(first, Duration::from_millis(1));
        let rows = r.capture();
        for id in r.ids {
            assert_eq!(requested(&rows, id), Some(Progress::Pending));
        }
        let calibration: [CameraInfo; 2] = std::array::from_fn(|owner| {
            info(
                receive(&infos[owner], b"gz.msgs.CameraInfo"),
                owner,
                first,
                explicit_p,
            )
        });
        r.ready(&[0, 1]);
        let workers = [worker(r, 0), worker(r, 1)];
        assert_ne!(workers[0].pid, workers[1].pid);
        let frames: [Image; 2] =
            std::array::from_fn(|owner| publish(r, owner, &images[owner], first, 0, explicit_p));
        let mut sizes = [0; 2];
        for owner in 0..2 {
            for (kind, expected) in [
                ("IMAGE", frames[owner].encode_to_vec()),
                ("INFO", calibration[owner].encode_to_vec()),
            ] {
                let line = cpp[owner].line();
                let (actual, raw) = line.split_once(' ').unwrap();
                assert_eq!(actual, kind);
                assert_eq!(
                    hex(raw),
                    expected,
                    "actual signed-skew canonical full bytes"
                );
            }
            assert_eq!(cpp[owner].line(), format!("PASS canonical {mode} {owner}"));
            sizes[owner] = cpp[owner].finish();
            assert_eq!(cpp[owner].actual_exit(), Some(0));
        }
        let receipt=serde_json::to_vec_pretty(&serde_json::json!({"actual_consumer_exit_codes":[cpp[0].actual_exit(),cpp[1].actual_exit()],"actual_reaped":[true,true],
            "consumer_PIDs":[cpp[0].pid(),cpp[1].pid()],"stdout_bytes":sizes,"per_stdout_cap":65536,"per_stderr_cap":4096,"aggregate_cap":131072,
            "K":CALIBRATION,"skew":SKEW,"explicit_P":explicit_p,"P_when_explicit":K,"sensor_ids":r.ids,"worker_exit_codes":"UNKNOWN",
            "scope":"Actual canonical consumer interoperability and software images; native producer skew omission is Source-derived, not a new native capture."})).unwrap();
        assert!(sizes.into_iter().sum::<usize>() + receipt.len() <= 131072);
        let mut saved = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(
                peer.binary
                    .parent()
                    .unwrap()
                    .join(format!("m74-{mode}-consumers.json")),
            )
            .unwrap();
        std::io::Write::write_all(&mut saved, &receipt).unwrap();
        saved.sync_all().unwrap();
        (images, infos, workers)
    }
    #[test]
    fn signed_skew_real_workers_and_canonical_consumers_keep_sheared_rays_cadence_and_reset() {
        if isolation::child(
            "skew::signed_skew_real_workers_and_canonical_consumers_keep_sheared_rays_cadence_and_reset",
            180,
        ) {
            return;
        }
        let mut r = Rig::with_xml(true, &source(false), "m74-camera-skew.sdf");
        original_metadata(&r, false);
        let ids = r.ids;
        let (images, infos, old) = canonical_first(&mut r, false);
        let first = Duration::from_millis(1);
        let mut stamps: [Vec<Duration>; 2] = std::array::from_fn(|_| vec![first]);
        while r.app.server().simulation().elapsed() < Duration::from_millis(200) {
            let stamp = r.tick();
            let rows = r.capture();
            let mut due = Vec::new();
            for (owner, sub) in infos.iter().enumerate() {
                match requested(&rows, ids[owner]) {
                    Some(Progress::Pending) => {
                        info(receive(sub, b"gz.msgs.CameraInfo"), owner, stamp, false);
                        due.push(owner);
                    }
                    None => {}
                    other => panic!("skew cadence {other:?}"),
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
                    false,
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
        for sub in images.iter().chain(&infos) {
            assert!(sub.try_recv().unwrap().is_none());
        }
        r.reset();
        for w in &old {
            retired(w);
        }
        assert_eq!(r.ids, ids);
        original_metadata(&r, false);
        r.matching([(true, true); 2]);
        r.pause(false);
        assert_eq!(r.tick(), first);
        let rows = r.capture();
        for id in ids {
            assert_eq!(requested(&rows, id), Some(Progress::Pending));
        }
        for (owner, sub) in infos.iter().enumerate() {
            info(receive(sub, b"gz.msgs.CameraInfo"), owner, first, false);
        }
        r.ready(&[0, 1]);
        let fresh = [worker(&r, 0), worker(&r, 1)];
        for (owner, sub) in images.iter().enumerate() {
            publish(&mut r, owner, sub, first, 0, false);
        }
        r.app.shutdown();
        for w in &fresh {
            retired(w);
        }
        r.observer.close().wait().unwrap();
    }
    #[test]
    fn signed_skew_explicit_p_override_keeps_lazy_messages_and_discards_drifted_frames() {
        if isolation::child(
            "skew::signed_skew_explicit_p_override_keeps_lazy_messages_and_discards_drifted_frames",
            180,
        ) {
            return;
        }
        // Independent info-only path has no native consumer image interest.
        let mut lazy = Rig::with_xml(true, &source(true), "m74-skew-projection-lazy.sdf");
        assert_eq!(lazy.tick(), Duration::from_millis(1));
        for row in lazy.capture() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::NoDemand));
        }
        let infos: [Subscriber; 2] = std::array::from_fn(|owner| sub(&lazy, owner, false));
        lazy.matching([(false, true); 2]);
        let stamp = lazy.tick();
        assert_eq!(stamp, Duration::from_millis(2));
        for row in lazy.capture() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::InfoPublished));
        }
        for (owner, sub) in infos.iter().enumerate() {
            info(receive(sub, b"gz.msgs.CameraInfo"), owner, stamp, true);
            assert!(
                lazy.app
                    .camera_worker_diagnostics(lazy.ids[owner])
                    .is_none()
            );
        }
        lazy.app.shutdown();
        lazy.observer.close().wait().unwrap();
        let mut r = Rig::with_xml(true, &source(true), "m74-skew-projection.sdf");
        original_metadata(&r, true);
        let ids = r.ids;
        let (images, infos, old) = canonical_first(&mut r, true);
        r.advance(100);
        let rows = r.capture();
        for id in ids {
            assert_eq!(requested(&rows, id), Some(Progress::Pending));
        }
        for (owner, sub) in infos.iter().enumerate() {
            info(
                receive(sub, b"gz.msgs.CameraInfo"),
                owner,
                Duration::from_millis(100),
                true,
            );
        }
        r.ready(&[0, 1]);
        r.pause(true);
        r.remove.store(5, Ordering::Release);
        assert_eq!(r.tick(), Duration::from_millis(100));
        assert!(matches!(
            r.app.pump_camera(ids[0]),
            Err(CameraServerError::Camera(
                gz_camera_output::CameraOutputError::OwnerInvalid
            ))
        ));
        for w in &old {
            retired(w);
        }
        assert!(!r.app.has_camera_output());
        for sub in &images {
            assert!(sub.try_recv().unwrap().is_none());
        }
        assert!(!r.app.server().zenoh_session().is_closed());
        r.reset();
        assert_eq!(r.ids, ids);
        original_metadata(&r, true);
        r.matching([(true, true); 2]);
        r.pause(false);
        let first = r.tick();
        assert_eq!(first, Duration::from_millis(1));
        let rows = r.capture();
        for id in ids {
            assert_eq!(requested(&rows, id), Some(Progress::Pending));
        }
        for (owner, sub) in infos.iter().enumerate() {
            info(receive(sub, b"gz.msgs.CameraInfo"), owner, first, true);
        }
        r.ready(&[0, 1]);
        let fresh = [worker(&r, 0), worker(&r, 1)];
        for (owner, sub) in images.iter().enumerate() {
            publish(&mut r, owner, sub, first, 0, true);
        }
        r.pause(true);
        r.remove.store(1, Ordering::Release);
        assert_eq!(r.tick(), first);
        assert_eq!(r.app.pump_camera(ids[0]).unwrap(), None);
        retired(&fresh[0]);
        assert_eq!(proc_identity(fresh[1].pid).unwrap().0, fresh[1].start);
        assert!(!fresh[1].lifetime.is_retired());
        r.remove.store(2, Ordering::Release);
        assert_eq!(r.tick(), first);
        assert_eq!(r.app.pump_camera(ids[1]).unwrap(), None);
        retired(&fresh[1]);
        assert!(!r.app.has_camera_output());
        assert!(!r.app.server().zenoh_session().is_closed());
        r.app.shutdown();
        r.observer.close().wait().unwrap();
    }
}

// M75 keeps earlier oracles unchanged and filters these independent literal
// box intersections with the same declared bitwise visibility predicate.
mod visibility {
    use super::*;
    const XML: &str = include_str!("../../../examples/m75_camera_visibility.sdf");
    const FLAGS: [u32; 3] = [4, 1, 2];
    // Renderer leaves ClearColorConfig::Default unchanged. Pinned Bevy0.20.0
    // bevy_camera/src/clear_color.rs returns srgb_u8(43,44,47). This is Source
    // rationale for a NEW oracle, not a claim an empty frame was observed.
    const CLEAR: [u8; 3] = [43, 44, 47];
    const CENTERS: [[f64; 3]; 3] = [[6.0, 0.0, 1.0], [2.0, -2.0, 1.0], [3.0, 2.0, 1.0]];
    const HALVES: [[f64; 3]; 3] = [[0.1, 8.0, 8.0], [0.2; 3], [0.2; 3]];
    fn source(masks: [u32; 2], defaults: bool) -> String {
        if defaults {
            return XML
                .replace("<visibility_mask>5</visibility_mask>", "")
                .replace("<visibility_mask>6</visibility_mask>", "")
                .replace("<visibility_flags>4</visibility_flags>", "")
                .replace("<visibility_flags>1</visibility_flags>", "")
                .replace("<visibility_flags>2</visibility_flags>", "");
        }
        XML.replace("<visibility_mask>5</visibility_mask>", "<m75_rgb_mask/>")
            .replace("<visibility_mask>6</visibility_mask>", "<m75_depth_mask/>")
            .replace(
                "<m75_rgb_mask/>",
                &format!("<visibility_mask>{}</visibility_mask>", masks[0]),
            )
            .replace(
                "<m75_depth_mask/>",
                &format!("<visibility_mask>{}</visibility_mask>", masks[1]),
            )
    }
    fn sub(r: &Rig, owner: usize, image: bool) -> Subscriber {
        let topic = format!(
            "/m75/{}/{}",
            if owner == 0 { "rgb" } else { "depth" },
            if image { "image" } else { "camera_info" }
        );
        r.observer
            .declare_subscriber(r.app.server().options().topic_key(&topic).unwrap())
            .wait()
            .unwrap()
    }
    fn hit(owner: usize, x: usize, y: usize, mask: u32, flags: [u32; 3]) -> (f64, usize, bool) {
        let [fx, fy, cx, cy] = K[owner];
        let origin = [0.0, if owner == 0 { -2.0 } else { 2.0 }, 1.0];
        let ray = [1.0, (cx - x as f64 - 0.5) / fx, (cy - y as f64 - 0.5) / fy];
        let mut best = (f64::INFINITY, 0, false);
        for (box_id, (center, half)) in CENTERS.iter().zip(HALVES).enumerate() {
            if flags[box_id] & mask == 0 {
                continue;
            }
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
                best = (
                    enter,
                    box_id,
                    (enter - (center[0] - half[0])).abs() < 1e-10
                        && (origin[1] + enter * ray[1] - center[1]).abs() < half[1] - 0.03
                        && (origin[2] + enter * ray[2] - center[2]).abs() < half[2] - 0.03,
                );
            }
        }
        best
    }
    fn homogeneous(
        owner: usize,
        x: usize,
        y: usize,
        current: (f64, usize, bool),
        mask: u32,
        flags: [u32; 3],
    ) -> bool {
        if current.0.is_finite() {
            if !current.2 {
                return false;
            }
            let b = projected_box_bounds(owner, current.1, true);
            if !(x as f64 > b[0]
                && (x as f64 + 1.0) < b[1]
                && y as f64 > b[2]
                && (y as f64 + 1.0) < b[3])
            {
                return false;
            }
        }
        for (box_id, front) in [5.9, 1.8, 2.8].into_iter().enumerate() {
            if flags[box_id] & mask != 0
                && (!current.0.is_finite() || (box_id != current.1 && front < current.0))
            {
                let b = projected_box_bounds(owner, box_id, false);
                if x as f64 + 1.0 >= b[0]
                    && x as f64 <= b[1]
                    && y as f64 + 1.0 >= b[2]
                    && y as f64 <= b[3]
                {
                    return false;
                }
            }
        }
        true
    }
    fn image(
        wire: Vec<u8>,
        owner: usize,
        stamp: Duration,
        sequence: u64,
        mask: u32,
        flags: [u32; 3],
    ) -> Image {
        let v = Image::decode(wire.as_slice()).unwrap();
        assert_eq!(v.encode_to_vec(), wire);
        header(v.header.as_ref().unwrap(), owner, stamp, Some(sequence));
        assert_eq!((v.width, v.height), (64, 64));
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
        let mut checked = 0;
        let mut excluded = 0;
        let mut red = 0;
        let mut blue = 0;
        let mut clear = 0;
        let mut infinite = 0;
        let mut changed = 0;
        for y in 0..64 {
            for x in 0..64 {
                let expected = hit(owner, x, y, mask, flags);
                let all = hit(owner, x, y, u32::MAX, flags);
                if owner == 0 {
                    if !homogeneous(owner, x, y, expected, mask, flags) {
                        excluded += 1;
                        continue;
                    }
                    checked += 1;
                    let color = if !expected.0.is_finite() {
                        CLEAR
                    } else {
                        match expected.1 {
                            0 => [0, 0, 255],
                            1 => [255, 0, 0],
                            2 => [0, 255, 0],
                            _ => unreachable!(),
                        }
                    };
                    let actual = &v.data[(y * 64 + x) * 3..(y * 64 + x) * 3 + 3];
                    assert_eq!(
                        actual, color,
                        "visibility RGB mask{mask:#x} pixel{x},{y} expected{color:?} actual{actual:?}"
                    );
                    red += usize::from(color == [255, 0, 0]);
                    blue += usize::from(color == [0, 0, 255]);
                    clear += usize::from(color == CLEAR);
                    changed += usize::from(
                        expected.0.is_infinite() != all.0.is_infinite() || expected.1 != all.1,
                    );
                } else {
                    checked += 1;
                    let offset = (y * 64 + x) * 4;
                    let actual = f32::from_le_bytes(v.data[offset..offset + 4].try_into().unwrap());
                    if expected.0.is_infinite() {
                        assert_eq!(
                            actual,
                            f32::INFINITY,
                            "visibility depth mask{mask:#x} pixel{x},{y} expected+INF actual{actual}"
                        );
                        infinite += 1;
                    } else {
                        assert!(
                            actual.is_finite()
                                && actual > 0.0
                                && (f64::from(actual) - expected.0).abs() < 1e-4,
                            "visibility depth mask{mask:#x} pixel{x},{y} expected{} actual{actual}",
                            expected.0
                        );
                    }
                    changed += usize::from(
                        expected.0.is_infinite() != all.0.is_infinite()
                            || (expected.0.is_finite()
                                && all.0.is_finite()
                                && (expected.0 - all.0).abs() > 0.1),
                    );
                }
            }
        }
        if owner == 0 {
            assert_eq!(checked + excluded, 4096);
            if mask == 0 {
                assert_eq!((checked, excluded, clear), (4096, 0, 4096));
            } else {
                assert!(blue > 100);
                if flags[1] & mask != 0 {
                    assert!(red > 20);
                } else {
                    assert_eq!(red, 0);
                    if flags[1] != 0 {
                        assert!(
                            changed > 20,
                            "hidden foreground must change actual checked pixels"
                        );
                    }
                }
            }
            let expected = if mask == 0 {
                CLEAR
            } else if flags[1] & mask != 0 {
                [255, 0, 0]
            } else {
                [0, 0, 255]
            };
            assert_eq!(
                &v.data[(25 * 64 + 20) * 3..(25 * 64 + 20) * 3 + 3],
                expected,
                "selective RGB foreground20,25"
            );
        } else {
            assert_eq!((checked, excluded), (4096, 0));
            assert!(infinite > 100);
            if mask == 0 {
                assert_eq!(infinite, 4096);
            } else if flags[2] & mask == 0 && flags[2] != 0 {
                assert!(changed > 0);
            }
            let offset = (20 * 64 + 42) * 4;
            let actual = f32::from_le_bytes(v.data[offset..offset + 4].try_into().unwrap());
            let expected = if mask == 0 {
                f32::INFINITY
            } else if flags[2] & mask != 0 {
                2.8
            } else {
                5.9
            };
            if expected.is_infinite() {
                assert_eq!(actual, expected);
            } else {
                assert!(
                    (actual - expected).abs() < 1e-4,
                    "selective axial depth42,20 expected{expected} actual{actual}"
                );
            }
        }
        v
    }
    fn publish(
        r: &mut Rig,
        owner: usize,
        sub: &Subscriber,
        stamp: Duration,
        sequence: u64,
        mask: u32,
        flags: [u32; 3],
    ) -> Image {
        assert_eq!(
            r.app.pump_camera(r.ids[owner]).unwrap(),
            Some(Progress::ImagePublished)
        );
        image(
            receive(sub, b"gz.msgs.Image"),
            owner,
            stamp,
            sequence,
            mask,
            flags,
        )
    }
    fn first(
        r: &mut Rig,
        masks: [u32; 2],
        flags: [u32; 3],
        mode: Option<&'static str>,
    ) -> ([Subscriber; 2], [Subscriber; 2], [Worker; 2]) {
        let peers = mode.map(|mode| {
            let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .parent()
                .unwrap()
                .parent()
                .unwrap();
            let peer = selected_native::NativePeer::build(root, "camera");
            let cpp: [native::Native; 2] = std::array::from_fn(|owner| {
                native::Native::start_mode(
                    &r.endpoint,
                    &r.app.server().options().partition,
                    &peer,
                    owner,
                    mode,
                )
            });
            assert_ne!(cpp[0].pid(), cpp[1].pid());
            for (owner, c) in cpp.iter().enumerate() {
                assert_eq!(c.line(), format!("ready {mode} {owner}"));
            }
            // Each topic has a single native subscriber until the Rust probes
            // register. Observe all four remote subscriptions before the first
            // capture so a local probe cannot satisfy a missing remote match.
            r.matching([(true, true); 2]);
            (peer, cpp, mode)
        });
        let images: [Subscriber; 2] = std::array::from_fn(|owner| sub(r, owner, true));
        let infos: [Subscriber; 2] = std::array::from_fn(|owner| sub(r, owner, false));
        r.matching([(true, true); 2]);
        for id in r.ids {
            assert!(r.app.camera_worker_diagnostics(id).is_none());
        }
        let stamp = r.tick();
        assert_eq!(stamp, Duration::from_millis(1));
        let rows = r.capture();
        for id in r.ids {
            assert_eq!(
                requested(&rows, id),
                Some(Progress::Pending),
                "zero mask is image demand, not NoDemand"
            );
        }
        let calibration: [CameraInfo; 2] = std::array::from_fn(|owner| {
            super::info(receive(&infos[owner], b"gz.msgs.CameraInfo"), owner, stamp)
        });
        r.ready(&[0, 1]);
        let workers = [worker(r, 0), worker(r, 1)];
        assert_ne!(workers[0].pid, workers[1].pid);
        let frames: [Image; 2] = std::array::from_fn(|owner| {
            publish(r, owner, &images[owner], stamp, 0, masks[owner], flags)
        });
        if let Some((peer, mut cpp, mode)) = peers {
            let mut sizes = [0; 2];
            for owner in 0..2 {
                for (kind, expected) in [
                    ("IMAGE", frames[owner].encode_to_vec()),
                    ("INFO", calibration[owner].encode_to_vec()),
                ] {
                    let line = cpp[owner].line();
                    let (actual, raw) = line.split_once(' ').unwrap();
                    assert_eq!(actual, kind);
                    assert_eq!(hex(raw), expected, "complete canonical visibility bytes");
                }
                assert_eq!(cpp[owner].line(), format!("PASS canonical {mode} {owner}"));
                sizes[owner] = cpp[owner].finish();
                assert_eq!(cpp[owner].actual_exit(), Some(0));
            }
            let receipt=serde_json::to_vec_pretty(&serde_json::json!({"actual_consumer_exit_codes":[cpp[0].actual_exit(),cpp[1].actual_exit()],"actual_reaped":[true,true],"consumer_PIDs":[cpp[0].pid(),cpp[1].pid()],"stdout_bytes":sizes,"per_stdout_cap":65536,"per_stderr_cap":4096,"aggregate_cap":131072,"masks":masks,"visual_flags":flags,"empty_RGB_expected_clear":CLEAR,"empty_RGB_source_basis":"bevy_camera0.20 ClearColor::default; actual full frame is checked, no changed renderer policy", "worker_exit_codes":"UNKNOWN","scope":"Canonical consumer interoperability/software visibility selection; no native renderer image-equivalence claim."})).unwrap();
            assert!(sizes.into_iter().sum::<usize>() + receipt.len() <= 131072);
            let mut file = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(
                    peer.binary
                        .parent()
                        .unwrap()
                        .join(format!("m75-{mode}-consumers.json")),
                )
                .unwrap();
            std::io::Write::write_all(&mut file, &receipt).unwrap();
            file.sync_all().unwrap();
        }
        (images, infos, workers)
    }
    #[test]
    fn camera_visibility_real_workers_and_canonical_consumers_cover_disjoint_overlap_zero_and_defaults()
     {
        if isolation::child(
            "visibility::camera_visibility_real_workers_and_canonical_consumers_cover_disjoint_overlap_zero_and_defaults",
            180,
        ) {
            return;
        }
        for (masks, defaults, mode) in [
            ([5, 6], false, Some("camera-visibility")),
            ([6, 5], false, Some("camera-visibility-hidden")),
            ([0, 0], false, Some("camera-visibility-zero")),
            ([7, 7], false, None),
            ([u32::MAX; 2], true, None),
        ] {
            let flags = if defaults { [u32::MAX; 3] } else { FLAGS };
            let mut r = Rig::with_xml(true, &source(masks, defaults), "m75-camera-visibility.sdf");
            let (images, infos, workers) = first(&mut r, masks, flags, mode);
            if masks == [5, 6] {
                let mut stamps: [Vec<Duration>; 2] =
                    std::array::from_fn(|_| vec![Duration::from_millis(1)]);
                while r.app.server().simulation().elapsed() < Duration::from_millis(200) {
                    let stamp = r.tick();
                    let rows = r.capture();
                    let mut due = Vec::new();
                    for (owner, s) in infos.iter().enumerate() {
                        match requested(&rows, r.ids[owner]) {
                            Some(Progress::Pending) => {
                                super::info(receive(s, b"gz.msgs.CameraInfo"), owner, stamp);
                                due.push(owner);
                            }
                            None => {}
                            other => panic!("visibility cadence{other:?}"),
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
                            masks[owner],
                            flags,
                        );
                        stamps[owner].push(stamp);
                    }
                }
                assert_eq!(stamps[0], [1, 100, 200].map(Duration::from_millis));
                assert_eq!(stamps[1], [1, 50, 100, 150, 200].map(Duration::from_millis));
            }
            r.pause(true);
            let stamp = r.app.server().simulation().elapsed();
            for _ in 0..8 {
                assert_eq!(r.tick(), stamp);
                for row in r.capture() {
                    assert!(row.result.unwrap().requested.is_none());
                }
            }
            for s in images.iter().chain(&infos) {
                assert!(s.try_recv().unwrap().is_none());
            }
            r.app.shutdown();
            for w in &workers {
                retired(w);
            }
            r.observer.close().wait().unwrap();
        }
    }
    #[test]
    fn camera_visibility_acquisitions_freeze_visual_flags_and_mask_drift_reset_retires_stale_frames()
     {
        if isolation::child(
            "visibility::camera_visibility_acquisitions_freeze_visual_flags_and_mask_drift_reset_retires_stale_frames",
            180,
        ) {
            return;
        }
        let mut r = Rig::with_xml(true, XML, "m75-camera-visibility.sdf");
        let ids = r.ids;
        assert_eq!(r.tick(), Duration::from_millis(1));
        for row in r.capture() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::NoDemand));
        }
        let infos: [Subscriber; 2] = std::array::from_fn(|owner| sub(&r, owner, false));
        r.matching([(false, true); 2]);
        let stamp = r.tick();
        for row in r.capture() {
            assert_eq!(row.result.unwrap().requested, Some(Progress::InfoPublished));
        }
        for (owner, s) in infos.iter().enumerate() {
            super::info(receive(s, b"gz.msgs.CameraInfo"), owner, stamp);
            assert!(r.app.camera_worker_diagnostics(ids[owner]).is_none());
        }
        let rgb = sub(&r, 0, true);
        r.matching([(true, true), (false, true)]);
        r.advance(100);
        let rows = r.capture();
        assert_eq!(requested(&rows, ids[0]), Some(Progress::Pending));
        assert_eq!(requested(&rows, ids[1]), Some(Progress::InfoPublished));
        for (owner, s) in infos.iter().enumerate() {
            super::info(
                receive(s, b"gz.msgs.CameraInfo"),
                owner,
                Duration::from_millis(100),
            );
        }
        // The admitted request already owns its visual snapshot. Hiding the red
        // visual before any pumping cannot mutate that buffered acquisition.
        r.pause(true);
        r.remove.store(7, Ordering::Release);
        assert_eq!(r.tick(), Duration::from_millis(100));
        r.ready(&[0]);
        let old = worker(&r, 0);
        publish(&mut r, 0, &rgb, Duration::from_millis(100), 0, 5, FLAGS);
        assert!(r.app.camera_worker_diagnostics(ids[1]).is_none());
        r.pause(false);
        r.advance(200);
        let rows = r.capture();
        assert_eq!(requested(&rows, ids[0]), Some(Progress::Pending));
        assert_eq!(requested(&rows, ids[1]), Some(Progress::InfoPublished));
        for (owner, s) in infos.iter().enumerate() {
            super::info(
                receive(s, b"gz.msgs.CameraInfo"),
                owner,
                Duration::from_millis(200),
            );
        }
        r.ready(&[0]);
        publish(&mut r, 0, &rgb, Duration::from_millis(200), 1, 5, [4, 0, 2]);
        r.advance(300);
        let rows = r.capture();
        assert_eq!(requested(&rows, ids[0]), Some(Progress::Pending));
        for (owner, s) in infos.iter().enumerate() {
            super::info(
                receive(s, b"gz.msgs.CameraInfo"),
                owner,
                Duration::from_millis(300),
            );
        }
        r.ready(&[0]);
        r.pause(true);
        r.remove.store(6, Ordering::Release);
        assert_eq!(r.tick(), Duration::from_millis(300));
        assert!(matches!(
            r.app.pump_camera(ids[0]),
            Err(CameraServerError::Camera(
                gz_camera_output::CameraOutputError::OwnerInvalid
            ))
        ));
        retired(&old);
        assert!(rgb.try_recv().unwrap().is_none());
        r.reset();
        assert_eq!(r.ids, ids);
        let world = r.app.server().simulation().world();
        for (owner, id) in ids.iter().copied().enumerate() {
            let e = entity_by_id(world, id).unwrap();
            assert_eq!(
                world
                    .get::<gz_sim_scene::CameraSensor>(e)
                    .unwrap()
                    .0
                    .camera()
                    .unwrap()
                    .visibility_mask,
                [5, 6][owner]
            );
        }
        for (name, flag) in [("backdrop", 4), ("occluder_a", 1), ("occluder_b", 2)] {
            let visual = world
                .iter_entities()
                .find(|e| {
                    e.get::<Name>()
                        .is_some_and(|n| n.0 == format!("default::{name}::body::surface"))
                })
                .unwrap();
            assert_eq!(
                visual
                    .get::<gz_sim_scene::VisualProperties>()
                    .unwrap()
                    .0
                    .visibility_flags,
                flag
            );
        }
        let depth = sub(&r, 1, true);
        r.matching([(true, true); 2]);
        r.pause(false);
        let first = r.tick();
        assert_eq!(first, Duration::from_millis(1));
        let rows = r.capture();
        for id in ids {
            assert_eq!(requested(&rows, id), Some(Progress::Pending));
        }
        for (owner, s) in infos.iter().enumerate() {
            super::info(receive(s, b"gz.msgs.CameraInfo"), owner, first);
        }
        r.ready(&[0, 1]);
        let fresh = [worker(&r, 0), worker(&r, 1)];
        publish(&mut r, 0, &rgb, first, 0, 5, FLAGS);
        publish(&mut r, 1, &depth, first, 0, 6, FLAGS);
        r.pause(true);
        r.remove.store(1, Ordering::Release);
        assert_eq!(r.tick(), first);
        assert_eq!(r.app.pump_camera(ids[0]).unwrap(), None);
        retired(&fresh[0]);
        assert_eq!(proc_identity(fresh[1].pid).unwrap().0, fresh[1].start);
        assert!(!fresh[1].lifetime.is_retired());
        r.remove.store(2, Ordering::Release);
        assert_eq!(r.tick(), first);
        assert_eq!(r.app.pump_camera(ids[1]).unwrap(), None);
        retired(&fresh[1]);
        assert!(!r.app.has_camera_output());
        assert!(!r.app.server().zenoh_session().is_closed());
        r.app.shutdown();
        r.observer.close().wait().unwrap();
    }
}
