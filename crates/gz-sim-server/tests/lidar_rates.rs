// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../../../tests/support/zenoh_peer.rs"]
mod selected_native;
use gz_sim_core::EntityId;
use gz_sim_proto::{
    msgs::{Double, LaserScan, PointCloudPacked, WorldControl, WorldReset},
    prost::Message,
};
use gz_sim_scene::{LidarPublication, PhysicsEngine, lidar_rate, load_world_with_sdf_step};
use gz_sim_server::{Server, ServerOptions};
use std::{
    io::{BufRead, BufReader, Read, Write},
    num::NonZeroUsize,
    path::Path,
    process::{Child, Command, Stdio},
    sync::mpsc,
    thread,
    time::{Duration, Instant},
};
use zenoh::Wait;

const XML: &str = "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='rig'><static>true</static><link name='body'><collision name='self'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision><sensor name='scan' type='lidar'><update_rate>100</update_rate><topic>/m115/scan</topic><frame_id>frame</frame_id><lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></horizontal></scan><range><min>0</min><max>20</max></range><noise><type>gaussian</type><mean>0.1</mean><stddev>0.2</stddev></noise></lidar></sensor></link></model><model name='wall'><static>true</static><pose>5 0 0 0 0 0</pose><link name='body'><collision name='wall'><geometry><box><size>2 100 100</size></box></geometry></collision></link></model></world></sdf>";
fn config() -> zenoh::Config {
    let mut c = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        c.insert_json5(key, value).unwrap();
    }
    c
}
fn server(cloud: bool) -> (Server, u64) {
    let sim = load_world_with_sdf_step(
        XML,
        "rates.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<LidarPublication>())
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    (
        Server::with_zenoh_config(
            sim,
            "default",
            ServerOptions {
                partition: format!("lidar-rate-{}", uuid::Uuid::new_v4()),
                lidar_point_clouds: cloud,
                ..Default::default()
            },
            config(),
            NonZeroUsize::new(2).unwrap(),
        )
        .unwrap(),
        id,
    )
}
fn processed(server: &Server) -> u64 {
    let c = server.counters();
    c.lidar_rates_applied + c.lidar_rates_ignored + c.lidar_rates_rejected
}
fn advance(server: &mut Server, count: u32) {
    let before = server.simulation().elapsed();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        multi_step: count,
        ..Default::default()
    }));
    while server.pending_steps() > 0 {
        server.update().unwrap();
    }
    assert_eq!(
        server.simulation().elapsed(),
        before + Duration::from_millis(u64::from(count))
    );
}
fn rate(server: &mut Server, id: u64, value: f64, expected: f64) {
    let session = server.zenoh_session();
    let key = server.options().topic_key("/m115/scan/set_rate").unwrap();
    let ready = Instant::now() + Duration::from_secs(5);
    let query = session
        .declare_querier(key)
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    while !query.matching_status().wait().unwrap().matching() {
        assert!(Instant::now() < ready);
        thread::yield_now();
    }
    let count = processed(server);
    let time = server.simulation().elapsed();
    let queued = server.counters().lidars_queued;
    let before_rate = lidar_rate(server.simulation(), id).unwrap().rate_hz;
    let replies = query
        .get()
        .payload(
            Double {
                data: value,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .wait()
        .unwrap();
    let reply = replies.recv().unwrap();
    let sample = reply.result().unwrap();
    assert!(sample.payload().is_empty() && sample.attachment().is_none());
    assert!(replies.recv().is_err());
    assert_eq!(
        lidar_rate(server.simulation(), id).unwrap().rate_hz,
        before_rate
    );
    while processed(server) == count {
        assert!(Instant::now() < ready);
        server.update().unwrap();
        thread::yield_now();
    }
    assert_eq!(processed(server), count + 1);
    assert_eq!(
        lidar_rate(server.simulation(), id).unwrap().rate_hz,
        expected
    );
    assert_eq!(server.simulation().elapsed(), time);
    assert_eq!(server.counters().lidars_queued, queued);
}
fn stamp(header: &Option<gz_sim_proto::msgs::Header>) -> u64 {
    let t = header.as_ref().unwrap().stamp.as_ref().unwrap();
    u64::try_from(t.sec).unwrap() * 1_000_000_000 + u64::try_from(t.nsec).unwrap()
}
fn scan_message(
    sub: &zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>,
) -> LaserScan {
    let sample = sub.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    LaserScan::decode(sample.payload().to_bytes().as_ref()).unwrap()
}

#[test]
fn real_lidar_rate_handoff_without_interest_keeps_time_and_scan_phase_then_changes_cadence() {
    let (mut server, id) = server(false);
    let session = server.zenoh_session();
    rate(&mut server, id, 50., 50.);
    assert_eq!(server.counters().lidars_queued, 0);
    rate(&mut server, id, 100., 100.);
    let scan = session
        .declare_subscriber(server.options().topic_key("/m115/scan").unwrap())
        .wait()
        .unwrap();
    let end = Instant::now() + Duration::from_secs(5);
    let first = loop {
        assert!(Instant::now() < end);
        advance(&mut server, 1);
        if let Some(s) = scan.try_recv().unwrap() {
            break LaserScan::decode(s.payload().to_bytes().as_ref()).unwrap();
        }
        thread::sleep(Duration::from_millis(1));
    };
    let at = server.simulation().elapsed().as_nanos() as u64;
    assert!(stamp(&first.header) <= at);
    // Drain earlier queued packets without advancing the simulation clock.
    while scan.try_recv().unwrap().is_some() {}
    rate(&mut server, id, 50., 50.);
    let next = (at / 10_000_000 + 1) * 10_000_000;
    let until = next + 40_000_000;
    advance(
        &mut server,
        u32::try_from((until - at) / 1_000_000).unwrap(),
    );
    let mut times = Vec::new();
    while times.len() < 3 {
        let message = scan_message(&scan);
        let t = stamp(&message.header);
        if t > at {
            times.push(t);
        }
    }
    assert_eq!(times, [next, next + 20_000_000, next + 40_000_000]);
    assert_eq!(
        server.counters().imu_rates_applied
            + server.counters().imu_rates_ignored
            + server.counters().imu_rates_rejected,
        0
    );
    server.shutdown();
}

#[test]
fn canonical_lidar_rate_reset_discards_unread_controls_and_reconstructs_authored_rate() {
    let (mut server, id) = server(true);
    let session = server.zenoh_session();
    rate(&mut server, id, 25., 25.);
    let key = server.options().topic_key("/m115/scan/set_rate").unwrap();
    let replies = session
        .get(key)
        .payload(
            Double {
                data: 50.,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    assert!(
        replies
            .recv()
            .unwrap()
            .result()
            .unwrap()
            .payload()
            .is_empty()
    );
    assert!(replies.recv().is_err());
    let before = processed(&server);
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.update().unwrap();
    assert_eq!(server.simulation().elapsed(), Duration::ZERO);
    assert_eq!(lidar_rate(server.simulation(), id).unwrap().rate_hz, 100.);
    assert_eq!(
        processed(&server),
        before,
        "old queued request is not consumed by replacement"
    );
    rate(&mut server, id, 50., 50.);
    server.shutdown();
}

struct Peer {
    child: Child,
    lines: mpsc::Receiver<String>,
    out: Option<thread::JoinHandle<()>>,
    err: Option<thread::JoinHandle<Vec<u8>>>,
}
impl Drop for Peer {
    fn drop(&mut self) {
        if self.child.try_wait().ok().flatten().is_none() {
            let _ = self.child.kill();
        }
        let _ = self.child.wait();
        if let Some(t) = self.out.take() {
            let _ = t.join();
        }
        if let Some(t) = self.err.take() {
            let _ = t.join();
        }
    }
}
impl Peer {
    fn send(&mut self, line: &str) {
        writeln!(self.child.stdin.as_mut().unwrap(), "{line}").unwrap();
    }
    fn line(&mut self, server: &mut Server) -> String {
        let end = Instant::now() + Duration::from_secs(8);
        loop {
            match self.lines.try_recv() {
                Ok(s) => return s,
                Err(mpsc::TryRecvError::Empty) => {}
                Err(e) => panic!("native peer closed: {e}"),
            }
            assert!(Instant::now() < end);
            server.update().unwrap();
            thread::sleep(Duration::from_millis(1));
        }
    }
    fn snapshot(&mut self, server: &mut Server) -> serde_json::Value {
        self.send("snapshot");
        serde_json::from_str(&self.line(server)).unwrap()
    }
}
fn bytes(value: &serde_json::Value) -> Vec<u8> {
    let s = value.as_str().unwrap();
    assert!(s.len() <= 2048 && s.len().is_multiple_of(2));
    s.as_bytes()
        .chunks_exact(2)
        .map(|p| u8::from_str_radix(std::str::from_utf8(p).unwrap(), 16).unwrap())
        .collect()
}

#[test]
fn actual_cpp_lidar_double_empty_service_changes_shared_scan_cloud_acquisition_without_extra_noise_draws()
 {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let fixture = selected_native::NativePeer::build(&root, "lidar_rate");
    let (mut server, id) = server(true);
    let session = server.zenoh_session();
    let endpoints: Vec<_> = session
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|x| x.to_string())
        .collect();
    assert_eq!(endpoints.len(), 1);
    let json = serde_json::json!({"mode":"client","listen":{"endpoints":[]},"connect":{"endpoints":endpoints,"timeout_ms":2000},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},"transport":{"shared_memory":{"enabled":false}}});
    let dir = tempfile::tempdir().unwrap();
    let config = dir.path().join("client.json");
    std::fs::write(&config, json.to_string()).unwrap();
    let mut command = Command::new(&fixture.binary);
    fixture.configure_runtime(&mut command);
    command
        .env("GZ_PARTITION", &server.options().partition)
        .env("GZ_IP", "127.0.0.1")
        .env("ZENOH_CONFIG", &config)
        .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
        .env_remove("GZ_DISCOVERY_MSG_PORT")
        .env_remove("GZ_DISCOVERY_SRV_PORT")
        .env_remove("GZ_DISCOVERY_MULTICAST_IP")
        .env_remove("GZ_TRANSPORT_USERNAME")
        .env_remove("GZ_TRANSPORT_PASSWORD")
        .env_remove("GZ_RELAY")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = command.spawn().unwrap();
    let stdout = child.stdout.take().unwrap();
    let stderr = child.stderr.take().unwrap();
    let (sender, lines) = mpsc::sync_channel(32);
    let out = thread::spawn(move || {
        let mut reader = BufReader::new(stdout.take(2 * 1024 * 1024 + 1));
        let mut total = 0;
        loop {
            let mut line = String::new();
            let n = reader.read_line(&mut line).unwrap();
            if n == 0 {
                break;
            }
            total += n;
            assert!(n <= 131072 && total <= 2 * 1024 * 1024);
            if sender
                .send(line.trim_end_matches(['\r', '\n']).to_owned())
                .is_err()
            {
                break;
            }
        }
    });
    let err = thread::spawn(move || {
        let mut b = Vec::new();
        stderr.take(4097).read_to_end(&mut b).unwrap();
        b
    });
    let mut peer = Peer {
        child,
        lines,
        out: Some(out),
        err: Some(err),
    };
    assert_eq!(peer.line(&mut server), "READY");
    // This is an actual two-stream acquisition barrier, not a fixed discovery sleep.
    assert!(server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    peer.send("captured");
    assert_eq!(peer.line(&mut server), "CAPTURED");
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    server.update().unwrap();
    peer.send("rate 50");
    let before = processed(&server);
    let at = server.simulation().elapsed().as_nanos() as u64;
    let queued = server.counters().lidars_queued;
    assert_eq!(peer.line(&mut server), "SENT");
    let deadline = Instant::now() + Duration::from_secs(8);
    while processed(&server) == before {
        assert!(Instant::now() < deadline);
        server.update().unwrap();
        thread::yield_now();
    }
    assert_eq!(lidar_rate(server.simulation(), id).unwrap().rate_hz, 50.);
    assert_eq!(server.counters().lidars_queued, queued);
    peer.send("arm");
    assert_eq!(peer.line(&mut server), "ARMED");
    let next = (at / 10_000_000 + 1) * 10_000_000;
    advance(
        &mut server,
        u32::try_from((next + 40_000_000 - at) / 1_000_000).unwrap(),
    );
    let mut received = None;
    for _ in 0..20 {
        let v = peer.snapshot(&mut server);
        let scans: Vec<_> = v["scan"]
            .as_array()
            .unwrap()
            .iter()
            .map(|x| LaserScan::decode(bytes(x).as_slice()).unwrap())
            .filter(|x| stamp(&x.header) > at)
            .collect();
        let points: Vec<_> = v["points"]
            .as_array()
            .unwrap()
            .iter()
            .map(|x| PointCloudPacked::decode(bytes(x).as_slice()).unwrap())
            .filter(|x| stamp(&x.header) > at)
            .collect();
        assert!(
            scans.len() <= 3 && points.len() <= 3,
            "unexpected post-control acquisition"
        );
        if scans.len() == 3 && points.len() == 3 {
            received = Some((scans, points));
            break;
        }
        thread::sleep(Duration::from_millis(2));
    }
    let (scans, points) = received.expect("three native scan and cloud acquisitions");
    for (i, (a, b)) in scans.iter().zip(&points).enumerate() {
        assert_eq!(stamp(&a.header), next + i as u64 * 20_000_000);
        assert_eq!(stamp(&a.header), stamp(&b.header));
        assert_eq!(
            (
                a.count,
                a.vertical_count,
                b.width,
                b.height,
                b.point_step,
                b.row_step
            ),
            (3, 1, 3, 1, 18, 54)
        );
        for j in 0..3 {
            let yaw = -0.3 + j as f64 * 0.3;
            for (axis, d) in [yaw.cos(), yaw.sin(), 0.].into_iter().enumerate() {
                let p = j * 18 + axis * 4;
                let actual = f32::from_le_bytes(b.data[p..p + 4].try_into().unwrap());
                assert!((f64::from(actual) - d * a.ranges[j]).abs() < 1e-6);
            }
        }
    }
    peer.send("quit");
    assert_eq!(peer.line(&mut server), "BYE");
    let end = Instant::now() + Duration::from_secs(5);
    let status = loop {
        if let Some(s) = peer.child.try_wait().unwrap() {
            break s;
        }
        assert!(Instant::now() < end);
        thread::yield_now();
    };
    assert!(status.success());
    peer.out.take().unwrap().join().unwrap();
    let err = peer.err.take().unwrap().join().unwrap();
    assert!(err.len() <= 4096 && err.is_empty());
    server.shutdown();
}
