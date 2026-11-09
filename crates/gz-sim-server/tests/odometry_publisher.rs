// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_proto::{
    msgs::{Odometry, OdometryWithCovariance, PoseV, WorldControl, WorldReset},
    prost::{Message, Name as ProtoName},
};
use gz_sim_scene::{
    GroundTruthOdometrySample, Name, PhysicsEngine, load_world_with_engine,
    queue_link_world_velocity,
};
use gz_sim_server::{Error, Server, ServerOptions};
use std::{
    io::Read,
    num::NonZeroUsize,
    path::Path,
    process::{Command, Stdio},
    sync::OnceLock,
    time::{Duration, Instant},
};
use zenoh::Wait;
#[path = "../../../tests/support/zenoh_peer.rs"]
mod zenoh_peer;
const XML: &str = include_str!("../../../examples/m34_odometry_publisher.sdf");
fn load(moving: bool) -> Simulation {
    let mut sim = load_world_with_engine(
        XML,
        "m34.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    if moving {
        let link = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::rig::body"))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        queue_link_world_velocity(&mut sim, link, [1.0, 0.5, 0.0], [0.0, 0.0, 0.4]).unwrap();
    }
    sim
}
fn sample(server: &Server) -> GroundTruthOdometrySample {
    *server
        .simulation()
        .world()
        .iter_entities()
        .find_map(|e| e.get::<GroundTruthOdometrySample>())
        .unwrap()
}
struct Network {
    observer: zenoh::Session,
    clients: zenoh::Config,
    native: tempfile::TempDir,
}
impl Network {
    fn new() -> Self {
        let cfg=zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}},transport:{shared_memory:{enabled:false}}}"#).unwrap();
        let observer = zenoh::open(cfg).wait().unwrap();
        let locators: Vec<_> = observer
            .info()
            .locators()
            .wait()
            .into_iter()
            .map(|v| v.to_string())
            .collect();
        assert_eq!(locators.len(), 1);
        assert!(locators[0].starts_with("tcp/127.0.0.1:"));
        assert_ne!(locators[0], "tcp/127.0.0.1:0");
        let cfg = serde_json::json!({"mode":"client","listen":{"endpoints":[]},"connect":{"endpoints":locators,"timeout_ms":2000},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},"transport":{"shared_memory":{"enabled":false}}});
        let clients = zenoh::Config::from_json5(&cfg.to_string()).unwrap();
        let native = tempfile::tempdir().unwrap();
        std::fs::write(native.path().join("client.json"), cfg.to_string()).unwrap();
        Self {
            observer,
            clients,
            native,
        }
    }
    fn options(&self) -> ServerOptions {
        ServerOptions {
            capacity: 4096,
            partition: format!("m34-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        }
    }
    fn server(&self, sim: Simulation, options: ServerOptions) -> Result<Server, Error> {
        Server::with_zenoh_config(
            sim,
            "default",
            options,
            self.clients.clone(),
            NonZeroUsize::new(8).unwrap(),
        )
    }
}
fn steps(server: &mut Server, count: u32) {
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        multi_step: count,
        ..Default::default()
    }));
    for _ in 0..count {
        server.update().unwrap();
    }
}
#[test]
fn three_canonical_channels_share_actual_model_samples_and_loaded_reset_is_epoch_atomic() {
    let network = Network::new();
    let options = network.options();
    let odom = network
        .observer
        .declare_subscriber(options.topic_key("/m34/odometry").unwrap())
        .wait()
        .unwrap();
    let cov = network
        .observer
        .declare_subscriber(options.topic_key("/m34/odometry_covariance").unwrap())
        .wait()
        .unwrap();
    let tf = network
        .observer
        .declare_subscriber(options.topic_key("/m34/transform").unwrap())
        .wait()
        .unwrap();
    let mut server = network.server(load(true), options).unwrap();
    steps(&mut server, 200);
    let expected = sample(&server);
    assert_eq!(expected.time, Duration::from_millis(200));
    assert!(expected.pose.translation[0] > 2.15);
    assert!(expected.angular[2] > 0.39);
    assert_eq!(server.counters().ground_truth_odometry_queued, 10);
    assert_eq!(
        server.counters().ground_truth_odometry_covariance_queued,
        10
    );
    assert_eq!(server.counters().ground_truth_odometry_tf_queued, 10);
    assert_eq!(
        server.counters().odometries_queued,
        0,
        "no wheel odometry is invented"
    );
    let mut last = None;
    for i in 1..=10 {
        let o = odom.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
        assert_eq!(
            o.attachment().unwrap().to_bytes().as_ref(),
            Odometry::full_name().as_bytes()
        );
        let o = Odometry::decode(o.payload().to_bytes().as_ref()).unwrap();
        let c = cov.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
        assert_eq!(
            c.attachment().unwrap().to_bytes().as_ref(),
            OdometryWithCovariance::full_name().as_bytes()
        );
        let c = OdometryWithCovariance::decode(c.payload().to_bytes().as_ref()).unwrap();
        let t = tf.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
        assert_eq!(
            t.attachment().unwrap().to_bytes().as_ref(),
            PoseV::full_name().as_bytes()
        );
        let t = PoseV::decode(t.payload().to_bytes().as_ref()).unwrap();
        let h = o.header.as_ref().unwrap();
        assert_eq!(h.data.len(), 2);
        assert_eq!(h.data[0].value, ["m34/odom"]);
        assert_eq!(h.data[1].value, ["m34/base"]);
        assert_eq!(h.stamp.as_ref().unwrap().nsec, i * 20_000_000);
        assert_eq!(c.header, o.header);
        assert_eq!(t.pose.len(), 1);
        assert_eq!(t.pose[0].header, o.header);
        assert!(t.header.is_none());
        assert_eq!(c.pose_with_covariance.as_ref().unwrap().pose, o.pose);
        assert_eq!(c.twist_with_covariance.as_ref().unwrap().twist, o.twist);
        assert_eq!(
            c.pose_with_covariance.unwrap().covariance.unwrap().data,
            vec![0.0; 36]
        );
        assert_eq!(
            c.twist_with_covariance.unwrap().covariance.unwrap().data,
            vec![0.0; 36]
        );
        let pose = o.pose.as_ref().unwrap();
        assert_eq!(pose.position.as_ref().unwrap().z, 0.0);
        last = Some(o);
    }
    let last = last.unwrap();
    let p = last.pose.unwrap().position.unwrap();
    assert_eq!([p.x, p.y, p.z], expected.pose.translation);
    let v = last.twist.unwrap().linear.unwrap();
    assert_eq!([v.x, v.y, v.z], expected.linear);
    server.update().unwrap();
    assert_eq!(server.counters().ground_truth_odometry_queued, 10);
    assert_eq!(sample(&server).time, expected.time);
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.update().unwrap();
    assert_eq!(
        (server.simulation().epoch(), server.simulation().elapsed()),
        (1, Duration::ZERO)
    );
    assert!(
        !server
            .simulation()
            .world()
            .iter_entities()
            .any(|e| e.contains::<GroundTruthOdometrySample>())
    );
    steps(&mut server, 20);
    let reset = sample(&server);
    assert_eq!(reset.epoch, 1);
    assert_eq!(reset.pose.translation, [2.0, -1.0, 0.0]);
    assert_eq!(reset.linear, [0.0; 3]);
    server.shutdown();
}
#[test]
fn final_endpoint_alias_collisions_refuse_initial_setup_and_no_subscribers_do_not_stop_history() {
    let network = Network::new();
    let mut options = network.options();
    options
        .topic_remaps
        .insert("/m34/transform".into(), "/m34/odometry".into());
    assert!(matches!(
        network.server(load(false), options),
        Err(Error::GroundTruthOdometryIdentity)
    ));
    let mut server = network.server(load(false), network.options()).unwrap();
    steps(&mut server, 100);
    let reading = sample(&server);
    assert_eq!(reading.linear, [0.0; 3]);
    assert_eq!(reading.angular, [0.0; 3]);
    assert_eq!(reading.time, Duration::from_millis(100));
    assert_eq!(server.counters().ground_truth_odometry_queued, 5);
    assert_eq!(server.counters().ground_truth_odometry_invalid_samples, 0);
    server.shutdown();
}
fn native_peer() -> &'static zenoh_peer::NativePeer {
    static PEER: OnceLock<zenoh_peer::NativePeer> = OnceLock::new();
    PEER.get_or_init(|| {
        zenoh_peer::NativePeer::build(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."),
            "odometry_publisher",
        )
    })
}
struct Peer(std::process::Child);
impl Drop for Peer {
    fn drop(&mut self) {
        if self.0.try_wait().unwrap().is_none() {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
}
#[test]
fn actual_cpp_consumer_checks_odometry_covariance_and_transform_under_canonical_control() {
    let network = Network::new();
    let options = network.options();
    let mut server = network.server(load(true), options.clone()).unwrap();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    server.update().unwrap();
    let native = native_peer();
    let mut command = Command::new(&native.binary);
    native.configure_runtime(&mut command);
    command
        .env("GZ_PARTITION", &options.partition)
        .env("GZ_IP", "127.0.0.1")
        .env("ZENOH_CONFIG", network.native.path().join("client.json"))
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut peer = Peer(command.spawn().unwrap());
    let deadline = Instant::now() + Duration::from_secs(35);
    let mut failure = None;
    let status = loop {
        if let Err(error) = server.update() {
            failure = Some(format!("server update: {error}"));
            break None;
        }
        if let Some(status) = peer.0.try_wait().unwrap() {
            break Some(status);
        }
        if Instant::now() >= deadline {
            failure = Some("native odometry peer 35s deadline".to_owned());
            break None;
        }
        std::thread::yield_now();
    };
    // Retire only this owned helper before reading both pipes. Keep diagnostics
    // even on an actual helper failure or deadline; never publish a status alone.
    let status = status.unwrap_or_else(|| {
        let _ = peer.0.kill();
        peer.0.wait().unwrap()
    });
    let mut stdout = Vec::new();
    let mut stderr = Vec::new();
    peer.0
        .stdout
        .take()
        .unwrap()
        .take(16 * 1024 + 1)
        .read_to_end(&mut stdout)
        .unwrap();
    peer.0
        .stderr
        .take()
        .unwrap()
        .take(16 * 1024 + 1)
        .read_to_end(&mut stderr)
        .unwrap();
    assert!(
        failure.is_none()
            && status.success()
            && stdout.len() <= 16 * 1024
            && stderr.len() <= 16 * 1024,
        "native peer status={status}, failure={failure:?}, stdout={}, stderr={}",
        String::from_utf8_lossy(&stdout),
        String::from_utf8_lossy(&stderr)
    );
    assert_eq!(
        stdout,
        b"ground-truth odometry conformance passed\n",
        "stderr={}",
        String::from_utf8_lossy(&stderr)
    );
    assert!(
        stderr.is_empty(),
        "native peer stderr={}",
        String::from_utf8_lossy(&stderr)
    );
    assert!(server.counters().ground_truth_odometry_queued >= 10);
    assert_eq!(server.counters().ground_truth_odometry_failures, 0);
    server.shutdown();
}
