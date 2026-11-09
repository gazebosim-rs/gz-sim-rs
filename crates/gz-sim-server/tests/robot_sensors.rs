// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Real M19 robot, unchanged old streams, and simultaneous canonical IMU/lidar.
use gz_sensors::ImuReading;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_proto::{
    msgs::{Header, Imu, LaserScan, Time, Twist, Vector3d, WorldControl, WorldReset},
    prost::{Message, Name as ProtoName},
};
use gz_sim_scene::{
    DiffDriveSample, ImuPublication, ImuSample, LidarPublication, LidarSample, Name, PhysicsEngine,
    WorldPose, load_world_seeded, remove_model, reset_loaded_world, set_imu_interest,
    set_lidar_interest,
};
use gz_sim_server::{Error, InterestPolicy, Server, ServerOptions};
use std::{
    num::NonZeroUsize,
    path::Path,
    process::{Child, Command, Stdio},
    sync::OnceLock,
    time::{Duration, Instant},
};
use zenoh::Wait;

#[path = "../../../tests/support/zenoh_peer.rs"]
mod zenoh_peer;
const XML: &str = include_str!("../../../examples/m33_robot_sensors.sdf");

fn load() -> Simulation {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/m33_robot_sensors.sdf")
        .canonicalize()
        .unwrap();
    // The file URI preserves the existing relative OBJ resolver contract.
    load_world_seeded(
        XML,
        &format!("file://{}", path.display()),
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        [33; 32],
    )
    .unwrap()
}
fn entity(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn identity(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .get::<EntityId>(entity(sim, name))
        .unwrap()
        .get()
}
fn config(connect: &[String]) -> zenoh::Config {
    let value = serde_json::json!({
        "mode":"peer", "listen":{"endpoints":if connect.is_empty() {vec!["tcp/127.0.0.1:0"]} else {vec![]}},
        "connect":{"endpoints":connect,"timeout_ms":2000},
        "scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},
        "transport":{"shared_memory":{"enabled":false}}
    });
    zenoh::Config::from_json5(&value.to_string()).unwrap()
}
struct Network {
    observer: zenoh::Session,
    clients: zenoh::Config,
    directory: tempfile::TempDir,
}
impl Network {
    fn new() -> Self {
        // Inspect an actually owned listener: never drop a speculative reservation.
        let observer = zenoh::open(config(&[])).wait().unwrap();
        let locators: Vec<_> = observer
            .info()
            .locators()
            .wait()
            .into_iter()
            .map(|locator| locator.to_string())
            .collect();
        assert_eq!(locators.len(), 1);
        assert!(locators[0].starts_with("tcp/127.0.0.1:"));
        assert_ne!(locators[0], "tcp/127.0.0.1:0");
        let clients = config(&locators);
        let directory = tempfile::tempdir().unwrap();
        // Same explicit endpoint is consumed by the qualified native peer.
        let native = serde_json::json!({"mode":"client","listen":{"endpoints":[]},
            "connect":{"endpoints":locators,"timeout_ms":2000},
            "scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},
            "transport":{"shared_memory":{"enabled":false}}});
        std::fs::write(directory.path().join("client.json"), native.to_string()).unwrap();
        Self {
            observer,
            clients,
            directory,
        }
    }
    fn options(&self) -> ServerOptions {
        ServerOptions {
            capacity: 4096,
            partition: format!("m33-{}", uuid::Uuid::new_v4()),
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
fn pause(server: &mut Server) {
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    server.update().unwrap();
}
fn reset(server: &mut Server, epoch: u64) {
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
        (epoch, Duration::ZERO)
    );
}
fn vector([x, y, z]: [f64; 3]) -> Vector3d {
    Vector3d {
        x,
        y,
        z,
        ..Default::default()
    }
}
fn twist(linear: f64, angular: f64) -> Twist {
    Twist {
        linear: Some(vector([linear, 0.0, 0.0])),
        angular: Some(vector([0.0, 0.0, angular])),
        ..Default::default()
    }
}
fn stamp(time: Duration) -> Time {
    Time {
        sec: time.as_secs() as i64,
        nsec: time.subsec_nanos() as i32,
    }
}
fn header(header: &Header, time: Duration, frame: &str, seq: usize) {
    assert_eq!(header.stamp, Some(stamp(time)));
    assert_eq!(header.data.len(), 2);
    assert_eq!(
        (&*header.data[0].key, &header.data[0].value),
        ("frame_id", &vec![frame.to_owned()])
    );
    assert_eq!(
        (&*header.data[1].key, &header.data[1].value),
        ("seq", &vec![seq.to_string()])
    );
}
fn decode<T: Message + Default + ProtoName>(sample: zenoh::sample::Sample, key: &str) -> T {
    assert_eq!(sample.key_expr().as_str(), key);
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        T::full_name().as_bytes()
    );
    T::decode(sample.payload().to_bytes().as_ref()).unwrap()
}
#[derive(Clone, Debug, PartialEq)]
struct ImuFrame {
    identity: u64,
    epoch: u64,
    time: Duration,
    reading: ImuReading,
}
#[derive(Clone, Debug)]
struct ScanFrame {
    identity: u64,
    epoch: u64,
    time: Duration,
    pose: WorldPose,
    ranges: Vec<f64>,
}
fn imu(sim: &Simulation) -> ImuFrame {
    let e = entity(sim, "default::robot::base_link::imu");
    let s = sim.world().get::<ImuSample>(e).unwrap();
    ImuFrame {
        identity: sim.world().get::<EntityId>(e).unwrap().get(),
        epoch: s.epoch,
        time: s.time,
        reading: *s.reading.as_ref().unwrap(),
    }
}
fn scan(sim: &Simulation) -> ScanFrame {
    let e = entity(sim, "default::robot::base_link::lidar");
    let s = sim.world().get::<LidarSample>(e).unwrap();
    ScanFrame {
        identity: sim.world().get::<EntityId>(e).unwrap().get(),
        epoch: s.epoch,
        time: s.time,
        pose: s.world_pose,
        ranges: s.ranges.clone(),
    }
}
fn check_imu(m: Imu, s: &ImuFrame, seq: usize) {
    header(
        m.header.as_ref().unwrap(),
        s.time,
        "robot/base_link/imu",
        seq,
    );
    assert_eq!(m.entity_name, "robot::base_link::imu");
    let q = m.orientation.unwrap();
    assert_eq!([q.x, q.y, q.z, q.w], s.reading.orientation);
    assert_eq!(m.angular_velocity, Some(vector(s.reading.angular_velocity)));
    assert_eq!(
        m.linear_acceleration,
        Some(vector(s.reading.specific_force))
    );
    for (actual, diagonal) in [
        m.orientation_covariance,
        m.angular_velocity_covariance,
        m.linear_acceleration_covariance,
    ]
    .into_iter()
    .zip([
        [0.0; 3],
        s.reading.angular_velocity_covariance,
        s.reading.linear_acceleration_covariance,
    ]) {
        assert_eq!(
            actual.unwrap().data,
            vec![
                diagonal[0] as f32,
                0.0,
                0.0,
                0.0,
                diagonal[1] as f32,
                0.0,
                0.0,
                0.0,
                diagonal[2] as f32
            ]
        );
    }
    assert_eq!(
        s.reading.linear_acceleration_covariance,
        [0.0001_f32 as f64, 0.0, 0.0]
    );
    assert_eq!(s.reading.angular_velocity_covariance, [0.0; 3]);
    assert!(
        s.reading
            .specific_force
            .into_iter()
            .chain(s.reading.angular_velocity)
            .all(f64::is_finite)
    );
}
fn check_scan(m: LaserScan, s: &ScanFrame, seq: usize) {
    header(
        m.header.as_ref().unwrap(),
        s.time,
        "robot/base_link/lidar",
        seq,
    );
    assert_eq!(m.frame, "robot/base_link/lidar");
    assert_eq!((m.count, m.vertical_count), (3, 1));
    assert_eq!(
        (m.angle_min, m.angle_max, m.angle_step),
        (
            -std::f64::consts::FRAC_PI_4,
            std::f64::consts::FRAC_PI_4,
            std::f64::consts::FRAC_PI_4
        )
    );
    assert_eq!(
        (
            m.vertical_angle_min,
            m.vertical_angle_max,
            m.vertical_angle_step
        ),
        (0.0, 0.0, 0.0)
    );
    assert_eq!((m.range_min, m.range_max), (0.0, 10.0));
    assert_eq!(m.ranges, s.ranges);
    assert_eq!(m.intensities, [0.0; 3]);
    assert!(
        m.ranges
            .iter()
            .all(|v| *v == f64::INFINITY || v.is_finite() && (0.0..=10.0).contains(v))
    );
    let pose = m.world_pose.unwrap();
    assert!(pose.header.is_none() && pose.name.is_empty() && pose.id == 0);
    assert_eq!(pose.position, Some(vector(s.pose.translation)));
    let q = pose.orientation.unwrap();
    assert_eq!([q.x, q.y, q.z, q.w], s.pose.rotation);
}

#[test]
fn canonical_twist_and_dual_packets_preserve_samples_rates_pause_reset_and_reconnect() {
    let network = Network::new();
    let options = network.options();
    let ik = options.topic_key("/m33/imu").unwrap();
    let lk = options.topic_key("/m33/scan").unwrap();
    let commands = network
        .observer
        .declare_publisher(options.topic_key("/model/robot/cmd_vel").unwrap())
        .wait()
        .unwrap();
    let imus = network
        .observer
        .declare_subscriber(ik.clone())
        .with(zenoh::handlers::FifoChannel::new(1024))
        .wait()
        .unwrap();
    let scans = network
        .observer
        .declare_subscriber(lk.clone())
        .with(zenoh::handlers::FifoChannel::new(1024))
        .wait()
        .unwrap();
    let mut server = network.server(load(), options).unwrap();
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    assert_eq!(server.interest_policy(), InterestPolicy::Subscribers);
    assert!(server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    let deadline = Instant::now() + Duration::from_secs(5);
    while server.counters().imus_queued == 0
        || server.counters().lidars_queued == 0
        || !commands.matching_status().wait().unwrap().matching()
    {
        assert!(
            Instant::now() < deadline,
            "actual dual-stream discovery deadline"
        );
        server.update().unwrap();
        std::thread::sleep(Duration::from_millis(1));
    }
    pause(&mut server);
    for _ in 0..server.counters().imus_queued {
        imus.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    }
    for _ in 0..server.counters().lidars_queued {
        scans.recv_timeout(Duration::from_secs(5)).unwrap().unwrap();
    }
    // Actual delivery established demand. Explicit acquisition now makes the two
    // loaded-reset windows independent of transport rematching latency.
    server.set_interest_policy(InterestPolicy::Always);
    let ids = (
        identity(server.simulation(), "default::robot::base_link::imu"),
        identity(server.simulation(), "default::robot::base_link::lidar"),
    );
    let mut first = None;
    for epoch in 1..=2 {
        reset(&mut server, epoch);
        let baseline = server.counters();
        let mut expected_imus = Vec::new();
        let mut expected_scans = Vec::new();
        for (segment, (linear, angular)) in [(0.4, 0.0), (0.2, 0.8)].into_iter().enumerate() {
            let applied = server.counters().diff_drive_commands_applied;
            commands
                .put(twist(linear, angular).encode_to_vec())
                .attachment(Twist::full_name())
                .wait()
                .unwrap();
            let deadline = Instant::now() + Duration::from_secs(5);
            while server.counters().diff_drive_commands_applied == applied {
                assert!(
                    Instant::now() < deadline,
                    "actual Twist application deadline"
                );
                server.update().unwrap();
                assert!(server.simulation().is_paused());
                std::thread::yield_now();
            }
            assert_eq!(server.counters().diff_drive_commands_applied, applied + 1);
            assert!(server.apply_local_control(WorldControl {
                pause: true,
                multi_step: 300,
                ..Default::default()
            }));
            for tick in 1..=300 {
                let info = server.update().unwrap();
                assert_eq!(
                    info.sim_time,
                    Duration::from_millis((segment * 300 + tick) as u64)
                );
                let i = imu(server.simulation());
                let l = scan(server.simulation());
                assert_eq!((i.identity, l.identity), ids);
                assert_eq!((i.epoch, l.epoch), (epoch, epoch));
                if i.time == info.sim_time {
                    expected_imus.push(i);
                }
                if l.time == info.sim_time {
                    expected_scans.push(l);
                }
            }
            let drive = server
                .simulation()
                .world()
                .iter_entities()
                .find_map(|e| e.get::<DiffDriveSample>())
                .unwrap();
            assert!(
                drive.twist[0] > 0.0,
                "actual wheel encoders advance under canonical forward command"
            );
            if angular != 0.0 {
                assert!(drive.twist[1] > 0.0, "actual differential wheels turn");
            }
        }
        assert_eq!((expected_imus.len(), expected_scans.len()), (61, 13));
        assert_eq!(server.counters().imus_queued - baseline.imus_queued, 61);
        assert_eq!(server.counters().lidars_queued - baseline.lidars_queued, 13);
        let pair = (expected_imus[0].reading, expected_scans[0].ranges.clone());
        if let Some(ref previous) = first {
            assert_eq!(
                &pair, previous,
                "loaded reset repeats both seeded initial acquisitions"
            );
        } else {
            first = Some(pair);
        }
        for (seq, expected) in expected_imus.iter().enumerate() {
            check_imu(
                decode(
                    imus.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
                    &ik,
                ),
                expected,
                seq,
            );
        }
        for (seq, expected) in expected_scans.iter().enumerate() {
            check_scan(
                decode(
                    scans.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
                    &lk,
                ),
                expected,
                seq,
            );
        }
        let queued = (
            server.counters().imus_queued,
            server.counters().lidars_queued,
        );
        for _ in 0..20 {
            server.update().unwrap();
        }
        assert_eq!(server.simulation().elapsed(), Duration::from_millis(600));
        assert_eq!(
            (
                server.counters().imus_queued,
                server.counters().lidars_queued
            ),
            queued
        );
        assert!(imus.try_recv().unwrap().is_none() && scans.try_recv().unwrap().is_none());
        assert!(
            server.counters().altimeters_queued > 0
                && server.counters().air_pressures_queued > 0
                && server.counters().magnetometers_queued > 0
                && server.counters().joint_states_queued > 0
                && server.counters().odometries_queued > 0
        );
    }
    drop(imus);
    drop(scans);
    server.set_interest_policy(InterestPolicy::Subscribers);
    assert!(server.apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut unchanged = 0;
    let mut queued = (
        server.counters().imus_queued,
        server.counters().lidars_queued,
    );
    loop {
        server.update().unwrap();
        let now = (
            server.counters().imus_queued,
            server.counters().lidars_queued,
        );
        unchanged = if now == queued { unchanged + 1 } else { 0 };
        queued = now;
        // IMU deliberately retains its previous local reading when unobserved;
        // lidar removes its scan. Neither may acquire or republish stale data.
        let sim = server.simulation();
        if unchanged >= 100
            && sim
                .world()
                .get::<LidarSample>(entity(sim, "default::robot::base_link::lidar"))
                .is_none()
        {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "both disconnected acquisitions must become idle"
        );
        std::thread::sleep(Duration::from_millis(1));
    }
    let held_imu = imu(server.simulation());
    let before = server.counters();
    for _ in 0..100 {
        server.update().unwrap();
    }
    assert_eq!(
        (
            server.counters().imus_queued,
            server.counters().lidars_queued
        ),
        (before.imus_queued, before.lidars_queued)
    );
    assert_eq!(imu(server.simulation()), held_imu);
    let imus = network
        .observer
        .declare_subscriber(ik.clone())
        .wait()
        .unwrap();
    let scans = network
        .observer
        .declare_subscriber(lk.clone())
        .wait()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while server.counters().imus_queued == before.imus_queued
        || server.counters().lidars_queued == before.lidars_queued
    {
        assert!(Instant::now() < deadline, "dual reconnect must reacquire");
        server.update().unwrap();
        std::thread::yield_now();
    }
    pause(&mut server);
    let i: Imu = decode(
        imus.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
        &ik,
    );
    let l: LaserScan = decode(
        scans.recv_timeout(Duration::from_secs(5)).unwrap().unwrap(),
        &lk,
    );
    let later = |t: Time| i128::from(t.sec) * 1_000_000_000 + i128::from(t.nsec) > 600_000_000;
    assert!(later(i.header.unwrap().stamp.unwrap()));
    assert!(later(l.header.unwrap().stamp.unwrap()));
    assert_eq!(
        (
            server.counters().imu_failures,
            server.counters().lidar_failures,
            server.counters().invalid_imu_samples,
            server.counters().invalid_lidar_samples
        ),
        (0, 0, 0, 0)
    );
    server.shutdown();
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}

#[test]
fn no_demand_removal_and_initial_collision_refusal_keep_the_complete_robot_contract() {
    let mut sim = load();
    let robot = identity(&sim, "default::robot");
    let ids = (
        identity(&sim, "default::robot::base_link::imu"),
        identity(&sim, "default::robot::base_link::lidar"),
    );
    assert!(ids.0 > 0 && ids.1 > 0 && ids.0 != ids.1);
    let ie = entity(&sim, "default::robot::base_link::imu");
    let le = entity(&sim, "default::robot::base_link::lidar");
    assert_eq!(
        sim.world().get::<ImuPublication>(ie).unwrap().topic,
        "/m33/imu"
    );
    assert_eq!(
        sim.world().get::<LidarPublication>(le).unwrap().topic,
        "/m33/scan"
    );
    sim.step_exact(1).unwrap();
    let first = (imu(&sim).reading, scan(&sim).ranges);
    set_imu_interest(&mut sim, []);
    set_lidar_interest(&mut sim, []);
    sim.step_exact(100).unwrap();
    assert_eq!(imu(&sim).time, Duration::from_millis(1));
    assert_eq!(imu(&sim).reading, first.0);
    assert!(sim.world().get::<LidarSample>(le).is_none());
    remove_model(&mut sim, robot).unwrap();
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<ImuPublication>() || e.contains::<LidarPublication>())
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        (
            identity(&sim, "default::robot::base_link::imu"),
            identity(&sim, "default::robot::base_link::lidar")
        ),
        ids
    );
    // Loaded reset restores acquisition selection; explicit local interest uses
    // these restored public entities and the original sensor-local seeded streams.
    let ie = entity(&sim, "default::robot::base_link::imu");
    let le = entity(&sim, "default::robot::base_link::lidar");
    set_imu_interest(&mut sim, [ie]);
    set_lidar_interest(&mut sim, [le]);
    sim.step_exact(1).unwrap();
    assert_eq!((imu(&sim).reading, scan(&sim).ranges), first);
    let network = Network::new();
    for (from, to) in [
        ("/m33/scan", "/m33/imu"),
        ("/m33/imu", "/m33/scan"),
        ("/m33/scan", "/m17_m18/magnetometer"),
    ] {
        let mut options = network.options();
        options.add_topic_remap(from, to).unwrap();
        let result = network.server(load(), options);
        if to == "/m17_m18/magnetometer" {
            assert!(
                matches!(result, Err(Error::MagnetometerIdentity)),
                "magnetometer alias must fail before declaration"
            );
        } else {
            assert!(
                matches!(result, Err(Error::LidarIdentity)),
                "IMU/lidar alias must fail atomically: {from} -> {to}"
            );
        }
    }
}

fn native_peer() -> &'static zenoh_peer::NativePeer {
    static PEER: OnceLock<zenoh_peer::NativePeer> = OnceLock::new();
    PEER.get_or_init(|| {
        zenoh_peer::NativePeer::build(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."),
            "robot_sensors",
        )
    })
}
struct Peer(Child);
impl Drop for Peer {
    fn drop(&mut self) {
        if self.0.try_wait().unwrap().is_none() {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
}
#[test]
fn cpp_peer_drives_the_actual_robot_and_receives_both_canonical_sensor_types() {
    let network = Network::new();
    let options = network.options();
    let mut server = network.server(load(), options.clone()).unwrap();
    assert!(server.prepare_run(Duration::from_secs(5)).unwrap());
    let mut command = Command::new(&native_peer().binary);
    native_peer().configure_runtime(&mut command);
    command
        .env("GZ_PARTITION", &options.partition)
        .env("GZ_IP", "127.0.0.1")
        .env("GZ_TRANSPORT_IMPLEMENTATION", "zenoh")
        .env("GZ_TRANSPORT_TOPIC_STATISTICS", "0")
        .env("ZENOH_CONFIG", network.directory.path().join("client.json"))
        .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
        .env_remove("GZ_DISCOVERY_MSG_PORT")
        .env_remove("GZ_DISCOVERY_SRV_PORT")
        .env_remove("GZ_DISCOVERY_MULTICAST_IP")
        .env_remove("GZ_TRANSPORT_USERNAME")
        .env_remove("GZ_TRANSPORT_PASSWORD")
        .env_remove("GZ_RELAY")
        .env_remove("LD_PRELOAD")
        .env_remove("LD_AUDIT")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut peer = Peer(command.spawn().unwrap());
    let deadline = Instant::now() + Duration::from_secs(45);
    let status = loop {
        server.update().unwrap();
        if let Some(status) = peer.0.try_wait().unwrap() {
            break status;
        }
        assert!(
            Instant::now() < deadline,
            "native dual-sensor peer deadline"
        );
        std::thread::sleep(Duration::from_millis(1));
    };
    use std::io::Read;
    let mut stdout = String::new();
    let mut stderr = String::new();
    peer.0
        .stdout
        .take()
        .unwrap()
        .read_to_string(&mut stdout)
        .unwrap();
    peer.0
        .stderr
        .take()
        .unwrap()
        .read_to_string(&mut stderr)
        .unwrap();
    assert!(
        status.success(),
        "native status={status} stdout={stdout} stderr={stderr}"
    );
    assert_eq!(
        stdout.trim(),
        "PASS native robot_sensors IMU LaserScan forward turn"
    );
    assert!(stderr.is_empty(), "{stderr}");
    assert_eq!(server.counters().diff_drive_commands_applied, 2);
    assert!(
        server.counters().imus_queued > 0
            && server.counters().lidars_queued > 0
            && server.counters().odometries_queued > 0
    );
    assert!(server.simulation().is_paused());
    assert_eq!(
        (
            server.counters().imu_failures,
            server.counters().lidar_failures
        ),
        (0, 0)
    );
    server.shutdown();
}
