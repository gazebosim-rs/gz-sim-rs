// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_msgs::msgs::{Imu, PoseV};
use gz_sim_core::EntityId;
use gz_sim_scene::{
    Name, PhysicsEngine, WorldPose, load_world_with_sdf_step, queue_link_world_velocity,
};
use std::collections::BTreeSet;

fn stamp(value: &Time) -> u64 {
    assert!(value.sec >= 0 && value.sec <= 2);
    assert!((0..1_000_000_000).contains(&value.nsec));
    value.sec as u64 * 1_000_000_000 + value.nsec as u64
}

fn check_clock(message: &Clock, sequence: u64, last_real: &mut (i64, i32)) -> u64 {
    assert!(message.header.is_none());
    let sim = stamp(message.sim.as_ref().unwrap());
    assert_eq!(sim, (sequence + 1) * 1_000_000);
    let real = message.real.as_ref().unwrap();
    assert!(real.sec >= 0 && (0..1_000_000_000).contains(&real.nsec));
    let current = (real.sec, real.nsec);
    assert!(current >= *last_real);
    *last_real = current;
    let system = message.system.as_ref().unwrap();
    assert!(system.sec >= 1_000_000_000 && (0..1_000_000_000).contains(&system.nsec));
    let observed = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs();
    assert!(system.sec.abs_diff(i64::try_from(observed).unwrap()) <= 1);
    sim
}

fn check_pose(
    message: &PoseV,
    sequence: u64,
    identities: &mut BTreeMap<String, u32>,
) -> (u64, usize) {
    let header = message.header.as_ref().unwrap();
    assert!(header.data.is_empty());
    let time = stamp(header.stamp.as_ref().unwrap());
    assert_eq!(time, (sequence + 1) * 1_000_000);
    assert_eq!(message.pose.len(), 4);
    let mut names = BTreeSet::new();
    let mut ids = BTreeSet::new();
    let mut id_bytes = 0;
    for (index, pose) in message.pose.iter().enumerate() {
        assert!(names.insert(pose.name.as_str()) && ids.insert(pose.id));
        assert_ne!(pose.id, 0);
        assert!(pose.header.is_none());
        // Each small fixture record keeps its length delimiter to one byte.
        assert!(pose.encoded_len() < 128);
        let original = identities.entry(pose.name.clone()).or_insert(pose.id);
        assert_eq!(*original, pose.id);
        assert_eq!(index < 2, matches!(pose.name.as_str(), "ground" | "ball"));
        let position = pose.position.as_ref().unwrap();
        let rotation = pose.orientation.as_ref().unwrap();
        assert!(position.header.is_none() && rotation.header.is_none());
        let x = if pose.name == "ball" {
            (sequence + 1) as f64 * 0.001
        } else {
            0.0
        };
        let z = match pose.name.as_str() {
            "ball" => 5.0,
            "base" => -0.5,
            "ground" | "body" => 0.0,
            other => panic!("unknown pose role {other}"),
        };
        assert!((position.x - x).abs() < 1e-9);
        assert_eq!([position.y, position.z], [0.0, z]);
        assert_eq!(
            [rotation.x, rotation.y, rotation.z, rotation.w],
            [0.0, 0.0, 0.0, 1.0]
        );
        id_bytes += gz_msgs::prost::encoding::encoded_len_varint(u64::from(pose.id));
    }
    assert_eq!(names, BTreeSet::from(["ground", "ball", "base", "body"]));
    (time, id_bytes)
}

fn check_imu(message: &Imu, sequence: u64, metadata: &ImuPublication) -> (u64, f64) {
    assert_eq!(
        stamp(message.header.as_ref().unwrap().stamp.as_ref().unwrap()),
        (sequence + 1) * 1_000_000
    );
    assert!(message.orientation.as_ref().unwrap().header.is_none());
    assert!(message.angular_velocity.as_ref().unwrap().header.is_none());
    assert!(
        message
            .linear_acceleration
            .as_ref()
            .unwrap()
            .header
            .is_none()
    );
    for covariance in [
        &message.orientation_covariance,
        &message.angular_velocity_covariance,
        &message.linear_acceleration_covariance,
    ] {
        assert!(covariance.as_ref().unwrap().header.is_none());
    }
    super::pipeline_benchmark::check(message, sequence, metadata)
}

#[test]
fn tpe_clock_pose_imu_complete_lockstep() {
    let steps: u64 = std::env::var("GZ_PIPELINE_STEPS")
        .unwrap_or_else(|_| "64".into())
        .parse()
        .unwrap();
    assert!((1..=1000).contains(&steps));
    let mut sim = load_world_with_sdf_step(
        include_str!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
        "headless_imu.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    match std::env::var("GZ_PIPELINE_EXECUTOR")
        .as_deref()
        .unwrap_or("default")
    {
        "default" => {}
        "parallel" => sim.set_step_executor(bevy_ecs::schedule::MultiThreadedExecutor::new()),
        "single" => sim.set_step_executor(bevy_ecs::schedule::SingleThreadedExecutor::new()),
        _ => panic!("invalid pipeline executor"),
    }
    let body = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::ball::body")
        })
        .unwrap();
    let body_entity = body.id();
    let body_id = body.get::<EntityId>().unwrap().get();
    queue_link_world_velocity(&mut sim, body_id, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
    let metadata = sim
        .world()
        .iter_entities()
        .find_map(|entity| entity.get::<ImuPublication>().cloned())
        .unwrap();
    let mut server = Server::new(
        sim,
        "default",
        Config {
            partition: format!(
                "streams-{}-{}",
                std::process::id(),
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_nanos()
            ),
            interface: std::net::Ipv4Addr::LOCALHOST,
            ..Config::default()
        },
        NonZeroUsize::new(64).unwrap(),
    )
    .unwrap();
    let imus = server
        .node
        .subscribe(&metadata.topic, "gz.msgs.IMU")
        .unwrap();
    let clocks = server
        .node
        .subscribe("/world/default/clock", "gz.msgs.Clock")
        .unwrap();
    let poses = server
        .node
        .subscribe("/world/default/pose/info", "gz.msgs.Pose_V")
        .unwrap();
    assert!(server.apply_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    let mut identities = BTreeMap::new();
    let mut last_real = (0, 0);
    server.tick().unwrap();
    check_clock(
        &Clock::decode(
            clocks
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .data
                .as_slice(),
        )
        .unwrap(),
        0,
        &mut last_real,
    );
    check_pose(
        &PoseV::decode(
            poses
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .data
                .as_slice(),
        )
        .unwrap(),
        0,
        &mut identities,
    );
    let first = Imu::decode(
        imus.recv_timeout(Duration::from_secs(5))
            .unwrap()
            .data
            .as_slice(),
    )
    .unwrap();
    check_imu(&first, 0, &metadata);
    let warmup_force_x = first.linear_acceleration.as_ref().unwrap().x;
    let (mut logical_bytes, mut time_sum, mut force_sum) = (0, 0, 0.0);
    let (
        mut clock_bytes,
        mut clock_time_sum,
        mut pose_bytes,
        mut pose_time_sum,
        mut normalized_pose_bytes,
    ) = (0, 0, 0, 0, 0);
    let period_us: u64 = std::env::var("GZ_STREAM_PERIOD_US")
        .unwrap_or_else(|_| "20000".into())
        .parse()
        .unwrap();
    assert!((20_000..=1_000_000).contains(&period_us));
    let period = Duration::from_micros(period_us);
    let wall_start = Instant::now();
    let mut next_request = wall_start + period;
    let (mut nanos, mut pacing_wait_nanos) = (0, 0);

    for sequence in 1..=steps {
        let waiting = Instant::now();
        std::thread::sleep(next_request.saturating_duration_since(waiting));
        let start = Instant::now();
        pacing_wait_nanos += start.duration_since(waiting).as_nanos();
        let info = server.tick().unwrap();
        assert!(!info.paused);
        assert_eq!(info.sim_time, Duration::from_millis(sequence + 1));
        let body = server
            .simulation
            .world()
            .get::<WorldPose>(body_entity)
            .unwrap();
        assert!((body.translation[0] - (sequence + 1) as f64 * 0.001).abs() < 1e-9);
        assert_eq!([body.translation[1], body.translation[2]], [0.0, 5.0]);
        let clock = clocks.recv_timeout(Duration::from_secs(5)).unwrap();
        clock_bytes += clock.data.len();
        clock_time_sum += check_clock(
            &Clock::decode(clock.data.as_slice()).unwrap(),
            sequence,
            &mut last_real,
        );
        let pose = poses.recv_timeout(Duration::from_secs(5)).unwrap();
        pose_bytes += pose.data.len();
        let (time, id_bytes) = check_pose(
            &PoseV::decode(pose.data.as_slice()).unwrap(),
            sequence,
            &mut identities,
        );
        pose_time_sum += time;
        normalized_pose_bytes += pose.data.len() - id_bytes;
        let imu = imus.recv_timeout(Duration::from_secs(5)).unwrap();
        logical_bytes += imu.data.len();
        let (time, force) = check_imu(
            &Imu::decode(imu.data.as_slice()).unwrap(),
            sequence,
            &metadata,
        );
        time_sum += time;
        force_sum += force;
        let end = Instant::now();
        nanos += end.duration_since(start).as_nanos();
        next_request = end + period;
    }
    let wall_nanos = wall_start.elapsed().as_nanos();
    for stream in [&clocks, &poses, &imus] {
        assert!(matches!(
            stream.recv_timeout(Duration::from_millis(1)),
            Err(TransportError::Timeout)
        ));
    }
    let counters = server.counters();
    assert_eq!(
        (
            counters.clocks_queued,
            counters.poses_queued,
            counters.imus_queued
        ),
        (steps + 1, steps + 1, steps + 1)
    );
    assert_eq!(
        (
            counters.clock_failures,
            counters.pose_failures,
            counters.imu_failures,
            counters.invalid_pose_frames,
            counters.invalid_imu_samples
        ),
        (0, 0, 0, 0, 0)
    );
    assert_eq!(server.transport_statistics(), Statistics::default());
    let final_x = server
        .simulation
        .world()
        .get::<WorldPose>(body_entity)
        .unwrap()
        .translation[0];
    server.shutdown();
    let fixture_hex: String =
        include_bytes!("../../../gz-sim-scene/tests/fixtures/headless_imu.sdf")
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect();
    let report = serde_json::json!({
        "steps": steps,
        "nanos": nanos,
        "wall_nanos": wall_nanos,
        "pacing_wait_nanos": pacing_wait_nanos,
        "request_period_us": period_us,
        "logical_bytes": logical_bytes,
        "time_sum": time_sum,
        "force_sum": force_sum,
        "final_x": final_x,
        "warmup_force_x": warmup_force_x,
        "clock_count": steps,
        "pose_count": steps,
        "clock_time_sum": clock_time_sum,
        "pose_time_sum": pose_time_sum,
        "clock_bytes": clock_bytes,
        "pose_bytes": pose_bytes,
        "pose_bytes_without_id_values": normalized_pose_bytes,
        "fixture_hex": fixture_hex,
        "pose_ids": identities,
    });
    println!("PIPELINE {report}");
}
