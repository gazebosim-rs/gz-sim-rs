// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Genuine M8 graphs and canonical Rust transport; C++ peer proof is separate.
use gz_sim_core::{EntityId, Simulation};
use gz_sim_proto::{
    msgs::{
        Boolean, Entity as EntityRequest, Odometry, Twist, Vector3d, WorldControl, WorldReset,
        entity::Type as EntityType,
    },
    prost::{Message, Name as ProtoName},
};
use gz_sim_scene::{
    DiffDriveSample, EntityKind, JointHandle, Name, PhysicsEngine, WorldPose,
    load_world_with_sdf_step,
};
use gz_sim_server::{Error, InterestPolicy, Server, ServerOptions};
use std::{
    num::NonZeroUsize,
    time::{Duration, Instant},
};
use zenoh::Wait;

const XML: &str = include_str!("../../../examples/m8_diff_drive.sdf");
fn xml() -> String {
    let start = XML.find("<model name=\"robot\"").unwrap();
    let end = start + XML[start..].find("</model>").unwrap() + "</model>".len();
    let robot = &XML[start..end];
    let world = XML[..start].to_owned() + &XML[end..];
    let mut models = String::new();
    for (side, y, rate) in [("left", -2, 50), ("right", 2, 25)] {
        models.push_str(
            &robot
                .replace("name=\"robot\"", &format!("name=\"robot_{side}\""))
                .replace(
                    "<pose>0 0 0.2 0 0 0</pose>",
                    &format!("<pose>0 {y} 0.2 0 0 0</pose>"),
                )
                .replace("/model/robot/cmd_vel", &format!("/m59/{side}/cmd_vel"))
                .replace("/model/robot/odometry", &format!("/m59/{side}/odom"))
                .replace("robot/odom", &format!("m59_{side}_odom"))
                .replace("robot/base_link", &format!("m59_{side}_base"))
                .replace(
                    "<odom_publish_frequency>50</odom_publish_frequency>",
                    &format!("<odom_publish_frequency>{rate}</odom_publish_frequency>"),
                ),
        );
    }
    world.replace("</world>", &(models + "</world>"))
}
fn loaded() -> Simulation {
    load_world_with_sdf_step(
        &xml(),
        "m59-public-server.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn config(connect: &[String]) -> zenoh::Config {
    zenoh::Config::from_json5(&serde_json::json!({"mode":"peer",
        "listen":{"endpoints":if connect.is_empty() {vec!["tcp/127.0.0.1:0"]} else {vec![]}},
        "connect":{"endpoints":connect}, "scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},
        "transport":{"shared_memory":{"enabled":false}}}).to_string()).unwrap()
}
fn wait(mut condition: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !condition() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
}
fn pause(server: &mut Server, value: bool) {
    assert!(server.apply_local_control(WorldControl {
        pause: value,
        ..Default::default()
    }));
}
fn twist(linear: f64, angular: f64) -> Twist {
    Twist {
        linear: Some(Vector3d {
            x: linear,
            ..Default::default()
        }),
        angular: Some(Vector3d {
            z: angular,
            ..Default::default()
        }),
        ..Default::default()
    }
}
fn model(sim: &Simulation, side: &str) -> (bevy_ecs::entity::Entity, u64) {
    let name = format!("default::robot_{side}");
    let node = sim
        .world()
        .iter_entities()
        .find(|n| n.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    assert_eq!(node.get::<EntityKind>(), Some(&EntityKind::Model));
    (node.id(), node.get::<EntityId>().unwrap().get())
}
fn chassis_x(sim: &Simulation, side: &str) -> f64 {
    let name = format!("default::robot_{side}::base_link");
    sim.world()
        .iter_entities()
        .find(|n| n.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<WorldPose>()
        .unwrap()
        .translation[0]
}
// Failure context uses measured states; encoder odometry is not chassis truth.
fn motion_diagnostics(server: &Server, start: Duration, chassis_before: [f64; 2]) -> String {
    let sim = server.simulation();
    let chassis_after = ["left", "right"].map(|side| chassis_x(sim, side));
    let joints: Vec<_> = native(sim)
        .into_iter()
        .map(|row| (row.id, row.state.map(f64::from_bits)))
        .collect();
    let encoders =
        ["left", "right"].map(|side| sim.world().get::<DiffDriveSample>(model(sim, side).0));
    format!(
        "start={start:?} end={:?} elapsed={:?} chassis_before={chassis_before:?} chassis_after={chassis_after:?} native_joints={joints:?} encoders={encoders:?}",
        sim.elapsed(),
        sim.elapsed() - start
    )
}
#[derive(Debug, PartialEq, Eq)]
struct NativeJointSnapshot {
    id: u64,
    state: [u64; 2],
}
fn native(sim: &Simulation) -> Vec<NativeJointSnapshot> {
    let mut result: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|node| {
            let handle = node.get::<JointHandle>()?;
            let state = sim
                .world()
                .resource::<gz_physics::PhysicsBackend>()
                .joint_state(handle.id())
                .unwrap();
            Some(NativeJointSnapshot {
                id: node.get::<EntityId>().unwrap().get(),
                state: [state.position.to_bits(), state.velocity.to_bits()],
            })
        })
        .collect();
    result.sort_by_key(|row| row.id);
    result
}
struct Expected {
    time: Duration,
    pose: [f64; 3],
    twist: [f64; 2],
}
fn received(
    sample: zenoh::sample::Sample,
    key: &str,
    side: &str,
    previous: &mut Option<u64>,
) -> (Duration, Odometry) {
    assert_eq!(sample.key_expr().as_str(), key);
    assert_eq!(
        sample.attachment().unwrap().to_bytes().as_ref(),
        b"gz.msgs.Odometry"
    );
    let message = Odometry::decode(sample.payload().to_bytes().as_ref()).unwrap();
    let header = message.header.as_ref().unwrap();
    assert_eq!(header.data.len(), 3);
    assert_eq!(header.data[0].key, "frame_id");
    assert_eq!(header.data[0].value, [format!("m59_{side}_odom")]);
    assert_eq!(header.data[1].key, "child_frame_id");
    assert_eq!(header.data[1].value, [format!("m59_{side}_base")]);
    assert_eq!(header.data[2].key, "seq");
    assert_eq!(header.data[2].value.len(), 1);
    let sequence = header.data[2].value[0].parse::<u64>().unwrap();
    if let Some(previous) = *previous {
        assert_eq!(sequence, previous.wrapping_add(1));
    }
    *previous = Some(sequence);
    let stamp = header.stamp.as_ref().unwrap();
    assert!(stamp.sec >= 0 && stamp.nsec >= 0 && stamp.nsec < 1_000_000_000);
    (Duration::new(stamp.sec as u64, stamp.nsec as u32), message)
}
fn complete(message: Odometry, expected: &Expected) {
    let pose = message.pose.unwrap();
    assert_eq!(pose.id, 0);
    assert!(pose.name.is_empty());
    let p = pose.position.unwrap();
    assert_eq!(
        [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()],
        [
            expected.pose[0].to_bits(),
            expected.pose[1].to_bits(),
            0.0f64.to_bits()
        ]
    );
    let q = pose.orientation.unwrap();
    let (sin, cos) = (expected.pose[2] * 0.5).sin_cos();
    assert_eq!(
        [q.x.to_bits(), q.y.to_bits(), q.z.to_bits(), q.w.to_bits()],
        [
            0.0f64.to_bits(),
            0.0f64.to_bits(),
            sin.to_bits(),
            cos.to_bits()
        ]
    );
    let twist = message.twist.unwrap();
    let linear = twist.linear.unwrap();
    let angular = twist.angular.unwrap();
    assert_eq!(
        [linear.x.to_bits(), linear.y.to_bits(), linear.z.to_bits()],
        [
            expected.twist[0].to_bits(),
            0.0f64.to_bits(),
            0.0f64.to_bits()
        ]
    );
    assert_eq!(
        [
            angular.x.to_bits(),
            angular.y.to_bits(),
            angular.z.to_bits()
        ],
        [
            0.0f64.to_bits(),
            0.0f64.to_bits(),
            expected.twist[1].to_bits()
        ]
    );
}

fn remove_loaded_model(
    server: &mut Server,
    observer: &zenoh::Session,
    options: &ServerOptions,
    model: u64,
) {
    assert!(server.simulation().is_paused());
    let before = native_paused(server);
    let counters = format!("{:?}", server.counters());
    let model_ids = |server: &Server| {
        let mut ids: Vec<_> = server
            .simulation()
            .world()
            .iter_entities()
            .filter(|node| node.get::<EntityKind>() == Some(&EntityKind::Model))
            .map(|node| node.get::<EntityId>().unwrap().get())
            .collect();
        ids.sort_unstable();
        ids
    };
    let original_ids = model_ids(server);
    assert!(matches!(
        server.remove_editable_model(model),
        Err(Error::WorldEdit(
            gz_sim_scene::WorldEditError::UnsupportedModel
        ))
    ));
    assert_eq!(native_paused(server), before);
    assert_eq!(format!("{:?}", server.counters()), counters);
    assert_eq!(model_ids(server), original_ids);
    let time = server.simulation().elapsed();
    let epoch = server.simulation().epoch();
    let old = server.counters();
    let deadline = Instant::now() + Duration::from_secs(5);
    let querier = observer
        .declare_querier(options.topic_key("/world/default/remove/blocking").unwrap())
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    while !querier.matching_status().wait().unwrap().matching() {
        assert!(Instant::now() < deadline);
        assert!(server.update().unwrap().dt.is_zero());
        assert_eq!(
            (server.simulation().elapsed(), server.simulation().epoch()),
            (time, epoch)
        );
        std::thread::yield_now();
    }
    let replies = querier
        .get()
        .payload(
            EntityRequest {
                id: model,
                r#type: EntityType::Model as i32,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .wait()
        .unwrap();
    loop {
        assert!(Instant::now() < deadline, "actual blocking removal reply");
        assert!(server.update().unwrap().dt.is_zero());
        assert_eq!(
            (server.simulation().elapsed(), server.simulation().epoch()),
            (time, epoch)
        );
        if let Some(reply) = replies.recv_timeout(Duration::from_millis(20)).unwrap() {
            let response = reply.result().unwrap();
            assert!(response.payload().len() <= 64);
            assert_eq!(
                response.payload().to_bytes().as_ref(),
                Boolean {
                    data: true,
                    ..Default::default()
                }
                .encode_to_vec()
            );
            break;
        }
    }
    let new = server.counters();
    assert_eq!(
        (
            new.removals_queued,
            new.removals_completed,
            new.removals_failed,
            new.removals_rejected
        ),
        (
            old.removals_queued + 1,
            old.removals_completed + 1,
            old.removals_failed,
            old.removals_rejected
        )
    );
    assert!(
        Instant::now() < deadline,
        "actual blocking removal completion"
    );
    assert!(
        !server
            .simulation()
            .world()
            .iter_entities()
            .any(|node| { node.get::<EntityId>().is_some_and(|id| id.get() == model) })
    );
}

#[test]
fn two_real_motor_owners_keep_canonical_packets_cadences_pause_reset_and_independent_retirement() {
    let observer = zenoh::open(config(&[])).wait().unwrap();
    let locators: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|v| v.to_string())
        .collect();
    assert_eq!(locators.len(), 1);
    let options = ServerOptions {
        capacity: 4096,
        max_message_bytes: 512,
        partition: format!("m59-public-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    };
    let command_keys =
        ["left", "right"].map(|side| options.topic_key(&format!("/m59/{side}/cmd_vel")).unwrap());
    let odometry_keys =
        ["left", "right"].map(|side| options.topic_key(&format!("/m59/{side}/odom")).unwrap());
    let publishers = command_keys
        .each_ref()
        .map(|key| observer.declare_publisher(key.clone()).wait().unwrap());
    let subscribers = odometry_keys
        .each_ref()
        .map(|key| observer.declare_subscriber(key.clone()).wait().unwrap());
    let mut server = Server::with_zenoh_config(
        loaded(),
        "default",
        options.clone(),
        config(&locators),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    server.set_interest_policy(InterestPolicy::Always);
    wait(|| {
        publishers
            .iter()
            .all(|p| p.matching_status().wait().unwrap().matching())
    });
    let owners = [
        model(server.simulation(), "left"),
        model(server.simulation(), "right"),
    ];
    assert!(owners[0].1 != 0 && owners[1].1 != 0 && owners[0].1 != owners[1].1);
    let mut sequences = [None, None];
    let mut seen = [false; 2];
    // Actual output receipt establishes both routes. It is neither a private
    // matching getter nor a self-subscriber masquerading as Server readiness.
    pause(&mut server, false);
    wait(|| {
        server.update().unwrap();
        for (index, subscriber) in subscribers.iter().enumerate() {
            while let Ok(Some(sample)) = subscriber.try_recv() {
                received(
                    sample,
                    &odometry_keys[index],
                    ["left", "right"][index],
                    &mut sequences[index],
                );
                seen[index] = true;
            }
        }
        seen.into_iter().all(|value| value)
    });
    while !server.simulation().elapsed().as_millis().is_multiple_of(40) {
        server.update().unwrap();
    }
    pause(&mut server, true);
    let start = server.simulation().elapsed();
    let before = native(server.simulation());
    let x0 = [
        chassis_x(server.simulation(), "left"),
        chassis_x(server.simulation(), "right"),
    ];
    for (publisher, (v, w)) in publishers.iter().zip([(0.4, 0.0), (-0.25, 0.5)]) {
        publisher
            .put(twist(v, w).encode_to_vec())
            .attachment(Twist::full_name())
            .wait()
            .unwrap();
    }
    wait(|| {
        server.update().unwrap();
        server.counters().diff_drive_commands_applied == 2
    });
    assert_eq!(server.simulation().elapsed(), start);
    assert_eq!(native(server.simulation()), before);
    let baseline = server.counters().odometries_queued;
    // Three fixed one-second actuation segments, each retaining every acquired wire.
    for _ in 0..3 {
        let segment_baseline = server.counters().odometries_queued;
        let mut expected: [Vec<Expected>; 2] = [Vec::new(), Vec::new()];
        let mut last = [None, None];
        pause(&mut server, false);
        for _ in 0..1000 {
            let info = server.update().unwrap();
            for (index, owner) in owners.iter().enumerate() {
                let value = server
                    .simulation()
                    .world()
                    .get::<DiffDriveSample>(owner.0)
                    .unwrap();
                if value.time == info.sim_time && last[index] != Some(value.time) {
                    last[index] = Some(value.time);
                    expected[index].push(Expected {
                        time: value.time,
                        pose: value.pose,
                        twist: value.twist,
                    });
                }
            }
        }
        pause(&mut server, true);
        assert_eq!(expected.each_ref().map(Vec::len), [50, 25]);
        assert_eq!(server.counters().odometries_queued - segment_baseline, 75);
        for (index, values) in expected.iter().enumerate() {
            for value in values {
                loop {
                    let sample = subscribers[index]
                        .recv_timeout(Duration::from_secs(5))
                        .unwrap()
                        .unwrap();
                    let (stamp, packet) = received(
                        sample,
                        &odometry_keys[index],
                        ["left", "right"][index],
                        &mut sequences[index],
                    );
                    if stamp <= start {
                        continue;
                    }
                    assert_eq!(stamp, value.time);
                    complete(packet, value);
                    break;
                }
            }
        }
    }
    assert_eq!(
        server.simulation().elapsed() - start,
        Duration::from_secs(3)
    );
    assert_eq!(server.counters().odometries_queued - baseline, 225);
    let diagnostic = motion_diagnostics(&server, start, x0);
    assert!(
        chassis_x(server.simulation(), "left") - x0[0] > 0.05,
        "{diagnostic}"
    );
    assert!(
        chassis_x(server.simulation(), "right") - x0[1] < -0.05,
        "{diagnostic}"
    );
    let moving = native(server.simulation());
    assert_eq!(moving.len(), 4);
    assert!(
        moving
            .iter()
            .all(|row| f64::from_bits(row.state[0]).is_finite()
                && f64::from_bits(row.state[1]).is_finite())
    );
    assert!(
        moving
            .iter()
            .any(|row| f64::from_bits(row.state[1]).abs() > 0.01)
    );
    let held = server.counters().odometries_queued;
    for _ in 0..100 {
        assert!(server.update().unwrap().dt.is_zero());
    }
    assert_eq!(native(server.simulation()), moving);
    assert_eq!(server.counters().odometries_queued, held);
    for (bytes, attachment) in [
        (vec![255], Twist::full_name()),
        (twist(f64::NAN, 0.0).encode_to_vec(), Twist::full_name()),
        (twist(0.5, 0.0).encode_to_vec(), "gz.msgs.Double".into()),
        (vec![0; 513], Twist::full_name()),
    ] {
        let count = server.counters().diff_drive_messages_received;
        publishers[0]
            .put(bytes)
            .attachment(attachment)
            .wait()
            .unwrap();
        wait(|| {
            server.update().unwrap();
            server.counters().diff_drive_messages_received == count + 1
        });
    }
    assert_eq!(server.counters().diff_drive_commands_applied, 2);
    assert_eq!(server.counters().diff_drive_messages_rejected, 4);
    assert_eq!(native(server.simulation()), moving);
    let epoch = server.simulation().epoch();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.prepare_io().unwrap();
    let reset = server.tick().unwrap();
    assert!(reset.dt.is_zero() && reset.sim_time.is_zero());
    assert_eq!(server.simulation().epoch(), epoch + 1);
    assert_eq!(
        [
            model(server.simulation(), "left").1,
            model(server.simulation(), "right").1
        ],
        owners.map(|value| value.1)
    );
    remove_loaded_model(&mut server, &observer, &options, owners[0].1);
    server.prepare_io().unwrap();
    server.tick().unwrap();
    wait(|| !publishers[0].matching_status().wait().unwrap().matching());
    wait(|| publishers[1].matching_status().wait().unwrap().matching());
    let count = server.counters().diff_drive_commands_applied;
    publishers[1]
        .put(twist(-0.2, 0.0).encode_to_vec())
        .attachment(Twist::full_name())
        .wait()
        .unwrap();
    wait(|| {
        server.update().unwrap();
        server.counters().diff_drive_commands_applied == count + 1
    });
    let previous = server.counters().odometries_queued;
    pause(&mut server, false);
    for _ in 0..200 {
        server.update().unwrap();
    }
    assert!(server.counters().odometries_queued > previous);
    assert!(!server.simulation().world().iter_entities().any(|node| {
        node.get::<EntityId>()
            .is_some_and(|id| id.get() == owners[0].1)
    }));
    assert_eq!(model(server.simulation(), "right").1, owners[1].1);
    drop(server);
    wait(|| !publishers[1].matching_status().wait().unwrap().matching());
    observer.close().wait().unwrap();
}

#[test]
fn genuine_pair_duplicate_cross_channel_and_world_aliases_refuse_without_drive_input_prefix() {
    let observer = zenoh::open(config(&[])).wait().unwrap();
    let endpoints: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|v| v.to_string())
        .collect();
    assert_eq!(endpoints.len(), 1);
    for destination in [
        "/m59/left/cmd_vel",
        "/m59/left/odom",
        "/clock",
        "/world/default/clock",
        "/world/default/control",
        "/world/default/pose/info",
    ] {
        let mut options = ServerOptions {
            partition: format!("m59-collision-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        };
        options
            .add_topic_remap("/m59/right/cmd_vel", destination)
            .unwrap();
        let left = observer
            .declare_publisher(options.topic_key("/m59/left/cmd_vel").unwrap())
            .wait()
            .unwrap();
        assert!(!left.matching_status().wait().unwrap().matching());
        assert!(matches!(
            Server::with_zenoh_config(
                loaded(),
                "default",
                options,
                config(&endpoints),
                NonZeroUsize::new(2).unwrap()
            ),
            Err(Error::DiffDriveIdentity)
        ));
        assert!(!left.matching_status().wait().unwrap().matching());
    }
    observer.close().wait().unwrap();
}

// The owned child/capture pattern follows the maintained M58 canonical peer.
use std::{
    io::{BufRead, BufReader, Read, Write},
    path::Path,
    process::{Child, Command, Stdio},
    sync::{Arc, Mutex, OnceLock, mpsc},
};
#[path = "../../../tests/support/zenoh_peer.rs"]
mod zenoh_peer;
fn peer() -> &'static zenoh_peer::NativePeer {
    static PEER: OnceLock<zenoh_peer::NativePeer> = OnceLock::new();
    PEER.get_or_init(|| {
        zenoh_peer::NativePeer::build(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."),
            "multiple_diff_drives",
        )
    })
}
const PEER_CHANNEL_CAP: usize = 1024 * 1024;
#[derive(Default)]
struct Captured {
    bytes: Vec<u8>,
    error: Option<String>,
    exceeded: bool,
}
fn capture<R: Read + Send + 'static>(
    stream: R,
    state: Arc<Mutex<Captured>>,
    lines: Option<mpsc::SyncSender<String>>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut reader = BufReader::new(stream).take((PEER_CHANNEL_CAP + 1) as u64);
        loop {
            let mut line = Vec::new();
            match reader.read_until(b'\n', &mut line) {
                Ok(0) => return,
                Ok(_) => {
                    let mut saved = state.lock().unwrap();
                    let remaining = PEER_CHANNEL_CAP - saved.bytes.len();
                    saved
                        .bytes
                        .extend_from_slice(&line[..line.len().min(remaining)]);
                    if line.len() > remaining {
                        saved.exceeded = true;
                        return;
                    }
                    drop(saved);
                    if let Some(sender) = &lines {
                        if line.last() == Some(&b'\n') {
                            line.pop();
                        }
                        let text = match String::from_utf8(line) {
                            Ok(text) => text,
                            Err(error) => {
                                state.lock().unwrap().error = Some(error.to_string());
                                return;
                            }
                        };
                        if let Err(error) = sender.try_send(text) {
                            state.lock().unwrap().error =
                                Some(format!("bounded phase channel: {error}"));
                            return;
                        }
                    }
                }
                Err(error) => {
                    state.lock().unwrap().error = Some(error.to_string());
                    return;
                }
            }
        }
    })
}
struct Peer {
    child: Child,
    lines: mpsc::Receiver<String>,
    readers: Vec<std::thread::JoinHandle<()>>,
    stdout: Arc<Mutex<Captured>>,
    stderr: Arc<Mutex<Captured>>,
    status: Option<std::process::ExitStatus>,
    retired: bool,
    retired_clean: bool,
    reported: bool,
}
impl Peer {
    fn diagnostics(&self) -> String {
        let channel = |name: &str, state: &Arc<Mutex<Captured>>| {
            let saved = state.lock().unwrap();
            let bytes = match std::str::from_utf8(&saved.bytes) {
                Ok(text) => format!("{text:?}"),
                Err(_) => format!("{:?}", saved.bytes),
            };
            format!(
                "{name}: captured_bytes={} exceeded={} read_error={:?} contents={bytes}",
                saved.bytes.len(),
                saved.exceeded,
                saved.error
            )
        };
        format!(
            "actual_wait={:?}; {}\n{}",
            self.status,
            channel("stdout", &self.stdout),
            channel("stderr", &self.stderr)
        )
    }
    fn retire(&mut self, force: bool) -> String {
        if self.retired {
            return self.diagnostics();
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        let mut cleanup = Vec::new();
        if force && self.status.is_none() {
            match self.child.try_wait() {
                Ok(Some(status)) => self.status = Some(status),
                Ok(None) => {
                    if let Err(error) = self.child.kill() {
                        cleanup.push(format!("owned kill: {error}"));
                    }
                }
                Err(error) => {
                    cleanup.push(format!("owned wait: {error}"));
                    if let Err(error) = self.child.kill() {
                        cleanup.push(format!("owned kill: {error}"));
                    }
                }
            }
        }
        while self.status.is_none() && Instant::now() < deadline {
            match self.child.try_wait() {
                Ok(Some(status)) => self.status = Some(status),
                Ok(None) => std::thread::sleep(Duration::from_millis(1)),
                Err(error) => {
                    cleanup.push(format!("owned wait: {error}"));
                    break;
                }
            }
        }
        if self.status.is_some() {
            while self.readers.iter().any(|reader| !reader.is_finished())
                && Instant::now() < deadline
            {
                std::thread::sleep(Duration::from_millis(1));
            }
            let readers = std::mem::take(&mut self.readers);
            for reader in readers {
                if reader.is_finished() {
                    if reader.join().is_err() {
                        cleanup.push("channel reader panicked".into());
                    }
                } else {
                    cleanup.push("channel reader completion UNKNOWN at cleanup deadline".into());
                }
            }
        } else {
            cleanup.push("actual native wait UNKNOWN at cleanup deadline".into());
        }
        self.retired_clean = self.status.is_some() && cleanup.is_empty();
        self.retired = true;
        format!("{}; cleanup={cleanup:?}", self.diagnostics())
    }
    fn fail(&mut self, message: impl std::fmt::Display) -> ! {
        let details = self.retire(true);
        self.reported = true;
        panic!("native multiple DiffDrive peer: {message}; {details}");
    }
    fn line(&mut self) -> String {
        let deadline = Instant::now() + Duration::from_secs(8);
        loop {
            let broken = [&self.stdout, &self.stderr].into_iter().any(|state| {
                let saved = state.lock().unwrap();
                saved.exceeded || saved.error.is_some()
            });
            if broken {
                self.fail("bounded channel capture failed");
            }
            match self.lines.try_recv() {
                Ok(value) => return value,
                Err(mpsc::TryRecvError::Empty) => {}
                Err(error) => self.fail(format!("phase channel closed: {error}")),
            }
            if Instant::now() >= deadline {
                self.fail("unchanged eight-second phase deadline");
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    fn command(&mut self, command: &str) {
        let result = self
            .child
            .stdin
            .as_mut()
            .ok_or_else(|| std::io::Error::other("missing owned stdin"))
            .and_then(|input| {
                writeln!(input, "{command}")?;
                input.flush()
            });
        if let Err(error) = result {
            self.fail(format!("phase input: {error}"));
        }
    }
    fn finish(&mut self) -> (std::process::ExitStatus, String) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while self.status.is_none() {
            match self.child.try_wait() {
                Ok(Some(status)) => self.status = Some(status),
                Ok(None) => {}
                Err(error) => self.fail(format!("exit wait: {error}")),
            }
            if Instant::now() >= deadline {
                self.fail("unchanged five-second exit deadline");
            }
            if self.status.is_none() {
                std::thread::yield_now();
            }
        }
        let details = self.retire(false);
        let failed = [&self.stdout, &self.stderr].into_iter().any(|state| {
            let saved = state.lock().unwrap();
            saved.exceeded || saved.error.is_some()
        });
        if failed || !self.retired_clean {
            self.reported = true;
            panic!("native multiple DiffDrive channel capture failed: {details}");
        }
        let status = self.status.unwrap();
        let stderr = String::from_utf8(self.stderr.lock().unwrap().bytes.clone())
            .unwrap_or_else(|error| format!("non-UTF8 native stderr: {error}"));
        (status, stderr)
    }
}
impl Drop for Peer {
    fn drop(&mut self) {
        if !self.retired || std::thread::panicking() && !self.reported {
            eprintln!(
                "native multiple DiffDrive retained diagnostics: {}",
                self.retire(true)
            );
        }
    }
}

fn decode_hex(value: &str) -> Vec<u8> {
    assert!(!value.is_empty() && value.len() <= 1024 && value.len().is_multiple_of(2));
    value
        .as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let digit = |c: u8| match c {
                b'0'..=b'9' => c - b'0',
                b'a'..=b'f' => c - b'a' + 10,
                _ => panic!("noncanonical packet hex"),
            };
            digit(pair[0]) * 16 + digit(pair[1])
        })
        .collect()
}
fn start_peer(client_config: &Path, options: &ServerOptions, count: usize) -> Peer {
    let mut command = Command::new(&peer().binary);
    command.arg(count.to_string());
    peer().configure_runtime(&mut command);
    command
        .env("GZ_PARTITION", &options.partition)
        .env("GZ_IP", "127.0.0.1")
        .env("ZENOH_CONFIG", client_config)
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
    let out = child.stdout.take().unwrap();
    let err = child.stderr.take().unwrap();
    let (send, lines) = mpsc::sync_channel(129);
    let stdout = Arc::new(Mutex::new(Captured::default()));
    let stderr = Arc::new(Mutex::new(Captured::default()));
    let readers = vec![
        capture(out, Arc::clone(&stdout), Some(send)),
        capture(err, Arc::clone(&stderr), None),
    ];
    Peer {
        child,
        lines,
        readers,
        stdout,
        stderr,
        status: None,
        retired: false,
        retired_clean: false,
        reported: false,
    }
}

#[derive(Debug, PartialEq, Eq)]
struct NativePoseSnapshot {
    id: u64,
    pose: [u64; 7],
}
#[derive(Debug, PartialEq, Eq)]
struct NativeBodySnapshot {
    id: u64,
    state: Vec<u64>,
}
#[derive(Debug, PartialEq, Eq)]
struct NativeSampleSnapshot {
    id: u64,
    time: Duration,
    epoch: u64,
    values: [u64; 5],
}
#[derive(Debug, PartialEq, Eq)]
struct NativePaused {
    time: Duration,
    epoch: u64,
    joints: Vec<NativeJointSnapshot>,
    bodies: Vec<NativeBodySnapshot>,
    poses: Vec<NativePoseSnapshot>,
    samples: Vec<NativeSampleSnapshot>,
}
fn native_paused(server: &Server) -> NativePaused {
    let sim = server.simulation();
    let backend = sim.world().resource::<gz_physics::PhysicsBackend>();
    let mut bodies = Vec::new();
    let mut poses = Vec::new();
    let mut samples = Vec::new();
    for node in sim.world().iter_entities() {
        let Some(id) = node.get::<EntityId>().map(EntityId::get) else {
            continue;
        };
        if let Some(handle) = node.get::<gz_sim_scene::BodyHandle>() {
            let state = backend.state(handle.id()).unwrap();
            bodies.push(NativeBodySnapshot {
                id,
                state: state
                    .position
                    .into_iter()
                    .chain(state.orientation)
                    .chain(state.velocity)
                    .chain(state.angular_velocity)
                    .map(f64::to_bits)
                    .collect(),
            });
        }
        if let Some(pose) = node.get::<WorldPose>() {
            poses.push(NativePoseSnapshot {
                id,
                pose: [
                    pose.translation[0].to_bits(),
                    pose.translation[1].to_bits(),
                    pose.translation[2].to_bits(),
                    pose.rotation[0].to_bits(),
                    pose.rotation[1].to_bits(),
                    pose.rotation[2].to_bits(),
                    pose.rotation[3].to_bits(),
                ],
            });
        }
        if let Some(sample) = node.get::<DiffDriveSample>() {
            samples.push(NativeSampleSnapshot {
                id,
                time: sample.time,
                epoch: sample.epoch,
                values: [
                    sample.pose[0].to_bits(),
                    sample.pose[1].to_bits(),
                    sample.pose[2].to_bits(),
                    sample.twist[0].to_bits(),
                    sample.twist[1].to_bits(),
                ],
            });
        }
    }
    bodies.sort_by_key(|row| row.id);
    poses.sort_by_key(|row| row.id);
    samples.sort_by_key(|row| row.id);
    NativePaused {
        time: sim.elapsed(),
        epoch: sim.epoch(),
        joints: native(sim),
        bodies,
        poses,
        samples,
    }
}
#[derive(Default)]
struct NativeSequence {
    previous: Option<(Duration, u64)>,
}
fn native_odometry(
    line: &str,
    owner: usize,
    sequence: &mut NativeSequence,
) -> (Duration, Odometry) {
    let words: Vec<_> = line.split(' ').collect();
    assert_eq!(words.len(), 3, "{line}");
    assert_eq!(words[0], "ODOM");
    assert_eq!(words[1], owner.to_string());
    let wire = decode_hex(words[2]);
    assert!(wire.len() <= 512);
    let message = Odometry::decode(wire.as_slice()).unwrap();
    // Reject extra/presence/noncanonical wire fields, not only numeric subsets.
    assert_eq!(message.encode_to_vec(), wire);
    let header = message.header.as_ref().unwrap();
    assert_eq!(header.data.len(), 3);
    let side = ["left", "right"][owner];
    assert_eq!(header.data[0].key, "frame_id");
    assert_eq!(header.data[0].value, [format!("m59_{side}_odom")]);
    assert_eq!(header.data[1].key, "child_frame_id");
    assert_eq!(header.data[1].value, [format!("m59_{side}_base")]);
    assert_eq!(header.data[2].key, "seq");
    assert_eq!(header.data[2].value.len(), 1);
    let next = header.data[2].value[0].parse::<u64>().unwrap();
    assert_eq!(header.data[2].value[0], next.to_string());
    let stamp = header.stamp.as_ref().unwrap();
    assert!(stamp.sec >= 0 && stamp.nsec >= 0 && stamp.nsec < 1_000_000_000);
    let time = Duration::new(stamp.sec as u64, stamp.nsec as u32);
    if let Some((previous, value)) = sequence.previous {
        assert!(time > previous);
        assert_eq!(next, value.wrapping_add(1));
    } else {
        // This consumer opens a stream only initially or after its real reset.
        // Rewatch by itself is never treated as an epoch/sequence-zero proof.
        assert_eq!(next, 0);
    }
    sequence.previous = Some((time, next));
    let pose = message.pose.as_ref().unwrap();
    assert!(pose.header.is_none());
    assert!(pose.position.as_ref().unwrap().header.is_none());
    assert!(pose.orientation.as_ref().unwrap().header.is_none());
    let twist = message.twist.as_ref().unwrap();
    assert!(twist.header.is_none());
    assert!(twist.linear.as_ref().unwrap().header.is_none());
    assert!(twist.angular.as_ref().unwrap().header.is_none());
    (time, message)
}
fn native_take(
    peer: &mut Peer,
    owner: usize,
    expected: &[Expected],
    sequence: &mut NativeSequence,
) {
    if expected.is_empty() {
        return;
    }
    assert!(expected.len() <= 128);
    peer.command(&format!("take {owner} {}", expected.len()));
    for value in expected {
        let (time, message) = native_odometry(&peer.line(), owner, sequence);
        assert_eq!(time, value.time);
        complete(message, value);
    }
    assert_eq!(peer.line(), format!("TAKEN {owner} {}", expected.len()));
}
fn native_step_rows(
    server: &mut Server,
    owners: &[bevy_ecs::entity::Entity; 2],
    rows: &mut [Vec<Expected>; 2],
    last: &mut [Option<Duration>; 2],
) {
    let info = server.update().unwrap();
    assert!(!info.dt.is_zero());
    for (owner, entity) in owners.iter().enumerate() {
        if let Some(value) = server.simulation().world().get::<DiffDriveSample>(*entity)
            && value.time == info.sim_time
            && last[owner] != Some(value.time)
        {
            assert_eq!(value.epoch, server.simulation().epoch());
            last[owner] = Some(value.time);
            rows[owner].push(Expected {
                time: value.time,
                pose: value.pose,
                twist: value.twist,
            });
        }
    }
}
fn native_fence(
    server: &mut Server,
    peer: &mut Peer,
    owner: usize,
    watched: &mut [bool; 2],
    sequences: &mut [NativeSequence; 2],
) {
    assert!(!watched[owner]);
    peer.command(&format!("watch {owner}"));
    assert_eq!(peer.line(), format!("WATCHING {owner}"));
    let owners = [
        model(server.simulation(), "left").0,
        model(server.simulation(), "right").0,
    ];
    let baseline = server.counters().odometries_queued;
    let mut rows: [Vec<Expected>; 2] = [Vec::new(), Vec::new()];
    let mut last = [None, None];
    let deadline = Instant::now() + Duration::from_secs(5);
    peer.command(&format!("take {owner} 1"));
    pause(server, false);
    let line = loop {
        assert!(
            Instant::now() < deadline,
            "actual first native Odometry fence"
        );
        // At most 50 acquisitions/owner: the other active callback stays below128.
        assert!(
            rows.iter().all(|values| values.len() < 50),
            "bounded warmup acquisition window"
        );
        native_step_rows(server, &owners, &mut rows, &mut last);
        match peer.lines.try_recv() {
            Ok(line) => break line,
            Err(mpsc::TryRecvError::Empty) => {}
            Err(error) => peer.fail(format!("actual output fence: {error}")),
        }
        std::thread::sleep(Duration::from_millis(1));
    };
    assert!(
        Instant::now() < deadline,
        "actual output fence deadline after receipt"
    );
    pause(server, true);
    let (first_time, first) = native_odometry(&line, owner, &mut sequences[owner]);
    let first_index = rows[owner]
        .iter()
        .position(|row| row.time == first_time)
        .unwrap();
    complete(first, &rows[owner][first_index]);
    assert_eq!(peer.line(), format!("TAKEN {owner} 1"));
    let mut actual_count = 0;
    for (index, values) in rows.iter().enumerate() {
        if index == owner {
            actual_count += values.len() - first_index;
            native_take(
                peer,
                index,
                &values[first_index + 1..],
                &mut sequences[index],
            );
        } else if watched[index] {
            actual_count += values.len();
            native_take(peer, index, values, &mut sequences[index]);
        }
    }
    assert_eq!(
        server.counters().odometries_queued - baseline,
        actual_count as u64
    );
    watched[owner] = true;
}
fn native_commands(server: &mut Server, peer: &mut Peer, commands: &[(usize, f64, f64)]) {
    let before = native_paused(server);
    let received = server.counters().diff_drive_messages_received;
    let applied = server.counters().diff_drive_commands_applied;
    let rejected = server.counters().diff_drive_messages_rejected;
    for &(owner, linear, angular) in commands {
        let linear_word = format!("{:016x}", linear.to_bits());
        let angular_word = format!("{:016x}", angular.to_bits());
        peer.command(&format!("send {owner} {linear_word} {angular_word}"));
        let line = peer.line();
        let words: Vec<_> = line.split(' ').collect();
        assert_eq!(words.len(), 5, "{line}");
        assert_eq!(
            words[..4],
            ["SENT", &owner.to_string(), &linear_word, &angular_word]
        );
        let wire = decode_hex(words[4]);
        let message = Twist::decode(wire.as_slice()).unwrap();
        assert_eq!(message, twist(linear, angular));
        assert_eq!(message.encode_to_vec(), wire);
    }
    wait(|| {
        assert!(server.update().unwrap().dt.is_zero());
        assert!(server.counters().diff_drive_messages_received <= received + commands.len() as u64);
        server.counters().diff_drive_messages_received == received + commands.len() as u64
    });
    assert_eq!(
        server.counters().diff_drive_commands_applied,
        applied + commands.len() as u64
    );
    assert_eq!(server.counters().diff_drive_messages_rejected, rejected);
    assert_eq!(native_paused(server), before);
}
fn native_window(
    server: &mut Server,
    peer: &mut Peer,
    steps: usize,
    watched: [bool; 2],
    sequences: &mut [NativeSequence; 2],
) {
    let owners = ["left", "right"].map(|side| {
        server
            .simulation()
            .world()
            .iter_entities()
            .find(|node| {
                node.get::<Name>()
                    .is_some_and(|name| name.0 == format!("default::robot_{side}"))
            })
            .map(|node| node.id())
    });
    let baseline = server.counters().odometries_queued;
    let mut rows: [Vec<Expected>; 2] = [Vec::new(), Vec::new()];
    let mut last = [None, None];
    pause(server, false);
    for _ in 0..steps {
        let info = server.update().unwrap();
        assert!(!info.dt.is_zero());
        for (owner, entity) in owners.iter().copied().enumerate() {
            if let Some(entity) = entity
                && let Some(value) = server.simulation().world().get::<DiffDriveSample>(entity)
                && value.time == info.sim_time
                && last[owner] != Some(value.time)
            {
                assert_eq!(value.epoch, server.simulation().epoch());
                last[owner] = Some(value.time);
                rows[owner].push(Expected {
                    time: value.time,
                    pose: value.pose,
                    twist: value.twist,
                });
            }
        }
    }
    pause(server, true);
    let mut count = 0;
    for (owner, values) in rows.iter().enumerate() {
        assert_eq!(
            values.len(),
            if owners[owner].is_some() {
                steps / 20
            } else {
                0
            }
        );
        if watched[owner] {
            count += values.len();
            native_take(peer, owner, values, &mut sequences[owner]);
        }
    }
    assert_eq!(server.counters().odometries_queued - baseline, count as u64);
    let held = native_paused(server);
    let queued = server.counters().odometries_queued;
    for _ in 0..16 {
        assert!(server.update().unwrap().dt.is_zero());
    }
    assert_eq!(native_paused(server), held);
    assert_eq!(server.counters().odometries_queued, queued);
}
fn native_unwatch_all(peer: &mut Peer, watched: &mut [bool; 2]) {
    for (owner, watching) in watched.iter_mut().enumerate() {
        if *watching {
            peer.command(&format!("unwatch {owner}"));
            assert_eq!(peer.line(), format!("UNWATCHED {owner}"));
            *watching = false;
        }
    }
}
fn native_reset(server: &mut Server, initial: &NativePaused, ids: [u64; 2]) {
    let epoch = server.simulation().epoch();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.prepare_io().unwrap();
    let info = server.tick().unwrap();
    assert!(info.sim_time.is_zero() && info.dt.is_zero());
    assert_eq!(server.simulation().epoch(), epoch + 1);
    assert_eq!(
        [
            model(server.simulation(), "left").1,
            model(server.simulation(), "right").1
        ],
        ids
    );
    let reset = native_paused(server);
    assert_eq!(reset.joints, initial.joints);
    assert_eq!(reset.bodies, initial.bodies);
    assert_eq!(reset.poses, initial.poses);
    assert!(reset.samples.is_empty());
}

#[test]
fn canonical_cpp_twist_and_odometry_keep_two_real_motor_owners_demand_and_reset_epochs() {
    const NATIVE_XML: &str = include_str!("../../../examples/m59_multiple_diff_drives.sdf");
    let observer = zenoh::open(config(&[])).wait().unwrap();
    let endpoints: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|v| v.to_string())
        .collect();
    assert_eq!(endpoints.len(), 1);
    let dir = tempfile::tempdir().unwrap();
    let client_config = dir.path().join("client.json");
    let clients = serde_json::json!({"mode":"client", "listen":{"endpoints":[]},
        "connect":{"endpoints":endpoints,"timeout_ms":2000},
        "scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},
        "transport":{"shared_memory":{"enabled":false}}});
    std::fs::write(&client_config, clients.to_string()).unwrap();
    let options = ServerOptions {
        partition: format!("m59-native-{}", uuid::Uuid::new_v4()),
        capacity: 4096,
        max_message_bytes: 512,
        ..Default::default()
    };
    let sim = load_world_with_sdf_step(
        NATIVE_XML,
        "m59-canonical-peer.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let mut server = Server::with_zenoh_config(
        sim,
        "default",
        options.clone(),
        config(&endpoints),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    let ids = [
        model(server.simulation(), "left").1,
        model(server.simulation(), "right").1,
    ];
    let initial = native_paused(&server);
    assert!(initial.samples.is_empty());
    let mut peer = start_peer(&client_config, &options, 2);
    assert_eq!(peer.line(), "READY multiple-diff-drives 2");
    let mut watched = [false; 2];
    let mut sequences = [NativeSequence::default(), NativeSequence::default()];

    // Right is NEVER subscribed in this interval. Its native motion and encoder
    // still progress; only the actual demanded left acquisition list is queued.
    native_fence(&mut server, &mut peer, 0, &mut watched, &mut sequences);
    let start = server.simulation().elapsed();
    let x0 = [
        chassis_x(server.simulation(), "left"),
        chassis_x(server.simulation(), "right"),
    ];
    native_commands(&mut server, &mut peer, &[(0, 0.5, 0.0), (1, -0.25, 0.5)]);
    let baseline = server.counters().odometries_queued;
    assert_eq!(watched, [true, false]);
    for _ in 0..3 {
        native_window(&mut server, &mut peer, 1000, watched, &mut sequences);
    }
    assert_eq!(
        server.simulation().elapsed() - start,
        Duration::from_secs(3)
    );
    assert_eq!(server.counters().odometries_queued - baseline, 150);
    assert_eq!(watched, [true, false]);
    let diagnostic = motion_diagnostics(&server, start, x0);
    assert!(
        chassis_x(server.simulation(), "left") - x0[0] > 0.05,
        "{diagnostic}"
    );
    assert!(
        chassis_x(server.simulation(), "right") - x0[1] < -0.05,
        "{diagnostic}"
    );
    for side in ["left", "right"] {
        let backend = server
            .simulation()
            .world()
            .resource::<gz_physics::PhysicsBackend>();
        let states: Vec<_> = server
            .simulation()
            .world()
            .iter_entities()
            .filter(|node| {
                node.get::<Name>()
                    .is_some_and(|name| name.0.starts_with(&format!("default::robot_{side}::")))
            })
            .filter_map(|node| {
                node.get::<JointHandle>()
                    .map(|handle| backend.joint_state(handle.id()).unwrap())
            })
            .collect();
        assert_eq!(states.len(), 2);
        assert!(
            states
                .iter()
                .all(|state| state.position.is_finite() && state.velocity.abs() > 0.01)
        );
    }
    native_fence(&mut server, &mut peer, 1, &mut watched, &mut sequences);
    native_window(&mut server, &mut peer, 1000, watched, &mut sequences);

    native_unwatch_all(&mut peer, &mut watched);
    native_reset(&mut server, &initial, ids);
    sequences = [NativeSequence::default(), NativeSequence::default()];
    for owner in 0..2 {
        peer.command(&format!("matched {owner}"));
        assert_eq!(peer.line(), format!("MATCHED {owner}"));
        native_fence(&mut server, &mut peer, owner, &mut watched, &mut sequences);
    }
    native_commands(&mut server, &mut peer, &[(0, 0.25, 0.0), (1, -0.5, 0.0)]);
    native_window(&mut server, &mut peer, 200, watched, &mut sequences);
    remove_loaded_model(&mut server, &observer, &options, ids[0]);
    server.prepare_io().unwrap();
    assert!(server.tick().unwrap().dt.is_zero());
    peer.command("retired 0");
    assert_eq!(peer.line(), "RETIRED 0");
    peer.command("unwatch 0");
    assert_eq!(peer.line(), "UNWATCHED 0");
    watched[0] = false;
    assert!(!server.simulation().world().iter_entities().any(|node| {
        node.get::<EntityId>()
            .is_some_and(|value| value.get() == ids[0])
    }));
    peer.command("matched 1");
    assert_eq!(peer.line(), "MATCHED 1");
    native_commands(&mut server, &mut peer, &[(1, -0.25, 0.0)]);
    native_window(&mut server, &mut peer, 200, watched, &mut sequences);

    native_unwatch_all(&mut peer, &mut watched);
    native_reset(&mut server, &initial, ids);
    sequences = [NativeSequence::default(), NativeSequence::default()];
    for owner in 0..2 {
        peer.command(&format!("matched {owner}"));
        assert_eq!(peer.line(), format!("MATCHED {owner}"));
        native_fence(&mut server, &mut peer, owner, &mut watched, &mut sequences);
    }
    native_window(&mut server, &mut peer, 40, watched, &mut sequences);
    drop(server);
    for owner in 0..2 {
        peer.command(&format!("retired {owner}"));
        assert_eq!(peer.line(), format!("RETIRED {owner}"));
    }
    native_unwatch_all(&mut peer, &mut watched);
    peer.command("finish");
    assert_eq!(peer.line(), "PASS canonical multiple diff drives");
    let (status, stderr) = peer.finish();
    assert!(status.success(), "{status:?}: {stderr}");
    assert!(stderr.is_empty(), "{stderr}");
    observer.close().wait().unwrap();
}
