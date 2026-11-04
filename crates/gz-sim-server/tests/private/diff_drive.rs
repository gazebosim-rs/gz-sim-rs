// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn metadata() -> DiffDriveConfiguration {
    DiffDriveConfiguration {
        model_name: "robot".into(),
        topic: "/model/robot/cmd_vel".into(),
        odom_topic: "/model/robot/odometry".into(),
        frame_id: "robot/odom".into(),
        child_frame_id: "robot/base_link".into(),
    }
}
fn sample() -> DiffDriveSample {
    DiffDriveSample {
        time: Duration::from_nanos(1_234_567_890),
        epoch: 7,
        pose: [1.0, -2.0, std::f64::consts::FRAC_PI_2],
        twist: [0.75, -0.25],
    }
}

#[test]
fn canonical_odometry_uses_encoder_pose_planar_twist_frames_sequence_and_integer_time() {
    let packet = packet(&metadata(), &sample(), 12).unwrap();
    let bytes = packet.encode_to_vec();
    let decoded = Odometry::decode(bytes.as_slice()).unwrap();
    assert_eq!(decoded, packet);
    let header = decoded.header.unwrap();
    assert_eq!(
        header.stamp,
        Some(Time {
            sec: 1,
            nsec: 234_567_890
        })
    );
    assert_eq!(
        header
            .data
            .iter()
            .map(|d| (d.key.as_str(), d.value[0].as_str()))
            .collect::<Vec<_>>(),
        vec![
            ("frame_id", "robot/odom"),
            ("child_frame_id", "robot/base_link"),
            ("seq", "12")
        ]
    );
    let pose = decoded.pose.unwrap();
    assert_eq!(pose.position.unwrap(), vector3([1.0, -2.0, 0.0]));
    let q = pose.orientation.unwrap();
    assert_eq!([q.x, q.y], [0.0; 2]);
    assert!((q.z - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12);
    assert!((q.w - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12);
    let twist = decoded.twist.unwrap();
    assert_eq!(twist.linear.unwrap(), vector3([0.75, 0.0, 0.0]));
    assert_eq!(twist.angular.unwrap(), vector3([0.0, 0.0, -0.25]));
    assert_eq!(header.data.len(), 3); // Internal reset epoch is not a wire field.
}

#[test]
fn malformed_public_metadata_nonfinite_samples_and_unrepresentable_times_are_refused() {
    for field in 0..5 {
        let mut m = metadata();
        match field {
            0 => m.model_name.clear(),
            1 => m.topic.clear(),
            2 => m.odom_topic = "relative".into(),
            3 => m.frame_id.clear(),
            _ => m.child_frame_id.clear(),
        }
        assert!(packet(&m, &sample(), 0).is_none());
    }
    for topic in ["/bad topic", "/bad//topic", "@bad"] {
        let mut m = metadata();
        m.topic = topic.into();
        assert!(!valid(&m));
    }
    let mut m = metadata();
    m.odom_topic.clone_from(&m.topic);
    assert!(!valid(&m));
    for index in 0..5 {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut s = sample();
            if index < 3 {
                s.pose[index] = value;
            } else {
                s.twist[index - 3] = value;
            }
            assert!(packet(&metadata(), &s, 0).is_none());
        }
    }
    let mut s = sample();
    s.time = Duration::MAX;
    assert!(packet(&metadata(), &s, 0).is_none());
}

use gz_sim_proto::{
    msgs::{WorldControl, WorldReset},
    prost::Name,
};
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, remove_model};
use std::time::Instant;
const XML: &str = include_str!("../../../../examples/m8_diff_drive.sdf");

fn config(endpoint: Option<&str>) -> zenoh::Config {
    let mut config = zenoh::Config::default();
    let connect = endpoint.map_or_else(|| "[]".into(), |value| format!("[\"{value}\"]"));
    for (path, value) in [
        ("mode", "\"peer\""),
        (
            "listen/endpoints",
            if endpoint.is_some() {
                "[]"
            } else {
                "[\"tcp/127.0.0.1:0\"]"
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
fn build_server(xml: &str) -> (Server, zenoh::Session, String, String) {
    let observer = zenoh::open(config(None)).wait().unwrap();
    let locators: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|x| x.to_string())
        .collect();
    assert_eq!(locators.len(), 1);
    assert_ne!(locators[0], "tcp/127.0.0.1:0");
    let sim = load_world_with_sdf_step(
        xml,
        "m8-server.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let options = ServerOptions {
        capacity: 4096,
        partition: format!("drive-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    };
    let command = options.topic_key("/model/robot/cmd_vel").unwrap();
    let odometry = options.topic_key("/model/robot/odometry").unwrap();
    let server = Server::with_zenoh_config(
        sim,
        "default",
        options,
        config(Some(&locators[0])),
        NonZeroUsize::new(4).unwrap(),
    )
    .unwrap();
    (server, observer, command, odometry)
}
fn wait(mut condition: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !condition() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
}
fn command(linear: f64, angular: f64) -> Twist {
    Twist {
        linear: Some(vector3([linear, 0.0, 0.0])),
        angular: Some(vector3([0.0, 0.0, angular])),
        ..Default::default()
    }
}
fn ns(time: &Time) -> i128 {
    i128::from(time.sec) * 1_000_000_000 + i128::from(time.nsec)
}

#[test]
fn actual_twist_and_encoder_odometry_preserve_frames_cadence_pause_reset_and_removal() {
    let (mut server, observer, command_key, odometry_key) = build_server(XML);
    let publisher = observer.declare_publisher(command_key).wait().unwrap();
    let subscriber = observer.declare_subscriber(odometry_key).wait().unwrap();
    wait(|| {
        publisher.matching_status().wait().unwrap().matching()
            && server
                .diff_drive
                .as_ref()
                .unwrap()
                .output
                .matching()
                .unwrap()
    });
    server.set_paused(true);
    publisher
        .put(command(0.4, 0.0).encode_to_vec())
        .attachment(Twist::full_name())
        .wait()
        .unwrap();
    wait(|| {
        let info = server.update().unwrap();
        assert!(info.dt.is_zero());
        server.counters().diff_drive_commands_applied == 1
    });
    assert_eq!(server.simulation.elapsed(), Duration::ZERO);
    // Drain any real initial snapshot before counting the advancing acquisition window.
    let baseline = server.counters().odometries_queued;
    for _ in 0..baseline {
        subscriber
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap();
    }
    let identity = server.diff_drive.as_ref().unwrap().identity;
    let entity = server.diff_drive.as_ref().unwrap().entity;
    let sequence = server.diff_drive.as_ref().unwrap().sequence;
    let mut expected = Vec::new();
    let mut previous = None;
    server.set_paused(false);
    for tick in 1..=1000 {
        let info = server.update().unwrap();
        assert_eq!(info.sim_time, Duration::from_millis(tick));
        if let Some(sample) = server.simulation.world().get::<DiffDriveSample>(entity)
            && sample.time == info.sim_time
            && previous != Some(sample.time)
        {
            previous = Some(sample.time);
            expected.push((sample.time, sample.pose, sample.twist));
        }
    }
    assert_eq!(expected.len(), 50);
    assert_eq!(server.counters().odometries_queued - baseline, 50);
    let mut previous_stamp = None;
    for (index, (time, pose, twist)) in expected.iter().enumerate() {
        let received = subscriber
            .recv_timeout(Duration::from_secs(5))
            .unwrap()
            .unwrap();
        assert_eq!(
            received.attachment().unwrap().to_bytes().as_ref(),
            b"gz.msgs.Odometry"
        );
        let message = Odometry::decode(received.payload().to_bytes().as_ref()).unwrap();
        let header = message.header.unwrap();
        assert_eq!(
            header.data,
            vec![
                Map {
                    key: "frame_id".into(),
                    value: vec!["robot/odom".into()]
                },
                Map {
                    key: "child_frame_id".into(),
                    value: vec!["robot/base_link".into()]
                },
                Map {
                    key: "seq".into(),
                    value: vec![(sequence + index as u64).to_string()]
                }
            ]
        );
        let stamp = ns(&header.stamp.unwrap());
        assert_eq!(stamp, time.as_nanos() as i128);
        if let Some(previous) = previous_stamp {
            assert_eq!(stamp - previous, 20_000_000);
        }
        previous_stamp = Some(stamp);
        let position = message.pose.as_ref().unwrap().position.as_ref().unwrap();
        assert_eq!(
            [position.x, position.y, position.z],
            [pose[0], pose[1], 0.0]
        );
        let q = message.pose.unwrap().orientation.unwrap();
        let (sin, cos) = (pose[2] * 0.5).sin_cos();
        assert_eq!([q.x, q.y, q.z, q.w], [0.0, 0.0, sin, cos]);
        let wire_twist = message.twist.unwrap();
        assert_eq!(wire_twist.linear.unwrap(), vector3([twist[0], 0.0, 0.0]));
        assert_eq!(wire_twist.angular.unwrap(), vector3([0.0, 0.0, twist[1]]));
    }
    assert!(expected.last().unwrap().1[0] > 0.01);
    assert!(expected.last().unwrap().2[0] > 0.0);
    server.set_paused(true);
    let time = server.simulation.elapsed();
    for _ in 0..3 {
        assert!(server.update().unwrap().dt.is_zero());
    }
    assert_eq!(server.simulation.elapsed(), time);
    assert!(subscriber.try_recv().unwrap().is_none());
    let mut unsupported = command(1.0, 0.0);
    unsupported.linear.as_mut().unwrap().y = 0.01;
    for (message, kind) in [
        (command(f64::NAN, 0.0), Twist::full_name()),
        (unsupported, Twist::full_name()),
        (command(-0.4, 0.0), "gz.msgs.Double".into()),
    ] {
        let received = server.counters().diff_drive_messages_received;
        publisher
            .put(message.encode_to_vec())
            .attachment(kind)
            .wait()
            .unwrap();
        wait(|| {
            assert!(server.update().unwrap().dt.is_zero());
            server.counters().diff_drive_messages_received > received
        });
    }
    assert_eq!(server.counters().diff_drive_messages_rejected, 3);
    assert_eq!(server.counters().diff_drive_commands_applied, 1);
    let weak = Arc::downgrade(&server.diff_drive.as_ref().unwrap().output);
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    let reset = server.update().unwrap();
    assert_eq!((reset.epoch, reset.sim_time), (1, Duration::ZERO));
    assert!(weak.upgrade().is_none());
    assert_eq!(server.diff_drive.as_ref().unwrap().identity, identity);
    // First real post-reset frame, whether the zero snapshot or first due sample, is seq0.
    let queued = server.counters().odometries_queued;
    if queued == baseline + 50 {
        server.set_paused(false);
        for _ in 0..20 {
            server.update().unwrap();
            if server.counters().odometries_queued > queued {
                break;
            }
        }
    }
    let received = subscriber
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap();
    let message = Odometry::decode(received.payload().to_bytes().as_ref()).unwrap();
    assert_eq!(message.header.as_ref().unwrap().data[2].value, ["0"]);
    assert!(ns(message.header.as_ref().unwrap().stamp.as_ref().unwrap()) <= 20_000_000);
    assert!(server.counters().odometries_queued > baseline + 50);
    server.set_paused(true);
    remove_model(&mut server.simulation, identity).unwrap();
    server.update().unwrap();
    assert!(server.diff_drive.is_none());
    wait(|| !publisher.matching_status().wait().unwrap().matching());
    assert_eq!(
        (
            server.counters().odometry_failures,
            server.counters().invalid_odometry_samples
        ),
        (0, 0)
    );
    server.shutdown();
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
}

#[test]
fn output_interest_never_gates_actuation_and_runtime_owner_mutation_retires_the_lease() {
    let (mut server, observer, command_key, _) = build_server(XML);
    let publisher = observer.declare_publisher(command_key).wait().unwrap();
    wait(|| publisher.matching_status().wait().unwrap().matching());
    server.set_paused(true);
    publisher
        .put(command(0.3, 0.4).encode_to_vec())
        .attachment(Twist::full_name())
        .wait()
        .unwrap();
    wait(|| {
        server.update().unwrap();
        server.counters().diff_drive_commands_applied == 1
    });
    let entity = server.diff_drive.as_ref().unwrap().entity;
    server.set_paused(false);
    for _ in 0..200 {
        server.update().unwrap();
    }
    assert_eq!(server.counters().odometries_queued, 0);
    let s = server
        .simulation
        .world()
        .get::<DiffDriveSample>(entity)
        .unwrap();
    assert!(s.pose[0] > 0.0 && s.twist[0] > 0.0 && s.twist[1] > 0.0);
    let fresh = DiffDriveSample {
        time: server.simulation.elapsed(),
        epoch: server.simulation.epoch(),
        pose: s.pose,
        twist: s.twist,
    };
    let weak = Arc::downgrade(&server.diff_drive.as_ref().unwrap().output);
    // A fresh valid public sample cannot bypass the immutable model-owner guard.
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert((EntityKind::Link, fresh));
    server.set_interest_policy(InterestPolicy::Always);
    server.publish_odometry();
    assert_eq!(server.counters().odometries_queued, 0);
    server.retire_diff_drive();
    assert!(server.diff_drive.is_none() && weak.upgrade().is_none());
    wait(|| !publisher.matching_status().wait().unwrap().matching());
}

#[test]
fn setup_and_reset_require_exact_unique_model_metadata_and_absent_system_has_no_stream() {
    let (mut server, _observer, _, _) = build_server(XML);
    let entity = server.diff_drive.as_ref().unwrap().entity;
    let original = server
        .simulation
        .world()
        .get::<DiffDriveConfiguration>(entity)
        .unwrap()
        .clone();
    let check = |sim: &Simulation| {
        stream(
            sim,
            &server.world_io,
            &server.network_config,
            &server.imus,
            &server.lidars,
            None,
        )
    };
    let mut changed = original.clone();
    changed.topic = "/bad topic".into();
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(changed);
    assert!(matches!(
        check(&server.simulation),
        Err(Error::DiffDriveIdentity)
    ));
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(original.clone());
    for topic in [
        "/clock",
        "/world/default/clock",
        "/world/default/pose/info",
        "/world/default/control",
    ] {
        let mut changed = original.clone();
        changed.topic = topic.into();
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(changed);
        assert!(matches!(
            check(&server.simulation),
            Err(Error::DiffDriveIdentity)
        ));
    }
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(original.clone());
    let extra = server.simulation.world_mut().spawn(original.clone()).id();
    assert!(matches!(
        check(&server.simulation),
        Err(Error::DiffDriveIdentity)
    ));
    server.simulation.world_mut().despawn(extra);
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(EntityKind::Link);
    assert!(matches!(
        check(&server.simulation),
        Err(Error::DiffDriveIdentity)
    ));
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(EntityKind::Model);
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .remove::<DiffDriveConfiguration>();
    assert!(
        stream(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &server.imus,
            &server.lidars,
            None
        )
        .unwrap()
        .is_none()
    );
    assert!(matches!(
        stream(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &server.imus,
            &server.lidars,
            Some(&vec![original.clone()])
        ),
        Err(Error::ResetDiffDrive)
    ));
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(original);
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.simulation.add_systems(
        gz_sim_core::Phase::Reset,
        |q: bevy_ecs::prelude::Query<(bevy_ecs::entity::Entity, &DiffDriveConfiguration)>,
         mut commands: bevy_ecs::prelude::Commands| {
            for (entity, metadata) in &q {
                let mut changed = metadata.clone();
                changed.frame_id.push_str("/changed");
                commands.entity(entity).insert(changed);
            }
        },
    );
    assert!(matches!(server.update(), Err(Error::ResetDiffDrive)));
    assert!(server.diff_drive.is_none());
    assert!(matches!(server.tick(), Err(Error::ResetStreams)));
    let xml = XML.split("<plugin").next().unwrap().to_owned() + "</model></world></sdf>";
    // Finite wheel limits belong only to the configured drive's closed domain.
    // Without that system, retain the existing generic unlimited-joint contract.
    let finite_limits = "<limit><effort>5</effort><velocity>10</velocity></limit>";
    assert_eq!(xml.matches(finite_limits).count(), 2);
    let xml = xml.replace(finite_limits, "<limit/>");
    let (mut absent, _, _, _) = build_server(&xml);
    assert!(absent.diff_drive.is_none());
    absent.update().unwrap();
    assert!(absent.diff_drive.is_none());
    assert_eq!(
        (
            absent.counters().diff_drive_messages_received,
            absent.counters().odometries_queued
        ),
        (0, 0)
    );
}

#[test]
fn invalid_acquisitions_deduplicate_and_actual_message_bound_failures_advance_sequence() {
    let (mut server, _observer, _, _) = build_server(XML);
    let entity = server.diff_drive.as_ref().unwrap().entity;
    server.set_interest_policy(InterestPolicy::Always);
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .remove::<DiffDriveSample>();
    server.publish_odometry();
    assert_eq!(server.counters().invalid_odometry_samples, 0);
    let insert = |server: &mut Server, epoch, time, pose| {
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(DiffDriveSample {
                epoch,
                time,
                pose,
                twist: [0.0; 2],
            });
    };
    insert(&mut server, u64::MAX, Duration::ZERO, [0.0; 3]);
    server.publish_odometry();
    server.publish_odometry();
    assert_eq!(server.counters().invalid_odometry_samples, 1);
    insert(&mut server, 0, Duration::from_secs(1), [0.0; 3]);
    server.publish_odometry();
    assert_eq!(server.counters().invalid_odometry_samples, 2);
    insert(&mut server, 0, Duration::ZERO, [f64::NAN, 0.0, 0.0]);
    server.publish_odometry();
    assert_eq!(server.counters().invalid_odometry_samples, 3);
    assert_eq!(server.diff_drive.as_ref().unwrap().sequence, 0);
    insert(&mut server, 0, Duration::ZERO, [0.0; 3]);
    server.publish_odometry();
    assert_eq!(server.counters().odometries_queued, 0);
    // A new admitted attempt is bounded by the actual shared WorldIo byte limit.
    server.diff_drive.as_mut().unwrap().last_sample = None;
    server.diff_drive.as_mut().unwrap().sequence = u64::MAX;
    let bound = server.world_io.max_message_bytes;
    server.world_io.max_message_bytes = 1;
    server.publish_odometry();
    assert_eq!(server.counters().odometry_failures, 1);
    assert_eq!(server.diff_drive.as_ref().unwrap().sequence, 0);
    server.world_io.max_message_bytes = bound;
    server.publish_odometry();
    assert_eq!(server.counters().odometries_queued, 0);
    server.diff_drive.as_mut().unwrap().last_sample = None;
    server.publish_odometry();
    assert_eq!(server.counters().odometries_queued, 1);
    assert_eq!(server.diff_drive.as_ref().unwrap().sequence, 1);
    server.set_interest_policy(InterestPolicy::Subscribers);
    server.world_io.session.close().wait().unwrap();
    wait(|| {
        server
            .diff_drive
            .as_ref()
            .unwrap()
            .output
            .matching()
            .is_err()
    });
    server.diff_drive.as_mut().unwrap().last_sample = None;
    server.publish_odometry();
    assert_eq!(server.counters().interest_failures, 1);
}
