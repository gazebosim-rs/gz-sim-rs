// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::{
    msgs::{WorldControl, WorldReset},
    prost::Name as ProtoName,
};
use gz_sim_scene::{
    BodyHandle, JointDefinition, JointHandle, LocalPose, PhysicsEngine, WorldPose,
    load_world_with_sdf_step, remove_model,
};
use std::time::Instant;

const XML: &str = include_str!("../../../../examples/m8_diff_drive.sdf");
fn xml(count: usize) -> String {
    let start = XML.find("<model name=\"robot\"").unwrap();
    let end = start + XML[start..].find("</model>").unwrap() + "</model>".len();
    let robot = &XML[start..end];
    let mut result = XML[..start].to_owned() + &XML[end..];
    let mut models = String::new();
    for (index, side) in ["left", "right", "third"]
        .into_iter()
        .take(count)
        .enumerate()
    {
        models.push_str(
            &robot
                .replace("name=\"robot\"", &format!("name=\"robot_{side}\""))
                .replace(
                    "<pose>0 0 0.2 0 0 0</pose>",
                    &format!("<pose>0 {} 0.2 0 0 0</pose>", [-2, 2, 4][index]),
                )
                .replace("/model/robot/cmd_vel", &format!("/m59/{side}/cmd_vel"))
                .replace("/model/robot/odometry", &format!("/m59/{side}/odom"))
                .replace("robot/odom", &format!("m59_{side}_odom"))
                .replace("robot/base_link", &format!("m59_{side}_base")),
        );
    }
    result = result.replace("</world>", &(models + "</world>"));
    result
}
// Actual M8 models: only names, XY placement and canonical routes differ.
pub(super) fn fleet_xml(count: usize) -> String {
    let start = XML.find("<model name=\"robot\"").unwrap();
    let end = start + XML[start..].find("</model>").unwrap() + "</model>".len();
    let robot = &XML[start..end];
    let mut models = String::new();
    for index in 0..count {
        let x = if index.is_multiple_of(2) { -1.25 } else { 1.25 };
        let y = -2.25 + (index / 2) as f64 * 1.5;
        models.push_str(
            &robot
                .replace("name=\"robot\"", &format!("name=\"robot_{index}\""))
                .replace(
                    "<pose>0 0 0.2 0 0 0</pose>",
                    &format!("<pose>{x} {y} 0.2 0 0 0</pose>"),
                )
                .replace(
                    "/model/robot/cmd_vel",
                    &format!("/m61/robot_{index}/cmd_vel"),
                )
                .replace("/model/robot/odometry", &format!("/m61/robot_{index}/odom"))
                .replace("robot/odom", &format!("m61_robot_{index}_odom"))
                .replace("robot/base_link", &format!("m61_robot_{index}_base")),
        );
    }
    (XML[..start].to_owned() + &XML[end..]).replace("</world>", &(models + "</world>"))
}
fn config(endpoint: Option<&str>) -> zenoh::Config {
    let mut result = zenoh::Config::default();
    let connect = endpoint.map_or_else(|| "[]".into(), |value| format!("[\"{value}\"]"));
    for (key, value) in [
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
        result.insert_json5(key, value).unwrap();
    }
    result
}
fn loaded(source: &str) -> Simulation {
    load_world_with_sdf_step(
        source,
        "m59-server.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn build(count: usize, budget: usize) -> (Server, zenoh::Session) {
    build_source(&xml(count), budget)
}
fn build_fleet(count: usize, budget: usize) -> (Server, zenoh::Session) {
    build_source(&fleet_xml(count), budget)
}
fn build_source(source: &str, budget: usize) -> (Server, zenoh::Session) {
    let observer = zenoh::open(config(None)).wait().unwrap();
    let locators: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|value| value.to_string())
        .collect();
    assert_eq!(locators.len(), 1);
    assert_ne!(locators[0], "tcp/127.0.0.1:0");
    let server = Server::with_zenoh_config(
        loaded(source),
        "default",
        ServerOptions {
            capacity: 4096,
            partition: format!("m59-private-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        },
        config(Some(&locators[0])),
        NonZeroUsize::new(budget).unwrap(),
    )
    .unwrap();
    (server, observer)
}
fn wait(mut condition: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !condition() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
}
fn pause(server: &mut Server) {
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
}
fn twist(v: f64, w: f64) -> Twist {
    Twist {
        linear: Some(vector3([v, 0.0, 0.0])),
        angular: Some(vector3([0.0, 0.0, w])),
        ..Default::default()
    }
}
#[derive(Debug, PartialEq, Eq)]
struct StreamSnapshot {
    identity: u64,
    command: String,
    odometry: String,
    lease: uuid::Uuid,
    pointer: usize,
    strong: usize,
    sequence: u64,
    last: Option<(u64, Duration)>,
}
#[derive(Debug, PartialEq, Eq)]
struct BoundarySnapshot {
    epoch: u64,
    time: Duration,
    received: u64,
    applied: u64,
    rejected: u64,
    queued: u64,
    streams: Vec<StreamSnapshot>,
    native: Vec<(u64, [u64; 2])>,
    samples: Vec<(u64, Duration, u64, [u64; 5])>,
}
fn snapshot(server: &Server) -> BoundarySnapshot {
    let mut native = Vec::new();
    let mut samples = Vec::new();
    for node in server.simulation.world().iter_entities() {
        let id = node.get::<EntityId>().map(EntityId::get).unwrap_or(0);
        if let Some(handle) = node.get::<JointHandle>() {
            let state = server
                .simulation
                .world()
                .resource::<gz_physics::PhysicsBackend>()
                .joint_state(handle.id())
                .unwrap();
            native.push((id, [state.position.to_bits(), state.velocity.to_bits()]));
        }
        if let Some(value) = node.get::<DiffDriveSample>() {
            samples.push((
                id,
                value.time,
                value.epoch,
                [
                    value.pose[0].to_bits(),
                    value.pose[1].to_bits(),
                    value.pose[2].to_bits(),
                    value.twist[0].to_bits(),
                    value.twist[1].to_bits(),
                ],
            ));
        }
    }
    native.sort_by_key(|row| row.0);
    samples.sort_by_key(|row| row.0);
    BoundarySnapshot {
        epoch: server.simulation.epoch(),
        time: server.simulation.elapsed(),
        received: server.counters.diff_drive_messages_received,
        applied: server.counters.diff_drive_commands_applied,
        rejected: server.counters.diff_drive_messages_rejected,
        queued: server.counters.odometries_queued,
        streams: server
            .diff_drive
            .iter()
            .flat_map(|drives| drives.streams.values())
            .map(|s| StreamSnapshot {
                identity: s.identity,
                command: s.command_key.clone(),
                odometry: s.odometry_key.clone(),
                lease: s.lease_id,
                pointer: Arc::as_ptr(&s.output) as usize,
                strong: Arc::strong_count(&s.output),
                sequence: s.sequence,
                last: s.last_sample,
            })
            .collect(),
        native,
        samples,
    }
}
fn foreign(server: &Server) -> (crate::diff_drive_io::Declarations, Installed) {
    let endpoints = server
        .diff_drive
        .as_ref()
        .unwrap()
        .streams
        .values()
        .map(|stream| {
            let (input, _receiver) = mpsc::sync_channel(1);
            Endpoint {
                owner: stream.identity,
                command_key: stream.command_key.clone(),
                odometry_key: stream.odometry_key.clone(),
                input,
            }
        })
        .collect();
    crate::diff_drive_io::Declarations::new(
        &server.world_io.session,
        Setup {
            endpoints,
            max_bytes: 512,
            drops: Arc::clone(&server.world_io.ingress_drops),
            epoch: Arc::clone(&server.world_io.epoch),
        },
        "m59",
        "foreign",
    )
    .unwrap()
}

#[test]
fn zero_one_two_original_owners_and_manual_singleton_preserve_closed_admission() {
    for count in 0..=2 {
        let (server, _observer) = build(count, 2);
        let expected =
            preflight(&server.simulation, &server.world_io, &server.network_config).unwrap();
        assert_eq!(expected.len(), count);
        assert_eq!(
            server.diff_drive.as_ref().map_or(0, |d| d.streams.len()),
            count
        );
        assert_eq!(initial(&server.simulation).unwrap().len(), count);
        assert_eq!(originals(&server.simulation).unwrap().unwrap().len(), count);
        if count == 0 {
            assert!(
                prepare(
                    &server.simulation,
                    &server.world_io,
                    &server.network_config,
                    &expected
                )
                .unwrap()
                .is_none()
            );
        }
    }
    let (server, _) = build(0, 2);
    let mut manual = Simulation::new(Duration::from_millis(1)).unwrap();
    let metadata = DiffDriveConfiguration {
        model_name: "manual".into(),
        topic: "/manual/cmd".into(),
        odom_topic: "/manual/odom".into(),
        frame_id: "manual_odom".into(),
        child_frame_id: "manual_base".into(),
    };
    assert!(initial(&manual).unwrap().is_empty());
    let entity = manual
        .world_mut()
        .spawn((EntityKind::Model, metadata.clone()))
        .id();
    let id = gz_sim_core::assign_id(manual.world_mut(), entity).unwrap();
    assert_eq!(
        preflight(&manual, &server.world_io, &server.network_config).unwrap(),
        BTreeMap::from([(id, metadata.clone())])
    );
    let second = manual.world_mut().spawn((EntityKind::Model, metadata)).id();
    gz_sim_core::assign_id(manual.world_mut(), second).unwrap();
    assert!(matches!(
        preflight(&manual, &server.world_io, &server.network_config),
        Err(Error::DiffDriveIdentity)
    ));
}

#[test]
fn all_four_mapped_keys_and_late_owner_identity_refuse_before_io_or_lease_clones() {
    let (mut server, _observer) = build(2, 2);
    let expected = prepare_reset(&server).unwrap();
    let ids: Vec<_> = server
        .diff_drive
        .as_ref()
        .unwrap()
        .streams
        .keys()
        .copied()
        .collect();
    let entity = server.diff_drive.as_ref().unwrap().streams[&ids[1]].entity;
    for fault in 0..4 {
        let before = snapshot(&server);
        let node = server.simulation.world().entity(entity);
        let name = Name(node.get::<Name>().unwrap().0.clone());
        let metadata = node.get::<DiffDriveConfiguration>().unwrap().clone();
        match fault {
            0 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(EntityKind::Link);
            }
            1 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(Name("foreign::model".into()));
            }
            2 => {
                let mut changed = metadata.clone();
                changed.frame_id.push_str("/changed");
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(changed);
            }
            _ => {
                let mut changed = metadata.clone();
                changed.model_name.push_str("_foreign");
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(changed);
            }
        }
        assert!(matches!(
            preflight(&server.simulation, &server.world_io, &server.network_config),
            Err(Error::DiffDriveIdentity)
        ));
        assert!(matches!(
            prepare(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &expected
            ),
            Err(Error::DiffDriveIdentity)
        ));
        let restored = prepare_restored(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &expected,
        );
        if fault == 2 {
            assert!(matches!(restored, Err(Error::ResetDiffDrive)));
        } else {
            assert!(matches!(restored, Err(Error::DiffDriveIdentity)));
        }
        assert_eq!(snapshot(&server), before);
        server.simulation.world_mut().entity_mut(entity).insert((
            EntityKind::Model,
            name,
            metadata,
        ));
    }
    let original = server
        .simulation
        .world()
        .get::<DiffDriveConfiguration>(entity)
        .unwrap()
        .clone();
    for malformed in [false, true] {
        let before = snapshot(&server);
        let mut changed = original.clone();
        if malformed {
            changed.frame_id.clear();
        } else {
            changed.frame_id.push_str("/changed");
            changed.topic = "/clock".into();
        }
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(changed);
        assert!(matches!(
            prepare_restored(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &expected
            ),
            Err(Error::DiffDriveIdentity)
        ));
        assert_eq!(snapshot(&server), before);
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(original.clone());
    }
    let configured: Vec<_> = server
        .diff_drive
        .as_ref()
        .unwrap()
        .streams
        .values()
        .map(|stream| (stream.entity, stream.metadata.clone()))
        .collect();
    for cross_channel in [false, true] {
        let before = snapshot(&server);
        let mut first = configured[0].1.clone();
        let mut second = configured[1].1.clone();
        if cross_channel {
            std::mem::swap(&mut first.topic, &mut first.odom_topic);
        } else {
            std::mem::swap(&mut first.topic, &mut second.topic);
        }
        server
            .simulation
            .world_mut()
            .entity_mut(configured[0].0)
            .insert(first);
        server
            .simulation
            .world_mut()
            .entity_mut(configured[1].0)
            .insert(second);
        // Actual keys remain distinct. Their immutable owner/channel claims do not.
        assert!(matches!(
            prepare_restored(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &expected
            ),
            Err(Error::DiffDriveIdentity)
        ));
        assert_eq!(snapshot(&server), before);
        for (entity, metadata) in &configured {
            server
                .simulation
                .world_mut()
                .entity_mut(*entity)
                .insert(metadata.clone());
        }
    }
    for destination in [
        "/m59/left/cmd_vel",
        "/m59/left/odom",
        "/clock",
        "/world/default/control",
        "/world/default/pose/info",
    ] {
        let mut config = server.network_config.clone();
        config
            .add_topic_remap("/m59/right/cmd_vel", destination)
            .unwrap();
        let before = snapshot(&server);
        assert!(matches!(
            preflight(&server.simulation, &server.world_io, &config),
            Err(Error::DiffDriveIdentity)
        ));
        assert_eq!(snapshot(&server), before);
    }
}

#[test]
fn actual_receipt_uuid_rejects_foreign_same_key_and_duplicate_prepared_or_retained_arcs() {
    let (mut server, _observer) = build(2, 2);
    let owners = own_set(&server.simulation, &server.network_config, None)
        .unwrap()
        .owners;
    let ids: Vec<_> = owners.keys().copied().collect();
    let (_first, mut installed) = foreign(&server);
    let (_second, foreign) = foreign(&server);
    prepared(&owners, &installed, &server.network_config).unwrap();
    assert_ne!(installed.lease_ids[&ids[0]], foreign.lease_ids[&ids[0]]);
    assert!(!Arc::ptr_eq(
        &installed.outputs[&ids[0]],
        &foreign.outputs[&ids[0]]
    ));
    let stream = &server.diff_drive.as_ref().unwrap().streams[&ids[0]];
    assert!(foreign.outputs[&ids[0]].matches_lease(
        foreign.lease_ids[&ids[0]],
        &stream.command_key,
        &stream.odometry_key
    ));
    let original = installed
        .outputs
        .insert(ids[0], Arc::clone(&foreign.outputs[&ids[0]]))
        .unwrap();
    let expected = installed.lease_ids.clone();
    assert!(matches!(
        prepared(&owners, &installed, &server.network_config),
        Err(Error::DiffDriveIdentity)
    ));
    assert_eq!(installed.lease_ids, expected);
    installed.outputs.insert(ids[0], original);
    let original = installed
        .outputs
        .insert(ids[1], Arc::clone(&installed.outputs[&ids[0]]))
        .unwrap();
    assert!(matches!(
        prepared(&owners, &installed, &server.network_config),
        Err(Error::DiffDriveIdentity)
    ));
    installed.outputs.insert(ids[1], original);
    let saved_id = installed
        .lease_ids
        .insert(ids[0], uuid::Uuid::nil())
        .unwrap();
    assert!(matches!(
        prepared(&owners, &installed, &server.network_config),
        Err(Error::DiffDriveIdentity)
    ));
    installed.lease_ids.insert(ids[0], saved_id);
    let missing = installed.outputs.remove(&ids[1]).unwrap();
    assert!(matches!(
        prepared(&owners, &installed, &server.network_config),
        Err(Error::DiffDriveIdentity)
    ));
    installed.outputs.insert(ids[1], missing);
    let original = {
        let stream = server
            .diff_drive
            .as_mut()
            .unwrap()
            .streams
            .get_mut(&ids[0])
            .unwrap();
        std::mem::replace(&mut stream.output, Arc::clone(&foreign.outputs[&ids[0]]))
    };
    let before = snapshot(&server);
    assert!(matches!(
        prepare_reset(&server),
        Err(Error::DiffDriveIdentity)
    ));
    assert_eq!(snapshot(&server), before);
    server
        .diff_drive
        .as_mut()
        .unwrap()
        .streams
        .get_mut(&ids[0])
        .unwrap()
        .output = original;
    prepare_reset(&server).unwrap();
    let (queued, receiver) = mpsc::sync_channel(1);
    queued
        .send((server.simulation.epoch(), Some([0.25, 0.0])))
        .unwrap();
    server
        .diff_drive
        .as_mut()
        .unwrap()
        .streams
        .get_mut(&ids[1])
        .unwrap()
        .receiver = receiver;
    let stream = server
        .diff_drive
        .as_mut()
        .unwrap()
        .streams
        .get_mut(&ids[0])
        .unwrap();
    let saved = stream.lease_id;
    stream.output = Arc::clone(&foreign.outputs[&ids[0]]);
    assert_eq!(stream.lease_id, saved);
    let before = snapshot(&server);
    assert!(matches!(
        server.poll_diff_drive(),
        Err(Error::DiffDriveIdentity)
    ));
    assert!(server.reset_failed && server.diff_drive.is_none());
    let after = snapshot(&server);
    assert_eq!(
        (after.received, after.applied, after.rejected, after.queued),
        (
            before.received,
            before.applied,
            before.rejected,
            before.queued
        )
    );
    assert_eq!(after.native, before.native);
    assert_eq!(after.samples, before.samples);
}

#[test]
fn old_lease_pairs_are_checked_before_new_remap_and_removed_originals_restore() {
    let (mut server, _observer) = build(2, 2);
    pause(&mut server);
    let old: Vec<_> = server
        .diff_drive
        .as_ref()
        .unwrap()
        .streams
        .values()
        .map(|s| (s.identity, s.lease_id, Arc::downgrade(&s.output)))
        .collect();
    let removed_metadata = server.diff_drive.as_ref().unwrap().streams[&old[1].0]
        .metadata
        .clone();
    for side in ["left", "right"] {
        for channel in ["cmd_vel", "odom"] {
            server
                .network_config
                .add_topic_remap(
                    &format!("/m59/{side}/{channel}"),
                    &format!("/m59/new/{side}/{channel}"),
                )
                .unwrap();
        }
    }
    let before = snapshot(&server);
    assert_eq!(prepare_reset(&server).unwrap().len(), 2);
    assert_eq!(snapshot(&server), before);
    remove_model(&mut server.simulation, old[1].0).unwrap();
    server.retire_diff_drive();
    assert_eq!(server.diff_drive.as_ref().unwrap().streams.len(), 1);
    assert!(old[1].2.upgrade().is_none());
    let expected = prepare_reset(&server).unwrap();
    assert_eq!(expected.len(), 2);
    let survivor = server
        .diff_drive
        .as_ref()
        .unwrap()
        .streams
        .values()
        .next()
        .unwrap()
        .entity;
    let original = server
        .simulation
        .world()
        .get::<DiffDriveConfiguration>(survivor)
        .unwrap()
        .clone();
    for removed_topic in [&removed_metadata.topic, &removed_metadata.odom_topic] {
        let before = snapshot(&server);
        let mut changed = original.clone();
        changed.topic = removed_topic.clone();
        server
            .simulation
            .world_mut()
            .entity_mut(survivor)
            .insert(changed);
        assert!(matches!(
            prepare_restored(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &expected
            ),
            Err(Error::DiffDriveIdentity)
        ));
        assert_eq!(snapshot(&server), before);
        server
            .simulation
            .world_mut()
            .entity_mut(survivor)
            .insert(original.clone());
    }
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
    assert_eq!(info.sim_time, Duration::ZERO);
    assert_eq!(server.diff_drive.as_ref().unwrap().streams.len(), 2);
    for (id, previous, weak) in old {
        assert!(weak.upgrade().is_none());
        let current = &server.diff_drive.as_ref().unwrap().streams[&id];
        assert_ne!(current.lease_id, previous);
        assert!(
            current.command_key.contains("/m59/new/") && current.odometry_key.contains("/m59/new/")
        );
    }
}

#[test]
fn one_global_ingress_budget_is_fair_and_invalid_old_epochs_consume_it() {
    let (mut server, observer) = build(2, 1);
    pause(&mut server);
    let keys: Vec<_> = server
        .diff_drive
        .as_ref()
        .unwrap()
        .streams
        .values()
        .map(|s| s.command_key.clone())
        .collect();
    let publishers: Vec<_> = keys
        .iter()
        .map(|key| observer.declare_publisher(key.clone()).wait().unwrap())
        .collect();
    wait(|| {
        publishers
            .iter()
            .all(|p| p.matching_status().wait().unwrap().matching())
    });
    // Actual callbacks capture the deliberately old epoch; the native simulation is not stepped.
    server
        .world_io
        .epoch
        .store(u64::MAX, std::sync::atomic::Ordering::Release);
    for publisher in &publishers {
        publisher
            .put(twist(0.25, 0.0).encode_to_vec())
            .attachment(Twist::full_name())
            .wait()
            .unwrap();
    }
    wait(|| {
        server.poll_diff_drive().unwrap();
        server.counters.diff_drive_messages_received == 1
    });
    assert_eq!(server.counters.diff_drive_messages_rejected, 1);
    assert_eq!(server.counters.diff_drive_commands_applied, 0);
    wait(|| {
        server.poll_diff_drive().unwrap();
        server.counters.diff_drive_messages_received == 2
    });
    assert_eq!(server.counters.diff_drive_messages_rejected, 2);
    server.world_io.epoch.store(
        server.simulation.epoch(),
        std::sync::atomic::Ordering::Release,
    );
    for publisher in &publishers {
        publisher
            .put(vec![255])
            .attachment(Twist::full_name())
            .wait()
            .unwrap();
    }
    wait(|| {
        server.poll_diff_drive().unwrap();
        server.counters.diff_drive_messages_received == 3
    });
    assert_eq!(server.counters.diff_drive_messages_rejected, 3);
    wait(|| {
        server.poll_diff_drive().unwrap();
        server.counters.diff_drive_messages_received == 4
    });
    assert_eq!(server.counters.diff_drive_messages_rejected, 4);
    assert_eq!(server.counters.diff_drive_commands_applied, 0);
    for publisher in &publishers {
        publisher
            .put(twist(0.25, 0.0).encode_to_vec())
            .attachment(Twist::full_name())
            .wait()
            .unwrap();
    }
    wait(|| {
        server.poll_diff_drive().unwrap();
        server.counters.diff_drive_messages_received == 5
    });
    assert_eq!(server.counters.diff_drive_commands_applied, 1);
    wait(|| {
        server.poll_diff_drive().unwrap();
        server.counters.diff_drive_messages_received == 6
    });
    assert_eq!(server.counters.diff_drive_commands_applied, 2);
    assert_eq!(server.simulation.elapsed(), Duration::ZERO);
}

#[test]
fn alive_original_without_public_or_private_components_faults_even_after_direct_retirement() {
    for remove_private_too in [false, true] {
        let (mut server, _observer) = build(2, 2);
        pause(&mut server);
        let entity = server
            .diff_drive
            .as_ref()
            .unwrap()
            .streams
            .values()
            .last()
            .unwrap()
            .entity;
        let healthy = *server
            .diff_drive
            .as_ref()
            .unwrap()
            .streams
            .keys()
            .next()
            .unwrap();
        let (queued, receiver) = mpsc::sync_channel(1);
        queued
            .send((server.simulation.epoch(), Some([0.25, 0.0])))
            .unwrap();
        // A finite CPU handoff is already pending, independently of transport timing.
        server
            .diff_drive
            .as_mut()
            .unwrap()
            .streams
            .get_mut(&healthy)
            .unwrap()
            .receiver = receiver;
        let before = snapshot(&server);
        // Exercise missing public metadata separately from losing BOTH components.
        // The original entity and its identity/parent edges remain alive.
        if remove_private_too {
            server.simulation.world_mut().entity_mut(entity).retain::<(
                EntityId,
                EntityKind,
                Name,
                ChildOf,
                bevy_ecs::prelude::Children,
                DiffDriveSample,
                WorldPose,
                LocalPose,
                BodyHandle,
            )>();
        } else {
            server
                .simulation
                .world_mut()
                .entity_mut(entity)
                .remove::<DiffDriveConfiguration>();
        }
        assert!(server.simulation.world().get_entity(entity).is_ok());
        assert!(
            !server
                .simulation
                .world()
                .entity(entity)
                .contains::<DiffDriveConfiguration>()
        );
        server.retire_diff_drive();
        assert_eq!(server.diff_drive.as_ref().unwrap().streams.len(), 1);
        assert!(matches!(
            server.poll_diff_drive(),
            Err(Error::DiffDriveIdentity)
        ));
        assert!(server.reset_failed && server.diff_drive.is_none());
        let after = snapshot(&server);
        assert_eq!(
            (after.received, after.applied, after.rejected, after.queued),
            (
                before.received,
                before.applied,
                before.rejected,
                before.queued
            )
        );
        assert_eq!(after.native, before.native);
        assert_eq!(after.samples, before.samples);
    }
}

#[test]
fn coordinated_real_wheel_handle_and_child_swap_is_refused_before_io_and_poll() {
    let (mut server, _observer) = build(2, 2);
    let expected = prepare_reset(&server).unwrap();
    let mut joints: Vec<_> = server
        .simulation
        .world()
        .iter_entities()
        .filter(|node| {
            node.get::<Name>()
                .is_some_and(|n| n.0.starts_with("default::robot_right::"))
                && node.contains::<JointHandle>()
        })
        .map(|n| n.id())
        .collect();
    joints.sort();
    assert_eq!(joints.len(), 2);
    let a = *server
        .simulation
        .world()
        .get::<JointHandle>(joints[0])
        .unwrap();
    let b = *server
        .simulation
        .world()
        .get::<JointHandle>(joints[1])
        .unwrap();
    assert_ne!(a.id(), b.id());
    let mut da = server
        .simulation
        .world()
        .get::<JointDefinition>(joints[0])
        .unwrap()
        .clone();
    let mut db = server
        .simulation
        .world()
        .get::<JointDefinition>(joints[1])
        .unwrap()
        .clone();
    std::mem::swap(&mut da.0.child, &mut db.0.child);
    let before = snapshot(&server);
    server
        .simulation
        .world_mut()
        .entity_mut(joints[0])
        .insert((b, da));
    server
        .simulation
        .world_mut()
        .entity_mut(joints[1])
        .insert((a, db));
    let owner = server
        .diff_drive
        .as_ref()
        .unwrap()
        .streams
        .values()
        .find(|stream| stream.metadata.model_name == "robot_right")
        .unwrap()
        .entity;
    let mut metadata = server
        .simulation
        .world()
        .get::<DiffDriveConfiguration>(owner)
        .unwrap()
        .clone();
    metadata.frame_id.push_str("/changed");
    server
        .simulation
        .world_mut()
        .entity_mut(owner)
        .insert(metadata);
    assert!(matches!(
        prepare_restored(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &expected
        ),
        Err(Error::DiffDriveIdentity)
    ));
    assert!(matches!(
        preflight(&server.simulation, &server.world_io, &server.network_config),
        Err(Error::DiffDriveIdentity)
    ));
    assert!(matches!(
        prepare_reset(&server),
        Err(Error::DiffDriveIdentity)
    ));
    assert_eq!(snapshot(&server), before);
    assert!(matches!(
        server.poll_diff_drive(),
        Err(Error::DiffDriveIdentity)
    ));
    let after = snapshot(&server);
    assert_eq!(
        (after.received, after.applied, after.rejected, after.queued),
        (
            before.received,
            before.applied,
            before.rejected,
            before.queued
        )
    );
    assert_eq!(after.native, before.native);
    assert_eq!(after.samples, before.samples);
}

#[test]
fn whole_io_installation_rejects_owner_count_and_cross_channel_aliases_before_first_declare() {
    let (server, observer) = build(0, 2);
    let prefix = format!("@/m59-install-{}@", uuid::Uuid::new_v4());
    let key = |owner, channel| format!("{prefix}/{owner}/{channel}");
    let monitor = observer.declare_publisher(key(1, "cmd")).wait().unwrap();
    assert!(!monitor.matching_status().wait().unwrap().matching());
    for fault in 0..8 {
        let mut endpoints: Vec<_> = (1..=2)
            .map(|owner| {
                let (input, _receiver) = mpsc::sync_channel(1);
                Endpoint {
                    owner,
                    command_key: key(owner, "cmd"),
                    odometry_key: key(owner, "odom"),
                    input,
                }
            })
            .collect();
        match fault {
            0 => endpoints.clear(),
            1 => {
                for owner in 3..=9 {
                    let (input, _) = mpsc::sync_channel(1);
                    endpoints.push(Endpoint {
                        owner,
                        command_key: key(owner, "cmd"),
                        odometry_key: key(owner, "odom"),
                        input,
                    });
                }
            }
            2 => endpoints[1].owner = endpoints[0].owner,
            3 => endpoints[1].command_key = endpoints[0].odometry_key.clone(),
            4 => endpoints[1].odometry_key.clear(),
            5 => endpoints[1].odometry_key = format!("{prefix}/2/#"),
            6 => endpoints[1].odometry_key = "unqualified/odom".into(),
            _ => endpoints[1].odometry_key = format!("{prefix}/2/*"),
        }
        let before = snapshot(&server);
        assert!(
            crate::diff_drive_io::Declarations::new(
                &server.world_io.session,
                Setup {
                    endpoints,
                    max_bytes: 512,
                    drops: Arc::clone(&server.world_io.ingress_drops),
                    epoch: Arc::clone(&server.world_io.epoch),
                },
                "m59",
                "invalid-set"
            )
            .is_err()
        );
        assert!(!monitor.matching_status().wait().unwrap().matching());
        assert_eq!(snapshot(&server), before);
    }
}

#[test]
fn genuine_three_seven_eight_loaded_owners_and_ninth_refusal_are_atomic() {
    for count in [3, 7, 8] {
        let (mut server, _observer) = build_fleet(count, 3);
        pause(&mut server);
        let before = snapshot(&server);
        let metadata =
            preflight(&server.simulation, &server.world_io, &server.network_config).unwrap();
        assert_eq!(metadata.len(), count);
        assert_eq!(initial(&server.simulation).unwrap().len(), count);
        assert_eq!(originals(&server.simulation).unwrap().unwrap().len(), count);
        let streams = &server.diff_drive.as_ref().unwrap().streams;
        assert_eq!(streams.len(), count);
        let keys: BTreeSet<_> = streams
            .values()
            .flat_map(|s| [&s.command_key, &s.odometry_key])
            .collect();
        assert_eq!(keys.len(), 2 * count);
        assert_eq!(prepare_reset(&server).unwrap(), metadata);
        assert_eq!(snapshot(&server), before);
        let ids: Vec<_> = streams.keys().copied().collect();
        let last = streams[ids.last().unwrap()].entity;
        let original = metadata[ids.last().unwrap()].clone();
        for topic in [&metadata[&ids[0]].topic, &metadata[&ids[0]].odom_topic] {
            let mut changed = original.clone();
            changed.topic = topic.clone();
            server
                .simulation
                .world_mut()
                .entity_mut(last)
                .insert(changed);
            let before = snapshot(&server);
            assert!(matches!(
                preflight(&server.simulation, &server.world_io, &server.network_config),
                Err(Error::DiffDriveIdentity)
            ));
            assert!(matches!(
                prepare_reset(&server),
                Err(Error::DiffDriveIdentity)
            ));
            assert!(matches!(
                server
                    .diff_drive
                    .as_ref()
                    .unwrap()
                    .validate_live(&server.simulation, true),
                Err(Error::DiffDriveIdentity)
            ));
            assert_eq!(snapshot(&server), before);
            server
                .simulation
                .world_mut()
                .entity_mut(last)
                .insert(original.clone());
        }
        if count == 8 {
            let admission = own_set(&server.simulation, &server.network_config, None).unwrap();
            let (_registry, mut installed) = foreign(&server);
            let (_foreign_registry, foreign_installed) = foreign(&server);
            prepared(&admission.owners, &installed, &server.network_config).unwrap();
            let last_id = *ids.last().unwrap();
            let saved = installed
                .outputs
                .insert(last_id, Arc::clone(&foreign_installed.outputs[&last_id]))
                .unwrap();
            let before = snapshot(&server);
            assert!(matches!(
                prepared(&admission.owners, &installed, &server.network_config),
                Err(Error::DiffDriveIdentity)
            ));
            assert_eq!(snapshot(&server), before);
            installed.outputs.insert(last_id, saved);
            let saved = installed.lease_ids.remove(&last_id).unwrap();
            assert!(matches!(
                prepared(&admission.owners, &installed, &server.network_config),
                Err(Error::DiffDriveIdentity)
            ));
            assert_eq!(snapshot(&server), before);
            installed.lease_ids.insert(last_id, saved);
            prepared(&admission.owners, &installed, &server.network_config).unwrap();
            let mut counterfeit = original.clone();
            counterfeit.model_name = "not_an_original_owner".into();
            counterfeit.topic = "/m61/ninth/cmd_vel".into();
            counterfeit.odom_topic = "/m61/ninth/odom".into();
            let entity = server
                .simulation
                .world_mut()
                .spawn((EntityKind::Model, counterfeit))
                .id();
            gz_sim_core::assign_id(server.simulation.world_mut(), entity).unwrap();
            let before = snapshot(&server);
            assert!(matches!(
                preflight(&server.simulation, &server.world_io, &server.network_config),
                Err(Error::DiffDriveIdentity)
            ));
            assert!(matches!(
                prepare_reset(&server),
                Err(Error::DiffDriveIdentity)
            ));
            assert_eq!(snapshot(&server), before);
            server.simulation.world_mut().despawn(entity);
            assert_eq!(prepare_reset(&server).unwrap(), metadata);
        }
    }
    assert!(
        load_world_with_sdf_step(
            &fleet_xml(9),
            "m61-nine.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .is_err()
    );
}

#[test]
fn eight_owner_ingress_keeps_one_budget_and_round_robin_fairness() {
    let (mut server, _observer) = build_fleet(8, 3);
    pause(&mut server);
    let epoch = server.simulation.epoch();
    let native = snapshot(&server).native;
    let mut inputs = Vec::new();
    for stream in server.diff_drive.as_mut().unwrap().streams.values_mut() {
        let (input, receiver) = mpsc::sync_channel(3);
        input.send((epoch, None)).unwrap();
        input
            .send((epoch.wrapping_add(1), Some([0.25, 0.0])))
            .unwrap();
        input.send((epoch, Some([0.25, 0.0]))).unwrap();
        assert!(matches!(
            input.try_send((epoch, None)),
            Err(mpsc::TrySendError::Full(_))
        ));
        stream.receiver = receiver;
        inputs.push(input);
    }
    // Each round visits every owner once: malformed, stale, then valid. A busy queue
    // cannot consume the other seven owners' turns, and three is the whole budget.
    for batch in 1..=8_u64 {
        server.poll_diff_drive().unwrap();
        let consumed = 3 * batch;
        assert_eq!(server.counters.diff_drive_messages_received, consumed);
        assert_eq!(
            server.counters.diff_drive_messages_rejected,
            consumed.min(16)
        );
        assert_eq!(
            server.counters.diff_drive_commands_applied,
            consumed.saturating_sub(16)
        );
        assert_eq!(snapshot(&server).native, native);
        assert_eq!(server.simulation.elapsed(), Duration::ZERO);
    }
    assert_eq!(server.diff_drive.as_ref().unwrap().next_poll, 0);
    for input in &inputs {
        input.try_send((epoch, Some([0.0, 0.0]))).unwrap();
    }
    for batch in 1..=3_u64 {
        server.poll_diff_drive().unwrap();
        assert_eq!(
            server.counters.diff_drive_messages_received,
            24 + (3 * batch).min(8)
        );
    }
    assert_eq!(server.counters.diff_drive_commands_applied, 16);
    assert_eq!(server.counters.diff_drive_messages_rejected, 16);
}

#[test]
fn eight_owner_reset_and_retirement_preserve_all_original_channel_claims() {
    let (mut server, _observer) = build_fleet(8, 3);
    pause(&mut server);
    let old: Vec<_> = server
        .diff_drive
        .as_ref()
        .unwrap()
        .streams
        .values()
        .map(|s| {
            (
                s.identity,
                s.lease_id,
                Arc::downgrade(&s.output),
                s.metadata.clone(),
            )
        })
        .collect();
    for index in 0..8 {
        for channel in ["cmd_vel", "odom"] {
            server
                .network_config
                .add_topic_remap(
                    &format!("/m61/robot_{index}/{channel}"),
                    &format!("/m61/reset/robot_{index}/{channel}"),
                )
                .unwrap();
        }
    }
    let before = snapshot(&server);
    let expected = prepare_reset(&server).unwrap();
    assert_eq!(expected.len(), 8);
    assert_eq!(snapshot(&server), before);
    remove_model(&mut server.simulation, old[7].0).unwrap();
    server.retire_diff_drive();
    assert_eq!(server.diff_drive.as_ref().unwrap().streams.len(), 7);
    assert!(old[7].2.upgrade().is_none());
    assert_eq!(prepare_reset(&server).unwrap(), expected);
    let survivor = server.diff_drive.as_ref().unwrap().streams[&old[0].0].entity;
    let original = old[0].3.clone();
    for retired_topic in [&old[7].3.topic, &old[7].3.odom_topic] {
        let mut changed = original.clone();
        changed.topic = retired_topic.clone();
        server
            .simulation
            .world_mut()
            .entity_mut(survivor)
            .insert(changed);
        let before = snapshot(&server);
        assert!(matches!(
            prepare_restored(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &expected,
            ),
            Err(Error::DiffDriveIdentity)
        ));
        assert_eq!(snapshot(&server), before);
        server
            .simulation
            .world_mut()
            .entity_mut(survivor)
            .insert(original.clone());
    }
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.prepare_io().unwrap();
    assert_eq!(server.tick().unwrap().sim_time, Duration::ZERO);
    let streams = &server.diff_drive.as_ref().unwrap().streams;
    assert_eq!(streams.len(), 8);
    for (id, lease, weak, _) in old {
        assert!(weak.upgrade().is_none());
        assert_ne!(streams[&id].lease_id, lease);
        assert_eq!(streams[&id].sequence, 0);
        assert_eq!(streams[&id].last_sample, None);
        assert!(streams[&id].command_key.contains("/m61/reset/"));
        assert!(streams[&id].odometry_key.contains("/m61/reset/"));
    }
    assert_eq!(originals(&server.simulation).unwrap().unwrap().len(), 8);
}
