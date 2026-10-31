// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::{
    msgs::Double,
    prost::{Message, Name},
};
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use sdformat::NoResources;
use std::time::Duration;
use zenoh::{Wait, sample::Sample};

fn scene() -> Simulation {
    let xml = include_str!("../fixtures/joint_force.sdf").replace(
        "</model>",
        "<plugin filename='gz-sim-apply-joint-force-system' name='gz::sim::systems::ApplyJointForce'><joint_name>hinge</joint_name></plugin></model>",
    );
    load_world_with_sdf_step(
        &xml,
        "force.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn definitions(sim: &Simulation) -> Vec<(u64, JointForceConfiguration)> {
    gz_sim_scene::initial_joint_force_configurations(sim)
        .unwrap()
        .map(|(id, cfg)| (id, cfg.clone()))
        .collect()
}
fn detached(sim: &Simulation) -> Forces {
    Forces {
        streams: Vec::new(),
        expected: definitions(sim).into_iter().collect(),
        next: 0,
        drops: Arc::new(AtomicU64::new(0)),
    }
}
fn prepare(
    io: &crate::world_io::WorldIo,
    config: &ServerOptions,
    configurations: impl IntoIterator<Item = (u64, JointForceConfiguration)>,
) -> Result<Forces, Error> {
    let (mut forces, setup) = Forces::prepare(config, configurations, &io.ingress_drops)?;
    if let Err(error) = io.install_forces(setup) {
        forces.clear();
        return Err(error);
    }
    Ok(forces)
}
fn restart(
    forces: &mut Forces,
    io: &crate::world_io::WorldIo,
    config: &ServerOptions,
    sim: &Simulation,
) -> Result<(), Error> {
    let setup = forces.restart(config, sim, &io.ingress_drops)?;
    if let Err(error) = io.install_forces(setup) {
        forces.clear();
        return Err(error);
    }
    Ok(())
}
fn local_io() -> crate::world_io::WorldIo {
    let prefix = format!("@/force-owner-{}@", uuid::Uuid::new_v4());
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"peer\"").unwrap();
    config.insert_json5("listen/endpoints", "[]").unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .unwrap();
    crate::world_io::WorldIo::new_with_config(
        format!("{prefix}/clock"),
        format!("{prefix}/control"),
        format!("{prefix}/poses"),
        64,
        16 * 1024 * 1024,
        config,
    )
    .unwrap()
}
fn config() -> ServerOptions {
    ServerOptions {
        partition: format!("force-unit-{}", uuid::Uuid::new_v4()),
        ..ServerOptions::default()
    }
}
fn local_session() -> zenoh::Session {
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"peer\"").unwrap();
    config.insert_json5("listen/endpoints", "[]").unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .unwrap();
    zenoh::open(config).wait().unwrap()
}
fn put(session: &zenoh::Session, config: &ServerOptions, topic: &str, bytes: Vec<u8>) {
    let key = crate::options::qualified_name(
        &config.partition,
        &config.namespace,
        config.topic_remap(topic).unwrap_or(topic),
    )
    .unwrap();
    session
        .put(key, bytes)
        .attachment(Double::full_name())
        .wait()
        .unwrap();
}
#[test]
fn invalid_configuration_and_duplicate_identity_fail_before_declaring_endpoints() {
    let io = local_io();
    for (capacity, max_message_bytes) in [(0, 100), (1, 0), (usize::MAX, 100), (1, usize::MAX)] {
        assert!(matches!(
            prepare(
                &io,
                &ServerOptions {
                    capacity,
                    max_message_bytes,
                    ..config()
                },
                []
            ),
            Err(Error::Setup(SetupError::InvalidConfig))
        ));
    }
    let sim = scene();
    let defs = definitions(&sim);
    assert!(matches!(
        prepare(&io, &config(), [defs[0].clone(), defs[0].clone()]),
        Err(Error::JointForceIdentity)
    ));
    assert!(matches!(
        prepare(
            &io,
            &ServerOptions {
                partition: "invalid partition".into(),
                ..config()
            },
            defs
        ),
        Err(Error::Setup(SetupError::InvalidName))
    ));
    let config = config();
    let world_drops = Arc::new(AtomicU64::new(0));
    let (mut forces, setup) = Forces::prepare(&config, [], &world_drops).unwrap();
    assert!(setup.is_empty());
    io.install_forces(setup).unwrap();
    assert!(Arc::ptr_eq(&forces.drops, &world_drops));
    assert_eq!(Arc::strong_count(&world_drops), 2);
    assert!(forces.streams.is_empty() && forces.expected.is_empty());
    assert_eq!(forces.streams.capacity(), 0);
    let mut counters = Counters::default();
    let mut empty = Simulation::new(Duration::from_millis(1)).unwrap();
    forces
        .poll(&mut empty, NonZeroUsize::new(1).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(counters.force_messages_received, 0);
    restart(&mut forces, &io, &config, &empty).unwrap();
    forces.shutdown();
}
#[test]
fn binding_rejects_corrupted_controller_identity_and_components() {
    for corruption in 0..6 {
        let mut sim = scene();
        let mut forces = detached(&sim);
        forces.bind(&sim).unwrap();
        let (id, cfg) = definitions(&sim).remove(0);
        let joint = entity_by_id(sim.world(), id).unwrap();
        match corruption {
            0 => {
                sim.world_mut().entity_mut(joint).remove::<EntityId>();
            }
            1 => {
                sim.world_mut().entity_mut(joint).insert(EntityKind::Model);
            }
            2 => {
                sim.world_mut().entity_mut(joint).remove::<JointHandle>();
            }
            3 => {
                sim.world_mut()
                    .entity_mut(joint)
                    .remove::<JointForceCommand>();
            }
            4 => {
                forces.expected.clear();
            }
            _ => {
                // Moving a public ID does not transfer its indexed ownership.
                let identity = sim
                    .world_mut()
                    .entity_mut(joint)
                    .take::<EntityId>()
                    .unwrap();
                sim.world_mut()
                    .entity_mut(joint)
                    .remove::<JointForceConfiguration>();
                let handle = *sim.world().get::<JointHandle>(joint).unwrap();
                sim.world_mut().spawn((
                    identity,
                    cfg,
                    handle,
                    EntityKind::Joint,
                    JointForceCommand::default(),
                ));
            }
        }
        assert!(
            matches!(forces.bind(&sim), Err(Error::JointForceIdentity)),
            "corruption {corruption}"
        );
        assert!(
            Error::JointForceIdentity
                .to_string()
                .contains("original identity")
        );
    }
}

fn pair_scene(other: &str) -> Simulation {
    let source = include_str!("../fixtures/joint_force.sdf");
    let first = &source[source.find("<model ").unwrap()..source.find("</model>").unwrap() + 8];
    let second = first.replacen("name='rotor'", &format!("name='{other}'"), 1);
    let source = source.replace("</world>", &format!("{second}</world>"))
        .replace("</model>", "<plugin filename='gz-sim-apply-joint-force-system' name='gz::sim::systems::ApplyJointForce'><joint_name>hinge</joint_name></plugin></model>");
    load_world_with_sdf_step(
        &source,
        "pair.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}

#[test]
fn normalized_and_remapped_endpoint_collisions_fan_out_one_message_to_both_joints() {
    for other in ["rotor~", "other"] {
        let io = local_io();
        let session = &io.session;
        let mut config = config();
        let topic = "/model/rotor/joint/hinge/cmd_force";
        if other == "other" {
            config
                .add_topic_remap("/model/other/joint/hinge/cmd_force", topic)
                .unwrap();
        }
        let mut sim = pair_scene(other);
        config.capacity = 1;
        let mut forces = prepare(&io, &config, definitions(&sim)).unwrap();
        forces.bind(&sim).unwrap();
        assert_eq!(forces.streams.len(), 1);
        assert_eq!(forces.streams[0].ids.len(), 2);
        put(
            session,
            &config,
            topic,
            Double {
                data: 0.001,
                ..Default::default()
            }
            .encode_to_vec(),
        );
        let mut counters = Counters::default();
        forces
            .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
            .unwrap();
        assert_eq!(counters.force_messages_received, 1);
        assert_eq!(counters.joint_forces_applied, 2);
        assert_eq!(counters.force_messages_rejected, 0);
        for (id, _) in definitions(&sim) {
            let joint = entity_by_id(sim.world(), id).unwrap();
            assert_eq!(
                sim.world()
                    .get::<JointForceCommand>(joint)
                    .unwrap()
                    .torque(),
                0.001
            );
        }
        put(session, &config, topic, vec![0xff]);
        forces
            .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
            .unwrap();
        assert_eq!(counters.force_messages_received, 2);
        assert_eq!(counters.force_messages_rejected, 1);
        assert_eq!(counters.joint_forces_applied, 2);
        for (id, _) in definitions(&sim) {
            let joint = entity_by_id(sim.world(), id).unwrap();
            assert_eq!(
                sim.world()
                    .get::<JointForceCommand>(joint)
                    .unwrap()
                    .torque(),
                0.001
            );
        }
        let ids: Vec<_> = definitions(&sim).into_iter().map(|(id, _)| id).collect();
        let invalid = entity_by_id(sim.world(), ids[0]).unwrap();
        sim.world_mut().entity_mut(invalid).remove::<JointHandle>();
        put(
            session,
            &config,
            topic,
            Double {
                data: 0.002,
                ..Default::default()
            }
            .encode_to_vec(),
        );
        forces
            .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
            .unwrap();
        assert_eq!(counters.force_messages_received, 3);
        assert_eq!(counters.force_messages_rejected, 2);
        assert_eq!(counters.joint_forces_applied, 3);
        assert_eq!(
            sim.world()
                .get::<JointForceCommand>(invalid)
                .unwrap()
                .torque(),
            0.001
        );
        let valid = entity_by_id(sim.world(), ids[1]).unwrap();
        assert_eq!(
            sim.world()
                .get::<JointForceCommand>(valid)
                .unwrap()
                .torque(),
            0.002
        );
        forces.shutdown();
    }
}

#[test]
fn failed_restart_clears_leases_and_closed_session_poll_is_an_error() {
    // CPU polling observes a closed scalar ingress. Actual IO shutdown and setup
    // receipts, not a Session handle in Forces, establish the transport failure.
    let mut io = local_io();
    let config = config();
    let mut sim = scene();
    let mut forces = prepare(&io, &config, definitions(&sim)).unwrap();
    io.shutdown();
    assert!(matches!(
        forces.poll(
            &mut sim,
            NonZeroUsize::new(1).unwrap(),
            &mut Counters::default()
        ),
        Err(Error::WorldClosed)
    ));
    assert!(matches!(
        restart(&mut forces, &io, &config, &sim),
        Err(Error::WorldClosed)
    ));
    assert!(forces.streams.is_empty() && forces.expected.is_empty());
    let io = local_io();
    let mut forces = prepare(&io, &config, definitions(&sim)).unwrap();
    let id = definitions(&sim)[0].0;
    let joint = entity_by_id(sim.world(), id).unwrap();
    sim.world_mut().entity_mut(joint).remove::<JointHandle>();
    assert!(matches!(
        restart(&mut forces, &io, &config, &sim),
        Err(Error::JointForceIdentity)
    ));
    assert!(forces.streams.is_empty() && forces.expected.is_empty());
    forces.shutdown();
}

#[test]
fn finite_budget_rotates_between_ready_bounded_endpoints() {
    // Worker grouping is an explicit D28 retirement. Endpoint count, bounded
    // queued work, rotation, values and exact per-message budget remain asserted.
    for capacity in [1, 2] {
        let io = local_io();
        let session = &io.session;
        let mut config = config();
        config.capacity = capacity;
        let mut sim = pair_scene("other");
        let mut forces = prepare(&io, &config, definitions(&sim)).unwrap();
        forces.bind(&sim).unwrap();
        assert_eq!(forces.streams.len(), 2);
        let ids: Vec<_> = forces.streams.iter().map(|s| s.ids[0]).collect();
        let keys: Vec<_> = [
            "/model/other/joint/hinge/cmd_force",
            "/model/rotor/joint/hinge/cmd_force",
        ]
        .into_iter()
        .map(|topic| {
            crate::options::qualified_name(&config.partition, &config.namespace, topic).unwrap()
        })
        .collect();
        for (index, key) in keys.iter().enumerate() {
            for value in 1..=capacity {
                session
                    .put(
                        key.clone(),
                        Double {
                            data: (index * 10 + value) as f64,
                            ..Default::default()
                        }
                        .encode_to_vec(),
                    )
                    .attachment(Double::full_name())
                    .wait()
                    .unwrap();
            }
        }
        assert_eq!(forces.queue_drops(), 0);
        let mut counters = Counters::default();
        for step in 0..capacity * 2 {
            forces
                .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
                .unwrap();
            assert_eq!(counters.force_messages_received, (step + 1) as u64);
            assert_eq!(counters.joint_forces_applied, (step + 1) as u64);
            assert_eq!(counters.force_messages_rejected, 0);
            for (index, id) in ids.iter().enumerate() {
                let delivered = (step + 2 - index) / 2;
                let expected = if delivered == 0 {
                    0.0
                } else {
                    (index * 10 + delivered) as f64
                };
                let entity = entity_by_id(sim.world(), *id).unwrap();
                assert_eq!(
                    sim.world()
                        .get::<JointForceCommand>(entity)
                        .unwrap()
                        .torque(),
                    expected
                );
            }
        }
        forces
            .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
            .unwrap();
        assert_eq!(counters.force_messages_received, (capacity * 2) as u64);
        forces.shutdown();
    }
}

#[test]
fn malformed_type_oversize_nonfinite_and_zero_keep_bounded_command_semantics() {
    let io = local_io();
    let session = &io.session;
    let config = ServerOptions {
        capacity: 1,
        max_message_bytes: 32,
        ..config()
    };
    let mut sim = scene();
    let topic = "/model/rotor/joint/hinge/cmd_force";
    let mut forces = prepare(&io, &config, definitions(&sim)).unwrap();
    forces.bind(&sim).unwrap();
    let mut counters = Counters::default();
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -0.0] {
        put(
            session,
            &config,
            topic,
            Double {
                data: value,
                ..Default::default()
            }
            .encode_to_vec(),
        );
        forces
            .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
            .unwrap();
    }
    assert_eq!(counters.force_messages_received, 5);
    assert_eq!(counters.force_messages_rejected, 3);
    assert_eq!(counters.joint_forces_applied, 2);
    let key = crate::options::qualified_name(&config.partition, &config.namespace, topic).unwrap();
    for (bytes, attachment) in [
        (vec![0; 64], Double::full_name()),
        (Vec::new(), "gz.msgs.Float".into()),
    ] {
        session
            .put(key.clone(), bytes)
            .attachment(attachment)
            .wait()
            .unwrap();
        forces
            .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
            .unwrap();
    }
    assert_eq!(counters.force_messages_received, 7);
    assert_eq!(counters.force_messages_rejected, 5);
    for _ in 0..3 {
        put(session, &config, topic, Vec::new());
    }
    assert_eq!(forces.queue_drops(), 2);
    forces.clear();
    assert!(forces.streams.is_empty() && forces.expected.is_empty());
    assert_eq!(forces.streams.capacity(), 0);
    assert_eq!(forces.queue_drops(), 2);
    forces
        .poll(&mut sim, NonZeroUsize::new(2).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(counters.force_messages_received, 7);
    restart(&mut forces, &io, &config, &sim).unwrap();
    for _ in 0..3 {
        put(session, &config, topic, Vec::new());
    }
    assert_eq!(forces.queue_drops(), 4);
    restart(&mut forces, &io, &config, &sim).unwrap();
    assert_eq!(forces.queue_drops(), 4);
}

#[test]
fn last_consumer_releases_endpoints_and_loaded_scene_reconstructs_demand() {
    let io = local_io();
    let session = &io.session;
    let config = config();
    let mut sim = pair_scene("other");
    let presence_count = || {
        session
            .liveliness()
            .get(format!("@gz/%{}/**", config.partition))
            .timeout(Duration::from_millis(100))
            .wait()
            .unwrap()
            .into_iter()
            .fold(0, |count, reply| {
                reply.into_result().unwrap();
                count + 1
            })
    };
    let assert_presence = |expected| {
        let deadline = std::time::Instant::now() + Duration::from_secs(3);
        loop {
            let actual = presence_count();
            if actual == expected {
                break;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "MS presence {actual}, expected {expected}"
            );
            std::thread::sleep(Duration::from_millis(1));
        }
    };
    let mut forces = prepare(&io, &config, definitions(&sim)).unwrap();
    forces.bind(&sim).unwrap();
    let models: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::Model))
        .map(|e| e.get::<EntityId>().unwrap().get())
        .collect();
    assert_eq!(models.len(), 2);
    assert_eq!(forces.streams.len(), 2);
    assert_presence(2);
    gz_sim_scene::remove_model(&mut sim, models[0]).unwrap();
    forces.retire(&sim);
    assert_eq!(forces.streams.len(), 1);
    assert_eq!(forces.expected.len(), 1);
    assert_presence(1);
    gz_sim_scene::remove_model(&mut sim, models[1]).unwrap();
    forces.retire(&sim);
    assert!(forces.streams.is_empty() && forces.expected.is_empty());
    assert_eq!(forces.streams.capacity(), 0);
    assert_presence(0);
    restart(&mut forces, &io, &config, &sim).unwrap();
    assert!(forces.streams.is_empty() && forces.expected.is_empty());
    assert_eq!(forces.streams.capacity(), 0);
    assert_presence(0);
    gz_sim_scene::reset_loaded_world(&mut sim, |_| ()).unwrap();
    restart(&mut forces, &io, &config, &sim).unwrap();
    assert_eq!(forces.streams.len(), 2);
    assert_eq!(forces.expected.len(), 2);
    assert_presence(2);
    forces.shutdown();
}

fn isolate_server(name: &str) -> bool {
    const CHILD: &str = "GZ_FORCE_RESET_CHILD";
    if std::env::var(CHILD).as_deref() == Ok(name) {
        return false;
    }
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("zenoh.json");
    std::fs::write(&path, r#"{"mode":"peer","listen":{"endpoints":[]},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}}}"#).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args(["--exact", &format!("forces::tests::{name}"), "--nocapture"])
        .env(CHILD, name)
        .env("ZENOH_CONFIG", path)
        .output()
        .unwrap();
    assert!(output.status.success(), "{output:?}");
    assert!(
        String::from_utf8(output.stdout)
            .unwrap()
            .contains("1 passed")
    );
    true
}
#[test]
fn force_session_failure_during_committed_server_reset_is_terminal() {
    if isolate_server("force_session_failure_during_committed_server_reset_is_terminal") {
        return;
    }
    use gz_sim_proto::msgs::{WorldControl, WorldReset};
    let mut server =
        crate::Server::new(scene(), "default", config(), NonZeroUsize::new(2).unwrap()).unwrap();
    // A closed actual Session replaces the retired worker-cancel mechanic.
    server.world_io.session.close().wait().unwrap();
    assert!(server.apply_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(server.update(), Err(Error::WorldClosed)));
    assert_eq!(
        server
            .simulation
            .world()
            .resource::<gz_sim_core::UpdateInfo>()
            .epoch,
        1
    );
    assert!(server.forces.streams.is_empty());
    assert!(server.update().is_err());
    server.shutdown();
}

#[test]
fn in_flight_overflow_remains_counted_after_retirement_and_restart() {
    for clear in [false, true] {
        let session = local_session();
        let key = format!("force-race/{}", uuid::Uuid::new_v4());
        let (sender, receiver) = mpsc::sync_channel(1);
        sender.try_send(None).unwrap();
        let drops = Arc::new(AtomicU64::new(0));
        let overflow = Arc::clone(&drops);
        let active = Arc::new(AtomicBool::new(true));
        let release = Arc::new((std::sync::Mutex::new(false), std::sync::Condvar::new()));
        let callback_release = Arc::clone(&release);
        let (started, ready) = mpsc::sync_channel(1);
        let (installed, prepared) = mpsc::sync_channel(1);
        let (finish, finished) = mpsc::sync_channel(1);
        let io_session = session.clone();
        let io_key = key.clone();
        let io = std::thread::spawn(move || {
            let _subscriber = io_session
                .declare_subscriber(io_key)
                .callback(move |_: Sample| {
                    assert!(matches!(
                        sender.try_send(None),
                        Err(mpsc::TrySendError::Full(_))
                    ));
                    started.send(()).unwrap();
                    let (lock, condition) = &*callback_release;
                    drop(
                        condition
                            .wait_while(lock.lock().unwrap(), |released| !*released)
                            .unwrap(),
                    );
                    overflow.fetch_add(1, Ordering::Relaxed);
                })
                .wait()
                .unwrap();
            let _presence = io_session
                .liveliness()
                .declare_token(format!("force-race/presence/{}", uuid::Uuid::new_v4()))
                .wait()
                .unwrap();
            installed.send(()).unwrap();
            finished.recv().unwrap();
        });
        prepared.recv_timeout(Duration::from_secs(3)).unwrap();
        let mut forces = Forces {
            streams: vec![Stream {
                ids: vec![1],
                receiver,
                active,
            }],
            expected: BTreeMap::new(),
            next: 0,
            drops: Arc::clone(&drops),
        };
        let writer = session.clone();
        let putter = std::thread::spawn(move || writer.put(key, vec![1u8]).wait().unwrap());
        ready.recv_timeout(Duration::from_secs(3)).unwrap();
        let (retired, result) = mpsc::sync_channel(1);
        let retired_drops = Arc::clone(&drops);
        let retirer = std::thread::spawn(move || {
            let empty = Simulation::new(Duration::from_millis(1)).unwrap();
            if clear {
                forces.clear();
            } else {
                forces.retire(&empty);
            }
            assert!(forces.streams.is_empty() && forces.expected.is_empty());
            assert_eq!(forces.streams.capacity(), 0);
            assert!(
                forces
                    .restart(&config(), &empty, &retired_drops)
                    .unwrap()
                    .is_empty()
            );
            retired.send(forces).unwrap();
        });
        let retired_while_callback_running = result.recv_timeout(Duration::from_secs(2));
        *release.0.lock().unwrap() = true;
        release.1.notify_all();
        putter.join().unwrap();
        retirer.join().unwrap();
        finish.send(()).unwrap();
        io.join().unwrap();
        let forces = retired_while_callback_running
            .expect("CPU retirement waited for an in-flight callback");
        assert!(Arc::ptr_eq(&forces.drops, &drops));
        assert_eq!(drops.load(Ordering::Relaxed), 1);
        assert_eq!(forces.queue_drops(), 1);
    }
}

#[test]
fn raw_tick_refuses_unprepared_reset_and_consumes_prepared_snapshot_without_advancing() {
    if isolate_server(
        "raw_tick_refuses_unprepared_reset_and_consumes_prepared_snapshot_without_advancing",
    ) {
        return;
    }
    use gz_sim_proto::msgs::{WorldControl, WorldReset};
    use gz_sim_scene::JointSample;
    let mut server =
        crate::Server::new(scene(), "default", config(), NonZeroUsize::new(2).unwrap()).unwrap();
    let (id, _) = definitions(server.simulation()).remove(0);
    let joint = entity_by_id(server.simulation().world(), id).unwrap();
    set_joint_force(&mut server.simulation, id, 0.001).unwrap();
    server.set_paused(false);
    server.update().unwrap();
    assert!(server.apply_control(WorldControl {
        pause: false,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    let before = *server
        .simulation
        .world()
        .resource::<gz_sim_core::UpdateInfo>();
    let counters = format!("{:?}", server.counters());
    let velocity = server
        .simulation
        .world()
        .get::<JointSample>(joint)
        .unwrap()
        .state
        .velocity;
    assert!(matches!(server.tick(), Err(Error::IoPreparationRequired)));
    assert_eq!(
        *server
            .simulation
            .world()
            .resource::<gz_sim_core::UpdateInfo>(),
        before
    );
    assert_eq!(format!("{:?}", server.counters()), counters);
    assert_eq!(
        server
            .simulation
            .world()
            .get::<JointSample>(joint)
            .unwrap()
            .state
            .velocity,
        velocity
    );
    assert_eq!(
        server
            .simulation
            .world()
            .get::<JointForceCommand>(joint)
            .unwrap()
            .torque(),
        0.001
    );
    server.prepare_io().unwrap();
    let prepared = *server
        .simulation
        .world()
        .resource::<gz_sim_core::UpdateInfo>();
    assert_eq!(prepared.epoch, before.epoch + 1);
    assert_eq!(
        (prepared.sim_time, prepared.dt, prepared.iteration),
        (Duration::ZERO, Duration::ZERO, 0)
    );
    assert!(!prepared.paused);
    server.prepare_io().unwrap();
    assert_eq!(
        *server
            .simulation
            .world()
            .resource::<gz_sim_core::UpdateInfo>(),
        prepared
    );
    assert_eq!(server.tick().unwrap(), prepared);
    assert_eq!(
        *server
            .simulation
            .world()
            .resource::<gz_sim_core::UpdateInfo>(),
        prepared
    );
    let restored = entity_by_id(server.simulation.world(), id).unwrap();
    assert_eq!(
        server
            .simulation
            .world()
            .get::<JointForceCommand>(restored)
            .unwrap()
            .torque(),
        0.0
    );
    assert_eq!(
        server
            .simulation
            .world()
            .get::<JointSample>(restored)
            .unwrap()
            .state
            .velocity,
        0.0
    );
    let next = server.tick().unwrap();
    assert_eq!(next.epoch, prepared.epoch);
    assert_eq!(next.iteration, 1);
    assert_eq!(next.sim_time, Duration::from_millis(1));
    server.shutdown();
}
