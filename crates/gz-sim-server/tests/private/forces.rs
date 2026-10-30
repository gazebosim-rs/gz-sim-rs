// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use sdformat::NoResources;

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
        workers: Vec::new(),
        templates: Vec::new(),
        expected: definitions(sim).into_iter().collect(),
        next: 0,
    }
}
#[test]
fn invalid_configuration_and_duplicate_identity_fail_before_allocating_workers() {
    assert!(matches!(
        Forces::prepare(
            &Config {
                capacity: 0,
                ..Config::default()
            },
            []
        ),
        Err(Error::Transport(TransportError::InvalidConfig))
    ));
    let sim = scene();
    let defs = definitions(&sim);
    assert!(matches!(
        Forces::prepare(&Config::default(), [defs[0].clone(), defs[0].clone()]),
        Err(Error::JointForceIdentity)
    ));
    assert!(matches!(
        Forces::prepare(
            &Config {
                partition: "invalid partition".into(),
                ..Config::default()
            },
            defs
        ),
        Err(Error::Transport(TransportError::InvalidName))
    ));
    let mut forces = Forces::prepare(&Config::default(), []).unwrap();
    assert!(forces.workers.is_empty());
    let mut counters = Counters::default();
    let mut empty = Simulation::new(Duration::from_millis(1)).unwrap();
    forces
        .poll(&mut empty, NonZeroUsize::new(1).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(counters.force_messages_received, 0);
    forces.restart(&empty).unwrap();
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

fn network() -> (Config, std::net::UdpSocket, std::net::UdpSocket) {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static ID: AtomicUsize = AtomicUsize::new(0);
    let message = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    let service = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    for socket in [&message, &service] {
        socket2::SockRef::from(socket)
            .set_reuse_address(true)
            .unwrap();
        #[cfg(target_os = "linux")]
        socket2::SockRef::from(socket)
            .set_multicast_all_v4(false)
            .unwrap();
    }
    (
        Config {
            partition: format!(
                "force-unit-{}-{}",
                std::process::id(),
                ID.fetch_add(1, Ordering::Relaxed)
            ),
            message_port: message.local_addr().unwrap().port(),
            service_port: service.local_addr().unwrap().port(),
            ..Config::default()
        },
        message,
        service,
    )
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
        let (mut config, _message, _service) = network();
        let topic = "/model/rotor/joint/hinge/cmd_force";
        if other == "other" {
            config
                .add_topic_remap("/model/other/joint/hinge/cmd_force", topic)
                .unwrap();
        }
        let mut sim = pair_scene(other);
        config.capacity = 1;
        let mut forces = Forces::prepare(&config, definitions(&sim)).unwrap();
        forces.bind(&sim).unwrap();
        assert_eq!(forces.workers.len(), 1);
        assert_eq!(forces.streams.len(), 1);
        assert_eq!(forces.streams[0].ids.len(), 2);
        let mut sender = Node::new(Config {
            capacity: 16,
            ..config
        })
        .unwrap();
        let publisher = sender.advertise(topic, "gz.msgs.Double").unwrap();
        publisher
            .publish(
                Double {
                    data: 0.001,
                    ..Default::default()
                }
                .encode_to_vec(),
            )
            .unwrap();
        // A query queued after publication fences the sender's local routing.
        sender.known_publisher_topics().unwrap();
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
        publisher.publish(vec![0xff]).unwrap();
        sender.known_publisher_topics().unwrap();
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
        publisher
            .publish(
                Double {
                    data: 0.002,
                    ..Default::default()
                }
                .encode_to_vec(),
            )
            .unwrap();
        sender.known_publisher_topics().unwrap();
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
        sender.shutdown();
    }
}

#[test]
fn failed_restart_clears_leases_and_shutdown_poll_is_an_error() {
    let (config, _message, _service) = network();
    let mut sim = scene();
    let mut forces = Forces::prepare(&config, definitions(&sim)).unwrap();
    forces.workers[0].shutdown();
    assert!(matches!(
        forces.poll(
            &mut sim,
            NonZeroUsize::new(1).unwrap(),
            &mut Counters::default()
        ),
        Err(Error::Transport(TransportError::Shutdown))
    ));
    assert!(matches!(
        forces.restart(&sim),
        Err(Error::Transport(TransportError::Shutdown))
    ));
    assert!(forces.streams.is_empty());
    let mut forces = Forces::prepare(&config, definitions(&sim)).unwrap();
    let id = definitions(&sim)[0].0;
    let joint = entity_by_id(sim.world(), id).unwrap();
    sim.world_mut().entity_mut(joint).remove::<JointHandle>();
    assert!(matches!(
        forces.restart(&sim),
        Err(Error::JointForceIdentity)
    ));
    assert!(forces.streams.is_empty());
    forces.shutdown();
}

#[test]
fn finite_budget_rotates_between_ready_endpoints_and_workers() {
    for capacity in [1, 2] {
        let (mut config, _message, _service) = network();
        config.capacity = capacity;
        let mut sim = pair_scene("other");
        let mut forces = Forces::prepare(&config, definitions(&sim)).unwrap();
        forces.bind(&sim).unwrap();
        assert_eq!(forces.workers.len(), 2usize.div_ceil(capacity));
        assert_eq!(forces.streams.len(), 2);
        let ids: Vec<_> = forces.streams.iter().map(|s| s.ids[0]).collect();
        let mut sender = Node::new(Config {
            capacity: 16,
            ..config
        })
        .unwrap();
        let publishers: Vec<_> = forces
            .templates
            .iter()
            .map(|t| sender.advertise(&t.topic, "gz.msgs.Double").unwrap())
            .collect();
        for (index, publisher) in publishers.iter().enumerate() {
            for value in 1..=capacity {
                publisher
                    .publish(
                        Double {
                            data: (index * 10 + value) as f64,
                            ..Default::default()
                        }
                        .encode_to_vec(),
                    )
                    .unwrap();
            }
        }
        sender.known_publisher_topics().unwrap();
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
        sender.shutdown();
    }
}

#[test]
fn force_worker_failure_during_committed_server_reset_is_terminal() {
    use gz_msgs::msgs::{WorldControl, WorldReset};
    let (config, _message, _service) = network();
    let mut server =
        crate::Server::new(scene(), "default", config, NonZeroUsize::new(2).unwrap()).unwrap();
    server.forces.workers[0].cancel();
    assert!(server.apply_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(
        server.tick(),
        Err(Error::Transport(TransportError::Shutdown))
    ));
    assert_eq!(
        server
            .simulation
            .world()
            .resource::<gz_sim_core::UpdateInfo>()
            .epoch,
        1
    );
    assert!(server.forces.streams.is_empty());
    assert!(server.tick().is_err());
    server.shutdown();
}
