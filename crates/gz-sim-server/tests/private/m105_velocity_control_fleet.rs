// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::{
    msgs::{Twist, WorldControl, WorldReset},
    prost::{Message, Name as ProtoName},
};
use gz_sim_scene::{PhysicsEngine, Velocity, load_world_with_sdf_step};
use std::time::Instant;
use zenoh::Wait;
fn source(count: usize) -> String {
    let mut models = String::new();
    for i in 0..count {
        models += &format!(
            "<model name='r{i}'><pose>{} 0 10 0 0 0</pose><link name='body'><inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>2</iyy><izz>2</izz></inertia></inertial><collision name='c'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link><plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'><topic>/m105/{i}/cmd</topic></plugin></model>",
            20 * i
        );
    }
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}
fn load(xml: &str, engine: PhysicsEngine) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m105-server",
        "default",
        &sdformat::NoResources,
        engine,
    )
    .unwrap()
}
fn network() -> zenoh::Config {
    zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}}}"#).unwrap()
}
fn build(count: usize, engine: PhysicsEngine, budget: usize) -> Server {
    Server::with_zenoh_config(
        load(&source(count), engine),
        "default",
        ServerOptions {
            partition: format!("m105-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        },
        network(),
        NonZeroUsize::new(budget).unwrap(),
    )
    .unwrap()
}
fn linear(server: &Server, id: u64) -> [f64; 3] {
    let e = entity_by_id(server.simulation.world(), id).unwrap();
    let c = server
        .simulation
        .world()
        .get::<VelocityControlConfiguration>(e)
        .unwrap();
    server
        .simulation
        .world()
        .get::<Velocity>(entity_by_id(server.simulation.world(), c.link).unwrap())
        .unwrap()
        .linear
}
fn near(a: [f64; 3], b: [f64; 3]) {
    for (a, b) in a.into_iter().zip(b) {
        assert!(a.is_finite() && (a - b).abs() < 1e-11, "{a} != {b}");
    }
}
#[test]
fn m105_fair_global_budget_rotates_across_eight_busy_owner_queues() {
    let mut server = build(8, PhysicsEngine::Rapier, 1);
    let epoch = server.simulation.epoch();
    let first = server.velocity_control.as_mut().unwrap();
    let mut ids = Vec::new();
    let mut senders = Vec::new();
    let mut queue = |c: &mut Controller, i: usize| {
        let (tx, rx) = mpsc::sync_channel(4);
        c.receiver = rx;
        ids.push(c.original.id);
        for _ in 0..4 {
            tx.send((epoch, Some(([(i + 1) as f64, 0., 0.], [0.; 3]))))
                .unwrap();
        }
        senders.push(tx);
    };
    queue(first, 0);
    for (i, c) in first.others.iter_mut().enumerate() {
        queue(c, i + 1);
    }
    for _ in 0..8 {
        server.poll_velocity_control().unwrap();
    }
    assert_eq!(server.counters.velocity_control_messages_received, 8);
    assert_eq!(server.counters.velocity_control_commands_pending, 8);
    server.simulation.step_exact(2).unwrap();
    for (i, id) in ids.into_iter().enumerate() {
        near(linear(&server, id), [(i + 1) as f64, 0., 0.]);
    }
    drop(senders);
    server.shutdown();
}
#[test]
fn m105_real_topic_ingress_drives_two_and_eight_owners_on_both_backends() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for count in [2, 8] {
            let mut server = build(count, engine, 64);
            let saved = originals(&server.simulation).unwrap();
            let session = server.zenoh_session();
            let publishers: Vec<_> = (0..count)
                .map(|i| {
                    session
                        .declare_publisher(
                            server
                                .network_config
                                .topic_key(&format!("/m105/{i}/cmd"))
                                .unwrap(),
                        )
                        .wait()
                        .unwrap()
                })
                .collect();
            let deadline = Instant::now() + Duration::from_secs(5);
            while publishers
                .iter()
                .any(|p| !p.matching_status().wait().unwrap().matching())
            {
                assert!(Instant::now() < deadline);
                std::thread::yield_now();
            }
            for (i, p) in publishers.iter().enumerate() {
                p.put(
                    Twist {
                        linear: Some(vector3([(i + 1) as f64, 0., 0.])),
                        angular: Some(vector3([0.; 3])),
                        ..Default::default()
                    }
                    .encode_to_vec(),
                )
                .attachment(Twist::full_name())
                .wait()
                .unwrap();
            }
            while server.counters.velocity_control_commands_pending < count as u64 {
                assert!(Instant::now() < deadline);
                server.update().unwrap();
                std::thread::yield_now();
            }
            assert!(server.apply_local_control(WorldControl {
                pause: false,
                ..Default::default()
            }));
            server.update().unwrap();
            server.update().unwrap();
            for (i, s) in saved.iter().enumerate() {
                near(linear(&server, s.id), [(i + 1) as f64, 0., 0.]);
            }
            drop(publishers);
            server.shutdown();
        }
    }
}
#[test]
fn m105_all_owner_and_canonical_topic_preflight_happens_before_network() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let sim = load(&source(8), engine);
        let options = ServerOptions {
            topic_remaps: std::collections::HashMap::from([
                ("/m105/0/cmd".into(), "/shared".into()),
                ("/m105/7/cmd".into(), "/shared".into()),
            ]),
            ..Default::default()
        };
        assert!(matches!(
            before_network(&sim, "default", &options),
            Err(Error::VelocityControlIdentity)
        ));
        let mut sim = load(&source(8), engine);
        let last = originals(&sim).unwrap().pop().unwrap();
        let e = entity_by_id(sim.world(), last.id).unwrap();
        sim.world_mut()
            .entity_mut(e)
            .remove::<VelocityControlConfiguration>();
        assert!(matches!(
            before_network(&sim, "default", &ServerOptions::default()),
            Err(Error::VelocityControlIdentity)
        ));
    }
}
#[test]
fn m105_late_owner_fault_retires_every_lease_before_consuming_messages() {
    let mut server = build(8, PhysicsEngine::Rapier, 8);
    let first = server.velocity_control.as_ref().unwrap();
    let leases: Vec<_> = std::iter::once(first)
        .chain(first.others.iter())
        .map(|c| (Arc::clone(&c.lease), c.lease_id, c.key.clone()))
        .collect();
    let e = first.others.last().unwrap().entity;
    server
        .simulation
        .world_mut()
        .entity_mut(e)
        .remove::<VelocityControlConfiguration>();
    assert!(matches!(
        server.poll_velocity_control(),
        Err(Error::VelocityControlIdentity)
    ));
    assert!(server.velocity_control.is_none());
    assert_eq!(server.counters.velocity_control_messages_received, 0);
    for (l, id, key) in leases {
        assert!(!l.matches(id, &key));
    }
    server.shutdown();
}
#[test]
fn m105_removal_promotes_survivors_and_full_reset_rebuilds_original_eight_leases() {
    let mut server = build(8, PhysicsEngine::Tpe, 8);
    let first = server.velocity_control.as_ref().unwrap();
    let removed = first.original.id;
    let old = Arc::clone(&first.lease);
    let old_id = first.lease_id;
    let key = first.key.clone();
    gz_sim_scene::remove_model(&mut server.simulation, removed).unwrap();
    server.update().unwrap();
    assert!(!old.matches(old_id, &key));
    assert_eq!(server.velocity_control.as_ref().unwrap().others.len(), 6);
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.update().unwrap();
    let first = server.velocity_control.as_ref().unwrap();
    assert_eq!(first.original.id, removed);
    assert_eq!(first.others.len(), 7);
    assert_ne!(first.lease_id, old_id);
    server.shutdown();
    assert!(server.velocity_control.is_none());
}
#[test]
fn m105_stale_and_invalid_messages_share_one_bounded_fleet_budget() {
    let mut server = build(2, PhysicsEngine::Tpe, 2);
    let epoch = server.simulation.epoch();
    let first = server.velocity_control.as_mut().unwrap();
    let (a, ar) = mpsc::sync_channel(2);
    first.receiver = ar;
    let (b, br) = mpsc::sync_channel(2);
    first.others[0].receiver = br;
    a.send((epoch.wrapping_add(1), Some(([9.; 3], [0.; 3]))))
        .unwrap();
    b.send((epoch, None)).unwrap();
    server.poll_velocity_control().unwrap();
    assert_eq!(server.counters.velocity_control_messages_received, 2);
    assert_eq!(server.counters.velocity_control_messages_rejected, 2);
    server.simulation.step_exact(2).unwrap();
    for s in originals(&server.simulation).unwrap() {
        near(linear(&server, s.id), [0.; 3]);
    }
    drop((a, b));
    server.shutdown();
}

type LinkState = ([f64; 3], [f64; 4], [f64; 3], [f64; 3]);
fn link_states(sim: &Simulation, links: &[u64]) -> Vec<LinkState> {
    links
        .iter()
        .map(|id| {
            let entity = entity_by_id(sim.world(), *id).unwrap();
            let pose = sim.world().get::<gz_sim_scene::WorldPose>(entity).unwrap();
            let velocity = sim.world().get::<Velocity>(entity).unwrap();
            (
                pose.translation,
                pose.rotation,
                velocity.linear,
                velocity.angular,
            )
        })
        .collect()
}

#[test]
fn m105_late_owner_identity_faults_retire_all_queues_without_consuming_or_moving() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for fault in [
            "missing-id",
            "wrong-kind",
            "wrong-name",
            "wrong-parent",
            "missing-local-pose",
            "missing-rapier-body",
        ] {
            // Both backends require the original LINK pose. Only Rapier owns
            // BodyHandle; removing that component from TPE would be a no-op.
            if fault == "missing-rapier-body" && !matches!(engine, PhysicsEngine::Rapier) {
                continue;
            }
            let mut server = build(8, engine, 8);
            let saved = originals(&server.simulation).unwrap();
            let links: Vec<_> = saved.iter().map(|owner| owner.metadata.link).collect();
            let epoch = server.simulation.epoch();
            let first = server.velocity_control.as_mut().unwrap();
            let mut senders = Vec::new();
            let mut leases = Vec::new();
            for index in 0..=first.others.len() {
                let c = if index == 0 {
                    &mut *first
                } else {
                    &mut first.others[index - 1]
                };
                let (send, receive) = mpsc::sync_channel(1);
                c.receiver = receive;
                send.send((epoch, Some(([3., 4., 5.], [0.1, 0.2, 0.3]))))
                    .unwrap();
                senders.push(send);
                leases.push((Arc::clone(&c.lease), c.lease_id, c.key.clone()));
            }
            let last = saved.last().unwrap();
            let entity = entity_by_id(server.simulation.world(), last.id).unwrap();
            match fault {
                "missing-id" => {
                    server
                        .simulation
                        .world_mut()
                        .entity_mut(entity)
                        .remove::<EntityId>();
                }
                "wrong-kind" => {
                    server
                        .simulation
                        .world_mut()
                        .entity_mut(entity)
                        .insert(EntityKind::Link);
                }
                "wrong-name" => {
                    server
                        .simulation
                        .world_mut()
                        .entity_mut(entity)
                        .insert(Name("default::forged".into()));
                }
                "wrong-parent" => {
                    let other = entity_by_id(server.simulation.world(), saved[0].id).unwrap();
                    server
                        .simulation
                        .world_mut()
                        .entity_mut(entity)
                        .insert(ChildOf(other));
                }
                "missing-local-pose" => {
                    let link = entity_by_id(server.simulation.world(), last.metadata.link).unwrap();
                    assert!(
                        server
                            .simulation
                            .world_mut()
                            .entity_mut(link)
                            .take::<gz_sim_scene::LocalPose>()
                            .is_some()
                    );
                }
                "missing-rapier-body" => {
                    let link = entity_by_id(server.simulation.world(), last.metadata.link).unwrap();
                    assert!(
                        server
                            .simulation
                            .world_mut()
                            .entity_mut(link)
                            .take::<gz_sim_scene::BodyHandle>()
                            .is_some()
                    );
                }
                _ => unreachable!(),
            }
            let before = link_states(&server.simulation, &links);
            let time = server.simulation.elapsed();
            assert!(
                matches!(
                    server.poll_velocity_control(),
                    Err(Error::VelocityControlIdentity)
                ),
                "fault={fault}"
            );
            assert!(server.velocity_control.is_none() && server.reset_failed);
            assert_eq!(server.counters.velocity_control_messages_received, 0);
            assert_eq!(server.counters.velocity_control_commands_pending, 0);
            assert_eq!(
                (server.simulation.epoch(), server.simulation.elapsed()),
                (epoch, time)
            );
            assert_eq!(link_states(&server.simulation, &links), before);
            assert!(
                leases
                    .iter()
                    .all(|(lease, id, key)| !lease.matches(*id, key))
            );
            assert!(senders.iter().all(|send| matches!(
                send.try_send((epoch, None)),
                Err(mpsc::TrySendError::Disconnected(_))
            )));
            server.shutdown();
        }
    }
}

#[test]
fn m105_controller_roster_and_lease_faults_fail_before_any_owner_command() {
    for fault in [
        "absent-controller",
        "missing-controller",
        "wrong-lease-id",
        "wrong-key",
    ] {
        let mut server = build(8, PhysicsEngine::Rapier, 8);
        let saved = originals(&server.simulation).unwrap();
        let links: Vec<_> = saved.iter().map(|owner| owner.metadata.link).collect();
        let epoch = server.simulation.epoch();
        let first = server.velocity_control.as_mut().unwrap();
        let mut senders = Vec::new();
        let mut leases = Vec::new();
        for index in 0..=first.others.len() {
            let c = if index == 0 {
                &mut *first
            } else {
                &mut first.others[index - 1]
            };
            let (send, receive) = mpsc::sync_channel(1);
            c.receiver = receive;
            send.send((epoch, Some(([7.; 3], [0.; 3])))).unwrap();
            senders.push(send);
            leases.push((Arc::clone(&c.lease), c.lease_id, c.key.clone()));
        }
        match fault {
            "absent-controller" => {
                server.velocity_control = None;
            }
            "missing-controller" => {
                server.velocity_control.as_mut().unwrap().others.pop();
            }
            "wrong-lease-id" => {
                server
                    .velocity_control
                    .as_mut()
                    .unwrap()
                    .others
                    .last_mut()
                    .unwrap()
                    .lease_id = uuid::Uuid::nil();
            }
            "wrong-key" => {
                server
                    .velocity_control
                    .as_mut()
                    .unwrap()
                    .others
                    .last_mut()
                    .unwrap()
                    .key
                    .push_str("/forged");
            }
            _ => unreachable!(),
        }
        let before = link_states(&server.simulation, &links);
        assert!(
            matches!(
                server.poll_velocity_control(),
                Err(Error::VelocityControlIdentity)
            ),
            "fault={fault}"
        );
        assert!(server.velocity_control.is_none() && server.reset_failed);
        assert_eq!(server.counters.velocity_control_messages_received, 0);
        assert_eq!(server.counters.velocity_control_commands_pending, 0);
        assert_eq!(
            (server.simulation.epoch(), server.simulation.elapsed()),
            (epoch, Duration::ZERO)
        );
        assert_eq!(link_states(&server.simulation, &links), before);
        assert!(
            leases
                .iter()
                .all(|(lease, id, key)| !lease.matches(*id, key))
        );
        assert!(senders.iter().all(|send| matches!(
            send.try_send((epoch, None)),
            Err(mpsc::TrySendError::Disconnected(_))
        )));
        server.shutdown();
    }
}

#[test]
fn m105_prebuild_removal_keeps_survivors_and_full_reset_restores_original_fleet() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for removed_index in [0, 7] {
            let mut sim = load(&source(8), engine);
            let original = originals(&sim).unwrap();
            let removed = original[removed_index].id;
            gz_sim_scene::remove_model(&mut sim, removed).unwrap();
            let collision = ServerOptions {
                topic_remaps: std::collections::HashMap::from([(
                    original[removed_index].metadata.topic.clone(),
                    "/world/default/clock".into(),
                )]),
                ..Default::default()
            };
            assert!(matches!(
                before_network(&sim, "default", &collision),
                Err(Error::VelocityControlIdentity)
            ));
            let mut server = Server::with_zenoh_config(
                sim,
                "default",
                ServerOptions {
                    partition: format!("m105-prebuild-{}", uuid::Uuid::new_v4()),
                    ..Default::default()
                },
                network(),
                NonZeroUsize::new(8).unwrap(),
            )
            .unwrap();
            let first = server.velocity_control.as_mut().unwrap();
            assert_eq!(first.others.len(), 6);
            let epoch = server.simulation.epoch();
            let mut senders = Vec::new();
            let mut leases = Vec::new();
            let mut survivors = Vec::new();
            for index in 0..=first.others.len() {
                let c = if index == 0 {
                    &mut *first
                } else {
                    &mut first.others[index - 1]
                };
                assert_ne!(c.original.id, removed);
                survivors.push(c.original.id);
                leases.push((Arc::clone(&c.lease), c.lease_id, c.key.clone()));
                let (send, receive) = mpsc::sync_channel(1);
                c.receiver = receive;
                send.send((epoch, Some(([2., 0., 0.], [0.; 3])))).unwrap();
                senders.push(send);
            }
            assert!(server.apply_local_control(WorldControl {
                pause: false,
                ..Default::default()
            }));
            server.update().unwrap();
            server.update().unwrap();
            for id in &survivors {
                near(linear(&server, *id), [2., 0., 0.]);
            }
            assert_eq!(server.counters.velocity_control_commands_pending, 7);
            assert!(entity_by_id(server.simulation.world(), removed).is_none());
            assert!(server.apply_local_control(WorldControl {
                pause: true,
                reset: Some(WorldReset {
                    all: true,
                    ..Default::default()
                }),
                ..Default::default()
            }));
            server.update().unwrap();
            assert_eq!(server.simulation.epoch(), epoch + 1);
            assert_eq!(server.simulation.elapsed(), Duration::ZERO);
            let first = server.velocity_control.as_ref().unwrap();
            let restored: Vec<_> = std::iter::once(first).chain(first.others.iter()).collect();
            assert_eq!(
                restored.iter().map(|c| c.original.id).collect::<Vec<_>>(),
                original.iter().map(|o| o.id).collect::<Vec<_>>()
            );
            assert!(
                leases
                    .iter()
                    .all(|(lease, id, key)| !lease.matches(*id, key))
            );
            for c in restored {
                assert!(c.lease.matches(c.lease_id, &c.key));
                assert!(leases.iter().all(|(_, id, _)| *id != c.lease_id));
                near(linear(&server, c.original.id), [0.; 3]);
            }
            drop(senders);
            server.shutdown();
        }
    }
}

#[test]
fn m105_unowned_initial_metadata_and_disconnected_queue_cannot_deliver_commands() {
    let loaded = load(&source(2), PhysicsEngine::Rapier);
    let metadata = originals(&loaded).unwrap()[0].metadata.clone();
    let mut bare = Simulation::new(Duration::from_millis(1)).unwrap();
    bare.world_mut().spawn(metadata);
    assert!(matches!(
        originals(&bare),
        Err(Error::VelocityControlIdentity)
    ));
    assert_eq!(bare.elapsed(), Duration::ZERO);
    let mut server = build(2, PhysicsEngine::Tpe, 2);
    let saved = originals(&server.simulation).unwrap();
    let links: Vec<_> = saved.iter().map(|owner| owner.metadata.link).collect();
    let first = server.velocity_control.as_mut().unwrap();
    let (send, receive) = mpsc::sync_channel(1);
    first.receiver = receive;
    drop(send);
    let before = link_states(&server.simulation, &links);
    assert!(matches!(
        server.poll_velocity_control(),
        Err(Error::WorldClosed)
    ));
    assert_eq!(server.counters.velocity_control_messages_received, 0);
    assert_eq!(server.counters.velocity_control_commands_pending, 0);
    assert_eq!(server.simulation.elapsed(), Duration::ZERO);
    assert_eq!(link_states(&server.simulation, &links), before);
    server.shutdown();
    assert!(server.velocity_control.is_none());
}

#[test]
fn m105_loaded_world_identity_must_still_own_every_original_fleet_member() {
    for fault in [
        "missing-world",
        "missing-id",
        "missing-name",
        "renamed-world",
        "second-world",
    ] {
        let mut sim = load(&source(2), PhysicsEngine::Rapier);
        let world = sim
            .world()
            .iter_entities()
            .find(|node| node.get::<EntityKind>() == Some(&EntityKind::World))
            .unwrap()
            .id();
        let links: Vec<_> = originals(&sim)
            .unwrap()
            .iter()
            .map(|owner| owner.metadata.link)
            .collect();
        match fault {
            "missing-world" => {
                sim.world_mut().entity_mut(world).remove::<EntityKind>();
            }
            "missing-id" => {
                sim.world_mut().entity_mut(world).remove::<EntityId>();
            }
            "missing-name" => {
                sim.world_mut().entity_mut(world).remove::<Name>();
            }
            "renamed-world" => {
                sim.world_mut()
                    .entity_mut(world)
                    .insert(Name("another".into()));
            }
            "second-world" => {
                sim.world_mut().spawn(EntityKind::World);
            }
            _ => unreachable!(),
        }
        let before = link_states(&sim, &links);
        assert!(
            matches!(
                before_network(&sim, "default", &ServerOptions::default()),
                Err(Error::VelocityControlIdentity)
            ),
            "fault={fault}"
        );
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert_eq!(link_states(&sim, &links), before);
    }
}
