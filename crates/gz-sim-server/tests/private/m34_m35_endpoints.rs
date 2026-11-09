// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::msgs::WorldReset;
use gz_sim_scene::{Name, PhysicsEngine, WorldPose, load_world_with_sdf_step};

const WRENCH: &str =
    "<plugin name='gz::sim::systems::ApplyLinkWrench' filename='gz-sim-apply-link-wrench-system'/>";
const ODOM: &str = include_str!("../../../../examples/m34_odometry_publisher.sdf");

fn combined() -> String {
    ODOM.replace(
        "<world name=\"default\">",
        &format!("<world name=\"default\">{WRENCH}"),
    )
}
fn loaded(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "system-endpoints.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn config() -> zenoh::Config {
    zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}},transport:{shared_memory:{enabled:false}}}"#).unwrap()
}
fn server(sim: Simulation, mut options: ServerOptions) -> Result<Server, Error> {
    if options.partition.is_empty() {
        options.partition = format!("m34-m35-{}", uuid::Uuid::new_v4());
    }
    Server::with_zenoh_config(
        sim,
        "default",
        options,
        config(),
        NonZeroUsize::new(8).unwrap(),
    )
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}

#[test]
fn all_three_new_input_and_output_keys_refuse_raw_and_remapped_aliases() {
    let xml = combined();
    let raw = xml.replace(WRENCH, "<plugin name='gz::sim::systems::ApplyLinkWrench' filename='gz-sim-apply-link-wrench-system'><topic>/m34/odometry</topic></plugin>");
    assert!(matches!(
        server(loaded(&raw), ServerOptions::default()),
        Err(Error::GroundTruthOdometryIdentity | Error::ApplyLinkWrenchIdentity)
    ));
    for (from, to) in [
        ("/world/default/wrench", "/m34/odometry"),
        (
            "/world/default/wrench/persistent",
            "/m34/odometry_covariance",
        ),
        ("/world/default/wrench/clear", "/m34/transform"),
        ("/m34/odometry", "/clock"),
        ("/m34/odometry_covariance", "/m34/transform"),
        ("/world/default/wrench", "/world/default/wrench/clear"),
    ] {
        let mut options = ServerOptions::default();
        options.add_topic_remap(from, to).unwrap();
        assert!(
            matches!(
                server(loaded(&xml), options),
                Err(Error::GroundTruthOdometryIdentity | Error::ApplyLinkWrenchIdentity)
            ),
            "{from} -> {to}"
        );
    }
}

#[test]
fn removed_original_odometry_keeps_reset_reservations_and_refusal_is_atomic() {
    let mut server = server(loaded(&combined()), ServerOptions::default()).unwrap();
    let owner = id(&server.simulation, "default::rig");
    let leases: Vec<_> = server
        .ground_truth_odometries
        .iter()
        .map(|s| Arc::downgrade(&s.publisher))
        .collect();
    server.remove_editable_model(owner).unwrap();
    assert!(server.ground_truth_odometries.is_empty());
    assert!(leases.iter().all(|lease| lease.upgrade().is_none()));
    let before = (
        server.simulation.epoch(),
        server.simulation.elapsed(),
        server.simulation.world().entities().len(),
    );
    server
        .network_config
        .add_topic_remap("/world/default/wrench", "/m34/odometry")
        .unwrap();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(
        server.prepare_io(),
        Err(Error::GroundTruthOdometryIdentity | Error::ApplyLinkWrenchIdentity)
    ));
    assert_eq!(
        (
            server.simulation.epoch(),
            server.simulation.elapsed(),
            server.simulation.world().entities().len()
        ),
        before
    );
    assert!(server.ground_truth_odometries.is_empty());
    assert!(server.link_wrenches.is_none());
    assert!(leases.iter().all(|lease| lease.upgrade().is_none()));
}

#[test]
fn dormant_wrench_server_spawn_refuses_before_mutation_and_loaded_reset_reactivates() {
    let xml = include_str!("../../../../examples/m35_apply_link_wrench.sdf");
    let sim = loaded(xml);
    let sdf = sdformat::load_str(xml, "wrench-template.sdf", &sdformat::NoResources).unwrap();
    let template =
        gz_sim_scene::prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let mut server = server(sim, ServerOptions::default()).unwrap();
    assert!(server.link_wrenches.is_some());
    let rig = id(&server.simulation, "default::rig");
    let other = id(&server.simulation, "default::other");
    server.remove_editable_model(rig).unwrap();
    server.remove_editable_model(other).unwrap();
    assert!(!gz_sim_scene::apply_link_wrench_active(&server.simulation));
    assert!(server.link_wrenches.is_none());
    let before = (
        server.simulation.epoch(),
        server.simulation.elapsed(),
        server.simulation.world().entities().len(),
        gz_sim_core::next_entity_id(server.simulation.world()),
    );
    assert!(matches!(
        server.spawn_model_template(
            &template,
            WorldPose {
                translation: [4.0, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0]
            }
        ),
        Err(Error::ApplyLinkWrenchReactivation)
    ));
    assert_eq!(
        (
            server.simulation.epoch(),
            server.simulation.elapsed(),
            server.simulation.world().entities().len(),
            gz_sim_core::next_entity_id(server.simulation.world())
        ),
        before
    );
    assert!(!gz_sim_scene::apply_link_wrench_active(&server.simulation));
    assert!(server.link_wrenches.is_none());
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    server.prepare_io().unwrap();
    assert!(gz_sim_scene::apply_link_wrench_active(&server.simulation));
    assert!(server.link_wrenches.is_some());
    assert_eq!(server.simulation.epoch(), before.0 + 1);
    assert_eq!(server.simulation.elapsed(), Duration::ZERO);
    assert_eq!(id(&server.simulation, "default::rig"), rig);
    assert_eq!(id(&server.simulation, "default::other"), other);
}

#[test]
fn wrench_setup_rejects_corrupt_world_identity_and_topics_without_scene_mutation() {
    use crate::apply_link_wrench::ApplyLinkWrenches;
    use gz_sim_scene::{ApplyLinkWrenchConfiguration, EntityKind};
    let xml = include_str!("../../../../examples/m35_apply_link_wrench.sdf");
    let options = ServerOptions {
        partition: "m35-identity".into(),
        ..Default::default()
    };
    let drops = Arc::new(std::sync::atomic::AtomicU64::new(0));
    for fault in [
        "missing-id",
        "duplicate-world",
        "wrong-kind",
        "invalid-topic",
    ] {
        let mut sim = loaded(xml);
        let world = gz_sim_core::entity_by_id(sim.world(), id(&sim, "default")).unwrap();
        let mut configuration = sim
            .world()
            .get::<ApplyLinkWrenchConfiguration>(world)
            .unwrap()
            .clone();
        match fault {
            "missing-id" => {
                sim.world_mut().entity_mut(world).remove::<EntityId>();
            }
            "duplicate-world" => {
                let extra = sim
                    .world_mut()
                    .spawn((EntityKind::World, configuration))
                    .id();
                gz_sim_core::assign_id(sim.world_mut(), extra).unwrap();
            }
            "wrong-kind" => {
                sim.world_mut().entity_mut(world).insert(EntityKind::Model);
            }
            "invalid-topic" => {
                configuration.topic = "/m35/bad topic".into();
                sim.world_mut().entity_mut(world).insert(configuration);
            }
            _ => unreachable!(),
        }
        let before = (
            sim.epoch(),
            sim.elapsed(),
            sim.world().entities().len(),
            gz_sim_core::next_entity_id(sim.world()),
        );
        assert!(
            matches!(
                ApplyLinkWrenches::prepare(&sim, &options, &BTreeSet::new(), &drops),
                Err(Error::ApplyLinkWrenchIdentity)
            ),
            "{fault}"
        );
        assert_eq!(
            (
                sim.epoch(),
                sim.elapsed(),
                sim.world().entities().len(),
                gz_sim_core::next_entity_id(sim.world())
            ),
            before
        );
        assert_eq!(drops.load(std::sync::atomic::Ordering::Relaxed), 0);
    }
}

#[test]
fn wrench_prepared_owner_rejects_metadata_or_epoch_drift_and_stops_after_shutdown() {
    use crate::apply_link_wrench::{ApplyLinkWrenches, prepare_reset};
    let xml = include_str!("../../../../examples/m35_apply_link_wrench.sdf");
    let options = ServerOptions {
        partition: "m35-bind".into(),
        ..Default::default()
    };
    let drops = Arc::new(std::sync::atomic::AtomicU64::new(0));
    for fault in ["prepared-key", "metadata", "epoch"] {
        let mut sim = loaded(xml);
        let mut prepared = prepare_reset(&sim, &options, &BTreeSet::new())
            .unwrap()
            .unwrap();
        let (mut owner, setup) =
            ApplyLinkWrenches::prepare(&sim, &options, &BTreeSet::new(), &drops)
                .unwrap()
                .unwrap();
        owner.bind(&sim, &prepared).unwrap();
        match fault {
            "prepared-key" => prepared.keys[0].push_str("/different"),
            "metadata" => {
                let world = gz_sim_core::entity_by_id(sim.world(), prepared.world_id).unwrap();
                let mut configuration = prepared.configuration.clone();
                configuration.clear_topic.push_str("/different");
                sim.world_mut().entity_mut(world).insert(configuration);
            }
            "epoch" => {
                sim.reset().unwrap();
            }
            _ => unreachable!(),
        }
        let clock = (sim.epoch(), sim.elapsed());
        assert!(
            matches!(
                owner.bind(&sim, &prepared),
                Err(Error::ApplyLinkWrenchIdentity)
            ),
            "{fault}"
        );
        let mut counters = Counters::default();
        if fault != "prepared-key" {
            assert!(
                matches!(
                    owner.poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters),
                    Err(Error::ApplyLinkWrenchIdentity)
                ),
                "{fault}"
            );
        }
        assert_eq!(
            (
                counters.wrench_messages_received,
                counters.wrench_messages_rejected,
                counters.link_wrenches_queued,
                counters.link_wrench_clears_queued
            ),
            (0, 0, 0, 0)
        );
        owner.shutdown();
        assert!(owner.is_empty());
        assert!(
            setup
                .endpoints
                .iter()
                .all(|e| !e.active.load(std::sync::atomic::Ordering::Acquire))
        );
        owner
            .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
            .unwrap();
        assert_eq!((sim.epoch(), sim.elapsed()), clock);
    }
}

#[test]
fn wrench_poll_refuses_stale_misrouted_and_ambiguous_commands_before_physics() {
    use crate::apply_link_wrench::ApplyLinkWrenches;
    use crate::apply_link_wrench_io::{Channel, Command, Envelope};
    use gz_sim_proto::msgs::{Entity as WireEntity, EntityWrench, entity::Type};
    let xml = include_str!("../../../../examples/m35_apply_link_wrench.sdf");
    let mut sim = loaded(xml);
    let body = id(&sim, "default::rig::body");
    let duplicate = sim
        .world_mut()
        .spawn((
            gz_sim_scene::EntityKind::Link,
            Name("default::rig::body".into()),
        ))
        .id();
    gz_sim_core::assign_id(sim.world_mut(), duplicate).unwrap();
    let options = ServerOptions {
        partition: "m35-poll".into(),
        ..Default::default()
    };
    let drops = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let (mut owner, setup) = ApplyLinkWrenches::prepare(&sim, &options, &BTreeSet::new(), &drops)
        .unwrap()
        .unwrap();
    let target = WireEntity {
        id: body,
        r#type: Type::Link as i32,
        ..Default::default()
    };
    let mut counters = Counters::default();
    let mut rejected = 0;
    // Exercise the typed post-callback boundary directly; the existing real
    // canonical peers remain the independent transport/physics controls.
    for (channel, epoch, command) in [
        (
            Channel::Clear,
            sim.epoch() + 1,
            Command::Clear(target.clone()),
        ),
        (Channel::Once, sim.epoch(), Command::Clear(target.clone())),
        (
            Channel::Clear,
            sim.epoch(),
            Command::Wrench(Box::<EntityWrench>::default()),
        ),
        (
            Channel::Clear,
            sim.epoch(),
            Command::Clear(WireEntity {
                id: 0,
                name: String::new(),
                ..target.clone()
            }),
        ),
        (
            Channel::Clear,
            sim.epoch(),
            Command::Clear(WireEntity {
                id: 0,
                name: "missing".into(),
                ..target.clone()
            }),
        ),
        (
            Channel::Clear,
            sim.epoch(),
            Command::Clear(WireEntity {
                id: 0,
                name: "default::rig::body".into(),
                ..target
            }),
        ),
    ] {
        assert!(
            setup.endpoints[channel.index()]
                .input
                .send(Envelope {
                    epoch,
                    command: Some(command)
                })
                .is_ok()
        );
        owner
            .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
            .unwrap();
        rejected += 1;
        assert_eq!(
            (
                counters.wrench_messages_received,
                counters.wrench_messages_rejected
            ),
            (rejected, rejected)
        );
        assert_eq!(
            (
                counters.link_wrenches_queued,
                counters.link_wrench_clears_queued
            ),
            (0, 0)
        );
        assert_eq!((sim.epoch(), sim.elapsed()), (0, Duration::ZERO));
    }
    // A disconnected owned ingress is an explicit failure, not an empty queue.
    drop(setup);
    assert!(matches!(
        owner.poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters),
        Err(Error::WorldClosed)
    ));
    assert_eq!(
        (
            counters.wrench_messages_received,
            counters.wrench_messages_rejected
        ),
        (6, 6)
    );
}
