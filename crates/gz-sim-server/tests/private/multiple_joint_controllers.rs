// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::msgs::WorldReset;
use gz_sim_scene::{
    JointControllerMode, JointSample, PhysicsEngine, load_world_with_sdf_step, remove_model,
};
use std::time::Instant;
const XML: &str = include_str!("../../../../examples/m58_multiple_joint_controllers.sdf");
fn config(endpoint: Option<&str>) -> zenoh::Config {
    let mut config = zenoh::Config::default();
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
        config.insert_json5(key, value).unwrap();
    }
    config
}
fn loaded(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m9-server.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn build(xml: &str) -> (Server, zenoh::Session) {
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
        loaded(xml),
        "default",
        ServerOptions {
            capacity: 8192,
            partition: format!("joint-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        },
        config(Some(&locators[0])),
        NonZeroUsize::new(2).unwrap(),
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
fn reset(server: &mut Server) {
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
    assert!(info.dt.is_zero());
}

#[derive(Debug, PartialEq, Eq)]
struct BoundarySnapshot {
    epoch: u64,
    time: Duration,
    received: u64,
    applied: u64,
    rejected: u64,
    streams: Vec<(u64, String, usize)>,
}
fn snapshot(server: &Server) -> BoundarySnapshot {
    BoundarySnapshot {
        epoch: server.simulation.epoch(),
        time: server.simulation.elapsed(),
        received: server.counters.joint_controller_messages_received,
        applied: server.counters.joint_controller_commands_applied,
        rejected: server.counters.joint_controller_messages_rejected,
        streams: server
            .joint_controllers
            .as_ref()
            .map(|controllers| {
                controllers
                    .streams
                    .iter()
                    .map(|stream| {
                        (
                            stream.identity,
                            stream.endpoint.clone(),
                            Arc::as_ptr(&stream._lease) as usize,
                        )
                    })
                    .collect()
            })
            .unwrap_or_default(),
    }
}
fn selected(server: &Server, model: &str, mode: JointControllerMode) -> Entity {
    server
        .joint_controllers
        .as_ref()
        .unwrap()
        .streams
        .iter()
        .find(|stream| stream.metadata.model_name == model && stream.metadata.mode == mode)
        .unwrap()
        .entity
}
fn joint_state(server: &Server, entity: Entity) -> (f64, f64) {
    let sample = server
        .simulation
        .world()
        .get::<JointSample>(entity)
        .unwrap();
    assert_eq!(
        (sample.epoch, sample.time),
        (server.simulation.epoch(), server.simulation.elapsed())
    );
    (sample.state.position, sample.state.velocity)
}

#[derive(Debug, PartialEq, Eq)]
struct JointSnapshot {
    native: Option<[u64; 2]>,
    sample: Option<(Duration, u64, [u64; 2])>,
}
fn physical(sim: &Simulation, owners: &[(Entity, JointHandle)]) -> Vec<JointSnapshot> {
    owners
        .iter()
        .map(|(entity, joint)| JointSnapshot {
            native: sim
                .world()
                .resource::<gz_physics::PhysicsBackend>()
                .joint_state(joint.id())
                .map(|state| [state.position.to_bits(), state.velocity.to_bits()]),
            sample: sim.world().get::<JointSample>(*entity).map(|sample| {
                (
                    sample.time,
                    sample.epoch,
                    [
                        sample.state.position.to_bits(),
                        sample.state.velocity.to_bits(),
                    ],
                )
            }),
        })
        .collect()
}
fn swap_same_owner_native_bindings(server: &mut Server) {
    let position = selected(server, "arm_right", JointControllerMode::Position);
    let velocity = selected(server, "arm_right", JointControllerMode::Velocity);
    assert_eq!(
        server
            .simulation
            .world()
            .get::<ChildOf>(position)
            .unwrap()
            .parent(),
        server
            .simulation
            .world()
            .get::<ChildOf>(velocity)
            .unwrap()
            .parent()
    );
    let first = *server
        .simulation
        .world()
        .get::<JointHandle>(position)
        .unwrap();
    let second = *server
        .simulation
        .world()
        .get::<JointHandle>(velocity)
        .unwrap();
    assert_ne!(first.id(), second.id());
    assert_ne!(first.child(), second.child());
    let mut first_definition = server
        .simulation
        .world()
        .get::<JointDefinition>(position)
        .unwrap()
        .clone();
    let mut second_definition = server
        .simulation
        .world()
        .get::<JointDefinition>(velocity)
        .unwrap()
        .clone();
    std::mem::swap(
        &mut first_definition.0.child,
        &mut second_definition.0.child,
    );
    server
        .simulation
        .world_mut()
        .entity_mut(position)
        .insert((second, first_definition));
    server
        .simulation
        .world_mut()
        .entity_mut(velocity)
        .insert((first, second_definition));
}

#[test]
fn all_four_original_current_identities_and_late_native_bindings_preflight_before_io() {
    let (empty, _) = build(include_str!("../../../../examples/mvp_primitives.sdf"));
    assert!(
        boundary_keys(&empty.simulation, &empty.network_config, None)
            .unwrap()
            .is_empty()
    );
    assert!(empty.joint_controllers.is_none());
    for mutation in 0..14 {
        let (mut server, _observer) = build(XML);
        assert_eq!(initial(&server.simulation).unwrap().len(), 4);
        assert_eq!(
            boundary_keys(&server.simulation, &server.network_config, None)
                .unwrap()
                .len(),
            4
        );
        let entity = selected(&server, "arm_right", JointControllerMode::Velocity);
        let other = selected(&server, "arm_left", JointControllerMode::Velocity);
        let original = server
            .simulation
            .world()
            .get::<JointControllerConfiguration>(entity)
            .unwrap()
            .clone();
        let joint = *server
            .simulation
            .world()
            .get::<JointHandle>(entity)
            .unwrap();
        let child = entity_by_id(server.simulation.world(), joint.child()).unwrap();
        let before = snapshot(&server);
        match mutation {
            0 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(Name("default::arm_left::tip_hinge".into()));
            }
            1 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(EntityKind::Link);
            }
            2 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .remove::<EntityId>();
            }
            3 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .remove::<ChildOf>();
            }
            4 => {
                let parent = server
                    .simulation
                    .world()
                    .get::<ChildOf>(other)
                    .unwrap()
                    .parent();
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(ChildOf(parent));
            }
            5 => {
                let handle = *server.simulation.world().get::<JointHandle>(other).unwrap();
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(handle);
            }
            6 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(child)
                    .remove::<BodyHandle>();
            }
            7 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(child)
                    .insert(Name("default::arm_right::wrong".into()));
            }
            8 => {
                let mut definition = server
                    .simulation
                    .world()
                    .get::<JointDefinition>(entity)
                    .unwrap()
                    .clone();
                definition.0.child = "base".into();
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(definition);
            }
            9 => {
                let mut changed = original;
                changed.topic.push_str("/drift");
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(changed);
            }
            10 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .remove::<JointControllerConfiguration>();
            }
            11 => {
                let parent = server
                    .simulation
                    .world()
                    .get::<ChildOf>(entity)
                    .unwrap()
                    .parent();
                server
                    .simulation
                    .world_mut()
                    .entity_mut(parent)
                    .insert(Name("default::wrong_model".into()));
            }
            12 => {
                server.simulation.world_mut().spawn(original);
            }
            _ => {
                assert!(
                    server
                        .simulation
                        .world_mut()
                        .resource_mut::<gz_physics::PhysicsBackend>()
                        .remove_joint(joint.id())
                );
            }
        }
        assert!(boundary_keys(&server.simulation, &server.network_config, None).is_err());
        assert!(prepare_reset(&server).is_err());
        assert!(
            prepare(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &initial(&server.simulation).unwrap()
            )
            .is_err()
        );
        assert_eq!(snapshot(&server), before);
        assert!(!server.reset_failed);
    }
    // Both swapped handles remain real/unique and match the coordinated public
    // child definitions. Only the authoritative private Scene ownership witness
    // can refuse this plausible public-component pair before input IO.
    let (mut server, _observer) = build(XML);
    let owners: Vec<_> = server
        .joint_controllers
        .as_ref()
        .unwrap()
        .streams
        .iter()
        .map(|stream| (stream.entity, stream.joint))
        .collect();
    let before = snapshot(&server);
    let native_before = physical(&server.simulation, &owners);
    swap_same_owner_native_bindings(&mut server);
    for mode in [JointControllerMode::Position, JointControllerMode::Velocity] {
        let entity = selected(&server, "arm_right", mode);
        let node = server.simulation.world().entity(entity);
        let id = node.get::<EntityId>().unwrap().get();
        let cfg = node.get::<JointControllerConfiguration>().unwrap();
        let handle = node.get::<JointHandle>().unwrap();
        let definition = node.get::<JointDefinition>().unwrap();
        let child = entity_by_id(server.simulation.world(), handle.child()).unwrap();
        let child_node = server.simulation.world().entity(child);
        assert_eq!(
            node.get::<Name>().unwrap().0,
            format!("default::arm_right::{}", cfg.joint_name)
        );
        assert_eq!(definition.0.name, cfg.joint_name);
        assert_eq!(
            child_node.get::<Name>().unwrap().0,
            format!("default::arm_right::{}", definition.0.child)
        );
        assert_eq!(
            server
                .simulation
                .world()
                .resource::<gz_physics::PhysicsBackend>()
                .joint_child(handle.id()),
            Some(child_node.get::<BodyHandle>().unwrap().id())
        );
        assert!(!gz_sim_scene::joint_controller_binding_matches(
            &server.simulation,
            id
        ));
    }
    assert!(boundary_keys(&server.simulation, &server.network_config, None).is_err());
    assert!(prepare_reset(&server).is_err());
    assert!(
        prepare(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &initial(&server.simulation).unwrap()
        )
        .is_err()
    );
    assert_eq!(snapshot(&server), before);
    assert_eq!(physical(&server.simulation, &owners), native_before);
    swap_same_owner_native_bindings(&mut server);
    prepare_reset(&server).unwrap();
    assert_eq!(snapshot(&server), before);

    // A genuine admitted graph can also contain an unconfigured joint. Forging
    // its public configuration does not create the private Scene Controller.
    let right = XML.find("<model name=\"arm_right\"").unwrap();
    let mut xml = XML.to_owned();
    let start = right
        + xml[right..]
            .find("<plugin filename=\"gz-sim-joint-position-controller-system\"")
            .unwrap();
    let end = start + xml[start..].find("</plugin>").unwrap() + "</plugin>".len();
    xml.replace_range(start..end, "");
    let tail = xml[right..].replacen(
        "<limit><effort>3</effort><velocity>4</velocity></limit>",
        "",
        1,
    );
    xml.replace_range(right.., &tail);
    let (mut server, _observer) = build(&xml);
    let node = server
        .simulation
        .world()
        .iter_entities()
        .find(|node| {
            node.get::<Name>()
                .is_some_and(|name| name.0 == "default::arm_right::base_hinge")
        })
        .unwrap();
    let entity = node.id();
    let id = node.get::<EntityId>().unwrap().get();
    assert!(node.contains::<JointHandle>() && !node.contains::<JointControllerConfiguration>());
    assert!(!gz_sim_scene::joint_controller_binding_matches(
        &server.simulation,
        id
    ));
    let mut forged = server
        .simulation
        .world()
        .get::<JointControllerConfiguration>(selected(
            &server,
            "arm_right",
            JointControllerMode::Velocity,
        ))
        .unwrap()
        .clone();
    forged.joint_name = "base_hinge".into();
    forged.mode = JointControllerMode::Position;
    forged.topic = "/model/arm_right/joint/base_hinge/0/cmd_pos".into();
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(forged);
    let before = snapshot(&server);
    assert!(!gz_sim_scene::joint_controller_binding_matches(
        &server.simulation,
        id
    ));
    assert!(boundary_keys(&server.simulation, &server.network_config, None).is_err());
    assert_eq!(snapshot(&server), before);
}

#[test]
fn prepared_and_retained_distinct_double_arc_swaps_and_duplicates_are_rejected() {
    let (mut server, _observer) = build(XML);
    let owners = candidates(&server.simulation, &server.network_config).unwrap();
    let mut leases: Vec<_> = server
        .joint_controllers
        .as_ref()
        .unwrap()
        .streams
        .iter()
        .map(|stream| Arc::clone(&stream._lease))
        .collect();
    prepared(&owners, &leases).unwrap();
    // A real four-input handoff cannot silently omit one declared lease.
    let before_arity = snapshot(&server);
    let physical_owners: Vec<_> = owners
        .iter()
        .map(|owner| (owner.entity, owner.joint))
        .collect();
    let native_before_arity = physical(&server.simulation, &physical_owners);
    let lease_counts = || {
        server
            .joint_controllers
            .as_ref()
            .unwrap()
            .streams
            .iter()
            .map(|stream| Arc::strong_count(&stream._lease))
            .collect::<Vec<_>>()
    };
    let counts_before_arity = lease_counts();
    let missing = leases.pop().unwrap();
    assert_eq!(leases.len() + 1, owners.len());
    assert!(matches!(
        prepared(&owners, &leases),
        Err(Error::JointControllerIdentity)
    ));
    assert_eq!(snapshot(&server), before_arity);
    assert_eq!(
        physical(&server.simulation, &physical_owners),
        native_before_arity
    );
    assert_eq!(lease_counts(), counts_before_arity);
    leases.push(missing);
    prepared(&owners, &leases).unwrap();
    assert_eq!(snapshot(&server), before_arity);
    assert_eq!(
        physical(&server.simulation, &physical_owners),
        native_before_arity
    );
    assert_eq!(lease_counts(), counts_before_arity);
    assert!(!Arc::ptr_eq(&leases[0], &leases[2]));
    leases.swap(0, 2);
    assert!(matches!(
        prepared(&owners, &leases),
        Err(Error::JointControllerIdentity)
    ));
    leases.swap(0, 2);
    let original = Arc::clone(&leases[2]);
    leases[2] = Arc::clone(&leases[0]);
    assert!(matches!(
        prepared(&owners, &leases),
        Err(Error::JointControllerIdentity)
    ));
    leases[2] = original;
    prepared(&owners, &leases).unwrap();
    drop(leases);
    let before = snapshot(&server);
    {
        let streams = &mut server.joint_controllers.as_mut().unwrap().streams;
        let (first, rest) = streams.split_at_mut(2);
        std::mem::swap(&mut first[0]._lease, &mut rest[0]._lease);
    }
    assert!(matches!(
        prepare_reset(&server),
        Err(Error::JointControllerIdentity)
    ));
    {
        let streams = &mut server.joint_controllers.as_mut().unwrap().streams;
        let (first, rest) = streams.split_at_mut(2);
        std::mem::swap(&mut first[0]._lease, &mut rest[0]._lease);
    }
    let original = Arc::clone(&server.joint_controllers.as_ref().unwrap().streams[2]._lease);
    server.joint_controllers.as_mut().unwrap().streams[2]._lease =
        Arc::clone(&server.joint_controllers.as_ref().unwrap().streams[0]._lease);
    assert!(matches!(
        prepare_reset(&server),
        Err(Error::JointControllerIdentity)
    ));
    server.joint_controllers.as_mut().unwrap().streams[2]._lease = original;
    prepare_reset(&server).unwrap();
    assert_eq!(snapshot(&server), before);
}

#[test]
fn remapped_reset_validates_old_keys_then_restores_removed_originals_with_fresh_epochs() {
    let (mut server, _observer) = build(XML);
    server.set_paused(true);
    let expected = initial(&server.simulation).unwrap();
    let mut inputs = Vec::new();
    let mut old = Vec::new();
    for (index, stream) in server
        .joint_controllers
        .as_mut()
        .unwrap()
        .streams
        .iter_mut()
        .enumerate()
    {
        let (input, receiver) = mpsc::sync_channel(2);
        stream.receiver = receiver;
        input.try_send((0, Some(1.0))).unwrap();
        inputs.push(input);
        old.push(Arc::downgrade(&stream._lease));
        server.network_config.topic_remaps.insert(
            stream.metadata.topic.clone(),
            format!("/m58/remapped/{index}"),
        );
    }
    let before = snapshot(&server);
    assert_eq!(prepare_reset(&server).unwrap(), expected);
    assert_eq!(snapshot(&server), before);
    reset(&mut server);
    assert_eq!(server.simulation.epoch(), 1);
    assert!(old.iter().all(|lease| lease.upgrade().is_none()));
    assert!(
        inputs
            .iter()
            .all(|input| input.try_send((0, Some(1.0))).is_err())
    );
    assert_eq!(server.counters.joint_controller_commands_applied, 0);
    for stream in &server.joint_controllers.as_ref().unwrap().streams {
        assert_eq!(
            stream.endpoint,
            server
                .network_config
                .topic_key(&stream.metadata.topic)
                .unwrap()
        );
        assert!(stream._lease.matches_endpoint(&stream.endpoint));
    }
    let model = server
        .joint_controllers
        .as_ref()
        .unwrap()
        .streams
        .iter()
        .find(|stream| stream.metadata.model_name == "arm_left")
        .unwrap()
        .parent;
    let old: Vec<_> = server
        .joint_controllers
        .as_ref()
        .unwrap()
        .streams
        .iter()
        .filter(|stream| stream.parent == model)
        .map(|stream| Arc::downgrade(&stream._lease))
        .collect();
    remove_model(&mut server.simulation, model).unwrap();
    server.reconcile_joint_controllers().unwrap();
    assert_eq!(server.joint_controllers.as_ref().unwrap().streams.len(), 2);
    assert!(old.iter().all(|lease| lease.upgrade().is_none()));
    assert!(!server.reset_failed);
    assert_eq!(prepare_reset(&server).unwrap(), expected);
    reset(&mut server);
    assert_eq!(server.simulation.epoch(), 2);
    assert_eq!(server.joint_controllers.as_ref().unwrap().streams.len(), 4);
    assert_eq!(initial(&server.simulation).unwrap(), expected);
    assert_eq!(server.counters.joint_controller_commands_applied, 0);
}

#[test]
fn each_owner_keeps_its_budget_epoch_rejections_and_last_finite_target() {
    let (mut server, _observer) = build(XML);
    let mut inputs = Vec::new();
    for stream in &mut server.joint_controllers.as_mut().unwrap().streams {
        let value = if stream.metadata.model_name == "arm_left" {
            0.4
        } else {
            -0.4
        };
        let (input, receiver) = mpsc::sync_channel(4);
        stream.receiver = receiver;
        for command in [
            (0, Some(value)),
            (0, None),
            (1, Some(3.0)),
            (0, Some(f64::NAN)),
        ] {
            input.try_send(command).unwrap();
        }
        inputs.push(input);
    }
    server.poll_joint_controllers().unwrap();
    assert_eq!(server.counters.joint_controller_messages_received, 8);
    assert_eq!(server.counters.joint_controller_commands_applied, 4);
    assert_eq!(server.counters.joint_controller_messages_rejected, 4);
    server.poll_joint_controllers().unwrap();
    assert_eq!(server.counters.joint_controller_messages_received, 16);
    assert_eq!(server.counters.joint_controller_commands_applied, 4);
    assert_eq!(server.counters.joint_controller_messages_rejected, 12);
    server.set_paused(false);
    for _ in 0..1000 {
        server.update().unwrap();
    }
    for (model, target) in [("arm_left", 0.4), ("arm_right", -0.4)] {
        let (q, v) = joint_state(
            &server,
            selected(&server, model, JointControllerMode::Position),
        );
        assert!(q.is_finite() && v.is_finite() && (q - target).abs() < 0.1);
        let (q, v) = joint_state(
            &server,
            selected(&server, model, JointControllerMode::Velocity),
        );
        assert!(q * target > 0.03 && v * target > 0.03 && q.is_finite() && v.is_finite());
    }
    assert_eq!(server.counters.joint_controller_commands_applied, 4);
    drop(inputs);
}

#[test]
fn a_live_late_owner_fault_prevents_every_queue_read_and_retires_all_leases() {
    for coordinated_swap in [false, true] {
        let (mut server, _observer) = build(XML);
        let mut inputs = Vec::new();
        let mut old = Vec::new();
        for stream in &mut server.joint_controllers.as_mut().unwrap().streams {
            let (input, receiver) = mpsc::sync_channel(1);
            stream.receiver = receiver;
            input.try_send((0, Some(0.5))).unwrap();
            inputs.push(input);
            old.push(Arc::downgrade(&stream._lease));
        }
        let owners: Vec<_> = server
            .joint_controllers
            .as_ref()
            .unwrap()
            .streams
            .iter()
            .map(|stream| (stream.entity, stream.joint))
            .collect();
        let native_before = physical(&server.simulation, &owners);
        if coordinated_swap {
            swap_same_owner_native_bindings(&mut server);
        } else {
            let entity = selected(&server, "arm_right", JointControllerMode::Velocity);
            server
                .simulation
                .world_mut()
                .entity_mut(entity)
                .remove::<ChildOf>();
        }
        assert!(matches!(
            server.poll_joint_controllers(),
            Err(Error::JointControllerIdentity)
        ));
        assert_eq!(server.counters.joint_controller_messages_received, 0);
        assert_eq!(server.counters.joint_controller_commands_applied, 0);
        assert_eq!(server.simulation.elapsed(), Duration::ZERO);
        assert_eq!(physical(&server.simulation, &owners), native_before);
        assert!(server.joint_controllers.is_none() && server.reset_failed);
        assert!(old.iter().all(|lease| lease.upgrade().is_none()));
        assert!(
            inputs
                .iter()
                .all(|input| input.try_send((0, Some(0.5))).is_err())
        );
        assert!(matches!(server.update(), Err(Error::ResetStreams)));
        assert!(!server.apply_local_control(WorldControl::default()));
    }
}

#[test]
fn four_distinct_double_declarations_are_bounded_and_a_fifth_never_installs() {
    let (server, _observer) = build(include_str!("../../../../examples/mvp_primitives.sdf"));
    let setup = |count| {
        let mut endpoints = Vec::new();
        let mut receivers = Vec::new();
        for slot in 0..count {
            let (input, receiver) = mpsc::sync_channel(1);
            endpoints.push(Endpoint {
                key: server
                    .network_config
                    .topic_key(&format!("/m58/input/{slot}"))
                    .unwrap(),
                input,
            });
            receivers.push(receiver);
        }
        (
            Setup {
                endpoints,
                max_bytes: 64,
                drops: Arc::clone(&server.world_io.ingress_drops),
                epoch: Arc::clone(&server.world_io.epoch),
            },
            receivers,
        )
    };
    let (invalid, _held) = setup(5);
    assert!(server.world_io.joint_controller_inputs(invalid).is_err());
    let (valid, _held) = setup(4);
    let leases = server.world_io.joint_controller_inputs(valid).unwrap();
    assert_eq!(leases.len(), 4);
    for (slot, lease) in leases.iter().enumerate() {
        assert!(
            lease.matches_endpoint(
                &server
                    .network_config
                    .topic_key(&format!("/m58/input/{slot}"))
                    .unwrap()
            )
        );
    }
    let weak: Vec<_> = leases.iter().map(Arc::downgrade).collect();
    drop(leases);
    wait(|| weak.iter().all(|lease| lease.upgrade().is_none()));
}

#[test]
fn restored_metadata_drift_keeps_identity_and_peer_errors_before_reset_classification() {
    const POSITION: &str = "/model/arm_left/joint/base_hinge/0/cmd_pos";
    for fault in [
        "free-topic",
        "relative-topic",
        "clock",
        "duplicate-topic",
        "removed-owner-key",
        "kind",
        "handle",
        "public-missing",
        "expected",
    ] {
        let (mut server, _observer) = build(XML);
        let mut expected = crate::joint_controllers::initial(&server.simulation).unwrap();
        let entity = server
            .simulation
            .world()
            .iter_entities()
            .find(|node| {
                node.get::<JointControllerConfiguration>()
                    .is_some_and(|m| m.topic == POSITION)
            })
            .unwrap()
            .id();
        let other_topic = expected
            .values()
            .find(|m| m.topic != POSITION)
            .unwrap()
            .topic
            .clone();
        let mut changed = server
            .simulation
            .world()
            .get::<JointControllerConfiguration>(entity)
            .unwrap()
            .clone();
        let original_topic = changed.topic.clone();
        let removed_topic = if fault == "removed-owner-key" {
            let other = expected
                .values()
                .find(|m| m.model_name != changed.model_name)
                .unwrap();
            let model_name = format!("default::{}", other.model_name);
            let topic = other.topic.clone();
            let model = server
                .simulation
                .world()
                .iter_entities()
                .find(|node| node.get::<Name>().is_some_and(|name| name.0 == model_name))
                .unwrap()
                .get::<EntityId>()
                .unwrap()
                .get();
            remove_model(&mut server.simulation, model).unwrap();
            Some(topic)
        } else {
            None
        };
        changed.topic = match fault {
            "relative-topic" => "relative".into(),
            "clock" => "/clock".into(),
            "duplicate-topic" => other_topic,
            "removed-owner-key" => removed_topic.unwrap(),
            "expected" => original_topic,
            _ => "/m58/reset/metadata_drift".into(),
        };
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(changed);
        if fault == "kind" {
            server
                .simulation
                .world_mut()
                .entity_mut(entity)
                .insert(EntityKind::Link);
        } else if fault == "handle" {
            server
                .simulation
                .world_mut()
                .entity_mut(entity)
                .remove::<gz_sim_scene::JointHandle>();
        } else if fault == "public-missing" {
            server
                .simulation
                .world_mut()
                .entity_mut(entity)
                .remove::<JointControllerConfiguration>();
        } else if fault == "expected" {
            expected
                .values_mut()
                .next()
                .unwrap()
                .topic
                .push_str("/unrequested");
        }
        let before = crate::multiple_joint_controller_boundaries::snapshot(&server);
        let leases = |server: &Server| {
            server
                .joint_controllers
                .as_ref()
                .unwrap()
                .streams
                .iter()
                .map(|stream| {
                    (
                        Arc::as_ptr(&stream._lease) as usize,
                        Arc::strong_count(&stream._lease),
                    )
                })
                .collect::<Vec<_>>()
        };
        let old_leases = leases(&server);
        if fault == "free-topic" {
            assert!(matches!(
                crate::joint_controllers::prepare(
                    &server.simulation,
                    &server.world_io,
                    &server.network_config,
                    &expected
                ),
                Err(Error::JointControllerIdentity)
            ));
        }
        let result = crate::joint_controllers::prepare_restored(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            &expected,
        );
        if matches!(fault, "free-topic" | "expected") {
            assert!(matches!(result, Err(Error::ResetJointControllers)));
        } else {
            assert!(matches!(result, Err(Error::JointControllerIdentity)));
        }
        assert_eq!(
            crate::multiple_joint_controller_boundaries::snapshot(&server),
            before
        );
        assert_eq!(leases(&server), old_leases);
    }
}

#[test]
fn restored_legacy_channels_survive_real_model_removal_without_treating_absence_as_vacancy() {
    let mut config = ServerOptions {
        partition: format!("m58-legacy-reservations-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    };
    config
        .add_topic_remap("/m33/imu/set_rate", "/m58/restored/imu_rate")
        .unwrap();
    config
        .add_topic_remap("/model/rotor/joint/hinge/cmd_force", "/m58/restored/force")
        .unwrap();
    let clock_only = BTreeSet::from([config.topic_key("/clock").unwrap()]);
    // Preserve M33's file-URI mesh resolution and seeded loading contract.
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/m33_robot_sensors.sdf")
        .canonicalize()
        .unwrap();
    let mut sensors = gz_sim_scene::load_world_seeded(
        include_str!("../../../../examples/m33_robot_sensors.sdf"),
        &format!("file://{}", path.display()),
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        [33; 32],
    )
    .unwrap();
    assert_eq!(
        gz_sim_scene::initial_imu_publications(&sensors)
            .unwrap()
            .count(),
        1
    );
    assert_eq!(
        gz_sim_scene::initial_lidar_publications(&sensors)
            .unwrap()
            .count(),
        1
    );
    assert_eq!(
        gz_sim_scene::initial_diff_drive_configurations(&sensors)
            .unwrap()
            .count(),
        1
    );
    let sensor_keys: BTreeSet<_> = [
        "/clock",
        "/m33/imu",
        "/m33/imu/set_rate",
        "/m33/scan",
        "/model/robot/cmd_vel",
        "/model/robot/odometry",
    ]
    .into_iter()
    .map(|topic| config.topic_key(topic).unwrap())
    .collect();
    assert_eq!(
        legacy_occupied(&sensors, &config, false).unwrap(),
        sensor_keys
    );
    assert_eq!(
        legacy_occupied(&sensors, &config, true).unwrap(),
        sensor_keys
    );
    let model = sensors
        .world()
        .iter_entities()
        .find(|node| {
            node.get::<Name>()
                .is_some_and(|name| name.0 == "default::robot")
                && node.get::<EntityKind>() == Some(&EntityKind::Model)
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    assert!(remove_model(&mut sensors, model).unwrap().contains(&model));
    assert!(entity_by_id(sensors.world(), model).is_none());
    assert_eq!(
        legacy_occupied(&sensors, &config, false).unwrap(),
        clock_only
    );
    assert_eq!(
        legacy_occupied(&sensors, &config, true).unwrap(),
        sensor_keys
    );
    assert_eq!(sensors.elapsed(), Duration::ZERO);

    let mut force = loaded(include_str!(
        "../../../../examples/m42_joint_force_feedback.sdf"
    ));
    assert_eq!(
        gz_sim_scene::initial_joint_force_configurations(&force)
            .unwrap()
            .count(),
        1
    );
    let force_keys = BTreeSet::from([
        config.topic_key("/clock").unwrap(),
        config
            .topic_key("/model/rotor/joint/hinge/cmd_force")
            .unwrap(),
    ]);
    assert_eq!(legacy_occupied(&force, &config, false).unwrap(), force_keys);
    assert_eq!(legacy_occupied(&force, &config, true).unwrap(), force_keys);
    let model = force
        .world()
        .iter_entities()
        .find(|node| {
            node.get::<Name>()
                .is_some_and(|name| name.0 == "default::rotor")
                && node.get::<EntityKind>() == Some(&EntityKind::Model)
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    assert!(remove_model(&mut force, model).unwrap().contains(&model));
    assert!(entity_by_id(force.world(), model).is_none());
    assert_eq!(legacy_occupied(&force, &config, false).unwrap(), clock_only);
    assert_eq!(legacy_occupied(&force, &config, true).unwrap(), force_keys);
    assert_eq!(force.elapsed(), Duration::ZERO);

    let bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        gz_sim_scene::initial_imu_publications(&bare),
        Err(gz_sim_scene::SceneResetError::MissingInitialState)
    ));
    assert_eq!(legacy_occupied(&bare, &config, true).unwrap(), clock_only);
    assert_eq!(legacy_occupied(&bare, &config, false).unwrap(), clock_only);
}
