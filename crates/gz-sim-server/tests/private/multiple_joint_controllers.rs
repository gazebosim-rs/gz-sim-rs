// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::msgs::WorldReset;
use gz_sim_scene::{
    JointControllerMode, JointSample, PhysicsEngine, load_world_with_sdf_step, remove_model,
};
use std::time::Instant;
const XML: &str = include_str!("../../../../examples/m58_multiple_joint_controllers.sdf");
// Complete admitted M58 graph, with names and model placement changed only.
// In a one-slot model the unconfigured joint must retain infinite limits.
fn fleet_xml(count: usize, slots: usize) -> String {
    assert!((1..=2).contains(&slots));
    let start = XML.find("<model name=\"arm_left\"").unwrap();
    let end = start + XML[start..].find("</model>").unwrap() + "</model>".len();
    let mut model = XML[start..end].to_owned();
    if slots == 1 {
        let start = model
            .find("<plugin filename=\"gz-sim-joint-controller-system\"")
            .unwrap();
        let end = start + model[start..].find("</plugin>").unwrap() + "</plugin>".len();
        model.replace_range(start..end, "");
        let start = model.find("<joint name=\"tip_hinge\"").unwrap();
        let end = start + model[start..].find("</joint>").unwrap() + "</joint>".len();
        let joint = model[start..end].replace(
            "<limit><effort>3</effort><velocity>4</velocity></limit>",
            "",
        );
        model.replace_range(start..end, &joint);
    }
    let right = XML.find("<model name=\"arm_right\"").unwrap();
    let right_end = right + XML[right..].find("</model>").unwrap() + "</model>".len();
    let mut models = String::new();
    for index in 0..count {
        models.push_str(&model.replace("arm_left", &format!("arm_{index}")).replace(
            "<pose>0 -1.5 1 0 0 0</pose>",
            &format!(
                "<pose>{} {} 1 0 0 0</pose>",
                if index.is_multiple_of(2) { -1.5 } else { 1.5 },
                -4.5 + (index / 2) as f64 * 3.0
            ),
        ));
    }
    (XML[..start].to_owned() + &XML[end..right] + &XML[right_end..])
        .replace("</world>", &(models + "</world>"))
}
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
fn physical(sim: &Simulation, owners: &[(Entity, PhysicalJoint)]) -> Vec<JointSnapshot> {
    owners
        .iter()
        .map(|(entity, joint)| JointSnapshot {
            native: sim
                .world()
                .resource::<gz_physics::PhysicsBackend>()
                .joint_state(joint.native.expect("Rapier fixture"))
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
    let (invalid, _held) = setup(17);
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

#[test]
fn genuine_controller_fleets_preserve_zero_one_two_three_seven_eight_and_refuse_ninth() {
    for slots in [1, 2] {
        for count in [0, 1, 2, 3, 7, 8] {
            let (server, _observer) = build(&fleet_xml(count, slots));
            let before = snapshot(&server);
            let expected = initial(&server.simulation).unwrap();
            assert_eq!(expected.len(), count * slots);
            assert_eq!(
                preflight(&server.simulation, &server.world_io, &server.network_config).unwrap(),
                expected
            );
            assert_eq!(prepare_reset(&server).unwrap(), expected);
            let streams = server
                .joint_controllers
                .as_ref()
                .map_or(&[][..], |c| c.streams.as_slice());
            assert_eq!(streams.len(), count * slots);
            assert_eq!(
                streams
                    .iter()
                    .map(|s| s.endpoint.as_str())
                    .collect::<BTreeSet<_>>()
                    .len(),
                count * slots
            );
            assert_eq!(
                streams
                    .iter()
                    .map(|s| s.parent)
                    .collect::<BTreeSet<_>>()
                    .len(),
                count
            );
            assert_eq!(snapshot(&server), before);
        }
    }
    // A ninth genuine one-controller model exceeds owner count with only nine inputs.
    assert!(
        load_world_with_sdf_step(
            &fleet_xml(9, 1),
            "m62-nine.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
    let (server, _observer) = build(&fleet_xml(8, 1));
    let metadata = initial(&server.simulation).unwrap();
    let mut groups = BTreeMap::new();
    for (index, cfg) in metadata.values().enumerate() {
        group(&mut groups, index as u64 + 1, cfg).unwrap();
    }
    let mut ninth = metadata.values().next().unwrap().clone();
    ninth.model_name = "arm_ninth".into();
    assert!(matches!(
        group(&mut groups, 9, &ninth),
        Err(Error::JointControllerIdentity)
    ));
}

#[test]
fn all_sixteen_original_controller_bindings_preflight_before_queue_or_lease_changes() {
    let (mut server, _observer) = build(&fleet_xml(8, 2));
    server.set_paused(true);
    let owners = candidates(&server.simulation, &server.network_config).unwrap();
    assert_eq!(owners.len(), 16);
    let physical_owners: Vec<_> = owners.iter().map(|o| (o.entity, o.joint)).collect();
    let native = physical(&server.simulation, &physical_owners);
    let last = owners.last().unwrap();
    let entity = last.entity;
    let original_name = server
        .simulation
        .world()
        .get::<Name>(entity)
        .unwrap()
        .0
        .clone();
    let original_id = gz_sim_core::reserve_id(server.simulation.world(), entity).unwrap();
    let original_handle = *server
        .simulation
        .world()
        .get::<JointHandle>(entity)
        .unwrap();
    let foreign_handle = *server
        .simulation
        .world()
        .get::<JointHandle>(owners[0].entity)
        .unwrap();
    for fault in 0..3 {
        match fault {
            0 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(Name("foreign::joint".into()));
            }
            1 => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .remove::<EntityId>();
            }
            _ => {
                server
                    .simulation
                    .world_mut()
                    .entity_mut(entity)
                    .insert(foreign_handle);
            }
        }
        let before = snapshot(&server);
        assert!(matches!(
            preflight(&server.simulation, &server.world_io, &server.network_config),
            Err(Error::JointControllerIdentity)
        ));
        assert!(matches!(
            prepare_reset(&server),
            Err(Error::JointControllerIdentity)
        ));
        assert_eq!(snapshot(&server), before);
        assert_eq!(physical(&server.simulation, &physical_owners), native);
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert((Name(original_name.clone()), original_handle));
        gz_sim_core::restore_id(server.simulation.world_mut(), entity, &original_id).unwrap();
    }
    let mut leases: Vec<_> = server
        .joint_controllers
        .as_ref()
        .unwrap()
        .streams
        .iter()
        .map(|s| Arc::clone(&s._lease))
        .collect();
    let last_lease = leases.pop().unwrap();
    let before = snapshot(&server);
    assert!(matches!(
        prepared(&owners, &leases),
        Err(Error::JointControllerIdentity)
    ));
    leases.push(Arc::clone(&leases[0]));
    assert!(matches!(
        prepared(&owners, &leases),
        Err(Error::JointControllerIdentity)
    ));
    assert!(leases.pop().is_some());
    leases.push(last_lease);
    prepared(&owners, &leases).unwrap();
    assert_eq!(snapshot(&server), before);
    assert_eq!(physical(&server.simulation, &physical_owners), native);
    let mut extra = owners[0].metadata.clone();
    extra.model_name = "counterfeit_seventeenth".into();
    extra.topic = "/m62/counterfeit".into();
    let extra = server
        .simulation
        .world_mut()
        .spawn((EntityKind::Joint, extra))
        .id();
    gz_sim_core::assign_id(server.simulation.world_mut(), extra).unwrap();
    let before = snapshot(&server);
    assert!(matches!(
        preflight(&server.simulation, &server.world_io, &server.network_config),
        Err(Error::JointControllerIdentity)
    ));
    assert_eq!(snapshot(&server), before);
    assert_eq!(physical(&server.simulation, &physical_owners), native);
}

#[test]
fn sixteen_controller_inputs_keep_closed_queue_budget_and_fair_retirement() {
    let (mut server, _observer) = build(&fleet_xml(8, 2));
    server.set_paused(true);
    let epoch = server.simulation.epoch();
    let mut inputs = Vec::new();
    for stream in &mut server.joint_controllers.as_mut().unwrap().streams {
        let (input, receiver) = mpsc::sync_channel(4);
        for command in [
            (epoch, None),
            (epoch.wrapping_add(1), Some(1.0)),
            (epoch, Some(f64::NAN)),
            (epoch, Some(0.25)),
        ] {
            input.try_send(command).unwrap();
        }
        assert!(matches!(
            input.try_send((epoch, None)),
            Err(mpsc::TrySendError::Full(_))
        ));
        stream.receiver = receiver;
        inputs.push(input);
    }
    let owners: Vec<_> = candidates(&server.simulation, &server.network_config)
        .unwrap()
        .iter()
        .map(|o| (o.entity, o.joint))
        .collect();
    let native = physical(&server.simulation, &owners);
    server.poll_joint_controllers().unwrap();
    assert_eq!(server.counters.joint_controller_messages_received, 32);
    assert_eq!(server.counters.joint_controller_messages_rejected, 32);
    assert_eq!(server.counters.joint_controller_commands_applied, 0);
    assert_eq!(physical(&server.simulation, &owners), native);
    server.poll_joint_controllers().unwrap();
    assert_eq!(server.counters.joint_controller_messages_received, 64);
    assert_eq!(server.counters.joint_controller_messages_rejected, 48);
    assert_eq!(server.counters.joint_controller_commands_applied, 16);
    assert_eq!(physical(&server.simulation, &owners), native);
    let retired = server
        .joint_controllers
        .as_ref()
        .unwrap()
        .streams
        .last()
        .unwrap()
        .parent;
    let old: Vec<_> = server
        .joint_controllers
        .as_ref()
        .unwrap()
        .streams
        .iter()
        .filter(|s| s.parent == retired)
        .map(|s| Arc::downgrade(&s._lease))
        .collect();
    remove_model(&mut server.simulation, retired).unwrap();
    server.reconcile_joint_controllers().unwrap();
    assert_eq!(server.joint_controllers.as_ref().unwrap().streams.len(), 14);
    assert!(old.iter().all(|weak| weak.upgrade().is_none()));
    let mut surviving = 0;
    let mut disconnected = 0;
    for input in &inputs {
        match input.try_send((epoch, Some(0.0))) {
            Ok(()) => surviving += 1,
            Err(mpsc::TrySendError::Disconnected(_)) => disconnected += 1,
            Err(mpsc::TrySendError::Full(_)) => panic!("drained fleet queue still full"),
        }
    }
    assert_eq!((surviving, disconnected), (14, 2));
    server.poll_joint_controllers().unwrap();
    assert_eq!(server.counters.joint_controller_messages_received, 78);
    assert_eq!(server.counters.joint_controller_commands_applied, 30);
    assert_eq!(server.simulation.elapsed(), Duration::ZERO);
}

#[test]
fn eight_controller_owners_reset_and_retire_with_all_original_keys_reserved() {
    let (mut server, _observer) = build(&fleet_xml(8, 2));
    server.set_paused(true);
    let expected = initial(&server.simulation).unwrap();
    let mut old = Vec::new();
    let mut inputs = Vec::new();
    for (index, stream) in server
        .joint_controllers
        .as_mut()
        .unwrap()
        .streams
        .iter_mut()
        .enumerate()
    {
        let (input, receiver) = mpsc::sync_channel(1);
        input
            .try_send((server.simulation.epoch(), Some(1.0)))
            .unwrap();
        stream.receiver = receiver;
        inputs.push(input);
        old.push(Arc::downgrade(&stream._lease));
        server
            .network_config
            .topic_remaps
            .insert(stream.metadata.topic.clone(), format!("/m62/reset/{index}"));
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
    let streams = &server.joint_controllers.as_ref().unwrap().streams;
    assert_eq!(streams.len(), 16);
    for stream in streams {
        assert_eq!(
            stream.endpoint,
            server
                .network_config
                .topic_key(&stream.metadata.topic)
                .unwrap()
        );
        assert!(stream._lease.matches_endpoint(&stream.endpoint));
    }
    let model = streams.last().unwrap().parent;
    let retired_topics: Vec<_> = streams
        .iter()
        .filter(|s| s.parent == model)
        .map(|s| s.metadata.topic.clone())
        .collect();
    let retired: Vec<_> = streams
        .iter()
        .filter(|s| s.parent == model)
        .map(|s| Arc::downgrade(&s._lease))
        .collect();
    assert_eq!(retired_topics.len(), 2);
    remove_model(&mut server.simulation, model).unwrap();
    server.reconcile_joint_controllers().unwrap();
    assert_eq!(server.joint_controllers.as_ref().unwrap().streams.len(), 14);
    assert!(retired.iter().all(|weak| weak.upgrade().is_none()));
    assert_eq!(prepare_reset(&server).unwrap(), expected);
    let first = &server.joint_controllers.as_ref().unwrap().streams[0];
    let entity = first.entity;
    let original = first.metadata.clone();
    let owners: Vec<_> = candidates(&server.simulation, &server.network_config)
        .unwrap()
        .iter()
        .map(|o| (o.entity, o.joint))
        .collect();
    let native = physical(&server.simulation, &owners);
    for topic in retired_topics {
        let mut changed = original.clone();
        changed.topic = topic;
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(changed);
        let before = snapshot(&server);
        assert!(matches!(
            prepare_restored(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                &expected
            ),
            Err(Error::JointControllerIdentity)
        ));
        assert_eq!(snapshot(&server), before);
        assert_eq!(physical(&server.simulation, &owners), native);
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(original.clone());
    }
    reset(&mut server);
    assert_eq!(server.simulation.epoch(), 2);
    assert_eq!(server.joint_controllers.as_ref().unwrap().streams.len(), 16);
    assert_eq!(initial(&server.simulation).unwrap(), expected);
    assert_eq!(server.counters.joint_controller_commands_applied, 0);
}

#[test]
fn maximum_drive_and_controller_fleets_keep_disjoint_ingress_and_loaded_reset() {
    use gz_sim_proto::prost::{Message, Name as ProtoName};
    use gz_sim_scene::{BodyHandle, DiffDriveSample, WorldPose};
    let drive_world = include_str!("../../../../examples/m61_eight_diff_drives.sdf");
    let controller_world = include_str!("../../../../examples/m62_eight_joint_controllers.sdf");
    assert!(drive_world.contains("<gravity>0 0 -9.81</gravity>"));
    let mut controllers = String::new();
    for owner in 0..8 {
        let start = controller_world
            .find(&format!("<model name=\"arm_{owner}\""))
            .unwrap();
        let end = start + controller_world[start..].find("</model>").unwrap() + "</model>".len();
        let model = &controller_world[start..end];
        let pose_start = model.find("<pose>").unwrap() + "<pose>".len();
        let pose_end = pose_start + model[pose_start..].find("</pose>").unwrap();
        let pose: Vec<f64> = model[pose_start..pose_end]
            .split_whitespace()
            .map(|v| v.parse().unwrap())
            .collect();
        assert_eq!(pose.len(), 6);
        assert_eq!(&pose[2..], &[1.0, 0.0, 0.0, 0.0]);
        controllers.push_str(
            &(model[..pose_start].to_owned()
                + &format!("{} {} 1 0 0 0", pose[0] + 20.0, pose[1])
                + &model[pose_end..]),
        );
    }
    let combined = drive_world.replace("</world>", &(controllers + "</world>"));
    let (mut server, observer) = build(&combined);
    server.set_paused(true);
    let drive_originals: Vec<_> =
        gz_sim_scene::initial_diff_drive_configurations_with_identity(&server.simulation)
            .unwrap()
            .map(|row| (row.id, row.configuration.clone()))
            .collect();
    let controller_originals = initial(&server.simulation).unwrap();
    assert_eq!(drive_originals.len(), 8);
    assert_eq!(controller_originals.len(), 16);
    let controller_parents: BTreeSet<_> = candidates(&server.simulation, &server.network_config)
        .unwrap()
        .iter()
        .map(|row| row.parent)
        .collect();
    assert_eq!(controller_parents.len(), 8);
    let drive_keys =
        crate::diff_drive::boundary_keys(&server.simulation, &server.network_config, None).unwrap();
    let controller_keys = boundary_keys(&server.simulation, &server.network_config, None).unwrap();
    assert_eq!((drive_keys.len(), controller_keys.len()), (16, 16));
    assert!(drive_keys.is_disjoint(&controller_keys));
    assert_eq!(drive_keys.union(&controller_keys).count(), 32);
    assert!(
        crate::diff_drive_reserved_keys(
            &server.simulation,
            &server.world_io,
            &server.network_config
        )
        .unwrap()
        .is_superset(&controller_keys)
    );
    assert!(
        crate::joint_controller_reserved_keys(
            &server.simulation,
            &server.world_io,
            &server.network_config
        )
        .unwrap()
        .is_superset(&drive_keys)
    );
    let joints: Vec<_> = server
        .simulation
        .world()
        .iter_entities()
        .filter_map(|node| node.get::<JointHandle>().map(|handle| (node.id(), *handle)))
        .collect();
    assert_eq!(joints.len(), 32);
    // Read the actual backend, every sample and all world poses; target receipt may
    // change counters but cannot advance physics, samples, time or body coordinates.
    let held = |server: &Server| {
        let sim = &server.simulation;
        let backend = sim.world().resource::<gz_physics::PhysicsBackend>();
        let mut bodies = BTreeMap::new();
        let mut poses = BTreeMap::new();
        let mut samples = BTreeMap::new();
        for node in sim.world().iter_entities() {
            let Some(id) = node.get::<EntityId>().map(EntityId::get) else {
                continue;
            };
            if let Some(handle) = node.get::<BodyHandle>() {
                let state = backend.state(handle.id()).unwrap();
                bodies.insert(
                    id,
                    state
                        .position
                        .into_iter()
                        .chain(state.orientation)
                        .chain(state.velocity)
                        .chain(state.angular_velocity)
                        .map(f64::to_bits)
                        .collect::<Vec<_>>(),
                );
            }
            if let Some(pose) = node.get::<WorldPose>() {
                poses.insert(
                    id,
                    pose.translation
                        .into_iter()
                        .chain(pose.rotation)
                        .map(f64::to_bits)
                        .collect::<Vec<_>>(),
                );
            }
            if let Some(sample) = node.get::<DiffDriveSample>() {
                samples.insert(
                    id,
                    (
                        sample.epoch,
                        sample.time,
                        sample
                            .pose
                            .into_iter()
                            .chain(sample.twist)
                            .map(f64::to_bits)
                            .collect::<Vec<_>>(),
                    ),
                );
            }
        }
        let mut native_joints = BTreeMap::new();
        let mut joint_samples = BTreeMap::new();
        for node in sim.world().iter_entities() {
            if let Some(handle) = node.get::<JointHandle>() {
                let id = node.get::<EntityId>().unwrap().get();
                let state = backend.joint_state(handle.id()).unwrap();
                native_joints.insert(id, [state.position.to_bits(), state.velocity.to_bits()]);
                if let Some(sample) = node.get::<JointSample>() {
                    joint_samples.insert(
                        id,
                        (
                            sample.epoch,
                            sample.time,
                            [
                                sample.state.position.to_bits(),
                                sample.state.velocity.to_bits(),
                            ],
                        ),
                    );
                }
            }
        }
        (
            sim.epoch(),
            sim.elapsed(),
            native_joints,
            bodies,
            poses,
            samples,
            joint_samples,
        )
    };
    let initial_state = held(&server);
    let chassis = |server: &Server, owner: usize| {
        server
            .simulation
            .world()
            .iter_entities()
            .find(|node| {
                node.get::<Name>()
                    .is_some_and(|name| name.0 == format!("default::robot_{owner}::base_link"))
            })
            .unwrap()
            .get::<WorldPose>()
            .unwrap()
            .translation[0]
    };
    let start_x: [f64; 8] = std::array::from_fn(|owner| chassis(&server, owner));
    let drive_publishers: Vec<_> = drive_originals
        .iter()
        .map(|(_, cfg)| {
            observer
                .declare_publisher(server.network_config.topic_key(&cfg.topic).unwrap())
                .wait()
                .unwrap()
        })
        .collect();
    let controller_publishers: Vec<_> = controller_originals
        .values()
        .map(|cfg| {
            observer
                .declare_publisher(server.network_config.topic_key(&cfg.topic).unwrap())
                .wait()
                .unwrap()
        })
        .collect();
    assert_eq!(drive_publishers.len() + controller_publishers.len(), 24);
    wait(|| {
        drive_publishers
            .iter()
            .chain(&controller_publishers)
            .all(|p| p.matching_status().wait().unwrap().matching())
    });
    let twist = gz_sim_proto::msgs::Twist {
        linear: Some(vector3([0.25, 0.0, 0.0])),
        angular: Some(vector3([0.0; 3])),
        ..Default::default()
    }
    .encode_to_vec();
    for publisher in &drive_publishers {
        for _ in 0..3 {
            publisher
                .put(twist.clone())
                .attachment(gz_sim_proto::msgs::Twist::full_name())
                .wait()
                .unwrap();
        }
    }
    for (cfg, publisher) in controller_originals.values().zip(&controller_publishers) {
        let index: usize = cfg
            .model_name
            .strip_prefix("arm_")
            .unwrap()
            .parse()
            .unwrap();
        let value = if index.is_multiple_of(2) { 0.4 } else { -0.4 };
        for _ in 0..3 {
            publisher
                .put(
                    gz_sim_proto::msgs::Double {
                        data: value,
                        ..Default::default()
                    }
                    .encode_to_vec(),
                )
                .attachment(gz_sim_proto::msgs::Double::full_name())
                .wait()
                .unwrap();
        }
    }
    wait(|| {
        let before = (
            server.counters.diff_drive_messages_received,
            server.counters.joint_controller_messages_received,
        );
        assert!(server.update().unwrap().dt.is_zero());
        assert!(server.counters.diff_drive_messages_received - before.0 <= 2);
        assert!(server.counters.joint_controller_messages_received - before.1 <= 32);
        assert_eq!(held(&server), initial_state);
        server.counters.diff_drive_messages_received == 24
            && server.counters.joint_controller_messages_received == 48
    });
    assert_eq!(server.counters.diff_drive_commands_applied, 24);
    assert_eq!(server.counters.joint_controller_commands_applied, 48);
    assert_eq!(
        (
            server.counters.diff_drive_messages_rejected,
            server.counters.joint_controller_messages_rejected
        ),
        (0, 0)
    );
    server.set_paused(false);
    for step in 1..=3000 {
        let info = server.update().unwrap();
        assert_eq!(info.sim_time, Duration::from_millis(step));
        assert_eq!(info.dt, Duration::from_millis(1));
    }
    // Every joint in both families must expose a fresh real backend sample.
    let backend = server
        .simulation
        .world()
        .resource::<gz_physics::PhysicsBackend>();
    for (entity, handle) in &joints {
        let sample = server
            .simulation
            .world()
            .get::<JointSample>(*entity)
            .unwrap();
        let state = backend.joint_state(handle.id()).unwrap();
        assert_eq!(
            (sample.time, sample.epoch),
            (server.simulation.elapsed(), server.simulation.epoch())
        );
        assert_eq!(
            [
                sample.state.position.to_bits(),
                sample.state.velocity.to_bits()
            ],
            [state.position.to_bits(), state.velocity.to_bits()]
        );
    }
    for (owner, &start) in start_x.iter().enumerate() {
        let displacement = chassis(&server, owner) - start;
        assert!(
            displacement > 0.05,
            "combined robot{owner} chassis displacement={displacement}"
        );
        let drive = server
            .simulation
            .world()
            .iter_entities()
            .find(|node| {
                node.get::<Name>()
                    .is_some_and(|name| name.0 == format!("default::robot_{owner}"))
            })
            .unwrap();
        let sample = drive.get::<DiffDriveSample>().unwrap();
        assert_eq!(
            (sample.time, sample.epoch),
            (server.simulation.elapsed(), server.simulation.epoch())
        );
        assert!(sample.twist[0] > 0.01);
        let model = format!("arm_{owner}");
        let target = if owner.is_multiple_of(2) { 0.4 } else { -0.4 };
        let (q, v) = joint_state(
            &server,
            selected(&server, &model, JointControllerMode::Position),
        );
        assert!(q.is_finite() && v.is_finite() && (q - target).abs() < 0.1);
        let (q, v) = joint_state(
            &server,
            selected(&server, &model, JointControllerMode::Velocity),
        );
        assert!(q * target > 0.03 && v * target > 0.03 && q.is_finite() && v.is_finite());
        let base = server
            .simulation
            .world()
            .iter_entities()
            .find(|node| {
                node.get::<Name>()
                    .is_some_and(|name| name.0 == format!("default::{model}::base"))
            })
            .unwrap();
        assert!(base.get::<WorldPose>().unwrap().rotation[2].abs() > 0.01);
    }
    let backend = server
        .simulation
        .world()
        .resource::<gz_physics::PhysicsBackend>();
    for node in server.simulation.world().iter_entities().filter(|node| {
        node.get::<Name>()
            .is_some_and(|name| name.0.starts_with("default::robot_"))
    }) {
        if let Some(handle) = node.get::<JointHandle>() {
            let state = backend.joint_state(handle.id()).unwrap();
            assert!(state.position.is_finite() && state.velocity.abs() > 0.01);
        }
    }
    server.set_paused(true);
    let paused = held(&server);
    for _ in 0..3 {
        assert!(server.update().unwrap().dt.is_zero());
        assert_eq!(held(&server), paused);
    }
    let before_reset = (
        server.counters.diff_drive_commands_applied,
        server.counters.joint_controller_commands_applied,
    );
    reset(&mut server);
    let restored = held(&server);
    assert_eq!(restored.0, initial_state.0 + 1);
    assert_eq!(restored.1, Duration::ZERO);
    assert_eq!(
        (restored.2, restored.3, restored.4),
        (initial_state.2, initial_state.3, initial_state.4)
    );
    assert!(restored.5.is_empty());
    assert_eq!(initial(&server.simulation).unwrap(), controller_originals);
    assert_eq!(
        preflight(&server.simulation, &server.world_io, &server.network_config).unwrap(),
        controller_originals
    );
    assert_eq!(
        gz_sim_scene::initial_diff_drive_configurations_with_identity(&server.simulation)
            .unwrap()
            .map(|row| (row.id, row.configuration.clone()))
            .collect::<Vec<_>>(),
        drive_originals
    );
    assert_eq!(server.joint_controllers.as_ref().unwrap().streams.len(), 16);
    assert_eq!(
        crate::diff_drive::boundary_keys(&server.simulation, &server.network_config, None)
            .unwrap()
            .len(),
        16
    );
    assert_eq!(
        (
            server.counters.diff_drive_commands_applied,
            server.counters.joint_controller_commands_applied
        ),
        before_reset
    );
    // No new commands after reset: the world-rooted controller chains retain
    // fresh zero targets. Passive M8 ground/contact settling is not a rest oracle.
    server.set_paused(false);
    for _ in 0..100 {
        server.update().unwrap();
    }
    let backend = server
        .simulation
        .world()
        .resource::<gz_physics::PhysicsBackend>();
    for stream in &server.joint_controllers.as_ref().unwrap().streams {
        let state = backend
            .joint_state(stream.joint.native.expect("Rapier fixture"))
            .unwrap();
        assert!(
            state.position.is_finite()
                && state.position.abs() < 1e-12
                && state.velocity.abs() < 1e-12
        );
    }
    assert_eq!(
        (
            server.counters.diff_drive_commands_applied,
            server.counters.joint_controller_commands_applied
        ),
        before_reset
    );
}

#[test]
fn absent_native_joint_body_and_world_reject_controller_registration() {
    let (mut server, _observer) = build(&fleet_xml(1, 2));
    let owner = &candidates(&server.simulation, &server.network_config).unwrap()[0];
    let (id, entity) = (owner.identity, owner.entity);
    let child = entity_by_id(server.simulation.world(), owner.joint.child).unwrap();
    let world = server.simulation.world_mut();
    let body = world.entity_mut(child).take::<BodyHandle>().unwrap();
    assert!(PhysicalJoint::resolve(&server.simulation, entity, id).is_none());
    server.simulation.world_mut().entity_mut(child).insert(body);
    let backend = server
        .simulation
        .world_mut()
        .remove_resource::<gz_physics::PhysicsBackend>()
        .unwrap();
    assert!(PhysicalJoint::resolve(&server.simulation, entity, id).is_none());
    server.simulation.world_mut().insert_resource(backend);
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .remove::<JointHandle>();
    assert!(PhysicalJoint::resolve(&server.simulation, entity, id).is_none());
}
