// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Pure key reservations and actual constructor/reset rejection boundaries.
//! Metadata-drift/manual cases are key-only and never step an incoherent graph.
use super::*;
use bevy_ecs::hierarchy::ChildOf;
use gz_sim_scene::{
    AirPressurePublication, AltimeterPublication, DiffDriveConfiguration, EntityKind,
    JointControllerConfiguration, JointSample, JointStatePublication, MagnetometerPublication,
    Name, PendingResetRemoval, PhysicsEngine, WorldPose, load_world_with_sdf_step, remove_model,
};

const CONTROLLERS: &str = include_str!("../../../../examples/m58_multiple_joint_controllers.sdf");
const DRIVE: &str = include_str!("../../../../examples/m8_diff_drive.sdf");
const POSITION: &str = "/model/arm_left/joint/base_hinge/0/cmd_pos";
const PEER_TOPICS: [&str; 4] = [
    "/m21/altimeter",
    "/m22/air_pressure",
    "/m23/magnetometer",
    "/m18/joint_state",
];
const PEER_MODELS: [&str; 4] = ["alt_rig", "pressure_rig", "mag_rig", "state_rig"];

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
struct Network {
    observer: zenoh::Session,
    endpoint: String,
}
impl Network {
    fn new() -> Self {
        let observer = zenoh::open(config(None)).wait().unwrap();
        let endpoints: Vec<_> = observer
            .info()
            .locators()
            .wait()
            .into_iter()
            .map(|value| value.to_string())
            .collect();
        assert_eq!(endpoints.len(), 1);
        assert_ne!(endpoints[0], "tcp/127.0.0.1:0");
        Self {
            observer,
            endpoint: endpoints[0].clone(),
        }
    }
    fn options(&self) -> ServerOptions {
        assert!(!self.observer.is_closed());
        ServerOptions {
            partition: format!("m58-boundary-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        }
    }
    fn server(&self, sim: Simulation, options: ServerOptions) -> Result<Server, Error> {
        Server::with_zenoh_config(
            sim,
            "default",
            options,
            config(Some(&self.endpoint)),
            NonZeroUsize::new(2).unwrap(),
        )
    }
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m58-controller-boundaries.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn with_typed_peers() -> String {
    // Reuse complete admitted models. No sensor or unconfigured joint is added
    // to either controller chain, and no numerical/topology parameters change.
    let inputs = [
        (
            include_str!("../../../../examples/m21_altimeter_noise.sdf"),
            "name='rig'",
            "name='alt_rig'",
        ),
        (
            include_str!("../../../../examples/m22_air_pressure_noise.sdf"),
            "name='rig'",
            "name='pressure_rig'",
        ),
        (
            include_str!("../../../../examples/m23_magnetometer_noise.sdf"),
            "name='rig'",
            "name='mag_rig'",
        ),
        (
            include_str!("../../../gz-sim-scene/tests/fixtures/m18_joint_state.sdf"),
            "name=\"arm\"",
            "name=\"state_rig\"",
        ),
    ];
    let mut models = String::new();
    for (xml, before, after) in inputs {
        let start = xml.find("<model ").unwrap();
        let end = xml.rfind("</model>").unwrap() + "</model>".len();
        assert!(xml[start..end].contains(before));
        models.push_str(&xml[start..end].replacen(before, after, 1));
    }
    CONTROLLERS.replace(
        "</world>",
        &format!("<magnetic_field>0.00002 -0.00003 0.00004</magnetic_field>{models}</world>"),
    )
}
fn peer_entity(sim: &Simulation, family: usize) -> Entity {
    let mut rows = sim.world().iter_entities().filter(|node| match family {
        0 => node.contains::<AltimeterPublication>(),
        1 => node.contains::<AirPressurePublication>(),
        2 => node.contains::<MagnetometerPublication>(),
        3 => node.contains::<JointStatePublication>(),
        _ => panic!("closed typed family"),
    });
    let entity = rows.next().unwrap().id();
    assert!(rows.next().is_none());
    entity
}
fn assert_typed(error: Error, family: usize) {
    let matches = match family {
        0 => matches!(error, Error::AltimeterIdentity),
        1 => matches!(error, Error::AirPressureIdentity),
        2 => matches!(error, Error::MagnetometerIdentity),
        3 => matches!(error, Error::JointStateIdentity),
        _ => false,
    };
    assert!(matches, "expected family {family}: {error:?}");
}
fn constructor_error(network: &Network, sim: Simulation, options: ServerOptions) -> Error {
    match network.server(sim, options) {
        Err(error) => error,
        Ok(_) => panic!("aliased inputs unexpectedly admitted"),
    }
}
#[derive(Debug, PartialEq, Eq)]
struct RowSnapshot {
    entity: Entity,
    identity: Option<u64>,
    kind: Option<EntityKind>,
    name: Option<String>,
    parent: Option<Entity>,
    pending_removal: bool,
    controller: Option<JointControllerConfiguration>,
    pose: Option<[u64; 7]>,
    joint_sample: Option<(Duration, u64, [u64; 2])>,
}
#[derive(Debug, PartialEq, Eq)]
pub(super) struct BoundarySnapshot {
    time: Duration,
    epoch: u64,
    pending_steps: u64,
    reset_failed: bool,
    prepared_reset: bool,
    counters: [u64; 5],
    output_counts: [usize; 6],
    fixed_keys: BTreeSet<String>,
    clock_and_pose_leases: [(usize, usize); 2],
    rows: Vec<RowSnapshot>,
}
pub(super) fn snapshot(server: &Server) -> BoundarySnapshot {
    let mut rows: Vec<_> = server
        .simulation
        .world()
        .iter_entities()
        .map(|node| RowSnapshot {
            entity: node.id(),
            identity: node.get::<EntityId>().map(EntityId::get),
            kind: node.get::<EntityKind>().copied(),
            name: node.get::<Name>().map(|name| name.0.clone()),
            parent: node.get::<ChildOf>().map(ChildOf::parent),
            pending_removal: node.contains::<PendingResetRemoval>(),
            controller: node.get::<JointControllerConfiguration>().cloned(),
            pose: node.get::<WorldPose>().map(|pose| {
                [
                    pose.translation[0].to_bits(),
                    pose.translation[1].to_bits(),
                    pose.translation[2].to_bits(),
                    pose.rotation[0].to_bits(),
                    pose.rotation[1].to_bits(),
                    pose.rotation[2].to_bits(),
                    pose.rotation[3].to_bits(),
                ]
            }),
            joint_sample: node.get::<JointSample>().map(|sample| {
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
        .collect();
    rows.sort_by_key(|row| row.entity);
    let counters = server.counters();
    BoundarySnapshot {
        time: server.simulation.elapsed(),
        epoch: server.simulation.epoch(),
        pending_steps: server.pending_steps,
        reset_failed: server.reset_failed,
        prepared_reset: server.prepared_reset.is_some(),
        counters: [
            counters.joint_controller_messages_received,
            counters.joint_controller_messages_rejected,
            counters.joint_controller_commands_applied,
            counters.clocks_queued,
            counters.controls_accepted,
        ],
        output_counts: [
            server.imus.len(),
            server.lidars.len(),
            server.altimeters.len(),
            server.air_pressures.len(),
            server.magnetometers.len(),
            server.joint_states.len(),
        ],
        fixed_keys: server.world_io.reserved_keys(),
        clock_and_pose_leases: [
            (
                Arc::as_ptr(&server.world_io.clock) as usize,
                Arc::strong_count(&server.world_io.clock),
            ),
            (
                Arc::as_ptr(&server.world_io.poses) as usize,
                Arc::strong_count(&server.world_io.poses),
            ),
        ],
        rows,
    }
}

#[test]
fn controller_reservations_exclude_all_own_inputs_and_retain_every_fixed_world_role() {
    let network = Network::new();
    let server = network
        .server(load(CONTROLLERS), network.options())
        .unwrap();
    let before = snapshot(&server);
    let own =
        joint_controllers::boundary_keys(&server.simulation, &server.network_config, None).unwrap();
    assert_eq!(own.len(), 4);
    let reserved = joint_controller_reserved_keys(
        &server.simulation,
        &server.world_io,
        &server.network_config,
    )
    .unwrap();
    assert!(reserved.is_disjoint(&own));
    assert!(server.world_io.reserved_keys().is_subset(&reserved));
    for topic in [
        "/clock",
        "/world/default/clock",
        "/world/default/control",
        "/world/default/pose/info",
        "/world/default/remove",
        "/world/default/remove/blocking",
    ] {
        assert!(
            reserved.contains(&server.network_config.topic_key(topic).unwrap()),
            "{topic}"
        );
    }
    assert_eq!(
        joint_controllers::preflight(&server.simulation, &server.world_io, &server.network_config)
            .unwrap()
            .len(),
        4
    );
    assert_eq!(snapshot(&server), before);
}

#[test]
fn drive_command_and_odometry_reservations_distinguish_current_drift_pending_and_removed_originals()
{
    let network = Network::new();
    let mut server = network.server(load(DRIVE), network.options()).unwrap();
    let node = server
        .simulation
        .world()
        .iter_entities()
        .find(|node| node.contains::<DiffDriveConfiguration>())
        .unwrap();
    let entity = node.id();
    let model = node.get::<EntityId>().unwrap().get();
    let original = node.get::<DiffDriveConfiguration>().unwrap().clone();
    let keys = |server: &Server| {
        joint_controller_reserved_keys(&server.simulation, &server.world_io, &server.network_config)
            .unwrap()
    };
    let old_keys = BTreeSet::from([
        server.network_config.topic_key(&original.topic).unwrap(),
        server
            .network_config
            .topic_key(&original.odom_topic)
            .unwrap(),
    ]);
    assert!(old_keys.is_subset(&keys(&server)));
    let mut drift = original.clone();
    drift.topic = "/m58/current_drive/command".into();
    drift.odom_topic = "/m58/current_drive/odometry".into();
    let current_keys = BTreeSet::from([
        server.network_config.topic_key(&drift.topic).unwrap(),
        server.network_config.topic_key(&drift.odom_topic).unwrap(),
    ]);
    // KEY-ONLY drift: do not poll, step, or claim this changed metadata reconfigures Drive.
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(drift);
    let before = snapshot(&server);
    let all = keys(&server);
    assert!(old_keys.is_subset(&all) && current_keys.is_subset(&all));
    assert_eq!(snapshot(&server), before);
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(PendingResetRemoval);
    let before = snapshot(&server);
    let pending = keys(&server);
    assert!(old_keys.is_subset(&pending) && current_keys.is_disjoint(&pending));
    assert!(
        joint_controllers::legacy_occupied(&server.simulation, &server.network_config, false)
            .unwrap()
            .is_disjoint(&old_keys)
    );
    assert!(
        old_keys.is_subset(
            &joint_controllers::legacy_occupied(&server.simulation, &server.network_config, true)
                .unwrap()
        )
    );
    assert_eq!(snapshot(&server), before);
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .remove::<PendingResetRemoval>();
    server
        .simulation
        .world_mut()
        .entity_mut(entity)
        .insert(original);
    remove_model(&mut server.simulation, model).unwrap();
    assert!(entity_by_id(server.simulation.world(), model).is_none());
    let before = snapshot(&server);
    assert!(old_keys.is_subset(&keys(&server)));
    assert!(current_keys.is_disjoint(&keys(&server)));
    assert_eq!(snapshot(&server), before);
}

#[test]
fn bare_missing_initial_scene_manual_metadata_has_only_explicit_key_reservation_scope() {
    let network = Network::new();
    let holder = network
        .server(load(CONTROLLERS), network.options())
        .unwrap();
    let mut bare = Simulation::new(Duration::from_millis(1)).unwrap();
    assert!(matches!(
        gz_sim_scene::initial_diff_drive_configurations(&bare),
        Err(gz_sim_scene::SceneResetError::MissingInitialState)
    ));
    let before = snapshot(&holder);
    let baseline =
        joint_controller_reserved_keys(&bare, &holder.world_io, &holder.network_config).unwrap();
    let mut expected = holder.world_io.reserved_keys();
    expected.insert(holder.network_config.topic_key("/clock").unwrap());
    assert_eq!(baseline, expected);
    let drive = bare
        .world_mut()
        .spawn(DiffDriveConfiguration {
            model_name: "manual".into(),
            topic: "/m58/manual_drive/command".into(),
            odom_topic: "/m58/manual_drive/odometry".into(),
            frame_id: "manual/odom".into(),
            child_frame_id: "manual/base".into(),
        })
        .id();
    bare.world_mut().spawn(JointControllerConfiguration {
        model_name: "manual".into(),
        joint_name: "hinge".into(),
        topic: "/m58/manual_controller".into(),
        mode: gz_sim_scene::JointControllerMode::Velocity,
    });
    let own = holder
        .network_config
        .topic_key("/m58/manual_controller")
        .unwrap();
    let extra = [
        holder
            .network_config
            .topic_key("/m58/manual_drive/command")
            .unwrap(),
        holder
            .network_config
            .topic_key("/m58/manual_drive/odometry")
            .unwrap(),
    ];
    let reserved =
        joint_controller_reserved_keys(&bare, &holder.world_io, &holder.network_config).unwrap();
    assert!(!reserved.contains(&own));
    assert!(extra.iter().all(|key| reserved.contains(key)));
    bare.world_mut()
        .entity_mut(drive)
        .insert(PendingResetRemoval);
    assert_eq!(
        joint_controller_reserved_keys(&bare, &holder.world_io, &holder.network_config).unwrap(),
        baseline
    );
    assert_eq!((bare.elapsed(), bare.epoch()), (Duration::ZERO, 0));
    assert_eq!(snapshot(&holder), before);
}

#[test]
fn genuine_typed_peers_keep_constructor_reset_priority_and_legacy_simultaneous_winners() {
    let network = Network::new();
    let xml = with_typed_peers();
    for first in 0..4 {
        let mut options = network.options();
        options
            .topic_remaps
            .insert(POSITION.into(), "/m58/typed_alias".into());
        for topic in &PEER_TOPICS[first..] {
            options
                .topic_remaps
                .insert((*topic).into(), "/m58/typed_alias".into());
        }
        assert_typed(constructor_error(&network, load(&xml), options), first);
        let mut server = network.server(load(&xml), network.options()).unwrap();
        server
            .network_config
            .topic_remaps
            .insert(POSITION.into(), "/m58/typed_alias".into());
        for topic in &PEER_TOPICS[first..] {
            server
                .network_config
                .topic_remaps
                .insert((*topic).into(), "/m58/typed_alias".into());
        }
        let before = snapshot(&server);
        assert_typed(
            validate_joint_controller_peer_topics(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                None,
            )
            .unwrap_err(),
            first,
        );
        assert_typed(server.reset_scene().unwrap_err(), first);
        assert_eq!(snapshot(&server), before);

        // Clock belongs to the old controller subset: constructor JCI precedes
        // typed peers, while reset prepares these original peers first.
        server.network_config.topic_remaps.clear();
        server
            .network_config
            .topic_remaps
            .insert(POSITION.into(), "/clock".into());
        server
            .network_config
            .topic_remaps
            .insert(PEER_TOPICS[first].into(), "/clock".into());
        let before = snapshot(&server);
        assert!(matches!(
            validate_joint_controller_peer_topics(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                None
            ),
            Err(Error::JointControllerIdentity)
        ));
        assert_typed(server.reset_scene().unwrap_err(), first);
        assert_eq!(snapshot(&server), before);
        let mut options = network.options();
        options
            .topic_remaps
            .insert(POSITION.into(), "/clock".into());
        options
            .topic_remaps
            .insert(PEER_TOPICS[first].into(), "/clock".into());
        assert!(matches!(
            constructor_error(&network, load(&xml), options),
            Error::JointControllerIdentity
        ));
    }
    // With no later typed peer claiming this key, reset must reach the old
    // controller/clock collision winner after validating the retained OLD leases.
    let mut server = network
        .server(load(CONTROLLERS), network.options())
        .unwrap();
    assert!(!server.simulation.world().iter_entities().any(|node| {
        node.contains::<AltimeterPublication>()
            || node.contains::<AirPressurePublication>()
            || node.contains::<MagnetometerPublication>()
            || node.contains::<JointStatePublication>()
    }));
    server
        .network_config
        .topic_remaps
        .insert(POSITION.into(), "/clock".into());
    let before = snapshot(&server);
    let clock = server.network_config.topic_key("/clock").unwrap();
    // This real production boundary includes retained OLD endpoint witnesses.
    // Its success isolates subsequent refusal to the full peer reservations.
    let planned =
        joint_controllers::boundary_keys(&server.simulation, &server.network_config, Some(&server))
            .unwrap();
    assert_eq!(planned.len(), 4);
    assert!(planned.contains(&clock));
    assert_eq!(snapshot(&server), before);
    assert!(matches!(
        joint_controllers::prepare_reset(&server),
        Err(Error::JointControllerIdentity)
    ));
    assert_eq!(snapshot(&server), before);
    assert!(matches!(
        validate_joint_controller_peer_topics(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            Some(&server),
        ),
        Err(Error::JointControllerIdentity)
    ));
    assert_eq!(snapshot(&server), before);
    assert!(matches!(
        server.reset_scene(),
        Err(Error::JointControllerIdentity)
    ));
    assert_eq!(snapshot(&server), before);
    assert!(matches!(
        constructor_error(&network, load(CONTROLLERS), server.network_config.clone(),),
        Error::JointControllerIdentity
    ));
    assert_eq!(snapshot(&server), before);
}

#[test]
fn pending_and_removed_genuine_typed_peers_are_constructor_inactive_but_reset_originals() {
    let network = Network::new();
    let xml = with_typed_peers();
    for family in 0..4 {
        let mut server = network.server(load(&xml), network.options()).unwrap();
        let entity = peer_entity(&server.simulation, family);
        server
            .network_config
            .topic_remaps
            .insert(POSITION.into(), "/m58/restored_alias".into());
        server
            .network_config
            .topic_remaps
            .insert(PEER_TOPICS[family].into(), "/m58/restored_alias".into());
        // Pending markers change only pure eligibility; no schedule is stepped.
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .insert(PendingResetRemoval);
        let before = snapshot(&server);
        validate_joint_controller_peer_topics(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            None,
        )
        .unwrap();
        assert_typed(
            validate_joint_controller_peer_topics(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                Some(&server),
            )
            .unwrap_err(),
            family,
        );
        assert!(matches!(
            joint_controllers::preflight(
                &server.simulation,
                &server.world_io,
                &server.network_config
            ),
            Err(Error::JointControllerIdentity)
        ));
        assert_eq!(snapshot(&server), before);
        server
            .simulation
            .world_mut()
            .entity_mut(entity)
            .remove::<PendingResetRemoval>();
        let model = server
            .simulation
            .world()
            .iter_entities()
            .find(|node| {
                node.get::<Name>()
                    .is_some_and(|name| name.0 == format!("default::{}", PEER_MODELS[family]))
            })
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        remove_model(&mut server.simulation, model).unwrap();
        let before = snapshot(&server);
        validate_joint_controller_peer_topics(
            &server.simulation,
            &server.world_io,
            &server.network_config,
            None,
        )
        .unwrap();
        assert_typed(server.reset_scene().unwrap_err(), family);
        assert_eq!(snapshot(&server), before);
    }
}

#[test]
fn last_controller_owner_preserves_full_peer_reservations_and_typed_error_priority() {
    let start = CONTROLLERS.find("<model name=\"arm_left\"").unwrap();
    let end = start + CONTROLLERS[start..].find("</model>").unwrap() + "</model>".len();
    let right = CONTROLLERS.find("<model name=\"arm_right\"").unwrap();
    let right_end = right + CONTROLLERS[right..].find("</model>").unwrap() + "</model>".len();
    let model = &CONTROLLERS[start..end];
    let mut models = String::new();
    for index in 0..8 {
        models.push_str(&model.replace("arm_left", &format!("arm_{index}")).replace(
            "<pose>0 -1.5 1 0 0 0</pose>",
            &format!("<pose>0 {} 1 0 0 0</pose>", index as f64 * 3.0),
        ));
    }
    let fleet =
        (CONTROLLERS[..start].to_owned() + &CONTROLLERS[end..right] + &CONTROLLERS[right_end..])
            .replace("</world>", &(models + "</world>"));
    let network = Network::new();
    let server = network.server(load(&fleet), network.options()).unwrap();
    let before = snapshot(&server);
    let metadata = joint_controllers::initial(&server.simulation).unwrap();
    assert_eq!(metadata.len(), 16);
    let own =
        joint_controllers::boundary_keys(&server.simulation, &server.network_config, None).unwrap();
    assert_eq!(own.len(), 16);
    let reserved = joint_controller_reserved_keys(
        &server.simulation,
        &server.world_io,
        &server.network_config,
    )
    .unwrap();
    assert!(own.is_disjoint(&reserved));
    for topic in [
        "/clock",
        "/world/default/clock",
        "/world/default/control",
        "/world/default/pose/info",
        "/world/default/remove",
        "/world/default/remove/blocking",
    ] {
        assert!(reserved.contains(&server.network_config.topic_key(topic).unwrap()));
        for cfg in metadata.values() {
            let mut options = server.network_config.clone();
            options.add_topic_remap(&cfg.topic, topic).unwrap();
            assert!(matches!(
                joint_controllers::preflight(&server.simulation, &server.world_io, &options),
                Err(Error::JointControllerIdentity)
            ));
            assert_eq!(snapshot(&server), before);
        }
    }
    let typed = with_typed_peers();
    let prefix = &CONTROLLERS[..CONTROLLERS.find("</world>").unwrap()];
    let fleet_prefix = &fleet[..fleet.find("</world>").unwrap()];
    let typed_fleet = fleet_prefix.to_owned() + typed.strip_prefix(prefix).unwrap();
    for (family, peer) in PEER_TOPICS.iter().copied().enumerate() {
        let mut options = network.options();
        options
            .add_topic_remap(
                "/model/arm_7/joint/base_hinge/0/cmd_pos",
                "/m62/typed_alias",
            )
            .unwrap();
        options.add_topic_remap(peer, "/m62/typed_alias").unwrap();
        assert_typed(
            constructor_error(&network, load(&typed_fleet), options),
            family,
        );
        let mut server = network
            .server(load(&typed_fleet), network.options())
            .unwrap();
        server
            .network_config
            .add_topic_remap(
                "/model/arm_7/joint/base_hinge/0/cmd_pos",
                "/m62/typed_alias",
            )
            .unwrap();
        server
            .network_config
            .add_topic_remap(peer, "/m62/typed_alias")
            .unwrap();
        let before = snapshot(&server);
        assert!(matches!(
            joint_controllers::preflight(
                &server.simulation,
                &server.world_io,
                &server.network_config
            ),
            Err(Error::JointControllerIdentity)
        ));
        assert_eq!(snapshot(&server), before);
        assert_typed(server.reset_scene().unwrap_err(), family);
        assert_eq!(snapshot(&server), before);
    }
}
