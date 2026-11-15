// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Actual loaded peer reservations and pure constructor/reset/live boundaries.
use super::*;
use crate::multiple_joint_controller_boundaries::snapshot;
use gz_sim_proto::msgs::WorldControl;
use gz_sim_scene::{
    AirPressurePublication, AltimeterPublication, JointControllerConfiguration, JointHandle,
    JointStatePublication, MagnetometerPublication, PhysicsEngine, load_world_with_sdf_step,
    remove_model,
};

const DRIVES: &str = include_str!("../../../../examples/m59_multiple_diff_drives.sdf");
const CONTROLLERS: &str = include_str!("../../../../examples/m58_multiple_joint_controllers.sdf");
const COMMAND: &str = "/m59/left/cmd_vel";
const PEERS: [&str; 4] = [
    "/m21/altimeter",
    "/m22/air_pressure",
    "/m23/magnetometer",
    "/m18/joint_state",
];

fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m59-boundaries.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
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
            partition: format!("m59-boundary-{}", uuid::Uuid::new_v4()),
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
fn keys(sim: &Simulation, server: &Server) -> BTreeSet<String> {
    crate::diff_drive_reserved_keys(sim, &server.world_io, &server.network_config).unwrap()
}
fn model_ids(sim: &Simulation) -> Vec<u64> {
    sim.world()
        .iter_entities()
        .filter(|node| node.get::<EntityKind>() == Some(&EntityKind::Model))
        .map(|node| node.get::<EntityId>().unwrap().get())
        .collect()
}
fn typed_xml() -> String {
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
    for (xml, old, new) in inputs {
        let start = xml.find("<model ").unwrap();
        let end = xml.rfind("</model>").unwrap() + "</model>".len();
        assert!(xml[start..end].contains(old));
        models.push_str(&xml[start..end].replacen(old, new, 1));
    }
    DRIVES.replace(
        "</world>",
        &format!("<magnetic_field>0.00002 -0.00003 0.00004</magnetic_field>{models}</world>"),
    )
}
fn typed_error(error: Error, family: usize) {
    assert!(
        match family {
            0 => matches!(error, Error::AltimeterIdentity),
            1 => matches!(error, Error::AirPressureIdentity),
            2 => matches!(error, Error::MagnetometerIdentity),
            3 => matches!(error, Error::JointStateIdentity),
            _ => false,
        },
        "unexpected family {family}: {error:?}"
    );
}

#[test]
fn own_excluded_drive_reservations_cover_fixed_world_and_every_loaded_peer_role() {
    let network = Network::new();
    let server = network.server(load(DRIVES), network.options()).unwrap();
    let before = snapshot(&server);
    let own = boundary_keys(&server.simulation, &server.network_config, None).unwrap();
    assert_eq!(own.len(), 4);
    let reserved = keys(&server.simulation, &server);
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
        preflight(&server.simulation, &server.world_io, &server.network_config)
            .unwrap()
            .len(),
        2
    );
    assert_eq!(snapshot(&server), before);
    // Complete admitted fixtures, not invented metadata, exercise each family.
    let cases: &[(&str, &[&str])] = &[
        (
            include_str!("../../../../examples/m21_altimeter_noise.sdf"),
            &["/m21/altimeter"],
        ),
        (
            include_str!("../../../../examples/m22_air_pressure_noise.sdf"),
            &["/m22/air_pressure"],
        ),
        (
            include_str!("../../../../examples/m23_magnetometer_noise.sdf"),
            &["/m23/magnetometer"],
        ),
        (
            include_str!("../../../gz-sim-scene/tests/fixtures/m18_joint_state.sdf"),
            &["/m18/joint_state"],
        ),
        (
            include_str!("../../../../examples/m26_air_speed.sdf"),
            &["/m26/air_speed"],
        ),
        (
            include_str!("../../../../examples/m31_pose_static_publisher.sdf"),
            &["/m31/pose", "/m31/pose_static"],
        ),
        (
            include_str!("../../../../examples/m30_logical_camera.sdf"),
            &["/m30/logical_camera", "/m30/logical_camera/frustum"],
        ),
        (
            include_str!("../../../../examples/m34_odometry_publisher.sdf"),
            &[
                "/m34/odometry",
                "/m34/odometry_covariance",
                "/m34/transform",
            ],
        ),
        (
            include_str!("../../../../examples/m35_apply_link_wrench.sdf"),
            &[
                "/world/default/wrench",
                "/world/default/wrench/persistent",
                "/world/default/wrench/clear",
            ],
        ),
        (
            include_str!("../../../../examples/m42_joint_force_feedback.sdf"),
            &["/model/rotor/joint/hinge/cmd_force", "/m42/joint_state"],
        ),
    ];
    for &(xml, topics) in cases {
        let mut sim = load(xml);
        let expected: BTreeSet<_> = topics
            .iter()
            .map(|raw| server.network_config.topic_key(raw).unwrap())
            .collect();
        let before = (sim.elapsed(), sim.epoch(), sim.world().entities().len());
        assert!(expected.is_subset(&keys(&sim, &server)), "{topics:?}");
        assert_eq!(
            (sim.elapsed(), sim.epoch(), sim.world().entities().len()),
            before
        );
        for owner in model_ids(&sim) {
            remove_model(&mut sim, owner).unwrap();
        }
        assert!(
            expected.is_subset(&keys(&sim, &server)),
            "removed originals: {topics:?}"
        );
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
    let source = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/m33_robot_sensors.sdf")
        .canonicalize()
        .unwrap();
    let mut sensors = gz_sim_scene::load_world_seeded(
        include_str!("../../../../examples/m33_robot_sensors.sdf"),
        &format!("file://{}", source.display()),
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        [33; 32],
    )
    .unwrap();
    let expected: BTreeSet<_> = ["/m33/imu", "/m33/imu/set_rate", "/m33/scan"]
        .into_iter()
        .map(|raw| server.network_config.topic_key(raw).unwrap())
        .collect();
    assert!(expected.is_subset(&keys(&sensors, &server)));
    for raw in ["/model/robot/cmd_vel", "/model/robot/odometry"] {
        assert!(!keys(&sensors, &server).contains(&server.network_config.topic_key(raw).unwrap()));
    }
    for owner in model_ids(&sensors) {
        remove_model(&mut sensors, owner).unwrap();
    }
    assert!(expected.is_subset(&keys(&sensors, &server)));
    let bare = Simulation::new(Duration::from_millis(1)).unwrap();
    let mut baseline = server.world_io.reserved_keys();
    baseline.insert(server.network_config.topic_key("/clock").unwrap());
    assert_eq!(keys(&bare, &server), baseline);
    assert_eq!(snapshot(&server), before);
}

#[test]
fn controller_peer_reservations_keep_originals_and_only_current_nonpending_drift() {
    let network = Network::new();
    let server = network.server(load(DRIVES), network.options()).unwrap();
    let mut sim = load(CONTROLLERS);
    let node = sim
        .world()
        .iter_entities()
        .find(|node| node.contains::<JointControllerConfiguration>())
        .unwrap();
    let entity = node.id();
    let model = sim.world().get::<ChildOf>(entity).unwrap().parent();
    let owner = sim.world().get::<EntityId>(model).unwrap().get();
    let original = node.get::<JointControllerConfiguration>().unwrap().clone();
    let mut drift = original.clone();
    drift.topic = "/m59/current/controller".into();
    sim.world_mut().entity_mut(entity).insert(drift.clone());
    let old = server.network_config.topic_key(&original.topic).unwrap();
    let new = server.network_config.topic_key(&drift.topic).unwrap();
    assert!(keys(&sim, &server).contains(&old));
    assert!(keys(&sim, &server).contains(&new));
    sim.world_mut()
        .entity_mut(entity)
        .insert(PendingResetRemoval);
    assert!(keys(&sim, &server).contains(&old));
    assert!(!keys(&sim, &server).contains(&new));
    sim.world_mut()
        .entity_mut(entity)
        .remove::<PendingResetRemoval>();
    sim.world_mut().entity_mut(entity).insert(original);
    remove_model(&mut sim, owner).unwrap();
    assert!(keys(&sim, &server).contains(&old));
    assert!(!keys(&sim, &server).contains(&new));
}

#[test]
fn typed_peer_error_winners_precede_reset_drift_and_survive_real_peer_removal() {
    let network = Network::new();
    let xml = typed_xml();
    for (family, peer) in PEERS.iter().copied().enumerate() {
        let mut server = network.server(load(&xml), network.options()).unwrap();
        server
            .network_config
            .topic_remaps
            .insert(COMMAND.into(), "/clock".into());
        server
            .network_config
            .topic_remaps
            .insert(peer.into(), "/clock".into());
        let before = snapshot(&server);
        assert!(matches!(
            crate::validate_diff_drive_peer_topics(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                None
            ),
            Err(Error::DiffDriveIdentity)
        ));
        assert_eq!(snapshot(&server), before);
        // Healthy retained live metadata reaches the earlier typed reset winner.
        typed_error(server.reset_scene().unwrap_err(), family);
        assert_eq!(snapshot(&server), before);
        let owner = server
            .simulation
            .world()
            .iter_entities()
            .find(|node| {
                node.get::<DiffDriveConfiguration>()
                    .is_some_and(|cfg| cfg.topic == COMMAND)
            })
            .unwrap()
            .id();
        let original = server
            .simulation
            .world()
            .get::<DiffDriveConfiguration>(owner)
            .unwrap()
            .clone();
        let mut drift = original.clone();
        drift.frame_id.push_str("_drift");
        server
            .simulation
            .world_mut()
            .entity_mut(owner)
            .insert(drift.clone());
        let before = snapshot(&server);
        // Pre-reset public drift is live corruption: the OLD lease witness stays strict.
        assert!(matches!(
            server.reset_scene(),
            Err(Error::DiffDriveIdentity)
        ));
        assert_eq!(snapshot(&server), before);
        assert_eq!(
            server
                .simulation
                .world()
                .get::<DiffDriveConfiguration>(owner),
            Some(&drift)
        );
        server
            .simulation
            .world_mut()
            .entity_mut(owner)
            .insert(original);
        let mut options = network.options();
        options.topic_remaps = server.network_config.topic_remaps.clone();
        let error = match network.server(load(&xml), options) {
            Err(error) => error,
            Ok(_) => panic!("collision admitted"),
        };
        assert!(matches!(error, Error::DiffDriveIdentity));
        // Removing the real peer retires constructor eligibility, not its saved keys.
        let node = server
            .simulation
            .world()
            .iter_entities()
            .find(|node| match family {
                0 => node.contains::<AltimeterPublication>(),
                1 => node.contains::<AirPressurePublication>(),
                2 => node.contains::<MagnetometerPublication>(),
                3 => node.contains::<JointStatePublication>(),
                _ => false,
            })
            .unwrap();
        let mut entity = node.id();
        while server.simulation.world().get::<EntityKind>(entity) != Some(&EntityKind::Model) {
            entity = server
                .simulation
                .world()
                .get::<ChildOf>(entity)
                .unwrap()
                .parent();
        }
        let owner = server
            .simulation
            .world()
            .get::<EntityId>(entity)
            .unwrap()
            .get();
        remove_model(&mut server.simulation, owner).unwrap();
        let before = snapshot(&server);
        assert!(
            keys(&server.simulation, &server)
                .contains(&server.network_config.topic_key(peer).unwrap())
        );
        assert!(matches!(
            crate::validate_diff_drive_peer_topics(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                None
            ),
            Err(Error::DiffDriveIdentity)
        ));
        typed_error(
            crate::validate_diff_drive_peer_topics(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                Some(&server),
            )
            .unwrap_err(),
            family,
        );
        assert_eq!(snapshot(&server), before);
    }
    // Force belongs to full reservations, but was not in the old Drive priority
    // subset. A simultaneous later typed collision therefore keeps its winner.
    let force = include_str!("../../../../examples/m42_joint_force_feedback.sdf");
    let start = force.find("<model name=\"rotor\"").unwrap();
    let end = start + force[start..].find("</model>").unwrap() + "</model>".len();
    let force_xml = xml.replace("</world>", &format!("{}</world>", &force[start..end]));
    for (family, peer) in PEERS.iter().copied().enumerate() {
        let mut server = network.server(load(&force_xml), network.options()).unwrap();
        let force_topic = "/model/rotor/joint/hinge/cmd_force";
        assert_eq!(
            gz_sim_scene::initial_joint_force_configurations(&server.simulation)
                .unwrap()
                .count(),
            1
        );
        server
            .network_config
            .topic_remaps
            .insert(COMMAND.into(), force_topic.into());
        server
            .network_config
            .topic_remaps
            .insert("/m59/left/odom".into(), peer.into());
        assert_ne!(
            server.network_config.topic_key(COMMAND).unwrap(),
            server.network_config.topic_key("/m59/left/odom").unwrap()
        );
        let before = snapshot(&server);
        assert!(
            keys(&server.simulation, &server)
                .contains(&server.network_config.topic_key(force_topic).unwrap())
        );
        assert!(matches!(
            preflight(&server.simulation, &server.world_io, &server.network_config),
            Err(Error::DiffDriveIdentity)
        ));
        typed_error(
            crate::validate_diff_drive_peer_topics(
                &server.simulation,
                &server.world_io,
                &server.network_config,
                None,
            )
            .unwrap_err(),
            family,
        );
        assert_eq!(snapshot(&server), before);
        typed_error(server.reset_scene().unwrap_err(), family);
        assert_eq!(snapshot(&server), before);
        let mut options = network.options();
        options.topic_remaps = server.network_config.topic_remaps.clone();
        let error = match network.server(load(&force_xml), options) {
            Err(error) => error,
            Ok(_) => panic!("Force/typed simultaneous collision admitted"),
        };
        typed_error(error, family);
        assert_eq!(snapshot(&server), before);
    }
}

type PhysicalSnapshot = (
    Duration,
    u64,
    String,
    Vec<(u64, [u64; 2])>,
    Vec<(u64, Duration, u64, [u64; 5])>,
);
fn physical_snapshot(server: &Server) -> PhysicalSnapshot {
    let mut native = Vec::new();
    let mut samples = Vec::new();
    for node in server.simulation.world().iter_entities() {
        let id = node.get::<EntityId>().map(EntityId::get).unwrap_or(0);
        if let Some(handle) = node.get::<JointHandle>() {
            let value = server
                .simulation
                .world()
                .resource::<gz_physics::PhysicsBackend>()
                .joint_state(handle.id())
                .unwrap();
            native.push((id, [value.position.to_bits(), value.velocity.to_bits()]));
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
    (
        server.simulation.elapsed(),
        server.simulation.epoch(),
        format!("{:?}", server.counters()),
        native,
        samples,
    )
}
#[test]
fn tick_detects_live_original_corruption_without_cached_drive_streams_before_any_work() {
    let network = Network::new();
    for direct_retirement in [false, true] {
        let mut server = network.server(load(DRIVES), network.options()).unwrap();
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            ..Default::default()
        }));
        // Both original entities remain alive. Public name corruption cannot become vacancy.
        let owners: Vec<_> = server
            .simulation
            .world()
            .iter_entities()
            .filter(|node| node.contains::<DiffDriveConfiguration>())
            .map(|node| node.id())
            .collect();
        assert_eq!(owners.len(), 2);
        for entity in owners {
            server
                .simulation
                .world_mut()
                .entity_mut(entity)
                .insert(Name("corrupted".into()));
        }
        if direct_retirement {
            server.retire_diff_drive();
            assert!(server.diff_drive.is_none());
        } else {
            server.diff_drive = None;
        }
        let before = physical_snapshot(&server);
        assert!(!server.reset_failed);
        assert!(matches!(server.tick(), Err(Error::DiffDriveIdentity)));
        assert!(server.reset_failed && server.diff_drive.is_none());
        assert_eq!(physical_snapshot(&server), before);
    }
    // Full model removal is legitimate retirement and must not invent a live fault.
    let mut server = network.server(load(DRIVES), network.options()).unwrap();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    for owner in model_ids(&server.simulation)
        .into_iter()
        .filter(|id| {
            gz_sim_scene::initial_diff_drive_configurations_with_identity(&server.simulation)
                .unwrap()
                .any(|row| row.id == *id)
        })
        .collect::<Vec<_>>()
    {
        remove_model(&mut server.simulation, owner).unwrap();
    }
    server.retire_diff_drive();
    assert!(server.diff_drive.is_none());
    let time = server.simulation.elapsed();
    server.tick().unwrap();
    assert!(!server.reset_failed && server.diff_drive.is_none());
    assert_eq!(server.simulation.elapsed(), time);
}
