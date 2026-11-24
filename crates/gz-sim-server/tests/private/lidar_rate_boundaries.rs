// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy_ecs::entity::Entity;
use gz_sim_scene::{EntityKind, LidarRateError, LidarSample, PendingResetRemoval, PhysicsEngine};
use std::time::Duration;

fn options() -> ServerOptions {
    ServerOptions {
        partition: format!("lidar-rate-boundary-{}", uuid::Uuid::new_v4()),
        capacity: 2,
        lidar_point_clouds: true,
        ..Default::default()
    }
}

fn transport() -> zenoh::Config {
    let mut config = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(key, value).unwrap();
    }
    config
}

fn bare() -> Simulation {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    sim.initialize_schedules().unwrap();
    sim
}

fn host(config: &ServerOptions) -> crate::Server {
    crate::Server::with_zenoh_config(
        bare(),
        "default",
        config.clone(),
        transport(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap()
}

fn loaded(count: usize) -> Simulation {
    let sensors = (0..count).map(|i| format!(
        "<sensor name='scan{i}' type='lidar'><pose>0 0 0.25 0 0 0</pose><update_rate>100</update_rate><topic>/boundary/scan{i}</topic><frame_id>scan{i}</frame_id><lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></horizontal></scan><range><min>0</min><max>20</max></range></lidar></sensor>"
    )).collect::<String>();
    let xml = format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><model name='rig'><static>true</static><link name='body'><collision name='self'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision>{sensors}</link></model><model name='wall'><static>true</static><pose>5 0 0 0 0 0</pose><link name='body'><collision name='wall'><geometry><box><size>2 100 100</size></box></geometry></collision></link></model></world></sdf>"
    );
    let mut sim = gz_sim_scene::load_world_with_engine(
        &xml,
        "lidar-rate-boundaries.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    // Establish Bevy schedule resources before queuing or taking cold baselines.
    sim.initialize_schedules().unwrap();
    sim
}

fn sensor(sim: &Simulation, index: usize) -> (Entity, u64, LidarPublication) {
    let topic = format!("/boundary/scan{index}");
    let entity = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<LidarPublication>()
                .is_some_and(|m| m.topic == topic)
        })
        .unwrap();
    (
        entity.id(),
        entity.get::<EntityId>().unwrap().get(),
        entity.get::<LidarPublication>().unwrap().clone(),
    )
}

#[derive(Debug, PartialEq)]
struct SampleBits {
    time: Duration,
    epoch: u64,
    translation: [u64; 3],
    rotation: [u64; 4],
    ranges: Vec<u64>,
}

fn sample(sim: &Simulation, entity: Entity) -> Option<SampleBits> {
    sim.world()
        .get::<LidarSample>(entity)
        .map(|sample| SampleBits {
            time: sample.time,
            epoch: sample.epoch,
            translation: sample.world_pose.translation.map(f64::to_bits),
            rotation: sample.world_pose.rotation.map(f64::to_bits),
            ranges: sample.ranges.iter().map(|value| value.to_bits()).collect(),
        })
}

#[derive(Debug, PartialEq)]
struct Snapshot {
    time: Duration,
    schedules: (usize, usize),
    entities: usize,
    id: Option<u64>,
    kind: Option<EntityKind>,
    pending: bool,
    metadata: Option<LidarPublication>,
    sample: Option<SampleBits>,
}

fn snapshot(sim: &Simulation, entity: Entity) -> Snapshot {
    Snapshot {
        time: sim.elapsed(),
        schedules: sim.scheduled_system_counts(),
        entities: sim.world().iter_entities().count(),
        id: sim.world().get::<EntityId>(entity).map(EntityId::get),
        kind: sim.world().get::<EntityKind>(entity).copied(),
        pending: sim.world().get::<PendingResetRemoval>(entity).is_some(),
        metadata: sim.world().get::<LidarPublication>(entity).cloned(),
        sample: sample(sim, entity),
    }
}

fn assert_wall(sim: &Simulation, entity: Entity, at: Duration) {
    let actual = sim.world().get::<LidarSample>(entity).unwrap();
    assert_eq!(actual.time, at);
    assert_eq!(actual.world_pose.translation, [0.0, 0.0, 0.25]);
    assert_eq!(actual.world_pose.rotation, [0.0, 0.0, 0.0, 1.0]);
    assert_eq!(actual.ranges.len(), 3);
    // The near wall face is x=4; each sensor-local yaw intersects it at 4/cos(yaw).
    for (range, yaw) in actual.ranges.iter().zip([-0.3_f64, 0.0, 0.3]) {
        assert!((*range - 4.0 / yaw.cos()).abs() < 1e-10);
    }
}

fn continuation(sim: &mut Simulation, entity: Entity, twin: &mut Simulation, twin_entity: Entity) {
    // Both clocks start at 1ms with the original 10ms deadline still pending.
    for (steps, acquisition) in [(9, 10), (19, 10), (1, 30)] {
        sim.step_exact(steps).unwrap();
        twin.step_exact(steps).unwrap();
        assert_eq!(sample(sim, entity), sample(twin, twin_entity));
        assert_wall(sim, entity, Duration::from_millis(acquisition));
    }
}

#[test]
fn bare_rate_restart_preserves_manual_imu_and_filters_pending_lidar_without_acquisition() {
    let config = options();
    let mut empty = bare();
    let mut server = host(&config);
    let reserved = server.world_io.reserved_keys();
    crate::validate_lidar_rate_topics(&empty, &server.world_io, &config).unwrap();
    assert!(server.rates.is_none() && server.lidars.is_empty());
    assert_eq!(server.world_io.reserved_keys(), reserved);
    let drops = Arc::new(AtomicU64::new(0));
    let (rates, setup) = Rates::restart(&config, &empty, &drops).unwrap();
    assert!(rates.is_empty() && setup.is_empty());
    let imu = ImuPublication {
        entity_name: "manual-imu".into(),
        frame_id: "manual-imu".into(),
        topic: "/manual/imu".into(),
        enable_orientation: true,
    };
    let imu_entity = empty.world_mut().spawn((imu, EntityKind::Sensor)).id();
    let imu_id = gz_sim_core::assign_id(empty.world_mut(), imu_entity).unwrap();
    let fixture = loaded(1);
    let (_, _, metadata) = sensor(&fixture, 0);
    let pending = empty
        .world_mut()
        .spawn((metadata, EntityKind::Sensor, PendingResetRemoval))
        .id();
    let before = snapshot(&empty, pending);
    let (mut rates, setup) = Rates::restart(&config, &empty, &drops).unwrap();
    assert_eq!(snapshot(&empty, pending), before);
    assert_eq!(rates.providers.len(), 1);
    assert!(matches!(&rates.providers[0].family, Family::Imu));
    assert_eq!(rates.providers[0].id, Some(imu_id));
    assert_eq!(setup.providers.len(), 1);
    assert_eq!(
        setup.providers[0].key,
        config.topic_key("/manual/imu/set_rate").unwrap()
    );
    // Public metadata alone is not a private Scene IMU. The legacy CPU path
    // retains its typed rejection, and the new lidar family remains absent.
    setup.providers[0].input.try_send(Some(50.0)).unwrap();
    let mut counters = Counters::default();
    rates
        .poll(&mut empty, NonZeroUsize::new(1).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(counters.imu_rates_rejected, 1);
    assert_eq!(
        (
            counters.lidar_rates_applied,
            counters.lidar_rates_ignored,
            counters.lidar_rates_rejected
        ),
        (0, 0, 0)
    );
    assert_eq!(snapshot(&empty, pending), before);
    assert_eq!(drops.load(Ordering::Relaxed), 0);
    rates.shutdown();
    server.shutdown();
}

#[test]
fn lidar_rate_preflight_refuses_corrupt_owners_and_preserves_queued_sensor_continuation() {
    let config = options();
    let mut server = host(&config);
    let mut sim = loaded(2);
    let mut twin = loaded(2);
    let (entity, id, metadata) = sensor(&sim, 0);
    let (twin_entity, twin_id, _) = sensor(&twin, 0);
    sim.step_exact(1).unwrap();
    twin.step_exact(1).unwrap();
    assert_wall(&sim, entity, Duration::from_millis(1));
    let originals: Vec<_> = gz_sim_scene::initial_lidar_publications(&sim)
        .unwrap()
        .cloned()
        .collect();
    let drops = Arc::new(AtomicU64::new(0));
    let (mut rates, setup) =
        Rates::prepare(&config, std::iter::empty(), &originals, &drops).unwrap();
    rates.bind(&sim).unwrap();
    let selected = setup
        .providers
        .iter()
        .find(|p| p.key == lidar_endpoint(&config, &metadata).unwrap())
        .unwrap();
    selected.input.try_send(Some(50.0)).unwrap();
    let keys = server.world_io.reserved_keys();
    let reject = |sim: &Simulation, config: &ServerOptions| {
        let before = snapshot(sim, entity);
        assert!(matches!(
            crate::validate_lidar_rate_topics(sim, &server.world_io, config),
            Err(Error::LidarIdentity)
        ));
        assert_eq!(snapshot(sim, entity), before);
        assert_eq!(server.world_io.reserved_keys(), keys);
    };
    sim.world_mut()
        .get_mut::<LidarPublication>(entity)
        .unwrap()
        .entity_name
        .clear();
    reject(&sim, &config);
    *sim.world_mut().get_mut::<LidarPublication>(entity).unwrap() = metadata.clone();
    sim.world_mut().entity_mut(entity).insert(EntityKind::Link);
    reject(&sim, &config);
    sim.world_mut()
        .entity_mut(entity)
        .insert(EntityKind::Sensor);
    let reserved_id = gz_sim_core::reserve_id(sim.world(), entity).unwrap();
    assert!(
        sim.world_mut()
            .entity_mut(entity)
            .take::<EntityId>()
            .is_some()
    );
    reject(&sim, &config);
    let bound_ids: Vec<_> = rates.providers.iter().map(|provider| provider.id).collect();
    assert!(matches!(rates.bind(&sim), Err(Error::LidarIdentity)));
    assert_eq!(
        rates
            .providers
            .iter()
            .map(|provider| provider.id)
            .collect::<Vec<_>>(),
        bound_ids
    );
    assert!(
        setup
            .providers
            .iter()
            .all(|provider| provider.active.load(Ordering::Acquire))
    );
    assert_eq!(
        gz_sim_core::restore_id(sim.world_mut(), entity, &reserved_id).unwrap(),
        id
    );
    let (_, _, second) = sensor(&sim, 1);
    let mut aliased = config.clone();
    aliased
        .add_topic_remap(
            &format!("{}/set_rate", second.topic),
            &format!("{}/set_rate", metadata.topic),
        )
        .unwrap();
    reject(&sim, &aliased);
    let invalid = ServerOptions {
        partition: "/".into(),
        ..config.clone()
    };
    let before_invalid = snapshot(&sim, entity);
    assert!(matches!(
        crate::validate_lidar_rate_topics(&sim, &server.world_io, &invalid),
        Err(Error::Setup(SetupError::InvalidName))
    ));
    assert_eq!(snapshot(&sim, entity), before_invalid);
    assert_eq!(server.world_io.reserved_keys(), keys);
    crate::validate_lidar_rate_topics(&sim, &server.world_io, &config).unwrap();
    assert!(server.rates.is_none() && server.lidars.is_empty());
    assert_eq!(lidar_rate(&sim, id).unwrap().rate_hz, 100.0);
    assert_eq!(sample(&sim, entity), sample(&twin, twin_entity));
    // The refusal/repair transaction neither assigned nor consumed a public ID.
    let probe = sim.world_mut().spawn_empty().id();
    let twin_probe = twin.world_mut().spawn_empty().id();
    assert_eq!(
        gz_sim_core::assign_id(sim.world_mut(), probe).unwrap(),
        gz_sim_core::assign_id(twin.world_mut(), twin_probe).unwrap()
    );
    let mut counters = Counters::default();
    rates
        .poll(&mut sim, NonZeroUsize::new(2).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(
        (
            counters.lidar_rates_applied,
            counters.lidar_rates_ignored,
            counters.lidar_rates_rejected
        ),
        (1, 0, 0)
    );
    assert_eq!(lidar_rate(&sim, id).unwrap().rate_hz, 50.0);
    assert_eq!(
        gz_sim_scene::set_lidar_rate(&mut twin, twin_id, 50.0).unwrap(),
        RateChange::Applied
    );
    continuation(&mut sim, entity, &mut twin, twin_entity);
    assert!(
        setup
            .providers
            .iter()
            .all(|p| p.active.load(Ordering::Acquire))
    );
    assert_eq!(drops.load(Ordering::Relaxed), 0);
    rates.shutdown();
    server.shutdown();
}

#[test]
fn pending_loaded_lidar_keeps_its_original_control_reservation_until_honest_restore() {
    let config = options();
    let mut server = host(&config);
    let mut sim = loaded(1);
    let mut twin = loaded(1);
    let (entity, id, metadata) = sensor(&sim, 0);
    let (twin_entity, _, _) = sensor(&twin, 0);
    sim.world_mut()
        .entity_mut(entity)
        .insert(PendingResetRemoval);
    let before = snapshot(&sim, entity);
    let roles = crate::new_system_roles(&sim).unwrap();
    let rate_roles: Vec<_> = roles
        .iter()
        .filter(|r| r.0 == crate::PublicationFamily::LidarRate)
        .collect();
    assert_eq!(rate_roles.len(), 1);
    assert_eq!(rate_roles[0].1, id);
    let rate = lidar_endpoint(&config, &metadata).unwrap();
    crate::validate_lidar_rate_topics(&sim, &server.world_io, &config).unwrap();
    let peer_keys = crate::typed_reserved_keys(
        &sim,
        &server.world_io,
        &config,
        crate::PublicationFamily::Altimeter,
    )
    .unwrap();
    assert!(peer_keys.contains(&rate));
    let own_keys = crate::lidar_rate_reserved_keys(&sim, &server.world_io, &config).unwrap();
    assert!(!own_keys.contains(&rate));
    assert!(own_keys.contains(&config.topic_key(&metadata.topic).unwrap()));
    assert!(
        own_keys.contains(
            &config
                .topic_key(&format!("{}/points", metadata.topic))
                .unwrap()
        )
    );
    let mut collision = config.clone();
    collision
        .add_topic_remap(&format!("{}/set_rate", metadata.topic), "/clock")
        .unwrap();
    assert!(matches!(
        crate::validate_lidar_rate_topics(&sim, &server.world_io, &collision),
        Err(Error::LidarIdentity)
    ));
    assert_eq!(snapshot(&sim, entity), before);
    sim.world_mut()
        .entity_mut(entity)
        .remove::<PendingResetRemoval>();
    crate::validate_lidar_rate_topics(&sim, &server.world_io, &config).unwrap();
    sim.step_exact(1).unwrap();
    twin.step_exact(1).unwrap();
    assert_eq!(sample(&sim, entity), sample(&twin, twin_entity));
    assert_wall(&sim, entity, Duration::from_millis(1));
    assert_eq!(lidar_rate(&sim, id).unwrap().rate_hz, 100.0);
    assert!(server.rates.is_none());
    server.shutdown();
}

#[test]
fn absent_prepared_lidar_retires_queued_handoff_and_repair_requires_fresh_provider() {
    let config = options();
    let mut sim = loaded(1);
    let mut twin = loaded(1);
    let (entity, id, metadata) = sensor(&sim, 0);
    let (twin_entity, twin_id, _) = sensor(&twin, 0);
    sim.step_exact(1).unwrap();
    twin.step_exact(1).unwrap();
    let drops = Arc::new(AtomicU64::new(0));
    let (mut rates, old_setup) =
        Rates::prepare(&config, std::iter::empty(), [&metadata], &drops).unwrap();
    old_setup.providers[0].input.try_send(Some(25.0)).unwrap();
    assert!(
        sim.world_mut()
            .entity_mut(entity)
            .take::<LidarPublication>()
            .is_some()
    );
    let before = snapshot(&sim, entity);
    rates.bind(&sim).unwrap();
    assert!(rates.is_empty());
    assert!(!old_setup.providers[0].active.load(Ordering::Acquire));
    assert!(matches!(
        old_setup.providers[0].input.try_send(Some(10.0)),
        Err(mpsc::TrySendError::Disconnected(_))
    ));
    assert_eq!(snapshot(&sim, entity), before);
    sim.world_mut().entity_mut(entity).insert(metadata);
    rates.retire(&sim);
    assert!(
        rates.is_empty(),
        "metadata repair cannot resurrect an old CPU queue"
    );
    assert_eq!(lidar_rate(&sim, id).unwrap().rate_hz, 100.0);
    assert_eq!(sample(&sim, entity), sample(&twin, twin_entity));
    let (mut fresh, setup) = Rates::restart(&config, &sim, &drops).unwrap();
    assert_eq!(fresh.providers.len(), 1);
    assert_eq!(fresh.providers[0].id, Some(id));
    setup.providers[0].input.try_send(Some(50.0)).unwrap();
    let mut counters = Counters::default();
    fresh
        .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(
        (
            counters.lidar_rates_applied,
            counters.lidar_rates_ignored,
            counters.lidar_rates_rejected
        ),
        (1, 0, 0)
    );
    assert_eq!(
        gz_sim_scene::set_lidar_rate(&mut twin, twin_id, 50.0).unwrap(),
        RateChange::Applied
    );
    continuation(&mut sim, entity, &mut twin, twin_entity);
    assert_eq!(drops.load(Ordering::Relaxed), 0);
    fresh.shutdown();
}

#[test]
fn bare_forged_lidar_metadata_cannot_authorize_a_live_or_pending_rate_owner() {
    let fixture = loaded(1);
    let (_, _, metadata) = sensor(&fixture, 0);
    let mut sim = bare();
    let entity = sim.world_mut().spawn((metadata, EntityKind::Sensor)).id();
    let id = gz_sim_core::assign_id(sim.world_mut(), entity).unwrap();
    let config = options();
    let drops = Arc::new(AtomicU64::new(0));
    let before = snapshot(&sim, entity);
    assert!(
        matches!(lidar_rate(&sim, id), Err(LidarRateError::MissingLidar(missing)) if missing == id)
    );
    assert!(matches!(
        Rates::restart(&config, &sim, &drops),
        Err(Error::LidarIdentity)
    ));
    let invalid = ServerOptions {
        partition: "/".into(),
        ..config.clone()
    };
    assert!(matches!(
        Rates::restart(&invalid, &sim, &drops),
        Err(Error::Setup(SetupError::InvalidName))
    ));
    assert_eq!(snapshot(&sim, entity), before);
    assert_eq!(
        Arc::strong_count(&drops),
        1,
        "failed preparation drops every plain handoff"
    );
    sim.world_mut()
        .entity_mut(entity)
        .insert(PendingResetRemoval);
    let before_pending = snapshot(&sim, entity);
    let (mut rates, setup) = Rates::restart(&config, &sim, &drops).unwrap();
    assert!(rates.is_empty() && setup.is_empty());
    let mut counters = Counters::default();
    rates
        .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(
        (
            counters.lidar_rates_applied,
            counters.lidar_rates_ignored,
            counters.lidar_rates_rejected
        ),
        (0, 0, 0)
    );
    assert_eq!(snapshot(&sim, entity), before_pending);
    sim.world_mut()
        .entity_mut(entity)
        .remove::<PendingResetRemoval>();
    assert!(matches!(
        Rates::restart(&config, &sim, &drops),
        Err(Error::LidarIdentity)
    ));
    assert_eq!(snapshot(&sim, entity), before);
    assert_eq!(entity_by_id(sim.world(), id), Some(entity));
    assert_eq!(drops.load(Ordering::Relaxed), 0);
}
