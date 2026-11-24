// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::{LidarPublication, PhysicsEngine, lidar_rate, load_world_with_sdf_step};
use std::time::Duration;

fn config() -> zenoh::Config {
    let mut c = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        c.insert_json5(key, value).unwrap();
    }
    c
}
fn xml(count: usize, imus: bool) -> String {
    let sensors=(0..count).map(|i|format!("<sensor name='scan{i}' type='lidar'><update_rate>100</update_rate><topic>/rate/scan{i}</topic><frame_id>frame{i}</frame_id><lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></horizontal></scan><range><min>0</min><max>20</max></range></lidar></sensor>")).collect::<String>();
    let imus = if imus {
        "<sensor name='imu0' type='imu'><update_rate>100</update_rate><topic>/imu-shared</topic><imu/></sensor><sensor name='imu1' type='imu'><update_rate>100</update_rate><topic>/imu-shared</topic><imu/></sensor>"
    } else {
        ""
    };
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='rig'><static>true</static><link name='body'><collision name='self'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision>{sensors}{imus}</link></model><model name='wall'><static>true</static><pose>5 0 0 0 0 0</pose><link name='body'><collision name='wall'><geometry><box><size>2 100 100</size></box></geometry></collision></link></model></world></sdf>"
    )
}
fn simulation(count: usize, imus: bool) -> Simulation {
    load_world_with_sdf_step(
        &xml(count, imus),
        "lidar-rates.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn metadata(sim: &Simulation) -> Vec<LidarPublication> {
    sim.world()
        .iter_entities()
        .filter_map(|e| e.get::<LidarPublication>().cloned())
        .collect()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| {
            e.get::<LidarPublication>()
                .is_some_and(|m| m.entity_name == name)
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn options() -> ServerOptions {
    ServerOptions {
        partition: format!("lidar-rates-{}", uuid::Uuid::new_v4()),
        capacity: 2,
        ..Default::default()
    }
}
fn total(c: &Counters) -> u64 {
    c.lidar_rates_applied + c.lidar_rates_ignored + c.lidar_rates_rejected
}

#[test]
fn lidar_rate_endpoints_are_independent_unique_bounded_and_disjoint_from_shared_imus() {
    let sim = simulation(8, false);
    let metadata = metadata(&sim);
    let mut options = options();
    let original = lidar_endpoint(&options, &metadata[0]).unwrap();
    options
        .add_topic_remap(&metadata[0].topic, "/renamed-scan")
        .unwrap();
    assert_eq!(lidar_endpoint(&options, &metadata[0]).unwrap(), original);
    let sibling = format!("{}/set_rate", metadata[0].topic);
    options
        .add_topic_remap(&sibling, "/independent-rate")
        .unwrap();
    assert_eq!(
        lidar_endpoint(&options, &metadata[0]).unwrap(),
        options.topic_key("/independent-rate").unwrap()
    );
    let drops = Arc::new(AtomicU64::new(0));
    assert_eq!(
        Rates::prepare(&options, std::iter::empty(), &metadata, &drops)
            .unwrap()
            .0
            .providers
            .len(),
        8
    );
    let mut bad = metadata.clone();
    bad.push(metadata[0].clone());
    assert!(matches!(
        Rates::prepare(&options, std::iter::empty(), &bad, &drops),
        Err(Error::LidarIdentity)
    ));
    let mut bad = metadata.clone();
    bad[7].topic = bad[0].topic.clone();
    assert!(matches!(
        Rates::prepare(&options, std::iter::empty(), &bad, &drops),
        Err(Error::LidarIdentity)
    ));
    let imu = ImuPublication {
        entity_name: "imu".into(),
        frame_id: "imu".into(),
        topic: metadata[1].topic.clone(),
        enable_orientation: true,
    };
    assert!(matches!(
        Rates::prepare(&options, [&imu], &metadata, &drops),
        Err(Error::LidarIdentity)
    ));
    let mut bad = metadata.clone();
    bad[7].entity_name = bad[0].entity_name.clone();
    assert!(matches!(
        Rates::prepare(&options, std::iter::empty(), &bad, &drops),
        Err(Error::LidarIdentity)
    ));
    // Construct first, then use native forbidden-pair removal: this is not
    // filesystem path normalization and does not collapse // to a single slash.
    let defaults = ServerOptions {
        partition: format!("trailing-lidar-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    };
    let mut trailing = metadata
        .iter()
        .find(|m| m.topic == "/rate/scan0")
        .unwrap()
        .clone();
    trailing.topic.push('/');
    assert_eq!(
        crate::options::as_valid_topic(&format!("{}/set_rate", trailing.topic)),
        "/rate/scan0set_rate"
    );
    let expected = defaults.topic_key("/rate/scan0set_rate").unwrap();
    assert_eq!(lidar_endpoint(&defaults, &trailing).unwrap(), expected);
    let mut mapped = defaults.clone();
    mapped
        .add_topic_remap("/rate/scan0set_rate", "/remapped-rate")
        .unwrap();
    mapped
        .add_topic_remap("/remapped-rate", "/must-not-chain")
        .unwrap();
    assert_eq!(
        lidar_endpoint(&mapped, &trailing).unwrap(),
        defaults.topic_key("/remapped-rate").unwrap()
    );
    assert_eq!(
        trailing.topic, "/rate/scan0/",
        "the raw publication topic is never rewritten"
    );
    let trailing_xml =
        xml(1, false).replace("<topic>/rate/scan0</topic>", "<topic>/rate/scan0/</topic>");
    let trailing_sim = load_world_with_sdf_step(
        &trailing_xml,
        "trailing.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let rows: Vec<_> = gz_sim_scene::initial_lidar_publications(&trailing_sim)
        .unwrap()
        .cloned()
        .collect();
    assert_eq!(rows[0].topic, "/rate/scan0/");
    let (_, setup) = Rates::prepare(&defaults, std::iter::empty(), &rows, &drops).unwrap();
    assert_eq!(setup.providers[0].key, expected);
    drop(setup);
    let host = crate::Server::with_zenoh_config(
        trailing_sim,
        "default",
        defaults,
        config(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    assert_eq!(host.lidars[0].metadata.topic, "/rate/scan0/");
    assert_eq!(
        host.rates.as_ref().unwrap().providers.len(),
        1,
        "default admission survives without a rate request"
    );
    let options = ServerOptions {
        capacity: 0,
        ..options
    };
    assert!(matches!(
        Rates::prepare(&options, std::iter::empty(), std::iter::empty(), &drops),
        Err(Error::Setup(SetupError::InvalidConfig))
    ));
}

#[test]
fn mixed_busy_rate_queues_are_globally_budgeted_fair_and_keep_imu_provider_sharing() {
    let mut sim = simulation(8, true);
    let lidar = metadata(&sim);
    let imu: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<ImuPublication>().cloned())
        .collect();
    let drops = Arc::new(AtomicU64::new(0));
    let (mut rates, setup) = Rates::prepare(&options(), &imu, &lidar, &drops).unwrap();
    rates.bind(&sim).unwrap();
    assert_eq!(rates.providers.len(), 10);
    assert_eq!(
        setup
            .providers
            .iter()
            .filter(|p| p.key.ends_with("/imu-shared/set_rate"))
            .count(),
        2
    );
    for p in &setup.providers {
        p.input.try_send(Some(50.)).unwrap();
        p.input.try_send(Some(25.)).unwrap();
    }
    let mut counters = Counters::default();
    rates
        .poll(&mut sim, NonZeroUsize::new(3).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(total(&counters) + counters.imu_rates_applied, 3);
    for _ in 0..3 {
        rates
            .poll(&mut sim, NonZeroUsize::new(3).unwrap(), &mut counters)
            .unwrap();
    }
    // Ten busy owners have each had one turn before a second complete round.
    assert!(
        lidar
            .iter()
            .all(|m| lidar_rate(&sim, id(&sim, &m.entity_name)).unwrap().rate_hz <= 50.)
    );
    for _ in 0..8 {
        rates
            .poll(&mut sim, NonZeroUsize::new(3).unwrap(), &mut counters)
            .unwrap();
    }
    assert_eq!(
        (counters.imu_rates_applied, counters.lidar_rates_applied),
        (4, 16)
    );
    assert_eq!(
        (
            counters.imu_rates_rejected,
            counters.lidar_rates_rejected,
            counters.lidar_rates_ignored
        ),
        (0, 0, 0)
    );
    assert!(
        lidar
            .iter()
            .all(|m| lidar_rate(&sim, id(&sim, &m.entity_name)).unwrap().rate_hz == 25.)
    );
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(drops.load(Ordering::Relaxed), 0);
}

#[test]
fn lidar_binding_preflights_late_metadata_drift_and_retirement_is_family_specific() {
    let mut sim = simulation(8, false);
    let metadata = metadata(&sim);
    let drops = Arc::new(AtomicU64::new(0));
    let (mut rates, _setup) =
        Rates::prepare(&options(), std::iter::empty(), &metadata, &drops).unwrap();
    let last = id(&sim, &metadata[7].entity_name);
    let entity = entity_by_id(sim.world(), last).unwrap();
    sim.world_mut()
        .get_mut::<LidarPublication>(entity)
        .unwrap()
        .topic = "/changed".into();
    assert!(matches!(rates.bind(&sim), Err(Error::LidarIdentity)));
    assert!(
        rates
            .providers
            .iter()
            .all(|p| p.id.is_none() && p.active.load(Ordering::Acquire))
    );
    *sim.world_mut().get_mut::<LidarPublication>(entity).unwrap() = metadata[7].clone();
    rates.bind(&sim).unwrap();
    let active = Arc::clone(
        &rates
            .providers
            .iter()
            .find(|p| p.id == Some(last))
            .unwrap()
            .active,
    );
    sim.world_mut()
        .get_mut::<LidarPublication>(entity)
        .unwrap()
        .frame_id
        .push_str("-drift");
    rates.retire(&sim);
    assert_eq!(rates.providers.len(), 7);
    assert!(!active.load(Ordering::Acquire));
    *sim.world_mut().get_mut::<LidarPublication>(entity).unwrap() = metadata[7].clone();
    rates.retire(&sim);
    assert_eq!(
        rates.providers.len(),
        7,
        "repair cannot resurrect a retired provider"
    );
    let (replacement, setup) = Rates::restart(&options(), &sim, &drops).unwrap();
    assert_eq!(replacement.providers.len(), 8);
    assert_eq!(setup.providers.len(), 8);
    let mut fake = simulation(1, false);
    let fake_metadata = metadata[0].clone();
    let original = entity_by_id(fake.world(), id(&fake, &fake_metadata.entity_name)).unwrap();
    fake.world_mut()
        .entity_mut(original)
        .remove::<LidarPublication>();
    let forged = fake
        .world_mut()
        .spawn((fake_metadata.clone(), gz_sim_scene::EntityKind::Sensor))
        .id();
    gz_sim_core::assign_id(fake.world_mut(), forged).unwrap();
    let (mut replacement, _) =
        Rates::prepare(&options(), std::iter::empty(), [&fake_metadata], &drops).unwrap();
    assert!(matches!(replacement.bind(&fake), Err(Error::LidarIdentity)));
    assert!(replacement.providers.iter().all(|p| p.id.is_none()));
}

#[test]
fn real_lidar_double_empty_receipt_precedes_cpu_policy_and_invalid_markers_are_not_success() {
    let mut sim = simulation(1, false);
    let metadata = metadata(&sim);
    let identity = id(&sim, &metadata[0].entity_name);
    let options = ServerOptions {
        max_message_bytes: 16,
        ..options()
    };
    let drops = Arc::new(AtomicU64::new(0));
    let (mut rates, setup) =
        Rates::prepare(&options, std::iter::empty(), &metadata, &drops).unwrap();
    rates.bind(&sim).unwrap();
    let session = zenoh::open(config()).wait().unwrap();
    let (sender, incoming) = mpsc::sync_channel(2);
    let mut declarations = RateDeclarations::new(&session, setup, sender).unwrap();
    let key = lidar_endpoint(&options, &metadata[0]).unwrap();
    let mut counters = Counters::default();
    for (bytes, receipt, expected, rate) in [
        (
            Double {
                data: 50.,
                ..Default::default()
            }
            .encode_to_vec(),
            true,
            (1, 0, 0),
            50.,
        ),
        (
            Double {
                data: 101.,
                ..Default::default()
            }
            .encode_to_vec(),
            true,
            (1, 1, 0),
            50.,
        ),
        (vec![0xff], true, (1, 1, 1), 50.),
        (vec![0; 17], false, (1, 1, 2), 50.),
    ] {
        let before = lidar_rate(&sim, identity).unwrap().rate_hz;
        let replies = session
            .get(key.clone())
            .payload(bytes)
            .timeout(Duration::from_secs(5))
            .wait()
            .unwrap();
        let world_io::ControlWork::Rate(request) =
            incoming.recv_timeout(Duration::from_secs(5)).unwrap()
        else {
            panic!("rate IO request");
        };
        assert_eq!(request.query.is_some(), receipt);
        request.execute().unwrap();
        if receipt {
            let reply = replies.recv().unwrap();
            let sample = reply.result().unwrap();
            assert!(sample.payload().is_empty() && sample.attachment().is_none());
        }
        assert!(replies.recv().is_err());
        assert_eq!(lidar_rate(&sim, identity).unwrap().rate_hz, before);
        rates
            .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
            .unwrap();
        assert_eq!(
            (
                counters.lidar_rates_applied,
                counters.lidar_rates_ignored,
                counters.lidar_rates_rejected
            ),
            expected
        );
        assert_eq!(lidar_rate(&sim, identity).unwrap().rate_hz, rate);
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
    assert_eq!(
        counters.imu_rates_applied + counters.imu_rates_rejected + counters.imu_rates_ignored,
        0
    );
    rates.clear();
    assert!(declarations.reconcile() && declarations.is_empty());
}

#[test]
fn lidar_rate_ingress_and_cpu_pressure_keep_receipt_and_drop_accounting_bounded() {
    let mut sim = simulation(1, false);
    let metadata = metadata(&sim);
    let identity = id(&sim, &metadata[0].entity_name);
    let options = ServerOptions {
        capacity: 1,
        ..options()
    };
    let drops = Arc::new(AtomicU64::new(0));
    let (mut rates, setup) =
        Rates::prepare(&options, std::iter::empty(), &metadata, &drops).unwrap();
    rates.bind(&sim).unwrap();
    let session = zenoh::open(config()).wait().unwrap();
    let (sender, incoming) = mpsc::sync_channel(1);
    let mut declarations = RateDeclarations::new(&session, setup, sender.clone()).unwrap();
    let key = lidar_endpoint(&options, &metadata[0]).unwrap();
    let bytes = Double {
        data: 50.,
        ..Default::default()
    }
    .encode_to_vec();
    let first = session
        .get(key.clone())
        .payload(bytes.clone())
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let held = incoming.recv_timeout(Duration::from_secs(5)).unwrap();
    sender.try_send(held).unwrap();
    let rejected = session
        .get(key.clone())
        .payload(bytes.clone())
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    assert!(rejected.recv().is_err());
    assert_eq!(drops.load(Ordering::Relaxed), 1);
    let world_io::ControlWork::Rate(request) = incoming.recv().unwrap() else {
        panic!("rate request");
    };
    request.execute().unwrap();
    assert!(first.recv().unwrap().result().unwrap().payload().is_empty());
    assert!(first.recv().is_err());
    assert_eq!(lidar_rate(&sim, identity).unwrap().rate_hz, 100.);
    let full = session
        .get(key)
        .payload(bytes)
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let world_io::ControlWork::Rate(request) =
        incoming.recv_timeout(Duration::from_secs(5)).unwrap()
    else {
        panic!("rate request");
    };
    request.execute().unwrap();
    assert!(full.recv().unwrap().result().is_err());
    assert!(full.recv().is_err());
    assert_eq!(drops.load(Ordering::Relaxed), 2);
    let mut counters = Counters::default();
    rates
        .poll(&mut sim, NonZeroUsize::new(1).unwrap(), &mut counters)
        .unwrap();
    assert_eq!((counters.lidar_rates_applied, total(&counters)), (1, 1));
    rates.clear();
    assert!(declarations.reconcile());
}

#[test]
fn queued_lidar_queries_retire_on_io_owner_and_old_generation_never_rebinds() {
    let mut sim = simulation(1, false);
    let metadata = metadata(&sim);
    let identity = id(&sim, &metadata[0].entity_name);
    let options = options();
    let drops = Arc::new(AtomicU64::new(0));
    let (mut rates, setup) =
        Rates::prepare(&options, std::iter::empty(), &metadata, &drops).unwrap();
    rates.bind(&sim).unwrap();
    let session = zenoh::open(config()).wait().unwrap();
    let (sender, incoming) = mpsc::sync_channel(1);
    let mut declarations = RateDeclarations::new(&session, setup, sender).unwrap();
    let replies = session
        .get(lidar_endpoint(&options, &metadata[0]).unwrap())
        .payload(
            Double {
                data: 50.,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let world_io::ControlWork::Rate(request) =
        incoming.recv_timeout(Duration::from_secs(5)).unwrap()
    else {
        panic!("rate request");
    };
    rates.clear();
    assert!(!declarations.reconcile());
    request.execute().unwrap();
    assert!(replies.recv().is_err());
    assert!(declarations.reconcile() && declarations.is_empty());
    let (mut replacement, _setup) = Rates::restart(&options, &sim, &drops).unwrap();
    let mut counters = Counters::default();
    replacement
        .poll(&mut sim, NonZeroUsize::new(2).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(total(&counters), 0);
    assert_eq!(lidar_rate(&sim, identity).unwrap().rate_hz, 100.);
    replacement.shutdown();
    assert!(replacement.is_empty());
    session.close().wait().unwrap();
    assert!(matches!(
        RateDeclarations::new(&session, _setup, mpsc::sync_channel(1).0),
        Err(Error::WorldClosed)
    ));
}

#[test]
fn empty_lidar_rate_family_has_no_providers_and_does_not_change_unrelated_imu_bind_semantics() {
    let drops = Arc::new(AtomicU64::new(0));
    let (mut rates, setup) =
        Rates::prepare(&options(), std::iter::empty(), std::iter::empty(), &drops).unwrap();
    assert!(rates.is_empty() && setup.is_empty());
    let mut sim = simulation(0, false);
    let mut counters = Counters::default();
    rates.bind(&sim).unwrap();
    rates
        .poll(&mut sim, NonZeroUsize::new(2).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(total(&counters), 0);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert!(rates.into_active().is_none());
}

#[test]
fn live_and_removed_original_lidar_rate_cross_family_collisions_fail_pure_preflight_and_repair() {
    let base = ServerOptions {
        lidar_point_clouds: true,
        ..options()
    };
    let host = crate::Server::with_zenoh_config(
        simulation(0, false),
        "default",
        base.clone(),
        config(),
        NonZeroUsize::new(2).unwrap(),
    )
    .unwrap();
    assert!(host.rates.is_none());
    let mut sim = simulation(1, true);
    let row = metadata(&sim).remove(0);
    let rate_raw = format!("{}/set_rate", row.topic);
    for removed in [false, true] {
        if removed {
            let owner = sim
                .world()
                .iter_entities()
                .find(|e| {
                    e.get::<gz_sim_scene::Name>()
                        .is_some_and(|n| n.0 == "default::rig")
                })
                .unwrap()
                .get::<EntityId>()
                .unwrap()
                .get();
            gz_sim_scene::remove_model(&mut sim, owner).unwrap();
        }
        for target in [
            "/rate/scan0",
            "/rate/scan0/points",
            "/imu-shared/set_rate",
            "/clock",
            "/world/default/control",
        ] {
            let mut bad = base.clone();
            bad.add_topic_remap(&rate_raw, target).unwrap();
            assert!(
                matches!(
                    crate::validate_lidar_rate_topics(&sim, &host.world_io, &bad),
                    Err(Error::LidarIdentity)
                ),
                "{target}, removed={removed}"
            );
        }
        let mut repaired = base.clone();
        repaired
            .add_topic_remap(&rate_raw, "/independent-lidar-control")
            .unwrap();
        crate::validate_lidar_rate_topics(&sim, &host.world_io, &repaired).unwrap();
        let occupied = crate::lidar_rate_reserved_keys(&sim, &host.world_io, &repaired).unwrap();
        assert!(!occupied.contains(&lidar_endpoint(&repaired, &row).unwrap()));
        assert!(occupied.contains(&repaired.topic_key("/rate/scan0").unwrap()));
        assert!(occupied.contains(&repaired.topic_key("/rate/scan0/points").unwrap()));
    }
    assert!(
        host.rates.is_none(),
        "pure rejection never installs a rate family"
    );
}
