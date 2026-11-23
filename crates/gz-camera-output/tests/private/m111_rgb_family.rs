// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::camera_test_isolation as isolation;
use gz_sim_proto::{
    msgs::{CameraInfo, WorldControl, WorldReset},
    prost::Message,
};
use std::{
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};
fn network() -> zenoh::Config {
    zenoh::Config::from_json5(r#"{mode:'peer',listen:{endpoints:['tcp/127.0.0.1:0']},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}}}"#).unwrap()
}
fn source(count: usize) -> String {
    let xml = include_str!("../../../gz-sim-scene/tests/fixtures/camera-binding.sdf");
    let start = xml.find("<model name=\"robot\">").unwrap();
    let end = start + xml[start..].find("</model>").unwrap() + "</model>".len();
    let mut models = String::new();
    for i in 0..count {
        models += &xml[start..end]
            .replace("name=\"robot\"", &format!("name=\"robot_{i}\""))
            .replace(
                "<topic>/explicit-camera</topic>",
                &format!(
                    "<update_rate>{}</update_rate><topic>/m111/{i}/image</topic>",
                    [10, 20, 25, 50, 10][i]
                ),
            )
            .replace(
                "<camera>",
                &format!("<camera><camera_info_topic>/m111/{i}/info</camera_info_topic>"),
            );
    }
    (xml[..start].to_owned()+&models+&xml[end..]).replace("<world name=\"default\">","<world name=\"default\"><plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/>")
}
fn load(xml: &str) -> Simulation {
    gz_sim_scene::load_world_with_sdf_step(
        xml,
        "m111",
        "default",
        &sdformat::NoResources,
        gz_sim_scene::PhysicsEngine::Tpe,
    )
    .unwrap()
}
fn settings() -> CameraServerOptions {
    CameraServerOptions {
        worker_path: "/nonexistent/worker".into(),
        origin: [1e9, 0., 0.],
        zenoh: network(),
    }
}
fn options() -> ServerOptions {
    ServerOptions {
        partition: format!("m111-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    }
}
fn build(sim: Simulation, s: &CameraServerOptions) -> CameraServer {
    CameraServer::new_rgb_family_with_server_session(
        sim,
        "default",
        options(),
        network(),
        NonZeroUsize::new(8).unwrap(),
        s,
    )
    .unwrap()
}
fn matching(app: &mut CameraServer, ids: &[u64], expected: (bool, bool)) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while ids
        .iter()
        .any(|id| app.camera_demand_for(*id).unwrap() != Some(expected))
    {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
}
#[test]
fn m111_explicit_cardinality_rgb_kind_and_all_topic_preflight_preserve_legacy_paths() {
    if isolation::child(
        "application::rgb_family_tests::m111_explicit_cardinality_rgb_kind_and_all_topic_preflight_preserve_legacy_paths",
        30,
    ) {
        return;
    }
    let s = settings();
    for count in [0, 1, 2, 5] {
        assert!(matches!(
            CameraServer::new_rgb_family_with_server_session(
                load(&source(count)),
                "default",
                options(),
                network(),
                NonZeroUsize::new(1).unwrap(),
                &s
            ),
            Err(CameraServerError::Camera(CameraOutputError::RgbFamilyCount))
        ));
    }
    for count in [3, 4] {
        assert!(matches!(
            CameraServer::new_with_server_session(
                load(&source(count)),
                "default",
                options(),
                network(),
                NonZeroUsize::new(1).unwrap(),
                &s
            ),
            Err(CameraServerError::Camera(
                CameraOutputError::MultipleCameras
            ))
        ));
        assert!(matches!(
            CameraServer::new_pair(
                load(&source(count)),
                "default",
                options(),
                network(),
                NonZeroUsize::new(1).unwrap(),
                &s
            ),
            Err(CameraServerError::Camera(
                CameraOutputError::PairCameraCount
            ))
        ));
    }
    let duplicate = source(3).replace("/m111/2/image", "/m111/0/info");
    assert!(matches!(
        prepare_rgb_family(&load(&duplicate), &options(), &s),
        Err(CameraServerError::Camera(
            CameraOutputError::DuplicateTopics
        ))
    ));
    let depth = source(3).replacen("type=\"camera\"", "type=\"depth_camera\"", 1);
    assert!(prepare_rgb_family(&load(&depth), &options(), &s).is_err());
}
#[test]
fn m111_three_and_four_info_only_owners_have_independent_native_cadence_without_workers() {
    if isolation::child(
        "application::rgb_family_tests::m111_three_and_four_info_only_owners_have_independent_native_cadence_without_workers",
        40,
    ) {
        return;
    }
    for count in [3, 4] {
        let s = settings();
        let mut sim = load(&source(count));
        sim.set_paused(false);
        let scheduled = sim.scheduled_system_counts();
        let mut app = build(sim, &s);
        let ids = app.rgb_family_ids.clone().unwrap();
        let session = app.server.zenoh_session();
        let infos: Vec<_> = (0..count)
            .map(|i| {
                session
                    .declare_subscriber(
                        app.server
                            .options()
                            .topic_key(&format!("/m111/{i}/info"))
                            .unwrap(),
                    )
                    .wait()
                    .unwrap()
            })
            .collect();
        matching(&mut app, &ids, (false, true));
        let mut stamps = vec![Vec::new(); count];
        for tick in 1..=200 {
            app.tick(&s).unwrap();
            for (i, row) in app.capture_all().unwrap().into_iter().enumerate() {
                if row.result.unwrap().requested == Some(Progress::InfoPublished) {
                    let sample = infos[i]
                        .recv_timeout(Duration::from_secs(2))
                        .unwrap()
                        .unwrap();
                    let packet = CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
                    let stamp = packet.header.unwrap().stamp.unwrap();
                    assert_eq!(stamp.sec, 0);
                    assert_eq!(stamp.nsec, tick * 1_000_000);
                    assert_eq!((packet.width, packet.height), (64, 64));
                    stamps[i].push(tick);
                }
            }
        }
        for (i, actual) in stamps.into_iter().enumerate() {
            let period = [100, 50, 40, 20][i];
            let expected: Vec<_> = (1..=200).filter(|t| *t == 1 || *t % period == 0).collect();
            assert_eq!(actual, expected);
        }
        assert!(
            ids.iter()
                .all(|id| app.camera_worker_diagnostics(*id).is_none())
        );
        assert_eq!(app.server.simulation().scheduled_system_counts(), scheduled);
        assert!(matches!(
            app.capture(),
            Err(CameraServerError::Camera(CameraOutputError::PairMode))
        ));
        drop(infos);
        app.shutdown();
        assert!(session.is_closed());
    }
}
#[test]
fn m111_removal_subset_and_epoch_reset_restore_original_family_on_shared_session() {
    if isolation::child(
        "application::rgb_family_tests::m111_removal_subset_and_epoch_reset_restore_original_family_on_shared_session",
        35,
    ) {
        return;
    }
    let s = settings();
    let mut sim = load(&source(4));
    let trigger = Arc::new(AtomicU64::new(0));
    let remove = Arc::clone(&trigger);
    sim.add_systems(
        gz_sim_core::Phase::Finalize,
        move |world: &mut bevy_ecs::world::World| {
            let id = remove.swap(0, Ordering::AcqRel);
            if id != 0 {
                assert!(world.despawn(entity_by_id(world, id).unwrap()));
            }
        },
    );
    let mut app = build(sim, &s);
    let ids = app.rgb_family_ids.clone().unwrap();
    let session = app.server.zenoh_session();
    trigger.store(ids[1], Ordering::Release);
    app.tick(&s).unwrap();
    assert_eq!(app.capture_all().unwrap().len(), 3);
    assert_eq!(app.camera_demand_for(ids[1]).unwrap(), None);
    assert!(!session.is_closed());
    assert!(app.server_mut().apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    app.tick(&s).unwrap();
    assert_eq!(app.rgb_family_ids.as_ref(), Some(&ids));
    assert_eq!(app.capture_all().unwrap().len(), 4);
    assert!(!session.is_closed());
    app.shutdown();
    assert!(session.is_closed());
}
#[test]
fn m111_partial_output_install_drops_all_earlier_writer_reservations() {
    if isolation::child(
        "application::rgb_family_tests::m111_partial_output_install_drops_all_earlier_writer_reservations",
        25,
    ) {
        return;
    }
    let s = settings();
    let sim = load(&source(4));
    let o = options();
    let session = zenoh::open(network()).wait().unwrap();
    let mut prepared = prepare_rgb_family(&sim, &o, &s).unwrap();
    prepared[3].config.frame_id.clear();
    assert!(matches!(
        install_pair_with_session(&sim, prepared, session.clone(), false),
        Err(CameraOutputError::InvalidFrameId)
    ));
    assert!(!session.is_closed());
    let pair = install_pair_with_session(
        &sim,
        prepare_rgb_family(&sim, &o, &s).unwrap(),
        session.clone(),
        false,
    )
    .unwrap();
    assert_eq!(pair.outputs.len(), 4);
    drop(pair);
    assert!(!session.is_closed());
    session.close().wait().unwrap();
}
#[test]
fn m111_late_public_binding_drift_retires_entire_family_before_any_output() {
    if isolation::child(
        "application::rgb_family_tests::m111_late_public_binding_drift_retires_entire_family_before_any_output",
        25,
    ) {
        return;
    }
    let s = settings();
    let mut sim = load(&source(4));
    let change = Arc::new(AtomicU64::new(0));
    let trigger = Arc::clone(&change);
    sim.add_systems(
        gz_sim_core::Phase::Finalize,
        move |world: &mut bevy_ecs::world::World| {
            let id = trigger.swap(0, Ordering::AcqRel);
            if id != 0 {
                world
                    .entity_mut(entity_by_id(world, id).unwrap())
                    .insert(LocalName("foreign".into()));
            }
        },
    );
    let mut app = build(sim, &s);
    let ids = app.rgb_family_ids.clone().unwrap();
    change.store(ids[3], Ordering::Release);
    app.tick(&s).unwrap();
    assert!(matches!(
        app.capture_all(),
        Err(CameraServerError::Camera(CameraOutputError::OwnerInvalid))
    ));
    assert!(app.pair.is_none());
    app.shutdown();
}

#[test]
fn m111_busy_first_owner_does_not_starve_three_later_info_only_outputs() {
    if isolation::child(
        "application::rgb_family_tests::m111_busy_first_owner_does_not_starve_three_later_info_only_outputs",
        30,
    ) {
        return;
    }
    let s = settings();
    let mut sim = load(&source(4));
    sim.set_paused(false);
    let mut app = build(sim, &s);
    let ids = app.rgb_family_ids.clone().unwrap();
    let session = app.server.zenoh_session();
    let image = session
        .declare_subscriber(app.server.options().topic_key("/m111/0/image").unwrap())
        .wait()
        .unwrap();
    let infos: Vec<_> = (1..4)
        .map(|i| {
            session
                .declare_subscriber(
                    app.server
                        .options()
                        .topic_key(&format!("/m111/{i}/info"))
                        .unwrap(),
                )
                .wait()
                .unwrap()
        })
        .collect();
    let until = Instant::now() + Duration::from_secs(5);
    while app.camera_demand_for(ids[0]).unwrap() != Some((true, false))
        || ids[1..]
            .iter()
            .any(|id| app.camera_demand_for(*id).unwrap() != Some((false, true)))
    {
        assert!(Instant::now() < until);
        std::thread::yield_now();
    }
    app.tick(&s).unwrap();
    let first = app.capture_all().unwrap();
    assert_eq!(
        first[0].result.as_ref().unwrap().requested,
        Some(Progress::Pending)
    );
    for info in &infos {
        info.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
    }
    for _ in 1..100 {
        app.tick(&s).unwrap();
    }
    let busy = app
        .pair
        .as_mut()
        .unwrap()
        .outputs
        .get_mut(&ids[0])
        .unwrap()
        .output
        .request_due(app.server.simulation());
    assert!(matches!(busy, Err(CameraOutputError::Busy)));
    let rows = app.capture_all().unwrap();
    assert_eq!(rows.len(), 4);
    assert!(rows[0].result.is_err());
    for row in &rows[1..] {
        assert_eq!(
            row.result.as_ref().unwrap().requested,
            Some(Progress::InfoPublished)
        );
    }
    for info in &infos {
        info.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
    }
    assert_eq!(
        app.server.simulation().elapsed(),
        Duration::from_millis(100)
    );
    app.shutdown();
    drop((image, infos));
}
