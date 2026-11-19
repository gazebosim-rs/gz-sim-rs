// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::sync::{
    Arc,
    atomic::{AtomicU64, Ordering},
};

fn mixed_xml() -> String {
    let source = pair_xml()
        .replace("/m63/a/", "/m71/rgb/")
        .replace("/m63/b/", "/m71/depth/");
    let second = source.find("<model name=\"robot_b\"").unwrap();
    source[..second].to_owned()
        + &source[second..].replacen("type=\"camera\"", "type=\"depth_camera\"", 1)
}

fn mixed_application(
    sim: Simulation,
    settings: &CameraServerOptions,
    shared: bool,
) -> CameraServer {
    let constructor = if shared {
        CameraServer::new_mixed_pair_with_server_session
    } else {
        CameraServer::new_mixed_pair
    };
    constructor(
        sim,
        "default",
        options(),
        config(),
        NonZeroUsize::new(1).unwrap(),
        settings,
    )
    .unwrap()
}

fn demand(app: &mut CameraServer, id: u64, expected: (bool, bool)) {
    let until = Instant::now() + Duration::from_secs(5);
    while app.camera_demand_for(id).unwrap() != Some(expected) {
        assert!(Instant::now() < until, "mixed camera demand did not match");
        std::thread::yield_now();
    }
}

#[test]
fn mixed_pair_preflights_complete_kinds_bindings_rates_and_four_keys() {
    let source = mixed_xml();
    let settings = settings();
    let options = options();
    let prepared = prepare_mixed_pair(&load(&source), &options, &settings).unwrap();
    assert_eq!(prepared.len(), 2);
    assert!(prepared[0].binding.id < prepared[1].binding.id);
    assert_eq!(
        [prepared[0].kind, prepared[1].kind],
        [CameraKind::Rgb, CameraKind::Depth]
    );
    assert_eq!(
        prepared
            .iter()
            .flat_map(|p| [&p.config.image_key, &p.config.info_key])
            .collect::<BTreeSet<_>>()
            .len(),
        4
    );
    for homogeneous in [pair_xml(), depth_pair_xml()] {
        assert!(matches!(
            prepare_mixed_pair(&load(&homogeneous), &options, &settings),
            Err(CameraServerError::Camera(
                CameraOutputError::MixedCameraKinds
            ))
        ));
    }
    for rejected in [
        format!("<sdf version='1.12'><world name='default'>{SENSORS}</world></sdf>"),
        xml(true),
        source.replace(SENSORS, ""),
        source.replacen(
            "</world>",
            &(source[source.find("<model name=\"robot_b\"").unwrap()
                ..source.rfind("</model>").unwrap() + 8]
                .replace("robot_b", "robot_c")
                + "</world>"),
            1,
        ),
    ] {
        assert!(matches!(
            prepare_mixed_pair(&load(&rejected), &options, &settings),
            Err(CameraServerError::Camera(
                CameraOutputError::MixedCameraCount
            ))
        ));
    }
    for (from, to) in [
        ("/m71/rgb/image", "/m71/depth/image"),
        ("/m71/rgb/camera_info", "/m71/depth/camera_info"),
        ("/m71/rgb/image", "/m71/depth/camera_info"),
        ("/m71/rgb/camera_info", "/m71/depth/image"),
    ] {
        let mut remapped = super::options();
        remapped.add_topic_remap(from, to).unwrap();
        assert!(matches!(
            prepare_mixed_pair(&load(&source), &remapped, &settings),
            Err(CameraServerError::Camera(
                CameraOutputError::DuplicateTopics
            ))
        ));
    }
    let second = source.find("<model name=\"robot_b\"").unwrap();
    let invalid_topic = source[..second].to_owned()
        + &source[second..].replacen("/m71/depth/image", "invalid**", 1);
    assert!(prepare_mixed_pair(&load(&invalid_topic), &options, &settings).is_err());
    for fault in 0..5 {
        let mut sim = load(&source);
        let mut cameras = sim
            .world()
            .iter_entities()
            .filter(|e| e.contains::<CameraSensor>());
        cameras.next().unwrap();
        let second = cameras.next().unwrap().id();
        drop(cameras);
        match fault {
            0 => {
                sim.world_mut().entity_mut(second).remove::<EntityId>();
            }
            1 => {
                sim.world_mut()
                    .entity_mut(second)
                    .insert(Name("default::wrong".into()));
            }
            2 => {
                Arc::make_mut(&mut sim.world_mut().get_mut::<CameraSensor>(second).unwrap().0)
                    .update_rate = f64::NAN;
            }
            3 => {
                Arc::make_mut(&mut sim.world_mut().get_mut::<CameraSensor>(second).unwrap().0)
                    .kind = "wideanglecamera".into();
            }
            4 => {
                let mut metadata = sim.world_mut().get_mut::<CameraSensor>(second).unwrap();
                let sdformat::SensorConfig::Camera(camera) =
                    &mut Arc::make_mut(&mut metadata.0).config
                else {
                    unreachable!("loaded camera metadata");
                };
                camera.image_width = 32;
            }
            _ => unreachable!(),
        }
        assert!(prepare_mixed_pair(&sim, &options, &settings).is_err());
        assert_eq!((sim.elapsed(), sim.epoch()), (Duration::ZERO, 0));
    }
}

#[test]
fn explicit_mixed_admission_precedes_io_and_old_constructors_still_refuse_it() {
    if isolation::child(
        "application::tests::mixed_pair::explicit_mixed_admission_precedes_io_and_old_constructors_still_refuse_it",
        30,
    ) {
        return;
    }
    let occupied = TcpListener::bind("127.0.0.1:0").unwrap();
    let mut transport = config();
    transport
        .insert_json5(
            "listen/endpoints",
            &format!("[\"tcp/{}\"]", occupied.local_addr().unwrap()),
        )
        .unwrap();
    let invalid = mixed_xml().replace("/m71/depth/camera_info", "invalid**");
    for shared in [false, true] {
        let constructor = if shared {
            CameraServer::new_mixed_pair_with_server_session
        } else {
            CameraServer::new_mixed_pair
        };
        assert!(matches!(
            constructor(
                load(&invalid),
                "default",
                options(),
                transport.clone(),
                NonZeroUsize::new(1).unwrap(),
                &settings()
            ),
            Err(CameraServerError::Camera(_)) | Err(CameraServerError::Topic(_))
        ));
        assert!(matches!(
            constructor(
                load(&mixed_xml()),
                "default",
                options(),
                transport.clone(),
                NonZeroUsize::new(1).unwrap(),
                &settings()
            ),
            Err(CameraServerError::Server(
                gz_sim_server::Error::WorldNetwork(_)
            ))
        ));
    }
    for constructor in [
        CameraServer::new,
        CameraServer::new_pair,
        CameraServer::new_depth_pair,
        CameraServer::new_with_server_session,
    ] {
        assert!(matches!(
            constructor(
                load(&mixed_xml()),
                "default",
                options(),
                transport.clone(),
                NonZeroUsize::new(1).unwrap(),
                &settings()
            ),
            Err(CameraServerError::Camera(_))
        ));
    }
}

#[test]
fn mixed_info_only_keeps_independent_rates_and_no_renderer_even_when_paused() {
    if isolation::child(
        "application::tests::mixed_pair::mixed_info_only_keeps_independent_rates_and_no_renderer_even_when_paused",
        30,
    ) {
        return;
    }
    let settings = settings();
    let mut sim = load(&mixed_xml());
    sim.set_paused(true);
    let mut app = mixed_application(sim, &settings, true);
    let ids = app.pair_ids.unwrap();
    let world = app.server().zenoh_session();
    assert_eq!(app.pair.as_ref().unwrap().session.zid(), world.zid());
    assert!(!app.pair.as_ref().unwrap().owns_session);
    app.tick(&settings).unwrap();
    for row in app.capture_all().unwrap() {
        assert_eq!(row.result.unwrap().requested, None);
        assert!(app.camera_worker_diagnostics(row.sensor_id).is_none());
    }
    let subscribers: Vec<_> = ["/m71/rgb/camera_info", "/m71/depth/camera_info"]
        .iter()
        .map(|topic| {
            world
                .declare_subscriber(app.server().options().topic_key(topic).unwrap())
                .wait()
                .unwrap()
        })
        .collect();
    for id in ids {
        demand(&mut app, id, (false, true));
    }
    for refusal in [
        app.capture().map(|_| ()),
        app.update(&settings).map(|_| ()),
        app.pump().map(|_| ()),
        app.camera_demand().map(|_| ()),
    ] {
        assert!(matches!(
            refusal,
            Err(CameraServerError::Camera(CameraOutputError::PairMode))
        ));
    }
    assert_eq!(app.server().simulation().elapsed(), Duration::ZERO);
    assert!(app.server_mut().apply_local_control(WorldControl {
        pause: false,
        ..Default::default()
    }));
    let mut stamps = [Vec::new(), Vec::new()];
    for step in 1..=100_u64 {
        assert_eq!(
            app.tick(&settings).unwrap().sim_time,
            Duration::from_millis(step)
        );
        for (index, row) in app.capture_all().unwrap().into_iter().enumerate() {
            assert_eq!(row.sensor_id, ids[index]);
            let due = step == 1 || step == 100 || (index == 1 && step == 50);
            assert_eq!(
                row.result.unwrap().requested,
                due.then_some(Progress::InfoPublished)
            );
            if due {
                let sample = subscribers[index]
                    .recv_timeout(Duration::from_secs(2))
                    .unwrap()
                    .unwrap();
                let info = CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
                assert_eq!((info.width, info.height), (64, 64));
                assert_eq!(
                    info.header.unwrap().stamp.unwrap().nsec,
                    i32::try_from(step * 1_000_000).unwrap()
                );
                stamps[index].push(step);
            }
            assert!(subscribers[index].try_recv().unwrap().is_none());
            assert!(app.camera_worker_diagnostics(ids[index]).is_none());
        }
    }
    assert_eq!(stamps, [vec![1, 100], vec![1, 50, 100]]);
    assert!(app.server_mut().apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    for _ in 0..2 {
        assert!(app.tick(&settings).unwrap().dt.is_zero());
        assert!(
            app.capture_all().unwrap().into_iter().all(|row| row
                .result
                .unwrap()
                .requested
                .is_none())
        );
    }
    assert_eq!(
        app.server().simulation().elapsed(),
        Duration::from_millis(100)
    );
    app.shutdown();
    assert!(world.is_closed());
}

#[test]
fn either_mixed_image_owner_busy_or_failed_cannot_starve_its_peer() {
    if isolation::child(
        "application::tests::mixed_pair::either_mixed_image_owner_busy_or_failed_cannot_starve_its_peer",
        35,
    ) {
        return;
    }
    for selected in 0..2 {
        let mut settings = settings();
        settings.worker_path = "/bin/cat".into();
        let mut sim = load(&mixed_xml());
        sim.set_paused(false);
        let mut app = mixed_application(sim, &settings, false);
        let ids = app.pair_ids.unwrap();
        let session = app.pair.as_ref().unwrap().session.clone();
        let image_topic = if selected == 0 {
            "/m71/rgb/image"
        } else {
            "/m71/depth/image"
        };
        let info_topic = if selected == 0 {
            "/m71/depth/camera_info"
        } else {
            "/m71/rgb/camera_info"
        };
        let images = session
            .declare_subscriber(app.server().options().topic_key(image_topic).unwrap())
            .wait()
            .unwrap();
        let infos = session
            .declare_subscriber(app.server().options().topic_key(info_topic).unwrap())
            .wait()
            .unwrap();
        demand(&mut app, ids[selected], (true, false));
        demand(&mut app, ids[1 - selected], (false, true));
        app.tick(&settings).unwrap();
        let rows = app.capture_all().unwrap();
        assert_eq!(
            rows[selected].result.as_ref().unwrap().requested,
            Some(Progress::Pending)
        );
        assert_eq!(
            rows[1 - selected].result.as_ref().unwrap().requested,
            Some(Progress::InfoPublished)
        );
        infos.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
        let (_, lifetime) = app.camera_worker_diagnostics(ids[selected]).unwrap();
        assert!(!lifetime.is_retired());
        assert!(app.camera_worker_diagnostics(ids[1 - selected]).is_none());
        for _ in 1..100 {
            app.tick(&settings).unwrap();
        }
        let before = app.server().simulation().elapsed();
        let output = &mut app
            .pair
            .as_mut()
            .unwrap()
            .outputs
            .get_mut(&ids[selected])
            .unwrap()
            .output;
        assert!(matches!(
            output.request_due(app.server.simulation()),
            Err(CameraOutputError::Busy)
        ));
        let rows = app.capture_all().unwrap();
        assert!(rows[selected].result.is_err());
        assert_eq!(
            rows[1 - selected].result.as_ref().unwrap().requested,
            Some(Progress::InfoPublished)
        );
        let packet = infos.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
        assert_eq!(
            CameraInfo::decode(packet.payload().to_bytes().as_ref())
                .unwrap()
                .header
                .unwrap()
                .stamp
                .unwrap()
                .nsec,
            100_000_000
        );
        assert_eq!(app.server().simulation().elapsed(), before);
        assert!(images.try_recv().unwrap().is_none());
        drop(images);
        demand(&mut app, ids[selected], (false, false));
        assert_eq!(
            app.capture_all().unwrap()[selected]
                .result
                .as_ref()
                .unwrap()
                .pumped,
            Some(Progress::RetireRequired)
        );
        assert!(lifetime.is_retired());
        assert!(!session.is_closed());
        app.shutdown();
    }
}

#[test]
fn shared_mixed_removal_and_loaded_reset_preserve_exact_id_type_intent() {
    if isolation::child(
        "application::tests::mixed_pair::shared_mixed_removal_and_loaded_reset_preserve_exact_id_type_intent",
        35,
    ) {
        return;
    }
    let settings = settings();
    let mut sim = load(&mixed_xml());
    sim.set_paused(true);
    let removal = Arc::new(AtomicU64::new(0));
    let trigger = Arc::clone(&removal);
    sim.add_systems(
        gz_sim_core::Phase::Finalize,
        move |world: &mut bevy_ecs::world::World| {
            let id = trigger.swap(0, Ordering::AcqRel);
            if id != 0 {
                assert!(world.despawn(entity_by_id(world, id).unwrap()));
            }
        },
    );
    let mut app = mixed_application(sim, &settings, true);
    let ids = app.pair_ids.unwrap();
    let kinds = app.pair_sensor_kinds;
    let world = app.server().zenoh_session();
    removal.store(ids[0], Ordering::Release);
    app.tick(&settings).unwrap();
    let captures = app.capture_all().unwrap();
    assert_eq!(captures.len(), 1);
    assert_eq!(captures[0].sensor_id, ids[1]);
    assert_eq!(app.camera_demand_for(ids[0]).unwrap(), None);
    assert!(!world.is_closed());
    removal.store(ids[1], Ordering::Release);
    app.tick(&settings).unwrap();
    assert!(app.capture_all().unwrap().is_empty() && !app.has_camera_output());
    assert!(!world.is_closed());
    control(
        &mut app,
        &world,
        WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        },
    );
    app.prepare_io(&settings).unwrap();
    assert_eq!(
        (
            app.server().simulation().epoch(),
            app.server().simulation().elapsed()
        ),
        (1, Duration::ZERO)
    );
    assert_eq!(app.pair_ids, Some(ids));
    assert_eq!(app.pair_sensor_kinds, kinds);
    assert_eq!(app.pair_kind, PairKind::Mixed);
    assert!(!world.is_closed());
    assert!(!app.pair.as_ref().unwrap().owns_session);
    for id in ids {
        assert!(app.camera_worker_diagnostics(id).is_none());
    }
    let subscribers: Vec<_> = ["/m71/rgb/camera_info", "/m71/depth/camera_info"]
        .iter()
        .map(|topic| {
            world
                .declare_subscriber(app.server().options().topic_key(topic).unwrap())
                .wait()
                .unwrap()
        })
        .collect();
    for id in ids {
        demand(&mut app, id, (false, true));
    }
    let reset = app.tick(&settings).unwrap();
    assert_eq!(reset.sim_time, Duration::ZERO);
    assert!(reset.paused && reset.dt.is_zero());
    let captures = app.capture_all().unwrap();
    assert_eq!(captures.len(), 2);
    for (capture, id) in captures.into_iter().zip(ids) {
        assert_eq!(capture.sensor_id, id);
        assert!(capture.result.unwrap().requested.is_none());
    }
    assert!(app.server_mut().apply_local_control(WorldControl {
        pause: true,
        step: true,
        ..Default::default()
    }));
    assert_eq!(
        app.tick(&settings).unwrap().sim_time,
        Duration::from_millis(1)
    );
    assert!(
        app.capture_all()
            .unwrap()
            .into_iter()
            .all(|r| r.result.unwrap().requested == Some(Progress::InfoPublished))
    );
    for subscriber in subscribers {
        assert_eq!(
            CameraInfo::decode(
                subscriber
                    .recv_timeout(Duration::from_secs(2))
                    .unwrap()
                    .unwrap()
                    .payload()
                    .to_bytes()
                    .as_ref()
            )
            .unwrap()
            .header
            .unwrap()
            .stamp
            .unwrap()
            .nsec,
            1_000_000
        );
    }
    let mut server = app.into_server();
    assert!(!world.is_closed());
    server.shutdown();
    assert!(world.is_closed());
}

#[test]
fn mixed_drift_closed_session_and_failed_reset_never_enroll_a_different_family() {
    if isolation::child(
        "application::tests::mixed_pair::mixed_drift_closed_session_and_failed_reset_never_enroll_a_different_family",
        40,
    ) {
        return;
    }
    let settings = settings();
    for fault in 0..3 {
        let mut sim = load(&mixed_xml());
        sim.set_paused(false);
        let entity = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<CameraSensor>().is_some_and(|s| {
                    s.0.effective_type() == Some(sdformat::SensorType::DepthCamera)
                })
            })
            .unwrap()
            .id();
        let template = sim.world().get::<CameraSensor>(entity).unwrap().clone();
        if fault != 2 {
            sim.add_systems(
                gz_sim_core::Phase::Finalize,
                move |world: &mut bevy_ecs::world::World| {
                    if fault == 0 {
                        Arc::make_mut(&mut world.get_mut::<CameraSensor>(entity).unwrap().0).kind =
                            "camera".into();
                    } else {
                        let extra = world.spawn(template.clone()).id();
                        gz_sim_core::assign_id(world, extra).unwrap();
                    }
                },
            );
        }
        let mut app = mixed_application(sim, &settings, true);
        let ids = app.pair_ids;
        let kinds = app.pair_sensor_kinds;
        let world = app.server().zenoh_session();
        if fault == 2 {
            world.close().wait().unwrap();
        } else {
            app.tick(&settings).unwrap();
        }
        let before = app.server().simulation().elapsed();
        assert!(matches!(
            app.capture_all(),
            Err(CameraServerError::Camera(
                CameraOutputError::OwnerInvalid | CameraOutputError::SessionClosed
            ))
        ));
        assert!(app.pair.is_none());
        assert_eq!(app.pair_ids, ids);
        assert_eq!(app.pair_sensor_kinds, kinds);
        assert_eq!(app.server().simulation().elapsed(), before);
        assert_eq!(world.is_closed(), fault == 2);
        app.shutdown();
    }
    let mut app = mixed_application(load(&mixed_xml()), &settings, true);
    let ids = app.pair_ids;
    let kinds = app.pair_sensor_kinds;
    let world = app.server().zenoh_session();
    assert!(app.server_mut().apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    let invalid = CameraServerOptions {
        origin: [f64::NAN; 3],
        worker_path: settings.worker_path.clone(),
        zenoh: settings.zenoh.clone(),
    };
    assert!(app.prepare_io(&invalid).is_err());
    assert!(app.pair.is_none());
    assert!(!world.is_closed());
    assert_eq!(app.pair_ids, ids);
    assert_eq!(app.pair_sensor_kinds, kinds);
    assert_eq!(app.pair_kind, PairKind::Mixed);
    app.prepare_io(&settings).unwrap();
    assert_eq!(app.pair_ids, ids);
    assert_eq!(app.pair_sensor_kinds, kinds);
    app.shutdown();
}

#[test]
fn mixed_separate_peer_is_released_after_world_listener_bind_failure() {
    if isolation::child(
        "application::tests::mixed_pair::mixed_separate_peer_is_released_after_world_listener_bind_failure",
        25,
    ) {
        return;
    }
    let camera = TcpListener::bind("127.0.0.2:0").unwrap();
    let address = camera.local_addr().unwrap();
    drop(camera);
    let occupied = TcpListener::bind("127.0.0.1:0").unwrap();
    let mut settings = settings();
    settings
        .zenoh
        .insert_json5("listen/endpoints", &format!("[\"tcp/{address}\"]"))
        .unwrap();
    let mut world = config();
    world
        .insert_json5(
            "listen/endpoints",
            &format!("[\"tcp/{}\"]", occupied.local_addr().unwrap()),
        )
        .unwrap();
    assert!(matches!(
        CameraServer::new_mixed_pair(
            load(&mixed_xml()),
            "default",
            options(),
            world,
            NonZeroUsize::new(1).unwrap(),
            &settings
        ),
        Err(CameraServerError::Server(
            gz_sim_server::Error::WorldNetwork(_)
        ))
    ));
    assert!(TcpListener::bind(address).is_ok());
}

fn camera_tokens(session: &Session, image_key: &str) -> usize {
    let (partition, topic) = image_key
        .strip_prefix('@')
        .unwrap()
        .split_once('@')
        .unwrap();
    let selector = format!(
        "@gz/{}/{}/*/*/MP/%/%/%/{}/*/%/%",
        partition.replace('/', "%"),
        session.zid(),
        topic.replace('/', "%")
    );
    let deadline = Instant::now() + Duration::from_secs(2);
    let replies = session
        .liveliness()
        .get(selector)
        .timeout(Duration::from_secs(2))
        .wait()
        .unwrap();
    let mut count = 0;
    loop {
        match replies.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
            Ok(Some(reply)) => {
                reply.into_result().unwrap();
                count += 1;
            }
            Ok(None) => panic!("mixed camera publisher snapshot did not finish"),
            Err(error) => {
                assert!(replies.is_disconnected(), "{error}");
                assert!(!session.is_closed());
                return count;
            }
        }
    }
}

#[test]
fn partial_mixed_install_retires_first_publisher_without_closing_world_io() {
    if isolation::child(
        "application::tests::mixed_pair::partial_mixed_install_retires_first_publisher_without_closing_world_io",
        35,
    ) {
        return;
    }
    let settings = settings();
    let mut simulation = load(&mixed_xml());
    simulation.set_paused(true);
    let options = options();
    let mut prepared = prepare_mixed_pair(&simulation, &options, &settings).unwrap();
    let ids = [prepared[0].binding.id, prepared[1].binding.id];
    let first_key = prepared[0].config.image_key.clone();
    // Complete public preflight succeeded; only the second private installation
    // input is damaged, after the first actual publisher can be constructed.
    prepared[1].config.frame_id.clear();
    let mut server = Server::with_external_mixed_camera_outputs(
        simulation,
        "default",
        options,
        config(),
        NonZeroUsize::new(1).unwrap(),
        ids,
    )
    .unwrap();
    let shared = server.zenoh_session();
    assert!(matches!(
        install_pair_with_session(server.simulation(), prepared, shared.clone(), false),
        Err(CameraOutputError::InvalidFrameId)
    ));
    assert!(!shared.is_closed());
    assert_eq!(camera_tokens(&shared, &first_key), 0);
    let prepared = prepare_mixed_pair(server.simulation(), server.options(), &settings).unwrap();
    let pair =
        install_pair_with_session(server.simulation(), prepared, shared.clone(), false).unwrap();
    assert_eq!(camera_tokens(&shared, &first_key), 1);
    drop(pair);
    assert!(!shared.is_closed());
    assert_eq!(camera_tokens(&shared, &first_key), 0);
    let clock = shared
        .declare_subscriber(server.options().topic_key("/world/default/clock").unwrap())
        .wait()
        .unwrap();
    let replies = shared
        .get(
            server
                .options()
                .topic_key("/world/default/control")
                .unwrap(),
        )
        .payload(
            WorldControl {
                pause: false,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let before = server.counters().controls_accepted;
    let deadline = Instant::now() + Duration::from_secs(5);
    while server.counters().controls_accepted == before {
        assert!(
            Instant::now() < deadline,
            "world control stopped after partial mixed install"
        );
        assert_eq!(server.update().unwrap().sim_time, Duration::ZERO);
        let sample = clock.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
        let stamp = gz_sim_proto::msgs::Clock::decode(sample.payload().to_bytes().as_ref())
            .unwrap()
            .sim
            .unwrap();
        assert_eq!((stamp.sec, stamp.nsec), (0, 0));
        std::thread::yield_now();
    }
    assert_eq!(server.counters().controls_accepted, before + 1);
    let reply = replies
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
        .unwrap()
        .into_result()
        .unwrap();
    assert!(
        Boolean::decode(reply.payload().to_bytes().as_ref())
            .unwrap()
            .data
    );
    assert_eq!(server.update().unwrap().sim_time, Duration::from_millis(1));
    let sample = clock.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
    let stamp = gz_sim_proto::msgs::Clock::decode(sample.payload().to_bytes().as_ref())
        .unwrap()
        .sim
        .unwrap();
    assert_eq!((stamp.sec, stamp.nsec), (0, 1_000_000));
    server.shutdown();
    assert!(shared.is_closed());
}
