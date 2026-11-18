// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

fn shared_application(
    simulation: Simulation,
    settings: &CameraServerOptions,
) -> (CameraServer, Session) {
    let reserve = TcpListener::bind("127.0.0.2:0").unwrap();
    let endpoint = format!("tcp/{}", reserve.local_addr().unwrap());
    let mut transport = config();
    transport
        .insert_json5("listen/endpoints", &format!("[\"{endpoint}\"]"))
        .unwrap();
    drop(reserve);
    let app = CameraServer::new_with_server_session(
        simulation,
        "default",
        options(),
        transport,
        NonZeroUsize::new(1).unwrap(),
        settings,
    )
    .unwrap();
    let mut outgoing = config();
    outgoing.insert_json5("listen/endpoints", "[]").unwrap();
    outgoing
        .insert_json5("connect/endpoints", &format!("[\"{endpoint}\"]"))
        .unwrap();
    outgoing.insert_json5("connect/timeout_ms", "2000").unwrap();
    (app, zenoh::open(outgoing).wait().unwrap())
}

fn camera_ids(app: &CameraServer) -> Vec<u64> {
    if let Some(pair) = &app.pair {
        pair.outputs.keys().copied().collect()
    } else {
        vec![app.runtime.as_ref().unwrap().binding.id]
    }
}

fn matching(app: &mut CameraServer, ids: &[u64], expected: (bool, bool)) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while ids
        .iter()
        .any(|id| app.camera_demand_for(*id).unwrap() != Some(expected))
    {
        assert!(
            Instant::now() < deadline,
            "remote camera declaration did not reach the world peer"
        );
        std::thread::yield_now();
    }
    assert!(app.server().simulation().is_paused());
    assert_eq!(app.server().simulation().elapsed(), Duration::ZERO);
}

fn same_session(app: &CameraServer, world: &Session) {
    if let Some(pair) = &app.pair {
        assert_eq!(pair.session.zid(), world.zid());
        assert!(!pair.owns_session);
    } else {
        let runtime = app.runtime.as_ref().unwrap();
        assert_eq!(runtime.session.zid(), world.zid());
        assert!(!runtime.owns_session);
    }
}

#[test]
fn remote_image_and_info_interest_share_world_routing_before_exact_first_step() {
    if isolation::child(
        "application::tests::shared_server_session::remote_image_and_info_interest_share_world_routing_before_exact_first_step",
        45,
    ) {
        return;
    }
    for (source, topics) in [
        (xml(true), vec![("/explicit-camera", "/camera_info")]),
        (
            pair_xml(),
            vec![
                ("/m63/a/image", "/m63/a/camera_info"),
                ("/m63/b/image", "/m63/b/camera_info"),
            ],
        ),
        (
            depth_pair_xml(),
            vec![
                ("/m69/a/image", "/m69/a/camera_info"),
                ("/m69/b/image", "/m69/b/camera_info"),
            ],
        ),
    ] {
        let mut settings = settings();
        // An existing test worker exercises real image demand without building
        // a renderer. No image packet or numerical render result is claimed.
        settings.worker_path = "/bin/cat".into();
        let unrelated = TcpListener::bind("127.0.0.1:0").unwrap();
        settings
            .zenoh
            .insert_json5(
                "listen/endpoints",
                &format!("[\"tcp/{}\"]", unrelated.local_addr().unwrap()),
            )
            .unwrap();
        let mut simulation = load(&source);
        simulation.set_paused(true);
        let (mut app, observer) = shared_application(simulation, &settings);
        let world = app.server().zenoh_session();
        same_session(&app, &world);
        let ids = camera_ids(&app);
        let images: Vec<_> = topics
            .iter()
            .map(|(topic, _)| {
                observer
                    .declare_subscriber(app.server().options().topic_key(topic).unwrap())
                    .wait()
                    .unwrap()
            })
            .collect();
        let infos: Vec<_> = topics
            .iter()
            .map(|(_, topic)| {
                observer
                    .declare_subscriber(app.server().options().topic_key(topic).unwrap())
                    .wait()
                    .unwrap()
            })
            .collect();
        matching(&mut app, &ids, (true, true));
        assert!(
            ids.iter()
                .all(|id| app.camera_worker_diagnostics(*id).is_none())
        );
        assert!(app.server_mut().apply_local_control(WorldControl {
            pause: true,
            step: true,
            ..Default::default()
        }));
        let update = app.tick(&settings).unwrap();
        assert_eq!(update.sim_time, Duration::from_millis(1));
        assert!(!update.paused);
        let captures = app.capture_all().unwrap();
        assert_eq!(captures.len(), ids.len());
        for row in captures {
            assert_eq!(row.result.unwrap().requested, Some(Progress::Pending));
        }
        for (image, info) in images.iter().zip(&infos) {
            let sample = info.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
            let info = CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
            assert_eq!((info.width, info.height), (64, 64));
            assert_eq!(info.header.unwrap().stamp.unwrap().nsec, 1_000_000);
            assert!(image.try_recv().unwrap().is_none());
        }
        assert_eq!(
            app.server().simulation().elapsed(),
            Duration::from_millis(1)
        );
        assert!(app.server().simulation().is_paused());
        app.shutdown();
        assert!(world.is_closed());
        assert_eq!(
            TcpListener::bind(unrelated.local_addr().unwrap())
                .unwrap_err()
                .kind(),
            std::io::ErrorKind::AddrInUse
        );
    }
}

#[test]
fn shared_reset_and_camera_removal_keep_world_session_until_explicit_shutdown() {
    if isolation::child(
        "application::tests::shared_server_session::shared_reset_and_camera_removal_keep_world_session_until_explicit_shutdown",
        40,
    ) {
        return;
    }
    for (source, topics) in [
        (xml(true), vec!["/camera_info"]),
        (pair_xml(), vec!["/m63/a/camera_info", "/m63/b/camera_info"]),
        (
            depth_pair_xml(),
            vec!["/m69/a/camera_info", "/m69/b/camera_info"],
        ),
    ] {
        let settings = settings();
        let mut simulation = load(&source);
        simulation.set_paused(true);
        let ids: Vec<_> = simulation
            .world()
            .iter_entities()
            .filter(|e| e.contains::<CameraSensor>())
            .map(|e| e.get::<EntityId>().unwrap().get())
            .collect();
        let remove = Arc::new(AtomicBool::new(false));
        let trigger = Arc::clone(&remove);
        simulation.add_systems(
            gz_sim_core::Phase::Finalize,
            move |world: &mut bevy_ecs::world::World| {
                if trigger.swap(false, Ordering::AcqRel) {
                    for id in &ids {
                        assert!(world.despawn(entity_by_id(world, *id).unwrap()));
                    }
                }
            },
        );
        let (mut app, observer) = shared_application(simulation, &settings);
        let world = app.server().zenoh_session();
        let ids = camera_ids(&app);
        let subscribers: Vec<_> = topics
            .iter()
            .map(|topic| {
                observer
                    .declare_subscriber(app.server().options().topic_key(topic).unwrap())
                    .wait()
                    .unwrap()
            })
            .collect();
        matching(&mut app, &ids, (false, true));
        assert!(app.server_mut().apply_local_control(WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        }));
        app.prepare_io(&settings).unwrap();
        assert_eq!(app.server().simulation().epoch(), 1);
        assert!(
            !world.is_closed(),
            "camera reset must not close the shared world session"
        );
        same_session(&app, &world);
        let reset = app.tick(&settings).unwrap();
        assert_eq!(reset.sim_time, Duration::ZERO);
        assert!(
            app.capture_all().unwrap().iter().all(|r| r
                .result
                .as_ref()
                .unwrap()
                .requested
                .is_none())
        );
        matching(&mut app, &ids, (false, true));
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
        for subscriber in &subscribers {
            let sample = subscriber
                .recv_timeout(Duration::from_secs(2))
                .unwrap()
                .unwrap();
            let info = CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
            assert_eq!(info.header.unwrap().stamp.unwrap().nsec, 1_000_000);
        }
        remove.store(true, Ordering::Release);
        app.tick(&settings).unwrap();
        assert!(app.capture_all().unwrap().is_empty());
        assert!(!app.has_camera_output());
        assert!(
            !world.is_closed(),
            "retiring all cameras must leave world IO usable"
        );
        control(
            &mut app,
            &observer,
            WorldControl {
                pause: true,
                ..Default::default()
            },
        );
        let mut server = app.into_server();
        assert!(!world.is_closed());
        assert_eq!(server.zenoh_session().zid(), world.zid());
        server.shutdown();
        assert!(world.is_closed());
    }
}

#[test]
fn shared_absence_and_preflight_failures_do_not_open_a_camera_peer_or_world_listener() {
    if isolation::child(
        "application::tests::shared_server_session::shared_absence_and_preflight_failures_do_not_open_a_camera_peer_or_world_listener",
        35,
    ) {
        return;
    }
    let mut bad = settings();
    bad.worker_path = "relative-worker".into();
    bad.origin = [f64::NAN; 3];
    for source in [
        xml(false),
        format!("<sdf version='1.12'><world name='default'>{SENSORS}</world></sdf>"),
    ] {
        let (app, _) = shared_application(load(&source), &bad);
        assert!(!app.has_camera_output());
        assert!(app.runtime.is_none() && app.pair.is_none());
        let session = app.server().zenoh_session();
        let server = app.into_server();
        assert!(!session.is_closed());
        drop(server);
        assert!(session.is_closed());
    }
    let reserved = TcpListener::bind("127.0.0.1:0").unwrap();
    let endpoint = format!("[\"tcp/{}\"]", reserved.local_addr().unwrap());
    let mut world = config();
    world.insert_json5("listen/endpoints", &endpoint).unwrap();
    let invalid_pair = pair_xml().replacen("/m63/b/image", "invalid**", 1);
    let mixed_pair = pair_xml().replacen("type=\"camera\"", "type=\"depth_camera\"", 1);
    for source in [xml(true), pair_xml(), depth_pair_xml()] {
        let result = CameraServer::new_with_server_session(
            load(&source),
            "default",
            options(),
            world.clone(),
            NonZeroUsize::new(1).unwrap(),
            &bad,
        );
        assert!(
            matches!(result, Err(CameraServerError::Camera(_))),
            "complete camera preflight must precede the occupied world listener"
        );
    }
    for source in [invalid_pair, mixed_pair] {
        let result = CameraServer::new_with_server_session(
            load(&source),
            "default",
            options(),
            world.clone(),
            NonZeroUsize::new(1).unwrap(),
            &settings(),
        );
        assert!(
            matches!(
                result,
                Err(CameraServerError::Camera(_)) | Err(CameraServerError::Topic(_))
            ),
            "the second owner and whole family must be validated before world IO"
        );
    }
    assert_eq!(
        TcpListener::bind(reserved.local_addr().unwrap())
            .unwrap_err()
            .kind(),
        std::io::ErrorKind::AddrInUse
    );
    // With valid camera inputs the same concrete listener must fail at world IO.
    for source in [xml(true), pair_xml(), depth_pair_xml()] {
        assert!(matches!(
            CameraServer::new_with_server_session(
                load(&source),
                "default",
                options(),
                world.clone(),
                NonZeroUsize::new(1).unwrap(),
                &settings(),
            ),
            Err(CameraServerError::Server(
                gz_sim_server::Error::WorldNetwork(_)
            ))
        ));
    }
}

#[test]
fn legacy_separate_camera_sessions_keep_configured_peer_and_close_ownership() {
    if isolation::child(
        "application::tests::shared_server_session::legacy_separate_camera_sessions_keep_configured_peer_and_close_ownership",
        35,
    ) {
        return;
    }
    for family in 0..3 {
        let settings = settings();
        let mut app = match family {
            0 => CameraServer::new(
                load(&xml(true)),
                "default",
                options(),
                config(),
                NonZeroUsize::new(1).unwrap(),
                &settings,
            )
            .unwrap(),
            1 => pair_application(load(&pair_xml()), &settings),
            _ => depth_pair_application(load(&depth_pair_xml()), &settings),
        };
        let world = app.server().zenoh_session();
        let camera = app
            .runtime
            .as_ref()
            .map(|r| r.session.clone())
            .unwrap_or_else(|| app.pair.as_ref().unwrap().session.clone());
        assert_ne!(camera.zid(), world.zid());
        if let Some(runtime) = &app.runtime {
            assert!(runtime.owns_session);
        }
        if let Some(pair) = &app.pair {
            assert!(pair.owns_session);
        }
        app.runtime = None;
        app.pair = None;
        assert!(camera.is_closed());
        assert!(!world.is_closed());
        app.shutdown();
        assert!(world.is_closed());
    }
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
            Ok(None) => panic!("camera publisher snapshot did not finish"),
            Err(error) => {
                assert!(replies.is_disconnected(), "{error}");
                assert!(!session.is_closed());
                return count;
            }
        }
    }
}

#[test]
fn partial_shared_pair_install_retires_first_publisher_without_closing_world_io() {
    if isolation::child(
        "application::tests::shared_server_session::partial_shared_pair_install_retires_first_publisher_without_closing_world_io",
        35,
    ) {
        return;
    }
    let settings = settings();
    let mut simulation = load(&pair_xml());
    simulation.set_paused(true);
    let options = options();
    let mut prepared = prepare_pair(&simulation, &options, &settings).unwrap();
    let ids = [prepared[0].binding.id, prepared[1].binding.id];
    let first_key = prepared[0].config.image_key.clone();
    // Both owners passed complete application preflight. Corrupt only the
    // second private installation input so the first has already declared its
    // real publisher and native discovery token before this later failure.
    prepared[1].config.frame_id.clear();
    let mut transport = config();
    transport.insert_json5("listen/endpoints", "[]").unwrap();
    let mut server = Server::with_external_camera_outputs(
        simulation,
        "default",
        options,
        transport,
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
    assert_eq!(
        camera_tokens(&shared, &first_key),
        0,
        "the successfully installed first owner's discovery must retire on later failure"
    );

    let prepared = prepare_pair(server.simulation(), server.options(), &settings).unwrap();
    let pair =
        install_pair_with_session(server.simulation(), prepared, shared.clone(), false).unwrap();
    assert_eq!(
        camera_tokens(&shared, &first_key),
        1,
        "replacement must have exactly its new publisher, with no leaked first declaration"
    );
    drop(pair);
    assert!(!shared.is_closed());
    assert_eq!(camera_tokens(&shared, &first_key), 0);

    let clocks = shared
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
            "world control stopped after partial camera installation"
        );
        assert_eq!(server.update().unwrap().sim_time, Duration::ZERO);
        let sample = clocks
            .recv_timeout(Duration::from_secs(2))
            .unwrap()
            .unwrap();
        let clock =
            gz_sim_proto::msgs::Clock::decode(sample.payload().to_bytes().as_ref()).unwrap();
        let stamp = clock.sim.unwrap();
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
    let sample = clocks
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
        .unwrap();
    let clock = gz_sim_proto::msgs::Clock::decode(sample.payload().to_bytes().as_ref()).unwrap();
    let stamp = clock.sim.unwrap();
    assert_eq!((stamp.sec, stamp.nsec), (0, 1_000_000));
    server.shutdown();
    assert!(shared.is_closed());
}
