// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[path = "../support/isolated_camera.rs"]
mod isolation;
use gz_sim_proto::{
    msgs::{Boolean, CameraInfo, WorldControl, WorldReset},
    prost::Message,
};
use std::{
    net::TcpListener,
    time::{Duration, Instant},
};

const SENSORS: &str = "<plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/>";
fn config() -> zenoh::Config {
    let mut config = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\""),
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("connect/endpoints", "[]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(key, value).unwrap();
    }
    config
}
fn settings() -> CameraServerOptions {
    CameraServerOptions {
        worker_path: "/nonexistent/worker".into(),
        origin: [1_000_000_000., 0., 0.],
        zenoh: config(),
    }
}
fn options() -> ServerOptions {
    ServerOptions {
        partition: format!("automatic-camera-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    }
}
fn xml(configured: bool) -> String {
    let source = include_str!("../../../gz-sim-scene/tests/fixtures/camera-binding.sdf");
    if configured {
        source.replace(
            "<world name=\"default\">",
            &format!("<world name=\"default\">{SENSORS}"),
        )
    } else {
        source.into()
    }
}
fn load(source: &str) -> Simulation {
    gz_sim_scene::load_world_with_sdf_step(
        source,
        "automatic-camera.sdf",
        "default",
        &sdformat::NoResources,
        gz_sim_scene::PhysicsEngine::Tpe,
    )
    .unwrap()
}

// A separately connected control client exercises the public application API;
// no private Server control or simulation mutation is used by these tests.
fn application(sim: Simulation, settings: &CameraServerOptions) -> (CameraServer, Session) {
    let reserved = TcpListener::bind("127.0.0.2:0").unwrap();
    let endpoint = format!("tcp/{}", reserved.local_addr().unwrap());
    let mut server = config();
    server
        .insert_json5("listen/endpoints", &format!("[\"{endpoint}\"]"))
        .unwrap();
    drop(reserved);
    let app = CameraServer::new(
        sim,
        "default",
        options(),
        server,
        NonZeroUsize::new(1).unwrap(),
        settings,
    )
    .unwrap();
    let mut client = config();
    client.insert_json5("listen/endpoints", "[]").unwrap();
    client
        .insert_json5("connect/endpoints", &format!("[\"{endpoint}\"]"))
        .unwrap();
    client.insert_json5("connect/timeout_ms", "2000").unwrap();
    (app, zenoh::open(client).wait().unwrap())
}

fn control(app: &mut CameraServer, client: &Session, value: WorldControl) {
    let key = app
        .server()
        .options()
        .topic_key("/world/default/control")
        .unwrap();
    let replies = client
        .get(key)
        .payload(value.encode_to_vec())
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let before = app.server().counters().controls_accepted;
    let deadline = Instant::now() + Duration::from_secs(5);
    // One request, consumed through real frames. This is not a request retry or
    // a camera-output retry; preparation/output remains the caller's next phase.
    while app.server().counters().controls_accepted == before {
        assert!(Instant::now() < deadline, "world control was not consumed");
        app.server_mut().update().unwrap();
        std::thread::yield_now();
    }
    assert_eq!(app.server().counters().controls_accepted, before + 1);
    let sample = replies
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap()
        .into_result()
        .unwrap();
    assert!(
        Boolean::decode(sample.payload().to_bytes().as_ref())
            .unwrap()
            .data
    );
}

#[test]
fn automatic_intent_is_required_and_absent_options_own_no_camera_state() {
    if isolation::child(
        "application::tests::automatic_intent_is_required_and_absent_options_own_no_camera_state",
        20,
    ) {
        return;
    }
    let mut bad = settings();
    bad.worker_path = "relative".into();
    bad.origin = [f64::NAN; 3];
    let empty = format!("<sdf version='1.12'><world name='default'>{SENSORS}</world></sdf>");
    for source in [xml(false), empty] {
        let mut sim = load(&source);
        sim.set_paused(false);
        let counts = sim.scheduled_system_counts();
        let mut app = CameraServer::new(
            sim,
            "default",
            options(),
            config(),
            NonZeroUsize::new(1).unwrap(),
            &bad,
        )
        .unwrap();
        assert!(app.runtime.is_none());
        assert_eq!(app.camera_demand().unwrap(), None);
        for _ in 0..2 {
            let frame = app.update(&bad).unwrap();
            assert!(frame.pumped.is_none() && frame.requested.is_none());
        }
        assert_eq!(app.server().simulation().scheduled_system_counts(), counts);
        app.shutdown();
        assert!(!app.has_camera_output());
    }
}

#[test]
fn loaded_topics_follow_native_preprocessing_before_namespace_and_remap() {
    let cases = [
        (
            "",
            "",
            "/world/default/model/robot/link/body/sensor/camera/image",
            "/world/default/model/robot/link/body/sensor/camera/camera_info",
        ),
        ("/frames/rgb", "", "/frames/rgb", "/frames/camera_info"),
        ("image with spaces", "", "image_with_spaces", "/camera_info"),
        ("/frames/rgb", "/custom_info", "/frames/rgb", "/custom_info"),
    ];
    for (topic, info, image_expected, info_expected) in cases {
        let source = xml(true)
            .replace(
                "<topic>/explicit-camera</topic>",
                &format!("<topic>{topic}</topic>"),
            )
            .replace(
                "<camera>",
                &format!("<camera><camera_info_topic>{info}</camera_info_topic>"),
            );
        let mut options = options();
        options.namespace = "ns".into();
        let sim = load(&source);
        let prepared = prepare_camera(&sim, &options, &settings())
            .unwrap()
            .unwrap();
        assert_eq!(
            prepared.config.image_key,
            options.topic_key(image_expected).unwrap()
        );
        assert_eq!(
            prepared.config.info_key,
            options.topic_key(info_expected).unwrap()
        );
        assert_eq!(prepared.config.frame_id, "robot::body::camera");
    }
    let mut options = options();
    options
        .add_topic_remap("/explicit-camera", "/other/image")
        .unwrap();
    options
        .add_topic_remap("/camera_info", "/calibration")
        .unwrap();
    let sim = load(&xml(true));
    let p = prepare_camera(&sim, &options, &settings())
        .unwrap()
        .unwrap();
    assert_eq!(
        p.config.image_key,
        options.topic_key("/explicit-camera").unwrap()
    );
    assert_eq!(
        p.config.info_key,
        options.topic_key("/camera_info").unwrap()
    );
    assert!(!p.config.info_key.contains("/other/"));
}

#[test]
fn automatic_info_only_output_is_cadenced_without_a_worker() {
    if isolation::child(
        "application::tests::automatic_info_only_output_is_cadenced_without_a_worker",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(true);
    let counts = sim.scheduled_system_counts();
    let settings = settings();
    let (mut app, client) = application(sim, &settings);
    let runtime = app.runtime.as_ref().unwrap();
    let key = app.server().options().topic_key("/camera_info").unwrap();
    let (send, receive) = std::sync::mpsc::sync_channel(4);
    let listener = runtime
        .session
        .declare_subscriber(key)
        .allowed_origin(zenoh::sample::Locality::SessionLocal)
        .callback(move |sample| send.try_send(sample).unwrap())
        .wait()
        .unwrap();
    assert!(app.update(&settings).unwrap().requested.is_none());
    assert!(matches!(
        receive.try_recv(),
        Err(std::sync::mpsc::TryRecvError::Empty)
    ));
    control(
        &mut app,
        &client,
        WorldControl {
            pause: false,
            ..Default::default()
        },
    );
    for ns in [1_000_000, 2_000_000] {
        assert_eq!(
            app.update(&settings).unwrap().requested,
            Some(Progress::InfoPublished)
        );
        let sample = receive.recv_timeout(Duration::from_secs(2)).unwrap();
        let info = CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
        assert_eq!(info.width, 64);
        assert_eq!(info.height, 64);
        assert_eq!(
            info.header.as_ref().unwrap().data[0].value,
            ["robot::body::camera"]
        );
        assert_eq!(info.header.unwrap().stamp.unwrap().nsec, ns);
        // The configured worker path does not exist: image startup would fail.
        // Successful info publication therefore exercises the CPU-only path.
    }
    assert_eq!(app.server().simulation().scheduled_system_counts(), counts);
    app.shutdown();
    assert!(app.runtime.is_none());
    drop(listener);
}

#[test]
fn camera_session_is_released_when_subsequent_server_construction_fails() {
    if isolation::child(
        "application::tests::camera_session_is_released_when_subsequent_server_construction_fails",
        20,
    ) {
        return;
    }
    let reserved = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = reserved.local_addr().unwrap();
    let endpoints = format!("[\"tcp/{address}\"]");
    let mut settings = settings();
    settings
        .zenoh
        .insert_json5("listen/endpoints", &endpoints)
        .unwrap();
    let mut server = config();
    server.insert_json5("listen/endpoints", &endpoints).unwrap();
    drop(reserved);
    // Camera IO binds first. The following real Server bind must fail. Without
    // actual camera setup this constructor would instead succeed.
    let result = CameraServer::new(
        load(&xml(true)),
        "default",
        options(),
        server,
        NonZeroUsize::new(1).unwrap(),
        &settings,
    );
    assert!(matches!(
        result,
        Err(CameraServerError::Server(
            gz_sim_server::Error::WorldNetwork(_)
        ))
    ));
    // Exactly one rebind, with no wait/retry, proves constructor unwind released
    // the already-prepared camera Session's listener.
    let rebound = TcpListener::bind(address).unwrap();
    assert_eq!(rebound.local_addr().unwrap(), address);
}

#[test]
fn exposed_server_shutdown_prevents_camera_work_without_an_epoch_change() {
    if isolation::child(
        "application::tests::exposed_server_shutdown_prevents_camera_work_without_an_epoch_change",
        20,
    ) {
        return;
    }
    let settings = settings();
    let mut app = CameraServer::new(
        load(&xml(true)),
        "default",
        options(),
        config(),
        NonZeroUsize::new(1).unwrap(),
        &settings,
    )
    .unwrap();
    let camera_session = app.runtime.as_ref().unwrap().session.clone();
    let epoch = app.server().simulation().epoch();
    let elapsed = app.server().simulation().elapsed();
    assert!(!camera_session.is_closed());
    app.server_mut().shutdown();
    assert!(app.server().is_shutdown());
    assert!(app.has_camera_output()); // Not yet visited by the outer application.
    for _ in 0..2 {
        assert!(matches!(
            app.pump(),
            Err(CameraServerError::Server(gz_sim_server::Error::WorldClosed))
        ));
        assert!(matches!(
            app.update(&settings),
            Err(CameraServerError::Server(gz_sim_server::Error::WorldClosed))
        ));
        assert!(!app.has_camera_output());
        assert!(camera_session.is_closed());
        assert_eq!(app.server().simulation().epoch(), epoch);
        assert_eq!(app.server().simulation().elapsed(), elapsed);
    }
}

#[test]
fn reset_reconstruction_failure_retires_old_session_before_returning_error() {
    if isolation::child(
        "application::tests::reset_reconstruction_failure_retires_old_session_before_returning_error",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(true);
    let settings = settings();
    let (mut app, client) = application(sim, &settings);
    let old_session = app.runtime.as_ref().unwrap().session.clone();
    control(
        &mut app,
        &client,
        WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        },
    );
    let mut invalid = settings;
    invalid.origin = [f64::NAN; 3];
    assert!(matches!(
        app.prepare_io(&invalid),
        Err(CameraServerError::Camera(CameraOutputError::Snapshot(_)))
    ));
    assert_eq!(app.server().simulation().epoch(), 1);
    assert_eq!(app.server().simulation().elapsed(), Duration::ZERO);
    assert!(app.runtime.is_none());
    assert!(old_session.is_closed());
    assert_eq!(app.pump().unwrap(), None);
}

#[test]
fn committed_server_reset_error_retires_camera_io_before_return() {
    if isolation::child(
        "application::tests::committed_server_reset_error_retires_camera_io_before_return",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(true);
    sim.add_systems(
        gz_sim_core::Phase::Reset,
        |fault: bevy_ecs::prelude::Res<gz_sim_core::StepFault>| {
            fault.report(std::io::Error::other("camera application reset control"));
        },
    );
    let settings = settings();
    let (mut app, client) = application(sim, &settings);
    let old_session = app.runtime.as_ref().unwrap().session.clone();
    control(
        &mut app,
        &client,
        WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        },
    );
    assert!(matches!(
        app.prepare_io(&settings),
        Err(CameraServerError::Server(gz_sim_server::Error::Reset(
            gz_sim_scene::SceneResetError::CommittedReset
        )))
    ));
    assert_eq!(app.server().simulation().epoch(), 1);
    assert!(app.runtime.is_none());
    assert!(old_session.is_closed());
    assert_eq!(app.pump().unwrap(), None);
}

#[test]
fn preflight_refuses_invalid_keys_frames_bounds_and_multiple_cameras_before_io() {
    for replacement in [
        "<topic>@</topic>",
        "<topic></topic><gz_frame_id></gz_frame_id>",
    ] {
        let source = xml(true).replace("<topic>/explicit-camera</topic>", replacement);
        assert!(prepare_camera(&load(&source), &options(), &settings()).is_err());
    }
    let source = xml(true).replace(
        "<camera>",
        "<camera><camera_info_topic>/explicit-camera</camera_info_topic>",
    );
    assert!(matches!(
        prepare_camera(&load(&source), &options(), &settings()),
        Err(CameraServerError::Camera(
            CameraOutputError::DuplicateTopics
        ))
    ));
    let mut settings = settings();
    settings.worker_path = "relative-worker".into();
    assert!(matches!(
        prepare_camera(&load(&xml(true)), &options(), &settings),
        Err(CameraServerError::Camera(CameraOutputError::Client(_)))
    ));
    let sim = load(&xml(true));
    let mut invalid_bounds = options();
    invalid_bounds.max_message_bytes = 0;
    assert!(matches!(
        prepare_camera(&sim, &invalid_bounds, &settings),
        Err(CameraServerError::Topic(SetupError::InvalidConfig))
    ));

    let mut sim = load(&xml(true));
    let camera = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<CameraSensor>())
        .unwrap()
        .get::<CameraSensor>()
        .unwrap()
        .clone();
    sim.world_mut().spawn(camera);
    assert!(matches!(
        prepare_camera(&sim, &options(), &settings),
        Err(CameraServerError::Camera(
            CameraOutputError::MultipleCameras
        ))
    ));
}

#[test]
fn observed_removal_and_replacement_cancel_native_ownership_after_step() {
    if isolation::child(
        "application::tests::observed_removal_and_replacement_cancel_native_ownership_after_step",
        20,
    ) {
        return;
    }
    for remove in [true, false] {
        let mut sim = load(&xml(true));
        sim.set_paused(false);
        let camera = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<CameraSensor>())
            .unwrap()
            .id();
        // A real configured Bevy lifecycle system changes the Scene during the
        // committed frame. The app must validate before touching old output.
        sim.add_systems(
            gz_sim_core::Phase::Finalize,
            move |world: &mut bevy_ecs::world::World| {
                if remove {
                    world.entity_mut(camera).remove::<CameraSensor>();
                } else {
                    world
                        .entity_mut(camera)
                        .insert(Name("default::changed".into()));
                }
            },
        );
        let settings = settings();
        let mut app = CameraServer::new(
            sim,
            "default",
            options(),
            config(),
            NonZeroUsize::new(1).unwrap(),
            &settings,
        )
        .unwrap();
        let session = app.runtime.as_ref().unwrap().session.clone();
        let result = app.update(&settings);
        if remove {
            let step = result.unwrap();
            assert!(step.pumped.is_none() && step.requested.is_none());
        } else {
            assert!(matches!(
                result,
                Err(CameraServerError::Camera(CameraOutputError::OwnerInvalid))
            ));
        }
        assert_eq!(
            app.server().simulation().elapsed(),
            Duration::from_millis(1)
        );
        assert!(app.runtime.is_none());
        assert!(session.is_closed());
        assert_eq!(app.pump().unwrap(), None);
    }
}

#[test]
fn accepted_reset_refuses_old_camera_pump_before_epoch_commit_then_rebuilds() {
    if isolation::child(
        "application::tests::accepted_reset_refuses_old_camera_pump_before_epoch_commit_then_rebuilds",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(true);
    let settings = settings();
    let (mut app, client) = application(sim, &settings);
    let old_session = app.runtime.as_ref().unwrap().session.clone();
    control(
        &mut app,
        &client,
        WorldControl {
            pause: true,
            reset: Some(WorldReset {
                all: true,
                ..Default::default()
            }),
            ..Default::default()
        },
    );
    assert!(app.server().io_preparation_required());
    assert_eq!(app.server().simulation().epoch(), 0);
    assert!(matches!(
        app.pump(),
        Err(CameraServerError::Server(
            gz_sim_server::Error::IoPreparationRequired
        ))
    ));
    assert!(app.runtime.is_none());
    assert!(old_session.is_closed());
    assert_eq!(app.server().simulation().epoch(), 0);
    app.prepare_io(&settings).unwrap();
    assert!(!app.server().io_preparation_required());
    assert_eq!(app.server().simulation().epoch(), 1);
    assert!(app.runtime.is_some());
    assert!(!app.runtime.as_ref().unwrap().session.is_closed());
    let reset = app.update(&settings).unwrap();
    assert_eq!(reset.info.epoch, 1);
    assert_eq!(reset.info.sim_time, Duration::ZERO);
    assert!(reset.requested.is_none());
}

#[test]
fn reset_accepted_by_application_update_retires_before_same_frame_publication() {
    if isolation::child(
        "application::tests::reset_accepted_by_application_update_retires_before_same_frame_publication",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(false);
    let settings = settings();
    let (mut app, client) = application(sim, &settings);
    let old_session = app.runtime.as_ref().unwrap().session.clone();
    let key = app.server().options().topic_key("/camera_info").unwrap();
    let (send, receive) = std::sync::mpsc::sync_channel(8);
    let _listener = old_session
        .declare_subscriber(key)
        .allowed_origin(zenoh::sample::Locality::SessionLocal)
        .callback(move |sample| send.try_send(sample).unwrap())
        .wait()
        .unwrap();
    let key = app
        .server()
        .options()
        .topic_key("/world/default/control")
        .unwrap();
    let replies = client
        .get(key)
        .payload(
            WorldControl {
                pause: true,
                reset: Some(WorldReset {
                    all: true,
                    ..Default::default()
                }),
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap();
    let before = app.server().counters().controls_accepted;
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let result = app.update(&settings);
        let stamps: Vec<_> = receive
            .try_iter()
            .map(|sample| {
                CameraInfo::decode(sample.payload().to_bytes().as_ref())
                    .unwrap()
                    .header
                    .unwrap()
                    .stamp
                    .unwrap()
            })
            .collect();
        if app.server().counters().controls_accepted == before + 1 {
            let accepted_time = app.server().simulation().elapsed();
            eprintln!(
                "accepted reset: result={result:?}, epoch={}, time={accepted_time:?}, info_stamps={stamps:?}",
                app.server().simulation().epoch()
            );
            assert!(matches!(
                result,
                Err(CameraServerError::Server(
                    gz_sim_server::Error::IoPreparationRequired
                ))
            ));
            assert!(
                stamps.iter().all(
                    |stamp| Duration::new(stamp.sec as u64, stamp.nsec as u32) != accepted_time
                )
            );
            assert!(app.server().io_preparation_required());
            assert_eq!(app.server().simulation().epoch(), 0);
            assert!(app.runtime.is_none());
            assert!(old_session.is_closed());
            break;
        }
        result.unwrap();
        assert!(Instant::now() < deadline, "reset request was not consumed");
        std::thread::yield_now();
    }
    let sample = replies
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap()
        .into_result()
        .unwrap();
    assert!(
        Boolean::decode(sample.payload().to_bytes().as_ref())
            .unwrap()
            .data
    );
    app.prepare_io(&settings).unwrap();
    assert_eq!(app.server().simulation().epoch(), 1);
}

#[test]
fn own_matching_observation_preserves_cadence_and_retires_terminal_output() {
    if isolation::child(
        "application::tests::own_matching_observation_preserves_cadence_and_retires_terminal_output",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(false);
    let settings = settings();
    let mut app = CameraServer::new(
        sim,
        "default",
        options(),
        config(),
        NonZeroUsize::new(1).unwrap(),
        &settings,
    )
    .unwrap();
    let session = app.runtime.as_ref().unwrap().session.clone();
    assert_eq!(app.camera_demand().unwrap(), Some((false, false)));
    let key = app.server().options().topic_key("/camera_info").unwrap();
    let (send, receive) = std::sync::mpsc::sync_channel(4);
    let listener = session
        .declare_subscriber(key)
        .allowed_origin(zenoh::sample::Locality::SessionLocal)
        .callback(move |sample| send.try_send(sample).unwrap())
        .wait()
        .unwrap();
    for _ in 0..2 {
        assert_eq!(app.camera_demand().unwrap(), Some((false, true)));
        assert_eq!(app.server().simulation().elapsed(), Duration::ZERO);
        assert!(receive.try_recv().is_err());
    }
    assert_eq!(
        app.update(&settings).unwrap().requested,
        Some(Progress::InfoPublished)
    );
    let sample = receive.recv_timeout(Duration::from_secs(2)).unwrap();
    let info = CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
    assert_eq!(info.header.unwrap().stamp.unwrap().nsec, 1_000_000);
    app.server_mut().shutdown();
    assert!(matches!(
        app.camera_demand(),
        Err(CameraServerError::Server(gz_sim_server::Error::WorldClosed))
    ));
    assert!(!app.has_camera_output());
    assert!(session.is_closed());
    drop(listener);
}

#[test]
fn own_matching_observation_refuses_pending_reset_and_changed_binding() {
    if isolation::child(
        "application::tests::own_matching_observation_refuses_pending_reset_and_changed_binding",
        20,
    ) {
        return;
    }
    for reset in [true, false] {
        let mut sim = load(&xml(true));
        sim.set_paused(reset);
        if !reset {
            let camera = sim
                .world()
                .iter_entities()
                .find(|e| e.contains::<CameraSensor>())
                .unwrap()
                .id();
            sim.add_systems(
                gz_sim_core::Phase::Finalize,
                move |world: &mut bevy_ecs::world::World| {
                    world
                        .entity_mut(camera)
                        .insert(Name("default::changed".into()));
                },
            );
        }
        let settings = settings();
        let (mut app, client) = application(sim, &settings);
        let session = app.runtime.as_ref().unwrap().session.clone();
        if reset {
            control(
                &mut app,
                &client,
                WorldControl {
                    pause: true,
                    reset: Some(WorldReset {
                        all: true,
                        ..Default::default()
                    }),
                    ..Default::default()
                },
            );
            assert!(matches!(
                app.camera_demand(),
                Err(CameraServerError::Server(
                    gz_sim_server::Error::IoPreparationRequired
                ))
            ));
            assert!(app.server().io_preparation_required());
        } else {
            app.server_mut().update().unwrap();
            assert!(matches!(
                app.camera_demand(),
                Err(CameraServerError::Camera(CameraOutputError::OwnerInvalid))
            ));
        }
        assert_eq!(app.server().simulation().epoch(), 0);
        assert!(!app.has_camera_output());
        assert!(session.is_closed());
    }
}

#[test]
fn into_simulation_retires_camera_io_and_preserves_a_usable_scene() {
    if isolation::child(
        "application::tests::into_simulation_retires_camera_io_and_preserves_a_usable_scene",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(false);
    let settings = settings();
    let mut app = CameraServer::new(
        sim,
        "default",
        options(),
        config(),
        NonZeroUsize::new(1).unwrap(),
        &settings,
    )
    .unwrap();
    app.update(&settings).unwrap();
    let session = app.runtime.as_ref().unwrap().session.clone();
    let before = app.server().simulation();
    let ids: Vec<_> = before
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(EntityId::get))
        .collect();
    let metadata = before
        .world()
        .iter_entities()
        .find_map(|e| e.get::<CameraSensor>().cloned())
        .unwrap();
    let counts = before.scheduled_system_counts();
    let epoch = before.epoch();
    let elapsed = before.elapsed();
    let mut sim = app.into_simulation();
    assert!(session.is_closed());
    assert_eq!(sim.epoch(), epoch);
    assert_eq!(sim.elapsed(), elapsed);
    assert_eq!(sim.scheduled_system_counts(), counts);
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter_map(|e| e.get::<EntityId>().map(EntityId::get))
            .collect::<Vec<_>>(),
        ids
    );
    let restored = sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<CameraSensor>())
        .unwrap();
    assert!(std::sync::Arc::ptr_eq(&restored.0, &metadata.0));
    assert_eq!(format!("{:?}", restored.0), format!("{:?}", metadata.0));
    sim.step_exact(1).unwrap();
    assert_eq!(sim.elapsed(), elapsed + Duration::from_millis(1));
    assert_eq!(sim.epoch(), epoch);
}

#[test]
fn physics_step_failure_retires_camera_io_and_preserves_typed_fault() {
    if isolation::child(
        "application::tests::physics_step_failure_retires_camera_io_and_preserves_typed_fault",
        20,
    ) {
        return;
    }
    let mut sim = load(&xml(true));
    sim.set_paused(false);
    sim.add_systems(
        gz_sim_core::Phase::PhysicsStep,
        |fault: bevy_ecs::prelude::Res<gz_sim_core::StepFault>| {
            fault.report(std::io::Error::other("automatic camera physics failure"));
        },
    );
    let settings = settings();
    let mut app = CameraServer::new(
        sim,
        "default",
        options(),
        config(),
        NonZeroUsize::new(1).unwrap(),
        &settings,
    )
    .unwrap();
    let session = app.runtime.as_ref().unwrap().session.clone();
    assert!(matches!(
        app.update(&settings),
        Err(CameraServerError::Server(gz_sim_server::Error::Step(
            gz_sim_core::StepError::Faulted
        )))
    ));
    assert!(app.runtime.is_none());
    assert!(session.is_closed());
    assert!(app.server().simulation().is_faulted());
    assert_eq!(app.server().simulation().epoch(), 0);
    assert_eq!(app.server().simulation().elapsed(), Duration::ZERO);
    assert!(
        app.server()
            .simulation()
            .fault()
            .unwrap()
            .to_string()
            .contains("automatic camera physics failure")
    );
    let elapsed = app.server().simulation().elapsed();
    assert_eq!(app.pump().unwrap(), None);
    assert!(matches!(
        app.update(&settings),
        Err(CameraServerError::Server(gz_sim_server::Error::Step(
            gz_sim_core::StepError::Faulted
        )))
    ));
    assert_eq!(app.server().simulation().elapsed(), elapsed);
}

#[test]
fn preflight_rejects_broken_loaded_identity_and_hierarchy_before_admission() {
    for case in 0..12 {
        let mut sim = load(&xml(true));
        let camera = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<CameraSensor>())
            .unwrap()
            .id();
        let id = sim.world().get::<EntityId>(camera).unwrap().get();
        let link = sim.world().get::<ChildOf>(camera).unwrap().parent();
        let root = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<EntityKind>() == Some(&EntityKind::World))
            .unwrap()
            .id();
        match case {
            0 => {
                sim.world_mut().entity_mut(camera).remove::<EntityId>();
            }
            1 => {
                sim.world_mut().entity_mut(camera).remove::<EntityKind>();
            }
            2 => {
                sim.world_mut().entity_mut(camera).remove::<LocalName>();
            }
            3 => {
                sim.world_mut().entity_mut(camera).remove::<Name>();
            }
            4 => {
                sim.world_mut().entity_mut(camera).remove::<ChildOf>();
            }
            5 => {
                sim.world_mut()
                    .entity_mut(camera)
                    .insert(LocalName(String::new()));
            }
            6 => {
                sim.world_mut()
                    .entity_mut(camera)
                    .insert(LocalName("bad::scope".into()));
            }
            7 => {
                sim.world_mut().entity_mut(link).insert(EntityKind::Model);
            }
            8 => {
                sim.world_mut()
                    .entity_mut(camera)
                    .insert(Name("default::wrong".into()));
            }
            9 => {
                sim.world_mut().entity_mut(camera).insert(ChildOf(root));
            }
            10 => {
                sim.world_mut()
                    .spawn((EntityKind::World, RenderingSensorsConfiguration));
            }
            11 => {
                sim.world_mut().entity_mut(root).remove::<EntityId>();
            }
            _ => unreachable!(),
        }
        let expected = if case == 0 || case == 10 { 0 } else { id };
        assert!(
            matches!(prepare_camera(&sim, &options(), &settings()),
            Err(CameraServerError::Camera(CameraOutputError::Snapshot(SnapshotError::MissingCamera(found)))) if found == expected),
            "case {case}"
        );
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert_eq!(sim.epoch(), 0);
    }
}

#[test]
fn nested_model_preflight_uses_the_full_typed_topic_and_scoped_frame() {
    let source = xml(true)
        .replace(
            "<model name=\"robot\">",
            "<model name=\"outer\"><model name=\"robot\">",
        )
        .replace("</model>", "</model></model>")
        .replace("<topic>/explicit-camera</topic>", "<topic></topic>");
    // TPE explicitly refuses nested models; use the existing nested-capable backend.
    let sim = gz_sim_scene::load_world_with_sdf_step(
        &source,
        "automatic-camera.sdf",
        "default",
        &sdformat::NoResources,
        gz_sim_scene::PhysicsEngine::Rapier,
    )
    .unwrap();
    let options = options();
    let prepared = prepare_camera(&sim, &options, &settings())
        .unwrap()
        .unwrap();
    let expected = "/world/default/model/outer/model/robot/link/body/sensor/camera/image";
    assert_eq!(
        prepared.config.image_key,
        options.topic_key(expected).unwrap()
    );
    assert_eq!(prepared.config.frame_id, "outer::robot::body::camera");
    assert!(prepared.binding.current(&sim));
    assert_eq!(prepared.binding.nodes.len(), 5);
    assert_eq!(sim.elapsed(), Duration::ZERO);
}

#[test]
fn committed_binding_mutations_retire_before_any_camera_output() {
    if isolation::child(
        "application::tests::committed_binding_mutations_retire_before_any_camera_output",
        20,
    ) {
        return;
    }
    for case in 0..4 {
        let mut sim = load(&xml(true));
        sim.set_paused(false);
        let camera = sim
            .world()
            .iter_entities()
            .find(|e| e.contains::<CameraSensor>())
            .unwrap()
            .id();
        let root = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<EntityKind>() == Some(&EntityKind::World))
            .unwrap()
            .id();
        sim.add_systems(
            gz_sim_core::Phase::Finalize,
            move |world: &mut bevy_ecs::world::World| match case {
                0 => {
                    world.entity_mut(camera).insert(EntityKind::Link);
                }
                1 => {
                    world
                        .entity_mut(camera)
                        .insert(LocalName("replacement".into()));
                }
                2 => {
                    world.entity_mut(camera).insert(ChildOf(root));
                }
                3 => {
                    world
                        .entity_mut(root)
                        .remove::<RenderingSensorsConfiguration>();
                }
                _ => unreachable!(),
            },
        );
        let settings = settings();
        let mut app = CameraServer::new(
            sim,
            "default",
            options(),
            config(),
            NonZeroUsize::new(1).unwrap(),
            &settings,
        )
        .unwrap();
        let session = app.runtime.as_ref().unwrap().session.clone();
        assert!(
            matches!(
                app.update(&settings),
                Err(CameraServerError::Camera(CameraOutputError::OwnerInvalid))
            ),
            "case {case}"
        );
        assert_eq!(
            app.server().simulation().elapsed(),
            Duration::from_millis(1)
        );
        assert!(app.runtime.is_none());
        assert!(session.is_closed());
        assert_eq!(app.camera_demand().unwrap(), None);
        assert_eq!(app.pump().unwrap(), None);
    }
}

fn pair_xml() -> String {
    let source = xml(true);
    let start = source.find("<model name=\"robot\"").unwrap();
    let end = start + source[start..].find("</model>").unwrap() + "</model>".len();
    let model = &source[start..end];
    let mut models = String::new();
    for (side, rate) in [("a", 10), ("b", 20)] {
        models.push_str(
            &model
                .replace("name=\"robot\"", &format!("name=\"robot_{side}\""))
                .replace(
                    "<topic>/explicit-camera</topic>",
                    &format!("<update_rate>{rate}</update_rate><topic>/m63/{side}/image</topic>"),
                )
                .replace(
                    "<camera>",
                    &format!(
                        "<camera><camera_info_topic>/m63/{side}/camera_info</camera_info_topic>"
                    ),
                ),
        );
    }
    source[..start].to_owned() + &models + &source[end..]
}
fn pair_application(sim: Simulation, settings: &CameraServerOptions) -> CameraServer {
    CameraServer::new_pair(
        sim,
        "default",
        options(),
        config(),
        NonZeroUsize::new(1).unwrap(),
        settings,
    )
    .unwrap()
}

#[test]
fn two_loaded_rgb_owners_preflight_all_bindings_and_four_keys() {
    let source = pair_xml();
    let opts = options();
    let settings = settings();
    let prepared = prepare_pair(&load(&source), &opts, &settings).unwrap();
    assert_eq!(prepared.len(), 2);
    let keys = [
        &prepared[0].config.image_key,
        &prepared[0].config.info_key,
        &prepared[1].config.image_key,
        &prepared[1].config.info_key,
    ];
    assert_eq!(keys.into_iter().collect::<BTreeSet<_>>().len(), 4);
    for (first, second) in [
        ("/m63/a/image", "/m63/b/image"),
        ("/m63/a/camera_info", "/m63/b/camera_info"),
        ("/m63/a/image", "/m63/b/camera_info"),
        ("/m63/a/camera_info", "/m63/b/image"),
    ] {
        let mut opts = options();
        opts.add_topic_remap(first, second).unwrap();
        assert!(matches!(
            prepare_pair(&load(&source), &opts, &settings),
            Err(CameraServerError::Camera(
                CameraOutputError::DuplicateTopics
            ))
        ));
    }
    for source in [
        source.replacen("type=\"camera\"", "type=\"depth_camera\"", 1),
        source.replace("type=\"camera\"", "type=\"depth_camera\""),
    ] {
        assert!(matches!(
            prepare_pair(&load(&source), &opts, &settings),
            Err(CameraServerError::Camera(CameraOutputError::Snapshot(_)))
        ));
    }
    assert!(matches!(
        prepare_pair(&load(&xml(true)), &opts, &settings),
        Err(CameraServerError::Camera(
            CameraOutputError::PairCameraCount
        ))
    ));
    assert!(matches!(
        prepare_pair(&load(&source.replace(SENSORS, "")), &opts, &settings),
        Err(CameraServerError::Camera(
            CameraOutputError::PairCameraCount
        ))
    ));
    let mut sim = load(&source);
    let entity = sim
        .world()
        .iter_entities()
        .find(|node| {
            node.get::<Name>()
                .is_some_and(|name| name.0.ends_with("robot_b::body::camera"))
        })
        .unwrap()
        .id();
    sim.world_mut()
        .entity_mut(entity)
        .insert(Name("default::wrong".into()));
    assert!(prepare_pair(&sim, &opts, &settings).is_err());
    let mut sim = load(&source);
    let template = sim
        .world()
        .iter_entities()
        .find(|node| node.contains::<CameraSensor>())
        .unwrap()
        .get::<CameraSensor>()
        .unwrap()
        .clone();
    sim.world_mut().spawn(template);
    assert!(matches!(
        prepare_pair(&sim, &opts, &settings),
        Err(CameraServerError::Camera(
            CameraOutputError::PairCameraCount
        ))
    ));
}

#[test]
fn independent_image_info_demand_keeps_two_clocks_and_bounded_acquisitions() {
    if isolation::child(
        "application::tests::independent_image_info_demand_keeps_two_clocks_and_bounded_acquisitions",
        30,
    ) {
        return;
    }
    let settings = settings();
    let mut sim = load(&pair_xml());
    sim.set_paused(false);
    let mut app = pair_application(sim, &settings);
    let ids = app.pair_ids.unwrap();
    let session = app.pair.as_ref().unwrap().session.clone();
    let info_a = session
        .declare_subscriber(
            app.server()
                .options()
                .topic_key("/m63/a/camera_info")
                .unwrap(),
        )
        .wait()
        .unwrap();
    let before = (
        app.server().simulation().elapsed(),
        app.server().simulation().epoch(),
    );
    assert!(matches!(
        app.capture(),
        Err(CameraServerError::Camera(CameraOutputError::PairMode))
    ));
    assert!(matches!(
        app.pump(),
        Err(CameraServerError::Camera(CameraOutputError::PairMode))
    ));
    assert!(matches!(
        app.camera_demand(),
        Err(CameraServerError::Camera(CameraOutputError::PairMode))
    ));
    assert!(matches!(
        app.update(&settings),
        Err(CameraServerError::Camera(CameraOutputError::PairMode))
    ));
    assert_eq!(
        (
            app.server().simulation().elapsed(),
            app.server().simulation().epoch()
        ),
        before
    );
    assert_eq!(app.camera_demand_for(ids[0]).unwrap(), Some((false, true)));
    assert_eq!(app.camera_demand_for(ids[1]).unwrap(), Some((false, false)));
    for step in 1..=101 {
        app.tick(&settings).unwrap();
        let rows = app.capture_all().unwrap();
        assert_eq!(rows.len(), 2);
        for row in rows {
            row.result.unwrap();
        }
        if [1, 100].contains(&step) {
            let sample = info_a
                .recv_timeout(Duration::from_secs(2))
                .unwrap()
                .unwrap();
            let info = CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
            assert_eq!((info.width, info.height), (64, 64));
            assert_eq!(info.header.unwrap().stamp.unwrap().nsec, step * 1_000_000);
        } else {
            assert!(info_a.try_recv().unwrap().is_none());
        }
        assert!(
            app.pair
                .as_ref()
                .unwrap()
                .outputs
                .values()
                .all(|owner| owner.output.client_diagnostics().is_none())
        );
    }
    let info_b = session
        .declare_subscriber(
            app.server()
                .options()
                .topic_key("/m63/b/camera_info")
                .unwrap(),
        )
        .wait()
        .unwrap();
    for _ in 0..49 {
        app.tick(&settings).unwrap();
        for row in app.capture_all().unwrap() {
            row.result.unwrap();
        }
    }
    // The first newly interested sample coalesces at102ms, then rejoins the
    // preserved20Hz phase at150ms; no-demand did not consume a clock slot.
    for expected in [102_000_000, 150_000_000] {
        let info = CameraInfo::decode(
            info_b
                .recv_timeout(Duration::from_secs(2))
                .unwrap()
                .unwrap()
                .payload()
                .to_bytes()
                .as_ref(),
        )
        .unwrap();
        assert_eq!(info.header.unwrap().stamp.unwrap().nsec, expected);
    }
    assert!(app.server_mut().apply_local_control(WorldControl {
        pause: true,
        ..Default::default()
    }));
    let time = app.server().simulation().elapsed();
    for _ in 0..3 {
        assert!(app.tick(&settings).unwrap().dt.is_zero());
        for row in app.capture_all().unwrap() {
            assert!(row.result.unwrap().requested.is_none());
        }
        assert_eq!(app.server().simulation().elapsed(), time);
    }
    app.shutdown();
    assert!(session.is_closed());
}

#[test]
fn busy_camera_does_not_starve_its_peer() {
    if isolation::child(
        "application::tests::busy_camera_does_not_starve_its_peer",
        30,
    ) {
        return;
    }
    let settings = settings();
    let mut sim = load(&pair_xml());
    sim.set_paused(false);
    let mut app = pair_application(sim, &settings);
    let ids = app.pair_ids.unwrap();
    let session = app.pair.as_ref().unwrap().session.clone();
    let images = session
        .declare_subscriber(app.server().options().topic_key("/m63/a/image").unwrap())
        .wait()
        .unwrap();
    let infos = session
        .declare_subscriber(
            app.server()
                .options()
                .topic_key("/m63/b/camera_info")
                .unwrap(),
        )
        .wait()
        .unwrap();
    app.tick(&settings).unwrap();
    let first = app.capture_all().unwrap();
    assert_eq!(
        first[0].result.as_ref().unwrap().requested,
        Some(Progress::Pending)
    );
    assert_eq!(
        first[1].result.as_ref().unwrap().requested,
        Some(Progress::InfoPublished)
    );
    infos.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
    assert!(app.camera_worker_diagnostics(ids[0]).is_some());
    // Keep the actual outstanding acquisition pending; no pump is performed here.
    for _ in 0..99 {
        app.tick(&settings).unwrap();
    }
    let pending = &mut app
        .pair
        .as_mut()
        .unwrap()
        .outputs
        .get_mut(&ids[0])
        .unwrap()
        .output;
    assert!(matches!(
        pending.request_due(app.server.simulation()),
        Err(CameraOutputError::Busy)
    ));
    // capture_all still visits the healthy info-only owner after a real worker failure/Busy.
    let rows = app.capture_all().unwrap();
    assert_eq!(rows.len(), 2);
    assert!(rows[0].result.is_err());
    assert_eq!(
        rows[1].result.as_ref().unwrap().requested,
        Some(Progress::InfoPublished)
    );
    infos.recv_timeout(Duration::from_secs(2)).unwrap().unwrap();
    assert_eq!(
        app.server().simulation().elapsed(),
        Duration::from_millis(100)
    );
    app.shutdown();
    drop((images, infos));
    assert!(session.is_closed());
}

#[test]
fn camera_pair_removal_and_loaded_reset_retire_only_owned_generations() {
    if isolation::child(
        "application::tests::camera_pair_removal_and_loaded_reset_retire_only_owned_generations",
        30,
    ) {
        return;
    }
    use std::sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    };
    let mut sim = load(&pair_xml());
    sim.set_paused(true);
    let remove = Arc::new(AtomicU64::new(0));
    let trigger = Arc::clone(&remove);
    sim.add_systems(
        gz_sim_core::Phase::Finalize,
        move |world: &mut bevy_ecs::world::World| {
            let id = trigger.swap(0, Ordering::AcqRel);
            if id != 0 {
                let entity = entity_by_id(world, id).unwrap();
                assert!(world.despawn(entity));
            }
        },
    );
    let settings = settings();
    let mut app = pair_application(sim, &settings);
    let ids = app.pair_ids.unwrap();
    let session = app.pair.as_ref().unwrap().session.clone();
    remove.store(ids[0], Ordering::Release);
    app.tick(&settings).unwrap();
    let rows = app.capture_all().unwrap();
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].sensor_id, ids[1]);
    assert!(!session.is_closed());
    assert_eq!(app.pump_camera(ids[0]).unwrap(), None);
    assert_eq!(app.camera_demand_for(ids[0]).unwrap(), None);
    assert!(matches!(
        app.capture(),
        Err(CameraServerError::Camera(CameraOutputError::PairMode))
    ));
    remove.store(ids[1], Ordering::Release);
    app.tick(&settings).unwrap();
    assert!(app.capture_all().unwrap().is_empty());
    assert!(!app.has_camera_output());
    assert!(session.is_closed());
    assert!(matches!(
        app.update(&settings),
        Err(CameraServerError::Camera(CameraOutputError::PairMode))
    ));
    assert!(app.server_mut().apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(
        app.capture_all(),
        Err(CameraServerError::Server(
            gz_sim_server::Error::IoPreparationRequired
        ))
    ));
    app.prepare_io(&settings).unwrap();
    assert_eq!(app.server().simulation().epoch(), 1);
    assert_eq!(app.pair_ids, Some(ids));
    assert_eq!(
        app.pair
            .as_ref()
            .unwrap()
            .outputs
            .keys()
            .copied()
            .collect::<Vec<_>>(),
        ids.to_vec()
    );
    assert!(!app.pair.as_ref().unwrap().session.is_closed());
    for owner in app.pair.as_ref().unwrap().outputs.values() {
        assert!(owner.output.client_diagnostics().is_none());
        assert!(owner.output.binding_valid(app.server().simulation()));
    }
    app.shutdown();
    // A live component/name fault or an unowned extra camera is global. The
    // healthy peer must publish nothing before the whole set is rejected.
    for fault in 0..3 {
        let mut sim = load(&pair_xml());
        sim.set_paused(false);
        let camera = sim
            .world()
            .iter_entities()
            .find(|node| node.contains::<CameraSensor>())
            .unwrap()
            .id();
        let template = sim.world().get::<CameraSensor>(camera).unwrap().clone();
        sim.add_systems(
            gz_sim_core::Phase::Finalize,
            move |world: &mut bevy_ecs::world::World| match fault {
                0 => {
                    world.entity_mut(camera).remove::<CameraSensor>();
                }
                1 => {
                    world
                        .entity_mut(camera)
                        .insert(Name("default::drifted".into()));
                }
                _ => {
                    world.spawn(template.clone());
                }
            },
        );
        let mut app = pair_application(sim, &settings);
        let session = app.pair.as_ref().unwrap().session.clone();
        let (send, receive) = std::sync::mpsc::sync_channel(1);
        let _peer = session
            .declare_subscriber(
                app.server()
                    .options()
                    .topic_key("/m63/b/camera_info")
                    .unwrap(),
            )
            .callback(move |sample| {
                let _ = send.try_send(sample);
            })
            .wait()
            .unwrap();
        app.tick(&settings).unwrap();
        assert!(matches!(
            app.capture_all(),
            Err(CameraServerError::Camera(CameraOutputError::OwnerInvalid))
        ));
        assert!(app.pair.is_none() && session.is_closed());
        assert!(matches!(
            receive.try_recv(),
            Err(std::sync::mpsc::TryRecvError::Empty | std::sync::mpsc::TryRecvError::Disconnected)
        ));
    }
}

#[test]
fn capture_all_absent_and_single_keep_id_scoped_results_without_extra_ticks() {
    if isolation::child(
        "application::tests::capture_all_absent_and_single_keep_id_scoped_results_without_extra_ticks",
        30,
    ) {
        return;
    }
    let settings = settings();
    let empty = format!("<sdf version='1.12'><world name='default'>{SENSORS}</world></sdf>");
    let (mut absent, client) = application(load(&empty), &settings);
    assert!(absent.capture_all().unwrap().is_empty());
    assert_eq!(absent.pump_camera(u64::MAX).unwrap(), None);
    assert_eq!(absent.camera_demand_for(u64::MAX).unwrap(), None);
    assert!(absent.camera_worker_diagnostics(u64::MAX).is_none());
    assert!(!absent.has_camera_output());
    assert_eq!(absent.server().simulation().elapsed(), Duration::ZERO);
    absent.shutdown();
    drop(client);

    let mut sim = load(&xml(true));
    sim.set_paused(false);
    let (mut app, client) = application(sim, &settings);
    let id = app.runtime.as_ref().unwrap().binding.id;
    let session = app.runtime.as_ref().unwrap().session.clone();
    let before = (
        app.server().simulation().elapsed(),
        app.server().simulation().epoch(),
    );
    assert!(app.camera_worker_diagnostics(id).is_none());
    assert!(app.camera_worker_diagnostics(u64::MAX).is_none());
    for error in [
        app.pump_camera(u64::MAX).map(|_| ()),
        app.camera_demand_for(u64::MAX).map(|_| ()),
    ] {
        assert!(
            matches!(error, Err(CameraServerError::Camera(CameraOutputError::Snapshot(SnapshotError::MissingCamera(value)))) if value == u64::MAX)
        );
    }
    assert_eq!(app.pump_camera(id).unwrap(), Some(Progress::NoDemand));
    assert_eq!(app.camera_demand_for(id).unwrap(), Some((false, false)));
    let rows = app.capture_all().unwrap();
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].sensor_id, id);
    assert!(rows[0].result.is_ok());
    assert_eq!(
        (
            app.server().simulation().elapsed(),
            app.server().simulation().epoch()
        ),
        before
    );
    let images = session
        .declare_subscriber(
            app.server()
                .options()
                .topic_key("/explicit-camera")
                .unwrap(),
        )
        .wait()
        .unwrap();
    app.tick(&settings).unwrap();
    let rows = app.capture_all().unwrap();
    assert_eq!(rows[0].sensor_id, id);
    assert_eq!(
        rows[0].result.as_ref().unwrap().requested,
        Some(Progress::Pending)
    );
    assert!(app.camera_worker_diagnostics(id).is_some());
    let time = app.server().simulation().elapsed();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let rows = app.capture_all().unwrap();
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].sensor_id, id);
        if let Err(error) = &rows[0].result {
            assert!(matches!(
                error,
                CameraServerError::Camera(CameraOutputError::Client(_))
            ));
            break;
        }
        assert!(
            Instant::now() < deadline,
            "missing worker did not report its real failure"
        );
        std::thread::yield_now();
    }
    assert_eq!(app.server().simulation().elapsed(), time);
    app.shutdown();
    assert!(session.is_closed());
    drop((images, client));
}

#[test]
fn pair_unknown_ids_and_scalar_calls_preserve_both_loaded_owners() {
    if isolation::child(
        "application::tests::pair_unknown_ids_and_scalar_calls_preserve_both_loaded_owners",
        30,
    ) {
        return;
    }
    let settings = settings();
    let mut app = pair_application(load(&pair_xml()), &settings);
    let ids = app.pair_ids.unwrap();
    let session = app.pair.as_ref().unwrap().session.clone();
    let info = session
        .declare_subscriber(
            app.server()
                .options()
                .topic_key("/m63/b/camera_info")
                .unwrap(),
        )
        .wait()
        .unwrap();
    let before = (
        app.server().simulation().elapsed(),
        app.server().simulation().epoch(),
    );
    for error in [
        app.pump_camera(u64::MAX).map(|_| ()),
        app.camera_demand_for(u64::MAX).map(|_| ()),
    ] {
        assert!(
            matches!(error, Err(CameraServerError::Camera(CameraOutputError::Snapshot(SnapshotError::MissingCamera(value)))) if value == u64::MAX)
        );
    }
    for error in [
        app.capture().map(|_| ()),
        app.update(&settings).map(|_| ()),
        app.pump().map(|_| ()),
        app.camera_demand().map(|_| ()),
    ] {
        assert!(matches!(
            error,
            Err(CameraServerError::Camera(CameraOutputError::PairMode))
        ));
    }
    assert_eq!(app.pump_camera(ids[0]).unwrap(), Some(Progress::NoDemand));
    assert_eq!(app.camera_demand_for(ids[0]).unwrap(), Some((false, false)));
    assert_eq!(app.camera_demand_for(ids[1]).unwrap(), Some((false, true)));
    assert_eq!(
        (
            app.server().simulation().elapsed(),
            app.server().simulation().epoch()
        ),
        before
    );
    assert_eq!(
        app.pair
            .as_ref()
            .unwrap()
            .outputs
            .keys()
            .copied()
            .collect::<Vec<_>>(),
        ids.to_vec()
    );
    assert!(
        ids.into_iter()
            .all(|id| app.camera_worker_diagnostics(id).is_none())
    );
    assert!(info.try_recv().unwrap().is_none());
    assert!(!session.is_closed());
    app.shutdown();
    assert!(session.is_closed());
}

#[test]
fn pair_fault_closed_session_and_unowned_identity_refuse_the_whole_capture() {
    if isolation::child(
        "application::tests::pair_fault_closed_session_and_unowned_identity_refuse_the_whole_capture",
        40,
    ) {
        return;
    }
    let settings = settings();
    for variant in 0..3 {
        let mut sim = load(&pair_xml());
        sim.set_paused(false);
        if variant == 0 {
            sim.add_systems(
                gz_sim_core::Phase::PhysicsStep,
                |fault: bevy_ecs::prelude::Res<gz_sim_core::StepFault>| {
                    fault.report(std::io::Error::other("pair whole-family fault"));
                },
            );
        } else if variant == 2 {
            let entity = sim
                .world()
                .iter_entities()
                .find(|node| node.contains::<CameraSensor>())
                .unwrap();
            let camera = entity.get::<CameraSensor>().unwrap().clone();
            sim.add_systems(
                gz_sim_core::Phase::Finalize,
                move |world: &mut bevy_ecs::world::World| {
                    let extra = world.spawn(camera.clone()).id();
                    gz_sim_core::assign_id(world, extra).unwrap();
                },
            );
        }
        let mut app = pair_application(sim, &settings);
        let session = app.pair.as_ref().unwrap().session.clone();
        let (send, receive) = std::sync::mpsc::sync_channel(1);
        let _info = session
            .declare_subscriber(
                app.server()
                    .options()
                    .topic_key("/m63/b/camera_info")
                    .unwrap(),
            )
            .callback(move |sample| {
                let _ = send.try_send(sample);
            })
            .wait()
            .unwrap();
        if variant == 0 {
            assert!(matches!(
                app.server_mut().update(),
                Err(gz_sim_server::Error::Step(gz_sim_core::StepError::Faulted))
            ));
            assert!(app.server().simulation().is_faulted());
        } else if variant == 1 {
            session.close().wait().unwrap();
        } else {
            app.tick(&settings).unwrap();
        }
        let before = (
            app.server().simulation().elapsed(),
            app.server().simulation().epoch(),
        );
        let result = app.capture_all();
        if variant == 1 {
            assert!(matches!(
                result,
                Err(CameraServerError::Camera(CameraOutputError::SessionClosed))
            ));
        } else {
            assert!(matches!(
                result,
                Err(CameraServerError::Camera(CameraOutputError::OwnerInvalid))
            ));
        }
        assert_eq!(
            (
                app.server().simulation().elapsed(),
                app.server().simulation().epoch()
            ),
            before
        );
        assert!(app.pair.is_none());
        assert!(session.is_closed());
        // No output was requested before whole-family validation failed.
        assert!(matches!(
            receive.try_recv(),
            Err(std::sync::mpsc::TryRecvError::Empty | std::sync::mpsc::TryRecvError::Disconnected)
        ));
        app.shutdown();
    }
}

#[test]
fn pair_constructor_server_bind_failure_releases_the_shared_camera_listener() {
    if isolation::child(
        "application::tests::pair_constructor_server_bind_failure_releases_the_shared_camera_listener",
        30,
    ) {
        return;
    }
    let reserved = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = reserved.local_addr().unwrap();
    let endpoints = format!("[\"tcp/{address}\"]");
    let mut settings = settings();
    settings
        .zenoh
        .insert_json5("listen/endpoints", &endpoints)
        .unwrap();
    let mut server = config();
    server.insert_json5("listen/endpoints", &endpoints).unwrap();
    drop(reserved);
    let result = CameraServer::new_pair(
        load(&pair_xml()),
        "default",
        options(),
        server,
        NonZeroUsize::new(1).unwrap(),
        &settings,
    );
    assert!(matches!(
        result,
        Err(CameraServerError::Server(
            gz_sim_server::Error::WorldNetwork(_)
        ))
    ));
    // The pair bound first, so this one rebind witnesses real constructor unwind.
    let rebound = TcpListener::bind(address).unwrap();
    assert_eq!(rebound.local_addr().unwrap(), address);
}

#[test]
fn pair_reset_refusal_and_failed_reconstruction_preserve_original_owner_intent() {
    if isolation::child(
        "application::tests::pair_reset_refusal_and_failed_reconstruction_preserve_original_owner_intent",
        30,
    ) {
        return;
    }
    let settings = settings();
    let mut app = pair_application(load(&pair_xml()), &settings);
    let ids = app.pair_ids.unwrap();
    let old_session = app.pair.as_ref().unwrap().session.clone();
    assert!(app.server_mut().apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    for result in [
        app.capture_all().map(|_| ()),
        app.pump_camera(ids[0]).map(|_| ()),
        app.camera_demand_for(ids[1]).map(|_| ()),
    ] {
        assert!(matches!(
            result,
            Err(CameraServerError::Server(
                gz_sim_server::Error::IoPreparationRequired
            ))
        ));
    }
    assert!(old_session.is_closed());
    let invalid = CameraServerOptions {
        worker_path: settings.worker_path.clone(),
        origin: [f64::NAN; 3],
        zenoh: settings.zenoh.clone(),
    };
    assert!(matches!(
        app.prepare_io(&invalid),
        Err(CameraServerError::Camera(CameraOutputError::Snapshot(_)))
    ));
    assert!(app.pair.is_none());
    assert_eq!(app.pair_ids, Some(ids));
    assert_eq!(app.server().simulation().epoch(), 1);
    assert_eq!(app.server().simulation().elapsed(), Duration::ZERO);
    app.prepare_io(&settings).unwrap();
    assert_eq!(app.pair_ids, Some(ids));
    assert_eq!(
        app.pair
            .as_ref()
            .unwrap()
            .outputs
            .keys()
            .copied()
            .collect::<Vec<_>>(),
        ids.to_vec()
    );
    assert!(!app.pair.as_ref().unwrap().session.is_closed());
    assert!(
        ids.into_iter()
            .all(|id| app.camera_worker_diagnostics(id).is_none())
    );
    app.shutdown();
}

#[test]
fn pair_real_image_demand_loss_retires_only_that_worker_and_keeps_peer_cadence() {
    if isolation::child(
        "application::tests::pair_real_image_demand_loss_retires_only_that_worker_and_keeps_peer_cadence",
        30,
    ) {
        return;
    }
    let mut settings = settings();
    settings.worker_path = "/bin/cat".into();
    let mut sim = load(&pair_xml());
    sim.set_paused(false);
    let mut app = pair_application(sim, &settings);
    let ids = app.pair_ids.unwrap();
    let session = app.pair.as_ref().unwrap().session.clone();
    let images = session
        .declare_subscriber(app.server().options().topic_key("/m63/a/image").unwrap())
        .wait()
        .unwrap();
    let infos = session
        .declare_subscriber(
            app.server()
                .options()
                .topic_key("/m63/b/camera_info")
                .unwrap(),
        )
        .wait()
        .unwrap();
    app.tick(&settings).unwrap();
    let rows = app.capture_all().unwrap();
    assert_eq!(
        rows[0].result.as_ref().unwrap().requested,
        Some(Progress::Pending)
    );
    assert_eq!(
        rows[1].result.as_ref().unwrap().requested,
        Some(Progress::InfoPublished)
    );
    let first = CameraInfo::decode(
        infos
            .recv_timeout(Duration::from_secs(2))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(first.header.unwrap().stamp.unwrap().nsec, 1_000_000);
    let (_, lifetime) = app.camera_worker_diagnostics(ids[0]).unwrap();
    assert!(!lifetime.is_retired());
    drop(images);
    assert_eq!(app.camera_demand_for(ids[0]).unwrap(), Some((false, false)));
    for _ in 0..49 {
        app.tick(&settings).unwrap();
    }
    let rows = app.capture_all().unwrap();
    assert_eq!(
        rows[0].result.as_ref().unwrap().pumped,
        Some(Progress::RetireRequired)
    );
    assert_eq!(rows[0].result.as_ref().unwrap().requested, None);
    assert!(lifetime.is_retired());
    assert!(app.camera_worker_diagnostics(ids[0]).is_none());
    assert_eq!(
        rows[1].result.as_ref().unwrap().requested,
        Some(Progress::InfoPublished)
    );
    let second = CameraInfo::decode(
        infos
            .recv_timeout(Duration::from_secs(2))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(second.header.unwrap().stamp.unwrap().nsec, 50_000_000);
    assert_eq!(
        app.server().simulation().elapsed(),
        Duration::from_millis(50)
    );
    assert!(!session.is_closed());
    assert_eq!(app.pair.as_ref().unwrap().outputs.len(), 2);
    app.shutdown();
    assert!(session.is_closed());
}

fn depth_pair_xml() -> String {
    pair_xml()
        .replace("type=\"camera\"", "type=\"depth_camera\"")
        .replace("/m63/", "/m69/")
}
fn depth_pair_application(sim: Simulation, settings: &CameraServerOptions) -> CameraServer {
    CameraServer::new_depth_pair(
        sim,
        "default",
        options(),
        config(),
        NonZeroUsize::new(1).unwrap(),
        settings,
    )
    .unwrap()
}

#[test]
fn homogeneous_depth_pair_preflights_counts_types_bindings_rates_and_all_four_keys() {
    let source = depth_pair_xml();
    let settings = settings();
    let opts = options();
    let sim = load(&source);
    let prepared = prepare_depth_pair(&sim, &opts, &settings).unwrap();
    assert_eq!(prepared.len(), 2);
    assert!(prepared[0].binding.id < prepared[1].binding.id);
    assert_eq!(
        prepared
            .iter()
            .flat_map(|p| [&p.config.image_key, &p.config.info_key])
            .collect::<BTreeSet<_>>()
            .len(),
        4
    );
    for p in &prepared {
        assert!(p.binding.current(&sim));
    }
    for (from, to) in [
        ("/m69/a/image", "/m69/b/image"),
        ("/m69/a/camera_info", "/m69/b/camera_info"),
        ("/m69/a/image", "/m69/b/camera_info"),
        ("/m69/a/camera_info", "/m69/b/image"),
    ] {
        let mut remapped = options();
        remapped.add_topic_remap(from, to).unwrap();
        assert!(matches!(
            prepare_depth_pair(&load(&source), &remapped, &settings),
            Err(CameraServerError::Camera(
                CameraOutputError::DuplicateTopics
            ))
        ));
    }
    let empty = format!("<sdf version='1.12'><world name='default'>{SENSORS}</world></sdf>");
    let one = xml(true).replace("type=\"camera\"", "type=\"depth_camera\"");
    let start = source.find("<model name=\"robot_a\"").unwrap();
    let end = start + source[start..].find("</model>").unwrap() + "</model>".len();
    let third = source[start..end]
        .replace("robot_a", "robot_c")
        .replace("/m69/a/", "/m69/c/");
    let three = source.replacen("</world>", &(third + "</world>"), 1);
    for candidate in [empty, one, three, source.replace(SENSORS, "")] {
        assert!(matches!(
            prepare_depth_pair(&load(&candidate), &opts, &settings),
            Err(CameraServerError::Camera(
                CameraOutputError::PairCameraCount
            ))
        ));
    }
    for candidate in [
        pair_xml(),
        source.replacen("type=\"depth_camera\"", "type=\"camera\"", 1),
    ] {
        assert!(matches!(
            prepare_depth_pair(&load(&candidate), &opts, &settings),
            Err(CameraServerError::Camera(CameraOutputError::Snapshot(
                SnapshotError::InvalidCamera(_)
            )))
        ));
    }
    // Strict old RGB admission must not be widened by the new factory.
    assert!(matches!(
        prepare_pair(&load(&source), &opts, &settings),
        Err(CameraServerError::Camera(CameraOutputError::Snapshot(
            SnapshotError::InvalidCamera(_)
        )))
    ));
    for fault in 0..3 {
        let mut sim = load(&source);
        let camera = sim
            .world()
            .iter_entities()
            .find(|r| {
                r.get::<Name>()
                    .is_some_and(|n| n.0.ends_with("robot_b::body::camera"))
            })
            .unwrap()
            .id();
        let id = sim.world().get::<EntityId>(camera).unwrap().get();
        match fault {
            0 => {
                sim.world_mut().entity_mut(camera).remove::<EntityId>();
            }
            1 => {
                sim.world_mut()
                    .entity_mut(camera)
                    .insert(Name("default::invalid".into()));
            }
            _ => {
                std::sync::Arc::make_mut(
                    &mut sim.world_mut().get_mut::<CameraSensor>(camera).unwrap().0,
                )
                .update_rate = f64::NAN;
            }
        }
        match fault {
            0 => assert!(matches!(
                prepare_depth_pair(&sim, &opts, &settings),
                Err(CameraServerError::Camera(CameraOutputError::Snapshot(
                    SnapshotError::MissingCamera(0)
                )))
            )),
            1 => assert!(
                matches!(prepare_depth_pair(&sim, &opts, &settings), Err(CameraServerError::Camera(CameraOutputError::Snapshot(SnapshotError::MissingCamera(v)))) if v == id)
            ),
            _ => assert!(prepare_depth_pair(&sim, &opts, &settings).is_err()),
        }
        assert_eq!((sim.elapsed(), sim.epoch()), (Duration::ZERO, 0));
    }
}

#[test]
fn depth_pair_info_clocks_and_scalar_refusals_preserve_lazy_independent_owners() {
    if isolation::child(
        "application::tests::depth_pair_info_clocks_and_scalar_refusals_preserve_lazy_independent_owners",
        30,
    ) {
        return;
    }
    let settings = settings();
    let mut sim = load(&depth_pair_xml());
    sim.set_paused(false);
    let mut app = depth_pair_application(sim, &settings);
    let ids = app.pair_ids.unwrap();
    let session = app.pair.as_ref().unwrap().session.clone();
    assert_eq!(app.pair_kind, PairKind::Depth);
    let subscribers: Vec<_> = ["/m69/a/camera_info", "/m69/b/camera_info"]
        .into_iter()
        .map(|topic| {
            session
                .declare_subscriber(app.server().options().topic_key(topic).unwrap())
                .wait()
                .unwrap()
        })
        .collect();
    let before = (
        app.server().simulation().elapsed(),
        app.server().simulation().epoch(),
    );
    for error in [
        app.capture().map(|_| ()),
        app.update(&settings).map(|_| ()),
        app.pump().map(|_| ()),
        app.camera_demand().map(|_| ()),
    ] {
        assert!(matches!(
            error,
            Err(CameraServerError::Camera(CameraOutputError::PairMode))
        ));
    }
    assert_eq!(
        (
            app.server().simulation().elapsed(),
            app.server().simulation().epoch()
        ),
        before
    );
    assert!(
        matches!(app.pump_camera(u64::MAX), Err(CameraServerError::Camera(CameraOutputError::Snapshot(SnapshotError::MissingCamera(v)))) if v == u64::MAX)
    );
    assert!(
        matches!(app.camera_demand_for(u64::MAX), Err(CameraServerError::Camera(CameraOutputError::Snapshot(SnapshotError::MissingCamera(v)))) if v == u64::MAX)
    );
    let mut stamps = [Vec::new(), Vec::new()];
    for step in 1..=100 {
        let update = app.tick(&settings).unwrap();
        assert_eq!(update.sim_time, Duration::from_millis(step));
        let rows = app.capture_all().unwrap();
        assert_eq!(rows.len(), 2);
        for (index, row) in rows.into_iter().enumerate() {
            assert_eq!(row.sensor_id, ids[index]);
            let progress = row.result.unwrap();
            let due =
                step == 1 || (index == 0 && step == 100) || (index == 1 && step.is_multiple_of(50));
            if due {
                assert_eq!(progress.requested, Some(Progress::InfoPublished));
                let sample = subscribers[index]
                    .recv_timeout(Duration::from_secs(2))
                    .unwrap()
                    .unwrap();
                let info = CameraInfo::decode(sample.payload().to_bytes().as_ref()).unwrap();
                assert_eq!((info.width, info.height), (64, 64));
                let header = info.header.unwrap();
                let stamp = header.stamp.unwrap();
                assert_eq!(stamp.nsec, i32::try_from(step * 1_000_000).unwrap());
                assert_eq!(
                    header
                        .data
                        .iter()
                        .find(|x| x.key == "frame_id")
                        .unwrap()
                        .value,
                    [format!(
                        "robot_{}::body::camera",
                        if index == 0 { "a" } else { "b" }
                    )]
                );
                stamps[index].push(step);
            } else {
                assert_eq!(progress.requested, None);
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
        let update = app.tick(&settings).unwrap();
        assert!(update.paused && update.dt.is_zero());
        for row in app.capture_all().unwrap() {
            assert_eq!(row.result.unwrap().requested, None);
        }
    }
    assert_eq!(
        app.server().simulation().elapsed(),
        Duration::from_millis(100)
    );
    app.shutdown();
    assert!(session.is_closed());
}

#[test]
fn depth_pair_busy_acquisition_keeps_peer_clock_and_never_repeats_a_tick() {
    if isolation::child(
        "application::tests::depth_pair_busy_acquisition_keeps_peer_clock_and_never_repeats_a_tick",
        30,
    ) {
        return;
    }
    let mut settings = settings();
    settings.worker_path = "/bin/cat".into();
    let mut sim = load(&depth_pair_xml());
    sim.set_paused(false);
    let mut app = depth_pair_application(sim, &settings);
    let ids = app.pair_ids.unwrap();
    let session = app.pair.as_ref().unwrap().session.clone();
    let image = session
        .declare_subscriber(app.server().options().topic_key("/m69/a/image").unwrap())
        .wait()
        .unwrap();
    let info = session
        .declare_subscriber(
            app.server()
                .options()
                .topic_key("/m69/b/camera_info")
                .unwrap(),
        )
        .wait()
        .unwrap();
    app.tick(&settings).unwrap();
    let rows = app.capture_all().unwrap();
    assert_eq!(
        rows[0].result.as_ref().unwrap().requested,
        Some(Progress::Pending)
    );
    assert_eq!(
        rows[1].result.as_ref().unwrap().requested,
        Some(Progress::InfoPublished)
    );
    let first = CameraInfo::decode(
        info.recv_timeout(Duration::from_secs(2))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(first.header.unwrap().stamp.unwrap().nsec, 1_000_000);
    let (_, lifetime) = app.camera_worker_diagnostics(ids[0]).unwrap();
    assert!(!lifetime.is_retired());
    for _ in 1..100 {
        app.tick(&settings).unwrap();
    }
    let before = app.server().simulation().elapsed();
    assert_eq!(before, Duration::from_millis(100));
    // Prove typed Busy at admission before pumping the asynchronous worker.
    let pending = &mut app
        .pair
        .as_mut()
        .unwrap()
        .outputs
        .get_mut(&ids[0])
        .unwrap()
        .output;
    assert!(matches!(
        pending.request_due(app.server.simulation()),
        Err(CameraOutputError::Busy)
    ));
    // Pumping can observe the real non-Ready worker's failure before Busy;
    // either owner error must still leave the peer's clock progressing.
    let rows = app.capture_all().unwrap();
    assert_eq!(rows.len(), 2);
    assert!(rows[0].result.is_err());
    assert_eq!(
        rows[1].result.as_ref().unwrap().requested,
        Some(Progress::InfoPublished)
    );
    let second = CameraInfo::decode(
        info.recv_timeout(Duration::from_secs(2))
            .unwrap()
            .unwrap()
            .payload()
            .to_bytes()
            .as_ref(),
    )
    .unwrap();
    assert_eq!(second.header.unwrap().stamp.unwrap().nsec, 100_000_000);
    assert_eq!(app.server().simulation().elapsed(), before);
    assert!(image.try_recv().unwrap().is_none());
    assert_eq!(app.camera_demand_for(ids[0]).unwrap(), Some((true, false)));
    drop(image);
    assert_eq!(app.camera_demand_for(ids[0]).unwrap(), Some((false, false)));
    let rows = app.capture_all().unwrap();
    assert_eq!(
        rows[0].result.as_ref().unwrap().pumped,
        Some(Progress::RetireRequired)
    );
    assert!(lifetime.is_retired() && app.camera_worker_diagnostics(ids[0]).is_none());
    assert!(!session.is_closed());
    app.shutdown();
    assert!(session.is_closed());
}

#[test]
fn depth_pair_subset_retirement_and_loaded_reset_keep_exact_original_depth_intent() {
    if isolation::child(
        "application::tests::depth_pair_subset_retirement_and_loaded_reset_keep_exact_original_depth_intent",
        30,
    ) {
        return;
    }
    use std::sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    };
    let mut sim = load(&depth_pair_xml());
    sim.set_paused(true);
    let remove = Arc::new(AtomicU64::new(0));
    let trigger = Arc::clone(&remove);
    sim.add_systems(
        gz_sim_core::Phase::Finalize,
        move |world: &mut bevy_ecs::world::World| {
            let id = trigger.swap(0, Ordering::AcqRel);
            if id != 0 {
                let entity = entity_by_id(world, id).unwrap();
                assert!(world.despawn(entity));
            }
        },
    );
    let settings = settings();
    let mut app = depth_pair_application(sim, &settings);
    let ids = app.pair_ids.unwrap();
    let session = app.pair.as_ref().unwrap().session.clone();
    remove.store(ids[0], Ordering::Release);
    app.tick(&settings).unwrap();
    let rows = app.capture_all().unwrap();
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].sensor_id, ids[1]);
    assert!(!session.is_closed());
    assert_eq!(app.pair_kind, PairKind::Depth);
    assert_eq!(app.pump_camera(ids[0]).unwrap(), None);
    assert_eq!(app.camera_demand_for(ids[0]).unwrap(), None);
    assert!(matches!(
        app.capture(),
        Err(CameraServerError::Camera(CameraOutputError::PairMode))
    ));
    remove.store(ids[1], Ordering::Release);
    app.tick(&settings).unwrap();
    assert!(app.capture_all().unwrap().is_empty() && !app.has_camera_output());
    assert!(session.is_closed());
    assert_eq!(app.pair_ids, Some(ids));
    assert_eq!(app.pair_kind, PairKind::Depth);
    assert!(matches!(
        app.update(&settings),
        Err(CameraServerError::Camera(CameraOutputError::PairMode))
    ));
    assert!(app.server_mut().apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(
        app.capture_all(),
        Err(CameraServerError::Server(
            gz_sim_server::Error::IoPreparationRequired
        ))
    ));
    app.prepare_io(&settings).unwrap();
    assert_eq!(
        (
            app.server().simulation().epoch(),
            app.server().simulation().elapsed()
        ),
        (1, Duration::ZERO)
    );
    assert_eq!(app.pair_ids, Some(ids));
    assert_eq!(app.pair_kind, PairKind::Depth);
    let pair = app.pair.as_ref().unwrap();
    assert_eq!(
        pair.outputs.keys().copied().collect::<Vec<_>>(),
        ids.to_vec()
    );
    assert!(!pair.session.is_closed());
    for id in ids {
        let entity = entity_by_id(app.server().simulation().world(), id).unwrap();
        assert_eq!(
            app.server()
                .simulation()
                .world()
                .get::<CameraSensor>(entity)
                .unwrap()
                .0
                .effective_type(),
            Some(sdformat::SensorType::DepthCamera)
        );
        assert!(
            pair.outputs[&id]
                .output
                .binding_valid(app.server().simulation())
        );
        assert!(app.camera_worker_diagnostics(id).is_none());
    }
    app.shutdown();
}

#[test]
fn depth_pair_constructor_bind_unwind_releases_its_one_family_listener() {
    if isolation::child(
        "application::tests::depth_pair_constructor_bind_unwind_releases_its_one_family_listener",
        30,
    ) {
        return;
    }
    let reserved = TcpListener::bind("127.0.0.1:0").unwrap();
    let address = reserved.local_addr().unwrap();
    let endpoints = format!("[\"tcp/{address}\"]");
    let mut settings = settings();
    settings
        .zenoh
        .insert_json5("listen/endpoints", &endpoints)
        .unwrap();
    let mut server = config();
    server.insert_json5("listen/endpoints", &endpoints).unwrap();
    drop(reserved);
    let result = CameraServer::new_depth_pair(
        load(&depth_pair_xml()),
        "default",
        options(),
        server,
        NonZeroUsize::new(1).unwrap(),
        &settings,
    );
    assert!(matches!(
        result,
        Err(CameraServerError::Server(
            gz_sim_server::Error::WorldNetwork(_)
        ))
    ));
    // The camera family binds first; Server's actual conflicting bind fails.
    // This rebind witnesses unwind releasing the family-owned Session.
    let rebound = TcpListener::bind(address).unwrap();
    assert_eq!(rebound.local_addr().unwrap(), address);
}

#[test]
fn depth_pair_global_faults_and_failed_reset_cannot_enroll_or_change_family() {
    if isolation::child(
        "application::tests::depth_pair_global_faults_and_failed_reset_cannot_enroll_or_change_family",
        40,
    ) {
        return;
    }
    let settings = settings();
    for fault in 0..3 {
        let mut sim = load(&depth_pair_xml());
        sim.set_paused(false);
        let camera = sim
            .world()
            .iter_entities()
            .find(|r| r.contains::<CameraSensor>())
            .unwrap()
            .id();
        let template = sim.world().get::<CameraSensor>(camera).unwrap().clone();
        if fault != 2 {
            sim.add_systems(
                gz_sim_core::Phase::Finalize,
                move |world: &mut bevy_ecs::world::World| {
                    if fault == 0 {
                        // A live sensor-kind drift is not legitimate disappearance.
                        std::sync::Arc::make_mut(
                            &mut world.get_mut::<CameraSensor>(camera).unwrap().0,
                        )
                        .kind = "camera".into();
                    } else {
                        let extra = world.spawn(template.clone()).id();
                        gz_sim_core::assign_id(world, extra).unwrap();
                    }
                },
            );
        }
        let mut app = depth_pair_application(sim, &settings);
        let ids = app.pair_ids.unwrap();
        let session = app.pair.as_ref().unwrap().session.clone();
        let info = session
            .declare_subscriber(
                app.server()
                    .options()
                    .topic_key("/m69/b/camera_info")
                    .unwrap(),
            )
            .wait()
            .unwrap();
        if fault == 2 {
            session.close().wait().unwrap();
        } else {
            app.tick(&settings).unwrap();
        }
        let before = (
            app.server().simulation().elapsed(),
            app.server().simulation().epoch(),
        );
        let result = app.capture_all();
        if fault == 2 {
            assert!(matches!(
                result,
                Err(CameraServerError::Camera(CameraOutputError::SessionClosed))
            ));
        } else {
            assert!(matches!(
                result,
                Err(CameraServerError::Camera(CameraOutputError::OwnerInvalid))
            ));
        }
        assert!(app.pair.is_none() && session.is_closed());
        assert_eq!(app.pair_ids, Some(ids));
        assert_eq!(app.pair_kind, PairKind::Depth);
        assert_eq!(
            (
                app.server().simulation().elapsed(),
                app.server().simulation().epoch()
            ),
            before
        );
        // Session closure may disconnect the local subscriber; neither case is
        // an actual Image/Info packet or permission to enroll the extra owner.
        assert!(!matches!(info.try_recv(), Ok(Some(_))));
        app.shutdown();
    }
    let mut app = depth_pair_application(load(&depth_pair_xml()), &settings);
    let ids = app.pair_ids.unwrap();
    let old_session = app.pair.as_ref().unwrap().session.clone();
    assert!(app.server_mut().apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(matches!(
        app.pump_camera(ids[0]),
        Err(CameraServerError::Server(
            gz_sim_server::Error::IoPreparationRequired
        ))
    ));
    assert!(old_session.is_closed());
    let invalid = CameraServerOptions {
        worker_path: settings.worker_path.clone(),
        origin: [f64::NAN; 3],
        zenoh: settings.zenoh.clone(),
    };
    assert!(matches!(
        app.prepare_io(&invalid),
        Err(CameraServerError::Camera(CameraOutputError::Snapshot(_)))
    ));
    assert!(app.pair.is_none());
    assert_eq!(app.pair_kind, PairKind::Depth);
    assert_eq!(app.pair_ids, Some(ids));
    assert_eq!(
        (
            app.server().simulation().epoch(),
            app.server().simulation().elapsed()
        ),
        (1, Duration::ZERO)
    );
    app.prepare_io(&settings).unwrap();
    assert_eq!(app.pair_kind, PairKind::Depth);
    assert_eq!(app.pair_ids, Some(ids));
    assert_eq!(
        app.pair
            .as_ref()
            .unwrap()
            .outputs
            .keys()
            .copied()
            .collect::<Vec<_>>(),
        ids.to_vec()
    );
    assert!(
        ids.into_iter()
            .all(|id| app.camera_worker_diagnostics(id).is_none())
    );
    app.shutdown();
}

#[path = "shared_server_session.rs"]
mod shared_server_session;

#[path = "mixed_pair.rs"]
mod mixed_pair;

#[path = "explicit_intrinsics.rs"]
mod explicit_intrinsics;

#[path = "projection.rs"]
mod projection;

#[path = "skew.rs"]
mod skew;

#[path = "visibility.rs"]
mod visibility;
