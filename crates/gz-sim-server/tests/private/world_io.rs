// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn tcp_profile_sets_finite_defaults_without_changing_endpoints() {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("listen/endpoints", "[\"tcp/127.0.0.1:0\"]")
        .unwrap();
    let endpoints = config.get_json("listen/endpoints").unwrap();
    configure_tcp_profile(&mut config).unwrap();
    for (path, expected) in TCP_PROFILE {
        assert_eq!(
            config.get_json(path).unwrap().parse::<u64>().unwrap(),
            expected
        );
    }
    assert_eq!(config.get_json("listen/endpoints").unwrap(), endpoints);
    configure_tcp_profile(&mut config).unwrap();
}

#[test]
fn tcp_profile_rejects_unqualified_and_infinite_waits_before_open() {
    for (path, expected) in TCP_PROFILE {
        let mut config = zenoh::Config::default();
        config
            .insert_json5(path, &(expected + 1).to_string())
            .unwrap();
        let error = configure_tcp_profile(&mut config).unwrap_err().to_string();
        assert!(error.contains(path), "{error}");
    }
    for path in [TCP_PROFILE[2].0, TCP_PROFILE[3].0, TCP_PROFILE[4].0] {
        let mut config = zenoh::Config::default();
        config.insert_json5(path, "-1").unwrap();
        assert!(configure_tcp_profile(&mut config).is_err());
    }
}

fn isolated_world(max_bytes: usize) -> WorldIo {
    isolated_world_with_capacity(4, max_bytes)
}

fn isolated_world_with_capacity(capacity: usize, max_bytes: usize) -> WorldIo {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("listen/endpoints", "[\"tcp/127.0.0.1:0\"]")
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .unwrap();
    WorldIo::new_with_config(
        "@/test@/clock".into(),
        "@/test@/control".into(),
        "@/test@/poses".into(),
        capacity,
        max_bytes,
        config,
    )
    .unwrap()
}

#[test]
fn removal_routes_keep_queries_on_io_until_plain_replies_complete() {
    let mut io = isolated_world(1024);
    let keys = [
        "@/test@/remove".to_owned(),
        "@/test@/remove/blocking".to_owned(),
    ];
    let inputs = io.removal_inputs(keys.clone()).unwrap();
    for ((key, input), accepted) in keys.iter().zip(&inputs).zip([true, false]) {
        let replies = io
            .session
            .get(key.clone())
            .payload(vec![8, 42])
            .allowed_destination(zenoh::sample::Locality::SessionLocal)
            .timeout(Duration::from_secs(2))
            .wait()
            .unwrap();
        let request = input.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(request.payload, [8, 42]);
        io.reply_boolean(request, accepted).unwrap();
        let reply = replies
            .recv_timeout(Duration::from_secs(2))
            .unwrap()
            .unwrap();
        let sample = reply.result().unwrap();
        assert_eq!(sample.key_expr().as_str(), key);
        assert_eq!(
            Boolean::decode(sample.payload().to_bytes().as_ref())
                .unwrap()
                .data,
            accepted
        );
        assert!(
            replies
                .recv_timeout(Duration::from_secs(2))
                .unwrap_err()
                .to_string()
                .contains("closed")
        );
    }
    io.shutdown();
}

#[test]
fn removal_service_collisions_and_closed_sessions_fail_before_install() {
    let mut io = isolated_world(1024);
    for keys in [
        ["@/test@/same", "@/test@/same"],
        ["@/test@/control", "@/test@/other"],
    ] {
        assert!(matches!(
            io.removal_inputs(keys.map(str::to_owned)),
            Err(crate::Error::WorldNetwork(_))
        ));
    }
    io.shutdown();
    assert!(matches!(
        io.removal_inputs(["remove".into(), "blocking".into()]),
        Err(crate::Error::WorldClosed)
    ));
}

#[test]
fn boolean_reply_limit_and_abandoned_cpu_request_finalize_without_a_reply() {
    let mut io = isolated_world(1);
    let [input, _] = io
        .removal_inputs(["@/test@/remove".into(), "@/test@/blocking".into()])
        .unwrap();
    for reply in [true, false] {
        let replies = io
            .session
            .get("@/test@/remove")
            .allowed_destination(zenoh::sample::Locality::SessionLocal)
            .timeout(Duration::from_secs(2))
            .wait()
            .unwrap();
        let request = input.recv_timeout(Duration::from_secs(2)).unwrap();
        if reply {
            assert!(matches!(
                io.reply_boolean(request, true),
                Err(crate::Error::WorldNetwork(_))
            ));
        } else {
            drop(request);
        }
        assert!(
            replies
                .recv_timeout(Duration::from_secs(2))
                .unwrap_err()
                .to_string()
                .contains("closed")
        );
    }
    io.shutdown();
}

#[test]
fn rate_only_retirement_completes_without_another_publication_or_install() {
    let mut io = isolated_world(1024);
    let sensor = gz_sim_scene::ImuPublication {
        entity_name: "sensor".into(),
        frame_id: "sensor".into(),
        topic: "/imu".into(),
        enable_orientation: true,
    };
    let config = crate::ServerOptions {
        partition: "test".into(),
        ..Default::default()
    };
    let (mut rates, setup) =
        crate::rates::Rates::prepare(&config, [&sensor], &io.ingress_drops).unwrap();
    io.install_rates(setup).unwrap();
    let count = || {
        io.session
            .liveliness()
            .get("@gz/**")
            .timeout(Duration::from_secs(1))
            .wait()
            .unwrap()
            .into_iter()
            .filter(|reply| {
                reply
                    .result()
                    .unwrap()
                    .key_expr()
                    .as_str()
                    .contains("%set_rate/")
            })
            .count()
    };
    assert_eq!(count(), 1);
    rates.clear();
    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    while count() != 0 {
        assert!(
            std::time::Instant::now() < deadline,
            "retired rate token remains without IO traffic"
        );
        thread::sleep(Duration::from_millis(1));
    }
    io.shutdown();
}

#[test]
fn shutdown_releases_force_callback_inputs_and_joins_both_io_owners() {
    let mut io = isolated_world(1024);
    let (input, receiver) = mpsc::sync_channel(1);
    let active = Arc::new(AtomicBool::new(true));
    io.install_forces(crate::force_io::ForceSetup {
        endpoints: vec![crate::force_io::ForceEndpoint {
            key: "@/test@/force".into(),
            input,
            active: Arc::clone(&active),
        }],
        max_bytes: 1024,
        ingress_drops: Arc::clone(&io.ingress_drops),
    })
    .unwrap();
    io.session
        .put(
            "@/test@/force",
            gz_sim_proto::msgs::Double {
                data: 9.5,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .attachment("gz.msgs.Double")
        .wait()
        .unwrap();
    assert_eq!(
        receiver.recv_timeout(Duration::from_secs(2)).unwrap(),
        Some(9.5)
    );
    assert!(active.load(Ordering::Acquire));
    io.shutdown();
    assert!(!active.load(Ordering::Acquire));
    assert!(matches!(
        receiver.try_recv(),
        Err(mpsc::TryRecvError::Disconnected)
    ));
    assert!(io.control_worker.is_none());
    assert!(io.publication_worker.is_none());
    assert!(io.session.is_closed());
    io.shutdown();
}

#[test]
fn canceled_force_setup_receipt_retires_input_without_cpu_intervention() {
    let mut io = isolated_world(1024);
    let (input, receiver) = mpsc::sync_channel(1);
    let active = Arc::new(AtomicBool::new(true));
    let (receipt, ready) = mpsc::sync_channel(1);
    drop(ready);
    io.incoming
        .try_send(WorldWork::InstallForces(
            crate::force_io::ForceSetup {
                endpoints: vec![crate::force_io::ForceEndpoint {
                    key: "@/test@/canceled-force".into(),
                    input,
                    active: Arc::clone(&active),
                }],
                max_bytes: 1024,
                ingress_drops: Arc::clone(&io.ingress_drops),
            },
            receipt,
        ))
        .unwrap();
    assert!(matches!(
        receiver.recv_timeout(Duration::from_secs(2)),
        Err(mpsc::RecvTimeoutError::Disconnected)
    ));
    assert!(!active.load(Ordering::Acquire));
    io.shutdown();
}

#[test]
fn disconnected_removal_input_is_not_capacity_overflow() {
    let mut io = isolated_world(1024);
    let [input, _blocking] = io
        .removal_inputs(["@/test@/remove".into(), "@/test@/blocking".into()])
        .unwrap();
    drop(input);
    let replies = io
        .session
        .get("@/test@/remove")
        .allowed_destination(zenoh::sample::Locality::SessionLocal)
        .timeout(Duration::from_secs(2))
        .wait()
        .unwrap();
    assert!(
        replies
            .recv_timeout(Duration::from_secs(2))
            .unwrap_err()
            .to_string()
            .contains("closed")
    );
    assert_eq!(io.ingress_drops.load(Ordering::Relaxed), 0);
    io.shutdown();
}

#[test]
fn concurrent_io_polling_never_rejects_an_empty_cpu_reply_slot() {
    let count = 1024_u32;
    let failures = Arc::new(AtomicU64::new(0));
    let (requests, incoming) = mpsc::sync_channel(1);
    let reader = thread::spawn(move || {
        for expected in 0..count {
            let response: mpsc::Receiver<ControlResponse> = incoming.recv().unwrap();
            let deadline = std::time::Instant::now() + Duration::from_secs(2);
            loop {
                match response.try_recv() {
                    Ok(mut reply) => {
                        assert_eq!(reply.bytes, expected.to_le_bytes());
                        reply.accounted = true;
                        assert!(matches!(
                            response.recv_timeout(Duration::from_secs(2)),
                            Err(mpsc::RecvTimeoutError::Disconnected)
                        ));
                        break;
                    }
                    Err(mpsc::TryRecvError::Empty) => {
                        assert!(std::time::Instant::now() < deadline);
                        thread::yield_now();
                    }
                    Err(error) => panic!("reply disappeared: {error}"),
                }
            }
        }
    });
    for value in 0..count {
        let (response, replies) = mpsc::sync_channel(1);
        requests.send(replies).unwrap();
        ControlRequest {
            payload: Vec::new(),
            response,
            reply_failures: Arc::clone(&failures),
        }
        .reply(value.to_le_bytes().to_vec())
        .unwrap();
    }
    reader.join().unwrap();
    let (response, replies) = mpsc::sync_channel(1);
    drop(replies);
    assert!(
        ControlRequest {
            payload: Vec::new(),
            response,
            reply_failures: Arc::clone(&failures),
        }
        .reply(vec![1])
        .is_err()
    );
    assert_eq!(failures.load(Ordering::Relaxed), 0);
}

#[test]
fn reply_discard_after_empty_observation_counts_only_admitted_responses() {
    let failures = Arc::new(AtomicU64::new(0));
    let (response, replies) = mpsc::sync_channel(1);
    assert!(matches!(replies.try_recv(), Err(mpsc::TryRecvError::Empty)));
    ControlRequest {
        payload: Vec::new(),
        response,
        reply_failures: Arc::clone(&failures),
    }
    .reply(Vec::new())
    .unwrap();
    // Shutdown can reach the final receiver drop after a CPU admission.
    drop(replies);
    assert_eq!(failures.load(Ordering::Relaxed), 1);

    let (response, replies) = mpsc::sync_channel(1);
    let duplicate = response.clone();
    ControlRequest {
        payload: Vec::new(),
        response,
        reply_failures: Arc::clone(&failures),
    }
    .reply(vec![1])
    .unwrap();
    assert!(
        ControlRequest {
            payload: Vec::new(),
            response: duplicate,
            reply_failures: Arc::clone(&failures),
        }
        .reply(vec![2])
        .is_err()
    );
    assert_eq!(failures.load(Ordering::Relaxed), 1);
    drop(replies);
    assert_eq!(failures.load(Ordering::Relaxed), 2);
}

fn local_query(
    io: &WorldIo,
    key: &str,
    bytes: Vec<u8>,
) -> zenoh::handlers::FifoChannelHandler<zenoh::query::Reply> {
    io.session
        .get(key.to_owned())
        .payload(bytes)
        .allowed_destination(zenoh::sample::Locality::SessionLocal)
        .timeout(Duration::from_secs(2))
        .wait()
        .unwrap()
}
#[test]
fn partial_setup_rolls_back_and_duplicate_install_preserves_original_routes() {
    let io = isolated_world_with_capacity(4, 1024);
    let key = "@/test@/remove";
    assert!(
        io.removal_inputs([key.into(), "@/test@/bad//route".into()])
            .is_err()
    );
    let tokens = io
        .session
        .liveliness()
        .get("@gz/**")
        .timeout(Duration::from_secs(1))
        .wait()
        .unwrap();
    assert_eq!(
        tokens
            .into_iter()
            .filter(|reply| reply
                .result()
                .unwrap()
                .key_expr()
                .as_str()
                .contains("gz.msgs.Entity&gz.msgs.Boolean"))
            .count(),
        0
    );
    let [input, _] = io
        .removal_inputs([key.into(), "@/test@/blocking".into()])
        .unwrap();
    assert!(
        io.removal_inputs(["@/test@/other".into(), "@/test@/other/blocking".into()])
            .is_err()
    );
    let replies = local_query(&io, key, vec![8, 42]);
    let request = input.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(request.payload, vec![8, 42]);
    io.reply_boolean(request, true).unwrap();
    assert!(
        Boolean::decode(
            replies
                .recv_timeout(Duration::from_secs(2))
                .unwrap()
                .unwrap()
                .result()
                .unwrap()
                .payload()
                .to_bytes()
                .as_ref()
        )
        .unwrap()
        .data
    );
}
#[test]
fn native_oversize_is_not_copied_or_admitted_and_pending_capacity_recovers() {
    let io = isolated_world_with_capacity(1, 1);
    let keys = ["@/test@/remove".to_owned(), "@/test@/blocking".to_owned()];
    let [first, second] = io.removal_inputs(keys.clone()).unwrap();
    let oversized = local_query(&io, &keys[0], vec![8, 42]);
    assert!(oversized.recv_timeout(Duration::from_secs(2)).is_err());
    assert!(first.try_recv().is_err());
    assert_eq!(io.ingress_drops.load(Ordering::Relaxed), 0);
    let one = local_query(&io, &keys[0], Vec::new());
    let held = first.recv_timeout(Duration::from_secs(2)).unwrap();
    let blocked = local_query(&io, &keys[1], Vec::new());
    assert!(blocked.recv_timeout(Duration::from_secs(2)).is_err());
    assert!(second.try_recv().is_err());
    assert_eq!(io.ingress_drops.load(Ordering::Relaxed), 1);
    io.reply_boolean(held, false).unwrap();
    assert!(
        !Boolean::decode(
            one.recv_timeout(Duration::from_secs(2))
                .unwrap()
                .unwrap()
                .result()
                .unwrap()
                .payload()
                .to_bytes()
                .as_ref()
        )
        .unwrap()
        .data
    );
    let after = local_query(&io, &keys[1], Vec::new());
    io.reply_boolean(second.recv_timeout(Duration::from_secs(2)).unwrap(), false)
        .unwrap();
    assert!(
        !Boolean::decode(
            after
                .recv_timeout(Duration::from_secs(2))
                .unwrap()
                .unwrap()
                .result()
                .unwrap()
                .payload()
                .to_bytes()
                .as_ref()
        )
        .unwrap()
        .data
    );
}
#[test]
fn channel_shutdown_drains_real_rate_queries_after_reply_callback_returns() {
    let mut io = isolated_world_with_capacity(8, 1024);
    let sensor = gz_sim_scene::ImuPublication {
        entity_name: "sensor".into(),
        frame_id: "sensor".into(),
        topic: "/imu".into(),
        enable_orientation: true,
    };
    let config = crate::ServerOptions {
        partition: "test".into(),
        ..Default::default()
    };
    let (rates, setup) =
        crate::rates::Rates::prepare(&config, [&sensor], &io.ingress_drops).unwrap();
    io.install_rates(setup).unwrap();
    let ready = local_query(
        &io,
        "@/test@/imu/set_rate",
        gz_sim_proto::msgs::Double {
            data: 1.0,
            ..Default::default()
        }
        .encode_to_vec(),
    );
    assert!(
        ready
            .recv_timeout(Duration::from_secs(2))
            .unwrap()
            .unwrap()
            .result()
            .is_ok()
    );
    let barrier = Arc::new((std::sync::Mutex::new(false), std::sync::Condvar::new()));
    let release = Arc::clone(&barrier);
    let returned = Arc::new(AtomicBool::new(false));
    let finished_callback = Arc::clone(&returned);
    let (entered, callback_entered) = mpsc::sync_channel(1);
    io.session
        .get("@/test@/control")
        .allowed_destination(zenoh::sample::Locality::SessionLocal)
        .timeout(Duration::from_secs(3))
        .callback(move |reply| {
            assert!(
                Boolean::decode(reply.result().unwrap().payload().to_bytes().as_ref())
                    .unwrap()
                    .data
            );
            entered.send(()).unwrap();
            let (mutex, wake) = &*barrier;
            let (released, timeout) = wake
                .wait_timeout_while(mutex.lock().unwrap(), Duration::from_secs(3), |released| {
                    !*released
                })
                .unwrap();
            assert!(!timeout.timed_out() && *released);
            finished_callback.store(true, Ordering::Release);
        })
        .wait()
        .unwrap();
    let request = io.controls.recv_timeout(Duration::from_secs(2)).unwrap();
    io.reply_boolean(request, true).unwrap();
    callback_entered
        .recv_timeout(Duration::from_secs(2))
        .unwrap();
    let requests = [
        local_query(
            &io,
            "@/test@/imu/set_rate",
            gz_sim_proto::msgs::Double {
                data: 10.0,
                ..Default::default()
            }
            .encode_to_vec(),
        ),
        local_query(
            &io,
            "@/test@/imu/set_rate",
            gz_sim_proto::msgs::Double {
                data: 20.0,
                ..Default::default()
            }
            .encode_to_vec(),
        ),
    ];
    let (finished, done) = mpsc::sync_channel(1);
    let shutdown = thread::spawn(move || {
        io.shutdown();
        finished
            .send((
                io.control_worker.is_none(),
                io.publication_worker.is_none(),
                io.session.is_closed(),
            ))
            .unwrap();
    });
    assert!(matches!(
        done.recv_timeout(Duration::from_millis(30)),
        Err(mpsc::RecvTimeoutError::Timeout)
    ));
    {
        let (mutex, wake) = &*release;
        *mutex.lock().unwrap() = true;
        wake.notify_one();
    }
    assert_eq!(
        done.recv_timeout(Duration::from_secs(3)).unwrap(),
        (true, true, true)
    );
    shutdown.join().unwrap();
    assert!(returned.load(Ordering::Acquire));
    for request in requests {
        assert!(request.recv_timeout(Duration::from_secs(2)).is_err());
    }
    drop(rates);
}

#[test]
fn actual_query_discard_after_empty_observation_is_counted_once() {
    let mut io = isolated_world(1024);
    let endpoint = io
        .session
        .declare_queryable("@/test@/shutdown-scan-race")
        .wait()
        .unwrap();
    let replies = local_query(&io, "@/test@/shutdown-scan-race", Vec::new());
    let query = endpoint
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
        .unwrap();
    let (commands, incoming) = mpsc::sync_channel(1);
    let mut pending = Vec::new();
    queue_request(query, &commands, &mut pending, &io.reply_failures).unwrap();
    let plain = incoming.recv_timeout(Duration::from_secs(2)).unwrap();
    let request = pending.pop().unwrap();
    assert!(matches!(
        request.response.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));
    thread::spawn(move || plain.reply(Vec::new()))
        .join()
        .unwrap()
        .unwrap();
    drop(request);
    assert!(
        replies
            .recv_timeout(Duration::from_secs(2))
            .unwrap_err()
            .to_string()
            .contains("closed")
    );
    assert_eq!(io.reply_failures.load(Ordering::Relaxed), 1);
    io.shutdown();
    assert_eq!(io.reply_failures.load(Ordering::Relaxed), 1);
}

#[test]
fn closed_session_ready_reply_failure_is_counted_once() {
    let mut io = isolated_world(1024);
    let endpoint = io
        .session
        .declare_queryable("@/test@/closed-reply")
        .wait()
        .unwrap();
    let replies = local_query(&io, "@/test@/closed-reply", Vec::new());
    let query = endpoint
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
        .unwrap();
    let (commands, incoming) = mpsc::sync_channel(1);
    let mut pending = Vec::new();
    queue_request(query, &commands, &mut pending, &io.reply_failures).unwrap();
    incoming
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
        .reply(Vec::new())
        .unwrap();
    io.session.close().wait().unwrap();
    reply_controls(&mut pending, &io.session, &io.reply_failures);
    assert!(pending.is_empty());
    assert_eq!(io.reply_failures.load(Ordering::Relaxed), 1);
    assert!(replies.recv_timeout(Duration::from_secs(2)).is_err());
    io.shutdown();
    assert_eq!(io.reply_failures.load(Ordering::Relaxed), 1);
}

#[test]
fn world_construction_rejects_invalid_bounds_and_qualified_metadata() {
    let mut config = zenoh::Config::default();
    for (path, value) in [
        ("listen/endpoints", "[\"tcp/127.0.0.1:0\"]"),
        ("scouting/multicast/enabled", "false"),
        ("scouting/gossip/enabled", "false"),
    ] {
        config.insert_json5(path, value).unwrap();
    }
    let keys = ["@/test@/clock", "@/test@/control", "@/test@/poses"];
    for (capacity, bytes) in [(0, 1), (1, 0)] {
        let error = WorldIo::new_with_config(
            keys[0].into(),
            keys[1].into(),
            keys[2].into(),
            capacity,
            bytes,
            config.clone(),
        )
        .err()
        .unwrap();
        assert!(error.to_string().contains("invalid world ingress bounds"));
    }
    for (index, invalid, message) in [
        (0, "clock", "invalid qualified world key"),
        (1, "control", "invalid qualified world key"),
        (2, "poses", "invalid qualified world key"),
        (0, "@@/clock", "unsupported world key"),
        (0, "@/test@clock", "unsupported world key"),
    ] {
        let mut input = keys.map(str::to_owned);
        input[index] = invalid.into();
        let [clock, control, poses] = input;
        let error = WorldIo::new_with_config(clock, control, poses, 1, 1024, config.clone())
            .err()
            .unwrap();
        assert!(error.to_string().contains(message), "{invalid}: {error}");
    }
    let mut io = isolated_world(1024);
    assert!(
        io.prepare_root_clock("clock".into(), Duration::from_secs(2))
            .is_err()
    );
    let root = io
        .prepare_root_clock("@/test@/root".into(), Duration::from_secs(2))
        .unwrap()
        .unwrap();
    assert_eq!(root.publisher.key_expr().as_str(), "@/test@/root");
    io.shutdown();
}

#[test]
fn world_control_oversize_and_full_ingress_preserve_admission_and_recovery() {
    struct Release(Arc<(std::sync::Mutex<bool>, std::sync::Condvar)>);
    impl Drop for Release {
        fn drop(&mut self) {
            let (state, wake) = &*self.0;
            *state.lock().unwrap() = true;
            wake.notify_one();
        }
    }
    let mut io = isolated_world_with_capacity(1, 2);
    let oversized = gz_sim_proto::msgs::WorldControl {
        pause: true,
        multi_step: 9,
        ..Default::default()
    }
    .encode_to_vec();
    assert!(gz_sim_proto::msgs::WorldControl::decode(oversized.as_slice()).is_ok());
    assert!(oversized.len() > io.max_message_bytes);
    let rejected = local_query(&io, "@/test@/control", oversized);
    assert!(
        rejected
            .recv_timeout(Duration::from_secs(2))
            .unwrap_err()
            .to_string()
            .contains("closed")
    );
    assert!(matches!(
        io.controls.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));
    assert_eq!(io.rejected.load(Ordering::Relaxed), 1);
    assert_eq!(io.ingress_drops.load(Ordering::Relaxed), 0);
    let recovered = local_query(&io, "@/test@/control", Vec::new());
    io.reply_boolean(
        io.controls.recv_timeout(Duration::from_secs(2)).unwrap(),
        false,
    )
    .unwrap();
    assert!(
        !Boolean::decode(
            recovered
                .recv_timeout(Duration::from_secs(2))
                .unwrap()
                .unwrap()
                .result()
                .unwrap()
                .payload()
                .to_bytes()
                .as_ref()
        )
        .unwrap()
        .data
    );
    let barrier = Arc::new((std::sync::Mutex::new(false), std::sync::Condvar::new()));
    let release = Release(Arc::clone(&barrier));
    let returned = Arc::new(AtomicBool::new(false));
    let finished = Arc::clone(&returned);
    let (entered, callback_entered) = mpsc::sync_channel(1);
    io.session
        .get("@/test@/control")
        .allowed_destination(zenoh::sample::Locality::SessionLocal)
        .timeout(Duration::from_secs(3))
        .callback(move |reply| {
            assert!(
                Boolean::decode(reply.result().unwrap().payload().to_bytes().as_ref())
                    .unwrap()
                    .data
            );
            entered.send(()).unwrap();
            let (state, wake) = &*barrier;
            let (state, timed) = wake
                .wait_timeout_while(state.lock().unwrap(), Duration::from_secs(3), |released| {
                    !*released
                })
                .unwrap();
            assert!(!timed.timed_out() && *state);
            finished.store(true, Ordering::Release);
        })
        .wait()
        .unwrap();
    io.reply_boolean(
        io.controls.recv_timeout(Duration::from_secs(2)).unwrap(),
        true,
    )
    .unwrap();
    callback_entered
        .recv_timeout(Duration::from_secs(2))
        .unwrap();
    let queued = local_query(&io, "@/test@/control", Vec::new());
    let full = local_query(&io, "@/test@/control", Vec::new());
    assert!(
        full.recv_timeout(Duration::from_secs(2))
            .unwrap_err()
            .to_string()
            .contains("closed")
    );
    assert!(matches!(
        io.controls.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));
    assert_eq!(io.rejected.load(Ordering::Relaxed), 2);
    assert_eq!(io.ingress_drops.load(Ordering::Relaxed), 1);
    drop(release);
    let request = io.controls.recv_timeout(Duration::from_secs(2)).unwrap();
    assert!(returned.load(Ordering::Acquire));
    assert!(request.payload.is_empty());
    io.reply_boolean(request, false).unwrap();
    assert!(
        !Boolean::decode(
            queued
                .recv_timeout(Duration::from_secs(2))
                .unwrap()
                .unwrap()
                .result()
                .unwrap()
                .payload()
                .to_bytes()
                .as_ref()
        )
        .unwrap()
        .data
    );
    assert_eq!(io.rejected.load(Ordering::Relaxed), 2);
    assert_eq!(io.ingress_drops.load(Ordering::Relaxed), 1);
    assert_eq!(io.reply_failures.load(Ordering::Relaxed), 0);
    io.shutdown();
}

#[test]
fn failed_sensor_setups_propagate_errors_without_poisoning_valid_installation() {
    let mut io = isolated_world(1024);
    let invalid = "@/test@/invalid/#";
    assert!(zenoh::key_expr::OwnedKeyExpr::try_from(invalid).is_err());
    assert!(io.imu_output(invalid.into()).is_err());
    let imu = io.imu_output("@/test@/imu".into()).unwrap();
    let sensor = gz_sim_scene::ImuPublication {
        entity_name: "sensor".into(),
        frame_id: "sensor".into(),
        topic: "/imu".into(),
        enable_orientation: true,
    };
    let mut options = crate::ServerOptions {
        partition: "test".into(),
        ..Default::default()
    };
    options
        .add_topic_remap("/imu/set_rate", "/invalid/#")
        .unwrap();
    let (mut bad_rates, setup) =
        crate::rates::Rates::prepare(&options, [&sensor], &io.ingress_drops).unwrap();
    assert!(matches!(
        io.install_rates(setup),
        Err(crate::Error::WorldNetwork(_))
    ));
    bad_rates.clear();
    options.topic_remaps.clear();
    let (mut rates, setup) =
        crate::rates::Rates::prepare(&options, [&sensor], &io.ingress_drops).unwrap();
    io.install_rates(setup).unwrap();
    let reply = local_query(
        &io,
        "@/test@/imu/set_rate",
        gz_sim_proto::msgs::Double {
            data: 3.0,
            ..Default::default()
        }
        .encode_to_vec(),
    );
    assert!(
        reply
            .recv_timeout(Duration::from_secs(2))
            .unwrap()
            .unwrap()
            .result()
            .is_ok()
    );
    let (bad_input, bad_receiver) = mpsc::sync_channel(1);
    let bad_active = Arc::new(AtomicBool::new(true));
    let bad_setup = crate::force_io::ForceSetup {
        endpoints: vec![crate::force_io::ForceEndpoint {
            key: invalid.into(),
            input: bad_input,
            active: Arc::clone(&bad_active),
        }],
        max_bytes: 1024,
        ingress_drops: Arc::clone(&io.ingress_drops),
    };
    assert!(matches!(
        io.install_forces(bad_setup),
        Err(crate::Error::WorldNetwork(_))
    ));
    bad_active.store(false, Ordering::Release);
    assert!(matches!(
        bad_receiver.try_recv(),
        Err(mpsc::TryRecvError::Disconnected)
    ));
    let (input, received) = mpsc::sync_channel(1);
    let active = Arc::new(AtomicBool::new(true));
    io.install_forces(crate::force_io::ForceSetup {
        endpoints: vec![crate::force_io::ForceEndpoint {
            key: "@/test@/force".into(),
            input,
            active: Arc::clone(&active),
        }],
        max_bytes: 1024,
        ingress_drops: Arc::clone(&io.ingress_drops),
    })
    .unwrap();
    io.session
        .put(
            "@/test@/force",
            gz_sim_proto::msgs::Double {
                data: 1.25,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .attachment("gz.msgs.Double")
        .wait()
        .unwrap();
    assert_eq!(
        received.recv_timeout(Duration::from_secs(2)).unwrap(),
        Some(1.25)
    );
    io.shutdown();
    assert!(!active.load(Ordering::Acquire));
    assert!(matches!(
        received.try_recv(),
        Err(mpsc::TryRecvError::Disconnected)
    ));
    rates.clear();
    drop(imu);
}

#[test]
fn publication_caps_and_shutdown_reject_new_frames_and_setup() {
    fn observe(
        session: &zenoh::Session,
        key: &str,
    ) -> (
        zenoh::pubsub::Subscriber<()>,
        mpsc::Receiver<zenoh::sample::Sample>,
    ) {
        let (send, receive) = mpsc::sync_channel(1);
        let subscriber = session
            .declare_subscriber(key.to_owned())
            .allowed_origin(zenoh::sample::Locality::SessionLocal)
            .callback(move |sample| send.try_send(sample).unwrap())
            .wait()
            .unwrap();
        (subscriber, receive)
    }
    let mut io = isolated_world_with_capacity(8, 2);
    let root = io
        .prepare_root_clock("@/test@/root".into(), Duration::from_secs(2))
        .unwrap()
        .unwrap();
    let imu = io.imu_output("@/test@/imu".into()).unwrap();
    let observers = [
        "@/test@/clock",
        "@/test@/poses",
        "@/test@/root",
        "@/test@/imu",
    ]
    .map(|key| observe(&io.session, key));
    let time = gz_sim_proto::msgs::Time {
        sec: 1,
        ..Default::default()
    };
    let stamp = gz_sim_proto::msgs::Header {
        stamp: Some(time),
        ..Default::default()
    };
    let large_clock = Clock {
        sim: Some(time),
        ..Default::default()
    }
    .encode_to_vec();
    let large_poses = PoseV {
        header: Some(stamp.clone()),
        ..Default::default()
    }
    .encode_to_vec();
    let large_imu = gz_sim_proto::msgs::Imu {
        header: Some(stamp),
        ..Default::default()
    }
    .encode_to_vec();
    assert!(
        [large_clock.len(), large_poses.len(), large_imu.len()]
            .iter()
            .all(|size| *size > io.max_message_bytes)
    );
    assert!(io.publish_clock(large_clock.clone()).is_err());
    assert!(io.publish_poses(large_poses).is_err());
    assert!(io.publish_root_clock(&root, large_clock).is_err());
    assert!(io.publish_imu(&imu, large_imu).is_err());
    for (_, receive) in &observers {
        assert!(matches!(
            receive.recv_timeout(Duration::from_millis(20)),
            Err(mpsc::RecvTimeoutError::Timeout)
        ));
    }
    let clock = Clock {
        sim: Some(Default::default()),
        ..Default::default()
    }
    .encode_to_vec();
    let poses = PoseV {
        header: Some(Default::default()),
        ..Default::default()
    }
    .encode_to_vec();
    let imu_frame = gz_sim_proto::msgs::Imu {
        header: Some(Default::default()),
        ..Default::default()
    }
    .encode_to_vec();
    assert!(
        [clock.len(), poses.len(), imu_frame.len()]
            .iter()
            .all(|size| *size == io.max_message_bytes)
    );
    io.publish_clock(clock.clone()).unwrap();
    io.publish_poses(poses.clone()).unwrap();
    io.publish_root_clock(&root, clock.clone()).unwrap();
    io.publish_imu(&imu, imu_frame.clone()).unwrap();
    for ((_, receive), (key, kind, bytes)) in observers.iter().zip([
        ("@/test@/clock", "gz.msgs.Clock", clock.clone()),
        ("@/test@/poses", "gz.msgs.Pose_V", poses.clone()),
        ("@/test@/root", "gz.msgs.Clock", clock.clone()),
        ("@/test@/imu", "gz.msgs.IMU", imu_frame.clone()),
    ]) {
        let sample = receive.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(sample.key_expr().as_str(), key);
        assert_eq!(
            sample.attachment().unwrap().to_bytes().as_ref(),
            kind.as_bytes()
        );
        assert_eq!(sample.payload().to_bytes().as_ref(), bytes);
    }
    io.shutdown();
    assert!(io.publish_clock(clock.clone()).is_err());
    assert!(io.publish_poses(poses).is_err());
    assert!(io.publish_root_clock(&root, clock).is_err());
    assert!(io.publish_imu(&imu, imu_frame).is_err());
    let options = crate::ServerOptions {
        partition: "test".into(),
        ..Default::default()
    };
    let (_, rates) = crate::rates::Rates::prepare(
        &options,
        std::iter::empty::<&gz_sim_scene::ImuPublication>(),
        &io.ingress_drops,
    )
    .unwrap();
    assert!(matches!(
        io.install_rates(rates),
        Err(crate::Error::WorldClosed)
    ));
    let (_, forces) =
        crate::forces::Forces::prepare(&options, std::iter::empty(), &io.ingress_drops).unwrap();
    assert!(matches!(
        io.install_forces(forces),
        Err(crate::Error::WorldClosed)
    ));
    assert!(io.control_worker.is_none() && io.publication_worker.is_none());
    assert_eq!(io.reply_failures.load(Ordering::Relaxed), 0);
}

#[test]
fn full_publication_queue_rejects_new_frames_and_recovers_after_callback_release() {
    struct Release(Arc<(std::sync::Mutex<bool>, std::sync::Condvar)>);
    impl Drop for Release {
        fn drop(&mut self) {
            let (state, wake) = &*self.0;
            *state.lock().unwrap() = true;
            wake.notify_one();
        }
    }
    let mut io = isolated_world_with_capacity(1, 1024);
    let root = io
        .prepare_root_clock("@/test@/root".into(), Duration::from_secs(2))
        .unwrap()
        .unwrap();
    let imu = io.imu_output("@/test@/imu".into()).unwrap();
    let barrier = Arc::new((std::sync::Mutex::new(false), std::sync::Condvar::new()));
    let release = Release(Arc::clone(&barrier));
    let (entered, callback_entered) = mpsc::sync_channel(1);
    let clock_observer = io
        .session
        .declare_subscriber("@/test@/clock")
        .allowed_origin(zenoh::sample::Locality::SessionLocal)
        .callback(move |sample| {
            entered.send(sample).unwrap();
            let (state, wake) = &*barrier;
            let (state, timed) = wake
                .wait_timeout_while(state.lock().unwrap(), Duration::from_secs(3), |released| {
                    !*released
                })
                .unwrap();
            assert!(!timed.timed_out() && *state);
        })
        .wait()
        .unwrap();
    let observers = ["@/test@/poses", "@/test@/root", "@/test@/imu"].map(|key| {
        let (send, received) = mpsc::sync_channel(1);
        let subscriber = io
            .session
            .declare_subscriber(key)
            .allowed_origin(zenoh::sample::Locality::SessionLocal)
            .callback(move |sample| send.try_send(sample).unwrap())
            .wait()
            .unwrap();
        (subscriber, received)
    });
    let clock = Clock {
        sim: Some(gz_sim_proto::msgs::Time {
            sec: 1,
            ..Default::default()
        }),
        ..Default::default()
    }
    .encode_to_vec();
    let poses = PoseV {
        header: Some(Default::default()),
        ..Default::default()
    }
    .encode_to_vec();
    let imu_frame = gz_sim_proto::msgs::Imu {
        header: Some(Default::default()),
        ..Default::default()
    }
    .encode_to_vec();
    io.publish_clock(clock.clone()).unwrap();
    let sample = callback_entered
        .recv_timeout(Duration::from_secs(2))
        .unwrap();
    assert_eq!(sample.payload().to_bytes().as_ref(), clock.as_slice());
    io.publish_poses(poses.clone()).unwrap();
    for result in [
        io.publish_clock(clock.clone()),
        io.publish_poses(poses.clone()),
        io.publish_root_clock(&root, clock.clone()),
        io.publish_imu(&imu, imu_frame.clone()),
    ] {
        assert_eq!(
            result.unwrap_err().to_string(),
            "world publication queue unavailable"
        );
    }
    for (_, received) in &observers {
        assert!(matches!(
            received.try_recv(),
            Err(mpsc::TryRecvError::Empty)
        ));
    }
    drop(release);
    let queued = observers[0].1.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(queued.payload().to_bytes().as_ref(), poses.as_slice());
    for (_, received) in &observers[1..] {
        assert!(matches!(
            received.recv_timeout(Duration::from_millis(20)),
            Err(mpsc::RecvTimeoutError::Timeout)
        ));
    }
    io.publish_root_clock(&root, clock.clone()).unwrap();
    let recovered = observers[1].1.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(recovered.payload().to_bytes().as_ref(), clock.as_slice());
    io.publish_imu(&imu, imu_frame.clone()).unwrap();
    let recovered = observers[2].1.recv_timeout(Duration::from_secs(2)).unwrap();
    assert_eq!(
        recovered.payload().to_bytes().as_ref(),
        imu_frame.as_slice()
    );
    assert!(
        io.publication_failures
            .iter()
            .all(|count| count.load(Ordering::Relaxed) == 0)
    );
    io.shutdown();
    drop(clock_observer);
}
