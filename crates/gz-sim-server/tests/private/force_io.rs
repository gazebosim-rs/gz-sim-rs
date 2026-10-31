// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::time::Duration;

fn session() -> zenoh::Session {
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"peer\"").unwrap();
    config
        .insert_json5("listen/endpoints", "[\"tcp/127.0.0.1:0\"]")
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .unwrap();
    zenoh::open(config).wait().unwrap()
}
fn make_endpoint(capacity: usize) -> (ForceEndpoint, mpsc::Receiver<Option<f64>>) {
    let (input, receiver) = mpsc::sync_channel(capacity);
    (
        ForceEndpoint {
            key: format!("@/force-{}@/joint/cmd_force", uuid::Uuid::new_v4()),
            input,
            active: Arc::new(AtomicBool::new(true)),
        },
        receiver,
    )
}
fn setup(endpoints: Vec<ForceEndpoint>, drops: &Arc<AtomicU64>, max_bytes: usize) -> ForceSetup {
    ForceSetup {
        endpoints,
        max_bytes,
        ingress_drops: Arc::clone(drops),
    }
}
fn put(session: &zenoh::Session, key: &str, bytes: Vec<u8>, attachment: Option<Vec<u8>>) {
    let put = session.put(key.to_owned(), bytes);
    match attachment {
        Some(attachment) => put.attachment(attachment).wait().unwrap(),
        None => put.wait().unwrap(),
    }
}
fn scalar(session: &zenoh::Session, key: &str, data: f64) {
    put(
        session,
        key,
        Double {
            data,
            ..Default::default()
        }
        .encode_to_vec(),
        Some(Double::full_name().into_bytes()),
    );
}
fn receive(input: &mpsc::Receiver<Option<f64>>) -> Option<f64> {
    input.recv_timeout(Duration::from_secs(3)).unwrap()
}
fn tokens(session: &zenoh::Session) -> usize {
    session
        .liveliness()
        .get("@gz/**")
        .timeout(Duration::from_secs(1))
        .wait()
        .unwrap()
        .into_iter()
        .fold(0, |count, reply| {
            reply.into_result().unwrap();
            count + 1
        })
}

#[test]
fn standard_double_domain_and_unknown_fields_reach_only_bounded_scalar_cpu_state() {
    let session = session();
    let drops = Arc::new(AtomicU64::new(0));
    let (endpoint, receiver) = make_endpoint(1);
    let key = endpoint.key.clone();
    let _declarations =
        ForceDeclarations::new(&session, setup(vec![endpoint], &drops, 1024)).unwrap();
    for value in [
        1.25,
        -8.0,
        0.0,
        -0.0,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NAN,
    ] {
        scalar(&session, &key, value);
        assert_eq!(receive(&receiver).unwrap().to_bits(), value.to_bits());
    }
    let mut bytes = Double {
        data: 9.0,
        ..Default::default()
    }
    .encode_to_vec();
    bytes.extend([0x98, 0x06, 0x01]);
    put(
        &session,
        &key,
        bytes,
        Some(Double::full_name().into_bytes()),
    );
    assert_eq!(receive(&receiver), Some(9.0));
    // Both accepted buffers may be fragmented by the maintained transport API.
    let bytes = Double {
        data: -0.0,
        ..Default::default()
    }
    .encode_to_vec();
    let mut payload = zenoh::bytes::ZBytes::writer();
    for byte in bytes {
        payload.append(vec![byte].into());
    }
    let mut attachment = zenoh::bytes::ZBytes::writer();
    for byte in Double::full_name().into_bytes() {
        attachment.append(vec![byte].into());
    }
    session
        .put(key, zenoh::bytes::ZBytes::from(payload))
        .attachment(zenoh::bytes::ZBytes::from(attachment))
        .wait()
        .unwrap();
    assert_eq!(receive(&receiver).unwrap().to_bits(), (-0.0f64).to_bits());
    assert_eq!(drops.load(Ordering::Relaxed), 0);
}

#[test]
fn malformed_wire_type_and_oversized_buffers_preserve_one_invalid_budget_marker() {
    let session = session();
    let drops = Arc::new(AtomicU64::new(0));
    let (endpoint, receiver) = make_endpoint(1);
    let key = endpoint.key.clone();
    let _declarations =
        ForceDeclarations::new(&session, setup(vec![endpoint], &drops, 16)).unwrap();
    for (bytes, attachment) in [
        (vec![0xff], Some(Double::full_name().into_bytes())),
        (Vec::new(), None),
        (Vec::new(), Some(b"gz.msgs.Float".to_vec())),
        (Vec::new(), Some(vec![0xff; Double::full_name().len()])),
        (vec![0; 17], Some(Double::full_name().into_bytes())),
        (Vec::new(), Some(vec![0; 1024 * 1024])),
    ] {
        put(&session, &key, bytes, attachment);
        assert_eq!(receive(&receiver), None);
        assert!(receiver.try_recv().is_err());
    }
    scalar(&session, &key, -0.0);
    assert_eq!(receive(&receiver).unwrap().to_bits(), (-0.0f64).to_bits());
}

#[test]
fn real_full_drops_accumulate_on_world_metric_and_disconnected_receiver_is_not_a_drop() {
    let session = session();
    let drops = Arc::new(AtomicU64::new(7));
    let (endpoint, receiver) = make_endpoint(1);
    let key = endpoint.key.clone();
    let active = Arc::clone(&endpoint.active);
    let mut declarations =
        ForceDeclarations::new(&session, setup(vec![endpoint], &drops, 1024)).unwrap();
    assert!(Arc::ptr_eq(
        &declarations.endpoints[0].state.ingress_drops,
        &drops
    ));
    for value in [1.0, 2.0, 3.0] {
        scalar(&session, &key, value);
    }
    assert_eq!(drops.load(Ordering::Relaxed), 9);
    assert_eq!(receive(&receiver), Some(1.0));
    drop(receiver);
    scalar(&session, &key, 4.0);
    assert_eq!(drops.load(Ordering::Relaxed), 9);
    active.store(false, Ordering::Release);
    assert!(declarations.reconcile());
    assert!(declarations.is_empty());
    let (next, receiver) = make_endpoint(1);
    let key = next.key.clone();
    let _next = ForceDeclarations::new(&session, setup(vec![next], &drops, 1024)).unwrap();
    for value in [5.0, 6.0] {
        scalar(&session, &key, value);
    }
    assert_eq!(receive(&receiver), Some(5.0));
    assert_eq!(drops.load(Ordering::Relaxed), 10);
}

#[test]
fn inactive_and_empty_setup_have_no_native_declarations_or_retained_capacity() {
    let session = session();
    let drops = Arc::new(AtomicU64::new(0));
    let empty = setup(Vec::new(), &drops, 1);
    assert!(empty.is_empty());
    let mut declarations = ForceDeclarations::new(&session, empty).unwrap();
    assert!(declarations.is_empty());
    assert_eq!(declarations.endpoints.capacity(), 0);
    assert!(declarations.reconcile());
    let (endpoint, receiver) = make_endpoint(1);
    endpoint.active.store(false, Ordering::Release);
    let declarations = ForceDeclarations::new(&session, setup(vec![endpoint], &drops, 1)).unwrap();
    assert!(declarations.is_empty());
    assert_eq!(declarations.endpoints.capacity(), 0);
    assert_eq!(tokens(&session), 0);
    assert!(receiver.try_recv().is_err());
    assert_eq!(Arc::strong_count(&drops), 1);
}

#[test]
fn retirement_disables_handoff_then_io_owner_removes_native_tokens_and_capture() {
    let session = session();
    let drops = Arc::new(AtomicU64::new(0));
    let (endpoint, receiver) = make_endpoint(1);
    let key = endpoint.key.clone();
    let active = Arc::clone(&endpoint.active);
    let mut declarations =
        ForceDeclarations::new(&session, setup(vec![endpoint], &drops, 1024)).unwrap();
    let weak = Arc::downgrade(&declarations.endpoints[0].state);
    assert_eq!(tokens(&session), 1);
    active.store(false, Ordering::Release);
    scalar(&session, &key, 1.0);
    assert!(receiver.try_recv().is_err());
    assert!(declarations.reconcile());
    assert!(declarations.is_empty());
    assert_eq!(declarations.endpoints.capacity(), 0);
    assert!(weak.upgrade().is_none());
    assert_eq!(tokens(&session), 0);
}

#[test]
fn setup_errors_never_leave_partial_active_endpoints() {
    let session = session();
    let drops = Arc::new(AtomicU64::new(0));
    for limit in [0, usize::MAX] {
        assert!(ForceDeclarations::new(&session, setup(Vec::new(), &drops, limit)).is_err());
    }
    let (first, _) = make_endpoint(1);
    let (mut duplicate, _) = make_endpoint(1);
    duplicate.key = first.key.clone();
    assert!(ForceDeclarations::new(&session, setup(vec![first, duplicate], &drops, 1024)).is_err());
    assert_eq!(tokens(&session), 0);
    for bad_key in ["invalid", "@/force@/bad//key"] {
        let (first, _) = make_endpoint(1);
        let gate = Arc::clone(&first.active);
        let (mut invalid, _) = make_endpoint(1);
        invalid.key = bad_key.into();
        assert!(
            ForceDeclarations::new(&session, setup(vec![first, invalid], &drops, 1024)).is_err()
        );
        assert!(!gate.load(Ordering::Acquire));
        assert_eq!(tokens(&session), 0);
    }
    session.close().wait().unwrap();
    assert!(ForceDeclarations::new(&session, setup(Vec::new(), &drops, 1024)).is_err());
}

#[test]
fn real_running_callback_keeps_only_bounded_release_witness_after_io_retirement() {
    let session = session();
    let drops = Arc::new(AtomicU64::new(0));
    let (endpoint, receiver) = make_endpoint(1);
    let key = endpoint.key.clone();
    let active = Arc::clone(&endpoint.active);
    let setup = setup(vec![endpoint], &drops, 1024);
    let release = Arc::new((std::sync::Mutex::new(false), std::sync::Condvar::new()));
    let callback_release = Arc::clone(&release);
    let (started, entered) = mpsc::sync_channel(1);
    let (ready, prepared) = mpsc::sync_channel(1);
    let (retire, retiring) = mpsc::sync_channel(1);
    let (witness, observed) = mpsc::sync_channel(1);
    let (finish, resumed) = mpsc::sync_channel(1);
    let io_session = session.clone();
    let io_key = key.clone();
    let io = std::thread::spawn(move || {
        let mut declarations = ForceDeclarations::new(&io_session, setup).unwrap();
        let endpoint = &mut declarations.endpoints[0];
        // Replace only the adapter with a controlled real upstream callback. The
        // production receive function and declaration state/lifetime stay unchanged.
        drop(endpoint.subscriber.take());
        let state = Arc::clone(&endpoint.state);
        let weak = Arc::downgrade(&state);
        endpoint.subscriber = Some(
            io_session
                .declare_subscriber(io_key)
                .callback(move |sample| {
                    started.send(()).unwrap();
                    let (lock, condition) = &*callback_release;
                    drop(
                        condition
                            .wait_while(lock.lock().unwrap(), |released| !*released)
                            .unwrap(),
                    );
                    state.receive(sample);
                })
                .wait()
                .unwrap(),
        );
        ready.send(()).unwrap();
        retiring.recv().unwrap();
        let drained = declarations.reconcile();
        witness
            .send((
                drained,
                declarations.endpoints.len(),
                declarations.endpoints[0].subscriber.is_none(),
                declarations.endpoints[0].presence.is_none(),
            ))
            .unwrap();
        resumed.recv().unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(3);
        while !declarations.reconcile() {
            assert!(std::time::Instant::now() < deadline);
            std::thread::yield_now();
        }
        assert!(declarations.is_empty());
        assert_eq!(declarations.endpoints.capacity(), 0);
        assert!(weak.upgrade().is_none());
    });
    prepared.recv_timeout(Duration::from_secs(3)).unwrap();
    let writer = session.clone();
    let publisher = std::thread::spawn(move || scalar(&writer, &key, 1.0));
    entered.recv_timeout(Duration::from_secs(3)).unwrap();
    active.store(false, Ordering::Release);
    drop(receiver);
    retire.send(()).unwrap();
    let released_without_waiting_for_callback = observed.recv_timeout(Duration::from_secs(1));
    *release.0.lock().unwrap() = true;
    release.1.notify_all();
    publisher.join().unwrap();
    finish.send(()).unwrap();
    io.join().unwrap();
    assert_eq!(
        released_without_waiting_for_callback.unwrap(),
        (false, 1, true, true)
    );
    assert_eq!(tokens(&session), 0);
    assert_eq!(drops.load(Ordering::Relaxed), 0);
    assert_eq!(Arc::strong_count(&drops), 1);
}

#[test]
fn declaration_shutdown_disables_cpu_gate_before_native_capture_cleanup() {
    let session = session();
    let drops = Arc::new(AtomicU64::new(0));
    let (endpoint, receiver) = make_endpoint(1);
    let active = Arc::clone(&endpoint.active);
    let mut declarations =
        ForceDeclarations::new(&session, setup(vec![endpoint], &drops, 1024)).unwrap();
    declarations.begin_shutdown();
    assert!(!active.load(Ordering::Acquire));
    assert!(declarations.reconcile());
    assert!(declarations.is_empty());
    assert!(receiver.try_recv().is_err());
    assert_eq!(tokens(&session), 0);
}
