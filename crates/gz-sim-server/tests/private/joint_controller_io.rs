// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::time::{Duration, Instant};

fn session() -> zenoh::Session {
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
    zenoh::open(config).wait().unwrap()
}
fn setup(key: String, input: mpsc::SyncSender<Command>) -> Setup {
    Setup {
        endpoints: vec![Endpoint { key, input }],
        max_bytes: 64,
        drops: Arc::new(AtomicU64::new(0)),
        epoch: Arc::new(AtomicU64::new(7)),
    }
}
fn wait(mut condition: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(3);
    while !condition() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
}

#[test]
fn actual_double_packets_validate_type_bytes_finiteness_budget_and_acquisition_epoch() {
    let session = session();
    let key = format!("@/joint-{}@/command", uuid::Uuid::new_v4());
    let publisher = session.declare_publisher(key.clone()).wait().unwrap();
    let (input, receiver) = mpsc::sync_channel(1);
    let setup = setup(key, input);
    let drops = Arc::clone(&setup.drops);
    let epoch = Arc::clone(&setup.epoch);
    let (mut declarations, leases) = Declarations::new(&session, setup, "process", "node").unwrap();
    wait(|| publisher.matching_status().wait().unwrap().matching());
    for value in [0.0, -1.25, f64::MAX] {
        publisher
            .put(
                Double {
                    data: value,
                    ..Default::default()
                }
                .encode_to_vec(),
            )
            .attachment(Double::full_name())
            .wait()
            .unwrap();
        assert_eq!(
            receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
            (7, Some(value))
        );
    }
    for (bytes, attachment) in [
        (vec![255], Double::full_name()),
        (vec![0; 65], Double::full_name()),
        (
            Double {
                data: 1.0,
                ..Default::default()
            }
            .encode_to_vec(),
            "gz.msgs.Twist".into(),
        ),
        (
            Double {
                data: f64::NAN,
                ..Default::default()
            }
            .encode_to_vec(),
            Double::full_name(),
        ),
        (
            Double {
                data: f64::INFINITY,
                ..Default::default()
            }
            .encode_to_vec(),
            Double::full_name(),
        ),
        (
            Double {
                data: f64::NEG_INFINITY,
                ..Default::default()
            }
            .encode_to_vec(),
            Double::full_name(),
        ),
    ] {
        publisher.put(bytes).attachment(attachment).wait().unwrap();
        assert_eq!(
            receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
            (7, None)
        );
    }
    publisher.put(vec![]).wait().unwrap();
    assert_eq!(
        receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
        (7, None)
    );
    let monitor = session
        .declare_subscriber(publisher.key_expr().clone())
        .wait()
        .unwrap();
    let bytes = Double {
        data: 2.0,
        ..Default::default()
    }
    .encode_to_vec();
    publisher
        .put(bytes.clone())
        .attachment(Double::full_name())
        .wait()
        .unwrap();
    let captured = monitor
        .recv_timeout(Duration::from_secs(3))
        .unwrap()
        .unwrap();
    assert_eq!(
        receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
        (7, Some(2.0))
    );
    // Replay the actual upstream sample synchronously to fence the admission instant.
    declarations.endpoints[0].state.receive(captured);
    epoch.store(8, Ordering::Release);
    publisher
        .put(bytes.clone())
        .attachment(Double::full_name())
        .wait()
        .unwrap();
    wait(|| drops.load(Ordering::Relaxed) == 1);
    // Updating the clock cannot relabel a command already admitted in epoch seven.
    assert_eq!(
        receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
        (7, Some(2.0))
    );
    publisher
        .put(bytes)
        .attachment(Double::full_name())
        .wait()
        .unwrap();
    assert_eq!(
        receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
        (8, Some(2.0))
    );
    drop(leases);
    wait(|| declarations.reconcile(false));
    session.close().wait().unwrap();
}

#[test]
fn retiring_one_stream_releases_its_lease_and_capture_without_retiring_the_other() {
    let session = session();
    let prefix = format!("@/joint-{}@", uuid::Uuid::new_v4());
    let first_key = format!("{prefix}/first");
    let second_key = format!("{prefix}/second");
    let publisher = session.declare_publisher(first_key.clone()).wait().unwrap();
    let monitor = session
        .declare_subscriber(first_key.clone())
        .wait()
        .unwrap();
    let second = session
        .declare_publisher(second_key.clone())
        .wait()
        .unwrap();
    let (input, receiver) = mpsc::sync_channel(2);
    let (second_input, second_receiver) = mpsc::sync_channel(2);
    let mut setup = setup(first_key, input);
    setup.endpoints.push(Endpoint {
        key: second_key,
        input: second_input,
    });
    let (mut declarations, mut leases) =
        Declarations::new(&session, setup, "process", "node").unwrap();
    wait(|| second.matching_status().wait().unwrap().matching());
    publisher
        .put(
            Double {
                data: 1.0,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .attachment(Double::full_name())
        .wait()
        .unwrap();
    let sample = monitor
        .recv_timeout(Duration::from_secs(3))
        .unwrap()
        .unwrap();
    assert_eq!(
        receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
        (7, Some(1.0))
    );
    let held = Arc::clone(&declarations.endpoints[0].state);
    let weak = Arc::downgrade(&leases[0]);
    drop(leases.remove(0));
    assert!(weak.upgrade().is_none());
    assert!(!declarations.reconcile(false));
    assert!(declarations.endpoints[0].subscriber.is_none());
    assert!(declarations.endpoints[0].presence.is_none());
    held.receive(sample);
    assert!(receiver.try_recv().is_err());
    second
        .put(
            Double {
                data: -2.0,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .attachment(Double::full_name())
        .wait()
        .unwrap();
    assert_eq!(
        second_receiver
            .recv_timeout(Duration::from_secs(3))
            .unwrap(),
        (7, Some(-2.0))
    );
    drop(held);
    wait(|| {
        declarations.reconcile(false);
        declarations.endpoints.len() == 1
    });
    session.close().wait().unwrap();
    wait(|| declarations.reconcile(true));
    drop(leases);
}

#[test]
fn setup_rejects_empty_excess_duplicate_or_unbounded_input_and_closed_transport() {
    let session = session();
    let key = format!("@/joint-{}@/command", uuid::Uuid::new_v4());
    for bound in [0, usize::MAX] {
        let (input, _) = mpsc::sync_channel(1);
        let mut setup = setup(key.clone(), input);
        setup.max_bytes = bound;
        assert!(Declarations::new(&session, setup, "process", "node").is_err());
    }
    for count in [0, 2, 3] {
        let (input, _) = mpsc::sync_channel(1);
        let mut setup = setup(key.clone(), input);
        setup.endpoints.clear();
        for _ in 0..count {
            let (input, _) = mpsc::sync_channel(1);
            setup.endpoints.push(Endpoint {
                key: key.clone(),
                input,
            });
        }
        assert!(Declarations::new(&session, setup, "process", "node").is_err());
    }
    let (input, _) = mpsc::sync_channel(1);
    let mut invalid = setup(key.clone(), input);
    invalid.endpoints[0].key.clear();
    assert!(Declarations::new(&session, invalid, "process", "node").is_err());
    session.close().wait().unwrap();
    let (input, _) = mpsc::sync_channel(1);
    assert!(Declarations::new(&session, setup(key, input), "process", "node").is_err());
}

#[test]
fn sixteen_double_declarations_validate_the_last_endpoint_before_any_installation() {
    let session = session();
    let prefix = format!("@/m62-{}@", uuid::Uuid::new_v4());
    let keys: Vec<_> = (0..16)
        .map(|index| format!("{prefix}/command/{index}"))
        .collect();
    let publishers: Vec<_> = keys
        .iter()
        .map(|key| session.declare_publisher(key.clone()).wait().unwrap())
        .collect();
    for fault in 0..6 {
        let mut endpoints: Vec<_> = keys
            .iter()
            .map(|key| {
                let (input, _) = mpsc::sync_channel(1);
                Endpoint {
                    key: key.clone(),
                    input,
                }
            })
            .collect();
        match fault {
            0 => endpoints[15].key = endpoints[0].key.clone(),
            1 => endpoints[15].key = format!("{prefix}/*"),
            2 => endpoints[15].key = "bare/key".into(),
            3 => endpoints[15].key.clear(),
            4 => endpoints[15].key = format!("{prefix}/#"),
            _ => {
                let (input, _) = mpsc::sync_channel(1);
                endpoints.push(Endpoint {
                    key: format!("{prefix}/seventeenth"),
                    input,
                });
            }
        }
        assert!(
            Declarations::new(
                &session,
                Setup {
                    endpoints,
                    max_bytes: 64,
                    drops: Arc::new(AtomicU64::new(0)),
                    epoch: Arc::new(AtomicU64::new(7)),
                },
                "m62",
                "invalid"
            )
            .is_err()
        );
        assert!(
            publishers
                .iter()
                .all(|p| !p.matching_status().wait().unwrap().matching())
        );
    }
    for (process, node) in [("*", "m62"), ("m62", "*")] {
        let mut receivers = Vec::new();
        let endpoints = keys
            .iter()
            .map(|key| {
                assert!(
                    !zenoh::key_expr::OwnedKeyExpr::try_from(key.as_str())
                        .unwrap()
                        .is_wild()
                );
                let token =
                    crate::world_io::presence(key, process, node, "MS", &Double::full_name())
                        .unwrap();
                assert!(
                    zenoh::key_expr::OwnedKeyExpr::try_from(token)
                        .unwrap()
                        .is_wild()
                );
                let (input, receiver) = mpsc::sync_channel(1);
                receivers.push(receiver);
                Endpoint {
                    key: key.clone(),
                    input,
                }
            })
            .collect();
        let result = Declarations::new(
            &session,
            Setup {
                endpoints,
                max_bytes: 64,
                drops: Arc::new(AtomicU64::new(0)),
                epoch: Arc::new(AtomicU64::new(7)),
            },
            process,
            node,
        );
        assert_eq!(
            result.err().map(|error| error.to_string()).as_deref(),
            Some("joint controller declarations require concrete keys")
        );
        assert!(
            publishers.iter().all(|publisher| !publisher
                .matching_status()
                .wait()
                .unwrap()
                .matching())
        );
        assert!(
            receivers.iter().all(|receiver| matches!(
                receiver.try_recv(),
                Err(mpsc::TryRecvError::Disconnected)
            ))
        );
        assert!(
            session
                .liveliness()
                .get("@gz/**")
                .timeout(Duration::from_secs(1))
                .wait()
                .unwrap()
                .into_iter()
                .next()
                .is_none()
        );
    }
    let mut receivers = Vec::new();
    let endpoints = keys
        .iter()
        .map(|key| {
            let (input, receiver) = mpsc::sync_channel(1);
            receivers.push(receiver);
            Endpoint {
                key: key.clone(),
                input,
            }
        })
        .collect();
    let drops = Arc::new(AtomicU64::new(0));
    let epoch = Arc::new(AtomicU64::new(7));
    let (mut declarations, mut leases) = Declarations::new(
        &session,
        Setup {
            endpoints,
            max_bytes: 64,
            drops: Arc::clone(&drops),
            epoch: Arc::clone(&epoch),
        },
        "m62",
        "fleet",
    )
    .unwrap();
    assert_eq!(leases.len(), 16);
    for (index, lease) in leases.iter().enumerate() {
        assert!(lease.matches_endpoint(&keys[index]));
        assert!(
            leases[..index]
                .iter()
                .all(|prior| !Arc::ptr_eq(prior, lease))
        );
    }
    wait(|| {
        publishers
            .iter()
            .all(|p| p.matching_status().wait().unwrap().matching())
    });
    for (index, publisher) in publishers.iter().enumerate() {
        for _ in 0..2 {
            publisher
                .put(
                    Double {
                        data: index as f64 / 10.0,
                        ..Default::default()
                    }
                    .encode_to_vec(),
                )
                .attachment(Double::full_name())
                .wait()
                .unwrap();
        }
    }
    wait(|| drops.load(Ordering::Relaxed) == 16);
    for (index, receiver) in receivers.iter().enumerate() {
        assert_eq!(
            receiver.recv_timeout(Duration::from_secs(3)).unwrap(),
            (7, Some(index as f64 / 10.0))
        );
        assert!(receiver.try_recv().is_err());
    }
    let retired = leases.pop().unwrap();
    let weak = Arc::downgrade(&retired);
    drop(retired);
    assert!(weak.upgrade().is_none());
    assert!(!declarations.reconcile(false));
    wait(|| !publishers[15].matching_status().wait().unwrap().matching());
    assert!(
        publishers[..15]
            .iter()
            .all(|p| p.matching_status().wait().unwrap().matching())
    );
    epoch.store(8, Ordering::Release);
    publishers[0]
        .put(
            Double {
                data: -0.25,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .attachment(Double::full_name())
        .wait()
        .unwrap();
    assert_eq!(
        receivers[0].recv_timeout(Duration::from_secs(3)).unwrap(),
        (8, Some(-0.25))
    );
    drop(leases);
    wait(|| declarations.reconcile(false));
    session.close().wait().unwrap();
}
