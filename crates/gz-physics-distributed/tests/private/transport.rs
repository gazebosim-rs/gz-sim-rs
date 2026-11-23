// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn only_concrete_ipv4_loopback_ports_can_route_a_worker_session() {
    assert!(config(Some("tcp/127.0.0.1:12345")).is_ok());
    for ep in [
        "tcp/0.0.0.0:9015",
        "tcp/127.0.0.1:0",
        "tcp/localhost:9015",
        "tcp/127.0.0.1:9015#mode=peer",
        "udp/127.0.0.1:9015",
    ] {
        assert_eq!(
            config(Some(ep)).unwrap_err().kind(),
            io::ErrorKind::InvalidInput
        );
    }
}

#[test]
fn encoding_stops_at_the_exact_wire_boundary() {
    let exact = "z".repeat(MAX_PAYLOAD - 2);
    assert_eq!(encode(exact.as_str()).unwrap().len(), MAX_PAYLOAD);
    let oversize = format!("{exact}z");
    assert!(matches!(
        encode(oversize.as_str()),
        Err(TransportError::Wire(_))
    ));
}

#[test]
fn child_announcement_requires_one_bounded_line_and_exact_endpoint_identity() {
    let mut ready = Ready {
        profile: NativeSolverProfile::Standard32,
        run_id: 71,
        worker_id: 1,
        endpoint: "tcp/127.0.0.1:1234".into(),
        session_id: "abc".into(),
        capability: [1; 16],
    };
    let line = |r: &Ready| {
        let mut bytes = serde_json::to_vec(r).unwrap();
        bytes.push(b'\n');
        bytes
    };
    assert_eq!(Ready::read(line(&ready).as_slice(), 71, 1).unwrap(), ready);
    for bytes in [
        vec![b'a'; 1100],
        serde_json::to_vec(&ready).unwrap(),
        b"{}\n".to_vec(),
    ] {
        assert!(Ready::read(bytes.as_slice(), 71, 1).is_err());
    }
    assert!(Ready::read(line(&ready).as_slice(), 72, 1).is_err());
    assert!(Ready::read(line(&ready).as_slice(), 71, 0).is_err());
    ready.endpoint = "tcp/192.0.2.1:1234".into();
    assert!(Ready::read(line(&ready).as_slice(), 71, 1).is_err());
}

#[test]
fn seed_refusal_is_specific_to_the_launch_and_one_bounded_printable_message() {
    let error = |run_id, worker_id, reason: &str| {
        let record = RejectedSeed {
            run_id,
            worker_id,
            reason: reason.to_string(),
        };
        let mut line = serde_json::to_vec(&record).unwrap();
        line.push(b'\n');
        Ready::read(line.as_slice(), 71, 1).unwrap_err().to_string()
    };
    assert_eq!(
        error(71, 1, "contact prediction"),
        "worker rejected initial state: contact prediction"
    );
    assert!(error(72, 1, "contact prediction").contains("wrong worker identity"));
    assert!(error(71, 0, "contact prediction").contains("wrong worker identity"));
    for reason in ["", "line\nbreak", &"x".repeat(MAX_SEED_REASON + 1)] {
        assert_eq!(error(71, 1, reason), "invalid child rejection reason");
    }
}

const PIPE_CHILD_FLAG: &str = "GZ_DISTRIBUTED_TRANSPORT_PIPE_CHILD";

#[test]
#[ignore = "invoked as a subprocess by the scripted Zenoh peer"]
fn pipe_child() {
    if std::env::var_os(PIPE_CHILD_FLAG).is_some() {
        io::copy(&mut io::stdin().lock(), &mut io::sink()).unwrap();
    }
}

fn pipe_process() -> ChildGuard {
    ChildGuard(
        Command::new(std::env::current_exe().unwrap())
            .args(["--ignored", "--exact", "transport::tests::pipe_child"])
            .env(PIPE_CHILD_FLAG, "1")
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .spawn()
            .unwrap(),
    )
}

enum ReplyAction {
    Answer(Vec<u8>),
    Drop,
    Reject,
    Stall,
}

// A private Zenoh responder injects wire faults that the real worker would never
// emit. A pipe-held instance of this Rust test binary provides process lifecycle.
fn scripted_peer(
    make: impl FnOnce(&Ready) -> Vec<(ReplyAction, bool)>,
) -> (WorkerProcess, thread::JoinHandle<()>) {
    let server = zenoh::open(config(None).unwrap()).wait().unwrap();
    let ready = Ready {
        profile: NativeSolverProfile::Standard32,
        run_id: 71,
        worker_id: 0,
        endpoint: server.info().locators().wait()[0].to_string(),
        session_id: server.zid().to_string(),
        capability: [2; 16],
    };
    let scope = key(ready.run_id, ready.worker_id);
    let queries = server
        .declare_queryable(scope.as_str())
        .with(FifoChannel::new(8))
        .wait()
        .unwrap();
    let session = zenoh::open(config(Some(&ready.endpoint)).unwrap())
        .wait()
        .unwrap();
    let script = make(&ready);
    let mut child = pipe_process();
    let mut keep_alive = child.0.stdin.take();
    let peer = WorkerProcess {
        child,
        session,
        ready,
        key: scope.clone(),
        executable: PathBuf::new(),
        seed: vec![],
        traffic: Traffic::default(),
        unavailable: false,
        native_work_timeout: REQUEST_TIMEOUT,
        packed_observations: true,
        packed_handoffs: false,
        supports_packed_handoffs: false,
        fault_after: None,
    };
    let task = thread::spawn(move || {
        for (reply, exit) in script {
            let query = queries
                .recv_timeout(Duration::from_secs(3))
                .unwrap()
                .expect("expected a client RPC");
            match reply {
                ReplyAction::Answer(bytes) => query.reply(scope.as_str(), bytes).wait().unwrap(),
                ReplyAction::Drop => {}
                ReplyAction::Reject => query.reply_err("explicit RPC rejection").wait().unwrap(),
                ReplyAction::Stall => thread::sleep(Duration::from_millis(120)),
            }
            if exit {
                drop(keep_alive.take());
            }
        }
        // A stalled worker stays alive even after acknowledging Shutdown.
        if keep_alive.is_some() {
            thread::sleep(Duration::from_millis(1300));
        }
        drop(queries);
        server.close().wait().unwrap();
    });
    (peer, task)
}

#[test]
fn client_checks_readiness_session_request_sequence_and_payload_before_admission() {
    let (mut peer, task) = scripted_peer(|ready| {
        let mut stale = ready.clone();
        stale.session_id.push('1');
        let r = crate::protocol::Reply {
            run_id: 71,
            worker_id: 0,
            sequence: 9,
            outcome: crate::protocol::Outcome::State(crate::protocol::Observation {
                components: Vec::new(),
                snapshot: Snapshot::default(),
                generations: Default::default(),
            }),
        };
        vec![
            (ReplyAction::Drop, false), // Queryable registered but a readiness publication was lost.
            (
                ReplyAction::Answer(encode(&Response::Ready(ready.clone())).unwrap()),
                false,
            ),
            (
                ReplyAction::Answer(encode(&Response::Ready(stale)).unwrap()),
                false,
            ),
            (ReplyAction::Answer(b"broken reply".to_vec()), false),
            (ReplyAction::Answer(vec![b'x'; MAX_PAYLOAD + 1]), false),
            (
                ReplyAction::Answer(encode(&Response::Answer(Ok(r))).unwrap()),
                false,
            ),
            (
                ReplyAction::Answer(encode(&Response::Ready(ready.clone())).unwrap()),
                true,
            ), // Incorrect shutdown ACK.
        ]
    });
    peer.confirm_ready().unwrap();
    assert!(
        peer.confirm_ready()
            .unwrap_err()
            .to_string()
            .contains("identity")
    );
    assert!(peer.confirm_ready().is_err()); // Malformed JSON.
    let req = Request {
        run_id: 71,
        worker_id: 0,
        sequence: 1,
        command: crate::protocol::Command::Observe,
    };
    assert!(matches!(peer.request(&req), Err(TransportError::Wire(_))));
    assert!(matches!(peer.request(&req), Err(TransportError::Wire(_))));
    assert!(
        peer.shutdown()
            .unwrap_err()
            .to_string()
            .contains("unexpected shutdown")
    );
    task.join().unwrap();
}

#[test]
fn missing_shutdown_ack_and_unresponsive_process_are_reported_and_reaped() {
    let (mut peer, task) = scripted_peer(|_| vec![(ReplyAction::Drop, true)]);
    assert!(peer.shutdown().is_err());
    assert!(peer.child.0.try_wait().unwrap().is_some());
    task.join().unwrap();
    let (mut stalled, task) = scripted_peer(|_| {
        vec![(
            ReplyAction::Answer(encode(&Response::Stopped).unwrap()),
            false,
        )]
    });
    assert!(
        stalled
            .shutdown()
            .unwrap_err()
            .to_string()
            .contains("timed out")
    );
    assert!(
        stalled
            .shutdown()
            .unwrap_err()
            .to_string()
            .contains("worker exited")
    );
    assert!(stalled.child.0.try_wait().unwrap().is_some());
    task.join().unwrap();
}

struct BrokenRead;
impl Read for BrokenRead {
    fn read(&mut self, _buf: &mut [u8]) -> io::Result<usize> {
        Err(io::ErrorKind::BrokenPipe.into())
    }
}
#[derive(Clone, Copy, Debug)]
enum BrokenWrite {
    Json,
    Line,
    Flush,
}
impl Write for BrokenWrite {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if matches!(self, Self::Json) || (matches!(self, Self::Line) && buf == b"\n") {
            return Err(io::ErrorKind::BrokenPipe.into());
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        if matches!(self, Self::Flush) {
            Err(io::ErrorKind::BrokenPipe.into())
        } else {
            Ok(())
        }
    }
}

#[test]
fn broken_seed_or_readiness_pipe_aborts_without_reporting_an_address() {
    assert_eq!(
        Ready::read(BrokenRead, 71, 0).unwrap_err().kind(),
        io::ErrorKind::BrokenPipe
    );
    assert!(matches!(
        serve(71, 0, BrokenRead, io::sink()),
        Err(TransportError::Io(_))
    ));
    let mut child = pipe_process();
    assert_eq!(
        bootstrap(
            &mut child,
            BrokenWrite::Json,
            io::empty(),
            b"[]".to_vec(),
            (71, 0),
            NativeSolverProfile::Standard32,
            Duration::from_secs(1)
        )
        .unwrap_err()
        .kind(),
        io::ErrorKind::BrokenPipe
    );
    for out in [BrokenWrite::Json, BrokenWrite::Line, BrokenWrite::Flush] {
        assert!(serve(71, 0, b"[]".as_slice(), out).is_err());
    }
}

#[test]
fn slow_announcement_fences_and_reaps_the_real_child_at_deadline() {
    struct SlowRead;
    impl Read for SlowRead {
        fn read(&mut self, _buf: &mut [u8]) -> io::Result<usize> {
            thread::sleep(Duration::from_millis(60));
            Ok(0)
        }
    }
    let mut child = pipe_process();
    let result = bootstrap(
        &mut child,
        io::sink(),
        SlowRead,
        vec![],
        (71, 0),
        NativeSolverProfile::Standard32,
        Duration::from_millis(10),
    );
    assert!(result.is_err());
    assert!(child.0.try_wait().unwrap().is_some());
}

#[test]
fn explicit_zenoh_rejection_and_timed_out_live_peer_cannot_publish_a_reply() {
    let (mut peer, task) = scripted_peer(|_| {
        vec![
            (ReplyAction::Reject, false),
            (ReplyAction::Stall, false),
            (
                ReplyAction::Answer(encode(&Response::Stopped).unwrap()),
                true,
            ),
        ]
    });
    let req = Request {
        run_id: 71,
        worker_id: 0,
        sequence: 1,
        command: crate::protocol::Command::Observe,
    };
    let err = peer.request(&req).unwrap_err();
    assert!(matches!(err, TransportError::Unavailable(_)));
    assert!(err.to_string().contains("explicit RPC rejection"));
    assert!(matches!(
        peer.request_with_timeout(&req, Duration::from_millis(20)),
        Err(TransportError::Unavailable(_))
    ));
    assert_eq!(peer.traffic().replies, 0);
    peer.shutdown().unwrap();
    task.join().unwrap();
}

#[test]
fn packed_observations_are_self_contained_across_steps_materials_and_native_activation() {
    use crate::physics::{ComponentInfo, ContactImpulse, GroundSupport, Tier, fixture};
    use crate::protocol::{Observation, Outcome};
    let mut bodies = fixture();
    bodies[0].tier = Tier::Fixed;
    let other = Body {
        id: u64::MAX,
        primitive: crate::physics::Primitive::Sphere,
        size: [0.7; 3],
        linear: [-0., 1.25, -3e-13],
        angular: [0., 0.1, -0.],
        friction: 0.65,
        ..bodies[1].clone()
    };
    bodies.push(other);
    bodies[0].mass = 0.;
    bodies[0].friction = -0.;
    bodies[1].tier = Tier::Prescribed;
    for step in [
        SimulationStep::Ms1,
        SimulationStep::Ms2,
        SimulationStep::Ms5,
        SimulationStep::Ms10,
    ] {
        for ground in [
            None,
            Some(GroundPlane::new(-0.).unwrap()),
            Some(GroundPlane::with_friction(-0.5, 0.72).unwrap()),
            Some(GroundPlane::oriented([0.25, -0.75, 0.5], [0.6, 0., 0.8], 0.72).unwrap()),
        ] {
            let o = Observation {
                snapshot: Snapshot {
                    tick: 123456,
                    step,
                    ground,
                    bodies: bodies.clone(),
                    contacts: 1,
                    contact_history: true,
                    impulses: vec![ContactImpulse {
                        a: 1,
                        b: 2,
                        normal_impulse: 0.000072,
                    }],
                    supports: ground
                        .iter()
                        .map(|_| GroundSupport {
                            body: 2,
                            normal_impulse: -0.,
                        })
                        .collect(),
                },
                components: vec![
                    ComponentInfo {
                        ids: vec![1, 2],
                        contact_history: true,
                    },
                    ComponentInfo {
                        ids: vec![u64::MAX],
                        contact_history: false,
                    },
                ],
                generations: [(1, 0), (2, 4), (u64::MAX, u64::MAX)].into(),
            };
            for outcome in [
                Outcome::State(o.clone()),
                Outcome::Activated(o.clone()),
                Outcome::WarmActivated(o),
            ] {
                let r = Reply {
                    run_id: 987,
                    worker_id: 1,
                    sequence: 8,
                    outcome,
                };
                let payload = observation::encode(&r).unwrap().unwrap();
                assert_eq!(observation::decode(&payload).unwrap(), r);
                for i in 0..payload.len() {
                    assert!(observation::decode(&payload[..i]).is_err());
                }
                let mut trailer = payload.clone();
                trailer.push(0);
                assert!(observation::decode(&trailer).is_err());
                assert!(
                    encode_answer(
                        Ok(r.clone()),
                        NativeSolverProfile::Standard32,
                        true,
                        false,
                        payload.len() - 1
                    )
                    .is_err()
                );
                assert_eq!(
                    encode_answer(
                        Ok(r),
                        NativeSolverProfile::Standard32,
                        true,
                        false,
                        payload.len()
                    )
                    .unwrap(),
                    payload
                );
            }
        }
    }
}

#[test]
fn packed_ground_discriminator_refuses_unknown_and_noncanonical_oblique_planes() {
    use crate::protocol::{Observation, Outcome};
    let payload = |ground| {
        observation::encode(&Reply {
            run_id: 1,
            worker_id: 0,
            sequence: 1,
            outcome: Outcome::State(Observation {
                snapshot: Snapshot {
                    ground,
                    ..Default::default()
                },
                components: vec![],
                generations: Default::default(),
            }),
        })
        .unwrap()
        .unwrap()
    };
    let plain = payload(None);
    let inclined = payload(Some(
        GroundPlane::oriented([0., 0., 0.], [0.6, 0., 0.8], 0.).unwrap(),
    ));
    let offset = plain
        .iter()
        .zip(&inclined)
        .position(|(a, b)| a != b)
        .unwrap();
    assert_eq!(inclined[offset], 2);
    let mut wrong = inclined.clone();
    wrong[offset] = 7;
    assert!(observation::decode(&wrong).is_err());
    for normal in [[0., 0., 1.], [1., 1., 1.], [f64::NAN, 0., 1.]] {
        let mut wire = inclined.clone();
        for (i, v) in normal.into_iter().enumerate() {
            let start = offset + 1 + 8 * (3 + i);
            wire[start..start + 8].copy_from_slice(&v.to_le_bytes());
        }
        assert!(observation::decode(&wire).is_err());
    }
}

#[test]
fn packed_observations_require_the_negotiated_profile_even_if_the_tag_is_forged() {
    use crate::physics::{ComponentInfo, fixture};
    use crate::protocol::{Observation, Outcome};
    let members = (0..64).collect::<Vec<_>>();
    let bodies = members
        .iter()
        .map(|&id| Body {
            id,
            ..fixture()[1].clone()
        })
        .collect();
    let reply = Reply {
        run_id: 1,
        worker_id: 0,
        sequence: 1,
        outcome: Outcome::State(Observation {
            generations: members.iter().map(|&id| (id, 0)).collect(),
            components: vec![ComponentInfo {
                ids: members,
                contact_history: false,
            }],
            snapshot: Snapshot {
                bodies,
                ..Default::default()
            },
        }),
    };
    use crate::physics::NativeFrictionModel::{Grouped, PerPoint};
    for (model, small_tag, large_tag) in [(Grouped, 1, 2), (PerPoint, 3, 4)] {
        let old = NativeSolverProfile::Standard32.with_friction(model);
        let wide = NativeSolverProfile::Extended64.with_friction(model);
        assert!(observation::encode_with_profile(&reply, old).is_err());
        let mut bytes = observation::encode_with_profile(&reply, wide)
            .unwrap()
            .unwrap();
        assert_eq!(&bytes[..4], b"GZOB");
        assert_eq!(bytes[4], large_tag);
        assert_eq!(
            observation::decode_with_profile(&bytes, wide).unwrap(),
            reply
        );
        assert!(observation::decode_with_profile(&bytes, old).is_err());
        bytes[4] = small_tag;
        assert!(observation::decode_with_profile(&bytes, old).is_err());
        assert!(observation::decode_with_profile(&bytes, wide).is_err());
    }
}

#[test]
fn an_old_or_wrong_startup_offer_never_receives_an_extended_seed() {
    // A write would return BrokenPipe, distinguishable from the capacity refusal.
    for output in [b"".as_slice(), b"GZNATIVE64/2\n", b"old-ready\n"] {
        let mut child = pipe_process();
        let error = bootstrap(
            &mut child,
            BrokenWrite::Json,
            output,
            b"[extended data]".to_vec(),
            (71, 0),
            NativeSolverProfile::Extended64,
            Duration::from_secs(1),
        )
        .unwrap_err();
        assert!(error.to_string().contains("does not offer"), "{error}");
    }
}

#[test]
fn point_profiles_bind_capacity_and_friction_before_any_cold_or_image_bytes() {
    use crate::physics::NativeFrictionModel;
    for capacity in [
        NativeSolverProfile::Standard32,
        NativeSolverProfile::Extended64,
    ] {
        let selected = capacity.with_friction(NativeFrictionModel::PerPoint);
        for image in [false, true] {
            let flag = wire_profile(selected).mode(image);
            assert_eq!(managed_worker_profile(flag), Some((selected, image)));
            for offer in [
                b"".as_slice(),
                b"old-ready\n",
                EXTENDED_OFFER,
                if capacity == NativeSolverProfile::Standard32 {
                    b"GZNATIVE64/POINT/1\n"
                } else {
                    b"GZNATIVE32/POINT/1\n"
                },
            ] {
                let mut child = pipe_process();
                let error = bootstrap(
                    &mut child,
                    BrokenWrite::Json,
                    offer,
                    if image {
                        b"{native image}".to_vec()
                    } else {
                        b"[cold seed]".to_vec()
                    },
                    (71, 0),
                    selected,
                    Duration::from_secs(1),
                )
                .unwrap_err();
                assert!(error.to_string().contains("does not offer"), "{error}");
            }
        }
        // A child must repeat the selected profile in its post-seed readiness.
        // An offer alone cannot authorize a connected session of another model.
        let mut bytes = wire_profile(selected).offer.unwrap().to_vec();
        serde_json::to_writer(
            &mut bytes,
            &Ready {
                profile: capacity,
                run_id: 71,
                worker_id: 0,
                endpoint: "tcp/127.0.0.1:3456".into(),
                session_id: "test".into(),
                capability: [3; 16],
            },
        )
        .unwrap();
        bytes.push(b'\n');
        let mut child = pipe_process();
        let error = bootstrap(
            &mut child,
            Vec::<u8>::new(),
            std::io::Cursor::new(bytes),
            b"[]".to_vec(),
            (71, 0),
            selected,
            Duration::from_secs(1),
        )
        .unwrap_err();
        assert!(error.to_string().contains("did not acknowledge"), "{error}");
    }
}

#[test]
fn explicit_frictions_are_bound_to_cold_image_and_observation_wires() {
    use crate::physics::{NativeFrictionModel, fixture};
    let standard = NativeSolverProfile::Standard32;
    let extended = NativeSolverProfile::Extended64;
    let profiles = [
        standard,
        extended,
        standard.with_friction(NativeFrictionModel::PerPoint),
        extended.with_friction(NativeFrictionModel::PerPoint),
    ];
    for profile in profiles {
        let mut layouts = vec![NativeSeedLayout::from(profile)];
        if profile.capacity_profile() == extended {
            layouts.push(NativeSeedLayout::Canonical64.with_friction(profile.friction()));
        }
        for layout in layouts {
            let bodies = vec![fixture()[0].promoted()];
            let ground: Option<GroundPlane> = None;
            let seed = if profile == standard {
                encode(&(3, SimulationStep::Ms10, ground, &bodies))
            } else {
                encode(&(layout, 3, SimulationStep::Ms10, ground, &bodies))
            }
            .unwrap();
            let mut worker = worker_from_seed(71, 0, &seed, profile).unwrap();
            let reply = worker
                .handle(Request {
                    run_id: 71,
                    worker_id: 0,
                    sequence: 1,
                    command: WorkerCommand::Observe,
                })
                .unwrap();
            let image = worker.recovery_image(1, 0).unwrap();
            let bytes = WorkerProcess::encode_pool_image(3, &image).unwrap();
            if profile != standard {
                let err = WorkerProcess::spawn_from_image(
                    std::path::Path::new("unstarted-child"),
                    &image,
                )
                .err()
                .unwrap();
                assert!(err.to_string().contains("two-worker entry"), "{err}");
            }
            let packed = observation::encode_with_profile(&reply, profile)
                .unwrap()
                .unwrap();
            assert_eq!(
                observation::decode_with_profile(&packed, profile).unwrap(),
                reply
            );
            assert_eq!(
                worker_from_image_seed(71, 0, &bytes, profile)
                    .unwrap()
                    .observation(),
                worker.observation()
            );
            for other in profiles.into_iter().filter(|&p| p != profile) {
                assert!(worker_from_seed(71, 0, &seed, other).is_err());
                assert!(worker_from_image_seed(71, 0, &bytes, other).is_err());
                assert!(
                    WorkerProcess::image_answer(
                        Response::Saved(Ok(Box::new(image.clone()))),
                        other
                    )
                    .is_err()
                );
                assert!(observation::decode_with_profile(&packed, other).is_err());
            }
        }
    }
}
