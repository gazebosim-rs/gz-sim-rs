// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    physics::fixture,
    protocol::{Command, Outcome, ProtocolError, Reply, Request},
    transport::{TransportError, WorkerProcess, serve},
};
use std::{
    path::Path,
    time::{Duration, Instant},
};
const RUN: u64 = 0x52a9_7124;

fn request(worker_id: u8, sequence: u64, command: Command) -> Request {
    Request {
        run_id: RUN,
        worker_id,
        sequence,
        command,
    }
}
fn tick(reply: &Reply) -> u64 {
    let Outcome::State(state) = &reply.outcome else {
        panic!("expected a committed state")
    };
    state.snapshot.tick
}

#[test]
fn isolated_workers_deduplicate_real_dropped_reply_and_restart_from_the_seed() {
    let path = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let bodies = fixture();
    let mut left = WorkerProcess::spawn(path, RUN, 0, &bodies[..1]).unwrap();
    let mut right = WorkerProcess::spawn(path, RUN, 1, &bodies[1..]).unwrap();
    assert_ne!(left.pid(), right.pid());
    assert_ne!(left.session_id(), right.session_id());
    assert_ne!(left.endpoint(), right.endpoint());
    assert!(left.endpoint().starts_with("tcp/127.0.0.1:"));
    let starting = request(0, 1, Command::Observe);
    for bad_timeout in [Duration::ZERO, Duration::from_secs(6)] {
        assert!(matches!(
            left.request_with_timeout(&starting, bad_timeout),
            Err(TransportError::Wire(_))
        ));
    }
    let init = left.request(&starting).unwrap();
    assert_eq!(tick(&init), 0);
    let step = request(0, 2, Command::Step { tick: 0 });
    let timeout = Duration::from_millis(300);
    let before = Instant::now();
    let traffic = left.traffic();
    assert!(matches!(
        left.request_dropping_reply(&step, timeout),
        Err(TransportError::Unavailable(_))
    ));
    assert!(before.elapsed() < Duration::from_secs(2));
    assert_eq!(left.traffic().replies, traffic.replies);
    let committed = left.request(&step).unwrap();
    assert_eq!(tick(&committed), 1);
    assert_eq!(left.request(&step).unwrap(), committed);
    // Looking up the wrong owner cannot direct a request to its neighbor.
    assert!(matches!(
        left.request(&request(1, 1, Command::Observe)),
        Err(TransportError::Protocol(_))
    ));
    let right_state = right.request(&request(1, 1, Command::Observe)).unwrap();
    assert_eq!(tick(&right_state), 0);
    // Deliver sequence 4 ahead of 3, then deliver a held-back duplicate of 2
    // after 3. Both deliveries reach the live worker through its TCP session.
    let future = request(0, 4, Command::Observe);
    assert!(matches!(
        left.request(&future),
        Err(TransportError::Protocol(ProtocolError::Sequence {
            expected: 3,
            received: 4
        }))
    ));
    let stepped = left.request(&request(0, 3, Command::Observe)).unwrap();
    assert!(matches!(
        left.request(&step),
        Err(TransportError::Protocol(ProtocolError::Sequence {
            expected: 4,
            received: 2
        }))
    ));
    assert_eq!(tick(&left.request(&future).unwrap()), 1);
    assert_eq!(tick(&stepped), 1);
    let transport = left.traffic();
    assert!(transport.request_bytes > 0 && transport.reply_bytes > 0);
    let killed_pid = left.pid();
    let killed_session = left.session_id().to_owned();
    left.kill().unwrap();
    left.kill().unwrap();
    assert!(matches!(
        left.request(&step),
        Err(TransportError::Unavailable(_))
    ));
    assert_eq!(left.traffic().requests, transport.requests);
    left.restart().unwrap();
    assert_ne!(left.pid(), killed_pid);
    assert_ne!(left.session_id(), killed_session);
    assert_eq!(left.request(&starting).unwrap(), init);
    assert_eq!(left.request(&step).unwrap(), committed);
    assert!(left.traffic().requests > transport.requests);
    left.shutdown().unwrap();
    right.shutdown().unwrap();
    // Explicit shutdowns are idempotent; both children have already been reaped.
    left.shutdown().unwrap();
}

#[test]
fn malformed_and_oversized_seeds_are_refused_before_listening() {
    for input in [b"not JSON".to_vec(), vec![b' '; 256 * 1024 + 1]] {
        let mut ready = vec![];
        let result = serve(RUN, 0, input.as_slice(), &mut ready);
        assert!(matches!(result, Err(TransportError::Wire(_))));
        assert!(ready.is_empty());
    }
}

#[test]
fn invalid_worker_arguments_exit_before_announcing_readiness() {
    use std::process::Command as Process;
    for args in [
        &[][..],
        &["--other", "1", "--worker", "0"],
        &["--run", "bad", "--worker", "0"],
        &["--run", "1", "--worker", "256"],
    ] {
        let output = Process::new(env!("CARGO_BIN_EXE_physics_worker"))
            .args(args)
            .output()
            .unwrap();
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
        assert!(!output.stderr.is_empty());
    }
}

#[test]
fn worker_requires_its_pipe_capability_and_refuses_malformed_authenticated_requests() {
    use serde_json::{Value, json};
    use std::{
        io::{BufRead, BufReader},
        process::{Child, Command as Process, Stdio},
    };
    use zenoh::Wait;
    // Launch as a coordinator would, reading the capability solely from the
    // bootstrap pipe so this test can exercise authenticated invalid requests.
    struct Guard(Child);
    impl Drop for Guard {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
    let mut child = Guard(
        Process::new(env!("CARGO_BIN_EXE_physics_worker"))
            .args(["--run", &RUN.to_string(), "--worker", "0"])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .unwrap(),
    );
    serde_json::to_writer(child.0.stdin.take().unwrap(), &fixture()[..1]).unwrap();
    let mut line = String::new();
    BufReader::new(child.0.stdout.take().unwrap())
        .read_line(&mut line)
        .unwrap();
    let ready: Value = serde_json::from_str(&line).unwrap();
    let token: Vec<u8> = serde_json::from_value(ready["capability"].clone()).unwrap();
    let mut config = zenoh::Config::default();
    for (k, v) in [
        ("mode", r#""client""#.to_string()),
        ("listen/endpoints", "[]".into()),
        ("scouting/multicast/enabled", "false".into()),
        ("scouting/gossip/enabled", "false".into()),
        ("connect/endpoints", json!([ready["endpoint"]]).to_string()),
    ] {
        config.insert_json5(k, &v).unwrap();
    }
    let session = zenoh::open(config).wait().unwrap();
    let key = format!("gz/physics/distributed/v1/{RUN}/worker/0");
    let send = |payload: Vec<u8>, capability: Option<Vec<u8>>| {
        let mut get = session
            .get(key.as_str())
            .payload(payload)
            .timeout(Duration::from_secs(1));
        if let Some(cap) = capability {
            get = get.attachment(cap);
        }
        get.wait()
            .unwrap()
            .recv_timeout(Duration::from_secs(2))
            .unwrap()
            .unwrap()
    };
    for cap in [None, Some(vec![0; 16])] {
        let denied = send(serde_json::to_vec("Shutdown").unwrap(), cap);
        assert_eq!(
            denied
                .result()
                .unwrap_err()
                .payload()
                .try_to_string()
                .unwrap(),
            "unknown worker capability"
        );
    }
    let step = json!({"Request":request(0,1,Command::Step{tick:0})});
    let mut oversized_step = serde_json::to_vec(&step).unwrap();
    oversized_step.resize(256 * 1024 + 1, b' ');
    for payload in [b"unknown envelope".to_vec(), oversized_step] {
        let denied = send(payload, Some(token.clone()));
        assert!(
            denied
                .result()
                .unwrap_err()
                .payload()
                .try_to_string()
                .unwrap()
                .contains("invalid worker message")
        );
    }
    let response = send(serde_json::to_vec(&step).unwrap(), Some(token.clone()));
    let received: Value =
        serde_json::from_slice(&response.result().unwrap().payload().to_bytes()).unwrap();
    assert_eq!(
        tick(&serde_json::from_value(received["Answer"]["Ok"].clone()).unwrap()),
        1
    );
    assert!(
        send(serde_json::to_vec("Shutdown").unwrap(), Some(token))
            .result()
            .is_ok()
    );
    session.close().wait().unwrap();
    assert!(child.0.wait().unwrap().success());
}

#[test]
fn invalid_seed_and_missing_executable_do_not_return_a_live_peer() {
    let path = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    assert!(WorkerProcess::spawn(path, RUN, 2, &[]).is_err());
    assert!(WorkerProcess::spawn(&path.with_extension("missing"), RUN, 0, &[]).is_err());
}

#[test]
fn oversized_bootstrap_is_rejected_before_launch() {
    let bodies = vec![fixture()[0].clone(); 4096];
    let result = WorkerProcess::spawn(Path::new("absent-worker-executable"), RUN, 0, &bodies);
    let error = result.err().expect("oversized seed");
    assert!(error.to_string().contains("256 KiB"));
}

#[test]
fn maximum_contact_group_transfers_through_two_real_worker_processes() {
    use gz_physics_distributed::{physics::MAX_CHECKPOINT_BODIES, protocol::WarmTicket};
    let executable = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let template = fixture()[0].promoted();
    let ids: Vec<_> = (1..=MAX_CHECKPOINT_BODIES as u64).collect();
    let chain: Vec<_> = ids
        .iter()
        .map(|&id| {
            let mut b = template.clone();
            b.id = id;
            b.position = [0.65 * id as f64, 0., 0.];
            b.linear = [8. * (16.5 - id as f64), 0., 0.];
            b
        })
        .collect();
    let mut source = WorkerProcess::spawn(executable, RUN, 0, &chain).unwrap();
    let mut destination = WorkerProcess::spawn(executable, RUN, 1, &[]).unwrap();
    for tick in 0..2 {
        source
            .request(&request(0, tick + 1, Command::Step { tick }))
            .unwrap();
        destination
            .request(&request(1, tick + 1, Command::Step { tick }))
            .unwrap();
    }
    let Outcome::State(before) = source
        .request(&request(0, 3, Command::Observe))
        .unwrap()
        .outcome
    else {
        panic!("expected committed state");
    };
    assert_eq!(before.snapshot.impulses.len(), ids.len() - 1);
    let ticket = WarmTicket {
        run_id: RUN,
        serial: 1,
        source: 0,
        destination: 1,
        tick: 2,
        ids: ids.clone(),
        generations: vec![1; ids.len()],
    };
    let old_source = source.traffic();
    let export = request(0, 4, Command::ExportWarm { ticket });
    let legacy_export = source.request(&export).unwrap();
    let json_export_bytes = source.traffic().reply_bytes - old_source.reply_bytes;
    assert!(source.set_packed_handoffs(true));
    let before_retry = source.traffic();
    let packed_export = source.request(&export).unwrap();
    assert_eq!(
        legacy_export, packed_export,
        "retry identity spans encodings"
    );
    let export_bytes = source.traffic().reply_bytes - before_retry.reply_bytes;
    assert!(export_bytes < json_export_bytes);
    let Outcome::WarmExported(handoff) = packed_export.outcome else {
        panic!("expected warm image");
    };
    let native_bytes = handoff.checkpoint.native_bytes();
    let r = handoff.receipt.clone();
    let old_dest = destination.traffic();
    let prepare = request(1, 3, Command::PrepareWarm { handoff });
    assert!(destination.set_packed_handoffs(true));
    let prepared = destination.request(&prepare).unwrap();
    let prepare_bytes = destination.traffic().request_bytes - old_dest.request_bytes;
    assert_eq!(prepared.outcome, Outcome::WarmPrepared(r.prepared()));
    assert!(destination.traffic().reply_bytes - old_dest.reply_bytes < 2048);
    destination.set_packed_handoffs(false);
    let before_retry = destination.traffic();
    assert_eq!(destination.request(&prepare).unwrap(), prepared);
    let json_prepare_bytes = destination.traffic().request_bytes - before_retry.request_bytes;
    assert!(prepare_bytes < json_prepare_bytes);
    let released = source
        .request(&request(
            0,
            5,
            Command::ReleaseWarm {
                receipt: r.prepared(),
            },
        ))
        .unwrap();
    assert_eq!(released.outcome, Outcome::WarmReleased(r.released()));
    let Outcome::WarmActivated(activated) = destination
        .request(&request(
            1,
            4,
            Command::ActivateWarm {
                receipt: r.released(),
            },
        ))
        .unwrap()
        .outcome
    else {
        panic!("expected activation");
    };
    assert_eq!(activated.snapshot, before.snapshot);
    assert_eq!(
        activated.generations,
        ids.into_iter().map(|id| (id, 1)).collect()
    );
    eprintln!(
        "32-member native {native_bytes} B, JSON / packed export {json_export_bytes} / {export_bytes} B, prepare {json_prepare_bytes} / {prepare_bytes} B"
    );
    source.shutdown().unwrap();
    destination.shutdown().unwrap();
}

#[test]
fn managed_bootstrap_needs_a_bounded_seed_line_and_fences_on_pipe_loss() {
    use std::{
        io::{BufRead, BufReader, Write},
        process::{Command as Process, Stdio},
        thread,
    };
    let launch = || {
        Process::new(env!("CARGO_BIN_EXE_physics_worker"))
            .args(["--run", "1", "--worker", "0", "--managed"])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .unwrap()
    };
    for invalid in [b"[]".to_vec(), vec![b'x'; 256 * 1024 + 2]] {
        let mut child = launch();
        child.stdin.take().unwrap().write_all(&invalid).unwrap();
        let out = child.wait_with_output().unwrap();
        assert!(!out.status.success());
        assert!(out.stdout.is_empty());
    }
    for unexpected in [false, true] {
        let mut child = launch();
        let mut owner = child.stdin.take().unwrap();
        owner.write_all(b"[]\n").unwrap();
        let mut ready = String::new();
        BufReader::new(child.stdout.take().unwrap())
            .read_line(&mut ready)
            .unwrap();
        assert_eq!(
            serde_json::from_str::<serde_json::Value>(&ready).unwrap()["worker_id"],
            0
        );
        assert!(child.try_wait().unwrap().is_none());
        if unexpected {
            owner.write_all(b"x").unwrap();
        }
        drop(owner);
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            if let Some(status) = child.try_wait().unwrap() {
                assert_eq!(status.success(), !unexpected);
                break;
            }
            if Instant::now() > deadline {
                let _ = child.kill();
                let _ = child.wait();
                panic!("managed worker outlived its owner pipe");
            }
            thread::sleep(Duration::from_millis(5));
        }
    }
}

#[test]
fn switching_wire_formats_on_a_real_peer_reproduces_signed_bits_and_pending_receipts() {
    use gz_physics_distributed::physics::{GroundPlane, SimulationStep};
    let path = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let mut boxes = fixture();
    boxes[0] = boxes[0].promoted();
    for (i, b) in boxes.iter_mut().enumerate() {
        b.position = [0.76 * i as f64, 0., 0.38];
        b.linear = [-0., 0., -1.];
        b.friction = 0.42;
    }
    let mut peer = WorkerProcess::spawn_with_ground(
        path,
        RUN,
        0,
        &boxes,
        SimulationStep::Ms5,
        Some(GroundPlane::with_friction(0., 0.72).unwrap()),
    )
    .unwrap();
    let mut json_bytes = 0;
    let mut packed_bytes = 0;
    for n in 1..=80 {
        let command = if n == 1 {
            Command::Observe
        } else {
            Command::StepInField {
                tick: n - 2,
                field: gz_physics_distributed::physics::StepField {
                    gravity: [0., 0., -9.81],
                    wrenches: vec![],
                },
            }
        };
        let r = request(0, n, command);
        peer.set_packed_observations(false);
        let before = peer.traffic();
        let json = peer.request(&r).unwrap();
        let middle = peer.traffic();
        peer.set_packed_observations(true);
        let packed = peer.request(&r).unwrap();
        let after = peer.traffic();
        json_bytes += middle.reply_bytes - before.reply_bytes;
        packed_bytes += after.reply_bytes - middle.reply_bytes;
        assert_eq!(json, packed);
        assert!(matches!(packed.outcome, Outcome::State(_)));
    }
    assert!(packed_bytes < json_bytes / 2);
    eprintln!("80 actual worker frames: packed {packed_bytes} bytes, JSON {json_bytes} bytes");
    peer.shutdown().unwrap();
}

#[test]
fn managed_seed_refusal_reports_native_admission_reason_to_the_launcher() {
    let path = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let mut bodies = fixture();
    bodies[1].position = bodies[0].position;
    let error = WorkerProcess::spawn(path, RUN, 0, &bodies)
        .err()
        .expect("overlapping seed must be rejected");
    let message = error.to_string();
    assert!(
        message.contains("worker rejected initial state:")
            && message.contains("contact prediction bounds"),
        "{message}"
    );
}
