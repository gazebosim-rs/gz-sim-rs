// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_client::{Client, ClientError, ShutdownOutcome, SubmitError, TrustedWorker};
use gz_render_protocol::{Reply, Request, Stamp, Visual};
use std::{
    path::PathBuf,
    thread,
    time::{Duration, Instant},
};
// Pin the exact worker identity and observe it without reaping it.
fn live_worker_fd(client: &Client) -> rustix::fd::OwnedFd {
    use rustix::fd::AsFd;
    use rustix::process::{Pid, PidfdFlags, WaitId, WaitIdOptions, pidfd_open, waitid};
    let pid = Pid::from_raw(client.worker_pid().unwrap() as i32).unwrap();
    let fd = pidfd_open(pid, PidfdFlags::empty()).unwrap();
    let options = WaitIdOptions::EXITED | WaitIdOptions::NOHANG | WaitIdOptions::NOWAIT;
    assert!(matches!(
        waitid(WaitId::PidFd(fd.as_fd()), options),
        Ok(None)
    ));
    fd
}
fn assert_reaped(fd: &rustix::fd::OwnedFd) {
    use rustix::fd::AsFd;
    use rustix::process::{WaitId, WaitIdOptions, waitid};
    let options = WaitIdOptions::EXITED | WaitIdOptions::NOHANG | WaitIdOptions::NOWAIT;
    assert!(matches!(
        waitid(WaitId::PidFd(fd.as_fd()), options),
        Err(rustix::io::Errno::CHILD)
    ));
}
fn start(binary: PathBuf) -> Result<Client, ClientError> {
    Client::start(TrustedWorker::from_operator_config(binary)?)
}
fn wait(client: &Client) -> Reply {
    let deadline = Instant::now() + Duration::from_secs(20);
    while Instant::now() < deadline {
        match client.try_reply() {
            Ok(Some(reply)) => return reply,
            Ok(None) => thread::sleep(Duration::from_millis(5)),
            Err(error) => panic!("worker failed: {error:?}"),
        }
    }
    panic!("worker timed out")
}
#[test]
fn failed_worker_start_is_reported_and_reaped() {
    let mut client = start(PathBuf::from("/missing/render-worker")).unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match client.try_reply() {
            Err(ClientError::Io(error)) => {
                assert_eq!(error.kind(), std::io::ErrorKind::NotFound);
                break;
            }
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
            other => panic!("unexpected startup result: {other:?}"),
        }
    }
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn stalled_worker_is_killed_and_threads_join() {
    let path = PathBuf::from(env!("CARGO_BIN_EXE_stall_fixture"));
    let mut client = start(path.clone()).unwrap();
    let lifetime = client.diagnostic_lifetime();
    assert!(!lifetime.is_retired());
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    let worker = live_worker_fd(&client);
    let make_snapshot = |request_id| Request::Snapshot {
        request_id,
        stamp: Stamp {
            epoch: 1,
            seconds: 0,
            nanoseconds: 0,
        },
        origin: [0.0; 3],
        visuals: (1..=128)
            .map(|id| Visual {
                id,
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0],
                size: [1.0; 3],
                color: [1.0; 3],
            })
            .collect(),
    };
    let mut saw_busy = false;
    for request_id in 1..=1000 {
        match client.try_send(make_snapshot(request_id)) {
            Ok(()) => {}
            Err(SubmitError::Busy(request)) => {
                assert!(matches!(request, Request::Snapshot { .. }));
                saw_busy = true;
                break;
            }
            Err(error) => panic!(
                "unexpected submit result: {error:?}; worker status: {:?}",
                client.try_reply()
            ),
        }
    }
    assert!(saw_busy, "stalled input never exerted backpressure");
    let start = Instant::now();
    assert_eq!(
        client.shutdown(Duration::from_secs(5)).unwrap(),
        ShutdownOutcome::Forced
    );
    assert!(start.elapsed() < Duration::from_secs(5));
    assert!(client.diagnostic_shutdown_complete());
    assert_reaped(&worker);
    drop(client);
    let remaining = usize::from(!lifetime.is_retired());
    assert_eq!(remaining, 0);
    assert_reaped(&worker);
}

fn prepare_cancel_burst(client: &Client) {
    assert!(matches!(wait(client), Reply::Ready { version: 1 }));
    client.try_send(empty_snapshot(1)).unwrap();
    assert!(matches!(
        wait(client),
        Reply::SnapshotAccepted { request_id: 1 }
    ));
    for mut request in [
        Request::Capture { request_id: 5 },
        Request::Cancel { request_id: 5 },
    ] {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            match client.try_send(request) {
                Ok(()) => break,
                Err(SubmitError::Busy(returned)) if Instant::now() < deadline => {
                    request = returned;
                    thread::sleep(Duration::from_millis(5));
                }
                Err(error) => panic!("unexpected submit result: {error:?}"),
            }
        }
    }
}
#[test]
fn full_output_applies_backpressure_without_losing_replies() {
    let path = PathBuf::from(env!("CARGO_BIN_EXE_overflow_fixture"));
    let mut client = start(path.clone()).unwrap();
    prepare_cancel_burst(&client);
    let deadline = Instant::now() + Duration::from_secs(5);
    while !client.is_output_blocked() && Instant::now() < deadline {
        thread::sleep(Duration::from_millis(5));
    }
    assert!(client.is_output_blocked());
    assert!(!client.is_stopping());
    assert!(matches!(
        wait(&client),
        Reply::CancelAccepted { request_id: 5 }
    ));
    assert!(matches!(wait(&client), Reply::Cancelled { request_id: 5 }));
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn unexpected_child_exit_is_reported() {
    let path = PathBuf::from(env!("CARGO_BIN_EXE_exit_fixture"));
    let mut client = start(path.clone()).unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match client.try_reply() {
            Err(ClientError::WorkerExited(_)) => break,
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
            other => panic!("unexpected exit result: {other:?}"),
        }
    }
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn shutdown_unblocks_a_full_output_slot() {
    let path = PathBuf::from(env!("CARGO_BIN_EXE_overflow_fixture"));
    let mut client = start(path).unwrap();
    let lifetime = client.diagnostic_lifetime();
    assert!(!lifetime.is_retired());
    prepare_cancel_burst(&client);
    let worker = live_worker_fd(&client);
    let deadline = Instant::now() + Duration::from_secs(5);
    while !client.is_output_blocked() && Instant::now() < deadline {
        thread::sleep(Duration::from_millis(5));
    }
    assert!(client.is_output_blocked());
    assert_eq!(
        client.shutdown(Duration::from_secs(3)).unwrap(),
        ShutdownOutcome::Forced
    );
    assert!(client.diagnostic_shutdown_complete());
    assert_reaped(&worker);
    drop(client);
    let remaining = usize::from(!lifetime.is_retired());
    assert_eq!(remaining, 0);
    assert_reaped(&worker);
}

#[test]
fn malformed_child_output_is_reported() {
    let binary = PathBuf::from(env!("CARGO_BIN_EXE_bad_frame_fixture"));
    let mut client = start(binary).unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match client.try_reply() {
            Err(ClientError::Io(error)) => {
                assert_eq!(error.kind(), std::io::ErrorKind::InvalidData);
                break;
            }
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
            other => panic!("unexpected malformed output result: {other:?}"),
        }
    }
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn timeout_leaves_handle_available_for_a_later_reap() {
    let binary = PathBuf::from(env!("CARGO_BIN_EXE_stall_fixture"));
    let mut client = start(binary).unwrap();
    assert!(matches!(
        client.shutdown(Duration::MAX),
        Err(ClientError::InvalidTimeout)
    ));
    assert!(matches!(
        client.shutdown(Duration::ZERO),
        Err(ClientError::Timeout)
    ));
    client.shutdown(Duration::from_secs(5)).unwrap();
    assert!(matches!(
        client.try_send(Request::Capture { request_id: 77 }),
        Err(SubmitError::Closed(Request::Capture { request_id: 77 }))
    ));
    assert!(matches!(client.try_reply(), Err(ClientError::Closed)));
}

#[test]
fn supervisor_thread_exhaustion_is_returned() {
    let output = std::process::Command::new("bash")
        .arg("-c")
        .arg("ulimit -u 1; exec \"$1\"")
        .arg("fixture")
        .arg(env!("CARGO_BIN_EXE_spawn_limit_fixture"))
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "fixture failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn closed_worker_input_reports_broken_pipe_and_reaps() {
    let binary = PathBuf::from(env!("CARGO_BIN_EXE_closed_input_fixture"));
    let mut client = start(binary).unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    client.try_send(empty_snapshot(9)).unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match client.try_reply() {
            Err(ClientError::Io(error)) => {
                assert_eq!(error.kind(), std::io::ErrorKind::BrokenPipe);
                break;
            }
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
            other => panic!("unexpected closed input result: {other:?}"),
        }
    }
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn incompatible_worker_version_is_rejected() {
    let binary = PathBuf::from(env!("CARGO_BIN_EXE_bad_version_fixture"));
    let mut client = start(binary).unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match client.try_reply() {
            Err(ClientError::ProtocolVersion) => break,
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
            other => panic!("unexpected version result: {other:?}"),
        }
    }
    assert!(matches!(
        client.try_send(Request::Capture { request_id: 99 }),
        Err(SubmitError::Closed(Request::Capture { request_id: 99 }))
    ));
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn structured_startup_failure_is_delivered_before_ready() {
    let binary = PathBuf::from(env!("CARGO_BIN_EXE_startup_failure_fixture"));
    let mut client = start(binary).unwrap();
    assert!(matches!(
        wait(&client),
        Reply::Error {
            kind: gz_render_protocol::FailureKind::Startup,
            request_id: None,
            ..
        }
    ));
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn requests_before_handshake_are_returned_untouched() {
    let binary = PathBuf::from(env!("CARGO_BIN_EXE_delayed_ready_fixture"));
    let mut client = start(binary).unwrap();
    assert!(matches!(
        client.try_send(Request::Capture { request_id: 123 }),
        Err(SubmitError::NotReady(Request::Capture { request_id: 123 }))
    ));
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    client.shutdown(Duration::from_secs(3)).unwrap();
}

#[test]
fn graceful_shutdown_releases_a_blocked_reader_after_child_exit() {
    let binary = PathBuf::from(env!("CARGO_BIN_EXE_exit_with_replies_fixture"));
    let mut client = start(binary).unwrap();
    prepare_cancel_burst(&client);
    let deadline = Instant::now() + Duration::from_secs(5);
    while !client.is_output_blocked() && Instant::now() < deadline {
        thread::sleep(Duration::from_millis(5));
    }
    assert!(client.is_output_blocked());
    let started = Instant::now();
    let outcome = client.shutdown(Duration::from_secs(5)).unwrap();
    assert_eq!(outcome, ShutdownOutcome::Graceful);
    assert!(started.elapsed() < Duration::from_secs(2));
}

fn empty_snapshot(request_id: u64) -> Request {
    Request::Snapshot {
        request_id,
        stamp: Stamp {
            epoch: 1,
            seconds: 2,
            nanoseconds: 3,
        },
        origin: [0.0; 3],
        visuals: vec![],
    }
}
fn correlation_error(client: &Client) {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match client.try_reply() {
            Err(ClientError::ProtocolCorrelation) => return,
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
            other => panic!("unexpected correlation result: {other:?}"),
        }
    }
}
#[test]
fn duplicate_snapshot_reply_is_rejected() {
    let mut client = start(PathBuf::from(env!(
        "CARGO_BIN_EXE_duplicate_snapshot_fixture"
    )))
    .unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    client.try_send(empty_snapshot(10)).unwrap();
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 10 }
    ));
    correlation_error(&client);
    client.shutdown(Duration::from_secs(5)).unwrap();
}
#[test]
fn mismatched_snapshot_reply_is_rejected() {
    let mut client = start(PathBuf::from(env!(
        "CARGO_BIN_EXE_mismatched_snapshot_fixture"
    )))
    .unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    client.try_send(empty_snapshot(10)).unwrap();
    correlation_error(&client);
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn snapshots_are_distinct_at_equal_time_and_cannot_overlap() {
    let mut client = start(PathBuf::from(env!(
        "CARGO_BIN_EXE_delayed_snapshot_fixture"
    )))
    .unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    client.try_send(empty_snapshot(10)).unwrap();
    assert!(matches!(
        client.try_send(empty_snapshot(10)),
        Err(SubmitError::Stale(Request::Snapshot { request_id: 10, .. }))
    ));
    assert!(matches!(
        client.try_send(empty_snapshot(11)),
        Err(SubmitError::Busy(Request::Snapshot { request_id: 11, .. }))
    ));
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 10 }
    ));
    assert!(matches!(
        client.try_send(empty_snapshot(10)),
        Err(SubmitError::Stale(Request::Snapshot { request_id: 10, .. }))
    ));
    let stale_time = Request::Snapshot {
        request_id: 11,
        stamp: Stamp {
            epoch: 1,
            seconds: 1,
            nanoseconds: 0,
        },
        origin: [0.0; 3],
        visuals: vec![],
    };
    assert!(matches!(
        client.try_send(stale_time),
        Err(SubmitError::Stale(Request::Snapshot { request_id: 11, .. }))
    ));
    client.try_send(empty_snapshot(11)).unwrap();
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 11 }
    ));
    let reset = Request::Snapshot {
        request_id: 12,
        stamp: Stamp {
            epoch: 2,
            seconds: 0,
            nanoseconds: 0,
        },
        origin: [0.0; 3],
        visuals: vec![],
    };
    client.try_send(reset).unwrap();
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 12 }
    ));
    client.shutdown(Duration::from_secs(5)).unwrap();
}

fn send_snapshot_and_capture(client: &Client) {
    client.try_send(empty_snapshot(1)).unwrap();
    assert!(matches!(
        wait(client),
        Reply::SnapshotAccepted { request_id: 1 }
    ));
    client.try_send(Request::Capture { request_id: 5 }).unwrap();
}
#[test]
fn mismatched_frame_id_is_rejected() {
    let mut client = start(PathBuf::from(env!(
        "CARGO_BIN_EXE_mismatched_frame_fixture"
    )))
    .unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    send_snapshot_and_capture(&client);
    correlation_error(&client);
    client.shutdown(Duration::from_secs(5)).unwrap();
}
#[test]
fn mismatched_frame_timestamp_is_rejected() {
    let mut client = start(PathBuf::from(env!("CARGO_BIN_EXE_stale_frame_fixture"))).unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    send_snapshot_and_capture(&client);
    correlation_error(&client);
    client.shutdown(Duration::from_secs(5)).unwrap();
}
#[test]
fn duplicate_frame_is_rejected() {
    let mut client = start(PathBuf::from(env!("CARGO_BIN_EXE_duplicate_frame_fixture"))).unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    send_snapshot_and_capture(&client);
    assert!(matches!(wait(&client), Reply::Frame { request_id: 5, .. }));
    correlation_error(&client);
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn frame_invalidated_by_newer_snapshot_is_rejected_even_at_equal_time() {
    let mut client = start(PathBuf::from(env!(
        "CARGO_BIN_EXE_invalidated_frame_fixture"
    )))
    .unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    client.try_send(empty_snapshot(1)).unwrap();
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 1 }
    ));
    client.try_send(Request::Capture { request_id: 5 }).unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut request = empty_snapshot(2);
    loop {
        match client.try_send(request) {
            Ok(()) => break,
            Err(SubmitError::Busy(returned)) if Instant::now() < deadline => {
                request = returned;
                thread::sleep(Duration::from_millis(5));
            }
            other => panic!("unexpected snapshot result: {other:?}"),
        }
    }
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 2 }
    ));
    correlation_error(&client);
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn duplicate_cancel_ack_is_rejected() {
    let mut client = start(PathBuf::from(env!(
        "CARGO_BIN_EXE_duplicate_cancel_fixture"
    )))
    .unwrap();
    prepare_cancel_burst(&client);
    assert!(matches!(
        wait(&client),
        Reply::CancelAccepted { request_id: 5 }
    ));
    correlation_error(&client);
    client.shutdown(Duration::from_secs(5)).unwrap();
}
#[test]
fn frame_can_complete_before_cancel_reaches_worker() {
    let mut client = start(PathBuf::from(env!(
        "CARGO_BIN_EXE_frame_before_cancel_fixture"
    )))
    .unwrap();
    prepare_cancel_burst(&client);
    assert!(matches!(wait(&client), Reply::Frame { request_id: 5, .. }));
    assert!(matches!(
        wait(&client),
        Reply::Error {
            kind: gz_render_protocol::FailureKind::UnknownCapture,
            request_id: Some(5),
            ..
        }
    ));
    assert!(matches!(
        client.try_send(Request::Cancel { request_id: 5 }),
        Err(SubmitError::Stale(Request::Cancel { request_id: 5 }))
    ));
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn cancel_ack_retains_reservation_until_terminal_reply() {
    let mut client = start(PathBuf::from(env!("CARGO_BIN_EXE_delayed_cancel_fixture"))).unwrap();
    prepare_cancel_burst(&client);
    assert!(matches!(
        wait(&client),
        Reply::CancelAccepted { request_id: 5 }
    ));
    assert!(matches!(
        client.try_send(Request::Cancel { request_id: 5 }),
        Err(SubmitError::Busy(Request::Cancel { request_id: 5 }))
    ));
    assert!(matches!(
        client.try_send(Request::Capture { request_id: 6 }),
        Err(SubmitError::Busy(Request::Capture { request_id: 6 }))
    ));
    assert_eq!(
        client.shutdown(Duration::from_secs(5)).unwrap(),
        ShutdownOutcome::Forced
    );
}

#[test]
fn snapshot_cancellation_can_precede_explicit_cancel() {
    let mut client = start(PathBuf::from(env!(
        "CARGO_BIN_EXE_snapshot_cancel_race_fixture"
    )))
    .unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    send_snapshot_and_capture(&client);
    for mut request in [empty_snapshot(2), Request::Cancel { request_id: 5 }] {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            match client.try_send(request) {
                Ok(()) => break,
                Err(SubmitError::Busy(returned)) if Instant::now() < deadline => {
                    request = returned;
                    thread::sleep(Duration::from_millis(5));
                }
                Err(error) => panic!("unexpected submit result: {error:?}"),
            }
        }
    }
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 2 }
    ));
    assert!(matches!(wait(&client), Reply::Cancelled { request_id: 5 }));
    assert!(matches!(
        wait(&client),
        Reply::Error {
            kind: gz_render_protocol::FailureKind::UnknownCapture,
            request_id: Some(5),
            ..
        }
    ));
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn worker_selection_requires_an_absolute_operator_path() {
    assert!(matches!(
        TrustedWorker::from_operator_config(PathBuf::from("render-worker")),
        Err(ClientError::InvalidWorkerPath)
    ));
    assert!(
        TrustedWorker::from_operator_config(PathBuf::from("/operator-selected/render-worker"))
            .is_ok()
    );
}

#[test]
fn failed_snapshots_and_captures_release_their_reservations() {
    use gz_render_protocol::FailureKind;
    let mut client = start(PathBuf::from(env!(
        "CARGO_BIN_EXE_failure_sequence_fixture"
    )))
    .unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    client.try_send(empty_snapshot(1)).unwrap();
    assert!(matches!(
        wait(&client),
        Reply::SnapshotBusy { request_id: 1 }
    ));
    client.try_send(empty_snapshot(2)).unwrap();
    assert!(matches!(
        wait(&client),
        Reply::Error {
            kind: FailureKind::InvalidSnapshot,
            request_id: Some(2),
            ..
        }
    ));
    client.try_send(empty_snapshot(3)).unwrap();
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 3 }
    ));
    client.try_send(Request::Capture { request_id: 5 }).unwrap();
    assert!(matches!(wait(&client), Reply::Busy { request_id: 5 }));
    client.try_send(Request::Capture { request_id: 6 }).unwrap();
    assert!(matches!(
        wait(&client),
        Reply::Error {
            kind: FailureKind::Capture,
            request_id: Some(6),
            ..
        }
    ));
    client.try_send(Request::Capture { request_id: 7 }).unwrap();
    assert!(matches!(
        wait(&client),
        Reply::Error {
            kind: FailureKind::Readback,
            request_id: Some(7),
            ..
        }
    ));
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn duplicate_ready_handshake_is_rejected() {
    let mut client = start(PathBuf::from(env!("CARGO_BIN_EXE_duplicate_ready_fixture"))).unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match client.try_reply() {
            Err(ClientError::ProtocolVersion) => break,
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
            other => panic!("unexpected handshake result: {other:?}"),
        }
    }
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn isolated_process_and_io_thread_exhaustion_are_reported_and_reaped() {
    for limit in ["2", "3", "4"] {
        let output = std::process::Command::new("unshare")
            .args([
                "-Ur",
                "bash",
                "-c",
                "ulimit -u \"$1\"; exec \"$2\" \"$3\"",
                "fixture",
                limit,
            ])
            .arg(env!("CARGO_BIN_EXE_thread_limit_probe"))
            .arg(env!("CARGO_BIN_EXE_stall_fixture"))
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "limit {limit}: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }
}

fn fatal_fixture(mode: f64) -> Client {
    let client = start(PathBuf::from(env!("CARGO_BIN_EXE_fatal_reply_fixture"))).unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    let Request::Snapshot {
        request_id,
        stamp,
        visuals,
        ..
    } = empty_snapshot(1)
    else {
        unreachable!()
    };
    client
        .try_send(Request::Snapshot {
            request_id,
            stamp,
            origin: [mode, 0.0, 0.0],
            visuals,
        })
        .unwrap();
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 1 }
    ));
    client.try_send(Request::Capture { request_id: 5 }).unwrap();
    client
}
#[test]
fn fatal_device_error_with_wrong_capture_id_is_rejected() {
    let mut client = fatal_fixture(1.0);
    correlation_error(&client);
    client.shutdown(Duration::from_secs(5)).unwrap();
}
#[test]
fn valid_fatal_errors_are_delivered_then_stop_submission() {
    use gz_render_protocol::FailureKind;
    for (mode, kind, request_id) in [
        (2.0, FailureKind::Device, Some(5)),
        (3.0, FailureKind::Shader, None),
        (4.0, FailureKind::Shader, Some(5)),
    ] {
        let mut client = fatal_fixture(mode);
        assert!(
            matches!(wait(&client), Reply::Error { kind: observed, request_id: id, .. } if observed == kind && id == request_id)
        );
        assert!(matches!(
            client.try_send(Request::Capture { request_id: 6 }),
            Err(SubmitError::Closed(Request::Capture { request_id: 6 }))
        ));
        client.shutdown(Duration::from_secs(5)).unwrap();
    }
}
#[test]
fn frame_after_cancel_ack_is_rejected() {
    let mut client = fatal_fixture(5.0);
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut request = Request::Cancel { request_id: 5 };
    loop {
        match client.try_send(request) {
            Ok(()) => break,
            Err(SubmitError::Busy(returned)) if Instant::now() < deadline => {
                request = returned;
                thread::sleep(Duration::from_millis(5));
            }
            Err(error) => panic!("unexpected cancellation: {error:?}"),
        }
    }
    assert!(matches!(
        wait(&client),
        Reply::CancelAccepted { request_id: 5 }
    ));
    correlation_error(&client);
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn fatal_reply_waits_for_a_full_output_slot_without_being_lost() {
    let mut client = fatal_fixture(6.0);
    let deadline = Instant::now() + Duration::from_secs(5);
    while !client.is_output_blocked() && Instant::now() < deadline {
        thread::sleep(Duration::from_millis(5));
    }
    assert!(client.is_output_blocked());
    assert!(client.is_stopping());
    assert!(matches!(wait(&client), Reply::Busy { request_id: 5 }));
    assert!(matches!(
        wait(&client),
        Reply::Error {
            kind: gz_render_protocol::FailureKind::Device,
            request_id: None,
            ..
        }
    ));
    client.shutdown(Duration::from_secs(5)).unwrap();
}
#[test]
fn shutdown_releases_a_fatal_reply_blocked_by_output_backpressure() {
    let mut client = fatal_fixture(6.0);
    let deadline = Instant::now() + Duration::from_secs(5);
    while !client.is_output_blocked() && Instant::now() < deadline {
        thread::sleep(Duration::from_millis(5));
    }
    assert!(client.is_output_blocked());
    client.shutdown(Duration::from_secs(5)).unwrap();
}

#[test]
fn dropping_a_stalled_worker_forces_exit_reaps_and_retires_ownership() {
    let path = PathBuf::from(env!("CARGO_BIN_EXE_stall_fixture"));
    let client = start(path).unwrap();
    let lifetime = client.diagnostic_lifetime();
    assert!(!lifetime.is_retired());
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    let worker = live_worker_fd(&client);
    drop(client);
    let remaining = usize::from(!lifetime.is_retired());
    assert_eq!(remaining, 0);
    assert_reaped(&worker);
}
