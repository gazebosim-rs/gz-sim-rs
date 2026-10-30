// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_client::{Client, ClientError, ShutdownOutcome, SubmitError, TrustedWorker};
use gz_render_protocol::{Reply, Request, Stamp, Visual};
use std::{
    path::PathBuf,
    thread,
    time::{Duration, Instant},
};
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
fn real_worker_roundtrip_and_join() {
    let binary = PathBuf::from(env!("CARGO_BIN_EXE_gz-render-worker"));
    let mut client = start(binary).unwrap();
    let prefix = client.diagnostic_thread_prefix();
    let owned_threads = || {
        std::fs::read_dir("/proc/self/task")
            .unwrap()
            .filter_map(Result::ok)
            .filter_map(|entry| std::fs::read_to_string(entry.path().join("comm")).ok())
            .filter(|name| name.starts_with(&prefix))
            .count()
    };
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    assert!(matches!(
        client.try_send(Request::Capture { request_id: 4 }),
        Err(SubmitError::NoSnapshot(Request::Capture { request_id: 4 }))
    ));
    let invalid = Request::Snapshot {
        request_id: 1,
        stamp: Stamp {
            epoch: 3,
            seconds: 0,
            nanoseconds: 1_000_000_000,
        },
        origin: [0.0; 3],
        visuals: vec![],
    };
    assert!(matches!(
        client.try_send(invalid),
        Err(SubmitError::Invalid(Request::Snapshot { .. }))
    ));
    client
        .try_send(Request::Snapshot {
            request_id: 1,
            stamp: Stamp {
                epoch: 3,
                seconds: 4,
                nanoseconds: 5,
            },
            origin: [1e12; 3],
            visuals: vec![Visual {
                id: 1,
                translation: [1e12; 3],
                rotation: [0.0, 0.0, 0.0, 1.0],
                size: [2.0; 3],
                color: [1.0, 0.0, 0.0],
            }],
        })
        .unwrap();
    assert!(matches!(
        wait(&client),
        Reply::SnapshotAccepted { request_id: 1 }
    ));
    let duplicate = Request::Snapshot {
        request_id: 1,
        stamp: Stamp {
            epoch: 3,
            seconds: 4,
            nanoseconds: 5,
        },
        origin: [1e12; 3],
        visuals: vec![],
    };
    assert!(matches!(
        client.try_send(duplicate),
        Err(SubmitError::Stale(Request::Snapshot { request_id: 1, .. }))
    ));
    client.try_send(Request::Capture { request_id: 6 }).unwrap();
    match wait(&client) {
        Reply::Frame {
            request_id,
            stamp,
            rgba,
        } => {
            assert_eq!(
                (request_id, stamp.epoch, stamp.seconds, stamp.nanoseconds),
                (6, 3, 4, 5)
            );
            let offset = (32 * 64 + 32) * 4;
            assert_eq!(&rgba[offset..offset + 4], &[255, 0, 0, 255]);
        }
        reply => panic!("unexpected reply: {reply:?}"),
    }
    assert!(matches!(
        client.try_send(Request::Capture { request_id: 6 }),
        Err(SubmitError::Stale(Request::Capture { request_id: 6 }))
    ));
    client.try_send(Request::Capture { request_id: 7 }).unwrap();
    assert!(matches!(
        wait(&client),
        Reply::Frame {
            request_id: 7,
            stamp: Stamp {
                epoch: 3,
                seconds: 4,
                nanoseconds: 5
            },
            ..
        }
    ));
    assert_eq!(
        client.shutdown(Duration::from_secs(10)).unwrap(),
        ShutdownOutcome::Graceful
    );
    assert_eq!(owned_threads(), 0);
}
#[test]
fn last_handle_drop_shuts_down_worker_and_joins_threads() {
    let binary = PathBuf::from(env!("CARGO_BIN_EXE_gz-render-worker"));
    let client = start(binary).unwrap();
    let prefix = client.diagnostic_thread_prefix();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    drop(client);
    let remaining = std::fs::read_dir("/proc/self/task")
        .unwrap()
        .filter_map(Result::ok)
        .filter_map(|entry| std::fs::read_to_string(entry.path().join("comm")).ok())
        .filter(|name| name.starts_with(&prefix))
        .count();
    assert_eq!(remaining, 0);
}

#[test]
fn killed_real_worker_is_reported_and_reaped() {
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    let binary = PathBuf::from(env!("CARGO_BIN_EXE_gz-render-worker"));
    let mut client = start(binary).unwrap();
    assert!(matches!(wait(&client), Reply::Ready { version: 1 }));
    let pid = Pid::from_raw(client.worker_pid().unwrap() as i32).unwrap();
    let fd = pidfd_open(pid, PidfdFlags::empty()).unwrap();
    pidfd_send_signal(&fd, Signal::KILL).unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match client.try_reply() {
            Err(ClientError::WorkerExited(None)) => break,
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
            other => panic!("unexpected killed-worker result: {other:?}"),
        }
    }
    assert!(matches!(
        client.try_send(Request::Capture { request_id: 88 }),
        Err(SubmitError::Closed(Request::Capture { request_id: 88 }))
    ));
    assert_eq!(
        client.shutdown(Duration::from_secs(5)).unwrap(),
        ShutdownOutcome::Exited(None)
    );
    assert_eq!(client.worker_pid(), None);
}
