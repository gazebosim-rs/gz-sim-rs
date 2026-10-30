// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn client() -> (Client, Receiver<Request>) {
    let (input, rx) = mpsc::sync_channel(1);
    let (_out, output) = mpsc::sync_channel(1);
    let (_fail, failed) = mpsc::sync_channel(1);
    let (_finish, finished) = mpsc::sync_channel(1);
    (
        Client {
            token: 0,
            worker_pid: Arc::new(AtomicU32::new(0)),
            input: Some(input),
            output,
            failed,
            stop: Arc::new(AtomicBool::new(false)),
            output_blocked: Arc::new(AtomicBool::new(false)),
            ready: Arc::new(AtomicBool::new(true)),
            snapshots: Arc::new(Mutex::new(SnapshotCorrelation::default())),
            done: Arc::new(AtomicBool::new(false)),
            shutdown_requested: Arc::new(AtomicBool::new(false)),
            finished,
            supervisor: None,
        },
        rx,
    )
}
fn snapshot() -> Request {
    Request::Snapshot {
        request_id: 1,
        stamp: Stamp {
            epoch: 1,
            seconds: 0,
            nanoseconds: 0,
        },
        origin: [0.0; 3],
        visuals: vec![],
    }
}
fn capture() -> Request {
    Request::Capture { request_id: 1 }
}
fn cancel() -> Request {
    Request::Cancel { request_id: 1 }
}
fn prepare(client: &Client) {
    let mut state = client.snapshots.lock().unwrap();
    let stamp = Stamp {
        epoch: 1,
        seconds: 0,
        nanoseconds: 0,
    };
    state.committed_stamp = Some(stamp);
    state.pending_capture = Some((1, stamp, false));
}
#[test]
fn contended_or_poisoned_state_returns_the_original_request() {
    let (client, _rx) = client();
    let guard = client.snapshots.lock().unwrap();
    for make_request in [snapshot as fn() -> Request, capture, cancel] {
        match client.try_send(make_request()) {
            Err(SubmitError::Busy(returned)) => assert_eq!(returned, make_request()),
            other => panic!("unexpected submission: {other:?}"),
        }
    }
    drop(guard);
    let lock = client.snapshots.clone();
    assert!(
        thread::spawn(move || {
            let _guard = lock.lock().unwrap();
            panic!("poison state");
        })
        .join()
        .is_err()
    );
    for make_request in [snapshot as fn() -> Request, capture, cancel] {
        match client.try_send(make_request()) {
            Err(SubmitError::Closed(returned)) => assert_eq!(returned, make_request()),
            other => panic!("unexpected submission: {other:?}"),
        }
    }
}
#[test]
fn full_and_closed_input_returns_the_original_request() {
    for request in [snapshot(), capture(), cancel(), Request::Shutdown] {
        let (client, rx) = client();
        prepare(&client);
        if matches!(request, Request::Capture { .. }) {
            client.snapshots.lock().unwrap().pending_capture = None;
        }
        client
            .input
            .as_ref()
            .unwrap()
            .try_send(Request::Shutdown)
            .unwrap();
        assert!(matches!(
            client.try_send(request),
            Err(SubmitError::Busy(_))
        ));
        drop(rx);
        let request = match client.snapshots.lock().unwrap().pending_capture {
            Some(_) => cancel(),
            None => capture(),
        };
        assert!(matches!(
            client.try_send(request),
            Err(SubmitError::Closed(_))
        ));
    }
    let (client, rx) = client();
    drop(rx);
    assert!(matches!(
        client.try_send(snapshot()),
        Err(SubmitError::Closed(_))
    ));
    assert!(matches!(
        client.try_send(Request::Shutdown),
        Err(SubmitError::Closed(_))
    ));
}

#[cfg(unix)]
fn executable_peer(script: &str) -> PathBuf {
    use std::io::Write;
    use std::os::unix::fs::PermissionsExt;
    static SERIAL: AtomicU64 = AtomicU64::new(0);
    let path = std::env::temp_dir().join(format!(
        "gz-render-test-peer-{}-{}",
        std::process::id(),
        SERIAL.fetch_add(1, Ordering::Relaxed)
    ));
    let pending = path.with_extension("pending");
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&pending)
        .unwrap();
    file.write_all(script.as_bytes()).unwrap();
    file.sync_all().unwrap();
    drop(file);
    std::fs::set_permissions(&pending, std::fs::Permissions::from_mode(0o700)).unwrap();
    std::fs::rename(&pending, &path).unwrap();
    path
}
#[cfg(unix)]
fn delayed_peer() -> PathBuf {
    executable_peer(
        "#!/bin/bash\nsleep 0.2\nprintf '\\027\\000\\000\\000{\"Ready\":{\"version\":1}}'\nIFS= read -r -n 1 _\nprintf '\\045\\000\\000\\000{\"SnapshotAccepted\":{\"request_id\":1}}'\nexec sleep 60\n",
    )
}
#[cfg(unix)]
fn wait_error(client: &Client, expected: fn(&ClientError) -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match client.try_reply() {
            Err(error) => {
                assert!(expected(&error), "unexpected client error: {error:?}");
                return;
            }
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
            other => panic!("unexpected reply: {other:?}"),
        }
    }
}
#[cfg(unix)]
#[test]
fn reader_reports_a_disconnected_reply_consumer() {
    let path = delayed_peer();
    let mut client =
        Client::start(TrustedWorker::from_operator_config(path.clone()).unwrap()).unwrap();
    let (_sender, replacement) = mpsc::sync_channel(1);
    drop(std::mem::replace(&mut client.output, replacement));
    wait_error(&client, |error| matches!(error, ClientError::Closed));
    client.shutdown(Duration::from_secs(5)).unwrap();
    std::fs::remove_file(path).unwrap();
}
#[cfg(unix)]
#[test]
fn reader_reports_poisoned_correlation_state() {
    let path = delayed_peer();
    let mut client =
        Client::start(TrustedWorker::from_operator_config(path.clone()).unwrap()).unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        match client.try_reply() {
            Ok(Some(Reply::Ready { .. })) => break,
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
            other => panic!("unexpected handshake: {other:?}"),
        }
    }
    let state = client.snapshots.clone();
    assert!(
        thread::spawn(move || {
            let _guard = state.lock().unwrap();
            panic!("poison correlation state");
        })
        .join()
        .is_err()
    );
    client.try_send(Request::Shutdown).unwrap();
    wait_error(&client, |error| {
        matches!(error, ClientError::ProtocolCorrelation)
    });
    client.shutdown(Duration::from_secs(5)).unwrap();
    std::fs::remove_file(path).unwrap();
}

#[test]
fn shutdown_control_is_delivered_when_input_is_available() {
    let (client, rx) = client();
    client.try_send(Request::Shutdown).unwrap();
    assert_eq!(rx.try_recv().unwrap(), Request::Shutdown);
}

#[test]
fn diagnostics_reflect_worker_and_lifecycle_state() {
    let (client, _rx) = client();
    assert_eq!(client.worker_pid(), None);
    assert!(!client.is_stopping());
    assert!(!client.is_output_blocked());
    assert_eq!(client.diagnostic_thread_prefix(), "r0000000000000");
    client.worker_pid.store(123, Ordering::Release);
    client.stop.store(true, Ordering::Release);
    client.output_blocked.store(true, Ordering::Release);
    assert_eq!(client.worker_pid(), Some(123));
    assert!(client.is_stopping());
    assert!(client.is_output_blocked());
}

#[cfg(unix)]
#[test]
fn reader_commits_matching_snapshot_from_a_live_peer() {
    let path = delayed_peer();
    let mut client =
        Client::start(TrustedWorker::from_operator_config(path.clone()).unwrap()).unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut ready = false;
    loop {
        match client.try_reply() {
            Ok(Some(Reply::Ready { .. })) if !ready => {
                ready = true;
                client.try_send(snapshot()).unwrap();
            }
            Ok(Some(Reply::SnapshotAccepted { request_id: 1 })) if ready => break,
            Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
            other => panic!("unexpected peer reply: {other:?}"),
        }
    }
    client.try_send(capture()).unwrap();
    client.shutdown(Duration::from_secs(5)).unwrap();
    std::fs::remove_file(path).unwrap();
}

#[cfg(unix)]
#[test]
fn missing_configured_worker_fails_asynchronously_without_a_pid() {
    let path =
        std::env::temp_dir().join(format!("gz-no-such-render-worker-{}", std::process::id()));
    let mut client = Client::start(TrustedWorker::from_operator_config(path).unwrap()).unwrap();
    wait_error(
        &client,
        |error| matches!(error, ClientError::Io(error) if error.kind() == io::ErrorKind::NotFound),
    );
    assert_eq!(client.worker_pid(), None);
    assert_eq!(
        client.shutdown(Duration::from_secs(5)).unwrap(),
        ShutdownOutcome::Exited(None)
    );
}

#[cfg(target_os = "linux")]
#[test]
fn isolated_io_thread_exhaustion_cleans_up_worker() {
    use std::os::unix::fs::PermissionsExt;
    if std::env::var_os("GZ_RENDER_LIMIT_CHILD").is_some() {
        let path = std::env::temp_dir().join(format!("gz-limit-peer-{}", std::process::id()));
        std::fs::write(&path, "#!/bin/sh\nwhile :; do :; done\n").unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o700)).unwrap();
        let mut client =
            Client::start(TrustedWorker::from_operator_config(path.clone()).unwrap()).unwrap();
        wait_error(
            &client,
            |error| matches!(error, ClientError::Io(error) if error.kind() == io::ErrorKind::WouldBlock),
        );
        assert_eq!(
            client.shutdown(Duration::from_secs(5)).unwrap(),
            ShutdownOutcome::Exited(None)
        );
        assert_eq!(client.worker_pid(), None);
        std::fs::remove_file(path).unwrap();
        return;
    }
    for limit in ["4", "5"] {
        let output = Command::new("unshare")
            .args(["-Ur", "bash", "-c", "ulimit -u \"$1\"; exec \"$2\" --exact tests::isolated_io_thread_exhaustion_cleans_up_worker --test-threads=1", "fixture", limit])
            .arg(std::env::current_exe().unwrap())
            .env("GZ_RENDER_LIMIT_CHILD", "1")
            .output().unwrap();
        assert!(
            output.status.success(),
            "limit {limit}: stdout={} stderr={}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
}

#[cfg(target_os = "linux")]
#[test]
fn monitor_reports_when_its_own_child_was_already_reaped() {
    if std::env::var_os("GZ_RENDER_REAP_CHILD").is_some() {
        use rustix::process::{Pid, WaitOptions, waitpid};
        let mut child = Command::new("/bin/true").spawn().unwrap();
        let pid = Pid::from_raw(child.id() as i32).unwrap();
        assert!(waitpid(Some(pid), WaitOptions::empty()).unwrap().is_some());
        let (tx, rx) = mpsc::sync_channel(1);
        monitor_child(&mut child, &AtomicBool::new(false), &tx);
        assert!(
            matches!(rx.try_recv(), Ok(ClientError::Io(error)) if error.raw_os_error() == Some(rustix::io::Errno::CHILD.raw_os_error()))
        );
        assert_eq!(
            child.wait().unwrap_err().raw_os_error(),
            Some(rustix::io::Errno::CHILD.raw_os_error())
        );
        return;
    }
    let output = Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "tests::monitor_reports_when_its_own_child_was_already_reaped",
        ])
        .env("GZ_RENDER_REAP_CHILD", "1")
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "stdout={} stderr={}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[cfg(unix)]
#[test]
fn live_peer_with_wrong_protocol_version_is_rejected() {
    let path = executable_peer(
        "#!/bin/sh\nprintf '\\027\\000\\000\\000{\"Ready\":{\"version\":2}}'\nexec sleep 60\n",
    );
    let mut client =
        Client::start(TrustedWorker::from_operator_config(path.clone()).unwrap()).unwrap();
    wait_error(&client, |error| {
        matches!(error, ClientError::ProtocolVersion)
    });
    client.shutdown(Duration::from_secs(5)).unwrap();
    std::fs::remove_file(path).unwrap();
}

#[cfg(unix)]
#[test]
fn monitor_honors_stop_without_reaping_the_running_child() {
    let mut child = Command::new("sleep").arg("60").spawn().unwrap();
    let (tx, rx) = mpsc::sync_channel(1);
    monitor_child(&mut child, &AtomicBool::new(true), &tx);
    assert!(child.try_wait().unwrap().is_none());
    assert!(matches!(rx.try_recv(), Err(TryRecvError::Empty)));
    child.kill().unwrap();
    child.wait().unwrap();
}

#[test]
fn rejects_invalid_unready_and_closed_submissions() {
    let (mut client, _rx) = client();
    assert!(matches!(
        client.try_send(Request::Capture { request_id: 0 }),
        Err(SubmitError::Invalid(Request::Capture { request_id: 0 }))
    ));
    client.ready.store(false, Ordering::Release);
    assert!(matches!(
        client.try_send(capture()),
        Err(SubmitError::NotReady(Request::Capture { request_id: 1 }))
    ));
    client.ready.store(true, Ordering::Release);
    client.stop.store(true, Ordering::Release);
    assert!(matches!(
        client.try_send(capture()),
        Err(SubmitError::Closed(Request::Capture { request_id: 1 }))
    ));
    client.stop.store(false, Ordering::Release);
    client.input.take();
    assert!(matches!(
        client.try_send(capture()),
        Err(SubmitError::Closed(Request::Capture { request_id: 1 }))
    ));
}

#[test]
fn stale_and_overlapping_requests_preserve_their_identity() {
    let (client, rx) = client();
    assert!(matches!(
        client.try_send(capture()),
        Err(SubmitError::NoSnapshot(Request::Capture { request_id: 1 }))
    ));
    client.try_send(snapshot()).unwrap();
    assert_eq!(rx.try_recv().unwrap(), snapshot());
    assert!(matches!(
        client.try_send(snapshot()),
        Err(SubmitError::Stale(Request::Snapshot { request_id: 1, .. }))
    ));
    let newer = Request::Snapshot {
        request_id: 2,
        stamp: Stamp {
            epoch: 1,
            seconds: 0,
            nanoseconds: 0,
        },
        origin: [0.0; 3],
        visuals: vec![],
    };
    assert!(matches!(
        client.try_send(newer),
        Err(SubmitError::Busy(Request::Snapshot { request_id: 2, .. }))
    ));
    assert!(matches!(
        client.try_send(capture()),
        Err(SubmitError::Busy(Request::Capture { request_id: 1 }))
    ));
    assert!(matches!(
        client.try_send(cancel()),
        Err(SubmitError::Stale(Request::Cancel { request_id: 1 }))
    ));
    {
        let mut state = client.snapshots.lock().unwrap();
        state.pending_id = None;
        state.last_capture_id = 1;
    }
    assert!(matches!(
        client.try_send(capture()),
        Err(SubmitError::Stale(Request::Capture { request_id: 1 }))
    ));
    prepare(&client);
    client.try_send(cancel()).unwrap();
    assert_eq!(rx.try_recv().unwrap(), cancel());
    assert!(matches!(
        client.try_send(cancel()),
        Err(SubmitError::Busy(Request::Cancel { request_id: 1 }))
    ));
}
