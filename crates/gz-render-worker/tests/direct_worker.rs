// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_protocol::{FailureKind, Reply, Request, Stamp, Visual, read_reply, write_request};
use std::{
    io::{self, Read, Write},
    process::{Child, Command, Stdio},
    sync::mpsc::{self, Receiver},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

struct Worker {
    child: Child,
    replies: Option<Receiver<io::Result<Option<Reply>>>>,
    reader: Option<JoinHandle<()>>,
}
fn worker_command() -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_gz-render-worker"));
    command.arg(std::process::id().to_string());
    if let Some(profile) = std::env::var_os("GZ_RENDER_WORKER_PROFILE") {
        command.env("LLVM_PROFILE_FILE", profile);
    }
    command
}
impl Worker {
    fn start() -> Self {
        Self::start_with_icd(None)
    }
    fn start_with_icd(icd: Option<&str>) -> Self {
        let mut command = worker_command();
        command.stdin(Stdio::piped()).stdout(Stdio::piped());
        if let Some(icd) = icd {
            command.env("VK_ICD_FILENAMES", icd);
        }
        let mut child = command.spawn().unwrap();
        let mut stdout = child.stdout.take().unwrap();
        let (tx, replies) = mpsc::sync_channel(1);
        let reader = thread::spawn(move || {
            loop {
                let result = read_reply(&mut stdout);
                let terminal = !matches!(result, Ok(Some(_)));
                if tx.send(result).is_err() || terminal {
                    break;
                }
            }
        });
        Self {
            child,
            replies: Some(replies),
            reader: Some(reader),
        }
    }
    fn send(&mut self, request: Request) {
        let stdin = self.child.stdin.as_mut().unwrap();
        write_request(stdin, &request).unwrap();
        stdin.flush().unwrap();
    }
    fn next(&self) -> Reply {
        self.replies
            .as_ref()
            .unwrap()
            .recv_timeout(Duration::from_secs(30))
            .expect("worker reply timed out")
            .expect("worker reply failed")
            .expect("worker exited before reply")
    }
    fn finish(&mut self) {
        self.send(Request::Shutdown);
        assert!(self.wait_exit().success());
    }
    fn wait_exit(&mut self) -> std::process::ExitStatus {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            match self.child.try_wait().unwrap() {
                Some(status) => return status,
                None if Instant::now() < deadline => thread::sleep(Duration::from_millis(5)),
                None => panic!("worker shutdown timed out"),
            }
        }
    }
}
impl Drop for Worker {
    fn drop(&mut self) {
        self.replies.take();
        let _ = self.child.kill();
        let _ = self.child.wait();
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
        }
    }
}
#[test]
fn direct_worker_renders_a_stamped_snapshot_and_exits() {
    let mut worker = Worker::start();
    assert!(matches!(worker.next(), Reply::Ready { version: 1 }));
    worker.send(Request::Cancel { request_id: 99 });
    assert!(matches!(
        worker.next(),
        Reply::Error {
            kind: FailureKind::UnknownCapture,
            request_id: Some(99),
            ..
        }
    ));
    worker.send(Request::Capture { request_id: 100 });
    assert!(matches!(
        worker.next(),
        Reply::Error {
            kind: FailureKind::Capture,
            request_id: Some(100),
            ..
        }
    ));
    worker.send(Request::Snapshot {
        request_id: 1,
        stamp: Stamp {
            epoch: 4,
            seconds: 0,
            nanoseconds: 0,
        },
        origin: [0.0; 3],
        visuals: vec![Visual {
            id: 1,
            translation: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
            size: [0.0; 3],
            color: [1.0; 3],
        }],
    });
    assert!(matches!(
        worker.next(),
        Reply::Error {
            kind: FailureKind::InvalidSnapshot,
            request_id: Some(1),
            ..
        }
    ));
    let stamp = Stamp {
        epoch: 4,
        seconds: 5,
        nanoseconds: 6,
    };
    worker.send(Request::Snapshot {
        request_id: 2,
        stamp,
        origin: [1e12; 3],
        visuals: vec![Visual {
            id: 1,
            translation: [1e12; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
            size: [2.0; 3],
            color: [1.0, 0.0, 0.0],
        }],
    });
    assert!(matches!(
        worker.next(),
        Reply::SnapshotAccepted { request_id: 2 }
    ));
    worker.send(Request::Snapshot {
        request_id: 1,
        stamp,
        origin: [0.0; 3],
        visuals: vec![],
    });
    assert!(matches!(
        worker.next(),
        Reply::Error {
            kind: FailureKind::StaleRequest,
            request_id: Some(1),
            ..
        }
    ));
    worker.send(Request::Snapshot {
        request_id: 3,
        stamp: Stamp {
            epoch: 3,
            seconds: 0,
            nanoseconds: 0,
        },
        origin: [0.0; 3],
        visuals: vec![],
    });
    assert!(matches!(
        worker.next(),
        Reply::Error {
            kind: FailureKind::StaleRequest,
            request_id: Some(3),
            ..
        }
    ));
    worker.send(Request::Capture { request_id: 1 });
    match worker.next() {
        Reply::Frame {
            request_id,
            stamp: observed,
            rgba,
        } => {
            assert_eq!(request_id, 1);
            assert_eq!(observed, stamp);
            let offset = (32 * 64 + 32) * 4;
            assert_eq!(&rgba[offset..offset + 4], &[255, 0, 0, 255]);
        }
        other => panic!("unexpected reply: {other:?}"),
    }
    worker.finish();
}

#[test]
fn unusable_vulkan_adapter_returns_a_startup_error() {
    let mut worker = Worker::start_with_icd(Some("/nonexistent/gz-render-icd.json"));
    assert!(matches!(
        worker.next(),
        Reply::Error {
            kind: FailureKind::Startup,
            request_id: None,
            ..
        }
    ));
    assert!(!worker.wait_exit().success());
}

#[cfg(target_os = "linux")]
#[test]
fn input_thread_failure_precedes_renderer_initialization() {
    let mut command = Command::new("unshare");
    command
        .args([
            "-Ur",
            "bash",
            "-c",
            "ulimit -u 1; exec \"$1\" \"$PPID\"",
            "fixture",
            env!("CARGO_BIN_EXE_gz-render-worker"),
        ])
        .env("VK_ICD_FILENAMES", "/nonexistent/gz-render-icd.json");
    if let Some(profile) = std::env::var_os("GZ_RENDER_WORKER_PROFILE") {
        command.env("LLVM_PROFILE_FILE", profile);
    }
    let output = command.output().unwrap();
    assert!(!output.status.success());
    assert!(String::from_utf8_lossy(&output.stderr).contains("input thread startup failed"));
    assert!(matches!(
        read_reply(&mut output.stdout.as_slice()).unwrap(),
        Some(Reply::Error {
            kind: FailureKind::Startup,
            request_id: None,
            ..
        })
    ));
}

#[cfg(target_os = "linux")]
#[test]
fn input_thread_failure_with_full_output_exits_with_io_error() {
    let full = std::fs::OpenOptions::new()
        .write(true)
        .open("/dev/full")
        .unwrap();
    let mut command = Command::new("unshare");
    command
        .args([
            "-Ur",
            "bash",
            "-c",
            "ulimit -u 1; exec \"$1\" \"$PPID\"",
            "fixture",
            env!("CARGO_BIN_EXE_gz-render-worker"),
        ])
        .env("VK_ICD_FILENAMES", "/nonexistent/gz-render-icd.json")
        .stdout(Stdio::from(full));
    if let Some(profile) = std::env::var_os("GZ_RENDER_WORKER_PROFILE") {
        command.env("LLVM_PROFILE_FILE", profile);
    }
    let output = command.output().unwrap();
    assert!(!output.status.success());
    assert!(String::from_utf8_lossy(&output.stderr).contains("No space left on device"));
}

#[cfg(target_os = "linux")]
#[test]
fn renderer_startup_failure_with_full_output_exits_with_io_error() {
    let full = std::fs::OpenOptions::new()
        .write(true)
        .open("/dev/full")
        .unwrap();
    let output = worker_command()
        .env("VK_ICD_FILENAMES", "/nonexistent/gz-render-icd.json")
        .stdout(Stdio::from(full))
        .stderr(Stdio::piped())
        .output()
        .unwrap();
    assert!(!output.status.success());
    assert!(String::from_utf8_lossy(&output.stderr).contains("No space left on device"));
}
#[test]
fn oversized_wire_frame_reports_protocol_failure() {
    let mut worker = Worker::start();
    assert!(matches!(worker.next(), Reply::Ready { version: 1 }));
    let stdin = worker.child.stdin.as_mut().unwrap();
    stdin.write_all(&131_073_u32.to_le_bytes()).unwrap();
    stdin.flush().unwrap();
    assert!(matches!(
        worker.next(),
        Reply::Error {
            kind: FailureKind::Protocol,
            request_id: None,
            ..
        }
    ));
    assert!(!worker.wait_exit().success());
}

#[test]
fn missing_or_invalid_parent_argument_is_rejected() {
    for argument in [None, Some("not-a-pid")] {
        let mut command = Command::new(env!("CARGO_BIN_EXE_gz-render-worker"));
        if let Some(profile) = std::env::var_os("GZ_RENDER_WORKER_PROFILE") {
            command.env("LLVM_PROFILE_FILE", profile);
        }
        if let Some(argument) = argument {
            command.arg(argument);
        }
        let result = command.output().unwrap();
        assert!(!result.status.success());
        assert!(result.stdout.is_empty());
        let stderr = String::from_utf8(result.stderr).unwrap();
        assert!(stderr.contains(if argument.is_none() {
            "parent pid required"
        } else {
            "InvalidDigit"
        }));
    }
}

#[test]
fn eof_exits_cleanly_and_a_wrong_parent_is_rejected() {
    let mut worker = Worker::start();
    assert!(matches!(worker.next(), Reply::Ready { version: 1 }));
    drop(worker.child.stdin.take());
    assert!(worker.wait_exit().success());
    let status = Command::new(env!("CARGO_BIN_EXE_gz-render-worker"))
        .arg("0")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .unwrap();
    assert!(!status.success());
}

#[test]
fn disconnected_output_reports_failure_and_worker_exits() {
    let mut worker = Worker::start();
    assert!(matches!(worker.next(), Reply::Ready { version: 1 }));
    drop(worker.replies.take());
    worker.send(Request::Cancel { request_id: 99 });
    worker.reader.take().unwrap().join().unwrap();
    worker.send(Request::Cancel { request_id: 100 });
    assert!(!worker.wait_exit().success());
}

#[test]
fn disconnected_output_after_frame_terminates_worker_on_next_frame() {
    let mut worker = Worker::start();
    assert!(matches!(worker.next(), Reply::Ready { version: 1 }));
    let stamp = Stamp {
        epoch: 3,
        seconds: 2,
        nanoseconds: 1,
    };
    worker.send(Request::Snapshot {
        request_id: 1,
        stamp,
        origin: [0.0; 3],
        visuals: vec![],
    });
    assert!(matches!(
        worker.next(),
        Reply::SnapshotAccepted { request_id: 1 }
    ));
    worker.send(Request::Capture { request_id: 1 });
    assert!(
        matches!(worker.next(), Reply::Frame { request_id: 1, stamp: observed, .. } if observed == stamp)
    );
    drop(worker.replies.take());
    worker.send(Request::Capture { request_id: 2 });
    let deadline = Instant::now() + Duration::from_secs(30);
    while !worker.reader.as_ref().unwrap().is_finished() {
        assert!(Instant::now() < deadline, "second capture timed out");
        thread::sleep(Duration::from_millis(5));
    }
    worker.reader.take().unwrap().join().unwrap();
    worker.send(Request::Capture { request_id: 3 });
    assert!(!worker.wait_exit().success());
}

#[test]
fn closed_output_terminates_worker_on_deterministic_error_replies() {
    let stamp = Stamp {
        epoch: 1,
        seconds: 0,
        nanoseconds: 0,
    };
    let requests = [
        Request::Cancel { request_id: 9 },
        Request::Capture { request_id: 10 },
        Request::Snapshot {
            request_id: 1,
            stamp,
            origin: [0.0; 3],
            visuals: vec![],
        },
        Request::Snapshot {
            request_id: 1,
            stamp,
            origin: [0.0; 3],
            visuals: vec![Visual {
                id: 1,
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0],
                size: [0.0; 3],
                color: [1.0; 3],
            }],
        },
    ];
    for (index, request) in requests.into_iter().enumerate() {
        let child = worker_command()
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .unwrap();
        let mut worker = Worker {
            child,
            replies: None,
            reader: None,
        };
        let mut output = worker.child.stdout.take().unwrap();
        assert!(matches!(
            read_reply(&mut output).unwrap(),
            Some(Reply::Ready { version: 1 })
        ));
        if index == 2 {
            worker.send(Request::Snapshot {
                request_id: 1,
                stamp,
                origin: [0.0; 3],
                visuals: vec![],
            });
            assert!(matches!(
                read_reply(&mut output).unwrap(),
                Some(Reply::SnapshotAccepted { request_id: 1 })
            ));
        }
        drop(output);
        worker.send(request);
        assert!(!worker.wait_exit().success());
    }
}

#[test]
fn closed_output_terminates_on_protocol_stale_time_and_snapshot_replies() {
    for case in 0..3 {
        let child = worker_command()
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let mut worker = Worker {
            child,
            replies: None,
            reader: None,
        };
        let mut output = worker.child.stdout.take().unwrap();
        assert!(matches!(
            read_reply(&mut output).unwrap(),
            Some(Reply::Ready { version: 1 })
        ));
        if case == 1 {
            worker.send(Request::Snapshot {
                request_id: 1,
                stamp: Stamp {
                    epoch: 2,
                    seconds: 0,
                    nanoseconds: 0,
                },
                origin: [0.0; 3],
                visuals: vec![],
            });
            assert!(matches!(
                read_reply(&mut output).unwrap(),
                Some(Reply::SnapshotAccepted { request_id: 1 })
            ));
        }
        drop(output);
        if case == 0 {
            let stdin = worker.child.stdin.as_mut().unwrap();
            stdin.write_all(&131_073_u32.to_le_bytes()).unwrap();
            stdin.flush().unwrap();
        } else {
            worker.send(Request::Snapshot {
                request_id: if case == 1 { 2 } else { 1 },
                stamp: Stamp {
                    epoch: 1,
                    seconds: 0,
                    nanoseconds: 0,
                },
                origin: [0.0; 3],
                visuals: vec![],
            });
        }
        assert!(!worker.wait_exit().success());
        let mut stderr = String::new();
        worker
            .child
            .stderr
            .take()
            .unwrap()
            .read_to_string(&mut stderr)
            .unwrap();
        assert!(
            stderr.contains("BrokenPipe"),
            "unexpected worker error: {stderr}"
        );
    }
}

#[test]
fn overlapping_capture_and_cancellation_replies_are_coherent() {
    let mut worker = Worker::start();
    assert!(matches!(worker.next(), Reply::Ready { version: 1 }));
    worker.send(Request::Snapshot {
        request_id: 1,
        stamp: Stamp {
            epoch: 1,
            seconds: 2,
            nanoseconds: 3,
        },
        origin: [0.0; 3],
        visuals: vec![],
    });
    assert!(matches!(
        worker.next(),
        Reply::SnapshotAccepted { request_id: 1 }
    ));
    let mut batch = Vec::new();
    for request in [
        Request::Capture { request_id: 1 },
        Request::Capture { request_id: 2 },
        Request::Cancel { request_id: 1 },
    ] {
        write_request(&mut batch, &request).unwrap();
    }
    let stdin = worker.child.stdin.as_mut().unwrap();
    stdin.write_all(&batch).unwrap();
    stdin.flush().unwrap();
    let (
        mut first_frame,
        mut first_cancelled,
        mut second_frame,
        mut second_busy,
        mut accepted,
        mut unknown,
    ) = (false, false, false, false, false, false);
    for _ in 0..3 {
        match worker.next() {
            Reply::Frame {
                request_id: 1,
                stamp,
                ..
            } => {
                assert_eq!(
                    stamp,
                    Stamp {
                        epoch: 1,
                        seconds: 2,
                        nanoseconds: 3
                    }
                );
                assert!(!first_frame);
                first_frame = true;
            }
            Reply::Frame {
                request_id: 2,
                stamp,
                ..
            } => {
                assert_eq!(
                    stamp,
                    Stamp {
                        epoch: 1,
                        seconds: 2,
                        nanoseconds: 3
                    }
                );
                assert!(!second_frame);
                second_frame = true;
            }
            Reply::Busy { request_id: 2 } => {
                assert!(!second_busy);
                second_busy = true;
            }
            Reply::CancelAccepted { request_id: 1 } => {
                assert!(!accepted);
                accepted = true;
            }
            Reply::Cancelled { request_id: 1 } => {
                assert!(!first_cancelled);
                first_cancelled = true;
            }
            Reply::Error {
                kind: FailureKind::UnknownCapture,
                request_id: Some(1),
                ..
            } => {
                assert!(!unknown);
                unknown = true;
            }
            other => panic!("unexpected overlap reply: {other:?}"),
        }
    }
    assert_ne!(first_frame, first_cancelled);
    assert_ne!(second_frame, second_busy);
    assert_ne!(accepted, unknown);
    assert_eq!(accepted, first_cancelled);
    worker.finish();
}

#[test]
fn replacement_snapshot_invalidates_pending_capture_and_removes_visual() {
    let mut worker = Worker::start();
    assert!(matches!(worker.next(), Reply::Ready { version: 1 }));
    let old_stamp = Stamp {
        epoch: 1,
        seconds: 1,
        nanoseconds: 0,
    };
    let new_stamp = Stamp {
        epoch: 2,
        seconds: 0,
        nanoseconds: 0,
    };
    worker.send(Request::Snapshot {
        request_id: 1,
        stamp: old_stamp,
        origin: [0.0; 3],
        visuals: vec![Visual {
            id: 1,
            translation: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
            size: [2.0; 3],
            color: [1.0, 0.0, 0.0],
        }],
    });
    assert!(matches!(
        worker.next(),
        Reply::SnapshotAccepted { request_id: 1 }
    ));
    let mut batch = Vec::new();
    write_request(&mut batch, &Request::Capture { request_id: 1 }).unwrap();
    write_request(
        &mut batch,
        &Request::Snapshot {
            request_id: 2,
            stamp: new_stamp,
            origin: [0.0; 3],
            visuals: vec![],
        },
    )
    .unwrap();
    let stdin = worker.child.stdin.as_mut().unwrap();
    stdin.write_all(&batch).unwrap();
    stdin.flush().unwrap();
    let (mut cancelled, mut old_frame, mut accepted, mut busy) = (false, false, false, false);
    for _ in 0..2 {
        match worker.next() {
            Reply::Cancelled { request_id: 1 } => {
                assert!(!cancelled);
                cancelled = true;
            }
            Reply::Frame {
                request_id: 1,
                stamp,
                ..
            } => {
                assert_eq!(stamp, old_stamp);
                assert!(!old_frame);
                old_frame = true;
            }
            Reply::SnapshotAccepted { request_id: 2 } => {
                assert!(!accepted);
                accepted = true;
            }
            Reply::SnapshotBusy { request_id: 2 } => {
                assert!(!busy);
                busy = true;
            }
            other => panic!("unexpected replacement reply: {other:?}"),
        }
    }
    assert_ne!(cancelled, old_frame);
    assert_ne!(accepted, busy);
    assert!(!cancelled || accepted);
    if busy {
        let deadline = Instant::now() + Duration::from_secs(10);
        for id in 3..=130 {
            worker.send(Request::Snapshot {
                request_id: id,
                stamp: new_stamp,
                origin: [0.0; 3],
                visuals: vec![],
            });
            match worker.next() {
                Reply::SnapshotAccepted { request_id } if request_id == id => {
                    accepted = true;
                    break;
                }
                Reply::SnapshotBusy { request_id }
                    if request_id == id && Instant::now() < deadline =>
                {
                    thread::sleep(Duration::from_millis(5))
                }
                other => panic!("unexpected retry reply: {other:?}"),
            }
        }
    }
    assert!(accepted, "replacement snapshot remained busy");
    worker.send(Request::Capture { request_id: 2 });
    match worker.next() {
        Reply::Frame {
            request_id: 2,
            stamp,
            rgba,
        } => {
            assert_eq!(stamp, new_stamp);
            let offset = (32 * 64 + 32) * 4;
            assert_ne!(&rgba[offset..offset + 4], &[255, 0, 0, 255]);
        }
        other => panic!("unexpected new frame: {other:?}"),
    }
    worker.finish();
}

#[cfg(target_os = "linux")]
#[test]
fn startup_output_failure_exits_without_ready() {
    let full = std::fs::OpenOptions::new()
        .write(true)
        .open("/dev/full")
        .unwrap();
    let child = worker_command()
        .stdin(Stdio::null())
        .stdout(Stdio::from(full))
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    let mut worker = Worker {
        child,
        replies: None,
        reader: None,
    };
    assert!(!worker.wait_exit().success());
}

#[test]
fn burst_of_snapshots_has_correlated_bounded_outcomes() {
    let mut worker = Worker::start();
    assert!(matches!(worker.next(), Reply::Ready { version: 1 }));
    let stamp = Stamp {
        epoch: 9,
        seconds: 1,
        nanoseconds: 0,
    };
    let mut batch = Vec::new();
    for request_id in 1..=32 {
        write_request(
            &mut batch,
            &Request::Snapshot {
                request_id,
                stamp,
                origin: [0.0; 3],
                visuals: vec![],
            },
        )
        .unwrap();
    }
    let stdin = worker.child.stdin.as_mut().unwrap();
    stdin.write_all(&batch).unwrap();
    stdin.flush().unwrap();
    let mut accepted = 0;
    for expected in 1..=32 {
        match worker.next() {
            Reply::SnapshotAccepted { request_id } if request_id == expected => accepted += 1,
            Reply::SnapshotBusy { request_id } if request_id == expected => {}
            other => panic!("unexpected snapshot burst reply: {other:?}"),
        }
    }
    assert!(accepted > 0);
    worker.finish();
}

#[cfg(target_os = "linux")]
#[test]
fn worker_exits_after_parent_death_with_capture_requested() {
    use rustix::{
        event::{PollFd, PollFlags, Timespec, poll},
        process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal},
    };
    if let Some(path) = std::env::var_os("GZ_RENDER_PARENT_DEATH_HANDSHAKE") {
        let child = worker_command()
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .unwrap();
        let mut worker = Worker {
            child,
            replies: None,
            reader: None,
        };
        let mut output = worker.child.stdout.take().unwrap();
        assert!(matches!(
            read_reply(&mut output).unwrap(),
            Some(Reply::Ready { version: 1 })
        ));
        let stdin = worker.child.stdin.as_mut().unwrap();
        write_request(
            stdin,
            &Request::Snapshot {
                request_id: 1,
                stamp: Stamp {
                    epoch: 1,
                    seconds: 0,
                    nanoseconds: 0,
                },
                origin: [0.0; 3],
                visuals: vec![],
            },
        )
        .unwrap();
        stdin.flush().unwrap();
        assert!(matches!(
            read_reply(&mut output).unwrap(),
            Some(Reply::SnapshotAccepted { request_id: 1 })
        ));
        write_request(stdin, &Request::Capture { request_id: 1 }).unwrap();
        stdin.flush().unwrap();
        let path = std::path::PathBuf::from(path);
        let pending = path.with_extension("pending");
        std::fs::write(&pending, worker.child.id().to_string()).unwrap();
        std::fs::rename(pending, path).unwrap();
        loop {
            thread::park();
        }
    }
    let path = std::env::temp_dir().join(format!("gz-render-parent-death-{}", std::process::id()));
    let child = Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "worker_exits_after_parent_death_with_capture_requested",
            "--nocapture",
        ])
        .env("GZ_RENDER_PARENT_DEATH_HANDSHAKE", &path)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    let mut helper = Worker {
        child,
        replies: None,
        reader: None,
    };
    let deadline = Instant::now() + Duration::from_secs(30);
    let pid = loop {
        if let Ok(value) = std::fs::read_to_string(&path) {
            break value.parse::<i32>().unwrap();
        }
        assert!(
            helper.child.try_wait().unwrap().is_none(),
            "parent helper exited before handshake"
        );
        assert!(
            Instant::now() < deadline,
            "parent helper handshake timed out"
        );
        thread::sleep(Duration::from_millis(5));
    };
    std::fs::remove_file(path).unwrap();
    let pidfd = pidfd_open(Pid::from_raw(pid).unwrap(), PidfdFlags::empty()).unwrap();
    helper.child.kill().unwrap();
    helper.child.wait().unwrap();
    let mut fds = [PollFd::new(&pidfd, PollFlags::IN)];
    let timeout = Timespec {
        tv_sec: 10,
        tv_nsec: 0,
    };
    if poll(&mut fds, Some(&timeout)).unwrap() == 0 {
        let _ = pidfd_send_signal(&pidfd, Signal::KILL);
        panic!("worker remained alive after parent death");
    }
    assert!(fds[0].revents().contains(PollFlags::IN));
}
