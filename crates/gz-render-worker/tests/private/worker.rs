// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Default)]
struct ReplyWriter {
    bytes: Vec<u8>,
    fail_write: bool,
    fail_flush: bool,
    flushed: bool,
}
impl Write for ReplyWriter {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        if self.fail_write {
            return Err(io::ErrorKind::BrokenPipe.into());
        }
        self.bytes.extend_from_slice(bytes);
        Ok(bytes.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        self.flushed = true;
        if self.fail_flush {
            return Err(io::ErrorKind::WriteZero.into());
        }
        Ok(())
    }
}

#[test]
fn reply_output_validates_frames_and_propagates_write_and_flush_failures() {
    let mut writer = ReplyWriter::default();
    send(&mut writer, Reply::Busy { request_id: 7 }).unwrap();
    assert!(writer.flushed);
    assert_eq!(
        gz_render_protocol::read_reply(&mut writer.bytes.as_slice()).unwrap(),
        Some(Reply::Busy { request_id: 7 })
    );

    let mut writer = ReplyWriter::default();
    let invalid = Reply::Frame {
        request_id: 1,
        stamp: Stamp {
            epoch: 0,
            seconds: 0,
            nanoseconds: 0,
        },
        rgba: vec![],
    };
    assert_eq!(
        send(&mut writer, invalid).unwrap_err().kind(),
        io::ErrorKind::InvalidData
    );
    assert!(writer.bytes.is_empty());
    assert!(!writer.flushed);

    let mut writer = ReplyWriter {
        fail_write: true,
        ..Default::default()
    };
    assert_eq!(
        send(&mut writer, Reply::Ready { version: 1 })
            .unwrap_err()
            .kind(),
        io::ErrorKind::BrokenPipe
    );
    assert!(!writer.flushed);

    let mut writer = ReplyWriter {
        fail_flush: true,
        ..Default::default()
    };
    assert_eq!(
        send(&mut writer, Reply::Ready { version: 1 })
            .unwrap_err()
            .kind(),
        io::ErrorKind::WriteZero
    );
    assert!(writer.flushed);
    assert_eq!(
        gz_render_protocol::read_reply(&mut writer.bytes.as_slice()).unwrap(),
        Some(Reply::Ready { version: 1 })
    );
}

#[test]
fn error_message_truncates_only_at_utf8_boundaries() {
    let original = format!("{}💥", "a".repeat(1023));
    let Reply::Error {
        kind,
        request_id,
        message,
    } = failure(FailureKind::Device, Some(4), original)
    else {
        unreachable!()
    };
    assert_eq!(kind, FailureKind::Device);
    assert_eq!(request_id, Some(4));
    assert_eq!(message, "a".repeat(1023));
    let exact = "é".repeat(512);
    let Reply::Error { message, .. } = failure(FailureKind::Shader, None, exact.clone()) else {
        unreachable!()
    };
    assert_eq!(message, exact);
}

#[test]
fn capture_slot_admits_one_and_requires_matching_cancel() {
    let mut slot = CaptureSlot::default();
    assert_eq!(slot.active(), None);
    assert!(!slot.cancel(9));
    assert!(slot.begin(1));
    assert_eq!(
        slot.active(),
        Some(Capture {
            request_id: 1,
            cancelled: false
        })
    );
    assert!(!slot.begin(2));
    assert!(!slot.cancel(2));
    assert_eq!(
        slot.active(),
        Some(Capture {
            request_id: 1,
            cancelled: false
        })
    );
    assert!(slot.cancel(1));
    assert!(slot.cancel(1));
    assert_eq!(
        slot.active(),
        Some(Capture {
            request_id: 1,
            cancelled: true
        })
    );
    slot.clear();
    assert_eq!(slot.active(), None);
    assert!(slot.begin(2));
}
#[test]
fn snapshot_replacement_cancels_only_an_existing_capture() {
    let mut slot = CaptureSlot::default();
    slot.snapshot_replaced();
    assert_eq!(slot.active(), None);
    assert!(slot.begin(7));
    slot.snapshot_replaced();
    assert_eq!(
        slot.active(),
        Some(Capture {
            request_id: 7,
            cancelled: true
        })
    );
    assert!(!slot.begin(8));
    slot.clear();
    assert!(slot.begin(8));
    assert_eq!(
        slot.active(),
        Some(Capture {
            request_id: 8,
            cancelled: false
        })
    );
}

#[test]
fn update_failures_keep_their_scope_and_type() {
    for request_id in [None, Some(7)] {
        let Reply::Error {
            kind,
            request_id: observed,
            message,
        } = update_failure(request_id, RenderError::Shader("shader stopped".into()))
        else {
            unreachable!()
        };
        assert_eq!(kind, FailureKind::Shader);
        assert_eq!(observed, request_id);
        assert!(message.contains("shader stopped"));
        let Reply::Error {
            kind,
            request_id: observed,
            message,
        } = update_failure(
            request_id,
            RenderError::RenderStopped("device stopped".into()),
        )
        else {
            unreachable!()
        };
        assert_eq!(kind, FailureKind::Device);
        assert_eq!(observed, request_id);
        assert!(message.contains("device stopped"));
    }
}

#[test]
fn update_transition_preserves_pending_scene_until_advanced() {
    let mut pending = true;
    assert!(apply_update(Ok(UpdateStatus::Waiting), None, &mut pending).is_ok());
    assert!(pending);
    let Err(Reply::Error {
        kind, request_id, ..
    }) = apply_update(
        Err(RenderError::Shader("pipeline".into())),
        Some(12),
        &mut pending,
    )
    else {
        panic!("shader failure must be reported")
    };
    assert_eq!(kind, FailureKind::Shader);
    assert_eq!(request_id, Some(12));
    assert!(pending);
    assert!(apply_update(Ok(UpdateStatus::Advanced), None, &mut pending).is_ok());
    assert!(!pending);
}

#[test]
fn readback_transition_correlates_and_releases_capture() {
    let mut slot = CaptureSlot::default();
    assert!(slot.begin(3));
    assert!(matches!(
        slot.readback(Ok(None), true),
        ReadbackOutcome::Pending
    ));
    assert!(matches!(
        slot.readback(Ok(None), false),
        ReadbackOutcome::Pending
    ));
    assert!(!slot.begin(4));
    let frame = Frame {
        epoch: 7,
        time: Duration::new(4, 5),
        rgba: vec![1, 2, 3, 4],
    };
    let ReadbackOutcome::Complete(Reply::Frame {
        request_id,
        stamp,
        rgba,
    }) = slot.readback(Ok(Some(frame)), false)
    else {
        panic!("frame must complete the active capture")
    };
    assert_eq!(request_id, 3);
    assert_eq!(
        stamp,
        Stamp {
            epoch: 7,
            seconds: 4,
            nanoseconds: 5
        }
    );
    assert_eq!(rgba, [1, 2, 3, 4]);
    assert!(slot.begin(4));
    assert!(slot.cancel(4));
    assert!(matches!(
        slot.readback(Ok(None), true),
        ReadbackOutcome::Pending
    ));
    assert!(matches!(
        slot.readback(Ok(None), false),
        ReadbackOutcome::Complete(Reply::Cancelled { request_id: 4 })
    ));
    assert!(slot.begin(5));
    assert!(slot.cancel(5));
    let ReadbackOutcome::Failed(Reply::Error {
        kind, request_id, ..
    }) = slot.readback(Err(RenderError::ReadbackFailed), false)
    else {
        panic!("readback failure must terminate the worker")
    };
    assert_eq!(kind, FailureKind::Readback);
    assert_eq!(request_id, Some(5));
    assert_eq!(slot.active(), None);
}

#[test]
fn busy_snapshot_reply_preserves_committed_state_and_capture() {
    let previous = Stamp {
        epoch: 3,
        seconds: 4,
        nanoseconds: 5,
    };
    let candidate = Stamp {
        epoch: 3,
        seconds: 4,
        nanoseconds: 6,
    };
    let mut accepted = Some(previous);
    let mut needs_update = true;
    let mut capture = CaptureSlot::default();
    assert!(capture.begin(17));
    let reply = snapshot_reply(
        42,
        candidate,
        Err(SnapshotFailure::from(RenderError::SnapshotBusy)),
        &mut accepted,
        &mut needs_update,
        &mut capture,
    );
    assert!(matches!(reply, Reply::SnapshotBusy { request_id: 42 }));
    assert_eq!(accepted, Some(previous));
    assert!(needs_update);
    assert_eq!(
        capture.active(),
        Some(Capture {
            request_id: 17,
            cancelled: false
        })
    );
}

#[cfg(unix)]
#[test]
fn reply_failure_bypasses_a_controlled_rust_destructor() {
    use std::{
        os::unix::net::UnixStream,
        path::PathBuf,
        process::{Child, Command, Stdio},
    };
    struct StalledDrop(PathBuf);
    impl Drop for StalledDrop {
        fn drop(&mut self) {
            std::fs::write(&self.0, b"rust_cleanup_entered\n").unwrap();
            loop {
                std::thread::park();
            }
        }
    }
    fn legacy_return_error(marker: PathBuf) -> io::Result<()> {
        let _cleanup = StalledDrop(marker);
        let (mut output, peer) = UnixStream::pair()?;
        drop(peer);
        // Model the former ?/return path: its locals are destroyed before the
        // caller receives the IO error. This is not an injected GPU fault.
        send(&mut output, Reply::Ready { version: 2 })
    }
    if let Some(mode) = std::env::var_os("GZ_RENDER_RUST_DROP_CONTROL") {
        let marker = PathBuf::from(std::env::var_os("GZ_RENDER_RUST_DROP_MARKER").unwrap());
        let exit = ExitDeadline::new(getpid()).unwrap();
        if mode == "legacy" {
            let _ = legacy_return_error(marker);
        } else {
            let _cleanup = StalledDrop(marker);
            let (mut output, peer) = UnixStream::pair().unwrap();
            drop(peer);
            send_or_exit(&mut output, Reply::Ready { version: 2 }, &exit);
        }
        panic!("fatal reply unexpectedly returned");
    }
    struct ChildOwner(Child);
    impl Drop for ChildOwner {
        fn drop(&mut self) {
            if matches!(self.0.try_wait(), Ok(None)) {
                let _ = self.0.kill();
            }
            let _ = self.0.wait();
        }
    }
    let directory = tempfile::tempdir().unwrap();
    for mode in ["legacy", "guarded"] {
        let marker = directory.path().join(mode);
        let mut child = ChildOwner(
            Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "tests::reply_failure_bypasses_a_controlled_rust_destructor",
                    "--nocapture",
                ])
                .env("GZ_RENDER_RUST_DROP_CONTROL", mode)
                .env("GZ_RENDER_RUST_DROP_MARKER", &marker)
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .spawn()
                .unwrap(),
        );
        let deadline = std::time::Instant::now() + Duration::from_secs(10);
        if mode == "legacy" {
            while !marker.exists() {
                assert!(child.0.try_wait().unwrap().is_none());
                assert!(
                    std::time::Instant::now() < deadline,
                    "cleanup control did not enter"
                );
                std::thread::sleep(Duration::from_millis(5));
            }
            assert_eq!(std::fs::read(marker).unwrap(), b"rust_cleanup_entered\n");
            assert!(child.0.try_wait().unwrap().is_none());
            child.0.kill().unwrap();
            child.0.wait().unwrap();
        } else {
            let status = loop {
                if let Some(status) = child.0.try_wait().unwrap() {
                    break status;
                }
                assert!(
                    std::time::Instant::now() < deadline,
                    "guarded reply did not exit"
                );
                std::thread::sleep(Duration::from_millis(5));
            };
            assert_eq!(status.code(), Some(1));
            assert!(!marker.exists(), "fatal path entered Rust cleanup");
        }
    }
}

#[test]
fn depth_capture_retains_its_kind_through_busy_replacement_and_retirement() {
    let mut slot = CaptureSlot::default();
    assert!(slot.begin_with_kind(41, CaptureKind::Depth));
    assert_eq!(slot.kind(), CaptureKind::Depth);
    assert!(!slot.begin_with_kind(42, CaptureKind::Rgba));
    assert_eq!(slot.kind(), CaptureKind::Depth);
    assert_eq!(slot.active().unwrap().request_id, 41);
    slot.snapshot_replaced();
    assert!(matches!(
        slot.readback_depth(Ok(None), true),
        ReadbackOutcome::Pending
    ));
    assert_eq!(slot.kind(), CaptureKind::Depth);
    assert!(matches!(
        slot.readback_depth(Ok(None), false),
        ReadbackOutcome::Complete(Reply::Cancelled { request_id: 41 })
    ));
    assert_eq!(slot.active(), None);
    assert_eq!(slot.kind(), CaptureKind::Rgba);
    assert!(slot.begin_with_kind(42, CaptureKind::Rgba));
    let result = slot.readback(
        Ok(Some(Frame {
            epoch: 9,
            time: Duration::from_nanos(12),
            rgba: vec![7; 64 * 64 * 4],
        })),
        false,
    );
    assert!(matches!(
        result,
        ReadbackOutcome::Complete(Reply::Frame {
            request_id: 42,
            stamp: Stamp {
                epoch: 9,
                seconds: 0,
                nanoseconds: 12
            },
            ..
        })
    ));
    assert_eq!(slot.active(), None);
}

#[test]
fn depth_readback_preserves_float_words_stamp_and_error_retirement() {
    let mut slot = CaptureSlot::default();
    assert!(slot.begin_with_kind(51, CaptureKind::Depth));
    let mut metres = vec![1.25; 64 * 64];
    metres[1] = f32::INFINITY;
    let result = slot.readback_depth(
        Ok(Some(DepthFrame {
            epoch: 12,
            time: Duration::new(3, 4),
            metres,
        })),
        false,
    );
    let ReadbackOutcome::Complete(Reply::DepthFrame {
        request_id,
        stamp,
        metres_le,
    }) = result
    else {
        panic!("expected depth completion");
    };
    assert_eq!(request_id, 51);
    assert_eq!(
        stamp,
        Stamp {
            epoch: 12,
            seconds: 3,
            nanoseconds: 4
        }
    );
    assert_eq!(metres_le.len(), 64 * 64 * 4);
    assert_eq!(&metres_le[..4], &1.25f32.to_le_bytes());
    assert_eq!(&metres_le[4..8], &f32::INFINITY.to_le_bytes());
    assert_eq!(slot.active(), None);
    assert!(slot.begin_with_kind(52, CaptureKind::Depth));
    assert!(matches!(
        slot.readback_depth(Err(RenderError::ReadbackFailed), false),
        ReadbackOutcome::Failed(Reply::Error {
            kind: FailureKind::Readback,
            request_id: Some(52),
            ..
        })
    ));
    assert_eq!(slot.active(), None);
    assert_eq!(slot.kind(), CaptureKind::Rgba);
}
