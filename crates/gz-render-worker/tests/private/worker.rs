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
