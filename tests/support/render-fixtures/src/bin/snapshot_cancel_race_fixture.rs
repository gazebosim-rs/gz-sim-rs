// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_protocol::{PROTOCOL_VERSION, Reply, Request, read_request, write_reply};
use std::io::Write;
fn main() {
    let mut out = std::io::stdout().lock();
    let mut input = std::io::stdin().lock();
    write_reply(
        &mut out,
        &Reply::Ready {
            version: PROTOCOL_VERSION,
        },
    )
    .unwrap();
    out.flush().unwrap();
    let Some(Request::Snapshot { request_id, .. }) = read_request(&mut input).unwrap() else {
        panic!("snapshot expected");
    };
    write_reply(&mut out, &Reply::SnapshotAccepted { request_id }).unwrap();
    out.flush().unwrap();
    let Some(Request::Capture {
        request_id: capture_id,
    }) = read_request(&mut input).unwrap()
    else {
        panic!("capture expected");
    };
    let Some(Request::Snapshot { request_id, .. }) = read_request(&mut input).unwrap() else {
        panic!("replacement snapshot expected");
    };
    let Some(Request::Cancel {
        request_id: cancel_id,
    }) = read_request(&mut input).unwrap()
    else {
        panic!("cancel expected")
    };
    assert_eq!(cancel_id, capture_id);
    write_reply(&mut out, &Reply::SnapshotAccepted { request_id }).unwrap();
    write_reply(
        &mut out,
        &Reply::Cancelled {
            request_id: capture_id,
        },
    )
    .unwrap();
    write_reply(
        &mut out,
        &Reply::Error {
            kind: gz_render_protocol::FailureKind::UnknownCapture,
            request_id: Some(capture_id),
            message: "capture already cancelled".into(),
        },
    )
    .unwrap();
    out.flush().unwrap();
    std::thread::sleep(std::time::Duration::from_secs(60));
}
