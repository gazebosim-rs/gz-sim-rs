// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_protocol::{
    FailureKind, PROTOCOL_VERSION, Reply, Request, read_request, write_reply,
};
use std::io::Write;
fn main() {
    let mut input = std::io::stdin().lock();
    let mut output = std::io::stdout().lock();
    let mut send = |reply| {
        write_reply(&mut output, &reply).unwrap();
        output.flush().unwrap();
    };
    send(Reply::Ready {
        version: PROTOCOL_VERSION,
    });
    for id in 1..=3 {
        assert!(
            matches!(read_request(&mut input).unwrap(), Some(Request::Snapshot { request_id, .. }) if request_id == id)
        );
        let reply = match id {
            1 => Reply::SnapshotBusy { request_id: id },
            2 => Reply::Error {
                kind: FailureKind::InvalidSnapshot,
                request_id: Some(id),
                message: "invalid scene".into(),
            },
            _ => Reply::SnapshotAccepted { request_id: id },
        };
        send(reply);
    }
    for id in 5..=7 {
        assert!(
            matches!(read_request(&mut input).unwrap(), Some(Request::Capture { request_id }) if request_id == id)
        );
        let reply = if id == 5 {
            Reply::Busy { request_id: id }
        } else {
            Reply::Error {
                kind: if id == 6 {
                    FailureKind::Capture
                } else {
                    FailureKind::Readback
                },
                request_id: Some(id),
                message: "capture failed".into(),
            }
        };
        send(reply);
    }
    std::thread::sleep(std::time::Duration::from_secs(60));
}
