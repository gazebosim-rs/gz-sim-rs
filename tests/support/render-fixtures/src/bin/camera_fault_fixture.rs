// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Typed protocol failure peer; it does not render or fabricate image frames.
use gz_render_protocol::{
    FailureKind, PROTOCOL_VERSION, Reply, Request, read_request, write_reply,
};
use std::io::Write;

fn send(output: &mut impl Write, reply: &Reply) {
    write_reply(output, reply).unwrap();
    output.flush().unwrap();
}
fn main() {
    let mut input = std::io::stdin().lock();
    let mut output = std::io::stdout().lock();
    send(
        &mut output,
        &Reply::Ready {
            version: PROTOCOL_VERSION,
        },
    );
    let Some(Request::CalibratedSnapshot {
        request_id, origin, ..
    }) = read_request(&mut input).unwrap()
    else {
        panic!("calibrated snapshot expected");
    };
    let mode = origin[1] as u8;
    let first = match mode {
        1 => Reply::SnapshotBusy { request_id },
        2 => Reply::Error {
            kind: FailureKind::InvalidSnapshot,
            request_id: Some(request_id),
            message: "invalid snapshot".into(),
        },
        _ => Reply::SnapshotAccepted { request_id },
    };
    send(&mut output, &first);
    if mode >= 3 {
        let Some(Request::Capture { request_id }) = read_request(&mut input).unwrap() else {
            panic!("capture expected");
        };
        let reply = match mode {
            3 => Reply::Busy { request_id },
            4 => Reply::Error {
                kind: FailureKind::Device,
                request_id: Some(request_id),
                message: "device failure".into(),
            },
            5 => Reply::Error {
                kind: FailureKind::Shader,
                request_id: None,
                message: "shader failure".into(),
            },
            6 => return,
            7 => Reply::Cancelled { request_id },
            _ => panic!("unknown mode"),
        };
        send(&mut output, &reply);
    }
    // Keep the actual pipe alive until the supervisor retires it.
    let _ = read_request(&mut input);
}
