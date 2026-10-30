// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_protocol::{
    FailureKind, PROTOCOL_VERSION, Reply, Request, read_request, write_reply,
};
use std::io::Write;
fn main() {
    let mut input = std::io::stdin().lock();
    let mut output = std::io::stdout().lock();
    write_reply(
        &mut output,
        &Reply::Ready {
            version: PROTOCOL_VERSION,
        },
    )
    .unwrap();
    output.flush().unwrap();
    let Some(Request::Snapshot {
        request_id,
        origin,
        stamp,
        ..
    }) = read_request(&mut input).unwrap()
    else {
        panic!("snapshot expected")
    };
    write_reply(&mut output, &Reply::SnapshotAccepted { request_id }).unwrap();
    output.flush().unwrap();
    let Some(Request::Capture { request_id }) = read_request(&mut input).unwrap() else {
        panic!("capture expected")
    };
    let reply = match origin[0] as u32 {
        1 => Reply::Error {
            kind: FailureKind::Device,
            request_id: Some(request_id + 100),
            message: "mismatched device failure".into(),
        },
        2 => Reply::Error {
            kind: FailureKind::Device,
            request_id: Some(request_id),
            message: "device failure".into(),
        },
        3 => Reply::Error {
            kind: FailureKind::Shader,
            request_id: None,
            message: "global shader failure".into(),
        },
        4 => Reply::Error {
            kind: FailureKind::Shader,
            request_id: Some(request_id),
            message: "shader failure".into(),
        },
        6 => {
            write_reply(&mut output, &Reply::Busy { request_id }).unwrap();
            Reply::Error {
                kind: FailureKind::Device,
                request_id: None,
                message: "global device failure".into(),
            }
        }
        5 => {
            assert!(
                matches!(read_request(&mut input).unwrap(), Some(Request::Cancel { request_id: id }) if id == request_id)
            );
            write_reply(&mut output, &Reply::CancelAccepted { request_id }).unwrap();
            Reply::Frame {
                request_id,
                stamp,
                rgba: vec![0; 64 * 64 * 4],
            }
        }
        _ => panic!("unknown mode"),
    };
    write_reply(&mut output, &reply).unwrap();
    output.flush().unwrap();
    std::thread::sleep(std::time::Duration::from_secs(60));
}
