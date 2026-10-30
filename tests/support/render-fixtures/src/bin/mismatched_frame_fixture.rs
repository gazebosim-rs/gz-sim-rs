// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_protocol::{PROTOCOL_VERSION, Reply, Request, read_request, write_reply};
use std::io::Write;
fn main() {
    let mut out = std::io::stdout().lock();
    write_reply(
        &mut out,
        &Reply::Ready {
            version: PROTOCOL_VERSION,
        },
    )
    .unwrap();
    out.flush().unwrap();
    let mut input = std::io::stdin().lock();
    let Some(Request::Snapshot {
        request_id, stamp, ..
    }) = read_request(&mut input).unwrap()
    else {
        panic!("snapshot expected");
    };
    write_reply(&mut out, &Reply::SnapshotAccepted { request_id }).unwrap();
    out.flush().unwrap();
    let Some(Request::Capture { request_id }) = read_request(&mut input).unwrap() else {
        panic!("capture expected");
    };
    let mode = "mismatched";
    let mut observed = stamp;
    if mode == "stale" {
        observed.nanoseconds += 1;
    }
    let response_id = if mode == "mismatched" {
        request_id + 1
    } else {
        request_id
    };
    let count = if mode == "duplicate" { 2 } else { 1 };
    for _ in 0..count {
        write_reply(
            &mut out,
            &Reply::Frame {
                request_id: response_id,
                stamp: observed,
                rgba: vec![0; 64 * 64 * 4],
            },
        )
        .unwrap();
    }
    out.flush().unwrap();
    std::thread::sleep(std::time::Duration::from_secs(60));
}
