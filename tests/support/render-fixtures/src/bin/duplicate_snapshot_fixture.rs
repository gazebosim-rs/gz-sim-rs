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
    let Some(Request::Snapshot { request_id, .. }) = read_request(&mut input).unwrap() else {
        panic!("snapshot expected");
    };
    for _ in 0..2 {
        write_reply(&mut out, &Reply::SnapshotAccepted { request_id }).unwrap();
    }
    out.flush().unwrap();
    std::thread::sleep(std::time::Duration::from_secs(60));
}
