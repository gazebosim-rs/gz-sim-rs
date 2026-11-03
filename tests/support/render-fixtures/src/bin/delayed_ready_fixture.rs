// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_protocol::{PROTOCOL_VERSION, Reply, write_reply};
use std::io::Write;
fn main() {
    std::thread::sleep(std::time::Duration::from_millis(200));
    let mut out = std::io::stdout().lock();
    write_reply(
        &mut out,
        &Reply::Ready {
            version: PROTOCOL_VERSION,
        },
    )
    .unwrap();
    out.flush().unwrap();
    std::thread::sleep(std::time::Duration::from_secs(60));
}
