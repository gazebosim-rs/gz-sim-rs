// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_protocol::{PROTOCOL_VERSION, Reply, write_reply};
use std::io::Write;
fn main() {
    let mut output = std::io::stdout().lock();
    for _ in 0..2 {
        write_reply(
            &mut output,
            &Reply::Ready {
                version: PROTOCOL_VERSION,
            },
        )
        .unwrap();
    }
    output.flush().unwrap();
    std::thread::sleep(std::time::Duration::from_secs(60));
}
