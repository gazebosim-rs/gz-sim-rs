// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_render_protocol::{PROTOCOL_VERSION, Reply, write_reply};
use std::os::unix::process::CommandExt;
fn main() {
    let mut ready = Vec::new();
    write_reply(
        &mut ready,
        &Reply::Ready {
            version: PROTOCOL_VERSION,
        },
    )
    .unwrap();
    // Construct only bytes here: the shell must close stdin BEFORE emitting
    // Ready, so a caller observing Ready cannot race the broken-pipe witness.
    let escaped: String = ready.iter().map(|byte| format!("\\{byte:03o}")).collect();
    let error = std::process::Command::new("/bin/sh")
        .arg("-c")
        .arg(format!("exec 0<&-; printf '{escaped}'; exec sleep 60"))
        .exec();
    panic!("exec failed: {error}");
}
