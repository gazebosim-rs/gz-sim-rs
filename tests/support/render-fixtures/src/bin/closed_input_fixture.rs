// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::os::unix::process::CommandExt;
fn main() {
    let error = std::process::Command::new("/bin/sh")
        .arg("-c")
        .arg("exec 0<&-; printf '\\027\\000\\000\\000{\"Ready\":{\"version\":1}}'; exec sleep 60")
        .exec();
    panic!("exec failed: {error}");
}
