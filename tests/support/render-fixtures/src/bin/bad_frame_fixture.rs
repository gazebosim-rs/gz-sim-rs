// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::io::Write;
fn main() {
    let mut out = std::io::stdout().lock();
    out.write_all(&131_073_u32.to_le_bytes()).unwrap();
    out.flush().unwrap();
    std::thread::sleep(std::time::Duration::from_secs(60));
}
