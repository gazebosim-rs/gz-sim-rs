// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Regeneration uses the same qualified compiler, sources and loaded DART/ODE.
use std::{
    process::{Command, Stdio},
    time::{Duration, Instant},
};

#[test]
fn selected_three_body_static_native_records_replay() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let directory = root.join(format!(
        "artifacts/three-static-native-{}",
        std::process::id()
    ));
    let mut child = Command::new("python3")
        .arg(root.join("scripts/reference-three-body-static-contacts.py"))
        .arg("--output")
        .arg(&directory)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(240);
    let mut timeout = false;
    while child.try_wait().unwrap().is_none() {
        if Instant::now() >= deadline {
            timeout = true;
            child.kill().unwrap();
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    let output = child.wait_with_output().unwrap();
    assert!(
        !timeout && output.status.success(),
        "native replay {:?}:\n{}\n{}",
        output.status,
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(String::from_utf8_lossy(&output.stdout).contains(
        "Seven source-bound native static-contact controls and selected records replayed"
    ));
}
