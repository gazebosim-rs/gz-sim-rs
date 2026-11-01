// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Test-only camera TCP isolation; no application behavior or deadline is changed.
use std::{
    net::TcpListener,
    path::PathBuf,
    process::{Command, Stdio},
};

const CHILD: &str = "GZ_AUTOMATIC_CAMERA_TEST";
const HOST_NAMESPACE: &str = "GZ_AUTOMATIC_CAMERA_HOST_NETNS";
const ADDRESS: &str = "GZ_AUTOMATIC_CAMERA_HOST_ADDRESS";

/// Return true in the parent after one successful exact child test. Each child
/// owns its network namespace; it still runs the test's original single-attempt
/// binds, bounded frames and unchanged worker lifecycle assertions.
pub fn child(test: &str, outer_seconds: u32) -> bool {
    if let Some(selected) = std::env::var_os(CHILD) {
        assert_eq!(selected, test);
        assert_ne!(
            std::fs::read_link("/proc/self/ns/net").unwrap(),
            PathBuf::from(std::env::var_os(HOST_NAMESPACE).unwrap()),
            "camera test did not enter a private network namespace"
        );
        // This address stays occupied in the parent throughout this test.
        let witness = TcpListener::bind(std::env::var(ADDRESS).unwrap()).unwrap();
        assert!(witness.local_addr().unwrap().ip().is_loopback());
        return false;
    }
    let host = TcpListener::bind("127.0.0.2:0").unwrap();
    let address = host.local_addr().unwrap();
    // The outer watchdog owns/reaps the child group. It is separate from the
    // unchanged per-operation/readback/shutdown deadlines inside the test.
    let output = Command::new("timeout")
        .args([
            "--signal=KILL",
            &format!("{outer_seconds}s"),
            "unshare",
            "-Urn",
            "--",
            "sh",
            "-c",
            "ip link set dev lo up && exec \"$@\"",
            "camera-test",
        ])
        .arg(std::env::current_exe().unwrap())
        .args(["--exact", test, "--nocapture"])
        .env(CHILD, test)
        .env(
            HOST_NAMESPACE,
            std::fs::read_link("/proc/self/ns/net").unwrap(),
        )
        .env(ADDRESS, address.to_string())
        .stdin(Stdio::null())
        .output()
        .unwrap();
    let stdout = String::from_utf8(output.stdout).unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success(),
        "{}: {stdout} {stderr}",
        output.status
    );
    assert!(
        stdout.contains(&format!("test {test} ... ok"))
            && stdout.contains("test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured;"),
        "camera child did not execute exactly one test: {stdout}"
    );
    assert_eq!(host.local_addr().unwrap(), address);
    assert_eq!(
        TcpListener::bind(address).unwrap_err().kind(),
        std::io::ErrorKind::AddrInUse
    );
    true
}
