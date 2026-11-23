// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::{fs, process::Command};

#[test]
fn finite_server_and_headless_runs_commit_the_same_mixed_world() {
    let directory = tempfile::tempdir().unwrap();
    let config = directory.path().join("zenoh.json");
    fs::write(
        &config,
        r#"{"mode":"peer","listen":{"endpoints":["tcp/127.0.0.1:0"]},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}}}"#,
    )
    .unwrap();
    let launch = |server: bool| {
        let mut command = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
        command.args([
            concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../examples/multifidelity_platform.sdf"
            ),
            "--physics-engine",
            "multifidelity",
            "--iterations",
            "500",
        ]);
        if server {
            command
                .args(["--server", "--run"])
                .env("ZENOH_CONFIG", &config)
                .env(
                    "GZ_PARTITION",
                    format!("multifidelity-server-{}", std::process::id()),
                )
                .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
                .env_remove("GZ_RELAY");
        }
        let result = command.output().unwrap();
        assert!(
            result.status.success(),
            "{}",
            String::from_utf8_lossy(&result.stderr)
        );
        String::from_utf8(result.stdout).unwrap()
    };
    let headless = launch(false);
    assert_eq!(launch(true), headless);
    let mut lines = headless.lines();
    assert_eq!(lines.next(), Some("simulation_ns=500000000 iterations=500"));
    let cargo = lines
        .next()
        .unwrap()
        .strip_prefix("default::cargo::body ")
        .unwrap();
    let platform = lines
        .next()
        .unwrap()
        .strip_prefix("default::platform::body ")
        .unwrap();
    assert!(lines.next().is_none());
    let mut cargo_xyz = cargo.split_whitespace().map(|v| v.parse::<f64>().unwrap());
    let cargo_x = cargo_xyz.next().unwrap();
    let cargo_z = cargo_xyz.nth(1).unwrap();
    let platform_x: f64 = platform.split_whitespace().next().unwrap().parse().unwrap();
    assert!((0.15..0.5).contains(&cargo_x));
    assert!((0.29..0.31).contains(&cargo_z));
    assert!((platform_x - 0.5).abs() < 1e-12);
}
