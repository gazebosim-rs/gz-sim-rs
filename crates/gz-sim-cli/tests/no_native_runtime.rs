// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use std::process::Command;

fn command(engine: &str) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .args([
            concat!(env!("CARGO_MANIFEST_DIR"), "/tests/empty.sdf"),
            "--physics-engine",
            engine,
            "--iterations",
            "2",
            "--step-ns",
            "1000000",
        ])
        .output()
        .unwrap()
}

#[test]
fn non_native_executable_runs_rapier_and_tpe() {
    #[cfg(not(feature = "rgb-camera"))]
    {
        let result = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
            .args([
                "/nonexistent/m5-world.sdf",
                "--server",
                "--camera-worker",
                "/nonexistent/m5-worker",
            ])
            .output()
            .unwrap();
        assert!(!result.status.success());
        assert!(result.stdout.is_empty());
        assert!(
            String::from_utf8(result.stderr)
                .unwrap()
                .contains("--camera-worker requires a build with the rgb-camera feature")
        );
    }
    for engine in ["rapier", "tpe"] {
        let result = command(engine);
        assert!(result.status.success(), "{result:?}");
        assert_eq!(
            String::from_utf8(result.stdout).unwrap(),
            "simulation_ns=2000000 iterations=2\n"
        );
    }
}

#[test]
fn native_short_names_and_plugin_aliases_fail_instead_of_falling_back() {
    for engine in [
        "dart",
        "gz-physics-dartsim-plugin",
        "mujoco",
        "gz-physics-mujoco-plugin",
    ] {
        let result = command(engine);
        assert!(
            !result.status.success(),
            "{engine} silently used a different engine"
        );
        assert!(result.stdout.is_empty());
        let error = String::from_utf8(result.stderr).unwrap();
        assert_eq!(result.status.code(), Some(2));
        assert!(
            error.contains("expected rapier, tpe, multifidelity, distributed, or gpu-batch"),
            "{error}"
        );
    }
}
