// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../build.rs"]
mod build_script;

#[cfg(not(feature = "native-backends"))]
use std::process::Command;

#[cfg(not(feature = "native-backends"))]
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
#[cfg(not(feature = "native-backends"))]
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
#[cfg(not(feature = "native-backends"))]
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
        assert!(error.contains("unavailable in this build"), "{error}");
        assert!(error.contains("native-backends"), "{error}");
    }
}

#[test]
fn disabled_native_build_scripts_ignore_absent_or_ambient_native_metadata() {
    assert_eq!(build_script::configured_link_directives(false, None), "");
    assert_eq!(
        build_script::configured_link_directives(false, Some("/unselected/native/sdk".into())),
        ""
    );
}

#[test]
fn enabled_native_build_scripts_keep_verified_rpath_and_missing_metadata_failure() {
    assert_eq!(
        build_script::configured_link_directives(true, Some("/opt/dart/lib".into())),
        "cargo:rustc-link-arg=-Wl,--disable-new-dtags\ncargo:rustc-link-arg=-Wl,-rpath,/opt/dart/lib\n"
    );
    assert!(
        std::panic::catch_unwind(|| build_script::configured_link_directives(true, None)).is_err()
    );
}
