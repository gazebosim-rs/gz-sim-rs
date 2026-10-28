// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The executable loads the DART the adapter was built against, even when another
//! copy appears first on LD_LIBRARY_PATH.

#[allow(dead_code)]
#[path = "../build.rs"]
mod build_script;

use std::process::Command;

#[test]
fn directives_pin_binaries_with_rpath_ahead_of_the_library_path() {
    assert_eq!(
        build_script::link_directives(Some("/opt/dart/lib".into())),
        "cargo:rustc-link-arg=-Wl,--disable-new-dtags\ncargo:rustc-link-arg=-Wl,-rpath,/opt/dart/lib\n"
    );
}

/// Runs the build script's entry point the way Cargo does, in a child process
/// with DEP_DART_LIBDIR set, since the parent cannot safely change its environment.
#[test]
fn build_script_main_prints_the_directives_cargo_reads() {
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "build_script_main_child",
            "--ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env("DEP_DART_LIBDIR", "/opt/dart/lib")
        .output()
        .unwrap();
    assert!(output.status.success(), "{output:?}");
    let stdout = String::from_utf8(output.stdout).unwrap();
    assert!(stdout.contains("cargo:rustc-link-arg=-Wl,--disable-new-dtags\ncargo:rustc-link-arg=-Wl,-rpath,/opt/dart/lib\n"), "{stdout}");
}

#[test]
#[ignore = "run by build_script_main_prints_the_directives_cargo_reads"]
fn build_script_main_child() {
    build_script::main();
}

#[test]
#[should_panic(expected = "DEP_DART_LIBDIR")]
fn a_missing_dart_library_directory_fails_the_build() {
    build_script::link_directives(None);
}

#[test]
fn dart_runs_with_a_shadowing_library_path() {
    // An empty directory named first on the path cannot satisfy libdart, but a
    // wrong-version libdart there would be loaded without DT_RPATH. Point the path
    // at a decoy copy of a different library under DART's soname.
    let decoy = tempfile::tempdir().unwrap();
    std::fs::copy("/proc/self/exe", decoy.path().join("libdart.so.6.13")).unwrap();
    let fixture = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../gz-sim-scene/tests/fixtures/fall.sdf"
    );
    let result = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .args([fixture, "--physics-engine", "dart", "--iterations", "10"])
        .env("LD_LIBRARY_PATH", decoy.path())
        .output()
        .unwrap();
    assert!(result.status.success(), "{result:?}");
    assert!(
        String::from_utf8_lossy(&result.stdout)
            .starts_with("simulation_ns=10000000 iterations=10\n")
    );
}
