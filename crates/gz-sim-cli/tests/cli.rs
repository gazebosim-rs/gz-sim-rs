// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::{
    path::Path,
    process::{Command, Output},
};

fn command(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .args(args)
        .output()
        .unwrap()
}
fn fixture() -> &'static str {
    concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../gz-sim-scene/tests/fixtures/fall.sdf"
    )
}

#[test]
fn help_version_and_argument_errors_are_actionable() {
    for args in [&["--help"][..], &["--version"]] {
        assert!(command(args).status.success());
    }
    assert!(
        String::from_utf8(command(&["--help"]).stdout)
            .unwrap()
            .contains("--step-ns")
    );
    for args in [
        &[][..],
        &[fixture(), "--step-ns", "0"],
        &[fixture(), "--iterations", "invalid"],
        &[fixture(), "--unknown"],
    ] {
        assert_eq!(command(args).status.code(), Some(2));
    }
}

#[test]
fn exact_headless_steps_produce_sorted_physical_state() {
    assert!(Path::new(fixture()).is_file());
    let result = command(&[fixture(), "--iterations", "100"]);
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let text = String::from_utf8(result.stdout).unwrap();
    let lines = text.lines().collect::<Vec<_>>();
    assert_eq!(lines.len(), 3);
    assert_eq!(lines[0], "simulation_ns=100000000 iterations=100");
    assert!(lines[1].starts_with("default::ball::body "));
    let z = lines[1]
        .split_whitespace()
        .last()
        .unwrap()
        .parse::<f64>()
        .unwrap();
    assert!((z - 4.95095).abs() < 0.001);
    assert_eq!(lines[2], "default::ground::base 0 0 -0.5");
    let result = command(&[
        fixture(),
        "--iterations",
        "0",
        "--world",
        "default",
        "--step-ns",
        "1000000",
    ]);
    assert!(result.status.success());
    assert!(
        String::from_utf8(result.stdout)
            .unwrap()
            .contains("default::ball::body 0 0 5")
    );
}

#[test]
fn file_and_world_errors_are_not_reported_as_success() {
    for args in [
        &["/nonexistent/gz-sim-fixture.sdf"][..],
        &[fixture(), "--world", "missing"],
    ] {
        let result = command(args);
        assert_eq!(result.status.code(), Some(1));
        assert!(!result.stderr.is_empty());
        assert!(result.stdout.is_empty());
    }
}

#[test]
fn unrepresentable_step_request_fails_before_running() {
    let result = command(&[
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/empty.sdf"),
        "--step-ns",
        "18446744073709551615",
        "--iterations",
        "18446744073709551615",
    ]);
    assert_eq!(result.status.code(), Some(1));
    assert!(
        String::from_utf8(result.stderr)
            .unwrap()
            .contains("overflow")
    );
    assert!(result.stdout.is_empty());
}

#[cfg(target_os = "linux")]
#[test]
fn output_failure_returns_an_error() {
    let result = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .args([fixture(), "--iterations", "0"])
        .stdout(
            std::fs::File::options()
                .write(true)
                .open("/dev/full")
                .unwrap(),
        )
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(1));
    assert!(
        String::from_utf8(result.stderr)
            .unwrap()
            .contains("No space left")
    );
}

#[test]
fn tpe_selection_preserves_headless_output_and_rejects_unknown_engines() {
    let result = command(&[fixture(), "--physics-engine", "tpe", "--iterations", "100"]);
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert_eq!(
        String::from_utf8(result.stdout).unwrap(),
        "simulation_ns=100000000 iterations=100\ndefault::ball::body 0 0 5\ndefault::ground::base 0 0 -0.5\n"
    );
    let result = command(&[fixture(), "--physics-engine", "bullet"]);
    assert_eq!(result.status.code(), Some(2));
    assert!(
        String::from_utf8(result.stderr)
            .unwrap()
            .contains("expected rapier or tpe")
    );
    assert!(result.stdout.is_empty());
}
