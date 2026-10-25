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
        &[fixture(), "--max-xml-bytes", "0"],
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

#[test]
fn local_model_directory_includes_and_budgets_work_through_the_cli() {
    let temp = tempfile::TempDir::new().unwrap();
    let world = temp.path().join("world");
    let models = temp.path().join("models");
    let robot = models.join("local_fixture_robot");
    std::fs::create_dir_all(&world).unwrap();
    std::fs::create_dir_all(&robot).unwrap();
    std::fs::write(world.join("model.config"),"<model><name>w</name><sdf version='1.12'>world.sdf</sdf><sdf version='9.0'>future.sdf</sdf></model>").unwrap();
    std::fs::write(world.join("world.sdf"),"<sdf version='1.12'><world name='default'><include><uri>model://local_fixture_robot</uri></include></world></sdf>").unwrap();
    std::fs::write(
        robot.join("model.config"),
        "<model><sdf version='1.12'>robot.sdf</sdf></model>",
    )
    .unwrap();
    std::fs::write(robot.join("robot.sdf"),"<sdf version='1.12'><model name='robot'><link name='body'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model></sdf>").unwrap();
    let result = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .arg(&world)
        .arg("--resource-path")
        .arg(&models)
        .args(["--physics-engine", "tpe", "--iterations", "3"])
        .env_remove("SDF_PATH")
        .env_remove("GZ_SIM_RESOURCE_PATH")
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert_eq!(
        String::from_utf8(result.stdout).unwrap(),
        "simulation_ns=3000000 iterations=3\ndefault::robot::body 0 0 0\n"
    );
    assert!(
        String::from_utf8(result.stderr)
            .unwrap()
            .contains("NewerVersion")
    );
    let env_models = temp.path().join("env-models");
    let env_robot = env_models.join("local_fixture_robot");
    std::fs::create_dir_all(&env_robot).unwrap();
    std::fs::copy(robot.join("model.config"), env_robot.join("model.config")).unwrap();
    let xml = std::fs::read_to_string(robot.join("robot.sdf"))
        .unwrap()
        .replace(
            "<model name='robot'>",
            "<model name='robot'><pose>1 0 0 0 0 0</pose>",
        );
    std::fs::write(env_robot.join("robot.sdf"), xml).unwrap();
    let from_env = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .arg(&world)
        .arg("--resource-path")
        .arg(&env_models)
        .arg("--resource-path")
        .arg(&models)
        .args(["--physics-engine", "tpe", "--iterations", "3"])
        .env(
            "SDF_PATH",
            std::env::join_paths([&env_models, &env_models]).unwrap(),
        )
        .env(
            "GZ_SIM_RESOURCE_PATH",
            std::env::join_paths([&env_models, &models]).unwrap(),
        )
        .output()
        .unwrap();
    assert!(
        from_env.status.success(),
        "{}",
        String::from_utf8_lossy(&from_env.stderr)
    );
    assert!(
        String::from_utf8(from_env.stdout)
            .unwrap()
            .contains("default::robot::body 1 0 0")
    );
    let result = command(&[fixture(), "--max-xml-bytes", "1"]);
    assert_eq!(result.status.code(), Some(1));
    assert!(result.stdout.is_empty());
    assert!(
        String::from_utf8(result.stderr)
            .unwrap()
            .contains("byte limit")
    );
    let limited = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .arg(&world)
        .args(["--max-resource-warnings", "0"])
        .output()
        .unwrap();
    assert_eq!(limited.status.code(), Some(1));
    assert!(limited.stdout.is_empty());
    assert!(
        String::from_utf8(limited.stderr)
            .unwrap()
            .contains("diagnostic budget")
    );
    std::fs::remove_file(world.join("world.sdf")).unwrap();
    let failed = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .arg(&world)
        .output()
        .unwrap();
    assert_eq!(failed.status.code(), Some(1));
    assert!(failed.stdout.is_empty());
    assert!(
        String::from_utf8(failed.stderr)
            .unwrap()
            .contains("NewerVersion")
    );
}

#[test]
fn offline_fuel_cache_override_and_environment_drive_both_headless_engines() {
    let temp = tempfile::TempDir::new().unwrap();
    let cache = temp.path().join("cache");
    let directory = cache.join("fuel.gazebosim.org/alice/models/robot/2");
    std::fs::create_dir_all(&directory).unwrap();
    std::fs::write(
        directory.join("model.config"),
        "<model><sdf version='1.12'>model.sdf</sdf></model>",
    )
    .unwrap();
    std::fs::write(directory.join("model.sdf"), "<sdf version='1.12'><model name='robot'><static>true</static><link name='body'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model></sdf>").unwrap();
    let world = temp.path().join("world.sdf");
    std::fs::write(&world, "<sdf version='1.12'><world name='default'><include><uri>https://fuel.gazebosim.org/1.0/alice/models/robot/2</uri><pose>1 2 3 0 0 0</pose></include></world></sdf>").unwrap();
    for engine in ["rapier", "tpe"] {
        for override_path in [true, false] {
            let mut cmd = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
            cmd.arg(&world)
                .args(["--physics-engine", engine, "--iterations", "3"])
                .env_remove("SDF_PATH")
                .env_remove("GZ_SIM_RESOURCE_PATH");
            if override_path {
                cmd.arg("--fuel-cache-path")
                    .arg(&cache)
                    .env("GZ_FUEL_CACHE_PATH", temp.path().join("unused"));
            } else {
                cmd.env("GZ_FUEL_CACHE_PATH", &cache);
            }
            let result = cmd.output().unwrap();
            assert!(
                result.status.success(),
                "{}",
                String::from_utf8_lossy(&result.stderr)
            );
            assert_eq!(
                String::from_utf8(result.stdout).unwrap(),
                "simulation_ns=3000000 iterations=3\ndefault::robot::body 1 2 3\n"
            );
        }
    }
    std::fs::remove_dir_all(&cache).unwrap();
    let result = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .arg(&world)
        .arg("--fuel-cache-path")
        .arg(&cache)
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(1));
    assert!(result.stdout.is_empty());
    assert!(
        String::from_utf8(result.stderr)
            .unwrap()
            .contains("no usable cached Fuel model")
    );
}

#[test]
fn sdf_first_profile_step_and_explicit_cli_override_have_clear_precedence() {
    let temp = tempfile::TempDir::new().unwrap();
    let path = temp.path().join("physics.sdf");
    std::fs::write(&path, "<sdf version='1.12'><world name='default'><physics name='first' type='ode'><max_step_size>0.002</max_step_size><real_time_factor>0</real_time_factor></physics><physics name='marked' type='custom' default='true'><max_step_size>0.005</max_step_size></physics></world></sdf>").unwrap();
    for engine in ["rapier", "tpe"] {
        for (override_step, expected_ns) in [(None, 6_000_000), (Some("3000000"), 9_000_000)] {
            let mut cmd = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
            cmd.arg(&path)
                .args(["--physics-engine", engine, "--iterations", "3"]);
            if let Some(step) = override_step {
                cmd.args(["--step-ns", step]);
            }
            let result = cmd.output().unwrap();
            assert!(
                result.status.success(),
                "{}",
                String::from_utf8_lossy(&result.stderr)
            );
            assert_eq!(
                String::from_utf8(result.stdout).unwrap(),
                format!("simulation_ns={expected_ns} iterations=3\n")
            );
            assert!(result.stderr.is_empty());
        }
    }
    let invalid = "<sdf version='1.12'><world name='default'><physics name='zero' type='ode'><max_step_size>0</max_step_size></physics></world></sdf>";
    std::fs::write(&path, invalid).unwrap();
    let result = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .arg(&path)
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(1));
    assert!(result.stdout.is_empty());
    assert!(
        String::from_utf8(result.stderr)
            .unwrap()
            .contains("unrepresentable physics step")
    );
    let result = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .arg(&path)
        .args(["--step-ns", "3000000", "--iterations", "3"])
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert_eq!(
        String::from_utf8(result.stdout).unwrap(),
        "simulation_ns=9000000 iterations=3\n"
    );
    assert_eq!(std::fs::read_to_string(&path).unwrap(), invalid);
    std::fs::write(&path, "<sdf version='1.12'><world name='default'><physics name='limited' type='ode'><max_contacts>1</max_contacts></physics></world></sdf>").unwrap();
    for engine in ["rapier", "tpe"] {
        for explicit_override in [false, true] {
            let mut cmd = Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"));
            cmd.arg(&path).args(["--physics-engine", engine]);
            if explicit_override {
                cmd.args(["--step-ns", "3000000"]);
            }
            let result = cmd.output().unwrap();
            assert_eq!(result.status.code(), Some(1));
            assert!(result.stdout.is_empty());
            assert!(
                String::from_utf8(result.stderr)
                    .unwrap()
                    .contains("physics maximum contacts")
            );
        }
    }
}
