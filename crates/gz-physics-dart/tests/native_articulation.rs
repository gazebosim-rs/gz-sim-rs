// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Direct native topology observations, not adapter or scene parity claims.

#[allow(dead_code)]
#[path = "../build.rs"]
mod build_script;

use std::{
    path::{Path, PathBuf},
    process::{Command, Output, Stdio},
    time::{Duration, Instant},
};

struct Scratch(PathBuf);
impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn run(mut command: Command, success: bool) -> Output {
    let mut child = command
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(60);
    let timed_out = loop {
        if child.try_wait().unwrap().is_some() {
            break false;
        }
        if Instant::now() >= deadline {
            child.kill().unwrap();
            break true;
        }
        std::thread::sleep(Duration::from_millis(5));
    };
    let result = child.wait_with_output().unwrap();
    assert!(
        !timed_out && result.status.success() == success,
        "{command:?}: {result:?}"
    );
    result
}

fn output(command: Command) -> Output {
    run(command, true)
}

fn pkg_config(argument: &str) -> String {
    let mut command = Command::new("pkg-config");
    command.args([argument, "dart"]);
    String::from_utf8(output(command).stdout)
        .unwrap()
        .trim()
        .to_owned()
}

#[test]
fn native_revolute_topology_ownership_and_motion() {
    let selected = build_script::probe(build_script::DART_VERSION, false).unwrap();
    let libdir = PathBuf::from(env!("GZ_PHYSICS_DART_LIBDIR"))
        .canonicalize()
        .unwrap();
    let ode_libdir = PathBuf::from(env!("GZ_PHYSICS_DART_ODE_LIBDIR"))
        .canonicalize()
        .unwrap();
    assert_eq!(selected.libdir, libdir);
    assert_eq!(selected.ode_libdir, ode_libdir);
    let flags = pkg_config("--cflags");
    let libraries = pkg_config("--libs");
    let path = std::env::temp_dir().join(format!("gz-dart-articulation-{}", std::process::id()));
    std::fs::create_dir(&path).unwrap();
    let scratch = Scratch(path);
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    for (source, expected) in [
        (
            "body_revolute_free.cc",
            "detach pose/velocity preserved; parent remains live",
        ),
        (
            "body_revolute_fixed.cc",
            "merged skeleton and both bodies retire with last owner",
        ),
        (
            "body_revolute_chain.cc",
            "two-DOF world-root chain preserved moving parent/child twists, anchor and reset",
        ),
        (
            "body_revolute_response.cc",
            "expanded dependent Jacobians match native point impulse response",
        ),
    ] {
        let executable = scratch.0.join(source.trim_end_matches(".cc"));
        let mut compile = Command::new(std::env::var_os("CXX").unwrap_or_else(|| "c++".into()));
        compile
            .args(["-O0", "-g0", "-Wall", "-Wextra", "-Werror"])
            .arg(fixtures.join(source))
            .arg("-o")
            .arg(&executable);
        for include in &selected.includes {
            compile.arg("-I").arg(include);
        }
        compile
            .args(flags.split_whitespace())
            .args(libraries.split_whitespace())
            .arg("-UNDEBUG")
            .arg("-std=c++17")
            .arg("-Wl,--disable-new-dtags")
            .arg(format!("-Wl,-rpath,{}", libdir.display()));
        output(compile);
        if source == "body_revolute_free.cc" {
            let wrong = scratch.0.canonicalize().unwrap();
            assert_ne!(wrong, libdir);
            let mut mismatched = Command::new(&executable);
            mismatched
                .env_remove("LD_PRELOAD")
                .env_remove("LD_LIBRARY_PATH")
                .env_remove("LD_AUDIT")
                .env("GZ_DART_PROBE_LIBDIR", wrong);
            assert_eq!(run(mismatched, false).status.code(), Some(2));
        }
        let mut native = Command::new(&executable);
        native
            .env_remove("LD_PRELOAD")
            .env_remove("LD_LIBRARY_PATH")
            .env_remove("LD_AUDIT")
            .env("GZ_DART_PROBE_LIBDIR", &libdir);
        let result = output(native);
        assert!(String::from_utf8(result.stdout).unwrap().contains(expected));
    }
}
