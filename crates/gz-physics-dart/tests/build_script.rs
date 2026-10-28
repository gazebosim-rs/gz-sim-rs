// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The DART the bridge was compiled against is the DART loaded at run time, and
//! the build script refuses installs it cannot pin.

#[allow(dead_code)]
#[path = "../build.rs"]
mod build_script;

use gz_physics::PhysicsError;
use gz_physics_dart::DartBackend;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Directories of every mapped file named `name` or `name.<version>`.
fn mapped(name: &str) -> Vec<PathBuf> {
    let maps = std::fs::read_to_string("/proc/self/maps").unwrap();
    let mut directories: Vec<PathBuf> = maps
        .lines()
        .filter_map(|line| line.split_whitespace().nth(5))
        .map(Path::new)
        .filter(|path| {
            path.file_name()
                .and_then(|file| file.to_str())
                .is_some_and(|file| file == name || file.starts_with(&format!("{name}.")))
        })
        .filter_map(|path| path.parent().map(Path::to_path_buf))
        .collect();
    directories.dedup();
    directories
}

#[test]
fn loaded_dart_libraries_come_from_the_pkg_config_libdir() {
    let _world = DartBackend::new([0.0; 3]).unwrap();
    let dart = Path::new(env!("GZ_PHYSICS_DART_LIBDIR"));
    let ode = Path::new(env!("GZ_PHYSICS_DART_ODE_LIBDIR"));
    let libraries = build_script::LIBRARIES
        .into_iter()
        .map(|library| (library, dart))
        .chain([("libode.so", ode)]);
    for (library, expected) in libraries {
        let directories = mapped(library);
        assert!(!directories.is_empty(), "{library} is not loaded");
        for directory in directories {
            assert_eq!(
                directory.canonicalize().unwrap(),
                expected.canonicalize().unwrap(),
                "{library}"
            );
        }
    }
}

#[test]
fn link_directives_pin_the_libdir_ahead_of_ld_library_path() {
    let libdir = Path::new("/opt/dart/lib");
    let lines = build_script::link_directives(libdir, Path::new("/opt/ode/lib"));
    let position = |line: &str| lines.iter().position(|l| l == line).unwrap();
    // --disable-new-dtags must precede -rpath to emit DT_RPATH, not DT_RUNPATH.
    assert!(
        position("cargo:rustc-link-arg=-Wl,--disable-new-dtags")
            < position("cargo:rustc-link-arg=-Wl,-rpath,/opt/dart/lib")
    );
    position("cargo:rustc-link-lib=dart-collision-ode");
    position("cargo:rustc-env=GZ_PHYSICS_DART_LIBDIR=/opt/dart/lib");
    position("cargo:rustc-env=GZ_PHYSICS_DART_ODE_LIBDIR=/opt/ode/lib");
    position("cargo:rustc-link-lib=dart-external-odelcpsolver");
    position("cargo:rerun-if-changed=src/bridge.cc");
    for package in ["dart", "ode"] {
        let error = build_script::variable(package, "no-such-variable").unwrap_err();
        assert!(error.starts_with(&format!("{package}.pc")), "{error}");
    }
}

#[test]
fn build_rejects_other_versions_and_mixed_installs() {
    let error = build_script::probe("0.0.0", false).err().unwrap();
    assert!(error.starts_with("DART 0.0.0 is required"), "{error}");
    let native = build_script::probe(build_script::DART_VERSION, false).unwrap();
    assert_eq!(native.libdir, Path::new(env!("GZ_PHYSICS_DART_LIBDIR")));
    // A dart.pc pairing one prefix's headers with another's libraries fails.
    let error = build_script::check_install(
        build_script::DART_VERSION,
        Path::new("/usr"),
        Path::new("/usr/include/dart"),
        Path::new("/opt/ros/jazzy/opt/gz_dartsim_vendor/lib"),
    )
    .unwrap_err();
    assert!(error.contains("mixes prefixes"), "{error}");
    // Headers or libraries of another release fail too.
    let root = Path::new("/");
    let includedir = build_script::variable("dart", "includedir").unwrap();
    for version in ["6.13.1", "6.14.0"] {
        let error =
            build_script::check_install(version, root, &includedir, &native.libdir).unwrap_err();
        assert!(error.contains("does not declare DART"), "{error}");
    }
    let empty = std::env::temp_dir();
    let error = build_script::check_install(build_script::DART_VERSION, root, &includedir, &empty)
        .unwrap_err();
    assert!(error.ends_with("is missing"), "{error}");
}

/// A binary that does not carry this crate's rpath can still load another
/// libdart.so.6.13 first; construction must refuse it rather than simulate with
/// a library other than the one compiled against. The child process preloads a
/// copy of the pinned library from another directory.
#[test]
fn a_libdart_loaded_from_another_directory_is_refused() {
    const CHILD: &str = "GZ_PHYSICS_DART_FOREIGN_CHILD";
    if std::env::var_os(CHILD).is_some() {
        let error = DartBackend::new([0.0; 3]).err().unwrap();
        let PhysicsError::Native { engine, message } = &error else {
            panic!("unexpected {error:?}");
        };
        assert_eq!(*engine, "dart");
        assert!(message.contains("built against"), "{message}");
        return;
    }
    let libdir = Path::new(env!("GZ_PHYSICS_DART_LIBDIR"));
    let source = libdir.join(format!("libdart.so.{}", build_script::DART_VERSION));
    let foreign =
        std::env::temp_dir().join(format!("gz-physics-dart-foreign-{}", std::process::id()));
    std::fs::create_dir_all(&foreign).unwrap();
    let copy = foreign.join("libdart.so.6.13");
    std::fs::copy(source, &copy).unwrap();
    let output = Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "a_libdart_loaded_from_another_directory_is_refused",
        ])
        .env(CHILD, "1")
        .env("LD_PRELOAD", &copy)
        .output()
        .unwrap();
    std::fs::remove_dir_all(&foreign).unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(output.status.success(), "{stdout}");
    assert!(stdout.contains("1 passed"), "{stdout}");
}
