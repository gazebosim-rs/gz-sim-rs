// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Builds the cxx bridge against one DART 6.13.2 installation, headers and
//! libraries from the same prefix, and pins that library directory at run time.

use std::path::{Path, PathBuf};

/// The exact DART release linked by the upstream dartsim plugin reference.
pub(crate) const DART_VERSION: &str = "6.13.2";

/// DART libraries the bridge loads, which must all come from dart.pc's `libdir`.
pub(crate) const LIBRARIES: [&str; 3] = [
    "libdart.so",
    "libdart-collision-ode.so",
    "libdart-external-odelcpsolver.so",
];

pub(crate) struct Native {
    /// Header search paths; the DART prefix's include root comes first.
    pub(crate) includes: Vec<PathBuf>,
    pub(crate) libdir: PathBuf,
    /// ode.pc's `libdir`, which may differ from DART's (e.g. ROS-vendored DART).
    pub(crate) ode_libdir: PathBuf,
}

pub(crate) fn variable(package: &str, name: &str) -> Result<PathBuf, String> {
    let value = pkg_config::get_variable(package, name)
        .map_err(|error| format!("{package}.pc has no {name}: {error}"))?;
    Path::new(&value)
        .canonicalize()
        .map_err(|error| format!("{package}.pc {name} {value} is not usable: {error}"))
}

/// Rejects a dart.pc whose headers and libraries are not one `version` install.
pub(crate) fn check_install(
    version: &str,
    prefix: &Path,
    includedir: &Path,
    libdir: &Path,
) -> Result<PathBuf, String> {
    if !includedir.starts_with(prefix) || !libdir.starts_with(prefix) {
        return Err(format!(
            "dart.pc mixes prefixes: headers {} and libraries {} are not both under {}",
            includedir.display(),
            libdir.display(),
            prefix.display()
        ));
    }
    // dart.pc's includedir is <prefix>/include/dart; sources include <dart/...>.
    let root = includedir.parent().unwrap_or(includedir).to_path_buf();
    let config = root.join("dart/config.hpp");
    let define = format!("#define DART_VERSION \"{version}\"");
    if !std::fs::read_to_string(&config).is_ok_and(|text| text.contains(&define)) {
        return Err(format!(
            "{} does not declare DART {version}",
            config.display()
        ));
    }
    for library in LIBRARIES {
        let file = libdir.join(format!("{library}.{version}"));
        if !file.is_file() {
            return Err(format!("{} is missing", file.display()));
        }
    }
    Ok(root)
}

/// Finds exactly `version` through pkg-config. `emit` prints Cargo directives.
pub(crate) fn probe(version: &str, emit: bool) -> Result<Native, String> {
    let dart = pkg_config::Config::new()
        .exactly_version(version)
        .cargo_metadata(emit)
        .probe("dart")
        .map_err(|error| format!("DART {version} is required: {error}"))?;
    let ode = pkg_config::Config::new()
        .cargo_metadata(emit)
        .probe("ode")
        .map_err(|error| format!("ODE is required: {error}"))?;
    let libdir = variable("dart", "libdir")?;
    let ode_libdir = variable("ode", "libdir")?;
    let root = check_install(
        version,
        &variable("dart", "prefix")?,
        &variable("dart", "includedir")?,
        &libdir,
    );
    // The compiler already searches /usr/include, and naming it with -I would
    // break libstdc++'s #include_next; other roots must precede dependencies.
    root.map(|root| Native {
        includes: (root != Path::new("/usr/include"))
            .then_some(root)
            .into_iter()
            .chain(dart.include_paths)
            .chain(ode.include_paths)
            .collect(),
        libdir,
        ode_libdir,
    })
}

/// Cargo directives that link the bridge's libraries from `libdir` and pin them.
pub(crate) fn link_directives(libdir: &Path, ode_libdir: &Path) -> Vec<String> {
    let libdir = libdir.display();
    let ode_libdir = ode_libdir.display();
    let mut lines = vec![
        // dart.pc does not list the ODE collision component the bridge selects.
        "cargo:rustc-link-lib=dart-collision-ode".to_owned(),
        // The bridge names an LCP symbol so the run-time check can locate it.
        "cargo:rustc-link-lib=dart-external-odelcpsolver".to_owned(),
        // DT_RPATH, unlike DT_RUNPATH, precedes LD_LIBRARY_PATH, so another DART
        // 6.13 on the library path cannot replace the one compiled against.
        "cargo:rustc-link-arg=-Wl,--disable-new-dtags".to_owned(),
        format!("cargo:rustc-link-arg=-Wl,-rpath,{libdir}"),
        format!("cargo:rustc-env=GZ_PHYSICS_DART_LIBDIR={libdir}"),
        format!("cargo:rustc-env=GZ_PHYSICS_DART_ODE_LIBDIR={ode_libdir}"),
        // Downstream build scripts read this as DEP_DART_LIBDIR to pin binaries.
        format!("cargo:libdir={libdir}"),
    ];
    for path in [
        "src/ffi.rs",
        "src/bridge.cc",
        "src/joint_state.hh",
        "include/bridge.h",
    ] {
        lines.push(format!("cargo:rerun-if-changed={path}"));
    }
    lines
}

fn main() {
    let native = probe(DART_VERSION, true).unwrap_or_else(|error| panic!("{error}"));
    cxx_build::bridge("src/ffi.rs")
        .file("src/bridge.cc")
        .includes(&native.includes)
        .std("c++17")
        .compile("gz-physics-dart-bridge");
    for line in link_directives(&native.libdir, &native.ode_libdir) {
        println!("{line}");
    }
}
