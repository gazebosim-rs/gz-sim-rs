// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Pins every linked artifact (binary, tests) to the DART the adapter verified.

/// Cargo directives from gz-physics-dart's `links = "dart"` library directory.
pub(crate) fn link_directives(libdir: Option<String>) -> String {
    let libdir = libdir.expect("gz-physics-dart exports its library directory as DEP_DART_LIBDIR");
    // DT_RPATH, unlike DT_RUNPATH, precedes LD_LIBRARY_PATH, so another DART 6.13
    // on the library path cannot replace the one the adapter verified.
    format!(
        "cargo:rustc-link-arg=-Wl,--disable-new-dtags\ncargo:rustc-link-arg=-Wl,-rpath,{libdir}\n"
    )
}

/// A disabled native feature needs neither metadata nor native linker arguments.
pub(crate) fn configured_link_directives(enabled: bool, libdir: Option<String>) -> String {
    if enabled {
        link_directives(libdir)
    } else {
        String::new()
    }
}

pub(crate) fn main() {
    print!(
        "{}",
        configured_link_directives(
            cfg!(feature = "native-backends"),
            std::env::var("DEP_DART_LIBDIR").ok()
        )
    );
}
