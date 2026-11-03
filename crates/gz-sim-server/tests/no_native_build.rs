// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../build.rs"]
mod build_script;

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
