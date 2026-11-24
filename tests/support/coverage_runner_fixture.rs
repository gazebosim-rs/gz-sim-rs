// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Small libtest target used by the native coverage runner's regression tests.
use std::{env, fs::File, io::Write, path::PathBuf, process};

fn record(name: &str) {
    let directory = PathBuf::from(env::var_os("GZ_RUNNER_FIXTURE_DIR").unwrap());
    let mut file = File::create_new(directory.join(name)).unwrap();
    write!(file, "{}", process::id()).unwrap();
}

#[test]
fn first() {
    record("first");
}

#[test]
fn second() {
    record("second");
    assert_eq!(env::var("GZ_RUNNER_FIXTURE_FAIL").unwrap(), "0");
}

#[test]
#[ignore = "only selected on an explicit ignored run"]
fn dormant() {
    record("dormant");
}
