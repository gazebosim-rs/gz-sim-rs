// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/routed_contacts.rs"]
mod scenario;
use std::{fs, path::Path};

#[test]
fn lost_route_activation_and_normal_delivery_match_native_solve_and_record_the_same_owners() {
    let dir = tempfile::tempdir().unwrap();
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let captures = [dir.path().join("normal"), dir.path().join("retry")];
    let a = scenario::run(worker, Some(&captures[0]), false).unwrap();
    let b = scenario::run(worker, Some(&captures[1]), true).unwrap();
    assert_eq!(a, b);
    for file in ["frames.jsonl", "manifest.json"] {
        assert_eq!(
            fs::read(captures[0].join(file)).unwrap(),
            fs::read(captures[1].join(file)).unwrap()
        );
    }
}
