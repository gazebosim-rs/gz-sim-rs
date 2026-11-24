// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn invalid_hinge_coefficients_fail_during_world_admission() {
    for value in ["-0.01", "sticky"] {
        let xml = WORLD.replace(
            "bend_damping=\"0.08\"",
            &format!("bend_damping=\"{value}\""),
        );
        assert!(Recording::new(&xml, "invalid.sdf", "default").is_err());
    }
}

#[test]
fn measurement_requires_each_named_sample() {
    let xml = WORLD.replace("elastic", "replacement");
    let r = Recording::new(&xml, "renamed.sdf", "default").unwrap();
    assert_eq!(
        positions(&r.sim).unwrap_err().to_string(),
        "missing measured cloth"
    );
}
