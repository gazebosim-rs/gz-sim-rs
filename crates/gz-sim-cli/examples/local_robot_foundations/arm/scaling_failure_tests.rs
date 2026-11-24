// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn inertia_measurements_require_the_authored_arm_and_sufficient_precision() {
    assert!(inertia_probe("<sdf", 1000).is_err());
    let no_shoulder = SCENE.replace("name=\"shoulder\"", "name=\"other\"");
    assert!(
        qualified_inertia(&no_shoulder)
            .unwrap_err()
            .to_string()
            .contains("shoulder")
    );
    let no_lift = SCENE.replace("name=\"lift\"", "name=\"other\"");
    assert!(
        inertia_probe(&no_lift, 1000)
            .unwrap_err()
            .to_string()
            .contains("lift")
    );
    let renamed_body = SCENE
        .replace("name=\"right\"", "name=\"other\"")
        .replace("<child>right</child>", "<child>other</child>");
    assert!(
        inertia_probe(&renamed_body, 1000)
            .unwrap_err()
            .to_string()
            .contains("right")
    );
    let poor = SCENE.replace(
        "<additional_pgs_iterations>41</additional_pgs_iterations>",
        "",
    );
    assert!(
        qualified_inertia(&poor)
            .unwrap_err()
            .to_string()
            .contains("outside accuracy budget")
    );
}

#[test]
fn invalid_or_unexercised_workloads_do_not_generate_scaling_numbers() {
    assert!(scale("<sdf", 4, false).is_err());
    assert!(
        scale(&chain(4, false), 8, false)
            .unwrap_err()
            .to_string()
            .contains("chain::l7")
    );
    assert!(
        scale(&chain(4, false), 4, true)
            .unwrap_err()
            .to_string()
            .contains("support::top")
    );
    let unsupported = chain(4, true).replace("0.4648", "-2.0");
    assert!(
        scale(&unsupported, 4, true)
            .unwrap_err()
            .to_string()
            .contains("loaded=0")
    );
    let stationary = chain(4, false).replace("0 0 -9.81", "0 0 0");
    assert!(
        scale(&stationary, 4, false)
            .unwrap_err()
            .to_string()
            .contains("invalid chain workload")
    );
}

#[test]
fn precision_costs_require_the_same_arm_and_a_successful_full_transfer() {
    let missing = SCENE.replace("name=\"shoulder\"", "name=\"wrong\"");
    assert!(
        iteration_sweep(&missing)
            .unwrap_err()
            .to_string()
            .contains("shoulder")
    );
    let obstructed = SCENE.replace(".22627 .22627 .1175", ".22627 .22627 .225");
    assert!(
        iteration_sweep(&obstructed)
            .unwrap_err()
            .to_string()
            .contains("arm transfer failed")
    );
}
