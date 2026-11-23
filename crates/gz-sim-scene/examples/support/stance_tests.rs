// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::run;
use serde_json::Value;

fn simulate(mode: &str, step: &str) -> Value {
    run([mode, "2", "15", step].into_iter().map(str::to_owned)).unwrap()
}
fn metric(v: &Value, name: &str) -> f64 {
    v[name].as_f64().unwrap()
}

#[test]
fn loaded_feet_support_the_reach_and_recover_from_a_measured_shove() {
    let result = simulate("reduced", "1000");
    assert_eq!(result["other_ground_ticks"], 0);
    assert_eq!(metric(&result, "both_feet_fraction"), 1.);
    assert!(metric(&result, "min_double_support_margin_m") > 0.05);
    assert!(metric(&result, "max_sole_penetration_m") < 0.006);
    assert!(metric(&result, "peak_tracking_rad") < 0.02);
    assert!(metric(&result, "max_pelvis_tilt_rad") < 0.035);
    assert!(metric(&result, "peak_horizontal_speed_after_shove_m_s") > 0.18);
    assert!(metric(&result, "stable_after_shove_s") < 0.7);
    assert!(metric(&result, "final_half_second_vertical_balance_residual_ns").abs() < 0.1);
    for error in result["final_hand_target_error_m"].as_array().unwrap() {
        assert!(error.as_f64().unwrap() < 0.004);
    }
    for reach in result["reach_m"].as_array().unwrap() {
        assert!((0.46..0.48).contains(&reach.as_f64().unwrap()));
    }
}

#[test]
fn independent_maximal_coordinate_reference_supports_the_reach_on_refinement() {
    let reference = simulate("impulse", "100");
    assert_eq!(reference["other_ground_ticks"], 0);
    assert!(metric(&reference, "both_feet_fraction") > 0.995);
    assert!(metric(&reference, "stable_after_shove_s") < 0.7);
    assert!(metric(&reference, "max_pelvis_tilt_rad") < 0.035);
    for reach in reference["reach_m"].as_array().unwrap() {
        assert!((0.46..0.48).contains(&reach.as_f64().unwrap()));
    }
    let coarse = simulate("impulse", "1000");
    assert!(metric(&coarse, "other_ground_ticks") > 100.);
    assert!(coarse["stable_after_shove_s"].is_null());
}

#[test]
fn removing_the_motor_commands_loses_upright_stance_before_the_push() {
    let passive = simulate("passive", "1000");
    assert!(metric(&passive, "first_other_ground_us") < super::SHOVE_US as f64);
    assert!(metric(&passive, "max_pelvis_tilt_rad") > 1.);
    assert!(passive["stable_after_shove_s"].is_null());
}
