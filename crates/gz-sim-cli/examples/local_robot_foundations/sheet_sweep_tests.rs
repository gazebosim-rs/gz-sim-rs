// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn comparisons_preserve_step_refusals_and_propagate_other_errors() {
    let c = Case::default();
    let refusal = || {
        Trial::from_result(
            c,
            Err(SheetStepFault {
                tick: 611,
                diagnostic: "unresolved sheet contact".into(),
            }
            .into()),
        )
        .unwrap()
    };
    let finished = || {
        Trial::from_result(
            c,
            Ok(Metrics {
                weight_n: 1.,
                ..Default::default()
            }),
        )
        .unwrap()
    };
    assert!(compare(&[refusal(), finished()], 0.18).is_none());
    assert!(compare(&[finished(), refusal()], 0.18).is_none());
    assert!(matches!(refusal(), Trial::Refused { tick: 611, .. }));
    assert!(Trial::from_result(c, Err("failed to export archive".into())).is_err());
    let unlifted = compare(&[finished(), finished()], 0.18).unwrap();
    assert!(!unlifted.both_clocks_meet_grasp_tolerances);
    assert_eq!(unlifted.max_panel_lift_change_m, 0.);
}
