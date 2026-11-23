// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn crease_nodes_with_zero_impulse_do_not_change_a_witness_panel() {
    assert_eq!(side_panel(&[2, 4, 7], &[0.2, 0.8, 0.], 3), Some(0));
    assert_eq!(side_panel(&[2, 4, 7], &[0., 0., 0.2], 3), Some(1));
    assert_eq!(side_panel(&[2, 4, 7], &[0.2, 0., 0.8], 3), None);
    assert_eq!(side_panel(&[2, 4, 7], &[0.; 3], 3), None);
}

#[test]
fn comparing_branches_rejects_an_unmatched_loaded_state() -> Result<()> {
    let out = tempfile::tempdir()?;
    let xml = arm_sheet::scene_with_loads(
        arm::SCENE,
        arm_sheet::Case {
            substeps: 8,
            ..Default::default()
        },
        0.,
        true,
    )?;
    let error = trial(&xml, grid(32, 24), &mut Some(Value::Null), out.path())
        .err()
        .unwrap();
    assert!(error.to_string().contains("differs before branching"));
    Ok(())
}
