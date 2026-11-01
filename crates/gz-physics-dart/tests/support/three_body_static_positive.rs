// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Same-input static positives shared with the retained isolated-child tests.
use super::three_body_static_support::{
    DT, approach_commands, approaching, compare, finish, fixture, native_row, obstacle, pair,
    warm_commands,
};
use gz_physics::DynamicBackend;

pub fn assert_attachment_positive() {
    let (mut candidate, ids, first) = pair();
    let (mut clean, controls, control_first) = pair();
    let floor = obstacle(&mut candidate, false, false);
    let clean_floor = obstacle(&mut clean, false, false);
    warm_commands(&mut candidate, ids, first);
    warm_commands(&mut clean, controls, control_first);
    let before = ids.map(|id| candidate.state(id));
    let rate = candidate.joint_state(first);
    let second = finish(&mut candidate, ids);
    assert_eq!(ids.map(|id| candidate.state(id)), before);
    assert_eq!(candidate.joint_state(first), rate);
    assert_eq!(second.raw(), first.raw() + 1);
    assert_eq!(candidate.joint_child(first), Some(ids[1]));
    assert_eq!(candidate.joint_child(second), Some(ids[2]));
    assert!(candidate.remove(floor));
    assert!(clean.remove(clean_floor));
    let control_second = finish(&mut clean, controls);
    for _ in 0..3 {
        candidate.step(DT).unwrap();
        clean.step(DT).unwrap();
        compare(
            &candidate,
            ids,
            [first, second],
            &clean,
            controls,
            [control_first, control_second],
        );
    }
}
pub fn assert_approaching_positive() {
    let (mut candidate, ids, joints, _floor) = approaching(false);
    let (mut clean, controls, control_joints, _clean_floor) = approaching(false);
    let data = fixture();
    let rows = data["cases"]["approach-live"]["rows"].as_array().unwrap();
    let mut matched = 0;
    let mut encountered = false;
    for tick in 1..=1000 {
        approach_commands(&mut candidate, ids, joints[1]);
        approach_commands(&mut clean, controls, control_joints[1]);
        candidate.step(DT).unwrap();
        clean.step(DT).unwrap();
        compare(&candidate, ids, joints, &clean, controls, control_joints);
        encountered |= ids
            .iter()
            .any(|id| candidate.state(*id).unwrap().position[2] - 0.1 <= -3.0);
        if let Some(row) = rows
            .iter()
            .find(|row| row["stage"] == format!("native-approach-step-{tick}"))
        {
            native_row(&candidate, ids, joints, row);
            matched += 1;
        }
    }
    assert_eq!(matched, rows.len());
    assert!(encountered, "the spheres never reached the static plane");
}
